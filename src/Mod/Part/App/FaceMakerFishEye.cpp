// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 FreeCAD contributors                              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include "FaceMakerFishEye.h"
#include "FaceMakerCheese.h"

#include <BOPAlgo_Tools.hxx>
#include <BOPTools_AlgoTools3D.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IntTools_Context.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <Base/Console.h>

FC_LOG_LEVEL_INIT("FaceMakerFishEye", true, true)

using namespace Part;

TYPESYSTEM_SOURCE(Part::FaceMakerFishEye, Part::FaceMakerPublic)

std::string FaceMakerFishEye::getUserFriendlyName() const
{
    return {tr("Fish-eye facemaker").toStdString()};
}

std::string FaceMakerFishEye::getBriefExplanation() const
{
    return {tr("Unified: handles nested holes, overlapping wires, and curved surfaces")
                .toStdString()};
}

void FaceMakerFishEye::setPlane(const gp_Pln& plane)
{
    myPlane = plane;
    planeSupplied = true;
}

namespace
{

TopoDS_Face makeFaceFromWire(const TopoDS_Wire& w)
{
    try {
        BRepBuilderAPI_MakeFace mf(w);
        if (mf.IsDone()) {
            return mf.Face();
        }
    }
    catch (...) {
    }
    return {};
}

// ─── Phase 1: Fuse overlapping wires ─────────────────────────────────────────
//
// Detect partial overlaps (bounding box pre-filter + BRepAlgoAPI_Common),
// group with union-find, fuse each group, extract outer wires.
// Full containment (hole inside outer) is NOT fused — it's preserved
// for even-odd nesting in Phase 2.

struct WireFace
{
    TopoDS_Wire wire;
    TopoDS_Face face;
    Bnd_Box box;
    double area {0.0};

    bool isValid() const
    {
        return !face.IsNull();
    }
};

double shapeArea(const TopoDS_Shape& s)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(s, props);
    return props.Mass();
}

bool hasPartialOverlap(const WireFace& a, const WireFace& b)
{
    if (a.box.IsOut(b.box)) {
        return false;
    }
    BRepAlgoAPI_Common common(a.face, b.face);
    if (!common.IsDone() || common.Shape().IsNull()) {
        return false;
    }
    double commonArea = shapeArea(common.Shape());
    double tol = Precision::Confusion();
    return commonArea > tol && commonArea < a.area - tol && commonArea < b.area - tol;
}

int findRoot(std::vector<int>& parent, int i)
{
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

void unite(std::vector<int>& parent, int a, int b)
{
    parent[findRoot(parent, a)] = findRoot(parent, b);
}

std::vector<TopoDS_Wire> fuseOverlappingWires(const std::vector<TopoDS_Wire>& inputWires)
{
    std::vector<WireFace> wfs;
    wfs.reserve(inputWires.size());
    for (const auto& w : inputWires) {
        WireFace wf;
        wf.wire = w;
        wf.face = makeFaceFromWire(w);
        if (wf.isValid()) {
            BRepBndLib::AddOptimal(w, wf.box, Standard_False);
            wf.area = shapeArea(wf.face);
        }
        wfs.push_back(std::move(wf));
    }

    std::vector<int> parent(wfs.size());
    for (size_t i = 0; i < parent.size(); ++i) {
        parent[i] = static_cast<int>(i);
    }

    bool hasOverlaps = false;
    for (size_t i = 0; i < wfs.size(); ++i) {
        if (!wfs[i].isValid()) {
            continue;
        }
        for (size_t j = i + 1; j < wfs.size(); ++j) {
            if (!wfs[j].isValid()) {
                continue;
            }
            if (hasPartialOverlap(wfs[i], wfs[j])) {
                unite(parent, static_cast<int>(i), static_cast<int>(j));
                hasOverlaps = true;
            }
        }
    }

    if (!hasOverlaps) {
        return inputWires;
    }

    std::map<int, std::vector<size_t>> groups;
    for (size_t i = 0; i < wfs.size(); ++i) {
        groups[findRoot(parent, static_cast<int>(i))].push_back(i);
    }

    std::vector<TopoDS_Wire> result;
    for (auto& [root, indices] : groups) {
        if (indices.size() == 1) {
            result.push_back(wfs[indices[0]].wire);
            continue;
        }

        TopTools_ListOfShape toFuse;
        for (size_t idx : indices) {
            if (wfs[idx].isValid()) {
                toFuse.Append(wfs[idx].face);
            }
        }
        if (toFuse.Size() < 2) {
            for (size_t idx : indices) {
                result.push_back(wfs[idx].wire);
            }
            continue;
        }

        BRepAlgoAPI_BuilderAlgo fuser;
        fuser.SetArguments(toFuse);
        fuser.Build();
        if (!fuser.IsDone()) {
            for (size_t idx : indices) {
                result.push_back(wfs[idx].wire);
            }
            continue;
        }

        for (TopExp_Explorer fExp(fuser.Shape(), TopAbs_FACE); fExp.More(); fExp.Next()) {
            TopoDS_Wire outerWire = BRepTools::OuterWire(TopoDS::Face(fExp.Current()));
            if (!outerWire.IsNull()) {
                result.push_back(outerWire);
            }
        }
    }

    return result;
}

// ─── Phase 2: Planar face building ──────────────────────────────────────────
//
// EdgesToWires handles edge splitting internally. WiresToFaces builds all
// face regions with hole nesting. Even-odd classification uses cached
// IntTools_Context for fast 2D point-in-polygon tests.

bool buildPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                      std::vector<TopoDS_Shape>& result)
{
    // Collect all edges into a compound
    BRep_Builder builder;
    TopoDS_Compound edgeCompound;
    builder.MakeCompound(edgeCompound);
    for (const auto& w : wires) {
        for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
            builder.Add(edgeCompound, exp.Current());
        }
    }

    // EdgesToWires: splits edges at intersections and assembles closed wires
    TopoDS_Shape wireShape;
    BOPAlgo_Tools::EdgesToWires(edgeCompound, wireShape);

    // WiresToFaces: finds planes, builds pcurves, runs BOPAlgo_BuilderFace
    TopoDS_Shape faceShape;
    if (!BOPAlgo_Tools::WiresToFaces(wireShape, faceShape)) {
        return false;
    }

    // Collect all face regions
    std::vector<TopoDS_Face> allFaces;
    for (TopExp_Explorer exp(faceShape, TopAbs_FACE); exp.More(); exp.Next()) {
        allFaces.push_back(TopoDS::Face(exp.Current()));
    }
    if (allFaces.empty()) {
        return false;
    }

    // Even-odd nesting: keep faces whose interior point is inside an
    // odd number of input wires. Uses IntTools_Context for cached
    // 2D point-in-polygon classification.
    Handle(IntTools_Context) ctx = new IntTools_Context();
    std::vector<TopoDS_Face> wireFaces;
    wireFaces.reserve(wires.size());
    for (const auto& w : wires) {
        wireFaces.push_back(makeFaceFromWire(w));
    }

    for (const auto& face : allFaces) {
        gp_Pnt interiorPt;
        gp_Pnt2d interiorPt2d;
        if (BOPTools_AlgoTools3D::PointInFace(face, interiorPt, interiorPt2d, ctx) != 0) {
            result.push_back(face);
            continue;
        }

        int containCount = 0;
        for (const auto& wf : wireFaces) {
            if (wf.IsNull()) {
                continue;
            }
            if (ctx->IsPointInFace(interiorPt, wf, Precision::Confusion())) {
                ++containCount;
            }
        }

        if (containCount % 2 == 1) {
            result.push_back(face);
        }
    }

    return !result.empty();
}

// ─── Non-planar fallback: delegate to FaceMakerCheese ────────────────────────

void buildNonPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                         std::vector<TopoDS_Shape>& result)
{
    FaceMakerCheese cheese;
    for (const auto& w : wires) {
        cheese.addWire(w);
    }
    cheese.Build();
    for (TopExp_Explorer exp(cheese.Shape(), TopAbs_FACE); exp.More(); exp.Next()) {
        result.push_back(exp.Current());
    }
}

}  // namespace

// ─── Build_Essence ───────────────────────────────────────────────────────────

void FaceMakerFishEye::Build_Essence()
{
    if (myWires.empty()) {
        return;
    }

    // Phase 1: Fuse overlapping wires (union semantics for crossing wires)
    std::vector<TopoDS_Wire> wires = fuseOverlappingWires(myWires);

    // Phase 2: Planar face building with even-odd nesting
    if (buildPlanarFaces(wires, myShapesToReturn)) {
        return;
    }

    // Phase 3: Non-planar fallback
    buildNonPlanarFaces(wires, myShapesToReturn);
}
