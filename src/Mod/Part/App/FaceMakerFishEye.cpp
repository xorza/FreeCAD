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

#include <BOPAlgo_Tools.hxx>
#include <BOPTools_AlgoTools3D.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IntTools_Context.hxx>
#include <Precision.hxx>
#include <ShapeFix_Shape.hxx>
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

// ─── Helpers ─────────────────────────────────────────────────────────────────

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

// ─── Phase 1: Fuse overlapping wires ─────────────────────────────────────────

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
            FC_WARN("FaceMakerFishEye: failed to fuse overlapping wires");
            for (size_t idx : indices) {
                result.push_back(wfs[idx].wire);
            }
            continue;
        }

        for (TopExp_Explorer fExp(fuser.Shape(), TopAbs_FACE); fExp.More(); fExp.Next()) {
            TopoDS_Face fusedFace = TopoDS::Face(fExp.Current());
            TopoDS_Wire outerWire = BRepTools::OuterWire(fusedFace);
            if (!outerWire.IsNull()) {
                result.push_back(outerWire);
            }
        }
    }

    return result;
}

// ─── Phase 2: Planar face building via BOPAlgo_Tools::WiresToFaces ───────────

bool buildPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                      std::vector<TopoDS_Shape>& result)
{
    // Collect all edges from input wires
    TopTools_ListOfShape edgeList;
    for (const auto& w : wires) {
        for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
            edgeList.Append(exp.Current());
        }
    }
    if (edgeList.IsEmpty()) {
        return false;
    }

    // Split edges at mutual intersections via General Fuse
    BRepAlgoAPI_BuilderAlgo splitter;
    splitter.SetArguments(edgeList);
    splitter.SetRunParallel(true);
    splitter.SetNonDestructive(Standard_True);
    splitter.Build();
    if (!splitter.IsDone()) {
        return false;
    }

    // Collect split edges into a compound for EdgesToWires
    BRep_Builder builder;
    TopoDS_Compound edgeCompound;
    builder.MakeCompound(edgeCompound);
    for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
        builder.Add(edgeCompound, exp.Current());
    }

    // EdgesToWires: connect split edges into closed wires
    TopoDS_Shape wireShape;
    BOPAlgo_Tools::EdgesToWires(edgeCompound, wireShape);

    // WiresToFaces: build planar faces with proper hole nesting.
    // This handles plane detection, pcurve building, BOPAlgo_BuilderFace,
    // and hole-to-face assignment in one call.
    TopoDS_Shape faceShape;
    if (!BOPAlgo_Tools::WiresToFaces(wireShape, faceShape)) {
        return false;
    }

    // Collect all resulting faces
    std::vector<TopoDS_Face> allFaces;
    for (TopExp_Explorer exp(faceShape, TopAbs_FACE); exp.More(); exp.Next()) {
        allFaces.push_back(TopoDS::Face(exp.Current()));
    }

    if (allFaces.empty()) {
        return false;
    }

    // BOPAlgo_BuilderFace (inside WiresToFaces) produces all bounded regions
    // with proper hole assignment. But it returns ALL growth faces including
    // islands inside holes. Apply even-odd nesting: keep faces whose interior
    // point is inside an odd number of original input wires.
    Handle(IntTools_Context) ctx = new IntTools_Context();
    std::vector<TopoDS_Face> wireFaces;
    wireFaces.reserve(wires.size());
    for (const auto& w : wires) {
        wireFaces.push_back(makeFaceFromWire(w));
    }

    for (const auto& face : allFaces) {
        // Get a robust interior point using OCCT's hatcher
        gp_Pnt interiorPt;
        gp_Pnt2d interiorPt2d;
        if (BOPTools_AlgoTools3D::PointInFace(face, interiorPt, interiorPt2d, ctx) != 0) {
            // Fallback: keep the face if we can't get an interior point
            result.push_back(face);
            continue;
        }

        int containCount = 0;
        for (const auto& wf : wireFaces) {
            if (wf.IsNull()) {
                continue;
            }
            BRepClass_FaceClassifier classifier(wf, interiorPt, Precision::Confusion());
            if (classifier.State() == TopAbs_IN) {
                ++containCount;
            }
        }

        if (containCount % 2 == 1) {
            result.push_back(face);
        }
    }

    return !result.empty();
}

// ─── Phase 3: Non-planar face building ───────────────────────────────────────

void buildNonPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                         std::vector<TopoDS_Shape>& result)
{
    struct WireBox
    {
        TopoDS_Wire wire;
        double extent;
    };
    std::vector<WireBox> sorted;
    sorted.reserve(wires.size());
    for (const auto& w : wires) {
        Bnd_Box box;
        BRepBndLib::Add(w, box);
        sorted.push_back({w, box.SquareExtent()});
    }
    std::stable_sort(sorted.begin(), sorted.end(),
                     [](const WireBox& a, const WireBox& b) { return a.extent > b.extent; });

    std::vector<TopoDS_Face> faces;
    for (const auto& wb : sorted) {
        bool addedAsHole = false;
        for (auto& face : faces) {
            TopExp_Explorer vExp(wb.wire, TopAbs_VERTEX);
            if (!vExp.More()) {
                continue;
            }
            gp_Pnt testPt = BRep_Tool::Pnt(TopoDS::Vertex(vExp.Current()));
            BRepClass_FaceClassifier classifier(face, testPt, Precision::Confusion());
            if (classifier.State() == TopAbs_IN) {
                BRepBuilderAPI_MakeFace mf(face);
                mf.Add(wb.wire);
                if (!mf.IsDone()) {
                    TopoDS_Wire reversed = TopoDS::Wire(wb.wire.Reversed());
                    mf.Init(face);
                    mf.Add(reversed);
                }
                if (mf.IsDone()) {
                    face = mf.Face();
                    addedAsHole = true;
                    break;
                }
            }
        }

        if (!addedAsHole) {
            BRepBuilderAPI_MakeFace mf(wb.wire);
            if (mf.IsDone()) {
                TopoDS_Face newFace = mf.Face();
                BRepCheck_Analyzer checker(newFace);
                if (!checker.IsValid()) {
                    ShapeFix_Shape fix(newFace);
                    fix.SetPrecision(Precision::Confusion());
                    fix.Perform();
                    newFace = TopoDS::Face(fix.Shape());
                }
                faces.push_back(newFace);
            }
        }
    }

    for (auto& f : faces) {
        result.push_back(f);
    }
}

}  // namespace

// ─── Build_Essence ───────────────────────────────────────────────────────────

void FaceMakerFishEye::Build_Essence()
{
    if (myWires.empty()) {
        return;
    }

    // Phase 1: Fuse overlapping wires into merged outlines
    std::vector<TopoDS_Wire> wires = fuseOverlappingWires(myWires);

    // Phase 2: Try planar face building
    if (buildPlanarFaces(wires, myShapesToReturn)) {
        return;
    }

    // Phase 3: Non-planar fallback
    buildNonPlanarFaces(wires, myShapesToReturn);
}
