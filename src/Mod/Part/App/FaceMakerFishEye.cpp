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
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAPI.hxx>
#include <GProp_GProps.hxx>
#include <IntTools_Context.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <algorithm>

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
    int n = static_cast<int>(inputWires.size());

    std::vector<WireFace> wfs;
    wfs.reserve(n);
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

    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i) {
        parent[i] = i;
    }

    bool hasOverlaps = false;
    for (int i = 0; i < n; ++i) {
        if (!wfs[i].isValid()) {
            continue;
        }
        for (int j = i + 1; j < n; ++j) {
            if (!wfs[j].isValid()) {
                continue;
            }
            if (hasPartialOverlap(wfs[i], wfs[j])) {
                unite(parent, i, j);
                hasOverlaps = true;
            }
        }
    }

    if (!hasOverlaps) {
        return inputWires;
    }

    std::map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i) {
        groups[findRoot(parent, i)].push_back(i);
    }

    std::vector<TopoDS_Wire> result;
    for (auto& [root, indices] : groups) {
        if (indices.size() == 1) {
            result.push_back(wfs[indices[0]].wire);
            continue;
        }

        // Iterative pairwise fuse to get a true union
        TopoDS_Shape fused;
        bool fusedValid = false;
        for (int idx : indices) {
            if (!wfs[idx].isValid()) {
                continue;
            }
            if (!fusedValid) {
                fused = wfs[idx].face;
                fusedValid = true;
                continue;
            }
            BRepAlgoAPI_Fuse fuseOp(fused, wfs[idx].face);
            if (fuseOp.IsDone() && !fuseOp.Shape().IsNull()) {
                fused = fuseOp.Shape();
            }
        }

        if (!fusedValid) {
            for (int idx : indices) {
                result.push_back(wfs[idx].wire);
            }
            continue;
        }

        for (TopExp_Explorer fExp(fused, TopAbs_FACE); fExp.More(); fExp.Next()) {
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

    // EdgesToWires: splits edges at intersections (internal BOPAlgo_Builder)
    // and assembles into closed wires
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
            FC_WARN("PointInFace failed, keeping face without classification");
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

// ─── Non-planar fallback: BRepFill_Filling (N-sided patch) ──────────────────

TopoDS_Face fillNonPlanarWire(const TopoDS_Wire& wire)
{
    try {
        BRepFill_Filling filler;
        for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next()) {
            filler.Add(TopoDS::Edge(ex.Current()), GeomAbs_C0);
        }
        filler.Build();
        if (filler.IsDone()) {
            return filler.Face();
        }
    }
    catch (...) {
        FC_WARN("BRepFill_Filling failed for non-planar wire");
    }
    return {};
}

void buildNonPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                         std::vector<TopoDS_Shape>& result)
{
    for (const auto& w : wires) {
        TopoDS_Face face = fillNonPlanarWire(w);
        if (!face.IsNull()) {
            result.push_back(face);
        }
    }
}

// ─── Pre-processing: split self-intersecting edges ─────────────────────────
//
// A single BSpline that crosses itself (e.g. figure-8) needs to be split
// at the crossing points before the main pipeline can process it.
// Uses Geom2dAPI_InterCurveCurve to detect self-intersections in 2D,
// splits into sub-edges, then reassembles into multiple wires via
// BRepAlgoAPI_BuilderAlgo + BOPAlgo_Tools::EdgesToWires.

std::vector<TopoDS_Wire> splitSelfIntersectingWires(const std::vector<TopoDS_Wire>& inputWires)
{
    const Standard_Real tol = Precision::Confusion();
    bool anySplit = false;

    // Collect all edges, splitting self-intersecting ones
    TopTools_ListOfShape allEdges;
    for (const auto& wire : inputWires) {
        for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());

            try {
                Standard_Real first, last;
                Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, first, last);
                if (curve3d.IsNull()) {
                    allEdges.Append(edge);
                    continue;
                }

                // Skip non-planar edges: XY projection creates false intersections
                // for 3D curves that don't actually cross themselves.
                {
                    bool hasZVariation = false;
                    double z0 = curve3d->Value(first).Z();
                    const int nSamples = 10;
                    for (int s = 1; s <= nSamples; s++) {
                        double t = first + (last - first) * s / nSamples;
                        if (std::abs(curve3d->Value(t).Z() - z0) > tol) {
                            hasZVariation = true;
                            break;
                        }
                    }
                    if (hasZVariation) {
                        allEdges.Append(edge);
                        continue;
                    }
                }

                // Project to XY plane for 2D self-intersection test.
                gp_Pln xyPlane;
                Handle(Geom2d_Curve) curve2d = GeomAPI::To2d(curve3d, xyPlane);
                if (curve2d.IsNull()) {
                    allEdges.Append(edge);
                    continue;
                }

                Geom2dAPI_InterCurveCurve selfInt(curve2d, tol);
                if (selfInt.NbPoints() == 0) {
                    allEdges.Append(edge);
                    continue;
                }

                // Collect all parameter values at self-intersection points
                std::vector<Standard_Real> params;
                for (int i = 1; i <= selfInt.NbPoints(); i++) {
                    Geom2dAPI_ProjectPointOnCurve proj(selfInt.Point(i), curve2d, first, last);
                    for (int j = 1; j <= proj.NbPoints(); j++) {
                        Standard_Real p = proj.Parameter(j);
                        if (p - first > tol && last - p > tol) {
                            params.push_back(p);
                        }
                    }
                }
                if (params.empty()) {
                    allEdges.Append(edge);
                    continue;
                }

                std::sort(params.begin(), params.end());
                params.erase(
                    std::unique(params.begin(), params.end(),
                                [tol](double a, double b) { return b - a < tol; }),
                    params.end());

                Standard_Real prev = first;
                bool didSplit = false;
                for (Standard_Real p : params) {
                    if (p - prev > tol) {
                        BRepBuilderAPI_MakeEdge me(curve3d, prev, p);
                        if (me.IsDone()) {
                            allEdges.Append(me.Edge());
                            didSplit = true;
                        }
                        prev = p;
                    }
                }
                if (last - prev > tol) {
                    BRepBuilderAPI_MakeEdge me(curve3d, prev, last);
                    if (me.IsDone()) {
                        allEdges.Append(me.Edge());
                        didSplit = true;
                    }
                }
                if (!didSplit) {
                    allEdges.Append(edge);
                }
                else {
                    anySplit = true;
                }
            }
            catch (...) {
                allEdges.Append(edge);
            }
        }
    }

    if (!anySplit) {
        return inputWires;
    }

    // Run BuilderAlgo to create shared vertices at crossing points,
    // then EdgesToWires to reassemble into closed wires
    if (allEdges.Size() > 1) {
        BRepAlgoAPI_BuilderAlgo splitter;
        splitter.SetArguments(allEdges);
        splitter.SetRunParallel(true);
        splitter.SetNonDestructive(Standard_True);
        splitter.Build();
        if (splitter.IsDone()) {
            allEdges.Clear();
            for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
                allEdges.Append(exp.Current());
            }
        }
    }

    BRep_Builder builder;
    TopoDS_Compound edgeCompound;
    builder.MakeCompound(edgeCompound);
    for (TopTools_ListIteratorOfListOfShape it(allEdges); it.More(); it.Next()) {
        builder.Add(edgeCompound, it.Value());
    }

    TopoDS_Shape wireShape;
    BOPAlgo_Tools::EdgesToWires(edgeCompound, wireShape);

    std::vector<TopoDS_Wire> result;
    for (TopExp_Explorer exp(wireShape, TopAbs_WIRE); exp.More(); exp.Next()) {
        result.push_back(TopoDS::Wire(exp.Current()));
    }
    return result.empty() ? inputWires : result;
}

}  // namespace

// ─── Build_Essence ───────────────────────────────────────────────────────────

void FaceMakerFishEye::Build_Essence()
{
    if (myWires.empty()) {
        return;
    }

    // Pre-process: split self-intersecting edges into separate wires
    std::vector<TopoDS_Wire> wires = splitSelfIntersectingWires(myWires);

    // Fast path: single wire needs no overlap detection or even-odd classification
    if (wires.size() == 1) {
        TopoDS_Face face = makeFaceFromWire(wires.front());
        if (!face.IsNull()) {
            myShapesToReturn.push_back(face);
            return;
        }
        // Planar MakeFace failed — try non-planar filling
        face = fillNonPlanarWire(wires.front());
        if (!face.IsNull()) {
            myShapesToReturn.push_back(face);
            return;
        }
    }

    // Phase 1: Fuse overlapping wires (union semantics for crossing wires)
    wires = fuseOverlappingWires(wires);

    // Phase 2: Planar face building with even-odd nesting
    if (buildPlanarFaces(wires, myShapesToReturn)) {
        return;
    }

    // Phase 3: Non-planar fallback
    buildNonPlanarFaces(wires, myShapesToReturn);
}
