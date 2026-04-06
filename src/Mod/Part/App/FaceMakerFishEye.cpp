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

#include <BOPAlgo_BuilderFace.hxx>
#include <BOPAlgo_Tools.hxx>
#include <BOPTools_AlgoTools3D.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepGProp.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepLib.hxx>
#include <BRepLib_FindSurface.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomAPI.hxx>
#include <Geom_Conic.hxx>
#include <Geom_Line.hxx>
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

void FaceMakerFishEye::setPlane(const gp_Pln& plane)
{
    myPlane = plane;
    planeSupplied = true;
}

std::string FaceMakerFishEye::getUserFriendlyName() const
{
    return {tr("Fish-eye facemaker").toStdString()};
}

std::string FaceMakerFishEye::getBriefExplanation() const
{
    return {tr("Unified: handles nested holes, overlapping wires, and curved surfaces").toStdString()};
}

// ─── Helpers ────────────────────────────────────────────────────────────────

namespace
{

TopoDS_Face makeFaceFromWire(const TopoDS_Wire& w, const gp_Pln* plane = nullptr)
{
    if (!BRep_Tool::IsClosed(w)) {
        return {};
    }
    BRepBuilderAPI_MakeFace mf = plane ? BRepBuilderAPI_MakeFace(*plane, w)
                                       : BRepBuilderAPI_MakeFace(w);
    if (mf.IsDone()) {
        return mf.Face();
    }
    return {};
}

double shapeArea(const TopoDS_Shape& s)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(s, props);
    return props.Mass();
}

// ─── Plane detection ────────────────────────────────────────────────────────

// findPlane is defined outside the anonymous namespace as a member of FaceMakerFishEye.
}  // namespace

bool FaceMakerFishEye::findPlane(const std::vector<TopoDS_Wire>& wires, gp_Pln& plane) const
{
    // Copy wires to strip embedded surface info — BRepLib_FindSurface
    // can return the edge's cached surface (e.g. sketch XY plane) instead
    // of fitting to actual 3D positions. BRepBuilderAPI_Copy clears this.
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    for (const auto& w : wires) {
        builder.Add(comp, BRepBuilderAPI_Copy(w).Shape());
    }
    BRepLib_FindSurface planeFinder(comp, -1, /*OnlyPlane=*/Standard_True);
    if (!planeFinder.Found()) {
        return false;
    }
    plane = GeomAdaptor_Surface(planeFinder.Surface()).Plane();

    // BRepLib_FindSurface returns an arbitrary normal direction that can
    // differ across platforms. If a reference plane was supplied (e.g. from
    // the sketch's Placement), align the normal with it.
    if (planeSupplied && plane.Axis().Direction().Dot(myPlane.Axis().Direction()) < 0) {
        plane = gp_Pln(plane.Location(), plane.Axis().Direction().Reversed());
    }
    return true;
}

namespace
{

// ─── Self-intersection splitting ────────────────────────────────────────────
//
// BSplines that cross themselves (figure-8) must be split at the crossing
// before the face pipeline can produce separate lobes.

std::vector<TopoDS_Wire> splitSelfIntersecting(
    const std::vector<TopoDS_Wire>& inputWires,
    const gp_Pln& plane
)
{
    const Standard_Real tol = Precision::Confusion();
    std::vector<TopoDS_Wire> result;
    TopTools_ListOfShape splitEdges;

    for (const auto& wire : inputWires) {
        bool wireSplit = false;
        TopTools_ListOfShape wireEdges;

        for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
            try {
                Standard_Real first, last;
                Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
                if (curve.IsNull() || curve->IsKind(STANDARD_TYPE(Geom_Line))
                    || curve->IsKind(STANDARD_TYPE(Geom_Conic))) {
                    wireEdges.Append(edge);
                    continue;
                }

                Handle(Geom2d_Curve) curve2d = GeomAPI::To2d(curve, plane);
                if (curve2d.IsNull()) {
                    wireEdges.Append(edge);
                    continue;
                }

                Geom2dAPI_InterCurveCurve selfInt(curve2d, tol);
                if (selfInt.NbPoints() == 0) {
                    wireEdges.Append(edge);
                    continue;
                }

                // Collect crossing parameters
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
                    wireEdges.Append(edge);
                    continue;
                }

                std::sort(params.begin(), params.end());
                params.erase(
                    std::unique(
                        params.begin(),
                        params.end(),
                        [tol](double a, double b) { return b - a < tol; }
                    ),
                    params.end()
                );

                // Split into sub-edges
                Standard_Real prev = first;
                bool didSplit = false;
                for (Standard_Real p : params) {
                    if (p - prev > tol) {
                        BRepBuilderAPI_MakeEdge me(curve, prev, p);
                        if (me.IsDone()) {
                            wireEdges.Append(me.Edge());
                            didSplit = true;
                        }
                        prev = p;
                    }
                }
                if (last - prev > tol) {
                    BRepBuilderAPI_MakeEdge me(curve, prev, last);
                    if (me.IsDone()) {
                        wireEdges.Append(me.Edge());
                        didSplit = true;
                    }
                }

                if (didSplit) {
                    wireSplit = true;
                }
                else {
                    wireEdges.Append(edge);
                }
            }
            catch (const Standard_Failure& e) {
                if (FC_LOG_INSTANCE.isEnabled(FC_LOGLEVEL_LOG)) {
                    FC_WARN("splitSelfIntersecting: " << e.GetMessageString());
                }
                wireEdges.Append(edge);
            }
        }

        if (wireSplit) {
            for (TopTools_ListIteratorOfListOfShape it(wireEdges); it.More(); it.Next()) {
                splitEdges.Append(it.Value());
            }
        }
        else {
            result.push_back(wire);
        }
    }

    if (splitEdges.IsEmpty()) {
        return inputWires;
    }

    // Reassemble split edges into wires (EdgesToWires creates shared vertices
    // at crossing points via its internal BOPAlgo_Builder)
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        builder.Add(comp, it.Value());
    }
    TopoDS_Shape wireShape;
    BOPAlgo_Tools::EdgesToWires(comp, wireShape);
    for (TopExp_Explorer exp(wireShape, TopAbs_WIRE); exp.More(); exp.Next()) {
        result.push_back(TopoDS::Wire(exp.Current()));
    }
    return result;
}

// ─── Overlap fusing ─────────────────────────────────────────────────────────
//
// Partially overlapping closed wires are fused into their union so that
// even-odd classification can treat each Venn region independently.
// Full containment (hole inside outer) is preserved for even-odd nesting.

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

struct WireFace
{
    TopoDS_Wire wire;
    TopoDS_Face face;
    Bnd_Box box;
    double area {0.0};
};

std::vector<TopoDS_Wire> fuseOverlaps(const std::vector<TopoDS_Wire>& inputWires, const gp_Pln& plane)
{
    int n = static_cast<int>(inputWires.size());

    // Build faces only from closed wires — open wires produce fake faces
    std::vector<WireFace> wfs;
    wfs.reserve(n);
    int closedCount = 0;
    for (const auto& w : inputWires) {
        WireFace wf;
        wf.wire = w;
        if (BRep_Tool::IsClosed(w)) {
            wf.face = makeFaceFromWire(w, &plane);
            if (!wf.face.IsNull()) {
                BRepBndLib::AddOptimal(w, wf.box, Standard_False);
                wf.area = shapeArea(wf.face);
                ++closedCount;
            }
        }
        wfs.push_back(std::move(wf));
    }
    if (closedCount < 2) {
        return inputWires;
    }

    // Pairwise overlap detection with bbox pre-filter
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    bool hasOverlaps = false;

    for (int i = 0; i < n; ++i) {
        if (wfs[i].face.IsNull()) {
            continue;
        }
        for (int j = i + 1; j < n; ++j) {
            if (wfs[j].face.IsNull() || wfs[i].box.IsOut(wfs[j].box)) {
                continue;
            }
            BRepAlgoAPI_Common common(wfs[i].face, wfs[j].face);
            if (!common.IsDone() || common.Shape().IsNull()) {
                continue;
            }
            double ca = shapeArea(common.Shape());
            double tol = Precision::Confusion();
            if (ca > tol && ca < wfs[i].area - tol && ca < wfs[j].area - tol) {
                unite(parent, i, j);
                hasOverlaps = true;
            }
        }
    }
    if (!hasOverlaps) {
        return inputWires;
    }

    // Fuse each overlap group
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
        TopoDS_Shape fused;
        bool valid = false;
        for (int idx : indices) {
            if (wfs[idx].face.IsNull()) {
                continue;
            }
            if (!valid) {
                fused = wfs[idx].face;
                valid = true;
            }
            else {
                BRepAlgoAPI_Fuse fuseOp(fused, wfs[idx].face);
                if (fuseOp.IsDone() && !fuseOp.Shape().IsNull()) {
                    fused = fuseOp.Shape();
                }
            }
        }
        if (!valid) {
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

// ─── Planar face building ───────────────────────────────────────────────────
//
// Pipeline: BRepAlgoAPI_BuilderAlgo (split edges at intersections)
//         → BOPAlgo_BuilderFace (find all bounded regions via WireSplitter)
//         → even-odd classification (nesting/holes)
//
// Uses BuilderFace directly instead of EdgesToWires + WiresToFaces.
// WireSplitter handles degree-4 vertices from self-intersecting curves
// (splitting figure-8 into 2 lobes via angular sorting).

void buildPlanar(
    const std::vector<TopoDS_Wire>& wires,
    const gp_Pln& plane,
    std::vector<TopoDS_Shape>& result
)
{
    // Collect all edges
    TopTools_ListOfShape edges;
    for (const auto& w : wires) {
        for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
            edges.Append(exp.Current());
        }
    }
    if (edges.IsEmpty()) {
        return;
    }

    // Split edges at mutual intersections
    TopTools_ListOfShape splitEdges = edges;
    if (edges.Size() > 1) {
        BRepAlgoAPI_BuilderAlgo splitter;
        splitter.SetArguments(edges);
        splitter.SetRunParallel(true);
        splitter.SetNonDestructive(Standard_True);
        splitter.Build();
        if (splitter.IsDone()) {
            splitEdges.Clear();
            for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
                splitEdges.Append(exp.Current());
            }
        }
    }

    // Build base face larger than the geometry bounds
    Bnd_Box geomBox;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        BRepBndLib::Add(it.Value(), geomBox);
    }
    // Base face must be larger than all geometry so BuilderFace can
    // distinguish bounded regions from the unbounded exterior.
    const Standard_Real aMax = std::max(1.0e8, 10.0 * std::sqrt(geomBox.SquareExtent()));
    TopoDS_Face baseFace = BRepBuilderAPI_MakeFace(plane, -aMax, aMax, -aMax, aMax).Face();
    // BuilderFace requires FORWARD orientation on the base face
    baseFace.Orientation(TopAbs_FORWARD);

    TopTools_ListOfShape faceEdges;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(it.Value());
        faceEdges.Append(e.Oriented(TopAbs_FORWARD));
        faceEdges.Append(e.Oriented(TopAbs_REVERSED));
    }
    BRepLib::BuildPCurveForEdgesOnPlane(faceEdges, baseFace);

    // BOPAlgo_BuilderFace: finds all bounded face regions.
    // SetAvoidInternalShapes prevents dangling edges from becoming
    // internal wires that create degenerate geometry when extruded.
    BOPAlgo_BuilderFace faceBuilder;
    faceBuilder.SetFace(baseFace);
    faceBuilder.SetShapes(faceEdges);
    faceBuilder.SetAvoidInternalShapes(Standard_True);
    faceBuilder.Perform();
    if (faceBuilder.HasErrors()) {
        return;
    }

    // Collect bounded faces, skip the unbounded outer face and degenerate faces
    const double outerThreshold = aMax * aMax;
    std::vector<TopoDS_Face> allFaces;
    for (TopTools_ListIteratorOfListOfShape it(faceBuilder.Areas()); it.More(); it.Next()) {
        Bnd_Box box;
        BRepBndLib::Add(it.Value(), box);
        if (box.SquareExtent() > outerThreshold) {
            continue;
        }
        if (shapeArea(it.Value()) < Precision::Confusion()) {
            continue;
        }
        allFaces.push_back(TopoDS::Face(it.Value()));
    }
    if (allFaces.empty()) {
        return;
    }

    // Even-odd classification: keep faces inside an odd number of input wires.
    // Faces not inside ANY wire (count=0) are kept unconditionally — they come
    // from geometry whose wires can't form reference faces (BSpline lobes etc).
    Handle(IntTools_Context) ctx = new IntTools_Context();
    std::vector<TopoDS_Face> wireFaces;
    wireFaces.reserve(wires.size());
    for (const auto& w : wires) {
        wireFaces.push_back(BRep_Tool::IsClosed(w) ? makeFaceFromWire(w, &plane) : TopoDS_Face());
    }

    for (const auto& face : allFaces) {
        gp_Pnt pt;
        gp_Pnt2d pt2d;
        if (BOPTools_AlgoTools3D::PointInFace(face, pt, pt2d, ctx) != 0) {
            result.push_back(face);
            continue;
        }

        int count = 0;
        for (const auto& wf : wireFaces) {
            if (!wf.IsNull() && ctx->IsPointInFace(pt, wf, Precision::Confusion())) {
                ++count;
            }
        }

        if (count == 0 || count % 2 == 1) {
            result.push_back(face);
        }
    }
}

// ─── Non-planar fallback ────────────────────────────────────────────────────

TopoDS_Face fillNonPlanar(const TopoDS_Wire& wire)
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
    catch (const Standard_Failure& e) {
        if (FC_LOG_INSTANCE.isEnabled(FC_LOGLEVEL_LOG)) {
            FC_WARN("fillNonPlanar: " << e.GetMessageString());
        }
    }
    return {};
}

}  // namespace

// ─── Build_Essence ──────────────────────────────────────────────────────────

void FaceMakerFishEye::Build_Essence()
{
    if (myWires.empty()) {
        return;
    }

    // Detect the plane from actual edge geometry. The supplied plane (from
    // setPlane) may not match the actual edge coordinates — PartDesign
    // transforms edges to global space but may supply the sketch-local plane.
    gp_Pln plane;
    bool planar = findPlane(myWires, plane);
    if (!planar && planeSupplied) {
        plane = myPlane;
        planar = true;
    }

    if (planar) {
        std::vector<TopoDS_Wire> wires = splitSelfIntersecting(myWires, plane);
        wires = fuseOverlaps(wires, plane);
        buildPlanar(wires, plane, myShapesToReturn);
    }
    else {
        // Non-planar: try MakeFace (analytical surfaces like cylinders),
        // then BRepFill_Filling (freeform BSpline patch).
        for (const auto& w : myWires) {
            TopoDS_Face face = makeFaceFromWire(w);
            if (face.IsNull() && BRep_Tool::IsClosed(w)) {
                face = fillNonPlanar(w);
            }
            if (!face.IsNull() && shapeArea(face) > Precision::Confusion()) {
                myShapesToReturn.push_back(face);
            }
        }
    }
}
