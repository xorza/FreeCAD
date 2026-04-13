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
#include <BRepBuilderAPI_MakeWire.hxx>
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

// Split self-intersecting edges (e.g., figure-8 BSplines) at their crossing
// points.  Records the mapping original -> fragments in myPreSplitHistory so
// that postBuild() can chain it with mySplitter for proper element naming.
std::vector<TopoDS_Wire> FaceMakerFishEye::splitSelfIntersecting(
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
                // Lines and conics (circles, ellipses, etc.) cannot self-intersect.
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
                TopTools_ListOfShape fragments;
                for (Standard_Real p : params) {
                    if (p - prev > tol) {
                        BRepBuilderAPI_MakeEdge me(curve, prev, p);
                        if (me.IsDone()) {
                            fragments.Append(me.Edge());
                        }
                        prev = p;
                    }
                }
                if (last - prev > tol) {
                    BRepBuilderAPI_MakeEdge me(curve, prev, last);
                    if (me.IsDone()) {
                        fragments.Append(me.Edge());
                    }
                }

                if (fragments.Size() > 0) {
                    if (myPreSplitHistory.IsNull()) {
                        myPreSplitHistory = new BRepTools_History();
                    }
                    for (TopTools_ListIteratorOfListOfShape fi(fragments); fi.More(); fi.Next()) {
                        myPreSplitHistory->AddModified(edge, fi.Value());
                        wireEdges.Append(fi.Value());
                    }
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
    // at crossing points via its internal BOPAlgo_Builder).
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

    // EdgesToWires may create new edge TShapes, so the fragment edges
    // recorded in myPreSplitHistory might not match the edges in the
    // output wires.  Rebuild the history using the actual output wire
    // edges: for each output edge, find the original edge whose fragment
    // shares the same 3D curve, and record original → output edge.
    if (!myPreSplitHistory.IsNull()) {
        myPreSplitHistory = new BRepTools_History();
        for (const auto& w : result) {
            for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
                const TopoDS_Edge& wireEdge = TopoDS::Edge(exp.Current());
                // Find which original edge this wire edge came from by
                // comparing the underlying 3D curve handle.
                Standard_Real wf, wl;
                Handle(Geom_Curve) wireCurve = BRep_Tool::Curve(wireEdge, wf, wl);
                for (const auto& origWire : inputWires) {
                    for (TopExp_Explorer oe(origWire, TopAbs_EDGE); oe.More(); oe.Next()) {
                        const TopoDS_Edge& origEdge = TopoDS::Edge(oe.Current());
                        Standard_Real of, ol;
                        Handle(Geom_Curve) origCurve = BRep_Tool::Curve(origEdge, of, ol);
                        if (!origCurve.IsNull() && origCurve == wireCurve) {
                            myPreSplitHistory->AddModified(origEdge, wireEdge);
                            break;
                        }
                    }
                }
            }
        }

        BRep_Builder compBuilder;
        compBuilder.MakeCompound(myPreSplitCompound);
        for (const auto& w : result) {
            for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
                compBuilder.Add(myPreSplitCompound, exp.Current());
            }
        }
    }
    return result;
}

namespace
{

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
    TopoDS_Wire wire {};
    TopoDS_Face face {};
    Bnd_Box box {};
    double area {0.0};
};

}  // namespace

// Partially overlapping closed wires are fused into their union so that
// even-odd classification can treat each Venn region independently.
// Full containment (hole inside outer) is preserved for even-odd nesting.
// Composes fuse history into myPreSplitHistory so postBuild() can trace
// names through self-intersection splitting + fusion in one stage.
std::vector<TopoDS_Wire> FaceMakerFishEye::fuseOverlaps(
    const std::vector<TopoDS_Wire>& inputWires,
    const gp_Pln& plane
)
{
    int n = static_cast<int>(inputWires.size());

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
    const double tol = Precision::Confusion();

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
            const double ca = shapeArea(common.Shape());
            if (ca > tol && ca < wfs[i].area - tol && ca < wfs[j].area - tol) {
                unite(parent, i, j);
                hasOverlaps = true;
            }
        }
    }
    if (!hasOverlaps) {
        return inputWires;
    }

    // Fuse each overlap group, tracking history for element naming
    std::map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i) {
        groups[findRoot(parent, i)].push_back(i);
    }

    // Save existing pre-split history (from splitSelfIntersecting) so we
    // can compose it with the fuse history into a single mapping.
    Handle(BRepTools_History) preSplitHist = myPreSplitHistory;
    Handle(BRepTools_History) fuseHist;

    std::vector<TopoDS_Wire> result;
    for (auto& [root, indices] : groups) {
        if (indices.size() == 1) {
            result.push_back(wfs[indices[0]].wire);
            continue;
        }
        TopoDS_Shape fused;
        bool valid = false;
        std::vector<Handle(BRepTools_History)> stepHistories;
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
                    stepHistories.push_back(fuseOp.History());
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

        // Trace each input wire edge through the chain of fuse histories
        // to find the final fused edges, then record the mapping.
        if (!stepHistories.empty()) {
            if (fuseHist.IsNull()) {
                fuseHist = new BRepTools_History();
            }
            for (int idx : indices) {
                for (TopExp_Explorer exp(wfs[idx].wire, TopAbs_EDGE); exp.More(); exp.Next()) {
                    const TopoDS_Edge& inputEdge = TopoDS::Edge(exp.Current());
                    TopTools_ListOfShape current;
                    current.Append(inputEdge);

                    for (const auto& hist : stepHistories) {
                        TopTools_ListOfShape next;
                        for (TopTools_ListIteratorOfListOfShape ci(current); ci.More(); ci.Next()) {
                            const TopTools_ListOfShape& modified = hist->Modified(ci.Value());
                            if (modified.IsEmpty()) {
                                if (!hist->IsRemoved(ci.Value())) {
                                    next.Append(ci.Value());
                                }
                            }
                            else {
                                for (TopTools_ListIteratorOfListOfShape mi(modified); mi.More();
                                     mi.Next()) {
                                    next.Append(mi.Value());
                                }
                            }
                        }
                        current = next;
                    }

                    for (TopTools_ListIteratorOfListOfShape fi(current); fi.More(); fi.Next()) {
                        if (!inputEdge.IsSame(fi.Value())) {
                            fuseHist->AddModified(inputEdge, fi.Value());
                        }
                    }
                }
            }
        }

        for (TopExp_Explorer fExp(fused, TopAbs_FACE); fExp.More(); fExp.Next()) {
            TopoDS_Wire outerWire = BRepTools::OuterWire(TopoDS::Face(fExp.Current()));
            if (!outerWire.IsNull()) {
                result.push_back(outerWire);
            }
        }
    }

    if (fuseHist.IsNull()) {
        return result;
    }

    // Compose pre-split + fuse histories into a single myPreSplitHistory.
    // Iterate the true original edges (myWires, before any modification),
    // trace each through preSplitHist (original → fragment) then fuseHist
    // (fragment → fused), and record original → fused in the new history.
    if (preSplitHist.IsNull()) {
        // No self-intersection splitting happened — the input wire edges
        // ARE the original source edges, so fuseHist maps directly.
        myPreSplitHistory = fuseHist;
    }
    else {
        myPreSplitHistory = new BRepTools_History();
        for (const auto& w : myWires) {
            for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
                const TopoDS_Edge& origEdge = TopoDS::Edge(exp.Current());
                const TopTools_ListOfShape& fragments = preSplitHist->Modified(origEdge);
                if (fragments.IsEmpty()) {
                    // Edge was not split — trace it directly through fuse
                    const TopTools_ListOfShape& fused = fuseHist->Modified(origEdge);
                    if (!fused.IsEmpty()) {
                        for (TopTools_ListIteratorOfListOfShape fi(fused); fi.More(); fi.Next()) {
                            myPreSplitHistory->AddModified(origEdge, fi.Value());
                        }
                    }
                    else if (!fuseHist->IsRemoved(origEdge)) {
                        // Unchanged by both stages — no entry needed
                    }
                }
                else {
                    // Edge was split into fragments — trace each through fuse
                    for (TopTools_ListIteratorOfListOfShape pi(fragments); pi.More(); pi.Next()) {
                        const TopoDS_Shape& fragment = pi.Value();
                        const TopTools_ListOfShape& fused = fuseHist->Modified(fragment);
                        if (!fused.IsEmpty()) {
                            for (TopTools_ListIteratorOfListOfShape fi(fused); fi.More(); fi.Next()) {
                                myPreSplitHistory->AddModified(origEdge, fi.Value());
                            }
                        }
                        else if (!fuseHist->IsRemoved(fragment)) {
                            // Fragment unchanged by fuse — carry forward
                            myPreSplitHistory->AddModified(origEdge, fragment);
                        }
                    }
                }
            }
        }
    }

    // Rebuild compound from all output wire edges
    BRep_Builder compBuilder;
    compBuilder.MakeCompound(myPreSplitCompound);
    for (const auto& w : result) {
        for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
            compBuilder.Add(myPreSplitCompound, exp.Current());
        }
    }
    return result;
}

// Pipeline: BRepAlgoAPI_BuilderAlgo (split edges at intersections)
//         -> BOPAlgo_BuilderFace (find all bounded regions)
//         -> even-odd classification (nesting/holes)
// Uses the base class mySplitter so postBuild() can chain its history
// with myPreSplitHistory for proper element naming.

void FaceMakerFishEye::buildPlanar(const std::vector<TopoDS_Wire>& wires, const gp_Pln& plane)
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

    // Split edges at mutual intersections using the base class mySplitter
    TopTools_ListOfShape splitEdges = edges;
    if (edges.Size() > 1) {
        mySplitter.SetArguments(edges);
        mySplitter.SetRunParallel(true);
        mySplitter.SetNonDestructive(Standard_True);
        mySplitter.Build();
        if (mySplitter.IsDone()) {
            splitEdges.Clear();
            for (TopExp_Explorer exp(mySplitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
                splitEdges.Append(exp.Current());
            }
        }
    }

    Bnd_Box geomBox;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        BRepBndLib::Add(it.Value(), geomBox);
    }
    // Base face must be larger than all geometry so BuilderFace can
    // distinguish bounded regions from the unbounded exterior.
    // BuilderFace also requires FORWARD orientation on the base face.
    const Standard_Real aMax = std::max(1.0e8, 10.0 * std::sqrt(geomBox.SquareExtent()));
    TopoDS_Face baseFace = BRepBuilderAPI_MakeFace(plane, -aMax, aMax, -aMax, aMax).Face();
    baseFace.Orientation(TopAbs_FORWARD);

    // BuilderFace needs both orientations of each edge to form closed wires,
    // and 2D parametric curves (pcurves) projected onto the base face.
    TopTools_ListOfShape faceEdges;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(it.Value());
        faceEdges.Append(e.Oriented(TopAbs_FORWARD));
        faceEdges.Append(e.Oriented(TopAbs_REVERSED));
    }
    BRepLib::BuildPCurveForEdgesOnPlane(faceEdges, baseFace);

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
            myShapesToReturn.push_back(face);
            continue;
        }

        int count = 0;
        for (const auto& wf : wireFaces) {
            if (!wf.IsNull() && ctx->IsPointInFace(pt, wf, Precision::Confusion())) {
                ++count;
            }
        }

        if (count == 0 || count % 2 == 1) {
            myShapesToReturn.push_back(face);
        }
    }
}

namespace
{

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
        buildPlanar(wires, plane);
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
