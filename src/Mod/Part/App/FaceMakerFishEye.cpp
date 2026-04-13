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
#include <BOPTools_AlgoTools3D.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepLib_FindSurface.hxx>
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
#include <numeric>

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

TopoDS_Face makeFaceFromWire(const TopoDS_Wire& w, const gp_Pln& plane)
{
    if (!BRep_Tool::IsClosed(w)) {
        return {};
    }
    BRepBuilderAPI_MakeFace mf(plane, w);
    return mf.IsDone() ? mf.Face() : TopoDS_Face();
}

double shapeArea(const TopoDS_Shape& s)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(s, props);
    return props.Mass();
}

int findRoot(std::vector<int>& parent, int i)
{
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

// Detect partially overlapping wire groups using union-find.
// Returns a group ID per wire.  Wires that partially overlap share a group.
// Full containment (hole-in-outer) is NOT grouped — even-odd handles that.
std::vector<int> findOverlapGroups(const std::vector<TopoDS_Wire>& wires, const gp_Pln& plane)
{
    int n = static_cast<int>(wires.size());
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);

    struct WireInfo
    {
        TopoDS_Face face;
        Bnd_Box box;
        double area = 0;
    };
    std::vector<WireInfo> info(n);

    int closedCount = 0;
    for (int i = 0; i < n; ++i) {
        if (!BRep_Tool::IsClosed(wires[i])) {
            continue;
        }
        info[i].face = makeFaceFromWire(wires[i], plane);
        if (!info[i].face.IsNull()) {
            BRepBndLib::AddOptimal(wires[i], info[i].box, Standard_False);
            info[i].area = shapeArea(info[i].face);
            ++closedCount;
        }
    }
    if (closedCount < 2) {
        return parent;
    }

    const double tol = Precision::Confusion();
    for (int i = 0; i < n; ++i) {
        if (info[i].face.IsNull()) {
            continue;
        }
        for (int j = i + 1; j < n; ++j) {
            if (info[j].face.IsNull() || info[i].box.IsOut(info[j].box)) {
                continue;
            }
            BRepAlgoAPI_Common common(info[i].face, info[j].face);
            if (!common.IsDone() || common.Shape().IsNull()) {
                continue;
            }
            double ca = shapeArea(common.Shape());
            // Partial overlap only — exclude full containment
            if (ca > tol && ca < info[i].area - tol && ca < info[j].area - tol) {
                parent[findRoot(parent, i)] = findRoot(parent, j);
            }
        }
    }
    // Flatten
    for (int i = 0; i < n; ++i) {
        parent[i] = findRoot(parent, i);
    }
    return parent;
}

}  // namespace

bool FaceMakerFishEye::findPlane(const std::vector<TopoDS_Wire>& wires, gp_Pln& plane) const
{
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    for (const auto& w : wires) {
        builder.Add(comp, BRepBuilderAPI_Copy(w).Shape());
    }
    BRepLib_FindSurface planeFinder(comp, -1, Standard_True);
    if (!planeFinder.Found()) {
        return false;
    }
    plane = GeomAdaptor_Surface(planeFinder.Surface()).Plane();

    if (planeSupplied && plane.Axis().Direction().Dot(myPlane.Axis().Direction()) < 0) {
        plane = gp_Pln(plane.Location(), plane.Axis().Direction().Reversed());
    }
    return true;
}

// Identical approach to FaceMakerBuildFace: flat edge list in/out,
// records original → fragment in myPreSplitHistory + myPreSplitCompound.
TopTools_ListOfShape FaceMakerFishEye::splitSelfIntersecting(
    const TopTools_ListOfShape& edges,
    const gp_Pln& plane
)
{
    const Standard_Real tol = Precision::Confusion();
    TopTools_ListOfShape result;

    for (TopTools_ListIteratorOfListOfShape it(edges); it.More(); it.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(it.Value());
        try {
            Standard_Real first {}, last {};
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
            if (curve.IsNull() || curve->IsKind(STANDARD_TYPE(Geom_Line))
                || curve->IsKind(STANDARD_TYPE(Geom_Conic))) {
                result.Append(edge);
                continue;
            }
            Handle(Geom2d_Curve) curve2d = GeomAPI::To2d(curve, plane);
            if (curve2d.IsNull()) {
                result.Append(edge);
                continue;
            }
            Geom2dAPI_InterCurveCurve selfInt(curve2d, tol);
            if (selfInt.NbPoints() == 0) {
                result.Append(edge);
                continue;
            }
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
                result.Append(edge);
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
                    result.Append(fi.Value());
                }
            }
            else {
                result.Append(edge);
            }
        }
        catch (const Standard_Failure& e) {
            if (FC_LOG_INSTANCE.isEnabled(FC_LOGLEVEL_LOG)) {
                FC_WARN("splitSelfIntersecting: " << e.GetMessageString());
            }
            result.Append(edge);
        }
    }
    if (!myPreSplitHistory.IsNull()) {
        BRep_Builder builder;
        builder.MakeCompound(myPreSplitCompound);
        for (TopTools_ListIteratorOfListOfShape ri(result); ri.More(); ri.Next()) {
            builder.Add(myPreSplitCompound, ri.Value());
        }
    }
    return result;
}

void FaceMakerFishEye::buildPlanar(const TopTools_ListOfShape& edges, const gp_Pln& plane)
{
    if (edges.IsEmpty()) {
        return;
    }

    // Split edges at mutual intersections
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

    // Build a large planar base face for BOPAlgo_BuilderFace
    Bnd_Box geomBox;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        BRepBndLib::Add(it.Value(), geomBox);
    }
    const Standard_Real aMax = std::max(1.0e8, 10.0 * std::sqrt(geomBox.SquareExtent()));
    TopoDS_Face baseFace = BRepBuilderAPI_MakeFace(plane, -aMax, aMax, -aMax, aMax).Face();
    baseFace.Orientation(TopAbs_FORWARD);

    // Add edges in both orientations with PCurves
    TopTools_ListOfShape faceEdges;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(it.Value());
        faceEdges.Append(e.Oriented(TopAbs_FORWARD));
        faceEdges.Append(e.Oriented(TopAbs_REVERSED));
    }
    BRepLib::BuildPCurveForEdgesOnPlane(faceEdges, baseFace);

    // Find all bounded face regions
    BOPAlgo_BuilderFace faceBuilder;
    faceBuilder.SetFace(baseFace);
    faceBuilder.SetShapes(faceEdges);
    faceBuilder.SetAvoidInternalShapes(Standard_True);
    faceBuilder.Perform();
    if (faceBuilder.HasErrors()) {
        return;
    }

    // Collect bounded faces
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

    // Overlap-group-aware even-odd classification.
    // Count containment per overlap group (not per wire) so that
    // partially overlapping wires act as a union outline while
    // fully nested wires still create holes via even-odd.
    std::vector<int> groups = findOverlapGroups(myWires, plane);

    Handle(IntTools_Context) ctx = new IntTools_Context();
    std::vector<TopoDS_Face> wireFaces;
    wireFaces.reserve(myWires.size());
    for (const auto& w : myWires) {
        wireFaces.push_back(BRep_Tool::IsClosed(w) ? makeFaceFromWire(w, plane) : TopoDS_Face());
    }

    for (const auto& face : allFaces) {
        gp_Pnt pt;
        gp_Pnt2d pt2d;
        if (BOPTools_AlgoTools3D::PointInFace(face, pt, pt2d, ctx) != 0) {
            myShapesToReturn.push_back(face);
            continue;
        }

        // Count how many overlap groups contain this point.
        // A group "contains" the point if ANY wire in the group does.
        std::set<int> containingGroups;
        for (size_t i = 0; i < wireFaces.size(); ++i) {
            if (!wireFaces[i].IsNull()
                && ctx->IsPointInFace(pt, wireFaces[i], Precision::Confusion())) {
                containingGroups.insert(groups[i]);
            }
        }

        int groupCount = static_cast<int>(containingGroups.size());
        if (groupCount == 0 || groupCount % 2 == 1) {
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

    gp_Pln plane;
    bool planar = findPlane(myWires, plane);
    if (!planar && planeSupplied) {
        plane = myPlane;
        planar = true;
    }

    if (planar) {
        // Collect all edges from input wires
        TopTools_ListOfShape edges;
        for (const auto& w : myWires) {
            for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
                edges.Append(exp.Current());
            }
        }
        edges = splitSelfIntersecting(edges, plane);
        buildPlanar(edges, plane);
    }
    else {
        for (const auto& w : myWires) {
            BRepBuilderAPI_MakeFace mf(w);
            TopoDS_Face face = mf.IsDone() ? mf.Face() : TopoDS_Face();
            if (face.IsNull() && BRep_Tool::IsClosed(w)) {
                face = fillNonPlanar(w);
            }
            if (!face.IsNull() && shapeArea(face) > Precision::Confusion()) {
                myShapesToReturn.push_back(face);
            }
        }
    }
}
