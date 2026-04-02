// SPDX-License-Identifier: LGPL-2.1-or-later

/****************************************************************************
 *                                                                          *
 *   Copyright (c) 2026 FreeCAD contributors                               *
 *                                                                          *
 *   This file is part of FreeCAD.                                          *
 *                                                                          *
 *   FreeCAD is free software: you can redistribute it and/or modify it     *
 *   under the terms of the GNU Lesser General Public License as            *
 *   published by the Free Software Foundation, either version 2.1 of the   *
 *   License, or (at your option) any later version.                        *
 *                                                                          *
 *   FreeCAD is distributed in the hope that it will be useful, but         *
 *   WITHOUT ANY WARRANTY; without even the implied warranty of             *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU       *
 *   Lesser General Public License for more details.                        *
 *                                                                          *
 *   You should have received a copy of the GNU Lesser General Public       *
 *   License along with FreeCAD. If not, see                                *
 *   <https://www.gnu.org/licenses/>.                                       *
 *                                                                          *
 ***************************************************************************/

#include "FaceMakerBuildFace.h"

#include <Bnd_Box.hxx>
#include <BOPAlgo_BuilderFace.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepLib.hxx>
#include <BRepLib_FindSurface.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomAPI.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <algorithm>

#include <Base/Console.h>

FC_LOG_LEVEL_INIT("FaceMakerBuildFace", true, true)

TYPESYSTEM_SOURCE(Part::FaceMakerBuildFace, Part::FaceMakerPublic)

std::string Part::FaceMakerBuildFace::getUserFriendlyName() const
{
    return tr("BuildFace facemaker").toStdString();
}

std::string Part::FaceMakerBuildFace::getBriefExplanation() const
{
    return tr("Splits edges at intersections and finds all bounded face regions. "
              "Handles arbitrary overlapping geometry.")
        .toStdString();
}

void Part::FaceMakerBuildFace::setPlane(const gp_Pln& plane)
{
    myPlane = plane;
    planeSupplied = true;
}

void Part::FaceMakerBuildFace::Build_Essence()
{
    // Step 1: Collect all edges from input wires
    TopTools_ListOfShape edgeList;
    for (const TopoDS_Wire& w : myWires) {
        for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
            edgeList.Append(exp.Current());
        }
    }
    if (edgeList.IsEmpty()) {
        return;
    }

    // Step 2: Split self-intersecting edges, then split all edges at mutual intersections.

    // Step 2a: Detect self-intersecting edges and split them at crossing points.
    // BRepAlgoAPI_BuilderAlgo only splits edges against OTHER edges, not an edge
    // against itself (e.g., a figure-8 BSpline).
    {
        TopTools_ListOfShape expanded;
        const Standard_Real tol = Precision::Confusion();
        for (TopTools_ListIteratorOfListOfShape it(edgeList); it.More(); it.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(it.Value());
            Standard_Real first, last;
            Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, first, last);
            if (curve3d.IsNull()) {
                expanded.Append(edge);
                continue;
            }

            // Project 3D curve to 2D for self-intersection detection
            gp_Pln xyPlane;
            Handle(Geom2d_Curve) curve2d = GeomAPI::To2d(curve3d, xyPlane);
            if (curve2d.IsNull()) {
                expanded.Append(edge);
                continue;
            }

            Geom2dAPI_InterCurveCurve selfInt(curve2d, tol);
            if (selfInt.NbPoints() == 0) {
                expanded.Append(edge);
                continue;
            }

            // For each self-intersection point, find ALL parameter values
            // where the curve passes through it (a crossing has 2 parameters).
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
                expanded.Append(edge);
                continue;
            }

            // Sort and deduplicate, then split into sub-edges
            std::sort(params.begin(), params.end());
            params.erase(std::unique(params.begin(), params.end(),
                                     [tol](double a, double b) { return b - a < tol; }),
                         params.end());

            Standard_Real prev = first;
            bool didSplit = false;
            for (Standard_Real p : params) {
                if (p - prev > tol) {
                    BRepBuilderAPI_MakeEdge me(curve3d, prev, p);
                    if (me.IsDone()) {
                        expanded.Append(me.Edge());
                        didSplit = true;
                    }
                    prev = p;
                }
            }
            if (last - prev > tol) {
                BRepBuilderAPI_MakeEdge me(curve3d, prev, last);
                if (me.IsDone()) {
                    expanded.Append(me.Edge());
                    didSplit = true;
                }
            }
            if (!didSplit) {
                expanded.Append(edge);
            }
        }
        edgeList = expanded;
    }

    // Step 2b: Split all edges at mutual intersections and merge shared vertices
    TopTools_ListOfShape splitEdges;
    if (edgeList.Size() > 1) {
        BRepAlgoAPI_BuilderAlgo splitter;
        splitter.SetArguments(edgeList);
        splitter.SetRunParallel(true);
        splitter.SetNonDestructive(Standard_True);
        splitter.Build();
        if (!splitter.IsDone()) {
            FC_WARN("FaceMakerBuildFace: failed to split edges at intersections");
            return;
        }
        for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
            splitEdges.Append(exp.Current());
        }
    }
    else {
        splitEdges = edgeList;
    }

    // Step 3: Determine the plane
    gp_Pln plane;
    if (planeSupplied) {
        plane = myPlane;
    }
    else {
        // Find a common plane from the edges
        BRep_Builder builder;
        TopoDS_Compound edgeCompound;
        builder.MakeCompound(edgeCompound);
        for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
            builder.Add(edgeCompound, it.Value());
        }
        BRepLib_FindSurface planeFinder(edgeCompound, -1, Standard_True);
        if (!planeFinder.Found()) {
            FC_WARN("FaceMakerBuildFace: edges are not coplanar");
            return;
        }
        plane = GeomAdaptor_Surface(planeFinder.Surface()).Plane();
    }

    // Step 4: Build a large planar base face
    const Standard_Real aMax = 1.0e8;
    TopoDS_Face baseFace = BRepBuilderAPI_MakeFace(plane, -aMax, aMax, -aMax, aMax).Face();
    baseFace.Orientation(TopAbs_FORWARD);

    // Step 5: Add each split edge in both orientations and build PCurves
    TopTools_ListOfShape edgesForFace;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(it.Value());
        edgesForFace.Append(e.Oriented(TopAbs_FORWARD));
        edgesForFace.Append(e.Oriented(TopAbs_REVERSED));
    }
    BRepLib::BuildPCurveForEdgesOnPlane(edgesForFace, baseFace);

    // Step 6: Run BOPAlgo_BuilderFace to find all bounded face regions
    BOPAlgo_BuilderFace faceBuilder;
    faceBuilder.SetFace(baseFace);
    faceBuilder.SetShapes(edgesForFace);
    faceBuilder.Perform();
    if (faceBuilder.HasErrors()) {
        FC_WARN("FaceMakerBuildFace: BOPAlgo_BuilderFace failed");
        return;
    }

    // Step 7: Collect result faces, excluding the unbounded outer face
    const double outerExtentThreshold = aMax * aMax;
    const TopTools_ListOfShape& builtFaces = faceBuilder.Areas();
    for (TopTools_ListIteratorOfListOfShape it(builtFaces); it.More(); it.Next()) {
        Bnd_Box box;
        BRepBndLib::Add(it.Value(), box);
        if (box.SquareExtent() > outerExtentThreshold) {
            continue;
        }
        myShapesToReturn.push_back(it.Value());
    }
}
