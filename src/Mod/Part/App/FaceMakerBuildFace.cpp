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
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepLib.hxx>
#include <BRepLib_FindSurface.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>

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

    // Step 2: Split edges at all mutual intersections
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

    // Step 2b: Remove dangling edges — edges where a vertex has degree 1.
    // Open wires that don't fully cross a region (incomplete intersections)
    // leave free endpoints that would create artifact faces in BuilderFace.
    // Prune iteratively to handle chains of dangling segments.
    BRep_Builder builder;
    TopoDS_Compound edgeCompound;
    {
        bool changed = true;
        while (changed) {
            changed = false;
            builder.MakeCompound(edgeCompound);
            for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
                builder.Add(edgeCompound, it.Value());
            }
            TopTools_IndexedDataMapOfShapeListOfShape vertexEdgeMap;
            TopExp::MapShapesAndAncestors(edgeCompound, TopAbs_VERTEX, TopAbs_EDGE, vertexEdgeMap);

            TopTools_ListOfShape filtered;
            for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
                const TopoDS_Edge& e = TopoDS::Edge(it.Value());
                TopoDS_Vertex v1, v2;
                TopExp::Vertices(e, v1, v2);
                bool dangling = false;
                if (!v1.IsNull() && vertexEdgeMap.Contains(v1)
                    && vertexEdgeMap.FindFromKey(v1).Size() == 1) {
                    dangling = true;
                }
                if (!v2.IsNull() && !v1.IsSame(v2) && vertexEdgeMap.Contains(v2)
                    && vertexEdgeMap.FindFromKey(v2).Size() == 1) {
                    dangling = true;
                }
                if (!dangling) {
                    filtered.Append(e);
                }
                else {
                    changed = true;
                }
            }
            splitEdges = filtered;
        }
        if (splitEdges.IsEmpty()) {
            return;
        }
    }

    // Step 3: Determine the plane (reuses edgeCompound from Step 2b)
    gp_Pln plane;
    if (planeSupplied) {
        plane = myPlane;
    }
    else {
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
