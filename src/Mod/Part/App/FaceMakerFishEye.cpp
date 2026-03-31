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
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepLib_FindSurface.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis.hxx>
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

// ─── Helpers ─────────────────────────────────────────────────────────────────

namespace
{

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

// Detect partial overlaps, group with union-find, fuse each group.
// Returns the resulting non-overlapping wires.
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

// ─── Phase 2: Planar face building via BOPAlgo_BuilderFace ───────────────────

bool buildPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                      std::vector<TopoDS_Shape>& result,
                      const gp_Pln* suppliedPlane)
{
    // Collect all edges
    TopTools_ListOfShape edgeList;
    for (const auto& w : wires) {
        for (TopExp_Explorer exp(w, TopAbs_EDGE); exp.More(); exp.Next()) {
            edgeList.Append(exp.Current());
        }
    }
    if (edgeList.IsEmpty()) {
        return false;
    }

    // Split edges at mutual intersections
    TopTools_ListOfShape splitEdges;
    if (edgeList.Size() > 1) {
        BRepAlgoAPI_BuilderAlgo splitter;
        splitter.SetArguments(edgeList);
        splitter.SetRunParallel(true);
        splitter.SetNonDestructive(Standard_True);
        splitter.Build();
        if (!splitter.IsDone()) {
            return false;
        }
        for (TopExp_Explorer exp(splitter.Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
            splitEdges.Append(exp.Current());
        }
    }
    else {
        splitEdges = edgeList;
    }

    // Find the plane
    gp_Pln plane;
    if (suppliedPlane) {
        plane = *suppliedPlane;
    }
    else {
        BRep_Builder builder;
        TopoDS_Compound edgeCompound;
        builder.MakeCompound(edgeCompound);
        for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
            builder.Add(edgeCompound, it.Value());
        }
        BRepLib_FindSurface planeFinder(edgeCompound, -1, Standard_True);
        if (!planeFinder.Found()) {
            return false;  // not planar — caller should try non-planar path
        }
        plane = GeomAdaptor_Surface(planeFinder.Surface()).Plane();
    }

    // Build a large base face on the plane
    const Standard_Real aMax = 1.0e8;
    TopoDS_Face baseFace = BRepBuilderAPI_MakeFace(plane, -aMax, aMax, -aMax, aMax).Face();
    baseFace.Orientation(TopAbs_FORWARD);

    // Add split edges in both orientations and build pcurves
    TopTools_ListOfShape edgesForFace;
    for (TopTools_ListIteratorOfListOfShape it(splitEdges); it.More(); it.Next()) {
        const TopoDS_Edge& e = TopoDS::Edge(it.Value());
        edgesForFace.Append(e.Oriented(TopAbs_FORWARD));
        edgesForFace.Append(e.Oriented(TopAbs_REVERSED));
    }
    BRepLib::BuildPCurveForEdgesOnPlane(edgesForFace, baseFace);

    // Run BOPAlgo_BuilderFace — finds all bounded regions, classifies holes
    BOPAlgo_BuilderFace faceBuilder;
    faceBuilder.SetFace(baseFace);
    faceBuilder.SetShapes(edgesForFace);
    faceBuilder.Perform();
    if (faceBuilder.HasErrors()) {
        return false;
    }

    // Collect bounded regions (filter out the unbounded outer face)
    const double outerExtentThreshold = aMax * aMax;
    const TopTools_ListOfShape& builtFaces = faceBuilder.Areas();
    std::vector<TopoDS_Face> regions;
    for (TopTools_ListIteratorOfListOfShape it(builtFaces); it.More(); it.Next()) {
        Bnd_Box box;
        BRepBndLib::Add(it.Value(), box);
        if (box.SquareExtent() > outerExtentThreshold) {
            continue;
        }
        regions.push_back(TopoDS::Face(it.Value()));
    }

    if (regions.empty()) {
        return false;
    }

    // BOPAlgo_BuilderFace produces ring faces (with holes) AND inner disc faces.
    // Apply even-odd fill: count how many input wires contain each region.
    // Use the center of mass of the region's OUTER WIRE (not the face) as
    // the test point — the face center of mass can fall inside a hole.
    //
    // Build faces from input wires for containment testing
    std::vector<TopoDS_Face> wireFaces;
    wireFaces.reserve(wires.size());
    for (const auto& w : wires) {
        wireFaces.push_back(makeFaceFromWire(w));
    }

    for (const auto& region : regions) {
        // Get a test point: midpoint of the outer wire's first edge.
        // This lies on the region boundary, which is ON a wire — so we
        // offset slightly inward using the face normal and edge tangent.
        TopoDS_Wire outerW = BRepTools::OuterWire(region);
        TopExp_Explorer edgeExp(outerW, TopAbs_EDGE);
        if (!edgeExp.More()) {
            continue;
        }
        const TopoDS_Edge& testEdge = TopoDS::Edge(edgeExp.Current());
        Standard_Real eFirst, eLast;
        BRep_Tool::Range(testEdge, eFirst, eLast);
        BRepAdaptor_Curve adaptor(testEdge);
        double eMid = (eFirst + eLast) / 2.0;
        gp_Pnt edgePt = adaptor.Value(eMid);

        // Offset inward: cross the face normal with the edge tangent
        gp_Vec tangent;
        gp_Pnt unused;
        adaptor.D1(eMid, unused, tangent);
        gp_Vec normal = gp_Vec(plane.Axis().Direction());
        gp_Vec inward = normal.Crossed(tangent);
        if (inward.Magnitude() > Precision::Confusion()) {
            inward.Normalize();
        }
        // Small offset to get strictly inside the face
        gp_Pnt testPt = edgePt.Translated(inward * Precision::Confusion() * 100);

        int containCount = 0;
        for (const auto& wf : wireFaces) {
            if (wf.IsNull()) {
                continue;
            }
            BRepClass_FaceClassifier classifier(wf, testPt, Precision::Confusion());
            if (classifier.State() == TopAbs_IN) {
                ++containCount;
            }
        }

        if (containCount % 2 == 1) {
            result.push_back(region);
        }
    }

    return true;
}

// ─── Phase 3: Non-planar face building ───────────────────────────────────────

void buildNonPlanarFaces(const std::vector<TopoDS_Wire>& wires,
                         std::vector<TopoDS_Shape>& result)
{
    // Sort wires by bounding box extent (largest first)
    struct WireBox
    {
        TopoDS_Wire wire;
        Bnd_Box box;
        double extent;
    };
    std::vector<WireBox> sorted;
    sorted.reserve(wires.size());
    for (const auto& w : wires) {
        WireBox wb;
        wb.wire = w;
        BRepBndLib::Add(w, wb.box);
        wb.extent = wb.box.SquareExtent();
        sorted.push_back(std::move(wb));
    }
    std::stable_sort(sorted.begin(), sorted.end(),
                     [](const WireBox& a, const WireBox& b) { return a.extent > b.extent; });

    // Build faces: largest wire first, smaller wires become holes if inside
    std::vector<TopoDS_Face> faces;
    for (const auto& wb : sorted) {
        // Check if this wire is inside any existing face
        bool addedAsHole = false;
        for (auto& face : faces) {
            // Get a point on the wire for hit testing
            TopExp_Explorer vExp(wb.wire, TopAbs_VERTEX);
            if (!vExp.More()) {
                continue;
            }
            gp_Pnt testPt = BRep_Tool::Pnt(TopoDS::Vertex(vExp.Current()));
            BRepClass_FaceClassifier classifier(face, testPt, Precision::Confusion());
            if (classifier.State() == TopAbs_IN) {
                // Wire is inside this face — add as hole
                BRepBuilderAPI_MakeFace mf(face);
                TopoDS_Wire holeWire = wb.wire;
                // Check orientation: hole wires must have opposite orientation to outer
                TopoDS_Wire outerWire = BRepTools::OuterWire(face);
                BRepAdaptor_Surface adapt(face);
                if (adapt.GetType() == GeomAbs_Plane) {
                    gp_Dir axis = adapt.Plane().Axis().Direction();
                    BRepBuilderAPI_MakeFace testFace(holeWire);
                    if (testFace.IsDone()) {
                        BRepAdaptor_Surface innerAdapt(testFace.Face());
                        if (innerAdapt.GetType() == GeomAbs_Plane) {
                            if (axis.Dot(innerAdapt.Plane().Axis().Direction()) > 0) {
                                holeWire.Reverse();
                            }
                        }
                    }
                }
                mf.Add(holeWire);
                if (mf.IsDone()) {
                    face = mf.Face();
                    addedAsHole = true;
                    break;
                }
            }
        }

        if (!addedAsHole) {
            // Not inside any face — create a new face
            BRepBuilderAPI_MakeFace mf(wb.wire);
            if (mf.IsDone()) {
                // Validate
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
    if (buildPlanarFaces(wires, myShapesToReturn, planeSupplied ? &myPlane : nullptr)) {
        return;
    }

    // Phase 3: Non-planar fallback
    buildNonPlanarFaces(wires, myShapesToReturn);
}
