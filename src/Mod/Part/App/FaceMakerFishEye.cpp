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
#include "FaceMakerBullseye.h"

#include <BRep_Tool.hxx>
#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
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

double faceArea(const TopoDS_Shape& s)
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
    double commonArea = faceArea(common.Shape());
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

}  // namespace


void FaceMakerFishEye::Build_Essence()
{
    if (myWires.empty()) {
        return;
    }

    // Build WireFace info for each wire
    std::vector<WireFace> wfs;
    wfs.reserve(myWires.size());
    for (const auto& w : myWires) {
        WireFace wf;
        wf.wire = w;
        wf.face = makeFaceFromWire(w);
        if (wf.isValid()) {
            BRepBndLib::AddOptimal(w, wf.box, Standard_False);
            wf.area = faceArea(wf.face);
        }
        wfs.push_back(std::move(wf));
    }

    // Detect partial overlaps and group them with union-find
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

    // If no overlaps, delegate entirely to Bullseye
    if (!hasOverlaps) {
        FaceMakerBullseye bullseye;
        if (planeSupplied) {
            bullseye.setPlane(myPlane);
        }
        for (const auto& w : myWires) {
            bullseye.addWire(w);
        }
        bullseye.Build();
        for (TopExp_Explorer exp(bullseye.Shape(), TopAbs_FACE); exp.More(); exp.Next()) {
            myShapesToReturn.push_back(exp.Current());
        }
        return;
    }

    // Group wires by overlap group
    std::map<int, std::vector<size_t>> groups;
    for (size_t i = 0; i < wfs.size(); ++i) {
        groups[findRoot(parent, static_cast<int>(i))].push_back(i);
    }

    // Process each group: fuse overlapping wires, keep singles as-is
    std::vector<TopoDS_Wire> finalWires;
    for (auto& [root, indices] : groups) {
        if (indices.size() == 1) {
            finalWires.push_back(wfs[indices[0]].wire);
            continue;
        }

        // Fuse all faces in this overlap group
        TopTools_ListOfShape toFuse;
        for (size_t idx : indices) {
            if (wfs[idx].isValid()) {
                toFuse.Append(wfs[idx].face);
            }
        }
        if (toFuse.Size() < 2) {
            for (size_t idx : indices) {
                finalWires.push_back(wfs[idx].wire);
            }
            continue;
        }

        BRepAlgoAPI_BuilderAlgo fuser;
        fuser.SetArguments(toFuse);
        fuser.Build();
        if (!fuser.IsDone()) {
            FC_WARN("FaceMakerFishEye: failed to fuse overlapping wires");
            for (size_t idx : indices) {
                finalWires.push_back(wfs[idx].wire);
            }
            continue;
        }

        // Extract outer wire(s) from fused result faces
        for (TopExp_Explorer fExp(fuser.Shape(), TopAbs_FACE); fExp.More(); fExp.Next()) {
            TopoDS_Face fusedFace = TopoDS::Face(fExp.Current());
            TopoDS_Wire outerWire = BRepTools::OuterWire(fusedFace);
            if (!outerWire.IsNull()) {
                finalWires.push_back(outerWire);
            }
        }
    }

    // Delegate final (non-overlapping) wires to Bullseye for nesting
    FaceMakerBullseye bullseye;
    if (planeSupplied) {
        bullseye.setPlane(myPlane);
    }
    for (const auto& w : finalWires) {
        bullseye.addWire(w);
    }
    bullseye.Build();

    for (TopExp_Explorer exp(bullseye.Shape(), TopAbs_FACE); exp.More(); exp.Next()) {
        myShapesToReturn.push_back(exp.Current());
    }
}
