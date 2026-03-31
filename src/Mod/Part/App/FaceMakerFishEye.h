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

#pragma once

#include "FaceMaker.h"

#include <Mod/Part/PartGlobal.h>

namespace Part
{

/**
 * @brief Unified face maker that handles all planar and curved face cases.
 *
 * Handles:
 * - Nested wires (outer with holes with islands, like Bullseye)
 * - Overlapping/crossing wires (splits at intersections, finds all bounded regions)
 * - Curved surfaces (not just planar faces)
 *
 * This replaces the waterfall pattern of trying Bullseye -> Cheese -> Simple
 * with a single algorithm that handles all cases correctly.
 */
class PartExport FaceMakerFishEye: public FaceMakerPublic
{
    TYPESYSTEM_HEADER_WITH_OVERRIDE();

public:
    std::string getUserFriendlyName() const override;
    std::string getBriefExplanation() const override;

protected:
    void Build_Essence() override;
};

}  // namespace Part
