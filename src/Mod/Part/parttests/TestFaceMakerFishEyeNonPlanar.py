# SPDX-License-Identifier: LGPL-2.1-or-later

"""Non-XY-plane tests for Part::FaceMakerFishEye.

These tests exercise cases that FaceMakerCheese handles via
BRepBuilderAPI_MakeFace: wires on non-XY planes, wires on different
planes, analytical surfaces, and nesting on arbitrary planes.

FishEye must handle all cases Cheese handles, plus overlapping wires,
self-intersecting BSplines, and edge splitting (tested separately in
TestFaceMakerFishEyePlanar.py).
"""

import math
import unittest

import FreeCAD
import Part

Vec = FreeCAD.Vector


def make_polygon(*points):
    vecs = [Vec(*p) if isinstance(p, (list, tuple)) else p for p in points]
    if vecs[0] != vecs[-1]:
        vecs.append(vecs[0])
    return Part.Wire(Part.makePolygon(vecs))


def fisheye(wires):
    if isinstance(wires, Part.Wire):
        wires = [wires]
    return Part.makeFace(Part.Compound(wires), "Part::FaceMakerFishEye").Faces


def total_area(faces):
    return sum(f.Area for f in faces)


# =========================================================================
# 1. Single shapes on non-XY planes
# =========================================================================


class TestSingleShapeNonXY(unittest.TestCase):
    """Single closed wires on various planes. Both Cheese and FishEye
    handle these via BRepBuilderAPI_MakeFace auto-detection."""

    def test_rectangle_xz(self):
        w = make_polygon((0, 0, 0), (10, 0, 0), (10, 0, 10), (0, 0, 10))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_rectangle_yz(self):
        w = make_polygon((0, 0, 0), (0, 10, 0), (0, 10, 10), (0, 0, 10))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_rectangle_tilted(self):
        s = 10 / math.sqrt(2)
        w = make_polygon((0, 0, 0), (10, 0, 0), (10, s, s), (0, s, s))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=1)

    def test_triangle_arbitrary(self):
        w = make_polygon((1, 2, 3), (11, 2, 3), (6, 10, 8))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)

    def test_circle_xz(self):
        w = Part.Wire(Part.makeCircle(10, Vec(0, 0, 0), Vec(0, 1, 0)))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * 100, places=1)

    def test_circle_tilted(self):
        w = Part.Wire(Part.makeCircle(10, Vec(0, 0, 0), Vec(1, 1, 1)))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * 100, places=1)

    def test_ellipse(self):
        ell = Part.Ellipse(Vec(0, 0, 0), 15, 8)
        w = Part.Wire(ell.toShape())
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * 15 * 8, places=0)


# =========================================================================
# 2. Multiple wires on different planes
# =========================================================================


class TestDifferentPlanes(unittest.TestCase):
    """Wires on different planes in one call. Cheese processes each
    independently. FishEye should do the same."""

    def test_two_rects_xy_and_xz(self):
        w1 = make_polygon((0, 0, 0), (10, 0, 0), (10, 10, 0), (0, 10, 0))
        w2 = make_polygon((20, 0, 0), (30, 0, 0), (30, 0, 10), (20, 0, 10))
        faces = fisheye([w1, w2])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 200.0, places=3)

    def test_two_circles_different_normals(self):
        c1 = Part.Wire(Part.makeCircle(8, Vec(0, 0, 0), Vec(0, 0, 1)))
        c2 = Part.Wire(Part.makeCircle(8, Vec(30, 0, 0), Vec(0, 1, 0)))
        faces = fisheye([c1, c2])
        self.assertEqual(len(faces), 2)

    def test_three_rects_xyz(self):
        w_xy = make_polygon((0, 0, 0), (5, 0, 0), (5, 5, 0), (0, 5, 0))
        w_xz = make_polygon((10, 0, 0), (15, 0, 0), (15, 0, 5), (10, 0, 5))
        w_yz = make_polygon((0, 10, 0), (0, 15, 0), (0, 15, 5), (0, 10, 5))
        faces = fisheye([w_xy, w_xz, w_yz])
        self.assertEqual(len(faces), 3)
        self.assertAlmostEqual(total_area(faces), 75.0, places=3)


# =========================================================================
# 3. Nesting on non-XY planes
# =========================================================================


class TestNestingNonXY(unittest.TestCase):
    """Hole-in-face on non-XY planes."""

    def test_circle_hole_on_xz(self):
        """Rectangle with circular hole, both on XZ plane."""
        outer = make_polygon((0, 0, 0), (20, 0, 0), (20, 0, 20), (0, 0, 20))
        inner = Part.Wire(Part.makeCircle(5, Vec(10, 0, 10), Vec(0, 1, 0)))
        faces = fisheye([outer, inner])
        self.assertEqual(len(faces), 1)
        expected = 400.0 - math.pi * 25
        self.assertAlmostEqual(faces[0].Area, expected, places=0)

    def test_concentric_circles_tilted(self):
        """Two concentric circles on a tilted plane → annulus."""
        normal = Vec(1, 1, 1)
        center = Vec(0, 0, 0)
        outer = Part.Wire(Part.makeCircle(10, center, normal))
        inner = Part.Wire(Part.makeCircle(5, center, normal))
        faces = fisheye([outer, inner])
        self.assertEqual(len(faces), 1)
        expected = math.pi * (100 - 25)
        self.assertAlmostEqual(faces[0].Area, expected, places=1)


# =========================================================================
# 4. Analytical surfaces (cylinder wire, etc.)
# =========================================================================


class TestAnalyticalSurfaces(unittest.TestCase):
    """Wires from analytical surfaces that MakeFace can auto-detect."""

    def test_cylinder_lateral_wire(self):
        """Outer wire of a cylinder lateral face → cylindrical face."""
        cyl = Part.makeCylinder(10, 20)
        lateral = [f for f in cyl.Faces if f.Surface.TypeId == "Part::GeomCylinder"][0]
        wire = lateral.OuterWire
        faces = fisheye(wire)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)

    def test_cone_lateral_wire(self):
        """Outer wire of a cone lateral face."""
        cone = Part.makeCone(10, 5, 20)
        lateral = [f for f in cone.Faces if f.Surface.TypeId == "Part::GeomCone"][0]
        wire = lateral.OuterWire
        faces = fisheye(wire)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)


# =========================================================================
# 5. Edge cases
# =========================================================================


class TestEdgeCases(unittest.TestCase):
    def test_near_planar(self):
        """Z offset below tolerance → treated as planar."""
        w = make_polygon((0, 0, 0), (10, 0, 0), (10, 10, 1e-8), (0, 10, 0))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=1)

    def test_bspline_coplanar_non_xy(self):
        """BSpline whose points are coplanar on a tilted plane."""
        # All points on the plane X + Z = 10
        pts = [Vec(0, 0, 10), Vec(5, 5, 5), Vec(10, 0, 0), Vec(5, -5, 5)]
        bs = Part.BSplineCurve()
        bs.interpolate(pts, PeriodicFlag=True)
        w = Part.Wire(bs.toShape())
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)
