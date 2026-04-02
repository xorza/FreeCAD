# SPDX-License-Identifier: LGPL-2.1-or-later

"""Non-planar tests for Part::FaceMakerFishEye.

These tests exercise the non-planar code paths (BRepFill_Filling) by
constructing 3D wires (vertices not coplanar) and calling Part.makeFace
directly.
"""

import math
import unittest

import FreeCAD
import Part

Vec = FreeCAD.Vector


def make_polygon_wire(*points):
    """Create a closed wire from 3D points (auto-closes if needed)."""
    vecs = [Vec(*p) if isinstance(p, (list, tuple)) else p for p in points]
    if vecs[0] != vecs[-1]:
        vecs.append(vecs[0])
    return Part.Wire(Part.makePolygon(vecs))


def fisheye(wires):
    """Run FaceMakerFishEye on wires, return list of faces."""
    if isinstance(wires, Part.Wire):
        wires = [wires]
    return Part.makeFace(Part.Compound(wires), "Part::FaceMakerFishEye").Faces


def make_bspline_wire(points, closed=True):
    """Create a wire from a BSpline curve interpolating 3D points."""
    vecs = [Vec(*p) if isinstance(p, (list, tuple)) else p for p in points]
    bsp = Part.BSplineCurve()
    bsp.interpolate(vecs, PeriodicFlag=closed)
    return Part.Wire(bsp.toShape())


# =========================================================================
# 1. Planar wires on non-XY planes (plane detection path)
# =========================================================================


class TestFishEyeAlternativePlanes(unittest.TestCase):
    """Wires that are planar but NOT on the XY plane.
    These exercise the plane-detection path, not the filling path."""

    def test_rectangle_on_xz_plane(self):
        w = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, 0, 10), (0, 0, 10))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_rectangle_on_yz_plane(self):
        w = make_polygon_wire((0, 0, 0), (0, 10, 0), (0, 10, 10), (0, 0, 10))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_rectangle_on_tilted_plane(self):
        s = 10 / math.sqrt(2)
        w = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, s, s), (0, s, s))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=1)

    def test_triangle_on_arbitrary_plane(self):
        w = make_polygon_wire((1, 2, 3), (11, 2, 3), (6, 10, 8))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)

    def test_bspline_planar(self):
        """Closed BSpline whose control points are coplanar."""
        w = make_bspline_wire([(0, 0, 0), (10, 0, 3), (10, 10, 5), (0, 10, 2)], closed=True)
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)


# =========================================================================
# 2. Single non-planar wire -> 1 face via BRepFill_Filling
# =========================================================================


class TestFishEyeNonPlanarSingle(unittest.TestCase):
    """Single truly non-planar closed wire -> 1 filled face."""

    def test_quad_slight_z(self):
        """One vertex slightly off-plane (Z=1)."""
        w = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, 10, 1), (0, 10, 0))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)

    def test_quad_large_z(self):
        """Significant Z variation."""
        w = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, 10, 5), (0, 10, 3))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)

    def test_pentagon_3d(self):
        """All vertices at different Z."""
        w = make_polygon_wire((0, 0, 0), (10, 0, 2), (12, 8, 5), (5, 14, 3), (-2, 8, 1))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)

    def test_hexagon_saddle(self):
        """Alternating Z (saddle shape)."""
        r = 10
        pts = []
        for i in range(6):
            angle = i * math.pi / 3
            z = 3 * (1 if i % 2 == 0 else -1)
            pts.append((r * math.cos(angle), r * math.sin(angle), z))
        w = make_polygon_wire(*pts)
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)

    def test_large_polygon_sinusoidal(self):
        """20-sided polygon with sinusoidal Z."""
        n = 20
        pts = []
        for i in range(n):
            angle = 2 * math.pi * i / n
            z = 3 * math.sin(2 * angle)
            pts.append((10 * math.cos(angle), 10 * math.sin(angle), z))
        w = make_polygon_wire(*pts)
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)

    def test_steep_fold(self):
        """Wire with a steep fold in Z."""
        w = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, 5, 20), (10, 10, 0), (0, 10, 0))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)


# =========================================================================
# 3. Multiple non-planar wires
# =========================================================================


class TestFishEyeNonPlanarMultiple(unittest.TestCase):
    def test_two_separate_quads(self):
        w1 = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, 10, 2), (0, 10, 1))
        w2 = make_polygon_wire((20, 0, 0), (30, 0, 1), (30, 10, 3), (20, 10, 0))
        faces = fisheye([w1, w2])
        self.assertEqual(len(faces), 2)

    def test_nested_small_z_offset(self):
        """Small Z offset treated as planar by OCCT."""
        outer = make_polygon_wire((-20, -20, 0), (20, -20, 1), (20, 20, 2), (-20, 20, 1))
        inner = make_polygon_wire((-5, -5, 0.5), (5, -5, 0.5), (5, 5, 1.5), (-5, 5, 1.0))
        faces = fisheye([outer, inner])
        self.assertGreaterEqual(len(faces), 1)


# =========================================================================
# 4. Edge cases
# =========================================================================


class TestFishEyeNonPlanarEdgeCases(unittest.TestCase):
    def test_near_planar(self):
        """Z offset below OCCT tolerance -> treated as planar."""
        w = make_polygon_wire((0, 0, 0), (10, 0, 0), (10, 10, 1e-8), (0, 10, 0))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=1)

    def test_narrow_strip(self):
        """Narrow strip — OCCT can fit a plane through this."""
        w = make_polygon_wire((0, 0, 0), (100, 0, 0), (100, 0.1, 5), (0, 0.1, 5))
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)

    def test_bspline_wavy(self):
        """Closed BSpline with non-coplanar control points."""
        w = make_bspline_wire([(0, 0, 0), (10, 0, 5), (10, 10, -3), (0, 10, 4)], closed=True)
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)

    def test_cylinder_lateral_wire(self):
        """Wire from a cylinder lateral face: produces a valid curved face."""
        cyl = Part.makeCylinder(10, 20)
        lateral = [f for f in cyl.Faces if f.Surface.TypeId == "Part::GeomCylinder"][0]
        wire = lateral.OuterWire
        faces = fisheye(wire)
        self.assertEqual(len(faces), 1)
        self.assertGreater(faces[0].Area, 0)
