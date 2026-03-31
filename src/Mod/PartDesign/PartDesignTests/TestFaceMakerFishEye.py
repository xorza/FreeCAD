# SPDX-License-Identifier: LGPL-2.1-or-later

import math
import unittest

import FreeCAD
import Part


def _make_rect_wire(x0, y0, x1, y1):
    return Part.makePolygon(
        [
            FreeCAD.Vector(x0, y0, 0),
            FreeCAD.Vector(x1, y0, 0),
            FreeCAD.Vector(x1, y1, 0),
            FreeCAD.Vector(x0, y1, 0),
            FreeCAD.Vector(x0, y0, 0),
        ]
    )


def _make_circle_wire(cx, cy, r):
    return Part.Wire(Part.makeCircle(r, FreeCAD.Vector(cx, cy, 0)))


def _make_rect_face(x0, y0, x1, y1):
    return Part.Face(_make_rect_wire(x0, y0, x1, y1))


def _make_faces(wires):
    """Make faces from wires using FaceMakerFishEye."""
    comp = Part.Compound(wires)
    return Part.makeFace(comp, "Part::FaceMakerFishEye")


def _total_area(shape):
    return sum(f.Area for f in shape.Faces)


class TestFaceMakerFishEye(unittest.TestCase):
    """Test FaceMakerFishEye: unified face maker for overlapping wires,
    nested holes, and curved surfaces.
    """

    # ------------------------------------------------------------------
    # Simple cases (no overlaps — Bullseye fallback)
    # ------------------------------------------------------------------

    def testSimpleRect(self):
        """Single rectangle: 1 face."""
        result = _make_faces([_make_rect_wire(0, 0, 10, 10)])
        self.assertEqual(len(result.Faces), 1)
        self.assertAlmostEqual(result.Faces[0].Area, 100.0, delta=0.1)

    def testRectWithHole(self):
        """Rect with inner rect hole: ring face."""
        result = _make_faces([
            _make_rect_wire(-20, -20, 20, 20),
            _make_rect_wire(-5, -5, 5, 5),
        ])
        self.assertEqual(len(result.Faces), 1)
        expected = 40 * 40 - 10 * 10
        self.assertAlmostEqual(result.Faces[0].Area, expected, delta=0.1)

    def testCircleWithHole(self):
        """Circle with inner circle: ring face."""
        result = _make_faces([
            _make_circle_wire(0, 0, 20),
            _make_circle_wire(0, 0, 5),
        ])
        self.assertEqual(len(result.Faces), 1)
        expected = math.pi * (20**2 - 5**2)
        self.assertAlmostEqual(result.Faces[0].Area, expected, delta=1.0)

    def testFourConcentricCircles(self):
        """4 concentric circles: bullseye with 2 ring faces."""
        r1, r2, r3, r4 = 25.8, 15.4, 6.4, 2.5
        result = _make_faces([
            _make_circle_wire(0, 0, r1),
            _make_circle_wire(0, 0, r2),
            _make_circle_wire(0, 0, r3),
            _make_circle_wire(0, 0, r4),
        ])
        ring1 = math.pi * (r1**2 - r2**2)
        ring3 = math.pi * (r3**2 - r4**2)
        expected = ring1 + ring3
        self.assertAlmostEqual(_total_area(result), expected, delta=1.0)

    def testOffsetCircleHole(self):
        """Offset hole inside a larger circle."""
        result = _make_faces([
            _make_circle_wire(0, 0, 20),
            _make_circle_wire(8, 0, 3),
        ])
        self.assertEqual(len(result.Faces), 1)
        expected = math.pi * (20**2 - 3**2)
        self.assertAlmostEqual(result.Faces[0].Area, expected, delta=1.0)

    # ------------------------------------------------------------------
    # Touching wires (shared edge, no overlap)
    # ------------------------------------------------------------------

    def testTwoRectsSharedEdge(self):
        """Two rects sharing an edge: 2 faces, correct total area."""
        result = _make_faces([
            _make_rect_wire(0, 0, 10, 10),
            _make_rect_wire(10, 0, 20, 10),
        ])
        self.assertAlmostEqual(_total_area(result), 200.0, delta=0.1)

    # ------------------------------------------------------------------
    # Two overlapping shapes
    # ------------------------------------------------------------------

    def testTwoOverlappingRects(self):
        """Two overlapping rects: union area."""
        result = _make_faces([
            _make_rect_wire(0, 0, 20, 20),
            _make_rect_wire(10, 10, 30, 30),
        ])
        r1 = _make_rect_face(0, 0, 20, 20)
        r2 = _make_rect_face(10, 10, 30, 30)
        expected = sum(f.Area for f in r1.fuse(r2).Faces)
        self.assertAlmostEqual(_total_area(result), expected, delta=1.0)

    def testTwoOverlappingCircles(self):
        """Two overlapping circles: union area."""
        result = _make_faces([
            _make_circle_wire(0, 0, 15),
            _make_circle_wire(12, 0, 15),
        ])
        c1 = Part.Face(_make_circle_wire(0, 0, 15))
        c2 = Part.Face(_make_circle_wire(12, 0, 15))
        expected = sum(f.Area for f in c1.fuse(c2).Faces)
        self.assertAlmostEqual(_total_area(result), expected, delta=1.0)

    # ------------------------------------------------------------------
    # Overlapping + hole
    # ------------------------------------------------------------------

    def testOverlappingRectsWithHole(self):
        """Overlapping rects with inner hole: correct area."""
        result = _make_faces([
            _make_rect_wire(-49.2, -40.0, 40.7, 42.4),
            _make_rect_wire(-84.2, -49.3, -21.9, -18.3),
            _make_rect_wire(1.7, 5.3, 20.7, 19.9),
        ])
        large = _make_rect_face(-49.2, -40.0, 40.7, 42.4)
        bottom = _make_rect_face(-84.2, -49.3, -21.9, -18.3)
        hole = _make_rect_face(1.7, 5.3, 20.7, 19.9)
        expected = sum(f.Area for f in large.fuse(bottom).cut(hole).Faces)
        self.assertAlmostEqual(_total_area(result), expected, delta=1.0)

    def testOverlappingCirclesWithCircleHole(self):
        """Two overlapping circles with a small circle hole inside."""
        result = _make_faces([
            _make_circle_wire(0, 0, 20),
            _make_circle_wire(15, 0, 20),
            _make_circle_wire(0, 0, 3),
        ])
        c1 = Part.Face(_make_circle_wire(0, 0, 20))
        c2 = Part.Face(_make_circle_wire(15, 0, 20))
        union_area = sum(f.Area for f in c1.fuse(c2).Faces)
        hole_area = math.pi * 3**2
        expected = union_area - hole_area
        self.assertAlmostEqual(_total_area(result), expected, delta=1.0)

    # ------------------------------------------------------------------
    # Three overlapping shapes
    # ------------------------------------------------------------------

    def testThreeOverlappingRects(self):
        """Three overlapping rects: union area."""
        result = _make_faces([
            _make_rect_wire(0, 0, 20, 20),
            _make_rect_wire(10, 5, 30, 25),
            _make_rect_wire(5, 10, 25, 30),
        ])
        r1 = _make_rect_face(0, 0, 20, 20)
        r2 = _make_rect_face(10, 5, 30, 25)
        r3 = _make_rect_face(5, 10, 25, 30)
        expected = sum(f.Area for f in r1.fuse(r2).fuse(r3).Faces)
        self.assertAlmostEqual(_total_area(result), expected, delta=1.0)

    # ------------------------------------------------------------------
    # Mixed: overlapping + separate non-overlapping
    # ------------------------------------------------------------------

    def testOverlappingPlusSeparateRect(self):
        """Two overlapping rects + one separate rect: correct total area."""
        result = _make_faces([
            _make_rect_wire(0, 0, 20, 20),
            _make_rect_wire(10, 10, 30, 30),
            _make_rect_wire(50, 0, 60, 10),
        ])
        r1 = _make_rect_face(0, 0, 20, 20)
        r2 = _make_rect_face(10, 10, 30, 30)
        overlap_area = sum(f.Area for f in r1.fuse(r2).Faces)
        separate_area = 10 * 10
        self.assertAlmostEqual(_total_area(result), overlap_area + separate_area, delta=1.0)

    def testOverlappingRectsWithHoleAndSeparate(self):
        """Overlapping rects with hole + separate rect: correct total area."""
        result = _make_faces([
            _make_rect_wire(-20, -20, 20, 20),
            _make_rect_wire(10, 10, 30, 30),
            _make_rect_wire(-5, -5, 5, 5),
            _make_rect_wire(50, 0, 60, 10),
        ])
        large = _make_rect_face(-20, -20, 20, 20)
        overlap = _make_rect_face(10, 10, 30, 30)
        hole = _make_rect_face(-5, -5, 5, 5)
        main_area = sum(f.Area for f in large.fuse(overlap).cut(hole).Faces)
        separate_area = 10 * 10
        self.assertAlmostEqual(_total_area(result), main_area + separate_area, delta=1.0)

    # ------------------------------------------------------------------
    # Non-planar (curved surface) — Cheese fallback
    # ------------------------------------------------------------------

    def testNonPlanarWire(self):
        """Wire cut from a cylinder: produces a valid curved face."""
        # Make a cylinder and extract a closed wire from one of its faces
        cyl = Part.makeCylinder(10, 20)
        # The lateral face has a closed wire
        lateral = [f for f in cyl.Faces if f.Surface.TypeId == "Part::GeomCylinder"][0]
        wire = lateral.OuterWire
        result = _make_faces([wire])
        self.assertEqual(len(result.Faces), 1)
        self.assertTrue(result.Faces[0].Area > 0)
