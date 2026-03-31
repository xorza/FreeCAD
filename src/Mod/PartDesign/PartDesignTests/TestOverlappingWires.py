# SPDX-License-Identifier: LGPL-2.1-or-later

import math
import unittest

import FreeCAD
import Part
import Sketcher


def _add_rectangle_cw(sketch, x0, y0, x1, y1):
    """Add rectangle with CW edge order (top-left-bottom-right), matching GUI."""
    i = int(sketch.GeometryCount)
    sketch.addGeometry([
        Part.LineSegment(FreeCAD.Vector(x1, y1, 0), FreeCAD.Vector(x0, y1, 0)),
        Part.LineSegment(FreeCAD.Vector(x0, y1, 0), FreeCAD.Vector(x0, y0, 0)),
        Part.LineSegment(FreeCAD.Vector(x0, y0, 0), FreeCAD.Vector(x1, y0, 0)),
        Part.LineSegment(FreeCAD.Vector(x1, y0, 0), FreeCAD.Vector(x1, y1, 0)),
    ], False)
    sketch.addConstraint([
        Sketcher.Constraint("Coincident", i, 2, i + 1, 1),
        Sketcher.Constraint("Coincident", i + 1, 2, i + 2, 1),
        Sketcher.Constraint("Coincident", i + 2, 2, i + 3, 1),
        Sketcher.Constraint("Coincident", i + 3, 2, i, 1),
        Sketcher.Constraint("Horizontal", i),
        Sketcher.Constraint("Horizontal", i + 2),
        Sketcher.Constraint("Vertical", i + 1),
        Sketcher.Constraint("Vertical", i + 3),
    ])


def _add_circle(sketch, cx, cy, radius):
    sketch.addGeometry(
        Part.Circle(FreeCAD.Vector(cx, cy, 0), FreeCAD.Vector(0, 0, 1), radius), False
    )


def _make_rect_face(x0, y0, x1, y1):
    return Part.Face(Part.makePolygon([
        FreeCAD.Vector(x0, y0, 0), FreeCAD.Vector(x1, y0, 0),
        FreeCAD.Vector(x1, y1, 0), FreeCAD.Vector(x0, y1, 0),
        FreeCAD.Vector(x0, y0, 0),
    ]))


class TestOverlappingWires(unittest.TestCase):
    """Test that Pad/Pocket correctly handles overlapping sketch wires.

    Uses CW rectangle edge order matching the GUI's DrawSketchHandler to
    expose winding-dependent bugs in FaceMakerBullseye.
    """

    def setUp(self):
        self.Doc = FreeCAD.newDocument("PartDesignTestOverlap")

    def tearDown(self):
        FreeCAD.closeDocument("PartDesignTestOverlap")

    def _makePad(self, sketch, length=10):
        body = self.Doc.addObject("PartDesign::Body", "Body")
        body.AllowCompound = True
        body.addObject(sketch)
        sketch.AttachmentSupport = (self.Doc.getObject("XY_Plane"), [""])
        sketch.MapMode = "FlatFace"
        sketch.MakeInternals = False
        pad = self.Doc.addObject("PartDesign::Pad", "Pad")
        body.addObject(pad)
        pad.Profile = (sketch, [""])
        pad.Length = length
        self.Doc.recompute()
        return pad

    # ------------------------------------------------------------------
    # Regression: simple cases (no overlaps — must use Bullseye fallback)
    # ------------------------------------------------------------------

    def testSimpleRect(self):
        """Single rectangle: 1 valid solid, correct volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, 0, 0, 10, 10)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        self.assertAlmostEqual(pad.Shape.Volume, 1000.0, delta=1.0)

    def testRectWithHole(self):
        """Rect with inner rect hole: correct hollow volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, -5, -5, 5, 5)
        _add_rectangle_cw(sk, -20, -20, 20, 20)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        expected = (40 * 40 - 10 * 10) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    def testCircleWithHole(self):
        """Circle with inner circle: hollow cylinder."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_circle(sk, 0, 0, 5)
        _add_circle(sk, 0, 0, 20)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        expected = math.pi * (20**2 - 5**2) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    def testFourConcentricCircles(self):
        """4 concentric circles: bullseye with correct total volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        r1, r2, r3, r4 = 25.8, 15.4, 6.4, 2.5
        _add_circle(sk, 0, 0, r1)
        _add_circle(sk, 0, 0, r2)
        _add_circle(sk, 0, 0, r3)
        _add_circle(sk, 0, 0, r4)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertGreaterEqual(len(pad.Shape.Solids), 1)
        for s in pad.Shape.Solids:
            self.assertTrue(s.isValid())
        ring1 = math.pi * (r1**2 - r2**2)
        ring3 = math.pi * (r3**2 - r4**2)
        expected = (ring1 + ring3) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    def testOffsetCircleHole(self):
        """Offset hole inside a larger circle: correct hollow volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_circle(sk, 8, 0, 3)
        _add_circle(sk, 0, 0, 20)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        expected = math.pi * (20**2 - 3**2) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    # ------------------------------------------------------------------
    # Touching wires (shared edge, no overlap)
    # ------------------------------------------------------------------

    def testTwoRectsSharedEdge(self):
        """Two rects sharing an edge: correct total volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, 0, 0, 10, 10)
        _add_rectangle_cw(sk, 10, 0, 20, 10)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertGreaterEqual(len(pad.Shape.Solids), 1)
        self.assertAlmostEqual(pad.Shape.Volume, 2000.0, delta=1.0)

    # ------------------------------------------------------------------
    # Two overlapping shapes
    # ------------------------------------------------------------------

    def testTwoOverlappingRects(self):
        """Two overlapping rects: 1 solid, union volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, 0, 0, 20, 20)
        _add_rectangle_cw(sk, 10, 10, 30, 30)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        r1 = _make_rect_face(0, 0, 20, 20)
        r2 = _make_rect_face(10, 10, 30, 30)
        expected = sum(f.Area for f in r1.fuse(r2).Faces) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    def testTwoOverlappingCircles(self):
        """Two overlapping circles: 1 valid solid, union volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_circle(sk, 0, 0, 15)
        _add_circle(sk, 12, 0, 15)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        c1 = Part.Face(Part.Wire(Part.makeCircle(15, FreeCAD.Vector(0, 0, 0))))
        c2 = Part.Face(Part.Wire(Part.makeCircle(15, FreeCAD.Vector(12, 0, 0))))
        expected = sum(f.Area for f in c1.fuse(c2).Faces) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    # ------------------------------------------------------------------
    # Overlapping + hole (the main bug case)
    # ------------------------------------------------------------------

    def testOverlappingRectsWithHole(self):
        """Overlapping rects with inner hole: 1 solid, correct volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, 1.7, 5.3, 20.7, 19.9)
        _add_rectangle_cw(sk, -49.2, -40.0, 40.7, 42.4)
        _add_rectangle_cw(sk, -84.2, -49.3, -21.9, -18.3)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        large = _make_rect_face(-49.2, -40.0, 40.7, 42.4)
        bottom = _make_rect_face(-84.2, -49.3, -21.9, -18.3)
        hole = _make_rect_face(1.7, 5.3, 20.7, 19.9)
        expected = sum(f.Area for f in large.fuse(bottom).cut(hole).Faces) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=10.0)

    def testOverlappingCirclesWithCircleHole(self):
        """Two overlapping circles with a small circle hole inside."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_circle(sk, 0, 0, 3)   # hole
        _add_circle(sk, 0, 0, 20)  # outer 1
        _add_circle(sk, 15, 0, 20) # outer 2, overlaps outer 1
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        # Volume should be less than union of two circles (hole subtracted)
        c1 = Part.Face(Part.Wire(Part.makeCircle(20, FreeCAD.Vector(0, 0, 0))))
        c2 = Part.Face(Part.Wire(Part.makeCircle(20, FreeCAD.Vector(15, 0, 0))))
        union_area = sum(f.Area for f in c1.fuse(c2).Faces)
        hole_area = math.pi * 9
        expected = (union_area - hole_area) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=10.0)

    # ------------------------------------------------------------------
    # Three overlapping shapes
    # ------------------------------------------------------------------

    def testThreeOverlappingRects(self):
        """Three overlapping rects: 1 valid solid."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, 0, 0, 20, 20)
        _add_rectangle_cw(sk, 10, 5, 30, 25)
        _add_rectangle_cw(sk, 5, 10, 25, 30)
        self.Doc.recompute()
        pad = self._makePad(sk)
        self.assertEqual(len(pad.Shape.Solids), 1)
        self.assertTrue(pad.Shape.Solids[0].isValid())
        r1 = _make_rect_face(0, 0, 20, 20)
        r2 = _make_rect_face(10, 5, 30, 25)
        r3 = _make_rect_face(5, 10, 25, 30)
        expected = sum(f.Area for f in r1.fuse(r2).fuse(r3).Faces) * 10
        self.assertAlmostEqual(pad.Shape.Volume, expected, delta=1.0)

    # ------------------------------------------------------------------
    # Mixed: overlapping + separate non-overlapping
    # ------------------------------------------------------------------

    def testOverlappingPlusSeparateRect(self):
        """Two overlapping rects + one separate rect: correct total volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, 0, 0, 20, 20)
        _add_rectangle_cw(sk, 10, 10, 30, 30)
        _add_rectangle_cw(sk, 50, 0, 60, 10)  # separate, no overlap
        self.Doc.recompute()
        pad = self._makePad(sk)
        for s in pad.Shape.Solids:
            self.assertTrue(s.isValid())
        r1 = _make_rect_face(0, 0, 20, 20)
        r2 = _make_rect_face(10, 10, 30, 30)
        overlap_vol = sum(f.Area for f in r1.fuse(r2).Faces) * 10
        separate_vol = 10 * 10 * 10
        self.assertAlmostEqual(pad.Shape.Volume, overlap_vol + separate_vol, delta=1.0)

    def testOverlappingRectsWithHoleAndSeparate(self):
        """Overlapping rects with hole + separate rect: correct total volume."""
        sk = self.Doc.addObject("Sketcher::SketchObject", "Sketch")
        _add_rectangle_cw(sk, -5, -5, 5, 5)     # hole inside large
        _add_rectangle_cw(sk, -20, -20, 20, 20)  # large
        _add_rectangle_cw(sk, 10, 10, 30, 30)    # overlaps large
        _add_rectangle_cw(sk, 50, 0, 60, 10)     # separate
        self.Doc.recompute()
        pad = self._makePad(sk)
        for s in pad.Shape.Solids:
            self.assertTrue(s.isValid())
        large = _make_rect_face(-20, -20, 20, 20)
        overlap = _make_rect_face(10, 10, 30, 30)
        hole = _make_rect_face(-5, -5, 5, 5)
        main_vol = sum(f.Area for f in large.fuse(overlap).cut(hole).Faces) * 10
        separate_vol = 10 * 10 * 10
        self.assertAlmostEqual(pad.Shape.Volume, main_vol + separate_vol, delta=10.0)
