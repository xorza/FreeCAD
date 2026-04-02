# SPDX-License-Identifier: LGPL-2.1-or-later

"""Planar tests for Part::FaceMakerFishEye.

Tests call Part.makeFace(..., "Part::FaceMakerFishEye") directly with
planar wires on the XY plane.
"""

import math
import unittest

import FreeCAD
import Part

Vec = FreeCAD.Vector


def rect_wire(x0, y0, x1, y1):
    return Part.Wire(
        Part.makePolygon(
            [Vec(x0, y0), Vec(x1, y0), Vec(x1, y1), Vec(x0, y1), Vec(x0, y0)]
        )
    )


def rect_face(x0, y0, x1, y1):
    return Part.Face(rect_wire(x0, y0, x1, y1))


def circle_wire(cx, cy, r):
    return Part.Wire(Part.makeCircle(r, Vec(cx, cy, 0)))


def triangle_wire(p1, p2, p3):
    return Part.Wire(
        Part.makePolygon([Vec(*p1, 0), Vec(*p2, 0), Vec(*p3, 0), Vec(*p1, 0)])
    )


def fisheye(wires):
    """Run FaceMakerFishEye on wires, return list of faces."""
    if isinstance(wires, Part.Wire):
        wires = [wires]
    return Part.makeFace(Part.Compound(wires), "Part::FaceMakerFishEye").Faces


def total_area(faces):
    return sum(f.Area for f in faces)


def union_area(*face_shapes):
    """Compute area of the boolean union of face shapes."""
    result = face_shapes[0]
    for s in face_shapes[1:]:
        result = result.fuse(s)
    return sum(f.Area for f in result.Faces)


def faces_overlap(faces):
    for i, f1 in enumerate(faces):
        for f2 in faces[i + 1 :]:
            common = f1.common(f2)
            if common.Faces:
                return True
    return False


# =========================================================================
# 1. Single closed shapes
# =========================================================================


class TestFishEyeSingleShape(unittest.TestCase):
    def test_rectangle(self):
        faces = fisheye(rect_wire(0, 0, 10, 10))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_circle(self):
        r = 10
        faces = fisheye(circle_wire(0, 0, r))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * r * r, places=1)

    def test_triangle(self):
        faces = fisheye(triangle_wire((0, 0), (10, 0), (5, 10)))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 50.0, places=3)

    def test_semicircle(self):
        r = 10
        arc = Part.ArcOfCircle(
            Part.Circle(Vec(0, 0, 0), Vec(0, 0, 1), r), 0, math.pi
        )
        line = Part.LineSegment(Vec(-r, 0, 0), Vec(r, 0, 0))
        w = Part.Wire([arc.toShape(), line.toShape()])
        faces = fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * r * r / 2, places=1)

    def test_small_rectangle(self):
        faces = fisheye(rect_wire(0, 0, 0.1, 0.1))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 0.01, places=5)

    def test_large_rectangle(self):
        faces = fisheye(rect_wire(0, 0, 1000, 1000))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 1e6, places=0)


# =========================================================================
# 2. Non-overlapping shapes
# =========================================================================


class TestFishEyeSeparateShapes(unittest.TestCase):
    def test_two_rectangles(self):
        faces = fisheye([rect_wire(0, 0, 5, 5), rect_wire(20, 20, 25, 25)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 50.0, places=3)

    def test_two_circles(self):
        faces = fisheye([circle_wire(0, 0, 5), circle_wire(30, 0, 5)])
        self.assertEqual(len(faces), 2)

    def test_three_separate_shapes(self):
        faces = fisheye(
            [
                rect_wire(0, 0, 5, 5),
                circle_wire(20, 0, 3),
                triangle_wire((40, 0), (50, 0), (45, 8)),
            ]
        )
        self.assertEqual(len(faces), 3)


# =========================================================================
# 3. Touching shapes (shared edge / vertex)
# =========================================================================


class TestFishEyeTouchingShapes(unittest.TestCase):
    def test_shared_edge(self):
        faces = fisheye([rect_wire(0, 0, 10, 10), rect_wire(10, 0, 20, 10)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 200.0, places=3)

    def test_shared_corner(self):
        faces = fisheye([rect_wire(0, 0, 10, 10), rect_wire(10, 10, 20, 20)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 200.0, places=3)


# =========================================================================
# 4. Nesting / even-odd holes
# =========================================================================


class TestFishEyeNesting(unittest.TestCase):
    """FishEye uses even-odd winding: a region contained by N wires is
    included only if N is odd. So 2 nested wires -> 1 face (ring with hole),
    3 nested -> 2 faces (outer ring + inner disc)."""

    def test_circle_inside_rectangle(self):
        """Even-odd: inner circle is a hole -> 1 face (ring)."""
        faces = fisheye([rect_wire(-20, -20, 20, 20), circle_wire(0, 0, 5)])
        self.assertEqual(len(faces), 1)
        expected = 1600.0 - math.pi * 25
        self.assertAlmostEqual(total_area(faces), expected, places=1)

    def test_concentric_circles(self):
        """Even-odd: inner circle is a hole -> 1 annulus face."""
        faces = fisheye([circle_wire(0, 0, 10), circle_wire(0, 0, 5)])
        self.assertEqual(len(faces), 1)
        expected = math.pi * (100 - 25)
        self.assertAlmostEqual(total_area(faces), expected, places=1)

    def test_rectangle_inside_rectangle(self):
        """Even-odd: inner rect is a hole -> 1 frame face."""
        faces = fisheye([rect_wire(-20, -20, 20, 20), rect_wire(-5, -5, 5, 5)])
        self.assertEqual(len(faces), 1)
        expected = 40 * 40 - 10 * 10
        self.assertAlmostEqual(total_area(faces), expected, places=1)

    def test_triple_nesting(self):
        """Even-odd: outer ring (odd=1) + inner disc (odd=3) -> 2 faces."""
        faces = fisheye(
            [rect_wire(0, 0, 30, 30), rect_wire(5, 5, 25, 25), rect_wire(10, 10, 20, 20)]
        )
        self.assertEqual(len(faces), 2)
        # outer ring = 900 - 400 = 500, inner disc = 100, total = 600
        self.assertAlmostEqual(total_area(faces), 600.0, places=3)

    def test_four_concentric_circles(self):
        """4 concentric circles: even-odd gives 2 annular rings."""
        r1, r2, r3, r4 = 25.8, 15.4, 6.4, 2.5
        faces = fisheye(
            [
                circle_wire(0, 0, r1),
                circle_wire(0, 0, r2),
                circle_wire(0, 0, r3),
                circle_wire(0, 0, r4),
            ]
        )
        ring1 = math.pi * (r1**2 - r2**2)
        ring2 = math.pi * (r3**2 - r4**2)
        self.assertAlmostEqual(total_area(faces), ring1 + ring2, delta=1.0)

    def test_offset_circle_hole(self):
        """Offset hole inside a larger circle."""
        faces = fisheye([circle_wire(0, 0, 20), circle_wire(8, 0, 3)])
        self.assertEqual(len(faces), 1)
        expected = math.pi * (20**2 - 3**2)
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)


# =========================================================================
# 5. Two overlapping shapes (fuse + split)
# =========================================================================


class TestFishEyeTwoOverlapping(unittest.TestCase):
    def test_two_overlapping_circles(self):
        """Venn diagram: 3 regions."""
        r = 10
        d = 12
        faces = fisheye([circle_wire(0, 0, r), circle_wire(d, 0, r)])
        self.assertEqual(len(faces), 3)
        cos_arg = d / (2 * r)
        lens = 2 * r * r * math.acos(cos_arg) - (d / 2) * math.sqrt(4 * r * r - d * d)
        expected = 2 * math.pi * r * r - lens
        self.assertAlmostEqual(total_area(faces), expected, places=1)

    def test_two_overlapping_circles_no_overlap(self):
        faces = fisheye([circle_wire(0, 0, 10), circle_wire(12, 0, 10)])
        self.assertFalse(faces_overlap(faces))

    def test_two_overlapping_rectangles(self):
        """Two overlapping rectangles: 3 regions, area = union."""
        faces = fisheye([rect_wire(0, 0, 20, 20), rect_wire(10, 10, 30, 30)])
        expected = union_area(rect_face(0, 0, 20, 20), rect_face(10, 10, 30, 30))
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_two_overlapping_rectangles_no_overlap(self):
        faces = fisheye([rect_wire(0, 0, 10, 10), rect_wire(5, 5, 15, 15)])
        self.assertFalse(faces_overlap(faces))

    def test_circle_overlapping_rectangle(self):
        faces = fisheye([rect_wire(0, 0, 20, 20), circle_wire(20, 10, 8)])
        self.assertEqual(len(faces), 3)

    def test_two_overlapping_circles_area_via_union(self):
        """Validate area against boolean union."""
        faces = fisheye([circle_wire(0, 0, 15), circle_wire(12, 0, 15)])
        c1 = Part.Face(circle_wire(0, 0, 15))
        c2 = Part.Face(circle_wire(12, 0, 15))
        expected = union_area(c1, c2)
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)


# =========================================================================
# 6. Subdivision by internal lines / cross patterns
# =========================================================================


class TestFishEyeSubdivision(unittest.TestCase):
    def test_rectangle_with_diagonal(self):
        """Rectangle + diagonal -> 2 triangular faces."""
        diag = Part.Wire(Part.makePolygon([Vec(0, 0), Vec(10, 10)]))
        faces = fisheye([rect_wire(0, 0, 10, 10), diag])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)

    def test_cross_pattern(self):
        """Two perpendicular rectangles (+ shape): 5 faces."""
        faces = fisheye([rect_wire(-5, -15, 5, 15), rect_wire(-15, -5, 15, 5)])
        self.assertEqual(len(faces), 5)
        self.assertAlmostEqual(total_area(faces), 500.0, places=2)

    def test_cross_pattern_no_overlap(self):
        faces = fisheye([rect_wire(-5, -15, 5, 15), rect_wire(-15, -5, 15, 5)])
        self.assertFalse(faces_overlap(faces))


# =========================================================================
# 7. Three+ overlapping shapes
# =========================================================================


class TestFishEyeThreeOverlapping(unittest.TestCase):
    def test_three_overlapping_circles(self):
        """Three overlapping circles -> 7 regions."""
        faces = fisheye(
            [circle_wire(0, 0, 10), circle_wire(8, 0, 10), circle_wire(4, 7, 10)]
        )
        self.assertEqual(len(faces), 7)

    def test_three_overlapping_circles_area(self):
        """Total face area must equal the union area."""
        r = 10
        faces = fisheye(
            [circle_wire(0, 0, r), circle_wire(8, 0, r), circle_wire(4, 7, r)]
        )
        c1 = Part.Face(circle_wire(0, 0, r))
        c2 = Part.Face(circle_wire(8, 0, r))
        c3 = Part.Face(circle_wire(4, 7, r))
        expected = union_area(c1, c2, c3)
        self.assertAlmostEqual(total_area(faces), expected, places=0)

    def test_three_overlapping_rectangles(self):
        """Area validated against boolean union."""
        faces = fisheye(
            [rect_wire(0, 0, 20, 20), rect_wire(10, 5, 30, 25), rect_wire(5, 10, 25, 30)]
        )
        expected = union_area(
            rect_face(0, 0, 20, 20), rect_face(10, 5, 30, 25), rect_face(5, 10, 25, 30)
        )
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_four_overlapping_circles(self):
        faces = fisheye(
            [
                circle_wire(0, 0, 10),
                circle_wire(8, 0, 10),
                circle_wire(0, 8, 10),
                circle_wire(8, 8, 10),
            ]
        )
        self.assertGreaterEqual(len(faces), 9)

    def test_mixed_rect_triangle_circle(self):
        faces = fisheye(
            [
                rect_wire(0, 0, 20, 20),
                triangle_wire((10, -5), (30, 15), (10, 15)),
                circle_wire(15, 10, 8),
            ]
        )
        self.assertGreaterEqual(len(faces), 4)


# =========================================================================
# 8. Overlapping + holes (mixed nesting and overlap)
# =========================================================================


class TestFishEyeOverlapWithHoles(unittest.TestCase):
    def test_overlapping_rects_with_hole(self):
        """Two overlapping rects with inner hole: area = union - hole."""
        faces = fisheye(
            [
                rect_wire(-49.2, -40.0, 40.7, 42.4),
                rect_wire(-84.2, -49.3, -21.9, -18.3),
                rect_wire(1.7, 5.3, 20.7, 19.9),
            ]
        )
        large = rect_face(-49.2, -40.0, 40.7, 42.4)
        bottom = rect_face(-84.2, -49.3, -21.9, -18.3)
        hole = rect_face(1.7, 5.3, 20.7, 19.9)
        expected = sum(f.Area for f in large.fuse(bottom).cut(hole).Faces)
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_overlapping_circles_with_hole(self):
        """Two overlapping circles with a small circle hole inside."""
        faces = fisheye(
            [circle_wire(0, 0, 20), circle_wire(15, 0, 20), circle_wire(0, 0, 3)]
        )
        c1 = Part.Face(circle_wire(0, 0, 20))
        c2 = Part.Face(circle_wire(15, 0, 20))
        union_a = union_area(c1, c2)
        hole_a = math.pi * 3**2
        expected = union_a - hole_a
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_overlapping_plus_separate(self):
        """Two overlapping rects + one separate rect."""
        faces = fisheye(
            [
                rect_wire(0, 0, 20, 20),
                rect_wire(10, 10, 30, 30),
                rect_wire(50, 0, 60, 10),
            ]
        )
        overlap_a = union_area(rect_face(0, 0, 20, 20), rect_face(10, 10, 30, 30))
        expected = overlap_a + 100.0
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_overlapping_with_hole_and_separate(self):
        """Overlapping rects with hole + separate rect."""
        faces = fisheye(
            [
                rect_wire(-20, -20, 20, 20),
                rect_wire(10, 10, 30, 30),
                rect_wire(-5, -5, 5, 5),
                rect_wire(50, 0, 60, 10),
            ]
        )
        large = rect_face(-20, -20, 20, 20)
        overlap = rect_face(10, 10, 30, 30)
        hole = rect_face(-5, -5, 5, 5)
        main_a = sum(f.Area for f in large.fuse(overlap).cut(hole).Faces)
        expected = main_a + 100.0
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)


# =========================================================================
# 9. Open / degenerate geometry
# =========================================================================


class TestFishEyeDegenerate(unittest.TestCase):
    def test_single_open_line(self):
        """A single open line should produce no faces."""
        w = Part.Wire(Part.makePolygon([Vec(0, 0), Vec(10, 0)]))
        try:
            faces = fisheye(w)
            self.assertEqual(len(faces), 0)
        except RuntimeError:
            pass  # also acceptable: FaceMaker throws

    def test_empty_compound(self):
        """Empty compound should produce no faces or throw."""
        try:
            shape = Part.makeFace(Part.Compound([]), "Part::FaceMakerFishEye")
            self.assertEqual(len(shape.Faces), 0)
        except RuntimeError:
            pass

    def test_figure_8_polygon(self):
        """Self-intersecting figure-8 as single wire: doesn't crash.
        BRepBuilderAPI_MakeFace produces a degenerate face (lobes cancel);
        proper handling would require self-intersection detection."""
        w = Part.Wire(
            Part.makePolygon(
                [
                    Vec(5, 0),
                    Vec(0, 5),
                    Vec(-5, 0),
                    Vec(0, -5),
                    Vec(5, 0),
                    Vec(10, 5),
                    Vec(15, 0),
                    Vec(10, -5),
                    Vec(5, 0),
                ]
            )
        )
        faces = fisheye(w)
        self.assertGreaterEqual(len(faces), 1)

    def test_figure_8_two_wires(self):
        """Figure-8 as two separate touching loops: 2 faces."""
        w1 = Part.Wire(
            Part.makePolygon(
                [Vec(-5, 0), Vec(0, 5), Vec(5, 0), Vec(0, -5), Vec(-5, 0)]
            )
        )
        w2 = Part.Wire(
            Part.makePolygon(
                [Vec(5, 0), Vec(10, 5), Vec(15, 0), Vec(10, -5), Vec(5, 0)]
            )
        )
        faces = fisheye([w1, w2])
        self.assertEqual(len(faces), 2)
        self.assertGreater(total_area(faces), 0)

    def test_bowtie(self):
        """Bowtie / hourglass: self-intersecting wire, produces valid face(s)."""
        w = Part.Wire(
            Part.makePolygon(
                [Vec(0, 0), Vec(10, 5), Vec(0, 10), Vec(10, 10), Vec(0, 5), Vec(10, 0), Vec(0, 0)]
            )
        )
        faces = fisheye(w)
        self.assertGreaterEqual(len(faces), 1)
        self.assertGreater(total_area(faces), 0)
