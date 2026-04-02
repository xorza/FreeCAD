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


def line_wire(*points):
    return Part.Wire(Part.makePolygon([Vec(*p) if len(p) >= 2 else p for p in points]))


def fisheye(wires):
    """Run FaceMakerFishEye on wires, return list of faces."""
    if isinstance(wires, Part.Wire):
        wires = [wires]
    return Part.makeFace(Part.Compound(wires), "Part::FaceMakerFishEye").Faces


def total_area(faces):
    return sum(f.Area for f in faces)


def union_area(*face_shapes):
    result = face_shapes[0]
    for s in face_shapes[1:]:
        result = result.fuse(s)
    return sum(f.Area for f in result.Faces)


def faces_overlap(faces):
    for i, f1 in enumerate(faces):
        for f2 in faces[i + 1 :]:
            if f1.common(f2).Faces:
                return True
    return False


# =========================================================================
# 1. Single closed shapes
# =========================================================================


class TestSingleShape(unittest.TestCase):
    def test_rectangle(self):
        faces = fisheye(rect_wire(0, 0, 10, 10))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_circle(self):
        faces = fisheye(circle_wire(0, 0, 10))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * 100, places=1)

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


# =========================================================================
# 2. Non-overlapping shapes
# =========================================================================


class TestSeparateShapes(unittest.TestCase):
    def test_two_rectangles(self):
        faces = fisheye([rect_wire(0, 0, 5, 5), rect_wire(20, 20, 25, 25)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 50.0, places=3)

    def test_three_mixed_shapes(self):
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


class TestTouchingShapes(unittest.TestCase):
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


class TestNesting(unittest.TestCase):
    def test_circle_inside_rectangle(self):
        """Even-odd: inner circle is a hole -> 1 face (ring)."""
        faces = fisheye([rect_wire(-20, -20, 20, 20), circle_wire(0, 0, 5)])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(total_area(faces), 1600.0 - math.pi * 25, places=1)

    def test_concentric_circles(self):
        """2 nested -> annulus (hole)."""
        faces = fisheye([circle_wire(0, 0, 10), circle_wire(0, 0, 5)])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(total_area(faces), math.pi * 75, places=1)

    def test_triple_nesting(self):
        """3 nested -> outer ring + inner disc (even-odd)."""
        faces = fisheye(
            [rect_wire(0, 0, 30, 30), rect_wire(5, 5, 25, 25), rect_wire(10, 10, 20, 20)]
        )
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 600.0, places=3)

    def test_four_concentric_circles(self):
        """4 nested -> 2 annular rings."""
        r1, r2, r3, r4 = 25.8, 15.4, 6.4, 2.5
        faces = fisheye(
            [
                circle_wire(0, 0, r1),
                circle_wire(0, 0, r2),
                circle_wire(0, 0, r3),
                circle_wire(0, 0, r4),
            ]
        )
        expected = math.pi * (r1**2 - r2**2 + r3**2 - r4**2)
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_offset_hole(self):
        """Non-concentric hole inside a circle."""
        faces = fisheye([circle_wire(0, 0, 20), circle_wire(8, 0, 3)])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(total_area(faces), math.pi * (400 - 9), delta=1.0)


# =========================================================================
# 5. Two overlapping shapes
# =========================================================================


class TestTwoOverlapping(unittest.TestCase):
    def test_two_circles(self):
        """Venn diagram: 3 non-overlapping regions."""
        r, d = 10, 12
        faces = fisheye([circle_wire(0, 0, r), circle_wire(d, 0, r)])
        self.assertEqual(len(faces), 3)
        cos_arg = d / (2 * r)
        lens = 2 * r * r * math.acos(cos_arg) - (d / 2) * math.sqrt(4 * r * r - d * d)
        self.assertAlmostEqual(total_area(faces), 2 * math.pi * r * r - lens, places=1)
        self.assertFalse(faces_overlap(faces))

    def test_two_rectangles(self):
        """3 non-overlapping regions, area = union."""
        faces = fisheye([rect_wire(0, 0, 20, 20), rect_wire(10, 10, 30, 30)])
        expected = union_area(rect_face(0, 0, 20, 20), rect_face(10, 10, 30, 30))
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)
        self.assertFalse(faces_overlap(faces))

    def test_circle_overlapping_rectangle(self):
        faces = fisheye([rect_wire(0, 0, 20, 20), circle_wire(20, 10, 8)])
        self.assertEqual(len(faces), 3)


# =========================================================================
# 6. Subdivision and dangling edge pruning
# =========================================================================


class TestSubdivision(unittest.TestCase):
    def test_diagonal(self):
        """Rectangle + diagonal -> 2 triangles."""
        faces = fisheye([rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10))])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)

    def test_cross_pattern(self):
        """Two perpendicular rectangles (+ shape): 5 non-overlapping faces."""
        faces = fisheye([rect_wire(-5, -15, 5, 15), rect_wire(-15, -5, 15, 5)])
        self.assertEqual(len(faces), 5)
        self.assertAlmostEqual(total_area(faces), 500.0, places=2)
        self.assertFalse(faces_overlap(faces))

    def test_both_diagonals(self):
        """Rectangle with both diagonals: 4 triangles."""
        faces = fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10)), line_wire((10, 0), (0, 10))]
        )
        self.assertEqual(len(faces), 4)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)
        self.assertFalse(faces_overlap(faces))

    def test_midpoint_cross(self):
        """Horizontal + vertical midpoint lines: 4 equal quadrants."""
        faces = fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 5), (10, 5)), line_wire((5, 0), (5, 10))]
        )
        self.assertEqual(len(faces), 4)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)
        self.assertFalse(faces_overlap(faces))
        for f in faces:
            self.assertAlmostEqual(f.Area, 25.0, places=2)

    def test_incomplete_intersection(self):
        """Line from edge stopping inside rectangle -> dangling, pruned."""
        faces = fisheye([rect_wire(0, 0, 10, 10), line_wire((5, 0), (5, 5))])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_t_junction(self):
        """Line from boundary to diagonal midpoint -> valid 3-face subdivision."""
        faces = fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10)), line_wire((5, 0), (5, 5))]
        )
        self.assertEqual(len(faces), 3)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)

    def test_dangling_chain(self):
        """Connected dangling segments: iterative degree-1 pruning."""
        faces = fisheye(
            [
                rect_wire(0, 0, 10, 10),
                line_wire((5, 0), (5, 4)),
                line_wire((5, 4), (7, 6)),
                line_wire((7, 6), (4, 8)),
            ]
        )
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_floating_line(self):
        """Line fully inside rectangle, touching nothing -> pruned."""
        faces = fisheye([rect_wire(0, 0, 10, 10), line_wire((3, 3), (7, 7))])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_dangling_from_circle(self):
        """Dangling line touching a curved boundary -> pruned."""
        faces = fisheye([circle_wire(0, 0, 10), line_wire((10, 0), (7, 0))])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * 100, places=1)

    def test_diagonal_plus_dangling(self):
        """Valid diagonal split + dangling line: 2 faces, dangling pruned."""
        faces = fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10)), line_wire((10, 5), (7, 5))]
        )
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)


# =========================================================================
# 7. Three+ overlapping shapes
# =========================================================================


class TestMultipleOverlapping(unittest.TestCase):
    def test_three_circles(self):
        """Three overlapping circles -> 7 regions, area = union."""
        r = 10
        faces = fisheye(
            [circle_wire(0, 0, r), circle_wire(8, 0, r), circle_wire(4, 7, r)]
        )
        self.assertEqual(len(faces), 7)
        c1 = Part.Face(circle_wire(0, 0, r))
        c2 = Part.Face(circle_wire(8, 0, r))
        c3 = Part.Face(circle_wire(4, 7, r))
        self.assertAlmostEqual(total_area(faces), union_area(c1, c2, c3), places=0)

    def test_four_circles(self):
        """Four overlapping circles in a square: >= 9 regions."""
        faces = fisheye(
            [
                circle_wire(0, 0, 10),
                circle_wire(8, 0, 10),
                circle_wire(0, 8, 10),
                circle_wire(8, 8, 10),
            ]
        )
        self.assertGreaterEqual(len(faces), 9)

    def test_mixed_geometry(self):
        """Rectangle + triangle + circle overlap."""
        faces = fisheye(
            [
                rect_wire(0, 0, 20, 20),
                triangle_wire((10, -5), (30, 15), (10, 15)),
                circle_wire(15, 10, 8),
            ]
        )
        self.assertGreaterEqual(len(faces), 4)


# =========================================================================
# 8. Overlapping + holes
# =========================================================================


class TestOverlapWithHoles(unittest.TestCase):
    def test_overlapping_rects_with_hole(self):
        """Two overlapping rects with inner hole: area = union - hole."""
        faces = fisheye(
            [rect_wire(0, 0, 90, 80), rect_wire(-30, -10, 30, 20), rect_wire(10, 20, 30, 40)]
        )
        large = rect_face(0, 0, 90, 80)
        small = rect_face(-30, -10, 30, 20)
        hole = rect_face(10, 20, 30, 40)
        expected = sum(f.Area for f in large.fuse(small).cut(hole).Faces)
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
        self.assertAlmostEqual(total_area(faces), main_a + 100.0, delta=1.0)


# =========================================================================
# 9. Self-intersecting / degenerate geometry
# =========================================================================


class TestDegenerate(unittest.TestCase):
    def test_single_open_line(self):
        """Open line -> no faces or RuntimeError."""
        try:
            faces = fisheye(line_wire((0, 0), (10, 0)))
            self.assertEqual(len(faces), 0)
        except RuntimeError:
            pass

    def test_empty_compound(self):
        """Empty compound -> no faces or RuntimeError."""
        try:
            shape = Part.makeFace(Part.Compound([]), "Part::FaceMakerFishEye")
            self.assertEqual(len(shape.Faces), 0)
        except RuntimeError:
            pass

    def test_crossing_edges(self):
        """Closed polygon with geometric crossing -> split into 2+ faces."""
        w = Part.Wire(
            Part.makePolygon(
                [Vec(-10, -5), Vec(10, 5), Vec(10, -5), Vec(-10, 5), Vec(-10, -5)]
            )
        )
        faces = fisheye(w)
        self.assertGreaterEqual(len(faces), 2)
        self.assertGreater(total_area(faces), 0)

    def test_bowtie(self):
        """Two triangles sharing a vertex: 2 faces."""
        w1 = Part.Wire(
            Part.makePolygon([Vec(-10, -5), Vec(-10, 5), Vec(0, 0), Vec(-10, -5)])
        )
        w2 = Part.Wire(
            Part.makePolygon([Vec(0, 0), Vec(10, 5), Vec(10, -5), Vec(0, 0)])
        )
        faces = fisheye([w1, w2])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=1)

    def test_figure_8_polygon(self):
        """Figure-8 with topological crossing (shared vertex, not geometric)."""
        w = Part.Wire(
            Part.makePolygon(
                [
                    Vec(5, 0), Vec(0, 5), Vec(-5, 0), Vec(0, -5), Vec(5, 0),
                    Vec(10, 5), Vec(15, 0), Vec(10, -5), Vec(5, 0),
                ]
            )
        )
        faces = fisheye(w)
        self.assertGreaterEqual(len(faces), 1)
