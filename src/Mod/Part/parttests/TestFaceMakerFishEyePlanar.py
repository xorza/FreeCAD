# SPDX-License-Identifier: LGPL-2.1-or-later

"""Planar tests for Part::FaceMakerFishEye.

All test geometry is defined on the XY plane, then transformed to the
target plane via a class-level placement. Each test mixin is instantiated
for every plane in _PLANES, so every test runs on XY, XZ, and a 45-deg
tilted plane automatically.
"""

import math
import unittest

import FreeCAD
import Part

Vec = FreeCAD.Vector


# =========================================================================
# Helpers — produce geometry on the XY plane
# =========================================================================


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
# Base mixin — transforms XY geometry to the target plane before testing
# =========================================================================


class _PlaneTestBase:
    """Mixin providing fisheye() that transforms wires to a target plane.

    Subclasses set ``placement`` as a class attribute. Areas and face counts
    are invariant under rigid transforms, so assertions stay the same.
    """

    placement = FreeCAD.Placement()

    def fisheye(self, wires):
        if isinstance(wires, Part.Wire):
            wires = [wires]
        mat = self.placement.toMatrix()
        transformed = [Part.Wire(w.transformed(mat).Edges) for w in wires]
        return Part.makeFace(
            Part.Compound(transformed), "Part::FaceMakerFishEye"
        ).Faces


# =========================================================================
# Test mixins (no unittest.TestCase — concrete classes generated below)
# =========================================================================


class _SingleShape(_PlaneTestBase):
    def test_rectangle(self):
        faces = self.fisheye(rect_wire(0, 0, 10, 10))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_circle(self):
        faces = self.fisheye(circle_wire(0, 0, 10))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * 100, places=1)

    def test_triangle(self):
        faces = self.fisheye(triangle_wire((0, 0), (10, 0), (5, 10)))
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 50.0, places=3)

    def test_semicircle(self):
        r = 10
        arc = Part.ArcOfCircle(
            Part.Circle(Vec(0, 0, 0), Vec(0, 0, 1), r), 0, math.pi
        )
        line = Part.LineSegment(Vec(-r, 0, 0), Vec(r, 0, 0))
        w = Part.Wire([arc.toShape(), line.toShape()])
        faces = self.fisheye(w)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * r * r / 2, places=1)


class _SeparateShapes(_PlaneTestBase):
    def test_two_rectangles(self):
        faces = self.fisheye([rect_wire(0, 0, 5, 5), rect_wire(20, 20, 25, 25)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 50.0, places=3)

    def test_three_mixed_shapes(self):
        faces = self.fisheye(
            [
                rect_wire(0, 0, 5, 5),
                circle_wire(20, 0, 3),
                triangle_wire((40, 0), (50, 0), (45, 8)),
            ]
        )
        self.assertEqual(len(faces), 3)

    def test_shared_edge(self):
        faces = self.fisheye([rect_wire(0, 0, 10, 10), rect_wire(10, 0, 20, 10)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 200.0, places=3)

    def test_shared_corner(self):
        faces = self.fisheye([rect_wire(0, 0, 10, 10), rect_wire(10, 10, 20, 20)])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 200.0, places=3)


class _Nesting(_PlaneTestBase):
    def test_circle_inside_rectangle(self):
        faces = self.fisheye([rect_wire(-20, -20, 20, 20), circle_wire(0, 0, 5)])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(total_area(faces), 1600.0 - math.pi * 25, places=1)

    def test_concentric_circles(self):
        faces = self.fisheye([circle_wire(0, 0, 10), circle_wire(0, 0, 5)])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(total_area(faces), math.pi * 75, places=1)

    def test_triple_nesting(self):
        faces = self.fisheye(
            [rect_wire(0, 0, 30, 30), rect_wire(5, 5, 25, 25), rect_wire(10, 10, 20, 20)]
        )
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 600.0, places=3)

    def test_four_concentric_circles(self):
        r1, r2, r3, r4 = 25.8, 15.4, 6.4, 2.5
        faces = self.fisheye(
            [
                circle_wire(0, 0, r1),
                circle_wire(0, 0, r2),
                circle_wire(0, 0, r3),
                circle_wire(0, 0, r4),
            ]
        )
        expected = math.pi * (r1**2 - r2**2 + r3**2 - r4**2)
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)


class _TwoOverlapping(_PlaneTestBase):
    def test_two_circles(self):
        r, d = 10, 12
        faces = self.fisheye([circle_wire(0, 0, r), circle_wire(d, 0, r)])
        self.assertEqual(len(faces), 3)
        cos_arg = d / (2 * r)
        lens = 2 * r * r * math.acos(cos_arg) - (d / 2) * math.sqrt(4 * r * r - d * d)
        self.assertAlmostEqual(total_area(faces), 2 * math.pi * r * r - lens, places=1)
        self.assertFalse(faces_overlap(faces))

    def test_two_rectangles(self):
        faces = self.fisheye([rect_wire(0, 0, 20, 20), rect_wire(10, 10, 30, 30)])
        expected = union_area(rect_face(0, 0, 20, 20), rect_face(10, 10, 30, 30))
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)
        self.assertFalse(faces_overlap(faces))


class _Subdivision(_PlaneTestBase):
    def test_diagonal(self):
        faces = self.fisheye([rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10))])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)

    def test_cross_pattern(self):
        faces = self.fisheye([rect_wire(-5, -15, 5, 15), rect_wire(-15, -5, 15, 5)])
        self.assertEqual(len(faces), 5)
        self.assertAlmostEqual(total_area(faces), 500.0, places=2)
        self.assertFalse(faces_overlap(faces))

    def test_midpoint_cross(self):
        faces = self.fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 5), (10, 5)), line_wire((5, 0), (5, 10))]
        )
        self.assertEqual(len(faces), 4)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)
        self.assertFalse(faces_overlap(faces))
        for f in faces:
            self.assertAlmostEqual(f.Area, 25.0, places=2)

    def test_incomplete_intersection(self):
        faces = self.fisheye([rect_wire(0, 0, 10, 10), line_wire((5, 0), (5, 5))])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_t_junction(self):
        faces = self.fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10)), line_wire((5, 0), (5, 5))]
        )
        self.assertEqual(len(faces), 3)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)

    def test_dangling_chain(self):
        faces = self.fisheye(
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
        faces = self.fisheye([rect_wire(0, 0, 10, 10), line_wire((3, 3), (7, 7))])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 100.0, places=3)

    def test_diagonal_plus_dangling(self):
        faces = self.fisheye(
            [rect_wire(0, 0, 10, 10), line_wire((0, 0), (10, 10)), line_wire((10, 5), (7, 5))]
        )
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=3)

    def test_circle_with_dangling_radius(self):
        """Circle + line from center to outside: dangling line should be
        pruned, producing 1 circular face with a single wire."""
        r = 10
        faces = self.fisheye([circle_wire(0, 0, r), line_wire((0, 0), (r + 5, 0))])
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, math.pi * r * r, places=1)
        # Face should have 1 clean wire — no internal dangling edges
        self.assertEqual(len(faces[0].Wires), 1)

    def test_circle_with_line_through(self):
        """Circle + line from outside through interior to outside: the line
        crosses the boundary twice, creating a chord that splits the circle
        into 2 faces. No dangling segments should remain."""
        r = 10
        faces = self.fisheye([circle_wire(0, 0, r), line_wire((-r - 5, 0), (r + 5, 0))])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), math.pi * r * r, places=1)



class _MultipleOverlapping(_PlaneTestBase):
    def test_three_circles(self):
        r = 10
        faces = self.fisheye(
            [circle_wire(0, 0, r), circle_wire(8, 0, r), circle_wire(4, 7, r)]
        )
        self.assertEqual(len(faces), 7)
        c1 = Part.Face(circle_wire(0, 0, r))
        c2 = Part.Face(circle_wire(8, 0, r))
        c3 = Part.Face(circle_wire(4, 7, r))
        self.assertAlmostEqual(total_area(faces), union_area(c1, c2, c3), places=0)

    def test_four_circles(self):
        faces = self.fisheye(
            [
                circle_wire(0, 0, 10),
                circle_wire(8, 0, 10),
                circle_wire(0, 8, 10),
                circle_wire(8, 8, 10),
            ]
        )
        self.assertGreaterEqual(len(faces), 9)


class _OverlapWithHoles(_PlaneTestBase):
    def test_overlapping_rects_with_hole(self):
        faces = self.fisheye(
            [rect_wire(0, 0, 90, 80), rect_wire(-30, -10, 30, 20), rect_wire(10, 20, 30, 40)]
        )
        large = rect_face(0, 0, 90, 80)
        small = rect_face(-30, -10, 30, 20)
        hole = rect_face(10, 20, 30, 40)
        expected = sum(f.Area for f in large.fuse(small).cut(hole).Faces)
        self.assertAlmostEqual(total_area(faces), expected, delta=1.0)

    def test_overlapping_with_hole_and_separate(self):
        faces = self.fisheye(
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


class _Degenerate(_PlaneTestBase):
    def test_crossing_edges(self):
        w = Part.Wire(
            Part.makePolygon(
                [Vec(-10, -5), Vec(10, 5), Vec(10, -5), Vec(-10, 5), Vec(-10, -5)]
            )
        )
        faces = self.fisheye(w)
        self.assertGreaterEqual(len(faces), 2)
        self.assertGreater(total_area(faces), 0)

    def test_bowtie(self):
        w1 = Part.Wire(
            Part.makePolygon([Vec(-10, -5), Vec(-10, 5), Vec(0, 0), Vec(-10, -5)])
        )
        w2 = Part.Wire(
            Part.makePolygon([Vec(0, 0), Vec(10, 5), Vec(10, -5), Vec(0, 0)])
        )
        faces = self.fisheye([w1, w2])
        self.assertEqual(len(faces), 2)
        self.assertAlmostEqual(total_area(faces), 100.0, places=1)

    def test_figure_8_polygon(self):
        w = Part.Wire(
            Part.makePolygon(
                [
                    Vec(5, 0), Vec(0, 5), Vec(-5, 0), Vec(0, -5), Vec(5, 0),
                    Vec(10, 5), Vec(15, 0), Vec(10, -5), Vec(5, 0),
                ]
            )
        )
        faces = self.fisheye(w)
        self.assertGreaterEqual(len(faces), 1)

    def test_figure_8_bspline(self):
        """Single self-intersecting BSpline forming a figure-8 -> 2 faces."""
        poles = [
            Vec(0, 26.06), Vec(14.6, 15.54), Vec(0.51, 0),
            Vec(-16.13, -17.23), Vec(0, -24.02), Vec(17.15, -9.42),
            Vec(-16.64, 15.20), Vec(0, 26.06),
        ]
        bs = Part.BSplineCurve(poles, None, None, False, 3, [1] * len(poles), False)
        w = Part.Wire([bs.toShape()])
        faces = self.fisheye(w)
        self.assertGreaterEqual(len(faces), 2)
        self.assertGreater(total_area(faces), 0)


# =========================================================================
# Standalone tests (plane-independent, run once)
# =========================================================================


class TestDegenerateInput(unittest.TestCase):
    def test_single_open_line(self):
        try:
            faces = Part.makeFace(
                Part.Compound([line_wire((0, 0), (10, 0))]),
                "Part::FaceMakerFishEye",
            ).Faces
            self.assertEqual(len(faces), 0)
        except RuntimeError:
            pass

    def test_empty_compound(self):
        try:
            shape = Part.makeFace(Part.Compound([]), "Part::FaceMakerFishEye")
            self.assertEqual(len(shape.Faces), 0)
        except RuntimeError:
            pass


# =========================================================================
# Plane definitions and class generation
# =========================================================================

_MIXINS = [
    _SingleShape,
    _SeparateShapes,
    _Nesting,
    _TwoOverlapping,
    _Subdivision,
    _MultipleOverlapping,
    _OverlapWithHoles,
    _Degenerate,
]

_PLANES = {
    "XY": FreeCAD.Placement(),
    "XZ": FreeCAD.Placement(Vec(0, 0, 0), FreeCAD.Rotation(Vec(1, 0, 0), 90)),
    "Tilted": FreeCAD.Placement(Vec(10, -20, 15), FreeCAD.Rotation(Vec(1, 0, 0), 45) * FreeCAD.Rotation(Vec(0, 0, 1), 30)),
}

for _plane_name, _placement in _PLANES.items():
    for _mixin in _MIXINS:
        _cls_name = f"{_mixin.__name__[1:]}_{_plane_name}"
        _cls = type(_cls_name, (_mixin, unittest.TestCase), {"placement": _placement})
        globals()[_cls_name] = _cls
