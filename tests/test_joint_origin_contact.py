"""A revolute joint's origin sits where the parts meet, not where SolidWorks
happened to put the cylinder's origin.

Two hip axles extruded from one sketch share one cylinder origin, so the
concentric mate reports the SAME point for the left and the right leg; used
verbatim, the right hip's frame lands on the left side of the body.  The
coincident mate between the two touching faces says where the joint is.
"""
import numpy as np
import pytest

from sw2robot.exporter.model import _contact_point_on_axis

AXIS = [0.0, -1.0, 0.0]


def _coincident(y, normal_a, normal_b):
    return {"type": "COINCIDENT", "etypes": [3, 3],
            "points": [[-0.11, y, 0.02], [0.085, y, 0.0]],
            "dirs": [normal_a, normal_b], "radii": [0.0, 0.0]}


def test_point_slides_along_the_axis_onto_the_contact_plane():
    # the right hip of the tutorial quadruped: SW point at +y, faces at -y
    rec = {"mates": [_coincident(-0.06, [0, -1, 0], [0, 1, 0])]}
    got = _contact_point_on_axis(rec, [0.085, 0.068, 0.0], AXIS)
    assert got == pytest.approx([0.085, -0.06, 0.0])


def test_nearest_of_several_contact_planes_wins():
    rec = {"mates": [_coincident(-0.06, [0, -1, 0], [0, 1, 0]),
                     _coincident(0.05, [0, 1, 0], [0, -1, 0])]}
    got = _contact_point_on_axis(rec, [0.0, 0.068, 0.0], AXIS)
    assert got == pytest.approx([0.0, 0.05, 0.0])


def test_a_plane_parallel_to_the_axis_is_not_a_contact_plane():
    # a coincident plane CONTAINING the axis (normal along x) says nothing
    # about where along the axis the joint is
    rec = {"mates": [_coincident(0.0, [1, 0, 0], [-1, 0, 0])]}
    pt = np.array([0.085, 0.068, 0.0])
    assert _contact_point_on_axis(rec, pt, AXIS) == pytest.approx(pt)


def test_no_coincident_mate_keeps_the_point():
    rec = {"mates": [{"type": "CONCENTRIC", "etypes": [4, 4],
                      "points": [[0.085, 0.068, 0.0]] * 2,
                      "dirs": [AXIS, AXIS], "radii": [0.005, 0.005]}]}
    pt = [0.085, 0.068, 0.0]
    assert _contact_point_on_axis(rec, pt, AXIS) == pytest.approx(pt)
