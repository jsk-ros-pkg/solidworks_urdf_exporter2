"""Twist-nullspace mate classification (classify_edge_geo) on synthetic cases.

Each case is a list of MateGeo-shaped dicts as build_mate_graph records them
(every mate duplicated, since GetMates returns it once per component)."""
import numpy as np
import pytest

from sw2robot.exporter.model import classify_edge_geo, classify_edge_auto

Z = [0.0, 0.0, 1.0]
X = [1.0, 0.0, 0.0]
Y = [0.0, 1.0, 0.0]
O = [0.0, 0.0, 0.0]


def mate(mtype, ents):
    """ents: list of (etype, point, dir)."""
    return {"type": mtype,
            "etypes": [e[0] for e in ents],
            "points": [list(map(float, e[1])) for e in ents],
            "dirs": [list(map(float, e[2])) for e in ents],
            "radii": [None] * len(ents)}


def dup(*mates_):
    """Simulate the per-component double iteration."""
    return list(mates_) + list(mates_)


PLANE, CYL, LINE, POINT = 3, 4, 1, 0


def conc(p, d=Z):
    return mate("CONCENTRIC", [(CYL, p, d), (CYL, p, d)])


def coinc_planes(p, n):
    return mate("COINCIDENT", [(PLANE, p, n), (PLANE, p, n)])


def dist_planes(p, n):
    return mate("DISTANCE", [(PLANE, p, n), (PLANE, p, n)])


def para_planes(n):
    return mate("PARALLEL", [(PLANE, O, n), (PLANE, O, n)])


def test_hinge_single_axis_is_revolute():
    # one shaft + end-face contact (plane normal along the axis)
    jt, ax, note = classify_edge_geo(dup(conc(O), coinc_planes(O, Z)))
    assert jt == "revolute"
    assert abs(np.dot(ax[1], Z)) > 0.999


def test_bolt_pair_two_offset_axes_is_fixed():
    # two parallel but offset concentric mates = bolt pattern, not a hinge
    jt, ax, note = classify_edge_geo(
        dup(conc([0.01, 0, 0]), conc([-0.01, 0, 0]),
            coinc_planes(O, Z)))
    assert jt == "fixed"


def test_bolt_pair_without_facecontact_still_not_revolute():
    # screw_yaw servo mount shape: only the bolts, no coincident plane --
    # rotation is dead; the leftover axial slide must NOT become revolute
    jt, ax, note = classify_edge_geo(
        dup(conc([0.01, 0, 0]), conc([-0.01, 0, 0])))
    assert jt != "revolute"


def test_servo_mount_concentric_plus_perp_parallel_is_fixed():
    # one boss + PARALLEL whose plane normal is PERPENDICULAR to the boss
    # axis: the parallel mate kills the rotation -> rigid mount
    jt, ax, note = classify_edge_geo(dup(conc(O, Z), para_planes(X)))
    assert jt == "fixed"


def test_hinge_with_axis_parallel_parallel_stays_revolute():
    # feetech case: PARALLEL just keeps the shaft dirs parallel (normal along
    # the hinge axis) -- it must NOT freeze the hinge
    jt, ax, note = classify_edge_geo(dup(conc(O, Z), para_planes(Z)))
    assert jt == "revolute"
    assert abs(np.dot(ax[1], Z)) > 0.999


def test_fingertip_distance_blocks_rotation():
    # vial_pick fingertips: screw-hole CONCENTRIC + DISTANCE planes whose
    # normal is perpendicular to that axis -> rigid
    jt, ax, note = classify_edge_geo(
        dup(conc(O, Z), dist_planes([0, 0.02, 0], Y),
            dist_planes([0.02, 0, 0], X)))
    assert jt == "fixed"


def test_axial_distance_does_not_block_hinge():
    # DISTANCE along the hinge axis (e.g. washer gap) leaves rotation free
    jt, ax, note = classify_edge_geo(
        dup(conc(O, Z), dist_planes([0, 0, 0.005], Z)))
    assert jt == "revolute"


def test_pure_prismatic_two_distances():
    # two DISTANCE plane pairs with non-parallel normals leave exactly one
    # translation (along n1 x n2)
    jt, ax, note = classify_edge_geo(
        dup(dist_planes(O, X), dist_planes(O, Z)))
    assert jt == "prismatic"
    assert abs(np.dot(ax[1], Y)) > 0.999


def test_lock_is_fixed():
    jt, ax, note = classify_edge_geo(dup(mate("LOCK", [(PLANE, O, Z)])))
    assert jt == "fixed"


def test_coupling_only_falls_back():
    # RACKPINION alone gives no constraint rows -> None (legacy fallback)
    assert classify_edge_geo(dup(mate("RACKPINION", [(CYL, O, Z)]))) is None


def test_coupling_noted_alongside_real_mates():
    jt, ax, note = classify_edge_geo(
        dup(conc(O, Z), coinc_planes(O, Z), mate("RACKPINION", [(CYL, O, Z)])))
    assert jt == "revolute"
    assert "coupling" in note


def test_double_concentric_same_axis_is_one_hinge():
    # two coaxial bearings on the same shaft must still be a hinge
    jt, ax, note = classify_edge_geo(
        dup(conc([0, 0, 0.01], Z), conc([0, 0, -0.01], Z),
            coinc_planes(O, Z)))
    assert jt == "revolute"


def test_auto_falls_back_to_legacy_without_geo():
    # old graph.json: no 'mates' -> legacy heuristic must keep working
    rec = {"types": ["CONCENTRIC", "COINCIDENT"] * 2,
           "axis": (np.zeros(3), np.array(Z, float)), "mates": []}
    jt, ax, note = classify_edge_auto(rec)
    assert jt == "revolute" and note is None


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
