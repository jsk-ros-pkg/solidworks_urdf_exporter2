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


def _hinge_origin(tmp_path, extra=""):
    """Build the fingertip package; the screwlock hinge's <origin xyz>."""
    import shutil
    import xml.etree.ElementTree as ET
    from pathlib import Path

    from sw2robot.exporter.export import build

    fingertip = Path(__file__).resolve().parent.parent / "examples" / "fingertip"
    pkg = tmp_path / "pkg"
    (pkg / "meshes").mkdir(parents=True)
    shutil.copy(fingertip / "graph.json", pkg / "graph.json")
    for mesh in (fingertip / "meshes").glob("*.3dxml"):
        shutil.copy(mesh, pkg / "meshes" / mesh.name)
    cfg = pkg / "joints.yaml"
    cfg.write_text(
        "base: fingertip_front_2\n"
        "joints:\n"
        "  - parent: fingertip_front_2\n"
        "    child: screwlock_male_hard_jointbase_v4_1\n"
        "    type: revolute\n" + extra, encoding="utf-8")
    urdf = ET.parse(build(str(pkg), config_path=str(cfg))).getroot()
    joint = next(j for j in urdf.findall("joint")
                 if j.find("child").get("link")
                 == "screwlock_male_hard_jointbase_v4_1")
    return [float(v) for v in joint.find("origin").get("xyz").split()]


def test_a_configured_axis_point_is_used_as_written(tmp_path):
    # the screwlock hinge has a contact plane 3.5 mm along its axis from the
    # concentric point; an axis_point in the config must not be slid onto it
    slid = _hinge_origin(tmp_path / "a")
    pinned = _hinge_origin(
        tmp_path / "b",
        "    axis_point: [0.0, -0.017, 0.01]\n"
        "    axis_dir: [0.0, -1.0, 0.0]\n")
    assert slid[1] == pytest.approx(-0.002)
    assert pinned[1] == pytest.approx(0.0015)
