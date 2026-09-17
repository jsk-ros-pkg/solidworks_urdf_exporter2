"""GraphBuilder: a robot written in code becomes a package that builds.

The joints are declared, so the build must reproduce them exactly, in metres,
whatever unit the code was written in.
"""
import importlib
import os

import numpy as np
import pytest
import trimesh

from sw2robot.exporter.check import check_package
from sw2robot.exporter.graph_builder import GraphBuilder
from sw2robot.exporter.model import build_model
from sw2robot.exporter.state import GraphState


def _import_extra(name):
    """Import an optional dependency, skipping when it is absent -- unless CI
    lists it in SW2ROBOT_REQUIRE_EXTRAS, where a broken install must fail."""
    required = os.environ.get("SW2ROBOT_REQUIRE_EXTRAS", "").split(",")
    if name in required:
        return importlib.import_module(name)
    return pytest.importorskip(name)


def _pose(xyz):
    T = np.eye(4)
    T[:3, 3] = xyz
    return T


def _arm(unit, s):
    """base + link1 on a hinge + slider on a rail, lengths scaled by ``s``."""
    g = GraphBuilder("two link", unit=unit, cad="trimesh")
    box = trimesh.creation.box(extents=[0.1 * s, 0.02 * s, 0.02 * s])
    g.add_part("base", box, fixed=True, density=1000)
    g.add_part("link1", box, pose=_pose([0.1 * s, 0, 0]), mass=0.2)
    g.add_part("slider", box, pose=_pose([0.2 * s, 0, 0]))
    g.add_joint("base", "link1", "revolute", axis_dir=[0, 0, 1],
                axis_point=[0.05 * s, 0, 0], lower=-1.0, upper=1.0,
                name="shoulder", source="ai")
    g.add_joint("link1", "slider", "prismatic", axis_dir=[1, 0, 0],
                lower=-0.01 * s, upper=0.03 * s, name="rail")
    return g


@pytest.mark.parametrize("unit, s", [("m", 1.0), ("mm", 1000.0)])
def test_code_robot_builds_in_metres(tmp_path, unit, s):
    pkg = _arm(unit, s).write(str(tmp_path))
    graph = GraphState.load(f"{pkg}/graph.json")
    assert graph.robot_name == "two_link"
    assert graph.source.cad == "trimesh"
    assert graph.ground == ["base"]
    mesh = trimesh.load(f"{pkg}/meshes/link1.stl")
    np.testing.assert_allclose(mesh.extents, [0.1, 0.02, 0.02])

    joints = {j.name: j for j in build_model(graph).joints}
    shoulder, rail = joints["shoulder"], joints["rail"]
    assert shoulder.jtype == "revolute"
    np.testing.assert_allclose(shoulder.xyz, [0.05, 0, 0], atol=1e-12)
    assert (shoulder.lower, shoulder.upper) == (-1.0, 1.0)
    assert rail.jtype == "prismatic"
    assert (rail.lower, rail.upper) == pytest.approx((-0.01, 0.03))

    report = check_package(pkg)
    assert report["ok"], report["findings"]
    assert report["counts"]["movable_joints"] == 2


def test_mass_and_density_reach_the_urdf(tmp_path):
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.export import build

    pkg = _arm("m", 1.0).write(str(tmp_path))
    root = ET.parse(build(pkg)).getroot()
    mass = {link.get("name"): float(link.find("inertial/mass").get("value"))
            for link in root.iter("link") if link.find("inertial") is not None}
    assert mass["link1"] == pytest.approx(0.2)
    # the root link is renamed base_link by default
    assert mass["base_link"] == pytest.approx(0.1 * 0.02 * 0.02 * 1000)


def test_mistakes_fail_where_they_are_made():
    g = GraphBuilder("r")
    box = trimesh.creation.box()
    g.add_part("a-1", box)
    with pytest.raises(ValueError, match="already added"):
        g.add_part("a-1", box)
    with pytest.raises(ValueError, match="sanitises"):
        g.add_part("a_1", box)
    with pytest.raises(ValueError, match="no part 'ghost'"):
        g.add_joint("a-1", "ghost", "fixed")
    with pytest.raises(ValueError, match="mass or density"):
        g.add_part("b", box, mass=1.0, density=1.0)
    with pytest.raises(TypeError, match="unsupported geometry"):
        g.add_part("c", object())
    with pytest.raises(ValueError, match="unit"):
        GraphBuilder("r", unit="inch")


def test_build123d_shapes(tmp_path):
    bd = _import_extra("build123d")
    g = GraphBuilder("b3d", unit="mm", cad="build123d")
    g.add_part("base", bd.Box(100, 20, 20), fixed=True, density=1200)
    g.add_part("arm", bd.Cylinder(5, 60), pose=_pose([0, 0, 40]))
    g.add_joint("base", "arm", "continuous", axis_dir=[0, 0, 1])
    pkg = g.write(str(tmp_path))
    mesh = trimesh.load(f"{pkg}/meshes/base.stl")
    np.testing.assert_allclose(mesh.extents, [0.1, 0.02, 0.02], atol=1e-6)
    assert check_package(pkg)["ok"]
