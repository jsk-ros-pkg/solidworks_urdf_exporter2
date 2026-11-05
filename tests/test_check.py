"""sw2robot.exporter.check: findings a program can act on.

The declared-joint check is the one an agent leans on: a joint it proposes must
be a motion the CAD mates between the same two parts actually allow.
"""
import asyncio
import json
import os

import numpy as np
import pytest

from sw2robot.exporter import check
from sw2robot.exporter.state import (
    ComponentState,
    DeclaredJoint,
    GraphState,
    MateEdge,
    MateGeo,
)

EXAMPLE = os.path.join(os.path.dirname(__file__), "..", "examples", "fingertip")
PIN = [0.1, 0.0, 0.0]          # the hinge pin: through PIN along +Z


def _comp(name, xyz=(0, 0, 0), fixed=False, **kw):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(name=name, link_name=name, fixed=fixed,
                          world=[float(x) for x in w.flatten()], **kw)


def _hinge_edge():
    # a concentric pin plus a coincident face: exactly one free rotation
    return MateEdge(a="base", b="arm", types=["CONCENTRIC", "COINCIDENT"], mates=[
        MateGeo(type="CONCENTRIC", etypes=[4, 4], points=[PIN, PIN],
                dirs=[[0, 0, 1], [0, 0, 1]], radii=[0.004, 0.004]),
        MateGeo(type="COINCIDENT", etypes=[3, 3], points=[PIN, PIN],
                dirs=[[0, 0, 1], [0, 0, 1]], radii=[None, None]),
    ])


def _graph(joints=(), edges=None, components=None):
    comps = components or [_comp("base", fixed=True), _comp("arm", PIN)]
    return GraphState(robot_name="t", source_assembly="t.iam", components=comps,
                      edges=[_hinge_edge()] if edges is None else list(edges),
                      ground=["base"], joints=list(joints))


def _codes(findings):
    return {f.code: f for f in findings}


def _revolute(**kw):
    args = {"parent": "base", "child": "arm", "type": "revolute",
            "axis_point": PIN, "axis_dir": [0, 0, 1], "name": "elbow", **kw}
    return DeclaredJoint(**args)


def test_the_hinge_fixture_leaves_exactly_one_rotation():
    from sw2robot.exporter.model import mate_motion_basis
    assert mate_motion_basis(_hinge_edge().mates).shape == (6, 1)


def test_a_joint_the_mates_allow_raises_nothing():
    assert "joint-motion-blocked" not in _codes(
        check.check_graph(_graph([_revolute()])))


def test_reversed_axis_is_the_same_motion():
    assert "joint-motion-blocked" not in _codes(
        check.check_graph(_graph([_revolute(axis_dir=[0, 0, -3])])))


@pytest.mark.parametrize("override", [
    {"axis_dir": [1, 0, 0]},                  # wrong direction
    {"axis_point": [0.1, 0.02, 0.0]},         # parallel but 2 cm off the pin
    {"type": "prismatic"},                    # slides along the pin instead
])
def test_a_joint_the_mates_forbid_is_an_error(override):
    found = _codes(check.check_graph(_graph([_revolute(**override)])))
    f = found["joint-motion-blocked"]
    assert f.severity == "error"
    assert f.subjects == ["elbow", "base", "arm"]
    assert f.data["mate_dof"] == 1


def test_fixed_over_a_hinge_is_noted():
    fixed = DeclaredJoint(parent="base", child="arm", type="fixed")
    f = _codes(check.check_graph(_graph([fixed])))["joint-under-declared"]
    assert f.severity == "info"
    assert f.data["mate_dof"] == 1


def test_joint_without_mates_is_not_second_guessed():
    graph = _graph([_revolute(axis_dir=[1, 0, 0])], edges=[])
    assert "joint-motion-blocked" not in _codes(check.check_graph(graph))


def test_joint_structure_problems():
    joints = [
        _revolute(),
        _revolute(name="again"),
        DeclaredJoint(parent="base", child="ghost", type="fixed"),
        DeclaredJoint(parent="arm", child="base", type="ball"),
    ]
    found = _codes(check.check_graph(_graph(joints)))
    assert found["joint-duplicate-pair"].severity == "warning"
    assert found["joint-unknown-component"].data == {"missing": ["ghost"]}
    assert found["joint-multi-dof"].data == {"type": "ball"}


def test_component_problems(tmp_path):
    comps = [
        _comp("base", fixed=True, mesh_file="meshes/base.stl"),
        _comp("arm", PIN, sw_mass=0.2,
              sw_inertia=[1e-4, 0, 0, 1e-4, 0, -1e-4]),
        _comp("loose", (0.5, 0, 0)),
    ]
    found = _codes(check.check_graph(_graph(components=comps), str(tmp_path)))
    assert found["mesh-missing"].subjects == ["base"]
    assert found["mass-invalid"].subjects == ["arm"]
    assert found["component-unconstrained"].subjects == ["loose"]


def test_default_range_is_reported_after_the_build():
    graph = _graph()
    report_codes = _codes(check.check_graph(graph))
    assert "joint-default-range" not in report_codes
    from sw2robot.exporter.model import build_model
    found = _codes(check.check_model(build_model(graph)))
    assert found["joint-default-range"].data == {"lower": -3.141592,
                                                 "upper": 3.141592}


def _package(tmp_path, graph):
    graph.save(str(tmp_path / "graph.json"))
    return str(tmp_path)


def test_report_is_plain_json_and_not_ok_on_errors(tmp_path):
    pkg = _package(tmp_path, _graph([_revolute(axis_dir=[1, 0, 0])]))
    report = check.check_package(pkg)
    assert json.loads(json.dumps(report)) == report
    assert report["ok"] is False
    assert report["findings"][0]["severity"] == "error"   # errors first
    assert report["counts"]["movable_joints"] == 1


def test_unreadable_graph_is_a_finding(tmp_path):
    (tmp_path / "graph.json").write_text("{}", encoding="utf-8")
    report = check.check_package(str(tmp_path))
    assert report["ok"] is False
    assert report["findings"][0]["code"] == "graph-invalid"


def test_cli_json_keeps_stdout_clean(tmp_path, capsys):
    pkg = _package(tmp_path, _graph([_revolute(axis_dir=[1, 0, 0])]))
    assert check.main([pkg, "--json"]) == 1
    out = capsys.readouterr()
    assert json.loads(out.out)["ok"] is False
    assert "[build]" not in out.out and "base link" in out.err


def test_bundled_example_has_no_errors():
    report = check.check_package(
        EXAMPLE, os.path.join(EXAMPLE, "fingertip.joints.yaml"))
    assert report["ok"], report["findings"]
    assert report["counts"]["movable_joints"] == 1


def test_mcp_server_exposes_the_check(tmp_path):
    pytest.importorskip("mcp")
    from sw2robot import mcp_server
    server = mcp_server.create_server()
    names = {t.name for t in asyncio.run(server.list_tools())}
    assert names == {"check_package", "build_package", "graph_schema"}

    pkg = _package(tmp_path, _graph([_revolute()]))
    result = asyncio.run(server.call_tool("build_package",
                                          {"package_dir": pkg}))
    assert not result.is_error, result.content
    payload = result.structured_content
    assert os.path.exists(payload["urdf"])
    assert payload["report"]["ok"] is True


def _slider_edge():
    # two coincident faces leave a slide along X; the DISTANCE mate between the
    # X faces is how a SolidWorks LimitDistance mate appears in the mates
    plane = lambda n, t: MateGeo(type=t, etypes=[3, 3], points=[PIN, PIN],
                                 dirs=[n, n], radii=[None, None])
    return MateEdge(a="base", b="arm",
                    types=["COINCIDENT", "COINCIDENT", "DISTANCE"],
                    mates=[plane([0, 0, 1], "COINCIDENT"),
                           plane([0, 1, 0], "COINCIDENT"),
                           plane([1, 0, 0], "DISTANCE")])


def test_a_limit_mate_bounds_the_slide_instead_of_blocking_it():
    from sw2robot.exporter.state import LimitJoint
    slide = DeclaredJoint(parent="base", child="arm", type="prismatic",
                          axis_dir=[1, 0, 0], name="rail")
    graph = _graph([slide], edges=[_slider_edge()])
    # without the limit mate the DISTANCE really is a lock
    assert "joint-motion-blocked" in _codes(check.check_graph(graph))
    graph.limit_joints = [LimitJoint(
        a="arm", b="base", type="prismatic", axis_point=PIN,
        axis_dir=[1, 0, 0], lower=0.0, upper=0.05)]
    assert "joint-motion-blocked" not in _codes(check.check_graph(graph))
    # a declared axis the limit mate does not allow is still caught
    graph.joints = [DeclaredJoint(parent="base", child="arm", type="prismatic",
                                  axis_dir=[0, 0, 1], name="rail")]
    assert "joint-motion-blocked" in _codes(check.check_graph(graph))


def test_a_mesh_outside_the_package_is_not_probed(tmp_path):
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    (tmp_path / "secret.stl").write_text("x")
    comps = [_comp("base", fixed=True, mesh_file="../secret.stl"),
             _comp("arm", PIN)]
    found = _codes(check.check_graph(_graph(components=comps), str(pkg)))
    assert found["mesh-outside-package"].subjects == ["base"]
    assert "mesh-missing" not in found
