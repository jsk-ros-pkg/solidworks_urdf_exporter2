"""sw2robot.exporter.sim_check: a model that compiles but does not simulate.

Each case is a tiny hand-written MJCF whose failure is known in advance, so the
check is tested against physics rather than against an exporter detail.
"""
import importlib
import json
import os

import pytest


def _import_extra(name):
    """Import an optional dependency, skipping when it is absent -- unless CI
    lists it in SW2ROBOT_REQUIRE_EXTRAS, where a broken install must fail."""
    required = os.environ.get("SW2ROBOT_REQUIRE_EXTRAS", "").split(",")
    if name in required:
        return importlib.import_module(name)
    return pytest.importorskip(name)


mujoco = _import_extra("mujoco")

from sw2robot.exporter import sim_check  # noqa: E402

EXAMPLE = os.path.join(os.path.dirname(__file__), "..", "examples", "fingertip")

FLOOR = '<geom name="floor" type="plane" size="2 2 0.1"/>'


def _write(tmp_path, body, option="", floor=FLOOR):
    xml = (f'<mujoco model="t"><option {option}/><worldbody>{floor}'
           f'{body}</worldbody></mujoco>')
    path = tmp_path / "t.xml"
    path.write_text(xml, encoding="utf-8")
    return str(path)


def _codes(report):
    return {f["code"]: f for f in report["findings"]}


def test_a_resting_box_is_fine(tmp_path):
    path = _write(tmp_path, '<body name="box" pos="0 0 0.1"><freejoint/>'
                            '<geom type="box" size="0.1 0.1 0.1"/></body>')
    report = sim_check.simulate(path, seconds=0.5)
    assert report["ok"], report["findings"]
    assert report["findings"] == []
    assert report["simulated"] == pytest.approx(0.5)


def test_parts_that_start_overlapping_are_named(tmp_path):
    path = _write(tmp_path, '<body name="sunk" pos="0 0 0.05"><freejoint/>'
                            '<geom type="box" size="0.1 0.1 0.1"/></body>')
    f = _codes(sim_check.simulate(path, seconds=0.1))["sim-initial-penetration"]
    assert f["severity"] == "warning"
    assert set(f["subjects"]) == {"sunk", "world"}
    assert f["data"]["depth_m"] == pytest.approx(0.05, abs=1e-6)


def test_an_exploding_joint_is_an_error(tmp_path):
    # a stiff spring on an almost massless arm, integrated with a huge step
    body = ('<body name="arm"><joint name="hinge" type="hinge" axis="0 0 1" '
            'springref="1" stiffness="1e9"/><geom type="capsule" fromto="0 0 1 0.2 0 1" '
            'size="0.01" mass="1e-6"/></body>')
    path = _write(tmp_path, body, option='timestep="0.01"', floor="")
    report = sim_check.simulate(path, seconds=1.0)
    assert not report["ok"]
    assert {"sim-unstable", "sim-diverged", "sim-warning"} & set(_codes(report))
    assert report["simulated"] < 1.0      # stopped at the failure


def test_falling_through_the_floor_is_an_error(tmp_path):
    # the box does not collide with anything, so it drops through the floor
    body = ('<body name="ghost" pos="0 0 0.2"><freejoint/>'
            '<geom type="box" size="0.1 0.1 0.1" contype="0" '
            'conaffinity="0"/></body>')
    f = _codes(sim_check.simulate(_write(tmp_path, body)))["sim-fell-through"]
    assert f["severity"] == "error"


def test_without_a_floor_falling_is_expected(tmp_path):
    body = ('<body name="free" pos="0 0 0.2"><freejoint/>'
            '<geom type="box" size="0.1 0.1 0.1"/></body>')
    report = sim_check.simulate(_write(tmp_path, body, floor=""))
    assert "sim-fell-through" not in _codes(report)


def test_the_home_keyframe_is_the_start_pose(tmp_path):
    # "stowed" (sunk into the floor) is listed first; the run must start from
    # "home", as the MJCF export names the standing pose
    body = ('<body name="box" pos="0 0 0.1"><freejoint/>'
            '<geom type="box" size="0.1 0.1 0.1"/></body>')
    keys = ('<keyframe><key name="stowed" qpos="0 0 0.05 1 0 0 0"/>'
            '<key name="home" qpos="0 0 0.1 1 0 0 0"/></keyframe>')
    path = _write(tmp_path, body)
    with open(path, encoding="utf-8") as f:
        xml = f.read().replace("</mujoco>", keys + "</mujoco>")
    with open(path, "w", encoding="utf-8") as f:
        f.write(xml)
    report = sim_check.simulate(path, seconds=0.1)
    assert "sim-initial-penetration" not in _codes(report), report["findings"]


def test_an_unloadable_model_is_a_finding(tmp_path):
    path = tmp_path / "bad.xml"
    path.write_text("<mujoco><worldbody><body><geom type='nope'/></body>"
                    "</worldbody></mujoco>", encoding="utf-8")
    report = sim_check.simulate(str(path))
    assert _codes(report)["sim-compile-failed"]["severity"] == "error"


def test_cli_json(tmp_path, capsys):
    path = _write(tmp_path, '<body name="sunk" pos="0 0 0.05"><freejoint/>'
                            '<geom type="box" size="0.1 0.1 0.1"/></body>')
    assert sim_check.main([path, "--seconds", "0.1", "--json"]) == 0
    assert json.loads(capsys.readouterr().out)["counts"]["warning"] == 1


def test_mcp_simulates_the_bundled_example(tmp_path):
    _import_extra("mcp")
    import asyncio
    import shutil

    from sw2robot import mcp_server

    pkg = tmp_path / "fingertip"
    shutil.copytree(EXAMPLE, pkg)
    server = mcp_server.create_server()
    result = asyncio.run(server.call_tool("simulate_package", {
        "package_dir": str(pkg),
        "config_path": str(pkg / "fingertip.joints.yaml"),
        "seconds": 0.5}))
    assert not result.is_error, result.content
    payload = result.structured_content
    assert os.path.exists(payload["mjcf"])
    assert payload["report"]["ok"], payload["report"]["findings"]
    assert payload["report"]["simulated"] == pytest.approx(0.5)
