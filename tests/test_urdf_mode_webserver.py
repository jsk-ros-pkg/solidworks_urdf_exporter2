"""Integration tests for URDF-input editing mode in the web server.

A package with NO graph.json is a plain URDF the user opened directly; the
server then holds the core edit overlay in memory, serves build_urdf(state) for
the URDF URL, and routes every edit through the core setters.  These tests drive
a real in-process HTTP server (no browser) against a graph-less copy of the
committed fingertip example.

The CAD path is untouched and is guarded separately by the golden + e2e suites.
"""

import json
import shutil
import threading
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
FINGERTIP = REPO_ROOT / "examples" / "fingertip"

REV_JOINT = "fingertip_front_2__fingertip_back_1"     # child = fingertip_back_1
FIXED_JOINT = "fingertip_front_2__screwlock_male_hard_jointbase_v4_1"
TIP_LINK = "fingertip_back_1"
SCREW_LINK = "screwlock_male_hard_jointbase_v4_1"


def _require_fixture():
    if not (FINGERTIP / "graph.json").is_file():
        pytest.skip("missing fingertip fixture")


def _urdf_only_pkg(tmp_path):
    """Build the fingertip example, then copy ONLY urdf/ + meshes/ into a fresh
    dir (no graph.json / joints.yaml) -- a plain URDF package."""
    from sw2robot.exporter.export import build

    cad = tmp_path / "cad"
    (cad / "meshes").mkdir(parents=True)
    shutil.copy2(FINGERTIP / "graph.json", cad / "graph.json")
    for f in (FINGERTIP / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, cad / "meshes" / f.name)
    build(str(cad))

    pkg = tmp_path / "urdf_only"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    shutil.copy2(cad / "urdf" / "fingertip.urdf", pkg / "urdf" / "fingertip.urdf")
    for f in (cad / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, pkg / "meshes" / f.name)
    assert not (pkg / "graph.json").exists()
    return pkg


@pytest.fixture(scope="module")
def _built_template(tmp_path_factory):
    """Build the URDF-only package ONCE for the module (build() dominates the
    per-test cost); each test gets a fresh copy for isolation."""
    _require_fixture()
    return _urdf_only_pkg(tmp_path_factory.mktemp("tmpl"))


@pytest.fixture
def server(_built_template, tmp_path):
    """A running web server with a fresh URDF-only fingertip package opened."""
    from sw2robot.editor import webserver

    pkg = tmp_path / "pkg"
    shutil.copytree(_built_template, pkg)
    httpd, port = webserver._bind_free_port(webserver._Handler,
                                            _free_port())
    httpd.daemon_threads = True
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    base = f"http://127.0.0.1:{port}"
    try:
        r = _get_json(base, f"/api/open?path={pkg}")
        assert r.get("name") == "fingertip"
        assert webserver._um["state"] is not None     # URDF mode engaged
        yield base
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def _free_port():
    import socket
    s = socket.socket()
    s.bind(("", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _get(base, path):
    with urllib.request.urlopen(base + path) as r:
        return r.read().decode("utf-8")


def _get_json(base, path):
    return json.loads(_get(base, path))


def _post(base, path, body):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def _served_urdf(base):
    info = _get_json(base, "/api/info")
    return ET.fromstring(_get(base, info["urdf"]))


def _joint(root, name):
    return next(j for j in root.findall("joint") if j.get("name") == name)


def _link(root, name):
    return next(l for l in root.findall("link") if l.get("name") == name)


# --------------------------------------------------------------- tests
def test_serves_base_urdf_unedited(server):
    _require_fixture()
    root = _served_urdf(server)
    assert root.get("name") == "fingertip"
    assert _link(root, TIP_LINK).find("visual/material") is None


def test_set_color_reflected_in_served_urdf(server):
    code, r = _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#ff0000"})
    assert code == 200 and r["color"] == "#ff0000"
    mat = _link(_served_urdf(server), TIP_LINK).find("visual/material/color")
    assert mat.get("rgba") == "1 0 0 1"


def test_set_color_invalid_is_400(server):
    code, r = _post(server, "/api/set_color", {"link": TIP_LINK, "color": "nope"})
    assert code == 400 and "error" in r


def test_set_inertial(server):
    code, r = _post(server, "/api/set_inertial",
                    {"link": TIP_LINK, "mass": 2.5, "com": [0.01, 0, 0]})
    assert code == 200 and r["mass"] == 2.5
    ine = _link(_served_urdf(server), TIP_LINK).find("inertial")
    assert float(ine.find("mass").get("value")) == 2.5
    assert ine.find("origin").get("xyz") == "0.01 0 0"


def test_set_inertial_nonphysical_is_400(server):
    code, r = _post(server, "/api/set_inertial",
                    {"link": TIP_LINK, "inertia": [1, 0, 0, 1, 0, 10]})
    assert code == 400 and "error" in r


def test_set_limits(server):
    code, r = _post(server, "/api/set_limits",
                    {"limits": [{"child": TIP_LINK, "lower": -0.3,
                                 "upper": 0.7, "continuous": False}]})
    assert code == 200 and r["applied"] == [TIP_LINK]
    lim = _joint(_served_urdf(server), REV_JOINT).find("limit")
    assert float(lim.get("lower")) == -0.3 and float(lim.get("upper")) == 0.7


def test_set_type_then_mimic(server):
    # make the fixed joint movable so it can drive a mimic
    code, _ = _post(server, "/api/set_types",
                    {"changes": [{"child": SCREW_LINK, "type": "revolute"}]})
    assert code == 200
    changed = _joint(_served_urdf(server), FIXED_JOINT)
    assert changed.get("type") == "revolute"
    assert changed.find("limit") is not None     # valid URDF: limit backfilled

    code, r = _post(server, "/api/set_mimic",
                    {"changes": [{"child": TIP_LINK, "master": FIXED_JOINT,
                                  "multiplier": 0.5, "offset": 0.1}]})
    assert code == 200 and r["applied"] == [TIP_LINK]
    mim = _joint(_served_urdf(server), REV_JOINT).find("mimic")
    assert mim.get("joint") == FIXED_JOINT
    assert mim.get("multiplier") == "0.5" and mim.get("offset") == "0.1"


def test_flip_axis(server):
    base_axis = _joint(_served_urdf(server), REV_JOINT).find("axis").get("xyz")
    code, r = _post(server, "/api/set_axis", {"joints": [REV_JOINT]})
    assert code == 200 and r["applied"] == [REV_JOINT]
    flipped = _joint(_served_urdf(server), REV_JOINT).find("axis").get("xyz")
    assert flipped != base_axis
    # self-inverse: flipping again restores the original axis
    _post(server, "/api/set_axis", {"joints": [REV_JOINT]})
    assert _joint(_served_urdf(server), REV_JOINT).find("axis").get("xyz") == base_axis


def test_flip_after_rename(server):
    """The flip endpoint accepts the CURRENT (renamed) joint name."""
    _post(server, "/api/rename", {"kind": "joint", "old": REV_JOINT, "new": "bend"})
    code, r = _post(server, "/api/set_axis", {"joints": ["bend"]})
    assert code == 200 and r["applied"] == ["bend"]


def test_export_colors_come_from_overlay(server):
    """URDF mode has no joints.yaml colours block; the exporter's repaint map is
    built from the overlay instead."""
    from sw2robot.editor import webserver

    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#abcdef"})
    assert webserver._um_colors(webserver._um["state"]) == {TIP_LINK: "#abcdef"}


def test_reset_names_clears_joint_renames(server):
    _post(server, "/api/rename", {"kind": "joint", "old": REV_JOINT, "new": "bend"})
    assert "bend" in [j.get("name") for j in _served_urdf(server).findall("joint")]
    code, r = _post(server, "/api/reset_names", {})
    assert code == 200 and r["ok"] and r["reset"] == 1
    names = [j.get("name") for j in _served_urdf(server).findall("joint")]
    assert REV_JOINT in names and "bend" not in names


def test_components_lists_links_with_overlay_colour(server):
    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#00ff00"})
    comp = _get_json(server, "/api/components")
    assert TIP_LINK in comp["links"]
    assert comp["colors"][TIP_LINK] == "#00ff00"


def test_edits_persist_to_sidecar(server, tmp_path):
    from sw2robot.editor import core, webserver

    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#123456"})
    _post(server, "/api/set_limits",
          {"limits": [{"child": TIP_LINK, "lower": -1, "upper": 1}]})
    # a fresh state loaded from disk must see the persisted overlay
    state = webserver._um["state"]
    reloaded = core.load_module(state.urdf_path, package_dir=state.package_dir)
    assert core.load_edits(reloaded) >= 2
    assert reloaded.link_edits[TIP_LINK].color == "#123456"
    assert reloaded.edits[REV_JOINT].lower == -1


def test_mimic_master_after_rename(server):
    """A mimic whose driver was renamed must still link: the UI sends the new
    name, which the server maps back to the original before validating."""
    _post(server, "/api/set_types",
          {"changes": [{"child": SCREW_LINK, "type": "revolute"}]})
    code, _ = _post(server, "/api/rename",
                    {"kind": "joint", "old": FIXED_JOINT, "new": "driver"})
    assert code == 200
    names = [j.get("name") for j in _served_urdf(server).findall("joint")]
    assert "driver" in names and FIXED_JOINT not in names

    code, r = _post(server, "/api/set_mimic",
                    {"changes": [{"child": TIP_LINK, "master": "driver",
                                  "multiplier": 1.0, "offset": 0.0}]})
    assert code == 200 and r["applied"] == [TIP_LINK]
    assert _joint(_served_urdf(server), REV_JOINT).find("mimic").get("joint") \
        == "driver"


def test_export_materializes_overlay_then_restores(server):
    """The on-disk URDF stays pristine while serving live, but the export window
    temporarily materializes the overlay so the ROS exporter picks edits up."""
    import os

    from sw2robot.editor import webserver

    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#ff0000"})
    state = webserver._um["state"]
    disk = Path(state.urdf_path)
    rel = os.path.relpath(state.urdf_path, state.package_dir).replace("\\", "/")

    assert "material" not in disk.read_text(encoding="utf-8")   # pristine on disk
    with webserver._um_materialized(state.package_dir, rel):
        assert "<material" in disk.read_text(encoding="utf-8")  # edits visible
    assert "material" not in disk.read_text(encoding="utf-8")   # restored
