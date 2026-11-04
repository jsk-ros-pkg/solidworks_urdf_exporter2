"""The editor's flip (``f``) survives a rebuild.

``/api/set_axis`` used to negate the joint's axis only in the served URDF, so
the next edit that rebuilds the package from graph.json + joints.yaml -- a
joint-type change, a re-extract -- put the old direction back without a
word.  It now also records ``flip_axis`` in joints.yaml and converts that
file's limits and mimic; these pin that the rebuilt URDF is the flipped one.

Runs on the committed fingertip package with the golden edit config: two
revolute joints, ``bend`` mimicking the screwlock hinge at x0.5 + 0.1.
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
EDIT_CFG = REPO_ROOT / "tests" / "golden" / "fingertip_edit.joints.yaml"
DRIVER = "fingertip_front_2__screwlock_male_hard_jointbase_v4_1"
FOLLOWER = "bend"


@pytest.fixture
def server(tmp_path):
    if not (FINGERTIP / "graph.json").is_file():
        pytest.skip("missing fingertip fixture")
    from sw2robot.editor import webserver
    from sw2robot.exporter.export import build

    pkg = tmp_path / "pkg"
    (pkg / "meshes").mkdir(parents=True)
    shutil.copy2(FINGERTIP / "graph.json", pkg / "graph.json")
    for f in (FINGERTIP / "meshes").glob("*.3dxml"):
        shutil.copy2(f, pkg / "meshes" / f.name)
    shutil.copy2(EDIT_CFG, pkg / "fingertip.joints.yaml")
    build(str(pkg), config_path=str(pkg / "fingertip.joints.yaml"))

    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        assert _get_json(base, f"/api/open?path={pkg}").get("mode") == "cad"
        yield base, pkg
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def _free_port():
    import socket
    s = socket.socket()
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _get(base, path):
    with urllib.request.urlopen(base + path) as r:
        return r.read().decode("utf-8")


def _get_json(base, path):
    return json.loads(_get(base, path))


def _post(base, path, body):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req) as r:
        return r.status, json.loads(r.read().decode("utf-8"))


def _joints(base):
    """{name: (axis, lower, upper, mimic multiplier, mimic offset)}."""
    root = ET.fromstring(_get(base, _get_json(base, "/api/info")["urdf"]))
    out = {}
    for j in root.findall("joint"):
        axis = j.find("axis")
        limit = j.find("limit")
        mimic = j.find("mimic")
        out[j.get("name")] = (
            [float(v) for v in axis.get("xyz").split()] if axis is not None
            else None,
            float(limit.get("lower")) if limit is not None else None,
            float(limit.get("upper")) if limit is not None else None,
            float(mimic.get("multiplier", 1)) if mimic is not None else None,
            float(mimic.get("offset", 0)) if mimic is not None else None)
    return out


def _rebuild(base):
    """An edit that rebuilds from joints.yaml: set a joint to its own type."""
    code, _ = _post(base, "/api/set_types", {"changes": [
        {"child": "fingertip_back_1", "type": "revolute"}]})
    assert code == 200


def _assert_same(got, want):
    for name, row in want.items():
        for g, w in zip(got[name], row):
            assert g == pytest.approx(w, abs=1e-9), (name, got[name], row)


@pytest.mark.parametrize("joint", [DRIVER, FOLLOWER])
def test_a_flip_is_still_there_after_a_rebuild(server, joint):
    base, _pkg = server
    before = _joints(base)
    code, r = _post(base, "/api/set_axis", {"joints": [joint]})
    assert code == 200 and r["applied"] == [joint]
    flipped = _joints(base)
    assert flipped[joint][0] == pytest.approx([-v for v in before[joint][0]])
    _rebuild(base)
    _assert_same(_joints(base), flipped)


def test_flipping_twice_gives_back_the_same_joints_yaml(server):
    base, pkg = server
    yml = pkg / "fingertip.joints.yaml"
    original = yml.read_text(encoding="utf-8")
    before = _joints(base)
    for _ in range(2):
        code, _r = _post(base, "/api/set_axis", {"joints": [DRIVER]})
        assert code == 200
    assert yml.read_text(encoding="utf-8") == original
    _rebuild(base)
    _assert_same(_joints(base), before)


def test_flip_records_converted_limits_and_mimic(server):
    """The yaml says what the rebuilt URDF will: the driver's travel swapped
    and negated, and its follower's multiplier negated (offset kept)."""
    import yaml

    base, pkg = server
    _post(base, "/api/set_axis", {"joints": [DRIVER]})
    cfg = yaml.safe_load((pkg / "fingertip.joints.yaml").read_text(
        encoding="utf-8"))
    by_child = {j["child"]: j for j in cfg["joints"]}
    driver = by_child["screwlock_male_hard_jointbase_v4_1"]
    follower = by_child["fingertip_back_1"]
    assert driver["flip_axis"] is True
    assert (driver["lower"], driver["upper"]) == (-0.5, 0.5)
    assert follower["mimic"]["multiplier"] == -0.5
    assert follower["mimic"]["offset"] == 0.1
    assert "flip_axis" not in follower
