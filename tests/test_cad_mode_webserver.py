"""Integration tests for the CAD-mode edit endpoints that were migrated off the
full ``build()`` rebuild onto an instant in-place URDF edit (set_limits,
set_mimic) -- the same pattern flip/rename already use.  joints.yaml is still
written (persistence / re-extract / undo); the served URDF reflects the edit
without recomputing inertia.

Drives a real in-process HTTP server against a freshly-built copy of the
committed fingertip example (which HAS a graph.json, so it is a CAD package).
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

REV_JOINT = "fingertip_front_2__fingertip_back_1"
FIXED_JOINT = "fingertip_front_2__screwlock_male_hard_jointbase_v4_1"
TIP_LINK = "fingertip_back_1"
SCREW_LINK = "screwlock_male_hard_jointbase_v4_1"


def _require_fixture():
    if not (FINGERTIP / "graph.json").is_file():
        pytest.skip("missing fingertip fixture")


@pytest.fixture(scope="module")
def _built_cad(tmp_path_factory):
    """Build the fingertip CAD package ONCE (graph.json + meshes + urdf +
    joints.yaml)."""
    _require_fixture()
    from sw2robot.exporter.export import build

    pkg = tmp_path_factory.mktemp("cad")
    (pkg / "meshes").mkdir()
    shutil.copy2(FINGERTIP / "graph.json", pkg / "graph.json")
    for f in (FINGERTIP / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, pkg / "meshes" / f.name)
    build(str(pkg))
    return pkg


@pytest.fixture
def server(_built_cad, tmp_path):
    from sw2robot.editor import webserver

    pkg = tmp_path / "pkg"
    shutil.copytree(_built_cad, pkg)
    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        r = _get_json(base, f"/api/open?path={pkg}")
        assert r.get("mode") == "cad"             # CAD package (has graph.json)
        yield base, pkg
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
    return ET.fromstring(_get(base, _get_json(base, "/api/info")["urdf"]))


def _joint(root, name):
    return next(j for j in root.findall("joint") if j.get("name") == name)


def _joints_yaml(pkg):
    return (pkg / "fingertip.joints.yaml").read_text(encoding="utf-8")


def _link(root, name):
    return next(l for l in root.findall("link") if l.get("name") == name)


# --------------------------------------------------------------- set_mass_only
def test_set_mass_only_drops_geometry_in_working_urdf(server):
    """Toggling mass-only on a fixed child strips its visual/collision from the
    served URDF straight away (weight kept), and clears it back on toggle-off."""
    base, _pkg = server
    assert _link(_served_urdf(base), SCREW_LINK).find("visual") is not None

    code, r = _post(base, "/api/set_mass_only", {"link": SCREW_LINK, "on": True})
    assert code == 200 and r["applied"] is True
    scr = _link(_served_urdf(base), SCREW_LINK)
    assert scr.find("visual") is None and scr.find("collision") is None
    assert scr.find("inertial") is not None            # weight kept
    # /api/components reports it mass-only + parent_joint fixed (drives the UI)
    comp = _get_json(base, "/api/components")
    assert SCREW_LINK in comp["mass_only"]
    assert comp["links"][SCREW_LINK]["parent_joint"] == "fixed"

    code, r = _post(base, "/api/set_mass_only", {"link": SCREW_LINK, "on": False})
    assert code == 200 and r["applied"] is False
    assert _link(_served_urdf(base), SCREW_LINK).find("visual") is not None


# --------------------------------------------------------------- set_limits
def test_set_limits_reflected_without_rebuild(server):
    base, pkg = server
    # capture the link inertia before, to confirm the edit did NOT recompute it
    tip_inertia = next(l.find("inertial/inertia").attrib
                       for l in _served_urdf(base).findall("link")
                       if l.get("name") == TIP_LINK)

    code, r = _post(base, "/api/set_limits",
                    {"limits": [{"child": TIP_LINK, "lower": -0.3,
                                 "upper": 0.7, "continuous": False}]})
    assert code == 200 and r["applied"] == [TIP_LINK]

    lim = _joint(_served_urdf(base), REV_JOINT).find("limit")
    assert float(lim.get("lower")) == -0.3 and float(lim.get("upper")) == 0.7
    # inertia untouched -> the slow build() rebuild was skipped
    after_inertia = next(l.find("inertial/inertia").attrib
                         for l in _served_urdf(base).findall("link")
                         if l.get("name") == TIP_LINK)
    assert after_inertia == tip_inertia
    # joints.yaml persisted the limit (survives re-extract / undo)
    assert "-0.30000" in _joints_yaml(pkg) and "0.70000" in _joints_yaml(pkg)


def test_set_limits_then_undo(server):
    """Undo still works across the new instant edit: it restores joints.yaml and
    rebuilds, so the original limit comes back in the served URDF."""
    base, _pkg = server
    orig_lo = _joint(_served_urdf(base), REV_JOINT).find("limit").get("lower")

    _post(base, "/api/set_limits",
          {"limits": [{"child": TIP_LINK, "lower": -0.3, "upper": 0.7,
                       "continuous": False}]})
    assert float(_joint(_served_urdf(base), REV_JOINT)
                 .find("limit").get("lower")) == -0.3

    code, r = _post(base, "/api/undo", {})
    assert code == 200 and r.get("done") == "undo"
    assert _joint(_served_urdf(base), REV_JOINT).find("limit").get("lower") == orig_lo


# --------------------------------------------------------------- set_mimic
def test_mass_only_keeps_stripped_link_in_working_urdf(server):
    """CAD mode: marking a fixed child mass-only writes type: fixed + the
    mass_only: list and rebuilds.  The working URDF KEEPS the link (so it stays
    selectable in the tree) but with its geometry stripped; the export folds it."""
    base, pkg = server
    code, _ = _post(base, "/api/set_types",
                    {"changes": [{"child": SCREW_LINK, "type": "mass_only"}]})
    assert code == 200
    # reported to the front-end + persisted in joints.yaml
    assert SCREW_LINK in _get_json(base, "/api/components")["mass_only"]
    assert "mass_only:" in _joints_yaml(pkg)
    # working URDF: link present (selectable) but geometry dropped
    screw = next((l for l in _served_urdf(base).findall("link")
                  if l.get("name") == SCREW_LINK), None)
    assert screw is not None
    assert screw.find("visual") is None and screw.find("collision") is None
    # switching back to a real type restores the geometry and clears the flag
    code, _ = _post(base, "/api/set_types",
                    {"changes": [{"child": SCREW_LINK, "type": "fixed"}]})
    assert code == 200
    assert SCREW_LINK not in _get_json(base, "/api/components")["mass_only"]
    screw = next(l for l in _served_urdf(base).findall("link")
                 if l.get("name") == SCREW_LINK)
    assert screw.find("visual") is not None


def test_mass_only_folds_in_merged_view_matching_export(server):
    """CAD mode: the plain editor view keeps the mass-only link (selectable), but
    the "merge fixed" preview (?merged=1) folds it into its fixed parent -- so the
    preview matches what the ROS export produces instead of leaving a stray
    geometry-less link behind."""
    base, pkg = server
    code, _ = _post(base, "/api/set_types",
                    {"changes": [{"child": SCREW_LINK, "type": "mass_only"}]})
    assert code == 200
    # plain view: link + its fixed joint kept (so the joint row stays)
    plain = _served_urdf(base)
    assert SCREW_LINK in {l.get("name") for l in plain.findall("link")}
    assert FIXED_JOINT in {j.get("name") for j in plain.findall("joint")}
    # merged view: the mass-only link folds away (weight lumped into the parent)
    merged = ET.fromstring(_get(base, _get_json(base, "/api/info")["urdf"]
                                + "?merged=1"))
    assert SCREW_LINK not in {l.get("name") for l in merged.findall("link")}
    assert FIXED_JOINT not in {j.get("name") for j in merged.findall("joint")}


def test_collapsed_preview_urdf_endpoint_is_preview_only(server):
    base, pkg = server

    r = _get_json(base, "/api/collapsed_preview_urdf")
    assert r["ok"] is True
    assert r["preview_only"] is True
    assert r["rebuilt"] is False
    assert r["urdf"].endswith(".collapsed-preview.urdf")
    assert Path(r["path"]).is_file()
    assert Path(r["path"]).name.startswith(".")

    preview = ET.fromstring(_get(base, r["urdf"]))
    normal = _served_urdf(base)
    assert preview.get("name").endswith("_collapsed_preview")
    assert {l.get("name") for l in preview.findall("link")} == {
        l.get("name") for l in normal.findall("link")}
    assert (pkg / "urdf" / "fingertip.urdf").is_file()


def test_set_mimic_reflected_without_rebuild(server):
    base, pkg = server
    # make the fixed joint movable first (set_types still uses build() in CAD --
    # type changes can re-derive the axis), so it can drive a mimic
    code, _ = _post(base, "/api/set_types",
                    {"changes": [{"child": SCREW_LINK, "type": "revolute"}]})
    assert code == 200

    code, r = _post(base, "/api/set_mimic",
                    {"changes": [{"child": TIP_LINK, "master": FIXED_JOINT,
                                  "multiplier": 0.5, "offset": 0.1}]})
    assert code == 200 and r["applied"] == [TIP_LINK]
    mim = _joint(_served_urdf(base), REV_JOINT).find("mimic")
    assert mim is not None
    assert mim.get("joint") == FIXED_JOINT
    assert float(mim.get("multiplier")) == 0.5 and float(mim.get("offset")) == 0.1
    assert "mimic" in _joints_yaml(pkg)

    # unlink removes it again
    code, r = _post(base, "/api/set_mimic",
                    {"changes": [{"child": TIP_LINK, "clear": True}]})
    assert code == 200
    assert _joint(_served_urdf(base), REV_JOINT).find("mimic") is None
