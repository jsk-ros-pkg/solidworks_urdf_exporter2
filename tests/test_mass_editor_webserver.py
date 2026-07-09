"""CAD-mode mass-editor endpoints (Part C): /api/components mass fields,
/api/set_masses (target mass, mutually exclusive with density), the
/api/set_mass_reviewed acknowledgement, /api/set_mass_only toggle, and the
default-mass export gate.

Drives a real in-process HTTP server against a synthetic CAD package (a
graph.json with known material/density state + a joints.yaml), so the
default-mass detection is deterministic without needing SolidWorks."""
import json
import threading
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


def _eye():
    return list(np.eye(4).flatten())


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


def _link_mass(base, link):
    for l in _served_urdf(base).findall("link"):
        if l.get("name") == link:
            return float(l.find("inertial/mass").get("value"))
    raise KeyError(link)


@pytest.fixture
def server(tmp_path):
    from sw2robot.editor import webserver
    from sw2robot.exporter.export import build
    from sw2robot.exporter.state import ComponentState, GraphState

    pkg = tmp_path / "pkg"
    pkg.mkdir()
    GraphState(
        robot_name="demo", source_assembly="x.SLDASM",
        components=[
            # material assigned + non-default density -> never flagged
            ComponentState(name="base", link_name="base", world=_eye(),
                           fixed=True, material="ABS", density=1040.0,
                           sw_mass=1.0, sw_com=[0, 0, 0],
                           sw_inertia=[1, 0, 0, 1, 0, 1]),
            # no material, SW default ~1000 density -> flagged as default mass
            ComponentState(name="guess", link_name="guess", world=_eye(),
                           density=1000.0, sw_mass=0.2, sw_com=[0, 0, 0],
                           sw_inertia=[2, 0, 0, 2, 0, 2]),
        ]).save(str(pkg / "graph.json"))
    cfg = pkg / "demo.joints.yaml"
    cfg.write_text(yaml.safe_dump({
        "base": "base",
        "joints": [{"parent": "base", "child": "guess", "type": "fixed"}]}))
    build(str(pkg), config_path=str(cfg))

    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        assert _get_json(base, f"/api/open?path={pkg}")["mode"] == "cad"
        yield base, pkg
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def _yaml(pkg):
    return yaml.safe_load((pkg / "demo.joints.yaml").read_text()) or {}


# --------------------------------------------------------------- /api/components

def test_components_exposes_mass_fields_and_default_flag(server):
    base, _pkg = server
    r = _get_json(base, "/api/components")
    assert r["default_mass_links"] == ["guess"]
    # links are keyed by the FINAL display name: the root component "base" is
    # emitted as the URDF root link "base_link"; every built link is present
    base_l, guess_l = r["links"]["base_link"], r["links"]["guess"]
    # base: assigned material, has SW mass, not flagged
    assert base_l["default_mass"] is False
    assert base_l["material"] == "ABS"
    assert np.isclose(base_l["current_mass"], 1.0)
    # guess: default mass, flagged, not yet reviewed
    assert guess_l["default_mass"] is True
    assert guess_l["reviewed"] is False
    assert guess_l["mass"] is None                       # no target override yet
    assert np.isclose(guess_l["current_mass"], 0.2)      # from the built URDF


# --------------------------------------------------------------- /api/set_masses

def test_set_masses_sets_urdf_mass_and_resolves_flag(server):
    base, pkg = server
    code, r = _post(base, "/api/set_masses", {"link": "guess", "mass": 0.5})
    assert code == 200 and np.isclose(r["mass"], 0.5)
    assert np.isclose(_link_mass(base, "guess"), 0.5)    # rescaled in the URDF
    assert _yaml(pkg)["masses"]["guess"] == 0.5
    # setting a mass resolves the default-mass flag
    assert _get_json(base, "/api/components")["default_mass_links"] == []


def test_components_reflects_stored_overrides_for_repopulation(server):
    """The panel repopulates its inputs from /api/components, so the payload must
    echo back stored overrides (target mass, density) + parent_joint (which gates
    the mass-only checkbox)."""
    base, _pkg = server
    _post(base, "/api/set_masses", {"link": "guess", "mass": 0.5})
    _post(base, "/api/set_material", {"link": "base_link", "density": 2700})
    r = _get_json(base, "/api/components")
    g, b = r["links"]["guess"], r["links"]["base_link"]
    assert g["mass"] == 0.5            # target-mass field repopulates
    assert b["override"] == 2700       # density field repopulates
    assert g["parent_joint"] == "fixed"   # fixed child -> mass-only enabled
    assert b["parent_joint"] is None      # root link -> mass-only disabled


def test_set_masses_rejects_non_positive(server):
    base, _pkg = server
    code, r = _post(base, "/api/set_masses", {"link": "guess", "mass": 0})
    assert code == 400 and "positive" in r["error"]


def test_set_masses_clears_density_and_vice_versa(server):
    """Mass and density are mutually exclusive per link."""
    base, pkg = server
    _post(base, "/api/set_material", {"link": "guess", "density": 1500})
    assert _yaml(pkg).get("densities", {}).get("guess") == 1500
    # now set a target mass -> the density entry for the link is dropped
    _post(base, "/api/set_masses", {"link": "guess", "mass": 0.4})
    y = _yaml(pkg)
    assert "guess" not in y.get("densities", {})
    assert y["masses"]["guess"] == 0.4
    # and back the other way: setting density drops the mass entry
    _post(base, "/api/set_material", {"link": "guess", "density": 900})
    y = _yaml(pkg)
    assert "guess" not in y.get("masses", {})
    assert y["densities"]["guess"] == 900


# ----------------------------------------------------------- /api/set_mass_reviewed

def test_set_mass_reviewed_clears_flag_without_changing_mass(server):
    base, pkg = server
    before = _link_mass(base, "guess")
    code, r = _post(base, "/api/set_mass_reviewed",
                    {"link": "guess", "reviewed": True})
    assert code == 200 and r["reviewed"] is True
    assert _yaml(pkg)["mass_reviewed"] == ["guess"]
    comp = _get_json(base, "/api/components")
    assert comp["default_mass_links"] == []
    assert comp["links"]["guess"]["reviewed"] is True
    assert np.isclose(_link_mass(base, "guess"), before)   # mass untouched
    # un-review restores the flag
    _post(base, "/api/set_mass_reviewed", {"link": "guess", "reviewed": False})
    assert _get_json(base, "/api/components")["default_mass_links"] == ["guess"]


# --------------------------------------------------------------- /api/set_mass_only

def test_set_mass_only_toggles_on_fixed_child(server):
    base, pkg = server
    code, r = _post(base, "/api/set_mass_only", {"link": "guess", "on": True})
    assert code == 200 and r["applied"] is True
    assert "guess" in (_yaml(pkg).get("mass_only") or [])
    code, r = _post(base, "/api/set_mass_only", {"link": "guess", "on": False})
    assert code == 200 and r["applied"] is False


# --------------------------------------------------------------- export gate

def _get_code(base, path):
    req = urllib.request.Request(base + path)
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        body = e.read().decode()
        try:
            return e.code, json.loads(body)
        except json.JSONDecodeError:
            return e.code, {}


def test_export_start_blocked_lists_default_links(server):
    base, _pkg = server
    # the export endpoints are GET; an unresolved default-mass link -> 409
    code, r = _get_code(base, "/api/export/zip/start?ros=2&meshes=dae")
    assert code == 409
    assert r["default_mass_links"] == ["guess"]
    # acknowledging resolves it, so the gate would no longer block
    _post(base, "/api/set_mass_reviewed", {"link": "guess", "reviewed": True})
    assert _get_json(base, "/api/components")["default_mass_links"] == []


def test_export_gate_helper_blocks_and_ack_bypasses(server):
    from sw2robot.editor import webserver
    _base, pkg = server

    class Cls:
        pkg_dir = str(pkg)
        urdf_rel = "urdf/demo.urdf"

    assert webserver._export_gate(Cls, {}) is not None        # blocked
    assert webserver._export_gate(Cls, {"ack": ["1"]}) is None  # bypass
