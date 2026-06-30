"""Integration tests for URDF-input editing mode in the web server.

A package with NO graph.json is a plain URDF the user opened directly; the
server then holds the core edit overlay in memory, serves build_urdf(state) for
the URDF URL, and routes every edit through the core setters.  These tests drive
a real in-process HTTP server (no browser) against a graph-less copy of the
committed fingertip example.

The CAD path is untouched and is guarded separately by the golden + e2e suites.
"""

import json
import re
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


def _get_status(base, path):
    try:
        with urllib.request.urlopen(base + path) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def _served_urdf(base):
    info = _get_json(base, "/api/info")
    return ET.fromstring(_get(base, info["urdf"]))


def _joint(root, name):
    return next(j for j in root.findall("joint") if j.get("name") == name)


def _link(root, name):
    return next(l for l in root.findall("link") if l.get("name") == name)


# --------------------------------------------------------------- tests
def test_info_reports_urdf_mode(server):
    """The frontend gates URDF-only controls (inertial editing) on info.mode."""
    assert _get_json(server, "/api/info")["mode"] == "urdf"


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


def test_set_inertial_malformed_body_is_400(server):
    """A null in the com/inertia arrays (e.g. an empty UI field) is a 400, not a
    500 -- the bad-input guard covers TypeError too."""
    code, r = _post(server, "/api/set_inertial",
                    {"link": TIP_LINK, "com": [None, 0, 0]})
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


def test_live_urdf_reflects_edits_and_is_cache_stable(server):
    """collision / auto-limits read a hidden live URDF that tracks the overlay
    but only rewrites when edits change (so their (path, mtime) cache holds)."""
    import os

    from sw2robot.editor import webserver

    state = webserver._um["state"]
    pkg = state.package_dir
    rel = os.path.relpath(state.urdf_path, pkg).replace("\\", "/")

    _post(server, "/api/set_types",
          {"changes": [{"child": SCREW_LINK, "type": "revolute"}]})
    live, _ = webserver._um_live_urdf(pkg, rel)
    assert os.path.basename(live).startswith(".") and live.endswith(".live.urdf")
    root = ET.fromstring(Path(live).read_text(encoding="utf-8"))
    assert _joint(root, FIXED_JOINT).get("type") == "revolute"

    m1 = os.path.getmtime(live)
    again, _ = webserver._um_live_urdf(pkg, rel)        # no edit -> not rewritten
    assert os.path.getmtime(again) == m1

    # the hidden live copy must never be picked as the package URDF on re-open
    _, picked = webserver._resolve_package(pkg)
    assert not os.path.basename(picked).startswith(".")


_TRI_STL = ("solid t\n facet normal 0 0 1\n  outer loop\n"
            "   vertex 0 0 0\n   vertex 0.1 0 0\n   vertex 0 0.1 0\n"
            "  endloop\n endfacet\nendsolid t\n")


def test_reexport_of_imported_package_urdf(tmp_path):
    """Import a URDF that uses package:// mesh refs, then re-export it: the ROS
    ZIP must actually CONTAIN the converted meshes and point at the NEW package
    (regression: package:// refs were dropped -> no meshes, stale refs)."""
    import io
    import zipfile

    import trimesh

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"      # dir name == the self-ref package name
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    # feetech-style meshes: a .dae visual + a .stl collision, plus a .stl-only
    # link to exercise BOTH passthrough (same format) and conversion (.stl->.dae)
    (pkg / "meshes" / "part.stl").write_text(_TRI_STL, encoding="utf-8")
    (pkg / "meshes" / "part.dae").write_bytes(
        trimesh.creation.box((0.1, 0.1, 0.1)).export(file_type="dae"))
    # refs use a DIFFERENT package name ('oldpkg') so the rewrite is observable
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot">'
        '<link name="base_link">'
        '<visual><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.dae"/></geometry></visual>'
        '<collision><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.stl"/></geometry></collision>'
        '</link>'
        '<link name="tip">'
        '<visual><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.stl"/></geometry></visual>'
        '</link>'
        '<joint name="j" type="fixed"><parent link="base_link"/>'
        '<child link="tip"/></joint>'
        '</robot>', encoding="utf-8")

    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        assert _get_json(base, f"/api/open?path={pkg}")["mode"] == "urdf"
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:300]
        zf = zipfile.ZipFile(io.BytesIO(data))
        names = zf.namelist()
        # the meshes are actually shipped: .dae visual (passthrough + the
        # .stl->.dae conversion) and the .stl collision (passthrough)
        assert "robot_description/meshes/part.dae" in names, names
        assert "robot_description/meshes/part.stl" in names, names
        # the exported URDF points at the NEW package, not the stale 'oldpkg'
        urdf = zf.read(next(n for n in names if n.endswith(".urdf"))).decode()
        assert "package://oldpkg/" not in urdf
        assert "package://robot_description/meshes/part.dae" in urdf
        assert "package://robot_description/meshes/part.stl" in urdf
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def _start(pkg):
    """Start a server on `pkg`, open it, return (base, httpd)."""
    from sw2robot.editor import webserver
    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    _get_json(base, f"/api/open?path={pkg}")
    return base, httpd


def test_reexport_nested_mesh_path(tmp_path):
    """A package:// ref into a SUBDIRECTORY (meshes/collision/part.stl) still
    resolves to its source -- not flattened to meshes/part.stl and lost."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"      # dir name == the self-ref package name
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes" / "collision").mkdir(parents=True)
    (pkg / "meshes" / "collision" / "part.stl").write_text(_TRI_STL, encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<collision><geometry>'
        '<mesh filename="package://oldpkg/meshes/collision/part.stl"/>'
        '</geometry></collision></link></robot>', encoding="utf-8")
    base, httpd = _start(pkg)
    try:
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:300]
        names = zipfile.ZipFile(io.BytesIO(data)).namelist()
        assert any(n.endswith("/meshes/part.stl") for n in names), names
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_reexport_glb_distinct_meshes_same_basename(tmp_path):
    """visual part.dae + collision part.stl exported to glb must NOT collapse to
    one part.glb -- distinct sources get distinct output names."""
    import io
    import zipfile

    import trimesh

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"      # dir name == the self-ref package name
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "part.stl").write_text(_TRI_STL, encoding="utf-8")
    (pkg / "meshes" / "part.dae").write_bytes(
        trimesh.creation.box((0.1, 0.1, 0.1)).export(file_type="dae"))
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<visual><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.dae"/></geometry></visual>'
        '<collision><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.stl"/></geometry></collision>'
        '</link></robot>', encoding="utf-8")
    base, httpd = _start(pkg)
    try:
        # uniform glb (visual + collision) -- collision format now defaults to
        # stl, so request glb collision explicitly to keep both meshes glb
        code, data = _get_status(base,
                                 "/api/export/zip?ros=1&meshes=glb&colfmt=glb")
        assert code == 200, data[:300]
        zf = zipfile.ZipFile(io.BytesIO(data))
        glbs = [n for n in zf.namelist() if n.endswith(".glb")]
        assert len(glbs) == 2, glbs            # both meshes shipped, not collapsed
        urdf = zf.read(next(n for n in zf.namelist() if n.endswith(".urdf"))).decode()
        # visual and collision reference DIFFERENT glb files
        refs = re.findall(r'filename="([^"]+\.glb)"', urdf)
        assert len(set(refs)) == 2, refs
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_reexport_keeps_external_package_refs(tmp_path):
    """A mesh from ANOTHER ROS package (not vendored here) must be left untouched
    -- export should not abort with 'no source mesh' on an external dependency."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"      # dir name == the self-ref package name
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "own.stl").write_text(_TRI_STL, encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<visual><geometry>'           # own mesh -> converted
        '<mesh filename="package://oldpkg/meshes/own.stl"/></geometry></visual>'
        '<collision><geometry>'        # external dep -> left as-is
        '<mesh filename="package://common_meshes/meshes/shared.stl"/>'
        '</geometry></collision></link></robot>', encoding="utf-8")
    base, httpd = _start(pkg)
    try:
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:300]      # did NOT abort on the external ref
        zf = zipfile.ZipFile(io.BytesIO(data))
        urdf = zf.read(next(n for n in zf.namelist()
                            if n.endswith(".urdf"))).decode()
        # own mesh repackaged; external dep untouched
        assert "package://robot_description/meshes/own.dae" in urdf
        assert "package://common_meshes/meshes/shared.stl" in urdf
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_reexport_dae_sidecar_textures(tmp_path):
    """A passthrough .dae that references a sidecar texture ships the texture in
    the exported package too (otherwise the visual breaks)."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "tex.png").write_bytes(b"\x89PNG\r\n fake-image")
    (pkg / "meshes" / "part.dae").write_text(
        '<?xml version="1.0"?>'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" '
        'version="1.4.1"><library_images><image id="t">'
        '<init_from>tex.png</init_from></image></library_images></COLLADA>',
        encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<visual><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.dae"/></geometry></visual>'
        '</link></robot>', encoding="utf-8")
    base, httpd = _start(pkg)
    try:
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:200]
        names = zipfile.ZipFile(io.BytesIO(data)).namelist()
        assert "robot_description/meshes/part.dae" in names, names
        assert "robot_description/meshes/tex.png" in names, names   # sidecar shipped
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_export_recolors_mesh_by_link_name(tmp_path):
    """A colour edit keyed by LINK name reaches the mesh converter even when the
    link name differs from the mesh basename (the URDF-input keying)."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "chassis.stl").write_text(_TRI_STL, encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(             # link != mesh basename
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<visual><geometry>'
        '<mesh filename="package://oldpkg/meshes/chassis.stl"/></geometry></visual>'
        '</link></robot>', encoding="utf-8")

    def _dae(base):
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:200]
        zf = zipfile.ZipFile(io.BytesIO(data))
        return zf.read(next(n for n in zf.namelist()
                            if n.endswith("chassis.dae")))

    base, httpd = _start(pkg)
    try:
        plain = _dae(base)
        _post(base, "/api/set_color", {"link": "base_link", "color": "#ff0000"})
        coloured = _dae(base)
        # the colour (keyed by the link name) changed the converted mesh
        assert coloured != plain
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_export_shared_mesh_distinct_per_link_colors(tmp_path):
    """Two links sharing ONE source mesh but with different colour edits export
    as two distinct coloured meshes (the cache keys on colour)."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    pkg = tmp_path / "oldpkg"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "shared.stl").write_text(_TRI_STL, encoding="utf-8")
    mesh = '<mesh filename="package://oldpkg/meshes/shared.stl"/>'
    (pkg / "urdf" / "robot.urdf").write_text(
        f'<?xml version="1.0"?><robot name="robot">'
        f'<link name="a"><visual><geometry>{mesh}</geometry></visual></link>'
        f'<link name="b"><visual><geometry>{mesh}</geometry></visual></link>'
        f'<joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>'
        f'</robot>', encoding="utf-8")
    base, httpd = _start(pkg)
    try:
        _post(base, "/api/set_color", {"link": "a", "color": "#ff0000"})
        _post(base, "/api/set_color", {"link": "b", "color": "#0000ff"})
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:200]
        zf = zipfile.ZipFile(io.BytesIO(data))
        daes = [n for n in zf.namelist() if n.endswith(".dae")]
        assert len(daes) == 2, daes                 # two distinct coloured meshes
        assert zf.read(daes[0]) != zf.read(daes[1])
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_reexport_owner_from_manifest_name(tmp_path):
    """Own meshes are vendored when package.xml <name> matches the ref, even if
    the containing FOLDER is named differently."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    pkg = tmp_path / "weird_folder_name"          # folder != package name
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "part.stl").write_text(_TRI_STL, encoding="utf-8")
    (pkg / "package.xml").write_text(
        '<?xml version="1.0"?><package format="2"><name>oldpkg</name></package>',
        encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<visual><geometry>'
        '<mesh filename="package://oldpkg/meshes/part.stl"/></geometry></visual>'
        '</link></robot>', encoding="utf-8")
    base, httpd = _start(pkg)
    try:
        code, data = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 200, data[:300]
        zf = zipfile.ZipFile(io.BytesIO(data))
        assert "robot_description/meshes/part.dae" in zf.namelist()
        urdf = zf.read(next(n for n in zf.namelist()
                            if n.endswith(".urdf"))).decode()
        assert "package://oldpkg/" not in urdf
        assert "package://robot_description/meshes/part.dae" in urdf
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_package_uris_rewritten_for_viewer(tmp_path):
    """A URDF that references meshes via package:// is served to the viewer with
    /pkg/ paths (so the browser fetches them from the package server instead of
    404-ing on http://host/<pkgname>/meshes/...)."""
    import posixpath as _pp

    from sw2robot.editor import webserver

    pkg = tmp_path / "weird_folder"    # folder name need NOT match the URI package
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "meshes" / "part.stl").write_text("solid x\nendsolid x\n", encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot"><link name="base_link">'
        '<visual><geometry>'                # own mesh exists here -> rewritten
        '<mesh filename="package://robot_description/meshes/part.stl"/>'
        '</geometry></visual>'
        '<collision><geometry>'             # not in this package -> left as-is
        '<mesh filename="package://other_pkg/meshes/absent.stl"/>'
        '</geometry></collision></link></robot>', encoding="utf-8")

    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        info = _get_json(base, f"/api/open?path={pkg}")
        assert info["mode"] == "urdf"
        served = _get(base, info["urdf"])
        # own mesh (exists here) rewritten to a relative path that resolves
        # (browser-style) to the package-served mesh; the absent external ref is
        # left as package:// (not mis-pointed to a local same-named file)
        assert "package://other_pkg/meshes/absent.stl" in served
        fns = re.findall(r'filename="([^"]+)"', served)
        own = next(f for f in fns if not f.startswith("package://"))
        assert not own.startswith("/")
        resolved = _pp.normpath(_pp.join(_pp.dirname(info["urdf"]), own))
        assert resolved == "/pkg/meshes/part.stl", resolved
        code, _ = _get_status(base, resolved)
        assert code == 200
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def _mini_urdf_pkg(parent, name, body):
    p = parent / name
    (p / "urdf").mkdir(parents=True)
    (p / "urdf" / "r.urdf").write_text(
        f'<?xml version="1.0"?><robot name="r">{body}</robot>', encoding="utf-8")
    return p


def test_um_reset_on_cad_open(tmp_path):
    """Opening a CAD package clears the URDF-mode overlay + undo history."""
    from sw2robot.editor import webserver

    urdfpkg = _mini_urdf_pkg(
        tmp_path, "u",
        '<link name="a"/><link name="b"/>'
        '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
        '</joint>')
    cadpkg = _mini_urdf_pkg(tmp_path, "c", '<link name="a"/>')
    (cadpkg / "graph.json").write_text("{}", encoding="utf-8")   # marks CAD mode
    base, httpd = _start(urdfpkg)
    try:
        _post(base, "/api/set_limits",
              {"limits": [{"child": "b", "lower": -0.5, "upper": 0.5}]})
        assert webserver._um["undo"]
        info = _get_json(base, f"/api/open?path={cadpkg}")
        assert info["mode"] == "cad"
        assert webserver._um["state"] is None and not webserver._um["undo"]
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_mimic_on_fixed_joint_is_missed(tmp_path):
    """A fixed joint can't follow a mimic -- the request is reported missed,
    not falsely applied."""
    from sw2robot.editor import webserver

    pkg = _mini_urdf_pkg(
        tmp_path, "p",
        '<link name="a"/><link name="b"/><link name="c"/>'
        '<joint name="jr" type="revolute"><parent link="a"/><child link="b"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
        '</joint>'
        '<joint name="jf" type="fixed"><parent link="a"/><child link="c"/></joint>')
    base, httpd = _start(pkg)
    try:
        code, r = _post(base, "/api/set_mimic",
                        {"changes": [{"child": "c", "master": "jr",
                                      "multiplier": 1.0, "offset": 0.0}]})
        assert code == 200 and r["applied"] == [] and r["missed"] == ["c"]
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_urdf_mode_undo_redo(tmp_path):
    """URDF-mode edits are undoable/redoable via the in-memory overlay stack."""
    from sw2robot.editor import webserver

    pkg = tmp_path / "robotpkg"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "urdf" / "robot.urdf").write_text(
        '<?xml version="1.0"?><robot name="robot">'
        '<link name="a"/><link name="b"/>'
        '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
        '</joint></robot>', encoding="utf-8")
    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"

    def _limit():
        return _joint(_served_urdf(base), "j").find("limit").get("lower")

    try:
        _get_json(base, f"/api/open?path={pkg}")
        assert _limit() == "-1"
        _post(base, "/api/set_limits",
              {"limits": [{"child": "b", "lower": -0.5, "upper": 0.5}]})
        assert float(_limit()) == -0.5
        assert _get_json(base, "/api/history")["undo"]          # has an entry
        code, r = _post(base, "/api/undo", {})
        assert code == 200 and r["done"] == "undo"
        assert _limit() == "-1"                                 # reverted
        code, r = _post(base, "/api/redo", {})
        assert code == 200 and float(_limit()) == -0.5          # reapplied
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def test_export_rejects_nonstandard_layout(_built_template, tmp_path):
    """A flat URDF package (urdf at the package root) can't be exported -- the
    ROS exporter needs urdf/<name>.urdf + meshes/; fail with a clear message."""
    from sw2robot.editor import webserver

    flat = tmp_path / "flat"
    (flat / "meshes").mkdir(parents=True)
    shutil.copy2(_built_template / "urdf" / "fingertip.urdf", flat / "fingertip.urdf")
    for f in (_built_template / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, flat / "meshes" / f.name)

    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        _get_json(base, f"/api/open?path={flat}")
        code, body = _get_status(base, "/api/export/zip?ros=1&meshes=dae")
        assert code == 400
        assert b"layout" in body.lower()
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


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


def test_export_zip_bakes_colour_and_restores_pristine(server):
    """Full parity check: in URDF-input mode a colour edit + ROS ZIP export
    produces a package whose URDF carries the colour, and the on-disk URDF is
    left pristine afterwards (export only materializes the overlay transiently)."""
    import io
    import zipfile

    from sw2robot.editor import webserver

    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#ff0000"})
    code, data = _get_status(server, "/api/export/zip?ros=1&meshes=dae")
    assert code == 200, data[:200]
    zf = zipfile.ZipFile(io.BytesIO(data))
    urdf_name = next(n for n in zf.namelist() if n.endswith(".urdf"))
    exported = zf.read(urdf_name).decode("utf-8")
    assert 'rgba="1 0 0 1"' in exported          # colour baked into the URDF
    assert any(n.endswith(".dae") for n in zf.namelist())   # visual meshes exported

    # the on-disk URDF stays pristine (no <material> colour element); edits live
    # in the overlay.  (The provenance "<!-- sw2robot material=... -->" comment
    # legitimately contains the word "material", so match the element, not it.)
    disk = Path(webserver._um["state"].urdf_path).read_text(encoding="utf-8")
    assert "<material" not in disk


def test_export_zip_rejects_bad_formats(server):
    """Invalid visual/collision mesh formats return a clean 400 (not a 500) -- the
    error message names the offending value."""
    code, data = _get_status(server, "/api/export/zip?ros=1&meshes=foo")
    assert code == 400 and b"foo" in data
    code, data = _get_status(server, "/api/export/zip?ros=1&colfmt=bar")
    assert code == 400 and b"bar" in data


def test_export_zip_stl_visual_emits_material(server):
    """An STL visual export (via the server) ships .stl visual meshes and carries
    the per-link colour as a URDF <material> (STL has no colour of its own)."""
    import io
    import zipfile

    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#ff0000"})
    code, data = _get_status(server,
                             "/api/export/zip?ros=1&meshes=stl&colfmt=stl")
    assert code == 200, data[:200]
    zf = zipfile.ZipFile(io.BytesIO(data))
    names = zf.namelist()
    assert any("/meshes/" in n and n.endswith(".stl") for n in names)
    assert not any(n.endswith((".dae", ".glb")) for n in names)
    exported = zf.read(next(n for n in names if n.endswith(".urdf"))).decode()
    assert "<material" in exported and 'rgba="1 0 0 1"' in exported


def test_export_materializes_overlay_then_restores(server):
    """The on-disk URDF stays pristine while serving live, but the export window
    temporarily materializes the overlay so the ROS exporter picks edits up."""
    import os

    from sw2robot.editor import webserver

    _post(server, "/api/set_color", {"link": TIP_LINK, "color": "#ff0000"})
    state = webserver._um["state"]
    disk = Path(state.urdf_path)
    rel = os.path.relpath(state.urdf_path, state.package_dir).replace("\\", "/")

    # match "<material" (the baked colour element), not the word "material":
    # the provenance "<!-- sw2robot material=... -->" comment carries it too.
    assert "<material" not in disk.read_text(encoding="utf-8")  # pristine on disk
    with webserver._um_materialized(state.package_dir, rel):
        assert "<material" in disk.read_text(encoding="utf-8")  # edits visible
    assert "<material" not in disk.read_text(encoding="utf-8")  # restored


def test_coacd_preview_endpoints(server, monkeypatch):
    """The observable CoACD job: /init starts it, /status streams per-link
    progress + served preview-GLB URLs, and the GLBs are fetchable.  CoACD itself
    is stubbed so the test is fast and needs no compiled wheel."""
    import os
    import time

    from sw2robot.exporter import ros_export

    links = [TIP_LINK, FIXED_JOINT.split("__")[1] if "__" in FIXED_JOINT
             else SCREW_LINK]

    def fake_preview(pkg_dir, robot_name, quality="balanced", progress=None,
                     urdf_path=None, should_cancel=None, on_start=None,
                     max_workers=None):
        pdir = os.path.join(pkg_dir, "meshes", ".coacd_cache", "preview")
        os.makedirs(pdir, exist_ok=True)
        out = {}
        for i, link in enumerate(links, 1):
            with open(os.path.join(pdir, link + ".glb"), "wb") as f:
                f.write(b"glTF-fake-" + link.encode())
            rel = f"meshes/.coacd_cache/preview/{link}.glb"
            out[link] = rel
            if progress:
                progress(i, len(links), link, rel)
        return out

    monkeypatch.setattr(ros_export, "coacd_preview_glbs", fake_preview)

    # bad quality -> 400, no job started
    code, _ = _get_status(server, "/api/collision/coacd/init?quality=ultra")
    assert code == 400

    r = _get_json(server, "/api/collision/coacd/init?quality=balanced")
    assert r.get("running") is True and r.get("quality") == "balanced"

    deadline, s = time.time() + 10, {}
    while time.time() < deadline:
        s = _get_json(server, "/api/collision/coacd/status")
        if not s.get("running"):
            break
        time.sleep(0.05)
    assert s.get("running") is False and s.get("error") is None
    assert set(s.get("parts", {})) == set(links)
    assert s.get("done") == s.get("total") == len(links)

    # every reported part URL is under /pkg/ and serves the preview GLB bytes
    url = s["parts"][links[0]]
    assert url.startswith("/pkg/meshes/.coacd_cache/preview/")
    code, body = _get_status(server, url)
    assert code == 200 and body.startswith(b"glTF-fake-")


def test_coacd_cancel_stops_at_link_boundary(server, monkeypatch):
    """/cancel stops the job at the next link boundary, leaving it not-running,
    flagged cancelled, with fewer than all links done."""
    import os
    import time

    from sw2robot.exporter import ros_export

    total = 8

    def slow_preview(pkg_dir, robot_name, quality="balanced", progress=None,
                     urdf_path=None, should_cancel=None, on_start=None,
                     max_workers=None):
        pdir = os.path.join(pkg_dir, "meshes", ".coacd_cache", "preview")
        os.makedirs(pdir, exist_ok=True)
        out = {}
        for i in range(total):
            if should_cancel and should_cancel():
                break
            link = f"l{i}"
            if progress:
                progress(i, total, link, None)
            time.sleep(0.15)               # simulate the slow per-link CoACD
            with open(os.path.join(pdir, link + ".glb"), "wb") as f:
                f.write(b"x")
            rel = f"meshes/.coacd_cache/preview/{link}.glb"
            out[link] = rel
            if progress:
                progress(i + 1, total, link, rel)
        return out

    monkeypatch.setattr(ros_export, "coacd_preview_glbs", slow_preview)

    assert _get_json(
        server, "/api/collision/coacd/init?quality=balanced")["running"] is True
    time.sleep(0.4)                          # let a couple links finish
    assert _get_json(server, "/api/collision/coacd/cancel")["cancelling"] is True

    deadline, s = time.time() + 10, {}
    while time.time() < deadline:
        s = _get_json(server, "/api/collision/coacd/status")
        if not s.get("running"):
            break
        time.sleep(0.05)
    assert s.get("running") is False
    assert s.get("cancelled") is True
    assert 0 < s.get("done") < total         # stopped early, after some progress
