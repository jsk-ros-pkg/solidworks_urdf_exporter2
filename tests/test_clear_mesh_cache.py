"""The 'drop meshes & re-extract' button: POST /api/clear_mesh_cache.

meshes/ is a cache -- it invalidates itself on a CAD edit (mtime) and on a
configuration change (meshes/cache_manifest.json) -- but there was no way to
rule it out by hand short of finding %TEMP%\\sw2robot\\output and deleting the
folder.

Invalidating must not DESTROY.  The re-extract that follows can fail --
SolidWorks busy with the user's own documents hands back an empty component
tree -- and a first version that deleted the meshes up front left the package
rendering nothing, with no way back.  So only the manifest goes: without it
every mesh is of unknown configuration and gets re-exported, while the files
stay on disk as a fallback.  graph.json (the extraction result) and
<name>.joints.yaml (the user's joint edits) are not cache and are never touched.
"""
import json
import os
import socket
import threading
import urllib.error
import urllib.request

import pytest


@pytest.fixture
def server(tmp_path):
    from sw2robot.editor import webserver

    s = socket.socket()
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    httpd, port = webserver._bind_free_port(webserver._Handler, port)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        yield f"http://127.0.0.1:{port}", webserver
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._Handler.pkg_dir = None


def _post(base, path):
    req = urllib.request.Request(base + path, data=b"", method="POST")
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def _pkg(tmp_path, source_assembly=None, configuration=None):
    """A minimal CAD package: graph.json + joints.yaml + a populated meshes/."""
    pkg = tmp_path / "pkg"
    (pkg / "meshes").mkdir(parents=True)
    graph = {"robot_name": "m", "source_assembly": str(source_assembly or ""),
             "components": [], "edges": [], "ground": []}
    if configuration is not None:
        graph["configuration"] = configuration
    (pkg / "graph.json").write_text(json.dumps(graph), encoding="utf-8")
    (pkg / "m.joints.yaml").write_text("joints: {}\n", encoding="utf-8")
    for name in ("a.3dxml", "a.3dxml.glb", "b.glb", "cache_manifest.json"):
        (pkg / "meshes" / name).write_bytes(b"x" * 100)
    return pkg


def test_invalidates_the_cache_without_destroying_it(server, tmp_path):
    base, webserver = server
    asm = tmp_path / "m.SLDASM"
    asm.write_bytes(b"stub")
    pkg = _pkg(tmp_path, source_assembly=asm, configuration="alt_mount")
    webserver._Handler.pkg_dir = str(pkg)

    code, r = _post(base, "/api/clear_mesh_cache")
    assert code == 200 and r["ok"]
    assert r["removed"] == 3 and r["bytes"] == 300      # the meshes, counted
    # ONLY the manifest is gone -- a failed re-extract must still have meshes
    assert sorted(p.name for p in (pkg / "meshes").iterdir()) == [
        "a.3dxml", "a.3dxml.glb", "b.glb"]
    # ... and the things that are NOT cache survived
    assert (pkg / "graph.json").is_file()
    assert (pkg / "m.joints.yaml").read_text(encoding="utf-8") == "joints: {}\n"
    # the client re-extracts from these two
    assert r["source_assembly"] == str(asm)
    assert r["configuration"] == "alt_mount"


def test_invalidated_meshes_are_no_longer_reusable(server, tmp_path):
    # the point of dropping the manifest: mesh.py refuses to reuse a mesh whose
    # configuration it cannot vouch for, so the re-extract rebuilds every one
    from sw2robot.exporter.mesh import _cache_holds_config, _manifest_record

    base, webserver = server
    pkg = _pkg(tmp_path)
    meshes = str(pkg / "meshes")
    _manifest_record(meshes, os.path.join(meshes, "a.3dxml"), "a.SLDPRT", "L1")
    assert _cache_holds_config(meshes, os.path.join(meshes, "a.3dxml"), "L1")
    webserver._Handler.pkg_dir = str(pkg)
    _post(base, "/api/clear_mesh_cache")
    assert not _cache_holds_config(meshes,
                                   os.path.join(meshes, "a.3dxml"), "L1")


def test_missing_source_assembly_is_reported_not_guessed(server, tmp_path):
    # source moved away: the cache still clears, but the client must not be
    # told to re-extract from a path that no longer exists
    base, webserver = server
    pkg = _pkg(tmp_path, source_assembly=tmp_path / "gone.SLDASM")
    webserver._Handler.pkg_dir = str(pkg)
    code, r = _post(base, "/api/clear_mesh_cache")
    assert code == 200 and r["removed"] == 3
    assert r["source_assembly"] is None


def test_older_package_without_a_configuration(server, tmp_path):
    base, webserver = server
    asm = tmp_path / "m.SLDASM"
    asm.write_bytes(b"stub")
    pkg = _pkg(tmp_path, source_assembly=asm)      # no "configuration" key
    webserver._Handler.pkg_dir = str(pkg)
    code, r = _post(base, "/api/clear_mesh_cache")
    assert code == 200 and r["configuration"] is None


def test_no_package_open_is_a_400(server):
    base, webserver = server
    webserver._Handler.pkg_dir = None
    code, r = _post(base, "/api/clear_mesh_cache")
    assert code == 400 and "no package open" in r["error"]


def test_refused_while_an_extraction_runs(server, tmp_path):
    # deleting meshes under a running export would race the exporter
    base, webserver = server
    pkg = _pkg(tmp_path)
    webserver._Handler.pkg_dir = str(pkg)
    with webserver._job_lock:
        webserver._job.update(running=True)
    try:
        code, r = _post(base, "/api/clear_mesh_cache")
    finally:
        with webserver._job_lock:
            webserver._job.update(running=False)
    assert code == 409 and "extraction is running" in r["error"]
    assert len(list((pkg / "meshes").iterdir())) == 4     # manifest still there


def test_empty_cache_is_not_an_error(server, tmp_path):
    base, webserver = server
    pkg = tmp_path / "bare"
    (pkg / "meshes").mkdir(parents=True)
    (pkg / "graph.json").write_text('{"robot_name": "m"}', encoding="utf-8")
    webserver._Handler.pkg_dir = str(pkg)
    code, r = _post(base, "/api/clear_mesh_cache")
    assert code == 200 and r["removed"] == 0 and r["bytes"] == 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
