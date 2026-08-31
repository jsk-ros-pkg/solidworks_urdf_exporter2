"""The web editor's MuJoCo export target.

The ROS and MuJoCo downloads share one start / progress / download pipeline, so
what needs guarding is that ``target=mujoco`` actually routes to the MJCF
builder and that the ROS-only query parameters do not leak into it.  Driven
against a real in-process server, as the other webserver tests are.
"""

import io
import json
import shutil
import threading
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
FINGERTIP = REPO_ROOT / "examples" / "fingertip"


def _free_port():
    import socket
    s = socket.socket()
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(scope="module")
def _built_pkg(tmp_path_factory):
    """The committed fingertip example, built once (build() dominates cost)."""
    if not (FINGERTIP / "graph.json").is_file():
        pytest.skip("missing fingertip fixture")
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
def server(_built_pkg, tmp_path):
    from sw2robot.editor import webserver

    pkg = tmp_path / "pkg"
    shutil.copytree(_built_pkg, pkg)
    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        with urllib.request.urlopen(f"{base}/api/open?path={pkg}") as r:
            assert json.loads(r.read().decode("utf-8")).get("name") == "fingertip"
        yield base
    finally:
        httpd.shutdown()
        httpd.server_close()


def _download(base, query):
    with urllib.request.urlopen(base + "/api/export/zip?" + query) as r:
        return r.headers.get("Content-Disposition", ""), r.read()


def test_mujoco_target_downloads_an_mjcf_package(server):
    disposition, data = _download(server, "target=mujoco&ack=1")
    assert 'filename="fingertip_mjcf.zip"' in disposition
    z = zipfile.ZipFile(io.BytesIO(data))
    names = z.namelist()
    assert "fingertip_mjcf/mjcf/fingertip.xml" in names
    assert "fingertip_mjcf/README.md" in names
    assert any(n.startswith("fingertip_mjcf/mjcf/assets/") for n in names)
    # no ROS manifests in a MuJoCo package
    assert not [n for n in names if n.endswith(("package.xml", "CMakeLists.txt"))]

    root = ET.fromstring(z.read("fingertip_mjcf/mjcf/fingertip.xml"))
    assert root.tag == "mujoco"
    assert root.find("keyframe") is not None


def test_fixed_base_reaches_the_builder(server):
    _disposition, data = _download(server, "target=mujoco&fixedbase=1&ack=1")
    z = zipfile.ZipFile(io.BytesIO(data))
    root = ET.fromstring(z.read("fingertip_mjcf/mjcf/fingertip.xml"))
    assert next(root.iter("freejoint"), None) is None


def test_ros_query_params_are_not_read_for_the_mujoco_target(server):
    # `meshes`/`colfmt`/`ros` mean nothing to MJCF (its assets are always binary
    # STL); passing them must not change the package or raise.
    _d1, plain = _download(server, "target=mujoco&ack=1")
    _d2, noisy = _download(server,
                           "target=mujoco&meshes=glb&colfmt=glb&ros=2&ack=1")
    assert (sorted(zipfile.ZipFile(io.BytesIO(plain)).namelist())
            == sorted(zipfile.ZipFile(io.BytesIO(noisy)).namelist()))


def test_unknown_target_is_rejected(server):
    with pytest.raises(urllib.error.HTTPError) as excinfo:
        _download(server, "target=gazebo&ack=1")
    assert excinfo.value.code == 400
    assert "gazebo" in excinfo.value.read().decode("utf-8")
