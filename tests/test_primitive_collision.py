"""collision='primitive'/'box'/'cylinder'/'sphere' replaces each <collision>'s
mesh with a fitted native URDF primitive (no mesh file), via scikit-robot's
per-mesh primitive fitting.  Skips when scikit-robot's primitive API is absent
(it is an OPTIONAL dependency, like coacd)."""
import os
import shutil
import xml.etree.ElementTree as ET

import pytest

from sw2robot.exporter import ros_export
from sw2robot.exporter.ros_export import (
    build_ros_description,
    skrobot_primitive_available,
)

_SAMPLE_MESH = os.path.join("examples", "fingertip", "meshes",
                            "fingertip_back_1.3dxml")

_needs_skrobot = pytest.mark.skipif(
    not skrobot_primitive_available(),
    reason="scikit-robot per-mesh primitive API not installed")

_PRIMS = {"box", "cylinder", "sphere"}


def _make_pkg(tmp_path, robot="fing", origin=None):
    """A one-link built package (visual + collision reference the sample mesh),
    optionally with a non-identity <collision> <origin>."""
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    tmp_path.mkdir(parents=True, exist_ok=True)
    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")
    org = f'<origin xyz="{origin}" rpy="0 0 0"/>' if origin else ""
    urdf = (
        f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
        '  <link name="base_link">\n'
        '    <visual><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></visual>\n'
        f'    <collision>{org}<geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></collision>\n'
        '  </link>\n</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


def _collision_geo_child(files, robot):
    root = ET.fromstring(
        files[f"{robot}_description/urdf/{robot}_description.urdf"].decode())
    link = root.find("link")
    cols = link.findall("collision")
    assert len(cols) == 1
    return link, cols[0]


@_needs_skrobot
def test_primitive_auto_replaces_mesh_with_shape(tmp_path):
    """collision='primitive' swaps the collision <mesh> for a fitted primitive
    (box/cylinder/sphere); NO collision mesh file ships, the visual is untouched."""
    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", collision="primitive"))

    # no collision mesh file of any kind is emitted; the visual .dae still is
    assert not any("_collision" in a for a in files)
    assert "fing_description/meshes/part.stl" not in files
    assert "fing_description/meshes/part.dae" in files

    link, col = _collision_geo_child(files, "fing")
    geo = col.find("geometry")
    assert geo.find("mesh") is None                     # mesh replaced
    shape = next(iter(geo))
    assert shape.tag in _PRIMS
    # the primitive carries a pose via <origin>
    assert col.find("origin") is not None
    # visual mesh path unchanged
    assert (link.find("visual").find(".//mesh").get("filename")
            == "package://fing_description/meshes/part.dae")


@_needs_skrobot
@pytest.mark.parametrize("mode,tag,attrs", [
    ("box", "box", ("size",)),
    ("cylinder", "cylinder", ("radius", "length")),
    ("sphere", "sphere", ("radius",)),
])
def test_forced_primitive_type(tmp_path, mode, tag, attrs):
    """Forcing box/cylinder/sphere yields exactly that URDF primitive with its
    dimension attributes populated with positive numbers."""
    pkg_dir = _make_pkg(tmp_path, robot="fp")
    files = dict(build_ros_description(pkg_dir, "fp", collision=mode))
    _link, col = _collision_geo_child(files, "fp")
    shape = col.find("geometry").find(tag)
    assert shape is not None
    for a in attrs:
        nums = [float(x) for x in shape.get(a).split()]
        assert nums and all(n > 0 for n in nums)


@_needs_skrobot
def test_primitive_origin_composes_collision_origin(tmp_path):
    """A non-identity <collision> <origin> is composed into the fitted
    primitive's pose (not dropped): shifting the collision origin by +0.05 in x
    shifts the primitive origin by the same amount."""
    base = _make_pkg(tmp_path / "id", robot="o")
    moved = _make_pkg(tmp_path / "mv", robot="o", origin="0.05 0 0")

    def _x(pkg):
        files = dict(build_ros_description(pkg, "o", collision="box"))
        _l, col = _collision_geo_child(files, "o")
        return float(col.find("origin").get("xyz").split()[0])

    assert abs((_x(moved) - _x(base)) - 0.05) < 1e-6


def test_primitive_needs_skrobot_raises(tmp_path, monkeypatch):
    """Requesting a primitive mode without scikit-robot raises a clear, actionable
    error (the optional-dependency contract, mirrored on coacd)."""
    monkeypatch.setattr(ros_export, "skrobot_primitive_available", lambda: False)
    pkg_dir = _make_pkg(tmp_path, robot="fing")
    with pytest.raises(ValueError, match="scikit-robot"):
        build_ros_description(pkg_dir, "fing", collision="primitive")
