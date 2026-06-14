"""The portable ROS export: package:// URLs + colour .dae meshes, in a
``<robot_name>_description`` package.  No SolidWorks needed -- a synthetic
package is built around one committed sample ``.3dxml`` mesh."""
import io
import os
import re
import shutil

import pytest

_SAMPLE_MESH = os.path.join("examples", "fingertip", "meshes",
                            "fingertip_back_1.3dxml")


def _make_pkg(tmp_path, robot="robot"):
    """A minimal built-package layout (urdf + one mesh) for the converter."""
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")
    urdf = (
        f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
        '  <link name="base_link">\n'
        '    <visual><geometry>'
        "<mesh filename = '../meshes/part.3dxml'/></geometry></visual>\n"
        '    <collision><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></collision>\n'
        '  </link>\n</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


def test_mesh_to_dae_scale_and_loadable():
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    import trimesh
    from sw2robot.exporter.ros_export import _mesh_to_dae_bytes

    src = trimesh.load(_SAMPLE_MESH)
    src = src.to_geometry() if hasattr(src, "to_geometry") \
        and isinstance(src, trimesh.Scene) else src
    src_m = (src.dump(concatenate=True) if isinstance(src, trimesh.Scene)
             else src).copy()
    src_m.apply_scale(0.001)                    # mm -> m, the same as the export

    data = _mesh_to_dae_bytes(_SAMPLE_MESH)
    assert data and len(data) > 1000
    dae = trimesh.load(io.BytesIO(data), file_type="dae")
    dae = dae.dump(concatenate=True) if isinstance(dae, trimesh.Scene) else dae

    src_ext = src_m.bounds[1] - src_m.bounds[0]
    dae_ext = dae.bounds[1] - dae.bounds[0]
    # round-trips in metres (3DXML mm scaled down), not 1000x off
    for a, b in zip(src_ext, dae_ext):
        assert abs(a - b) < 1e-4
    assert max(dae_ext) < 1.0                   # a fingertip part, not metres-big
    assert dae.visual is not None               # colour/material survived


def test_build_ros_description_layout(tmp_path):
    import trimesh
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing"))
    arcs = set(files)

    assert "fing_description/package.xml" in arcs
    assert "fing_description/CMakeLists.txt" in arcs
    assert "fing_description/urdf/fing.urdf" in arcs
    assert "fing_description/meshes/part.dae" in arcs

    urdf = files["fing_description/urdf/fing.urdf"].decode()
    refs = re.findall(r'filename\s*=\s*["\']([^"\']+)["\']', urdf)
    assert refs and all(
        r == "package://fing_description/meshes/part.dae" for r in refs)
    assert ".3dxml" not in urdf and "../meshes" not in urdf

    pxml = files["fing_description/package.xml"].decode()
    assert re.search(r"<name>\s*fing_description\s*</name>", pxml)
    cmake = files["fing_description/CMakeLists.txt"].decode()
    assert "project(fing_description)" in cmake

    # every emitted .dae is a real, loadable mesh
    for arc, data in files.items():
        if arc.endswith(".dae"):
            m = trimesh.load(io.BytesIO(data), file_type="dae")
            m = m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m
            assert len(m.faces) > 0


def test_working_package_is_not_modified(tmp_path):
    """The converter only READS the package -- the source urdf/mesh are untouched
    (the working URDF must stay mesh-relative for the viewer / auto-limits)."""
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="r")
    src_urdf = (tmp_path / "urdf" / "r.urdf").read_text(encoding="utf-8")
    before = sorted(os.listdir(tmp_path / "meshes"))

    build_ros_description(pkg_dir, "r")

    assert (tmp_path / "urdf" / "r.urdf").read_text(encoding="utf-8") == src_urdf
    assert '../meshes/part.3dxml' in src_urdf
    assert sorted(os.listdir(tmp_path / "meshes")) == before   # no .dae written


def test_texture_glb_colours_become_collada_materials(tmp_path):
    import numpy as np
    pytest.importorskip("PIL")
    from PIL import Image
    import trimesh
    from trimesh.visual.texture import TextureVisuals
    from sw2robot.exporter.ros_export import _mesh_to_dae_bytes

    mesh = trimesh.creation.box(extents=(1, 1, 1))
    img = Image.new("RGBA", (2, 2))
    img.putdata([(255, 0, 0, 255), (0, 255, 0, 255),
                 (0, 0, 255, 255), (255, 255, 0, 255)])
    uv = np.array([[0.25, 0.25], [0.75, 0.25],
                   [0.25, 0.75], [0.75, 0.75],
                   [0.25, 0.25], [0.75, 0.25],
                   [0.25, 0.75], [0.75, 0.75]])
    mesh.visual = TextureVisuals(uv=uv, image=img)
    src = tmp_path / "textured.glb"
    mesh.export(src, file_type="glb")

    data = _mesh_to_dae_bytes(str(src))
    txt = data.decode("utf-8")
    assert "colors-array" not in txt
    assert txt.count("<effect ") > 1
    # the texture's regions become >=2 distinct non-black material colours
    # (exact RGBA values are UV-interpolated, so don't assert literals)
    cols = re.findall(r"<color[^>]*>([^<]+)</color>", txt)
    distinct = {tuple(round(float(x), 2) for x in c.split()) for c in cols}
    nonblack = {c for c in distinct if any(v > 0.01 for v in c[:3])}
    assert len(nonblack) >= 2

    dae = trimesh.load(io.BytesIO(data), file_type="dae")
    assert isinstance(dae, trimesh.Scene)
    assert len(dae.geometry) > 1


def test_missing_mesh_aborts_instead_of_half_broken_package(tmp_path):
    from sw2robot.exporter.ros_export import build_ros_description

    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    urdf = ('<?xml version="1.0"?>\n<robot name="r">\n'
            '<link name="base"><visual><geometry>'
            '<mesh filename="../meshes/missing.3dxml"/>'
            '</geometry></visual></link>\n</robot>\n')
    (tmp_path / "urdf" / "r.urdf").write_text(urdf, encoding="utf-8")

    with pytest.raises(RuntimeError, match="no source mesh"):
        build_ros_description(str(tmp_path), "r")
    assert (tmp_path / "urdf" / "r.urdf").read_text(encoding="utf-8") == urdf
    assert sorted(os.listdir(tmp_path / "meshes")) == []
