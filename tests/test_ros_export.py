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
    assert "fing_description/meshes/part.dae" in arcs       # visual
    assert "fing_description/meshes/part.stl" in arcs       # collision

    urdf = files["fing_description/urdf/fing.urdf"].decode()
    import xml.etree.ElementTree as ET
    link = ET.fromstring(urdf).find("link")
    vis = link.find("visual").find(".//mesh").get("filename")
    col = link.find("collision").find(".//mesh").get("filename")
    assert vis == "package://fing_description/meshes/part.dae"
    assert col == "package://fing_description/meshes/part.stl"
    assert ".3dxml" not in urdf and "../meshes" not in urdf

    pxml = files["fing_description/package.xml"].decode()
    assert re.search(r"<name>\s*fing_description\s*</name>", pxml)
    cmake = files["fing_description/CMakeLists.txt"].decode()
    assert "project(fing_description)" in cmake

    # every emitted mesh is real + loadable (colour .dae visual, .stl collision)
    for arc, data in files.items():
        if arc.endswith((".dae", ".stl")):
            m = trimesh.load(io.BytesIO(data), file_type=arc.rsplit(".", 1)[1])
            m = m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m
            assert len(m.faces) > 0


def test_build_ros2_description_layout(tmp_path):
    """ros_version=2 emits an ament_cmake manifest + launch + rviz, on top of
    the same package:// URDF and converted meshes."""
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ros_version=2))
    arcs = set(files)

    # same description payload as ROS 1
    assert "fing_description/urdf/fing.urdf" in arcs
    assert "fing_description/meshes/part.dae" in arcs
    assert "fing_description/meshes/part.stl" in arcs
    urdf = files["fing_description/urdf/fing.urdf"].decode()
    assert "package://fing_description/meshes/part.dae" in urdf

    # ROS 2 specific files
    assert "fing_description/launch/display.launch.py" in arcs
    assert "fing_description/rviz/fing.rviz" in arcs

    pxml = files["fing_description/package.xml"].decode()
    assert 'format="3"' in pxml
    assert "<buildtool_depend>ament_cmake</buildtool_depend>" in pxml
    assert "<build_type>ament_cmake</build_type>" in pxml

    cmake = files["fing_description/CMakeLists.txt"].decode()
    assert "find_package(ament_cmake REQUIRED)" in cmake
    assert "ament_package()" in cmake
    assert "launch rviz" in cmake

    launch = files["fing_description/launch/display.launch.py"].decode()
    assert "robot_state_publisher" in launch
    assert 'get_package_share_directory("fing_description")' in launch
    # the launch python must be syntactically valid
    compile(launch, "display.launch.py", "exec")

    rviz = files["fing_description/rviz/fing.rviz"].decode()
    assert "RobotModel" in rviz
    assert "Fixed Frame: base_link" in rviz   # the urdf's first link


def test_invalid_ros_version_rejected(tmp_path):
    import pytest

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="r")
    with pytest.raises(ValueError, match="ros_version"):
        build_ros_description(pkg_dir, "r", ros_version=3)


def test_glb_ctx_exports_uniform_glb(tmp_path):
    import io
    import xml.etree.ElementTree as ET

    import trimesh

    from sw2robot.exporter.ros_export import GLB_CTX_FMT, build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="g")
    files = dict(build_ros_description(pkg_dir, "g", ctx_fmt=GLB_CTX_FMT))

    assert "g_description/meshes/part.glb" in files
    assert not any(a.endswith((".dae", ".stl")) for a in files)   # uniform glb
    link = ET.fromstring(files["g_description/urdf/g.urdf"].decode()).find("link")
    for ctx in ("visual", "collision"):
        assert (link.find(ctx).find(".//mesh").get("filename")
                == "package://g_description/meshes/part.glb")
    m = trimesh.load(io.BytesIO(files["g_description/meshes/part.glb"]),
                     file_type="glb")
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
    import trimesh
    from PIL import Image
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


def _write_desc(tmp_path, robot="fing", ros_version=1):
    """Write a real ``<robot>_description`` package to disk (the on-disk form ROS
    tooling sees) and return its directory."""
    from sw2robot.exporter.ros_export import write_ros_description_package

    src = tmp_path / "src"
    src.mkdir()
    pkg_dir = _make_pkg(src, robot=robot)
    return write_ros_description_package(pkg_dir, robot, str(tmp_path / "out"),
                                         ros_version=ros_version)


@pytest.mark.parametrize("ros_version, fmt, build_type",
                         [(1, 2, "catkin"), (2, 3, "ament_cmake")])
def test_package_xml_is_a_valid_ros_manifest(tmp_path, ros_version, fmt,
                                             build_type):
    """The manifest parses + validates under ``catkin_pkg`` -- the same parser
    catkin / colcon use -- not just a string match.  ``parse_package`` validates
    on read, so a malformed package.xml (bad format/build_type, illegal name or
    email, missing required tag) raises ``InvalidPackage`` and fails the test."""
    cp = pytest.importorskip("catkin_pkg.package")

    desc = _write_desc(tmp_path, "fing", ros_version)
    pkg = cp.parse_package(os.path.join(desc, "package.xml"))   # validates here

    assert pkg.name == "fing_description"
    assert pkg.package_format == fmt
    assert pkg.get_build_type() == build_type
    exec_deps = {d.name for d in pkg.exec_depends}
    assert "robot_state_publisher" in exec_deps


@pytest.mark.parametrize("ros_version", [1, 2])
def test_urdf_loads_in_skrobot_with_package_meshes_resolved(tmp_path,
                                                            ros_version):
    """The package loads as a real robot: skrobot parses the URDF into a link
    tree and resolves every ``package://`` mesh to a file with geometry.  No ROS
    environment is needed -- skrobot walks up from the urdf dir to find the
    sibling package -- so this guards URDF validity + mesh wiring in plain CI."""
    pytest.importorskip("skrobot")
    from skrobot.models.urdf import RobotModelFromURDF

    desc = _write_desc(tmp_path, "fing", ros_version)
    robot = RobotModelFromURDF(urdf_file=os.path.join(desc, "urdf", "fing.urdf"))

    assert "base_link" in [link.name for link in robot.link_list]
    vms = robot.link_list[0].visual_mesh
    vms = vms if isinstance(vms, (list, tuple)) else [vms]
    faces = sum(len(m.faces) for m in vms if hasattr(m, "faces"))
    assert faces > 0, "package:// visual mesh did not resolve to real geometry"


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
