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


def _make_fixed_pkg(tmp_path, robot="rb"):
    """A two-link package: parent 'a' + a fixed-joint child 'b', both meshed."""
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")
    urdf = (
        f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
        '  <link name="a"><visual><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></visual></link>\n'
        '  <joint name="a__b" type="fixed">\n'
        '    <origin xyz="0.1 0 0" rpy="0 0 0"/>\n'
        '    <parent link="a"/><child link="b"/>\n  </joint>\n'
        '  <link name="b"><visual><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></visual></link>\n'
        '</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


def test_export_merge_fixed_lumps_child_into_parent(tmp_path):
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_fixed_pkg(tmp_path, robot="rb")
    files = dict(build_ros_description(pkg_dir, "rb", merge_fixed=True))
    urdf = files["rb_description/urdf/rb_description.urdf"].decode()
    root = ET.fromstring(urdf)

    links = {ln.get("name") for ln in root.findall("link")}
    assert links == {"a"}                       # 'b' lumped into 'a'
    assert not root.findall("joint")            # the fixed joint is gone
    # 'a' now carries both meshes (its own + the moved child's)
    assert len(root.find("link").findall("visual")) == 2
    # without merge_fixed the two links + fixed joint are preserved
    plain = dict(build_ros_description(pkg_dir, "rb"))
    proot = ET.fromstring(plain["rb_description/urdf/rb_description.urdf"].decode())
    assert {ln.get("name") for ln in proot.findall("link")} == {"a", "b"}


def _make_mass_only_pkg(tmp_path, robot="mo"):
    """parent 'a' (meshed + inertial) + a fixed, geometry-less mass-only child
    'pcb', plus the ``mass_only.yaml`` sidecar build() writes for the export."""
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    import yaml
    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")
    urdf = (
        f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
        '  <link name="a">\n'
        '    <visual><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></visual>\n'
        '    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="2"/>'
        '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>\n'
        '  </link>\n'
        '  <joint name="a__pcb" type="fixed">\n'
        '    <origin xyz="0.2 0 0" rpy="0 0 0"/>\n'
        '    <parent link="a"/><child link="pcb"/>\n  </joint>\n'
        '  <link name="pcb">\n'          # mass-only: inertial ONLY, no geometry
        '    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="3"/>'
        '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>\n'
        '  </link>\n</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    (tmp_path / "mass_only.yaml").write_text(yaml.safe_dump(["pcb"]),
                                             encoding="utf-8")
    return str(tmp_path)


def test_export_folds_mass_only_link_from_sidecar(tmp_path):
    """End-to-end: build_ros_description reads mass_only.yaml and folds the named
    link into its fixed parent (the ``only=`` path, NOT a full merge_fixed) --
    the exported URDF drops the mass-only link while its weight lands on the
    parent, and every OTHER structure is left as-is."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_mass_only_pkg(tmp_path, robot="mo")
    # default merge_fixed=False: only the sidecar-named mass-only link folds
    files = dict(build_ros_description(pkg_dir, "mo"))
    root = ET.fromstring(
        files["mo_description/urdf/mo_description.urdf"].decode())

    links = {ln.get("name") for ln in root.findall("link")}
    assert "pcb" not in links                     # the mass-only link folded away
    assert links == {"a"}
    assert not root.findall("joint")              # its fixed joint is gone too
    a = root.find("link")
    # weight preserved on the parent (2 + 3 = 5)
    assert abs(float(a.find("inertial").find("mass").get("value")) - 5.0) < 1e-9
    # the fold added no geometry from the geometry-less child
    assert len(a.findall("visual")) == 1

    # sanity: WITHOUT the sidecar the fixed child would survive as a frame
    os.remove(os.path.join(pkg_dir, "mass_only.yaml"))
    plain = dict(build_ros_description(pkg_dir, "mo"))
    proot = ET.fromstring(
        plain["mo_description/urdf/mo_description.urdf"].decode())
    assert {ln.get("name") for ln in proot.findall("link")} == {"a", "pcb"}


def _make_fixed_pkg_with_collision(tmp_path, robot="rc"):
    """parent 'a' + fixed child 'b', both with visual AND collision meshes."""
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")

    def _link(name):
        m = '<mesh filename="../meshes/part.3dxml"/>'
        return (f'  <link name="{name}">'
                f'<visual><geometry>{m}</geometry></visual>'
                f'<collision><geometry>{m}</geometry></collision></link>\n')
    urdf = (f'<?xml version="1.0"?>\n<robot name="{robot}">\n' + _link("a")
            + '  <joint name="a__b" type="fixed">'
              '<origin xyz="0.1 0 0" rpy="0 0 0"/>'
              '<parent link="a"/><child link="b"/></joint>\n'
            + _link("b") + '</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


def test_merge_fixed_plus_coacd_compose(tmp_path, monkeypatch):
    """merge_fixed + collision='coacd' together: the child lumps into the parent
    FIRST, then CoACD decomposes every (now parent-owned) collision block."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter import ros_export

    monkeypatch.setattr(ros_export, "coacd_available", lambda: True)
    monkeypatch.setattr(ros_export, "_run_coacd",
                        lambda v, f, params: _two_unit_boxes())

    pkg_dir = _make_fixed_pkg_with_collision(tmp_path, robot="rc")
    files = dict(ros_export.build_ros_description(
        pkg_dir, "rc", merge_fixed=True, collision="coacd"))

    root = ET.fromstring(
        files["rc_description/urdf/rc_description.urdf"].decode())
    assert {ln.get("name") for ln in root.findall("link")} == {"a"}   # merged
    a = root.find("link")
    # a held 2 collision blocks after the merge (its own + b's); CoACD split each
    # into 2 convex parts -> 4 collision blocks, all pointing at coacd part STLs
    cols = a.findall("collision")
    assert len(cols) == 4
    assert all("_collision_" in c.find(".//mesh").get("filename") for c in cols)
    assert len(a.findall("visual")) == 2                 # both visuals lumped in


def test_collision_hull_single_convex_part(tmp_path):
    """collision='hull' replaces the link's collision mesh with ONE convex-hull
    STL (no optional dep, no CoACD), leaving the visual mesh untouched."""
    import xml.etree.ElementTree as ET

    import trimesh

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", collision="hull"))
    arcs = set(files)

    # the single hull STL ships; the per-link copy STL does NOT
    hull_arc = "fing_description/meshes/part_collision_hull.stl"
    assert hull_arc in arcs
    assert "fing_description/meshes/part.stl" not in arcs
    assert "fing_description/meshes/part.dae" in arcs            # visual untouched

    root = ET.fromstring(files["fing_description/urdf/fing_description.urdf"].decode())
    link = root.find("link")
    cols = link.findall("collision")
    assert len(cols) == 1                                        # one hull, not N
    assert (cols[0].find(".//mesh").get("filename")
            == "package://fing_description/meshes/part_collision_hull.stl")
    assert (link.find("visual").find(".//mesh").get("filename")
            == "package://fing_description/meshes/part.dae")

    # the emitted STL is genuinely a convex hull
    hull = trimesh.load(io.BytesIO(files[hull_arc]), file_type="stl")
    assert hull.is_convex
    assert len(hull.vertices) > 0


def test_collision_hull_preserves_nonidentity_origin(tmp_path):
    """A <collision> with a non-identity <origin> keeps that transform on the
    hull block: the hull STL is emitted in the SOURCE frame, so dropping the
    origin would misplace the collision volume (the same <origin> guarantee the
    CoACD path gives, but hull was never exercised with a moved origin)."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")
    urdf = (
        '<?xml version="1.0"?>\n<robot name="o">\n'
        '  <link name="base_link">\n'
        '    <visual><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></visual>\n'
        '    <collision><origin xyz="0.05 0 0" rpy="0 0 0"/><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></collision>\n'
        '  </link>\n</robot>\n')
    (tmp_path / "urdf" / "o.urdf").write_text(urdf, encoding="utf-8")

    files = dict(build_ros_description(str(tmp_path), "o", collision="hull"))
    link = ET.fromstring(
        files["o_description/urdf/o_description.urdf"].decode()).find("link")
    cols = link.findall("collision")
    assert len(cols) == 1
    assert "part_collision_hull.stl" in cols[0].find(".//mesh").get("filename")
    # the moved origin survives onto the hull block (not zeroed / not dropped)
    xyz = [float(x) for x in cols[0].find("origin").get("xyz").split()]
    assert abs(xyz[0] - 0.05) < 1e-6
    assert abs(xyz[1]) < 1e-6 and abs(xyz[2]) < 1e-6


def test_collision_hull_needs_no_coacd(tmp_path, monkeypatch):
    """hull mode is trimesh-only: it must still emit its convex hull when CoACD
    is unavailable -- this pins the PR's "no optional dependency" claim, which
    the plain hull test can't (it would pass with CoACD installed too)."""
    import trimesh

    from sw2robot.exporter import ros_export

    monkeypatch.setattr(ros_export, "coacd_available", lambda: False)
    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(ros_export.build_ros_description(
        pkg_dir, "fing", collision="hull"))
    hull_arc = "fing_description/meshes/part_collision_hull.stl"
    assert hull_arc in files
    hull = trimesh.load(io.BytesIO(files[hull_arc]), file_type="stl")
    assert hull.is_convex and len(hull.vertices) > 0


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
    # URDF inside is named after the package by default (not the assembly name)
    assert "fing_description/urdf/fing_description.urdf" in arcs
    assert "fing_description/meshes/part.dae" in arcs       # visual
    assert "fing_description/meshes/part.stl" in arcs       # collision

    urdf = files["fing_description/urdf/fing_description.urdf"].decode()
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


def test_rviz_tf_marker_scale_sized_to_model(tmp_path):
    """The generated .rviz caps the TF axis ("Marker Scale") at 1.0 and sizes it
    to the model, so the triads don't dwarf a small part."""
    import re as _re

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ros_version=2))
    rviz = files["fing_description/rviz/fing_description.rviz"].decode()

    assert "rviz_default_plugins/TF" in rviz
    m = _re.search(r"Marker Scale:\s*([0-9.]+)", rviz)
    assert m, "TF display has no Marker Scale"
    scale = float(m.group(1))
    assert 0.0 < scale <= 1.0


def test_build_ros_description_custom_mesh_dir(tmp_path):
    """``mesh_dir`` moves the emitted meshes (and repoints the URDF's
    package:// refs) to a custom package-relative directory."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", mesh_dir="urdf/mesh"))
    arcs = set(files)

    # meshes ship under the custom dir, NOT the default meshes/
    assert "fing_description/urdf/mesh/part.dae" in arcs
    assert "fing_description/urdf/mesh/part.stl" in arcs
    assert not any(a.startswith("fing_description/meshes/") for a in arcs)

    urdf = files["fing_description/urdf/fing_description.urdf"].decode()
    link = ET.fromstring(urdf).find("link")
    assert (link.find("visual").find(".//mesh").get("filename")
            == "package://fing_description/urdf/mesh/part.dae")
    assert (link.find("collision").find(".//mesh").get("filename")
            == "package://fing_description/urdf/mesh/part.stl")


def test_ros_mesh_dir_default_and_validation():
    from sw2robot.exporter.ros_export import ros_mesh_dir

    # default + blank fall back to 'meshes'
    assert ros_mesh_dir() == "meshes"
    assert ros_mesh_dir("") == "meshes"
    assert ros_mesh_dir("   ") == "meshes"
    # trims surrounding slashes, normalises back-slashes, keeps subdirs
    assert ros_mesh_dir("/urdf/mesh/") == "urdf/mesh"
    assert ros_mesh_dir("urdf\\mesh") == "urdf/mesh"
    # an escaping / absolute / malformed path is rejected
    for bad in ("../evil", "urdf/../mesh", "a//b", "me sh", "/", ".."):
        with pytest.raises(ValueError):
            ros_mesh_dir(bad)


def test_build_ros2_description_layout(tmp_path):
    """ros_version=2 emits an ament_cmake manifest + launch + rviz, on top of
    the same package:// URDF and converted meshes."""
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ros_version=2))
    arcs = set(files)

    # same description payload as ROS 1 (URDF named after the package)
    assert "fing_description/urdf/fing_description.urdf" in arcs
    assert "fing_description/meshes/part.dae" in arcs
    assert "fing_description/meshes/part.stl" in arcs
    urdf = files["fing_description/urdf/fing_description.urdf"].decode()
    assert "package://fing_description/meshes/part.dae" in urdf

    # ROS 2 specific files
    assert "fing_description/launch/display.launch.py" in arcs
    assert "fing_description/rviz/fing_description.rviz" in arcs

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
    assert '"fing_description.urdf"' in launch     # urdf named after the package
    assert '"fing_description.rviz"' in launch
    # the launch python must be syntactically valid
    compile(launch, "display.launch.py", "exec")

    rviz = files["fing_description/rviz/fing_description.rviz"].decode()
    assert "RobotModel" in rviz
    assert "Fixed Frame: base_link" in rviz   # the urdf's first link


def test_invalid_ros_version_rejected(tmp_path):
    import pytest

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="r")
    with pytest.raises(ValueError, match="ros_version"):
        build_ros_description(pkg_dir, "r", ros_version=3)


_CLOSURES = {
    "closures": [{"link_a": "c", "link_b": "d",
                  "point": [0.01, 0.02, 0.03], "axis": [0.0, 0.0, 1.0]}],
    "dependent": ["c__d", "e__f"],
    "independent": ["a__b"],
}


def test_ros2_loop_closures_ship_ik_relay_and_config(tmp_path):
    # a detected closed loop ships the skrobot IK relay + closure config + a
    # launch wired GUI -> relay -> robot_state_publisher so the loop tracks
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fb")
    files = dict(build_ros_description(pkg_dir, "fb", ros_version=2,
                                       loop_closures=_CLOSURES))
    arcs = set(files)
    assert "fb_description/config/loop_closures.yaml" in arcs
    assert "fb_description/scripts/loop_closure_relay.py" in arcs

    import yaml
    parsed = yaml.safe_load(files["fb_description/config/loop_closures.yaml"])
    assert parsed["dependent"] == ["c__d", "e__f"]
    assert parsed["independent"] == ["a__b"]
    assert parsed["closures"][0]["link_a"] == "c"

    # relay node is valid python, pure-numpy URDF FK + the IK loop closure
    relay = files["fb_description/scripts/loop_closure_relay.py"].decode()
    compile(relay, "loop_closure_relay.py", "exec")
    assert "joint_states_source" in relay and "_resid" in relay
    assert "skrobot" not in relay              # no pip-only dependency

    launch = files["fb_description/launch/display.launch.py"].decode()
    compile(launch, "display.launch.py", "exec")
    assert '("joint_states", "joint_states_source")' in launch
    assert "loop_closure_relay.py" in launch

    cmake = files["fb_description/CMakeLists.txt"].decode()
    assert "config" in cmake and "scripts/loop_closure_relay.py" in cmake
    pxml = files["fb_description/package.xml"].decode()
    assert "python3-numpy" in pxml and "<exec_depend>rclpy</exec_depend>" in pxml
    assert "scikit-robot" not in pxml          # only rosdep-resolvable deps


def test_ros2_loop_closures_autoload_from_sidecar(tmp_path):
    # the editor's ZIP export calls build_ros_description directly (no model);
    # a loop_closures.yaml build() left beside the package must still ship
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="sc")
    import yaml
    (tmp_path / "loop_closures.yaml").write_text(
        yaml.safe_dump(_CLOSURES), encoding="utf-8")
    # note: NO loop_closures kwarg -- must be picked up from the sidecar
    files = dict(build_ros_description(pkg_dir, "sc", ros_version=2))
    assert "sc_description/scripts/loop_closure_relay.py" in files
    cfg = files["sc_description/config/loop_closures.yaml"].decode()
    assert "c__d" in cfg and "a__b" in cfg


def test_dae_pure_black_lifted_to_dark_grey(tmp_path):
    # a pure-black COLLADA material renders as a RED fallback in RViz2/Ogre;
    # _collada_meshes must lift near-black faces to a dark grey (>=12/255)
    import numpy as np
    import trimesh

    from sw2robot.exporter.ros_export import _collada_meshes

    box = trimesh.creation.box(extents=(1, 1, 1))
    # half the faces pure black, half a real colour (blue) -> two materials
    fc = np.tile([0, 0, 0, 255], (len(box.faces), 1)).astype(np.uint8)
    fc[: len(box.faces) // 2] = [0, 0, 200, 255]
    box.visual.face_colors = fc
    parts = _collada_meshes(box)
    allc = np.vstack([p.visual.face_colors for p in parts])
    # no face is left pure black (RGB all < 12); the blue is untouched
    assert not ((allc[:, :3] < 12).all(axis=1)).any()
    assert (allc[:, 2] == 200).any()           # the blue survives


def test_ros2_without_couplings_has_no_relay(tmp_path):
    # the plain ROS 2 package (no loop) keeps the simple GUI -> RSP launch and
    # ships none of the coupling machinery
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="pl")
    files = dict(build_ros_description(pkg_dir, "pl", ros_version=2))
    arcs = set(files)
    assert "pl_description/config/loop_closures.yaml" not in arcs
    assert "pl_description/scripts/loop_closure_relay.py" not in arcs
    launch = files["pl_description/launch/display.launch.py"].decode()
    assert "joint_states_source" not in launch
    pxml = files["pl_description/package.xml"].decode()
    assert "rclpy" not in pxml


def test_glb_ctx_exports_uniform_glb(tmp_path):
    import io
    import xml.etree.ElementTree as ET

    import trimesh

    from sw2robot.exporter.ros_export import GLB_CTX_FMT, build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="g")
    files = dict(build_ros_description(pkg_dir, "g", ctx_fmt=GLB_CTX_FMT))

    assert "g_description/meshes/part.glb" in files
    assert not any(a.endswith((".dae", ".stl")) for a in files)   # uniform glb
    link = ET.fromstring(
        files["g_description/urdf/g_description.urdf"].decode()).find("link")
    for ctx in ("visual", "collision"):
        assert (link.find(ctx).find(".//mesh").get("filename")
                == "package://g_description/meshes/part.glb")
    m = trimesh.load(io.BytesIO(files["g_description/meshes/part.glb"]),
                     file_type="glb")
    m = m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m
    assert len(m.faces) > 0


def test_colour_override_repaints_visual_mesh(tmp_path):
    """A per-link colour override repaints the <visual> mesh one solid colour
    (keyed by the mesh basename), overriding the CAD colours, in both the direct
    GLB converter and the full .dae export."""
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    import numpy as np
    import trimesh

    from sw2robot.exporter.ros_export import (
        _hex_to_rgba,
        _mesh_to_glb_bytes,
        build_ros_description,
    )

    assert (_hex_to_rgba("#1188ff") == np.array([0x11, 0x88, 0xFF, 255])).all()
    assert _hex_to_rgba("#abc") is None and _hex_to_rgba(None) is None

    # direct GLB converter: every vertex/face wears the solid override colour
    glb = _mesh_to_glb_bytes(_SAMPLE_MESH, color="#1188ff")
    m = trimesh.load(io.BytesIO(glb), file_type="glb")
    m = m.dump(concatenate=True) if isinstance(m, trimesh.Scene) else m
    vis = m.visual.to_color() if m.visual.kind == "texture" else m.visual
    cols = np.asarray(vis.vertex_colors)
    assert len(cols) and (cols[:, :3] == [0x11, 0x88, 0xFF]).all()

    # full export threads `colors` (keyed by the mesh basename 'part') into .dae
    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing",
                                       colors={"part": "#1188ff"}))
    dae_txt = files["fing_description/meshes/part.dae"].decode()
    triples = {tuple(round(float(x), 2) for x in c.split()[:3])
               for c in re.findall(r"<color[^>]*>([^<]+)</color>", dae_txt)}
    assert (0.07, 0.53, 1.0) in triples           # #1188ff in 0..1 floats

    # with no override the export keeps the mesh's own (non-override) colours
    plain = dict(build_ros_description(pkg_dir, "fing"))
    plain_txt = plain["fing_description/meshes/part.dae"].decode()
    plain_triples = {tuple(round(float(x), 2) for x in c.split()[:3])
                     for c in re.findall(r"<color[^>]*>([^<]+)</color>",
                                         plain_txt)}
    assert (0.07, 0.53, 1.0) not in plain_triples


_STL_CTX = (("visual", "stl"), ("collision", "stl"))


def test_stl_visual_emits_material_from_override(tmp_path):
    """An STL <visual> carries no colour, so the per-link override is emitted as a
    URDF <material><color> (and the collision is stl too)."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ctx_fmt=_STL_CTX,
                                       colors={"part": "#1188ff"}))
    # both contexts are stl
    assert "fing_description/meshes/part.stl" in files
    assert not any(a.endswith((".dae", ".glb")) for a in files)

    link = ET.fromstring(
        files["fing_description/urdf/fing_description.urdf"].decode()).find("link")
    mat = link.find("visual").find("material")
    assert mat is not None
    rgba = [round(float(x), 2) for x in mat.find("color").get("rgba").split()]
    assert rgba[:3] == [0.07, 0.53, 1.0]                  # #1188ff
    assert link.find("collision").find("material") is None   # collision: no colour


def test_stl_visual_material_default_when_no_override(tmp_path):
    """With no override, the STL <material> uses the default colour (so the link
    isn't left RViz-default grey)."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import (
        _DEFAULT_VISUAL_RGBA,
        _rgba_attr,
        build_ros_description,
    )

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ctx_fmt=_STL_CTX))
    link = ET.fromstring(
        files["fing_description/urdf/fing_description.urdf"].decode()).find("link")
    mat = link.find("visual").find("material")
    assert mat is not None
    assert mat.find("color").get("rgba") == _rgba_attr(_DEFAULT_VISUAL_RGBA)


def test_dae_visual_emits_no_material(tmp_path):
    """The default dae visual keeps colour in the mesh -> no URDF <material>."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing"))   # default dae/stl
    link = ET.fromstring(
        files["fing_description/urdf/fing_description.urdf"].decode()).find("link")
    assert link.find("visual").find("material") is None


def _two_unit_boxes():
    """Two trivial convex parts (unit cubes), as CoACD returns ``(verts, faces)``
    pairs -- a stand-in for a real decomposition so tests stay fast + CoACD-free."""
    import trimesh

    a = trimesh.creation.box(extents=(1, 1, 1))
    b = trimesh.creation.box(extents=(1, 1, 1))
    b.apply_translation((2, 0, 0))
    return [(a.vertices, a.faces), (b.vertices, b.faces)]


def test_coacd_collision_expands_into_convex_parts(tmp_path, monkeypatch):
    """collision='coacd' replaces the single <collision> mesh with one block per
    convex part, each pointing at its own part STL; <visual> is untouched."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter import ros_export

    # stub CoACD itself so the test is fast and needs no compiled wheel
    monkeypatch.setattr(ros_export, "coacd_available", lambda: True)
    monkeypatch.setattr(ros_export, "_run_coacd",
                        lambda v, f, params: _two_unit_boxes())

    pkg_dir = _make_pkg(tmp_path, robot="c")
    files = dict(ros_export.build_ros_description(pkg_dir, "c",
                                                  collision="coacd"))

    # two convex collision parts emitted, visual still a single .dae
    assert "c_description/meshes/part_collision_0.stl" in files
    assert "c_description/meshes/part_collision_1.stl" in files
    assert "c_description/meshes/part.dae" in files
    assert "c_description/meshes/part.stl" not in files   # no copied collision

    link = ET.fromstring(
        files["c_description/urdf/c_description.urdf"].decode()).find("link")
    cols = link.findall("collision")
    assert len(cols) == 2                                  # expanded 1 -> 2
    refs = {c.find(".//mesh").get("filename") for c in cols}
    assert refs == {"package://c_description/meshes/part_collision_0.stl",
                    "package://c_description/meshes/part_collision_1.stl"}
    # visual is left as the one .dae mesh
    assert (link.find("visual").find(".//mesh").get("filename")
            == "package://c_description/meshes/part.dae")


def test_coacd_decomposition_is_cached(tmp_path, monkeypatch):
    """The slow CoACD run is cached on disk: a second export of the same mesh
    reuses ``meshes/.coacd_cache`` instead of re-running the decomposition."""
    from sw2robot.exporter import ros_export

    calls = {"n": 0}

    def _counting_coacd(v, f, params):
        calls["n"] += 1
        return _two_unit_boxes()

    monkeypatch.setattr(ros_export, "coacd_available", lambda: True)
    monkeypatch.setattr(ros_export, "_run_coacd", _counting_coacd)

    pkg_dir = _make_pkg(tmp_path, robot="c")
    ros_export.build_ros_description(pkg_dir, "c", collision="coacd")
    ros_export.build_ros_description(pkg_dir, "c", collision="coacd")
    assert calls["n"] == 1                                 # second run hit cache
    assert os.path.isdir(os.path.join(pkg_dir, "meshes", ".coacd_cache"))


def test_coacd_missing_package_errors(tmp_path, monkeypatch):
    """Requesting collision='coacd' without the optional package fails with a
    clear, install-pointing error rather than a confusing import traceback."""
    import pytest

    from sw2robot.exporter import ros_export

    monkeypatch.setattr(ros_export, "coacd_available", lambda: False)
    pkg_dir = _make_pkg(tmp_path, robot="c")
    with pytest.raises(ValueError, match="pip install coacd"):
        ros_export.build_ros_description(pkg_dir, "c", collision="coacd")


def test_coacd_invalid_quality_rejected(tmp_path, monkeypatch):
    import pytest

    from sw2robot.exporter import ros_export

    monkeypatch.setattr(ros_export, "coacd_available", lambda: True)
    pkg_dir = _make_pkg(tmp_path, robot="c")
    with pytest.raises(ValueError, match="coacd_quality"):
        ros_export.build_ros_description(pkg_dir, "c", collision="coacd",
                                         coacd_quality="ultra")


def test_preview_warms_export_cache(tmp_path, monkeypatch):
    """Generating the preview and then exporting with collision='coacd' share the
    on-disk part cache: CoACD runs ONCE per source mesh, and the ROS export ships
    those same convex parts as <collision>."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter import ros_export

    calls = {"n": 0}

    def _counting(v, f, params):
        calls["n"] += 1
        return _two_unit_boxes()

    monkeypatch.setattr(ros_export, "coacd_available", lambda: True)
    monkeypatch.setattr(ros_export, "_run_coacd", _counting)

    pkg_dir = _make_pkg(tmp_path, robot="c")
    # 1) generate the preview (decomposes the one mesh -> 1 CoACD run)
    ros_export.collision_preview_glbs(pkg_dir, "c", quality="balanced")
    assert calls["n"] == 1
    # 2) export with collision='coacd' -- reuses the cache, no second CoACD run
    files = dict(ros_export.build_ros_description(pkg_dir, "c",
                                                  collision="coacd"))
    assert calls["n"] == 1                        # cache shared (no recompute)

    # the export's <collision> blocks point at the convex parts
    assert "c_description/meshes/part_collision_0.stl" in files
    assert "c_description/meshes/part_collision_1.stl" in files
    link = ET.fromstring(
        files["c_description/urdf/c_description.urdf"].decode()).find("link")
    assert len(link.findall("collision")) == 2


def test_collision_preview_glbs_per_link(tmp_path, monkeypatch):
    """collision_preview_glbs writes one colour-coded GLB per link with a collision
    mesh, reports progress per link, and shares the export's part cache."""
    import trimesh

    from sw2robot.exporter import ros_export

    monkeypatch.setattr(ros_export, "coacd_available", lambda: True)
    monkeypatch.setattr(ros_export, "_run_coacd",
                        lambda v, f, params: _two_unit_boxes())

    pkg_dir = _make_pkg(tmp_path, robot="c")
    seen = []
    out = ros_export.collision_preview_glbs(
        pkg_dir, "c", quality="balanced",
        progress=lambda d, t, link, rel: seen.append((d, t, link, rel)))

    assert "base_link" in out
    rel = out["base_link"]
    assert rel.startswith("meshes/.coacd_cache/preview/")
    glb_path = os.path.join(pkg_dir, *rel.split("/"))
    assert os.path.isfile(glb_path)
    # progress: one link, done == total, rel reported
    assert seen and seen[-1][0] == seen[-1][1] and seen[-1][3] == rel
    # the preview GLB is loadable and non-empty (two unit boxes merged)
    m = trimesh.load(io.BytesIO(open(glb_path, "rb").read()), file_type="glb")
    m = (m.to_geometry() if isinstance(m, trimesh.Scene)
         and hasattr(m, "to_geometry") else m)
    assert len(m.faces) > 0
    # the part cache the export also uses was populated
    assert os.path.isdir(os.path.join(pkg_dir, "meshes", ".coacd_cache"))


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
    # the source meshes are untouched -- no converted .dae/.stl leaks into the
    # working package (a hidden .mesh_cache/.coacd_cache dir is fine: it speeds a
    # re-export and is never shipped, like the CoACD cache)
    after = [n for n in os.listdir(tmp_path / "meshes") if not n.startswith(".")]
    assert sorted(after) == before


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
    robot = RobotModelFromURDF(
        urdf_file=os.path.join(desc, "urdf", "fing_description.urdf"))

    assert "base_link" in [link.name for link in robot.link_list]
    vms = robot.link_list[0].visual_mesh
    vms = vms if isinstance(vms, (list, tuple)) else [vms]
    faces = sum(len(m.faces) for m in vms if hasattr(m, "faces"))
    assert faces > 0, "package:// visual mesh did not resolve to real geometry"


def test_custom_pkg_name_renames_everything(tmp_path):
    """An explicit ``pkg_name`` renames the package dir, the manifest <name>,
    the project(), every ``package://`` URL, AND the urdf file (which defaults
    to the package name) -- the assembly name 'fing' must not leak through."""
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing",
                                       pkg_name="bambu_a1_description"))
    arcs = set(files)

    assert "bambu_a1_description/package.xml" in arcs
    # urdf defaults to the package name, not the assembly name
    assert "bambu_a1_description/urdf/bambu_a1_description.urdf" in arcs
    assert "bambu_a1_description/meshes/part.dae" in arcs
    assert not any("/fing.urdf" in a for a in arcs)
    assert not any(a.startswith("fing_description/") for a in arcs)

    urdf = files["bambu_a1_description/urdf/bambu_a1_description.urdf"].decode()
    assert "package://bambu_a1_description/meshes/part.dae" in urdf
    pxml = files["bambu_a1_description/package.xml"].decode()
    assert re.search(r"<name>\s*bambu_a1_description\s*</name>", pxml)
    cmake = files["bambu_a1_description/CMakeLists.txt"].decode()
    assert "project(bambu_a1_description)" in cmake


def test_explicit_urdf_name_overrides_package_default(tmp_path):
    """An explicit ``urdf_name`` names the urdf (and ros2 launch/rviz), while
    the package keeps its own name; an empty urdf_name falls back to the pkg."""
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ros_version=2,
                                       pkg_name="my_arm", urdf_name="bambu_a1"))
    assert "my_arm/urdf/bambu_a1.urdf" in files            # explicit stem
    assert "my_arm/rviz/bambu_a1.rviz" in files
    launch = files["my_arm/launch/display.launch.py"].decode()
    assert 'get_package_share_directory("my_arm")' in launch   # pkg unchanged
    assert '"bambu_a1.urdf"' in launch                         # urdf stem
    compile(launch, "display.launch.py", "exec")

    # a '.urdf' suffix on the input is stripped, not doubled
    files2 = dict(build_ros_description(pkg_dir, "fing",
                                        pkg_name="my_arm", urdf_name="foo.urdf"))
    assert "my_arm/urdf/foo.urdf" in files2


def test_custom_pkg_name_ros2_launch_and_rviz(tmp_path):
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="fing")
    files = dict(build_ros_description(pkg_dir, "fing", ros_version=2,
                                       pkg_name="my_arm"))
    assert "my_arm/launch/display.launch.py" in files
    assert "my_arm/urdf/my_arm.urdf" in files              # urdf = pkg by default
    launch = files["my_arm/launch/display.launch.py"].decode()
    assert 'get_package_share_directory("my_arm")' in launch
    compile(launch, "display.launch.py", "exec")


def test_invalid_pkg_name_rejected():
    import pytest

    from sw2robot.exporter.ros_export import ros_pkg_name, ros_urdf_stem

    assert ros_pkg_name("fing") == "fing_description"          # default
    # the default sanitises an assembly name with capitals/punctuation into a
    # VALID ROS package name (so the no-name export never 400s)
    assert ros_pkg_name("Assem1") == "assem1_description"
    assert ros_pkg_name("My-Robot 2") == "my_robot_2_description"
    assert ros_pkg_name("123") == "robot_123_description"      # must start alpha
    assert ros_pkg_name("fing", "robot_x2") == "robot_x2"      # valid passes
    for bad in ("Bad-Name", "2leading", "has space", "Caps", "-x"):
        with pytest.raises(ValueError, match="invalid ROS package name"):
            ros_pkg_name("fing", bad)
    # urdf stem: defaults to the package, accepts filename-ish names, rejects junk
    assert ros_urdf_stem("my_pkg") == "my_pkg"
    assert ros_urdf_stem("my_pkg", "Bambu_A1-v2") == "Bambu_A1-v2"
    for bad in ("has space", "/etc/passwd", "-x", ".hidden"):
        with pytest.raises(ValueError, match="invalid URDF name"):
            ros_urdf_stem("my_pkg", bad)


def test_write_pkg_with_custom_name_returns_that_dir(tmp_path):
    from sw2robot.exporter.ros_export import write_ros_description_package

    src = tmp_path / "src"
    src.mkdir()
    pkg_dir = _make_pkg(src, robot="fing")
    out = write_ros_description_package(pkg_dir, "fing", str(tmp_path / "out"),
                                        pkg_name="custom_desc")
    assert os.path.basename(out) == "custom_desc"
    assert os.path.exists(os.path.join(out, "package.xml"))


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


@pytest.mark.parametrize("vfmt,cfmt", [("glb", "stl"), ("dae", "glb"),
                                       ("stl", "glb")])
def test_visual_and_collision_formats_are_independent(tmp_path, vfmt, cfmt):
    """The visual and collision selectors are orthogonal: each context converts
    the same source to its OWN format.  The default (dae/stl) and uniform-glb
    cases are covered elsewhere; this pins the ASYMMETRIC combinations so a
    regression that couples the two (e.g. collision inheriting the visual fmt)
    is caught.  _make_pkg's single link references the one mesh from BOTH a
    <visual> and a <collision>, so the two outputs must be distinct files."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="mix")
    ctx_fmt = (("visual", vfmt), ("collision", cfmt))
    files = dict(build_ros_description(pkg_dir, "mix", ctx_fmt=ctx_fmt))

    # each context's <mesh> points at a file with ITS format's extension
    link = ET.fromstring(
        files["mix_description/urdf/mix_description.urdf"].decode()).find("link")
    assert link.find("visual").find(".//mesh").get("filename") \
        == f"package://mix_description/meshes/part.{vfmt}"
    assert link.find("collision").find(".//mesh").get("filename") \
        == f"package://mix_description/meshes/part.{cfmt}"

    # both output files are shipped; if vfmt==cfmt they'd share one, but here
    # the combos are asymmetric so exactly the two expected files exist
    assert f"mix_description/meshes/part.{vfmt}" in files
    assert f"mix_description/meshes/part.{cfmt}" in files
    mesh_arcs = {a for a in files if a.startswith("mix_description/meshes/")}
    assert mesh_arcs == {f"mix_description/meshes/part.{vfmt}",
                         f"mix_description/meshes/part.{cfmt}"}


def test_build_ros_description_reports_progress(tmp_path):
    """build_ros_description invokes the progress callback for the mesh-convert
    stage with monotonically advancing counts ending at the total (issue #21)."""
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="rp")
    events = []
    build_ros_description(
        pkg_dir, "rp",
        progress=lambda stage, done, total, detail: events.append(
            (stage, done, total)))

    mesh = [(d, tot) for (s, d, tot) in events if s == "meshes"]
    assert mesh, "no mesh-conversion progress reported"
    total = mesh[-1][1]
    assert total >= 1
    dones = [d for d, _ in mesh]
    assert dones == sorted(dones)             # never regresses
    assert dones[-1] == total                 # ends at 100%
    assert all(tot == total for _d, tot in mesh)


def test_build_ros_description_cancels(tmp_path):
    """should_cancel turning true aborts the build with ExportCancelled rather
    than emitting a half-built package (issue #21 export stop button)."""
    import pytest as _pytest

    from sw2robot.exporter.ros_export import (
        ExportCancelled,
        build_ros_description,
    )

    pkg_dir = _make_pkg(tmp_path, robot="rc")
    with _pytest.raises(ExportCancelled):
        build_ros_description(pkg_dir, "rc", should_cancel=lambda: True)


def test_build_ros_description_cancels_during_collision_warm(tmp_path):
    """A cancel during the parallel CoACD warm aborts promptly via
    _parallel_cancellable (issue #21 — cancel mid-step, not after the whole
    phase), rather than running the whole batch first."""
    import pytest as _pytest

    from sw2robot.exporter.ros_export import (
        ExportCancelled,
        build_ros_description,
        coacd_available,
    )

    if not coacd_available():
        _pytest.skip("coacd not installed")
    pkg_dir = _make_pkg(tmp_path, robot="rw")
    with _pytest.raises(ExportCancelled):
        build_ros_description(pkg_dir, "rw", collision="coacd",
                              should_cancel=lambda: True)
