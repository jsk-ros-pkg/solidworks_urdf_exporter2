"""The MuJoCo (MJCF) export.

Built around the same committed ``.3dxml`` sample mesh the ROS export tests use,
so no SolidWorks and no MuJoCo install are needed for the structural checks; the
one test that actually compiles the model skips when ``mujoco`` is absent.
"""
import math
import os
import shutil
import xml.etree.ElementTree as ET

import pytest

_SAMPLE_MESH = os.path.join("examples", "fingertip", "meshes",
                            "fingertip_back_1.3dxml")


def _make_legged_pkg(tmp_path, robot="rb", tip_frames=False):
    """A body with two hinge-jointed legs, each ending in a meshed foot.

    Two legs rather than one so the foot detection has to find both, and a
    revolute joint with real effort/velocity limits so the derived damping has
    something to derive from.  ``tip_frames`` hangs a geometry-less frame link
    (a CAD coordinate system) off each foot, and a second frame off the left
    one, the way a tool tip or a sensor mount comes out of the exporter.
    """
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    (tmp_path / "meshes").mkdir(parents=True)
    (tmp_path / "urdf").mkdir(parents=True)
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")

    def link(name):
        return (f'  <link name="{name}">\n'
                '    <inertial><origin xyz="0 0 0" rpy="0 0 0"/>'
                '<mass value="0.2"/>'
                '<inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0"'
                ' izz="1e-4"/></inertial>\n'
                '    <visual><geometry>'
                '<mesh filename="../meshes/part.3dxml"/></geometry></visual>\n'
                '    <collision><geometry>'
                '<mesh filename="../meshes/part.3dxml"/></geometry></collision>\n'
                '  </link>\n')

    def leg(tag, y):
        return (f'  <joint name="{tag}_hip" type="revolute">\n'
                f'    <origin xyz="0 {y} -0.05" rpy="0 0 0"/>\n'
                f'    <parent link="base_link"/><child link="{tag}_foot"/>\n'
                '    <axis xyz="0 1 0"/>\n'
                '    <limit lower="-1.5" upper="1.5" effort="3.0"'
                ' velocity="6.0"/>\n'
                '  </joint>\n' + link(f"{tag}_foot")
                + (frame(f"{tag}_tip", f"{tag}_foot", "0 0 -0.02", "0 0 1.5708")
                   if tip_frames else ""))

    def frame(name, parent, xyz, rpy):
        return (f'  <link name="{name}"/>\n'
                f'  <joint name="{name}_joint" type="fixed">\n'
                f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n'
                f'    <parent link="{parent}"/><child link="{name}"/>\n'
                '  </joint>\n')

    urdf = (f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
            + link("base_link") + leg("l", 0.05) + leg("r", -0.05)
            + (frame("l_marker", "l_tip", "0.01 0 0", "0 0 0")
               if tip_frames else "")
            + '</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


def _export(tmp_path, tip_frames=False, **kwargs):
    from sw2robot.exporter.mjcf_export import write_mjcf_package

    pkg_dir = _make_legged_pkg(tmp_path / "src", robot="rb",
                               tip_frames=tip_frames)
    out = write_mjcf_package(pkg_dir, "rb", str(tmp_path / "out"), **kwargs)
    mjcf = os.path.join(out, "mjcf", "rb.xml")
    return out, mjcf, ET.parse(mjcf).getroot()


def _named(root, tag):
    return {el.get("name") for el in root.iter(tag)}


def test_package_layout(tmp_path):
    out, mjcf, root = _export(tmp_path)
    assert os.path.basename(out) == "rb_mjcf"
    assert os.path.isfile(mjcf)
    assert os.path.isfile(os.path.join(out, "README.md"))
    assert os.listdir(os.path.join(out, "mjcf", "assets"))
    assert root.tag == "mujoco"
    assert root.get("model") == "rb"          # the assembly, not the URDF stem


def test_joint_damping_comes_from_each_joints_own_limits(tmp_path):
    # effort 3.0 / velocity 6.0 -> 0.5 N*m*s/rad, the servo's torque-speed slope
    _out, _mjcf, root = _export(tmp_path)
    damping = {j.get("name"): j.get("damping")
               for body in root.iter("body") for j in body.findall("joint")}
    assert set(damping) == {"l_hip", "r_hip"}
    assert all(float(v) == pytest.approx(0.5) for v in damping.values())


def test_each_foot_gets_a_contact_sphere_and_a_site(tmp_path):
    _out, _mjcf, root = _export(tmp_path)
    spheres = {g.get("name"): g for g in root.iter("geom")
               if g.get("type") == "sphere"}
    assert set(spheres) == {"l_foot_toe", "r_foot_toe"}
    # sized from the foot's own contact patch, so a real positive radius
    assert all(float(g.get("size")) > 0 for g in spheres.values())
    assert {"l_foot", "r_foot"} <= _named(root, "site")


def test_frame_links_become_sites_and_keep_the_feet_found(tmp_path):
    # The converter writes a frame link as a site (scikit-robot >= 0.4.12);
    # as an empty body at the leg's end it would be the leaf the foot
    # detection picks, and the foot would get no sphere.
    _out, _mjcf, root = _export(tmp_path, tip_frames=True)
    assert not {"l_tip", "r_tip", "l_marker"} & _named(root, "body")
    feet = [b for b in root.iter("body") if b.get("name") in ("l_foot", "r_foot")]
    sites = {s.get("name"): s for b in feet for s in b.findall("site")}
    assert {"l_tip", "r_tip", "l_marker"} <= set(sites)
    assert {g.get("name") for g in root.iter("geom")
            if g.get("type") == "sphere"} == {"l_foot_toe", "r_foot_toe"}

    def floats(el, key):
        return [float(v) for v in el.get(key).split()]

    # poses in the foot's frame: the tip as written, and the marker composed
    # through the tip's 90 degree yaw (+x of the tip is +y of the foot)
    assert floats(sites["l_tip"], "pos") == pytest.approx([0, 0, -0.02], abs=1e-9)
    half = 1.5708 / 2
    assert floats(sites["l_tip"], "quat") == pytest.approx(
        [math.cos(half), 0, 0, math.sin(half)], abs=1e-6)
    assert floats(sites["l_marker"], "pos") == pytest.approx(
        [0, 0.01, -0.02], abs=1e-5)


def test_a_foot_tag_taken_by_a_frame_link_is_refused(tmp_path):
    # Site names are global in MuJoCo: with the frame "l_tip" already a site,
    # a foot tagged "l_tip" would leave the task reading the frame's pose
    # instead of the touch point.
    with pytest.raises(ValueError, match="l_tip"):
        _export(tmp_path, tip_frames=True,
                foot_links={"l_foot": "l_tip", "r_foot": "r_tip"})


def test_foot_extras_can_be_added_twice(tmp_path):
    # re-running the extras on a finished model recognises its own sites
    from sw2robot.exporter.mjcf_export import add_rl_extras

    _out, mjcf, root = _export(tmp_path)
    add_rl_extras(mjcf)
    again = ET.parse(mjcf).getroot()
    assert sorted(_named(again, "site")) == sorted(_named(root, "site"))


def test_imu_site_and_the_sensors_a_task_looks_up_by_name(tmp_path):
    _out, _mjcf, root = _export(tmp_path)
    assert "imu_in_body" in _named(root, "site")
    sensors = {s.get("name") for s in root.find("sensor")}
    assert sensors == {"imu_ang_vel", "imu_lin_vel", "imu_lin_acc",
                       "root_angmom"}


def test_home_keyframe_stands_on_the_floor(tmp_path):
    _out, _mjcf, root = _export(tmp_path)
    key = root.find("keyframe").find("key")
    assert key.get("name") == "home"
    qpos = [float(v) for v in key.get("qpos").split()]
    # free joint (7) + one entry per hinge
    assert len(qpos) == 7 + 2
    assert qpos[2] > 0.0                       # lifted clear of the ground
    assert qpos[3:7] == [1.0, 0.0, 0.0, 0.0]   # identity orientation


def test_fixed_base_drops_the_legged_robot_extras(tmp_path):
    # An arm bolted to a table has no feet and no IMU, and its keyframe carries
    # no free joint.
    _out, _mjcf, root = _export(tmp_path, floating_base=False)
    assert next(root.iter("freejoint"), None) is None
    assert not [g for g in root.iter("geom") if g.get("type") == "sphere"]
    assert root.find("sensor") is None
    qpos = root.find("keyframe").find("key").get("qpos").split()
    assert len(qpos) == 2


def test_actuator_type_and_forcerange_from_the_effort_limit(tmp_path):
    _out, _mjcf, root = _export(tmp_path, actuator="motor")
    actuators = root.find("actuator")
    assert [a.tag for a in actuators] == ["motor", "motor"]
    # a motor is commanded in torque, so its ctrlrange is the effort limit
    assert all(a.get("ctrlrange") == "-3 3" for a in actuators)


@pytest.mark.parametrize("tip_frames", [False, True])
def test_mujoco_compiles_the_exported_model(tmp_path, tip_frames):
    mujoco = pytest.importorskip("mujoco")
    _out, mjcf, _root = _export(tmp_path, tip_frames=tip_frames)
    model = mujoco.MjModel.from_xml_path(mjcf)
    assert model.nu == 2                       # one actuator per hinge
    assert model.nq == 7 + 2                   # free joint + two hinges
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert not any(v != v for v in data.qpos)  # no NaN


def test_build_returns_the_same_package_in_memory(tmp_path):
    from sw2robot.exporter.mjcf_export import build_mjcf_package

    pkg_dir = _make_legged_pkg(tmp_path / "src", robot="rb")
    pkg, files = build_mjcf_package(pkg_dir, "rb")
    assert pkg == "rb_mjcf"
    names = {arc for arc, _data in files}
    assert "rb_mjcf/mjcf/rb.xml" in names
    assert "rb_mjcf/README.md" in names
    assert any(n.startswith("rb_mjcf/mjcf/assets/") and n.endswith(".stl")
               for n in names)


def test_package_and_model_names_reject_path_separators(tmp_path):
    from sw2robot.exporter.mjcf_export import mjcf_model_stem, mjcf_pkg_name

    assert mjcf_pkg_name("robot") == "robot_mjcf"
    assert mjcf_pkg_name("robot", "custom") == "custom"
    assert mjcf_model_stem("robot") == "robot"
    assert mjcf_model_stem("robot", "scene.xml") == "scene"
    for bad in ("../evil", "a/b", "a\\b", "."):
        with pytest.raises(ValueError):
            mjcf_pkg_name("robot", bad)
    for bad in ("a/b", "a\\b"):
        with pytest.raises(ValueError):
            mjcf_model_stem("robot", bad)
