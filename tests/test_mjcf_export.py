"""The MuJoCo (MJCF) export.

Built around the same committed ``.3dxml`` sample mesh the ROS export tests use,
so no SolidWorks and no MuJoCo install are needed for the structural checks; the
one test that actually compiles the model skips when ``mujoco`` is absent.
"""
import os
import shutil
import xml.etree.ElementTree as ET

import pytest

_SAMPLE_MESH = os.path.join("examples", "fingertip", "meshes",
                            "fingertip_back_1.3dxml")


def _make_legged_pkg(tmp_path, robot="rb"):
    """A body with two hinge-jointed legs, each ending in a meshed foot.

    Two legs rather than one so the foot detection has to find both, and a
    revolute joint with real effort/velocity limits so the derived damping has
    something to derive from.
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
                '  </joint>\n' + link(f"{tag}_foot"))

    urdf = (f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
            + link("base_link") + leg("l", 0.05) + leg("r", -0.05)
            + '</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


def _export(tmp_path, **kwargs):
    from sw2robot.exporter.mjcf_export import write_mjcf_package

    pkg_dir = _make_legged_pkg(tmp_path / "src", robot="rb")
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


def test_mujoco_compiles_the_exported_model(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    _out, mjcf, _root = _export(tmp_path)
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
