"""End-to-end regression test for the sw2robot.editor bridge.

Exercises the agreed integration flow on the committed ``feetech_hand`` cache
(``output/feetech_hand/graph.json`` + meshes) WITHOUT SolidWorks:

    import_module (pure sw2robot.exporter.build)  ->  edit (rename / limits / servo / mimic)
    ->  validate  ->  register_module  ->  export_ros_package (ROS/MoveIt/... ZIP)

The pure-build + edit half runs anywhere.  The ``register``/``export`` half uses
``skrobot.urdf.ros_config``.
"""

import shutil
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import pytest

import sw2robot.editor.core as c

REPO_ROOT = Path(__file__).resolve().parent.parent
CACHE = REPO_ROOT / "output" / "feetech_hand"


@pytest.fixture(scope="module")
def state(tmp_path_factory):
    if not (CACHE / "graph.json").is_file():
        pytest.skip(f"missing cached graph: {CACHE / 'graph.json'}")
    # Build into a COPY of the cache: import_module -> sw2robot.exporter.build regenerates
    # the joints.yaml template and the urdf next to graph.json, so building in
    # place would dirty the committed output/feetech_hand/ cache on every run.
    work = tmp_path_factory.mktemp("feetech_hand")
    shutil.copy2(CACHE / "graph.json", work / "graph.json")
    meshes = work / "meshes"
    meshes.mkdir()
    for f in (CACHE / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, meshes / f.name)
    return c.import_module(str(work))


def test_import_builds_module(state):
    assert state.robot_name == "feetech_hand"
    assert state.root_link == "base_link"
    assert Path(state.urdf_path).is_file()
    # feetech_hand: 20 movable joints (revolute), 33 joints total
    assert len(c.movable_names(state)) == 20


def test_edit_rename_limits_servo_mimic_and_bake(state):
    mv = c.movable_names(state)  # ORIGINAL joint names; edits key off these
    driver = mv[0]
    c.rename_joint(state, driver, "thumb_drive")
    c.set_limits(state, driver, lower=-0.5, upper=1.2)
    c.set_servo(state, driver, servo_id=1, direction=1, angle_offset=0.0)

    follower = next(n for n in mv[1:] if n != driver)
    c.set_mimic(state, follower, mimic_joint=driver, multiplier=0.5, offset=0.0)

    assert c.validate(state) == []

    # build_urdf bakes the overlay; check rename + limits + mimic-repoint.
    root = ET.fromstring(c.build_urdf(state))
    joints = {j.get("name"): j for j in root.findall("joint")}
    assert "thumb_drive" in joints
    assert driver not in joints  # old name is gone

    lim = joints["thumb_drive"].find("limit")
    assert float(lim.get("lower")) == pytest.approx(-0.5)
    assert float(lim.get("upper")) == pytest.approx(1.2)

    mim = joints[follower].find("mimic")
    assert mim is not None
    # driver was renamed -> the mimic ref must be repointed to the new name
    assert mim.get("joint") == "thumb_drive"
    assert float(mim.get("multiplier")) == pytest.approx(0.5)


def test_register_and_export(state, tmp_path):
    # state already carries the edits from the prior test within this module.
    mod_urdf = c.register_module(state, tmp_path / "registry")
    assert mod_urdf.is_file()
    assert mod_urdf.name == "feetech_hand.urdf"
    assert (mod_urdf.parent.parent / "meshes").is_dir()

    zip_path = c.export_ros_package(
        state, tmp_path / "feetech_hand_ros.zip", strict=True)
    assert zip_path.is_file()

    with zipfile.ZipFile(zip_path) as z:
        members = z.namelist()
        servo_yaml = z.read("feetech_hand/config/servo_mapping.yaml").decode()
    assert "feetech_hand/urdf/feetech_hand.urdf" in members
    assert "feetech_hand/config/servo_mapping.yaml" in members
    # servo mapping uses the EFFECTIVE (renamed) joint name + the edited limits
    assert "thumb_drive:" in servo_yaml
    assert "servo_id: 1" in servo_yaml


def test_export_ros_package_from_the_quadruped_example(tmp_path):
    """The config ZIP from a committed package (no cache needed): skrobot's
    URDF/MoveIt/Gazebo configs plus sw2robot's servo mapping and IL config.
    Only the extraction (graph.json + meshes) is committed, so the URDF is
    built here, the way the editor's headless import does it."""
    example = REPO_ROOT / "examples" / "quadruped"
    pkg = tmp_path / "quadruped"
    (pkg / "meshes").mkdir(parents=True)
    for name in ("graph.json", "quadruped.joints.yaml"):
        shutil.copy(example / name, pkg / name)
    for mesh in (example / "meshes").glob("*.3dxml"):
        shutil.copy(mesh, pkg / "meshes" / mesh.name)
    state = c.import_module(pkg, config_path=pkg / "quadruped.joints.yaml")
    hip = c.movable_names(state)[0]
    c.rename_joint(state, hip, "fl_hip")
    c.set_servo(state, hip, servo_id=3, direction=-1, angle_offset=0.0)

    zip_path = c.export_ros_package(state, tmp_path / "q.zip", strict=True)
    with zipfile.ZipFile(zip_path) as z:
        members = set(z.namelist())
        servo_yaml = z.read("quadruped/config/servo_mapping.yaml").decode()
        ros2_control = z.read("quadruped/urdf/ros2_control.xacro").decode()
    assert {"quadruped/urdf/quadruped.urdf", "quadruped/config/quadruped.srdf",
            "quadruped/config/controllers.yaml", "quadruped/config/gazebo.xml",
            "quadruped/config/il_config.yaml", "quadruped/README.md"} <= members
    assert "fl_hip:" in servo_yaml and "servo_id: 3" in servo_yaml
    # skrobot's generator: gz_ros2_control, not the end-of-life Gazebo Classic
    assert "gz_ros2_control/GazeboSimSystem" in ros2_control
    assert "libgazebo_ros2_control.so" not in ros2_control

    only_urdf = c.export_ros_package(
        state, tmp_path / "u.zip",
        export_options={"include_moveit": False, "include_gazebo": False,
                        "include_servo_mapping": False,
                        "include_imitation_learning": False})
    with zipfile.ZipFile(only_urdf) as z:
        assert set(z.namelist()) == {"quadruped/urdf/quadruped.urdf",
                                     "quadruped/README.md"}


def test_il_config_carries_the_joint_limits():
    from skrobot.urdf.ros_config import parse_urdf_content

    from sw2robot.editor.imitation_generator import generate_il_config

    joints = parse_urdf_content("""<robot name="r">
      <link name="a"/><link name="b"/>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/>
        <limit lower="-0.5" upper="1.25" effort="1" velocity="2"/></joint>
    </robot>""")["joints"]
    text = generate_il_config({}, {"selectedJoints": ["j"]}, joints)
    assert "lower: -0.5" in text
    assert "upper: 1.25" in text
    assert "velocity: 2.0" in text


def test_export_refuses_an_unknown_option(tmp_path):
    # includeMoveIt was the old spelling: ignoring it would still write MoveIt
    state = c.RobotCompilerState(robot_name="r", urdf_path="", package_dir="")
    with pytest.raises(ValueError, match="includeMoveIt"):
        c.export_ros_package(state, tmp_path / "r.zip",
                             export_options={"includeMoveIt": False})


def test_an_old_sidecar_loads_with_the_current_keys(tmp_path):
    import json

    sidecar = tmp_path / "old.sw2robot.json"
    sidecar.write_text(json.dumps({
        "robot_name": "r", "urdf_path": "", "package_dir": "",
        "joints": [{"name": "j", "type": "revolute", "parentLink": "a",
                    "childLink": "b", "lowerLimit": -1.0, "upperLimit": 1.0}],
        "links": [{"name": "a", "hasVisual": True}],
    }), encoding="utf-8")
    state = c.load_state(sidecar)
    joint = state.joints[0]
    assert (joint["parent_link"], joint["child_link"]) == ("a", "b")
    assert (joint["lower_limit"], joint["upper_limit"]) == (-1.0, 1.0)
    assert "childLink" not in joint
    assert state.links[0]["has_visual"] is True
