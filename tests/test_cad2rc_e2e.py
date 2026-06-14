"""End-to-end regression test for the sw2robot.editor bridge.

Exercises the agreed integration flow on the committed ``feetech_hand`` cache
(``output/feetech_hand/graph.json`` + meshes) WITHOUT SolidWorks:

    import_module (pure sw2robot.exporter.build)  ->  edit (rename / limits / servo / mimic)
    ->  validate  ->  register_module  ->  export_ros_package (ROS/MoveIt/... ZIP)

The pure-build + edit half runs anywhere.  The ``register``/``export`` half uses
the vendored ``sw2robot.editor._vendor.rc_config`` package.
"""

import shutil
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import pytest

import sw2robot.editor.core as c

REPO_ROOT = Path(__file__).resolve().parent.parent
CACHE = REPO_ROOT / "output" / "feetech_hand"


def _rc_config_available() -> bool:
    try:
        from sw2robot.editor._vendor.rc_config.export import export_all_configs  # noqa: F401
        from sw2robot.editor._vendor.rc_config.urdf_parser import parse_urdf_content  # noqa: F401
    except Exception:
        return False
    return True


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
    if not _rc_config_available():
        pytest.skip("vendored rc_config not available")

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
