"""The tutorial's quadruped (docs/tutorial.md) still builds into what it shows.

``examples/quadruped`` ships the extracted ``graph.json``, the edited joint
config and the meshes, so the headless build runs anywhere.  This pins the
result the tutorial prints: the names from ``link_names`` / ``joint_names``,
eight revolute joints that all turn about -Y, and the SolidWorks mass.
"""
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sw2robot.exporter.export import build

QUADRUPED = Path(__file__).resolve().parent.parent / "examples" / "quadruped"
LEGS = ("front_left", "front_right", "rear_left", "rear_right")


@pytest.fixture(scope="module")
def robot(tmp_path_factory):
    pkg = tmp_path_factory.mktemp("quadruped") / "quadruped"
    (pkg / "meshes").mkdir(parents=True)
    for name in ("graph.json", "quadruped.joints.yaml"):
        shutil.copy(QUADRUPED / name, pkg / name)
    for mesh in (QUADRUPED / "meshes").glob("*.3dxml"):
        shutil.copy(mesh, pkg / "meshes" / mesh.name)
    build(str(pkg), config_path=str(pkg / "quadruped.joints.yaml"))
    return ET.parse(pkg / "urdf" / "quadruped.urdf").getroot()


def test_links_and_joints_carry_the_tutorial_names(robot):
    links = {link.get("name") for link in robot.findall("link")}
    joints = {joint.get("name") for joint in robot.findall("joint")}
    assert links == {"base_link"} | {f"{leg}_{part}" for leg in LEGS
                                     for part in ("thigh", "shin")}
    assert joints == {f"{leg}_{part}" for leg in LEGS
                      for part in ("hip", "knee")}


def test_every_joint_is_revolute_about_minus_y(robot):
    for joint in robot.findall("joint"):
        assert joint.get("type") == "revolute"
        axis = [float(v) for v in joint.find("axis").get("xyz").split()]
        assert axis == pytest.approx([0.0, -1.0, 0.0]), joint.get("name")


def test_limits_come_from_the_config(robot):
    for joint in robot.findall("joint"):
        limit = joint.find("limit")
        bound = 1.57 if joint.get("name").endswith("_hip") else 2.5
        assert float(limit.get("lower")) == pytest.approx(-bound)
        assert float(limit.get("upper")) == pytest.approx(bound)


def test_mass_is_the_solidworks_abs_mass(robot):
    # 220x120x40 mm body with two axles + 4 thighs + 4 shins, all ABS
    mass = sum(float(m.get("value")) for m in robot.iter("mass"))
    assert mass == pytest.approx(1.2406, abs=1e-3)
