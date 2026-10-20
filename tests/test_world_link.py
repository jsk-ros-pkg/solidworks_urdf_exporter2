"""``world_link``: a massless ``world`` root fixed to the base link at the
base's ASSEMBLY pose, so the URDF's origin is the CAD assembly's origin.

Without it the base link's own frame becomes the URDF origin, and a rig
whose base part sits 74 mm above the CAD floor loads 74 mm into the ground
in Isaac Sim / MuJoCo (measured on the dielectric rig)."""
import xml.etree.ElementTree as ET

import numpy as np


def _model(base_world):
    from sw2robot.exporter.model import Component, Joint, RobotModel
    comps = [
        Component(name="Station-2", link_name="station_2", part_path=None,
                  is_subassembly=False, world=base_world, fixed=True, dof=0,
                  mesh_file="meshes/station.3dxml", sw_mass=0.5,
                  sw_com=[0, 0, 0], sw_inertia=[1e-3, 0, 0, 1e-3, 0, 1e-3]),
        Component(name="Rail-1", link_name="rail_1", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=False, dof=0,
                  mesh_file="meshes/rail.3dxml", sw_mass=0.3,
                  sw_com=[0, 0, 0], sw_inertia=[1e-3, 0, 0, 1e-3, 0, 1e-3]),
    ]
    joints = [Joint(name="station_2__rail_1", parent="station_2",
                    child="rail_1", jtype="fixed", axis=None,
                    xyz=[0.2, 0.0, -0.074], rpy=[0, 0, 0])]
    return RobotModel(name="rig", components=comps, joints=joints,
                      base_link="station_2")


def _pose():
    M = np.eye(4)
    M[:3, 3] = [-0.022, 0.36, 0.074]
    c, s = np.cos(np.pi / 2), np.sin(np.pi / 2)
    M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]     # yaw 90 deg
    return M


def test_world_link_fixes_the_base_at_its_assembly_pose(tmp_path):
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "rig.urdf"
    urdf_writer.write_urdf(_model(_pose()), str(out), world_link=True)
    root = ET.parse(out).getroot()
    links = [link.get("name") for link in root.findall("link")]
    assert links[0] == "world"                       # the root comes first
    assert "base_link" in links                      # the base keeps its URDF name
    world = root.find("link[@name='world']")
    assert world.find("visual") is None and world.find("inertial") is None
    joint = root.find("joint[@name='world_to_base_link']")
    assert joint is not None and joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "world"
    assert joint.find("child").get("link") == "base_link"
    xyz = [float(v) for v in joint.find("origin").get("xyz").split()]
    rpy = [float(v) for v in joint.find("origin").get("rpy").split()]
    assert np.allclose(xyz, [-0.022, 0.36, 0.074])
    assert np.allclose(rpy, [0, 0, np.pi / 2], atol=1e-6)
    # every joint still names existing links
    names = set(links)
    for j in root.findall("joint"):
        assert j.find("parent").get("link") in names
        assert j.find("child").get("link") in names


def test_without_world_link_nothing_changes(tmp_path):
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "rig.urdf"
    urdf_writer.write_urdf(_model(_pose()), str(out))
    root = ET.parse(out).getroot()
    assert root.find("link").get("name") == "base_link"
    assert root.find("link[@name='world']") is None
    assert len(root.findall("joint")) == 1
