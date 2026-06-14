"""URDF link/joint names must never carry hyphens / spaces / dots etc.

Covers both output paths:
  * the exporter (``urdf_writer.write_urdf``) -- a hand-set ``root_link_name``
    or port name is sanitized;
  * the editor (``core._sanitize_urdf_names``) -- the final export pass cleans
    any name and rewrites every cross reference (parent/child link, mimic).
"""
import re
import xml.etree.ElementTree as ET

import numpy as np

from sw2robot.exporter.model import safe_name

_SAFE = re.compile(r"^[A-Za-z_][0-9A-Za-z_]*$")


def test_safe_name_strips_unsafe_chars():
    assert safe_name("my-part-1") == "my_part_1"
    assert safe_name("a b.c") == "a_b_c"
    assert safe_name("Part-1") == "Part_1"
    # already valid -> unchanged (idempotent)
    for ok in ("base_link", "dummy_link", "j0__j1"):
        assert safe_name(ok) == ok
        assert safe_name(safe_name(ok)) == ok
    # may not start with a digit
    assert _SAFE.match(safe_name("2link"))


def test_exporter_sanitizes_root_and_port(tmp_path):
    from sw2robot.exporter.model import Component, Joint, Port, RobotModel
    from sw2robot.exporter import urdf_writer

    comps = [
        Component(name="Base-1", link_name="base_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0),
        Component(name="Arm-1", link_name="arm_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=False, dof=1),
    ]
    joints = [Joint(name="base_link__arm_link", parent="base_link",
                    child="arm_link", jtype="revolute", axis=[0, 0, 1],
                    lower=-1, upper=1)]
    ports = [Port(name="my-root.frame", parent_link="arm_link", xyz=[0, 0, 0.1])]
    model = RobotModel(name="demo-robot", components=comps, joints=joints,
                       base_link="base_link", ports=ports,
                       root_link_name="my-root.frame")   # deliberately dirty

    out = tmp_path / "urdf" / "demo.urdf"
    urdf_writer.write_urdf(model, str(out))
    root = ET.fromstring(out.read_text(encoding="utf-8"))

    names = [e.get("name") for e in root.findall("link")] + \
            [e.get("name") for e in root.findall("joint")]
    assert names, "expected some links/joints"
    for n in names:
        assert _SAFE.match(n), f"unsafe name leaked into URDF: {n!r}"
    assert len(names) == len(set(names))
    # the dirty root_link_name became its sanitized form, and the joint's
    # parent link still resolves to it (references stay consistent)
    link_names = {e.get("name") for e in root.findall("link")}
    assert "my_root_frame" in link_names
    parent = root.find("joint").find("parent").get("link")
    assert parent in link_names


def test_editor_sanitize_pass_rewrites_references():
    from sw2robot.editor.core import _sanitize_urdf_names

    urdf = """<robot name="r">
      <link name="base-link"/>
      <link name="arm.1"/>
      <joint name="base-link__arm.1" type="revolute">
        <parent link="base-link"/>
        <child link="arm.1"/>
        <mimic joint="base-link__arm.1"/>
      </joint>
    </robot>"""
    root = ET.fromstring(urdf)
    _sanitize_urdf_names(root)

    for e in root.findall("link") + root.findall("joint"):
        assert _SAFE.match(e.get("name"))
    j = root.find("joint")
    links = {e.get("name") for e in root.findall("link")}
    assert j.find("parent").get("link") in links
    assert j.find("child").get("link") in links
    # mimic still points at an existing joint name
    assert j.find("mimic").get("joint") == j.get("name")


def test_editor_sanitize_is_noop_on_clean_names():
    from sw2robot.editor.core import _sanitize_urdf_names

    urdf = ('<robot name="r"><link name="base_link"/><link name="arm_link"/>'
            '<joint name="base_link__arm_link" type="fixed">'
            '<parent link="base_link"/><child link="arm_link"/></joint></robot>')
    root = ET.fromstring(urdf)
    _sanitize_urdf_names(root)
    assert {e.get("name") for e in root.findall("link")} == {"base_link", "arm_link"}
    assert root.find("joint").get("name") == "base_link__arm_link"


def test_servo_mapping_uses_collision_suffixed_final_joint_name(tmp_path):
    from sw2robot.editor._vendor.rc_config.urdf_parser import parse_urdf_content
    from sw2robot.editor.core import _servo_mappings, build_urdf
    from sw2robot.editor.state import JointEdit, RobotCompilerState

    urdf = """<robot name="r">
      <link name="base-link"/>
      <link name="arm.1"/>
      <link name="arm-1"/>
      <joint name="a-b" type="revolute">
        <parent link="base-link"/><child link="arm.1"/>
        <limit lower="-1" upper="1" effort="10" velocity="3.14"/>
      </joint>
      <joint name="a.b" type="revolute">
        <parent link="arm.1"/><child link="arm-1"/>
        <limit lower="-2" upper="2" effort="10" velocity="3.14"/>
      </joint>
    </robot>"""
    path = tmp_path / "r.urdf"
    path.write_text(urdf, encoding="utf-8")
    parsed_base = parse_urdf_content(urdf)
    state = RobotCompilerState(
        robot_name="r", urdf_path=str(path), package_dir=str(tmp_path),
        joints=parsed_base["joints"], links=parsed_base["links"],
        root_link=parsed_base["root_link"],
        edits={"a.b": JointEdit(servo_id=7)},
    )

    parsed_final = parse_urdf_content(build_urdf(state))
    mappings = _servo_mappings(state, parsed_final["joints"])

    assert mappings[0]["jointName"] == "a_b_2"
    assert mappings[0]["maxAngle"] == 2.0
