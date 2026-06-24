"""Fixed-joint lumping (sw2robot.exporter.merge): geometry is moved with a
composed origin, inertials combine (mass + parallel-axis), movable joints
re-parent, and mesh-less coordinate frames are preserved."""
import xml.etree.ElementTree as ET

from sw2robot.exporter.merge import merge_fixed_links

_URDF = """<?xml version="1.0"?>
<robot name="t">
  <link name="base">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="1 1 1"/></geometry></visual>
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <joint name="fix_c" type="fixed">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="c"/>
  </joint>
  <link name="c">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.5 0.5 0.5"/></geometry></visual>
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="3"/>
      <inertia ixx="2" ixy="0" ixz="0" iyy="2" iyz="0" izz="2"/></inertial>
  </link>
  <joint name="mov" type="revolute">
    <origin xyz="0 1 0" rpy="0 0 0"/>
    <parent link="c"/><child link="g"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <link name="g"><visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual></link>
  <joint name="fix_port" type="fixed">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/><child link="port"/>
  </joint>
  <link name="port"/>
</robot>"""


def _f(s):
    return [float(x) for x in s.split()]


def test_merge_lumps_geometry_inertia_and_reparents():
    root = ET.fromstring(_URDF)
    n, _ = merge_fixed_links(root)
    assert n == 1                                   # only 'c' merged

    links = {ln.get("name"): ln for ln in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}

    # child link + its fixed joint are gone; the mesh-less port frame is kept
    assert "c" not in links and "fix_c" not in joints
    assert "port" in links and "fix_port" in joints   # coordinate frame survives

    # base now carries its own + the child's visual (composed origin 1 0 0)
    base = links["base"]
    vis = base.findall("visual")
    assert len(vis) == 2
    moved = [v for v in vis
             if v.find("origin") is not None
             and _f(v.find("origin").get("xyz")) == [1, 0, 0]]
    assert len(moved) == 1

    # the movable joint re-parented onto base, origin composed (0,1,0)+(1,0,0)
    mov = joints["mov"]
    assert mov.find("parent").get("link") == "base"
    assert _f(mov.find("origin").get("xyz")) == [1, 1, 0]

    # inertials combined: m=5, com=(0.6,0,0), I=diag(3, 4.2, 4.2)
    inertial = base.find("inertial")
    assert abs(float(inertial.find("mass").get("value")) - 5.0) < 1e-9
    com = _f(inertial.find("origin").get("xyz"))
    assert abs(com[0] - 0.6) < 1e-9 and abs(com[1]) < 1e-9 and abs(com[2]) < 1e-9
    it = inertial.find("inertia")
    assert abs(float(it.get("ixx")) - 3.0) < 1e-9
    assert abs(float(it.get("iyy")) - 4.2) < 1e-9
    assert abs(float(it.get("izz")) - 4.2) < 1e-9
    assert abs(float(it.get("ixy"))) < 1e-12


def test_chain_of_fixed_joints_collapses():
    urdf = """<?xml version="1.0"?>
<robot name="c">
  <link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="f1" type="fixed"><origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="a"/><child link="b"/></joint>
  <link name="b"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="f2" type="fixed"><origin xyz="0 2 0" rpy="0 0 0"/>
    <parent link="b"/><child link="d"/></joint>
  <link name="d"><visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="1 1 1"/></geometry></visual></link>
</robot>"""
    root = ET.fromstring(urdf)
    n, _ = merge_fixed_links(root)
    assert n == 2
    links = {ln.get("name") for ln in root.findall("link")}
    assert links == {"a"}                            # everything collapsed onto a
    # d's visual lands at the composed (1,0,0)+(0,2,0) = (1,2,0)
    origins = [v.find("origin").get("xyz")
               for v in root.find("link").findall("visual")
               if v.find("origin") is not None]
    norm = {" ".join(f"{float(x):g}" for x in s.split()) for s in origins}
    assert "1 2 0" in norm


def test_meshless_frame_in_fixed_chain_is_preserved():
    """Regression: a mesh-less coordinate frame between a real parent and a
    meshed child must survive -- it must NOT receive the child's geometry and
    then be lumped away (would silently drop its TF)."""
    urdf = """<?xml version="1.0"?>
<robot name="f">
  <link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="to_frame" type="fixed"><origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/><child link="frame"/></joint>
  <link name="frame"/>
  <joint name="frame_part" type="fixed"><origin xyz="0 0 0.2" rpy="0 0 0"/>
    <parent link="frame"/><child link="part"/></joint>
  <link name="part"><visual><geometry><box size="0.3 0.3 0.3"/></geometry></visual></link>
</robot>"""
    root = ET.fromstring(urdf)
    merge_fixed_links(root)
    links = {ln.get("name") for ln in root.findall("link")}
    assert "frame" in links                          # the TF frame survives
    # the frame stays a pure coordinate frame (no geometry dumped into it)
    frame = next(ln for ln in root.findall("link") if ln.get("name") == "frame")
    assert frame.find("visual") is None and frame.find("collision") is None
