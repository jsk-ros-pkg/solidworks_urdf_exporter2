"""Fixed-joint lumping, as sw2robot relies on it.

The implementation is :func:`skrobot.urdf.merge_fixed_links`; these are the
behaviours the exporter and the editor depend on -- geometry moved with a
composed origin, inertials combined (mass + parallel-axis), movable joints
re-parented, mesh-less coordinate frames preserved, and the ``force_merge`` /
``only`` handling that the mass-only links need."""
import xml.etree.ElementTree as ET

from skrobot.urdf import merge_fixed_links

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
    n = merge_fixed_links(root)
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
    n = merge_fixed_links(root)
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


# ----------------------------------------------------------------- mass-only
# A mass-only link has an <inertial> but NO geometry.  Named in force_merge / only
# it folds into its fixed parent so the weight survives -- unlike a genuine
# mesh-less coordinate frame, which (absent from the set) stays.

_MASS_ONLY_URDF = """<?xml version="1.0"?>
<robot name="m">
  <link name="base">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="1 1 1"/></geometry></visual>
  </link>
  <joint name="fix_int" type="fixed">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="internal"/>
  </joint>
  <link name="internal">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="3"/>
      <inertia ixx="2" ixy="0" ixz="0" iyy="2" iyz="0" izz="2"/></inertial>
  </link>
  <joint name="fix_port" type="fixed">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/><child link="port"/>
  </joint>
  <link name="port"/>
</robot>"""


def test_mass_only_link_folds_into_parent_keeping_mass():
    """A geometry-less link named in force_merge folds into its fixed parent so
    its weight is kept; the bare port frame (NOT in the set) is preserved."""
    root = ET.fromstring(_MASS_ONLY_URDF)
    n = merge_fixed_links(root, force_merge={"internal"})
    assert n == 1                                   # only 'internal' folded

    links = {ln.get("name") for ln in root.findall("link")}
    joints = {j.get("name") for j in root.findall("joint")}
    # the mass-only link + its fixed joint are gone; the bare port frame stays
    assert "internal" not in links and "fix_int" not in joints
    assert "port" in links and "fix_port" in joints

    # mass conserved and inertia folded just like a meshed child would be:
    # m=5, com=(0.6,0,0), I=diag(3, 4.2, 4.2)  (parallel-axis about the new COM)
    base = next(ln for ln in root.findall("link") if ln.get("name") == "base")
    inertial = base.find("inertial")
    assert abs(float(inertial.find("mass").get("value")) - 5.0) < 1e-9
    com = _f(inertial.find("origin").get("xyz"))
    assert abs(com[0] - 0.6) < 1e-9 and abs(com[1]) < 1e-9 and abs(com[2]) < 1e-9
    it = inertial.find("inertia")
    assert abs(float(it.get("ixx")) - 3.0) < 1e-9
    assert abs(float(it.get("iyy")) - 4.2) < 1e-9
    assert abs(float(it.get("izz")) - 4.2) < 1e-9
    # base keeps its own single visual (the mass-only child contributed none)
    assert len(base.findall("visual")) == 1


def test_mass_only_not_in_force_merge_is_preserved_as_a_frame():
    """Without force_merge, a geometry-less link is treated as a coordinate
    frame and preserved -- the fold is strictly opt-in, never automatic."""
    root = ET.fromstring(_MASS_ONLY_URDF)
    n = merge_fixed_links(root)                  # no force_merge
    assert n == 0
    assert "internal" in {ln.get("name") for ln in root.findall("link")}


def test_mass_only_not_folded_when_joint_is_movable():
    """A force_merge name on a MOVABLE edge is not folded (merge only collapses
    fixed edges) -- merge must never silently weld a movable joint shut; the
    geometry-strip / movable guard is the writer's job."""
    urdf = _MASS_ONLY_URDF.replace('name="fix_int" type="fixed"',
                                   'name="fix_int" type="revolute"')
    # a revolute joint needs an axis/limit to be well-formed
    urdf = urdf.replace(
        '<parent link="base"/><child link="internal"/>\n  </joint>',
        '<parent link="base"/><child link="internal"/>\n'
        '    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>\n'
        '  </joint>')
    root = ET.fromstring(urdf)
    n = merge_fixed_links(root, force_merge={"internal"})
    assert n == 0
    assert "internal" in {ln.get("name") for ln in root.findall("link")}



def test_only_folds_just_the_named_link_not_the_whole_fixed_tree():
    """``only`` folds the mass-only links and leaves every OTHER fixed child in
    place -- so a mass-only fold never silently enables a full fixed merge."""
    urdf = """<?xml version="1.0"?>
<robot name="o">
  <link name="base"><inertial><mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="fix_bracket" type="fixed"><origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/><child link="bracket"/></joint>
  <link name="bracket"><inertial><mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="fix_pcb" type="fixed"><origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="pcb"/></joint>
  <link name="pcb"><inertial><mass value="2"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link>
</robot>"""
    root = ET.fromstring(urdf)
    n = merge_fixed_links(root, only={"pcb"})
    assert n == 1
    links = {ln.get("name") for ln in root.findall("link")}
    assert "pcb" not in links            # the mass-only link folded
    assert "bracket" in links            # the ordinary fixed child is untouched


def test_mass_only_folds_into_a_coordinate_frame_parent():
    """A mass-only link whose fixed parent is a coordinate frame still folds: the
    fold adds only <inertial> (no geometry), so the frame keeps its TF and is NOT
    itself lumped away -- the child's weight reaches the frame instead of being
    stranded on a geometry-less, never-folded link.  base--fixed-->frame--fixed-->pcb."""
    urdf = """<?xml version="1.0"?>
<robot name="f">
  <link name="base"><inertial><mass value="2"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="j1" type="fixed"><origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="frame"/></joint>
  <link name="frame"/>
  <joint name="j2" type="fixed"><origin xyz="0 1 0" rpy="0 0 0"/>
    <parent link="frame"/><child link="pcb"/></joint>
  <link name="pcb"><inertial><mass value="3"/>
    <inertia ixx="2" ixy="0" ixz="0" iyy="2" iyz="0" izz="2"/></inertial></link>
</robot>"""
    root = ET.fromstring(urdf)
    n = merge_fixed_links(root, only={"pcb"})
    assert n == 1
    links = {ln.get("name") for ln in root.findall("link")}
    assert "pcb" not in links            # the mass-only link folded into the frame
    assert "base" in links and "frame" in links
    frame = next(ln for ln in root.findall("link") if ln.get("name") == "frame")
    # the frame stays a pure coordinate frame (TF preserved, no geometry added)
    assert frame.find("visual") is None and frame.find("collision") is None
    # and it is NOT itself lumped into base -- its fixed joint survives
    assert "j1" in {j.get("name") for j in root.findall("joint")}
    # pcb's weight reached the frame (mass-only kept the inertial)
    assert abs(float(frame.find("inertial").find("mass").get("value")) - 3.0) < 1e-9


def test_fold_rotates_child_inertia_into_the_parent_frame():
    """A fixed joint with a NON-identity rotation (rpy) must rotate the child's
    inertia tensor into the parent frame (R @ I @ R.T) before combining -- the
    other fold tests all use rpy=0, so a dropped/transposed rotation there would
    pass them silently.  Here the joint yaws the child 90 deg about Z and the
    child tensor is anisotropic, so the rotation is observable.

    base:  m=2, com=(0,0,0), I=diag(1,1,1)
    joint: xyz=(1,0,0), rpy=(0,0,pi/2)      (child +x -> parent +y)
    pcb:   m=3, com=(0,0,0), I=diag(2,5,7)

    Rz(90) maps the child tensor diag(2,5,7) -> diag(5,2,7) in the parent frame.
    new mass = 5; new com = (2*0 + 3*1)/5 = (0.6,0,0).  Parallel-axis to the new
    com (offset along x only):
      base  diag(1,1,1)      + 2*(0.36 - dx^2 terms) = diag(1, 1.72, 1.72)
      pcb   diag(5,2,7)      + 3*(0.16 - dx^2 terms) = diag(5, 2.48, 7.48)
      total                                           = diag(6, 4.2, 9.2)
    (Without the R @ I @ R.T rotation the child stays diag(2,5,7) and the total
    is diag(3, 7.2, 9.2) -- so ixx/iyy cleanly distinguish the two.)"""
    import numpy as np

    hz = np.pi / 2
    urdf = f"""<?xml version="1.0"?>
<robot name="rot">
  <link name="base"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="2"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="fix_pcb" type="fixed"><origin xyz="1 0 0" rpy="0 0 {hz}"/>
    <parent link="base"/><child link="pcb"/></joint>
  <link name="pcb"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="3"/>
    <inertia ixx="2" ixy="0" ixz="0" iyy="5" iyz="0" izz="7"/></inertial></link>
</robot>"""
    root = ET.fromstring(urdf)
    n = merge_fixed_links(root, force_merge={"pcb"})
    assert n == 1
    assert "pcb" not in {ln.get("name") for ln in root.findall("link")}

    base = next(ln for ln in root.findall("link") if ln.get("name") == "base")
    inertial = base.find("inertial")
    assert abs(float(inertial.find("mass").get("value")) - 5.0) < 1e-9
    com = _f(inertial.find("origin").get("xyz"))
    assert abs(com[0] - 0.6) < 1e-9 and abs(com[1]) < 1e-9 and abs(com[2]) < 1e-9
    it = inertial.find("inertia")
    assert abs(float(it.get("ixx")) - 6.0) < 1e-9, it.get("ixx")
    assert abs(float(it.get("iyy")) - 4.2) < 1e-9, it.get("iyy")
    assert abs(float(it.get("izz")) - 9.2) < 1e-9, it.get("izz")
    # a yaw about Z keeps the tensor diagonal (no products of inertia appear)
    assert abs(float(it.get("ixy"))) < 1e-9
    assert abs(float(it.get("ixz"))) < 1e-9
    assert abs(float(it.get("iyz"))) < 1e-9


def test_massless_child_inertial_contributes_nothing():
    """A child whose <inertial> declares mass 0 must not push a phantom tensor
    into the parent: a body with no mass carries no inertia, and such a block is
    malformed URDF (validate_inertia flags it).  The parent keeps its own
    inertial untouched."""
    urdf = """<?xml version="1.0"?>
<robot name="z">
  <link name="base"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="2"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="1 1 1"/></geometry></visual></link>
  <joint name="fix_z" type="fixed"><origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="ghost"/></joint>
  <link name="ghost"><inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0"/>
    <inertia ixx="9" ixy="0" ixz="0" iyy="9" iyz="0" izz="9"/></inertial>
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
</robot>"""
    root = ET.fromstring(urdf)
    n = merge_fixed_links(root)
    assert n == 1

    base = next(ln for ln in root.findall("link") if ln.get("name") == "base")
    inertial = base.find("inertial")
    assert abs(float(inertial.find("mass").get("value")) - 2.0) < 1e-9
    assert _f(inertial.find("origin").get("xyz")) == [0.0, 0.0, 0.0]
    it = inertial.find("inertia")
    for key in ("ixx", "iyy", "izz"):
        assert abs(float(it.get(key)) - 1.0) < 1e-9, (key, it.get(key))
