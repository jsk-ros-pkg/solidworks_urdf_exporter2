"""Write a RobotModel as a URDF + minimal ROS package layout.

Layout::

    <pkg>/
      urdf/<name>.urdf
      meshes/<link>.3dxml        (written earlier by mesh.export_meshes)
      package.xml
      CMakeLists.txt

Mesh references use a path relative to the urdf file (``../meshes/x.3dxml``),
which scikit-robot / urdfpy resolve against the urdf directory -- this loads
locally with no ROS environment.  ROS users can switch these to
``package://<pkg>/meshes/...``.
"""

from __future__ import annotations

import os

from . import inertia as _inertia
from .model import safe_name

# Fallback inertial when no mesh is available / its volume is unusable.
_PLACEHOLDER_INERTIAL = {
    "mass": 0.1, "com": [0.0, 0.0, 0.0],
    "inertia": (1e-4, 0.0, 0.0, 1e-4, 0.0, 1e-4), "method": "placeholder",
}


def _fmt(v):
    return " ".join(f"{x:.8g}" for x in v)


def _comment_safe(text):
    """Make ``text`` safe to embed inside an XML comment.

    XML comments may not contain ``--`` and may not end with ``-``; collapse any
    such run so the comment stays well-formed regardless of the material name.
    """
    cleaned = str(text).replace("--", "-").strip()
    return cleaned.rstrip("-")


def _provenance_comment(comp, method):
    """Build the ``<!-- sw2robot ... -->`` provenance line for a link.

    Records the SolidWorks material, its density (kg/m^3) and how the inertial
    was obtained (``solidworks`` / ``mesh`` / ``hull`` / ``bbox`` /
    ``placeholder``), so the exported URDF is self-documenting. Returns ``None``
    when there is nothing worth recording.
    """
    fields = []
    material = getattr(comp, "material", None)
    density = getattr(comp, "density", None)
    if material:
        fields.append(f'material="{_comment_safe(material)}"')
    if density is not None:
        fields.append(f'density="{float(density):g}"')  # kg/m^3
    if method:
        fields.append(f'inertia="{method}"')
    if not fields:
        return None
    return f'    <!-- sw2robot {" ".join(fields)} -->'


def _inertial_xml(comp, mesh_dir, density):
    """Compute and format a link's ``<inertial>``.

    Source priority:

    1. SolidWorks-native mass properties (exact CAD geometry + material /
       manual override), unless the caller passed an explicit global
       ``density`` or the link carries a per-link density override -- both of
       which mean "drive the mass from this density instead".
    2. The mesh estimate (volume x density via trimesh), with the part's
       SolidWorks material density winning over the global default.
    3. A small placeholder if neither is usable.

    Returns ``(xml_lines, method, problems)`` so the caller can report which
    source / approximation each link used and flag any physically invalid
    inertial (``problems`` is a list of sanity-check failures, empty if OK)."""
    info = None
    # 1. SolidWorks-native values (only when no density override is in force)
    if density is None and not getattr(comp, "density_override", False):
        info = _inertia.link_inertial_from_sw(
            getattr(comp, "sw_mass", None), getattr(comp, "sw_com", None),
            getattr(comp, "sw_inertia", None),
            comp.visual_xyz, comp.visual_rpy)
    # 2. Mesh estimate
    if info is None and comp.mesh_file and mesh_dir:
        # mesh_file may carry Windows backslashes (extracted on Windows); split
        # on them so os.path.join builds a valid path on any platform.
        rel = comp.mesh_file.replace("\\", "/")
        # per-link density: the part's SolidWorks material wins over the
        # global default (config `density:` still overrides everything via
        # the caller)
        d = (getattr(comp, "density", None)
             or (density if density is not None else _inertia.DEFAULT_DENSITY))
        info = _inertia.link_inertial(
            os.path.join(mesh_dir, *rel.split("/")),
            comp.visual_xyz, comp.visual_rpy, density=d)
    if info is None:
        info = _PLACEHOLDER_INERTIAL
    ixx, ixy, ixz, iyy, iyz, izz = info["inertia"]
    lines = [
        "    <inertial>",
        f'      <origin xyz="{_fmt(info["com"])}" rpy="0 0 0"/>',
        f'      <mass value="{info["mass"]:.6g}"/>',
        f'      <inertia ixx="{ixx:.6g}" ixy="{ixy:.6g}" ixz="{ixz:.6g}" '
        f'iyy="{iyy:.6g}" iyz="{iyz:.6g}" izz="{izz:.6g}"/>',
        "    </inertial>",
    ]
    problems = _inertia.validate_inertia(info["mass"], info["inertia"])
    return lines, info["method"], problems


def _report_inertia(methods):
    """Print a one-line summary of how each link's inertia was obtained, so any
    approximation (non-watertight mesh / placeholder) is visible rather than
    silent.  ``solidworks`` (exact CAD value) and ``mesh`` (watertight volume
    integral) are exact; ``hull``/``bbox``/``placeholder`` are approximations."""
    if not methods:
        return
    _EXACT = ("solidworks", "mesh")
    exact = {k: v for k, v in methods.items() if k in _EXACT}
    approx = {k: v for k, v in methods.items() if k not in _EXACT}
    msg = "      inertia: " + (", ".join(f"{v} from {k}"
                                         for k, v in sorted(exact.items()))
                               or "0 exact")
    if approx:
        detail = ", ".join(f"{v} {k}" for k, v in sorted(approx.items()))
        msg += f"; APPROX -> {detail}"
    print(msg)


def _report_inertia_problems(bad):
    """Loudly flag links whose ``<inertial>`` fails a physics sanity check
    (non-positive-definite tensor, triangle-inequality violation, bad mass).
    These make simulators diverge and usually mean a units/frame/transform bug,
    so surface them rather than emit silently."""
    if not bad:
        return
    print(f"      WARN: {len(bad)} link(s) have a physically invalid inertial "
          f"(may diverge in simulation):")
    for link, problems in sorted(bad.items()):
        for p in problems:
            print(f"        - {link}: {p}")


def _link_xml(comp, ros_pkg=None, rn=lambda n: n, mesh_dir=None, density=None):
    lines = [f'  <link name="{rn(comp.link_name)}">']
    # a mass-only link keeps its <inertial> but drops all geometry (its weight is
    # lumped into the fixed parent on export); skip the visual/collision block
    if comp.mesh_file and not getattr(comp, "mass_only", False):
        mesh_ref = ("package://{}/{}".format(ros_pkg, comp.mesh_file.replace("\\", "/"))
                    if ros_pkg else "../" + comp.mesh_file.replace("\\", "/"))
        vorigin = (f'      <origin xyz="{_fmt(comp.visual_xyz)}" '
                   f'rpy="{_fmt(comp.visual_rpy)}"/>')
        for tag in ("visual", "collision"):
            lines.append(f"    <{tag}>")
            lines.append(vorigin)
            lines.append("      <geometry>")
            lines.append(f'        <mesh filename="{mesh_ref}"/>')
            lines.append("      </geometry>")
            lines.append(f"    </{tag}>")
    inertial_lines, method, problems = _inertial_xml(comp, mesh_dir, density)
    comment = _provenance_comment(comp, method)
    if comment is not None:
        lines.insert(1, comment)  # right after the <link name=...> line
    lines.extend(inertial_lines)
    lines.append("  </link>")
    return "\n".join(lines), method, problems


def _attrs(spec, source):
    """Render ``name="value"`` attribute pairs for the keys in ``spec`` that are
    present (not None) in mapping ``source``, in ``spec`` order."""
    if not isinstance(source, dict):
        return ""
    parts = []
    for key in spec:
        v = source.get(key)
        if v is not None:
            parts.append(f'{key}="{float(v):.6g}"')
    return " ".join(parts)


def _physics_lines(joint):
    """URDF ``<dynamics>`` / ``<safety_controller>`` / ``<calibration>`` lines
    for a joint, each emitted only when the joint carries values for it."""
    out = []
    dyn = _attrs(("damping", "friction"), getattr(joint, "dynamics", None))
    if dyn:
        out.append(f"    <dynamics {dyn}/>")
    saf = _attrs(("soft_lower_limit", "soft_upper_limit", "k_position",
                  "k_velocity"), getattr(joint, "safety", None))
    if saf:
        out.append(f"    <safety_controller {saf}/>")
    cal = _attrs(("rising", "falling"), getattr(joint, "calibration", None))
    if cal:
        out.append(f"    <calibration {cal}/>")
    return out


def _joint_xml(joint, rn=lambda n: n, jn=lambda n: n):
    lines = [f'  <joint name="{jn(joint.name)}" type="{joint.jtype}">']
    lines.append(f'    <origin xyz="{_fmt(joint.xyz)}" rpy="{_fmt(joint.rpy)}"/>')
    lines.append(f'    <parent link="{rn(joint.parent)}"/>')
    lines.append(f'    <child link="{rn(joint.child)}"/>')
    if joint.axis is not None:
        lines.append(f'    <axis xyz="{_fmt(joint.axis)}"/>')
    eff = getattr(joint, "effort", None)
    vel = getattr(joint, "velocity", None)
    eff = 10 if eff is None else eff
    vel = 3.14 if vel is None else vel
    if joint.jtype in ("revolute", "prismatic"):
        lo = joint.lower if joint.lower is not None else -3.14159
        up = joint.upper if joint.upper is not None else 3.14159
        lines.append(f'    <limit lower="{lo:.6g}" upper="{up:.6g}" '
                     f'effort="{eff:.6g}" velocity="{vel:.6g}"/>')
    elif joint.jtype == "continuous" and (
            getattr(joint, "effort", None) is not None
            or getattr(joint, "velocity", None) is not None):
        # continuous joints carry no lower/upper, but a <limit> may still pin
        # effort/velocity when the user set them
        lines.append(f'    <limit effort="{eff:.6g}" velocity="{vel:.6g}"/>')
    lines.extend(_physics_lines(joint))
    if joint.mimic:
        mj = jn(joint.mimic.get("joint"))
        mult = joint.mimic.get("multiplier", 1.0)
        off = joint.mimic.get("offset", 0.0)
        lines.append(f'    <mimic joint="{mj}" multiplier="{mult:g}" '
                     f'offset="{off:g}"/>')
    if joint.mate_types:
        lines.append(f'    <!-- mates: {", ".join(joint.mate_types)} -->')
    lines.append("  </joint>")
    return "\n".join(lines)


def _port_xml(port, rn=lambda n: n, link_name=None, joint_name=None):
    """An output port: an empty dummy_link on a fixed joint (robot-compiler
    detects ``dummy_link*`` as a bidirectional connection port)."""
    if link_name is None:
        link_name = rn(port.name)
    if joint_name is None:
        if getattr(port, "joint_name", ""):
            joint_name = safe_name(port.joint_name)
        else:
            suffix = port.name[len("dummy_link"):] \
                if port.name.startswith("dummy_link") else "_" + port.name
            joint_name = safe_name("dummy_joint" + suffix)
    return "\n".join([
        f'  <link name="{link_name}"/>',
        f'  <joint name="{joint_name}" type="fixed">',
        f'    <origin xyz="{_fmt(port.xyz)}" rpy="{_fmt(port.rpy)}"/>',
        f'    <parent link="{rn(port.parent_link)}"/>',
        f'    <child link="{link_name}"/>',
        "  </joint>",
    ])


def write_urdf(model, urdf_path, ros_pkg=None, density=None,
               link_overrides=None, joint_overrides=None):
    """Write the URDF.  ``link_overrides`` / ``joint_overrides`` map a COMPONENT
    link name / joint name to a user-chosen display name (from the editor's
    rename feature); they are applied before safe_name + collision suffixing, so
    every reference (parent/child/mimic) follows automatically.  The root link
    keeps using ``root_link_name``.

    Returns the set of FINAL (emitted) link names that are mass-only -- the
    export step folds these into their fixed parent.  write_urdf owns the name
    sanitising, so it is the authoritative source of those names."""
    os.makedirs(os.path.dirname(urdf_path), exist_ok=True)
    link_overrides = link_overrides or {}
    joint_overrides = joint_overrides or {}
    # The internal model keeps each component's own link name; the emitted URDF
    # renames the root to the module convention (base_link = input port) so the
    # module loads cleanly in robot-compiler.  Done here (not in the model) so
    # the joint-config template / viewer keep referring to component names.
    root_name = getattr(model, "root_link_name", "") or model.base_link

    def _unique_safe(items):
        mapping, used = {}, set()
        for key, raw in items:
            base = safe_name(raw)
            cand, i = base, 1
            while cand in used:
                i += 1
                cand = f"{base}_{i}"
            mapping[key] = cand
            used.add(cand)
        return mapping

    ports = list(getattr(model, "ports", []))
    comp_name_items = [
        (("comp", c.link_name),
         root_name if c.link_name == model.base_link
         else link_overrides.get(c.link_name) or c.link_name)
        for c in model.components]
    comp_name_items.sort(key=lambda item: item[0][1] != model.base_link)
    link_names = _unique_safe(
        comp_name_items
        + [(("port", i), p.name) for i, p in enumerate(ports)])
    joint_names = _unique_safe(
        [(("joint", j.name), joint_overrides.get(j.name, j.name))
         for j in model.joints]
        + [(("port", i), p.joint_name or ("dummy_joint" + (
            p.name[len("dummy_link"):] if p.name.startswith("dummy_link")
            else "_" + p.name))) for i, p in enumerate(ports)])

    # every emitted link / joint name passes through safe_name, with collision
    # suffixes shared by all references (root remap, ports, parent/child, mimic).
    def rn(name):
        key = ("comp", name)
        return link_names[key] if key in link_names else safe_name(name or "")

    def jn(name):
        key = ("joint", name)
        return joint_names[key] if key in joint_names else safe_name(name or "")

    # meshes are stored relative to the package root (<pkg>/meshes/x), and the
    # urdf lives at <pkg>/urdf/<name>.urdf -> the package root is two levels up.
    mesh_dir = os.path.dirname(os.path.dirname(os.path.abspath(urdf_path)))

    parts = ['<?xml version="1.0"?>', f'<robot name="{model.name}">']
    methods = {}
    bad_inertia = {}
    mass_only_links = set()
    for comp in model.components:
        xml, method, problems = _link_xml(comp, ros_pkg, rn, mesh_dir=mesh_dir,
                                          density=density)
        parts.append(xml)
        methods[method] = methods.get(method, 0) + 1
        if problems:
            bad_inertia[rn(comp.link_name)] = problems
        if getattr(comp, "mass_only", False):
            mass_only_links.add(rn(comp.link_name))
    _report_inertia(methods)
    _report_inertia_problems(bad_inertia)
    for joint in model.joints:
        parts.append(_joint_xml(joint, rn, jn))
    for i, port in enumerate(ports):
        parts.append(_port_xml(port, rn, link_names[("port", i)],
                               joint_names[("port", i)]))
    parts.append("</robot>\n")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    return mass_only_links


PACKAGE_XML = """<?xml version="1.0"?>
<package format="2">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>URDF generated from SolidWorks assembly {name}</description>
  <maintainer email="{email}">auto</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
</package>
"""

CMAKELISTS = """cmake_minimum_required(VERSION 3.0.2)
project({name})
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY urdf meshes
  DESTINATION ${{CATKIN_PACKAGE_SHARE_DESTINATION}})
"""

# --- ROS 2 (ament_cmake) variants -------------------------------------------
# Same description-package layout, but a format-3 manifest + ament_cmake build,
# plus a ready-to-run display launch file and an RViz config so the package is
# usable straight away: `ros2 launch <name> display.launch.py`.
PACKAGE_XML_ROS2 = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>URDF generated from SolidWorks assembly {name}</description>
  <maintainer email="{email}">auto</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

CMAKELISTS_ROS2 = """cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${{PROJECT_NAME}})
ament_package()
"""

# {name} = package name, {robot} = urdf file stem.  Reads the plain URDF (not
# xacro) and brings up robot_state_publisher + joint_state_publisher_gui + RViz.
DISPLAY_LAUNCH_ROS2 = '''import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("{name}")
    urdf = os.path.join(pkg, "urdf", "{robot}.urdf")
    with open(urdf) as f:
        robot_description = f.read()
    rviz = os.path.join(pkg, "rviz", "{robot}.rviz")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{{"robot_description": robot_description}}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz],
        ),
    ])
'''

# A minimal RViz2 config so the robot shows up immediately (RobotModel from the
# /robot_description topic, TF, grid).  {fixed_frame} = the root link name,
# {tf_scale} = TF axis ("Marker Scale") sized to the model so the triads do not
# dwarf small parts (<= 1.0).
RVIZ_CONFIG_ROS2 = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
      Collision Enabled: false
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Depth: 5
        History Policy: Keep Last
        Reliability Policy: Reliable
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Marker Scale: {tf_scale}
  Global Options:
    Fixed Frame: {fixed_frame}
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
"""

# --- ROS 2 closed-loop variants ---------------------------------------------
# A package with a detected kinematic loop ships three extra pieces: the loop
# closure config, a relay node that CLOSES the loop by runtime IK (pure-numpy
# URDF FK), and a launch file wired GUI -> relay -> robot_state_publisher so the
# linkage tracks exactly in RViz.  (robot_state_publisher honours a URDF <mimic>
# only LINEARLY, which a real loop is not; the relay solves the true closure.)
PACKAGE_XML_ROS2_COUPLED = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>URDF generated from SolidWorks assembly {name}</description>
  <maintainer email="{email}">auto</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

CMAKELISTS_ROS2_COUPLED = """cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${{PROJECT_NAME}})
# The relay must be executable for `ros2 run`/launch to find it in libexec.
# `colcon build --symlink-install` symlinks the installed file back to this
# source, so a non-executable source (a Windows export can't set the bit)
# leaves a dead libexec entry -- chmod the source here so the symlink works.
if(UNIX)
  execute_process(COMMAND chmod +x
    ${{CMAKE_CURRENT_SOURCE_DIR}}/scripts/loop_closure_relay.py)
endif()
install(PROGRAMS scripts/loop_closure_relay.py
  DESTINATION lib/${{PROJECT_NAME}})
ament_package()
"""

# {name} = package name, {robot} = urdf file stem.  Sliders drive the
# independent joints; the dependent (loop) joints carry no slider.  The GUI
# publishes on joint_states_source; the relay solves the loop closure for the
# dependent joints and republishes on joint_states, which RSP consumes.
DISPLAY_LAUNCH_ROS2_COUPLED = '''import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("{name}")
    urdf = os.path.join(pkg, "urdf", "{robot}.urdf")
    with open(urdf) as f:
        robot_description = f.read()
    rviz = os.path.join(pkg, "rviz", "{robot}.rviz")
    closures = os.path.join(pkg, "config", "loop_closures.yaml")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{{"robot_description": robot_description}}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            remappings=[("joint_states", "joint_states_source")],
        ),
        Node(
            package="{name}",
            executable="loop_closure_relay.py",
            output="screen",
            parameters=[{{"urdf_file": urdf, "config_file": closures}}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz],
        ),
    ])
'''

# A standalone rclpy node, pure numpy FK (no {{}} .format -- shipped verbatim).
LOOP_COUPLING_RELAY_PY = '''#!/usr/bin/env python3
"""Close kinematic loops at runtime by numerical IK (pure-numpy URDF FK).

A URDF is a tree, so a closed linkage (four-bar / parallel mechanism) has its
loops cut and robot_state_publisher honours the cut joints\' <mimic> only
linearly -- which a real loop is not.  This node parses the URDF and, on every
joint-state, sets the DRIVEN (independent) joints, then solves (damped
least-squares, sub-stepped to stay on the assembly branch) the DEPENDENT joints
so each cut loop closes (the two link points the dropped hinge joined coincide),
and republishes the corrected joint_states.  Exact and
general: any single-DOF-per-loop linkage, planar or spatial.  Needs only rclpy,
sensor_msgs, numpy and yaml -- no extra install.

Config (config/loop_closures.yaml): {independent:[...], dependent:[...],
closures:[{link_a, link_b, point:[xyz in base_link], axis:[xyz]}]}.
"""
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _rpy_matrix(rpy):
    rx, ry, rz = rpy
    cx, cy, cz = np.cos(rx), np.cos(ry), np.cos(rz)
    sx, sy, sz = np.sin(rx), np.sin(ry), np.sin(rz)
    rxm = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rym = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rzm = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    m = np.eye(4)
    m[:3, :3] = rzm @ rym @ rxm
    return m


class _FK:
    """Minimal URDF forward kinematics (revolute/continuous/prismatic/fixed)."""

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.j = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = ([float(x) for x in o.get("xyz", "0 0 0").split()]
                   if o is not None else [0, 0, 0])
            rpy = ([float(x) for x in o.get("rpy", "0 0 0").split()]
                   if o is not None else [0, 0, 0])
            ax = j.find("axis")
            axis = (np.array([float(x) for x in ax.get("xyz").split()])
                    if ax is not None else np.array([0.0, 0.0, 1.0]))
            t = _rpy_matrix(rpy)
            t[:3, 3] = xyz
            self.j[j.get("name")] = (j.get("type"),
                                     j.find("child").get("link"),
                                     j.find("parent").get("link"), t, axis)
        self.cj = {v[1]: n for n, v in self.j.items()}

    @staticmethod
    def _rot(a, q):
        a = a / (np.linalg.norm(a) or 1.0)
        x, y, z = a
        c, s = np.cos(q), np.sin(q)
        cc = 1.0 - c
        return np.array([[c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s, 0],
                         [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s, 0],
                         [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc, 0],
                         [0, 0, 0, 1.0]])

    def _tj(self, n, q):
        typ, _c, _p, t, axis = self.j[n]
        if typ in ("revolute", "continuous"):
            return t @ self._rot(axis, q)
        if typ == "prismatic":
            m = np.eye(4)
            m[:3, 3] = axis / (np.linalg.norm(axis) or 1.0) * q
            return t @ m
        return t

    def world(self, link, q):
        chain, ln = [], link
        while ln in self.cj:
            n = self.cj[ln]
            chain.append(n)
            ln = self.j[n][2]
        m = np.eye(4)
        for n in reversed(chain):
            m = m @ self._tj(n, q.get(n, 0.0))
        return m


class LoopClosureRelay(Node):
    def __init__(self):
        super().__init__("loop_closure_relay")
        self.declare_parameter("urdf_file", "")
        self.declare_parameter("config_file", "")
        self.fk = _FK(self.get_parameter("urdf_file").value)
        cfg = {}
        path = self.get_parameter("config_file").value
        if path:
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        self.deps = list(cfg.get("dependent", []))
        self.indep = set(cfg.get("independent", []))
        self.wit = []
        for c in cfg.get("closures", []):
            la, lb = c["link_a"], c["link_b"]
            t0a, t0b = self.fk.world(la, {}), self.fk.world(lb, {})
            p = np.asarray(c["point"], float)
            a = np.asarray(c["axis"], float)
            a = a / (np.linalg.norm(a) or 1.0)
            for q in (p, p + a * 0.03):
                la_loc = (np.linalg.inv(t0a) @ np.append(q, 1.0))[:3]
                lb_loc = (np.linalg.inv(t0b) @ np.append(q, 1.0))[:3]
                self.wit.append((la, la_loc, lb, lb_loc))
        self._x = np.zeros(len(self.deps))        # warm start (tracks branch)
        self._indep_prev = None                   # previous driven-joint values
        if not self.deps or not self.wit:
            self.get_logger().warn("no loop closures loaded; relaying unchanged")
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "joint_states_source",
                                 self._cb, 10)

    def _resid(self, q):
        out = []
        for la, lal, lb, lbl in self.wit:
            wa = (self.fk.world(la, q) @ np.append(lal, 1.0))[:3]
            wb = (self.fk.world(lb, q) @ np.append(lbl, 1.0))[:3]
            out.append(wa - wb)
        return np.concatenate(out) if out else np.zeros(0)

    def _solve(self, q):
        x0 = self._x.copy()
        x = x0.copy()
        n = len(self.deps)
        for _ in range(50):
            for i, dn in enumerate(self.deps):
                q[dn] = x[i]
            r = self._resid(q)
            if r.size == 0 or np.linalg.norm(r) < 1e-10:
                break
            jac = np.zeros((r.size, n))
            for i, dn in enumerate(self.deps):
                xb = x[i]
                q[dn] = xb + 1e-6
                jac[:, i] = (self._resid(q) - r) / 1e-6
                q[dn] = xb
            # damped least squares (Levenberg-Marquardt) + a step clamp keep the
            # solver on the CURRENT assembly branch: near a singularity (a
            # four-bar toggle) plain Gauss-Newton jumps/spins to the other mode
            dx = np.linalg.solve(jac.T @ jac + 1e-6 * np.eye(n), -jac.T @ r)
            s = float(np.linalg.norm(dx))
            if s > 0.15:
                dx *= 0.15 / s
            x = x + dx
        # at / past a toggle the loop has NO solution; rather than accept a
        # diverged (wrapped) config that warm-start would then lock in forever,
        # hold the last valid pose -- the linkage just stops at its limit
        for i, dn in enumerate(self.deps):
            q[dn] = x[i]
        if np.linalg.norm(self._resid(q)) > 1e-5:
            x = x0
        self._x = x
        return x

    def _cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not self.deps or not self.wit:
            self._pub.publish(msg)
            return
        target = {n: pos[n] for n in self.indep if n in pos}
        # SUB-STEP the driven joints from their previous values: a big jump (a
        # fast slider drag, or clicking a far value) solved in one shot walks to
        # the WRONG assembly branch and sticks; stepping it in small increments
        # keeps the warm-started solver on the current branch.
        prev = self._indep_prev if self._indep_prev is not None else target
        maxd = max((abs(target[k] - prev.get(k, target[k])) for k in target),
                   default=0.0)
        steps = max(1, int(maxd / 0.05))          # ~3 deg per sub-step
        x = self._x
        for s in range(1, steps + 1):
            f = s / steps
            q = {k: prev.get(k, target[k])
                 + (target[k] - prev.get(k, target[k])) * f for k in target}
            x = self._solve(q)
        self._indep_prev = dict(target)
        for i, dn in enumerate(self.deps):
            pos[dn] = float(x[i])
        out = JointState()
        out.header = msg.header
        out.name = list(pos.keys())
        out.position = [float(pos[n]) for n in out.name]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
'''


def write_ros_package(model, pkg_dir, email="auto@example.com"):
    with open(os.path.join(pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
        f.write(PACKAGE_XML.format(name=model.name, email=email))
    with open(os.path.join(pkg_dir, "CMakeLists.txt"), "w", encoding="utf-8") as f:
        f.write(CMAKELISTS.format(name=model.name))
