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
    if comp.mesh_file:
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
    lines.extend(inertial_lines)
    lines.append("  </link>")
    return "\n".join(lines), method, problems


def _joint_xml(joint, rn=lambda n: n, jn=lambda n: n):
    lines = [f'  <joint name="{jn(joint.name)}" type="{joint.jtype}">']
    lines.append(f'    <origin xyz="{_fmt(joint.xyz)}" rpy="{_fmt(joint.rpy)}"/>')
    lines.append(f'    <parent link="{rn(joint.parent)}"/>')
    lines.append(f'    <child link="{rn(joint.child)}"/>')
    if joint.axis is not None:
        lines.append(f'    <axis xyz="{_fmt(joint.axis)}"/>')
    if joint.jtype in ("revolute", "prismatic"):
        lo = joint.lower if joint.lower is not None else -3.14159
        up = joint.upper if joint.upper is not None else 3.14159
        lines.append(f'    <limit lower="{lo:.6g}" upper="{up:.6g}" '
                     f'effort="10" velocity="3.14"/>')
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
    keeps using ``root_link_name``."""
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
    for comp in model.components:
        xml, method, problems = _link_xml(comp, ros_pkg, rn, mesh_dir=mesh_dir,
                                          density=density)
        parts.append(xml)
        methods[method] = methods.get(method, 0) + 1
        if problems:
            bad_inertia[rn(comp.link_name)] = problems
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
    return urdf_path


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
# /robot_description topic, TF, grid).  {fixed_frame} = the root link name.
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
      Description Topic:
        Value: /robot_description
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
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


def write_ros_package(model, pkg_dir, email="auto@example.com"):
    with open(os.path.join(pkg_dir, "package.xml"), "w", encoding="utf-8") as f:
        f.write(PACKAGE_XML.format(name=model.name, email=email))
    with open(os.path.join(pkg_dir, "CMakeLists.txt"), "w", encoding="utf-8") as f:
        f.write(CMAKELISTS.format(name=model.name))
