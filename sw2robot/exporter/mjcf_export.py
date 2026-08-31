"""MuJoCo (MJCF) export.

The URDF a CAD export produces is not yet a MuJoCo model, and the gap is not
just a file format.  Everything here exists because a robot that loads in RViz
still fails, or trains badly, in MuJoCo:

* **The model has to compile.**  A CAD assembly is one link per screw, with
  per-geometry origins and material-split sub-meshes.  So the URDF is first
  rebuilt with fixed links merged and origins baked (the same machinery the ROS
  export uses), and only then converted.
* **The feet have to make clean contact.**  MuJoCo collides a mesh through its
  convex hull, and the hull of a flat bracket end gives contact points that jump
  around under a walking policy.  Each foot therefore gets a sphere sized from
  its own contact patch, tangent to the real lowest point.
* **The task has to find its sensors.**  A locomotion task reads an IMU site and
  named sensors; without them the model loads and the task does not run.

Entry point: :func:`write_mjcf_package`.

The URDF -> MJCF conversion itself is :func:`skrobot.urdf.urdf_to_mjcf`, and so
is anything derivable from the URDF alone: this module asks it for
``joint_damping='backemf'`` (each joint damped by its own ``effort / velocity``,
the torque-speed slope of its actual servo) and ``home_base_height='auto'`` (a
home keyframe whose base height is measured so nothing starts below the floor).
What stays here is what needs to know what the links MEAN -- which of them are
feet, and where the IMU goes -- plus preparing the converter's input.
"""

from __future__ import annotations

import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from .ros_export import write_ros_description_package

# MuJoCo geom groups the converter emits: 2 = visual (no contact), 3 =
# collision.  The extras below follow the same convention, and put sites on 4 so
# a viewer can toggle them separately.
_COLLISION_GROUP = "3"
_SITE_GROUP = "4"

# Sensor names the common MuJoCo locomotion tasks (mjlab's velocity task among
# them) look up on the base body.  They are part of the interface, not a
# preference, so they are spelled out here rather than derived.
_IMU_SITE = "imu_in_body"
_SENSORS = (("gyro", "imu_ang_vel"), ("velocimeter", "imu_lin_vel"),
            ("accelerometer", "imu_lin_acc"))
_ANGMOM_SENSOR = "root_angmom"

# A foot's contact patch is measured from the vertices sitting within this band
# above the link's lowest point -- whichever of the two is larger, so a thin
# bracket end and a tall foot are both measured sensibly.
_PATCH_BAND_M = 5e-4        # 0.5 mm
_PATCH_BAND_FRACTION = 0.05  # or 5% of the link's own height
_MIN_FOOT_RADIUS = 1e-3     # 1 mm: below this a contact sphere is pointless

_SITE_MARKER_SIZE = "0.005"


def mjcf_pkg_name(robot_name, pkg_name=None):
    """Directory name for the exported MuJoCo package.

    Defaults to ``<robot_name>_mjcf``.  Unlike a ROS package name this only has
    to be a usable directory name, so the validation is just that: no separators
    and no relative-path games.
    """
    if pkg_name and pkg_name.strip():
        pkg = pkg_name.strip()
    else:
        base = re.sub(r"[^A-Za-z0-9_.-]", "_", robot_name).strip("_")
        pkg = (base or "robot") + "_mjcf"
    if pkg in (".", "..") or "/" in pkg or "\\" in pkg:
        raise ValueError(f"invalid MuJoCo package name: {pkg_name!r}")
    return pkg


def mjcf_model_stem(robot_name, mjcf_name=None):
    """Stem of the ``.xml`` inside the package (default: the robot's name)."""
    if not mjcf_name:
        base = re.sub(r"[^A-Za-z0-9_.-]", "_", robot_name).strip("_")
        return base or "robot"
    stem = mjcf_name.strip()
    if stem.lower().endswith(".xml"):
        stem = stem[:-4]
    if not stem or "/" in stem or "\\" in stem:
        raise ValueError(f"invalid MJCF name: {mjcf_name!r}")
    return stem


# --------------------------------------------------------------- MJCF geometry
def _floats(text, default):
    if not text:
        return np.asarray(default, dtype=float)
    return np.asarray([float(v) for v in text.split()], dtype=float)


def _quat_matrix(quat):
    """MuJoCo quaternion (w x y z) -> rotation matrix."""
    from skrobot.coordinates.math import quaternion2matrix
    return quaternion2matrix(np.asarray(quat, dtype=float))


def _axis_rotation(axis, angle):
    """Rotation of ``angle`` radians about ``axis`` (Rodrigues)."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(3)
    axis = axis / norm
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return (np.eye(3) + np.sin(angle) * skew
            + (1.0 - np.cos(angle)) * (skew @ skew))


def _world_frames(worldbody, home):
    """``{body name: (R, p)}`` in world coordinates at the ``home`` pose.

    The floating base sits at the origin: this is used to work out how high that
    base has to start for the feet to rest on the ground, so it must be measured
    from a base at zero.
    """
    frames = {}

    def walk(parent_el, parent_R, parent_p):
        for body in parent_el.findall("body"):
            rot = parent_R @ _quat_matrix(_floats(body.get("quat"), (1, 0, 0, 0)))
            pos = parent_p + parent_R @ _floats(body.get("pos"), (0, 0, 0))
            joint = body.find("joint")
            if joint is not None:
                angle = float(home.get(joint.get("name"), 0.0))
                axis = _floats(joint.get("axis"), (0, 0, 1))
                if joint.get("type", "hinge") == "slide":
                    pos = pos + rot @ (axis * angle)
                else:
                    rot = rot @ _axis_rotation(axis, angle)
            frames[body.get("name")] = (rot, pos)
            walk(body, rot, pos)

    walk(worldbody, np.eye(3), np.zeros(3))
    return frames


def _mesh_vertices(asset_dir, filename, scale=None):
    import trimesh
    mesh = trimesh.load(os.path.join(asset_dir, filename), process=False)
    vertices = np.asarray(mesh.vertices, dtype=float)
    if scale is not None:
        vertices = vertices * np.asarray(scale, dtype=float)
    return vertices


def _collision_points(body_el, frame, mesh_files, asset_dir):
    """World-frame vertices of every collision geom on ``body_el``."""
    body_R, body_p = frame
    points = []
    for geom in body_el.findall("geom"):
        if geom.get("group") != _COLLISION_GROUP:
            continue
        geom_R = _quat_matrix(_floats(geom.get("quat"), (1, 0, 0, 0)))
        geom_p = _floats(geom.get("pos"), (0, 0, 0))
        gtype = geom.get("type")
        if gtype == "mesh":
            entry = mesh_files.get(geom.get("mesh"))
            if entry is None:
                continue
            local = _mesh_vertices(asset_dir, entry[0], entry[1])
        elif gtype == "box":
            half = _floats(geom.get("size"), (0, 0, 0))
            local = np.array([[sx * half[0], sy * half[1], sz * half[2]]
                              for sx in (-1, 1) for sy in (-1, 1)
                              for sz in (-1, 1)], dtype=float)
        elif gtype == "sphere":
            radius = float(_floats(geom.get("size"), (0,))[0])
            local = np.array([[0.0, 0.0, -radius], [0.0, 0.0, radius]])
        elif gtype == "cylinder":
            size = _floats(geom.get("size"), (0, 0))
            radius, half_length = float(size[0]), float(size[1])
            local = np.array([[sx * radius, sy * radius, sz * half_length]
                              for sx in (-1, 1) for sy in (-1, 1)
                              for sz in (-1, 1)], dtype=float)
        else:
            continue
        if len(local) == 0:
            continue
        points.append((body_R @ (geom_R @ local.T + geom_p[:, None])).T + body_p)
    if not points:
        return np.zeros((0, 3))
    return np.vstack(points)


def _mesh_file_map(root):
    """``{asset name: (filename, scale or None)}`` from the MJCF ``<asset>``."""
    asset = root.find("asset")
    if asset is None:
        return {}
    out = {}
    for mesh in asset.findall("mesh"):
        scale = mesh.get("scale")
        out[mesh.get("name")] = (
            mesh.get("file"),
            None if scale is None else _floats(scale, (1, 1, 1)))
    return out


def _leaf_bodies(worldbody):
    """Bodies with no child body: the ends of every kinematic chain."""
    return [b for b in worldbody.iter("body") if b.find("body") is None]


def _contact_patch(points):
    """``(centre_xy, lowest_z, radius)`` of the ground-contact patch.

    The patch is the footprint of the vertices nearest the bottom of the link;
    the radius is half its narrower side, so the sphere fits inside the real
    contact face instead of bulging out of it.
    """
    if len(points) == 0:
        return None
    z_min = float(points[:, 2].min())
    z_max = float(points[:, 2].max())
    band = max(_PATCH_BAND_M, _PATCH_BAND_FRACTION * (z_max - z_min))
    bottom = points[points[:, 2] <= z_min + band]
    if len(bottom) == 0:
        return None
    lo = bottom[:, :2].min(axis=0)
    hi = bottom[:, :2].max(axis=0)
    radius = 0.5 * float(min(hi - lo))
    if radius < _MIN_FOOT_RADIUS:
        return None
    return (lo + hi) / 2.0, z_min, radius


# --------------------------------------------------------------- MJCF surgery
def _add_foot_contacts(root, frames, mesh_files, asset_dir, foot_bodies,
                       tags=None):
    """Add a named contact sphere and a site to each foot.

    The sphere is ``<tag>_toe`` and the site is ``<tag>``, where ``tag``
    defaults to the body's own name.  A task config selects contacts by these
    names, and its convention is often the LEG rather than the link -- a body
    called ``FL_foot`` wanting ``FL_toe`` and ``FL`` -- which no exporter can
    guess, so ``tags`` lets the caller say it.

    Returns ``{body name: (radius, lowest world z)}`` for the feet that got one.
    """
    tags = tags or {}
    added = {}
    for body in foot_bodies:
        name = body.get("name")
        tag = tags.get(name, name)
        frame = frames.get(name)
        if frame is None:
            continue
        points = _collision_points(body, frame, mesh_files, asset_dir)
        patch = _contact_patch(points)
        if patch is None:
            continue
        centre_xy, z_min, radius = patch
        body_R, body_p = frame
        centre_world = np.array([centre_xy[0], centre_xy[1], z_min + radius])
        touch_world = np.array([centre_xy[0], centre_xy[1], z_min])
        centre_local = body_R.T @ (centre_world - body_p)
        touch_local = body_R.T @ (touch_world - body_p)

        geom_name = f"{tag}_toe"
        if not any(g.get("name") == geom_name for g in body.findall("geom")):
            ET.SubElement(body, "geom", {
                "name": geom_name,
                "type": "sphere",
                "size": f"{radius:.9g}",
                "pos": " ".join(f"{v:.9g}" for v in centre_local),
                "group": _COLLISION_GROUP,
                "rgba": "0.1 0.1 0.1 1",
            })
        if not any(s.get("name") == tag for s in body.findall("site")):
            ET.SubElement(body, "site", {
                "name": tag,
                "pos": " ".join(f"{v:.9g}" for v in touch_local),
                "size": _SITE_MARKER_SIZE, "group": _SITE_GROUP,
                "rgba": "1 0 0 1",
            })
        added[tag] = (radius, z_min)
    return added


def _add_imu(root, base_body):
    """IMU site on the base plus the sensors a locomotion task reads by name."""
    if base_body is None:
        return False
    if not any(s.get("name") == _IMU_SITE for s in base_body.findall("site")):
        ET.SubElement(base_body, "site", {
            "name": _IMU_SITE, "pos": "0 0 0",
            "size": _SITE_MARKER_SIZE, "group": _SITE_GROUP,
            "rgba": "0 1 0 1",
        })
    sensor = root.find("sensor")
    if sensor is None:
        sensor = ET.SubElement(root, "sensor")
    existing = {s.get("name") for s in sensor}
    for tag, name in _SENSORS:
        if name not in existing:
            ET.SubElement(sensor, tag, {"name": name, "site": _IMU_SITE})
    if _ANGMOM_SENSOR not in existing:
        ET.SubElement(sensor, "subtreeangmom",
                      {"name": _ANGMOM_SENSOR, "body": base_body.get("name")})
    return True


def _base_body(worldbody):
    bodies = worldbody.findall("body")
    return bodies[0] if bodies else None


def _measured(root):
    """Read back what the converter derived: per-joint damping and base height.

    These are ``urdf_to_mjcf``'s doing (``joint_damping='backemf'`` and
    ``home_base_height='auto'``), so the numbers are taken from the model it
    wrote rather than recomputed here and risk disagreeing with it.
    """
    damping = {}
    for body in root.iter("body"):
        for joint in body.findall("joint"):
            if joint.get("damping"):
                damping[joint.get("name")] = float(joint.get("damping"))
    base_height = 0.0
    keyframe = root.find("keyframe")
    if keyframe is not None and next(root.iter("freejoint"), None) is not None:
        key = next((k for k in keyframe.findall("key")
                    if k.get("name") == "home"), None)
        if key is not None and key.get("qpos"):
            base_height = float(key.get("qpos").split()[2])
    return damping, base_height


def add_rl_extras(mjcf_path, *, home=None, foot_contacts=True,
                  imu_sensors=True, foot_links=None):
    """Add to a converted MJCF the parts that are specific to THIS robot.

    Everything derivable from the URDF alone -- servo damping, the home
    keyframe and its base height -- is the converter's job and is already done
    by the time this runs.  What is left needs to know what the links MEAN: which
    of them are feet, and where the IMU goes.

    A contact sphere is tangent to its foot's lowest point, so it never reaches
    below the geometry the base height was measured against and the home pose
    still rests exactly on the floor.

    Returns a report dict describing the model, so callers (and the tests) can
    see the derived numbers instead of having to re-measure them.
    """
    home = dict(home or {})
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{mjcf_path} has no <worldbody>")

    asset_dir = os.path.join(os.path.dirname(os.path.abspath(mjcf_path)),
                             _meshdir(root))
    mesh_files = _mesh_file_map(root)
    frames = _world_frames(worldbody, home)

    damping, base_height = _measured(root)
    report = {"damping": damping, "feet": {}, "imu": False,
              "base_height": base_height}

    if foot_contacts:
        tags = dict(foot_links) if isinstance(foot_links, dict) else {}
        if foot_links:
            wanted = set(foot_links)
            bodies = [b for b in worldbody.iter("body")
                      if b.get("name") in wanted]
        else:
            bodies = _leaf_bodies(worldbody)
        report["feet"] = _add_foot_contacts(root, frames, mesh_files,
                                            asset_dir, bodies, tags=tags)

    if imu_sensors:
        report["imu"] = _add_imu(root, _base_body(worldbody))

    ET.indent(tree, space="  ")
    tree.write(mjcf_path, encoding="unicode", xml_declaration=False)
    return report


def _meshdir(root):
    compiler = root.find("compiler")
    if compiler is None:
        return "assets"
    return compiler.get("meshdir") or "assets"


def _set_model_name(mjcf_path, robot_name):
    """Name the MJCF after the assembly, not after the staging URDF's stem."""
    tree = ET.parse(mjcf_path)
    tree.getroot().set("model", robot_name)
    tree.write(mjcf_path, encoding="unicode", xml_declaration=False)


# ------------------------------------------------------------------- the export
_README = """# {pkg}

MuJoCo model exported by sw2robot from the SolidWorks assembly `{robot}`.

    mjcf/{stem}.xml   the model
    mjcf/assets/      its binary-STL mesh assets

Load it with:

    import mujoco
    model = mujoco.MjModel.from_xml_path("mjcf/{stem}.xml")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)   # the "home" keyframe

What the exporter derived from the CAD, rather than guessed:

{notes}
"""


def write_mjcf_package(pkg_dir, robot_name, dest_dir, *, pkg_name=None,
                       mjcf_name=None, collision="copy",
                       coacd_quality="balanced", merge_fixed=True, colors=None,
                       loop_closures=None, floating_base=True,
                       actuator="position", actuator_kp=50.0, actuator_kv=1.0,
                       armature=0.0, self_collision=False, add_ground=True,
                       backemf_damping=True, foot_contacts=None,
                       foot_links=None, imu_sensors=None, home=None,
                       progress=None):
    """Write a MuJoCo package under ``dest_dir`` and return its directory.

    Parameters
    ----------
    pkg_dir : str
        A built sw2robot package directory (the one holding ``urdf/`` and
        ``meshes/``), the same input :func:`write_ros_description_package`
        takes.
    robot_name : str
        The robot/URDF name inside ``pkg_dir``.
    dest_dir : str
        Where the ``<robot>_mjcf`` directory is created.
    collision, coacd_quality, merge_fixed, colors, loop_closures
        Passed through to the URDF rebuild; see
        :func:`sw2robot.exporter.ros_export.build_ros_description`.
        ``merge_fixed`` defaults to True here because MuJoCo has no use for one
        body per screw.
    floating_base : bool
        Give the base a free joint.  On for a legged robot; turn it off for an
        arm bolted to a table.
    actuator : {'position', 'velocity', 'motor'}
        Actuator type emitted per joint.
    actuator_kp, actuator_kv : float
        Gains for the position / velocity actuators.
    armature : float
        Rotor inertia reflected through the gearbox, added to every joint.  A
        CAD model cannot supply this -- it depends on the motor and gear ratio
        -- so it defaults to 0 and is left to the caller.
    self_collision : bool
        Keep full self-collision.  Off by default: neighbouring links of a CAD
        assembly touch by construction, which blows up at spawn time.
    backemf_damping : bool
        Derive each joint's damping from its own ``effort / velocity`` limits.
        Turn it OFF when the consumer supplies its own actuator model (mjlab's
        ``viscous_damping``, for one) -- otherwise both apply and the joint is
        damped twice.
    foot_contacts : bool or None
        Add a contact sphere + site to each chain end.  Defaults to
        ``floating_base``: it is a legged-robot thing.
    foot_links : list of str, dict, or None
        Use exactly these links as feet instead of auto-detecting chain ends.
        A dict maps each link to the name its contact sphere and site take:
        ``{'FL_foot': 'FL'}`` emits ``FL_toe`` and ``FL``, which is the
        convention locomotion task configs usually match on.
    imu_sensors : bool or None
        Add the IMU site and named sensors.  Defaults to ``floating_base``.
    home : dict or None
        ``{joint name: angle}`` for the ``home`` keyframe; missing joints are 0.
        The base height in that keyframe is computed so nothing starts below the
        floor.

    Returns
    -------
    str
        Path to the written package directory.
    """
    pkg = mjcf_pkg_name(robot_name, pkg_name)
    out_root = os.path.join(os.path.abspath(dest_dir), pkg)
    _produce_mjcf_package(
        pkg_dir, robot_name, out_root, pkg=pkg, mjcf_name=mjcf_name,
        collision=collision, coacd_quality=coacd_quality,
        merge_fixed=merge_fixed, colors=colors, loop_closures=loop_closures,
        floating_base=floating_base, actuator=actuator,
        actuator_kp=actuator_kp, actuator_kv=actuator_kv, armature=armature,
        self_collision=self_collision, add_ground=add_ground,
        backemf_damping=backemf_damping, foot_contacts=foot_contacts,
        foot_links=foot_links, imu_sensors=imu_sensors, home=home,
        progress=progress)
    return out_root


def build_mjcf_package(pkg_dir, robot_name, *, pkg_name=None, **kwargs):
    """The same package as :func:`write_mjcf_package`, as in-memory files.

    Returns ``(pkg, [(arcname, bytes), ...])`` so a caller that ships a download
    -- the web editor -- does not have to invent a directory to write into
    first.  ``arcname`` is package-relative-with-prefix, matching what
    :func:`sw2robot.exporter.ros_export.build_ros_description` returns.
    """
    pkg = mjcf_pkg_name(robot_name, pkg_name)
    out_dir = tempfile.mkdtemp(prefix="sw2robot-mjcf-out-")
    try:
        out_root = os.path.join(out_dir, pkg)
        _produce_mjcf_package(pkg_dir, robot_name, out_root, pkg=pkg, **kwargs)
        files = []
        for dirpath, _dirs, names in os.walk(out_root):
            for name in sorted(names):
                path = os.path.join(dirpath, name)
                arc = os.path.relpath(path, out_dir).replace(os.sep, "/")
                with open(path, "rb") as f:
                    files.append((arc, f.read()))
        return pkg, files
    finally:
        shutil.rmtree(out_dir, ignore_errors=True)


def _produce_mjcf_package(pkg_dir, robot_name, out_root, *, pkg,
                          mjcf_name=None, collision="copy",
                          coacd_quality="balanced", merge_fixed=True,
                          colors=None, loop_closures=None, floating_base=True,
                          actuator="position", actuator_kp=50.0,
                          actuator_kv=1.0, armature=0.0, self_collision=False,
                          add_ground=True, backemf_damping=True,
                          foot_contacts=None, foot_links=None,
                          imu_sensors=None, home=None, progress=None):
    """Fill ``out_root`` with the MuJoCo package; return the extras report."""
    from skrobot.urdf import urdf_to_mjcf

    if foot_contacts is None:
        foot_contacts = floating_base
    if imu_sensors is None:
        imu_sensors = floating_base

    stem = mjcf_model_stem(robot_name, mjcf_name)
    mjcf_dir = os.path.join(out_root, "mjcf")
    asset_dir = os.path.join(mjcf_dir, "assets")
    mjcf_path = os.path.join(mjcf_dir, stem + ".xml")

    def _report(stage, detail=""):
        if progress is not None:
            progress(stage, detail)

    # 1. rebuild the URDF the way MuJoCo wants it: fixed links merged, origins
    #    baked, meshes in a format trimesh can read.  This is the ROS export's
    #    job and it already does it well, so it runs into a scratch directory
    #    rather than being reimplemented here.
    _report("urdf", "rebuilding URDF for MuJoCo")
    staging = tempfile.mkdtemp(prefix="sw2robot-mjcf-")
    try:
        # the staging package's own name never reaches the output, so it is
        # left at the ROS default rather than derived from a MuJoCo package
        # name that need not be a legal ROS one
        ros_pkg = write_ros_description_package(
            pkg_dir, robot_name, staging, colors=colors,
            collision=collision, coacd_quality=coacd_quality,
            merge_fixed=merge_fixed, loop_closures=loop_closures,
            zero_origins=True)
        urdf_dir = os.path.join(ros_pkg, "urdf")
        urdfs = sorted(f for f in os.listdir(urdf_dir) if f.endswith(".urdf"))
        if not urdfs:
            raise RuntimeError(f"no URDF was written under {urdf_dir}")
        urdf_path = os.path.join(urdf_dir, urdfs[0])

        # 2. URDF -> MJCF, with the two things the converter can derive from
        #    that URDF on its own: each joint's back-EMF damping, and the base
        #    height at which the home pose rests on the floor.  The collision
        #    meshes are already the ones `collision` asked for, CoACD parts
        #    included, so it must not decompose them a second time.
        _report("mjcf", "converting to MJCF")
        os.makedirs(asset_dir, exist_ok=True)
        urdf_to_mjcf(
            urdf_path, mjcf_path, mesh_dir=asset_dir,
            floating_base=floating_base,
            add_position_actuators=True, actuator_type=actuator,
            actuator_kp=actuator_kp, actuator_kv=actuator_kv,
            add_actuator_forcerange=True,
            joint_armature=armature,
            joint_damping="backemf" if backemf_damping else 0.0,
            add_ground=add_ground, self_collision=self_collision,
            home=dict(home or {}), home_base_height="auto",
            convex_decompose_collision=False)
    finally:
        shutil.rmtree(staging, ignore_errors=True)

    _set_model_name(mjcf_path, robot_name)

    # 3. the parts that need to know what the links MEAN: which are feet, and
    #    where the IMU goes.
    _report("extras", "adding foot contacts and sensors")
    report = add_rl_extras(
        mjcf_path, home=home, foot_contacts=foot_contacts,
        foot_links=foot_links, imu_sensors=imu_sensors)

    with open(os.path.join(out_root, "README.md"), "w", encoding="utf-8") as f:
        f.write(_README.format(pkg=pkg, robot=robot_name, stem=stem,
                               notes=_readme_notes(report)))
    return report


def _readme_notes(report):
    lines = []
    damping = report.get("damping") or {}
    if damping:
        # a robot built from identical servos gets one value on every joint, so
        # grouping keeps this to a line instead of a wall of joint names
        by_value = {}
        for name, value in damping.items():
            by_value.setdefault(round(value, 9), []).append(name)
        lines.append(
            "* joint damping from each joint's own `effort / velocity` "
            "(N*m*s/rad): "
            + ", ".join("{:.4g} on {} joint{}".format(
                value, len(names), "" if len(names) == 1 else "s")
                for value, names in sorted(by_value.items())))
    feet = report.get("feet") or {}
    if feet:
        lines.append("* contact spheres sized from each foot's own contact "
                     "patch: " + ", ".join(
                         f"`{k}_toe` r={v[0]:.4g} m"
                         for k, v in sorted(feet.items())))
    if report.get("imu"):
        lines.append("* IMU site `{}` and sensors `{}`, `{}`".format(
            _IMU_SITE, "`, `".join(n for _t, n in _SENSORS), _ANGMOM_SENSOR))
    lines.append("* `home` keyframe with the base at {:.4g} m, the height at "
                 "which nothing is below the floor".format(
                     report.get("base_height", 0.0)))
    return "\n".join(lines)
