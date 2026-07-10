"""Core API for the CAD -> robot-compiler bridge (UI-independent).

Every function here is callable from a CLI, the web editor, or a FastAPI
endpoint with no change -- that is the whole point (see the package docstring).

Pipeline:
    import_module()   #3  CAD package (sw2robot.exporter, no SolidWorks) -> RobotCompilerState
    load_module()     #3  an already-built URDF -> state (no rebuild)
    register_module() #3  drop the module into a robot-compiler registry dir
    rename_joint() / set_limits() / set_mimic() / set_servo() /
    set_axis_flip() / set_joint_type()                         interactive edits
    validate()            list the problems in the current state
    export_ros_package()  #4  state (+edits) -> ROS/MoveIt/Gazebo config ZIP

Hard-invalid edits (rename collision, mimic to a non-chain joint, bad servo id,
bad joint type) raise ``ValueError`` at the setter so a script fails fast; soft
issues (lower>=upper, duplicate servo ids) are surfaced by ``validate()``.
"""

from __future__ import annotations

import math
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from .state import RobotCompilerState

__all__ = [
    "SERVO_PROFILES",
    "apply_servo_profile",
    "build_urdf",
    "chain_related",
    "export_ros_package",
    "extract_and_import",
    "import_module",
    "load_edits",
    "load_module",
    "load_state",
    "movable_names",
    "register_module",
    "rename_joint",
    "reverse_direction",
    "save_state",
    "set_actuator",
    "set_axis_flip",
    "set_color",
    "set_inertial",
    "set_joint_type",
    "set_limits",
    "set_mass_only",
    "set_mimic",
    "set_servo",
    "state_path",
    "sw_recent_assemblies",
    "sw_session_status",
    "validate",
]

JOINT_TYPES = ("fixed", "revolute", "continuous", "prismatic")
_SAFE_NAME = re.compile(r"^[A-Za-z_][0-9A-Za-z_]*$")


# --------------------------------------------------------------- topology
def movable_names(state: RobotCompilerState) -> list:
    return [j["name"] for j in state.movable_joints()]


def _link_parents(state: RobotCompilerState) -> dict:
    return {jj["childLink"]: jj["parentLink"] for jj in state.joints}


def _link_ancestors(link, link_parents) -> set:
    out, cur = set(), link
    while cur in link_parents:
        cur = link_parents[cur]
        out.add(cur)
    return out


def chain_related(state: RobotCompilerState, joint: str) -> list:
    """Movable joints that are an ancestor OR descendant of ``joint`` in the
    link tree (same kinematic chain) -- the only sensible mimic drivers."""
    by = {j["name"]: j for j in state.movable_joints()}
    if joint not in by:
        return []
    lp = _link_parents(state)
    a = by[joint]["childLink"]
    rel = []
    for k in state.movable_joints():
        if k["name"] == joint:
            continue
        b = k["childLink"]
        if a in _link_ancestors(b, lp) or b in _link_ancestors(a, lp):
            rel.append(k["name"])
    return rel


# --------------------------------------------------------------- persistence
def state_path(state: RobotCompilerState, path=None) -> Path:
    """Default sidecar for an interactive session's edits:
    ``<package_dir>/<robot_name>.sw2robot.json``."""
    if path:
        return Path(path)
    return Path(state.package_dir) / f"{state.robot_name}.sw2robot.json"


def save_state(state: RobotCompilerState, path=None) -> Path:
    """Write the full state (joints + edit overlay) to a JSON sidecar so an
    interactive session survives a restart."""
    p = state_path(state, path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(state.model_dump_json(indent=2), encoding="utf-8")
    return p


def load_state(path) -> RobotCompilerState:
    """Load a complete state previously written by :func:`save_state`."""
    return RobotCompilerState.model_validate_json(
        Path(path).read_text(encoding="utf-8"))


def load_edits(state: RobotCompilerState, path=None) -> int:
    """Merge a saved edit overlay into ``state`` IN PLACE, keeping ``state``'s
    freshly-built joints/URDF.  Only edits whose joint/link still exists are
    applied (so a rebuilt module with changed names is handled gracefully).
    Returns the number of edits applied; 0 if no sidecar exists."""
    p = state_path(state, path)
    if not p.is_file():
        return 0
    saved = load_state(p)
    valid = {j["name"] for j in state.joints}
    n = 0
    for jn, edit in saved.edits.items():
        if jn in valid:
            state.edits[jn] = edit
            n += 1
    valid_links = {l["name"] for l in state.links}
    for ln, ledit in saved.link_edits.items():
        if ln in valid_links:
            state.link_edits[ln] = ledit
            n += 1
    return n


# --------------------------------------------------------------- #3 import
def load_module(urdf_path, package_dir=None) -> RobotCompilerState:
    """Parse an already-built URDF into a state -- NO sw2robot.exporter rebuild.

    ``package_dir`` (for meshes / registration) defaults to the URDF's
    grandparent (the ``<pkg>/urdf/<name>.urdf`` layout)."""
    from ._vendor.rc_config.urdf_parser import parse_urdf_content

    urdf_path = Path(urdf_path)
    parsed = parse_urdf_content(urdf_path.read_text(encoding="utf-8"))
    pkg = Path(package_dir) if package_dir else urdf_path.parents[1]
    return RobotCompilerState(
        robot_name=urdf_path.stem, urdf_path=str(urdf_path), package_dir=str(pkg),
        joints=parsed["joints"], links=parsed["links"], root_link=parsed["root_link"])


def import_module(package_dir, config_path=None, base_hint=None,
                  exclude=None) -> RobotCompilerState:
    """Build the CAD module headlessly (sw2robot.exporter build half, NO
    SolidWorks) from a cached ``graph.json`` package, then load it into a
    state."""
    from sw2robot.exporter.export import build

    urdf_path = build(package_dir, config_path=config_path,
                      base_hint=base_hint, exclude=exclude)
    return load_module(urdf_path, package_dir=package_dir)


def extract_and_import(assembly_path, out_dir=None, robot_name=None,
                       base_hint=None, config_path=None,
                       visible=False, progress=None,
                       sw=None) -> RobotCompilerState:
    """Drive SolidWorks to extract a live assembly, then build it into a state.

    This is the *whole* CAD->state pipeline in one call (the slow ``extract``
    half PLUS the fast ``build`` half), so the web editor can offer a single
    "Import from SolidWorks" button.  ``extract`` opens a throwaway COPY of
    ``assembly_path`` in its own SolidWorks instance and never touches the
    user's original file (see :class:`sw2robot.exporter.swcom.SolidWorks`).

    SolidWorks/pywin32 are Windows-only and imported lazily, so importing this
    module stays cheap and OS-agnostic (the headless ``build`` path never needs
    them).  ``progress`` -- if given -- is called with short status strings so a
    UI can show what the (multi-minute) extract is doing.
    """
    from sw2robot.exporter.export import extract

    if progress:
        progress("reusing the warm SolidWorks session ..." if sw is not None
                 else "starting SolidWorks (this can take a minute) ...")
    pkg_dir = extract(assembly_path, out_dir=out_dir, robot_name=robot_name,
                      visible=visible, progress=progress, sw=sw)
    # A re-extract regenerates graph.json but extract() leaves the existing
    # <name>.joints.yaml in place.  Without a config the build would take the
    # auto path and OVERWRITE that file -- silently dropping every edit the user
    # made (joint types, axes, renames, base, and the travel range of a joint
    # with no SolidWorks limit mate, which lives ONLY here).  So reuse it as the
    # config: re-extracting refreshes the geometry while keeping the user's work.
    if config_path is None:
        existing = sorted(Path(pkg_dir).glob("*.joints.yaml"))
        if existing:
            config_path = str(existing[0])
            if progress:
                progress(f"reusing your joint config {existing[0].name} "
                         f"(keeps types/limits/renames across the re-extract)")
    if progress:
        progress("building URDF from the CAD graph ...")
    return import_module(pkg_dir, config_path=config_path, base_hint=base_hint)


def sw_session_status() -> dict:
    """READ-ONLY snapshot of the user's SolidWorks session, for UI guidance.

    Returns ``{running, instances, attachable, active_doc, active_assembly,
    dirty}``.  Never saves, closes or modifies anything.

    Empirics on this machine: the user's interactive SolidWorks registers in
    the COM Running Object Table only for processes in the SAME login session
    -- ``attachable`` is therefore True when this server was started from the
    user's own terminal and False from service-like shells, even while
    SolidWorks is visibly running.  ``dirty`` (unsaved changes) is only
    knowable when attachable; extraction always reads the SAVED file on disk,
    so a dirty active document means the user should save first (themselves;
    this tool never saves)."""
    st = {"running": False, "instances": 0, "attachable": False,
          "active_doc": None, "active_assembly": None, "dirty": None}
    try:
        import subprocess
        out = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq SLDWORKS.exe", "/FO", "CSV",
             "/NH"], capture_output=True, text=True, timeout=15).stdout
        st["instances"] = out.count("SLDWORKS.exe")
        st["running"] = st["instances"] > 0
    except Exception:
        pass
    if not st["running"]:
        return st
    try:
        from sw2robot.exporter.swcom import SolidWorks, safe_prop
        sw = SolidWorks(attach=True)
    except Exception:
        return st          # running but not attachable from this process
    st["attachable"] = True
    try:
        doc = safe_prop(sw.app, "ActiveDoc")
        if doc is not None:
            path = safe_prop(doc, "GetPathName") or safe_prop(doc, "GetTitle")
            st["active_doc"] = path
            if str(path or "").lower().endswith(".sldasm"):
                st["active_assembly"] = path
            flag = safe_prop(doc, "GetSaveFlag")
            st["dirty"] = bool(flag) if flag is not None else None
    finally:
        sw.shutdown()      # attach mode: drops the reference, touches nothing
    return st


def sw_recent_assemblies(limit: int = 10) -> list:
    """Best-effort list of recently-opened ``.sldasm``/``.sldprt`` paths from the
    SolidWorks MRU registry key, newest first -- used only to pre-fill / suggest
    an import path (an approximation of "the CAD file you have open").  Single
    parts are included because a lone ``.SLDPRT`` exports as a 1-link URDF too.
    Returns ``[]`` on any failure (non-Windows, no SolidWorks, key absent)."""
    try:
        import winreg
    except ImportError:
        return []
    out = []
    base = r"Software\SolidWorks"
    try:
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, base) as root:
            versions = []
            i = 0
            while True:
                try:
                    name = winreg.EnumKey(root, i)
                except OSError:
                    break
                i += 1
                if name.upper().startswith("SOLIDWORKS "):
                    versions.append(name)
    except OSError:
        return []
    # Newest SolidWorks version first (e.g. "SOLIDWORKS 2025" > "2024").  The MRU
    # lives under "<ver>\Recent File List" as values File1, File2, ... (full
    # paths, parts AND assemblies mixed); keep the .sldasm/.sldprt ones in
    # File-number order so File1 (most-recently-opened) is first.
    for ver in sorted(versions, reverse=True):
        sub = rf"{base}\{ver}\Recent File List"
        items = []
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, sub) as key:
                j = 0
                while True:
                    try:
                        vname, val, _t = winreg.EnumValue(key, j)
                    except OSError:
                        break
                    j += 1
                    if isinstance(val, str) and val.lower().endswith(
                            (".sldasm", ".sldprt")):
                        items.append((vname, val))
        except OSError:
            continue

        def _rank(item):
            digits = "".join(ch for ch in item[0] if ch.isdigit())
            return int(digits) if digits else 1 << 30
        for _vname, val in sorted(items, key=_rank):
            if val not in out:
                out.append(val)
        if out:
            break
    return out[:limit]


def register_module(state: RobotCompilerState, registry_dir) -> Path:
    """Copy the module (URDF + meshes) into a robot-compiler module registry dir
    so ``ModuleRegistry.scan()`` / the server picks it up."""
    registry_dir = Path(registry_dir)
    urdf_dir, meshes_dir = registry_dir / "urdf", registry_dir / "meshes"
    urdf_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir.mkdir(parents=True, exist_ok=True)
    src_meshes = Path(state.package_dir) / "meshes"
    if src_meshes.is_dir():
        for f in src_meshes.iterdir():
            if f.is_file():
                shutil.copy2(f, meshes_dir / f.name)
    dst_urdf = urdf_dir / f"{state.robot_name}.urdf"
    shutil.copy2(state.urdf_path, dst_urdf)
    return dst_urdf


# --------------------------------------------------------------- edits
def _require_joint(state, joint):
    if joint not in {j["name"] for j in state.joints}:
        raise ValueError(f"no such joint: {joint}")


def _require_link(state, link):
    if link not in {l["name"] for l in state.links}:
        raise ValueError(f"no such link: {link}")


def rename_joint(state: RobotCompilerState, joint: str, new_name: str) -> None:
    _require_joint(state, joint)
    new_name = (new_name or "").strip()
    if not _SAFE_NAME.match(new_name):
        raise ValueError(f"invalid joint name '{new_name}' "
                         f"(letters/digits/underscore, not starting with a digit)")
    for other in (j["name"] for j in state.joints):
        if other != joint and state.effective_name(other) == new_name:
            raise ValueError(f"name '{new_name}' already used by '{other}'")
    state.edit_for(joint).rename = new_name


def set_limits(state: RobotCompilerState, joint: str, lower: float, upper: float) -> None:
    # soft: lower>=upper is allowed transiently (GUI editing) but flagged by
    # validate(); raising here would thrash the two-field GUI editor.
    _require_joint(state, joint)
    e = state.edit_for(joint)
    e.lower, e.upper = float(lower), float(upper)


def set_mimic(state: RobotCompilerState, joint: str, mimic_joint: str,
              multiplier: float = 1.0, offset: float = 0.0) -> None:
    _require_joint(state, joint)
    if mimic_joint not in set(movable_names(state)):
        raise ValueError(f"mimic target '{mimic_joint}' is not a movable joint")
    if mimic_joint == joint:
        raise ValueError("a joint cannot mimic itself")
    # NOTE: no same-chain restriction -- the parent/child relationship is not
    # always reliably known from CAD, so any movable joint may be a driver.
    e = state.edit_for(joint)
    e.mimic_joint, e.mimic_multiplier, e.mimic_offset = mimic_joint, multiplier, offset


def clear_mimic(state: RobotCompilerState, joint: str) -> None:
    _require_joint(state, joint)
    state.edit_for(joint).mimic_joint = None


def set_servo(state: RobotCompilerState, joint: str, servo_id: int,
              direction: int = 1, angle_offset: float = 0.0) -> None:
    _require_joint(state, joint)
    if not (0 <= int(servo_id) <= 255):
        raise ValueError(f"servo_id {servo_id} out of range 0..255")
    if direction not in (1, -1):
        raise ValueError("direction must be +1 or -1")
    e = state.edit_for(joint)
    e.servo_id, e.direction, e.angle_offset = int(servo_id), direction, float(angle_offset)


# Servo profiles: model -> actuator + range defaults.
# effort N*m (stall torque), velocity rad/s (no-load), range rad.
# HLS3606M: 6 kg*cm @6V -> 0.588 N*m; 0.09 s/60deg @6V -> ~11.6 rad/s; 360deg.
SERVO_PROFILES = {
    "HLS3606M": {"effort": 0.588, "velocity": 11.6, "lower": -3.14159, "upper": 3.14159},
}


def set_actuator(state: RobotCompilerState, joint: str,
                 effort: float | None = None,
                 velocity: float | None = None) -> None:
    """Set the joint's effort (N*m) and/or velocity (rad/s) limits."""
    _require_joint(state, joint)
    e = state.edit_for(joint)
    if effort is not None:
        e.effort = float(effort)
    if velocity is not None:
        e.velocity = float(velocity)


# Joint-physics overlay fields (JointEdit attr -> None clears it). Grouped by
# the URDF element they populate; used by set_joint_physics and build_urdf.
_PHYSICS_FIELDS = (
    "damping", "friction",
    "soft_lower_limit", "soft_upper_limit", "k_position", "k_velocity",
    "cal_rising", "cal_falling",
)


def set_joint_physics(state: RobotCompilerState, joint: str, **fields) -> None:
    """Set optional URDF joint physics on the overlay -- ``<dynamics>`` (damping,
    friction), ``<safety_controller>`` (soft_lower_limit, soft_upper_limit,
    k_position, k_velocity) and ``<calibration>`` (cal_rising, cal_falling).

    Pass a field as ``None`` to clear it (drop that attribute); omit it to leave
    it unchanged.  Unknown keys are ignored."""
    _require_joint(state, joint)
    e = state.edit_for(joint)
    for key, val in fields.items():
        if key not in _PHYSICS_FIELDS:
            continue
        setattr(e, key, None if val is None else float(val))


def apply_servo_profile(state: RobotCompilerState, joint: str,
                        model: str | None) -> dict | None:
    """Record the servo model and, if known, auto-fill effort / velocity / range
    from its profile.  Returns the applied profile (or None)."""
    _require_joint(state, joint)
    e = state.edit_for(joint)
    e.servo_model = model or None
    p = SERVO_PROFILES.get(model or "")
    if p:
        e.effort = p["effort"]
        e.velocity = p["velocity"]
        e.lower = p["lower"]
        e.upper = p["upper"]
    return p


def set_axis_flip(state: RobotCompilerState, joint: str, flip: bool = True) -> None:
    _require_joint(state, joint)
    state.edit_for(joint).flip_axis = bool(flip)


def set_joint_type(state: RobotCompilerState, joint: str, jtype: str) -> None:
    _require_joint(state, joint)
    if jtype not in JOINT_TYPES:
        raise ValueError(f"invalid joint type '{jtype}' (one of {JOINT_TYPES})")
    state.edit_for(joint).jtype = jtype


def reverse_direction(state: RobotCompilerState, joint: str) -> None:
    """Reverse the joint's rotation sense: flip the axis AND remap the limits to
    ``[-upper, -lower]`` (negate+swap, so the range stays valid).  The physical
    motion range is preserved; the sign of the joint command is inverted."""
    _require_joint(state, joint)
    e = state.edit_for(joint)
    parsed = next((j for j in state.joints if j["name"] == joint), {})
    e.flip_axis = not e.flip_axis
    # only revolute/prismatic joints carry a travel range -- don't fabricate a
    # degenerate [0, 0] limit on a continuous/fixed joint that has none
    if state.effective_type(parsed) in ("revolute", "prismatic"):
        lo = e.lower if e.lower is not None else parsed.get("lowerLimit", 0.0)
        hi = e.upper if e.upper is not None else parsed.get("upperLimit", 0.0)
        e.lower, e.upper = -hi, -lo


# --------------------------------------------------------------- link edits
_HEX6 = re.compile(r"[0-9a-f]{6}")


def set_color(state: RobotCompilerState, link: str, color: str | None) -> None:
    """Set a link's visual colour as ``#RRGGBB`` (baked into the URDF
    ``<visual><material>`` by :func:`build_urdf`).  ``color=None`` / ``''``
    clears the override (keeping whatever the base URDF carries)."""
    _require_link(state, link)
    if not color:
        if link in state.link_edits:     # clear without creating a stray entry
            state.link_edits[link].color = None
        return
    h = str(color).strip().lstrip("#").lower()
    if not _HEX6.fullmatch(h):
        raise ValueError(f"invalid color {color!r}: want #RRGGBB")
    state.link_edit_for(link).color = "#" + h


def set_mass_only(state: RobotCompilerState, link: str, on: bool = True) -> None:
    """Mark a link mass-only: :func:`build_urdf` drops its visual/collision and
    folds its inertial into the fixed parent (weight kept, geometry gone).
    ``on=False`` clears it (without leaving a stray overlay entry)."""
    _require_link(state, link)
    if on:
        state.link_edit_for(link).mass_only = True
    elif link in state.link_edits:
        state.link_edits[link].mass_only = False


def set_inertial(state: RobotCompilerState, link: str,
                 mass: float | None = None,
                 com: list | None = None,
                 inertia: list | None = None) -> None:
    """Override a link's inertial properties (any subset).  ``mass`` kg (> 0),
    ``com`` the inertial-origin ``[x, y, z]``, ``inertia`` the tensor
    ``[ixx, ixy, ixz, iyy, iyz, izz]``.  Unset fields keep the base URDF value;
    pass a value to change just that one."""
    _require_link(state, link)
    # validate EVERYTHING before touching state, so a rejected request leaves no
    # partial override behind (matches the fail-fast contract of the setters)
    if mass is not None:
        mass = float(mass)
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError(f"mass must be a finite number > 0 (got {mass})")
    if com is not None:
        com = [float(x) for x in com]
        if len(com) != 3:
            raise ValueError("com must be [x, y, z]")
        if not all(math.isfinite(x) for x in com):
            raise ValueError(f"com must be finite (got {com})")
    if inertia is not None:
        inertia = [float(x) for x in inertia]
        if len(inertia) != 6:
            raise ValueError("inertia must be [ixx, ixy, ixz, iyy, iyz, izz]")
        # physics sanity (positive-definite + triangle inequality): the same
        # check the exporter applies to generated inertials, so a hand-entered
        # tensor can't export a simulator-breaking link.  The tensor checks are
        # mass-independent, so use a positive placeholder when mass is unset here.
        from sw2robot.exporter.inertia import validate_inertia
        probs = validate_inertia(mass if mass is not None else 1.0, inertia)
        if probs:
            raise ValueError("invalid inertia: " + "; ".join(probs))
    e = state.link_edit_for(link)
    if mass is not None:
        e.mass = mass
    if com is not None:
        e.com = com
    if inertia is not None:
        e.inertia = inertia


# --------------------------------------------------------------- validation
def validate(state: RobotCompilerState) -> list:
    """Return a list of human-readable problems with the current state.  Empty
    list = the module will export to a well-formed URDF."""
    problems = []
    movable = set(movable_names(state))

    # duplicate effective joint names
    seen = {}
    for j in state.joints:
        seen.setdefault(state.effective_name(j["name"]), []).append(j["name"])
    for name, origs in seen.items():
        if len(origs) > 1:
            problems.append(f"duplicate joint name '{name}' ({', '.join(origs)})")

    servo_ids = {}
    for orig, e in state.edits.items():
        if e.lower is not None and e.upper is not None and e.lower >= e.upper:
            problems.append(f"{orig}: lower >= upper ({e.lower} >= {e.upper})")
        if e.servo_id is not None:
            if not (0 <= e.servo_id <= 255):
                problems.append(f"{orig}: servo_id {e.servo_id} out of range 0..255")
            servo_ids.setdefault(e.servo_id, []).append(orig)
        if e.mimic_joint and e.mimic_joint not in movable:
            problems.append(f"{orig}: mimic target '{e.mimic_joint}' is not a movable joint")
        if e.jtype and e.jtype not in JOINT_TYPES:
            problems.append(f"{orig}: invalid joint type '{e.jtype}'")
    for sid, origs in servo_ids.items():
        if len(origs) > 1:
            problems.append(f"servo_id {sid} used by multiple joints ({', '.join(origs)})")
    return problems


# --------------------------------------------------------------- #4 export
def _set_xyz(elem, vec):
    elem.set("xyz", " ".join(f"{v:.8g}" for v in vec))


def _hex_to_rgba(hex_color: str) -> str:
    """``#RRGGBB`` -> a URDF ``rgba`` string with opaque alpha."""
    h = hex_color.lstrip("#")
    r, g, b = (int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    return f"{r:.6g} {g:.6g} {b:.6g} 1"


def _apply_color(link_elem, hex_color: str) -> None:
    """Set ``<material><color rgba>`` on every ``<visual>`` of the link (creating
    the material/color when absent) -- URDF's per-visual colour model.  A no-op
    when the link has no ``<visual>`` (nothing to paint)."""
    rgba = _hex_to_rgba(hex_color)
    mat_name = "color_" + hex_color.lstrip("#")
    for vis in link_elem.findall("visual"):
        mat = vis.find("material")
        if mat is None:
            mat = ET.SubElement(vis, "material")
        mat.set("name", mat_name)
        col = mat.find("color")
        if col is None:
            col = ET.SubElement(mat, "color")
        col.set("rgba", rgba)


def _apply_inertial(link_elem, mass, com, inertia) -> None:
    """Override the subset of ``<inertial>`` (mass / origin xyz / inertia tensor)
    that is not None, creating the ``<inertial>`` and its children as needed.

    A partial edit on a link that has NO ``<inertial>`` would otherwise emit an
    invalid block (URDF requires both ``<mass>`` and ``<inertia>``), so when the
    block is freshly created any field this edit did not supply is backfilled
    with the same neutral placeholder the exporter uses (mass 0.1 kg, diagonal
    inertia 1e-4)."""
    ine = link_elem.find("inertial")
    created = ine is None
    if created:
        ine = ET.SubElement(link_elem, "inertial")
    if com is not None:
        org = ine.find("origin")
        if org is None:
            org = ET.SubElement(ine, "origin")
            org.set("rpy", "0 0 0")
        _set_xyz(org, com)
    if mass is not None:
        m = ine.find("mass")
        if m is None:
            m = ET.SubElement(ine, "mass")
        m.set("value", f"{mass:.8g}")
    if inertia is not None:
        tensor = ine.find("inertia")
        if tensor is None:
            tensor = ET.SubElement(ine, "inertia")
        for k, v in zip(("ixx", "ixy", "ixz", "iyy", "iyz", "izz"), inertia):
            tensor.set(k, f"{v:.8g}")
    if created:                       # guarantee a complete, valid <inertial>
        if ine.find("mass") is None:
            ET.SubElement(ine, "mass").set("value", "0.1")
        if ine.find("inertia") is None:
            tensor = ET.SubElement(ine, "inertia")
            for k in ("ixx", "iyy", "izz"):
                tensor.set(k, "0.0001")
            for k in ("ixy", "ixz", "iyz"):
                tensor.set(k, "0")


def _safe_name(name: str) -> str:
    """Sanitize one URDF identifier (link/joint name) the same way the exporter
    does: keep ``[A-Za-z0-9_]``, collapse every other run to ``_``, never start
    with a digit.  Idempotent on names that are already valid."""
    from sw2robot.exporter.model import safe_name
    return safe_name(name)


def _sanitize_urdf_names(root) -> None:
    """Rewrite every ``<link>``/``<joint>`` name -- and all their cross
    references (joint parent/child link, mimic joint) -- through
    :func:`_safe_name`, so the emitted URDF never carries a hyphen, space, dot
    or other character that downstream tools (xacro, ROS, MoveIt) choke on.

    A no-op when the names are already clean (the usual case: the exporter and
    ``rename_joint`` both validate), so it is a safety net that also covers
    hand-edited root names, ports and externally imported URDFs."""
    def _remap(elems, attr):
        mapping, used = {}, set()
        for el in elems:
            orig = el.get(attr)
            if orig is None or orig in mapping:
                continue
            cand, i = _safe_name(orig), 1
            while cand in used:               # two dirty names -> same clean one
                i += 1
                cand = f"{_safe_name(orig)}_{i}"
            mapping[orig] = cand
            used.add(cand)
        return mapping

    links = root.findall("link")
    joints = root.findall("joint")
    lmap = _remap(links, "name")
    jmap = _remap(joints, "name")
    for le in links:
        if le.get("name") in lmap:
            le.set("name", lmap[le.get("name")])
    for je in joints:
        if je.get("name") in jmap:
            je.set("name", jmap[je.get("name")])
        for tag in ("parent", "child"):
            el = je.find(tag)
            if el is not None and el.get("link") in lmap:
                el.set("link", lmap[el.get("link")])
        mim = je.find("mimic")
        if mim is not None and mim.get("joint") in jmap:
            mim.set("joint", jmap[mim.get("joint")])


# <element>: (urdf attribute, JointEdit field) pairs, in URDF attribute order.
_PHYSICS_ELEMENTS = (
    ("dynamics", (("damping", "damping"), ("friction", "friction"))),
    ("safety_controller", (("soft_lower_limit", "soft_lower_limit"),
                           ("soft_upper_limit", "soft_upper_limit"),
                           ("k_position", "k_position"),
                           ("k_velocity", "k_velocity"))),
    ("calibration", (("rising", "cal_rising"), ("falling", "cal_falling"))),
)


def _apply_joint_physics(je, edit) -> None:
    """Bake the overlay's ``<dynamics>`` / ``<safety_controller>`` /
    ``<calibration>`` onto a movable joint element.  Each element is (re)built
    from the set fields, or removed when the overlay clears them all."""
    if je.get("type") not in ("revolute", "prismatic", "continuous"):
        return
    for tag, pairs in _PHYSICS_ELEMENTS:
        attrs = {attr: getattr(edit, fld) for attr, fld in pairs
                 if getattr(edit, fld) is not None}
        el = je.find(tag)
        if not attrs:
            if el is not None:
                je.remove(el)
            continue
        if el is None:
            el = ET.SubElement(je, tag)
        else:
            el.attrib.clear()
        for attr, val in attrs.items():
            el.set(attr, f"{float(val):.6g}")


def build_urdf(state: RobotCompilerState, sanitize: bool = True,
               fold_mass_only: bool = True) -> str:
    """The CAD URDF with the interactive overlay baked in -- per-joint (rename,
    limits, mimic, axis flip, joint type) AND per-link (colour, inertial) -- so
    the exported package and configs stay consistent.

    With ``sanitize`` (default), link/joint names are passed through
    :func:`_sanitize_urdf_names` last, so a CAD-derived URDF is guaranteed free
    of hyphens and other unsafe characters.  Pass ``sanitize=False`` when editing
    a URDF the user opened directly: its names are already its own contract (the
    viewer shows them, edits reference them), so they must be preserved verbatim.

    ``fold_mass_only`` (default) lumps each mass-only link into its fixed parent
    -- the export representation.  Pass ``False`` for the editor's served URDF so
    a mass-only link is kept (geometry stripped, link + fixed joint preserved) and
    stays selectable in the joint tree; otherwise it folds away and the user has
    no row left to toggle it back off (mirrors how CAD mode keeps the link)."""
    # Preserve XML comments on the round trip: a CAD-exported URDF carries
    # per-link ``<!-- sw2robot material=... density=... inertia=... -->``
    # provenance, and the user may open / edit / re-export it here.  The default
    # parser silently drops comments, so use a comment-keeping TreeBuilder; the
    # comment nodes have a non-string tag, so findall("link")/("joint") below
    # still ignore them.
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(Path(state.urdf_path).read_text(encoding="utf-8"),
                         parser=parser)
    renames = {j: e.rename for j, e in state.edits.items() if e.rename}

    for je in root.findall("joint"):
        e = state.edits.get(je.get("name"))
        if e is None:
            continue
        if e.jtype:
            je.set("type", e.jtype)
            if e.jtype in ("fixed", "continuous"):
                for tag in (("axis", "limit") if e.jtype == "fixed" else ("limit",)):
                    el = je.find(tag)
                    if el is not None:
                        je.remove(el)
        # backfill the <axis> BEFORE the flip, so flipping a joint that this same
        # edit just made movable (it had no axis) still takes effect
        if je.get("type") in ("revolute", "prismatic", "continuous") \
                and je.find("axis") is None:
            ET.SubElement(je, "axis").set("xyz", "0 0 1")
        if e.flip_axis:
            ax = je.find("axis")
            if ax is not None:
                cur = [float(x) for x in ax.get("xyz", "0 0 1").split()]
                _set_xyz(ax, [-v for v in cur])
        if e.rename:
            je.set("name", e.rename)
        _edits_limit = any(v is not None for v in
                           (e.lower, e.upper, e.effort, e.velocity))
        if _edits_limit and je.get("type") in ("revolute", "prismatic", "continuous"):
            lim = je.find("limit")
            if lim is None:
                lim = ET.SubElement(je, "limit")
                lim.set("effort", "10")
                lim.set("velocity", "3.14")
            # continuous joints carry no lower/upper, only effort/velocity
            if je.get("type") != "continuous":
                if e.lower is not None:
                    lim.set("lower", f"{e.lower:.6g}")
                if e.upper is not None:
                    lim.set("upper", f"{e.upper:.6g}")
            if e.effort is not None:
                lim.set("effort", f"{e.effort:.6g}")
            if e.velocity is not None:
                lim.set("velocity", f"{e.velocity:.6g}")
        if e.mimic_joint and je.get("type") != "fixed":
            mim = je.find("mimic")
            if mim is None:
                mim = ET.SubElement(je, "mimic")
            mim.set("joint", e.mimic_joint)
            mim.set("multiplier", f"{e.mimic_multiplier:g}")
            mim.set("offset", f"{e.mimic_offset:g}")
        # a joint made movable (e.g. fixed -> revolute) must satisfy URDF's
        # requirement of a <limit> for revolute/prismatic; add a placeholder when
        # the type edit didn't supply one (the <axis> was backfilled above), so
        # the type change alone never yields an invalid URDF.
        ftype = je.get("type")
        if ftype in ("revolute", "prismatic") and je.find("limit") is None:
            lim = ET.SubElement(je, "limit")
            lim.set("lower", "-3.14159" if ftype == "revolute" else "0")
            lim.set("upper", "3.14159" if ftype == "revolute" else "0.1")
            lim.set("effort", "10")
            lim.set("velocity", "3.14")
        # optional <dynamics>/<safety_controller>/<calibration> from the overlay
        _apply_joint_physics(je, e)

    # repoint any mimic that referenced a now-renamed joint
    for je in root.findall("joint"):
        mim = je.find("mimic")
        if mim is not None and mim.get("joint") in renames:
            mim.set("joint", renames[mim.get("joint")])

    # per-link overlay: visual colour + inertial (keyed by the base-URDF name,
    # applied before the name sanitize so references stay consistent)
    # mass-only is valid only on a fixed child (its inertial lumps into the fixed
    # parent); honour it only there, so a stray flag can't make a movable link
    # invisible.  Joint types are final by now (the overlay loop above ran).
    # only the mass-only path needs the child->joint-type map; skip building it
    # (a full joint sweep) on the common no-mass-only refresh
    any_mass_only = any(led.mass_only for led in state.link_edits.values())
    child_jtype = {je.find("child").get("link"): je.get("type")
                   for je in root.findall("joint")
                   if je.find("child") is not None} if any_mass_only else {}
    mass_only = set()
    for le in root.findall("link"):
        led = state.link_edits.get(le.get("name"))
        if led is None:
            continue
        if led.color:
            _apply_color(le, led.color)
        if led.mass is not None or led.com is not None or led.inertia is not None:
            _apply_inertial(le, led.mass, led.com, led.inertia)
        if led.mass_only and child_jtype.get(le.get("name")) == "fixed":
            for geo in le.findall("visual") + le.findall("collision"):
                le.remove(geo)              # weight kept (inertial), geometry gone
            mass_only.add(le.get("name"))

    # fold the mass-only links into their fixed parent so the weight reaches it
    # (export only; the editor keeps the stripped link so it stays selectable)
    if mass_only and fold_mass_only:
        from sw2robot.exporter.merge import merge_fixed_links
        merge_fixed_links(root, only=mass_only)

    # final guarantee: no hyphens/spaces/etc. in any emitted link or joint name
    if sanitize:
        _sanitize_urdf_names(root)
    return ET.tostring(root, encoding="unicode")


def _servo_mappings(state: RobotCompilerState, parsed_joints: list) -> list:
    by_name = {j["name"]: j for j in parsed_joints}
    final_name = {
        orig.get("name"): parsed.get("name")
        for orig, parsed in zip(state.joints, parsed_joints)
    }
    out = []
    for orig, e in state.edits.items():
        if e.servo_id is None:
            continue
        eff = final_name.get(orig) or _safe_name(e.rename or orig)
        j = by_name.get(eff, {})
        out.append({
            "jointName": eff, "servoId": e.servo_id, "direction": e.direction,
            "angleOffset": e.angle_offset,
            "minAngle": e.lower if e.lower is not None else j.get("lowerLimit", 0.0),
            "maxAngle": e.upper if e.upper is not None else j.get("upperLimit", 0.0),
        })
    return out


def export_ros_package(state: RobotCompilerState, out_path,
                       export_options: dict | None = None,
                       strict: bool = False) -> Path:
    """Produce the final ROS/MoveIt/Gazebo/IL config ZIP via the vendored
    ``rc_config.export_all_configs``.  With ``strict=True`` a non-empty
    ``validate(state)`` raises instead of writing a broken package."""
    from ._vendor.rc_config.export import export_all_configs
    from ._vendor.rc_config.urdf_parser import parse_urdf_content

    problems = validate(state)
    if problems and strict:
        raise ValueError("cannot export, state has problems:\n  - "
                         + "\n  - ".join(problems))

    urdf = build_urdf(state)
    parsed = parse_urdf_content(urdf)
    data = export_all_configs(
        urdf_content=urdf, joints=parsed["joints"],
        servo_mappings=_servo_mappings(state, parsed["joints"]),
        planning_groups=[], controllers=[], disabled_collision_pairs=[],
        gazebo_physics={}, gazebo_plugins=[], il_observation={}, il_action={},
        robot_name=state.robot_name, export_options=export_options)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(data)
    return out_path
