"""Core API for the CAD -> robot-compiler bridge (UI-independent).

Every function here is callable from a CLI, a viser callback, or a FastAPI
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

import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from .state import JointEdit, RobotCompilerState

__all__ = [
    "import_module", "extract_and_import", "sw_recent_assemblies",
    "sw_session_status", "load_module", "register_module",
    "rename_joint", "set_limits", "set_mimic", "set_servo",
    "set_axis_flip", "set_joint_type", "reverse_direction",
    "set_actuator", "apply_servo_profile", "SERVO_PROFILES",
    "chain_related", "movable_names", "validate",
    "build_urdf", "export_ros_package",
    "state_path", "save_state", "load_state", "load_edits",
]

JOINT_TYPES = ("fixed", "revolute", "continuous", "prismatic")
_SAFE_NAME = re.compile(r"^[A-Za-z_][0-9A-Za-z_]*$")
_MOVABLE = ("revolute", "continuous", "prismatic")


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
    ``<package_dir>/<robot_name>.cad2rc.json``."""
    if path:
        return Path(path)
    return Path(state.package_dir) / f"{state.robot_name}.cad2rc.json"


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
    freshly-built joints/URDF.  Only edits whose joint still exists are applied
    (so a rebuilt module with changed names is handled gracefully).  Returns the
    number of edits applied; 0 if no sidecar exists."""
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
    half PLUS the fast ``build`` half), so the viser View can offer a single
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
    """Best-effort list of recently-opened ``.sldasm`` paths from the SolidWorks
    MRU registry key, newest first -- used only to pre-fill the import path
    field (an approximation of "the assembly you have open").  Returns ``[]`` on
    any failure (non-Windows, no SolidWorks, key absent)."""
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
    # paths, parts AND assemblies mixed); keep the .sldasm ones in File-number
    # order so File1 (most-recently-opened) is first.
    for ver in sorted(versions, reverse=True):
        sub = r"%s\%s\Recent File List" % (base, ver)
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
                    if isinstance(val, str) and val.lower().endswith(".sldasm"):
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
                 effort: Optional[float] = None,
                 velocity: Optional[float] = None) -> None:
    """Set the joint's effort (N*m) and/or velocity (rad/s) limits."""
    _require_joint(state, joint)
    e = state.edit_for(joint)
    if effort is not None:
        e.effort = float(effort)
    if velocity is not None:
        e.velocity = float(velocity)


def apply_servo_profile(state: RobotCompilerState, joint: str,
                        model: Optional[str]) -> Optional[dict]:
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
    lo = e.lower if e.lower is not None else parsed.get("lowerLimit", 0.0)
    hi = e.upper if e.upper is not None else parsed.get("upperLimit", 0.0)
    e.flip_axis = not e.flip_axis
    e.lower, e.upper = -hi, -lo


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


def build_urdf(state: RobotCompilerState) -> str:
    """The CAD URDF with the interactive overlay baked in (rename, limits, mimic,
    axis flip, joint type), so the exported package and configs stay consistent."""
    root = ET.fromstring(Path(state.urdf_path).read_text(encoding="utf-8"))
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

    # repoint any mimic that referenced a now-renamed joint
    for je in root.findall("joint"):
        mim = je.find("mimic")
        if mim is not None and mim.get("joint") in renames:
            mim.set("joint", renames[mim.get("joint")])

    return ET.tostring(root, encoding="unicode")


def _servo_mappings(state: RobotCompilerState, parsed_joints: list) -> list:
    by_name = {j["name"]: j for j in parsed_joints}
    out = []
    for orig, e in state.edits.items():
        if e.servo_id is None:
            continue
        eff = e.rename or orig
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
    from ._vendor.rc_config.urdf_parser import parse_urdf_content
    from ._vendor.rc_config.export import export_all_configs

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
