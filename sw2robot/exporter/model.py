"""Extract a link/joint model from a SolidWorks assembly document.

Policy: one top-level component = one URDF link.  The link tree is a spanning
tree over the mate-connectivity graph rooted at a base component.

Joint type per tree edge is inferred from the mates connecting the two links:
a **CONCENTRIC** mate (shared cylinder axis) makes the edge **revolute** about
that axis; otherwise the edge is **fixed**.  (This family of assemblies is fully
constrained -- closed loops -- so ``GetRemainingDOFs`` yields no movable DOF;
the concentric-axis heuristic recovers the intended hinges.)

Frames: each link gets an *anchor* frame.  Revolute links anchor a world-aligned
frame on the rotation axis (so the URDF joint rotates the part about the real
hinge line); other links anchor their own component frame.  The mesh is placed
back with the link's ``<visual>`` origin.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field

import numpy as np

from .geometry import (
    frame_at_point,
    matrix_from_rpy,
    matrix_to_xyz_rpy,
    relative_matrix,
    transform_to_matrix,
)
from .state import (
    ComponentState,
    GraphState,
    LimitJoint,
    MateEdge,
    MateGeo,
    SubGraph,
)
from .swcom import as_iface, safe_call, safe_prop

MATE_TYPES = {0: "COINCIDENT", 1: "CONCENTRIC", 2: "PERPENDICULAR",
              3: "PARALLEL", 4: "TANGENT", 5: "DISTANCE", 6: "ANGLE",
              7: "UNIVERSALJOINT", 8: "SYMMETRIC", 9: "CAMFOLLOWER",
              10: "GEAR", 11: "WIDTH", 12: "LOCKTOSKETCH", 13: "RACKPINION",
              14: "SCREW", 15: "LINEARCOUPLER", 16: "PROFILECENTER",
              17: "SLOT", 18: "PATH", 19: "HINGE", 20: "LOCK"}
CONCENTRIC = 1
LOCK = 20


def safe_name(raw):
    s = re.sub(r"[^0-9A-Za-z_]+", "_", raw)
    s = s.strip("_")
    if not s or s[0].isdigit():
        s = "c_" + s
    return s


# Standard-hardware nomenclature, matched (case-insensitively) against BOTH a
# part's containing library FOLDER and its own name.  A bolt threaded into a
# tapped hole shows up as a concentric mate, which the joint classifier would
# otherwise read as a hinge -- so every screw/nut/washer/pin spawns a spurious
# revolute (and, mate-less mirror copies inherit one from a twin).  CAD does not
# expose a clean "I am a fastener" flag here (these are an in-house purchased-
# parts library, not SolidWorks Toolbox, and the threads are modelled geometry,
# not cosmetic-thread features), so we key off the catalogue naming that the
# library does carry.  EN + JP tokens; the size pattern (M3x6, FS-M3) is the
# unambiguous fastener tell.
# 'screw'/'pin' are bounded so they fire on the hardware noun but NOT on
# compound part names that merely contain them -- NejiNeji's "screwlock" /
# "ScrewRing" connectors and "Pinion" gears are structural, not fasteners.
_FASTENER_WORD = re.compile(
    r"(?i)(?:bolt|screw(?![a-z])|hex[\s_-]*socket|socket[\s_-]*head|washer|"
    r"[\s_-]nut\b|clinch|self[\s_-]*clinch|rivet|dowel|(?<![a-z])pin(?![a-z])|"
    r"[\s_-]stud\b|fastener|(?<![a-z])vida(?![a-z])|"   # vida = screw (TR)
    r"ねじ|ネジ|ビス|ボルト|ナット|ワッシャ|座金|止めねじ|皿ねじ|ピン)")
# e.g. M3x6, M3X8, FS-M3, M4x10 -- a metric size designation, the strongest
# tell.  chr(0xD7) = the full-width multiplication sign some catalogues use
_FASTENER_SIZE = re.compile(
    r"(?i)(?<![a-z0-9])(?:fs[\s_-]*)?m\d+(?:\.\d+)?\s*[x"
    + chr(0xD7) + r"]\s*\d+")
# a bare metric thread call-out used as a stand-alone library folder (e.g. "M3")
_FASTENER_FOLDER_BARE = re.compile(r"(?i)^(?:fs[\s_-]*)?m\d+(?:[\s_-]\w+)*$")


def is_fastener_part(name, part_path, extra=None, keep=None):
    """True if ``name``/``part_path`` looks like standard fastener hardware.

    ``extra`` -- additional case-insensitive substrings that also mark a
    fastener (config ``fastener:``); ``keep`` -- substrings that VETO the match
    (config ``not_fastener:``), so a wrongly-caught part stays a real link."""
    hay_name = name or ""
    # part_path is captured on Windows (backslash separators) but the build runs
    # on any OS, so split on BOTH separators rather than os.path (which only
    # honours the host's) -- otherwise the library folder is unreadable on Linux
    folder = ""
    if part_path:
        segs = [s for s in re.split(r"[\\/]+", part_path) if s]
        folder = segs[-2] if len(segs) >= 2 else ""
    hay = f"{folder}/{hay_name}"
    low = hay.lower()
    if keep and any(k.lower() in low for k in keep):
        return False
    if extra and any(e.lower() in low for e in extra):
        return True
    if _FASTENER_WORD.search(hay) or _FASTENER_SIZE.search(hay):
        return True
    # a leaf part sitting directly in a folder whose whole name is a thread
    # call-out ("Bolt/", "M3/") -- folder is the library category, very reliable
    if folder and _FASTENER_FOLDER_BARE.match(folder):
        return True
    return False


def _sw_mass_props(mp):
    """(mass, com3, inertia6) of a part from its SolidWorks ``IMassProperty``.

    The inertia tensor is taken ABOUT THE CENTRE OF MASS, in the part's own
    coordinate axes (the frame the mesh is exported in), in SI units (kg, m,
    kg.m^2).  We reconstruct it from the principal moments + principal axes,
    both of which the classic ``IMassProperty`` exposes::

        I_part = R^T @ diag(Px, Py, Pz) @ R

    where ``R``'s rows are the three principal-axis unit vectors expressed in
    the part frame (SolidWorks returns them as three consecutive triples).
    Returns ``(None, None, None)`` if the values are unavailable / degenerate
    so the caller falls back to the mesh estimate."""
    # Force SI output if this build's interface exposes the toggle
    # (IMassProperty2); the classic IMassProperty is already SI, so the
    # attribute is simply absent and this is a no-op.
    try:
        mp.UseSystemUnits = True
    except Exception:
        pass

    def _arr(name, n):
        v = safe_prop(mp, name)
        try:
            vals = [float(x) for x in v]
        except (TypeError, ValueError):
            return None
        return vals if len(vals) == n else None

    try:
        mass = safe_prop(mp, "Mass")
        mass = float(mass) if mass is not None else None
    except (TypeError, ValueError):
        mass = None
    com = _arr("CenterOfMass", 3)
    pm = _arr("PrincipalMomentsOfInertia", 3)
    pax = _arr("PrincipalAxesOfInertia", 9)
    if not (mass and mass > 0 and com and pm and pax):
        return None, None, None
    if not (np.all(np.isfinite(com)) and np.all(np.isfinite(pm))
            and np.all(np.isfinite(pax))):
        return None, None, None
    R = np.asarray(pax, float).reshape(3, 3)     # rows = principal axes (part frame)
    I = R.T @ np.diag(pm) @ R                      # tensor about COM, part axes
    inertia6 = (float(I[0, 0]), float(I[0, 1]), float(I[0, 2]),
                float(I[1, 1]), float(I[1, 2]), float(I[2, 2]))
    return mass, [float(x) for x in com], inertia6


@dataclass
class Component:
    name: str
    link_name: str
    part_path: str | None
    is_subassembly: bool
    world: np.ndarray            # 4x4 component local->world (assembled pose)
    fixed: bool
    dof: int
    mesh_file: str | None = None
    visual_xyz: list = field(default_factory=lambda: [0, 0, 0])
    visual_rpy: list = field(default_factory=lambda: [0, 0, 0])
    material: str | None = None      # SolidWorks material name
    density: float | None = None     # kg/m^3 from that material
    # SolidWorks-native mass properties (part-local frame, SI), preferred over
    # the mesh estimate when present -- see exporter.inertia.link_inertial_from_sw
    sw_mass: float | None = None              # kg
    sw_com: list | None = None                # centre of mass [x,y,z] (m)
    sw_inertia: list | None = None            # (ixx,ixy,ixz,iyy,iyz,izz) about COM
    # set when a per-link density override (config / web editor) should drive
    # the inertial from the mesh, overriding the SolidWorks-native values
    density_override: bool = False
    # standard hardware (screw/bolt/nut/washer/pin): weld it FIXED to whatever it
    # fastens and never let it be a tree parent -- see is_fastener_part
    is_fastener: bool = False
    # mass-only: emit <inertial> but no <visual>/<collision>, and lump the
    # inertial into the fixed parent on export (config `mass_only:` / the editor)
    mass_only: bool = False


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    jtype: str
    xyz: list = field(default_factory=lambda: [0, 0, 0])
    rpy: list = field(default_factory=lambda: [0, 0, 0])
    axis: list | None = None
    lower: float | None = None
    upper: float | None = None
    # actuator limits: None -> the URDF writer's defaults (effort 10, velocity 3.14)
    effort: float | None = None
    velocity: float | None = None
    # optional URDF joint physics (set via joints.yaml or the editor overlay);
    # each is None when unset so the element is simply omitted:
    #   dynamics    -> <dynamics damping= friction=>
    #   safety      -> <safety_controller soft_lower_limit= soft_upper_limit=
    #                                     k_position= k_velocity=>
    #   calibration -> <calibration rising= falling=>
    dynamics: dict | None = None
    safety: dict | None = None
    calibration: dict | None = None
    mate_types: list = field(default_factory=list)
    # mimic coupling (closed-loop / geared): {joint, multiplier, offset}
    mimic: dict | None = None
    # debug: the SolidWorks concentric-axis line in WORLD coords (revolute only)
    sw_axis_point: list | None = None
    sw_axis_dir: list | None = None
    # human-readable reason from the geometric classifier (template comment)
    geo_note: str | None = None


@dataclass
class Port:
    """An output connection port (robot-compiler / NejiNeji ``to_coords``).

    Emitted as an empty ``dummy_link`` attached by a fixed joint to a tip link.
    ``xyz``/``rpy`` are in ``parent_link``'s frame; the port's +Z should point
    along the outgoing connector axis (robot-compiler auto-aligns on +Z).
    ``parent_link`` holds the component link name; the URDF writer remaps it if
    it is the root (-> ``base_link``)."""
    name: str
    parent_link: str
    xyz: list = field(default_factory=lambda: [0, 0, 0])
    rpy: list = field(default_factory=lambda: [0, 0, 0])
    # optional explicit fixed-joint name; the URDF writer auto-derives one from
    # ``name`` when this is empty.
    joint_name: str = ""


@dataclass
class RobotModel:
    name: str
    components: list
    joints: list
    base_link: str
    detected_edges: list = field(default_factory=list)
    # robot-compiler module interface: output ports + the URDF name the root
    # link is written as (the convention is ``base_link`` = input port).
    ports: list = field(default_factory=list)
    root_link_name: str = "base_link"
    # closed-loop (four-bar / parallel) data for the runtime-IK relay:
    # {closures:[{link_a,link_b,point,axis}], dependent:[...], independent:[...]}
    loop_closures: dict | None = None


def _match_component(comps, ref):
    """Resolve a config reference (link name, exact or substring) to Name2."""
    for c in comps:
        if c.link_name == ref or c.name == ref:
            return c.name
    for c in comps:
        if ref in c.link_name or ref in c.name:
            return c.name
    return None


def _num(v):
    """Coerce a YAML scalar to float, or None if absent / unparseable."""
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _subdict(src, keys):
    """Pull ``keys`` out of mapping ``src`` as floats; None if none are present."""
    if not isinstance(src, dict):
        return None
    d = {k: _num(src.get(k)) for k in keys if _num(src.get(k)) is not None}
    return d or None


# URDF joint-physics sub-elements and their attribute names, in write order.
_DYNAMICS_KEYS = ("damping", "friction")
_SAFETY_KEYS = ("soft_lower_limit", "soft_upper_limit", "k_position", "k_velocity")
_CALIBRATION_KEYS = ("rising", "falling")


def physics_from_cfg(j):
    """Extract optional joint-physics from a joints.yaml ``joints`` entry.

    Returns ``{effort, velocity, dynamics, safety, calibration}`` with any
    unset field left as None, so a plain config entry adds nothing.  Accepts
    ``safety_controller`` (the URDF spelling) or ``safety`` as an alias."""
    return {
        "effort": _num(j.get("effort")),
        "velocity": _num(j.get("velocity")),
        "dynamics": _subdict(j.get("dynamics"), _DYNAMICS_KEYS),
        "safety": _subdict(j.get("safety_controller") or j.get("safety"),
                           _SAFETY_KEYS),
        "calibration": _subdict(j.get("calibration"), _CALIBRATION_KEYS),
    }


def resolve_directed(comps, joints_cfg):
    """Config 'joints' -> ordered list of directed edges.

    Each entry uses ``parent``/``child`` (preferred, defines the kinematic
    chain) or legacy ``between: [parent, child]``.  Returns dicts with resolved
    component Name2 plus type/limits/axis overrides."""
    out = []
    for j in joints_cfg or []:
        if "parent" in j and "child" in j:
            pa, ca = j["parent"], j["child"]
        elif j.get("between") and len(j["between"]) == 2:
            pa, ca = j["between"][0], j["between"][1]
        else:
            continue
        np_ = _match_component(comps, pa)
        nc = _match_component(comps, ca)
        if not (np_ and nc):
            print(f"      WARN: joint config '{pa}->{ca}' did not match links")
            continue
        out.append({"parent": np_, "child": nc,
                    "type": j.get("type", "fixed"),
                    "lower": j.get("lower"), "upper": j.get("upper"),
                    "axis_point": j.get("axis_point"),
                    "axis_dir": j.get("axis_dir"),
                    "mimic": j.get("mimic"),
                    **physics_from_cfg(j)})
    return out


def resolve_ports(comps, ports_cfg):
    """Config ``ports`` -> ``list[Port]`` (robot-compiler output connectors).

    Each entry: ``parent`` (link/component, substring) = the tip link the port
    hangs off; ``xyz``/``rpy`` = the dummy_link origin in that link's frame
    (+Z = outgoing connector axis); ``name`` (optional, defaults to
    ``dummy_link``, ``dummy_link2`` ...)."""
    out = []
    for i, p in enumerate(ports_cfg or []):
        ref = p.get("parent") or p.get("link")
        nm = _match_component(comps, ref) if ref else None
        parent_link = next((c.link_name for c in comps if c.name == nm), None)
        if parent_link is None:
            print(f"      WARN: port parent '{ref}' did not match a link; "
                  f"skipping port")
            continue
        name = p.get("name") or ("dummy_link" if i == 0 else f"dummy_link{i + 1}")
        out.append(Port(name=name, parent_link=parent_link,
                        xyz=list(p.get("xyz", [0.0, 0.0, 0.0])),
                        rpy=list(p.get("rpy", [0.0, 0.0, 0.0])),
                        joint_name=p.get("joint_name") or ""))
    return out


def _top_level(full_name):
    return full_name.split("/")[0] if full_name else full_name


def extract_components(doc, exclude=None, progress=None):
    """``progress(link_name)`` -- if given -- is called as each component is
    about to be read (its material/mass-property lookup is the per-part cost),
    so a UI can show WHICH part the (multi-minute) load is on, not just a stage."""
    exclude = [e.lower() for e in (exclude or [])]
    raw = list(safe_call(doc, "GetComponents", True) or [])
    comps = []
    used = set()
    n_skipped = 0
    n_excluded = 0
    matcache = {}        # part_path -> (material name, density kg/m^3)

    def _material_of(ct, path):
        key = path.lower()
        if key in matcache:
            return matcache[key]
        material = density = None
        sw = None
        try:
            md = safe_call(ct, "GetModelDoc2")
            if md is not None:
                try:
                    pd = as_iface(md, "IPartDoc")
                    res = pd.GetMaterialPropertyName2("", "")
                    if isinstance(res, (tuple, list)):
                        res = next((x for x in res if x), None)
                    material = str(res) if res else None
                except Exception:
                    pass
                try:
                    mdoc = as_iface(md, "IModelDoc2")
                    ext = as_iface(mdoc.Extension, "IModelDocExtension")
                    mp = ext.CreateMassProperty
                    if callable(mp):
                        mp = mp()
                    d = getattr(mp, "Density", None)
                    if d and d > 1.0:           # kg/m^3
                        density = float(d)
                    # SolidWorks-native mass/COM/inertia (exact CAD geometry +
                    # material/override) -- preferred over the mesh estimate
                    mass, com, inertia6 = _sw_mass_props(mp)
                    if mass is not None:
                        sw = {"mass": mass, "com": com, "inertia": inertia6}
                except Exception:
                    pass
        except Exception:
            pass
        matcache[key] = (material, density, sw)
        return material, density, sw
    for c in raw:
        ct = as_iface(c, "IComponent2")
        name = safe_prop(ct, "Name2")
        state = safe_call(ct, "GetSuppression")
        if not name or state == 0:
            n_skipped += 1
            # name a dropped part so a missing link (e.g. a suppressed finger)
            # is diagnosable instead of silently vanishing from the tree
            print(f"      skip component: name={name!r} "
                  f"suppression={state} (0=suppressed)")
            continue
        if any(e in name.lower() for e in exclude):
            n_excluded += 1
            continue
        path = safe_prop(ct, "GetPathName")
        is_asm = bool(path and path.lower().endswith(".sldasm"))
        tdata = safe_prop(ct, "Transform2")
        try:
            world = transform_to_matrix(tdata.ArrayData)
        except Exception:
            world = np.eye(4)
        fixed = bool(safe_call(ct, "IsFixed"))
        try:
            dof = ct.GetRemainingDOFs()[0]
        except Exception:
            dof = None
        ln = safe_name(name)
        base = ln
        i = 1
        while ln in used:
            i += 1
            ln = f"{base}_{i}"
        used.add(ln)
        if progress:
            progress(ln)
        material = density = None
        sw = None
        if path and not is_asm:
            material, density, sw = _material_of(ct, path)
        sw = sw or {}
        comps.append(Component(name=name, link_name=ln, part_path=path,
                               is_subassembly=is_asm, world=world,
                               fixed=fixed, dof=dof,
                               material=material, density=density,
                               sw_mass=sw.get("mass"), sw_com=sw.get("com"),
                               sw_inertia=sw.get("inertia")))
    if n_skipped or n_excluded:
        print(f"      (skipped {n_skipped} suppressed/unnamed, "
              f"excluded {n_excluded} components)")
    # Deterministic order so the spanning tree / base choice are reproducible
    # regardless of the order SolidWorks happens to return components in.
    comps.sort(key=lambda c: c.name)
    return comps


def _entity_axis_world(me):
    """World (point, unit-dir) of a concentric mate entity's axis, or None.

    ``IMateEntity2.EntityParams`` returns ``[px,py,pz, dx,dy,dz, radius, ...]``
    ALREADY IN ASSEMBLY (world) COORDINATES -- do NOT apply the component's
    Transform2 (that double-transforms it; verified by the fact that the two
    entities of a concentric mate are only co-axial in world when used raw)."""
    ep = safe_prop(me, "EntityParams")
    if not ep or not hasattr(ep, "__iter__"):
        return None
    vals = list(ep)
    if len(vals) < 6:
        return None
    p = np.asarray(vals[0:3], float)
    d = np.asarray(vals[3:6], float)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return None
    return (p, d / n)


def _com_int(v):
    """Coerce a SolidWorks enum property to ``int`` (or None).

    A typed interface returns these as plain ints, but when ``as_iface`` could
    not wrap the object (some entities marshal late-bound) the same property
    comes back as a win32com ``VARIANT`` -- ``int(VARIANT)`` then raises and
    aborts the whole extract.  Pull the value out of the VARIANT instead."""
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        inner = getattr(v, "value", None)
        try:
            return int(inner) if inner is not None else None
        except (TypeError, ValueError):
            return None


def _entity_geo(me):
    """(etype, point, dir, radius) of one mate entity in WORLD coords.

    Same EntityParams layout as :func:`_entity_axis_world` but keeps the
    entity even when the direction is degenerate (e.g. point entities), so
    the geometric classifier sees every constraint.  Returns None only when
    EntityParams itself is unavailable."""
    etype = safe_prop(me, "ReferenceType")     # swMateEntityType_e
    ep = safe_prop(me, "EntityParams")
    if not ep or not hasattr(ep, "__iter__"):
        return None
    vals = list(ep)
    if len(vals) < 3:
        return None
    p = [float(x) for x in vals[0:3]]
    d = [0.0, 0.0, 0.0]
    if len(vals) >= 6:
        dv = np.asarray(vals[3:6], float)
        n = np.linalg.norm(dv)
        if n > 1e-9:
            d = [float(x) for x in dv / n]
    radius = float(vals[6]) if len(vals) >= 7 else None
    return (_com_int(etype), p, d, radius)


def build_mate_graph(doc, comps):
    """Adjacency over top-level component names.

    Returns ``(adjacency, ground)`` where ``adjacency[frozenset({a,b})]`` is a
    dict ``{types: [...], axis: (point,dir) | None}`` and ``ground`` is the set
    of components mated to the assembly itself."""
    name_set = {c.name for c in comps}
    adjacency = {}
    ground = set()
    raw = list(safe_call(doc, "GetComponents", True) or [])
    mate_count = {}        # own_name -> number of mates GetMates returned
    unresolved = {}        # own_name -> {entity refs that are not known comps}
    # No mate de-duplication: each mate is returned by GetMates of BOTH its
    # components, so it is processed ~twice -- harmless for presence-based
    # classification, and far safer than de-duping by COM pointer address
    # (which is not a stable identity and was dropping concentric mates,
    # flipping hinges to fixed between runs).
    for c in raw:
        ct = as_iface(c, "IComponent2")
        own_name = safe_prop(ct, "Name2")
        _mates = list(safe_call(ct, "GetMates") or [])
        mate_count[own_name] = mate_count.get(own_name, 0) + len(_mates)
        for m in _mates:
            mate = as_iface(m, "IMate2")
            mtype = _com_int(safe_prop(mate, "Type"))
            mname = MATE_TYPES.get(mtype, str(mtype))
            ne = safe_call(mate, "GetMateEntityCount") or 0
            tops = []
            axis = None
            geo = {"type": mname, "etypes": [], "points": [], "dirs": [],
                   "radii": [], "owners": []}
            for i in range(ne):
                me = as_iface(safe_call(mate, "MateEntity", i), "IMateEntity2")
                rc = as_iface(safe_prop(me, "ReferenceComponent"), "IComponent2")
                rn = safe_prop(rc, "Name2") if rc else None
                top = _top_level(rn) if rn else None
                if top in name_set:
                    tops.append(top)
                else:
                    tops.append("__ground__")
                    if rn:        # record what this entity resolved to
                        unresolved.setdefault(own_name, set()).add(rn)
                if mtype == CONCENTRIC and axis is None:
                    axis = _entity_axis_world(me)
                eg = _entity_geo(me)
                if eg is not None:
                    et, p, d, r = eg
                    geo["etypes"].append(et)
                    geo["points"].append(p)
                    geo["dirs"].append(d)
                    geo["radii"].append(r)
                    geo["owners"].append(rn or "")
            real = list(dict.fromkeys(t for t in tops if t != "__ground__"))
            # An entity's ReferenceComponent can resolve to the ASSEMBLY
            # itself even though the geometry belongs to the component whose
            # GetMates returned this mate (seen on screw_body: connectors
            # mated onto servo outputs reported 'screw_body_module' as the
            # owner and the whole mate was silently dropped).  GetMates
            # guarantees own_name participates -- substitute it.
            if "__ground__" in tops and own_name in name_set \
                    and own_name not in real:
                real.append(own_name)
                k = tops.index("__ground__")
                tops[k] = own_name
                if len(geo["owners"]) > k:
                    geo["owners"][k] = own_name
            if "__ground__" in tops:
                ground.update(real)
            for i in range(len(real)):
                for j in range(i + 1, len(real)):
                    key = frozenset((real[i], real[j]))
                    rec = adjacency.setdefault(key, {"types": [], "axis": None,
                                                     "mates": []})
                    rec["types"].append(mname)
                    rec.setdefault("mates", []).append(geo)
                    if mtype == CONCENTRIC and axis is not None \
                            and rec["axis"] is None:
                        rec["axis"] = axis
    # Diagnostics: a top-level component with no mate edge attaches as FIXED and
    # cannot become a joint.  The usual cause is a MIRRORED sub-assembly instance
    # whose mate entities resolve to the mirror SOURCE (or internal parts) rather
    # than the instance, so every mate drops to ground.  Surface it so a
    # re-extraction shows whether GetMates returned nothing (mirror feature
    # carries no mates -> mirror the sibling) or returned mates that resolved to
    # unknown components (an extraction name-resolution gap we can fix).
    incident = set()
    for key in adjacency:
        incident.update(key)
    for c in comps:
        if c.name in incident or c.name in ground:
            continue
        seen = mate_count.get(c.name, 0)
        refs = sorted(unresolved.get(c.name, ()))[:4]
        print(f"      WARN: '{c.name}' has NO mate edge "
              f"({seen} mates from GetMates"
              + (f"; entities resolved to {refs}" if refs else "")
              + ") -> attaches FIXED")
    return adjacency, ground


def _limit_joint_from_feature(sfi, name_set):
    """A LimitDistance/LimitAngle mate FEATURE -> a joint spec, or None.

    A SolidWorks *limit* mate (a DISTANCE/ANGLE mate with min != max) is a real
    slider / hinge -- it lets the part travel between min..max.  Our geometric
    classifier only sees a plain DISTANCE/ANGLE constraint and over-fixes it, so
    we read the feature data: which two components, the axis, and the min/max
    travel (which become the URDF joint limits, relative to the assembled pose)."""
    mate2 = as_iface(safe_call(sfi, "GetSpecificFeature2"), "IMate2")
    if mate2 is None:
        return None
    mtype = _com_int(safe_prop(mate2, "Type"))
    if mtype not in (5, 6):                       # 5=DISTANCE, 6=ANGLE
        return None
    defn = safe_call(sfi, "GetDefinition")
    if defn is None:
        return None
    if mtype == 5:
        d = as_iface(defn, "IDistanceMateFeatureData")
        lo = safe_prop(d, "MinimumDistance")
        hi = safe_prop(d, "MaximumDistance")
        cur = safe_prop(d, "Distance")
        jtype = "prismatic"
    else:
        d = as_iface(defn, "IAngleMateFeatureData")
        lo = safe_prop(d, "MinimumAngle")
        hi = safe_prop(d, "MaximumAngle")
        cur = safe_prop(d, "Angle")
        jtype = "revolute"
    try:
        lo, hi, cur = float(lo), float(hi), float(cur)
    except (TypeError, ValueError):
        return None
    if not (hi - lo) > 1e-9:                       # min==max -> a fixed mate
        return None

    ne = safe_call(mate2, "GetMateEntityCount") or 0
    ents = []                                      # [(top_name, (etype,p,d,r))]
    for i in range(ne):
        me = as_iface(safe_call(mate2, "MateEntity", i), "IMateEntity2")
        rc = as_iface(safe_prop(me, "ReferenceComponent"), "IComponent2")
        rn = safe_prop(rc, "Name2") if rc else None
        ents.append((_top_level(rn) if rn else None, _entity_geo(me)))
    pair = list(dict.fromkeys(t for t, _ in ents if t in name_set))
    if len(pair) != 2:
        return None
    geo_of = {}                                    # top component -> its entity geo
    for top, geo in ents:
        if top in pair and geo is not None:
            geo_of.setdefault(top, geo)
    ga, gb = geo_of.get(pair[0]), geo_of.get(pair[1])
    if ga is None or gb is None:
        return None
    pa, pb = np.asarray(ga[1], float), np.asarray(gb[1], float)
    na, nb = np.asarray(ga[2], float), np.asarray(gb[2], float)
    pt = pa
    if jtype == "prismatic":
        # face normal = slide axis, ORIENTED so component `a` (=pair[0]) moving
        # +axis grows the mate distance -- the build flips it when `a` is the
        # tree CHILD's parent.  Without this the travel reads the wrong way.
        axis = na.copy()
        if float((pa - pb) @ na) < 0:
            axis = -axis
    else:
        # a hinge rotates about the intersection line of the two faces
        axis = np.cross(na, nb)
        if np.linalg.norm(axis) < 1e-9:
            axis = na.copy()
    n = float(np.linalg.norm(axis))
    if n < 1e-9:
        return None
    axis = axis / n
    # URDF joint position 0 == the assembled pose (current distance/angle)
    return {"a": pair[0], "b": pair[1], "type": jtype,
            "axis_point": [float(x) for x in pt],
            "axis_dir": [float(x) for x in axis],
            "lower": lo - cur, "upper": hi - cur}


def extract_limit_joints(doc, comps):
    """Walk the assembly's mate features -> explicit limit-mate joints.

    These ARE the assembly's real DOFs (SolidWorks LimitDistance/LimitAngle
    mates).  Returned as ``[{a,b,type,axis_point,axis_dir,lower,upper}]`` with
    ``a``/``b`` top-level component Name2; the build uses them to override the
    over-constrained geometric classification on those edges."""
    name_set = {c.name for c in comps}
    out = []
    feat = safe_call(doc, "FirstFeature")
    guard = 0
    while feat is not None and guard < 100000:
        guard += 1
        fi = as_iface(feat, "IFeature")
        if safe_call(fi, "GetTypeName2") == "MateGroup":
            sub = safe_call(fi, "GetFirstSubFeature")
            while sub is not None:
                sfi = as_iface(sub, "IFeature")
                try:
                    spec = _limit_joint_from_feature(sfi, name_set)
                except Exception as e:
                    spec = None
                    print(f"      WARN: limit-mate read failed: {e!r}")
                if spec:
                    print("      limit joint: {} {} <-> {} [{:.3f}, {:.3f}]"
                          .format(spec["type"], ascii(spec["a"]),
                                  ascii(spec["b"]), spec["lower"], spec["upper"]))
                    out.append(spec)
                sub = safe_call(sfi, "GetNextSubFeature")
        feat = safe_call(fi, "GetNextFeature")
    return out


def choose_base(comps, ground, base_hint=None, adjacency=None):
    """Pick the base/root component.

    If ``base_hint`` (a case-insensitive substring) matches a component name or
    link name, that wins -- the auto heuristic below is unreliable when the only
    assembly-grounded component is a manipulated object (e.g. a workpiece mated
    to the origin) rather than the structural frame.

    Auto pick: the mate-graph HUB -- the part the most components bolt to.  The
    CAD ground set is unreliable (the designer often fixes a convenient arm part
    near the origin, NOT the frame), and the part nearest the arbitrary CAD
    origin is just as arbitrary; the frame everything attaches to is the
    highest-degree node.  Ties break toward a grounded/fixed part, then the part
    nearest the assembly centroid.  (Falls back to grounded/centroid when no
    adjacency is supplied.)"""
    if base_hint:
        h = base_hint.lower()
        for c in comps:        # exact match wins (UI sends full link names)
            if h == c.name.lower() or h == c.link_name.lower():
                return c
        for c in comps:
            if h in c.name.lower() or h in c.link_name.lower():
                return c
        print(f"      WARN: base hint '{base_hint}' matched nothing; "
              f"falling back to auto")

    deg = {c.name: 0 for c in comps}
    for key in (adjacency or ()):
        for n in key:
            if n in deg:
                deg[n] += 1
    pts = np.array([np.asarray(c.world, float)[:3, 3] for c in comps], float)
    centroid = pts.mean(axis=0) if len(pts) else np.zeros(3)

    def score(c):
        p = np.asarray(c.world, float)[:3, 3]
        return (deg[c.name], bool(c.fixed) or c.name in ground,
                -float(np.sum((p - centroid) ** 2)))
    return max(comps, key=score)


def classify_edge(types, axis):
    """Default (auto) joint type from a mate set -- a starting suggestion only.

    Mates are counted ~twice (once per component), so one real CONCENTRIC mate
    shows up as 1-2 occurrences (a single shared shaft => revolute hinge) while
    a bolt circle / pressed bearing has 2+ real concentric mates (3+
    occurrences => rigid).  A **LOCK** mate genuinely blocks rotation => fixed.

    A **PARALLEL** mate does NOT: on a hinge it usually just keeps the two shaft
    axes parallel (redundant with the CONCENTRIC alignment) and leaves rotation
    free.  Treating PARALLEL as anti-rotation wrongly froze one joint per finger
    on the feetech_hand (the short-distal / metacarpal mates use PARALLEL where
    the others use COINCIDENT), so PARALLEL no longer forces ``fixed``.  Genuine
    rigid joints are still pinned down with a joint config (export --config)."""
    if axis is None:
        return "fixed", None
    nconc = types.count("CONCENTRIC")
    if 1 <= nconc <= 2 and "LOCK" not in types:
        return "revolute", axis
    return "fixed", None


# --------------------------------------------------------------------
# Geometric (twist-nullspace) classification.
#
# Mate-type counting cannot tell a hinge from a bolt pair: both are
# "CONCENTRIC x2".  With the full entity geometry we can: each mate
# contributes linear constraints on the relative twist xi = (omega, v)
# (world frame; velocity of a material point x is  v + omega x x), and the
# nullspace of the stacked constraint matrix is the set of motions the
# mates actually leave free.  A bolt pair (two parallel, offset axes)
# kills all rotation; a hinge (everything concentric to ONE line) leaves
# exactly the rotation about it.
# --------------------------------------------------------------------

# mates that COUPLE two otherwise-free DOFs (gear trains etc.) -- they do
# not remove DOF between the pair in a tree sense, so they contribute no
# constraint rows; their presence is surfaced in the note (mimic candidate).
_COUPLING_MATES = {"GEAR", "RACKPINION", "SCREW", "CAMFOLLOWER",
                   "LINEARCOUPLER", "PATH", "UNIVERSALJOINT"}
# swMateEntityType_e values
_ET_POINT, _ET_LINE, _ET_CIRCLE, _ET_PLANE, _ET_CYL = 0, 1, 2, 3, 4


def _perp_basis(d):
    d = np.asarray(d, float)
    a = np.array([1.0, 0.0, 0.0])
    if abs(d @ a) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    u1 = np.cross(d, a); u1 /= np.linalg.norm(u1)
    u2 = np.cross(d, u1)
    return u1, u2


def _row_trans(u, p):
    """Row for  u . (v + omega x p) = 0  (translation along u blocked at p)."""
    return np.concatenate([np.cross(p, u), u])


def _row_rot(u):
    """Row for  u . omega = 0  (rotation component along u blocked)."""
    return np.concatenate([u, np.zeros(3)])


def _dedup_geo(mates):
    """Drop the duplicate mate records from per-component GetMates iteration."""
    seen = {}
    for m in mates:
        key = (m["type"], tuple(sorted(
            tuple(round(float(v), 9) for v in (list(p) + list(d)))
            for p, d in zip(m["points"], m["dirs"]))))
        seen.setdefault(key, m)
    return list(seen.values())


def _mate_entities(m):
    """[(etype, point, dir|None), ...] with unit dirs, None when degenerate."""
    out = []
    for i, p in enumerate(m["points"]):
        et = m["etypes"][i] if i < len(m["etypes"]) else None
        d = np.asarray(m["dirs"][i], float) if i < len(m["dirs"]) else None
        if d is not None and np.linalg.norm(d) < 1e-9:
            d = None
        out.append((et, np.asarray(p, float), d))
    return out


def _axis_entity(ents):
    """First entity with a usable direction (axis or normal)."""
    for et, p, d in ents:
        if d is not None:
            return et, p, d
    return None


def _mate_rows(m, axes_out, flags):
    """Constraint rows (list of 6-vectors) for one deduped mate record."""
    t = m["type"]
    ents = _mate_entities(m)
    if t in _COUPLING_MATES:
        flags.add("coupling")
        return []
    if t == "LOCK":
        return [_row_rot(e) for e in np.eye(3)] + \
               [_row_trans(e, np.zeros(3)) for e in np.eye(3)]

    ax = _axis_entity(ents)
    if ax is None:
        flags.add("partial")
        return []
    _, p, d = ax

    if t in ("CONCENTRIC", "HINGE"):
        axes_out.append((p, d))
        u1, u2 = _perp_basis(d)
        rows = [_row_trans(u1, p), _row_trans(u2, p),
                _row_rot(u1), _row_rot(u2)]
        if t == "HINGE":                      # hinge also blocks axial slide
            rows.append(_row_trans(d, p))
        return rows

    if t == "PARALLEL":
        u1, u2 = _perp_basis(d)
        return [_row_rot(u1), _row_rot(u2)]

    if t in ("PERPENDICULAR", "ANGLE"):
        dirs = [dd for _, _, dd in ents if dd is not None]
        if len(dirs) >= 2:
            w = np.cross(dirs[0], dirs[1])
            n = np.linalg.norm(w)
            if n > 1e-9:
                return [_row_rot(w / n)]
        flags.add("partial")
        return []

    if t in ("COINCIDENT", "DISTANCE", "WIDTH", "SYMMETRIC", "TANGENT"):
        etypes = [et for et, _, _ in ents]
        planes = [(pp, dd) for et, pp, dd in ents
                  if et == _ET_PLANE and dd is not None]
        lines = [(pp, dd) for et, pp, dd in ents
                 if et in (_ET_LINE, _ET_CYL, _ET_CIRCLE) and dd is not None]
        points = [pp for et, pp, _ in ents if et in (_ET_POINT, None)]
        if len(planes) >= 1 and not lines and (_ET_POINT not in etypes):
            n, pp = planes[0][1], planes[0][0]
            if t == "TANGENT":
                return [_row_trans(n, pp)]
            u1, u2 = _perp_basis(n)
            return [_row_trans(n, pp), _row_rot(u1), _row_rot(u2)]
        if planes and lines:                  # line constrained into a plane
            n = planes[0][1]
            lp, ld = lines[0]
            w = np.cross(ld, n)
            rows = [_row_trans(n, lp)]
            if np.linalg.norm(w) > 1e-9:
                rows.append(_row_rot(w / np.linalg.norm(w)))
            return rows
        if planes and _ET_POINT in etypes:    # point held on a plane
            return [_row_trans(planes[0][1], planes[0][0])]
        if len(lines) >= 2 and t == "COINCIDENT":   # edge-on-edge = axis
            lp, ld = lines[0]
            axes_out.append((lp, ld))
            u1, u2 = _perp_basis(ld)
            return [_row_trans(u1, lp), _row_trans(u2, lp),
                    _row_rot(u1), _row_rot(u2)]
        if len(points) >= 2 and t == "COINCIDENT":  # point-on-point
            return [_row_trans(e, points[0]) for e in np.eye(3)]
        # unknown entity combination: fall back to plane-like if any dir
        if t != "TANGENT":
            u1, u2 = _perp_basis(d)
            return [_row_trans(d, p), _row_rot(u1), _row_rot(u2)]
        flags.add("partial")
        return []

    flags.add("partial")                      # unmodelled mate type
    return []


def _cluster_axes(axes):
    """Merge collinear (point, dir) axis candidates; keep first of each line."""
    out = []
    for p, d in axes:
        dup = False
        for q, e in out:
            if abs(float(d @ e)) > 1.0 - 1e-8 \
                    and np.linalg.norm(np.cross(p - q, e)) < 1e-6:
                dup = True
                break
        if not dup:
            out.append((p, d))
    return out


_STRICT_MIN_RADIUS = 0.003   # below this a free axis is a screw/pin, not a bearing


def _max_concentric_radius(recs):
    radii = [r for m in recs if m["type"] == "CONCENTRIC"
             for r in (m.get("radii") or []) if r]
    return max(radii) if radii else None


def classify_edge_geo(mates, strict=False):
    """(jtype, axis, note) from full mate geometry; None -> caller falls back.

    Builds the twist constraint matrix from the deduped mates and inspects
    its nullspace: 0 DOF -> fixed; free rotation about a concentric axis ->
    revolute about it; a single pure translation -> prismatic; anything
    else stays fixed with an explanatory note.

    ``strict`` (used for SUB-ASSEMBLY internals): a free rotation whose
    concentric radius is below ~3 mm is a screw or pin hole, not a bearing
    -- inside sub-assemblies these are almost always fasteners, so they
    classify fixed instead of revolute."""
    recs = _dedup_geo(mates)
    rows, axes, flags = [], [], set()
    for m in recs:
        rows.extend(_mate_rows(m, axes, flags))
    if not rows:
        return None                            # nothing usable -- legacy path
    A = np.asarray(rows, float)
    _, s, vt = np.linalg.svd(A)
    tol = max(s[0] * 1e-8, 1e-12) if len(s) else 1e-12
    rank = int((s > tol).sum())
    N = vt[rank:].T                            # 6 x k nullspace basis
    k = N.shape[1]
    extra = "; has coupling mate (mimic candidate)" if "coupling" in flags \
        else ""
    extra += "; some mates unmodelled" if "partial" in flags else ""

    if k == 0:
        return "fixed", None, "geo: fully constrained" + extra

    # is rotation about one of the CAD axes still free?
    for p, d in _cluster_axes(axes):
        xi = np.concatenate([d, -np.cross(d, p)])
        r = xi - N @ (N.T @ xi)
        if np.linalg.norm(r) < 1e-6 * np.linalg.norm(xi):
            if strict:
                rmax = _max_concentric_radius(recs)
                if rmax is not None and rmax < _STRICT_MIN_RADIUS:
                    return "fixed", None, \
                        (f"geo: free axis but r={rmax*1000:.1f}mm "
                         f"= fastener, not bearing -> fixed" + extra)
            note = f"geo: free rotation about mate axis ({k} DOF)" + extra
            return "revolute", (p, d), note
    if k == 1:
        w, v = N[:3, 0], N[3:, 0]
        if np.linalg.norm(w) < 1e-9:
            vn = v / np.linalg.norm(v)
            # axial slide along a bolt/boss (concentric) axis is just an
            # unmodelled face contact, not an intended prismatic joint
            if any(abs(float(vn @ d)) > 0.999 for _, d in axes):
                return "fixed", None, \
                    "geo: only axial slide along a fastener axis" + extra
            p0 = recs[0]["points"][0] if recs[0]["points"] else [0, 0, 0]
            return "prismatic", (np.asarray(p0, float), vn), \
                "geo: 1 DOF pure translation" + extra
        pitch = float(w @ v) / float(w @ w)
        if abs(pitch) < 1e-6:
            p0 = np.cross(w, v) / float(w @ w)
            return "revolute", (p0, w / np.linalg.norm(w)), \
                "geo: 1 DOF rotation (derived axis)" + extra
        return "fixed", None, "geo: screw-like 1 DOF -> fixed" + extra
    if len(axes) and k >= 2:
        return "fixed", None, \
            f"geo: rotation blocked, under-constrained ({k} DOF)" + extra
    return "fixed", None, \
        f"geo: under-constrained ({k} DOF) -> fixed; verify" + extra


def classify_edge_auto(rec):
    """(jtype, axis, note): geometric when the graph has it, else legacy."""
    if rec.get("force_fixed"):
        return "fixed", None, "config: force_fixed"
    lj = rec.get("limit_joint")
    if lj:                                  # SolidWorks LimitDistance/LimitAngle
        return lj["type"], lj["axis"], "limit mate (SolidWorks slider/hinge)"
    if rec.get("fastener"):
        return "fixed", None, "fastener welded fixed"
    mates = rec.get("mates")
    if mates:
        try:
            out = classify_edge_geo(mates, strict=rec.get("strict", False))
            if out is not None:
                return out
        except Exception as e:
            print(f"      WARN: geometric classify failed ({e!r}); "
                  f"using mate-type heuristic")
    jt, ax = classify_edge(rec.get("types", []), rec.get("axis"))
    return jt, ax, None


def _edge_is_weak(jt_ax_note, types):
    """True for edges that should be a LAST RESORT as tree parents.

    Inter-part alignment leftovers (under-constrained DISTANCE pairs, the
    axial slide of an unmodelled face contact) and pure coupling mates
    (RACKPINION/GEAR) describe a relationship, not an attachment -- routing
    the spanning tree through one of them parents a link to the wrong side
    (e.g. vial_pick's right fingertip ended up under the LEFT fingertip via
    their alignment mate instead of under its own bolted finger)."""
    note = jt_ax_note[2]
    if note and ("under-constrained" in note or "only axial slide" in note
                 or "screw-like" in note):
        return True
    if types and all(t in _COUPLING_MATES for t in types):
        return True
    return False


def _concentric_axis_of(rec):
    """(point, unit dir) of an edge's concentric/cylinder axis, or None."""
    try:
        recs = _dedup_geo(rec.get("mates") or [])
    except Exception:
        return None
    for m in recs:
        if m.get("type") in ("CONCENTRIC", "HINGE"):
            pts, dirs = m.get("points") or [], m.get("dirs") or []
            if pts and dirs:
                d = np.asarray(dirs[0], float)
                n = float(np.linalg.norm(d))
                if n > 1e-9:
                    return np.asarray(pts[0], float), d / n
    return None


def _axis_translation_is_free(rel, d, tol=1e-4):
    """True if the relative-twist nullspace ``rel`` (6 x k: rows wx,wy,wz,
    vx,vy,vz) can realise a PURE translation along unit ``d`` (zero rotation).

    Used to second-guess the single-edge axial-slide weld with the assembly-
    wide solve: if the two bodies can still slide freely along the concentric
    axis once EVERY mate in the assembly is accounted for, that slide is a real
    linear guide (the printer's Z gantry on its rails), not a bolt's unmodelled
    face contact -- exactly what SolidWorks lets you drag."""
    if rel.shape[1] == 0:
        return False
    d = np.asarray(d, float)
    nd = float(np.linalg.norm(d))
    if nd < 1e-9:
        return False
    target = np.concatenate([np.zeros(3), d / nd])
    Q, _ = np.linalg.qr(rel)                  # orthonormal basis of col-space
    resid = target - Q @ (Q.T @ target)
    return float(np.linalg.norm(resid)) < tol


def _demote_globally_locked(comps, adjacency, edge):
    """Replicate SolidWorks' GLOBAL drag solve.

    Pairwise mates can leave a rotation free while other paths through the
    assembly lock it (a cover hinge-mated to its twin but ALSO pinned to the
    frame on a different axis can rotate about neither).  Build the whole-
    assembly constraint system -- 6-DOF twist per component, every mate of
    every edge contributing rows on the RELATIVE twist of its pair -- and
    keep a movable edge only if the system's nullspace contains a motion in
    which that pair actually moves relative to each other."""
    names = sorted({n for key in edge for n in key})
    if not names or len(names) > 400:        # keep builds bounded
        return
    idx = {n: i for i, n in enumerate(names)}
    path_of = {c.name: c.part_path for c in comps}
    rows = []
    for key, rec in adjacency.items():
        a, b = tuple(key)
        if a not in idx or b not in idx:
            continue
        # a SolidWorks limit mate IS a slider/hinge -- its DISTANCE/ANGLE mate
        # rows would (over-)constrain the global system and falsely lock the
        # mechanism, so keep them out (and never demote it, below)
        if rec.get("limit_joint"):
            continue
        # CLOCKING mates encode a display pose, not structure, yet they
        # really do freeze the mechanism in SolidWorks.  Two shapes:
        #  - pure-alignment between two instances of the SAME part (wheel
        #    faces kept parallel) without any concentric;
        #  - orientation-only edges (PARALLEL/ANGLE and nothing else) --
        #    they carry no position at all, only a chosen pose (a hinge
        #    arm held parallel to the cover).
        # Keep both out of the global system so they cannot lock joints.
        types = set(rec.get("types") or [])
        if path_of.get(a) and path_of.get(a) == path_of.get(b) \
                and "CONCENTRIC" not in types:
            continue
        if types and types <= {"PARALLEL", "ANGLE"}:
            continue
        # pure plane-on-plane edges (COINCIDENT and nothing else) BETWEEN
        # different sub-assembly instances are resting contacts, not
        # structure; through a loop they freeze real hinges (vial gripper:
        # the stopper ring resting on linkB welded the A<->B hinge).
        # WITHIN one instance a plane stack is deliberate construction
        # (mechanum's battery box spacers) and must keep its rows.
        if types and types <= {"COINCIDENT"}:
            inst_a = a.rsplit("/", 1)[0] if "/" in a else None
            inst_b = b.rsplit("/", 1)[0] if "/" in b else None
            if inst_a != inst_b:
                continue
        try:
            recs = _dedup_geo(rec.get("mates") or [])
        except Exception:
            continue
        axes, flags = [], set()
        for m in recs:
            for r6 in _mate_rows(m, axes, flags):
                row = np.zeros(6 * len(names))
                row[6 * idx[a]:6 * idx[a] + 6] = -r6
                row[6 * idx[b]:6 * idx[b] + 6] = r6
                rows.append(row)
    if not rows:
        return
    A = np.asarray(rows)
    _, s, vt = np.linalg.svd(A, full_matrices=True)
    tol = max(s[0] * 1e-8, 1e-12) if len(s) else 1e-12
    rank = int((s > tol).sum())
    N = vt[rank:].T                          # (6n) x k, includes 6 rigid modes
    demoted = 0
    rigid = set()
    for key in list(edge.keys()):
        a, b = tuple(key)
        if adjacency.get(key, {}).get("limit_joint"):
            continue                          # authoritative slider/hinge
        rel = N[6 * idx[b]:6 * idx[b] + 6, :] - N[6 * idx[a]:6 * idx[a] + 6, :]
        if np.linalg.norm(rel) < 1e-6:
            rigid.add(key)
            jt, ax, note = edge[key]
            if jt in _MOVABLE_TYPES:
                edge[key] = ("fixed", None,
                             (note or "geo:") + "; globally locked (pairwise "
                             "free, but no assembly-wide motion moves this "
                             "pair -- matches the SolidWorks drag solve)")
                demoted += 1
    if demoted:
        print(f"      global solve: {demoted} pairwise-movable edge(s) "
              f"are locked by the rest of the assembly")
    # The reverse of demotion: classify_edge_geo welds a lone concentric-axis
    # slide to fixed because, edge-on-its-own, it cannot tell a linear guide
    # from a bolt's unmodelled face contact.  The assembly-wide solve CAN -- if
    # the pair still slides freely along that axis with EVERY mate accounted
    # for, it is a real prismatic (the Z gantry rides its rails), so promote it
    # back.  Tagged fasteners never reach here (their edge is already "fastener
    # welded fixed", a different note), so this won't resurrect the bolt forest.
    promoted = 0
    for key in list(edge.keys()):
        jt, _ax, note = edge[key]
        if jt != "fixed" or not note or "only axial slide" not in note:
            continue
        if key in rigid:
            continue                          # global solve says it's locked
        a, b = tuple(key)
        if a not in idx or b not in idx:
            continue
        cax = _concentric_axis_of(adjacency.get(key, {}))
        if cax is None:
            continue
        rel = N[6 * idx[b]:6 * idx[b] + 6, :] - N[6 * idx[a]:6 * idx[a] + 6, :]
        if not _axis_translation_is_free(rel, cax[1]):
            continue
        edge[key] = ("prismatic", cax,
                     "geo: concentric slide the global solve proves FREE "
                     "(linear guide, not a fastener) -> prismatic")
        promoted += 1
    if promoted:
        print(f"      global solve: {promoted} concentric slide(s) are FREE in "
              f"the assembly -> prismatic (linear guide, not a fastener)")
        # Two PARALLEL promoted slides off a common frame part, whose moving
        # ends are directly tied, are one carriage on twin rails (the gantry's
        # two rail holders): only ONE is the real DOF.  Weld the tie between
        # them rigid so the spanning tree keeps the carriage in one group and
        # drops the second rail as a loop closure -- otherwise the second
        # holder is a phantom slider that floats off when the gantry moves.
        prom = [k for k in edge if "linear guide" in (edge[k][2] or "")]
        welded = 0
        for i in range(len(prom)):
            for j in range(i + 1, len(prom)):
                k1, k2 = prom[i], prom[j]
                common = k1 & k2
                if len(common) != 1:
                    continue
                frame = next(iter(common))
                c1 = next(iter(k1 - {frame}))
                c2 = next(iter(k2 - {frame}))
                d1 = np.asarray(edge[k1][1][1], float)
                d2 = np.asarray(edge[k2][1][1], float)
                if abs(float(d1 @ d2)) < 0.999:
                    continue                  # not the same rail direction
                tie = frozenset((c1, c2))
                if tie in edge and edge[tie][0] == "fixed" and tie not in rigid:
                    rigid.add(tie)
                    welded += 1
        if welded:
            print(f"      global solve: {welded} twin-rail tie(s) welded "
                  f"rigid (one carriage, one slide DOF)")
    return rigid


def _demote_coaxial_duplicates(edge, adjacency):
    """Within ONE expanded sub-assembly instance, several mate pairs can ride
    the same physical axis (output bearing + far-side flange of a servo).
    Only one of them is the joint; the rest are support bearings -- left
    movable they add phantom DOF and half the unit (servo body included)
    spins.  Keep the largest-radius edge per axis line, fix the others."""
    def prefix(name):
        return name.rsplit("/", 1)[0] if "/" in name else None

    def radius_of(key):
        radii = [r for m in adjacency.get(key, {}).get("mates", [])
                 if m.get("type") == "CONCENTRIC"
                 for r in (m.get("radii") or []) if r]
        return max(radii) if radii else 0.0

    groups = {}
    for key, (jt, ax, _note) in edge.items():
        if jt not in _MOVABLE_TYPES or ax is None:
            continue
        prefs = {prefix(n) for n in key}
        if len(prefs) != 1 or None in prefs:
            continue                      # only inside one expanded instance
        groups.setdefault(next(iter(prefs)), []).append(key)

    for _pref, keys in groups.items():
        keys.sort(key=lambda k: (-radius_of(k), sorted(k)))
        kept = []
        for key in keys:
            jt, ax, note = edge[key]
            p = np.asarray(ax[0], float)
            d = np.asarray(ax[1], float)
            dup = any(abs(float(d @ kd)) > 1.0 - 1e-6
                      and np.linalg.norm(np.cross(p - kp, kd)) < 1e-4
                      for kp, kd in kept)
            if dup:
                edge[key] = ("fixed", None,
                             (note or "geo:") + "; coaxial support bearing "
                             "of an existing joint -> fixed")
            else:
                kept.append((p, d))


def _mirror_axis_fallback(comps, edge_info, inherit_type=False):
    """Give a mate-less child the joint of its mated twin.

    A SolidWorks mirror-feature copy or pattern copy carries no mates, so its
    edge reaches here with no axis (and, in the auto tree, no joint type).  But
    the SAME part is a real joint on the original side: the axis is a feature of
    that part, identical in the part's own frame, so reflect each twin's axis
    into this child's pose -- ``ax = W_child . W_twin^-1 . ax_twin`` -- and
    accept it when every same-part twin AGREES up to sign (four identical wheels
    share one spin axis; a part used in unrelated roles stays ambiguous and is
    left untouched).  With ``inherit_type`` the twin's joint TYPE is copied too
    (the auto tree has no mate to type the copy, so a mirrored revolute would
    otherwise weld as a fixed link)."""
    by_name = {c.name: c for c in comps}
    axis_by_child = {ch: info["axis"] for (ch, _pa), info in edge_info.items()
                     if info.get("axis") is not None}
    type_by_child = {ch: info.get("type") for (ch, _pa), info in
                     edge_info.items() if info.get("axis") is not None}
    for (child, parent), info in edge_info.items():
        if info.get("axis") is not None:
            continue
        cR = by_name.get(child)
        if cR is None or not cR.part_path:
            continue
        if cR.is_fastener:
            continue          # hardware welds fixed -- no twin spin axis to copy
        # a part RIGIDLY fixed onto its OWN same-part twin rides that twin's
        # joint (the gantry's two rail holders are one carriage on one slide);
        # inheriting a second copy of the joint here would double the DOF and
        # the part would float off when the real joint moves -- leave it fixed.
        pR = by_name.get(parent)
        if info.get("type") == "fixed" and pR is not None \
                and pR.part_path == cR.part_path:
            continue
        sibs = [ch for ch in axis_by_child
                if ch != child and by_name.get(ch)
                and by_name[ch].part_path == cR.part_path]
        cands = []
        WR = np.asarray(cR.world, float)
        for s in sibs:
            try:
                T = WR @ np.linalg.inv(np.asarray(by_name[s].world, float))
            except np.linalg.LinAlgError:
                continue
            axS = axis_by_child[s]
            pt = (T @ np.append(np.asarray(axS[0], float), 1.0))[:3]
            d = T[:3, :3] @ np.asarray(axS[1], float)
            nrm = float(np.linalg.norm(d))
            if nrm > 1e-9:
                cands.append((pt, d / nrm))
        if not cands:
            continue
        d0 = cands[0][1]
        if not all(abs(float(d0 @ d)) > 0.99 for _, d in cands):
            continue                      # siblings disagree -> truly ambiguous
        info["axis"] = cands[0]
        info["mirrored_axis"] = True
        if inherit_type and info.get("type") not in _MOVABLE_TYPES:
            ttypes = {type_by_child[s] for s in sibs
                      if type_by_child.get(s) in _MOVABLE_TYPES}
            if len(ttypes) == 1:
                info["type"] = next(iter(ttypes))
                info["lower"], info["upper"] = (
                    (-0.05, 0.05) if info["type"] == "prismatic"
                    else (-3.141592, 3.141592))
    return edge_info


def _auto_parent_map(comps, adjacency, base):
    """Spanning forest rooted at base -> (parent_of, edge_info).

    Three-tier BFS.  Tier 0: edges the GLOBAL solve proves rigid (zero
    relative motion in every assembly-wide mode) -- these define the rigid
    groups, so a frame bolted to the base never gets parented through a
    support bearing.  Tier 1: movable edges (the real joints between rigid
    groups); a redundant coaxial bearing of the same physical hinge then
    falls out naturally as a loop closure.  Tier 2: weak edges (alignment
    leftovers, couplings) as a last resort.  On graphs without mate
    geometry every edge is tier 0 and the traversal is the plain sorted
    BFS it always was.

    edge_info[(child,parent)] = {type, axis:(pt,dir)|None, lower, upper}."""
    neighbors = {c.name: [] for c in comps}
    edge = {}
    for key, rec in adjacency.items():
        a, b = tuple(key)
        if a not in neighbors or b not in neighbors:
            continue
        edge[key] = classify_edge_auto(rec)
        neighbors[a].append(b)
        neighbors[b].append(a)
    rigid = _demote_globally_locked(comps, adjacency, edge) or set()
    # A fixed edge the classifier is CONFIDENT about (a bolted/fastener mount, an
    # explicit weld, a fully-constrained pair) is rigid STRUCTURE even when the
    # global twist solve left it a spurious slide (an unmodelled face contact
    # behind a bolt's concentric).  Treat it as backbone so a bolted sub-frame
    # (e.g. the rail holder on the body) stays grouped with its parent instead of
    # being entered through a joint from the far side.  "under-constrained" stays
    # weak -- that one really is a low-confidence guess.
    for key, (jt, _ax, note) in edge.items():
        if jt == "fixed" and key not in rigid and any(
                s in (note or "") for s in
                ("fastener", "axial slide", "fully constrained",
                 "force_fixed", "welded")):
            rigid.add(key)
    _demote_coaxial_duplicates(edge, adjacency)
    weak = set()
    for key, rec in adjacency.items():
        if key in edge and _edge_is_weak(edge[key], rec.get("types")):
            weak.add(key)
    forced = {key for key, rec in adjacency.items()
              if rec.get("force_fixed") and key in edge}
    # A FIXED edge whose BOTH ends are independently driven (each carries its
    # own movable joint) is a loop closure / alignment tie -- e.g. two motorised
    # mecanum wheels barrel-mated to each other -- never a structural parent.
    # Defer it below everything so each part attaches through its OWN joint and
    # the redundant tie is dropped as a loop closure, instead of one wheel being
    # parented to the other and stealing its motor into its subtree.
    driven = set()
    for key, (jt, ax, _note) in edge.items():
        if jt in _MOVABLE_TYPES and ax is not None:
            driven.update(key)
    # ... but a GLOBALLY-RIGID fixed edge is structure, not a loop closure, even
    # when both ends also carry a movable joint: the bed plate is fixed to the
    # bed carriage while each separately slides, so this edge keeps the two in
    # ONE rigid group instead of being split across the tree.
    loop_closure = {key for key, (jt, _ax, _n) in edge.items()
                    if jt == "fixed" and key not in rigid
                    and all(n in driven for n in key)}
    # Synthetic LOCK edges tie a sub-assembly's grounded children (fixed to its
    # frame) rigidly together.  They carry no mate geometry, so the global twist
    # solve cannot see them as rigid and they fall to the weak tier -- then a
    # grounded part reachable ALSO through a revolute joint (a wheel unit fixed
    # to the movebase frame but spun by its motor) gets attached through the
    # joint, inverting the hierarchy.  Treat a LOCK as the rigid tie it is.
    locked = {key for key, rec in adjacency.items()
              if key in edge and "LOCK" in (rec.get("types") or [])}

    def tier(key):
        if key in forced or key in locked:
            return 0
        if key in loop_closure:
            return 3
        if rigid:
            if key in rigid:
                return 0
            return 1 if edge[key][0] in _MOVABLE_TYPES else 2
        return 1 if key in weak else 0

    by_name = {c.name: c for c in comps}

    def dist2(name):
        t = by_name[name].world[:3, 3]
        return float(t @ t)

    visited = {base.name}
    parent_of = {}
    mate_less = set()        # names attached by the no-mate fallback (untrusted)

    def bfs_within(max_tier, roots):
        queue = list(roots)
        while queue:
            cur = queue.pop(0)
            for nb in _pref_sorted(cur, neighbors[cur]):
                if nb not in visited \
                        and tier(frozenset((cur, nb))) <= max_tier:
                    visited.add(nb)
                    parent_of[nb] = cur
                    queue.append(nb)

    def _inst(name):
        return name.rsplit("/", 1)[0] if "/" in name else None

    def _edge_pref(cur, nb):
        """Higher = more natural parent: prefer the SAME expanded
        sub-assembly instance (a bearing inside linkB stays in linkB's
        branch instead of jumping to whatever cross-instance alignment
        happens to sort first), then the mate-richer edge."""
        rec = adjacency.get(frozenset((cur, nb)), {})
        types = rec.get("types") or []
        same = _inst(cur) is not None and _inst(cur) == _inst(nb)
        return (1 if same else 0,
                sum(1 for t in types if t == "CONCENTRIC"),
                len(types))

    def _pref_sorted(cur, nbs):
        # best preference first; alphabetical for ties (deterministic, and
        # matches the old plain-sorted behaviour when no preference applies)
        return sorted(nbs, key=lambda nb: (
            tuple(-x for x in _edge_pref(cur, nb)), nb))

    def _twin_parent_parts(root):
        """Parent PARTS of ``root``'s mated twins (other instances of the same
        part placed through REAL mates) -- the connection(s) the mirror copy can
        reuse."""
        rp = by_name[root].part_path
        if not rp:
            return set()
        out = set()
        for c in comps:
            if (c.name != root and c.part_path == rp
                    and c.name in visited and c.name not in mate_less
                    and c.name in parent_of):
                par = by_name.get(parent_of[c.name])
                if par is not None and par.part_path:
                    out.add(par.part_path)
        return out

    def _twin_host(root, rpos):
        """Mirror/copy rule for a mate-less part: SolidWorks 'Mirror Components'
        and pattern copies carry NO mates, so a mirrored part (e.g. the right
        gripper finger) reaches here with nothing to connect to and the plain
        nearest-component fallback wrongly bolts it to whatever sits closest --
        often a SIBLING copy, not its real mount.  The SAME part is mated on the
        original side, so reuse that: attach ``root`` to the nearest instance of
        a twin's parent PART -- i.e. connect the copy exactly like its mirror
        original.  When the part is used under several different parents (3 arm
        instances on different mounts), disambiguate by geometry: take the
        parent-part whose nearest instance is closest to ``root``.  Commit only
        if that instance is already placed; otherwise (the same-side parent is
        not reached yet) defer, so the rule never crosses sides."""
        cands = []
        for tgt in _twin_parent_parts(root):
            insts = [c.name for c in comps if c.part_path == tgt]
            if insts:
                n = min(insts, key=lambda x: float(np.sum(
                    (by_name[x].world[:3, 3] - rpos) ** 2)))
                d = float(np.sum((by_name[n].world[:3, 3] - rpos) ** 2))
                cands.append((d, n))
        if not cands:
            return None
        cands.sort()
        _d, host = cands[0]            # the closest mirror parent
        return host if host in visited else None

    def _depth(name):
        d = 0
        while name in parent_of and d <= len(comps):
            name = parent_of[name]
            d += 1
        return d

    def _twin_depth(root):
        """Depth in the mated (original) tree of ``root``'s shallowest twin --
        how far the limb's anchor is from the base.  Big when ``root`` has no
        mated twin (a true island), so those order by distance instead.  The
        base itself counts (a part whose twin IS the root is the limb anchor)."""
        rp = by_name[root].part_path
        if not rp:
            return 1 << 30
        ds = [_depth(c.name) for c in comps
              if c.name != root and c.part_path == rp
              and c.name in visited and c.name not in mate_less
              and (c.name in parent_of or c.name == base.name)]
        return min(ds) if ds else (1 << 30)

    while True:
        bfs_within(0, [base.name, *sorted(parent_of)])
        # attach ONE component over the lowest-tier edge available --
        # picking the BEST such edge (instance affinity, mate richness),
        # then resume rigid expansion from it
        attached = False
        for want in (1, 2, 3):
            cands = [(cur, nb) for cur in visited
                     for nb in neighbors[cur]
                     if nb not in visited
                     and tier(frozenset((cur, nb))) <= want]
            if cands:
                cands.sort(key=lambda e: (
                    tuple(-x for x in _edge_pref(*e)), e[0], e[1]))
                cur, nb = cands[0]
                visited.add(nb)
                parent_of[nb] = cur
                attached = True
                break
        if attached:
            continue
        rem = [c.name for c in comps if c.name not in visited]
        if not rem:
            break
        # mate-less / disconnected island.  PREFER the mirror/copy rule: among
        # the unplaced parts attach any whose mated twin's connection can be
        # mirrored NOW (its same-side parent is already placed).  Doing these
        # first lets a whole mirrored limb cascade in tree order -- shoulder
        # before elbow before wrist -- instead of a child being reached before
        # its parent and welding to whatever sits closest.
        mirrorable = [(r, _twin_host(r, by_name[r].world[:3, 3])) for r in rem]
        mirrorable = [(r, h) for r, h in mirrorable if h is not None]
        if mirrorable:
            root, host = min(mirrorable, key=lambda rh: dist2(rh[0]))
            print(f"      note: '{root}' has no mates; mirroring its mated "
                  f"twin's connection -> '{host}'")
        else:
            # Nothing can mirror yet -> a whole mirrored limb is waiting on its
            # own anchor (the part whose twin sits highest in the original tree,
            # e.g. the one whose twin is the root).  Break the deadlock THERE:
            # attach the leftover with the SHALLOWEST twin nearest-component, so
            # once it lands everything below it mirrors in tree order.  Distance
            # to base breaks ties and orders true islands with no twin.  An
            # unmated connector on a moving link thus rides that link, not the
            # base.
            root = min(rem, key=lambda r: (_twin_depth(r), dist2(r)))
            rpos = by_name[root].world[:3, 3]
            host = min(visited,
                       key=lambda n: float(np.sum(
                           (by_name[n].world[:3, 3] - rpos) ** 2)))
            print(f"      note: '{root}' has no usable mates; attaching to "
                  f"nearest component '{host}'")
        parent_of[root] = host
        visited.add(root)
        mate_less.add(root)

    edge_info = {}
    for child, parent in parent_of.items():
        jt, ax, note = edge.get(frozenset((child, parent)),
                                ("fixed", None, None))
        # a fastener welds rigidly to its host -- never a hinge, even when it was
        # attached mate-less (nearest/twin) and would otherwise default-then-
        # inherit a movable axis from the mirror fallback below
        if by_name[child].is_fastener or by_name[parent].is_fastener:
            jt, ax, note = "fixed", None, "fastener welded fixed"
        lo, hi = (-0.05, 0.05) if jt == "prismatic" else (-3.141592, 3.141592)
        # a SolidWorks limit mate carries the real axis + travel range; orient it
        # parent -> child so the slider moves the way it does in SolidWorks
        lj = adjacency.get(frozenset((child, parent)), {}).get("limit_joint")
        if lj and jt == lj["type"]:
            ax, lo, hi = _oriented_limit(lj, by_name[child], by_name[parent])
        edge_info[(child, parent)] = {"type": jt, "axis": ax, "note": note,
                                      "lower": lo, "upper": hi}
    # mate-less mirror/pattern copies inherit the joint (type + reflected axis)
    # of their mated twin, so the right gripper finger is a revolute like the
    # left one instead of a dead fixed link
    _mirror_axis_fallback(comps, edge_info, inherit_type=True)
    return parent_of, edge_info


_MOVABLE_TYPES = ("revolute", "continuous", "prismatic")


def _oriented_limit(lj, child_comp, parent_comp):
    """(axis, lower, upper) for a limit-mate joint, oriented for the tree CHILD.

    The extracted axis is oriented so the reference component ``lj['ref']``
    (=mate side ``a``) moving +axis GROWS the mate distance, and lower/upper are
    that growing travel.  In the URDF the joint moves the CHILD: if the child IS
    the reference, +axis already grows the distance; if it's the other side, its
    + motion shrinks it, so flip the axis -- then ``joint > 0`` always travels
    toward the maximum, matching the SolidWorks slider."""
    pt, d = lj["axis"]
    d = np.asarray(d, float)
    if lj.get("ref") and child_comp.name != lj["ref"]:
        d = -d
    return (np.asarray(pt, float), d), lj["lower"], lj["upper"]


def _auto_mimic(comps, adjacency, parent_of, edge_info):
    """Couple joints that a RACKPINION/GEAR mate ties to a common component.

    A dual-rack gripper mates BOTH racks to the same pinion; each rack's
    nearest movable tree ancestor is one prismatic jaw joint.  Their absolute
    motions are equal and opposite, so the follower mimics the master with
    multiplier -1 (independent jaws) or -2 (jaw2 is a tree CHILD of jaw1, so
    its joint value is relative to jaw1).  Only prismatic pairs are coupled;
    gear ratios for revolute pairs need tooth/radius data we don't trust yet."""
    link_of = {c.name: c.link_name for c in comps}

    def movable_anchor(name):
        cur = name
        while cur in parent_of:
            par = parent_of[cur]
            info = edge_info.get((cur, par))
            if info and info["type"] in _MOVABLE_TYPES:
                return (cur, par)
            cur = par
        return None

    def ancestors(name):
        out = []
        while name in parent_of:
            name = parent_of[name]
            out.append(name)
        return out

    partners = {}
    for key, rec in adjacency.items():
        if not (set(rec.get("types") or []) & _COUPLING_MATES):
            continue
        a, b = tuple(key)
        partners.setdefault(a, set()).add(b)
        partners.setdefault(b, set()).add(a)

    for common, parts in sorted((k, v) for k, v in partners.items()):
        if len(parts) != 2:
            continue
        q1, q2 = sorted(parts)
        j1, j2 = movable_anchor(q1), movable_anchor(q2)
        if not j1 or not j2 or j1 == j2:
            continue
        i1, i2 = edge_info[j1], edge_info[j2]
        if i1["type"] != "prismatic" or i2["type"] != "prismatic":
            continue
        if i1.get("axis") is None or i2.get("axis") is None:
            continue
        a1 = np.asarray(i1["axis"][1], float)
        a2 = np.asarray(i2["axis"][1], float)
        dot = float(a1 @ a2)
        if abs(dot) < 0.999:                  # racks must share the slide axis
            continue
        if j1[0] in ancestors(j2[0]):
            master, follower, mult = j1, j2, -2.0 * dot
        elif j2[0] in ancestors(j1[0]):
            master, follower, mult = j2, j1, -2.0 * dot
        else:
            master, follower, mult = j1, j2, -1.0 * dot
        fi = edge_info[follower]
        if fi.get("mimic"):
            continue
        mname = f"{link_of[master[1]]}__{link_of[master[0]]}"
        fi["mimic"] = {"joint": mname, "multiplier": round(mult, 6),
                       "offset": 0.0}
        # the follower must be able to reach multiplier x the master's whole
        # range, otherwise it clamps mid-stroke and the coupling freezes
        mi = edge_info[master]
        fi["lower"], fi["upper"] = sorted((mult * mi["lower"],
                                           mult * mi["upper"]))
        note = fi.get("note") or "geo:"
        fi["note"] = note + (f"; mimic {mname} x{round(mult, 3)} "
                             f"(dual rack on {link_of.get(common, common)})")


def _circle_isect(c0, r0, c1, r1, branch):
    """One intersection of circles (c0,r0)/(c1,r1) in the plane, or None.

    ``branch`` (+1/-1) picks which of the two; the caller fixes it once to the
    assembled pose so the four-bar tracks the same configuration as it sweeps."""
    d = np.linalg.norm(c1 - c0)
    if d < 1e-12 or d > r0 + r1 or d < abs(r0 - r1):
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h2 = r0 * r0 - a * a
    if h2 < 0:
        return None
    m = c0 + a * (c1 - c0) / d
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / d
    return m + branch * np.sqrt(h2) * perp


def _auto_loop_mimic(comps, adjacency, parent_of, edge_info, base):
    """Couple the passive joints of a closed-loop *planar four-bar* to its driver.

    A SolidWorks closed linkage (a needle-holder pantograph, a parallel-jaw
    gripper, ...) is fully constrained: four revolute hinges about parallel axes
    around one loop, so only ONE joint is really free.  The spanning tree keeps
    three of the four hinges as independent revolute joints and drops the fourth
    as a loop closure; with no coupling those three move independently and the
    linkage flies apart.  Detect the four-bar, keep the most crank-like grounded
    hinge as the driver, and ``<mimic>`` the other two tree hinges to it with the
    multiplier read off the linkage geometry.

    The ratio is exact only for a parallelogram, but a general four-bar's
    velocity ratio is near-constant over a wide travel, so a least-squares
    linear fit over a +-25 deg sweep tracks it closely -- far better than the
    frozen/free joints we would emit otherwise."""
    link_of = {c.name: c.link_name for c in comps}

    # rigid groups: union components joined by a FIXED tree edge, so a hinge
    # whose far link is bolted to the base reads as a hinge to GROUND.
    uf = {c.name: c.name for c in comps}

    def find(x):
        while uf[x] != x:
            uf[x] = uf[uf[x]]
            x = uf[x]
        return x

    movable_tree = {}
    for child, par in parent_of.items():
        info = edge_info.get((child, par), {})
        if info.get("type") in _MOVABLE_TYPES and info.get("axis") is not None:
            movable_tree[(child, par)] = info
        else:
            uf[find(child)] = find(par)

    tree_pairs = {frozenset((c, p)) for c, p in parent_of.items()}

    def root_path(n):
        out = [n]
        while n in parent_of:
            n = parent_of[n]
            out.append(n)
        return out

    for key, rec in adjacency.items():
        if key in tree_pairs:
            continue
        jt, ax, _note = classify_edge_auto(rec)
        if jt not in _MOVABLE_TYPES or ax is None:
            continue                          # only a movable loop closure
        a, b = tuple(key)
        if a not in parent_of or b not in parent_of:
            continue
        # fundamental cycle: tree paths to the root, cut at their LCA
        pa, pb = root_path(a), root_path(b)
        sb = set(pb)
        lca = next((x for x in pa if x in sb), None)
        if lca is None:
            continue
        # the loop's GROUND bar is the rigid group where its two branches rejoin
        # the tree toward the root (the LCA) -- NOT necessarily the URDF root, so
        # the four-bar is still found when the part is rooted elsewhere (e.g. at
        # a module-mount connector reached through an extra joint)
        ground = find(lca)
        ring = pa[:pa.index(lca) + 1] + list(reversed(pb[:pb.index(lca)]))
        # the four real hinges around the loop, in ring order
        hinges = []
        for u, v in zip(ring, ring[1:] + ring[:1]):
            if (u, v) in movable_tree:
                info, tedge = movable_tree[(u, v)], (u, v)
            elif (v, u) in movable_tree:
                info, tedge = movable_tree[(v, u)], (v, u)
            elif frozenset((u, v)) == key:
                info, tedge = {"axis": ax}, None      # the dropped closure
            else:
                continue                              # fixed edge -> collapses
            pt, d = info["axis"]
            hinges.append({"pt": np.asarray(pt, float),
                           "d": np.asarray(d, float),
                           "tedge": tedge, "gu": find(u), "gv": find(v)})
        groups = []
        for n in ring:
            g = find(n)
            if not groups or groups[-1] != g:
                groups.append(g)
        if groups and groups[0] == groups[-1]:
            groups.pop()
        # one 1-DOF loop = 4 distinct rigid groups, 4 hinges (3 tree + closure)
        if len(groups) != 4 or len(hinges) != 4:
            continue
        if sum(1 for h in hinges if h["tedge"] is None) != 1:
            continue
        if ground not in groups:
            continue                          # a free-floating loop -- skip
        # all hinge axes must be parallel (a planar single-DOF linkage)
        n_hat = hinges[0]["d"] / (np.linalg.norm(hinges[0]["d"]) or 1.0)
        if any(abs(abs(h["d"] @ n_hat /
                       (np.linalg.norm(h["d"]) or 1.0)) - 1.0) > 1e-3
               for h in hinges):
            continue
        # planar basis perpendicular to the common axis; project pivots to 2D
        e1, e2 = _perp_basis(n_hat)
        for h in hinges:
            h["p2"] = np.array([h["pt"] @ e1, h["pt"] @ e2])
        ground_h = [h for h in hinges if ground in (h["gu"], h["gv"])]
        if len(ground_h) != 2:
            continue                          # ground must be one contiguous bar
        drivers = [h for h in ground_h if h["tedge"] is not None]
        if not drivers:
            continue                          # both ground pivots dropped (rare)

        def depth(h):
            return len(root_path(h["tedge"][0]))

        def moving_group(h):
            return h["gu"] if h["gv"] == ground else h["gv"]

        def moving_bar(h):
            gm = moving_group(h)
            h2 = next((x for x in hinges
                       if x is not h and gm in (x["gu"], x["gv"])), None)
            return (np.linalg.norm(h["p2"] - h2["p2"])
                    if h2 is not None else float("inf"))
        # driver = the most crank-like grounded hinge: the shorter its moving
        # bar, the more it swings per unit linkage travel, so it is the natural
        # input (and the best-conditioned to drive).  Ties by depth then name.
        driver = min(drivers, key=lambda h: (round(moving_bar(h), 9), depth(h),
                                             f"{link_of[h['tedge'][1]]}__"
                                             f"{link_of[h['tedge'][0]]}"))
        other_ground = next(h for h in ground_h if h is not driver)
        g_in = moving_group(driver)
        g_out = moving_group(other_ground)
        g_cpl = next((g for g in groups
                      if g not in (ground, g_in, g_out)), None)
        if g_cpl is None or g_in == g_out:
            continue
        # the two non-ground hinges: one input<->coupler, one coupler<->output
        h_ic = next((h for h in hinges
                     if {h["gu"], h["gv"]} == {g_in, g_cpl}), None)
        h_oc = next((h for h in hinges
                     if {h["gu"], h["gv"]} == {g_out, g_cpl}), None)
        if h_ic is None or h_oc is None:
            continue

        Pd, Pe = driver["p2"], other_ground["p2"]
        Pi, Po = h_ic["p2"], h_oc["p2"]
        L_in = np.linalg.norm(Pi - Pd)
        L_cpl = np.linalg.norm(Po - Pi)
        L_out = np.linalg.norm(Po - Pe)
        if min(L_in, L_cpl, L_out) < 1e-6:
            continue
        phi0 = np.arctan2(*(Pi - Pd)[::-1])
        branch = None
        for br in (+1, -1):
            test = _circle_isect(Pi, L_cpl, Pe, L_out, br)
            if test is not None and np.linalg.norm(test - Po) < 1e-6:
                branch = br
                break
        if branch is None:
            continue

        in0 = np.arctan2((Pi - Pd)[1], (Pi - Pd)[0])
        cpl0 = np.arctan2((Po - Pi)[1], (Po - Pi)[0])
        out0 = np.arctan2((Po - Pe)[1], (Po - Pe)[0])

        def solve(dphi):
            """Group rotations {ground:0, in, coupler, out} at driver += dphi."""
            Pi_ = Pd + L_in * np.array([np.cos(phi0 + dphi),
                                        np.sin(phi0 + dphi)])
            Po_ = _circle_isect(Pi_, L_cpl, Pe, L_out, branch)
            if Po_ is None:
                return None
            d_in = np.arctan2((Pi_ - Pd)[1], (Pi_ - Pd)[0]) - in0
            d_cpl = np.arctan2((Po_ - Pi_)[1], (Po_ - Pi_)[0]) - cpl0
            d_out = np.arctan2((Po_ - Pe)[1], (Po_ - Pe)[0]) - out0
            return {ground: 0.0, g_in: d_in, g_cpl: d_cpl, g_out: d_out}

        def jval(h, dth):
            c, p = h["tedge"]
            s = 1.0 if (h["d"] @ n_hat) >= 0 else -1.0
            return s * (dth[find(c)] - dth[find(p)])

        # The loop itself bounds how far the driver can turn: a non-Grashof
        # four-bar binds at a TOGGLE (coupler & output go collinear) where
        # solve() has no solution.  Walk out from the home pose in each
        # direction until it binds -> the reachable driver arc.  This is the
        # real servo travel; the default +-pi is physically unreachable.
        sdrv = 1.0 if (driver["d"] @ n_hat) >= 0 else -1.0
        followers = [h for h in (h_ic, h_oc, other_ground)
                     if h is not driver and h["tedge"] is not None]
        step = np.radians(0.5)
        d0, d1 = solve(0.0), solve(step)
        if d0 is None or d1 is None or not followers:
            continue

        def frate(da, db):
            return max((abs(jval(f, da) - jval(f, db)) / step
                        for f in followers), default=0.0)
        # near a toggle the follower velocity ratio diverges; bound the usable
        # arc to where it stays well-conditioned (<= 4x the home gearing) so the
        # driver gets a real, drivable limit and the coupling fit stays accurate
        rate_cap = max(4.0 * frate(d1, d0), 3.0)

        def reach(direction):
            prev, d, last = d0, 0.0, 0.0
            while d < np.radians(178):
                d += step
                cur = solve(direction * d)
                if cur is None or frate(cur, prev) > rate_cap:
                    break
                last, prev = direction * d, cur
            return last
        hi_phi, lo_phi = reach(+1), reach(-1)
        # One toggle can be far (the input rocker can swing most of a turn the
        # "long way", which collisions we don't model would block anyway, and
        # over which the linear/cubic coupling is meaningless).  Take the NEAREST
        # toggle on either side and use it symmetrically: a safe arc that clears
        # both locks and keeps the coupling fit accurate.
        # also cap at 60 deg: a change-point linkage (parallelogram) keeps one
        # clean assembly branch only over a limited swing before the circle
        # solve flips to the other branch and the fit would be corrupted; 60 deg
        # is a safe, well-conditioned default (a CAD limit-mate can widen it)
        tight = min(hi_phi, -lo_phi, np.radians(60.0))
        tight -= min(np.radians(3.0), 0.08 * tight)    # margin off the toggle
        if tight < np.radians(1):
            continue                       # locked / degenerate -- not drivable

        mname = (f"{link_of[driver['tedge'][1]]}__"
                 f"{link_of[driver['tedge'][0]]}")
        di = edge_info[driver["tedge"]]
        # driver value q = sdrv * dphi.  Intersect the linkage arc with any
        # existing range (a SolidWorks limit-mate servo stop) so a real, tighter
        # CAD limit still wins.
        geo_lo, geo_hi = -tight, tight
        d_lo = geo_lo if di.get("lower") is None else max(geo_lo, di["lower"])
        d_hi = geo_hi if di.get("upper") is None else min(geo_hi, di["upper"])
        if d_hi - d_lo < np.radians(1):      # CAD limit disjoint -- keep linkage
            d_lo, d_hi = geo_lo, geo_hi
        di["lower"], di["upper"] = round(d_lo, 6), round(d_hi, 6)

        # sample the coupling over the REACHABLE range -> the poly fits the real
        # travel and each follower's limit is its actual swing (not mult*pi)
        qs = np.linspace(d_lo, d_hi, 25)
        dths = [(q, solve(q / sdrv)) for q in qs]
        dths = [(q, d) for q, d in dths if d is not None]
        if len(dths) < 4:
            continue
        qa = np.array([q for q, _ in dths])
        n_set = 0
        for follower in (h_ic, h_oc, other_ground):
            if follower is driver or follower["tedge"] is None:
                continue
            fi = edge_info[follower["tedge"]]
            if fi.get("mimic"):
                continue
            qf = np.array([jval(follower, d) for _, d in dths])
            den = float(qa @ qa)
            if den < 1e-12:
                continue
            mult = float(qa @ qf) / den
            # a cubic captures the four-bar's nonlinearity that the linear URDF
            # <mimic> cannot (a consumer can drive the follower by this poly for
            # an exact loop); highest-degree coeff first, numpy.polyval order
            deg = min(3, len(dths) - 1)
            poly = [round(float(c), 8) for c in np.polyfit(qa, qf, deg)]
            fi["mimic"] = {"joint": mname, "multiplier": round(mult, 6),
                           "offset": 0.0, "poly": poly}
            # the follower's real limit is the span it actually sweeps over the
            # driver's reachable arc
            fi["lower"], fi["upper"] = (round(float(qf.min()), 6),
                                        round(float(qf.max()), 6))
            note = fi.get("note") or "geo:"
            fi["note"] = note + (f"; mimic {mname} x{round(mult, 3)} "
                                 f"(four-bar loop closure)")
            n_set += 1
        if n_set:
            print(f"      four-bar loop: driver {link_of[driver['tedge'][1]]}"
                  f"->{link_of[driver['tedge'][0]]} "
                  f"range [{round(np.degrees(d_lo))},{round(np.degrees(d_hi))}] deg"
                  f", {n_set} mimic follower(s)")


def _collect_loop_closures(comps, adjacency, parent_of, edge_info, base):
    """General closed-loop data for the runtime-IK relay.

    For every dropped (non-tree) movable edge -- a loop the spanning tree had to
    cut -- record the two links it rejoined plus the hinge point/axis in
    base_link frame, and split the loop's movable joints into DRIVEN
    (independent) vs SOLVED (dependent).  The relay closes each loop numerically
    with skrobot FK, so NO four-bar-specific geometry is needed here: this works
    for any single-DOF-per-loop linkage (planar or spatial, revolute or
    prismatic).  Returns ``None`` when the assembly has no closed loop.

    Dependents are the loop joints carrying a ``<mimic>`` (the four-bar pass
    already chose those); on a loop the four-bar pass skipped, the joint nearest
    the root drives and the rest are solved."""
    link_of = {c.name: c.link_name for c in comps}
    wb_inv = np.linalg.inv(base.world)        # SW world -> base_link frame

    uf = {c.name: c.name for c in comps}

    def find(x):
        while uf[x] != x:
            uf[x] = uf[uf[x]]
            x = uf[x]
        return x

    movable_tree = {}
    for child, par in parent_of.items():
        info = edge_info.get((child, par), {})
        if info.get("type") in _MOVABLE_TYPES and info.get("axis") is not None:
            movable_tree[(child, par)] = info
        else:
            uf[find(child)] = find(par)
    tree_pairs = {frozenset((c, p)) for c, p in parent_of.items()}

    def root_path(n):
        out = [n]
        while n in parent_of:
            n = parent_of[n]
            out.append(n)
        return out

    def jname(te):                            # model joint name = parent__child
        c, p = te
        return f"{link_of[p]}__{link_of[c]}"

    def lname(comp):
        return "base_link" if comp == base.name else safe_name(link_of[comp])

    closures, dependent = [], set()
    for key, rec in adjacency.items():
        if key in tree_pairs:
            continue
        jt, ax, _n = classify_edge_auto(rec)
        if jt not in _MOVABLE_TYPES or ax is None:
            continue
        a, b = tuple(key)
        if a not in parent_of or b not in parent_of:
            continue
        pa, pb = root_path(a), root_path(b)
        sb = set(pb)
        lca = next((x for x in pa if x in sb), None)
        if lca is None:
            continue
        ring = pa[:pa.index(lca) + 1] + list(reversed(pb[:pb.index(lca)]))
        cyc = []
        for u, v in zip(ring, ring[1:] + ring[:1]):
            if (u, v) in movable_tree:
                cyc.append((u, v))
            elif (v, u) in movable_tree:
                cyc.append((v, u))
        if not cyc:
            continue
        mimic_deps = [te for te in cyc if edge_info[te].get("mimic")]
        if mimic_deps:
            dependent.update(jname(te) for te in mimic_deps)
        else:                                 # general loop: nearest-root drives
            driver = min(cyc, key=lambda te: (len(root_path(te[0])), jname(te)))
            dependent.update(jname(te) for te in cyc if te != driver)
        pt, d = ax
        p_b = (wb_inv @ np.append(np.asarray(pt, float), 1.0))[:3]
        d_b = wb_inv[:3, :3] @ np.asarray(d, float)
        closures.append({"link_a": lname(a), "link_b": lname(b),
                         "point": [round(float(x), 8) for x in p_b],
                         "axis": [round(float(x), 8) for x in d_b]})
    if not closures:
        return None
    movable_all = {jname(te) for te in movable_tree}
    return {"closures": closures, "dependent": sorted(dependent),
            "independent": sorted(movable_all - dependent)}


def _config_parent_map(comps, adjacency, base, directed):
    """Build parent_of + edge_info from an explicit directed joint list.

    The config defines the kinematic chain.  Axis geometry for a revolute edge
    comes from the config (axis_point/axis_dir) or, failing that, the concentric
    mate between the two components.  Components not mentioned are attached to
    the base with a fixed joint so the URDF stays a connected tree."""
    names = {c.name for c in comps}
    by_name = {c.name: c for c in comps}
    parent_of = {}
    edge_info = {}
    for d in directed:
        child, parent = d["child"], d["parent"]
        if child not in names or parent not in names or child == base.name:
            continue
        jtype = d.get("type", "fixed")
        rec = adjacency.get(frozenset((child, parent)), {})
        lj = rec.get("limit_joint")
        ax = None
        if d.get("axis_dir"):
            # explicit direction wins; a point is optional (default: child origin)
            pt = (np.asarray(d["axis_point"], float) if d.get("axis_point")
                  else by_name[child].world[:3, 3].copy())
            ax = (pt, np.asarray(d["axis_dir"], float))
        elif lj and jtype == lj["type"]:
            # a SolidWorks LimitDistance/LimitAngle mate is authoritative -- use
            # its CAD axis (the joints.yaml the editor rebuilds from stores the
            # TYPE but not the axis, so without this the limit joint silently
            # falls back to the world +Z default below), oriented parent -> child
            ax, _lo, _hi = _oriented_limit(lj, by_name[child], by_name[parent])
        else:
            ax = rec.get("axis")
            if ax is None and rec.get("mates"):
                # No CONCENTRIC mate axis -- e.g. a MIRRORED part whose hinge is
                # constrained by an ANGLE + coincident-plane mate set rather than
                # a concentric cylinder.  Derive the rotation axis from the full
                # mate geometry (the twist nullspace), the same way the auto
                # classifier does, so the configured revolute keeps its axis
                # instead of silently degrading to fixed.
                try:
                    geo = classify_edge_geo(rec["mates"])
                except Exception:
                    geo = None
                if geo and geo[1] is not None:
                    ax = geo[1]
        parent_of[child] = parent
        lo = d.get("lower"); up = d.get("upper")
        if lj and jtype == lj["type"]:        # oriented limit-mate travel
            _, olo, ohi = _oriented_limit(lj, by_name[child], by_name[parent])
            lo = olo if lo is None else lo
            up = ohi if up is None else up
        edge_info[(child, parent)] = {
            "type": jtype, "axis": ax,
            "lower": -3.141592 if lo is None else lo,
            "upper": 3.141592 if up is None else up,
            "mimic": d.get("mimic"),
            "effort": d.get("effort"), "velocity": d.get("velocity"),
            "dynamics": d.get("dynamics"), "safety": d.get("safety"),
            "calibration": d.get("calibration")}
    # Mirror / sibling fallback: a joint whose child got no axis (a SolidWorks
    # mirror-feature copy, or a wheel mated only by coincident planes) inherits
    # its mated twin's axis.  The config already supplies the joint TYPE here, so
    # only the axis is filled (inherit_type left off).
    _mirror_axis_fallback(comps, edge_info)
    # A movable joint the user asked for that STILL has no axis -- no CAD mate
    # (or a fully-constrained DISTANCE-locked one) and no mirror twin -- defaults
    # to world +Z through the child origin, so flipping a joint to prismatic /
    # revolute actually MOVES instead of silently snapping back to fixed.  Point
    # it with axis_dir (e.g. [0,1,0]).  EXCEPT a part whose same-part twin DID
    # resolve an axis: that one was left axis-less on purpose (ambiguous mirror).
    axed_parts = {by_name[ch].part_path
                  for (ch, _pa), info in edge_info.items()
                  if info.get("axis") is not None and by_name.get(ch)
                  and by_name[ch].part_path}
    for (child, parent), info in edge_info.items():
        if info["type"] in _MOVABLE_TYPES and info.get("axis") is None:
            cp = by_name[child].part_path
            if cp and cp in axed_parts:
                continue                      # ambiguous mirror twin -- leave it
            info["axis"] = (by_name[child].world[:3, 3].copy(),
                            np.array([0.0, 0.0, 1.0]))
            print(f"      note: {parent}->{child} {info['type']}: no CAD axis, "
                  f"defaulting to world +Z (set axis_dir to change)")
    # anything unlisted -> fixed to base
    for c in comps:
        if c.name != base.name and c.name not in parent_of:
            parent_of[c.name] = base.name
            edge_info[(c.name, base.name)] = {"type": "fixed", "axis": None,
                                              "lower": None, "upper": None}
    return parent_of, edge_info


def _finalize_tree(comps, adjacency, base, parent_of, edge_info, root_rpy=None,
                   root_z_offset=0.0, root_xyz=None):
    """Compute anchors, visual origins and Joint objects from a parent map."""
    by_name = {c.name: c for c in comps}

    # anchor: revolute links anchor a world-aligned frame on the rotation axis;
    # everything else anchors its own component frame.
    # The BASE link (root) is anchored at the base COMPONENT's own frame, so the
    # URDF root coordinate system IS that part's coordinate system (e.g. the
    # screwlock connector = "from_coords" per the NejiNeji convention).  Its
    # mesh then sits at the root origin (visual origin = identity).
    # An optional root_rpy re-orients the root frame (e.g. to put the connector
    # axis on +Z per the convention).
    base_anchor = base.world.copy()
    if root_rpy:
        base_anchor = base_anchor @ matrix_from_rpy(root_rpy)
    if root_xyz:
        # full origin shift in the (already re-oriented) root frame --
        # the generalisation of root_z_offset, used by the web editor's
        # click-to-align
        T = np.eye(4)
        T[:3, 3] = np.asarray(root_xyz, float)
        base_anchor = base_anchor @ T
    if root_z_offset:
        # slide the root origin along its (already-reoriented) +Z, e.g. onto the
        # connector / protrusion plane so Z=0 sits on that face.
        T = np.eye(4)
        T[2, 3] = root_z_offset
        base_anchor = base_anchor @ T
    anchor = {base.name: base_anchor}
    for child, parent in parent_of.items():
        info = edge_info.get((child, parent), {"type": "fixed", "axis": None})
        if info["type"] in ("revolute", "continuous", "prismatic") \
                and info.get("axis") is not None:
            pt, d = info["axis"]
            # Use the SolidWorks concentric-mate axis point verbatim (verified
            # to match SW exactly).  No projection -- that moved the frame to
            # the part origin, which is misleading for parts whose origin is
            # off their geometry.
            anchor[child] = frame_at_point(np.asarray(pt, float))
        else:
            anchor[child] = by_name[child].world.copy()

    for c in comps:
        rel = relative_matrix(anchor[c.name], c.world)
        c.visual_xyz, c.visual_rpy = matrix_to_xyz_rpy(rel)

    joints = []
    for child, parent in parent_of.items():
        ch = by_name[child]; pa = by_name[parent]
        rel = relative_matrix(anchor[parent], anchor[child])
        xyz, rpy = matrix_to_xyz_rpy(rel)
        info = edge_info.get((child, parent), {"type": "fixed", "axis": None})
        rec = adjacency.get(frozenset((child, parent)), {})
        jtype = info["type"]
        axis = lower = upper = sw_pt = sw_dir = None
        if jtype in ("revolute", "continuous", "prismatic"):
            if info.get("axis") is None:
                print(f"      WARN: {jtype} {pa.link_name}->{ch.link_name} "
                      f"has no axis; using fixed")
                jtype = "fixed"
            else:
                pt, d = info["axis"]
                d = np.asarray(d, float)
                axis = [round(float(x), 8) for x in d]
                lower = info.get("lower"); upper = info.get("upper")
                sw_pt = [round(float(x), 8) for x in np.asarray(pt, float)]
                sw_dir = axis
        if jtype == "fixed":
            # write the WOULD-BE axis (best concentric/mate line) even for
            # fixed joints: URDF consumers ignore <axis> on fixed, but the
            # web editor uses it to show a ghost axis on hover, so "this
            # could rotate here if you un-fix it" stays visible
            cand = info.get("axis") or rec.get("axis")
            if cand is None:
                for g in rec.get("mates") or []:
                    if g.get("dirs"):
                        cand = (None, g["dirs"][0])
                        break
            if cand is not None and cand[1] is not None:
                d = np.asarray(cand[1], float)
                n = float(np.linalg.norm(d))
                if n > 1e-9:
                    # joint axes are expressed in the CHILD anchor frame
                    d_local = anchor[child][:3, :3].T @ (d / n)
                    axis = [round(float(x), 8) for x in d_local]
        # physics sub-elements are meaningful only on movable joints; a joint
        # the config pinned to fixed drops them (URDF ignores them there anyway)
        movable = jtype in ("revolute", "continuous", "prismatic")
        joints.append(Joint(
            name=f"{pa.link_name}__{ch.link_name}",
            parent=pa.link_name, child=ch.link_name, jtype=jtype,
            xyz=xyz, rpy=rpy, axis=axis, lower=lower, upper=upper,
            effort=info.get("effort") if movable else None,
            velocity=info.get("velocity") if movable else None,
            dynamics=info.get("dynamics") if movable else None,
            safety=info.get("safety") if movable else None,
            calibration=info.get("calibration") if movable else None,
            mate_types=rec.get("types", []), mimic=info.get("mimic"),
            sw_axis_point=sw_pt, sw_axis_dir=sw_dir,
            geo_note=info.get("note")))
    return joints


def build_tree(comps, adjacency, base, directed=None, root_rpy=None,
               root_z_offset=0.0, root_xyz=None, closures_out=None):
    if directed:
        parent_of, edge_info = _config_parent_map(comps, adjacency, base,
                                                  directed)
    else:
        parent_of, edge_info = _auto_parent_map(comps, adjacency, base)
        _auto_mimic(comps, adjacency, parent_of, edge_info)
        _auto_loop_mimic(comps, adjacency, parent_of, edge_info, base)
    # closed-loop data for the runtime-IK relay (hinge in base_link frame, so use
    # the SAME base anchor the URDF root is built on, incl. any root re-orient)
    if closures_out is not None:
        base_anchor = base.world.copy()
        if root_rpy:
            base_anchor = base_anchor @ matrix_from_rpy(root_rpy)
        saved = base.world
        try:
            base.world = base_anchor
            lc = _collect_loop_closures(comps, adjacency, parent_of,
                                        edge_info, base)
        finally:
            base.world = saved
        if lc:
            closures_out.append(lc)
    return _finalize_tree(comps, adjacency, base, parent_of, edge_info,
                          root_rpy=root_rpy, root_z_offset=root_z_offset,
                          root_xyz=root_xyz)


# ====================================================================
# Extraction (SolidWorks, slow) <-> serializable GraphState
# ====================================================================

def extract_graph(doc, robot_name, source_assembly, progress=None):
    """SolidWorks -> internal (comps, adjacency, ground).

    Extracts ALL (non-suppressed) components and their mate graph.  Exclusion
    of parts is a BUILD-time decision, so nothing is excluded here.  Mesh files
    are filled in later (by mesh.export_meshes) before serializing.
    ``progress(link_name)`` is forwarded per component (see
    :func:`extract_components`)."""
    comps = extract_components(doc, progress=progress)
    adjacency, ground = build_mate_graph(doc, comps)
    mated = set()
    for key in adjacency:
        mated.update(key)
    for c in comps:
        if c.name not in mated and c.name not in ground:
            print(f"      WARN: '{c.name}' has NO mates in the top-level "
                  f"assembly -- it will be force-attached to the base "
                  f"(check for suppressed/lightweight mates)")
    _warn_unsolved_mates(comps, adjacency)
    return comps, adjacency, ground


def _warn_unsolved_mates(comps, adjacency):
    """Flag components whose saved pose ignores a CONCENTRIC mate.

    SolidWorks keeps the mate's entity axis at the SOLVED location even when
    the mate is suppressed/erroring, while the component transform stays at
    the last-dragged position.  Distance from a part's origin to a mate axis
    is meaningless in absolute terms (a long arm is mated at its tip), so
    compare INSTANCES of the same part in the same mate pattern: when some
    sit on their axis and a sibling sits decimetres off, the sibling's mate
    is not being solved."""
    pos = {c.name: c.world[:3, 3] for c in comps}
    path_of = {c.name: c.part_path for c in comps}
    # (owner part_path, partner part_path) -> [(owner instance, offset)]
    pattern = {}
    for key, rec in adjacency.items():
        a, b = tuple(key)
        for g in rec.get("mates", []):
            if g.get("type") != "CONCENTRIC":
                continue
            for owner, p, d in zip(g.get("owners", []), g.get("points", []),
                                   g.get("dirs", [])):
                top = _top_level(owner) if owner else None
                if top not in pos:
                    continue
                dvec = np.asarray(d, float)
                if np.linalg.norm(dvec) < 1e-9:
                    continue
                off = float(np.linalg.norm(np.cross(
                    pos[top] - np.asarray(p, float), dvec)))
                partner = b if top == a else a
                sig = (path_of.get(top), path_of.get(partner))
                per = pattern.setdefault(sig, {})
                # ONE representative value per owner INSTANCE: its smallest
                # offset.  A single instance legitimately carries several
                # mates at different distances (a cover bolted to two
                # boards) -- comparing those against each other mis-fires.
                if top not in per or off < per[top][0]:
                    per[top] = (off, p, d)
    flagged = {}
    for _sig, per in pattern.items():
        if len(per) < 2:               # needs at least two DISTINCT instances
            continue
        best = min(off for off, _, _ in per.values())
        for top, (off, p, d) in per.items():
            if top in flagged:
                continue
            if off > max(10 * best, 0.02) and off - best > 0.02:
                flagged[top] = (off, best, p, d)
                print(f"      WARN: '{top}' sits {off*1000:.0f} mm off its "
                      f"CONCENTRIC mate axis while a sibling instance sits "
                      f"at {best*1000:.0f} mm -- this mate is likely "
                      f"SUPPRESSED or erroring in SolidWorks; the exported "
                      f"pose is the dragged position")
    return flagged


def _snap_unsolved_mates(comps, adjacency):
    """Repair components whose saved pose ignores their CONCENTRIC mate.

    The mate's entity data remembers the SOLVED location while the saved
    transform is the dragged one (see :func:`_warn_unsolved_mates`).  The
    sibling instances both detect the outlier and license the fix: they show
    at what distance from the mate axis this part family is supposed to sit,
    so translate the outlier perpendicular to the axis until it matches.
    Rotation about the axis is left untouched."""
    flagged = _warn_unsolved_mates(comps, adjacency)
    if not flagged:
        return
    by_name = {c.name: c for c in comps}
    # original positions of every OTHER instance of each part, for the
    # collapse guard below (snapshot before any moves)
    same_part_pos = {}
    for c in comps:
        same_part_pos.setdefault(c.part_path, []).append(
            (c.name, c.world[:3, 3].copy()))
    for top, (off, best, p, d) in flagged.items():
        c = by_name.get(top)
        if c is None:
            continue
        pos = c.world[:3, 3]
        p = np.asarray(p, float)
        d = np.asarray(d, float)
        v = p - pos
        perp = v - d * float(d @ v)          # shortest path onto the axis
        if np.linalg.norm(perp) < 1e-12:
            continue
        move = perp * (1.0 - best / max(off, 1e-12))
        new_pos = pos + move
        # GUARD: the snap must not stack this part onto a SIBLING instance.
        # The same part is sometimes used in different roles (e.g. a finger
        # part as both the proximal AND the middle phalanx); the proximal one
        # legitimately sits the inter-joint distance off the shared mate axis,
        # so the "sibling sits at 0 mm" outlier test mis-fires and would snap
        # it right on top of its neighbour.  A genuine suppressed-mate repair
        # snaps to an EMPTY axis location, never onto another instance -- so if
        # the target coincides with a sibling, keep the exported pose instead.
        clash = min((float(np.linalg.norm(new_pos - q))
                     for nm, q in same_part_pos.get(c.part_path, [])
                     if nm != top), default=np.inf)
        if clash < 0.003:                    # 3 mm: a clear overlap, not a repair
            print(f"      skipped auto-correct of '{top}': snapping "
                  f"{np.linalg.norm(move)*1000:.1f} mm would stack it on a "
                  f"sibling instance (same part in a different role) -- "
                  f"keeping the exported pose")
            continue
        c.world[:3, 3] = new_pos
        print(f"      auto-corrected '{top}': moved {np.linalg.norm(move)*1000:.1f} mm "
              f"onto its mate-solved axis (matching sibling instances)")


def _component_states(comps):
    return [ComponentState(
            name=c.name, link_name=c.link_name, part_path=c.part_path,
            is_subassembly=c.is_subassembly,
            world=[float(x) for x in c.world.flatten()],
            fixed=c.fixed, dof=c.dof, mesh_file=c.mesh_file,
            material=c.material, density=c.density,
            sw_mass=c.sw_mass, sw_com=c.sw_com, sw_inertia=c.sw_inertia)
            for c in comps]


def _mate_edges(adjacency):
    edges = []
    for key, rec in adjacency.items():
        a, b = tuple(key)
        ax = rec.get("axis")
        mates = [MateGeo(**g) for g in rec.get("mates", [])] or None
        edges.append(MateEdge(
            a=a, b=b, types=list(rec.get("types", [])),
            axis_point=([float(x) for x in ax[0]] if ax is not None else None),
            axis_dir=([float(x) for x in ax[1]] if ax is not None else None),
            mates=mates))
    return edges


def to_graph_state(comps, adjacency, ground, robot_name, source_assembly,
                   assembly_mesh=None, subassemblies=None, deep_worlds=None,
                   hidden=None, limit_joints=None):
    subs = {}
    for path, (scomps, sadj, sground) in (subassemblies or {}).items():
        subs[path] = SubGraph(components=_component_states(scomps),
                              edges=_mate_edges(sadj),
                              ground=sorted(sground))
    return GraphState(robot_name=robot_name, source_assembly=source_assembly,
                      components=_component_states(comps),
                      edges=_mate_edges(adjacency), ground=sorted(ground),
                      assembly_mesh=assembly_mesh, subassemblies=subs,
                      deep_worlds=deep_worlds or {}, hidden=hidden or [],
                      limit_joints=[LimitJoint(**j) for j in
                                    (limit_joints or [])])


def capture_deep_worlds(doc):
    """(worlds, hidden): full Name2 -> 16-float ROOT-frame transform for
    EVERY component at every depth (GetComponents(False) returns the
    flattened tree), plus the names of components that are HIDDEN.  Records
    the AS-POSED state, so flexible sub-assembly instances keep their own
    internal poses; hidden parts (stowed mechanism states, reference
    bodies) are excluded from the build like SolidWorks excludes them from
    rendering."""
    out, hidden = {}, []
    for c in list(safe_call(doc, "GetComponents", False) or []):
        ct = as_iface(c, "IComponent2")
        name = safe_prop(ct, "Name2")
        if not name:
            continue
        if not safe_prop(ct, "Visible"):
            hidden.append(name)
        td = safe_prop(ct, "Transform2")
        try:
            world = transform_to_matrix(td.ArrayData)
        except Exception:
            continue
        out[name] = [float(x) for x in world.flatten()]
    if hidden:
        print(f"      {len(hidden)} HIDDEN components recorded "
              f"(excluded at build, like the SolidWorks render)")
    return out, hidden


def extract_subgraphs(doc, comps, sw=None, progress=None):
    """{part_path: (comps, adjacency, ground)} for every unique sub-assembly
    appearing in ``comps``, RECURSIVELY (each sub-assembly's own internals in
    its own local frame).  Prefers the in-memory doc the parent resolved;
    falls back to opening a throwaway copy.  Yields the open ModelDoc to the
    optional ``on_doc(path, md, subcomps)`` hook... (kept simple: caller may
    re-open for meshes via the returned part paths).  ``progress(link_name)`` is
    forwarded per child component (see :func:`extract_components`) so the load
    indicator shows which part is being read."""
    live = {}
    for c in list(safe_call(doc, "GetComponents", True) or []):
        ct = as_iface(c, "IComponent2")
        live[safe_prop(ct, "Name2")] = ct

    out = {}
    opened_docs = []
    work = [(c.part_path, live.get(c.name))
            for c in comps if c.is_subassembly and c.part_path]
    while work:
        path, ct = work.pop(0)
        if not path or path in out:
            continue
        md = safe_call(ct, "GetModelDoc2") if ct is not None else None
        if md is None and sw is not None:
            try:
                md = sw.open_copy(path)
                opened_docs.append(md)
            except Exception as e:
                print(f"      WARN: cannot open sub-assembly "
                      f"{os.path.basename(path)}: {e!r}")
                continue
        if md is None:
            print(f"      WARN: no document for sub-assembly "
                  f"{os.path.basename(path)}; internals not extracted")
            continue
        subcomps = extract_components(md, progress=progress)
        subadj, subground = build_mate_graph(md, subcomps)
        out[path] = (subcomps, subadj, subground)
        print(f"      sub-assembly {os.path.basename(path)}: "
              f"{len(subcomps)} children, {len(subadj)} internal mate pairs")
        live2 = {}
        for c2 in list(safe_call(md, "GetComponents", True) or []):
            ct2 = as_iface(c2, "IComponent2")
            live2[safe_prop(ct2, "Name2")] = ct2
        for sc in subcomps:
            if sc.is_subassembly and sc.part_path and sc.part_path not in out:
                work.append((sc.part_path, live2.get(sc.name)))
    if sw is not None:
        for md in opened_docs:
            sw.close_doc(md)
    return out


def _edge_rec(e):
    """MateEdge -> internal adjacency record."""
    ax = None
    if e.axis_point and e.axis_dir:
        ax = (np.asarray(e.axis_point, float), np.asarray(e.axis_dir, float))
    return {"types": list(e.types), "axis": ax,
            "mates": [g.model_dump() for g in e.mates] if e.mates else []}


def _excluded(name, link_name, exclude):
    """True if an ``exclude`` entry matches a component's SolidWorks ``name`` OR
    its (sanitised) URDF ``link_name``.  The editor's Delete may send either form
    -- the raw component name (``vial_phi30_all.SLDPRT-2/vial_phi30-1``) or the
    link name (``vial_phi30_all_SLDPRT_2__vial_phi30_1``) -- and the two differ
    by '.'/'/'/'-' vs '_', so matching only the component name silently missed
    links whose Delete fell back to the link name."""
    n = (name or "").lower()
    ln = (link_name or "").lower()
    return any(e in n or (ln and e in ln) for e in exclude)


def from_graph(graph, exclude=None, expand=None, no_expand=None):
    """GraphState -> (comps, adjacency, ground), applying ``exclude`` and
    expanding sub-assemblies whose internals move (see
    :func:`_expand_subassemblies`)."""
    exclude = [e.lower() for e in (exclude or [])]
    hidden = set(getattr(graph, "hidden", None) or [])
    if hidden:
        print(f"      excluding {len(hidden)} hidden components")
    comps = []
    for cs in graph.components:
        if _excluded(cs.name, cs.link_name, exclude):
            continue
        if cs.name in hidden:
            continue
        comps.append(Component(
            name=cs.name, link_name=cs.link_name, part_path=cs.part_path,
            is_subassembly=cs.is_subassembly, world=cs.world_matrix(),
            fixed=cs.fixed, dof=cs.dof, mesh_file=cs.mesh_file,
            material=cs.material, density=cs.density,
            sw_mass=cs.sw_mass, sw_com=cs.sw_com, sw_inertia=cs.sw_inertia,
            mass_only=getattr(cs, "mass_only", False)))
    names = {c.name for c in comps}
    adjacency = {}
    for e in graph.edges:
        if e.a not in names or e.b not in names:
            continue
        adjacency[frozenset((e.a, e.b))] = _edge_rec(e)
    ground = {g for g in graph.ground if g in names}
    _snap_unsolved_mates(comps, adjacency)
    comps, adjacency, ground = _expand_subassemblies(
        graph, comps, adjacency, ground, expand=expand, no_expand=no_expand)
    if exclude:
        # Apply `exclude` AGAIN after expansion: the filter at the top of this
        # function only sees top-level graph.components, so a part excluded from
        # INSIDE an expanded sub-assembly (e.g. the editor's Delete on a
        # palm_1__* child) would otherwise be spliced back in here.  Prune the
        # matching components and any adjacency edge / ground entry that
        # referenced them.
        keep = {c.name for c in comps
                if not _excluded(c.name, c.link_name, exclude)}
        comps = [c for c in comps if c.name in keep]
        adjacency = {k: v for k, v in adjacency.items()
                     if all(n in keep for n in k)}
        ground = {g for g in ground if g in keep}
    return comps, adjacency, ground


# --------------------------------------------------------------------
# Build-time sub-assembly expansion.
#
# A sub-assembly is one rigid link by default.  But some hide real joints
# (a servo unit whose horn turns, a gripper with sliding fingers): for
# those the extracted internals (GraphState.subassemblies) are spliced
# into the parent -- children become components (instance transform
# composed), internal mates become edges, and the top-level mates that
# touched the instance are re-attached to the child that actually owns
# the mated face.
# --------------------------------------------------------------------

def _transform_rec(rec, T):
    """Adjacency record with all geometry mapped through 4x4 ``T``."""
    R = T[:3, :3]
    out = {"types": list(rec["types"]), "axis": None, "mates": []}
    if rec.get("axis") is not None:
        p, d = rec["axis"]
        out["axis"] = (R @ np.asarray(p, float) + T[:3, 3],
                       R @ np.asarray(d, float))
    for g in rec.get("mates", []):
        g2 = dict(g)
        g2["points"] = [list(R @ np.asarray(p, float) + T[:3, 3])
                        for p in g.get("points", [])]
        g2["dirs"] = [list(R @ np.asarray(d, float))
                      for d in g.get("dirs", [])]
        out["mates"].append(g2)
    return out


def _subgraph_is_movable(sub, subs, _seen=None):
    """Does any internal mate edge -- at ANY nesting depth -- classify as a
    movable joint?  Internal edges are judged in strict mode (sub-assembly
    fastener heuristics).  Recurses so a rigid wrapper around a moving unit
    still expands."""
    _seen = _seen if _seen is not None else set()
    for e in sub.edges:
        rec = _edge_rec(e)
        rec["strict"] = True
        if classify_edge_auto(rec)[0] in _MOVABLE_TYPES:
            return True
    for cs in sub.components:
        p = cs.part_path
        if cs.is_subassembly and p and p in subs and p not in _seen:
            _seen.add(p)
            if _subgraph_is_movable(subs[p], subs, _seen):
                return True
    return False


def _expand_one(inst, sub, comps, adjacency, ground, deep=None, hidden=None):
    T = inst.world
    deep = deep or {}
    hidden = hidden or set()
    print(f"      expanding sub-assembly '{inst.name}' "
          f"({len(sub.components)} children)")
    used_links = {c.link_name for c in comps}
    children, name_map, local_of, world_of = [], {}, {}, {}
    flexible = 0
    for cs in sub.components:
        gname = f"{inst.name}/{cs.name}"
        if gname in hidden:
            continue
        ln = safe_name(f"{inst.link_name}__{cs.link_name}")
        base, i = ln, 1
        while ln in used_links:
            i += 1
            ln = f"{base}_{i}"
        used_links.add(ln)
        local = cs.world_matrix()
        composed = T @ local
        # a FLEXIBLE sub-assembly poses its internals differently per
        # instance: prefer the actual as-posed world captured from the root
        if gname in deep:
            actual = np.array(deep[gname], float).reshape(4, 4)
            if not np.allclose(actual, composed, atol=1e-6):
                flexible += 1
            world = actual
        else:
            world = composed
        children.append(Component(
            name=gname, link_name=ln, part_path=cs.part_path,
            is_subassembly=cs.is_subassembly, world=world,
            fixed=False, dof=None, mesh_file=cs.mesh_file,
            material=cs.material, density=cs.density,
            sw_mass=cs.sw_mass, sw_com=cs.sw_com, sw_inertia=cs.sw_inertia))
        name_map[cs.name] = gname
        local_of[cs.name] = local
        world_of[cs.name] = world
    if flexible:
        print(f"        ({flexible} children re-posed from the live "
              f"instance state -- flexible sub-assembly)")
    child_names = set(name_map.values())

    # internal edges -> world frame, instance-global names; classified in
    # strict mode (fastener-radius heuristics) at build time.  The mate
    # geometry is stored in the sub-assembly's SAVED frame; for a flexible
    # instance the joint moved with its child, so map sub-frame -> world
    # through one owning child:  M = world_actual(child) @ inv(local(child))
    for e in sub.edges:
        if e.a not in name_map or e.b not in name_map:
            continue
        M = T
        for owner in (e.a, e.b):
            try:
                M = world_of[owner] @ np.linalg.inv(local_of[owner])
                break
            except Exception:
                continue
        rec = _transform_rec(_edge_rec(e), M)
        rec["strict"] = True
        for g in rec["mates"]:
            g["owners"] = [f"{inst.name}/{o}" if o else ""
                           for o in g.get("owners", [])]
        adjacency[frozenset((name_map[e.a], name_map[e.b]))] = rec

    # children mated to the sub-assembly ORIGIN are rigid w.r.t. each other.
    # The synthetic LOCK needs a mate RECORD (not just a type string) so the
    # global twist solve sees its rows -- otherwise grounded electronics
    # (RasPi PCB + headers) stay "free" globally despite being grounded.
    grounded = [name_map[g] for g in sub.ground if g in name_map]
    for i in range(1, len(grounded)):
        key = frozenset((grounded[0], grounded[i]))
        adjacency.setdefault(key, {
            "types": ["LOCK"], "axis": None,
            "mates": [{"type": "LOCK", "etypes": [], "points": [],
                       "dirs": [], "radii": [], "owners": []}]})

    def child_of_owner(owner):
        # inst.name itself may contain '/' once expansion nests, so match
        # by prefix, not by first path segment
        pre = inst.name + "/"
        if owner and owner.startswith(pre):
            tail = owner[len(pre):].split("/")[0]
            cand = pre + tail
            if cand in child_names:
                return cand
        return None

    def nearest_child(point):
        return min(children, key=lambda c: float(
            np.sum((c.world[:3, 3] - np.asarray(point, float)) ** 2))).name

    # top-level edges that touched the collapsed instance -> per-child edges
    for key in [k for k in list(adjacency) if inst.name in k]:
        rec = adjacency.pop(key)
        others = [x for x in key if x != inst.name]
        if not others:
            continue
        other = others[0]
        groups, leftovers = {}, []
        for g in rec.get("mates", []):
            target = None
            for o in g.get("owners", []):
                target = child_of_owner(o)
                if target:
                    break
            (groups.setdefault(target, []) if target else leftovers) \
                .append(g)
        if leftovers:
            if groups:
                tgt = max(groups, key=lambda t: len(groups[t]))
            else:
                pts = [p for g in leftovers for p in g.get("points", [])]
                tgt = nearest_child(np.mean(np.asarray(pts, float), axis=0)
                                    if pts else inst.world[:3, 3])
            groups.setdefault(tgt, []).extend(leftovers)
        for tgt, gs in groups.items():
            rec2 = adjacency.setdefault(
                frozenset((tgt, other)),
                {"types": [], "axis": None, "mates": []})
            for g in gs:
                rec2["types"].append(g["type"])
                rec2["mates"].append(g)
                if rec2["axis"] is None and g["type"] == "CONCENTRIC" \
                        and g.get("dirs"):
                    d = np.asarray(g["dirs"][0], float)
                    if np.linalg.norm(d) > 1e-9:
                        rec2["axis"] = (np.asarray(g["points"][0], float), d)

    comps = [c for c in comps if c.name != inst.name] + children
    if inst.name in ground:
        ground.discard(inst.name)
        ground.add(nearest_child(inst.world[:3, 3]))
    return comps, adjacency, ground


def _expand_subassemblies(graph, comps, adjacency, ground,
                          expand=None, no_expand=None):
    """Expand instances whose internals move; ``expand``/``no_expand`` are
    case-insensitive substring overrides from the joint config."""
    subs = getattr(graph, "subassemblies", None) or {}
    if not subs:
        return comps, adjacency, ground
    expand = [s.lower() for s in (expand or [])]
    no_expand = [s.lower() for s in (no_expand or [])]
    movable = {}

    def want(inst):
        nm = inst.name.lower()
        if any(s in nm for s in no_expand):
            return False
        sub = subs.get(inst.part_path)
        if sub is None:
            return False
        if any(s in nm for s in expand):
            return True
        if inst.part_path not in movable:
            movable[inst.part_path] = _subgraph_is_movable(sub, subs)
        return movable[inst.part_path]

    deep = getattr(graph, "deep_worlds", None) or {}
    hidden = set(getattr(graph, "hidden", None) or [])
    while True:
        inst = next((c for c in comps if c.is_subassembly and want(c)), None)
        if inst is None:
            return comps, adjacency, ground
        comps, adjacency, ground = _expand_one(
            inst, subs[inst.part_path], comps, adjacency, ground,
            deep=deep, hidden=hidden)


# ====================================================================
# Build (no SolidWorks): GraphState + config -> RobotModel
# ====================================================================

def build_model(graph, robot_name=None, base_hint=None, config=None,
                exclude=None):
    robot_name = robot_name or graph.robot_name
    exclude = list(exclude or [])
    if config and config.get("exclude"):
        exclude += list(config["exclude"])
    comps, adjacency, ground = from_graph(
        graph, exclude=exclude,
        expand=config.get("expand") if config else None,
        no_expand=config.get("no_expand") if config else None)

    # SolidWorks LimitDistance/LimitAngle mates ARE the assembly's real DOFs --
    # promote those edges to prismatic/revolute with the CAD axis + travel
    # limits, overriding the (over-constrained) geometric classification.  On by
    # default; `use_limit_joints: false` falls back to pure geometry.
    if config is None or config.get("use_limit_joints", True):
        comp_names = {c.name for c in comps}
        n_lim = 0
        for lj in getattr(graph, "limit_joints", None) or []:
            if lj.a not in comp_names or lj.b not in comp_names:
                continue
            rec = adjacency.setdefault(frozenset((lj.a, lj.b)),
                                       {"types": [], "axis": None, "mates": []})
            ax_pt = np.asarray(lj.axis_point, float)
            ax_dir = np.asarray(lj.axis_dir, float)
            if lj.type == "revolute":
                # a LimitAngle mate fixes the travel but its plane point is NOT
                # on the rotation axis -- the hinge is the concentric/cylinder
                # mate on the same pair.  Pivoting about the angle-plane point
                # swings the part around an offset centre (the screen hinge was
                # ~5 cm off); use the concentric axis line so it rotates in
                # place.  Keep the limit's direction SIGN (lower/upper + ref are
                # relative to it).
                cax = _concentric_axis_of(rec)
                if cax is not None:
                    cp, cd = cax
                    n = np.linalg.norm(ax_dir) or 1.0
                    if abs(float(cd @ (ax_dir / n))) > 0.99:
                        ax_pt = cp
                        ax_dir = cd if float(cd @ ax_dir) >= 0 else -cd
            rec["limit_joint"] = {
                "type": lj.type, "ref": lj.a,
                "axis": (ax_pt, ax_dir),
                "lower": float(lj.lower), "upper": float(lj.upper)}
            n_lim += 1
        if n_lim:
            print(f"      limit-mate joints: {n_lim} (SolidWorks sliders/hinges)")

    # Standard hardware (screws/nuts/washers/pins) welds RIGIDLY to whatever it
    # fastens: tag it so its concentric-into-tapped-hole mate is never read as a
    # hinge.  On by default; `weld_fasteners: false` keeps the old behaviour,
    # `fastener:`/`not_fastener:` tune the match.
    weld = True if config is None else config.get("weld_fasteners", True)
    if weld:
        extra = config.get("fastener") if config else None
        keep = config.get("not_fastener") if config else None
        fastener_names = set()
        for c in comps:
            if is_fastener_part(c.name, c.part_path, extra=extra, keep=keep):
                c.is_fastener = True
                fastener_names.add(c.name)
        if fastener_names:
            for key, rec in adjacency.items():
                if any(n in fastener_names for n in key):
                    rec["fastener"] = True
            print(f"      fasteners welded fixed: {len(fastener_names)} "
                  f"parts (screws/nuts/washers/pins)")

    if config and config.get("force_fixed"):
        # weld these edges and REBUILD the auto tree around them (editing a
        # type in the joints list keeps the old tree shape; this re-routes)
        alias = {}
        for c in comps:
            alias[c.name] = c.name
            alias[c.link_name] = c.name
        for pair in config["force_fixed"]:
            a, b = alias.get(str(pair[0])), alias.get(str(pair[1]))
            key = frozenset((a, b)) if a and b else None
            if key in adjacency:
                adjacency[key]["force_fixed"] = True
                print(f"      force_fixed: {pair[0]} -- {pair[1]}")
            else:
                print(f"      WARN: force_fixed edge not found: {pair}")

    if config and config.get("densities"):
        # per-link density overrides (kg/m^3) -- the web editor's material
        # setting; wins over the part's SolidWorks material
        by_ln = {c.link_name: c for c in comps}
        by_nm = {c.name: c for c in comps}
        for k, v in config["densities"].items():
            c = by_ln.get(str(k)) or by_nm.get(str(k))
            if c is not None:
                c.density = float(v)
                # explicit density => drive mass from the mesh, not the
                # SolidWorks-native value computed with the CAD material
                c.density_override = True
            else:
                print(f"      WARN: densities: '{k}' matched no link")

    if config and config.get("mass_only"):
        # mass-only links: keep the weight, drop the geometry.  Same name
        # matching as densities (link name or SolidWorks name).  The only-fixed
        # check happens below, once the tree (and so each part's joint) is known.
        by_ln = {c.link_name: c for c in comps}
        by_nm = {c.name: c for c in comps}
        for k in config["mass_only"]:
            c = by_ln.get(str(k)) or by_nm.get(str(k))
            if c is not None:
                c.mass_only = True
            else:
                print(f"      WARN: mass_only: '{k}' matched no link")

    directed = None
    root_rpy = None
    ports = []
    root_link_name = "base_link"
    if config:
        base_hint = base_hint or config.get("base")
        directed = resolve_directed(comps, config.get("joints"))
        root_rpy = config.get("root_rpy")
        ports = resolve_ports(comps, config.get("ports"))
        if "root_link_name" in config:
            # falsy -> keep the component's own link name (no rename)
            root_link_name = config.get("root_link_name") or ""
    root_z_offset = (config.get("root_z_offset", 0.0) if config else 0.0)
    root_xyz = (config.get("root_xyz") if config else None)

    base = choose_base(comps, ground, base_hint, adjacency)
    print(f"      base link: {base.link_name}")
    closures_out = []
    joints = build_tree(comps, adjacency, base, directed=directed,
                        root_rpy=root_rpy, root_z_offset=root_z_offset,
                        root_xyz=root_xyz, closures_out=closures_out)
    nrev = sum(1 for j in joints if j.jtype == "revolute")
    npri = sum(1 for j in joints if j.jtype == "prismatic")
    print(f"      joint types: {nrev} revolute, {npri} prismatic, "
          f"{len(joints) - nrev - npri} fixed")

    # mass-only is valid only on a FIXED child (its inertial lumps into the fixed
    # parent on export); a movable or root link must keep its geometry, so clear
    # the flag there and say why rather than emit an invisible movable link.
    if any(c.mass_only for c in comps):
        parent_jtype = {j.child: j.jtype for j in joints}
        for c in comps:
            if c.mass_only and parent_jtype.get(c.link_name) != "fixed":
                why = ("it is the base link" if c.link_name == base.link_name
                       else f"its joint is '{parent_jtype.get(c.link_name)}', "
                            f"not fixed")
                print(f"      WARN: mass_only ignored for '{c.link_name}': {why}")
                c.mass_only = False

    name2link = {c.name: c.link_name for c in comps}
    detected = []
    for key, rec in adjacency.items():
        a, b = tuple(key)
        if a in name2link and b in name2link:
            jt, ax = classify_edge(rec.get("types", []), rec.get("axis"))
            detected.append({"between": [name2link[a], name2link[b]],
                             "mates": rec.get("types", []), "suggested": jt})
    # carry the meshes so the URDF writer can reference them
    for c in comps:
        if c.mesh_file is None:
            for cs in graph.components:
                if cs.name == c.name:
                    c.mesh_file = cs.mesh_file
                    break
    n_ports = ", ".join(p.name for p in ports)
    if ports:
        print(f"      output ports: {n_ports}")
    return RobotModel(name=robot_name, components=comps, joints=joints,
                      detected_edges=detected, base_link=base.link_name,
                      ports=ports,
                      root_link_name=root_link_name or base.link_name,
                      loop_closures=closures_out[0] if closures_out else None)
