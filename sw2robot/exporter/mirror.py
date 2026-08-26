"""Mass properties a SolidWorks MIRROR COPY silently loses, recovered from its
source part.

SolidWorks' *Insert > Mirror Part* (and the opposite-hand option of *Mirror
Components*) writes a NEW ``.SLDPRT`` that carries the reflected geometry and
nothing else: per-body material assignments and the whole-part *Override Mass
Properties* dialog do NOT come across.  The copy then falls back to SolidWorks'
default 1000 kg/m3 and reports a mass that is wrong by whatever the source's
real settings were -- while still looking like an exact CAD value all the way
into the URDF.

Measured on one humanoid whose whole mirrored side was heavier than the side it
was copied from -- most of it on a single 3D-printed shell that came out **156x**
its real weight, because that weight had come from per-body densities the copy
never inherited.

Three distinct loss modes showed up there, and only the last is the one people
expect:

* the source has NO part-level material and gets its weight from per-body
  densities (the zero-density envelope-body idiom) -- the copy has no material
  either, so a "copy the material name" fix would change nothing;
* the source carries an ``Override Mass Properties`` value (a catalogue weight)
  -- the copy has the SAME material assigned and is still several times too
  heavy;
* both sides name the same material, but the source assigns different materials
  per BODY, so only its resolved density differs.

What is deliberately NOT touched: a copy whose part-level material differs from
its source's.  That is a choice someone made on the copy (possibly a stale one),
not a setting that fell off, so it is reported and left alone.  Neither is a
source whose own weight is itself an unreviewed 1000 kg/m3 default -- copying
that across would leave both parts looking resolved when neither was reviewed.

The reflection itself is solved from geometry, never assumed: the unit-density
mass properties of both documents must agree under one axis-aligned reflection,
which also proves the two parts really are mirror twins.  When they do not
agree, nothing is inherited and the reason is printed -- a wrong mass that is
visibly wrong beats a plausible one that is quietly invented.
"""

from __future__ import annotations

import os

import numpy as np

from .swcom import (
    as_iface,
    doc_type_for,
    open_doc6,
    safe_call,
    safe_prop,
)

# swFeatureNameID_e / GetTypeName2 for "Insert > Mirror Part".  The feature's
# NAME is the localised "<source part>_ミラー コピー", so the type name is the
# only language-independent marker.
MIRROR_FEATURE_TYPE = "MirrorStock"

# SolidWorks computes a material-less part at this density (kg/m^3), so a mass
# derived from it is a guess wearing an exact value's clothes.
DEFAULT_SW_DENSITY = 1000.0

# Relative agreement required of the two documents' unit-density mass
# properties before they are accepted as mirror twins.  Mirroring is exact
# geometry, so the residual is only the rebuild's floating point: measured
# across the five real pairs in tests/fixtures/mirror_body_props.json it runs
# 5e-7 to 4.4e-5, while the nearest WRONG reflection of the same part sits at
# 8.2e-2 to 8.6e-1.  1e-3 is 23x above the worst true residual and 82x below
# the closest false one -- the gap is three and a half orders wide, so the
# exact value inside it does not matter.
REFLECTION_RTOL = 1e-3

# Axis-aligned improper transforms: one reflection per coordinate plane, plus
# the point inversion (also det -1) some mirror-then-rotate histories produce.
_CANDIDATE_SIGNS = ((-1, 1, 1), (1, -1, 1), (1, 1, -1), (-1, -1, -1))


# ---------------------------------------------------------------- tensor maths

def _inertia_matrix(inertia6):
    """(ixx,ixy,ixz,iyy,iyz,izz) -> the symmetric 3x3 tensor."""
    ixx, ixy, ixz, iyy, iyz, izz = (float(x) for x in inertia6)
    return np.array([[ixx, ixy, ixz],
                     [ixy, iyy, iyz],
                     [ixz, iyz, izz]], float)


def _inertia6(matrix):
    """The symmetric 3x3 tensor -> (ixx,ixy,ixz,iyy,iyz,izz)."""
    m = np.asarray(matrix, float)
    return (float(m[0, 0]), float(m[0, 1]), float(m[0, 2]),
            float(m[1, 1]), float(m[1, 2]), float(m[2, 2]))


def _about_origin(mass, com, inertia_com):
    """Parallel-axis shift of an inertia tensor from the COM to the origin."""
    c = np.asarray(com, float)
    return inertia_com + mass * (float(c @ c) * np.eye(3) - np.outer(c, c))


def _about_com(mass, com, inertia_origin):
    """The inverse of :func:`_about_origin`."""
    c = np.asarray(com, float)
    return inertia_origin - mass * (float(c @ c) * np.eye(3) - np.outer(c, c))


def reflect_inertial(mass, com, inertia6, signs):
    """Mirror one set of mass properties through an axis-aligned plane.

    A reflection is an isometry, so the mass is unchanged, the centre of mass
    reflects, and the inertia tensor -- a rank-2 tensor -- transforms as
    ``S I S`` (``S`` is its own transpose and inverse).  Only the two products
    of inertia that contain the flipped axis change sign.

    Parameters
    ----------
    mass : float
        Mass in kg.
    com : sequence of 3 floats
        Centre of mass in the source document's frame (m).
    inertia6 : sequence of 6 floats
        ``(ixx,ixy,ixz,iyy,iyz,izz)`` about the centre of mass.
    signs : sequence of 3 ints
        Per-axis +1/-1; an odd number of -1 makes it a reflection.

    Returns
    -------
    tuple
        ``(mass, [x,y,z], (ixx,ixy,ixz,iyy,iyz,izz))`` in the mirror
        document's frame.
    """
    s = np.asarray(signs, float)
    S = np.diag(s)
    com_m = (s * np.asarray(com, float)).tolist()
    return (float(mass), [float(x) for x in com_m],
            _inertia6(S @ _inertia_matrix(inertia6) @ S))


# --------------------------------------------------- geometry, without density

def unit_density_props(part_doc):
    """Density-free mass properties of a PART document, from its solid bodies.

    ``IBody2.GetMassProperties(density)`` takes the density as an ARGUMENT, so
    calling it with 1.0 gives pure geometry.  That is the whole point here: the
    two documents being compared disagree about density by construction, so
    neither one's reported centre of mass can be used to match them.

    Bodies are read including hidden ones -- the envelope body that carries most
    of a 3D-printed part's volume is routinely hidden, and leaving it out would
    make two genuine twins look unrelated.

    Returns
    -------
    tuple | None
        ``(volume, [x,y,z] centroid, (ixx,ixy,ixz,iyy,iyz,izz) about the
        centroid)`` at unit density, or None when the document exposes no solid
        body.
    """
    pd = as_iface(part_doc, "IPartDoc")
    try:
        bodies = list(safe_call(pd, "GetBodies2", 0, False) or [])
    except Exception:
        return None
    arrays = []
    for body in bodies:
        g = safe_call(as_iface(body, "IBody2"), "GetMassProperties", 1.0)
        if g:
            arrays.append([float(x) for x in g])
    return props_from_body_arrays(arrays)


def props_from_body_arrays(arrays):
    """:func:`unit_density_props` without SolidWorks: the maths on raw
    ``GetMassProperties`` arrays.

    Each array is 12 bare doubles --
    ``[comx, comy, comz, volume, area, mass, ixx, iyy, izz, ixy, izx, iyz]``,
    the moments taken about that BODY's own centre of mass, and note ``izx``
    before ``iyz``.  Nothing in the values says which slot is which, so this is
    split out to be checked against arrays captured from real documents (see
    ``tests/fixtures/mirror_body_props.json``).
    """
    total_v = 0.0
    first = np.zeros(3)                   # sum of volume * centroid
    tensor = np.zeros((3, 3))             # about the document origin
    for g in arrays:
        if not g or len(g) < 12:
            continue
        com = np.asarray(g[0:3], float)
        vol = float(g[3])
        if not np.isfinite(vol) or vol <= 0:
            continue
        ixx, iyy, izz, ixy, izx, iyz = g[6:12]
        i_com = np.array([[ixx, ixy, izx],
                          [ixy, iyy, iyz],
                          [izx, iyz, izz]], float)
        total_v += vol
        first += vol * com
        tensor += _about_origin(vol, com, i_com)
    if total_v <= 0:
        return None
    centroid = first / total_v
    return (float(total_v), [float(x) for x in centroid],
            _inertia6(_about_com(total_v, centroid, tensor)))


def solve_reflections(source_props, mirror_props, rtol=REFLECTION_RTOL):
    """Every axis-aligned reflection that maps ``source_props`` onto
    ``mirror_props``.

    Both arguments are :func:`unit_density_props` results, i.e. pure geometry.
    A candidate is accepted only when the volume, the reflected centroid AND
    the reflected inertia tensor all agree; the tensor is what names the axis
    for a part whose centroid happens to sit on the mirror plane.  Tolerances
    are relative to each quantity's own scale, so they hold for a 0.7 L bracket
    and a 38 L shell alike.

    More than one candidate can fit -- a part symmetric about two planes has
    nothing in its SHAPE that says which one was mirrored.  They are all
    returned rather than picked between here, because whether the ambiguity
    matters depends on the mass distribution the caller is about to reflect
    (see :func:`reflect_consistently`), not on the geometry.

    Returns
    -------
    tuple
        The accepted ``(sx, sy, sz)`` triples; empty when no reflection
        explains the pair, i.e. they are not mirror twins or the copy was
        edited after it was made.
    """
    if source_props is None or mirror_props is None:
        return ()
    vol_s, com_s, i_s = source_props
    vol_m, com_m, i_m = mirror_props
    if vol_s <= 0 or vol_m <= 0:
        return ()
    if abs(vol_m - vol_s) > rtol * max(vol_s, vol_m):
        return ()
    # length scale of the part, so the centroid tolerance means the same thing
    # for a wrist screw and a foot shell
    scale = max(float(np.linalg.norm(com_s)), vol_s ** (1.0 / 3.0))
    i_scale = max(abs(x) for x in i_s) or 1.0
    accepted = []
    for signs in _CANDIDATE_SIGNS:
        _m, com_r, i_r = reflect_inertial(vol_s, com_s, i_s, signs)
        if np.max(np.abs(np.asarray(com_r) - np.asarray(com_m))) > rtol * scale:
            continue
        if np.max(np.abs(np.asarray(i_r) - np.asarray(i_m))) > rtol * i_scale:
            continue
        accepted.append(signs)
    return tuple(accepted)


def reflect_consistently(candidates, mass, com, inertia6, rtol=REFLECTION_RTOL):
    """Reflect one inertial, but only when the candidate planes agree on it.

    A shape that fits several reflections is still unambiguous whenever its
    MASS is distributed symmetrically enough that each candidate lands on the
    same answer -- often the case, because the symmetry that made the shape
    ambiguous usually made the mass distribution ambiguous too.  When the
    candidates disagree, the geometry genuinely does not say which plane was
    used, and inventing one would put the products of inertia on the wrong side.

    Returns ``(mass, com, inertia6)`` or None.
    """
    if not candidates:
        return None
    results = [reflect_inertial(mass, com, inertia6, s) for s in candidates]
    first = results[0]
    scale = max(float(np.linalg.norm(com)), 1e-9)
    i_scale = max(abs(x) for x in inertia6) or 1.0
    for other in results[1:]:
        d_com = np.max(np.abs(np.asarray(other[1]) - np.asarray(first[1])))
        d_i = np.max(np.abs(np.asarray(other[2]) - np.asarray(first[2])))
        if d_com > rtol * scale or d_i > rtol * i_scale:
            return None
    return first


# -------------------------------------------------------- is it a mirror copy?

def mirror_source_path(app, part_path):
    """Path of the part ``part_path`` was mirrored from, or None.

    ``ISldWorks.GetDocumentDependencies2`` reads the reference out of the file
    on DISK -- no document has to be open -- and returns a flat
    ``(name, path, name, path, ...)`` tuple.  A mirror copy lists exactly two
    things: the source part, and the assembly the mirror was created in.  The
    assembly is dropped by extension, leaving the source.

    Returns None for a part with no external reference at all (the common
    case), which is why this is cheap enough to call on every part.
    """
    if not part_path:
        return None
    try:
        dep = app.GetDocumentDependencies2(part_path, False, False, False)
    except Exception:
        return None
    if not dep:
        return None
    for path in list(dep)[1::2]:
        path = str(path)
        if path.lower().endswith(".sldprt"):
            return path
    return None


def has_mirror_feature(part_doc):
    """Whether a PART document was built by *Insert > Mirror Part*.

    Confirms what :func:`mirror_source_path` inferred from the file's external
    reference: a derived part could also come from *Insert Part* or a split,
    and those are NOT reflections, so inheriting through one would flip an
    inertia tensor that was never mirrored.
    """
    # imported here rather than at module scope: model pulls in mesh, which
    # imports model back, and export loads this module before either
    from .model import _document_features

    try:
        for feature in _document_features(part_doc):
            if safe_call(feature, "GetTypeName2") == MIRROR_FEATURE_TYPE:
                return True
    except Exception:
        return False
    return False


# --------------------------------------------- was the weight lost, or chosen?

def _is_default_mass(material, density):
    """Whether a COPY's weight looks like SolidWorks' unreviewed default.

    Material-less counts, because that is exactly what a mirror copy is left
    as.  The source side needs the opposite question and gets
    :func:`_source_is_deliberate`.
    """
    if not material:
        return True
    return (density is not None
            and abs(density - DEFAULT_SW_DENSITY) < 1.0)


def _source_is_deliberate(source):
    """Whether the SOURCE's weight is a value someone chose.

    Material-less does NOT settle this: the part that started all of this -- a
    3D-printed shell -- has no part-level material and still weighs a deliberate
    value, because its bodies carry their own densities.  What settles it is the
    resolved density: single digits of kg/m^3 is a decision, 1000.0 is
    SolidWorks not having been told.

    Without this check, copying an unreviewed default from one part onto
    another would leave BOTH looking resolved in the editor while neither had
    ever been reviewed.
    """
    if source.get("overridden"):
        return True
    density = source.get("density")
    if density is None:
        return False
    return abs(density - DEFAULT_SW_DENSITY) >= 1.0


def inheritance_reason(mirror, source):
    """Why (or whether) a mirror copy should take its source's mass properties.

    ``mirror`` and ``source`` are ``{"material", "density", "mass",
    "overridden"}`` dicts.  Returns a short human-readable reason, or None to
    leave the copy alone.  The three accepted shapes are the three loss modes in
    the module docstring; anything else -- most importantly a copy carrying a
    DIFFERENT material from its source -- is a deliberate value on the copy and
    is never overwritten.
    """
    if source.get("mass") is None:
        return None
    if mirror.get("overridden"):
        return None                     # the copy has its own deliberate value
    if not _source_is_deliberate(source):
        return None                     # nothing on the source worth taking
    if _is_default_mass(mirror.get("material"), mirror.get("density")):
        return "no material on the copy (SolidWorks' 1000 kg/m^3 default)"
    if source.get("overridden"):
        return "the source carries an Override Mass Properties value"
    same_material = (mirror.get("material") or "") == (source.get("material") or "")
    d_m, d_s = mirror.get("density"), source.get("density")
    if same_material and d_m and d_s and abs(d_m - d_s) > 1e-3 * d_s:
        return (f"same material ({source.get('material')}) but the source "
                f"resolves to {d_s:.1f} kg/m^3, the copy to {d_m:.1f} "
                f"(per-body materials the copy did not inherit)")
    return None


def _read_props(doc):
    """``{"material","density","mass","com","inertia","overridden"}`` of a part."""
    from .model import _read_part_props  # see has_mirror_feature

    material, density, sw = _read_part_props(doc, is_assembly=False)
    sw = sw or {}
    return {"material": material, "density": density,
            "mass": sw.get("mass"), "com": sw.get("com"),
            "inertia": sw.get("inertia"),
            "overridden": bool(sw.get("overridden"))}


def _plan_for_part(app, path, doc_of_path):
    """Everything needed to fix one mirrored part file, or None.

    ``doc_of_path(path)`` returns an open ``IModelDoc2`` for a part path (or
    None).  The plan carries the copy's WRONG properties as well as the
    corrected ones, because a sub-assembly holding a mirror copy is repaired by
    swapping one for the other inside its total.

    A ``"skip"`` key instead of a correction means the pair WAS a mirror but
    something did not check out; the caller reports it rather than dropping it.
    """
    source_path = mirror_source_path(app, path)
    if not source_path:
        return None
    mirror_doc = doc_of_path(path)
    if mirror_doc is None or not has_mirror_feature(mirror_doc):
        return None
    source_doc = doc_of_path(source_path)
    if source_doc is None:
        return {"source": source_path,
                "skip": f"its source {os.path.basename(source_path)} "
                        f"could not be opened"}
    mirror = _read_props(mirror_doc)
    source = _read_props(source_doc)
    reason = inheritance_reason(mirror, source)
    if reason is None:
        # Worth a line only when the copy really does name a DIFFERENT material
        # from its source -- that is a divergence someone should look at.  Two
        # sides agreeing on the material still disagree in the last decimal
        # (rebuilt geometry), and on a 5 g bolt that clears any relative
        # threshold -- twenty identical "not inherited" lines for the twenty
        # instances of one screw is noise, not a finding.
        same_material = ((mirror.get("material") or "")
                         == (source.get("material") or ""))
        if (not same_material and mirror.get("mass") and source.get("mass")
                and abs(mirror["mass"] - source["mass"])
                > 1e-3 * max(mirror["mass"], source["mass"])):
            return {"source": source_path,
                    "skip": f"the copy sets its own material "
                            f"({mirror.get('material')!r} vs the source's "
                            f"{source.get('material')!r}); mass differs by "
                            f"{mirror['mass'] - source['mass']:+.3f} kg"}
        return None
    if source.get("com") is None or source.get("inertia") is None:
        return {"source": source_path,
                "skip": "the source exposes no usable inertia tensor"}
    signs = solve_reflections(unit_density_props(source_doc),
                              unit_density_props(mirror_doc))
    if not signs:
        return {"source": source_path,
                "skip": "no reflection maps the source's geometry onto the "
                        "copy's (edited after it was mirrored?)"}
    reflected = reflect_consistently(
        signs, source["mass"], source["com"], source["inertia"])
    if reflected is None:
        return {"source": source_path,
                "skip": "the shape fits several mirror planes and they "
                        "disagree on where the source's mass sits"}
    mass, com, inertia = reflected
    return {"source": source_path, "signs": signs, "reason": reason,
            "wrong": mirror, "mass": mass, "com": com, "inertia": inertia}


# ----------------------------------------------------------- assembly totals

def _recompose(total, swaps):
    """An assembly's mass properties with some children's contributions swapped.

    ``total`` is ``(mass, com, inertia6)`` of the assembly in its own frame;
    each swap is ``(transform, wrong, right)`` where the two sides are
    ``(mass, com, inertia6)`` in the CHILD's frame.  Inertia tensors taken about
    one common point add and subtract linearly, so the assembly's own total --
    already correct for every other child -- is repaired by removing what
    SolidWorks counted and adding what it should have.

    This is how a rigid sub-assembly LINK gets fixed at all: correcting the
    child part file does not move the assembly document's own reported mass,
    because SolidWorks summed that from the child's wrong density.

    Returns ``(mass, [x,y,z], (ixx,ixy,ixz,iyy,iyz,izz))``, or None if the swap
    leaves a non-positive mass (which would mean the inputs disagree).
    """
    mass, com, inertia6 = total
    acc_m = float(mass)
    acc_first = acc_m * np.asarray(com, float)
    acc_i = _about_origin(acc_m, com, _inertia_matrix(inertia6))
    for transform, wrong, right in swaps:
        T = np.asarray(transform, float)
        R, t = T[:3, :3], T[:3, 3]
        for sign, part in ((-1.0, wrong), (+1.0, right)):
            m, c, i6 = part
            c_p = R @ np.asarray(c, float) + t
            i_p = R @ _inertia_matrix(i6) @ R.T
            acc_m += sign * m
            acc_first += sign * m * c_p
            acc_i += sign * _about_origin(m, c_p, i_p)
    if acc_m <= 0:
        return None
    centroid = acc_first / acc_m
    return (float(acc_m), [float(x) for x in centroid],
            _inertia6(_about_com(acc_m, centroid, acc_i)))


def _world_matrix(component):
    """A component's transform as a 4x4, from either representation.

    A live extraction holds ``Component`` with a numpy array; a graph read back
    from disk holds ``ComponentState`` with 16 floats.
    """
    world = getattr(component, "world", None)
    if world is None:
        return np.eye(4)
    return np.asarray(world, float).reshape(4, 4)


def _own_total(component):
    """A component's own document mass properties, or None if incomplete."""
    if (component.sw_mass is None or component.sw_com is None
            or component.sw_inertia is None):
        return None
    return (float(component.sw_mass), list(component.sw_com),
            tuple(component.sw_inertia))


# ------------------------------------------------------------------- the pass

def apply_mirror_mass_inheritance(app, comps, subassemblies=None):
    """Give every mirror copy in the extraction the mass properties it lost.

    Runs at EXTRACT time, so the corrected values land in ``graph.json`` and
    every later build sees them with no SolidWorks.  A component is only touched
    when :func:`inheritance_reason` says its weight fell off rather than being
    chosen; a per-link ``masses:`` / ``densities:`` entry still wins, because
    the config is applied later.

    Both shapes are handled, and both are needed:

    * a link that IS a mirrored part;
    * a link that is a rigid sub-assembly CONTAINING one, whose document total
      SolidWorks summed from the wrong child density.

    ``subassemblies`` is :func:`extract_subgraphs`' ``{path: (components, ...)}``
    -- those children are corrected too, because a movable sub-assembly is
    expanded at build time and its children become the links.  Sub-assemblies
    are repaired deepest-first, so a nested one contributes its already-fixed
    total to its parent.

    Returns a list of report records for :func:`print_mirror_report`; also sets
    ``component.mass_inherited_from`` so the editor shows the provenance instead
    of flagging the link as an unreviewed default.
    """
    subassemblies = subassemblies or {}
    sub_by_key = {(k or "").lower(): v for k, v in subassemblies.items()}

    # Documents the session already holds must NOT be closed by this pass:
    # OpenDoc6 on an open document hands back the SAME one, and closing it would
    # pull the assembly's own child out from under the rest of the extract.
    session = set()
    try:
        for md in list(safe_call(app, "GetDocuments") or []):
            session.add((safe_prop(as_iface(md, "IModelDoc2"),
                                   "GetPathName") or "").lower())
    except Exception:
        pass

    docs = {}
    opened = []

    def doc_of_path(path):
        key = (path or "").lower()
        if key not in docs:
            md, _err, _warn = open_doc6(app, path, doc_type_for(path), 1 | 2)
            docs[key] = md
            if md is not None and key not in session:
                opened.append(md)
        return docs[key]

    plans = {}

    def plan_for(path):
        key = (path or "").lower()
        if key not in plans:
            try:
                plans[key] = _plan_for_part(app, path, doc_of_path)
            except Exception as e:
                print(f"      WARN: mirror check of "
                      f"{os.path.basename(path or '?')} failed: {e!r}")
                plans[key] = None
        return plans[key]

    every = list(comps)
    top_level = {id(component) for component in comps}
    for entry in subassemblies.values():
        every.extend(entry[0])

    # each sub-assembly's OWN document total, taken from any instance of it
    own_totals = {}
    for component in every:
        if component.is_subassembly and component.part_path:
            own_totals.setdefault((component.part_path or "").lower(),
                                  _own_total(component))

    fixed_subs = {}
    solving = set()

    def child_swap(child):
        """``(wrong, right)`` for one child, or None when it does not move."""
        if not child.part_path:
            return None
        if child.is_subassembly:
            right = fixed_sub(child.part_path)
            wrong = own_totals.get((child.part_path or "").lower())
            return None if (right is None or wrong is None) else (wrong, right)
        plan = plan_for(child.part_path)
        if not plan or plan.get("skip"):
            return None
        wrong = plan["wrong"]
        if wrong.get("com") is None or wrong.get("inertia") is None:
            return None
        return ((wrong["mass"], wrong["com"], wrong["inertia"]),
                (plan["mass"], plan["com"], plan["inertia"]))

    def fixed_sub(path):
        """The corrected total of one sub-assembly FILE, or None if untouched."""
        key = (path or "").lower()
        if key in fixed_subs:
            return fixed_subs[key]
        if key in solving or key not in sub_by_key:
            return None                      # unreadable internals, or a cycle
        solving.add(key)
        fixed_subs[key] = None
        total = own_totals.get(key)
        if total is not None:
            swaps = [(_world_matrix(child), *swap)
                     for child in sub_by_key[key][0]
                     for swap in [child_swap(child)] if swap is not None]
            if swaps:
                fixed_subs[key] = _recompose(total, swaps)
        solving.discard(key)
        return fixed_subs[key]

    def sources_inside(path):
        """Source part basenames behind a sub-assembly's correction."""
        entry = sub_by_key.get((path or "").lower())
        out = set()
        for child in (entry[0] if entry else []):
            if child.is_subassembly:
                out |= sources_inside(child.part_path)
                continue
            plan = plan_for(child.part_path) if child.part_path else None
            if plan and not plan.get("skip"):
                out.add(os.path.basename(plan["source"]))
        return out

    reports = []
    seen_skips = set()
    try:
        # parts first: repairing a sub-assembly reads its children's plans
        for component in every:
            if component.is_subassembly or not component.part_path:
                continue
            plan = plan_for(component.part_path)
            if not plan:
                continue
            if plan.get("skip"):
                key = (component.link_name, plan["source"])
                if key not in seen_skips:
                    seen_skips.add(key)
                    reports.append({"link": component.link_name,
                                    "source": plan["source"],
                                    "skip": plan["skip"]})
                continue
            before = component.sw_mass
            component.sw_mass = plan["mass"]
            component.sw_com = list(plan["com"])
            component.sw_inertia = list(plan["inertia"])
            component.mass_inherited_from = os.path.basename(plan["source"])
            reports.append({"link": component.link_name,
                            "source": plan["source"], "before": before,
                            "after": plan["mass"], "reason": plan["reason"],
                            "top": id(component) in top_level})
        for component in every:
            if not component.is_subassembly or not component.part_path:
                continue
            fixed = fixed_sub(component.part_path)
            sources = sorted(sources_inside(component.part_path))
            if fixed is None:
                # it DOES hold a corrected copy, but its own total could not be
                # rebuilt (no inertia tensor on the assembly document): say so,
                # because the link is still carrying the wrong weight
                key = (component.link_name, "|".join(sources))
                if sources and key not in seen_skips:
                    seen_skips.add(key)
                    reports.append({
                        "link": component.link_name, "source": sources[0],
                        "skip": "it holds a mirror copy, but the sub-assembly "
                                "document exposes no usable inertia tensor to "
                                "correct its total with"})
                continue
            before = component.sw_mass
            component.sw_mass, component.sw_com = fixed[0], list(fixed[1])
            component.sw_inertia = list(fixed[2])
            component.mass_inherited_from = ", ".join(sources)
            reports.append({
                "link": component.link_name,
                "source": sources[0] if sources else component.part_path,
                "before": before, "after": fixed[0],
                "top": id(component) in top_level,
                "reason": "a mirrored part inside it had lost its source's "
                          "settings, so SolidWorks summed the wrong weight"})
    finally:
        for md in opened:
            try:
                app.CloseDoc(safe_prop(as_iface(md, "IModelDoc2"), "GetTitle"))
            except Exception:
                pass
    return reports


def print_mirror_report(reports):
    """Print what :func:`apply_mirror_mass_inheritance` did, or say nothing.

    The robot-level total counts TOP-LEVEL components only.  One mass error is
    corrected twice on purpose -- once on the mirrored part, once in the total
    of the sub-assembly holding it -- because which of the two becomes a link is
    a build-time decision (a movable sub-assembly is expanded, a rigid one is
    not).  Exactly one of them reaches the URDF, so summing both reports twice
    the weight that actually moves -- which is what the first run of this on a
    real robot did.
    """
    fixed = [r for r in reports if not r.get("skip")]
    for r in [r for r in reports if r.get("skip")]:
        print(f"      mirror copy '{r['link']}': NOT inherited -- {r['skip']}")
    if not fixed:
        return
    delta = sum(r["after"] - (r["before"] or 0.0) for r in fixed if r.get("top"))
    print(f"      mirror copies: took the source part's mass properties for "
          f"{len(fixed)} component(s); the robot loses {-delta:.3f} kg of "
          f"phantom weight")
    for r in fixed:
        before = "?" if r["before"] is None else f"{r['before']:.3f}"
        print(f"        {'' if r.get('top') else '(inside) '}{r['link']}: "
              f"{before} -> {r['after']:.3f} kg "
              f"(from {os.path.basename(r['source'])}; {r['reason']})")


__all__ = [
    "DEFAULT_SW_DENSITY",
    "MIRROR_FEATURE_TYPE",
    "REFLECTION_RTOL",
    "apply_mirror_mass_inheritance",
    "has_mirror_feature",
    "inheritance_reason",
    "mirror_source_path",
    "print_mirror_report",
    "props_from_body_arrays",
    "reflect_consistently",
    "reflect_inertial",
    "solve_reflections",
    "unit_density_props",
]
