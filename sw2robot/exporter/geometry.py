"""Transform helpers: SolidWorks IMathTransform -> 4x4 and URDF xyz/rpy.

SolidWorks ``IMathTransform.ArrayData`` is ``[r0..r8, tx, ty, tz, scale, ...]``.
The documented point transform (local -> global) is the row-vector form::

    gx = x*r0 + y*r3 + z*r6 + tx
    gy = x*r1 + y*r4 + z*r7 + ty
    gz = x*r2 + y*r5 + z*r8 + tz

so the column-vector rotation matrix is ``reshape(r,(3,3)).T``.  Translations
are in METRES (the SW API is metric), which is what URDF wants.

Only helpers with sw2robot-specific behaviour live here (SW marshalling, URDF
text stability, ``<origin>`` parsing and writing).  BOTH directions of the
``<origin>`` round trip are in this module -- ``urdf_origin_matrix`` reads,
``set_urdf_origin`` / ``fmt_urdf_num`` write -- so neither the rpy convention
nor the number formatting can drift between the exporter and the editor.

Plain rotation/transform math is imported straight from
``skrobot.coordinates.math`` at each call site --
``matrix_relative`` for parent^-1 @ child, ``rpy2homogeneous`` / ``rpy2matrix``
for rpy -> matrix (URDF convention, extrinsic X-Y-Z: R = Rz @ Ry @ Rx).
NB: skrobot's legacy ``rpy_matrix(yaw, pitch, roll)`` takes its arguments in
the OPPOSITE order -- avoid it.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np
from skrobot.coordinates.math import (
    matrix2xyzrpy,
    orthonormalize_rotation_matrix,
    rpy2homogeneous,
)


def transform_to_matrix(array_data):
    """SolidWorks ArrayData (len>=12) -> 4x4 numpy (local->global, metres)."""
    a = list(array_data)
    r = a[:9]
    t = a[9:12]
    # column-vector rotation = transpose of the row-major 3x3
    R = np.array([[r[0], r[3], r[6]],
                  [r[1], r[4], r[7]],
                  [r[2], r[5], r[8]]], dtype=float)
    M = np.eye(4)
    # SW matrices carry float drift after long mate/assembly chains; project
    # back onto SO(3) here, at the single entry point, so downstream
    # rpy/quaternion conversions never see a non-orthogonal rotation.
    M[:3, :3] = orthonormalize_rotation_matrix(R)
    M[:3, 3] = t
    return M


def matrix_to_xyz_rpy(M):
    """4x4 -> (xyz, rpy) with URDF rpy = extrinsic XYZ (roll,pitch,yaw)."""
    xyz, rpy = matrix2xyzrpy(np.asarray(M, dtype=float))
    # snap sub-1e-12 values (metres / radians -- far below any physical
    # significance) to EXACT 0: the SVD in orthonormalize_rotation_matrix
    # leaves BLAS-dependent noise (~1e-16..1e-33) that differs per platform,
    # and fmt_urdf_num would faithfully print it, breaking cross-platform golden
    # comparisons of the URDF text.  Also folds IEEE negative zeros.
    def _snap(v):
        v = float(v)
        return 0.0 if abs(v) < 1e-12 else v
    return [_snap(v) for v in xyz], [_snap(v) for v in rpy]


def urdf_origin_matrix(el, strict=True):
    """4x4 from a URDF ``<origin>`` element (xyz + extrinsic-XYZ rpy
    attributes), or identity when the element is absent.  The ONE parser for
    ``<origin>`` -- exporter and editor both route through it so the rpy
    convention cannot drift between modules.

    ``strict`` (the default) is for URDFs sw2robot itself wrote: a wrong-length
    ``xyz`` / ``rpy`` there is an exporter bug, and raising surfaces it.  Pass
    ``strict=False`` for URDFs that arrived from outside -- hand-edited files,
    other tool chains -- where a single malformed attribute must not take the
    editor down with it; that attribute alone falls back to zeros."""
    if el is None:
        return np.eye(4)
    vecs = []
    for attr in ("xyz", "rpy"):
        v = [float(x) for x in (el.get(attr) or "0 0 0").split()]
        if len(v) != 3:
            if strict:
                raise ValueError(
                    "malformed <origin>: expected 3 values each, got "
                    f"xyz={el.get('xyz')!r} rpy={el.get('rpy')!r}")
            v = [0.0, 0.0, 0.0]
        vecs.append(v)
    xyz, rpy = vecs
    M = rpy2homogeneous(*rpy)
    M[:3, 3] = xyz
    return M


def fmt_urdf_num(v):
    """One float -> URDF attribute text.

    ``%.10g``: an editor edit re-writes every ``<origin>`` it touches, so the
    text is the storage format and has to survive write -> read -> write
    unchanged.  10 significant digits keep a millimetre-scale coordinate exact
    to well below a nanometre while staying readable; the 8 digits some call
    sites used to print truncated the value on the first edit.

    Rounding stops at that 10th digit and goes no further: floating-point
    noise is snapped where it is produced (``matrix_to_xyz_rpy``), not here,
    so a very small number that reaches this function is a real one and prints
    as itself.  The single exception is IEEE negative zero, which prints as
    ``0``: ``-0`` is legal XML but reads as a bug in an exported file.
    """
    v = float(v)
    return "0" if v == 0.0 else f"{v:.10g}"


def fmt_urdf_vec(vec):
    """A vector -> a space-separated URDF attribute value."""
    return " ".join(fmt_urdf_num(x) for x in vec)


def set_urdf_origin(el, M):
    """Write/overwrite ``el``'s ``<origin>`` child from a 4x4 matrix.

    A missing ``<origin>`` is created first among ``el``'s children, the
    position URDF conventionally puts it in."""
    origin = el.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        el.insert(0, origin)
    xyz, rpy = matrix_to_xyz_rpy(M)
    origin.set("xyz", fmt_urdf_vec(xyz))
    origin.set("rpy", fmt_urdf_vec(rpy))


def urdf_link_world_poses(root):
    """Zero-position link poses from URDF joint origins.

    Editor URDFs are routinely partial -- several roots, or links no joint
    reaches -- and every disconnected piece sits at identity.

    Only the kinematic skeleton goes to skrobot, because the rest of an editor
    URDF does not survive a load: meshes are still .3dxml, and inertials are
    empty placeholders until the mass pass runs.  Joints go over as `fixed` so
    that a limit range excluding zero cannot displace the frames -- skrobot
    starts a prismatic joint at its lower limit, not at zero.  Every root is
    then hung off one synthetic link by an identity joint, which leaves the
    poses untouched and hands skrobot the single connected tree it validates
    for; a disconnected editor URDF is the normal case here, and letting it
    log a graph error every time would bury the errors that matter.
    """
    from skrobot.models.urdf import RobotModelFromURDF

    names = list(dict.fromkeys(ln.get("name") for ln in root.findall("link")
                               if ln.get("name")))
    if not names:
        return {}

    edges, parented = [], set()
    for j in root.findall("joint"):
        p = j.find("parent")
        c = j.find("child")
        parent = p.get("link") if p is not None else ""
        child = c.get("link") if c is not None else ""
        # a link has exactly one parent, and none via a self-loop; keeping the
        # first joint per child mirrors the depth-first walk this replaced
        if not parent or not child or parent == child or child in parented:
            continue
        parented.add(child)
        edges.append((parent, child, j.find("origin")))

    # a joint may name a link the document never declares; carry it so the
    # chain below it still resolves, then drop it from the result
    linked = list(dict.fromkeys(
        names + [p for p, _, _ in edges] + [c for _, c, _ in edges]))
    anchor = "_fk_root"
    while anchor in linked:
        anchor += "_"

    skeleton = ET.Element("robot", {"name": "fk"})
    ET.SubElement(skeleton, "link", {"name": anchor})
    for name in linked:
        ET.SubElement(skeleton, "link", {"name": name})
    # joint names are generated, never copied, so a duplicated or missing name
    # in the source cannot collide here
    for i, (parent, child, origin) in enumerate(edges):
        elem = ET.SubElement(skeleton, "joint",
                              {"name": f"e{i}", "type": "fixed"})
        ET.SubElement(elem, "parent", {"link": parent})
        ET.SubElement(elem, "child", {"link": child})
        if origin is not None:
            ET.SubElement(elem, "origin", dict(origin.attrib))
    for i, name in enumerate(n for n in linked if n not in parented):
        elem = ET.SubElement(skeleton, "joint",
                              {"name": f"r{i}", "type": "fixed"})
        ET.SubElement(elem, "parent", {"link": anchor})
        ET.SubElement(elem, "child", {"link": name})

    rb = RobotModelFromURDF(urdf=ET.tostring(skeleton, encoding="unicode"))
    poses = {ln.name: ln.worldcoords().T() for ln in rb.link_list}
    return {name: poses.get(name, np.eye(4)) for name in names}
