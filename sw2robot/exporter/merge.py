"""Fixed-joint lumping for URDF: merge every fixed-joint child that carries
geometry into its parent, so the exported / displayed URDF has one rigid link
per moving body instead of a chain of rigidly-attached parts.

Operates on the URDF XML (``xml.etree.ElementTree``) so it works the same in CAD
and URDF-input mode, and feeds both the export and the viewer.

For each ``<joint type="fixed">`` whose child link has ``<visual>`` /
``<collision>`` geometry:
  * the child's ``<visual>`` / ``<collision>`` are moved into the parent link,
    their ``<origin>`` pre-composed with the fixed-joint origin (parent<-child),
  * the two ``<inertial>`` blocks are combined (mass add + parallel-axis tensor),
  * any joint that hung off the child is re-parented to the parent, its
    ``<origin>`` pre-composed with the fixed transform,
  * the fixed ``<joint>`` and the child ``<link>`` are removed.

Mesh-LESS fixed children (coordinate frames -- ``dummy_link`` / port markers,
i.e. links with no ``<visual>`` and no ``<collision>``) are NOT merged: they are
kept as links so their TF frame survives.  The transform iterates to a fixpoint,
so chains of fixed joints collapse onto the nearest moving (or root) link.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

from skrobot.utils.inertia import combine_inertials, transform_inertial

from .geometry import (
    fmt_urdf_num,
    fmt_urdf_vec,
    matrix_to_xyz_rpy,
    set_urdf_origin,
    urdf_origin_matrix,
)


def _has_geometry(link_el):
    """A link is mergeable geometry (not a bare coordinate frame) when it has a
    visual or collision."""
    return (link_el.find("visual") is not None
            or link_el.find("collision") is not None)


def _vec3(el, attr):
    """A 3-vector attribute of ``el`` (``"0 0 0"`` when absent/empty)."""
    raw = (el.get(attr) if el is not None else None) or "0 0 0"
    return [float(x) for x in raw.split()]


def _parse_inertial(link_el):
    """A link's ``<inertial>`` as skrobot's ``{mass, com, inertia, method}``
    dict, expressed in the LINK frame -- or None when the block is missing or
    carries a non-positive / non-finite mass.

    The inertial ``<origin>`` is applied by ``transform_inertial``: its ``rpy``
    rotates the tensor into link axes and its ``xyz`` places the centre of
    mass."""
    el = link_el.find("inertial")
    if el is None:
        return None
    it = el.find("inertia")
    if it is None:
        return None
    mass_el = el.find("mass")
    mass = float(mass_el.get("value", "0")) if mass_el is not None else 0.0
    components = tuple(float(it.get(k, "0")) for k in
                       ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
    origin = el.find("origin")
    return transform_inertial(mass, [0.0, 0.0, 0.0], components,
                              _vec3(origin, "xyz"), _vec3(origin, "rpy"))


def _write_inertial(link_el, info):
    """Replace ``link_el``'s ``<inertial>`` with ``info`` (a skrobot inertial
    dict), written axis-aligned: ``rpy=0``, since the tensor is already in link
    axes about ``com``."""
    old = link_el.find("inertial")
    if old is not None:
        link_el.remove(old)
    el = ET.SubElement(link_el, "inertial")
    o = ET.SubElement(el, "origin")
    o.set("xyz", fmt_urdf_vec(info["com"]))
    o.set("rpy", "0 0 0")
    ET.SubElement(el, "mass").set("value", fmt_urdf_num(info["mass"]))
    it = ET.SubElement(el, "inertia")
    for k, v in zip(("ixx", "ixy", "ixz", "iyy", "iyz", "izz"),
                    info["inertia"]):
        it.set(k, fmt_urdf_num(v))


def _combine_inertials(parent_el, child_el, T_pc):
    """Lump the child link's ``<inertial>`` into the parent's, with the child
    first moved into the parent frame by ``T_pc`` (parent<-child).  No-op if the
    child has no usable inertial; seeds the parent if it had none."""
    child = _parse_inertial(child_el)
    if child is None:
        return
    xyz, rpy = matrix_to_xyz_rpy(T_pc)
    child = transform_inertial(child["mass"], child["com"], child["inertia"],
                               xyz, rpy)
    combined = combine_inertials(_parse_inertial(parent_el), child)
    if combined is not None:
        _write_inertial(parent_el, combined)


def merge_fixed_links(root, force_merge=None, only=None):
    """Lump fixed-joint children with geometry into their parents, IN PLACE on
    the URDF ``root`` element.  Returns ``(merged_count, root)``.

    ``force_merge`` -- child-link names to lump into their fixed parent even
    when they carry no geometry, so a mass-only link's ``<inertial>`` still folds
    into the parent and its weight is preserved.  Geometry-less links NOT named
    here are coordinate frames and stay untouched.

    ``only`` -- restrict folding to these child links; every other fixed child
    is left in place.  Use it to fold just the mass-only links without collapsing
    the rest of the fixed structure (it implies ``force_merge`` when that is
    unset, so the named links fold regardless of geometry)."""
    if only is not None and force_merge is None:
        force_merge = only
    force_merge = set(force_merge or ())
    only = None if only is None else set(only)
    # Coordinate frames are the links that have NO geometry of their OWN (snapshot
    # before any merging).  They are preserved: never merged away as a child, and
    # never used as a merge target -- otherwise a frame in a chain like
    # base--fixed-->frame--fixed-->part would receive part's geometry and then get
    # lumped into base, silently dropping the frame's TF (caught in review).  A
    # mass-only link is geometry-less too, so EXCLUDE the force_merge names here --
    # they are meant to be folded away, not preserved as frames.
    original_frames = {ln.get("name") for ln in root.findall("link")
                       if not _has_geometry(ln)} - force_merge
    merged = 0
    while True:
        links = {ln.get("name"): ln for ln in root.findall("link")}
        joints = root.findall("joint")
        target = None
        for j in joints:
            if j.get("type") != "fixed":
                continue
            cp = j.find("parent"), j.find("child")
            if cp[0] is None or cp[1] is None:
                continue
            pname, cname = cp[0].get("link"), cp[1].get("link")
            if only is not None and cname not in only:
                continue                       # fold just the named children
            cl = links.get(cname)
            pl = links.get(pname)
            # a mass-only child (force_merge) folds even with no geometry, so its
            # mass reaches the parent; everything else still needs geometry to be
            # mergeable (mesh-less frames + danglers stay put)
            if cl is None or pl is None \
                    or not (_has_geometry(cl) or cname in force_merge):
                continue
            # frames are preserved: never merged away as a child, and normally
            # never a merge TARGET either (a frame that received geometry could
            # then be lumped away, dropping its TF).  EXCEPTION: a geometry-less
            # mass-only child (force_merge) folding into a frame parent adds only
            # <inertial>, never geometry, so the frame stays a frame -- allow it
            # so the child's weight still reaches the parent instead of stranding.
            if cname in original_frames:
                continue
            if pname in original_frames and not (
                    cname in force_merge and not _has_geometry(cl)):
                continue
            target = (j, pl, cl, pname, cname)
            break
        if target is None:
            break
        j, parent_link, child_link, pname, cname = target
        T_pc = urdf_origin_matrix(j.find("origin"))

        # 1) move the child's visual/collision into the parent (compose origin)
        for vc in list(child_link):
            if vc.tag not in ("visual", "collision"):
                continue
            set_urdf_origin(vc, T_pc @ urdf_origin_matrix(vc.find("origin")))
            child_link.remove(vc)
            parent_link.append(vc)
        # 2) combine inertials
        _combine_inertials(parent_link, child_link, T_pc)
        # 3) re-parent every joint that hung off the child onto the parent
        for k in joints:
            kp = k.find("parent")
            if kp is not None and kp.get("link") == cname:
                kp.set("link", pname)
                set_urdf_origin(k, T_pc @ urdf_origin_matrix(k.find("origin")))
        # 4) drop the fixed joint and the (now empty) child link
        root.remove(j)
        root.remove(child_link)
        merged += 1
    return merged, root


def merge_fixed_links_text(urdf_text, force_merge=None, only=None):
    """``urdf_text`` -> merged URDF text (XML declaration preserved).

    ``force_merge`` / ``only`` are forwarded to :func:`merge_fixed_links`.
    Comments are kept on the round trip so per-link provenance survives parsing."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(urdf_text, parser=parser)
    merge_fixed_links(root, force_merge=force_merge, only=only)
    out = ET.tostring(root, encoding="unicode")
    if not out.startswith("<?xml"):
        out = '<?xml version="1.0"?>\n' + out
    if not out.endswith("\n"):
        out += "\n"
    return out
