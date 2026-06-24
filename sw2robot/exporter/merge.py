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

import numpy as np

from .geometry import matrix_from_rpy, matrix_to_xyz_rpy


def _fmt(v):
    # compact, round-trippable; mirror the writer's style (no sci-notation noise)
    return f"{float(v):.10g}"


def _origin_matrix(el):
    """4x4 from an ``<origin>`` element (xyz + extrinsic-XYZ rpy), or identity."""
    if el is None:
        return np.eye(4)
    xyz = [float(x) for x in (el.get("xyz") or "0 0 0").split()]
    rpy = [float(x) for x in (el.get("rpy") or "0 0 0").split()]
    m = matrix_from_rpy(rpy)
    m[:3, 3] = xyz[:3]
    return m


def _set_origin(parent_el, M):
    """Write/overwrite ``parent_el``'s ``<origin>`` from a 4x4 matrix."""
    xyz, rpy = matrix_to_xyz_rpy(M)
    o = parent_el.find("origin")
    if o is None:
        o = ET.SubElement(parent_el, "origin")
    o.set("xyz", " ".join(_fmt(v) for v in xyz))
    o.set("rpy", " ".join(_fmt(v) for v in rpy))


def _has_geometry(link_el):
    """A link is mergeable geometry (not a bare coordinate frame) when it has a
    visual or collision."""
    return (link_el.find("visual") is not None
            or link_el.find("collision") is not None)


def _inertia_tensor(inertia_el):
    def g(k):
        return float(inertia_el.get(k, "0"))
    ixx, ixy, ixz = g("ixx"), g("ixy"), g("ixz")
    iyy, iyz, izz = g("iyy"), g("iyz"), g("izz")
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], float)


def _parse_inertial(link_el):
    """``(mass, com(3), I(3x3) about com)`` for a link, or None if no inertial.
    Inertial ``rpy`` is honoured (rotates the tensor into link axes)."""
    el = link_el.find("inertial")
    if el is None:
        return None
    mass_el = el.find("mass")
    mass = float(mass_el.get("value", "0")) if mass_el is not None else 0.0
    o = el.find("origin")
    com = np.array([float(x) for x in (o.get("xyz") if o is not None
                                       and o.get("xyz") else "0 0 0").split()],
                   float)
    rpy = [float(x) for x in (o.get("rpy") if o is not None and o.get("rpy")
                              else "0 0 0").split()]
    it = el.find("inertia")
    if it is None:
        return None
    inertia = _inertia_tensor(it)
    if any(rpy):                       # express the tensor in link axes
        R = matrix_from_rpy(rpy)[:3, :3]
        inertia = R @ inertia @ R.T
    return mass, com, inertia


def _write_inertial(link_el, mass, com, inertia):
    """Replace ``link_el``'s ``<inertial>`` with the given mass/com/tensor
    (axis-aligned: rpy=0)."""
    old = link_el.find("inertial")
    if old is not None:
        link_el.remove(old)
    el = ET.SubElement(link_el, "inertial")
    o = ET.SubElement(el, "origin")
    o.set("xyz", " ".join(_fmt(v) for v in com))
    o.set("rpy", "0 0 0")
    ET.SubElement(el, "mass").set("value", _fmt(mass))
    it = ET.SubElement(el, "inertia")
    for k, v in (("ixx", inertia[0, 0]), ("ixy", inertia[0, 1]),
                 ("ixz", inertia[0, 2]), ("iyy", inertia[1, 1]),
                 ("iyz", inertia[1, 2]), ("izz", inertia[2, 2])):
        it.set(k, _fmt(v))


def _parallel_axis(mass, com, inertia, new_com):
    """Shift an inertia tensor (about ``com``) to be about ``new_com``."""
    d = np.asarray(com, float) - np.asarray(new_com, float)
    return inertia + mass * (float(d @ d) * np.eye(3) - np.outer(d, d))


def _combine_inertials(parent_el, child_el, T_pc):
    """Lump the child link's <inertial> into the parent's, with the child first
    moved into the parent frame by ``T_pc`` (parent<-child).  No-op if the child
    has no inertial; seeds the parent if it had none."""
    ci = _parse_inertial(child_el)
    if ci is None:
        return
    m2, c2, I2 = ci
    R = T_pc[:3, :3]
    c2 = (T_pc @ np.append(c2, 1.0))[:3]      # child COM in parent frame
    I2 = R @ I2 @ R.T                          # child tensor in parent axes
    pi = _parse_inertial(parent_el)
    if pi is None or pi[0] <= 0:
        _write_inertial(parent_el, m2, c2, I2)
        return
    m1, c1, I1 = pi
    m = m1 + m2
    com = (m1 * c1 + m2 * c2) / m
    inertia = (_parallel_axis(m1, c1, I1, com)
               + _parallel_axis(m2, c2, I2, com))
    _write_inertial(parent_el, m, com, inertia)


def merge_fixed_links(root):
    """Lump fixed-joint children with geometry into their parents, IN PLACE on
    the URDF ``root`` element.  Returns ``(merged_count, root)``."""
    # Coordinate frames are the links that have NO geometry of their OWN (snapshot
    # before any merging).  They are preserved: never merged away as a child, and
    # never used as a merge target -- otherwise a frame in a chain like
    # base--fixed-->frame--fixed-->part would receive part's geometry and then get
    # lumped into base, silently dropping the frame's TF (caught in review).
    original_frames = {ln.get("name") for ln in root.findall("link")
                       if not _has_geometry(ln)}
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
            cl = links.get(cname)
            pl = links.get(pname)
            if cl is None or pl is None or not _has_geometry(cl):
                continue                       # keep mesh-less frames + danglers
            if cname in original_frames or pname in original_frames:
                continue                       # preserve coordinate frames
            target = (j, pl, cl, pname, cname)
            break
        if target is None:
            break
        j, parent_link, child_link, pname, cname = target
        T_pc = _origin_matrix(j.find("origin"))

        # 1) move the child's visual/collision into the parent (compose origin)
        for vc in list(child_link):
            if vc.tag not in ("visual", "collision"):
                continue
            _set_origin(vc, T_pc @ _origin_matrix(vc.find("origin")))
            child_link.remove(vc)
            parent_link.append(vc)
        # 2) combine inertials
        _combine_inertials(parent_link, child_link, T_pc)
        # 3) re-parent every joint that hung off the child onto the parent
        for k in joints:
            kp = k.find("parent")
            if kp is not None and kp.get("link") == cname:
                kp.set("link", pname)
                _set_origin(k, T_pc @ _origin_matrix(k.find("origin")))
        # 4) drop the fixed joint and the (now empty) child link
        root.remove(j)
        root.remove(child_link)
        merged += 1
    return merged, root


def merge_fixed_links_text(urdf_text):
    """``urdf_text`` -> merged URDF text (XML declaration preserved)."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(urdf_text, parser=parser)
    merge_fixed_links(root)
    out = ET.tostring(root, encoding="unicode")
    if not out.startswith("<?xml"):
        out = '<?xml version="1.0"?>\n' + out
    if not out.endswith("\n"):
        out += "\n"
    return out
