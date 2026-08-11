"""Shared primitives for reading an embedded SW2URDF configuration.

Geometry predicates (point-to-line distance, parallel / collinear tests),
component-name handling and the deterministic axis sign normalisation that
every reconstruction route relies on.  Imports nothing from its siblings, so
it stays the bottom of the package's import graph.
"""

from __future__ import annotations

import re

import numpy as np

_INSTANCE_SUFFIX_RE = re.compile(r"-\d+$")
# Payload CoordSys reference scoped to a component instance: "name <comp-1>"
_SCOPED_COORDSYS_RE = re.compile(r"^(.*) <([^<>]+)>$")
_EPS = 1e-12


def _warn(msg):
    print("      !!! SW2URDF config warning: " + str(msg))


def _field(obj, key, default=None):
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _component_stem(name):
    return _INSTANCE_SUFFIX_RE.sub("", str(name or ""))


def _origin_name_candidates(stem):
    names = [f"{stem}_origin"]
    if stem.endswith("_link"):
        names.append(f"{stem[:-5]}_origin")
    return names


def _coord_matrix(coord):
    vals = np.asarray(_field(coord, "document_from_frame"), float)
    return vals.reshape(4, 4)


def _axis_line(axis):
    p = np.asarray(_field(axis, "document_point"), float)
    d = np.asarray(_field(axis, "document_direction"), float)
    n = float(np.linalg.norm(d))
    if n < _EPS:
        return None
    return p, d / n


def _point_line_distance(point, line_point, line_dir):
    v = np.asarray(point, float) - np.asarray(line_point, float)
    return float(np.linalg.norm(np.cross(v, line_dir)))


def _parallel(d1, d2, tol):
    n1 = float(np.linalg.norm(d1))
    n2 = float(np.linalg.norm(d2))
    if n1 < _EPS or n2 < _EPS:
        return False
    return float(np.linalg.norm(np.cross(d1 / n1, d2 / n2))) <= tol


def _line_matches(line_a, line_b, tol):
    pa, da = line_a
    pb, db = line_b
    if not _parallel(da, db, tol):
        return False
    n = float(np.linalg.norm(db))
    if n < _EPS:
        return False
    return _point_line_distance(pa, pb, db / n) <= tol


def _normalize_axis_direction(d):
    dn = np.asarray(d, float)
    n = float(np.linalg.norm(dn))
    if n < _EPS:
        return dn, False
    dn = dn / n
    i = int(np.argmax(np.abs(dn)))
    flipped = bool(dn[i] < 0.0)
    if flipped:
        dn = -dn
    # Snap float noise (and negative zeros from the flip) so the URDF text
    # never carries '-0' or 1e-16 residue.
    dn[np.abs(dn) < 1e-12] = 0.0
    return dn, flipped


def _edge_line_candidates(edge):
    out = []
    p = _field(edge, "axis_point")
    d = _field(edge, "axis_dir")
    if p is not None and d is not None:
        dv = np.asarray(d, float)
        nv = float(np.linalg.norm(dv))
        if nv > _EPS:
            out.append((np.asarray(p, float), dv / nv))
    for mate in _field(edge, "mates", []) or []:
        raw = mate.model_dump() if hasattr(mate, "model_dump") else mate
        pts = list(_field(raw, "points", []) or [])
        dirs = list(_field(raw, "dirs", []) or [])
        for i, dv in enumerate(dirs):
            dvec = np.asarray(dv, float)
            nd = float(np.linalg.norm(dvec))
            if nd < _EPS:
                continue
            if i < len(pts):
                pv = np.asarray(pts[i], float)
            elif pts:
                pv = np.asarray(pts[0], float)
            else:
                continue
            out.append((pv, dvec / nd))
    return out


def _edge_supports_axis(edge, axis_line, tol):
    for cand in _edge_line_candidates(edge):
        if _line_matches(cand, axis_line, tol):
            return True
    return False


def _link_component_candidates(components, coordinate_systems):
    coord_names = {str(_field(cs, "name")) for cs in coordinate_systems or []}
    out = {}
    for comp in components or []:
        cname = str(_field(comp, "name"))
        stem = _component_stem(cname)
        matches = [n for n in _origin_name_candidates(stem) if n in coord_names]
        if matches:
            out[cname] = {"stem": stem, "origin_names": matches}
    return out


def _component_translation(component):
    """Return component world translation in document coordinates."""
    if hasattr(component, "world_matrix"):
        mat = np.asarray(component.world_matrix(), float)
        return np.asarray(mat[:3, 3], float)
    world = _field(component, "world")
    if world is None:
        return np.zeros(3, float)
    mat = np.asarray(world, float).reshape(4, 4)
    return np.asarray(mat[:3, 3], float)
