"""Reconstruction from the SW2URDF *naming convention*.

The add-in's dialog defaults name a link's coordinate system ``<link>_origin``
and the root's ``base_origin``; assemblies authored that way can be rebuilt
from the feature names alone.  Assemblies that used other names need
:mod:`.geometric` or the authoritative :mod:`.bridge` payload route.
"""

from __future__ import annotations

import numpy as np

from ._common import (
    _axis_line,
    _coord_matrix,
    _edge_supports_axis,
    _field,
    _link_component_candidates,
    _normalize_axis_direction,
    _point_line_distance,
    _warn,
)


def detect_sw2urdf_reference_geometry(components, coordinate_systems,
                                      reference_axes):
    """Return True when SW2URDF-style reference geometry is present.

    Parameters
    ----------
    components : sequence
        Top-level components.
    coordinate_systems : sequence
        Top-level coordinate systems.
    reference_axes : sequence
        Top-level reference axes.

    Returns
    -------
    bool
        True when at least two link components (matched by ``*_origin``) and
        at least one reference axis exist.
    """
    links = _link_component_candidates(components, coordinate_systems)
    return len(links) >= 2 and len(reference_axes or []) >= 1


def reconstruct_sw2urdf_config(components, coordinate_systems, reference_axes,
                               edges, ground, tolerance=1e-5):
    """Reconstruct SW2URDF mapping from graph-level data.

    Parameters
    ----------
    components : sequence
        Top-level components with ``name``.
    coordinate_systems : sequence
        Top-level coordinate systems with ``name`` and
        ``document_from_frame``.
    reference_axes : sequence
        Top-level reference axes with ``name``, ``document_point``,
        ``document_direction``.
    edges : sequence
        Top-level mate edges with ``a``, ``b``, axis and/or mate geometry.
    ground : sequence[str]
        Grounded top-level component names.
    tolerance : float, optional
        Distance/parallel tolerance in metres.

    Returns
    -------
    dict | None
        Mapping with keys ``links``, ``root_link``, ``joints`` when
        reconstruction succeeds, else None.
    """
    triggered = detect_sw2urdf_reference_geometry(
        components, coordinate_systems, reference_axes)
    if not triggered:
        return None

    coord_by_name = {str(_field(cs, "name")): cs for cs in coordinate_systems or []}
    link_candidates = _link_component_candidates(components, coordinate_systems)

    links = {}
    stems = {}
    for comp_name, rec in sorted(link_candidates.items()):
        origin_names = rec["origin_names"]
        if len(origin_names) != 1:
            _warn("step 1 failed: ambiguous origin mapping for component "
                  f"{comp_name!r}: {origin_names!r}")
            return None
        stem = rec["stem"]
        if stem in stems and stems[stem] != comp_name:
            _warn("step 1 failed: non-unique link stem "
                  f"{stem!r} for {stems[stem]!r} and {comp_name!r}")
            return None
        stems[stem] = comp_name
        origin_name = origin_names[0]
        links[comp_name] = {
            "link_name": stem,
            "origin_name": origin_name,
            "origin": _coord_matrix(coord_by_name[origin_name]),
        }

    if "base_origin" not in coord_by_name:
        _warn("step 1 failed: base_origin coordinate system not found")
        return None
    roots = [name for name, rec in links.items()
             if rec["origin_name"] == "base_origin"]
    if len(roots) != 1:
        _warn("step 1 failed: base_origin must map to exactly one link "
              f"component, got {roots!r}")
        return None
    root = roots[0]
    if root not in set(ground or []):
        _warn("step 1 failed: base_origin link "
              f"{root!r} is not grounded (ground={sorted(set(ground or []))!r})")
        return None

    origins = {name: rec["origin"][:3, 3].copy() for name, rec in links.items()}

    axis_meta = {}
    child_to_axis = {}
    for axis in reference_axes or []:
        axis_name = str(_field(axis, "name"))
        line = _axis_line(axis)
        if line is None:
            _warn("step 2 failed: reference axis "
                  f"{axis_name!r} has degenerate direction")
            return None
        dists = []
        for comp_name, point in origins.items():
            d = _point_line_distance(point, line[0], line[1])
            dists.append((comp_name, d))
        dists.sort(key=lambda x: (x[1], x[0]))
        if dists[0][1] > tolerance:
            _warn("step 2 failed: axis "
                  f"{axis_name!r} is {dists[0][1]:.3e} m away from nearest "
                  f"origin ({dists[0][0]!r}); tolerance={tolerance:g}")
            return None
        # Mirrored limbs make pitch axes COLLINEAR across the robot (both
        # thigh origins sit on one Y-line), so several origins can lie on
        # the axis line.  Disambiguate by 3D distance to the RefAxis's own
        # definition point, which sits in the owning link's geometry.
        on_line = [(name, float(np.linalg.norm(origins[name] - line[0])))
                   for name, d in dists if d <= tolerance]
        on_line.sort(key=lambda x: (x[1], x[0]))
        child = on_line[0][0]
        dmin = dict(dists)[child]
        if len(on_line) > 1 and on_line[1][1] - on_line[0][1] <= tolerance:
            _warn("step 2 failed: axis "
                  f"{axis_name!r} child assignment is not unique between "
                  f"{on_line[0][0]!r} and {on_line[1][0]!r} (both on the "
                  "axis line, equidistant from the axis point)")
            return None
        if child == root:
            _warn("step 2 failed: axis "
                  f"{axis_name!r} mapped to root link {root!r}")
            return None
        if child in child_to_axis:
            _warn("step 2 failed: link "
                  f"{child!r} received multiple axes "
                  f"{child_to_axis[child]!r} and {axis_name!r}")
            return None
        child_to_axis[child] = axis_name
        axis_meta[axis_name] = {
            "child": child,
            "axis_point": line[0].copy(),
            "axis_direction": line[1].copy(),
            "axis_origin_distance": float(dmin),
        }

    non_root = sorted(name for name in links if name != root)
    missing_children = [name for name in non_root if name not in child_to_axis]
    if missing_children:
        _warn("step 4 failed: non-root links without reference axis: "
              f"{missing_children!r}")
        return None

    edge_by_pair = {}
    link_names = set(links)
    for edge in edges or []:
        a = str(_field(edge, "a"))
        b = str(_field(edge, "b"))
        if a in link_names and b in link_names and a != b:
            edge_by_pair[frozenset((a, b))] = edge

    joints = []
    for axis in reference_axes or []:
        axis_name = str(_field(axis, "name"))
        meta = axis_meta.get(axis_name)
        if meta is None:
            continue
        child = meta["child"]
        axis_line = (meta["axis_point"], meta["axis_direction"])
        neighbors = sorted(
            name for name in link_names
            if name != child and frozenset((child, name)) in edge_by_pair)
        matches = []
        for nb in neighbors:
            edge = edge_by_pair[frozenset((child, nb))]
            if _edge_supports_axis(edge, axis_line, tolerance):
                matches.append(nb)
        if len(matches) != 1:
            _warn("step 3 failed: child "
                  f"{child!r} axis {axis_name!r} expected exactly one parent "
                  f"neighbor on-axis, got {matches!r} from neighbors "
                  f"{neighbors!r}")
            return None
        axis_dir, flipped = _normalize_axis_direction(meta["axis_direction"])
        joints.append({
            "name": axis_name,
            "parent": matches[0],
            "child": child,
            "axis_point": meta["axis_point"].copy(),
            "axis_direction": axis_dir,
            "axis_origin_distance": meta["axis_origin_distance"],
            "flipped": bool(flipped),
        })

    if len(joints) != len(links) - 1:
        _warn("step 4 failed: joint count mismatch, got "
              f"{len(joints)} for {len(links)} links")
        return None

    parent_of = {}
    for joint in joints:
        child = joint["child"]
        if child in parent_of:
            _warn("step 4 failed: duplicate child in reconstructed joints: "
                  f"{child!r}")
            return None
        parent_of[child] = joint["parent"]

    children_of = {}
    for child, parent in parent_of.items():
        children_of.setdefault(parent, []).append(child)
    seen = {root}
    queue = [root]
    while queue:
        cur = queue.pop()
        for child in children_of.get(cur, []):
            if child in seen:
                continue
            seen.add(child)
            queue.append(child)
    if seen != set(links):
        _warn("step 4 failed: some links are not reachable from root "
              f"{root!r}: {sorted(set(links) - seen)!r}")
        return None

    return {
        "links": links,
        "root_link": root,
        "joints": joints,
    }
