"""Name-free reconstruction from reference geometry.

Matches each RefAxis against the mate edges that lie on it to recover the
joint tree, so an assembly whose features follow no naming convention still
yields the configured kinematics.  Gated on the SW2URDF marker by the
caller: stray reference axes in an unconfigured assembly must not trigger it.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from ._common import (
    _axis_line,
    _component_stem,
    _component_translation,
    _coord_matrix,
    _edge_line_candidates,
    _field,
    _line_matches,
    _normalize_axis_direction,
    _point_line_distance,
    _warn,
)


def reconstruct_sw2urdf_config_geometric(components, coordinate_systems,
                                         reference_axes, edges, ground,
                                         tolerance=1e-5):
    """Reconstruct SW2URDF mapping from marker-gated geometric evidence.

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
    if not (reference_axes or []):
        _warn("geometric step 1 failed: no reference axes found")
        return None

    component_by_name = {
        str(_field(component, "name")): component
        for component in (components or [])
    }

    axis_meta = {}
    pair_to_axis = {}
    for axis in reference_axes or []:
        axis_name = str(_field(axis, "name"))
        line = _axis_line(axis)
        if line is None:
            _warn("geometric step 1 failed: reference axis "
                  f"{axis_name!r} has degenerate direction")
            return None

        matched_pairs = {}
        for edge in edges or []:
            a = str(_field(edge, "a"))
            b = str(_field(edge, "b"))
            if a == b:
                continue
            support = [cand for cand in _edge_line_candidates(edge)
                       if _line_matches(cand, line, tolerance)]
            if not support:
                continue
            dist = min(float(np.linalg.norm(np.asarray(pv, float) - line[0]))
                       for pv, _dv in support)
            key = frozenset((a, b))
            matched_pairs[key] = min(matched_pairs.get(key, float("inf")),
                                     dist)

        if not matched_pairs:
            _warn("geometric step 1 failed: axis "
                  f"{axis_name!r} matched no mate edge")
            return None
        # Mirrored limbs make pitch axes COLLINEAR across the robot, so
        # several component pairs can sit on one line.  The owning pair's
        # mate geometry lies at the axis feature's own definition point --
        # rank by that distance and require a clear winner.
        ranked = sorted(matched_pairs.items(),
                        key=lambda kv: (kv[1], tuple(sorted(kv[0]))))
        if len(ranked) > 1 and ranked[1][1] - ranked[0][1] <= tolerance:
            pairs = [tuple(sorted(kv[0])) for kv in ranked[:2]]
            _warn("geometric step 1 failed: axis "
                  f"{axis_name!r} is equidistant to component pairs "
                  f"{pairs!r}")
            return None

        pair = ranked[0][0]
        if pair in pair_to_axis:
            _warn("geometric step 1 failed: component pair "
                  f"{tuple(sorted(pair))!r} matched by multiple axes "
                  f"{pair_to_axis[pair]!r} and {axis_name!r}")
            return None
        pair_to_axis[pair] = axis_name
        axis_meta[axis_name] = {
            "pair": pair,
            "axis_point": line[0].copy(),
            "axis_direction": line[1].copy(),
        }

    link_set = set()
    for pair in pair_to_axis:
        link_set.update(pair)
    if not link_set:
        _warn("geometric step 2 failed: no link components matched by axes")
        return None

    ground_set = {str(name) for name in (ground or [])}
    grounded_in_links = sorted(name for name in link_set if name in ground_set)
    fixed_links = sorted(
        name for name in link_set
        if bool(_field(component_by_name.get(name), "fixed", False)))

    # Root identification, most to least specific.  ``ground`` alone is NOT
    # enough: the mate solver marks every immobile component as grounded, so
    # a fully-constrained robot lists nearly all links there.
    root = None
    if len(fixed_links) == 1:
        root = fixed_links[0]
    if root is None:
        # SW2URDF gives every link an origin CoordSys; the non-root ones sit
        # ON their joint's axis line, so a frame claimed by NO axis is the
        # root's.  Take the link component nearest to it, with a clear margin.
        lines = [(meta["axis_point"], meta["axis_direction"])
                 for meta in axis_meta.values()]
        leftovers = []
        for cs in coordinate_systems or []:
            p = _coord_matrix(cs)[:3, 3]
            if not any(_point_line_distance(p, lp, ld) <= tolerance
                       for lp, ld in lines):
                leftovers.append(p)
        if len(leftovers) == 1:
            dists = []
            for name in sorted(link_set):
                comp = component_by_name.get(name)
                world = np.asarray(_field(comp, "world"), float).reshape(4, 4)
                dists.append(
                    (float(np.linalg.norm(world[:3, 3] - leftovers[0])),
                     name))
            dists.sort()
            if len(dists) == 1 or dists[1][0] - dists[0][0] > tolerance:
                root = dists[0][1]
    if root is None and len(grounded_in_links) == 1:
        root = grounded_in_links[0]
    if root is None and not grounded_in_links and len(ground_set) == 1:
        root = next(iter(ground_set))
        link_set.add(root)
    if root is None:
        _warn("geometric step 3 failed: could not identify a unique root "
              f"(fixed={fixed_links!r}, grounded-in-links="
              f"{grounded_in_links!r})")
        return None

    if root not in component_by_name:
        _warn("geometric step 3 failed: root component "
              f"{root!r} is not present in components")
        return None

    neighbors = {name: [] for name in link_set}
    for pair, axis_name in pair_to_axis.items():
        a, b = tuple(pair)
        if a not in link_set or b not in link_set:
            continue
        neighbors[a].append((b, axis_name))
        neighbors[b].append((a, axis_name))
    for name in neighbors:
        neighbors[name].sort(key=lambda item: (item[0], item[1]))

    parent = {}
    axis_for_child = {}
    child_order = []
    seen = {root}
    queue = deque([root])
    while queue:
        current = queue.popleft()
        for neighbor, axis_name in neighbors.get(current, []):
            if parent.get(current) == neighbor:
                continue
            if neighbor in seen:
                _warn("geometric step 3 failed: axis-edge graph has a cycle")
                return None
            seen.add(neighbor)
            parent[neighbor] = current
            axis_for_child[neighbor] = axis_name
            child_order.append(neighbor)
            queue.append(neighbor)

    if seen != link_set:
        _warn("geometric step 3 failed: axis-edge graph is disconnected from "
              f"root {root!r}; unreachable={sorted(link_set - seen)!r}")
        return None
    if len(axis_for_child) != len(link_set) - 1:
        _warn("geometric step 3 failed: expected a tree with "
              f"{len(link_set) - 1} edges, got {len(axis_for_child)}")
        return None

    coord_entries = []
    for coord in coordinate_systems or []:
        try:
            coord_entries.append((str(_field(coord, "name")), _coord_matrix(coord)))
        except Exception as e:
            _warn(f"geometric step 4 warning: skipping invalid CoordSys: {e!r}")

    claimed_coords = set()
    links = {}

    child_by_axis = {}
    for child in child_order:
        axis_name = axis_for_child[child]
        child_by_axis[axis_name] = child
        meta = axis_meta[axis_name]
        axis_point = meta["axis_point"]
        axis_direction = meta["axis_direction"]

        candidates = []
        for coord_name, coord_matrix in coord_entries:
            if coord_name in claimed_coords:
                continue
            point = np.asarray(coord_matrix[:3, 3], float)
            if _point_line_distance(point, axis_point, axis_direction) <= tolerance:
                candidates.append((
                    float(np.linalg.norm(point - axis_point)),
                    coord_name,
                    coord_matrix,
                ))
        candidates.sort(key=lambda item: (item[0], item[1]))

        if candidates:
            if (len(candidates) > 1 and
                    abs(candidates[1][0] - candidates[0][0]) <= tolerance):
                _warn("geometric step 4 failed: child "
                      f"{child!r} has ambiguous CoordSys anchors on axis "
                      f"{axis_name!r}")
                return None
            _, origin_name, origin = candidates[0]
            claimed_coords.add(origin_name)
        else:
            origin_name = None
            origin = np.eye(4)
            origin[:3, 3] = axis_point

        links[child] = {
            "link_name": _component_stem(child),
            "origin_name": origin_name,
            "origin": origin,
        }

    leftover = [(name, mat) for name, mat in coord_entries
                if name not in claimed_coords]
    if len(leftover) == 1:
        root_origin_name, root_origin = leftover[0]
    else:
        root_origin_name = None
        root_origin = np.eye(4)
        root_origin[:3, 3] = _component_translation(component_by_name[root])

    links[root] = {
        "link_name": _component_stem(root),
        "origin_name": root_origin_name,
        "origin": root_origin,
    }

    joints = []
    for axis in reference_axes or []:
        axis_name = str(_field(axis, "name"))
        child = child_by_axis.get(axis_name)
        if child is None:
            continue
        parent_name = parent.get(child)
        if parent_name is None:
            _warn("geometric step 3 failed: child "
                  f"{child!r} has no parent for axis {axis_name!r}")
            return None
        meta = axis_meta[axis_name]
        axis_dir, flipped = _normalize_axis_direction(meta["axis_direction"])
        child_origin = np.asarray(links[child]["origin"][:3, 3], float)
        joints.append({
            "name": axis_name,
            "parent": parent_name,
            "child": child,
            "axis_point": meta["axis_point"].copy(),
            "axis_direction": axis_dir,
            "axis_origin_distance": _point_line_distance(
                child_origin, meta["axis_point"], meta["axis_direction"]),
            "flipped": bool(flipped),
        })

    if len(joints) != len(link_set) - 1:
        _warn("geometric step 3 failed: joint count mismatch, got "
              f"{len(joints)} for {len(link_set)} links")
        return None

    return {
        "links": links,
        "root_link": root,
        "joints": joints,
    }
