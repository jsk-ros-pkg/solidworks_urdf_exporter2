"""Turn a parsed SW2URDF payload into a link/joint mapping.

The payload states the configuration outright, so this route infers nothing:
it resolves each payload link to its top-level component(s), each joint to
the named RefAxis / CoordSys features, and mirrors limits when the axis sign
is normalised.  Everything it cannot resolve is reported and refused rather
than guessed.
"""

from __future__ import annotations

import numpy as np

from ._common import (
    _SCOPED_COORDSYS_RE,
    _axis_line,
    _component_translation,
    _coord_matrix,
    _field,
    _normalize_axis_direction,
    _point_line_distance,
    _unit,
    _warn,
)


def _payload_component_name(pid_value):
    return str(pid_value or "").split("@", 1)[0].strip()


def _payload_float(value):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _payload_number_dict(data, numeric_keys, text_keys=()):
    if not isinstance(data, dict):
        return None
    out = {}
    for key in text_keys:
        raw = data.get(key)
        if raw is None:
            continue
        text = str(raw).strip()
        if text:
            out[key] = text
    for key in numeric_keys:
        val = _payload_float(data.get(key))
        if val is not None:
            out[key] = val
    return out or None


def _orient_axis_to_payload(direction, axis_xyz, child_origin, joint_name):
    """Give ``direction`` the sign the add-in authored, instead of normalising.

    A SolidWorks reference axis is an undirected line, so the route's default
    is to pick a canonical sign (largest component positive).  The payload,
    though, records the sign SW2URDF actually exported, in the JOINT frame --
    which in URDF is the child link frame.  Rotating it into the document
    frame with the child's authored anchor makes the two comparable, so the
    RefAxis line keeps its (exact) geometry and only its sign is taken from
    the payload.

    Parameters
    ----------
    direction : array_like
        Unit axis direction in the document frame, from the RefAxis feature.
    axis_xyz : sequence | None
        The payload's ``axis`` triple, in the joint frame.
    child_origin : numpy.ndarray
        4x4 document-frame pose of the child link's authored frame.
    joint_name : str
        Joint name, for messages.

    Returns
    -------
    (numpy.ndarray, bool)
        Oriented direction and the ``flipped`` flag (always False here: the
        sign now MATCHES the payload, so its limits need no mirroring).
    """
    dn = _unit(direction)
    if dn is None:
        return np.asarray(direction, float), False
    payload_axis = None
    if axis_xyz is not None:
        try:
            payload_axis = _unit(axis_xyz)
        except Exception:
            payload_axis = None
    if payload_axis is None:
        _warn(f"sw2urdf_compat: joint {joint_name!r} has no payload axis to take a "
              "sign from; falling back to canonical sign normalisation")
        return _normalize_axis_direction(direction)

    world_axis = np.asarray(child_origin, float)[:3, :3] @ payload_axis
    world_axis = _unit(world_axis)
    if world_axis is None:
        _warn(f"sw2urdf_compat: joint {joint_name!r} payload axis degenerates under "
              "the child anchor rotation; falling back to canonical sign")
        return _normalize_axis_direction(direction)

    dot = float(np.dot(dn, world_axis))
    if abs(dot) < 0.9998:      # ~1.1 deg -- the two should be the same line
        _warn(f"sw2urdf_compat: joint {joint_name!r} payload axis and reference axis "
              f"disagree by {np.degrees(np.arccos(min(1.0, abs(dot)))):.3f} "
              "deg; keeping the reference axis geometry with the payload sign")
    if dot < 0.0:
        dn = -dn
    # Snap float noise (and negative zeros) so the URDF text stays clean.
    dn[np.abs(dn) < 1e-12] = 0.0
    return dn, False


def reconstruct_sw2urdf_config_from_payload(payload, components,
                                            coordinate_systems, reference_axes,
                                            tolerance=1e-5,
                                            subassemblies=None,
                                            compat=False):
    """Reconstruct SW2URDF mapping from parsed SW2URDF payload data.

    Parameters
    ----------
    payload : dict
        Parsed payload returned by ``parse_sw2urdf_payload``.
    components : sequence
        Top-level components with ``name``.
    coordinate_systems : sequence
        Top-level coordinate systems with ``name`` and
        ``document_from_frame``.
    reference_axes : sequence
        Top-level reference axes with ``name``, ``document_point``,
        ``document_direction``.
    tolerance : float, optional
        Distance tolerance for duplicate-origin sanity checks.
    subassemblies : mapping, optional
        ``{part_path: SubGraph}`` from ``GraphState.subassemblies``.  The
        add-in may reference a CoordSys that lives INSIDE a link
        sub-assembly, in which case the payload scopes its name as
        ``"<name> <component-N>"``; those are resolved through the owning
        component's sub-assembly and composed into the document frame.
    compat : bool, optional
        Reproduce the add-in's own output rather than sw2robot's conventions:
        keep the authored axis SIGN (no canonical normalisation, so limits
        need no mirroring) and carry each link's payload ``inertial`` through
        for the caller to apply.

    Returns
    -------
    dict | None
        Mapping with keys ``links``, ``root_link``, ``joints`` when
        reconstruction succeeds, else None.
    """
    if not isinstance(payload, dict):
        _warn("payload step 1 failed: parsed payload is not a dict")
        return None

    root_payload = payload.get("root")
    link_payloads = payload.get("links")
    if not isinstance(root_payload, dict) or not isinstance(link_payloads, list):
        _warn("payload step 1 failed: payload must contain dict root and list links")
        return None

    component_by_name = {
        str(_field(component, "name")): component
        for component in (components or [])
    }
    coord_by_name = {
        str(_field(coord, "name")): coord
        for coord in (coordinate_systems or [])
    }
    axis_by_name = {
        str(_field(axis, "name")): axis
        for axis in (reference_axes or [])
    }

    def resolve_component(payload_link_name, pid_values, main_pid=None):
        """(main_component, member_components) for one payload link.

        SW2URDF allows selecting SEVERAL loose components for one link; the
        MAIN one (SWMainComponentPID, mirrored as the first PID entry)
        anchors the joint tree and the rest ride along as rigid members.
        """
        candidates = []
        for pid_value in pid_values or []:
            top = _payload_component_name(pid_value)
            if top and top not in candidates:
                candidates.append(top)
        matches = [name for name in candidates if name in component_by_name]
        if not matches:
            _warn("payload step 1 failed: link "
                  f"{payload_link_name!r} resolved to zero top-level components "
                  f"from PIDs {candidates!r}")
            return None, []
        main = None
        if main_pid:
            main_name = _payload_component_name(main_pid)
            if main_name in matches:
                main = main_name
        if main is None:
            main = matches[0]
            if len(matches) > 1:
                _warn("payload step 1 warning: link "
                      f"{payload_link_name!r} has {len(matches)} member "
                      "components and no resolvable main; using first PID "
                      f"entry {main!r}")
        members = [name for name in matches if name != main]
        return main, members

    def resolve_coordsys_matrix(coordsys_name):
        """Document-frame 4x4 for a payload CoordSys reference, or None.

        Tries, in order: exact top-level name; a scoped
        ``"<name> <component-N>"`` reference resolved through that
        component's sub-assembly coordinate systems (composed with the
        component's world transform); the bare name at top level.
        """
        coord = coord_by_name.get(coordsys_name)
        if coord is not None:
            return _coord_matrix(coord)
        m = _SCOPED_COORDSYS_RE.match(coordsys_name)
        if m:
            base = m.group(1)
            component = component_by_name.get(m.group(2))
            if component is not None:
                part_path = _field(component, "part_path")
                sub = (subassemblies or {}).get(part_path)
                for cs in _field(sub, "coordinate_systems", None) or []:
                    if str(_field(cs, "name")) == base:
                        world = np.asarray(
                            _field(component, "world"), float).reshape(4, 4)
                        return world @ _coord_matrix(cs)
            coord = coord_by_name.get(base)
            if coord is not None:
                return _coord_matrix(coord)
        return None

    def origin_from_coordsys(component_name, coordsys_name, axis_point=None,
                             context="link"):
        if coordsys_name:
            mat = resolve_coordsys_matrix(coordsys_name)
            if mat is not None:
                return mat, coordsys_name
            _warn("payload step 2 warning: CoordSys "
                  f"{coordsys_name!r} for {context} is missing; "
                  "falling back to axis point")
        if axis_point is not None:
            mat = np.eye(4)
            mat[:3, 3] = np.asarray(axis_point, float)
            return mat, None
        _warn("payload step 2 warning: "
              f"{context} has no CoordSys and no axis point; "
              "falling back to component translation")
        mat = np.eye(4)
        component = component_by_name.get(component_name)
        if component is not None:
            mat[:3, 3] = _component_translation(component)
        return mat, None

    root_link_name = str(root_payload.get("link_name") or "").strip()
    if not root_link_name:
        _warn("payload step 1 failed: root link_name is empty")
        return None

    root_component, root_members = resolve_component(
        root_link_name,
        root_payload.get("components") or [],
        root_payload.get("main_component"))
    if root_component is None:
        return None

    link_to_component = {root_link_name: root_component}
    members_by_component = {root_component: root_members}
    frame_links = []
    ordered_links = []
    for rec in link_payloads:
        if not isinstance(rec, dict):
            _warn("payload step 1 failed: non-dict entry in payload links list")
            return None
        child_link_name = str(rec.get("link_name") or "").strip()
        parent_link_name = str(rec.get("parent_link_name") or "").strip()
        if not child_link_name or not parent_link_name:
            _warn("payload step 1 failed: payload link entry is missing "
                  "link_name or parent_link_name")
            return None
        if not (rec.get("components") or []):
            # SW2URDF lets a link carry NO component: a pure coordinate frame
            # (NejiNeji's `dummy_link` connector).  Keep it as a frame link
            # rather than failing the whole route.
            frame_links.append(rec)
            continue
        child_component, child_members = resolve_component(
            child_link_name,
            rec.get("components") or [],
            rec.get("main_component"))
        if child_component is None:
            return None
        members_by_component[child_component] = child_members
        if (child_link_name in link_to_component and
                link_to_component[child_link_name] != child_component):
            _warn("payload step 1 failed: payload link "
                  f"{child_link_name!r} maps to multiple components")
            return None
        link_to_component[child_link_name] = child_component
        ordered_links.append(rec)

    # The payload's links must PARTITION the components: a component shared
    # between links (gear-train / transmission-style configs encode couplings
    # this way) cannot become a URDF tree.
    usage = {}
    for link_name, comp in link_to_component.items():
        usage.setdefault(comp, set()).add(link_name)
    for comp, members in members_by_component.items():
        owner = next((ln for ln, c in link_to_component.items() if c == comp),
                     None)
        for member in members:
            usage.setdefault(member, set()).add(owner or comp)
    shared = {c: sorted(ls) for c, ls in usage.items() if len(ls) > 1}
    if shared:
        _warn("payload step 1 failed: components shared between links "
              f"(not a tree partition; transmission-style config?): {shared!r}")
        return None

    root_coordsys_name = str(root_payload.get("coordsys_name") or "").strip() or None
    root_origin, root_origin_name = origin_from_coordsys(
        root_component,
        root_coordsys_name,
        axis_point=None,
        context=f"root link {root_link_name!r}",
    )
    links = {
        root_component: {
            "link_name": root_link_name,
            "origin_name": root_origin_name,
            "origin": root_origin,
            "extra_components": list(root_members),
            "inertial": root_payload.get("inertial") if compat else None,
        }
    }

    joints = []
    for rec in ordered_links:
        child_link_name = str(rec.get("link_name") or "").strip()
        parent_link_name = str(rec.get("parent_link_name") or "").strip()
        child_component = link_to_component.get(child_link_name)
        parent_component = link_to_component.get(parent_link_name)
        if child_component is None or parent_component is None:
            _warn("payload step 3 failed: could not resolve parent/child "
                  f"components for {parent_link_name!r}->{child_link_name!r}")
            return None
        if child_component == parent_component:
            _warn("payload step 3 failed: parent/child collapse to same "
                  f"component {child_component!r}")
            return None
        if child_component == root_component:
            _warn("payload step 3 failed: root component cannot be a non-root child")
            return None

        joint = rec.get("joint") or {}
        joint_name = str(joint.get("name") or "").strip()
        if not joint_name:
            _warn("payload step 3 warning: missing joint name for "
                  f"{parent_link_name!r}->{child_link_name!r}; using fallback")
            joint_name = f"{parent_link_name}__{child_link_name}"

        joint_type = str(joint.get("type") or "revolute").strip().lower()
        if joint_type not in ("revolute", "continuous", "prismatic", "fixed"):
            _warn("payload step 3 warning: unknown joint type "
                  f"{joint_type!r} for {joint_name!r}; using 'fixed'")
            joint_type = "fixed"

        axis_name = str(joint.get("axis_name") or "").strip() or None
        coordsys_name = str(joint.get("coordsys_name") or "").strip() or None

        axis_point = None
        axis_direction = None
        if axis_name:
            axis_feature = axis_by_name.get(axis_name)
            if axis_feature is not None:
                line = _axis_line(axis_feature)
                if line is None:
                    _warn("payload step 3 failed: reference axis "
                          f"{axis_name!r} has degenerate direction")
                    return None
                axis_point = line[0].copy()
                axis_direction = line[1].copy()
            else:
                _warn("payload step 3 warning: reference axis "
                      f"{axis_name!r} is missing; falling back to payload axis "
                      "(joint-frame values)")

        if axis_direction is None:
            axis_xyz = joint.get("axis_xyz")
            if axis_xyz is not None:
                try:
                    axis_direction = _unit(axis_xyz)
                except Exception:
                    axis_direction = None
            if axis_direction is None:
                if joint_type in ("revolute", "continuous", "prismatic"):
                    _warn("payload step 3 failed: movable joint "
                          f"{joint_name!r} has no usable axis data")
                    return None

        origin, origin_name = origin_from_coordsys(
            child_component,
            coordsys_name,
            axis_point=axis_point,
            context=f"link {child_link_name!r}",
        )

        if child_component in links:
            prev = links[child_component]
            if prev["link_name"] != child_link_name:
                _warn("payload step 2 failed: component "
                      f"{child_component!r} is assigned to multiple payload links")
                return None
            prev_origin = np.asarray(prev["origin"][:3, 3], float)
            cur_origin = np.asarray(origin[:3, 3], float)
            if float(np.linalg.norm(prev_origin - cur_origin)) > tolerance:
                _warn("payload step 2 failed: conflicting origins for component "
                      f"{child_component!r}")
                return None
        else:
            links[child_component] = {
                "link_name": child_link_name,
                "origin_name": origin_name,
                "origin": origin,
                "extra_components": list(
                    members_by_component.get(child_component) or []),
                "inertial": rec.get("inertial") if compat else None,
            }

        if axis_direction is not None and axis_point is None:
            axis_point = np.asarray(origin[:3, 3], float)

        joint_rec = {
            "name": joint_name,
            "type": joint_type,
            "parent": parent_component,
            "child": child_component,
        }
        if axis_direction is not None and axis_point is not None:
            if compat:
                axis_direction, flipped = _orient_axis_to_payload(
                    axis_direction, joint.get("axis_xyz"), origin, joint_name)
            else:
                axis_direction, flipped = _normalize_axis_direction(axis_direction)
            child_origin = np.asarray(origin[:3, 3], float)
            joint_rec.update({
                "axis_point": np.asarray(axis_point, float).copy(),
                "axis_direction": axis_direction,
                "axis_origin_distance": _point_line_distance(
                    child_origin, axis_point, axis_direction),
                "flipped": bool(flipped),
            })

        limit = _payload_number_dict(
            joint.get("limit"), ("lower", "upper", "effort", "velocity"))
        if limit:
            for key in ("lower", "upper", "effort", "velocity"):
                if key in limit:
                    joint_rec[key] = limit[key]

        dynamics = _payload_number_dict(joint.get("dynamics"),
                                        ("damping", "friction"))
        mimic = _payload_number_dict(joint.get("mimic"),
                                     ("multiplier", "offset"),
                                     text_keys=("joint",))
        safety = _payload_number_dict(
            joint.get("safety"),
            ("soft_lower_limit", "soft_upper_limit",
             "k_position", "k_velocity"),
        )
        calibration = _payload_number_dict(joint.get("calibration"),
                                           ("rising", "falling"))
        if dynamics:
            joint_rec["dynamics"] = dynamics
        if mimic:
            joint_rec["mimic"] = mimic
        if safety:
            joint_rec["safety"] = safety
        if calibration:
            joint_rec["calibration"] = calibration

        joints.append(joint_rec)

    # Axis sign normalization NEGATES the joint coordinate, so every
    # payload quantity expressed in that coordinate must be mirrored
    # (the old add-in kept the raw sign and never needed this).
    flipped_by_name = {j["name"]: bool(j.get("flipped")) for j in joints}
    for j in joints:
        s_self = -1.0 if j.get("flipped") else 1.0
        if s_self < 0:
            lo = j.get("lower")
            up = j.get("upper")
            if lo is not None or up is not None:
                # +0.0 snaps IEEE negative zero so URDF text never shows -0
                j["lower"] = (-up + 0.0) if up is not None else None
                j["upper"] = (-lo + 0.0) if lo is not None else None
            safety = j.get("safety")
            if isinstance(safety, dict):
                sl = safety.pop("soft_lower_limit", None)
                su = safety.pop("soft_upper_limit", None)
                if su is not None:
                    safety["soft_lower_limit"] = -su
                if sl is not None:
                    safety["soft_upper_limit"] = -sl
            calibration = j.get("calibration")
            if isinstance(calibration, dict):
                rising = calibration.pop("rising", None)
                falling = calibration.pop("falling", None)
                if falling is not None:
                    calibration["rising"] = -falling
                if rising is not None:
                    calibration["falling"] = -rising
        mimic = j.get("mimic")
        if isinstance(mimic, dict) and mimic.get("joint"):
            s_driver = -1.0 if flipped_by_name.get(mimic["joint"]) else 1.0
            if s_self * s_driver < 0:
                mimic["multiplier"] = -float(mimic.get("multiplier", 1.0))
            if s_self < 0 and mimic.get("offset") is not None:
                mimic["offset"] = -float(mimic["offset"])

    if len(joints) != len(links) - 1:
        _warn("payload step 4 failed: joint count mismatch, got "
              f"{len(joints)} for {len(links)} links")
        return None

    parent_of = {}
    for joint in joints:
        child = joint["child"]
        if child in parent_of:
            _warn("payload step 4 failed: duplicate child in payload joints: "
                  f"{child!r}")
            return None
        parent_of[child] = joint["parent"]

    children_of = {}
    for child, parent in parent_of.items():
        children_of.setdefault(parent, []).append(child)

    seen = {root_component}
    queue = [root_component]
    while queue:
        cur = queue.pop()
        for child in children_of.get(cur, []):
            if child in seen:
                continue
            seen.add(child)
            queue.append(child)

    if seen != set(links):
        _warn("payload step 4 failed: some payload links are not reachable "
              f"from root {root_component!r}: {sorted(set(links) - seen)!r}")
        return None

    frame_out = []
    frame_names = {str(r.get("link_name") or "").strip() for r in frame_links}
    for rec in frame_links:
        name = str(rec.get("link_name") or "").strip()
        parent_name = str(rec.get("parent_link_name") or "").strip()
        if any(str(other.get("parent_link_name") or "").strip() == name
               for other in link_payloads):
            _warn("payload step 3 failed: frame link "
                  f"{name!r} has children; cannot express as a port")
            return None
        parent_component = link_to_component.get(parent_name)
        if parent_component is None:
            _warn("payload step 3 failed: frame link "
                  f"{name!r} hangs off unknown parent {parent_name!r}")
            return None
        joint = rec.get("joint") or {}
        coordsys_name = str(joint.get("coordsys_name") or "").strip() or None
        origin, _origin_name = origin_from_coordsys(
            parent_component, coordsys_name, axis_point=None,
            context=f"frame link {name!r}")
        frame_out.append({
            "link_name": name,
            "parent_component": parent_component,
            "origin": origin,
            "joint_name": str(joint.get("name") or "").strip(),
        })
    if frame_out:
        print(f"      SW2URDF config: {len(frame_out)} component-less frame "
              "link(s) emitted as ports: "
              + ", ".join(sorted(frame_names)))

    return {
        "links": links,
        "root_link": root_component,
        "joints": joints,
        "frame_links": frame_out,
    }
