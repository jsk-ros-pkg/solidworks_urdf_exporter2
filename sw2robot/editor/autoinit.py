"""CAD-derived initial values: self-collision joint-limit sweep + proximity.

The limit sweep is the user's idea, generalised from ``collision_limits.py``:
contacts present at the REST pose (parts touching by design) and parent/child
adjacency are the allowed baseline; each joint is then rotated until a NEW
colliding pair appears -- that angle (minus a small margin) is the limit.  A
joint that never collides within ``max_deg`` is suggested as ``continuous``.

For interactive use the collision geometry is each link's convex hull, so a full
query is ~1-2 ms (the raw CAD mesh is ~200 ms); the baseline is computed with the
same hulls, so hull-induced static overlaps cancel out.

This module needs skrobot (FK) + trimesh + python-fcl (the ``[ui]`` extra).
It is UI-independent: ``sw2robot.editor.ui`` calls it, and so can a headless
script.
"""

from __future__ import annotations

import numpy as np

REST_MARGIN = 0.002      # m of penetration before a pair counts as colliding


def _hull(mesh):
    """Convex-hull proxy of a link mesh (fast, watertight for fcl)."""
    try:
        h = mesh.convex_hull
        if h is not None and len(h.faces):
            return h
    except Exception:
        pass
    return mesh


def _rotational_joints(robot):
    return [j for j in robot.joint_list
            if type(j).__name__ == "RotationalJoint"]


def _child_map(robot):
    children = {}
    for j in robot.joint_list:
        if j.parent_link and j.child_link:
            children.setdefault(j.parent_link.name, []).append(j.child_link.name)
    return children


def _descendants(root, children):
    out, stack = set(), [root]
    while stack:
        n = stack.pop()
        if n in out:
            continue
        out.add(n)
        stack.extend(children.get(n, []))
    return out


class SelfCollision:
    """Convex-hull self-collision model over a skrobot robot.

    ``meshes`` maps link name -> local-frame trimesh (the UI already has these).
    """

    def __init__(self, robot, meshes, margin=REST_MARGIN, hull=True):
        from trimesh.collision import CollisionManager

        self.robot = robot
        self.margin = float(margin)
        self._link = {l.name: l for l in robot.link_list}
        self.names = [n for n in meshes if n in self._link]
        # hull=True: convex-hull proxies (fast, ~2 ms/query) for the live drag
        # check.  hull=False: the ACTUAL meshes -- slower but exact, for the
        # limit sweep (hulls touch at rest and mask collisions that only the
        # real concave geometry develops, giving limits that are too wide).
        self._hulls = {n: (_hull(meshes[n]) if hull else meshes[n])
                       for n in self.names}
        self.cm = CollisionManager()
        for n in self.names:
            self.cm.add_object(n, self._hulls[n],
                               transform=self._link[n].worldcoords().T())
        self.baseline = self._pairs(0.0)
        for j in robot.joint_list:
            if j.parent_link and j.child_link:
                self.baseline.add(
                    frozenset((j.parent_link.name, j.child_link.name)))

    def _pairs(self, margin) -> set:
        _, names, data = self.cm.in_collision_internal(
            return_names=True, return_data=True)
        if margin <= 0:
            return {frozenset(p) for p in names}
        depth = {}
        for d in data:
            k = frozenset(d.names)
            depth[k] = max(depth.get(k, 0.0), abs(d.depth))
        return {k for k, v in depth.items() if v > margin}

    def _sync(self):
        for n in self.names:
            self.cm.set_transform(n, self._link[n].worldcoords().T())

    def new_pairs(self) -> set:
        """Colliding link pairs (beyond the rest baseline) at the current pose."""
        self._sync()
        return self._pairs(self.margin) - self.baseline

    def min_distance(self):
        """``(distance_m, (link_a, link_b))`` of the closest non-adjacent pair,
        or ``(inf, None)`` if nothing is near.  Negative distance = penetration.

        fcl's ``min_distance_internal`` ignores objects already in contact, so a
        zero/negative reading is reported via ``new_pairs`` instead."""
        self._sync()
        try:
            d, names = self.cm.min_distance_internal(return_names=True)
        except Exception:
            return float("inf"), None
        return float(d), (tuple(sorted(names)) if names else None)


def sweep_limits(robot, meshes, step_deg=6.0, max_deg=180.0, margin_deg=2.0,
                 margin_m=REST_MARGIN, only=None, progress=None, refine=True,
                 refine_tol_deg=0.4, sc=None, hull=False):
    """Per-joint self-collision limit sweep, from the HOME pose (all angles 0).

    Returns ``{joint_name: {lower, upper, continuous, hit_lower, hit_upper,
    child}}`` with angles in radians.  ``hit_*`` is the colliding link pair
    that stopped that direction (or ``None`` if the sweep reached ``max_deg``
    freely).  A joint free in both directions is flagged ``continuous`` (and
    given the full +-max range as a nominal limit).

    The sweep is a COARSE linear scan (``step_deg``) to bracket the first new
    self-collision, then -- when ``refine`` -- a BISECTION between the last
    clear step and the first colliding step to pin the boundary to
    ``refine_tol_deg`` (the user's idea: a binary search beats fine stepping,
    so the coarse step can be large and the result is still precise).  The
    reported limit is the last clear angle minus a small ``margin_deg``.

    ``only`` (a set/list of joint names) restricts the sweep to those joints --
    used by the UI's per-joint "auto-fit" button.  Contacts and adjacency form
    the baseline at the home pose, and every other joint stays at 0 while one
    joint sweeps (so each limit is measured against the rest of the robot at
    rest).  Pass an already-built ``sc`` (SelfCollision) to skip rebuilding it.
    The robot's pre-call joint angles are restored before returning.
    """
    rjoints = _rotational_joints(robot)
    if only is not None:
        only = set(only)
        rjoints = [j for j in rjoints if j.name in only]

    # snapshot to restore at the end; widen limits so the sweep is not clamped
    snapshot = {j.name: float(j.joint_angle()) for j in robot.joint_list}
    saved_lims = {}
    for j in _rotational_joints(robot):
        saved_lims[j.name] = (j.min_angle, j.max_angle)
        j.min_angle, j.max_angle = -4 * np.pi, 4 * np.pi
    # baseline (rest contacts + adjacency) is defined at the HOME pose
    for j in _rotational_joints(robot):
        j.joint_angle(0.0)
    if sc is None:
        # hull=False by default here: the limit sweep needs the EXACT meshes
        # (convex hulls touch at rest and mask real collisions -> limits too
        # wide).  Callers that already built a hull `sc` (the live UI) pass it.
        sc = SelfCollision(robot, meshes, margin=margin_m, hull=hull)

    step = np.radians(step_deg)
    nmax = max(1, int(np.radians(max_deg) / step))
    margin = np.radians(margin_deg)
    tol = np.radians(refine_tol_deg)

    import fcl

    # Reuse the fcl objects SelfCollision already built (their BVH is built
    # ONCE here).  Per joint we register the prebuilt objects into throwaway
    # broadphase managers -- registerObjects does NOT rebuild the BVH, so a
    # query is ~1 ms even on the real (non-hull) meshes; rebuilding a manager
    # via trimesh.add_object instead cost ~2 s/joint (the whole sweep was 55 s).
    objs = {n: sc.cm._objs[n]["obj"] for n in sc.names}
    geom2name = {id(sc.cm._objs[n]["geom"]): n for n in sc.names}
    _req = fcl.CollisionRequest(num_max_contacts=1000, enable_contact=True)

    def _set_obj_T(n):
        T = sc._link[n].worldcoords().T()
        objs[n].setTranslation(np.ascontiguousarray(T[:3, 3]))
        objs[n].setRotation(np.ascontiguousarray(T[:3, :3]))

    def _moving_set(J):
        # Which links ACTUALLY move when only J rotates?  Determined by probing
        # (the skrobot parent/child tree-walk under-reports the subtree for
        # this model -- it returned just the immediate child while 18 links
        # really move).  The subtree is rigid, so one probe reveals all of it;
        # the full transform comparison catches on-axis rotation too.
        home = {n: sc._link[n].worldcoords().T() for n in sc.names}
        J.joint_angle(np.radians(7.0))
        moved = {n for n in sc.names
                 if not np.allclose(sc._link[n].worldcoords().T(),
                                    home[n], atol=1e-7)}
        J.joint_angle(0.0)
        return moved

    out = {}
    try:
        for J in rjoints:
            moving = _moving_set(J)
            static = [n for n in sc.names if n not in moving]
            if not moving or not static:
                # nothing can collide; treat as free
                out[J.name] = {
                    "lower": round(-np.radians(max_deg), 5),
                    "upper": round(np.radians(max_deg), 5),
                    "continuous": max_deg >= 180.0,
                    "hit_lower": None, "hit_upper": None,
                    "child": J.child_link.name if J.child_link else None}
                if progress:
                    progress(J.name, out[J.name])
                continue
            # static links don't move while J sweeps -> set their transforms
            # once; only the moving set updates per angle.
            st_mgr = fcl.DynamicAABBTreeCollisionManager()
            st_mgr.registerObjects([objs[n] for n in static])
            for n in static:
                _set_obj_T(n)
            st_mgr.setup()
            mv_mgr = fcl.DynamicAABBTreeCollisionManager()
            mv_mgr.registerObjects([objs[n] for n in moving])
            mv_mgr.setup()

            def _pairs(mag, direction):
                J.joint_angle(direction * mag)
                for n in moving:
                    _set_obj_T(n)
                mv_mgr.update()          # refresh AABBs after the move
                cdata = fcl.CollisionData(request=_req)
                mv_mgr.collide(st_mgr, cdata, fcl.defaultCollisionCallback)
                depth = {}
                for c in cdata.result.contacts:
                    a = geom2name.get(id(c.o1))
                    b = geom2name.get(id(c.o2))
                    if a and b:
                        k = frozenset((a, b))
                        depth[k] = max(depth.get(k, 0.0),
                                       abs(c.penetration_depth))
                return {k for k, v in depth.items() if v > sc.margin}

            # baseline: moving-vs-static pairs already in contact at HOME
            ignore = _pairs(0.0, 1) | sc.baseline

            def _collides(mag, direction):
                new = _pairs(mag, direction) - ignore
                return (tuple(sorted(next(iter(new)))) if new else None)

            lim = {}
            hit = {}
            for direction in (+1, -1):
                clear_mag, hit_mag, hit_pair = 0.0, None, None
                for k in range(1, nmax + 1):
                    mag = k * step
                    pair = _collides(mag, direction)
                    if pair:
                        hit_mag, hit_pair = mag, pair
                        break
                    clear_mag = mag
                # bisect [clear_mag, hit_mag] to the precise boundary
                if refine and hit_mag is not None:
                    a, b = clear_mag, hit_mag
                    while b - a > tol:
                        m = 0.5 * (a + b)
                        if _collides(m, direction):
                            b = m
                        else:
                            a = m
                    clear_mag = a
                J.joint_angle(0.0)   # back to home before the other direction
                if hit_mag is None:
                    lim[direction] = direction * np.radians(max_deg)
                else:
                    lim[direction] = direction * max(0.0, clear_mag - margin)
                hit[direction] = hit_pair
            lo, up = sorted([lim[-1], lim[+1]])
            continuous = hit[+1] is None and hit[-1] is None and max_deg >= 180.0
            out[J.name] = {
                "lower": round(float(lo), 5),
                "upper": round(float(up), 5),
                "continuous": continuous,
                "hit_lower": hit[-1],
                "hit_upper": hit[+1],
                "child": J.child_link.name if J.child_link else None,
            }
            if progress:
                progress(J.name, out[J.name])
    finally:
        for j in rjoints:
            j.min_angle, j.max_angle = saved_lims[j.name]
        for j in robot.joint_list:
            try:
                j.joint_angle(snapshot[j.name])
            except Exception:
                pass
    return out
