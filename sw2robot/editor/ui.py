"""A thin viser View over ``sw2robot.editor.core`` -- the browser UI for
configuring a CAD-derived module, modeled on robot-compiler's config page.

This is deliberately a *thin* View (see [[headless-core-thin-view]]): every widget
callback just calls a ``sw2robot.editor.core`` function that mutates the
``RobotCompilerState``.  The same core drives the CLI and would drive a future
FastAPI/React frontend.  No GUI logic lives here beyond wiring.

    uv run python -m sw2robot.editor.ui [package_dir] [--config c.yaml] [--port 8080]

With no ``package_dir`` the browser opens on a "🔌 SolidWorks" connect panel:
type the path to a ``.sldasm``, click *Import from SolidWorks*, and the whole
``extract -> build -> edit`` pipeline runs in-app (no prior CLI step needed).
Passing an existing ``package_dir`` (with a cached ``graph.json``) still loads
that module directly, as before.

Runs on this machine as-is (viser + scikit-robot), no FastAPI/jax/React build.
"""

from __future__ import annotations

import argparse
import copy
import itertools
import math
import os
import threading
import time
from pathlib import Path

import numpy as np

from . import autoinit, core
from .state import RobotCompilerState

_PI = math.pi


def _rot_z_to(axis):
    """Rotation matrix whose +Z points along ``axis`` (for axis markers)."""
    z = np.asarray(axis, float)
    n = np.linalg.norm(z)
    if n < 1e-9:
        return np.eye(3)
    z = z / n
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(ref, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _mat_to_wxyz(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([w, x, y, z])


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def _quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def _rotation_arrow(color, radius=0.025, tube=0.0028, span_deg=290.0, n=36):
    """A curved arrow circling +Z (an arc tube + a cone head pointing along the
    +theta travel) -- a 'which way it spins' indicator placed around the axis."""
    import trimesh

    angs = np.radians(np.linspace(0.0, span_deg, n))
    pts = np.column_stack([radius * np.cos(angs), radius * np.sin(angs),
                           np.zeros_like(angs)])
    parts = [trimesh.creation.cylinder(radius=tube, segment=[a, b], sections=8)
             for a, b in itertools.pairwise(pts)]
    t = angs[-1]
    tangent = np.array([-np.sin(t), np.cos(t), 0.0])   # +theta direction at the tip
    cone = trimesh.creation.cone(radius=3.2 * tube, height=11.0 * tube, sections=12)
    M = np.eye(4)
    M[:3, :3] = _rot_z_to(tangent)
    cone.apply_transform(M)
    cone.apply_translation(pts[-1])
    parts.append(cone)
    arrow = trimesh.util.concatenate(parts)
    arrow.visual = trimesh.visual.ColorVisuals(arrow, face_colors=[*list(color), 255])
    return arrow


def _slider_range(j: dict) -> tuple[float, float]:
    lo, hi = j.get("lowerLimit", 0.0), j.get("upperLimit", 0.0)
    if j.get("type") == "continuous" or lo == hi:
        return (-_PI, _PI)
    return (lo, hi)


def _pose(link):
    """(position xyz, quaternion wxyz) of a link in world coords."""
    c = link.worldcoords()
    return tuple(c.worldpos()), tuple(c.quaternion)


class RenderHandles:
    """viser scene handles for the robot, keyed by link name."""

    def __init__(self):
        self.frames = {}        # link -> frame handle (carries the world pose)
        self.normal = {}        # link -> normal-colour mesh handle
        self.red = {}           # link -> red-tinted mesh handle (collision)
        self.sel = {}           # link -> yellow-tinted mesh handle (multi-select)
        self.meshes = {}        # link -> local trimesh (for collision checks)
        self._red_set = set()
        self._sel_set = set()

    def _apply(self):
        """Per link, show red (collision) > yellow (selected) > normal."""
        for name in self.normal:
            is_red = name in self._red_set
            is_sel = (not is_red) and name in self._sel_set
            self.red[name].visible = is_red
            self.sel[name].visible = is_sel
            self.normal[name].visible = not (is_red or is_sel)

    def set_red(self, link_names):
        """Tint ``link_names`` red (self-collision)."""
        self._red_set = set(link_names)
        self._apply()

    def set_selected(self, link_names):
        """Tint ``link_names`` yellow (multi-select)."""
        self._sel_set = set(link_names)
        self._apply()


def render_robot(server, robot) -> tuple:
    """Add each link's visual mesh (plus a hidden red copy) to the viser scene
    under a frame at the link's world pose.  Returns ``(refresh, handles)``;
    ``refresh()`` re-syncs the frames after an FK change."""
    import trimesh

    h = RenderHandles()
    for link in robot.link_list:
        mesh = autoinit.link_visual_mesh(link)
        if mesh is None:
            continue
        pos, wxyz = _pose(link)
        fr = server.scene.add_frame(f"/robot/{link.name}", show_axes=False,
                                    position=pos, wxyz=wxyz)
        normal = server.scene.add_mesh_trimesh(f"/robot/{link.name}/vis", mesh)
        red_mesh = mesh.copy()
        red_mesh.visual = trimesh.visual.ColorVisuals(
            red_mesh, face_colors=[225, 40, 40, 255])
        red = server.scene.add_mesh_trimesh(f"/robot/{link.name}/col", red_mesh)
        red.visible = False
        sel_mesh = mesh.copy()
        sel_mesh.visual = trimesh.visual.ColorVisuals(
            sel_mesh, face_colors=[240, 200, 40, 255])
        sel = server.scene.add_mesh_trimesh(f"/robot/{link.name}/sel", sel_mesh)
        sel.visible = False
        h.frames[link.name] = fr
        h.normal[link.name] = normal
        h.red[link.name] = red
        h.sel[link.name] = sel
        h.meshes[link.name] = mesh

    def refresh():
        for link in robot.link_list:
            fr = h.frames.get(link.name)
            if fr is not None:
                fr.position, fr.wxyz = _pose(link)

    return refresh, h


def build_gui(server, refresh, handles, watcher, robot,
              state: RobotCompilerState, export_dir: Path):
    """Attach the config GUI to a running viser ``server``.  ``refresh`` re-syncs
    the 3D scene after an FK preview change; ``watcher`` flags self-collisions."""
    sk_joints = {j.name: j for j in robot.joint_list}
    movable = state.movable_joints()
    orig_names = [j["name"] for j in movable]
    by_orig = {j["name"]: j for j in movable}

    # link tree (incl. fixed joints) for the 3D click -> controlling joint lookup
    link_parent = {jj["childLink"]: jj["parentLink"] for jj in state.joints}

    # group movable joints by the root's direct-child branch they belong to
    # (a "finger"/chain), so the 20-joint list is navigable.
    def _branch_root(jn):
        link = by_orig[jn]["childLink"]
        while link in link_parent and link_parent[link] != state.root_link:
            link = link_parent[link]
        return link

    groups = {}
    for n in orig_names:
        groups.setdefault(_branch_root(n), []).append(n)
    group_names = list(groups)

    sliders = {}            # orig joint name -> angle slider handle
    mimic_map = {}          # driver orig -> [(follower orig, mult, offset), ...]
    _suppress = {"on": False}   # guard against programmatic slider feedback

    # original rotation axes, so a flip can be applied to / removed from the live
    # skrobot model (the preview then visibly reverses, not just the export).
    orig_axis = {}
    for n in orig_names:
        jt = sk_joints.get(n)
        if jt is not None and hasattr(jt, "axis"):
            try:
                orig_axis[n] = np.asarray(jt.axis, float).copy()
            except Exception:
                pass

    def apply_live_flip(jn):
        jt = sk_joints.get(jn)
        if jt is None or jn not in orig_axis:
            return
        e = state.edits.get(jn)
        sign = -1.0 if (e and e.flip_axis) else 1.0
        try:
            jt.axis = (orig_axis[jn] * sign).tolist()
            jt.joint_angle(float(jt.joint_angle()))  # recompute child transform
        except Exception:
            pass

    # undo stack of edit snapshots (settings only -- not FK preview angles)
    undo_stack = []

    def record():
        undo_stack.append(copy.deepcopy(state.edits))
        if len(undo_stack) > 100:
            undo_stack.pop(0)

    server.gui.add_markdown(
        f"## {state.robot_name}\n"
        f"root: `{state.root_link}` &nbsp; | &nbsp; "
        f"{len(state.joints)} joints ({len(movable)} movable)")

    # --- self-collision highlight (baseline = rest contacts + adjacency) ---
    collide_on = server.gui.add_checkbox("highlight self-collisions", initial_value=True)
    collide_status = server.gui.add_markdown("")

    def update_collisions():
        if not collide_on.value:
            handles.set_red([])
            collide_status.content = "_(collision check off)_"
            return
        new, links = watcher.offenders()
        handles.set_red(links)
        if new:
            pairs = ", ".join(" ↔ ".join(sorted(p)) for p in sorted(map(tuple, new)))
            collide_status.content = f"🔴 **{len(new)} new collision(s)**: {pairs}"
        else:
            d, pair = watcher.min_distance()
            if pair is not None and d != float("inf"):
                collide_status.content = (
                    f"🟢 no new collisions &nbsp;·&nbsp; closest "
                    f"`{pair[0]} ↔ {pair[1]}` **{d * 1000:.1f} mm**")
            else:
                collide_status.content = "🟢 no new collisions"

    def update_scene():
        refresh()
        place_axis("sel")
        place_axis("mim")
        place_gizmo()
        update_collisions()

    def rebuild_mimic():
        mimic_map.clear()
        for follower, e in state.edits.items():
            if e.mimic_joint:
                mimic_map.setdefault(e.mimic_joint, []).append(
                    (follower, e.mimic_multiplier, e.mimic_offset))

    def set_angle(jn, val, seen=None):
        """Set joint ``jn`` and cascade to any joints that mimic it (so the
        FK preview shows the real coupling), updating their sliders too."""
        seen = seen if seen is not None else set()
        if jn in seen:
            return
        seen.add(jn)
        jt = sk_joints.get(jn)
        if jt is not None and hasattr(jt, "joint_angle"):
            jt.joint_angle(val)
        w = sliders.get(jn)
        if w is not None and abs(w.value - val) > 1e-9:
            w.value = val  # reflect coupled value (callback is suppressed)
        for follower, mult, off in mimic_map.get(jn, []):
            set_angle(follower, mult * val + off, seen)

    collide_on.on_update(lambda _: update_collisions())

    # link -> the movable joint that drives it (walk up the link tree)
    mchild = {by_orig[n]["childLink"]: n for n in orig_names}

    def controlling_joint(link):
        cur = link
        while cur is not None:
            if cur in mchild:
                return mchild[cur]
            cur = link_parent.get(cur)
        return None

    # Axis markers along the joint's rotation axis (clearly visible, FK-tracked).
    # "sel" = joint being edited: RGB triad + green rod; "mim" = its mimic
    # driver: cyan rod.  Repositioned each update from the joint's world axis.
    _AX = {"sel": ((60, 200, 90), True), "mim": ((60, 170, 240), False)}
    axis_frame, axis_rod = {}, {}
    for role, (col, show_axes) in _AX.items():
        fr = server.scene.add_frame(f"/axis_{role}", show_axes=show_axes,
                                    axes_length=0.05, axes_radius=0.0035,
                                    visible=False)
        # a curved arrow encircling the axis = the positive rotation direction
        arrow = _rotation_arrow(col)
        rh = server.scene.add_mesh_trimesh(f"/axis_{role}/spin", arrow)
        rh.visible = False
        axis_frame[role], axis_rod[role] = fr, rh

    marked = {"sel": None, "mim": None}   # role -> joint name currently marked

    # single rotation ring (about Z) -> drag it to drive the selected joint
    gizmo = server.scene.add_transform_controls(
        "/spin_gizmo", scale=0.07, active_axes=(False, False, True),
        disable_axes=True, disable_sliders=True, visible=False)
    gizmo_state = {"jn": None, "base": np.array([1.0, 0, 0, 0]), "axis": np.array([0, 0, 1.0])}
    _giz_suppress = {"on": False}

    def place_gizmo():
        jn = marked["sel"]
        jt = sk_joints.get(jn) if jn else None
        e = state.edits.get(jn) if jn else None
        if jt is None or not hasattr(jt, "world_axis") or (e and e.mimic_joint):
            gizmo.visible = False          # hidden for non-movable / mimic followers
            return
        axis = np.asarray(jt.world_axis, float)
        n = np.linalg.norm(axis)
        if n < 1e-9:
            gizmo.visible = False
            return
        axis = axis / n
        base_q = _mat_to_wxyz(_rot_z_to(axis))
        th = float(jt.joint_angle())
        rq = np.array([np.cos(th / 2), *(np.sin(th / 2) * axis)])
        gizmo_state.update(jn=jn, base=base_q, axis=axis)
        _giz_suppress["on"] = True
        try:
            gizmo.position = tuple(float(x) for x in np.asarray(jt.world_position, float))
            gizmo.wxyz = tuple(float(x) for x in _quat_mul(rq, base_q))
            gizmo.visible = True
        finally:
            _giz_suppress["on"] = False

    def on_gizmo(_):
        if _giz_suppress["on"]:
            return
        jn = gizmo_state["jn"]
        if not jn:
            return
        d = _quat_mul(np.asarray(gizmo.wxyz, float), _quat_conj(gizmo_state["base"]))
        th = 2.0 * np.arctan2(float(np.dot(d[1:], gizmo_state["axis"])), float(d[0]))
        w = sliders.get(jn)
        if w is not None:
            th = min(max(th, w.min), w.max)
        _suppress["on"] = True
        try:
            set_angle(jn, th)     # drives skrobot + slider (+ mimic cascade)
            update_scene()        # repositions markers/gizmo, recomputes collisions
        finally:
            _suppress["on"] = False

    gizmo.on_update(on_gizmo)

    def place_axis(role):
        jn = marked[role]
        jt = sk_joints.get(jn) if jn else None
        if jt is None or not hasattr(jt, "world_axis"):
            axis_frame[role].visible = axis_rod[role].visible = False
            return
        R = _rot_z_to(np.asarray(jt.world_axis, float))
        axis_frame[role].position = tuple(float(x) for x in np.asarray(jt.world_position, float))
        axis_frame[role].wxyz = tuple(float(x) for x in _mat_to_wxyz(R))
        axis_frame[role].visible = axis_rod[role].visible = True

    def set_highlight(role, jn):
        marked[role] = jn
        place_axis(role)

    tabs = server.gui.add_tab_group()
    joints_tab = tabs.add_tab("Joints")
    with joints_tab:
        server.gui.add_markdown("**Click a link in 3D** (or pick below) to edit it.")
        group_dd = server.gui.add_dropdown(
            "chain", ("(all)", *group_names))
        select_dd = server.gui.add_dropdown("joint", tuple(orig_names))
        autoinit_btn = server.gui.add_button(
            "🪄 Auto-init limits (collision sweep)")
        autoinit_status = server.gui.add_markdown("")
        undo_btn = server.gui.add_button("↩ Undo (Ctrl+Z)")
        with server.gui.add_folder("💾 session"):
            save_btn = server.gui.add_button("Save edits")
            load_btn = server.gui.add_button("Load edits")
            persist_status = server.gui.add_markdown("")
        multi_chk = server.gui.add_checkbox(
            "multi-select (click links to add)", initial_value=False)
        with server.gui.add_folder("🔗 bulk edit (selected joints)"):
            bulk_status = server.gui.add_markdown("_none selected_")
            bulk_mimic = server.gui.add_dropdown(
                "mimic driver", ("(none)", *orig_names))
            bulk_mult = server.gui.add_number("mimic mult", initial_value=1.0)
            bulk_moff = server.gui.add_number("mimic offset", initial_value=0.0)
            bulk_mimic_btn = server.gui.add_button("apply mimic → selected")
            bulk_lo = server.gui.add_number("lower", initial_value=0.0)
            bulk_hi = server.gui.add_number("upper", initial_value=0.0)
            bulk_lim_btn = server.gui.add_button("apply limits → selected")
            bulk_servo = server.gui.add_dropdown(
                "servo model", ("(custom)", *tuple(core.SERVO_PROFILES)))
            bulk_servo_btn = server.gui.add_button("apply servo profile → selected")
            bulk_clear_btn = server.gui.add_button("clear selection")
        edit_status = server.gui.add_markdown("")

    # test hook: exercise the GUI callbacks headlessly (setting .value fires
    # on_update in viser, so a test can drive the panel without a browser).
    dbg = {"select": select_dd, "group": group_dd, "groups": groups,
           "state": state, "status": edit_status,
           "sk_joints": sk_joints, "gizmo": gizmo, "giz_state": gizmo_state,
           "on_gizmo": on_gizmo}
    server._sw2robot_dbg = dbg

    panel = {"handles": []}

    def clear_panel():
        for h in panel["handles"]:
            try:
                h.remove()
            except Exception:
                pass
        panel["handles"].clear()
        sliders.clear()

    def select_joint(jn):
        if jn not in by_orig:
            return
        clear_panel()
        j = by_orig[jn]
        e = state.edits.get(jn)
        jt = sk_joints.get(jn)
        lo, hi = _slider_range(j)
        # the slider is bounded by the configured limits, not the raw CAD range
        if e and e.lower is not None and e.upper is not None and e.lower < e.upper:
            lo, hi = e.lower, e.upper
        cur = float(jt.joint_angle()) if jt is not None and hasattr(jt, "joint_angle") else 0.0
        cur = min(max(cur, lo), hi)
        with joints_tab:
            fold = server.gui.add_folder(f"⚙ {state.effective_name(jn)}")
            with fold:
                angle = server.gui.add_slider("angle (preview)", min=lo, max=hi,
                                              step=(hi - lo) / 200 if hi > lo else 0.01,
                                              initial_value=cur)
                angle.disabled = bool(e and e.mimic_joint)
                rename = server.gui.add_text("name", initial_value=state.effective_name(jn))
                lower = server.gui.add_number("lower",
                    initial_value=(e.lower if e and e.lower is not None else j.get("lowerLimit", 0.0)))
                upper = server.gui.add_number("upper",
                    initial_value=(e.upper if e and e.upper is not None else j.get("upperLimit", 0.0)))
                set_lo = server.gui.add_button("lower ← current angle")
                set_hi = server.gui.add_button("upper ← current angle")
                autofit = server.gui.add_button("🪄 auto-fit limits (sweep)")
                effort = server.gui.add_number("effort (N·m, 0=keep)", step=0.05,
                    initial_value=(e.effort if e and e.effort is not None else 0.0))
                velocity = server.gui.add_number("velocity (rad/s, 0=keep)", step=0.5,
                    initial_value=(e.velocity if e and e.velocity is not None else 0.0))
                servo_model = server.gui.add_dropdown(
                    "servo model", ("(custom)", *tuple(core.SERVO_PROFILES)),
                    initial_value=(e.servo_model if e and e.servo_model in core.SERVO_PROFILES else "(custom)"))
                servo_id = server.gui.add_number("servo_id (-1=none)", step=1,
                    initial_value=(e.servo_id if e and e.servo_id is not None else -1))
                direction = server.gui.add_dropdown("direction", ("+1", "-1"),
                    initial_value=("-1" if e and e.direction == -1 else "+1"))
                offset = server.gui.add_number("angle_offset",
                    initial_value=(e.angle_offset if e else 0.0))
                cands = [n for n in orig_names if n != jn]  # any movable joint
                mimic = server.gui.add_dropdown("mimic", ("(none)", *cands),
                    initial_value=(e.mimic_joint if e and e.mimic_joint in cands else "(none)"))
                mult = server.gui.add_number("mimic mult",
                    initial_value=(e.mimic_multiplier if e else 1.0))
                moff = server.gui.add_number("mimic offset",
                    initial_value=(e.mimic_offset if e else 0.0))
                flip = server.gui.add_checkbox("flip axis",
                    initial_value=bool(e and e.flip_axis))
                reverse = server.gui.add_button("⇅ reverse direction (flip axis + swap limits)")
                jtype = server.gui.add_dropdown("type", ("(keep)", *core.JOINT_TYPES),
                    initial_value=(e.jtype if e and e.jtype else "(keep)"))
        panel["handles"] = [angle, rename, lower, upper, set_lo, set_hi, autofit,
                            effort, velocity, servo_model,
                            servo_id, direction, offset, mimic, mult, moff,
                            flip, reverse, jtype, fold]
        sliders[jn] = angle
        dbg["jn"] = jn
        dbg["w"] = {"angle": angle, "rename": rename, "lower": lower, "upper": upper,
                        "set_lo": set_lo, "set_hi": set_hi, "servo_id": servo_id,
                        "direction": direction, "offset": offset, "mimic": mimic,
                        "mult": mult, "moff": moff, "flip": flip, "reverse": reverse, "jtype": jtype}

        def on_angle(_):
            if _suppress["on"]:
                return
            _suppress["on"] = True
            try:
                set_angle(jn, angle.value)   # cascades to mimic followers
                update_scene()
            finally:
                _suppress["on"] = False

        def on_rename(_):
            record()
            try:
                core.rename_joint(state, jn, rename.value.strip() or jn)
                edit_status.content = ""
            except Exception as ex:
                edit_status.content = f"⚠️ rename: {ex}"
                return
            try:
                fold.label = f"⚙ {state.effective_name(jn)}"
            except Exception:
                pass

        def on_limits(_):
            record()
            lo_v, up_v = float(lower.value), float(upper.value)
            core.set_limits(state, jn, lo_v, up_v)
            if lo_v < up_v:
                # constrain the FK-preview slider to the configured limits.
                # widen-then-narrow so min<=max holds at every assignment;
                # suppressed so the clamp doesn't re-fire on_angle.
                _suppress["on"] = True
                try:
                    angle.min = min(angle.min, lo_v)
                    angle.max = max(angle.max, up_v)
                    angle.min, angle.max = lo_v, up_v
                finally:
                    _suppress["on"] = False
                edit_status.content = ""
            else:
                edit_status.content = "⚠️ lower ≥ upper"

        def on_set_lower(_):
            lower.value = float(angle.value)   # fires on_limits -> core.set_limits

        def on_set_upper(_):
            upper.value = float(angle.value)

        def on_servo(_):
            record()
            try:
                sid = int(servo_id.value)
                if sid < 0:
                    state.edit_for(jn).servo_id = None
                else:
                    core.set_servo(state, jn, sid,
                                   direction=(1 if direction.value == "+1" else -1),
                                   angle_offset=float(offset.value))
                edit_status.content = ""
            except Exception as ex:
                edit_status.content = f"⚠️ servo: {ex}"

        def on_mimic(_):
            record()
            try:
                if mimic.value == "(none)":
                    core.clear_mimic(state, jn)
                    angle.disabled = False
                    set_highlight("mim", None)
                else:
                    core.set_mimic(state, jn, mimic.value,
                                   multiplier=float(mult.value), offset=float(moff.value))
                    angle.disabled = True
                    set_highlight("mim", mimic.value)
                edit_status.content = ""
            except Exception as ex:
                edit_status.content = f"⚠️ mimic: {ex}"
                return
            rebuild_mimic()
            e2 = state.edits.get(jn)
            if e2 and e2.mimic_joint and sk_joints.get(e2.mimic_joint) is not None:
                drv = float(sk_joints[e2.mimic_joint].joint_angle())
                _suppress["on"] = True
                try:
                    set_angle(jn, e2.mimic_multiplier * drv + e2.mimic_offset)
                    update_scene()
                finally:
                    _suppress["on"] = False

        def on_flip(_):
            if _suppress["on"]:
                return
            record()
            cur = float(angle.value)
            core.set_axis_flip(state, jn, bool(flip.value))
            apply_live_flip(jn)
            _suppress["on"] = True
            try:
                set_angle(jn, -cur)   # preserve pose; the part now swings the other way
            finally:
                _suppress["on"] = False
            update_scene()

        def on_reverse(_):
            record()
            cur = float(angle.value)
            core.reverse_direction(state, jn)       # flip axis + remap limits [-up,-lo]
            apply_live_flip(jn)
            jt2 = sk_joints.get(jn)
            if jt2 is not None and hasattr(jt2, "joint_angle"):
                jt2.joint_angle(-cur)               # preserve pose
            select_joint(jn)                        # rebuild: new limits/flip/range/angle
            update_scene()

        def on_jtype(_):
            record()
            if jtype.value == "(keep)":
                state.edit_for(jn).jtype = None
            else:
                try:
                    core.set_joint_type(state, jn, jtype.value)
                    edit_status.content = ""
                except Exception as ex:
                    edit_status.content = f"⚠️ type: {ex}"

        def on_actuator(_):
            record()
            core.set_actuator(
                state, jn,
                effort=(effort.value if effort.value > 0 else None),
                velocity=(velocity.value if velocity.value > 0 else None))

        def on_servo_model(_):
            record()
            model = None if servo_model.value == "(custom)" else servo_model.value
            core.apply_servo_profile(state, jn, model)
            select_joint(jn)   # rebuild: profile auto-filled effort/velocity/limits

        angle.on_update(on_angle)
        rename.on_update(on_rename)
        for w in (lower, upper):
            w.on_update(on_limits)
        for w in (effort, velocity):
            w.on_update(on_actuator)
        servo_model.on_update(on_servo_model)
        for w in (servo_id, direction, offset):
            w.on_update(on_servo)
        for w in (mimic, mult, moff):
            w.on_update(on_mimic)
        flip.on_update(on_flip)
        reverse.on_click(on_reverse)
        jtype.on_update(on_jtype)
        set_lo.on_click(on_set_lower)
        set_hi.on_click(on_set_upper)
        autofit.on_click(lambda _: apply_sweep(only={jn}, status=edit_status))
        dbg["h"] = {"reverse": on_reverse, "flip": on_flip,
                    "set_lo": on_set_lower, "set_hi": on_set_upper}

        # axis markers: the edited joint (green rod + triad) + its mimic driver
        set_highlight("sel", jn)
        set_highlight("mim", e.mimic_joint if (e and e.mimic_joint) else None)
        place_gizmo()           # the drag-to-rotate ring for this joint

    def undo(_=None):
        if not undo_stack:
            edit_status.content = "↩ nothing to undo"
            return
        state.edits = undo_stack.pop()
        for n in orig_names:          # re-apply axis flips to the live model
            apply_live_flip(n)
        rebuild_mimic()
        cur = dbg.get("jn") or (orig_names[0] if orig_names else None)
        if cur:
            select_joint(cur)         # rebuild panel from restored state
        update_scene()
        edit_status.content = "↩ undone"

    undo_btn.on_click(undo)
    dbg["undo"] = undo

    def on_save(_=None):
        try:
            p = core.save_state(state)
            persist_status.content = f"💾 saved → `{p.name}`"
        except Exception as ex:
            persist_status.content = f"⚠️ save: {ex}"

    def on_load(_=None):
        record()
        try:
            n = core.load_edits(state)
        except Exception as ex:
            persist_status.content = f"⚠️ load: {ex}"
            return
        if not n:
            persist_status.content = "📂 no saved edits found"
            return
        for nm in orig_names:        # re-apply axis flips to the live model
            apply_live_flip(nm)
        rebuild_mimic()
        cur = dbg.get("jn") or (orig_names[0] if orig_names else None)
        if cur:
            select_joint(cur)
        update_scene()
        persist_status.content = f"📂 loaded {n} edit(s)"

    save_btn.on_click(on_save)
    load_btn.on_click(on_load)
    dbg["save"] = on_save
    dbg["load"] = on_load

    def apply_sweep(only=None, status=None):
        """Run the self-collision sweep and bake the discovered limits / a
        ``continuous`` type into the state.  ``only`` restricts to one joint
        (the per-joint button); ``None`` does every revolute joint."""
        status = status or autoinit_status
        status.content = "⏳ sweeping… (this can take a few seconds)"
        try:
            results = autoinit.sweep_limits(robot, handles.meshes, only=only)
        except Exception as ex:
            status.content = f"⚠️ sweep failed: {ex}"
            return
        if not results:
            status.content = "⚠️ no revolute joints to sweep"
            return
        record()
        n_lim = n_cont = n_hit = 0
        for jn, v in results.items():
            try:
                if v["continuous"]:
                    core.set_joint_type(state, jn, "continuous")
                    n_cont += 1
                else:
                    core.set_limits(state, jn, v["lower"], v["upper"])
                    n_lim += 1
                    if v["hit_lower"] or v["hit_upper"]:
                        n_hit += 1
            except Exception:
                pass
        rebuild_mimic()
        cur = dbg.get("jn") or (orig_names[0] if orig_names else None)
        if cur:
            select_joint(cur)   # rebuild panel + slider bounds from new limits
        update_scene()
        status.content = (f"✅ {n_lim} joint(s) limited ({n_hit} stopped by a "
                          f"collision), {n_cont} continuous")

    autoinit_btn.on_click(lambda _: apply_sweep())
    dbg["apply_sweep"] = apply_sweep

    def on_group(_):
        names = orig_names if group_dd.value == "(all)" \
            else groups.get(group_dd.value, orig_names)
        select_dd.options = tuple(names)
        if names and select_dd.value not in names:
            select_dd.value = names[0]   # fires on_update -> select_joint

    group_dd.on_update(on_group)
    select_dd.on_update(lambda _: select_joint(select_dd.value))

    # ---- multi-select + bulk edit (e.g. coupled finger joints) ----
    multi_sel = []   # joint orig names, in selection order

    def _sel_links():
        return [by_orig[n]["childLink"] for n in multi_sel if n in by_orig]

    def update_multi():
        handles.set_selected(_sel_links() if multi_chk.value else [])
        if multi_sel:
            bulk_status.content = (f"**{len(multi_sel)} selected:** "
                + ", ".join(state.effective_name(n) for n in multi_sel))
        else:
            bulk_status.content = "_none selected (turn on multi-select, click links)_"

    def toggle_multi(jn):
        if jn in multi_sel:
            multi_sel.remove(jn)
        else:
            multi_sel.append(jn)
        update_multi()

    multi_chk.on_update(lambda _: update_multi())

    def bulk_apply(fn, label):
        if not multi_sel:
            bulk_status.content = "⚠️ nothing selected"
            return
        record()
        n = 0
        for jn in list(multi_sel):
            try:
                fn(jn)
                n += 1
            except Exception as ex:
                edit_status.content = f"⚠️ {state.effective_name(jn)}: {ex}"
        rebuild_mimic()
        cur = dbg.get("jn")
        if cur:
            select_joint(cur)
        update_scene()
        bulk_status.content = f"✅ {label} → {n}/{len(multi_sel)} joint(s)"

    def _bulk_mimic(jn):
        if bulk_mimic.value in ("(none)", jn):
            return
        core.set_mimic(state, jn, bulk_mimic.value,
                       multiplier=bulk_mult.value, offset=bulk_moff.value)

    bulk_mimic_btn.on_click(lambda _: bulk_apply(_bulk_mimic, "mimic"))
    bulk_lim_btn.on_click(lambda _: bulk_apply(
        lambda jn: core.set_limits(state, jn, bulk_lo.value, bulk_hi.value), "limits"))
    bulk_servo_btn.on_click(lambda _: bulk_apply(
        lambda jn: core.apply_servo_profile(
            state, jn,
            None if bulk_servo.value == "(custom)" else bulk_servo.value), "servo"))
    bulk_clear_btn.on_click(lambda _: (multi_sel.clear(), update_multi()))

    def make_click(link):
        def cb(_):
            jn = controlling_joint(link)
            if jn is None:
                return
            if multi_chk.value:          # accumulate into the bulk selection
                toggle_multi(jn)
                return
            if jn not in select_dd.options:   # clicked outside the filter
                group_dd.value = "(all)"      # -> on_group widens the list
            select_dd.value = jn   # fires on_update -> select_joint
        return cb

    for link in handles.normal:
        cb = make_click(link)
        handles.normal[link].on_click(cb)
        handles.red[link].on_click(cb)
        handles.sel[link].on_click(cb)

    # ---- Export tab ----
    with tabs.add_tab("Export"):
        out_name = server.gui.add_text("zip name", initial_value=f"{state.robot_name}_ros.zip")
        status = server.gui.add_markdown("")
        check_btn = server.gui.add_button("Validate")
        export_btn = server.gui.add_button("Export ROS package")

        def _problems_md(probs):
            return ("🟢 no problems" if not probs
                    else f"⚠️ **{len(probs)} problem(s)**:\n"
                         + "\n".join(f"- {p}" for p in probs))

        def on_check(_):
            status.content = _problems_md(core.validate(state))

        def on_export(_):
            out = export_dir / out_name.value
            probs = core.validate(state)
            try:
                core.export_ros_package(state, out)
                msg = f"✅ exported → `{out}`"
                if probs:
                    msg += "\n\n" + _problems_md(probs)
                status.content = msg
            except Exception as e:  # surface failures in the UI
                status.content = f"❌ {type(e).__name__}: {e}"

        check_btn.on_click(on_check)
        export_btn.on_click(on_export)

    rebuild_mimic()
    if orig_names:
        select_joint(orig_names[0])   # show one joint's panel on load
    update_collisions()  # initial state (baseline -> should be green)


def mount_module(server, state: RobotCompilerState, export_dir: Path):
    """Build the 3D scene + config GUI for one already-loaded module onto a live
    ``server``.  Split out of :func:`launch` so it can be re-run after a fresh
    SolidWorks import (the caller clears the scene/GUI first)."""
    from skrobot.models.urdf import RobotModelFromURDF

    robot = RobotModelFromURDF(urdf_file=state.urdf_path)
    refresh, handles = render_robot(server, robot)
    watcher = autoinit.SelfCollision(robot, handles.meshes, confirm=True)
    build_gui(server, refresh, handles, watcher, robot, state, export_dir)
    print(f"[sw2robot.ui] mounted '{state.robot_name}': {len(state.joints)} joints "
          f"({len(state.movable_joints())} movable), root={state.root_link}")


def _pick_sldasm(initial=""):
    """Pop the OS native file-open dialog (tkinter) and return the chosen path.

    viser runs in the browser and a browser file picker only yields file
    *contents*, never a server-side path -- but SolidWorks must open a path on
    THIS machine.  For a local single-user tool the server IS the user's
    machine, so a native dialog is both a real "file manager" and gives a usable
    path.  Returns "" on cancel / any failure."""
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:
        return ""
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        kw = {}
        if initial:
            d = os.path.dirname(initial)
            if os.path.isdir(d):
                kw["initialdir"] = d
            if os.path.isfile(initial):
                kw["initialfile"] = os.path.basename(initial)
        path = filedialog.askopenfilename(
            title="Select a SolidWorks assembly",
            filetypes=[("SolidWorks assembly", "*.sldasm"),
                       ("All files", "*.*")], **kw)
        root.destroy()
        return path or ""
    except Exception:
        return ""


def build_connect_panel(server, defaults: dict, on_import):
    """The "🔌 SolidWorks" panel: pick a ``.sldasm`` and import it live.

    The multi-minute ``core.extract_and_import`` runs on a background thread so
    the viser event loop stays responsive; a per-stage ``progress`` callback plus
    a 1 Hz elapsed-time ticker stream "what it's doing + for how long" into a
    markdown widget.  On success ``on_import(state)`` rebuilds the UI.  Thin View:
    all real work is in ``sw2robot.editor.core`` (see
    [[headless-core-thin-view]])."""
    with server.gui.add_folder("🔌 SolidWorks"):
        session = server.gui.add_markdown("_checking SolidWorks session …_")
        use_active = server.gui.add_button("🎯 Use the assembly open in "
                                           "SolidWorks")
        use_active.visible = False
        recheck = server.gui.add_button("🔄 Re-check session")
        asm = server.gui.add_text("assembly (.sldasm)",
                                  initial_value=defaults.get("assembly", ""))
        browse = server.gui.add_button("📁 Browse…")
        name = server.gui.add_text("robot name (optional)",
                                   initial_value=defaults.get("name", ""))
        out = server.gui.add_text("output dir",
                                  initial_value=defaults.get("out_dir", ""))
        base = server.gui.add_text("base hint (optional)",
                                   initial_value=defaults.get("base", ""))
        visible = server.gui.add_checkbox("show SolidWorks window",
                                          initial_value=False)
        btn = server.gui.add_button("Import from SolidWorks")
        status = server.gui.add_markdown(
            "_Pick your assembly (📁 Browse…) or paste a `.sldasm` path._  "
            "The original file is never modified.")

    # ---- session awareness: detect the user's running SolidWorks and tell
    # them what to do in every state (never blocks the import path) ----------
    sess = {"active": None}

    def _session_text(st):
        if not st["running"]:
            return ("⚪ **SolidWorks is not running.**  Import will start a "
                    "hidden instance automatically — nothing to do on your "
                    "side.")
        if not st["attachable"]:
            return ("🟡 **SolidWorks is running** ({n} instance{s}) but this "
                    "server can't see inside it (it was started from a "
                    "different login session).  Importing still works: it "
                    "opens a throwaway COPY of the **saved file on disk** in "
                    "a separate hidden instance.\n\n"
                    "👉 If you are editing the assembly right now, press "
                    "**Ctrl+S in SolidWorks first**, then import.  (This "
                    "tool never saves or modifies your files.)\n"
                    "👉 To enable live detection of the open assembly, start "
                    "this UI from your own terminal."
                    ).format(n=st["instances"],
                             s="s" if st["instances"] > 1 else "")
        if st["active_assembly"]:
            warn = ""
            if st["dirty"]:
                warn = ("\n\n⚠️ **It has unsaved changes** — the import reads "
                        "the saved file on disk, so press **Ctrl+S in "
                        "SolidWorks first**.  (This tool never saves for "
                        "you.)")
            return (f"🟢 **SolidWorks is running.**  Active assembly:\n"
                    f"`{st['active_assembly']}`{warn}")
        return ("🟢 **SolidWorks is running**, but the active document is "
                "not an assembly ("
                + (f"`{st['active_doc']}`" if st["active_doc"] else "none")
                + ").  Open the `.sldasm` you want in SolidWorks, or pick "
                  "the file below.")

    def _refresh_session():
        session.content = "_checking SolidWorks session …_"
        st = core.sw_session_status()
        sess["active"] = st.get("active_assembly")
        session.content = _session_text(st)
        use_active.visible = bool(sess["active"])

    def on_use_active(_):
        if sess["active"]:
            asm.value = sess["active"]
            status.content = ("🎯 path taken from the open SolidWorks "
                              "assembly — press **Import from SolidWorks**.")

    def on_recheck(_):
        threading.Thread(target=_refresh_session, daemon=True).start()

    use_active.on_click(on_use_active)
    recheck.on_click(on_recheck)
    threading.Thread(target=_refresh_session, daemon=True).start()

    def on_browse(_):
        # the native dialog blocks until the user picks, so run it off-thread
        def run():
            p = _pick_sldasm(asm.value.strip())
            if p:
                asm.value = p
        threading.Thread(target=run, daemon=True).start()

    # ---- live status: stage text (progress cb) + elapsed seconds (ticker) ----
    cur = {"msg": "", "t0": 0.0, "running": False}

    def _ticker():
        while cur["running"]:
            secs = int(time.time() - cur["t0"])
            mm, ss = divmod(secs, 60)
            clock = f"{mm}:{ss:02d}" if mm else f"{ss}s"
            status.content = f"⏳ {cur['msg']}  ·  elapsed {clock}"
            time.sleep(1.0)

    def worker(path, nm, od, bh, vis):
        cur.update(msg="starting SolidWorks …", t0=time.time(), running=True)
        threading.Thread(target=_ticker, daemon=True).start()
        try:
            state = core.extract_and_import(
                path, out_dir=od or None, robot_name=nm or None,
                base_hint=bh or None, visible=vis,
                progress=lambda m: cur.__setitem__("msg", m))
            cur["running"] = False
            # remember the inputs so a re-import pre-fills them
            defaults.update(assembly=path, name=nm, out_dir=od, base=bh)
            status.content = f"✅ imported `{state.robot_name}` — building view …"
            on_import(state)   # rebuilds the whole GUI (this panel included)
        except Exception as e:  # surface SolidWorks/extract failures in the UI
            cur["running"] = False
            status.content = f"❌ {type(e).__name__}: {e}"
            btn.disabled = False

    def on_click(_):
        path = asm.value.strip()
        if not path:
            status.content = "⚠️ pick or enter the path to a `.sldasm` assembly first"
            return
        btn.disabled = True
        status.content = "⏳ starting SolidWorks …"
        threading.Thread(
            target=worker,
            args=(path, name.value.strip(), out.value.strip(),
                  base.value.strip(), visible.value),
            daemon=True).start()

    browse.on_click(on_browse)
    btn.on_click(on_click)


def launch(package_dir=None, config_path=None, base_hint=None, port=8080,
           export_dir=None, out_dir=None, block=True):
    """Serve the viser config UI.

    With ``package_dir`` the cached module is loaded immediately (legacy path);
    without it the UI opens on the SolidWorks connect panel so the user can
    extract + build + edit an assembly entirely in-app.

    Uses ``viser.ViserServer`` directly (no skrobot ViserViewer wrapper) so we
    own the port and the whole GUI; scikit-robot is used only for geometry + FK.
    """
    import viser

    server = viser.ViserServer(port=port)
    holder = {"state": None}
    defaults = {"assembly": "", "name": "", "base": base_hint or "",
                "out_dir": out_dir or str(Path.cwd() / "output")}
    recents = core.sw_recent_assemblies()
    if recents:
        defaults["assembly"] = recents[0]

    def _exp_dir(state):
        return Path(export_dir) if export_dir else Path(state.package_dir)

    def show(state, exp_dir):
        """(Re)build the entire UI: connect panel always, module UI if loaded."""
        server.scene.reset()
        server.gui.reset()
        holder["state"] = state
        build_connect_panel(server, defaults,
                            lambda st: show(st, _exp_dir(st)))
        if state is not None:
            mount_module(server, state, exp_dir)

    if package_dir:
        state = core.import_module(package_dir, config_path=config_path,
                                   base_hint=base_hint)
        show(state, Path(export_dir) if export_dir else Path(package_dir))
    else:
        show(None, None)

    print(f"[sw2robot.ui] serving -> http://localhost:{port}"
          + ("" if package_dir else "  (connect to SolidWorks in the panel)"))
    if not block:
        return server, holder
    try:
        import time
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return server, holder


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("package_dir", nargs="?", default=None,
                    help="existing sw2robot.exporter package dir "
                         "(with graph.json). "
                         "Omit to start on the SolidWorks connect panel.")
    ap.add_argument("--config", default=None)
    ap.add_argument("--base", default=None)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--export-dir", default=None)
    ap.add_argument("--out", default=None,
                    help="default output dir for SolidWorks imports "
                         "(default: ./output)")
    args = ap.parse_args(argv)
    launch(args.package_dir, config_path=args.config, base_hint=args.base,
           port=args.port, export_dir=args.export_dir, out_dir=args.out)


if __name__ == "__main__":
    main()
