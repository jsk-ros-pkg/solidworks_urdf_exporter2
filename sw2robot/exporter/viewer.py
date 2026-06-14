"""Load a generated URDF into scikit-robot's browser-based viser viewer.

    uv run python -m sw2robot.exporter.viewer <path/to/robot.urdf>

Adds:
  * link-name labels (so you can identify parts while dragging joints),
  * a draggable gizmo (transform control) at each movable joint's axis.

Drag a gizmo to where a joint axis SHOULD be; its live pose (position, quat,
and the gizmo Z direction = proposed rotation axis) is written to
``<cwd>/_gizmos.json`` so it can be read back as a hint.
"""

from __future__ import annotations

import argparse
import json
import os
import threading

import numpy as np

GIZMO_JSON = os.path.join(os.getcwd(), "_gizmos.json")


def _rot_from_z(z):
    z = np.asarray(z, float)
    n = np.linalg.norm(z)
    if n < 1e-9:
        return np.eye(3)
    z = z / n
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(ref, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _mat_to_wxyz(R):
    # rotation matrix -> quaternion (w, x, y, z)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
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


def _wxyz_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def view(urdf_path, open_browser=True, labels=True, gizmos=True,
         gizmo_scale=0.05, axis_viz=True, axis_length=0.05):
    import skrobot
    from skrobot.models.urdf import RobotModelFromURDF
    from skrobot.model import Axis, Cylinder
    from skrobot.coordinates import Coordinates

    print(f"loading URDF: {urdf_path}")
    robot = RobotModelFromURDF(urdf_file=urdf_path)
    print(f"loaded: {len(robot.link_list)} links")

    viewer = skrobot.viewers.ViserViewer()
    viewer.add(robot)
    server = viewer._server

    # root (base_link) coordinate system -- a larger triad at the URDF origin
    root = getattr(robot, "root_link", None) or robot.link_list[0]
    root_ax = Axis.from_coords(Coordinates(pos=root.worldcoords().worldpos(),
                                           rot=root.worldcoords().rotation),
                               axis_length=axis_length * 2.0,
                               axis_radius=axis_length * 0.12)
    viewer.add(root_ax)
    print(f"root/base_link = {root.name} (coordinate triad at origin)")

    if axis_viz:
        # rotation AXIS as a Cylinder (rod along the axis), rotation CENTER as
        # an Axis (RGB triad, Z aligned with the axis).
        n = 0
        for j in robot.joint_list:
            if type(j).__name__ not in ("RotationalJoint", "LinearJoint"):
                continue
            pos = np.asarray(j.world_position, float)
            axis = np.asarray(j.world_axis, float)
            R = _rot_from_z(axis)
            rod = Cylinder(radius=axis_length * 0.06, height=axis_length * 3.0,
                           face_colors=[255, 0, 255, 255],
                           pos=tuple(pos), rot=R)
            viewer.add(rod)
            ctr = Axis.from_coords(Coordinates(pos=pos, rot=R),
                                   axis_length=axis_length,
                                   axis_radius=axis_length * 0.1)
            viewer.add(ctr)
            n += 1
        print(f"added {n} axis rods (magenta Cylinder) + center triads (Axis)")

    if labels:
        for l in robot.link_list:
            pos = np.asarray(l.worldcoords().worldpos(), float)
            try:
                server.scene.add_label(f"/labels/{l.name}", text=l.name,
                                       position=tuple(pos))
            except Exception as e:
                print("label failed for", l.name, e)
        print(f"added {len(robot.link_list)} link labels")

    if gizmos:
        state = {}
        lock = threading.Lock()
        if os.path.exists(GIZMO_JSON):
            os.remove(GIZMO_JSON)

        def make_cb(jname, handle):
            def cb(_):
                with lock:
                    pos = [float(v) for v in handle.position]
                    wxyz = [float(v) for v in handle.wxyz]
                    axis = (_wxyz_to_mat(np.array(wxyz)) @ np.array([0, 0, 1.0]))
                    state[jname] = {"position": [round(v, 5) for v in pos],
                                    "wxyz": [round(v, 5) for v in wxyz],
                                    "axis_z": [round(float(v), 5) for v in axis]}
                    with open(GIZMO_JSON, "w") as f:
                        json.dump(state, f, indent=2)
            return cb

        n = 0
        for j in robot.joint_list:
            if type(j).__name__ not in ("RotationalJoint", "LinearJoint"):
                continue
            pos = np.asarray(j.world_position, float)
            axis = np.asarray(j.world_axis, float)
            wxyz = _mat_to_wxyz(_rot_from_z(axis))
            handle = server.scene.add_transform_controls(
                f"/gizmo/{j.name}", scale=gizmo_scale,
                position=tuple(pos), wxyz=tuple(wxyz))
            handle.on_update(make_cb(j.name, handle))
            n += 1
        print(f"added {n} joint gizmos; drag them -> {GIZMO_JSON}")
        print("  (gizmo Z axis = proposed rotation axis)")

    viewer.show()
    print("viser viewer running -- press Ctrl+C to stop")
    try:
        viewer.wait_until_close()
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("urdf", help="path to .urdf")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--no-labels", action="store_true")
    ap.add_argument("--no-gizmos", action="store_true")
    ap.add_argument("--no-axis-viz", action="store_true")
    ap.add_argument("--gizmo-scale", type=float, default=0.05)
    ap.add_argument("--axis-length", type=float, default=0.05)
    args = ap.parse_args()
    view(args.urdf, open_browser=not args.no_browser,
         labels=not args.no_labels, gizmos=not args.no_gizmos,
         gizmo_scale=args.gizmo_scale, axis_viz=not args.no_axis_viz,
         axis_length=args.axis_length)


if __name__ == "__main__":
    main()
