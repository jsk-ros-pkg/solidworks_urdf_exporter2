"""CLI: build URDF from a cached graph.json (fast, no SolidWorks).

    uv run python -m sw2robot.exporter.build <pkg_dir> [--config c.yaml] [--base S] [--exclude a,b] [--ros-pkg] [--ros2] [--mujoco]
"""
from __future__ import annotations

import argparse

from .export import build


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pkg_dir", help="package dir containing graph.json")
    ap.add_argument("--config", default=None)
    ap.add_argument("--base", default=None)
    ap.add_argument("--exclude", default=None)
    ap.add_argument("--ros-pkg", action="store_true")
    ap.add_argument("--ros2", action="store_true",
                    help="make --ros-pkg an ament_cmake (ROS 2) package with "
                         "launch/ + rviz/ instead of catkin; implies --ros-pkg")
    ap.add_argument("--ros-pkg-name", default=None,
                    help="name for the --ros-pkg package (default "
                         "<name>_description); lowercase letters/digits/_ only")
    ap.add_argument("--ros-urdf-name", default=None,
                    help="stem for the URDF inside --ros-pkg (default: pkg name)")
    ap.add_argument("--ros-robot-name", default=None,
                    help="<robot name> inside the --ros-pkg URDF "
                         "(default: the URDF stem)")
    ap.add_argument("--ros-mesh-dir", default=None,
                    help="package-relative mesh directory for --ros-pkg "
                         "(default: 'meshes'); e.g. 'urdf/mesh'")
    ap.add_argument("--collision",
                    choices=("copy", "hull", "coacd",
                             "primitive", "box", "cylinder", "sphere"),
                    default="copy",
                    help="<collision> geometry for the exported packages; see "
                         "sw2urdf --collision")
    ap.add_argument("--coacd-quality", choices=("balanced", "fine"),
                    default="balanced",
                    help="CoACD preset for --collision coacd")
    ap.add_argument("--mujoco", action="store_true",
                    help="also write a standalone <name>_mjcf package: an MJCF "
                         "MuJoCo model plus binary-STL assets")
    ap.add_argument("--mujoco-pkg-name", default=None,
                    help="directory name for --mujoco (default <name>_mjcf)")
    ap.add_argument("--mujoco-name", default=None,
                    help="stem for the .xml inside --mujoco "
                         "(default: the robot name)")
    ap.add_argument("--mujoco-fixed-base", action="store_true",
                    help="weld the --mujoco base to the world (an arm bolted "
                         "down), instead of a free joint; also turns off the "
                         "foot contact spheres and IMU sensors")
    ap.add_argument("--mujoco-actuator",
                    choices=("position", "velocity", "motor"),
                    default="position",
                    help="--mujoco actuator per joint (default: position)")
    ap.add_argument("--mujoco-kp", type=float, default=50.0,
                    help="gain of the --mujoco position servos (default 50)")
    ap.add_argument("--mujoco-armature", type=float, default=0.0,
                    help="reflected rotor inertia on every --mujoco joint "
                         "(kg*m^2); not derivable from CAD, so default 0")
    ap.add_argument("--mujoco-self-collision", action="store_true",
                    help="keep full self-collision in the --mujoco model")
    ap.add_argument("--mujoco-no-backemf-damping", action="store_true",
                    help="do not write each joint's effort/velocity damping "
                         "into the --mujoco model; for a consumer that brings "
                         "its own actuator model")
    args = ap.parse_args()
    exclude = [x.strip() for x in args.exclude.split(",")] if args.exclude else None
    build(args.pkg_dir, config_path=args.config, base_hint=args.base,
          exclude=exclude, ros_pkg=args.ros_pkg or args.ros2,
          ros_version=2 if args.ros2 else 1, ros_pkg_name=args.ros_pkg_name,
          ros_urdf_name=args.ros_urdf_name, ros_robot_name=args.ros_robot_name,
          ros_mesh_dir=args.ros_mesh_dir,
          collision=args.collision, coacd_quality=args.coacd_quality,
          mujoco=args.mujoco, mujoco_pkg_name=args.mujoco_pkg_name,
          mujoco_name=args.mujoco_name,
          mujoco_fixed_base=args.mujoco_fixed_base,
          mujoco_actuator=args.mujoco_actuator, mujoco_kp=args.mujoco_kp,
          mujoco_armature=args.mujoco_armature,
          mujoco_self_collision=args.mujoco_self_collision,
          mujoco_backemf_damping=not args.mujoco_no_backemf_damping)


if __name__ == "__main__":
    main()
