"""CLI: build URDF from a cached graph.json (fast, no SolidWorks).

    uv run python -m sw2robot.exporter.build <pkg_dir> [--config c.yaml] [--base S] [--exclude a,b] [--ros-pkg] [--ros2]
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
    args = ap.parse_args()
    exclude = [x.strip() for x in args.exclude.split(",")] if args.exclude else None
    build(args.pkg_dir, config_path=args.config, base_hint=args.base,
          exclude=exclude, ros_pkg=args.ros_pkg or args.ros2,
          ros_version=2 if args.ros2 else 1, ros_pkg_name=args.ros_pkg_name,
          ros_urdf_name=args.ros_urdf_name)


if __name__ == "__main__":
    main()
