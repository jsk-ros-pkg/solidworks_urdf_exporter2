"""Headless CLI for the CAD -> robot-compiler bridge.

    uv run python -m sw2robot.editor <package_dir> [--config c.yaml]
        [--register <registry_dir>] [--export <out.zip>] [--state-out s.json]
        [--export-mujoco <dir>]

The whole pipeline runs with no GUI and no SolidWorks (it consumes the cached
``graph.json`` a prior ``sw2robot.exporter.export.extract`` produced).  A GUI is
just a thin caller of the same ``sw2robot.editor.core`` functions.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from . import core


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("package_dir",
                    help="sw2robot.exporter package dir (has graph.json)")
    ap.add_argument("--config", default=None, help="joint-config YAML")
    ap.add_argument("--base", default=None, help="base/root link hint")
    ap.add_argument("--register", default=None, metavar="DIR",
                    help="copy the module into this robot-compiler registry dir")
    ap.add_argument("--export", default=None, metavar="ZIP",
                    help="write the final ROS/config package ZIP here")
    ap.add_argument("--state-out", default=None, metavar="JSON",
                    help="dump the RobotCompilerState as JSON")
    ap.add_argument("--export-mujoco", default=None, metavar="DIR",
                    help="write a <robot>_mjcf MuJoCo package (MJCF + STL "
                         "assets) under this directory")
    ap.add_argument("--mujoco-fixed-base", action="store_true",
                    help="weld the --export-mujoco base to the world instead "
                         "of giving it a free joint; also turns off the foot "
                         "contact spheres and the IMU sensors")
    ap.add_argument("--mujoco-collision",
                    choices=("copy", "hull", "coacd",
                             "primitive", "box", "cylinder", "sphere"),
                    default="copy",
                    help="<collision> geometry for --export-mujoco "
                         "(default: reuse the visual mesh)")
    ap.add_argument("--mujoco-armature", type=float, default=0.0,
                    help="reflected rotor inertia on every --export-mujoco "
                         "joint (kg*m^2); not derivable from CAD, default 0")
    args = ap.parse_args(argv)

    state = core.import_module(args.package_dir, config_path=args.config,
                               base_hint=args.base)
    print(f"[sw2robot] imported '{state.robot_name}': {len(state.joints)} joints "
          f"({len(state.movable_joints())} movable), root={state.root_link}")

    if args.register:
        dst = core.register_module(state, args.register)
        print(f"[sw2robot] registered -> {dst}")
    if args.export:
        out = core.export_ros_package(state, args.export)
        print(f"[sw2robot] exported  -> {out}")
    if args.export_mujoco:
        out = core.export_mjcf_package(
            state, args.export_mujoco,
            collision=args.mujoco_collision,
            floating_base=not args.mujoco_fixed_base,
            armature=args.mujoco_armature,
            progress=lambda stage, detail: print(f"[sw2robot] mujoco: {detail}"))
        print(f"[sw2robot] mujoco    -> {out}")
    if args.state_out:
        Path(args.state_out).write_text(state.model_dump_json(indent=2),
                                        encoding="utf-8")
        print(f"[sw2robot] state     -> {args.state_out}")
    return state


if __name__ == "__main__":
    main()
