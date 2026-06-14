"""Headless CLI for the CAD -> robot-compiler bridge.

    uv run python -m sw2robot.editor <package_dir> [--config c.yaml]
        [--register <registry_dir>] [--export <out.zip>] [--state-out s.json]

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
    args = ap.parse_args(argv)

    state = core.import_module(args.package_dir, config_path=args.config,
                               base_hint=args.base)
    print(f"[cad2rc] imported '{state.robot_name}': {len(state.joints)} joints "
          f"({len(state.movable_joints())} movable), root={state.root_link}")

    if args.register:
        dst = core.register_module(state, args.register)
        print(f"[cad2rc] registered -> {dst}")
    if args.export:
        out = core.export_ros_package(state, args.export)
        print(f"[cad2rc] exported  -> {out}")
    if args.state_out:
        Path(args.state_out).write_text(state.model_dump_json(indent=2),
                                        encoding="utf-8")
        print(f"[cad2rc] state     -> {args.state_out}")
    return state


if __name__ == "__main__":
    main()
