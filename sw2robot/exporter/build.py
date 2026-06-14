"""CLI: build URDF from a cached graph.json (fast, no SolidWorks).

    uv run python -m sw2robot.exporter.build <pkg_dir> [--config c.yaml] [--base S] [--exclude a,b] [--ros-pkg]
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
    args = ap.parse_args()
    exclude = [x.strip() for x in args.exclude.split(",")] if args.exclude else None
    build(args.pkg_dir, config_path=args.config, base_hint=args.base,
          exclude=exclude, ros_pkg=args.ros_pkg)


if __name__ == "__main__":
    main()
