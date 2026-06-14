"""CLI: extract the CAD graph + meshes from SolidWorks (slow, once).

    uv run python -m sw2robot.exporter.extract <assembly.sldasm> [-o OUT] [-n NAME] [--visible]
"""
from __future__ import annotations
import argparse
from .export import extract


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("assembly")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("-n", "--name", default=None)
    ap.add_argument("--visible", action="store_true")
    args = ap.parse_args()
    extract(args.assembly, args.out, args.name, args.visible)


if __name__ == "__main__":
    main()
