"""CLI: extract the CAD graph + meshes from SolidWorks (slow, once).

    uv run python -m sw2robot.exporter.extract <assembly.sldasm> [-o OUT] [-n NAME] [--visible]

Fast debug iteration:

    # reuse the running SolidWorks (keep the assembly open there = no reopen)
    uv run python -m sw2robot.exporter.extract <assembly.sldasm> --attach

    # re-read ONLY coordinate systems / reference axes into graph.json
    uv run python -m sw2robot.exporter.extract <assembly.sldasm|pkg_dir> --refresh frames --attach
"""
from __future__ import annotations

import argparse

from .export import extract, refresh_frames


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("assembly",
                    help="path to a .SLDASM/.SLDPRT -- or, with --refresh, "
                         "optionally the extracted package dir holding "
                         "graph.json")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("-n", "--name", default=None)
    ap.add_argument("--visible", action="store_true")
    ap.add_argument("--attach", action="store_true",
                    help="reuse the USER'S already-running SolidWorks instead "
                         "of starting a hidden private instance; if the "
                         "assembly is already open there, the multi-minute "
                         "reopen is skipped entirely.  Errors out (no "
                         "fallback) when no running SolidWorks is found")
    ap.add_argument("--refresh", choices=("frames",), default=None,
                    metavar="WHAT",
                    help="partial re-extract: 'frames' re-reads ONLY the "
                         "named coordinate systems + reference axes into the "
                         "existing graph.json (components/mates/masses/meshes "
                         "untouched) -- seconds instead of minutes when "
                         "iterating on frame selection")
    ap.add_argument("--configuration", default=None, metavar="NAME",
                    help="extract this ASSEMBLY configuration instead of the "
                         "file's saved-active one")
    args = ap.parse_args()
    import os
    if args.refresh is None and os.path.isdir(args.assembly):
        # without --refresh the positional is a CAD FILE; a bare directory
        # would fall through to the .SLDPRT path and die deep in a file copy
        ap.error(f"{args.assembly!r} is a directory -- a package dir is only "
                 f"accepted with '--refresh frames'; a full extract needs the "
                 f".SLDASM/.SLDPRT path itself")
    if args.refresh == "frames":
        refresh_frames(args.assembly, args.out, args.name, args.visible,
                       attach=args.attach)
    else:
        extract(args.assembly, args.out, args.name, args.visible,
                configuration=args.configuration, attach=args.attach)


if __name__ == "__main__":
    main()
