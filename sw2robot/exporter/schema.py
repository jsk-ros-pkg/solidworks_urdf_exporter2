"""CLI: write or check ``graph.schema.json``, the published graph.json contract.

    uv run python -m sw2robot.exporter.schema           # regenerate the file
    uv run python -m sw2robot.exporter.schema --check   # fail if it is stale
    uv run python -m sw2robot.exporter.schema --validate pkg/graph.json

The schema is generated from :class:`sw2robot.exporter.state.GraphState`, so the
file only needs regenerating after that model changes; the test suite fails
when someone forgets.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from .state import GraphState, graph_json_schema

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "graph.schema.json")


def render_schema():
    """The schema file's exact text."""
    return json.dumps(graph_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if graph.schema.json does not match the model")
    ap.add_argument("--validate", metavar="GRAPH_JSON", default=None,
                    help="load this graph.json and report whether it is valid")
    args = ap.parse_args(argv)

    if args.validate:
        try:
            graph = GraphState.load(args.validate)
        except (ValueError, OSError) as e:
            print(f"invalid: {e}", file=sys.stderr)
            return 1
        print(f"valid: schema_version {graph.schema_version}, "
              f"{len(graph.components)} components, {len(graph.edges)} "
              f"constraint edges, {len(graph.joints)} declared joints")
        return 0

    text = render_schema()
    if args.check:
        try:
            with open(SCHEMA_PATH, encoding="utf-8") as f:
                current = f.read()
        except OSError:
            current = None
        if current != text:
            print(f"{SCHEMA_PATH} is stale; run "
                  "`python -m sw2robot.exporter.schema`", file=sys.stderr)
            return 1
        print(f"{SCHEMA_PATH} is up to date")
        return 0

    with open(SCHEMA_PATH, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print(f"wrote {SCHEMA_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
