"""MCP server: the build and its checks as tools an agent can call.

    sw2robot-mcp            # stdio transport; needs `pip install sw2robot[mcp]`

An agent that edits ``graph.json`` (or proposes joints) runs ``check_package``,
reads the findings, fixes what they name, and builds.  Every tool returns plain
JSON.  The build narrates on stdout, which the stdio transport owns, so each
tool sends that log to stderr instead.
"""
from __future__ import annotations

import contextlib
import os
import sys
import threading
from typing import Any

INSTRUCTIONS = (
    "sw2robot turns a CAD graph (graph.json + meshes) into URDF. Call "
    "check_package first; fix every finding with severity 'error' (the code "
    "names the problem, subjects names the parts) before build_package. "
    "simulate_package runs the MuJoCo export for a moment and reports a "
    "model that overlaps, blows up or falls through the floor. "
    "graph_schema returns the graph.json contract.")


# redirect_stdout swaps the PROCESS-wide sys.stdout, MuJoCo's warning hook is
# process-wide too, and two builds of one package write the same files.  So the
# tools run one at a time: a second call waits instead of interleaving.
_TOOL_LOCK = threading.Lock()


@contextlib.contextmanager
def _quiet():
    with _TOOL_LOCK, contextlib.redirect_stdout(sys.stderr):
        yield


# The tools return `dict[str, Any]`, not bare `dict`: the SDK only returns
# structured content for a parameterised mapping.
def check_package(package_dir: str,
                  config_path: str | None = None) -> dict[str, Any]:
    """Check a package's graph.json and the robot built from it.

    Returns ``{ok, counts, findings: [{code, severity, message, subjects,
    data}]}``; ``ok`` is false when any finding is an error.
    """
    from .exporter.check import check_package as run_check
    with _quiet():
        return run_check(package_dir, config_path)


def build_package(package_dir: str,
                  config_path: str | None = None) -> dict[str, Any]:
    """Build the URDF from ``package_dir/graph.json``.

    Returns ``{urdf, report}``: the written URDF path and the same report
    ``check_package`` gives, so one call shows what came out and what is wrong.
    """
    from .exporter.check import check_package as run_check
    from .exporter.export import build
    with _quiet():
        urdf = build(package_dir, config_path=config_path)
        report = run_check(package_dir, config_path)
    return {"urdf": os.path.abspath(urdf), "report": report}


def simulate_package(package_dir: str, config_path: str | None = None,
                     fixed_base: bool = False,
                     seconds: float = 2.0) -> dict[str, Any]:
    """Build the URDF and the MuJoCo package, then simulate it for a moment.

    ``fixed_base`` welds the base to the world (an arm on a table) instead of
    giving it a free joint.  Returns ``{mjcf, report}``, where the report has
    the same shape as ``check_package``'s and flags a model that starts with
    parts overlapping, blows up, or falls through the floor.  The MuJoCo
    package is the build's normal output next to ``package_dir`` and is kept,
    so the model that was judged can be opened.  Needs
    ``pip install 'sw2robot[sim]'``.
    """
    from .exporter.export import build
    from .exporter.mjcf_export import mjcf_model_stem, mjcf_pkg_name
    from .exporter.sim_check import simulate
    from .exporter.state import GraphState
    with _quiet():
        build(package_dir, config_path=config_path, mujoco=True,
              mujoco_fixed_base=fixed_base)
        robot = GraphState.load(os.path.join(package_dir, "graph.json")).robot_name
        mjcf = os.path.join(os.path.dirname(os.path.abspath(package_dir)),
                            mjcf_pkg_name(robot), "mjcf",
                            mjcf_model_stem(robot) + ".xml")
        report = simulate(mjcf, seconds)
    return {"mjcf": mjcf, "report": report}


def graph_schema() -> dict[str, Any]:
    """The JSON Schema every graph.json must satisfy."""
    from .exporter.state import graph_json_schema
    return graph_json_schema()


TOOLS = (check_package, build_package, simulate_package, graph_schema)


def create_server():
    """An ``MCPServer`` with every tool in :data:`TOOLS` registered."""
    try:
        from mcp.server.mcpserver import MCPServer
    except ImportError as e:
        raise SystemExit(
            "sw2robot-mcp needs the MCP SDK: pip install 'sw2robot[mcp]'"
        ) from e
    from . import __version__
    server = MCPServer(name="sw2robot", version=__version__,
                       instructions=INSTRUCTIONS)
    for tool in TOOLS:
        server.tool()(tool)
    return server


def main():
    create_server().run()


if __name__ == "__main__":
    main()
