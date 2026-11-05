"""Check a package's graph.json, and the robot built from it, for problems.

    uv run python -m sw2robot.exporter.check <pkg_dir | graph.json> [--config c.yaml] [--json]

Every problem is a :class:`Finding` with a stable ``code``, so a program -- a CI
job, or an agent that generated the joints -- can act on it without parsing
prose.  ``--json`` prints one report object on stdout (the build's own log goes
to stderr); the exit status is 1 when any finding is an error.

The check that matters most for joints someone DECLARED is
``joint-motion-blocked``: it asks the mates between the same two parts whether
they allow the declared motion at all.  A CAD joint and the mates disagreeing
means one of them is wrong.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Literal

import numpy as np
from skrobot.utils.inertia import validate_inertia

from . import jointcfg
from .model import build_model, mate_motion_basis
from .state import SINGLE_DOF_JOINT_TYPES, GraphState

Severity = Literal["error", "warning", "info"]

GRAPH_FILE = "graph.json"

# A motion counts as allowed when the part of it the mates forbid is below
# this fraction of its size -- the same tolerance the joint classifier uses.
MOTION_TOL = 1e-6

# The ranges the build falls back to for a movable joint nobody bounded
# (model._auto_parent_map); a joint still carrying one was never given a range.
DEFAULT_RANGES = {"revolute": (-3.141592, 3.141592),
                  "prismatic": (-0.05, 0.05)}


@dataclass
class Finding:
    """One problem.  ``subjects`` names the components/links/joints involved."""
    code: str
    severity: Severity
    message: str
    subjects: list[str] = field(default_factory=list)
    data: dict = field(default_factory=dict)


def _declared_twist(dj, child_world):
    """Unit relative twist (omega, v) of the declared single-DOF motion."""
    d = np.asarray(dj.axis_dir, float)
    d = d / np.linalg.norm(d)
    if dj.type == "prismatic":
        return np.concatenate([np.zeros(3), d])
    p = (np.asarray(dj.axis_point, float) if dj.axis_point is not None
         else child_world[:3, 3])
    return np.concatenate([d, -np.cross(d, p)])


# The mate a SolidWorks limit mate shows up as in ``edge.mates``.  It bounds
# the travel; it does not lock it, which is why the build promotes the pair to
# a joint (see model.from_graph).
_LIMIT_MATE_TYPE = {"prismatic": "DISTANCE", "revolute": "ANGLE"}


def _blocking_mates(edge, limit_types):
    """The pair's mates that actually forbid motion: a limit mate's DISTANCE or
    ANGLE is a range, not a lock, so it is left out for that pair."""
    drop = {_LIMIT_MATE_TYPE[t] for t in limit_types if t in _LIMIT_MATE_TYPE}
    return [m for m in edge.mates if m.type not in drop]


def _check_declared_joints(graph, by_name, edges):
    out = []
    seen = {}
    limit_types = {}
    for lj in graph.limit_joints:
        limit_types.setdefault(frozenset((lj.a, lj.b)), set()).add(lj.type)
    for dj in graph.joints:
        label = dj.name or f"{dj.parent}--{dj.child}"
        subjects = [label, dj.parent, dj.child]
        missing = [n for n in (dj.parent, dj.child) if n not in by_name]
        if missing:
            out.append(Finding(
                "joint-unknown-component", "error",
                f"declared joint {label!r} names {', '.join(missing)}, which "
                "the graph has no component for", subjects,
                {"missing": missing}))
            continue
        pair = frozenset((dj.parent, dj.child))
        if pair in seen:
            out.append(Finding(
                "joint-duplicate-pair", "warning",
                f"declared joints {seen[pair]!r} and {label!r} join the same "
                "two components; the later one wins", subjects))
        seen[pair] = label
        if dj.type not in SINGLE_DOF_JOINT_TYPES and dj.type != "fixed":
            out.append(Finding(
                "joint-multi-dof", "warning",
                f"declared joint {label!r} is {dj.type}; URDF has no such "
                "joint, so it is exported fixed", subjects,
                {"type": dj.type}))
            continue

        edge = edges.get(pair)
        mates = (_blocking_mates(edge, limit_types.get(pair, ()))
                 if edge else [])
        basis = mate_motion_basis(mates) if mates else None
        if basis is None:
            continue                    # no mate geometry to compare against
        free = basis.shape[1]
        if dj.type == "fixed":
            if free:
                out.append(Finding(
                    "joint-under-declared", "info",
                    f"declared joint {label!r} is fixed, but its mates leave "
                    f"{free} DOF free", subjects, {"mate_dof": free}))
            continue
        xi = _declared_twist(dj, np.asarray(
            by_name[dj.child].world, float).reshape(4, 4))
        residual = xi - basis @ (basis.T @ xi)
        blocked = float(np.linalg.norm(residual)) / float(np.linalg.norm(xi))
        if blocked > MOTION_TOL:
            out.append(Finding(
                "joint-motion-blocked", "error",
                f"declared {dj.type} joint {label!r} moves in a way its mates "
                "forbid; the axis or the joint type disagrees with the CAD "
                "constraints", subjects,
                {"blocked_fraction": blocked, "mate_dof": free}))
        elif free > 1:
            out.append(Finding(
                "joint-under-declared", "info",
                f"declared joint {label!r} has 1 DOF, but its mates leave "
                f"{free} DOF free", subjects, {"mate_dof": free}))
    return out


def _check_components(graph, pkg_dir, constrained):
    out = []
    ground = set(graph.ground)
    names = {c.name for c in graph.components}
    if graph.base is not None and graph.base not in names:
        out.append(Finding(
            "base-unknown", "error",
            f"base {graph.base!r} is not a component of the graph",
            [graph.base]))
    if graph.components and not ground:
        out.append(Finding(
            "ground-missing", "warning",
            "no component is grounded; the build picks a base on its own"))
    for c in graph.components:
        if c.mesh_file and pkg_dir is not None:
            root = os.path.realpath(pkg_dir)
            path = os.path.realpath(os.path.join(
                root, *c.mesh_file.replace("\\", "/").split("/")))
            if os.path.commonpath([root, path]) != root:
                # a package is moved and zipped as one directory; a mesh outside
                # it breaks on the next machine (and the check must not probe
                # arbitrary paths for whoever wrote graph.json)
                out.append(Finding(
                    "mesh-outside-package", "error",
                    f"{c.name}: mesh {c.mesh_file} is outside the package",
                    [c.name], {"mesh_file": c.mesh_file}))
            elif not os.path.exists(path):
                out.append(Finding(
                    "mesh-missing", "error",
                    f"{c.name}: mesh {c.mesh_file} does not exist",
                    [c.name], {"mesh_file": c.mesh_file}))
        if c.sw_mass is not None:
            problems = validate_inertia(c.sw_mass, c.sw_inertia or ())
            if problems:
                out.append(Finding(
                    "mass-invalid", "error",
                    f"{c.name}: CAD mass properties are not physical: "
                    + "; ".join(problems), [c.name],
                    {"mass": c.sw_mass, "inertia": c.sw_inertia}))
        if c.name not in constrained and c.name not in ground \
                and len(graph.components) > 1:
            out.append(Finding(
                "component-unconstrained", "info",
                f"{c.name} has no constraint or declared joint; the build "
                "attaches it to the nearest part", [c.name]))
    return out


def check_graph(graph, pkg_dir=None):
    """Findings that need only ``graph.json`` (and the meshes beside it).

    Parameters
    ----------
    graph : GraphState
    pkg_dir : str | None
        Package directory the mesh paths are relative to; None skips the
        mesh-existence check.

    Returns
    -------
    list[Finding]
    """
    by_name = {c.name: c for c in graph.components}
    edges = {frozenset((e.a, e.b)): e for e in graph.edges}
    constrained = {n for e in graph.edges for n in (e.a, e.b)}
    constrained |= {n for j in graph.joints for n in (j.parent, j.child)}
    return (_check_declared_joints(graph, by_name, edges)
            + _check_components(graph, pkg_dir, constrained))


def check_model(model):
    """Findings about the robot the build produced.

    Parameters
    ----------
    model : RobotModel

    Returns
    -------
    list[Finding]
    """
    out = []
    for j in model.joints:
        default = DEFAULT_RANGES.get(j.jtype)
        if default and (j.lower, j.upper) == default:
            out.append(Finding(
                "joint-default-range", "info",
                f"{j.name}: {j.jtype} joint has no range from the CAD or the "
                f"config; the default {default} is used", [j.name],
                {"lower": j.lower, "upper": j.upper}))
    if model.loop_closures:
        out.append(Finding(
            "loop-closure", "info",
            "the assembly has closed kinematic loops; URDF keeps a tree and "
            "the loop-closing edges are recorded separately",
            data={"loop_closures": model.loop_closures}))
    return out


def check_package(path, config_path=None):
    """Check a package (directory or ``graph.json`` path) and report.

    Parameters
    ----------
    path : str
        Package directory holding ``graph.json``, or the file itself.
    config_path : str | None
        Joint-config YAML to build with, as ``sw2robot.exporter.build`` takes.

    Returns
    -------
    dict
        ``{ok, graph, schema_version, source, counts, findings}``; ``ok`` is
        False when any finding is an error.  Plain JSON types throughout.
    """
    graph_path = path if path.endswith(".json") else os.path.join(path, GRAPH_FILE)
    pkg_dir = os.path.dirname(os.path.abspath(graph_path))
    findings = []
    try:
        graph = GraphState.load(graph_path)
    except (OSError, ValueError) as e:
        findings.append(Finding("graph-invalid", "error", str(e), [graph_path]))
        return _report(graph_path, None, findings, None)
    findings += check_graph(graph, pkg_dir)
    model = None
    try:
        model = build_model(graph, config=(jointcfg.load(config_path)
                                           if config_path else None),
                            meshes_dir=os.path.join(pkg_dir, "meshes"))
    except Exception as e:
        findings.append(Finding("build-failed", "error",
                                f"{type(e).__name__}: {e}", [graph_path]))
    if model is not None:
        findings += check_model(model)
    return _report(graph_path, graph, findings, model)


def _report(graph_path, graph, findings, model):
    order = {"error": 0, "warning": 1, "info": 2}
    findings = sorted(findings, key=lambda f: order[f.severity])
    counts = {s: sum(f.severity == s for f in findings) for s in order}
    if model is not None:
        counts["links"] = len(model.components)
        counts["joints"] = len(model.joints)
        counts["movable_joints"] = sum(
            j.jtype in SINGLE_DOF_JOINT_TYPES for j in model.joints)
    return {
        "ok": counts["error"] == 0,
        "graph": graph_path,
        "schema_version": graph.schema_version if graph else None,
        "source": graph.source.model_dump() if graph and graph.source else None,
        "counts": counts,
        "findings": [asdict(f) for f in findings],
    }


def format_report(report):
    """The report as the lines a person reads."""
    lines = []
    for f in report["findings"]:
        lines.append(f"{f['severity'].upper():7} {f['code']}: {f['message']}")
    c = report["counts"]
    lines.append(f"{c['error']} error(s), {c['warning']} warning(s), "
                 f"{c['info']} note(s)")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("package", help="package dir holding graph.json, or the file")
    ap.add_argument("--config", default=None, help="joint-config YAML")
    ap.add_argument("--json", action="store_true",
                    help="print the report as JSON on stdout")
    args = ap.parse_args(argv)
    # the build narrates on stdout; keep stdout for the report alone
    with contextlib.redirect_stdout(sys.stderr):
        report = check_package(args.package, args.config)
    if args.json:
        print(json.dumps(report, indent=2, default=_json_default))
    else:
        print(format_report(report))
    return 0 if report["ok"] else 1


def _json_default(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.generic):
        return obj.item()
    raise TypeError(f"{type(obj).__name__} is not JSON serialisable")


if __name__ == "__main__":
    sys.exit(main())
