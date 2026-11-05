"""Smoke-test an exported MJCF by simulating it for a moment.

    uv run python -m sw2robot.exporter.sim_check <model.xml> [--seconds 2] [--json]

A model can compile and still be useless: parts that start inside each other
get thrown apart, a bad inertia makes the integrator blow up, a base falls
through the floor.  None of that shows until the model runs, so this runs it --
from the ``home`` keyframe when there is one -- and reports what went wrong in
the same finding format as :mod:`sw2robot.exporter.check`.  Needs ``mujoco``
(``pip install 'sw2robot[sim]'``).
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from dataclasses import asdict

import numpy as np

from .check import Finding

DEFAULT_SECONDS = 2.0

# The keyframe the MJCF export writes for the standing pose.
HOME_KEY = "home"

# mju_user_warning is one process-wide hook: two runs swapping it at once
# could restore each other's handler and let a warning reach the C stdout.
_WARNING_HOOK_LOCK = threading.Lock()

# Contacts deeper than this at the start mean two parts were exported
# overlapping (a sub-millimetre touch is normal for a resting foot).
PENETRATION_TOL_M = 1e-3

# No servo in a CAD robot turns a joint this fast or slides it this fast;
# reaching it means the integrator has lost the model.
MAX_JOINT_SPEED = {"hinge": 1e3, "slide": 1e2}       # rad/s, m/s
MAX_BASE_SPEED_M_S = 1e2

# A floating base this far below its start has gone through the floor.
FELL_THROUGH_M = 0.5

# MuJoCo warnings that mean the simulation itself failed (it resets the state
# when they fire, so NaN never reaches qpos), versus ones that mean a buffer was
# too small and contacts or constraints were dropped.
FATAL_WARNINGS = ("mjWARN_BADQACC", "mjWARN_BADQPOS", "mjWARN_BADQVEL",
                  "mjWARN_BADCTRL")
DEGRADING_WARNINGS = ("mjWARN_CONTACTFULL", "mjWARN_CNSTRFULL",
                      "mjWARN_INERTIA")


def _import_mujoco():
    try:
        import mujoco
    except ImportError as e:
        raise RuntimeError(
            "the simulation check needs MuJoCo: pip install 'sw2robot[sim]'"
        ) from e
    return mujoco


def _name(mujoco, model, obj, idx):
    return mujoco.mj_id2name(model, obj, idx) or f"{obj.name[6:].lower()}{idx}"


def _initial_penetrations(mujoco, model, data):
    """Findings for body pairs that start overlapping, deepest first."""
    deepest = {}
    for i in range(data.ncon):
        dist = float(data.contact.dist[i])
        if dist >= -PENETRATION_TOL_M:
            continue
        g1, g2 = (int(g) for g in data.contact.geom[i])
        bodies = tuple(sorted(
            _name(mujoco, model, mujoco.mjtObj.mjOBJ_BODY,
                  int(model.geom_bodyid[g])) for g in (g1, g2)))
        deepest[bodies] = min(deepest.get(bodies, 0.0), dist)
    return [Finding(
        "sim-initial-penetration", "warning",
        f"{a} and {b} start {-d * 1000:.1f} mm inside each other; the "
        "simulation pushes them apart on the first step", [a, b],
        {"depth_m": -d})
        for (a, b), d in sorted(deepest.items(), key=lambda kv: kv[1])]


def _speed_problem(mujoco, model, data):
    """The first joint moving impossibly fast, as ``(name, speed, limit)``."""
    for j in range(model.njnt):
        jtype = mujoco.mjtJoint(int(model.jnt_type[j]))
        adr = int(model.jnt_dofadr[j])
        if jtype == mujoco.mjtJoint.mjJNT_FREE:
            speed, limit = float(np.linalg.norm(data.qvel[adr:adr + 3])), \
                MAX_BASE_SPEED_M_S
        elif jtype == mujoco.mjtJoint.mjJNT_HINGE:
            speed, limit = abs(float(data.qvel[adr])), MAX_JOINT_SPEED["hinge"]
        elif jtype == mujoco.mjtJoint.mjJNT_SLIDE:
            speed, limit = abs(float(data.qvel[adr])), MAX_JOINT_SPEED["slide"]
        else:
            continue
        if speed > limit:
            return _name(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT, j), \
                speed, limit
    return None


def _free_base_height(mujoco, model, data):
    for j in range(model.njnt):
        if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
            return float(data.qpos[int(model.jnt_qposadr[j]) + 2])
    return None


def simulate(mjcf_path, seconds=DEFAULT_SECONDS):
    """Simulate ``mjcf_path`` and report what went wrong.

    Parameters
    ----------
    mjcf_path : str
        MJCF file to load.
    seconds : float
        Simulated time to run.

    Returns
    -------
    dict
        ``{ok, mjcf, seconds, simulated, counts, findings}``: ``simulated`` is
        the time reached before a failure stopped the run; ``ok`` is False when
        any finding is an error.
    """
    mujoco = _import_mujoco()
    findings = []
    reached = 0.0
    try:
        model = mujoco.MjModel.from_xml_path(mjcf_path)
    except Exception as e:
        findings.append(Finding("sim-compile-failed", "error",
                                f"MuJoCo cannot load the model: {e}",
                                [mjcf_path]))
        return _report(mjcf_path, seconds, reached, findings)

    messages = []
    with _WARNING_HOOK_LOCK:
        previous = mujoco.get_mju_user_warning()
        mujoco.set_mju_user_warning(messages.append)
        try:
            data, reached = _run(mujoco, model, seconds, findings)
        finally:
            mujoco.set_mju_user_warning(previous)
    for message in messages:
        print(f"[mujoco] {message}", file=sys.stderr)

    for name in FATAL_WARNINGS + DEGRADING_WARNINGS:
        idx = int(getattr(mujoco.mjtWarning, name))
        count = int(data.warning[idx].number)
        if not count:
            continue
        fatal = name in FATAL_WARNINGS
        findings.append(Finding(
            "sim-warning", "error" if fatal else "warning",
            f"MuJoCo raised {name} {count} time(s)"
            + (" and reset the state" if fatal else ""), [],
            {"warning": name, "count": count, "messages": messages}))
    return _report(mjcf_path, seconds, reached, findings)


def _run(mujoco, model, seconds, findings):
    """Step ``model`` from its start pose, appending what goes wrong.

    The start pose is the ``home`` keyframe, or the only keyframe when there is
    one; with several and none named ``home`` it is the model's own ``qpos0``,
    rather than whichever key happens to be listed first.

    Returns ``(data, reached)``: a fatal MuJoCo warning resets ``data.time``,
    so the time the run got to is tracked separately.

    MuJoCo prints its warnings on the C stdout, which an MCP stdio server
    cannot share, so the caller routes them through ``mju_user_warning``."""
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEY)
    if key < 0 and model.nkey == 1:
        key = 0
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    findings += _initial_penetrations(mujoco, model, data)
    start_height = _free_base_height(mujoco, model, data)
    reached = 0.0

    steps = round(seconds / float(model.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(model, data)
        reached = float(data.time)
        if not (np.all(np.isfinite(data.qpos))
                and np.all(np.isfinite(data.qvel))):
            findings.append(Finding(
                "sim-diverged", "error",
                f"the state became NaN/Inf at t={reached:.3f} s", [],
                {"time": reached}))
            break
        fast = _speed_problem(mujoco, model, data)
        if fast is not None:
            joint, speed, limit = fast
            findings.append(Finding(
                "sim-unstable", "error",
                f"{joint} reached {speed:.3g} (limit {limit:g}) at "
                f"t={reached:.3f} s; the integrator has lost the model",
                [joint], {"time": reached, "speed": speed, "limit": limit}))
            break

    # without a floor a floating base is SUPPOSED to fall
    has_floor = bool(np.any(model.geom_type == int(mujoco.mjtGeom.mjGEOM_PLANE)))
    end_height = _free_base_height(mujoco, model, data)
    if has_floor and start_height is not None and end_height is not None \
            and start_height - end_height > FELL_THROUGH_M:
        findings.append(Finding(
            "sim-fell-through", "error",
            f"the floating base dropped {start_height - end_height:.2f} m; "
            "it is falling through the floor", [],
            {"start_height": start_height, "end_height": end_height}))
    return data, reached


def _report(mjcf_path, seconds, reached, findings):
    order = {"error": 0, "warning": 1, "info": 2}
    findings = sorted(findings, key=lambda f: order[f.severity])
    counts = {s: sum(f.severity == s for f in findings) for s in order}
    return {"ok": counts["error"] == 0, "mjcf": mjcf_path,
            "seconds": seconds, "simulated": reached, "counts": counts,
            "findings": [asdict(f) for f in findings]}


def main(argv=None):
    from .check import format_report

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mjcf", help="MJCF file (an exported <robot>_mjcf/mjcf/*.xml)")
    ap.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    ap.add_argument("--json", action="store_true",
                    help="print the report as JSON on stdout")
    args = ap.parse_args(argv)
    report = simulate(args.mjcf, args.seconds)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"simulated {report['simulated']:.3f} of {args.seconds:g} s")
        print(format_report(report))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
