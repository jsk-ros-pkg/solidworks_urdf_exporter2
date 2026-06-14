"""Self-collision joint-limit sweep as a standalone subprocess (JSON on stdout).

The webserver shells out to this instead of sweeping in-process: the sweep is
CPU-bound and releases the GIL (numpy / python-fcl), so running it inside the
threaded HTTP server makes it thrash the GIL against the browser's idle
keep-alive connection threads -- the classic CPython convoy, which inflated an
8 s sweep to ~90 s just by having a page open.  A fresh process has its own GIL
and no server threads, so it stays at the true ~8 s (+~3 s to load the model).

    python -m sw2robot.editor._autolimits_cli <urdf> <step_deg> <max_deg>
    -> stdout: {"results": [{"child","lower","upper","continuous"}, ...]}
"""

from __future__ import annotations

import json
import os
import re
import sys
import tempfile



def _fast_urdf(urdf):
    """Return a URDF path whose mesh refs prefer the ``.3dxml.glb`` caches.

    skrobot loads the SolidWorks ``.3dxml`` (mm, slow XML parse -- the heavy
    ones are ~135 ms each and dominate load) and scales mm->m.  The web server
    already converts each to a ``.3dxml.glb`` (metres) on package open; loading
    those is ~5x faster and skrobot reads them at native scale, so the geometry
    is identical.  We rewrite the URDF to point at the .glb wherever the cache
    exists (falling back to .3dxml otherwise) and write it beside the original
    so the ``../meshes`` relative paths still resolve.  Returns (path, is_temp).
    """
    txt = open(urdf, encoding="utf-8").read()
    base = os.path.dirname(urdf)

    def repl(m):
        fn = m.group(1)
        full = os.path.normpath(os.path.join(base, fn))
        return f'filename="{fn}.glb"' if os.path.exists(full + ".glb") \
            else m.group(0)

    txt2 = re.sub(r'filename="([^"]+\.3dxml)"', repl, txt)
    if txt2 == txt:
        return urdf, False
    fd, tmp = tempfile.mkstemp(suffix=".urdf", dir=base)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt2)
    return tmp, True


def main():
    urdf, step_deg, max_deg = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    import trimesh
    from skrobot.models.urdf import RobotModelFromURDF
    from sw2robot.editor import autoinit

    load_urdf, is_tmp = _fast_urdf(urdf)
    try:
        robot = RobotModelFromURDF(urdf_file=load_urdf)
    finally:
        if is_tmp:
            try:
                os.remove(load_urdf)
            except OSError:
                pass
    meshes = {}
    for l in robot.link_list:
        vm = getattr(l, "visual_mesh", None)
        ms = (vm if isinstance(vm, (list, tuple)) else [vm]) \
            if vm is not None else []
        ms = [m for m in ms if hasattr(m, "vertices") and len(m.vertices)]
        if ms:
            meshes[l.name] = (trimesh.util.concatenate(ms)
                              if len(ms) > 1 else ms[0])

    results = autoinit.sweep_limits(
        robot, meshes, step_deg=step_deg, max_deg=max_deg, margin_deg=2.0,
        refine=True)
    out = [{"child": v["child"], "lower": v["lower"], "upper": v["upper"],
            "continuous": v["continuous"]}
           for v in results.values() if v.get("child")]
    # ONLY the JSON goes to stdout (skrobot's URDF warnings go to stderr)
    sys.stdout.write(json.dumps({"results": out}))
    sys.stdout.flush()


if __name__ == "__main__":
    main()
