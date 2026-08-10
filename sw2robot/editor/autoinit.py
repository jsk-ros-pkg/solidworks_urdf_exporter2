"""CAD-derived initial values: self-collision joint-limit sweep + proximity.

The machinery itself lives in :mod:`skrobot.collision` -- ``SelfCollision``
reports colliding link pairs beyond the rest-pose baseline, and
``sweep_limits`` rotates each joint until a NEW pair collides, that angle minus
a margin being the suggested limit.  They are re-exported here so the editor's
call sites keep reading ``autoinit.sweep_limits(...)``.

What is sw2robot's own is :func:`load_collision_parts`: the editor generates
CoACD convex decompositions as preview GLBs in its package layout, and feeding
those parts to ``SelfCollision`` is what makes the live check both fast (FCL
does convex-convex with GJK) and accurate (N parts approximate a concave shape
far better than one fat hull).

Needs skrobot + trimesh + python-fcl (the ``[ui]`` extra).  UI-independent: the
web editor (``sw2robot.editor.webserver``) and the auto-limit subprocess
(``_autolimits_cli``) call it, and so can a headless script.
"""

from __future__ import annotations

from skrobot.collision import (
    SelfCollision,
    is_fcl_available,
    link_meshes,
    link_visual_mesh,
    sweep_limits,
)
from skrobot.collision.self_collision import REST_MARGIN

__all__ = [
    "REST_MARGIN",
    "SelfCollision",
    "is_fcl_available",
    "link_meshes",
    "link_visual_mesh",
    "load_collision_parts",
    "sweep_limits",
]


def load_collision_parts(preview_dir, link_names):
    """``{link name -> [convex part trimesh, ...]}`` loaded from the CoACD preview
    GLBs (``<preview_dir>/<safe link>.glb``), each part in the link-local frame
    (the collision ``<origin>`` was baked in at generation).  Links without a
    preview GLB are omitted, so the caller falls back to their convex hull.

    These convex parts are what makes the live self-collision both fast (FCL does
    convex-convex with GJK) AND accurate (N parts approximate the concave shape
    far better than one fat hull), so no exact-mesh confirmation is needed."""
    import os
    import re

    import trimesh

    out = {}
    if not preview_dir or not os.path.isdir(preview_dir):
        return out
    for name in link_names:
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
        path = os.path.join(preview_dir, safe + ".glb")
        if not os.path.isfile(path):
            continue
        try:
            scene = trimesh.load(path)
        except Exception:
            continue
        parts = []
        if isinstance(scene, trimesh.Scene):
            # place each geometry in the scene (= link-local) frame via its graph
            # transform, so the parts line up exactly with the rendered overlay
            for node in scene.graph.nodes_geometry:
                tf, gname = scene.graph.get(node)
                g = scene.geometry[gname].copy()
                g.apply_transform(tf)
                if len(g.vertices):
                    parts.append(g)
        elif hasattr(scene, "vertices") and len(scene.vertices):
            parts.append(scene)
        if parts:
            out[name] = parts
    return out
