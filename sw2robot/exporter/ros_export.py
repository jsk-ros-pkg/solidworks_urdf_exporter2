"""Turn a built package into a portable ROS *description* package.

The in-house URDF (``urdf/<name>.urdf``) references meshes by a path relative to
the URDF (``../meshes/<link>.3dxml`` / ``.glb``).  That loads in our viewer and in
skrobot, but it is NOT a portable ROS package: RViz/Gazebo cannot read 3DXML/GLB,
and ROS tooling expects ``package://<pkg>/...`` URLs.

This module produces a SEPARATE file set -- it never touches the working package --
named ``<robot_name>_description`` with:

* ``<visual>`` meshes as COLLADA ``.dae`` (metres, colours preserved -- unlike STL),
* ``<collision>`` meshes as plain ``.stl`` (lighter, no colour -- the usual ROS
  collision convention),
* ``package://<robot_name>_description/meshes/<link>.{dae,stl}`` references, and
* a ``package.xml`` / ``CMakeLists.txt`` whose package name is that same
  ``<robot_name>_description``.

``build_ros_description`` returns ``[(arcname, bytes), ...]`` so the web server can
zip it in memory and the CLI can write it to disk.
"""

from __future__ import annotations

import copy
import hashlib
import io
import json
import os
import re
import threading
import xml.etree.ElementTree as ET

from .urdf_writer import (
    CMAKELISTS,
    CMAKELISTS_ROS2,
    CMAKELISTS_ROS2_COUPLED,
    DISPLAY_LAUNCH_ROS2,
    DISPLAY_LAUNCH_ROS2_COUPLED,
    LOOP_COUPLING_RELAY_PY,
    PACKAGE_XML,
    PACKAGE_XML_ROS2,
    PACKAGE_XML_ROS2_COUPLED,
    RVIZ_CONFIG_ROS2,
)

# fallback <material> colour for an STL <visual> with no per-link override (STL
# carries no colour, so without this RViz would show the link in its default grey)
_DEFAULT_VISUAL_RGBA = (191, 191, 191, 255)   # neutral light grey (~0.75)


def _loop_closures_yaml(closures):
    """Serialise the loop-closure config (closures + driven/solved joint split)
    to the IK relay's config YAML."""
    import yaml
    head = ("# Closed-loop closures for loop_closure_relay (runtime IK).\n"
            "# independent = driven joints (sliders); dependent = solved by IK\n"
            "# so each closure's two link points (the cut hinge) coincide.\n")
    return head + yaml.safe_dump(closures, sort_keys=False,
                                 default_flow_style=None)

# extensions we convert; anything else (already .dae/.stl, abs URLs) is left alone
# source mesh formats we can load + reconvert: CAD output (.3dxml mm, .glb m)
# plus the common URDF mesh formats (.stl/.dae/.obj, already in metres), so a
# URDF opened for re-export ships as a normal <name>_description package too.
_CONVERTIBLE = (".3dxml", ".glb", ".stl", ".dae", ".obj")


def _load_mesh_metres(src):
    """Load a mesh (``.3dxml`` is mm, everything else is already metres), flatten
    any scene to a single geometry, and return it pinned to metres."""
    import trimesh

    loaded = trimesh.load(src)
    if isinstance(loaded, trimesh.Scene):
        # to_geometry() is trimesh's current single-mesh flatten; fall back to
        # the older dump() on versions that predate it
        if hasattr(loaded, "to_geometry"):
            mesh = loaded.to_geometry()
        elif hasattr(loaded, "to_mesh"):
            mesh = loaded.to_mesh()
        else:
            mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded
    if src.lower().endswith(".3dxml"):
        mesh.apply_scale(0.001)            # 3DXML tessellation is in mm
    # the 'mm' units tag survives apply_scale; leaving it makes unit-aware
    # loaders shrink the mesh 1000x, so pin it to metres
    mesh.units = "meter"
    return mesh


def _hex_to_rgba(hex_color):
    """``'#RRGGBB'`` / ``'#RRGGBBAA'`` (the leading ``#`` optional) -> a uint8
    ``[R, G, B, A]`` numpy array, or ``None`` for a missing/malformed value."""
    if not hex_color:
        return None
    import numpy as np

    h = str(hex_color).strip().lstrip("#")
    if len(h) == 6:
        h += "ff"
    if len(h) != 8:
        return None
    try:
        return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4, 6)],
                        dtype=np.uint8)
    except ValueError:
        return None


def _apply_uniform_color(mesh, hex_color):
    """Repaint every face of ``mesh`` one solid colour (dropping any original
    texture/material), so a per-link override wins in every output format.  A
    falsy/invalid ``hex_color`` leaves the mesh untouched."""
    rgba = _hex_to_rgba(hex_color)
    if rgba is None:
        return mesh
    import numpy as np
    import trimesh

    mesh.visual = trimesh.visual.ColorVisuals(
        mesh=mesh, face_colors=np.tile(rgba, (len(mesh.faces), 1)))
    return mesh


def _baked(mesh, transform):
    """Apply a 4x4 ``<origin>`` transform into the mesh vertices (so the URDF
    origin can be set to identity -- what MuJoCo wants)."""
    if transform is not None:
        import numpy as np
        mesh.apply_transform(np.asarray(transform, float))
    return mesh


def _mesh_to_dae_bytes(src, color=None, transform=None, mesh=None):
    """COLLADA (.dae) bytes in metres, with colours baked into materials.
    ``color`` (``'#RRGGBB'``) overrides the mesh's own colours; ``transform``
    (4x4) is baked into the vertices so the URDF origin can be zeroed.  ``mesh``
    (a preloaded, OWNED-and-mutable trimesh) skips the per-call ``trimesh.load``
    so one source loaded once can feed several conversions."""
    m = mesh if mesh is not None else _load_mesh_metres(src)
    meshes = _collada_meshes(_apply_uniform_color(_baked(m, transform), color))
    if len(meshes) == 1:
        return meshes[0].export(file_type="dae")
    from trimesh.exchange.dae import export_collada
    return export_collada(meshes)


def _mesh_to_stl_bytes(src, color=None, transform=None, mesh=None):
    """STL bytes in metres (colour is dropped -- collision geometry needs none,
    so ``color`` is accepted for a uniform converter signature but ignored).
    ``transform`` (4x4) is baked into the vertices.  ``mesh`` (preloaded) skips
    the per-call ``trimesh.load``."""
    m = mesh if mesh is not None else _load_mesh_metres(src)
    return _baked(m, transform).export(file_type="stl")


def _mesh_to_glb_bytes(src, color=None, transform=None, mesh=None):
    """GLB bytes in metres, keeping the original material/texture (for
    three.js / skrobot consumers, not RViz).  ``color`` (``'#RRGGBB'``)
    overrides the mesh's own colours; ``transform`` (4x4) is baked in.  ``mesh``
    (preloaded) skips the per-call ``trimesh.load``."""
    m = mesh if mesh is not None else _load_mesh_metres(src)
    return _apply_uniform_color(_baked(m, transform), color).export(
        file_type="glb")


_CONVERT = {"dae": _mesh_to_dae_bytes, "stl": _mesh_to_stl_bytes,
            "glb": _mesh_to_glb_bytes}

# default: <visual> as colour COLLADA, <collision> as plain STL (the ROS split)
_CTX_FMT = (("visual", "dae"), ("collision", "stl"))
# a uniform GLB variant (both contexts) for native-mesh / three.js consumers
GLB_CTX_FMT = (("visual", "glb"), ("collision", "glb"))


# ----------------------------------------------------------- mesh cache + reuse
# The slow part of an export is per-source mesh work: trimesh's load (a 3DXML is
# a zipped XML the loader parses) plus the format conversion.  Two things speed
# it up, both here: (1) load each source ONCE and reuse it (a 'copy()' costs ~ms
# vs the ~100-200ms load -- the same source feeds the size measurement, the
# visual .dae and the collision .stl); (2) disk-cache the converted bytes + the
# geometry measurements keyed by source CONTENT, so a re-export (ros.zip then
# ros2.zip, or the same model again) skips trimesh entirely.
#
# NB the mesh path is deliberately SERIAL: trimesh's 3DXML loader and the
# COLLADA exporter are pure-Python (GIL-bound), so a thread pool over them only
# oversubscribes and runs ~2x SLOWER (measured).  CoACD below is different (a
# compiled lib that releases the GIL) and IS parallelised.

def _pool_workers(n_items, max_workers=None):
    """Worker count for a CoACD pool over ``n_items``: ``cpu_count - 2`` (capped
    at 8, floored at 1), never more than the work available."""
    cap = max_workers or min(8, max(1, (os.cpu_count() or 2) - 2))
    return max(1, min(cap, n_items))


def _parallel_cancellable(func, items, should_cancel=None, max_workers=None):
    """Concurrent fan-out (thread pool) for GIL-releasing work (CoACD /
    primitive fitting).  ``func`` MUST NOT raise (catch + return a sentinel);
    results are not collected.

    Without ``should_cancel`` it simply runs every item and returns.  With one,
    it returns promptly -- raising :class:`ExportCancelled` within ~0.2 s once
    ``should_cancel`` turns true -- WITHOUT waiting for the tasks already
    running (they finish in the background, since a CoACD part can't be
    interrupted mid-run; its result is just cached), and drops the not-yet-
    started ones."""
    items = list(items)
    if not items:
        return
    from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
    ex = ThreadPoolExecutor(max_workers=_pool_workers(len(items), max_workers))
    pending = {ex.submit(func, x) for x in items}
    try:
        while pending:
            if should_cancel and should_cancel():
                raise ExportCancelled()
            _done, pending = wait(pending, timeout=0.2,
                                  return_when=FIRST_COMPLETED)
    finally:
        # don't block on in-flight tasks when cancelling; drop unstarted ones
        ex.shutdown(wait=False, cancel_futures=True)


def _atomic_write(path, data):
    """Write ``data`` (bytes) to ``path`` via a unique temp + ``os.replace`` so a
    concurrent writer (another export hitting the same cache key) can't see a
    half-written file or race on the temp name."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.{os.getpid()}.{threading.get_ident()}.tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


# bump when the converted-bytes / measurement cache layout changes (stale caches
# under a previous version are then ignored rather than mis-read)
_MESH_CACHE_VERSION = 1


class _MeshCache:
    """Per-export mesh cache: load each source's trimesh ONCE (the dominant
    cost), hand out cheap mutable copies for conversion, and disk-cache both the
    converted bytes and the geometry measurements (keyed by source CONTENT) under
    ``cache_dir`` so a re-export of an unchanged model needs no trimesh at all.
    Used serially within one export (the mesh path is GIL-bound, see above)."""

    def __init__(self, cache_dir):
        self._cache_dir = cache_dir
        self._meshes = {}          # abspath -> trimesh (in-memory, load-once)
        self._shas = {}            # abspath -> source content sha1 (read-once)

    def sha(self, src):
        """The source file's content sha1 (memoised) -- the disk-cache key, so an
        edited mesh invalidates its cache while a moved/renamed one still hits."""
        key = os.path.abspath(src)
        h = self._shas.get(key)
        if h is None:
            with open(src, "rb") as f:
                h = hashlib.sha1(f.read()).hexdigest()
            self._shas[key] = h
        return h

    def _ensure(self, src):
        key = os.path.abspath(src)
        m = self._meshes.get(key)
        if m is None:
            m = self._meshes[key] = _load_mesh_metres(src)
        return m

    def copy(self, src):
        """An independent, mutable copy of the source mesh (each conversion bakes
        a different colour/transform, so they cannot share one object)."""
        return self._ensure(src).copy()

    def measure(self, src):
        """``(size, centroid)`` -- ``size`` = bounding-box diagonal in metres,
        ``centroid`` a 3-vector -- disk-cached by source content so a re-export's
        origin baking needs no trimesh load.  ``None`` if the mesh won't load."""
        import numpy as np
        path = os.path.join(
            self._cache_dir, f"{self.sha(src)}.measure{_MESH_CACHE_VERSION}.json")
        try:
            with open(path) as f:
                d = json.load(f)
            return float(d[0]), np.asarray(d[1:4], float)
        except (OSError, ValueError, IndexError, TypeError):
            pass
        try:
            m = self._ensure(src)
        except Exception:
            return None
        size = float(np.linalg.norm(m.extents))
        centroid = np.asarray(m.centroid, float)
        try:
            _atomic_write(path,
                          json.dumps([size, *centroid.tolist()]).encode("utf-8"))
        except OSError:
            pass
        return size, centroid

    def convert(self, src, fmt, color, transform):
        """Converted mesh bytes (``_CONVERT[fmt]``), disk-cached by source content
        + conversion params so a re-export skips the trimesh re-conversion."""
        tkey = None if transform is None else tuple(
            round(float(x), 7) for x in transform.flatten())
        h = hashlib.sha1(self.sha(src).encode())
        h.update(json.dumps([fmt, color, tkey, _MESH_CACHE_VERSION],
                            sort_keys=True).encode())
        path = os.path.join(self._cache_dir, f"{h.hexdigest()[:16]}.{fmt}")
        try:
            with open(path, "rb") as f:
                return f.read()
        except OSError:
            pass
        data = _CONVERT[fmt](src, color=color, transform=transform,
                             mesh=self.copy(src))
        try:
            _atomic_write(path, data)
        except OSError:
            pass
        return data


# ----------------------------------------------------- CoACD collision meshes
# Optionally replace each <collision>'s single (concave) mesh with a set of
# convex parts via CoACD (approximate convex decomposition).  Convex collision
# geometry is what physics engines (Gazebo/Bullet/MuJoCo) actually want; reusing
# the visual mesh works but is slow and can mis-collide on concave shapes.
#
# CoACD is an OPTIONAL dependency (the `coacd` extra) and decomposition is slow
# (tens of seconds per link), so this is opt-in and the result is cached on disk
# keyed by the source mesh content + parameters -- a re-export is then instant.

# CoACD parameters per quality preset.  CoACD's cost is dominated by the MCTS
# search, which runs to ``max_convex_hull`` cuts whenever ``threshold`` is too
# low to stop earlier -- so a low threshold + high part cap + many MCTS
# iterations makes EVERY part (even a tiny one) pay the full ~2-minute search.
# Measured on the feetech_hand parts: the old 'balanced' (0.1 / 8 / mcts 100)
# took 100-150 s on small parts; 'balanced' below is ~8-60 s (2-5x faster) with
# 'fine' kept for a tighter fit when the wait is worth it.
_COACD_PRESETS = {
    "balanced": {"threshold": 0.2, "max_convex_hull": 6,
                 "preprocess_resolution": 30, "mcts_iterations": 30},
    "fine": {"threshold": 0.1, "max_convex_hull": 8,
             "preprocess_resolution": 40, "mcts_iterations": 60},
}
# bump when the decomposition output format changes so stale caches are ignored
_COACD_CACHE_VERSION = 1

# per-cache-key locks: when several links share ONE source mesh (e.g. 5 instances
# of the same servo), the parallel preview would otherwise run + write the same
# cache files concurrently and race (a Windows file-in-use error).  Serialise per
# key so the first thread computes + writes and the rest read the cache.
_coacd_cache_locks = {}
_coacd_cache_locks_guard = threading.Lock()


def _coacd_key_lock(key):
    with _coacd_cache_locks_guard:
        lk = _coacd_cache_locks.get(key)
        if lk is None:
            lk = _coacd_cache_locks[key] = threading.Lock()
        return lk


def coacd_available():
    """True if the optional ``coacd`` package is importable."""
    import importlib.util
    return importlib.util.find_spec("coacd") is not None


def _run_coacd(vertices, faces, params):
    """Run CoACD and return ``[(verts, faces), ...]`` convex parts.  Thin
    indirection over the ``coacd`` package so tests can monkeypatch it without
    installing CoACD or paying its (tens-of-seconds) runtime."""
    import coacd

    coacd.set_log_level("error")
    mesh = coacd.Mesh(vertices, faces)
    return coacd.run_coacd(mesh, merge=True, **params)


def _coacd_part_stls(src, quality, cache_dir):
    """``[stl_bytes, ...]`` -- the source mesh ``src`` decomposed into convex
    collision parts (each a watertight STL in metres) per the ``quality`` preset.

    Cached under ``cache_dir/<key>/`` keyed by the source file content + params,
    so repeated exports of an unchanged mesh skip the slow CoACD run."""
    import numpy as np
    import trimesh

    params = _COACD_PRESETS[quality]
    with open(src, "rb") as f:
        src_bytes = f.read()
    h = hashlib.sha1(src_bytes)
    h.update(json.dumps([quality, params, _COACD_CACHE_VERSION],
                        sort_keys=True).encode())
    key = h.hexdigest()[:16]
    part_dir = os.path.join(cache_dir, key)
    manifest = os.path.join(part_dir, "parts.json")

    def _read_cache():
        try:
            with open(manifest) as f:
                names = json.load(f)
            return [open(os.path.join(part_dir, n), "rb").read() for n in names]
        except (OSError, ValueError):
            return None         # missing/corrupt/partial -- needs a (re)build

    cached = _read_cache()
    if cached is not None:
        return cached

    # hold the per-key lock across the check+build so parallel links that share
    # this source mesh don't all run CoACD / write the same files at once
    with _coacd_key_lock(key):
        cached = _read_cache()          # another thread may have built it first
        if cached is not None:
            return cached
        mesh = _load_mesh_metres(src)
        parts = _run_coacd(mesh.vertices, mesh.faces, params)
        os.makedirs(part_dir, exist_ok=True)
        out, names = [], []
        for i, (verts, faces) in enumerate(parts):
            # CoACD parts are convex; take the convex hull to drop any sliver
            # faces and guarantee a clean watertight collision mesh
            part = trimesh.Trimesh(vertices=np.asarray(verts),
                                   faces=np.asarray(faces),
                                   process=False).convex_hull
            part.units = "meter"
            data = part.export(file_type="stl")
            name = f"part_{i}.stl"
            with open(os.path.join(part_dir, name), "wb") as f:
                f.write(data)
            out.append(data)
            names.append(name)
        with open(manifest, "w") as f:
            json.dump(names, f)
        return out


def _hull_part_stls(src):
    """``[stl_bytes]`` -- the source mesh ``src`` reduced to a SINGLE watertight
    convex hull (one STL in metres). """
    hull = _load_mesh_metres(src).convex_hull
    hull.units = "meter"
    return [hull.export(file_type="stl")]


def _collision_part_stls(src, mode, quality, cache_dir):
    """``[stl_bytes, ...]`` -- the convex collision part(s) for ``src`` per the
    selected ``mode``: ``"coacd"`` runs CoACD approximate convex decomposition
    (many parts, slow, disk-cached under ``cache_dir``); ``"hull"`` returns a
    single convex hull (fast, no optional dep).  The single-vs-many distinction
    is the only difference downstream consumers see."""
    if mode == "hull":
        return _hull_part_stls(src)
    return _coacd_part_stls(src, quality, cache_dir)


# ------------------------------------------------ primitive collision shapes
# Optionally replace each <collision>'s mesh with a fitted URDF primitive
# (<box>/<cylinder>/<sphere>) instead of a mesh file -- the lightest collision
# geometry there is (native URDF primitives, no STL), for path planning where a
# coarse convex bound per link is enough.  The fitting lives UPSTREAM in
# scikit-robot (fit_primitive_to_mesh + primitive_params_to_origin); we only
# place the result.  scikit-robot is a core dependency, but a version predating
# the per-mesh primitive API still raises a clear error, like coacd.

# collision-mode string -> primitive_type for fit_primitive_to_mesh
# (None = auto-pick box/cylinder/sphere by voxel IoU)
_PRIMITIVE_MODES = {"primitive": None, "box": "box",
                    "cylinder": "cylinder", "sphere": "sphere"}
# every supported <collision> mode (single source of truth for the CLI + web)
COLLISION_MODES = ("copy", "hull", "coacd", *_PRIMITIVE_MODES)
# modes that produce a viewer preview / need generation (everything but 'copy')
PREVIEW_COLLISION_MODES = ("hull", "coacd", *_PRIMITIVE_MODES)


def skrobot_primitive_available():
    """True if scikit-robot exposes the per-mesh primitive-fitting API (the
    release that ships ``skrobot.utils.primitive_fitting``)."""
    import importlib.util
    try:
        return importlib.util.find_spec(
            "skrobot.utils.primitive_fitting") is not None
    except (ImportError, ValueError):
        return False


def _fit_collision_primitive(src, primitive_type):
    """``(params, M)`` for the source mesh ``src`` fitted to a URDF primitive:
    ``params`` is scikit-robot's primitive dict (type + dimensions) and ``M`` is
    the 4x4 primitive-origin transform in the mesh (source) frame.  ``oriented=
    None`` lets scikit-robot pick the tighter of an axis-aligned / oriented fit
    (and, when ``primitive_type`` is None, the best of box/cylinder/sphere) by
    voxel IoU."""
    import trimesh
    from skrobot.utils import fit_primitive_to_mesh, primitive_params_to_origin
    mesh = _load_mesh_metres(src)
    params = fit_primitive_to_mesh(mesh, primitive_type=primitive_type,
                                   oriented=None)
    xyz, rpy = primitive_params_to_origin(params)
    M = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    M[:3, 3] = xyz
    return params, M


def _primitive_geometry_el(params):
    """A URDF ``<geometry>`` element for a fitted primitive dict.  The shape's
    centre + rotation are carried by the ``<collision>``'s ``<origin>``, so the
    geometry here is axis-aligned at the origin."""
    geo = ET.Element("geometry")
    t = params["type"]
    if t == "box":
        ET.SubElement(geo, "box").set(
            "size", " ".join(_fmt_num(v) for v in params["extents"]))
    elif t == "sphere":
        ET.SubElement(geo, "sphere").set("radius", _fmt_num(params["radius"]))
    elif t in ("cylinder", "capsule"):
        cyl = ET.SubElement(geo, "cylinder")
        cyl.set("radius", _fmt_num(params["radius"]))
        length = float(params["height"])
        if t == "capsule":            # URDF has no capsule; fold the caps in
            length += 2 * float(params["radius"])
        cyl.set("length", _fmt_num(length))
    else:
        raise ValueError(f"unsupported primitive type: {t!r}")
    return geo


def _collision_preview_meshes(src, mode, quality, cache_dir):
    """``[trimesh, ...]`` in the source frame for a link's collision preview:
    the convex STL part(s) for ``coacd``/``hull``, or the single fitted primitive
    mesh for a primitive mode."""
    import trimesh

    if mode in _PRIMITIVE_MODES:
        from skrobot.utils.primitive_fitting import create_primitive_mesh
        params, _M = _fit_collision_primitive(src, _PRIMITIVE_MODES[mode])
        return [create_primitive_mesh(params)]
    return [trimesh.load(io.BytesIO(d), file_type="stl")
            for d in _collision_part_stls(src, mode, quality, cache_dir)]


# a fixed high-contrast palette so adjacent convex parts read apart in the viewer.
# NO red/salmon: red is the self-collision highlight colour, so a red preview part
# (a single-part hull is always palette[0]) would read as a collision.
_COLLISION_PALETTE = (
    (55, 126, 184), (77, 175, 74), (152, 78, 163), (255, 127, 0),
    (210, 210, 40), (166, 86, 40), (247, 129, 191), (102, 194, 165),
    (141, 160, 203), (153, 153, 153),
)


def _origin_matrix(origin_el):
    """4x4 transform for a URDF ``<origin>`` element (xyz + fixed-axis rpy), or
    identity when absent -- to bake a ``<collision>``'s origin into its part
    vertices so a preview mesh sits right when attached at the link frame."""
    import numpy as np
    import trimesh

    if origin_el is None:
        return np.eye(4)
    xyz = [float(v) for v in (origin_el.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (origin_el.get("rpy") or "0 0 0").split()]
    m = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    m[:3, 3] = xyz[:3]
    return m


def _fmt_num(v):
    """Format a float for a URDF attribute, snapping FP noise to exact 0."""
    v = float(v)
    if abs(v) < 1e-9:
        return "0"
    return repr(round(v, 9))


def _set_origin_el(el, M):
    import numpy as np

    from .geometry import matrix_to_xyz_rpy
    o = el.find("origin")
    if o is None:
        o = ET.SubElement(el, "origin")
    xyz, rpy = matrix_to_xyz_rpy(np.asarray(M, float))
    o.set("xyz", " ".join(_fmt_num(v) for v in xyz))
    o.set("rpy", " ".join(_fmt_num(v) for v in rpy))


def _tf_marker_scale(root, pkg_dir, own_pkgs, cache):
    """TF axis ("Marker Scale") sized to the model: the median visual-mesh
    diagonal (a typical part's size), floored at 0.02 m and capped at 1.0, so
    RViz triads match the parts instead of dwarfing a small robot."""
    sizes = []
    for link in root.findall("link"):
        vis = link.find("visual")
        me = vis.find(".//mesh") if vis is not None else None
        if me is None:
            continue
        _b, src, _e = _resolve_mesh(pkg_dir, me.get("filename"),
                                    own_pkgs=own_pkgs)
        if src is None:
            continue
        res = cache.measure(src)
        if res is not None and res[0] > 0.0:
            sizes.append(res[0])
    if not sizes:
        return 1.0
    sizes.sort()
    return round(min(1.0, max(0.02, sizes[len(sizes) // 2])), 4)


def _bake_origins(root, pkg_dir, own_pkgs, cache):
    """Normalise link origins for the portable package (MuJoCo-friendly):

    * bake every ``<visual>``/``<collision>`` ``<origin>`` into its mesh and set
      the origin to exactly ``0 0 0`` (skrobot ``--force-zero-origin``), which
      also clears the floating-point noise tools dislike, and
    * RE-CENTRE each FIXED link's frame on its visual geometry, so a sub-assembly
      whose CAD origin sits far from its parts (the frame floating ~1 m away)
      gets its frame moved onto the mesh -- the parent joint, child joints and
      inertial origin absorb the shift, so every world pose is unchanged.

    Returns ``(bake, tf_scale)`` where ``bake`` is ``{id(mesh_el): 4x4}`` so the
    mesh converter bakes the matching transform into each mesh's vertices, and
    ``tf_scale`` is a TF axis ("Marker Scale") sized to the model (the median
    visual-mesh diagonal, capped at 1.0) so RViz triads do not dwarf small parts.
    Best effort: a link whose mesh cannot be loaded keeps its (zeroed) origin and
    is not re-centred."""
    import numpy as np
    bake = {}
    sizes = []
    child_joint, child_lists = {}, {}
    for j in root.findall("joint"):
        c, p = j.find("child").get("link"), j.find("parent").get("link")
        child_joint[c] = j
        child_lists.setdefault(p, []).append(j)

    for link in root.findall("link"):
        name = link.get("name")
        pj = child_joint.get(name)
        fixed = pj is not None and pj.get("type") == "fixed"
        vis = link.find("visual")
        v_mat = _origin_matrix(vis.find("origin")) if vis is not None \
            else np.eye(4)
        d = np.zeros(3)
        if vis is not None:
            me = vis.find(".//mesh")
            if me is not None:
                _b, src, _e = _resolve_mesh(pkg_dir, me.get("filename"),
                                            own_pkgs=own_pkgs)
                if src is not None:
                    res = cache.measure(src)   # size + centroid, once (disk-cached)
                    if res is not None:
                        size, centroid = res
                        sizes.append(size)
                        if fixed:              # centroid in link frame
                            d = (v_mat @ np.append(centroid, 1.0))[:3]
        moved = float(np.linalg.norm(d)) > 1e-9
        t_neg = np.eye(4)
        t_neg[:3, 3] = -d
        if moved:
            # F' = F . T(+d): parent joint . T(+d); child joints, inertial . T(-d)
            t_pos = np.eye(4)
            t_pos[:3, 3] = d
            _set_origin_el(pj, _origin_matrix(pj.find("origin")) @ t_pos)
            for cj in child_lists.get(name, []):
                _set_origin_el(cj, t_neg @ _origin_matrix(cj.find("origin")))
            inel = link.find("inertial")
            if inel is not None:
                _set_origin_el(inel, t_neg @ _origin_matrix(inel.find("origin")))
        for ctx in ("visual", "collision"):
            for block in link.findall(ctx):
                vb = _origin_matrix(block.find("origin"))
                mb = t_neg @ vb                # bake into the mesh
                # only bake when it actually moves vertices; an identity (or pure
                # FP-noise) origin just gets zeroed -- no lossy mesh round-trip,
                # so a verbatim .dae copy (+ its sidecar textures) is preserved
                if not np.allclose(mb, np.eye(4), atol=1e-7):
                    for mesh_el in block.iter("mesh"):
                        bake[id(mesh_el)] = mb
                o = block.find("origin")
                if o is None:
                    o = ET.SubElement(block, "origin")
                o.set("xyz", "0 0 0")
                o.set("rpy", "0 0 0")
    sizes = [s for s in sizes if s > 0.0]
    tf_scale = 1.0
    if sizes:
        sizes.sort()
        tf_scale = round(min(1.0, max(0.02, sizes[len(sizes) // 2])), 4)
    return bake, tf_scale


def collision_preview_glbs(pkg_dir, robot_name, quality="balanced", progress=None,
                       urdf_path=None, should_cancel=None, on_start=None,
                       max_workers=None, mode="coacd"):
    """Build a colour-coded convex-collision preview GLB per link under
    ``meshes/.coacd_cache/preview/`` (the convex part(s), each a distinct colour,
    the collision ``<origin>`` baked in so the GLB sits at the link frame).

    ``mode`` selects the geometry: ``"coacd"`` (default) runs CoACD approximate
    convex decomposition into many parts -- shares the on-disk part cache with
    the ROS export, so generating the preview also warms a later ``collision=
    'coacd'`` export; ``"hull"`` produces a single convex hull per link (fast,
    no optional dependency); ``"primitive"`` / ``"box"`` / ``"cylinder"`` /
    ``"sphere"`` fit a native URDF primitive per link (needs scikit-robot).

    Links are processed CONCURRENTLY in a thread pool -- CoACD releases the GIL,
    so this scales with cores (measured ~4x on a 12-core box).  ``max_workers``
    defaults to ``cpu_count - 2`` (capped at 8).

    ``on_start(link_name)`` (optional) is called when a link's decomposition
    begins -- with the pool, several links are in flight at once, so a watcher
    can track the set.  ``progress(done, total, link_name, rel_glb_or_None)`` is
    called when a link FINISHES (``done`` = links finished so far, ``rel_glb``
    the package-relative GLB path or None for links with no mesh).  Both may be
    called from worker threads; keep them quick + thread-safe.  Returns
    ``{link_name: rel_glb}`` for the links that produced one.

    ``should_cancel`` is an optional zero-arg predicate; once it returns true no
    further links are started (in-flight ones finish -- CoACD can't be
    interrupted mid-link).  Raises a clear error if CoACD is unavailable."""
    import threading
    from concurrent.futures import ThreadPoolExecutor, as_completed

    import numpy as np
    import trimesh

    if mode == "coacd":
        if quality not in _COACD_PRESETS:
            raise ValueError(f"unsupported coacd_quality: {quality!r} "
                             f"(use one of {sorted(_COACD_PRESETS)})")
        if not coacd_available():
            raise ValueError(
                "CoACD collision decomposition needs the optional 'coacd' "
                "package -- install it with: pip install coacd")
    elif mode in _PRIMITIVE_MODES:
        if not skrobot_primitive_available():
            raise ValueError(
                "primitive collision fitting needs scikit-robot with the "
                "per-mesh primitive API -- install/upgrade it with: pip install "
                "-U scikit-robot")
    elif mode != "hull":
        raise ValueError(f"unsupported collision mode: {mode!r} "
                         f"(use one of {PREVIEW_COLLISION_MODES})")
    cache_dir = os.path.join(pkg_dir, "meshes", ".coacd_cache")
    preview_dir = os.path.join(cache_dir, "preview")
    os.makedirs(preview_dir, exist_ok=True)
    own_pkgs = _own_pkg_names(pkg_dir)
    urdf_path = urdf_path or os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    root = ET.parse(urdf_path).getroot()
    links = root.findall("link")
    total = len(links)
    if max_workers is None:
        max_workers = max(1, min(8, (os.cpu_count() or 2) - 2))

    # resolve each link's collision mesh sources up front (cheap, no CoACD), so
    # the pool tasks are pure decomposition work
    mesh_links, plain_links = [], []
    for i, link in enumerate(links):
        name = link.get("name") or f"link{i}"
        blocks = []
        for block in link.findall("collision"):
            ms = list(block.iter("mesh"))
            if len(ms) != 1:
                continue
            _b, src, _e = _resolve_mesh(pkg_dir, ms[0].get("filename"),
                                        own_pkgs=own_pkgs)
            if src:
                blocks.append((src, _origin_matrix(block.find("origin"))))
        (mesh_links if blocks else plain_links).append((name, blocks))

    out = {}
    lock = threading.Lock()
    state = {"done": 0}

    def _finish(name, rel):
        with lock:
            state["done"] += 1
            d = state["done"]
            if rel:
                out[name] = rel
        if progress:
            progress(d, total, name, rel)

    def _decompose(item):
        name, blocks = item
        if should_cancel and should_cancel():
            return name, None
        if on_start:
            on_start(name)
        scene = trimesh.Scene()
        part_i = 0
        for src, mat in blocks:
            for part in _collision_preview_meshes(src, mode, quality, cache_dir):
                part.apply_transform(mat)
                rgb = _COLLISION_PALETTE[part_i % len(_COLLISION_PALETTE)]
                part.visual = trimesh.visual.ColorVisuals(
                    mesh=part,
                    face_colors=np.tile([*rgb, 255], (len(part.faces), 1)))
                scene.add_geometry(part)
                part_i += 1
        if not part_i:
            return name, None
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
        with open(os.path.join(preview_dir, safe + ".glb"), "wb") as f:
            f.write(scene.export(file_type="glb"))
        return name, f"meshes/.coacd_cache/preview/{safe}.glb"

    # links without a convertible mesh just advance the counter
    for name, _blocks in plain_links:
        _finish(name, None)

    with ThreadPoolExecutor(max_workers=max_workers) as ex:
        futs = {ex.submit(_decompose, m): m for m in mesh_links}
        for fut in as_completed(futs):
            name, rel = fut.result()
            _finish(name, rel)
            if should_cancel and should_cancel():
                for f in futs:        # drop not-yet-started links (running ones
                    f.cancel()        # can't be killed; the pool waits for them)
                break
    return out


def _collada_meshes(mesh):
    """Meshes to hand to trimesh's DAE writer, with texture colours baked into
    material-coloured geometry because COLLADA texture export is not supported."""
    import numpy as np
    from trimesh.visual import color as vcolor

    if getattr(mesh.visual, "kind", None) == "texture":
        mesh.visual = mesh.visual.to_color()
    kind = getattr(mesh.visual, "kind", None)
    if kind == "vertex":
        face_colors = vcolor.vertex_to_face_color(mesh.visual.vertex_colors,
                                                  mesh.faces)
    elif kind == "face":
        face_colors = mesh.visual.face_colors
    else:
        return [mesh]

    face_colors = np.array(face_colors, dtype=np.uint8)   # copy: clamped below
    if len(face_colors) != len(mesh.faces):
        return [mesh]
    # A pure-black COLLADA material (diffuse 0,0,0) renders as a RED fallback in
    # RViz2/Ogre instead of black; lift near-black faces to a dark grey so black
    # CAD parts stay (almost) black rather than flashing red.  Alpha and any
    # genuinely coloured face are left untouched.
    if face_colors.shape[1] >= 3:
        black = (face_colors[:, :3] < 12).all(axis=1)
        face_colors[black, :3] = 12
    unique, inverse = np.unique(face_colors, axis=0, return_inverse=True)
    if len(unique) <= 1:
        mesh.visual.face_colors = np.tile(unique[0], (len(mesh.faces), 1))
        return [mesh]

    out = []
    for i, rgba in enumerate(unique):
        faces = np.nonzero(inverse == i)[0]
        part = mesh.submesh([faces], append=True, repair=False)
        part.visual.face_colors = np.tile(rgba, (len(part.faces), 1))
        part.units = mesh.units
        out.append(part)
    return out


def _rgba_attr(rgba):
    """uint8 ``[r,g,b,a]`` -> a URDF ``<color rgba>`` string of 0..1 floats."""
    vals = [max(0.0, min(1.0, float(c) / 255.0)) for c in rgba]
    return " ".join(f"{v:.4g}" for v in vals)


def _dae_sidecar_images(dae_path):
    """Relative sidecar image paths a COLLADA ``.dae`` references via
    ``<init_from>`` (textures), so a verbatim copy can ship them too.  Best
    effort: ``[]`` on any parse trouble."""
    try:
        root = ET.parse(dae_path).getroot()
    except Exception:
        return []
    out = []
    for el in root.iter():
        if not el.tag.endswith("init_from"):
            continue
        ref = (el.text or "").strip()
        if not ref:                              # COLLADA 1.5 nests <ref>
            child = next((c for c in el if c.tag.endswith("ref")), None)
            ref = (child.text or "").strip() if child is not None else ""
        ref = ref.replace("\\", "/")
        if ref.startswith("file://"):
            ref = ref[len("file://"):]
        # only relative sidecars next to the mesh; skip absolute (incl. Windows
        # drive), URL and parent-escaping refs
        if (ref and "://" not in ref and not ref.startswith("/")
                and not re.match(r"^[A-Za-z]:", ref) and ".." not in ref):
            out.append(ref)
    return out


def _read_mass_only(pkg_dir):
    """The mass-only link names build() persisted beside the package (final URDF
    link names), or an empty set when there are none / no sidecar."""
    side = os.path.join(pkg_dir, "mass_only.yaml")
    if not os.path.isfile(side):
        return set()
    try:
        import yaml
        with open(side, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return set(data or ())
    except Exception:
        return set()


def _own_pkg_names(pkg_dir):
    """The names by which the opened package may refer to its OWN meshes: the
    ROS manifest ``<name>`` (authoritative) plus the directory basename (a
    fallback / second accepted alias).  A ``package://<name>/...`` ref counts as
    ours only when ``<name>`` is one of these."""
    names = {os.path.basename(os.path.normpath(pkg_dir))}
    mani = os.path.join(pkg_dir, "package.xml")
    if os.path.exists(mani):
        try:
            n = ET.parse(mani).getroot().findtext("name")
            if n and n.strip():
                names.add(n.strip())
        except Exception:
            pass
    return names


def _resolve_mesh(pkg_dir, ref, own_pkgs=None):
    """``(base, source_path, ext)`` for a URDF mesh ref, or ``(None, ...)`` when it
    is not one of our convertible meshes; ``source_path`` is None if the file is
    missing.

    Resolve the ref to a path-within-the-package, preserving subdirectories:
      - package://<pkg>/<rest>      -> <rest>  (a URDF opened for re-export)
      - relative '../meshes/x.dae'  -> 'meshes/x.dae'  (the CAD working URDF)
    Other schemes (http://, file://) are external -- left untouched.  A
    ``package://<pkg>`` ref is only treated as ours when ``<pkg>`` is in
    ``own_pkgs`` (the opened package's names); a ref to ANOTHER ROS package is an
    external dependency and is left as-is even if a same-named file happens to
    exist here.
    """
    if not ref:
        return None, None, None
    ref = ref.replace("\\", "/")           # tolerate Windows-style separators
    is_pkg = "://" in ref
    root = os.path.normpath(pkg_dir)
    if is_pkg:
        if not ref.startswith("package://"):
            return None, None, None
        ref_pkg, _, rel = ref.split("://", 1)[1].partition("/")
        if own_pkgs is not None and ref_pkg not in own_pkgs:
            return None, None, None        # another package's mesh -- leave it
        base_dir = root                    # package:// is relative to the pkg root
    else:
        if ref.startswith("/") or re.match(r"^[A-Za-z]:", ref):
            return None, None, None        # absolute path -> external, leave it
        rel = ref                          # keep '../' so the escape check is real
        base_dir = os.path.join(root, "urdf")   # the CAD/working URDF lives here
    base, ext = os.path.splitext(os.path.basename(rel))
    if ext.lower() not in _CONVERTIBLE:
        return None, None, None
    src = os.path.normpath(os.path.join(base_dir, *rel.split("/")))
    try:
        escapes = os.path.commonpath([root, src]) != root
    except ValueError:                     # different drive (Windows) -> external
        escapes = True
    if escapes:
        return None, None, None            # a '..' escaping the package -> external
    if not os.path.exists(src):
        # the URDF may name one extension while only the other was produced
        # (a sub-assembly composed to .glb, say); accept whichever exists
        stem = os.path.splitext(src)[0]
        for alt_ext in _CONVERTIBLE:
            if os.path.exists(stem + alt_ext):
                src = stem + alt_ext
                break
        else:
            # external refs were already returned above; reaching here means an
            # OWN mesh (relative, or package:// matching own_pkg) is missing -- a
            # real error the caller surfaces as 'no source mesh'
            return base, None, ext
    return base, src, ext


def ros_pkg_name(robot_name, pkg_name=None):
    """The ROS package name to emit: an explicit ``pkg_name`` (validated) or the
    default ``<robot_name>_description``.  catkin/ament require a name starting
    with a lowercase letter and containing only lowercase, digits and
    underscores.

    The default is SANITIZED to that form (a SolidWorks assembly like 'Assem1'
    -> 'assem1_description'), since the assembly name often has capitals; an
    EXPLICIT name is validated strictly and rejected if malformed, so a typo
    surfaces instead of silently changing."""
    if not pkg_name:
        base = re.sub(r"[^a-z0-9_]", "_", robot_name.lower()).strip("_")
        if not base or not base[0].isalpha():
            base = "robot_" + base if base else "robot"
        return f"{base}_description"
    pkg = pkg_name.strip()
    if not re.fullmatch(r"[a-z][a-z0-9_]*", pkg):
        raise ValueError(
            f"invalid ROS package name {pkg_name!r}: must start with a "
            "lowercase letter and contain only lowercase letters, digits and "
            "underscores (e.g. 'bambu_a1_description')")
    return pkg


def ros_urdf_stem(pkg, urdf_name=None):
    """The URDF file stem to emit inside the package: an explicit ``urdf_name``
    (validated) or, when blank, the package name itself -- so the package ships
    ``<pkg>/urdf/<pkg>.urdf`` by default instead of leaking the SolidWorks
    assembly name."""
    if not urdf_name:
        return pkg
    stem = urdf_name.strip()
    if stem.lower().endswith(".urdf"):
        stem = stem[:-5]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", stem):
        raise ValueError(
            f"invalid URDF name {urdf_name!r}: use letters, digits, '_', '-' "
            "or '.' and start with a letter or digit (e.g. 'bambu_a1')")
    return stem


def ros_mesh_dir(mesh_dir=None):
    """The package-relative directory the exported meshes go in (and that the
    URDF's ``package://<pkg>/...`` refs point at): an explicit ``mesh_dir`` or the
    default ``meshes``.

    The relative path from the package root to the meshes differs between robots
    (``meshes``, ``urdf/mesh``, ...), so it is settable.  A leading/trailing slash
    is trimmed and back-slashes are accepted as separators; the path must stay
    inside the package -- no absolute path and no ``..`` segment -- and each
    segment may use only letters, digits, ``_``, ``-`` and ``.``."""
    if not mesh_dir or not mesh_dir.strip():
        return "meshes"
    raw = mesh_dir.strip().replace("\\", "/").strip("/")
    parts = raw.split("/") if raw else []
    if not parts or any(
            p in ("", ".", "..") or not re.fullmatch(r"[A-Za-z0-9_.-]+", p)
            for p in parts):
        raise ValueError(
            f"invalid mesh directory {mesh_dir!r}: use a package-relative path "
            "of letters, digits, '_', '-', '.' separated by '/' (e.g. 'meshes' "
            "or 'urdf/mesh')")
    return "/".join(parts)


def _iter_sources(root, pkg_dir, own_pkgs, contexts):
    """Yield the resolved source-mesh path for every convertible ``<mesh>`` under
    the given URDF ``contexts`` (``'visual'`` / ``'collision'``) -- used to warm
    the mesh cache (load + measure + convert) in parallel before the serial
    passes.  External/missing refs are skipped (they have no local source)."""
    for link in root.findall("link"):
        for ctx in contexts:
            for block in link.findall(ctx):
                for mesh in block.iter("mesh"):
                    _b, src, _e = _resolve_mesh(pkg_dir, mesh.get("filename"),
                                                own_pkgs=own_pkgs)
                    if src is not None:
                        yield src


class ExportCancelled(Exception):
    """Raised out of :func:`build_ros_description` when ``should_cancel`` turns
    true, so a caller (the editor's async export) can report a clean cancel
    rather than a half-built package."""


def build_ros_description(pkg_dir, robot_name, email="auto@example.com",
                          ctx_fmt=_CTX_FMT, ros_version=1, pkg_name=None,
                          urdf_name=None, colors=None, collision="copy",
                          coacd_quality="balanced", merge_fixed=False,
                          mesh_dir=None, loop_closures=None,
                          zero_origins=True, progress=None, should_cancel=None):
    """``pkg_dir`` (a built package) -> ``[(arcname, bytes), ...]`` for a portable
    ROS package, all behind ``package://`` URLs.  The package is named
    ``pkg_name`` if given (validated, see :func:`ros_pkg_name`), else
    ``<robot_name>_description``.  The URDF inside is named ``urdf_name`` if
    given, else the package name (so ``<pkg>/urdf/<pkg>.urdf`` by default).

    ``mesh_dir`` is the package-relative directory the meshes go in and that the
    URDF's ``package://<pkg>/...`` refs point at (validated, see
    :func:`ros_mesh_dir`); it defaults to ``meshes`` but may be set to match a
    robot whose mesh layout differs (e.g. ``urdf/mesh``).

    ``ctx_fmt`` maps each URDF context to a mesh format; the default emits
    ``<visual>`` as colour ``.dae`` and ``<collision>`` as plain ``.stl`` (the
    usual ROS split).  Pass :data:`GLB_CTX_FMT` for a uniform ``.glb`` package.

    ``colors`` is an optional ``{component link name -> '#RRGGBB'}`` map (the
    mesh basename equals the component link name) that repaints that link's
    ``<visual>`` mesh a solid colour, overriding its CAD colours; collision STL
    is colourless and unaffected.

    ``collision`` chooses how ``<collision>`` geometry is produced: ``'copy'``
    (default) reuses the visual mesh as one STL (the current behaviour);
    ``'hull'`` replaces each ``<collision>``'s mesh with a single convex hull
    STL (the "simple collision" alternative -- one convex shape per link, what
    path planning usually wants); ``'coacd'`` runs CoACD approximate convex
    decomposition, replacing each ``<collision>``'s mesh with a *set* of convex
    part STLs (better for high-fidelity physics); ``'primitive'`` / ``'box'`` /
    ``'cylinder'`` / ``'sphere'`` fit a native URDF primitive per ``<collision>``
    (no mesh file at all -- the lightest collision, ``'primitive'`` auto-picks
    the best of box/cylinder/sphere by voxel IoU).  ``coacd_quality``
    (``'balanced'`` | ``'fine'``) picks the CoACD preset and is ignored for the
    other modes.  ``'coacd'`` needs the optional ``coacd`` package (its absence
    raises a clear error); the primitive modes use ``scikit-robot``, a core
    dependency (a version predating the per-mesh primitive API also raises a
    clear error).  ``'hull'`` has no extra dependency.  CoACD
    decomposition is cached under ``meshes/.coacd_cache``.

    ``ros_version`` picks the build system: ``1`` (default) writes a catkin
    ``package.xml`` (format 2) + ``CMakeLists.txt``; ``2`` writes an ament_cmake
    ``package.xml`` (format 3) + ``CMakeLists.txt`` and bundles a
    ``launch/display.launch.py`` + ``rviz/<urdf>.rviz`` so the package runs with
    ``ros2 launch <name> display.launch.py``.

    ``loop_couplings`` (ROS 2 only) is an optional ``[{driver, followers:[{joint,
    poly}]}]`` list describing closed-loop (four-bar) couplings: when present the
    package additionally ships ``config/loop_couplings.yaml``, a
    ``loop_coupling_relay`` node, and a launch wired GUI -> relay ->
    robot_state_publisher so the loop follows its exact (cubic) coupling -- the
    linear URDF ``<mimic>`` alone tracks a general four-bar only approximately.

    Reads the on-disk ``urdf/<robot_name>.urdf`` (which already carries the
    editor's applied edits).  A missing/unconvertible mesh aborts the export
    before any entries are emitted, so a half-rewritten package never ships.

    ``progress`` -- if given -- is called ``progress(stage, done, total, detail)``
    as work advances, with ``stage`` one of ``"collision"`` (per unique
    CoACD/primitive source warmed) or ``"meshes"`` (per unique mesh converted),
    so a UI can show a live per-stage bar.

    ``should_cancel`` -- if given -- is a zero-arg predicate checked between
    links/meshes; once it returns true the build raises :class:`ExportCancelled`
    (CoACD is not interruptible mid-link, so a cancel lands at the next
    boundary)."""
    if ros_version not in (1, 2):
        raise ValueError(f"unsupported ros_version: {ros_version}")
    if collision not in COLLISION_MODES:
        raise ValueError(f"unsupported collision mode: {collision!r} "
                         f"(use one of {COLLISION_MODES})")
    if collision == "coacd":
        if coacd_quality not in _COACD_PRESETS:
            raise ValueError(
                f"unsupported coacd_quality: {coacd_quality!r} "
                f"(use one of {sorted(_COACD_PRESETS)})")
        if not coacd_available():
            raise ValueError(
                "CoACD collision decomposition needs the optional 'coacd' "
                "package -- install it with: pip install coacd")
    if collision in _PRIMITIVE_MODES and not skrobot_primitive_available():
        raise ValueError(
            "primitive collision fitting needs scikit-robot with the per-mesh "
            "primitive API -- install/upgrade it with: pip install -U "
            "scikit-robot")
    coacd_cache_dir = os.path.join(pkg_dir, "meshes", ".coacd_cache")
    if loop_closures is None:
        # the editor's ZIP export calls us directly (no model); pick up the
        # loop-closure data build() persisted beside the package so the ZIP
        # ships the IK relay node + closure config too
        side = os.path.join(pkg_dir, "loop_closures.yaml")
        if os.path.isfile(side):
            try:
                import yaml
                with open(side, encoding="utf-8") as f:
                    loop_closures = yaml.safe_load(f) or None
                if loop_closures and not loop_closures.get("closures"):
                    loop_closures = None
            except Exception:
                loop_closures = None
    pkg = ros_pkg_name(robot_name, pkg_name)
    urdf_stem = ros_urdf_stem(pkg, urdf_name)
    mesh_rel = ros_mesh_dir(mesh_dir)
    # the opened package's own name(s) -- only package:// refs to one of these
    # are vendored/repointed; refs to other ROS packages are left untouched
    own_pkgs = _own_pkg_names(pkg_dir)
    urdf_path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    # Keep XML comments: the default parser drops them, which both loses the
    # per-link ``<!-- sw2robot ... -->`` provenance and leaves a blank,
    # whitespace-only line where each comment was (its surrounding indentation
    # merges into the link's text).  Comment nodes have a non-string tag, so the
    # findall()/iter() traversals below ignore them.
    _parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(urdf_path, parser=_parser).getroot()
    if should_cancel and should_cancel():     # cancelled before any heavy work
        raise ExportCancelled()
    # Mass-only links (weight kept, geometry dropped) are folded into their fixed
    # parent on every export, independent of merge_fixed; build() persisted their
    # (final) names beside the package so this detached step knows them.
    mass_only = _read_mass_only(pkg_dir)
    from .merge import merge_fixed_links
    if merge_fixed:
        # lump ALL fixed-joint children with geometry into their parents (and the
        # mass-only ones too) BEFORE the mesh conversion / collision loop runs
        merge_fixed_links(root, force_merge=mass_only)
    elif mass_only:
        # fold ONLY the mass-only links, leaving the rest of the fixed tree intact
        merge_fixed_links(root, only=mass_only)
    colors = colors or {}
    # load each source mesh ONCE and reuse it (size measurement + visual .dae +
    # collision .stl all share one load); disk-cache converted bytes + the size
    # measurement so a re-export of an unchanged model touches no trimesh at all
    cache = _MeshCache(os.path.join(pkg_dir, "meshes", ".mesh_cache"))
    # bake visual/collision origins into meshes (-> origin 0 0 0, MuJoCo-ready)
    # and re-centre fixed links on their geometry; {id(mesh_el): 4x4 transform}
    if zero_origins:
        bake, tf_scale = _bake_origins(root, pkg_dir, own_pkgs, cache)
    else:
        bake, tf_scale = {}, _tf_marker_scale(root, pkg_dir, own_pkgs, cache)

    files = []
    used_names = set()
    errors = []

    def _report(stage, done, total, detail=""):
        # progress(stage, done, total, detail): stage in {"collision", "meshes"}.
        # Optional; a caller (the editor's async export) maps it onto its UI.
        if progress:
            try:
                progress(stage, done, total, detail)
            except Exception:
                pass

    def _ckcancel():
        # cooperative cancel: checked between links/meshes (CoACD itself is not
        # interruptible mid-link, so a cancel lands at the next boundary)
        if should_cancel and should_cancel():
            raise ExportCancelled()
    # convex collision parts ('coacd' = many, 'hull' = one): keyed by source +
    # mode + quality so a source shared by several links is built once (CoACD is
    # also disk-cached under .coacd_cache)
    coll_done = {}
    # fitted primitives (primitive/box/cylinder/sphere): (params, M) keyed by
    # source + mode so a source shared by several links is fitted once
    prim_done = {}

    def _emit_collision(base, src):
        # build the convex collision part STL(s) for a source mesh per the
        # selected `collision` mode; returns the emitted mesh names (one per
        # part -- a single hull, or N CoACD parts), or None on error.
        key = (os.path.abspath(src), collision, coacd_quality)
        if key in coll_done:
            return coll_done[key]
        try:
            blobs = _collision_part_stls(src, collision, coacd_quality,
                                         coacd_cache_dir)
        except Exception as e:
            errors.append(f"{base}: {collision} collision failed ({e!r})")
            return None
        names = []
        for i, data in enumerate(blobs):
            # hull is one part per link, so name it for what it is; CoACD parts
            # keep their indexed names
            stem = f"{base}_collision_hull" if collision == "hull" \
                else f"{base}_collision_{i}"
            name, j = f"{stem}.stl", 1
            while name in used_names:
                j += 1
                name = f"{stem}_{j}.stl"
            used_names.add(name)
            files.append((f"{pkg}/{mesh_rel}/{name}", data))
            names.append(name)
        coll_done[key] = names
        return names

    def _expand_collision(link):
        # replace each <collision> whose single mesh is a convertible source with
        # the convex collision part(s) (one block per part -- a single hull, or N
        # CoACD parts), preserving its <origin>.  Blocks we can't convert
        # (primitive geometry, external/missing mesh, multi-mesh) are left to the
        # normal per-mesh path below.
        for block in list(link.findall("collision")):
            meshes = list(block.iter("mesh"))
            if len(meshes) != 1:
                continue                   # primitive or multi-mesh -- leave it
            base, src, _ext = _resolve_mesh(pkg_dir, meshes[0].get("filename"),
                                            own_pkgs=own_pkgs)
            if base is None:
                continue                   # external ref -- leave as-is
            if src is None:
                errors.append(f"no source mesh for '{base}'")
                continue
            names = _emit_collision(base, src)
            if not names:
                continue
            m_bake = bake.get(id(meshes[0]))
            idx = list(link).index(block)
            link.remove(block)
            # deep-copy the original block per part so <origin> + formatting carry
            for k, name in enumerate(names):
                nb = copy.deepcopy(block)
                if m_bake is not None:         # parts are in the source frame:
                    _set_origin_el(nb, m_bake)  # carry the bake transform here
                next(nb.iter("mesh")).set(
                    "filename", f"package://{pkg}/{mesh_rel}/{name}")
                link.insert(idx + k, nb)

    def _fit_primitive_cached(base, src):
        # fit (once per source+mode) the primitive for `src`; (params, M) or None
        key = (os.path.abspath(src), collision)
        if key in prim_done:
            return prim_done[key]
        try:
            res = _fit_collision_primitive(src, _PRIMITIVE_MODES[collision])
        except Exception as e:
            errors.append(f"{base}: {collision} collision failed ({e!r})")
            return None
        prim_done[key] = res
        return res

    def _expand_collision_primitive(link):
        # replace each convertible <collision>'s mesh with a fitted URDF primitive
        # (<box>/<cylinder>/<sphere>), no mesh file: swap the <geometry> element
        # and set the block <origin> to the primitive pose in the link frame.
        # Blocks we can't fit (multi-mesh, external/missing, primitive already)
        # are left untouched.
        import numpy as np
        for block in list(link.findall("collision")):
            meshes = list(block.iter("mesh"))
            if len(meshes) != 1:
                continue                   # primitive or multi-mesh -- leave it
            base, src, _ext = _resolve_mesh(pkg_dir, meshes[0].get("filename"),
                                            own_pkgs=own_pkgs)
            if base is None:
                continue                   # external ref -- leave as-is
            if src is None:
                errors.append(f"no source mesh for '{base}'")
                continue
            res = _fit_primitive_cached(base, src)
            if res is None:
                continue
            params, M = res
            # the primitive is fitted in the source frame; place it in the link
            # frame via the bake transform (origins were zeroed) or, when origins
            # are kept, via the block's own <origin>
            m_bake = bake.get(id(meshes[0]))
            base_M = m_bake if m_bake is not None \
                else _origin_matrix(block.find("origin"))
            geo = block.find("geometry")
            new_geo = _primitive_geometry_el(params)
            if geo is not None:                # stdlib ElementTree: no .replace()
                gi = list(block).index(geo)
                block.remove(geo)
                block.insert(gi, new_geo)
            else:
                block.append(new_geo)
            _set_origin_el(block, np.asarray(base_M, float) @ M)

    _warm_lock = threading.Lock()
    _warm_state = {"n": 0}

    def _warm_tick(src, total):
        with _warm_lock:
            _warm_state["n"] += 1
            done = _warm_state["n"]
        _report("collision", done, total, os.path.basename(src))

    if collision == "coacd":
        # CoACD is the heaviest per-mesh op; warm its disk cache for every unique
        # collision source IN PARALLEL (CoACD releases the GIL) so the serial
        # expansion below is all cache hits.  'hull' is fast and not disk-cached,
        # so it needs no warming -- _emit_collision builds it inline.
        _coacd_srcs = list(set(
            _iter_sources(root, pkg_dir, own_pkgs, ("collision",))))
        _report("collision", 0, len(_coacd_srcs))

        def _warm_coacd(src):
            if should_cancel and should_cancel():
                return                      # drain the pool fast once cancelled
            try:
                _coacd_part_stls(src, coacd_quality, coacd_cache_dir)
            except Exception:
                pass                        # the real error resurfaces in _emit_collision
            _warm_tick(src, len(_coacd_srcs))
        _parallel_cancellable(_warm_coacd, _coacd_srcs, should_cancel)
    elif collision in _PRIMITIVE_MODES:
        # primitive fitting voxelises the mesh (auto mode scores several
        # candidates by IoU) -- a few hundred ms per unique source; warm them in
        # parallel so the serial expansion below is all cache hits
        _prim_srcs = list(set(
            _iter_sources(root, pkg_dir, own_pkgs, ("collision",))))
        _report("collision", 0, len(_prim_srcs))

        def _warm_prim(src):
            if should_cancel and should_cancel():
                return                      # drain the pool fast once cancelled
            key = (os.path.abspath(src), collision)
            if key not in prim_done:
                try:
                    prim_done[key] = _fit_collision_primitive(
                        src, _PRIMITIVE_MODES[collision])
                except Exception:
                    pass                # the real error resurfaces in expansion
            _warm_tick(src, len(_prim_srcs))
        _parallel_cancellable(_warm_prim, _prim_srcs, should_cancel)

    def _verbatim(job):
        # already the target format, no colour override, no bake: the source is
        # shipped byte-for-byte (no trimesh round-trip), so it needs no load
        return (job["src"].lower().endswith(f".{job['fmt']}")
                and not job["color"] and job["transform"] is None)

    # ---- phase 1: plan conversions, allocating names in tree order -----------
    # one job per unique (src, fmt, colour, bake) key; mesh elements sharing a key
    # share its single conversion + output name (so a source reused by N links is
    # loaded + converted once).  Names are allocated here in deterministic tree
    # order; phase 2 then produces each job's bytes exactly once.
    jobs = {}                              # key -> job dict
    job_order = []                         # keys, first-seen order (file order)
    _ckcancel()                        # cancel requested during collision warm?
    for link in root.findall("link"):
        _ckcancel()
        # colour overrides are keyed by LINK name in URDF-input mode and by the
        # component/mesh basename in the CAD path -- accept either
        link_color = colors.get(link.get("name"))
        if collision in ("coacd", "hull"):
            _expand_collision(link)
        elif collision in _PRIMITIVE_MODES:
            _expand_collision_primitive(link)
        for ctx, fmt in ctx_fmt:
            if ctx == "collision" and collision != "copy":
                continue                   # handled by the collision expanders
            for block in link.findall(ctx):
                for mesh in block.iter("mesh"):
                    base, src, _ext = _resolve_mesh(pkg_dir, mesh.get("filename"),
                                                    own_pkgs=own_pkgs)
                    if base is None:
                        continue           # not a convertible ref -- leave as-is
                    if src is None:
                        errors.append(f"no source mesh for '{base}'")
                        continue
                    color = link_color or colors.get(base)
                    transform = bake.get(id(mesh))
                    tkey = None if transform is None else tuple(
                        round(float(x), 7) for x in transform.flatten())
                    key = (os.path.abspath(src), fmt, color, tkey)
                    job = jobs.get(key)
                    if job is None:
                        name, i = f"{base}.{fmt}", 1
                        while name in used_names:   # distinct source, same basename
                            i += 1
                            name = f"{base}_{i}.{fmt}"
                        used_names.add(name)
                        job = jobs[key] = {"base": base, "src": src, "fmt": fmt,
                                           "color": color, "transform": transform,
                                           "name": name}
                        job_order.append(key)
                    mesh.set("filename",
                             f"package://{pkg}/{mesh_rel}/{job['name']}")
                    # STL carries no colour, so a <visual> STL would render grey
                    # in RViz.  Emit a URDF <material> instead: the per-link colour
                    # override if set, else a default colour.  dae/glb keep their
                    # colour in the mesh and get no <material>.
                    if ctx == "visual" and fmt == "stl" \
                            and block.find("material") is None:
                        rgba = _hex_to_rgba(color)
                        if rgba is None:
                            rgba = _DEFAULT_VISUAL_RGBA
                        mat = ET.SubElement(block, "material")
                        # name off the UNIQUE mesh output name, not the raw
                        # basename: two links whose visual meshes come from
                        # different sources sharing a basename (part.stl vs
                        # part_2.stl) would otherwise both emit
                        # <material name="part_material">, and urdfdom/RViz
                        # treat material names as a global map -- the second
                        # link would inherit the first's colour.
                        mat.set("name", f"{job['name'].rsplit('.', 1)[0]}_material")
                        ET.SubElement(mat, "color").set("rgba", _rgba_attr(rgba))

    if errors:
        raise RuntimeError("ROS description export failed: " + "; ".join(errors))

    # ---- phase 2: produce the mesh bytes (serial; the cache makes it cheap) ---
    # each unique job converts once; the disk cache makes a re-export near-instant
    # and the in-memory load-once means visual + collision share a single load
    _n_jobs = len(job_order)
    for _i, key in enumerate(job_order):
        _ckcancel()
        job = jobs[key]
        _report("meshes", _i, _n_jobs, job["base"])
        src, fmt, color, transform = (job["src"], job["fmt"], job["color"],
                                      job["transform"])
        try:
            if _verbatim(job):
                # ship the mesh verbatim + any sidecar textures a .dae references
                with open(src, "rb") as f:
                    data = f.read()
                if fmt == "dae":
                    dae_dir = os.path.dirname(src)
                    for rel in _dae_sidecar_images(src):
                        img = os.path.normpath(os.path.join(dae_dir, rel))
                        arc = f"{pkg}/{mesh_rel}/{rel}"
                        if os.path.exists(img) and arc not in used_names:
                            with open(img, "rb") as fh:
                                files.append((arc, fh.read()))
                            used_names.add(arc)
            else:
                data = cache.convert(src, fmt, color, transform)
        except Exception as e:
            errors.append(f"{job['base']}: {fmt} convert failed ({e!r})")
            continue
        files.append((f"{pkg}/{mesh_rel}/{job['name']}", data))
    _report("meshes", _n_jobs, _n_jobs)

    if errors:
        raise RuntimeError("ROS description export failed: " + "; ".join(errors))

    new_urdf = '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")
    if not new_urdf.endswith("\n"):
        new_urdf += "\n"
    files.append((f"{pkg}/urdf/{urdf_stem}.urdf", new_urdf.encode("utf-8")))

    if ros_version == 2:
        # the root (first) link is the natural RViz fixed frame
        first_link = root.find("link")
        fixed_frame = (first_link.get("name") if first_link is not None
                       else "base_link") or "base_link"
        # a detected kinematic loop ships the IK relay node + closure config + a
        # launch wired GUI -> relay -> robot_state_publisher (linear <mimic>
        # alone can't track a loop); a plain robot keeps the simpler GUI -> RSP
        coupled = bool(loop_closures and loop_closures.get("closures"))
        pkg_xml = PACKAGE_XML_ROS2_COUPLED if coupled else PACKAGE_XML_ROS2
        cmake = CMAKELISTS_ROS2_COUPLED if coupled else CMAKELISTS_ROS2
        launch = DISPLAY_LAUNCH_ROS2_COUPLED if coupled else DISPLAY_LAUNCH_ROS2
        files.append((f"{pkg}/package.xml",
                      pkg_xml.format(name=pkg, email=email).encode("utf-8")))
        files.append((f"{pkg}/CMakeLists.txt",
                      cmake.format(name=pkg).encode("utf-8")))
        files.append((f"{pkg}/launch/display.launch.py",
                      launch.format(name=pkg, robot=urdf_stem).encode("utf-8")))
        files.append((f"{pkg}/rviz/{urdf_stem}.rviz",
                      RVIZ_CONFIG_ROS2.format(fixed_frame=fixed_frame,
                                              tf_scale=tf_scale)
                      .encode("utf-8")))
        if coupled:
            files.append((f"{pkg}/config/loop_closures.yaml",
                          _loop_closures_yaml(loop_closures).encode("utf-8")))
            files.append((f"{pkg}/scripts/loop_closure_relay.py",
                          LOOP_COUPLING_RELAY_PY.encode("utf-8")))
    else:
        files.append((f"{pkg}/package.xml",
                      PACKAGE_XML.format(name=pkg, email=email).encode("utf-8")))
        files.append((f"{pkg}/CMakeLists.txt",
                      CMAKELISTS.format(name=pkg).encode("utf-8")))
    return files


def write_ros_description_package(pkg_dir, robot_name, dest_dir,
                                  email="auto@example.com", ros_version=1,
                                  pkg_name=None, urdf_name=None, colors=None,
                                  collision="copy",
                                  coacd_quality="balanced",
                                  merge_fixed=False, mesh_dir=None,
                                  loop_closures=None, zero_origins=True):
    """Write the ROS package under ``dest_dir`` and return its directory path.
    The package is named ``pkg_name`` if given, else ``<robot_name>_description``;
    the URDF inside is named ``urdf_name`` if given, else the package name.
    ``ros_version`` (1 = catkin, 2 = ament_cmake), ``colors`` (per-link colour
    overrides), ``collision`` / ``coacd_quality`` (CoACD collision-mesh
    decomposition), ``merge_fixed`` (lump fixed-joint children into parents),
    ``mesh_dir`` (package-relative mesh directory, default ``meshes``) and
    ``loop_couplings`` (ROS 2 closed-loop relay node + config) are passed
    through to :func:`build_ros_description`."""
    pkg = ros_pkg_name(robot_name, pkg_name)
    files = build_ros_description(pkg_dir, robot_name, email=email,
                                  ros_version=ros_version, pkg_name=pkg,
                                  urdf_name=urdf_name, colors=colors,
                                  collision=collision,
                                  coacd_quality=coacd_quality,
                                  merge_fixed=merge_fixed, mesh_dir=mesh_dir,
                                  loop_closures=loop_closures,
                                  zero_origins=zero_origins)
    root = os.path.abspath(dest_dir)
    for arc, data in files:
        dst = os.path.join(root, *arc.split("/"))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(data)
        # a node under scripts/ must be executable: ament's install(PROGRAMS)
        # copies 0755, but `colcon build --symlink-install` SYMLINKS back to
        # this source file, so a 0644 source leaves a non-executable libexec
        # entry that `ros2 launch` cannot find
        if "/scripts/" in arc and arc.endswith(".py"):
            os.chmod(dst, 0o755)
    return os.path.join(root, pkg)
