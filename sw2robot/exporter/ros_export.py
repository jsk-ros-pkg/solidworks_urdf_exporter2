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


def _loop_couplings_yaml(couplings):
    """Serialise the closed-loop coupling list to the relay node's config YAML.

    Hand-formatted (not via the yaml dumper) so the polynomial coefficients
    stay readable and the file carries an explaining header."""
    lines = [
        "# Closed-loop (four-bar) joint couplings for loop_coupling_relay.",
        "# Each follower angle = polyval(poly, driver_angle) (highest-degree",
        "# coefficient first); this is the exact coupling the linear URDF",
        "# <mimic> can only approximate.",
        "couplings:",
    ]
    for c in couplings:
        lines.append(f"  - driver: {c['driver']}")
        lines.append("    followers:")
        for f in c["followers"]:
            poly = ", ".join(repr(float(x)) for x in f["poly"])
            lines.append(f"      - joint: {f['joint']}")
            lines.append(f"        poly: [{poly}]")
    return "\n".join(lines) + "\n"

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


def _mesh_to_dae_bytes(src, color=None):
    """COLLADA (.dae) bytes in metres, with colours baked into materials.
    ``color`` (``'#RRGGBB'``) overrides the mesh's own colours when given."""
    meshes = _collada_meshes(_apply_uniform_color(_load_mesh_metres(src), color))
    if len(meshes) == 1:
        return meshes[0].export(file_type="dae")
    from trimesh.exchange.dae import export_collada
    return export_collada(meshes)


def _mesh_to_stl_bytes(src, color=None):
    """STL bytes in metres (colour is dropped -- collision geometry needs none,
    so ``color`` is accepted for a uniform converter signature but ignored)."""
    return _load_mesh_metres(src).export(file_type="stl")


def _mesh_to_glb_bytes(src, color=None):
    """GLB bytes in metres, keeping the original material/texture (for
    three.js / skrobot consumers, not RViz).  ``color`` (``'#RRGGBB'``)
    overrides the mesh's own colours when given."""
    return _apply_uniform_color(_load_mesh_metres(src), color).export(
        file_type="glb")


_CONVERT = {"dae": _mesh_to_dae_bytes, "stl": _mesh_to_stl_bytes,
            "glb": _mesh_to_glb_bytes}

# default: <visual> as colour COLLADA, <collision> as plain STL (the ROS split)
_CTX_FMT = (("visual", "dae"), ("collision", "stl"))
# a uniform GLB variant (both contexts) for native-mesh / three.js consumers
GLB_CTX_FMT = (("visual", "glb"), ("collision", "glb"))


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


# a fixed high-contrast palette so adjacent convex parts read apart in the viewer
_COACD_PALETTE = (
    (228, 26, 28), (55, 126, 184), (77, 175, 74), (152, 78, 163),
    (255, 127, 0), (210, 210, 40), (166, 86, 40), (247, 129, 191),
    (102, 194, 165), (252, 141, 98), (141, 160, 203), (153, 153, 153),
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


def coacd_preview_glbs(pkg_dir, robot_name, quality="balanced", progress=None,
                       urdf_path=None, should_cancel=None, on_start=None,
                       max_workers=None):
    """Decompose every link's ``<collision>`` mesh with CoACD and write one
    colour-coded preview GLB per link under ``meshes/.coacd_cache/preview/`` (the
    convex parts, each part a distinct colour, the collision ``<origin>`` baked
    in so the GLB sits at the link frame).  Shares the on-disk part cache with
    the ROS export, so generating the preview also warms a later ``collision=
    'coacd'`` export.

    Links are decomposed CONCURRENTLY in a thread pool -- CoACD releases the GIL,
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

    if quality not in _COACD_PRESETS:
        raise ValueError(f"unsupported collision_quality: {quality!r} "
                         f"(use one of {sorted(_COACD_PRESETS)})")
    if not coacd_available():
        raise ValueError("CoACD collision decomposition needs the optional "
                         "'coacd' package -- install it with: pip install coacd")
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
            for data in _coacd_part_stls(src, quality, cache_dir):
                part = trimesh.load(io.BytesIO(data), file_type="stl")
                part.apply_transform(mat)
                rgb = _COACD_PALETTE[part_i % len(_COACD_PALETTE)]
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


def build_ros_description(pkg_dir, robot_name, email="auto@example.com",
                          ctx_fmt=_CTX_FMT, ros_version=1, pkg_name=None,
                          urdf_name=None, colors=None, collision="copy",
                          collision_quality="balanced", merge_fixed=False,
                          mesh_dir=None, loop_couplings=None):
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
    ``'coacd'`` runs CoACD approximate convex decomposition, replacing each
    ``<collision>``'s mesh with a set of convex part STLs (better for physics
    engines).  ``collision_quality`` (``'balanced'`` | ``'fine'``) picks the
    CoACD preset.  ``'coacd'`` needs the optional ``coacd`` package; its absence
    raises a clear error.  Decomposition is cached under ``meshes/.coacd_cache``.

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
    before any entries are emitted, so a half-rewritten package never ships."""
    if ros_version not in (1, 2):
        raise ValueError(f"unsupported ros_version: {ros_version}")
    if collision not in ("copy", "coacd"):
        raise ValueError(f"unsupported collision mode: {collision!r} "
                         "(use 'copy' or 'coacd')")
    if collision == "coacd":
        if collision_quality not in _COACD_PRESETS:
            raise ValueError(
                f"unsupported collision_quality: {collision_quality!r} "
                f"(use one of {sorted(_COACD_PRESETS)})")
        if not coacd_available():
            raise ValueError(
                "CoACD collision decomposition needs the optional 'coacd' "
                "package -- install it with: pip install coacd")
    coacd_cache_dir = os.path.join(pkg_dir, "meshes", ".coacd_cache")
    if loop_couplings is None:
        # the editor's ZIP export calls us directly (no model); pick up the
        # closed-loop couplings build() persisted beside the package so the ZIP
        # ships the relay node + cubic config too
        side = os.path.join(pkg_dir, "loop_couplings.yaml")
        if os.path.isfile(side):
            try:
                import yaml
                with open(side, encoding="utf-8") as f:
                    loop_couplings = (yaml.safe_load(f) or {}).get("couplings")
            except Exception:
                loop_couplings = None
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
    if merge_fixed:
        # lump fixed-joint children with geometry into their parents BEFORE the
        # mesh conversion / collision loop runs over the (now fewer) links
        from .merge import merge_fixed_links
        merge_fixed_links(root)
    colors = colors or {}

    files = []
    # keyed by the SOURCE mesh (+fmt), so one source converted once is reused,
    # but two DISTINCT sources that happen to share a basename get distinct output
    # names instead of silently overwriting each other (e.g. a visual part.dae and
    # a collision part.stl both targeting part.glb).
    done = {}          # (abs_src, fmt, color) -> output mesh name ('part.glb')
    used_names = set()
    errors = []

    def _emit(base, src, fmt, color):
        # key on colour too: the same source reused by two links with DIFFERENT
        # colour overrides must emit two distinct (differently-coloured) meshes
        key = (os.path.abspath(src), fmt, color)
        if key in done:
            return done[key]
        name, i = f"{base}.{fmt}", 1
        while name in used_names:              # distinct source, same basename
            i += 1
            name = f"{base}_{i}.{fmt}"
        try:
            if src.lower().endswith(f".{fmt}") and not color:
                # already the target format and no colour override: copy the mesh
                # verbatim (a URDF re-export shipping its own .dae/.stl) -- avoids
                # a lossy + sometimes-failing trimesh round-trip
                with open(src, "rb") as f:
                    data = f.read()
                if fmt == "dae":             # ship the .dae's sidecar textures too
                    dae_dir = os.path.dirname(src)
                    for rel in _dae_sidecar_images(src):
                        img = os.path.normpath(os.path.join(dae_dir, rel))
                        arc = f"{pkg}/{mesh_rel}/{rel}"
                        if os.path.exists(img) and arc not in used_names:
                            with open(img, "rb") as f:
                                files.append((arc, f.read()))
                            used_names.add(arc)
            else:
                data = _CONVERT[fmt](src, color=color)
        except Exception as e:
            errors.append(f"{base}: {fmt} convert failed ({e!r})")
            return None
        used_names.add(name)
        files.append((f"{pkg}/{mesh_rel}/{name}", data))
        done[key] = name
        return name

    def _emit_coacd(base, src):
        # CoACD-decompose a source mesh into convex collision part STLs; returns
        # the list of emitted mesh names (one per convex part), or None on error.
        # Keyed (in `done`) by source + quality so a mesh shared by several links
        # decomposes once.
        key = (os.path.abspath(src), "coacd", collision_quality)
        if key in done:
            return done[key]
        try:
            blobs = _coacd_part_stls(src, collision_quality, coacd_cache_dir)
        except Exception as e:
            errors.append(f"{base}: coacd decompose failed ({e!r})")
            return None
        names = []
        for i, data in enumerate(blobs):
            name, j = f"{base}_collision_{i}.stl", 1
            while name in used_names:
                j += 1
                name = f"{base}_collision_{i}_{j}.stl"
            used_names.add(name)
            files.append((f"{pkg}/{mesh_rel}/{name}", data))
            names.append(name)
        done[key] = names
        return names

    def _expand_collision_coacd(link):
        # replace each <collision> whose single mesh is a convertible source with
        # N <collision> blocks (one per convex part), preserving its <origin>.
        # Blocks we can't decompose (primitive geometry, external/missing mesh,
        # multi-mesh) are left to the normal per-mesh path below.
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
            names = _emit_coacd(base, src)
            if not names:
                continue
            idx = list(link).index(block)
            link.remove(block)
            # deep-copy the original block per part so <origin> + formatting carry
            for k, name in enumerate(names):
                nb = copy.deepcopy(block)
                next(nb.iter("mesh")).set(
                    "filename", f"package://{pkg}/{mesh_rel}/{name}")
                link.insert(idx + k, nb)

    for link in root.findall("link"):
        # colour overrides are keyed by LINK name in URDF-input mode and by the
        # component/mesh basename in the CAD path -- accept either
        link_color = colors.get(link.get("name"))
        if collision == "coacd":
            _expand_collision_coacd(link)
        for ctx, fmt in ctx_fmt:
            if ctx == "collision" and collision == "coacd":
                continue                   # handled by _expand_collision_coacd
            for block in link.findall(ctx):
                for mesh in block.iter("mesh"):
                    base, src, _ext = _resolve_mesh(pkg_dir, mesh.get("filename"),
                                                    own_pkgs=own_pkgs)
                    if base is None:
                        continue           # not a convertible ref -- leave as-is
                    if src is None:
                        errors.append(f"no source mesh for '{base}'")
                        continue
                    name = _emit(base, src, fmt, link_color or colors.get(base))
                    if name is not None:
                        mesh.set("filename",
                                 f"package://{pkg}/{mesh_rel}/{name}")

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
        # a detected kinematic loop ships a relay node + cubic config + a launch
        # wired GUI -> relay -> robot_state_publisher (linear <mimic> alone can't
        # track a four-bar); a plain robot keeps the simpler GUI -> RSP launch
        coupled = bool(loop_couplings)
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
                      RVIZ_CONFIG_ROS2.format(fixed_frame=fixed_frame)
                      .encode("utf-8")))
        if coupled:
            files.append((f"{pkg}/config/loop_couplings.yaml",
                          _loop_couplings_yaml(loop_couplings).encode("utf-8")))
            files.append((f"{pkg}/scripts/loop_coupling_relay.py",
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
                                  collision_quality="balanced",
                                  merge_fixed=False, mesh_dir=None,
                                  loop_couplings=None):
    """Write the ROS package under ``dest_dir`` and return its directory path.
    The package is named ``pkg_name`` if given, else ``<robot_name>_description``;
    the URDF inside is named ``urdf_name`` if given, else the package name.
    ``ros_version`` (1 = catkin, 2 = ament_cmake), ``colors`` (per-link colour
    overrides), ``collision`` / ``collision_quality`` (CoACD collision-mesh
    decomposition), ``merge_fixed`` (lump fixed-joint children into parents),
    ``mesh_dir`` (package-relative mesh directory, default ``meshes``) and
    ``loop_couplings`` (ROS 2 closed-loop relay node + config) are passed
    through to :func:`build_ros_description`."""
    pkg = ros_pkg_name(robot_name, pkg_name)
    files = build_ros_description(pkg_dir, robot_name, email=email,
                                  ros_version=ros_version, pkg_name=pkg,
                                  urdf_name=urdf_name, colors=colors,
                                  collision=collision,
                                  collision_quality=collision_quality,
                                  merge_fixed=merge_fixed, mesh_dir=mesh_dir,
                                  loop_couplings=loop_couplings)
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
