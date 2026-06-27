"""Serve sw2robot.exporter module packages to the urdf-loaders web viewer.

    uv run python -m sw2robot.editor.webserver [package_dir] [--root output] [--port 8090]

Prototype of the sw2robot.editor web View: a static gkjohnson/urdf-loaders page
(``sw2robot/editor/web/``) + this tiny stdlib server.

Routes
    /                     the viewer page (sw2robot/editor/web/index.html)
    /api/info             current module: {"name", "urdf"}
    /api/list             packages under --root: [{"name", "path"}, ...]
    /api/open?path=P      switch the served package (package dir, a dir with
                          urdf/*.urdf, or a .urdf file path) -> /api/info JSON
    /pkg/<rel>            files from the CURRENT package dir
    /pkg/<rel>.3dxml?glb=1  the mesh converted to GLB (three.js cannot read
                          3DXML), cached next to the source as <rel>.3dxml.glb

Single-user LOCAL tool by design: /api/open accepts arbitrary local paths on
purpose (that's the file picker), so never expose this server beyond
localhost.  No third-party server deps; mesh conversion reuses trimesh which
sw2robot.exporter already requires.
"""
import argparse
import contextlib
import http.server
import json
import os
import posixpath
import re
import socket
import socketserver
import sys
import tempfile
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(HERE)
PROJECT_ROOT = os.path.dirname(PACKAGE_ROOT)
WEB_DIR = os.path.join(HERE, "web")

# The port the running server bound (set in serve()).  The self-update relaunch
# reads this (sw2robot.editor.update._current_bound_port) to reclaim the SAME
# port, so the page that reloads after an update reconnects at the same URL.
BOUND_PORT = None


def _app_data_dir():
    """Writable base for runtime side-files (the default package root, the
    client report).  A PyInstaller-frozen exe's PROJECT_ROOT
    lives INSIDE the read-only bundle, so fall back to a stable Windows temp
    dir there; a source checkout keeps using the repo root, so the dev workflow
    is unchanged."""
    if getattr(sys, "frozen", False):
        d = os.path.join(tempfile.gettempdir(), "sw2robot")
        os.makedirs(d, exist_ok=True)
        return d
    return PROJECT_ROOT


# repo root in a checkout; %TEMP%\sw2robot in a frozen .exe
_DATA_DIR = _app_data_dir()


def _default_root():
    """Default package root when the user gives no ``--root``: a double-clicked
    GUI exe has no useful CWD, so write under the Windows temp data dir; in a
    source checkout keep the historical ``output`` (relative to the CWD)."""
    if getattr(sys, "frozen", False):
        d = os.path.join(_DATA_DIR, "output")
        os.makedirs(d, exist_ok=True)
        return d
    return "output"

_CTYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript",
    ".css": "text/css",
    ".urdf": "application/xml",
    ".xml": "application/xml",
    ".json": "application/json",
    ".glb": "model/gltf-binary",
    ".gltf": "model/gltf+json",
    ".stl": "model/stl",
    ".dae": "model/vnd.collada+xml",
    ".3dxml": "application/octet-stream",
    ".yaml": "text/plain; charset=utf-8",
}


def _to_single_mesh(mesh):
    import trimesh
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.to_mesh() if hasattr(mesh, "to_mesh") \
            else mesh.dump(concatenate=True)
    return mesh


def _convert_3dxml_to_glb(src):
    """3DXML (SolidWorks, mm) -> GLB (metres), cached as ``<src>.glb``."""
    cache = src + ".glb"
    if os.path.exists(cache) \
            and os.path.getmtime(cache) >= os.path.getmtime(src):
        return cache
    print(f"[sw2robot.web] converting {os.path.basename(src)} -> glb ...")
    import trimesh
    mesh = _to_single_mesh(trimesh.load(src))
    mesh.apply_scale(0.001)
    mesh.units = "meter"
    tmp = cache + ".part.glb"
    mesh.export(tmp, file_type="glb")
    os.replace(tmp, cache)
    return cache


def _preconvert_meshes(pkg_dir):
    """Convert every .3dxml in the package to its GLB cache in a background
    thread, so the page never waits per-mesh on first view."""
    def run():
        n = 0
        for dirpath, _dirs, files in os.walk(pkg_dir):
            for f in files:
                if f.lower().endswith(".3dxml"):
                    try:
                        src = os.path.join(dirpath, f)
                        if not os.path.exists(src + ".glb") \
                                or os.path.getmtime(src + ".glb") \
                                < os.path.getmtime(src):
                            _convert_3dxml_to_glb(src)
                            n += 1
                    except Exception as e:
                        print(f"[sw2robot.web] preconvert {f}: {e!r}")
        if n:
            print(f"[sw2robot.web] preconverted {n} meshes to glb")
    threading.Thread(target=run, daemon=True).start()


def _resolve_package(path):
    """Accept a package dir, a dir containing urdf/, or a .urdf path; return
    ``(pkg_dir, urdf_rel)`` or raise ValueError."""
    path = os.path.abspath(os.path.expanduser(str(path).strip().strip('"')))
    if os.path.isfile(path) and path.lower().endswith(".urdf"):
        urdf_dir = os.path.dirname(path)
        pkg = os.path.dirname(urdf_dir) \
            if os.path.basename(urdf_dir).lower() == "urdf" else urdf_dir
        return pkg, os.path.relpath(path, pkg).replace("\\", "/")
    if os.path.isdir(path):
        for sub in ("urdf", "."):
            d = os.path.normpath(os.path.join(path, sub))
            if os.path.isdir(d):
                # skip dotfiles -- e.g. the hidden .<name>.live.urdf overlay copy
                urdfs = sorted(f for f in os.listdir(d)
                               if f.lower().endswith(".urdf")
                               and not f.startswith("."))
                if urdfs:
                    rel = os.path.relpath(os.path.join(d, urdfs[0]), path)
                    return path, rel.replace("\\", "/")
        raise ValueError(f"no *.urdf under {path}")
    raise ValueError(f"not a package dir or .urdf file: {path}")


# --- URDF-input editing mode -------------------------------------------------
# A package WITHOUT a graph.json is a plain URDF the user opened directly: there
# is no CAD graph to rebuild from, so edits cannot go through joints.yaml +
# build().  Instead we hold the core overlay (RobotCompilerState: per-joint and
# per-link edits) in memory and route every edit through the SAME core setters
# the headless CLI uses.  The URDF URL is served as build_urdf(state), so the
# on-disk .urdf stays the pristine base: the declarative overlay re-applies
# cleanly on every request and survives a restart via the .sw2robot.json sidecar.
# ``rev`` bumps on every edit so on-disk readers (collision / auto-limits) can
# cache against a stable (path, mtime) -- the live URDF is only rewritten when
# ``rev`` actually changed (see _um_live_urdf).
# undo/redo in URDF mode snapshot the overlay (edits + link_edits) as JSON, since
# there is no joints.yaml to snapshot (the CAD path's history mechanism).
_um = {"state": None, "rev": 0, "live_rev": -1, "undo": [], "redo": []}


def _um_close():
    """Drop the in-memory overlay (state + undo/redo + caches) when switching or
    closing a package.  The hidden .live.urdf copy is intentionally NOT deleted:
    a background collision / auto-limit worker may still be reading it, and the
    file is hidden + gitignored and overwritten on the package's next use."""
    _um.update(state=None, rev=0, live_rev=-1, undo=[], redo=[])


def _cad_mode(pkg_dir):
    """True when the package has a CAD graph (the joints.yaml + build() path);
    False for a plain URDF opened directly (the overlay path)."""
    return bool(pkg_dir) and os.path.exists(os.path.join(pkg_dir, "graph.json"))


def _um_load(pkg_dir, urdf_rel):
    """(Re)build the URDF-mode overlay state for the freshly-opened package and
    merge any saved sidecar edits.  Returns the state."""
    from . import core
    state = core.load_module(os.path.join(pkg_dir, urdf_rel), package_dir=pkg_dir)
    core.load_edits(state)            # restore a prior session's edits, if any
    _um["state"] = state
    _um["rev"], _um["live_rev"] = 0, -1
    _um["undo"], _um["redo"] = [], []
    return state


def _um_overlay_json(state):
    """A compact snapshot of just the editable overlay (for undo/redo)."""
    return state.model_dump_json(include={"edits", "link_edits"})


def _um_restore(snap):
    """Replace the live overlay with a snapshot's edits/link_edits."""
    from .state import JointEdit, LinkEdit
    data = json.loads(snap)
    st = _um["state"]
    st.edits = {k: JointEdit(**v) for k, v in (data.get("edits") or {}).items()}
    st.link_edits = {k: LinkEdit(**v)
                     for k, v in (data.get("link_edits") or {}).items()}
    _um_save(st)


def _um_save(state):
    from . import core
    _um["rev"] += 1                   # invalidate the live URDF / disk caches
    try:
        core.save_state(state)
    except OSError:
        pass                          # persistence is best-effort


def _um_live_urdf(pkg_dir, urdf_rel):
    """``(abs_path, rel)`` of a hidden, overlay-applied copy of the URDF kept in
    sync with the in-memory edits -- for on-disk readers that take a URDF path
    (the self-collision build, the auto-limit subprocess).  In CAD mode (or with
    no overlay) returns the package URDF itself.  Regenerated only when edits
    changed, so a caller's ``(path, mtime)`` cache key stays stable across polls;
    sits next to the base so the relative ``../meshes`` references still resolve."""
    if _cad_mode(pkg_dir) or _um["state"] is None:
        return os.path.join(pkg_dir, *urdf_rel.split("/")), urdf_rel
    from . import core
    d, base = posixpath.dirname(urdf_rel), posixpath.basename(urdf_rel)
    stem = base[:-len(".urdf")] if base.lower().endswith(".urdf") else base
    live_rel = posixpath.join(d, f".{stem}.live.urdf") if d else f".{stem}.live.urdf"
    live_path = os.path.join(pkg_dir, *live_rel.split("/"))
    if _um["live_rev"] != _um["rev"] or not os.path.exists(live_path):
        # atomic replace: an async reader (collision / auto-limits) holding this
        # path must never see a truncated file mid-rewrite.  The temp also starts
        # with '.' so a concurrent _resolve_package never picks it as the URDF.
        data = core.build_urdf(_um["state"], sanitize=False)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(live_path) or ".",
                                   prefix=".live", suffix=".urdf")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(data)
            os.replace(tmp, live_path)
            _um["live_rev"] = _um["rev"]
        except OSError:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            # tolerate a transient failure (e.g. a Windows lock from a reader)
            # ONLY when a previous valid live copy remains -- it is at worst one
            # revision stale and refreshes on the next call; with no copy at all
            # there is nothing usable to hand out, so signal the failure
            if not os.path.exists(live_path):
                raise
    return live_path, live_rel


@contextlib.contextmanager
def _um_materialized(pkg_dir, urdf_rel):
    """Temporarily write the overlay-applied URDF to disk so on-disk readers (the
    ROS exporter) see URDF-mode edits, then restore the pristine base.  The
    served URDF is normally computed on the fly (disk stays pristine); export is
    the one path that reads the file directly.  A no-op in CAD mode."""
    if _cad_mode(pkg_dir) or _um["state"] is None:
        yield
        return
    from . import core
    served = os.path.join(pkg_dir, urdf_rel)
    # build the edited URDF BEFORE truncating `served` -- build_urdf reads the
    # pristine base from that same path, so opening it "w" first would empty it
    edited = core.build_urdf(_um["state"], sanitize=False)
    try:
        with open(served, encoding="utf-8") as f:
            pristine = f.read()
    except OSError:
        pristine = None
    try:
        with open(served, "w", encoding="utf-8") as f:
            f.write(edited)
        yield
    finally:
        if pristine is not None:
            with open(served, "w", encoding="utf-8") as f:
                f.write(pristine)


def _um_joint_by_child(state, child):
    """Map a child-link name (what the limits/types/mimic UI sends) to its joint
    name (what the overlay is keyed by); None if no joint has that child."""
    return next((j["name"] for j in state.joints if j["childLink"] == child), None)


def _rewrite_package_urls(urdf_text, urdf_rel, pkg_dir):
    """Rewrite mesh ``package://<pkg>/<rest>`` URIs so the viewer can fetch them
    from the package root (served at ``/pkg/``).  urdf-loaders resolves a mesh
    path RELATIVE to the URDF's own URL, so emit ``<../ per dir>rest`` -- e.g. a
    URDF served at ``/pkg/urdf/x.urdf`` gets ``../<rest>`` so the browser
    normalizes ``/pkg/urdf/../<rest>`` -> ``/pkg/<rest>`` (the package root).
    Layout-independent (depth derived from ``urdf_rel``).  Applied ONLY to the
    URDF served to the viewer; the on-disk file (and the ROS export) keep their
    original ``package://`` references.

    A ref is rewritten only when its mesh actually EXISTS in this package -- so
    own meshes load regardless of the URI's package name (folder need not match),
    while a ref to another ROS package whose file is not here is left as
    ``package://`` (unresolvable in-browser either way, but not mis-pointed)."""
    depth = len([s for s in posixpath.dirname(urdf_rel).split("/") if s])
    prefix = "../" * depth
    root = os.path.normpath(pkg_dir)

    def repl(m):
        rest = m.group(1)
        local = os.path.normpath(os.path.join(root, *rest.split("/")))
        try:
            inside = os.path.commonpath([root, local]) == root
        except ValueError:
            inside = False
        if inside and os.path.exists(local):
            return f'filename="{prefix}{rest}"'
        return m.group(0)                  # not in this package -- leave as-is
    return re.sub(r'filename="package://[^/"]+/([^"]+)"', repl, urdf_text)


def _um_colors(state):
    """``{link -> '#RRGGBB'}`` from the overlay -- the ROS exporter uses this to
    repaint converted meshes (URDF mode has no joints.yaml ``colors:`` block)."""
    return {ln: le.color for ln, le in state.link_edits.items() if le.color}


def _um_components(state):
    """The /api/components payload for URDF mode: links straight from the parsed
    URDF, colours from the overlay (no CAD material/density concept here)."""
    links, colors = {}, {}
    for ln in state.links:
        name = ln["name"]
        le = state.link_edits.get(name)
        col = le.color if le else None
        links[name] = {"material": None, "density": None, "name": name,
                       "override": None, "color": col}
        if col:
            colors[name] = col
    return {"links": links, "excluded": [], "colors": colors}


def _um_set_limits(state, limits):
    from . import core
    applied, missed = [], []
    for lm in limits:
        child = lm.get("child")
        j = _um_joint_by_child(state, child)
        if not j:
            missed.append(child)
            continue
        if lm.get("continuous"):
            core.set_joint_type(state, j, "continuous")
        else:
            core.set_limits(state, j, float(lm.get("lower", 0.0)),
                            float(lm.get("upper", 0.0)))
        applied.append(child)
    _um_save(state)
    return {"applied": applied, "missed": missed}


def _um_set_types(state, changes):
    from . import core
    applied, missed = [], []
    for ch in changes:
        child, t = ch.get("child"), ch.get("type")
        j = _um_joint_by_child(state, child)
        if not j or t not in core.JOINT_TYPES:
            missed.append(ch)
            continue
        core.set_joint_type(state, j, t)
        applied.append(child)
    _um_save(state)
    return {"applied": applied, "missed": missed}


def _um_set_mimic(state, changes):
    from . import core
    applied, missed = [], []
    for ch in changes:
        child = ch.get("child")
        j = _um_joint_by_child(state, child)
        if not j:
            missed.append(child)
            continue
        jd = next((x for x in state.joints if x["name"] == j), None)
        if not ch.get("clear") and (jd is None or state.effective_type(jd)
                                    not in ("revolute", "continuous", "prismatic")):
            missed.append(child)           # a fixed joint can't follow a mimic
            continue
        try:
            if ch.get("clear"):
                core.clear_mimic(state, j)
            else:
                # the UI sends the master's CURRENT (effective) name; the overlay
                # is keyed by original names, so map it back before validating
                master = ch.get("master")
                master = next((m["name"] for m in state.joints
                               if state.effective_name(m["name"]) == master),
                              master)
                core.set_mimic(state, j, master,
                               float(ch.get("multiplier", 1.0)),
                               float(ch.get("offset", 0.0)))
        except ValueError:
            missed.append(child)
            continue
        applied.append(child)
    _um_save(state)
    return {"applied": applied, "missed": missed}


def _um_set_axis(state, joints):
    from . import core
    applied, missed = [], []
    # the UI posts CURRENT (effective) joint names; the overlay is keyed by
    # original names, so map each back before flipping (as mimic/rename do)
    by_effective = {state.effective_name(j["name"]): j["name"] for j in state.joints}
    for jn in joints:
        orig = by_effective.get(jn)
        if orig is None:
            missed.append(jn)
            continue
        core.reverse_direction(state, orig)  # flip axis + remap limits (self-inverse)
        applied.append(jn)
    _um_save(state)
    return {"applied": applied, "missed": missed}


def _um_set_color(state, link, color):
    from . import core
    core.set_color(state, link, color)      # raises ValueError on a bad hex
    _um_save(state)
    le = state.link_edits.get(link)
    return {"link": link, "color": le.color if le else None}


def _um_set_inertial(state, body):
    from . import core
    link = body.get("link")
    if not link:
        raise ValueError("link required")
    core.set_inertial(state, link, mass=body.get("mass"),
                      com=body.get("com"), inertia=body.get("inertia"))
    _um_save(state)
    le = state.link_edits.get(link)
    return {"link": link,
            "mass": le.mass if le else None,
            "com": le.com if le else None,
            "inertia": le.inertia if le else None}


def _um_reset_names(state):
    n = 0
    for e in state.edits.values():
        if e.rename:
            e.rename = None
            n += 1
    _um_save(state)
    return {"ok": True, "reset": n}


def _um_rename(state, kind, old, new):
    from . import core
    if kind == "link":
        raise ValueError("link rename is not yet supported in URDF-input mode")
    # joints are keyed by ORIGINAL name; the UI sends the CURRENT (effective)
    # name, so map back through the rename overlay
    orig = next((j["name"] for j in state.joints
                 if state.effective_name(j["name"]) == old), None)
    if orig is None:
        raise ValueError(f"no such joint: {old}")
    if not new:                              # empty -> reset to the original name
        if orig in state.edits:
            state.edits[orig].rename = None
    else:
        core.rename_joint(state, orig, new)
    _um_save(state)
    return {"kind": kind, "old": old, "new": new}


# Opening an assembly needs its REAL on-disk path (SolidWorks resolves the
# referenced parts relative to it), but a browser never reveals the path of a
# drag&dropped / file-dialog file -- only its name+size+bytes.  Rather than
# guess the path by walking the disk (slow, and the guessed locations are
# environment-specific), the editor opens by an actual path: the 🗄 server-side
# file browser (which lists SolidWorks' recent files for one-click access), or
# 📋 paste-a-full-path.  Both hand the server a real path; no indexing, no walk.


def _read_root_pose(txt):
    """Current root_rpy / root_xyz / root_z_offset from joints.yaml text."""
    import re

    def vec(key, default):
        m = re.search(r"(?m)^" + key + r":\s*\[([^\]]*)\]", txt)
        return [float(x) for x in m.group(1).split(",")] if m else default

    m = re.search(r"(?m)^root_z_offset:\s*([-\d.eE]+)", txt)
    return (vec("root_rpy", [0, 0, 0]), vec("root_xyz", [0, 0, 0]),
            float(m.group(1)) if m else 0.0)


# Build-and-launch one-liner (robot-compiler style).  {pkg} = ROS 2 package
# name, {zip_url} = this server's /api/export/zip URL.  Run with:
#   curl -s http://<host>/api/launch_it.sh | bash
_LAUNCH_IT_SH = r"""#!/bin/bash
set -e
G='\033[0;32m'; R='\033[0;31m'; N='\033[0m'
PKG="{pkg}"
ZIP_URL="{zip_url}"
# safety: PKG must be a plain package name (never empty, a slash or '..'), so the
# scoped removals below can only ever touch <ws>/{{src,build,install}}/$PKG
case "$PKG" in ""|*/*|*..*) echo -e "${{R}}refusing: unsafe package name${{N}}"; exit 1 ;; esac
WS="$(pwd)/${{PKG}}_ws"
echo -e "${{G}}sw2robot: build + launch ${{PKG}}${{N}}  ($WS)"
mkdir -p "$WS/src"
# replace ONLY this package (never wipe the whole workspace) -- any other
# packages or files you keep in ${{PKG}}_ws are left untouched
rm -rf "$WS/src/$PKG" "$WS/build/$PKG" "$WS/install/$PKG"
cd "$WS"
echo -e "${{G}}downloading package zip ...${{N}}"
code=$(curl -sSL -w "%{{http_code}}" -o robot.zip "$ZIP_URL")
if [ "$code" != "200" ]; then echo -e "${{R}}download failed (HTTP $code)${{N}}"; cat robot.zip; rm -f robot.zip; exit 1; fi
( cd src && unzip -oq ../robot.zip ) && rm -f robot.zip
source "/opt/ros/${{ROS_DISTRO:-humble}}/setup.bash"
echo -e "${{G}}rosdep + colcon build ...${{N}}"
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
colcon build --symlink-install --packages-select "$PKG"
source install/setup.bash
echo -e "${{G}}launching display.launch.py ...${{N}}"
exec ros2 launch "$PKG" display.launch.py
"""


def _export_zip(pkg_dir, robot_name, mesh_fmt="dae", ros_version=1,
                pkg_name=None, urdf_name=None, colors=None,
                collision="copy", collision_quality="balanced",
                merge_fixed=False, mesh_dir=None):
    """ZIP a portable ROS package (package:// URLs), named ``pkg_name`` if given
    else ``<robot_name>_description``; the URDF inside is named ``urdf_name`` if
    given, else the package name.

    ``mesh_fmt='dae'`` (default): ``<visual>`` as colour COLLADA ``.dae`` +
    ``<collision>`` as plain ``.stl`` -- the RViz/Gazebo-ready variant.
    ``mesh_fmt='glb'``: a uniform ``.glb`` package (colour kept) for three.js /
    skrobot / native-mesh consumers (not RViz-loadable).

    ``ros_version`` (1 = catkin, 2 = ament_cmake) selects the build files.
    Returns ``(pkg, bytes)`` so the caller can name the download after the
    actual package."""
    if mesh_fmt not in ("dae", "glb"):
        raise ValueError(f"unsupported mesh format: {mesh_fmt}")

    import io as _io
    import zipfile

    from sw2robot.exporter.ros_export import (
        GLB_CTX_FMT,
        build_ros_description,
        ros_pkg_name,
    )
    pkg = ros_pkg_name(robot_name, pkg_name)
    kwargs = {"ctx_fmt": GLB_CTX_FMT} if mesh_fmt == "glb" else {}
    buf = _io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for arc, data in build_ros_description(pkg_dir, robot_name,
                                               ros_version=ros_version,
                                               pkg_name=pkg,
                                               urdf_name=urdf_name,
                                               colors=colors,
                                               collision=collision,
                                               collision_quality=collision_quality,
                                               merge_fixed=merge_fixed,
                                               mesh_dir=mesh_dir,
                                               **kwargs):
            # a node under scripts/ must extract executable (0755) so ament's
            # install(PROGRAMS) + `colcon build --symlink-install` leaves a
            # runnable libexec entry that `ros2 launch` can find
            if "/scripts/" in arc and arc.endswith(".py"):
                zi = zipfile.ZipInfo(arc)
                zi.external_attr = 0o755 << 16
                zi.compress_type = zipfile.ZIP_DEFLATED
                z.writestr(zi, data)
            else:
                z.writestr(arc, data)
    return pkg, buf.getvalue()


# --- joint/link rename overlay (joints.yaml link_names / joint_names) --------
_VALID_NAME = re.compile(r"^[A-Za-z_][0-9A-Za-z_]*$")


def _names_inverse(yml_txt, mapkey):
    """``{display name -> stable key}`` for a joints.yaml rename map
    (``link_names`` / ``joint_names``); empty when the map is absent.  Used to
    reverse-map an edit request's display name back to the component key the
    rest of joints.yaml is keyed by."""
    import yaml as _yaml
    try:
        cfg = _yaml.safe_load(yml_txt) or {}
    except Exception:
        return {}
    m = cfg.get(mapkey) or {}
    return {disp: key for key, disp in m.items() if disp}


def _link_names_inverse(yml_txt):
    inv = _names_inverse(yml_txt, "link_names")
    import yaml as _yaml
    try:
        cfg = _yaml.safe_load(yml_txt) or {}
    except Exception:
        return inv
    base = cfg.get("base")
    root_name = cfg.get("root_link_name", "base_link")
    if base and root_name:
        inv[root_name] = base
    return inv


def _read_colors(pkg_dir, urdf_rel):
    """``{component link name -> '#RRGGBB'}`` from the package's joints.yaml
    ``colors:`` block; empty when there is no package/config/block."""
    if not pkg_dir or not urdf_rel:
        return {}
    name = os.path.splitext(os.path.basename(urdf_rel))[0]
    yml = os.path.join(pkg_dir, name + ".joints.yaml")
    if not os.path.exists(yml):
        return {}
    import yaml as _yaml
    try:
        cfg = _yaml.safe_load(open(yml, encoding="utf-8").read()) or {}
    except Exception:
        return {}
    colors = cfg.get("colors")
    return colors if isinstance(colors, dict) else {}


def _upsert_yaml_map(txt, mapkey, key, value):
    """Set ``mapkey: {key: value}`` (block style) in joints.yaml text, replacing
    any existing entry for ``key`` and creating the map if needed."""
    line = f"  {key}: {value}"
    block = re.compile(r"(?m)^" + re.escape(mapkey) + r":\n((?:[ \t]+\S.*\n?)*)")
    m = block.search(txt)
    if m:
        body = re.sub(r"(?m)^[ \t]+" + re.escape(key) + r":.*\n?", "",
                      m.group(1))
        if body and not body.endswith("\n"):
            body += "\n"
        return txt[:m.start()] + f"{mapkey}:\n{body}{line}\n" + txt[m.end():]
    return f"{mapkey}:\n{line}\n" + txt


def _set_root_link_name(txt, value):
    """Set the top-level ``root_link_name:`` (the root link's display name)."""
    if re.search(r"(?m)^#?\s*root_link_name:", txt):
        return re.sub(r"(?m)^#?\s*root_link_name:.*$",
                      f"root_link_name: {value}", txt, count=1)
    return f"root_link_name: {value}\n" + txt


def _clear_root_link_name(txt):
    """Drop the active ``root_link_name:`` override so the root reverts to the
    build default (``base_link``).  Leaves any commented template line."""
    return re.sub(r"(?m)^root_link_name:.*$\n?", "", txt)


def _remove_yaml_map_entry(txt, mapkey, key):
    """Remove ``mapkey: {key: ...}`` from joints.yaml text (reverting that one
    name to its default); drop the whole block if it becomes empty."""
    block = re.compile(r"(?m)^" + re.escape(mapkey) + r":\n((?:[ \t]+\S.*\n?)*)")
    m = block.search(txt)
    if not m:
        return txt
    body = re.sub(r"(?m)^[ \t]+" + re.escape(key) + r":.*\n?", "", m.group(1))
    if body.strip():
        return txt[:m.start()] + f"{mapkey}:\n{body}" + txt[m.end():]
    return txt[:m.start()] + txt[m.end():]


def _remove_yaml_block(txt, mapkey):
    """Remove an entire ``mapkey:`` block (reset every name under it)."""
    return re.sub(r"(?m)^" + re.escape(mapkey) + r":\n(?:[ \t]+\S.*\n?)*", "",
                  txt)


def _set_mimic_yaml(txt, child, master, multiplier, offset, clear, poly=None):
    """Add / replace / remove the ``mimic:`` block of the joints.yaml entry whose
    ``child:`` is the component ``child``.  ``master`` is the driver joint's URDF
    name (stored verbatim -- urdf_writer's name remap is the identity on an
    already-emitted name).  Returns ``(new_txt, applied)``.

    Line-based on purpose: it tolerates a mimic block placed anywhere inside the
    entry and any sibling keys (lower/upper/axis...), and preserves the file's
    reference comments -- unlike a YAML round-trip."""
    lines = txt.split("\n")
    starts = [i for i, ln in enumerate(lines)
              if re.match(r"^\s*-\s*parent:", ln)]
    cre = re.compile(r"^\s*child:\s*" + re.escape(child) + r"\s*(#.*)?$")
    for k in range(len(starts) - 1, -1, -1):          # last->first: edits stay
        s = starts[k]                                 # valid for earlier entries
        col = len(lines[s]) - len(lines[s].lstrip())
        end = starts[k + 1] if k + 1 < len(starts) else len(lines)
        for idx in range(s + 1, end):                 # stop at a dedented key
            ln = lines[idx]
            if ln.strip() == "":
                continue
            ind = len(ln) - len(ln.lstrip())
            if ind <= col and not ln.lstrip().startswith("-"):
                end = idx
                break
        entry = lines[s:end]
        if not any(cre.match(x) for x in entry):
            continue
        type_i = next((i for i, x in enumerate(entry)
                       if re.match(r"^\s*type:", x)), None)
        key_ind = (entry[type_i][:len(entry[type_i])
                                 - len(entry[type_i].lstrip())]
                   if type_i is not None else "    ")
        # drop any existing mimic: block (mimic line + deeper-indented children)
        cleaned, i2 = [], 0
        while i2 < len(entry):
            x = entry[i2]
            if re.match(r"^\s*mimic:\s*(#.*)?$", x):
                mind = len(x) - len(x.lstrip())
                i2 += 1
                while i2 < len(entry):
                    y = entry[i2]
                    yind = len(y) - len(y.lstrip())
                    if y.strip() == "" or yind > mind:
                        i2 += 1
                    else:
                        break
                continue
            cleaned.append(x)
            i2 += 1
        if not clear and master:
            block = [f"{key_ind}mimic:",
                     f"{key_ind}  joint: {master}",
                     f"{key_ind}  multiplier: {float(multiplier):g}",
                     f"{key_ind}  offset: {float(offset):g}"]
            if poly:
                pj = ", ".join(repr(float(x)) for x in poly)
                block.append(f"{key_ind}  poly: [{pj}]")
            ti = next((i for i, x in enumerate(cleaned)
                       if re.match(r"^\s*type:", x)), len(cleaned) - 1)
            cleaned = cleaned[:ti + 1] + block + cleaned[ti + 1:]
        lines[s:end] = cleaned
        return "\n".join(lines), True
    return txt, False


def _yaml_scalar(s):
    """Render ``s`` as a yaml plain scalar, single-quoting it when it carries
    characters (spaces, ``:`` ...) that would otherwise break a plain scalar."""
    s = str(s)
    if s and re.match(r"^[A-Za-z0-9_./-]+$", s):
        return s
    return "'" + s.replace("'", "''") + "'"


def _append_yaml_list_item(txt, listkey, item_lines):
    """Append a block-style list item under ``listkey:`` in joints.yaml text,
    creating the list when absent.  ``item_lines`` are this item's
    ``key: value`` strings -- the first becomes ``- key: value``, the rest are
    indented continuation lines."""
    body = "  - " + "\n    ".join(item_lines) + "\n"
    block = re.compile(r"(?m)^" + re.escape(listkey) + r":\n((?:[ \t]+\S.*\n?)*)")
    m = block.search(txt)
    if m:
        existing = m.group(1)
        if existing and not existing.endswith("\n"):
            existing += "\n"
        return txt[:m.start()] + f"{listkey}:\n{existing}{body}" + txt[m.end():]
    if txt and not txt.endswith("\n"):
        txt += "\n"
    return txt + f"{listkey}:\n{body}"


def _remove_yaml_list_item(txt, listkey, idx):
    """Remove the ``idx``-th (0-based) item from a block-style ``listkey:``
    list; drop the whole block if it becomes empty."""
    block = re.compile(r"(?m)^" + re.escape(listkey) + r":\n((?:[ \t]+\S.*\n?)*)")
    m = block.search(txt)
    if not m:
        return txt
    items = [p for p in re.split(r"(?m)^(?=[ \t]*-\s)", m.group(1)) if p.strip()]
    if idx < 0 or idx >= len(items):
        return txt
    del items[idx]
    if items:
        return txt[:m.start()] + f"{listkey}:\n{''.join(items)}" + txt[m.end():]
    return txt[:m.start()] + txt[m.end():]


def _rot_z_to(zdir):
    """3x3 minimal rotation (Rodrigues) taking +Z onto ``unit(zdir)``: an
    already +Z-aligned vector yields identity, an antiparallel one a 180° flip
    about X.  Shared by the root align (/api/set_root_pose) and ports."""
    import numpy as np
    z = np.asarray(zdir, float)
    nz = np.linalg.norm(z)
    if nz < 1e-12:
        return np.eye(3)
    z = z / nz
    ez = np.array([0.0, 0.0, 1.0])
    c = float(ez @ z)
    # guard 1/(1+c): near-antiparallel is a deterministic 180° flip about X
    if c < -1.0 + 1e-9:
        return np.diag([1.0, -1.0, -1.0])
    v = np.cross(ez, z)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)                         # already +Z aligned
    K = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1.0 / (1.0 + c))


def _zdir_to_rpy(zdir):
    """``_rot_z_to`` expressed as roll/pitch/yaw, for a port's fixed-joint
    origin (+Z = outgoing connector axis)."""
    import numpy as np

    from sw2robot.exporter.geometry import matrix_to_xyz_rpy
    M = np.eye(4)
    M[:3, :3] = _rot_z_to(zdir)
    _, rpy = matrix_to_xyz_rpy(M)
    return list(rpy)


def _parse_urdf(pkg_dir, urdf_rel):
    """Parse the served URDF via the editor's one canonical parser
    (``parse_urdf_content``) -- the same code path ``core.load_module`` uses --
    so link/joint listing and root-link detection don't drift between the
    server and the rest of the editor.  Returns the parsed dict or None."""
    from ._vendor.rc_config.urdf_parser import parse_urdf_content
    try:
        with open(os.path.join(pkg_dir, urdf_rel), encoding="utf-8") as f:
            return parse_urdf_content(f.read())
    except Exception:
        return None


def _rename_in_urdf(pkg_dir, urdf_rel, kind, old, new):
    """Apply a cosmetic link/joint rename DIRECTLY in the served URDF -- rewrite
    just the name and its references, no full rebuild.

    A rename is purely a display change: the kinematics, meshes and inertias are
    untouched, so re-running the whole build (which recomputes every link's
    convex-hull inertia) just to change a string is slow.  joints.yaml already
    carries the rename overlay, so a later full build reproduces the same names;
    this only makes the IMMEDIATE update instant.  Targets the exact ``name="..."``
    / ``link="..."`` attributes (quote-delimited) so it never hits a substring."""
    import re
    path = os.path.join(pkg_dir, urdf_rel)
    with open(path, encoding="utf-8") as f:
        txt = f.read()
    o = re.escape(old)
    if kind == "link":
        pats = [rf'(<link\s+name="){o}(")',
                rf'(<(?:parent|child)\s+link="){o}(")']
    else:
        pats = [rf'(<joint\s+name="){o}(")',
                rf'(<mimic\s+joint="){o}(")']
    n = 0
    for pat in pats:
        txt, k = re.subn(pat, lambda m: m.group(1) + new + m.group(2), txt)
        n += k
    with open(path, "w", encoding="utf-8") as f:
        f.write(txt)
    return n


def _fmt_num(v):
    """Compact URDF numeric attribute -- no ``-0.0``, trims trailing zeros."""
    return "0" if v == 0 else f"{v:.8g}"


def _flip_axis_in_urdf(pkg_dir, urdf_rel, joint_name):
    """Reverse one joint's rotation/translation SENSE directly in the served
    URDF: negate its ``<axis xyz>``, swap+negate ``<limit lower/upper>`` (the
    physical range is preserved -- only the command sign flips), and negate any
    ``<mimic multiplier/offset>``.  The joint ORIGIN is untouched so the frame
    does not move.  Self-inverse (flip twice = original).  Returns the number of
    axes flipped (0 when the joint has none -- e.g. fixed)."""
    import re
    path = os.path.join(pkg_dir, urdf_rel)
    with open(path, encoding="utf-8") as f:
        txt = f.read()
    jpat = re.compile(
        r'(<joint\b[^>]*\bname="' + re.escape(joint_name)
        + r'"[^>]*>)(.*?)(</joint>)', re.DOTALL)
    m = jpat.search(txt)
    if not m:
        return 0
    head, body, tail = m.group(1), m.group(2), m.group(3)

    def neg_vec(s):
        out = []
        for tok in s.split():
            try:
                out.append(_fmt_num(-float(tok)))
            except ValueError:
                return s
        return " ".join(out)

    axm = re.search(r'<axis\b[^>]*\bxyz="([^"]*)"', body)
    if axm is None:
        return 0                                  # no axis -> nothing to flip
    try:
        if not any(float(x) for x in axm.group(1).split()):
            return 0                              # zero axis (fixed) -> skip
    except ValueError:
        return 0
    body, na = re.subn(
        r'(<axis\b[^>]*\bxyz=")([^"]*)(")',
        lambda mm: mm.group(1) + neg_vec(mm.group(2)) + mm.group(3), body)

    def lim_repl(mm):                             # lower' = -upper, upper' = -lower
        seg = mm.group(0)
        lo = re.search(r'\blower="([^"]*)"', seg)
        hi = re.search(r'\bupper="([^"]*)"', seg)
        if lo and hi:
            lov, hiv = float(lo.group(1)), float(hi.group(1))
            seg = re.sub(r'\blower="[^"]*"', f'lower="{_fmt_num(-hiv)}"', seg)
            seg = re.sub(r'\bupper="[^"]*"', f'upper="{_fmt_num(-lov)}"', seg)
        elif lo:
            seg = re.sub(r'\blower="[^"]*"',
                         f'lower="{_fmt_num(-float(lo.group(1)))}"', seg)
        elif hi:
            seg = re.sub(r'\bupper="[^"]*"',
                         f'upper="{_fmt_num(-float(hi.group(1)))}"', seg)
        return seg
    body = re.sub(r'<limit\b[^>]*?>', lim_repl, body)

    def mim_repl(mm):                             # follower stays in phase: negate both
        # this joint is itself a FOLLOWER (q_f' = -q_f for its reversed axis):
        # q_f = M*q_d + off  ->  q_f' = (-M)*q_d + (-off), so negate both
        seg = mm.group(0)
        for attr in ("multiplier", "offset"):
            a = re.search(rf'\b{attr}="([^"]*)"', seg)
            if a:
                seg = re.sub(rf'\b{attr}="[^"]*"',
                             f'{attr}="{_fmt_num(-float(a.group(1)))}"', seg)
        return seg
    body = re.sub(r'<mimic\b[^>]*?>', mim_repl, body)

    full = txt[:m.start()] + head + body + tail + txt[m.end():]

    # this joint may also be the DRIVER that OTHER joints mimic -- their
    # <mimic joint="this"> tags live in THEIR blocks, not here, so flipping this
    # joint's axis must negate each follower's multiplier (offset unchanged):
    # q_f = M*q_d + off, and after q_d -> -q_d', q_f = (-M)*q_d' + off.
    def drv_repl(mm):
        seg = mm.group(0)
        a = re.search(r'\bmultiplier="([^"]*)"', seg)
        if a:
            seg = re.sub(r'\bmultiplier="[^"]*"',
                         f'multiplier="{_fmt_num(-float(a.group(1)))}"', seg)
        return seg
    full = re.sub(
        r'<mimic\b[^>]*\bjoint="' + re.escape(joint_name) + r'"[^>]*>',
        drv_repl, full)

    with open(path, "w", encoding="utf-8") as f:
        f.write(full)
    return na


def _set_attr_in_tag(seg, attr, val):
    """Set/replace one attribute in a single (self-closing) tag string."""
    import re
    if re.search(rf'\b{attr}="', seg):
        return re.sub(rf'\b{attr}="[^"]*"', f'{attr}="{val}"', seg)
    return re.sub(r'\s*/?>\s*$', f' {attr}="{val}"/>', seg, count=1)


def _edit_joint_block(pkg_dir, urdf_rel, child, edit):
    """Find the ``<joint>`` whose child link is ``child`` in the served URDF and
    rewrite its (head, body) via ``edit(head, body) -> (head, body)`` IN PLACE --
    the same instant, build-skipping pattern as :func:`_flip_axis_in_urdf` /
    :func:`_rename_in_urdf`, but matched by child link (what the limits/mimic UI
    sends).  Comments + formatting are preserved (regex, not an XML round-trip).
    Returns True if a joint was edited."""
    import re
    parsed = _parse_urdf(pkg_dir, urdf_rel)
    if not parsed:
        return False
    jname = next((j["name"] for j in parsed["joints"]
                  if j["childLink"] == child), None)
    if jname is None:
        return False
    path = os.path.join(pkg_dir, urdf_rel)
    with open(path, encoding="utf-8") as f:
        txt = f.read()
    jpat = re.compile(
        r'(<joint\b[^>]*\bname="' + re.escape(jname) + r'"[^>]*>)(.*?)(</joint>)',
        re.DOTALL)
    m = jpat.search(txt)
    if not m:
        return False
    head, body = edit(m.group(1), m.group(2))
    if head is None:
        return False                     # edit declined (e.g. wrong joint type)
    with open(path, "w", encoding="utf-8") as f:
        f.write(txt[:m.start()] + head + body + m.group(3) + txt[m.end():])
    return True


def _append_into_body(body, line):
    """Insert ``line`` (an indented element + newline) just before the body's
    trailing indentation (the run before ``</joint>``)."""
    import re
    cut = re.search(r'[ \t]*$', body).start()
    return body[:cut] + line + body[cut:]


def _body_indent(body):
    import re
    m = re.search(r'\n([ \t]+)<', body)
    return m.group(1) if m else "    "


def _set_limit_in_urdf(pkg_dir, urdf_rel, child, lower, upper, continuous):
    """Set a joint's ``<limit>`` (or make it continuous) directly in the served
    URDF, found by child link.  Skips the inertia-recomputing build (joints.yaml
    still persists the value).  Returns True if applied."""
    import re

    def edit(head, body):
        if continuous:
            head = re.sub(r'(\btype=")[^"]*(")', r'\1continuous\2', head)
            # continuous carries no lower/upper -- drop them, keep effort/velocity
            body = re.sub(
                r'<limit\b[^>]*?>',
                lambda mm: re.sub(r'\s*\b(?:lower|upper)="[^"]*"', '', mm.group(0)),
                body)
            return head, body
        tm = re.search(r'\btype="([^"]*)"', head)
        if tm and tm.group(1) not in ("revolute", "prismatic"):
            return None, None            # a limit only applies to revolute/prismatic
        lo, hi = _fmt_num(lower), _fmt_num(upper)
        lm = re.search(r'<limit\b[^>]*?>', body)
        if lm:
            seg = _set_attr_in_tag(lm.group(0), "lower", lo)
            seg = _set_attr_in_tag(seg, "upper", hi)
            body = body[:lm.start()] + seg + body[lm.end():]
        else:
            pad = _body_indent(body)
            body = _append_into_body(
                body, f'{pad}<limit lower="{lo}" upper="{hi}" '
                      f'effort="10" velocity="3.14"/>\n')
        return head, body

    return _edit_joint_block(pkg_dir, urdf_rel, child, edit)


def _set_mimic_in_urdf(pkg_dir, urdf_rel, child, master, multiplier, offset, clear):
    """Add / replace / remove a joint's ``<mimic>`` directly in the served URDF,
    found by child link (build-skipping; joints.yaml still persists it).  Returns
    True if the joint was found."""
    import re

    def edit(head, body):
        body = re.sub(r'[ \t]*<mimic\b[^>]*?>[ \t]*\n?', '', body)  # drop existing
        if not clear and master:
            pad = _body_indent(body)
            body = _append_into_body(
                body, f'{pad}<mimic joint="{master}" '
                      f'multiplier="{_fmt_num(multiplier)}" '
                      f'offset="{_fmt_num(offset)}"/>\n')
        return head, body

    return _edit_joint_block(pkg_dir, urdf_rel, child, edit)


def _urdf_names(pkg_dir, urdf_rel, tag):
    """Current ``<link>``/``<joint>`` names in the served URDF (the source of
    truth for what's on screen), for the rename collision check + root detect."""
    parsed = _parse_urdf(pkg_dir, urdf_rel)
    if parsed is None:
        return []
    return [e["name"] for e in parsed["links" if tag == "link" else "joints"]]


def _urdf_root_link(pkg_dir, urdf_rel):
    """The root link name (the link that is never a joint's child)."""
    parsed = _parse_urdf(pkg_dir, urdf_rel)
    return parsed["root_link"] if parsed else None


def _list_packages(root):
    out = []
    if not os.path.isdir(root):
        return out
    for name in sorted(os.listdir(root), key=str.lower):
        d = os.path.join(root, name)
        try:
            pkg, _rel = _resolve_package(d)
            out.append({"name": name, "path": pkg})
        except (ValueError, OSError):
            continue
    return out


# undo/redo: every persistent edit is a joints.yaml rewrite, so history is
# simply labelled snapshots of the yaml text, per package
_history = {}     # pkg_dir -> {"undo": [(label, text)], "redo": [...]}


def _hist(pkg_dir):
    return _history.setdefault(pkg_dir, {"undo": [], "redo": []})


def _snapshot(pkg_dir, yml, label):
    try:
        with open(yml, encoding="utf-8") as f:
            txt = f.read()
    except OSError:
        return
    h = _hist(pkg_dir)
    h["undo"].append((label, txt))
    del h["undo"][:-50]
    h["redo"].clear()


# one extraction job at a time (SolidWorks is a singleton resource anyway)
_job = {"running": False, "log": [], "error": None, "package": None,
        "cancel": False, "cancelled": False}
_job_lock = threading.Lock()


class _CancelExtract(Exception):
    """Raised from the progress callback to abort the extract cooperatively
    when the user asked to cancel (via /api/extract/cancel)."""
# warm SolidWorks session kept across extractions: starting SolidWorks is
# by far the slowest stage (~1-2 min), so pay it once per server lifetime
_sw = {"sess": None}

# We prefer to ATTACH to the user's running SolidWorks (no new process). When we
# can't (no attachable instance) we spawn a private one -- and that one can leak
# if the server is hard-killed (atexit never runs).  Record each PID we spawn to
# a file so the NEXT startup (and a clean shutdown) can reap exactly our own
# orphans, never the user's interactive SolidWorks.
_SW_PID_FILE = os.path.join(_DATA_DIR, "_sw_spawned_pids.txt")


def _sldworks_pids():
    """Set of PIDs of every running SLDWORKS.exe (empty on failure/non-Windows)."""
    import subprocess
    try:
        out = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq SLDWORKS.exe", "/FO", "CSV",
             "/NH"], capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return set()
    pids = set()
    for line in out.splitlines():
        parts = [p.strip().strip('"') for p in line.split('","')]
        if len(parts) >= 2 and parts[0].lower().startswith("sldworks"):
            try:
                pids.add(int(parts[1]))
            except ValueError:
                pass
    return pids


def _record_spawned_pid(pid):
    try:
        with open(_SW_PID_FILE, "a", encoding="utf-8") as f:
            f.write(f"{pid}\n")
    except OSError as e:
        print(f"[sw2robot.web] could not record spawned SW pid: {e!r}")


def _reap_spawned_sw():
    """Kill any SLDWORKS.exe THIS tool spawned in a previous/this run (matched
    by recorded PID, and only if it is still a live SLDWORKS.exe), then clear
    the record.  Never touches the user's interactive SolidWorks."""
    try:
        with open(_SW_PID_FILE, encoding="utf-8") as f:
            recorded = {int(x) for x in f.read().split() if x.strip().isdigit()}
    except OSError:
        return
    import subprocess
    for pid in recorded & _sldworks_pids():       # PID-reuse safe: must still be SW
        try:
            subprocess.run(["taskkill", "/PID", str(pid), "/F", "/T"],
                           capture_output=True, timeout=10)
            print(f"[sw2robot.web] reaped leftover spawned SolidWorks (pid {pid})")
        except Exception:
            pass
    try:
        os.remove(_SW_PID_FILE)
    except OSError:
        pass


def _warm_sw(progress):
    sess = _sw["sess"]
    if sess is not None:
        progress("checking the warm SolidWorks session (an idle session "
                 "can take a moment to respond) ...")
        t0 = time.time()
        for attempt in (1, 2, 3):    # transient RPC-busy is not death
            # _responds() makes a REAL call, so it catches a disconnected proxy
            # (CO_E_OBJNOTCONNECTED) -- e.g. a session whose creating thread has
            # ended -- which a bare Visible read can miss.  A dead one is dropped
            # and the caller starts a fresh instance on the current thread.
            alive = sess.app is not None and sess._responds()
            if alive:
                progress(f"warm session is alive ✓ (responded in "
                         f"{time.time() - t0:.1f}s)")
                return sess
            if attempt < 3:
                time.sleep(3)
        progress("previous SolidWorks session died; starting a new one ...")
        try:
            sess.shutdown()          # avoid leaving a zombie behind
        except Exception:
            pass
        _sw["sess"] = None
    return None


def _keepalive_loop():
    """Ping the warm session once a minute while idle, so Windows doesn't
    page it out and the next extraction starts instantly.

    A single failed COM call does NOT mean the session is dead -- a busy
    RPC rejects transiently, and discarding a LIVE session both forces a
    full relaunch on the next extraction (~20 s) and leaves the old
    process behind as a zombie.  Declare death only after 3 consecutive
    failures, and then try to shut the process down for real."""
    fails = 0
    while True:
        time.sleep(60)
        sess = _sw.get("sess")
        if sess is None or _job["running"]:
            fails = 0
            continue
        alive = False
        try:
            alive = sess.app is not None and sess._responds()
        except Exception:
            alive = False
        if alive:
            fails = 0
            continue
        fails += 1
        if fails < 3:
            print(f"[sw2robot.web] warm session ping failed "
                  f"({fails}/3) -- retrying before declaring it dead")
            continue
        print("[sw2robot.web] warm SolidWorks session died while idle; "
              "next extraction starts a fresh one")
        try:
            sess.shutdown()          # avoid leaving a zombie behind
        except Exception:
            pass
        _sw["sess"] = None
        fails = 0


def _shutdown_sw():
    sess = _sw.get("sess")
    if sess is not None:
        try:
            sess.shutdown()        # attach mode leaves the user's SW alone
        except Exception:
            pass
        _sw["sess"] = None
    _reap_spawned_sw()             # belt-and-suspenders: kill any we spawned


def _run_extract(sldasm):
    """Background thread: SolidWorks extract + build -> module package."""
    def progress(msg):
        # cooperative cancel: the extract calls progress() between COM steps
        # (per phase, per mesh), so raising here aborts at the next checkpoint
        if _job.get("cancel"):
            raise _CancelExtract()
        _job["log"].append(str(msg))
        print(f"[sw2robot.web] extract: {msg}")

    # COM calls (launch, first contact with an idle session, OpenDoc6
    # loading a big assembly) block with no events for tens of seconds to
    # minutes; whenever the log goes silent for 10 s, fill the silence with
    # elapsed time + the phase we are stuck in, for the WHOLE job
    hb_stop = threading.Event()

    def heartbeat():
        import subprocess
        t0 = time.time()
        real = lambda: [ln for ln in _job["log"]
                        if not ln.startswith("... still")]
        last_n, last_t = len(real()), time.time()
        while not hb_stop.wait(10.0):
            n = len(real())
            if n != last_n:               # progress flowed; stay quiet
                last_n, last_t = n, time.time()
                continue
            try:
                out = subprocess.run(
                    ["tasklist", "/FI", "IMAGENAME eq SLDWORKS.exe",
                     "/FO", "CSV", "/NH"],
                    capture_output=True, text=True, timeout=10).stdout
                detail = f"{out.count('SLDWORKS.exe')} SolidWorks " \
                         f"process(es) alive"
            except Exception:
                detail = "process count unavailable"
            phase = (real() or ["starting"])[-1]
            _job["log"].append(f"... still waiting "
                               f"({int(time.time() - last_t)}s in this "
                               f"phase, {int(time.time() - t0)}s total, "
                               f"{detail}) -- phase: {phase[:70]}")

    threading.Thread(target=heartbeat, daemon=True).start()
    try:
        from . import core
        progress(f"extracting {os.path.basename(sldasm)} -- this opens a "
                 f"throwaway COPY in a hidden SolidWorks instance "
                 f"(the original is never modified)")
        sw = _warm_sw(progress)
        if sw is None:
            from sw2robot.exporter.swcom import SolidWorks
            # Reuse the user's ALREADY-RUNNING SolidWorks if we can attach to it
            # (same login session): no new process to start (instant, no ~1 min
            # cold start) and nothing to leak.  The throwaway copy is opened in
            # that instance and closed afterwards; the user's own documents are
            # never modified.
            try:
                progress("attaching to the running SolidWorks ...")
                sw = SolidWorks(attach=True)
                progress("attached to the running SolidWorks ✓ (reusing it)")
            except Exception:
                progress("no attachable SolidWorks; starting a private instance "
                         "(this can take a minute; later extractions reuse it) ...")
                # Create the COM object in THIS thread.  SolidWorks is STA-bound:
                # an instance built on another thread (e.g. a timeout worker)
                # loses its apartment when that thread ends, so OpenDoc6 then
                # fails with CO_E_OBJNOTCONNECTED ("object not connected").
                before = _sldworks_pids()
                sw = SolidWorks(visible=False)
                for pid in _sldworks_pids() - before:   # the one(s) we just spawned
                    _record_spawned_pid(pid)
            _sw["sess"] = sw
        state = core.extract_and_import(
            sldasm, out_dir=_Handler.root_dir, progress=progress, sw=sw)
        _job["package"] = str(state.package_dir)
        _preconvert_meshes(str(state.package_dir))
        progress(f"done -> {state.package_dir} (SolidWorks kept warm for "
                 f"the next extraction)")
    except _CancelExtract:
        # the COM sequence was aborted mid-flight, so the warm session may hold
        # a half-open doc -- tear it down so the next extraction starts clean
        _job["cancelled"] = True
        _job["log"].append("cancelled by user; resetting SolidWorks session ...")
        print("[sw2robot.web] extract CANCELLED by user")
        try:
            _shutdown_sw()
        except Exception:
            pass
        _job["log"].append("cancelled.")
    except Exception as e:
        import traceback

        from sw2robot.exporter.swcom import SolidWorksUnavailable
        _job["error"] = f"{type(e).__name__}: {e}"
        print(f"[sw2robot.web] extract FAILED: {e!r}")
        traceback.print_exc()     # full traceback -> the exact failing line
        # Drop the cached session on anything that smells like a lost/failed
        # SolidWorks connection -- a couldn't-start/license error, a COM RPC
        # failure, or a dead instance -- so the NEXT extraction starts clean
        # and recovers the moment the license server is back (instead of
        # forever reusing a wedged session).
        if (isinstance(e, SolidWorksUnavailable)
                or "-2147417848" in repr(e) or "-2147417856" in repr(e)
                or "com_error" in type(e).__name__.lower()
                or "-21474" in repr(e)):
            _shutdown_sw()         # tear the wedged session down, then forget it
    finally:
        hb_stop.set()
        _job["running"] = False


# ---- live self-collision (autoinit.SelfCollision over the current URDF) --
# Built once per (urdf, mtime) in a background thread (~5 s: skrobot model +
# per-link convex hulls + the rest-pose baseline); a pose query is ~90 ms.
# Contacts present at the REST pose and parent/child adjacency are the
# allowed baseline -- only NEW colliding pairs are reported.
_coll_lock = threading.Lock()
_coll = {"key": None, "ctx": None, "building": False, "error": None}


def _build_collision(urdf_path, key, pkg_dir=None):
    try:
        import numpy as np
        from skrobot.models.urdf import RobotModelFromURDF

        from . import autoinit
        robot = RobotModelFromURDF(urdf_file=urdf_path)
        meshes = autoinit.link_meshes(robot)
        # prefer the CoACD convex parts (generated via the export panel) when
        # present: accurate AND convex-fast, so no exact-mesh confirm is needed.
        # Otherwise fall back to hull broadphase + exact-mesh verification, which
        # matches the exact-mesh limit sweep (fat hulls alone light up red before
        # the joint reaches its limit).
        parts = {}
        if pkg_dir:
            preview = os.path.join(pkg_dir, "meshes", ".coacd_cache", "preview")
            parts = autoinit.load_coacd_parts(preview, list(meshes))
        if parts:
            sc = autoinit.SelfCollision(robot, meshes, parts=parts)
        else:
            sc = autoinit.SelfCollision(robot, meshes, confirm=True)
        joints = {}
        for j in robot.joint_list:
            if type(j).__name__ in ("RotationalJoint", "LinearJoint"):
                try:    # widen limits: the page may preview beyond them
                    j.min_angle, j.max_angle = -4 * np.pi, 4 * np.pi
                except Exception:
                    pass
                joints[j.name] = j
        with _coll_lock:
            if _coll["key"] == key:      # not re-targeted meanwhile
                _coll.update(ctx={"sc": sc, "joints": joints},
                             building=False, error=None)
        print(f"[sw2robot.web] collision model ready: "
              f"{len(parts)} CoACD-part links + "
              f"{len(meshes) - len(parts)} hulls, "
              f"{len(sc.baseline)} baseline pairs")
    except Exception as e:
        with _coll_lock:
            if _coll["key"] == key:
                _coll.update(ctx=None, building=False, error=repr(e))
        print(f"[sw2robot.web] collision model FAILED: {e!r}")


# ---- CoACD collision-mesh generation (background, observable) ---------------
# CoACD decomposition is slow (tens of seconds per link), so run it as a
# background job that produces a colour-coded preview GLB per link as it goes;
# the client polls progress and pops each link's collision mesh into the viewer
# as it lands.  Parts are cached on disk, so a later collision='coacd' export is
# instant.  One job at a time.
_coacd_job = {"running": False, "done": 0, "total": 0, "current": None,
              "inflight": [], "parts": {}, "error": None, "quality": None,
              "cancel": False, "cancelled": False}
_coacd_lock = threading.Lock()


def _reset_coacd_job():
    with _coacd_lock:
        _coacd_job.update(running=False, done=0, total=0, current=None,
                          inflight=[], parts={}, error=None, quality=None,
                          cancel=False, cancelled=False)


def _run_coacd_job(pkg_dir, robot_name, urdf_rel, quality):
    from sw2robot.exporter.ros_export import coacd_preview_glbs

    def _on_start(link):                  # a link's decomposition began
        with _coacd_lock:
            if link not in _coacd_job["inflight"]:
                _coacd_job["inflight"].append(link)
            _coacd_job["current"] = link

    def _progress(done, total, link, rel):    # a link finished
        with _coacd_lock:
            _coacd_job["done"] = done
            _coacd_job["total"] = total
            if link in _coacd_job["inflight"]:
                _coacd_job["inflight"].remove(link)
            if rel:
                _coacd_job["parts"][link] = "/pkg/" + rel

    def _should_cancel():
        with _coacd_lock:
            return _coacd_job["cancel"]
    try:
        # materialise URDF-mode edits to disk for the job (no-op in CAD mode),
        # restoring the pristine base when it ends
        with _um_materialized(pkg_dir, urdf_rel):
            coacd_preview_glbs(pkg_dir, robot_name, quality=quality,
                               progress=_progress, on_start=_on_start,
                               should_cancel=_should_cancel)
        with _coacd_lock:
            stopped = _coacd_job["cancel"]
            _coacd_job.update(running=False, current=None, inflight=[],
                              cancelled=stopped)
        # the live self-collision model can now use these convex parts -- drop
        # the current one so the next /api/collision/init rebuilds with them
        with _coll_lock:
            _coll.update(key=None, ctx=None)
        print(f"[sw2robot.web] coacd preview "
              f"{'cancelled' if stopped else 'ready'}: "
              f"{len(_coacd_job['parts'])} links")
    except Exception as e:
        with _coacd_lock:
            _coacd_job.update(running=False, error=repr(e))
        print(f"[sw2robot.web] coacd preview FAILED: {e!r}")


# ---- auto joint limits: self-collision sweep over the live collision model.
# Coarse linear scan brackets the first new self-collision, then a bisection
# refines the boundary (the user's "binary search" idea -- far fewer queries
# than fine stepping, and more precise).  One job at a time; it holds the
# collision lock while it sweeps (it mutates joint angles), so the live drag
# check pauses for its duration.
_limjob = {"running": False, "log": [], "error": None, "results": None,
           "n": 0, "total": 0, "joint": None, "phase": None}
_limjob_lock = threading.Lock()


def _run_auto_limits(pkg_dir, urdf_rel, step_deg, max_deg,
                     margin_deg=2.0, margin_mm=2.0):
    """Run the self-collision limit sweep in a SUBPROCESS and return
    ``(results_list, error)``.  A subprocess on purpose: the sweep is CPU-bound
    and releases the GIL (numpy / fcl), so running it inside the threaded HTTP
    server makes it thrash the GIL against the browser's idle keep-alive
    threads -- the CPython convoy -- which inflated an 8 s sweep to ~90 s just
    by having a page open.  A fresh process has its own GIL and no server
    threads, so it stays at the true ~8 s (+~3 s to load the model)."""
    import importlib.util
    import subprocess
    urdf = os.path.join(pkg_dir, urdf_rel)
    if not os.path.exists(urdf):
        return None, "URDF not found"
    # the sweep needs the optional self-collision extra; the default .exe build
    # ships without it (skrobot/fcl excluded to stay small).  Check up front so
    # the user gets a clear reason instead of a subprocess ImportError tail.
    if importlib.util.find_spec("skrobot") is None \
            or importlib.util.find_spec("fcl") is None:
        return None, ("auto joint limits need the self-collision extra "
                      "(skrobot + fcl), which this build does not include. "
                      "Rebuild the .exe with `build_exe.py --with-ui`, or run "
                      "from a source checkout with the `ui` extra installed.")
    # a frozen .exe has no `python -m`; re-invoke the exe itself with a sentinel
    # (webserver.main dispatches it to _autolimits_cli).  A source run uses -m.
    if getattr(sys, "frozen", False):
        cmd = [sys.executable, "__autolimits__"]
        cwd = _DATA_DIR
    else:
        cmd = [sys.executable, "-m", "sw2robot.editor._autolimits_cli"]
        cwd = PROJECT_ROOT
    cmd += [urdf, str(step_deg), str(max_deg), str(margin_deg), str(margin_mm)]
    _t0 = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, cwd=cwd)

    # Drain stderr LIVE in a thread: the CLI emits per-joint progress there as
    # JSON lines (stdout carries only the final results JSON).  Reading it as it
    # flows both feeds the UI progress bar via _limjob and keeps the pipe from
    # filling.  Polling is GIL-safe now -- the sweep runs in this child process,
    # not in a server thread (the whole reason it is a subprocess).
    def _pump():
        for line in proc.stderr:
            line = line.strip()
            if not line:
                continue
            try:
                ev = json.loads(line)
            except Exception:
                with _limjob_lock:        # non-JSON (skrobot warnings): keep a tail
                    _limjob["log"].append(line)
                    del _limjob["log"][:-50]
                continue
            with _limjob_lock:
                kind = ev.get("event")
                if kind == "loading":
                    _limjob["phase"] = "loading"
                elif kind == "start":
                    _limjob.update(phase="sweeping", total=ev.get("total", 0),
                                   n=0)
                elif kind == "joint":
                    _limjob.update(n=ev.get("i", 0),
                                   total=ev.get("total", _limjob["total"]),
                                   joint=ev.get("joint"))

    pump = threading.Thread(target=_pump, daemon=True)
    pump.start()
    # Drain stdout in THIS thread while the pump drains stderr -- both pipes are
    # read concurrently, so neither can fill and deadlock the child.  A watchdog
    # enforces the timeout by killing the child (which gives stdout.read() its
    # EOF); we then always wait() to reap it and join the pump after its stderr
    # hits EOF, so a stale pump can never bleed into the next job's _limjob.
    timed_out = {"v": False}

    def _watchdog():
        timed_out["v"] = True
        proc.kill()

    timer = threading.Timer(900, _watchdog)
    timer.start()
    try:
        out_txt = proc.stdout.read()
    finally:
        timer.cancel()
        proc.wait()
        pump.join(timeout=2)
    if timed_out["v"]:
        return None, "sweep timed out"
    if proc.returncode != 0:
        with _limjob_lock:
            tail = _limjob["log"][-3:]
        return None, "sweep failed: " + " | ".join(tail)
    try:
        out = json.loads(out_txt)["results"]
    except Exception as e:
        return None, f"bad sweep output: {e}"
    print(f"[sw2robot.web] auto_limits sweep: {time.time() - _t0:.1f}s "
          f"({len(out)} joints, subprocess)", flush=True)
    return out, None


def _set_joint_limit(txt, child, lower, upper, continuous):
    """Edit one joint block in joints.yaml (matched by CHILD), preserving the
    inline ``# mates: ...`` comment.  continuous → set type + drop limits;
    else keep the type and write/replace lower/upper.  Returns (text, n)."""
    import re
    pat = re.compile(
        r"(- parent:\s*\S+\n)(\s*)child:(\s*)" + re.escape(child) +
        r"\s*\n(\s*)type:(\s*)(\S+)([^\n]*)\n"
        r"((?:[ \t]*(?:lower|upper):[^\n]*\n)*)")

    def repl(m):
        parent_l, ci, cs, ti, ts, typ, comment, _existing = m.groups()
        if continuous:
            return (f"{parent_l}{ci}child:{cs}{child}\n"
                    f"{ti}type:{ts}continuous{comment}\n")
        return (f"{parent_l}{ci}child:{cs}{child}\n"
                f"{ti}type:{ts}{typ}{comment}\n"
                f"{ti}lower: {lower:.5f}\n{ti}upper: {upper:.5f}\n")

    return pat.subn(repl, txt)


class _Handler(http.server.BaseHTTPRequestHandler):
    pkg_dir = None      # current package; set by serve()/api_open
    urdf_rel = None
    robot_name = None
    root_dir = None

    # The live collision query is 33 ms of real work but cost ~250 ms per
    # drag step: a delayed-ACK / Nagle stall on the small POST exchange.
    # Three belts: HTTP/1.1 keep-alive (the browser reuses ONE connection
    # for the whole drag instead of reopening per query -- the big win),
    # TCP_NODELAY (no Nagle hold), and a buffered wfile (headers + body
    # leave in one send() instead of two).  Every response here sets a
    # Content-Length, which keep-alive requires.
    protocol_version = "HTTP/1.1"
    wbufsize = -1

    def setup(self):
        super().setup()
        try:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    # -- helpers ----------------------------------------------------------
    def _send_bytes(self, data, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj, code=200):
        self._send_bytes(json.dumps(obj).encode(), "application/json", code)

    def _send_file(self, path):
        ext = os.path.splitext(path)[1].lower()
        with open(path, "rb") as f:
            self._send_bytes(f.read(), _CTYPES.get(ext,
                                                   "application/octet-stream"))

    def _resolve(self, root, rel):
        """Join + normalise; return None on directory escape."""
        rel = posixpath.normpath(urllib.parse.unquote(rel)).lstrip("/")
        if rel.startswith(".."):
            return None
        full = os.path.normpath(os.path.join(root, *rel.split("/")))
        if not full.startswith(os.path.normpath(root)):
            return None
        return full

    def _info(self):
        cls = type(self)
        if not cls.urdf_rel:
            return {"name": None, "urdf": None, "mode": None}
        # 'cad' = joints.yaml + build() path; 'urdf' = direct overlay editing
        # (the frontend gates URDF-only controls such as inertial editing on this)
        return {"name": cls.robot_name, "urdf": "/pkg/" + cls.urdf_rel,
                "mode": "cad" if _cad_mode(cls.pkg_dir) else "urdf"}

    def _um_reply(self, fn, *args):
        """Run a URDF-mode edit and JSON-reply, turning a bad-input error into a
        400 the editor surfaces (rather than the generic 500) -- TypeError covers
        a malformed body, e.g. a null in the com/inertia arrays.  Snapshots the
        pre-edit overlay for undo, but only once the edit actually succeeds."""
        snap = (_um_overlay_json(_um["state"])
                if _um["state"] is not None else None)
        try:
            result = fn(_um["state"], *args)
        except (ValueError, TypeError) as e:
            return self._send_json({"error": str(e)}, 400)
        if snap is not None:
            _um["undo"].append(snap)
            del _um["undo"][:-50]
            _um["redo"].clear()
        return self._send_json(result)

    # -- routes -----------------------------------------------------------
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path, query = parsed.path, urllib.parse.parse_qs(parsed.query)
        cls = type(self)
        try:
            if path in ("/", "/index.html"):
                return self._send_file(os.path.join(WEB_DIR, "index.html"))
            if path == "/api/info":
                return self._send_json(self._info())
            if path == "/api/list":
                return self._send_json(_list_packages(cls.root_dir))
            if path == "/api/version":
                # current build vs the latest GitHub Release (cached; ?force=1
                # re-checks).  Drives the in-browser update banner / version chip.
                from . import update
                return self._send_json(
                    update.check_for_update(force="force" in query))
            if path == "/api/update/status":
                from . import update
                return self._send_json(update.update_status())
            if path == "/api/recent":
                from . import core
                return self._send_json(core.sw_recent_assemblies())
            if path == "/api/root_pose":
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                txt = open(yml, encoding="utf-8").read() \
                    if os.path.exists(yml) else ""
                rpy, xyz, z0 = _read_root_pose(txt)
                xyz = [xyz[0], xyz[1], xyz[2] + z0]
                m = re.search(r"(?m)^base:\s*(\S+)", txt)
                return self._send_json({"rpy": rpy, "xyz": xyz,
                                        "base": m.group(1) if m else None})
            if path == "/api/components":
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):       # URDF-input mode
                    return self._send_json(_um_components(_um["state"]))
                from sw2robot.exporter.state import GraphState
                gs = GraphState.load(
                    os.path.join(cls.pkg_dir, "graph.json"))
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                overrides = {}
                if os.path.exists(yml):
                    txt = open(yml, encoding="utf-8").read()
                    m = re.search(r"(?m)^densities:\n((?:[ \t]+\S+:.*\n)*)",
                                  txt)
                    if m:
                        for ln in m.group(1).splitlines():
                            k, _, v = ln.strip().partition(":")
                            try:
                                overrides[k] = float(v)
                            except ValueError:
                                pass
                colors = _read_colors(cls.pkg_dir, cls.urdf_rel)
                links = {}
                for c in gs.components:
                    links[c.link_name] = {
                        "material": c.material, "density": c.density,
                        "name": c.name,
                        "override": overrides.get(c.link_name),
                        "color": colors.get(c.link_name)}
                excluded = []
                if os.path.exists(yml):
                    m = re.search(r"(?m)^exclude:\n((?:- .*\n)*)",
                                  open(yml, encoding="utf-8").read())
                    if m:
                        excluded = [ln[2:].strip()
                                    for ln in m.group(1).splitlines()]
                # raw colour overrides keyed by URDF/viewer link name (composed
                # sub-links like 'linkB_1__part_1' are NOT in `links`, which is
                # built per top-level component, so the viewer keys colours off
                # this map directly instead of compMeta)
                return self._send_json({"links": links,
                                        "excluded": excluded,
                                        "colors": colors})
            if path == "/api/fs":
                # tiny server-side file browser: the OS file dialog cannot
                # hand a PATH to the page, so the page browses through us
                target = (query.get("path") or [""])[0]
                if not target:
                    roots = []
                    import string
                    for d in string.ascii_uppercase:
                        if os.path.exists(f"{d}:\\"):
                            roots.append(f"{d}:\\")
                    if _Handler.root_dir and os.path.isdir(_Handler.root_dir) \
                            and _Handler.root_dir not in roots:
                        roots.append(_Handler.root_dir)
                    return self._send_json(
                        {"path": "", "parent": None,
                         "dirs": [{"name": r, "path": r, "package": False}
                                  for r in roots],
                         "files": []})
                p = os.path.abspath(target)
                if not os.path.isdir(p):
                    return self._send_json({"error": f"not a dir: {p}"},
                                           400)
                dirs, files = [], []
                try:
                    for e in sorted(os.listdir(p), key=str.lower):
                        full = os.path.join(p, e)
                        if e.startswith(("~$", ".")):
                            continue
                        if os.path.isdir(full):
                            pkg = os.path.isdir(os.path.join(full, "urdf"))
                            dirs.append({"name": e, "path": full,
                                         "package": pkg})
                        elif e.lower().endswith((".sldasm", ".urdf")):
                            files.append({"name": e, "path": full})
                except OSError as e:
                    return self._send_json({"error": str(e)}, 400)
                parent = os.path.dirname(p.rstrip("\\/"))
                return self._send_json(
                    {"path": p,
                     "parent": parent if parent != p else None,
                     "dirs": dirs[:400], "files": files[:400]})
            if path == "/api/history":
                cls = type(self)
                if cls.pkg_dir and not _cad_mode(cls.pkg_dir):   # URDF-mode stack
                    return self._send_json(
                        {"undo": ["edit"] * len(_um["undo"]),
                         "redo": ["edit"] * len(_um["redo"])})
                h = _hist(cls.pkg_dir) if cls.pkg_dir else {"undo": [],
                                                            "redo": []}
                return self._send_json(
                    {"undo": [l for l, _t in h["undo"]],
                     "redo": [l for l, _t in h["redo"]]})
            if path == "/api/swstatus":
                from . import core
                return self._send_json(core.sw_session_status())
            if path == "/api/release_sw":
                # cleanly close the warm session (call BEFORE killing the
                # server process, which would orphan SolidWorks instead)
                had = _sw.get("sess") is not None
                _shutdown_sw()
                return self._send_json({"released": had})
            if path == "/api/collision/init":
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                # URDF-input mode: collide against the live overlay (type/axis
                # edits included), not the pristine on-disk base
                urdf_path, _ = _um_live_urdf(cls.pkg_dir, cls.urdf_rel)
                key = (urdf_path, os.path.getmtime(urdf_path))
                with _coll_lock:
                    if _coll["key"] != key or (
                            _coll["ctx"] is None and not _coll["building"]
                            and not _coll["error"]):
                        _coll.update(key=key, ctx=None, building=True,
                                     error=None)
                        threading.Thread(target=_build_collision,
                                         args=(urdf_path, key, cls.pkg_dir),
                                         daemon=True).start()
                    ready = _coll["ctx"] is not None
                    return self._send_json({
                        "ready": ready, "building": _coll["building"],
                        "error": _coll["error"],
                        "baseline": (len(_coll["ctx"]["sc"].baseline)
                                     if ready else None)})
            if path == "/api/auto_limits":
                # SUBPROCESS sweep (avoids the GIL convoy; see _run_auto_limits).
                # Started ASYNC so the page can poll /api/auto_limits/status for
                # per-joint progress while it runs (the sweep is in the child
                # process, so polling no longer thrashes its GIL).  One at a time.
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                # parse + materialize BEFORE claiming the job, so a bad step/max
                # or a live-URDF write failure can't wedge the "running" flag
                # (no worker would start to clear it)
                try:
                    step = float((query.get("step") or ["10"])[0])
                    mx = float((query.get("max") or ["360"])[0])  # ±2π
                    # backoff margins from the colliding edge: deg / mm
                    margin_deg = float((query.get("margin_deg") or ["2"])[0])
                    margin_mm = float((query.get("margin_mm") or ["2"])[0])
                except ValueError:
                    return self._send_json({"error": "bad step/max/margin"}, 400)
                # URDF-input mode: sweep the live overlay (type edits included)
                try:
                    pkg = cls.pkg_dir
                    _, rel = _um_live_urdf(cls.pkg_dir, cls.urdf_rel)
                except Exception as e:
                    return self._send_json(
                        {"error": f"could not prepare URDF: {e}"}, 500)
                with _limjob_lock:
                    if _limjob["running"]:
                        return self._send_json(
                            {"error": "a limit sweep is already running"}, 409)
                    _limjob.update(running=True, log=[], error=None,
                                   results=None, n=0, total=0, joint=None,
                                   phase="loading")

                # NB: must NOT be named `_job` -- that shadows the module-global
                # `_job` (the extraction job) across this whole method and breaks
                # the /api/extract* handlers with an UnboundLocalError.
                def _sweep_job():
                    try:
                        results, err = _run_auto_limits(
                            pkg, rel, step, mx, margin_deg, margin_mm)
                    except Exception as e:           # never leave it "running"
                        results, err = None, f"{type(e).__name__}: {e}"
                    with _limjob_lock:
                        _limjob.update(results=results, error=err,
                                       running=False)

                threading.Thread(target=_sweep_job, daemon=True).start()
                return self._send_json({"started": True})
            if path == "/api/auto_limits/status":
                with _limjob_lock:
                    return self._send_json(
                        {"running": _limjob["running"], "n": _limjob["n"],
                         "total": _limjob["total"], "joint": _limjob["joint"],
                         "phase": _limjob["phase"], "error": _limjob["error"],
                         "results": _limjob["results"],
                         "done": not _limjob["running"]})
            if path == "/api/extract":
                target = (query.get("path") or [""])[0]
                target = os.path.abspath(os.path.expanduser(
                    target.strip().strip('"')))
                if not (os.path.isfile(target)
                        and target.lower().endswith(".sldasm")):
                    return self._send_json(
                        {"error": f"not a .sldasm file: {target}"}, 400)
                with _job_lock:
                    if _job["running"]:
                        return self._send_json(
                            {"error": "an extraction is already running"},
                            409)
                    _job.update(running=True, log=[], error=None,
                                package=None, cancel=False, cancelled=False)
                threading.Thread(target=_run_extract, args=(target,),
                                 daemon=True).start()
                return self._send_json({"started": True})
            if path == "/api/extract/status":
                return self._send_json(_job)
            if path == "/api/extract/cancel":
                # cooperative: the extract thread checks _job["cancel"] at its
                # next progress() checkpoint and aborts (see _run_extract)
                running = _job["running"]
                if running:
                    _job["cancel"] = True
                return self._send_json({"cancelling": running})
            if path == "/api/open":
                target = (query.get("path") or [""])[0]
                try:
                    pkg, rel = _resolve_package(target)
                except ValueError as e:
                    return self._send_json({"error": str(e)}, 400)
                cls.pkg_dir, cls.urdf_rel = pkg, rel
                cls.robot_name = os.path.splitext(os.path.basename(rel))[0]
                # drop the previous package's overlay + its hidden live URDF, then
                # load the new one (plain URDF -> in-memory overlay; a CAD package
                # uses the joints.yaml + build() path, no overlay)
                _um_close()
                _reset_coacd_job()        # drop the previous package's preview
                if not _cad_mode(pkg):
                    _um_load(pkg, rel)
                print(f"[sw2robot.web] open: {cls.robot_name} ({pkg})"
                      f"{'' if _cad_mode(pkg) else ' [urdf-input mode]'}")
                _preconvert_meshes(pkg)
                return self._send_json(self._info())
            if path == "/api/export/zip":
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                fmt = (query.get("meshes") or ["dae"])[0]
                if fmt not in ("dae", "glb"):
                    return self._send_json(
                        {"error": f"unsupported mesh format: {fmt}"}, 400)
                ros = (query.get("ros") or ["1"])[0]
                if ros not in ("1", "2"):
                    return self._send_json(
                        {"error": f"unsupported ros version: {ros}"}, 400)
                ros_version = int(ros)
                collision = (query.get("collision") or ["copy"])[0]
                if collision not in ("copy", "coacd"):
                    return self._send_json(
                        {"error": f"unsupported collision mode: {collision}"}, 400)
                cquality = (query.get("cquality") or ["balanced"])[0]
                if cquality not in ("balanced", "fine"):
                    return self._send_json(
                        {"error": f"unsupported collision quality: {cquality}"},
                        400)
                merge_fixed = (query.get("mergefixed") or ["0"])[0] == "1"
                pkg_name = (query.get("name") or [""])[0].strip() or None
                urdf_name = (query.get("urdf") or [""])[0].strip() or None
                mesh_dir = (query.get("meshdir") or [""])[0].strip() or None
                # the ROS exporter hardcodes the urdf/<robot_name>.urdf + meshes/
                # layout; in URDF-input mode the opened file may sit elsewhere, so
                # fail clearly instead of with a confusing missing-file 500
                if not _cad_mode(cls.pkg_dir) \
                        and os.path.normpath(os.path.join(cls.pkg_dir, cls.urdf_rel)) \
                        != os.path.normpath(os.path.join(
                            cls.pkg_dir, "urdf", cls.robot_name + ".urdf")):
                    return self._send_json(
                        {"error": "export needs the standard "
                         "<pkg>/urdf/<name>.urdf + <pkg>/meshes/ layout; open the "
                         "URDF from inside a urdf/ folder"}, 400)
                try:
                    # URDF-input mode keeps the on-disk URDF pristine and serves
                    # edits live; materialize them so the exporter picks them up
                    colors = (_read_colors(cls.pkg_dir, cls.urdf_rel)
                              if _cad_mode(cls.pkg_dir)
                              else _um_colors(_um["state"]))
                    with _um_materialized(cls.pkg_dir, cls.urdf_rel):
                        pkg, data = _export_zip(cls.pkg_dir, cls.robot_name, fmt,
                                                ros_version=ros_version,
                                                pkg_name=pkg_name,
                                                urdf_name=urdf_name,
                                                colors=colors,
                                                collision=collision,
                                                collision_quality=cquality,
                                                merge_fixed=merge_fixed,
                                                mesh_dir=mesh_dir)
                except ValueError as e:
                    return self._send_json({"error": str(e)}, 400)
                fname = (f"{cls.robot_name}_glb.zip" if fmt == "glb"
                         else f"{pkg}.zip")
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{fname}"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return None
            if path == "/api/launch_it.sh":
                # one-liner build+launch (robot-compiler style):
                #   curl -s http://<host>/api/launch_it.sh | bash
                # downloads the ROS 2 package zip from /api/export/zip, builds it
                # with colcon and brings up display.launch.py
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                from sw2robot.exporter.ros_export import ros_pkg_name
                pkg_name = (query.get("name") or [""])[0].strip() or None
                pkg = ros_pkg_name(cls.robot_name, pkg_name)
                host = self.headers.get("Host") or "localhost:8090"
                zip_q = "ros=2&meshes=dae"
                if pkg_name:
                    zip_q += "&name=" + pkg_name
                zip_url = f"http://{host}/api/export/zip?{zip_q}"
                script = _LAUNCH_IT_SH.format(pkg=pkg, zip_url=zip_url)
                body = script.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/x-shellscript")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return None
            if path == "/api/collision/coacd/init":
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                quality = (query.get("quality") or ["balanced"])[0]
                if quality not in ("balanced", "fine"):
                    return self._send_json(
                        {"error": f"unsupported collision quality: {quality}"},
                        400)
                # same standard-layout requirement as the ZIP export: the
                # generator reads urdf/<name>.urdf + meshes/
                if not _cad_mode(cls.pkg_dir) \
                        and os.path.normpath(os.path.join(cls.pkg_dir, cls.urdf_rel)) \
                        != os.path.normpath(os.path.join(
                            cls.pkg_dir, "urdf", cls.robot_name + ".urdf")):
                    return self._send_json(
                        {"error": "collision generation needs the standard "
                         "<pkg>/urdf/<name>.urdf + <pkg>/meshes/ layout"}, 400)
                with _coacd_lock:
                    if _coacd_job["running"]:
                        return self._send_json({"error": "already running"}, 409)
                _reset_coacd_job()
                with _coacd_lock:
                    _coacd_job.update(running=True, quality=quality)
                threading.Thread(
                    target=_run_coacd_job,
                    args=(cls.pkg_dir, cls.robot_name, cls.urdf_rel, quality),
                    daemon=True).start()
                return self._send_json({"running": True, "quality": quality})
            if path == "/api/collision/coacd/cancel":
                # request stop; the job ends at the next link boundary (CoACD
                # itself is not interruptible mid-link)
                with _coacd_lock:
                    if _coacd_job["running"]:
                        _coacd_job["cancel"] = True
                return self._send_json({"cancelling": True})
            if path == "/api/collision/coacd/status":
                with _coacd_lock:
                    return self._send_json(dict(_coacd_job))
            if path.startswith("/pkg/"):
                if not cls.pkg_dir:
                    return self.send_error(404, "no package open")
                rel = path[len("/pkg/"):]
                is_urdf = urllib.parse.unquote(rel) == cls.urdf_rel
                # ?merged=1 -> serve the fixed-joint-lumped URDF (the viewer's
                # "merge fixed" toggle uses this; mesh refs are unchanged so the
                # same meshes still resolve)
                merged = query.get("merged") == ["1"]

                def _maybe_merge(txt):
                    if not merged:
                        return txt
                    from sw2robot.exporter.merge import merge_fixed_links_text
                    return merge_fixed_links_text(txt)
                # URDF-input mode: the URDF URL is the overlay-applied URDF,
                # computed on the fly so the on-disk file stays the pristine base
                # (decode first -- urdf_rel is decoded, but the URL may carry %20)
                if (is_urdf and _um["state"] is not None
                        and not _cad_mode(cls.pkg_dir)):
                    from . import core
                    served = _rewrite_package_urls(
                        core.build_urdf(_um["state"], sanitize=False),
                        cls.urdf_rel, cls.pkg_dir)
                    return self._send_bytes(_maybe_merge(served).encode("utf-8"),
                                            "application/xml")
                full = self._resolve(cls.pkg_dir, rel)
                if full is None or not os.path.isfile(full):
                    return self.send_error(404)
                if is_urdf and merged:
                    with open(full, encoding="utf-8") as f:
                        return self._send_bytes(
                            _maybe_merge(f.read()).encode("utf-8"),
                            "application/xml")
                if full.lower().endswith(".3dxml") \
                        and query.get("glb") == ["1"]:
                    return self._send_file(_convert_3dxml_to_glb(full))
                return self._send_file(full)
            # other static assets from sw2robot/editor/web/
            full = self._resolve(WEB_DIR, path)
            if full and os.path.isfile(full):
                return self._send_file(full)
            return self.send_error(404)
        except BrokenPipeError:
            pass
        except Exception as e:           # surface failures to the client
            print(f"[sw2robot.web] {self.path}: {e!r}")
            try:
                self.send_error(500, str(e))
            except Exception:
                pass

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/api/collision":
                _t0 = time.time()
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                angles = body.get("angles") or {}
                _t1 = time.time()
                with _coll_lock:
                    ctx = _coll["ctx"]
                    if ctx is None:
                        return self._send_json({"ready": False})
                    for name, j in ctx["joints"].items():
                        try:
                            j.joint_angle(float(angles.get(name, 0.0)))
                        except Exception:
                            pass
                    pairs = sorted(tuple(sorted(p))
                                   for p in ctx["sc"].new_pairs())
                _t2 = time.time()
                links = sorted({l for p in pairs for l in p})
                if os.environ.get("SW2ROBOT_TIME_COLLISION"):
                    print(f"[sw2robot.web] /api/collision read={_t1-_t0:.3f}s "
                          f"compute={_t2-_t1:.3f}s", flush=True)
                return self._send_json({"ready": True, "pairs": pairs,
                                        "links": links})
            if parsed.path == "/api/set_limits":
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                limits = body.get("limits") or []
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_set_limits, limits)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                linv = _link_names_inverse(txt)   # display -> component
                applied, missed = [], []
                changed_yaml = False
                for lm in limits:
                    child = lm.get("child")
                    lo = float(lm.get("lower", 0.0))
                    hi = float(lm.get("upper", 0.0))
                    cont = bool(lm.get("continuous"))
                    # persist to joints.yaml (survives re-extract / undo) ...
                    txt, ky = _set_joint_limit(txt, linv.get(child, child),
                                               lo, hi, cont)
                    changed_yaml = changed_yaml or bool(ky)
                    # ... but apply it INSTANTLY by editing the served URDF in
                    # place, skipping the inertia-recomputing full build()
                    ok = _set_limit_in_urdf(cls.pkg_dir, cls.urdf_rel,
                                            child, lo, hi, cont)
                    (applied if ok else missed).append(child)
                if changed_yaml:
                    _snapshot(cls.pkg_dir, yml, f"limits x{len(applied)}")
                    with open(yml, "w", encoding="utf-8") as f:
                        f.write(txt)
                print(f"[sw2robot.web] set_limits: {len(applied)} applied, "
                      f"{len(missed)} not matched")
                return self._send_json({"applied": applied, "missed": missed})
            if parsed.path == "/api/set_axis":
                # reverse one or more joints' + direction directly in the served
                # URDF (axis negated, limits swapped, mimic negated; frame kept).
                # joints: [<urdf joint name>, ...].  Self-inverse; no rebuild, so
                # it works on any loaded URDF and the export reads it as-is.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                names = body.get("joints") or []
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_set_axis, names)
                applied, missed = [], []
                for jn in names:
                    try:
                        k = _flip_axis_in_urdf(cls.pkg_dir, cls.urdf_rel, jn)
                    except Exception as e:
                        return self._send_json(
                            {"error": f"flip failed: {e}"}, 500)
                    (applied if k else missed).append(jn)
                print(f"[sw2robot.web] set_axis: {len(applied)} flipped, "
                      f"{len(missed)} not matched")
                return self._send_json({"applied": applied, "missed": missed})
            if parsed.path == "/api/set_types":
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                changes = body.get("changes") or []
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_set_types, changes)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": f"{name}.joints.yaml not found -- this "
                                  f"package predates config templates; "
                                  f"re-extract it once"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                linv = _link_names_inverse(txt)   # display -> component
                applied, missed = [], []
                for ch in changes:
                    c, t = ch.get("child"), ch.get("type")
                    c = linv.get(c, c)
                    if t not in ("fixed", "revolute", "continuous",
                                 "prismatic") or not c:
                        missed.append(ch)
                        continue
                    # match by CHILD only: in a spanning tree each link is
                    # a child exactly once, and the URDF renames the root
                    # link to base_link while the yaml keeps the component
                    # name -- so the parent is NOT a reliable key
                    pat = re.compile(
                        r"(- parent:\s*\S+\s*\n\s*child:\s*" + re.escape(c)
                        + r"\s*\n\s*type:\s*)\S+")
                    txt, k = pat.subn(r"\g<1>" + t, txt)
                    (applied if k else missed).append(ch)
                if applied:
                    _snapshot(cls.pkg_dir, yml,
                              f"joint type x{len(applied)}")
                    with open(yml, "w", encoding="utf-8") as f:
                        f.write(txt)
                    from sw2robot.exporter.export import build
                    try:
                        build(cls.pkg_dir, config_path=yml)
                    except Exception as e:
                        return self._send_json(
                            {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] set_types: {len(applied)} applied, "
                      f"{len(missed)} not matched")
                return self._send_json(
                    {"applied": [c.get("name") for c in applied],
                     "missed": [c.get("name") for c in missed]})
            if parsed.path == "/api/set_mimic":
                # changes: [{child, master, multiplier, offset}] to link a
                # follower joint to a driver, or {child, clear: true} to unlink.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                changes = body.get("changes") or []
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_set_mimic, changes)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": f"{name}.joints.yaml not found -- re-extract "
                                  f"this package once"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                linv = _link_names_inverse(txt)   # display -> component
                applied, missed = [], []
                changed_yaml = False
                for ch in changes:
                    child = ch.get("child")
                    master = ch.get("master")
                    clear = bool(ch.get("clear"))
                    if not child or (not clear and not master):
                        missed.append(ch)
                        continue
                    mult = float(ch.get("multiplier", 1.0))
                    off = float(ch.get("offset", 0.0))
                    # persist to joints.yaml ...
                    txt, ky = _set_mimic_yaml(txt, linv.get(child, child),
                                              master, mult, off, clear)
                    changed_yaml = changed_yaml or bool(ky)
                    # ... and apply instantly in the served URDF (no rebuild)
                    ok = _set_mimic_in_urdf(cls.pkg_dir, cls.urdf_rel, child,
                                            master, mult, off, clear)
                    (applied if ok else missed).append(ch)
                if changed_yaml:
                    _snapshot(cls.pkg_dir, yml, f"mimic x{len(applied)}")
                    with open(yml, "w", encoding="utf-8") as f:
                        f.write(txt)
                print(f"[sw2robot.web] set_mimic: {len(applied)} applied, "
                      f"{len(missed)} not matched")
                return self._send_json(
                    {"applied": [c.get("child") for c in applied],
                     "missed": [c.get("child") for c in missed]})
            if parsed.path == "/api/redetect_couplings":
                # Re-run the AUTO closed-loop (four-bar) detection on the cached
                # CAD graph and MERGE the resulting <mimic> couplings into the
                # existing joints.yaml -- keeping the user's types/renames/base/
                # limits.  A re-extract reuses the config (directed path) and so
                # never re-detects; this is the non-destructive way to pick the
                # couplings up on a package built before the feature existed.
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._send_json(
                        {"error": "re-detect needs the CAD graph.json "
                                  "(re-extract this package once)"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                gj = os.path.join(cls.pkg_dir, "graph.json")
                if not (os.path.exists(yml) and os.path.exists(gj)):
                    return self._send_json(
                        {"error": "joints.yaml / graph.json missing -- "
                                  "re-extract this package once"}, 400)
                import yaml as _yaml

                from sw2robot.exporter import model as _M
                from sw2robot.exporter.state import GraphState
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                cfg = _yaml.safe_load(txt) or {}
                # detect on the AUTO tree, but rooted at the user's chosen base
                # so the joint names line up with their config
                graph = GraphState.load(gj)
                comps, adjacency, _gnd = _M.from_graph(
                    graph, exclude=list(cfg.get("exclude") or []),
                    expand=cfg.get("expand"), no_expand=cfg.get("no_expand"))
                base = _M.choose_base(comps, _gnd, cfg.get("base"), adjacency)
                joints = _M.build_tree(comps, adjacency, base)
                by_name = {j.name: j for j in joints}
                applied, missed, drivers = [], [], {}
                for j in joints:
                    m = j.mimic
                    if not (m and m.get("poly")):
                        continue
                    txt, ky = _set_mimic_yaml(
                        txt, j.child, m["joint"], m.get("multiplier", 1.0),
                        m.get("offset", 0.0), False, poly=m.get("poly"))
                    # the follower's real swing (the loop constrains it)
                    if j.lower is not None and j.upper is not None:
                        txt, _ = _set_joint_limit(txt, j.child, j.lower,
                                                  j.upper, False)
                    drivers[m["joint"]] = True
                    (applied if ky else missed).append(j.child)
                # the driver's travel is bounded by the four-bar toggle, not the
                # default +-pi -- push that limit too
                for dn in drivers:
                    dj = by_name.get(dn)
                    if dj is not None and dj.lower is not None \
                            and dj.upper is not None:
                        txt, _ = _set_joint_limit(txt, dj.child, dj.lower,
                                                  dj.upper, False)
                if not applied:
                    return self._send_json({"detected": 0, "applied": [],
                                            "missed": missed})
                _snapshot(cls.pkg_dir, yml, f"re-detect mimic x{len(applied)}")
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] redetect_couplings: {len(applied)} "
                      f"applied, {len(missed)} not matched")
                return self._send_json(
                    {"detected": len(applied) + len(missed),
                     "applied": applied, "missed": missed})
            if parsed.path == "/api/set_base":
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                new_root = body.get("link")
                if not cls.pkg_dir or not new_root:
                    return self._send_json({"error": "no package/link"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                # a renamed link comes back as its display name -> component
                new_root = _link_names_inverse(txt).get(new_root, new_root)
                # a directed joints list must stay a tree rooted at `base:`
                # or _config_parent_map drops edges.  FIRST compute the
                # path new_root -> old_root from the original text, THEN
                # flip those entries (flipping while walking ping-pongs on
                # the entry just flipped)
                entry = re.compile(
                    r"- parent:(\s*)(\S+)(\s*\n\s*child:\s*)(\S+)")
                up = {m.group(4): m.group(2) for m in entry.finditer(txt)}
                path, cur, seen = [], new_root, set()
                while cur in up and cur not in seen:
                    seen.add(cur)
                    path.append((up[cur], cur))      # original (parent,child)
                    cur = up[cur]
                flips = 0
                for parent, child in path:
                    pat = re.compile(
                        r"- parent:(\s*)" + re.escape(parent)
                        + r"(\s*\n\s*child:\s*)" + re.escape(child)
                        + r"(?=\s)")
                    txt, k = pat.subn(
                        lambda m, p=parent, c=child:
                        f"- parent:{m.group(1)}{c}{m.group(2)}{p}",
                        txt, count=1)
                    flips += k
                if flips != len(path):
                    return self._send_json(
                        {"error": f"re-root inconsistent: {flips}/"
                                  f"{len(path)} edges flipped"}, 500)
                if re.search(r"(?m)^base:", txt):
                    txt = re.sub(r"(?m)^base:.*$", f"base: {new_root}", txt,
                                 count=1)
                elif re.search(r"(?m)^#\s*base:", txt):
                    txt = re.sub(r"(?m)^#\s*base:.*$", f"base: {new_root}",
                                 txt, count=1)
                else:
                    txt = f"base: {new_root}\n" + txt
                # root_rpy/root_xyz were authored relative to the OLD base;
                # carrying them onto a new base puts the origin in a
                # nonsense place -- re-rooting starts from the new
                # component's own frame (undo restores the whole yaml)
                txt = re.sub(r"(?m)^root_(rpy|xyz|z_offset):.*$\n?", "", txt)
                _snapshot(cls.pkg_dir, yml, f"re-root to {new_root}")
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] set_base: {new_root} "
                      f"({flips} edges re-rooted)")
                return self._send_json({"base": new_root, "flipped": flips})
            if parsed.path == "/api/set_root_pose":
                # click-to-align: origin at `xyz`, +Z along `zdir` (both in
                # the CURRENT root frame).  Compose with the existing
                # root_rpy/root_xyz/root_z_offset and write back.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                import numpy as np

                from sw2robot.exporter.geometry import (
                    matrix_from_rpy,
                    matrix_to_xyz_rpy,
                )
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                rpy0, xyz0, z0 = _read_root_pose(txt)
                M_old = matrix_from_rpy(rpy0)
                M_old[:3, 3] = (M_old[:3, :3]
                                @ (np.asarray(xyz0, float)
                                   + np.asarray([0, 0, z0], float)))
                p = np.asarray(body.get("xyz") or [0, 0, 0], float)
                zdir = body.get("zdir")
                D = np.eye(4)
                if body.get("absolute"):
                    # numeric fields: replace instead of composing.  yaml
                    # semantics: rotate first, then translate in the
                    # rotated frame (matches _finalize_tree)
                    M_old = np.eye(4)
                    D = matrix_from_rpy(body["absolute"].get("rpy")
                                        or [0, 0, 0])
                    D[:3, 3] = D[:3, :3] @ np.asarray(
                        body["absolute"].get("xyz") or [0, 0, 0], float)
                    p = D[:3, 3].copy()     # the late D[:3,3]=p must not
                    zdir = None             # clobber the absolute shift
                elif body.get("rpy") is not None:
                    # +-90 deg style rotation delta about the CURRENT axes
                    D = matrix_from_rpy(body["rpy"])
                if zdir is not None:
                    # MINIMAL rotation taking +Z onto zdir (Rodrigues), so
                    # an already-aligned normal changes nothing -- a basis
                    # built from an arbitrary 'up' would add a surprise yaw
                    # (shared with /api/add_port via _rot_z_to)
                    D[:3, :3] = _rot_z_to(zdir)
                D[:3, 3] = p
                M_new = M_old @ D
                R_new = M_new[:3, :3]
                xyz_new = R_new.T @ M_new[:3, 3]
                _, rpy_new = matrix_to_xyz_rpy(M_new)
                fmt = lambda v: "[" + ", ".join(f"{x:.6g}" for x in v) + "]"
                for key, val in (("root_rpy", fmt(rpy_new)),
                                 ("root_xyz", fmt(xyz_new))):
                    if re.search(r"(?m)^" + key + r":", txt):
                        txt = re.sub(r"(?m)^" + key + r":.*$",
                                     f"{key}: {val}", txt, count=1)
                    else:
                        txt = f"{key}: {val}\n" + txt
                txt = re.sub(r"(?m)^root_z_offset:.*$\n?", "", txt)
                _snapshot(cls.pkg_dir, yml, "root frame change")
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] set_root_pose: xyz={p.tolist()} "
                      f"zdir={zdir}")
                return self._send_json({"rpy": list(rpy_new),
                                        "xyz": list(xyz_new)})
            if parsed.path == "/api/client_log":
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                out = os.path.join(_DATA_DIR, "_client_report.json")
                with open(out, "w", encoding="utf-8") as f:
                    json.dump(body, f, ensure_ascii=False, indent=1)
                print(f"[sw2robot.web] CLIENT REPORT -> {out}")
                print(f"  controlsEnabled={body.get('controlsEnabled')} "
                      f"dragging={body.get('dragManipulating')} "
                      f"hover={body.get('dragHovered')} "
                      f"cover={body.get('coverPresent')} "
                      f"eaters={body.get('eventEaters')}")
                return self._send_json({"saved": out})
            if parsed.path == "/api/set_exclude":
                # exclude COMPONENT(s) from the built URDF entirely (yaml
                # `exclude:` list), or restore.  {name} or {names: [...]} (a
                # whole subtree, deleted in one rebuild); {clear: true} restores.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                names = body.get("names")
                if names is None and body.get("name") is not None:
                    names = [body.get("name")]
                names = [x for x in (names or []) if x]
                if not cls.pkg_dir or (not names and not body.get("clear")):
                    return self._send_json({"error": "no package/name"}, 400)
                pkg = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, pkg + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                linv = _link_names_inverse(txt)    # renamed link -> component
                names = [linv.get(x, x) for x in names]
                _snapshot(cls.pkg_dir, yml,
                          "restore excluded" if body.get("clear")
                          else (f"exclude {names[0][:30]}"
                                + (f" +{len(names) - 1}" if len(names) > 1
                                   else "")))
                m = re.search(r"(?m)^exclude:\n((?:- .*\n)*)", txt)
                block = m.group(1) if m else ""
                if body.get("clear"):
                    block = ""
                else:
                    for nm in names:
                        block = re.sub(r"(?m)^- " + re.escape(nm)
                                       + r"\s*$\n?", "", block)
                        if body.get("on", True):
                            block += f"- {nm}\n"
                new = f"exclude:\n{block}" if block else ""
                txt = (txt[:m.start()] + new + txt[m.end():]) if m \
                    else (new + txt)
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] set_exclude: {names} "
                      f"on={body.get('on', True)} clear={body.get('clear')}")
                return self._send_json({"excluded": [
                    ln[2:].strip() for ln in block.splitlines()]})
            if parsed.path == "/api/set_material":
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                link = body.get("link")
                density = body.get("density")     # None = remove override
                if not cls.pkg_dir or not link:
                    return self._send_json({"error": "no package/link"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                link = _link_names_inverse(txt).get(link, link)
                _snapshot(cls.pkg_dir, yml, f"material of {link[:30]}")
                m = re.search(r"(?m)^densities:\n((?:[ \t]+\S+:.*\n)*)", txt)
                block = m.group(1) if m else ""
                pat = re.compile(r"(?m)^[ \t]+" + re.escape(link)
                                 + r":.*\n?")
                block = pat.sub("", block)
                if density:
                    block += f"  {link}: {float(density):g}\n"
                if m:
                    txt = txt[:m.start()] \
                        + (f"densities:\n{block}" if block else "") \
                        + txt[m.end():]
                elif block:
                    txt = f"densities:\n{block}" + txt
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] set_material: {link} -> {density}")
                return self._send_json({"link": link, "density": density})
            if parsed.path == "/api/set_color":
                # per-link visual colour override, stored as a `colors:` block in
                # joints.yaml ({component name -> '#RRGGBB'}).  The viewer paints
                # it client-side, so NO rebuild is needed here; the exporter bakes
                # it into the <visual> .dae/.glb at export time.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                link = body.get("link")
                color = body.get("color")         # None / '' removes the override
                if not cls.pkg_dir or not link:
                    return self._send_json({"error": "no package/link"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_set_color, link, color)
                norm = None
                if color:
                    h = str(color).strip().lstrip("#").lower()
                    if not re.fullmatch(r"[0-9a-f]{6}", h):
                        return self._send_json(
                            {"error": f"invalid color {color!r}: want #RRGGBB"},
                            400)
                    norm = "#" + h
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                comp = _link_names_inverse(txt).get(link, link)
                _snapshot(cls.pkg_dir, yml, f"color of {comp[:30]}")
                m = re.search(r"(?m)^colors:\n((?:[ \t]+\S+:.*\n)*)", txt)
                block = m.group(1) if m else ""
                pat = re.compile(r"(?m)^[ \t]+" + re.escape(comp) + r":.*\n?")
                block = pat.sub("", block)
                if norm:
                    # quote: a bare '#...' is a YAML comment
                    block += f"  {comp}: '{norm}'\n"
                if m:
                    txt = txt[:m.start()] \
                        + (f"colors:\n{block}" if block else "") \
                        + txt[m.end():]
                elif block:
                    txt = f"colors:\n{block}" + txt
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                print(f"[sw2robot.web] set_color: {comp} -> {norm}")
                return self._send_json({"link": comp, "color": norm})
            if parsed.path == "/api/set_inertial":
                # per-link inertial override (mass / com / inertia).  URDF-input
                # mode only: a CAD package derives inertia from the mesh + density
                # at build time, so it has no overlay to write to here.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if _cad_mode(cls.pkg_dir):
                    return self._send_json(
                        {"error": "inertial editing is only available in "
                                  "URDF-input mode"}, 400)
                return self._um_reply(_um_set_inertial, body)
            if parsed.path == "/api/rename":
                # rename a link or joint to a user-chosen name.  Stored as a
                # component->display overlay in joints.yaml (link_names /
                # joint_names; the root uses root_link_name), applied at build.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                kind = body.get("kind")
                old = (body.get("old") or "").strip()
                new = (body.get("new") or "").strip()
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                # an empty `new` means "reset this name to its default"
                reset = (new == "")
                if kind not in ("link", "joint") or not old:
                    return self._send_json(
                        {"error": "kind ('link'|'joint') and old required"}, 400)
                if not reset and not _VALID_NAME.match(new):
                    return self._send_json(
                        {"error": f"invalid name '{new}' -- letters / digits / "
                                  f"underscore, not starting with a digit"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_rename, kind, old, new)
                pkg = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, pkg + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found -- re-extract once"},
                        400)
                tag = "link" if kind == "link" else "joint"
                cur = _urdf_names(cls.pkg_dir, cls.urdf_rel, tag)
                if old not in cur:
                    return self._send_json({"error": f"no {kind} '{old}'"}, 400)
                if not reset and new in cur and new != old:
                    return self._send_json(
                        {"error": f"'{new}' already names another {kind}"}, 400)
                if old == new:
                    return self._send_json(
                        {"ok": True, "kind": kind, "old": old, "new": new})
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                is_root = (kind == "link"
                           and old == _urdf_root_link(cls.pkg_dir, cls.urdf_rel))
                if reset:
                    _snapshot(cls.pkg_dir, yml, f"reset {kind} name {old}")
                    if is_root:
                        txt = _clear_root_link_name(txt)
                    else:
                        mapkey = "link_names" if kind == "link" else "joint_names"
                        key = _names_inverse(txt, mapkey).get(old, old)
                        txt = _remove_yaml_map_entry(txt, mapkey, key)
                else:
                    _snapshot(cls.pkg_dir, yml, f"rename {kind} {old}->{new}")
                    if is_root:
                        txt = _set_root_link_name(txt, new)   # root display name
                    else:
                        mapkey = "link_names" if kind == "link" else "joint_names"
                        key = _names_inverse(txt, mapkey).get(old, old)
                        txt = _upsert_yaml_map(txt, mapkey, key, new)
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                try:
                    if reset:
                        # reset needs the DEFAULT name back, which only the full
                        # build knows -- rare, so a rebuild is fine here
                        from sw2robot.exporter.export import build
                        build(cls.pkg_dir, config_path=yml)
                    else:
                        # the common path: rewrite the name in the URDF in place
                        # (instant) instead of a full inertia-recomputing rebuild
                        _rename_in_urdf(cls.pkg_dir, cls.urdf_rel, kind, old, new)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] "
                      f"{'reset' if reset else 'rename'} {kind}: {old}"
                      f"{'' if reset else ' -> ' + new}")
                return self._send_json(
                    {"ok": True, "kind": kind, "old": old, "new": new})
            if parsed.path == "/api/add_port":
                # click-to-add a coordinate-only link (robot-compiler dummy_link
                # port): origin at `xyz`, +Z along `zdir`, both in the clicked
                # LINK's frame.  Appended to `ports:` and auto-named by build
                # (dummy_link, dummy_link2 ...).  Mirrors /api/set_root_pose but
                # targets a link instead of the root.
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                link = body.get("link")
                if not cls.pkg_dir or not link:
                    return self._send_json({"error": "no package/link"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                # the block-list helpers only understand a block-style `ports:`
                # (what the exporter and this endpoint emit); refuse to edit a
                # hand-written flow-style list rather than append a 2nd key
                if re.search(r"(?m)^ports:[ \t]*\S", txt):
                    return self._send_json(
                        {"error": "ports: is in inline/flow style; reformat it "
                                  "to a block list before adding ports"}, 400)
                # the rest of joints.yaml is keyed by the COMPONENT link name;
                # reverse-map the on-screen (display) name so resolve_ports'
                # _match_component finds the tip link
                comp = _link_names_inverse(txt).get(link, link)
                import math

                def _vec3(v, default):
                    v = default if v is None else v
                    try:
                        v = [float(x) for x in v]
                    except (TypeError, ValueError):
                        return None
                    return v if len(v) == 3 \
                        and all(math.isfinite(x) for x in v) else None

                xyz = _vec3(body.get("xyz"), [0, 0, 0])
                if xyz is None:
                    return self._send_json(
                        {"error": "xyz must be 3 finite numbers"}, 400)
                # full orientation: prefer an explicit rpy (the gizmo sends one),
                # else derive it from the +Z direction (the click-on-face flow)
                if body.get("rpy") is not None:
                    rpy = _vec3(body.get("rpy"), None)
                    if rpy is None:
                        return self._send_json(
                            {"error": "rpy must be 3 finite numbers"}, 400)
                else:
                    zdir = _vec3(body.get("zdir"), [0, 0, 1])
                    if zdir is None:
                        return self._send_json(
                            {"error": "zdir must be 3 finite numbers"}, 400)
                    rpy = _zdir_to_rpy(zdir)
                # optional user-chosen names for the dummy_link + its fixed joint
                pname = (body.get("name") or "").strip()
                jname = (body.get("joint_name") or "").strip()
                for label, nm in (("name", pname), ("joint_name", jname)):
                    if nm and not _VALID_NAME.match(nm):
                        return self._send_json(
                            {"error": f"invalid {label} '{nm}' -- letters / "
                                      f"digits / underscore, not starting with a "
                                      f"digit"}, 400)
                fmt = lambda v: "[" + ", ".join(f"{x:.6g}" for x in v) + "]"
                _snapshot(cls.pkg_dir, yml, f"add port on {comp[:30]}")
                item = [f"parent: {_yaml_scalar(comp)}",
                        f"xyz: {fmt(xyz)}", f"rpy: {fmt(rpy)}"]
                if pname:
                    item.append(f"name: {_yaml_scalar(pname)}")
                if jname:
                    item.append(f"joint_name: {_yaml_scalar(jname)}")
                txt = _append_yaml_list_item(txt, "ports", item)
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] add_port: parent={comp} "
                      f"name={pname or '(auto)'} xyz={xyz} rpy={rpy}")
                return self._send_json(
                    {"ok": True, "parent": comp, "name": pname,
                     "xyz": xyz, "rpy": rpy})
            if parsed.path == "/api/remove_port":
                # drop a previously-added dummy_link port by its emitted name
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                pname = body.get("name")
                if not cls.pkg_dir or not pname:
                    return self._send_json({"error": "no package/name"}, 400)
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                import yaml as _yaml
                try:
                    ports = (_yaml.safe_load(txt) or {}).get("ports") or []
                except Exception:
                    ports = []
                idx = None
                for i, p in enumerate(ports):
                    pn = (p or {}).get("name") or (
                        "dummy_link" if i == 0 else f"dummy_link{i + 1}")
                    if pn == pname:
                        idx = i
                        break
                if idx is None:
                    return self._send_json({"error": f"no port '{pname}'"}, 400)
                _snapshot(cls.pkg_dir, yml, f"remove port {pname}")
                txt = _remove_yaml_list_item(txt, "ports", idx)
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] remove_port: {pname}")
                return self._send_json({"ok": True, "name": pname})
            if parsed.path == "/api/reset_names":
                # drop ALL rename overlays -> every link/joint back to default
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                if not _cad_mode(cls.pkg_dir):
                    return self._um_reply(_um_reset_names)
                pkg = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, pkg + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json({"error": "joints.yaml not found"},
                                           400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                # only the rename overlays -- root_link_name is also a module
                # config (the base_link input port), so a renamed root is reset
                # individually, not wiped here
                if not re.search(r"(?m)^(link_names|joint_names):", txt):
                    return self._send_json({"ok": True, "reset": 0})
                _snapshot(cls.pkg_dir, yml, "reset all names")
                txt = _remove_yaml_block(txt, "link_names")
                txt = _remove_yaml_block(txt, "joint_names")
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(txt)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print("[sw2robot.web] reset all names to default")
                return self._send_json({"ok": True})
            if parsed.path in ("/api/undo", "/api/redo"):
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                src = "undo" if parsed.path.endswith("undo") else "redo"
                dst = "redo" if src == "undo" else "undo"
                if not _cad_mode(cls.pkg_dir):       # URDF-input mode: overlay stack
                    if not _um[src]:
                        return self._send_json({"error": f"nothing to {src}"}, 400)
                    _um[dst].append(_um_overlay_json(_um["state"]))
                    _um_restore(_um[src].pop())
                    return self._send_json(
                        {"done": src, "undo": len(_um["undo"]),
                         "redo": len(_um["redo"])})
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                h = _hist(cls.pkg_dir)
                if not h[src]:
                    return self._send_json({"error": f"nothing to {src}"},
                                           400)
                label, snap = h[src].pop()
                try:
                    with open(yml, encoding="utf-8") as f:
                        h[dst].append((label, f.read()))
                except OSError:
                    pass
                with open(yml, "w", encoding="utf-8") as f:
                    f.write(snap)
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
                except Exception as e:
                    return self._send_json(
                        {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] {src}: {label}")
                return self._send_json(
                    {"done": src, "label": label,
                     "undo": len(h["undo"]), "redo": len(h["redo"])})
            if parsed.path == "/api/update/apply":
                # download the latest Release asset, swap the running binary and
                # relaunch (frozen build only).  Returns immediately; the UI
                # polls /api/update/status for progress.
                from . import update
                n = int(self.headers.get("Content-Length", 0))
                if n:
                    self.rfile.read(n)          # drain body so keep-alive survives
                return self._send_json(update.start_update())
            return self.send_error(404)
        except BrokenPipeError:
            pass
        except Exception as e:
            print(f"[sw2robot.web] {self.path}: {e!r}")
            try:
                self.send_error(500, str(e))
            except Exception:
                pass

    def log_message(self, fmt, *args):   # quiet; errors print above
        pass


def _bind_free_port(handler, port, tries=20, wait_first=0.0):
    """Bind a ThreadingTCPServer on the first free port >= ``port``.

    A second editor instance (or a leftover one) would otherwise collide on the
    default port and the launch crashes with ``WinError 10048`` (address in
    use); walking forward to the next free port lets every instance just come
    up. Returns ``(httpd, bound_port)``.

    ``wait_first`` (seconds) makes a self-update relaunch RECLAIM the exact same
    port: the instance we are replacing is still mid-exit, so retry ``port`` for
    a moment before walking forward -- that way the browser tab that reloads
    after the update reconnects at the same URL instead of finding the new
    server on a drifted port."""
    last = None
    if wait_first > 0:
        deadline = time.time() + wait_first
        while time.time() < deadline:
            try:
                return socketserver.ThreadingTCPServer(("", port), handler), port
            except OSError as e:
                last = e
                time.sleep(0.3)
    for p in range(port, port + tries):
        try:
            return socketserver.ThreadingTCPServer(("", p), handler), p
        except OSError as e:
            last = e
    raise OSError(f"no free port in {port}..{port + tries - 1}: {last}")


def serve(package_dir=None, root_dir=None, port=8090, open_browser=True,
          reclaim_port=False):
    global BOUND_PORT
    import atexit
    import signal
    # reap any private SolidWorks instance a PREVIOUS run spawned and leaked
    # (hard-killed before atexit could run) -- by recorded PID, never the user's
    _reap_spawned_sw()
    # and reap the <exe>.old image a previous self-update left behind (it could
    # not delete itself while it was the running process)
    from . import update
    update.reap_leftovers()
    atexit.register(_shutdown_sw)     # close the warm session on exit
    # also catch Ctrl+C / termination so the spawned instance is torn down
    # rather than orphaned (atexit does not run on a signal default-kill)
    for _sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
        if _sig is not None:
            try:
                signal.signal(_sig, lambda *_a: (_shutdown_sw(), os._exit(0)))
            except (ValueError, OSError):
                pass            # not in main thread / unsupported -- atexit covers it
    threading.Thread(target=_keepalive_loop, daemon=True).start()
    _Handler.root_dir = os.path.abspath(root_dir or _default_root())
    if package_dir:
        pkg, rel = _resolve_package(package_dir)
        _Handler.pkg_dir, _Handler.urdf_rel = pkg, rel
        _Handler.robot_name = os.path.splitext(os.path.basename(rel))[0]
        if not _cad_mode(pkg):           # plain URDF -> overlay editing mode
            _um_load(pkg, rel)
        print(f"[sw2robot.web] serving '{_Handler.robot_name}' from {pkg}"
              f"{'' if _cad_mode(pkg) else ' [urdf-input mode]'}")
    else:
        print(f"[sw2robot.web] no package yet -- pick one in the browser "
              f"(root: {_Handler.root_dir})")
    # a self-update relaunch reclaims the exact port the old instance had (it is
    # mid-exit), so wait briefly for it instead of immediately drifting forward
    httpd, bound = _bind_free_port(_Handler, port,
                                   wait_first=8.0 if reclaim_port else 0.0)
    if bound != port:
        print(f"[sw2robot.web] port {port} busy -> using {bound}")
    port = bound
    BOUND_PORT = bound
    httpd.daemon_threads = True
    url = f"http://localhost:{port}"
    print(f"[sw2robot.web] open {url}")
    # the socket is already bound+listening here, so the page can load the
    # moment the default browser reaches it
    if open_browser:
        import webbrowser
        threading.Thread(target=webbrowser.open, args=(url,),
                         daemon=True).start()
    httpd.serve_forever()


# --- run-from-%LOCALAPPDATA% relocation (avoid self-updating inside OneDrive) -
# A self-updating single-file exe must never live in a cloud-synced folder: the
# sync client (OneDrive/Dropbox) watches the file, so the in-place swap fights
# the uploader (locks, conflict copies, a stale cloud copy rehydrating the OLD
# version).  Surface/Win11 ships with the Desktop redirected INTO OneDrive, so
# users routinely drop the exe there.  Fix: on launch, if we are a frozen exe
# sitting under a synced root, copy ourselves once to a stable, NON-synced
# install dir (%LOCALAPPDATA%\sw2robot\bin) and relaunch from there.  Every later
# self-update then rewrites that LocalAppData copy, which no sync client touches.

def _sync_roots():
    """Cloud-sync root dirs (OneDrive/Dropbox/Google Drive) a self-updating exe
    must not live in.  Best-effort; empty off Windows."""
    if sys.platform != "win32":
        return []
    roots, home = [], os.path.expanduser("~")
    for var in ("OneDrive", "OneDriveConsumer", "OneDriveCommercial"):
        v = os.environ.get(var)
        if v:
            roots.append(v)
    try:
        for n in os.listdir(home):
            low = n.lower()
            if low.startswith("onedrive") or low in ("dropbox", "google drive",
                                                      "my drive"):
                roots.append(os.path.join(home, n))
    except OSError:
        pass
    out, seen = [], set()
    for r in roots:
        nr = os.path.normcase(os.path.normpath(r))
        if nr not in seen:
            seen.add(nr)
            out.append(nr)
    return out


def _under_any(path, roots):
    p = os.path.normcase(os.path.normpath(os.path.realpath(path)))
    for r in roots:
        try:
            if os.path.commonpath([p, r]) == r:
                return True
        except ValueError:
            pass                              # different drive -> not under it
    return False


def _install_dir():
    base = os.environ.get("LOCALAPPDATA") or os.path.join(
        os.path.expanduser("~"), "AppData", "Local")
    return os.path.join(base, "sw2robot", "bin")


def _make_start_menu_shortcut(target):
    """Best-effort Start-menu .lnk to the installed exe, so there's a fast launch
    path that doesn't go through the synced stub.  win32com is bundled on the
    Windows build (it's the SolidWorks COM glue); absent -> just skip."""
    try:
        import win32com.client
        programs = os.path.join(os.environ["APPDATA"], "Microsoft", "Windows",
                                "Start Menu", "Programs")
        os.makedirs(programs, exist_ok=True)
        sc = win32com.client.Dispatch("WScript.Shell").CreateShortcut(
            os.path.join(programs, "sw2robot-web.lnk"))
        sc.TargetPath = target
        sc.WorkingDirectory = os.path.dirname(target)
        sc.Description = "sw2robot web editor"
        sc.save()
    except Exception as e:
        print(f"[sw2robot.web] start-menu shortcut skipped: {e!r}")


def _relocate_to_install_dir():
    """If we're a frozen exe running from a cloud-synced folder, install a copy
    to %LOCALAPPDATA%\\sw2robot\\bin and relaunch it.  Returns True if it
    relaunched (caller must exit), False to keep running in place.  No-op in a
    source checkout, off Windows, when already the installed copy, or when run
    from a normal local folder (stays portable)."""
    if not getattr(sys, "frozen", False) or sys.platform != "win32":
        return False
    if "--no-relocate" in sys.argv:           # we ARE the relaunched install
        return False
    try:
        me = os.path.normcase(os.path.realpath(sys.executable))
        canonical = os.path.join(_install_dir(), "sw2robot-web.exe")
        if me == os.path.normcase(os.path.realpath(canonical)):
            return False                      # already running from the install
        if not _under_any(sys.executable, _sync_roots()):
            return False                      # local/portable run -> leave as-is
        from . import update  # reuse the detached launcher + ver parse
        bindir = os.path.dirname(canonical)
        os.makedirs(bindir, exist_ok=True)
        verfile = os.path.join(bindir, "installed-version.txt")
        old = canonical + ".old"
        try:
            if os.path.exists(old):
                os.remove(old)                # reap a prior upgrade's leftover
        except OSError:
            pass
        try:
            with open(verfile, encoding="utf-8") as f:
                installed = f.read().strip()
        except OSError:
            installed = ""
        from .. import __version__
        fresh = (not os.path.exists(canonical)
                 or update._parse_version(__version__)
                 > update._parse_version(installed))
        if fresh:
            import shutil
            tmp = canonical + ".new"
            shutil.copyfile(sys.executable, tmp)
            try:
                os.replace(tmp, canonical)
            except OSError:                   # an installed instance is running
                os.replace(canonical, old)    # rename it aside (allowed)
                os.replace(tmp, canonical)
            try:
                with open(verfile, "w", encoding="utf-8") as f:
                    f.write(__version__)
            except OSError:
                pass
            _make_start_menu_shortcut(canonical)
            print(f"[sw2robot.web] installed v{__version__} -> {canonical}")
        # relaunch the install copy with the same args, flagged so it won't loop
        update._popen_detached([canonical, *sys.argv[1:], "--no-relocate"])
        print(f"[sw2robot.web] relaunching from {canonical} "
              f"(was running from a synced folder)")
        return True
    except Exception as e:
        print(f"[sw2robot.web] relocate skipped: {e!r}")
        return False


def main():
    # self-dispatch: the auto-limit sweep re-invokes this exe as a subprocess
    # (a frozen .exe has no `python -m` to call) with a sentinel first arg.
    if len(sys.argv) > 1 and sys.argv[1] == "__autolimits__":
        from ._autolimits_cli import main as _autolimits_main
        del sys.argv[1]                       # _autolimits_cli reads argv[1:]
        return _autolimits_main()

    # before anything else, move out of a OneDrive/Dropbox-synced folder so the
    # self-update never rewrites a binary the sync client is watching
    if _relocate_to_install_dir():
        return 0

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("package_dir", nargs="?", default=None)
    ap.add_argument("--root", default=None,
                    help="directory scanned for /api/list and where new "
                         "extractions are written (default: ./output in a "
                         "source checkout, %%TEMP%%\\sw2robot\\output for the "
                         "frozen .exe)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--no-browser", action="store_true",
                    help="do not open the editor in the default browser on "
                         "startup")
    ap.add_argument("--reclaim-port", action="store_true",
                    help=argparse.SUPPRESS)   # internal: set by the self-update
                    # relaunch so the new instance waits for the exact port the
                    # exiting old instance is about to free (see _bind_free_port)
    ap.add_argument("--no-relocate", action="store_true",
                    help=argparse.SUPPRESS)   # internal: set on the relaunch from
                    # the %LOCALAPPDATA% install copy so it doesn't re-relocate
    args = ap.parse_args()
    serve(args.package_dir, root_dir=args.root, port=args.port,
          open_browser=not args.no_browser, reclaim_port=args.reclaim_port)


if __name__ == "__main__":
    main()
