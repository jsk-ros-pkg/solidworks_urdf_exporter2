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
    /api/convert  (POST)  3DXML bytes in -> GLB bytes out (mm -> m), so the
                          page can also render drag&dropped local packages
    /pkg/<rel>            files from the CURRENT package dir
    /pkg/<rel>.3dxml?glb=1  the mesh converted to GLB (three.js cannot read
                          3DXML), cached next to the source as <rel>.3dxml.glb

Single-user LOCAL tool by design: /api/open accepts arbitrary local paths on
purpose (that's the file picker), so never expose this server beyond
localhost.  No third-party server deps; mesh conversion reuses trimesh which
sw2robot.exporter already requires.
"""
import argparse
import http.server
import io
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


def _app_data_dir():
    """Writable base for runtime side-files (the drive index, the default
    package root, the client report).  A PyInstaller-frozen exe's PROJECT_ROOT
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


def _convert_3dxml_bytes(data):
    """3DXML bytes (drag&dropped file) -> GLB bytes, mm -> m."""
    import trimesh
    mesh = _to_single_mesh(trimesh.load(io.BytesIO(data), file_type="3dxml"))
    mesh.apply_scale(0.001)
    mesh.units = "meter"
    return mesh.export(file_type="glb")


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
                urdfs = sorted(f for f in os.listdir(d)
                               if f.lower().endswith(".urdf"))
                if urdfs:
                    rel = os.path.relpath(os.path.join(d, urdfs[0]), path)
                    return path, rel.replace("\\", "/")
        raise ValueError(f"no *.urdf under {path}")
    raise ValueError(f"not a package dir or .urdf file: {path}")


# --- .sldasm basename index (NO SolidWorks needed: plain os.walk) ---------
# Google Drive for Desktop localises the "Shared drives" mount folder by the
# app's display LANGUAGE -- it is 共有ドライブ in Japanese, "Shared drives" in
# English, "Unidades compartidas" in Spanish, and so on.  Hardcoding one name
# silently breaks the index (and every absolute part reference) the moment the
# language is switched, so resolve the live name at runtime instead.
_GDRIVE_LETTER = "G:"
_SHARED_DRIVE_NAMES = ["共有ドライブ", "Shared drives", "Unidades compartidas",
                       "Geteilte Ablagen", "Drive partagés", "공유 드라이브",
                       "共享云端硬盘", "Gedeelde drives"]
# CAD roots as paths RELATIVE to the shared-drives mount folder
_CAD_ROOT_RELS = [
    r"KXR\kxr-design",
    r"Designs\Mechanical Design",
]


def _shared_drives_base():
    """The Google Drive 'Shared drives' mount folder under G:, by whatever
    localised name it currently carries; None if G: isn't mounted."""
    drive = _GDRIVE_LETTER + "\\"
    for name in _SHARED_DRIVE_NAMES:               # known localisations first
        p = os.path.join(drive, name)
        if os.path.isdir(p):
            return p
    # unknown locale: pick any G:\<dir> that actually holds our CAD drives
    firsts = {rel.split("\\", 1)[0].lower() for rel in _CAD_ROOT_RELS}
    try:
        for name in os.listdir(drive):
            p = os.path.join(drive, name)
            if os.path.isdir(p) and any(
                    os.path.isdir(os.path.join(p, d)) for d in
                    os.listdir(p) if d.lower() in firsts):
                return p
    except OSError:
        pass
    return None


def _default_cad_roots():
    """The configured CAD roots, resolved against the live shared-drives mount
    name; empty when G: isn't available (the index then simply finds nothing
    instead of walking dead Japanese-named paths)."""
    base = _shared_drives_base()
    if not base:
        return []
    return [os.path.join(base, rel) for rel in _CAD_ROOT_RELS]


def _remap_share(path, base=None):
    """Rewrite ``G:\\<old shared-drives name>\\...`` to the CURRENT mount name.

    When Google Drive's display language is switched the only thing that moves
    is the localised top folder (共有ドライブ <-> Shared drives <-> ...); the
    sub-tree is identical.  So a stale cached path is fixed by swapping just
    that one segment -- no need to re-walk the share (minutes over streaming).
    Leaves non-G:, already-current, and unrecognised paths untouched."""
    base = base or _shared_drives_base()
    drive = _GDRIVE_LETTER + "\\"
    if not base or not path.startswith(drive):
        return path
    oldname, _, tail = path[len(drive):].partition("\\")
    cur = os.path.basename(base)
    if tail and oldname != cur and oldname in _SHARED_DRIVE_NAMES:
        return os.path.join(base, tail)
    return path


_SKIP_DIRS = {"backup", "backups", "old", "_old", "bak", "trash", ".git",
              "sandbox"}
_INDEX_FILE = os.path.join(_DATA_DIR, "_sldasm_index.json")
_INDEX_MAX_AGE_S = 24 * 3600
_index = {"byname": {}, "building": False, "count": 0}


def _build_index(roots):
    _index["building"] = True
    byname, n = {}, 0
    try:
        for root in roots:
            if not os.path.isdir(root):
                print(f"[sw2robot.web] index: skipping missing root {root}")
                continue
            print(f"[sw2robot.web] index: walking {root} ...")
            for dirpath, dirnames, filenames in os.walk(root):
                dirnames[:] = [d for d in dirnames
                               if d.lower() not in _SKIP_DIRS
                               and not d.lower().endswith("_backups")]
                for f in filenames:
                    if f.lower().endswith(".sldasm") \
                            and not f.startswith("~$"):
                        byname.setdefault(f.lower(), []).append(
                            os.path.join(dirpath, f))
                        n += 1
        _index["byname"], _index["count"] = byname, n
        with open(_INDEX_FILE, "w", encoding="utf-8") as f:
            json.dump({"roots": list(roots), "byname": byname}, f)
        print(f"[sw2robot.web] index: {n} .sldasm files indexed")
    except Exception as e:
        print(f"[sw2robot.web] index build failed: {e!r}")
    finally:
        _index["building"] = False


def _ensure_index(roots):
    if _index["byname"] or _index["building"]:
        return
    if os.path.exists(_INDEX_FILE) \
            and time.time() - os.path.getmtime(_INDEX_FILE) \
            < _INDEX_MAX_AGE_S:
        try:
            with open(_INDEX_FILE, encoding="utf-8") as f:
                cached = json.load(f)
            # accept both the new {"roots","byname"} and the old plain-{byname}
            byname = cached["byname"] if isinstance(cached, dict) \
                and "byname" in cached else cached
            # the shared-drives mount may have been RENAMED (locale switch)
            # since the cache was written -- repoint the stored paths to the
            # live name in memory instead of re-walking the share (a walk over
            # Google Drive streaming costs minutes for ~20k files).
            base = _shared_drives_base()
            n_remap = 0
            for k, paths in byname.items():
                fixed = [_remap_share(p, base) for p in paths]
                n_remap += sum(a != b for a, b in zip(paths, fixed))
                byname[k] = fixed
            _index["byname"] = byname
            _index["count"] = sum(len(v) for v in byname.values())
            note = (f" ({n_remap} repointed to '{os.path.basename(base)}')"
                    if n_remap and base else "")
            print(f"[sw2robot.web] index: {_index['count']} entries loaded "
                  f"from cache{note}")
            return
        except Exception:
            pass
    threading.Thread(target=_build_index, args=(roots,),
                     daemon=True).start()


def _shell_folder_roots():
    """Desktop / Downloads / Documents as Windows ACTUALLY has them, read from
    the per-user 'User Shell Folders' registry.  This is the only reliable way
    when OneDrive's Known-Folder-Move has redirected them away from the plain
    ``~\\Desktop`` path (a guess that then silently doesn't exist)."""
    if os.name != "nt":
        return []
    keys = {"Desktop": "Desktop",
            "Personal": "Documents",                       # Documents
            "{374DE290-123F-4565-9164-39C4925E467B}": "Downloads"}
    out = []
    try:
        import winreg
        with winreg.OpenKey(
                winreg.HKEY_CURRENT_USER,
                r"Software\Microsoft\Windows\CurrentVersion\Explorer"
                r"\User Shell Folders") as k:
            for val in keys:
                try:
                    raw, _ = winreg.QueryValueEx(k, val)
                except OSError:
                    continue
                out.append(os.path.expandvars(raw))   # %USERPROFILE% etc.
    except Exception:
        pass
    return out


def _local_search_roots():
    """Likely local spots for a hand-dropped .sldasm that lives OUTSIDE the
    indexed CAD shares -- Desktop / Downloads / Documents under the user
    profile (and their OneDrive mirrors).  A GUI .exe user typically drops a
    working copy from one of these, which the drive index never walks."""
    cands = list(_shell_folder_roots())          # registry truth first
    bases = [os.path.expanduser("~")]
    for var in ("OneDrive", "OneDriveConsumer", "OneDriveCommercial"):
        if os.environ.get(var):
            bases.append(os.environ[var])
    for b in bases:                              # plain-path fallback guesses
        for sub in ("Desktop", "Downloads", "Documents"):
            cands.append(os.path.join(b, sub))
    seen, out = set(), []
    for c in cands:
        c = os.path.normpath(c)
        if c.lower() not in seen and os.path.isdir(c):
            seen.add(c.lower())
            out.append(c)
    return out


def _deep_find_basename(roots, base, time_budget_s=6.0):
    """Bounded on-demand walk of ``roots`` for a file whose lowercased basename
    is ``base``.  Stops at ``time_budget_s`` so a huge Downloads tree can't hang
    the /api/locate request; skips the same junk dirs as the index build."""
    hits = []
    deadline = time.time() + time_budget_s
    for root in roots:
        for dirpath, dirnames, filenames in os.walk(root):
            if time.time() > deadline:
                return hits, True            # timed out -- partial result
            dirnames[:] = [d for d in dirnames
                           if d.lower() not in _SKIP_DIRS
                           and not d.lower().endswith("_backups")]
            for f in filenames:
                if f.lower() == base and not f.startswith("~$"):
                    hits.append(os.path.join(dirpath, f))
    return hits, False


def _locate_sldasm(name, size=None):
    """Find the real on-disk path(s) of a drag&dropped .sldasm by fingerprint.

    Browsers hide real file paths on drop, but basename (+size as a
    preference, not a hard filter) identifies the file well enough to look
    it up in: the SolidWorks MRU, the source_assembly of every extracted
    package under output/, the drive index _sldasm_index.json (built by
    plain os.walk -- no SolidWorks), and _find_roots_sw.py's cache."""
    base = os.path.basename(str(name)).lower()
    seen, out = set(), []

    def consider(p):
        p = os.path.normpath(str(p))
        k = p.lower()
        if k in seen or os.path.basename(k) != base:
            return
        seen.add(k)
        try:
            st = os.stat(p)
        except OSError:
            return
        out.append({"path": p, "size": st.st_size,
                    "mtime": int(st.st_mtime),
                    "size_match": bool(size)
                    and st.st_size == int(size)})

    try:
        from . import core
        for p in core.sw_recent_assemblies(50):
            consider(p)
    except Exception:
        pass
    root = _DATA_DIR
    out_dir = os.path.join(root, "output")
    if os.path.isdir(out_dir):                  # extracted packages know
        for d in os.listdir(out_dir):           # their own source path
            g = os.path.join(out_dir, d, "graph.json")
            if os.path.exists(g):
                try:
                    from sw2robot.exporter.state import GraphState
                    consider(GraphState.load(g).source_assembly or "")
                except Exception:
                    pass
    for p in _index["byname"].get(base, []):
        consider(p)
    cache = os.path.join(root, "_roots_cache.jsonl")
    if os.path.exists(cache):
        with open(cache, encoding="utf-8") as f:
            for line in f:
                try:
                    key = json.loads(line).get("key", "")
                except Exception:
                    continue
                p = key.rsplit("|", 1)[0]
                if os.path.basename(p).lower() == base:
                    consider(p)
    # last resort: the indexed sources (MRU / output / drive index / roots
    # cache) all cover the CAD shares, not local folders.  A GUI .exe user
    # usually drops a working copy from Desktop/Downloads/Documents, so walk
    # those on demand before giving up.
    if not out:
        local, timed_out = _deep_find_basename(_local_search_roots(), base)
        for p in local:
            consider(p)
        if timed_out:
            print(f"[sw2robot.web] local search for {base} hit its time "
                  f"budget; results may be incomplete")
    # exact-size hits first, then newest
    out.sort(key=lambda h: (not h["size_match"], -h["mtime"]))
    return out


def _read_root_pose(txt):
    """Current root_rpy / root_xyz / root_z_offset from joints.yaml text."""
    import re

    def vec(key, default):
        m = re.search(r"(?m)^" + key + r":\s*\[([^\]]*)\]", txt)
        return [float(x) for x in m.group(1).split(",")] if m else default

    m = re.search(r"(?m)^root_z_offset:\s*([-\d.eE]+)", txt)
    return (vec("root_rpy", [0, 0, 0]), vec("root_xyz", [0, 0, 0]),
            float(m.group(1)) if m else 0.0)


def _export_zip(pkg_dir, robot_name, mesh_fmt="dae"):
    """ZIP a portable ``<robot_name>_description`` package (package:// URLs).

    ``mesh_fmt='dae'`` (default): ``<visual>`` as colour COLLADA ``.dae`` +
    ``<collision>`` as plain ``.stl`` -- the RViz/Gazebo-ready variant.
    ``mesh_fmt='glb'``: a uniform ``.glb`` package (colour kept) for three.js /
    skrobot / native-mesh consumers (not RViz-loadable)."""
    if mesh_fmt not in ("dae", "glb"):
        raise ValueError(f"unsupported mesh format: {mesh_fmt}")

    import io as _io
    import zipfile

    from sw2robot.exporter.ros_export import GLB_CTX_FMT, build_ros_description
    kwargs = {"ctx_fmt": GLB_CTX_FMT} if mesh_fmt == "glb" else {}
    buf = _io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for arc, data in build_ros_description(pkg_dir, robot_name, **kwargs):
            z.writestr(arc, data)
    return buf.getvalue()


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
_job = {"running": False, "log": [], "error": None, "package": None}
_job_lock = threading.Lock()
# warm SolidWorks session kept across extractions: starting SolidWorks is
# by far the slowest stage (~1-2 min), so pay it once per server lifetime
_sw = {"sess": None}


def _warm_sw(progress):
    sess = _sw["sess"]
    if sess is not None:
        from sw2robot.exporter.swcom import safe_prop
        progress("checking the warm SolidWorks session (an idle session "
                 "can take a moment to respond) ...")
        t0 = time.time()
        for attempt in (1, 2, 3):    # transient RPC-busy is not death
            alive = sess.app is not None \
                and safe_prop(sess.app, "Visible") is not None
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
    from sw2robot.exporter.swcom import safe_prop
    fails = 0
    while True:
        time.sleep(60)
        sess = _sw.get("sess")
        if sess is None or _job["running"]:
            fails = 0
            continue
        alive = False
        try:
            alive = sess.app is not None \
                and safe_prop(sess.app, "Visible") is not None
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
            sess.shutdown()
        except Exception:
            pass
        _sw["sess"] = None


def _run_extract(sldasm):
    """Background thread: SolidWorks extract + build -> module package."""
    def progress(msg):
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
            progress("starting SolidWorks (this can take a minute; later "
                     "extractions reuse this session) ...")
            sw = SolidWorks(visible=False)
            _sw["sess"] = sw
        state = core.extract_and_import(
            sldasm, out_dir=_Handler.root_dir, progress=progress, sw=sw)
        _job["package"] = str(state.package_dir)
        _preconvert_meshes(str(state.package_dir))
        progress(f"done -> {state.package_dir} (SolidWorks kept warm for "
                 f"the next extraction)")
    except Exception as e:
        _job["error"] = f"{type(e).__name__}: {e}"
        print(f"[sw2robot.web] extract FAILED: {e!r}")
        if "-2147417848" in repr(e) or "-2147417856" in repr(e):
            _sw["sess"] = None     # session died; next run starts fresh
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


def _build_collision(urdf_path, key):
    try:
        import numpy as np
        from skrobot.models.urdf import RobotModelFromURDF

        from . import autoinit
        robot = RobotModelFromURDF(urdf_file=urdf_path)
        meshes = autoinit.link_meshes(robot)
        # confirm=True: hull broadphase + exact-mesh verification, so the live
        # red highlight matches the exact-mesh joint-limit sweep (fat hulls
        # would otherwise light up red before the joint reaches its limit).
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
        print(f"[sw2robot.web] collision model ready: {len(meshes)} hulls, "
              f"{len(sc.baseline)} baseline pairs")
    except Exception as e:
        with _coll_lock:
            if _coll["key"] == key:
                _coll.update(ctx=None, building=False, error=repr(e))
        print(f"[sw2robot.web] collision model FAILED: {e!r}")


# ---- auto joint limits: self-collision sweep over the live collision model.
# Coarse linear scan brackets the first new self-collision, then a bisection
# refines the boundary (the user's "binary search" idea -- far fewer queries
# than fine stepping, and more precise).  One job at a time; it holds the
# collision lock while it sweeps (it mutates joint angles), so the live drag
# check pauses for its duration.
_limjob = {"running": False, "log": [], "error": None, "results": None,
           "n": 0, "total": 0, "joint": None, "phase": None}
_limjob_lock = threading.Lock()


def _run_auto_limits(pkg_dir, urdf_rel, step_deg, max_deg):
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
    cmd += [urdf, str(step_deg), str(max_deg)]
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
        return {"name": cls.robot_name, "urdf": "/pkg/" + cls.urdf_rel} \
            if cls.urdf_rel else {"name": None, "urdf": None}

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
                links = {}
                for c in gs.components:
                    links[c.link_name] = {
                        "material": c.material, "density": c.density,
                        "name": c.name,
                        "override": overrides.get(c.link_name)}
                excluded = []
                if os.path.exists(yml):
                    m = re.search(r"(?m)^exclude:\n((?:- .*\n)*)",
                                  open(yml, encoding="utf-8").read())
                    if m:
                        excluded = [ln[2:].strip()
                                    for ln in m.group(1).splitlines()]
                return self._send_json({"links": links,
                                        "excluded": excluded})
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
                    for r in [_Handler.root_dir, *_default_cad_roots()]:
                        if r and os.path.isdir(r) and r not in roots:
                            roots.append(r)
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
            if path == "/api/locate":
                name = (query.get("name") or [""])[0]
                size = (query.get("size") or [None])[0]
                if not name.lower().endswith(".sldasm"):
                    return self._send_json({"error": "need a .sldasm name"},
                                           400)
                hits = _locate_sldasm(name, size)
                print(f"[sw2robot.web] locate {name} ({size}B): "
                      f"{len(hits)} hit(s)"
                      + (" [index still building]"
                         if _index["building"] else ""))
                return self._send_json({"hits": hits,
                                        "building": _index["building"]})
            if path == "/api/collision/init":
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                urdf_path = os.path.join(cls.pkg_dir, cls.urdf_rel)
                key = (urdf_path, os.path.getmtime(urdf_path))
                with _coll_lock:
                    if _coll["key"] != key or (
                            _coll["ctx"] is None and not _coll["building"]
                            and not _coll["error"]):
                        _coll.update(key=key, ctx=None, building=True,
                                     error=None)
                        threading.Thread(target=_build_collision,
                                         args=(urdf_path, key),
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
                # parse BEFORE claiming the job, so a bad step/max can't wedge
                # the "running" flag (no worker would start to clear it)
                try:
                    step = float((query.get("step") or ["10"])[0])
                    mx = float((query.get("max") or ["360"])[0])  # ±2π
                except ValueError:
                    return self._send_json({"error": "bad step/max"}, 400)
                with _limjob_lock:
                    if _limjob["running"]:
                        return self._send_json(
                            {"error": "a limit sweep is already running"}, 409)
                    _limjob.update(running=True, log=[], error=None,
                                   results=None, n=0, total=0, joint=None,
                                   phase="loading")
                pkg, rel = cls.pkg_dir, cls.urdf_rel

                # NB: must NOT be named `_job` -- that shadows the module-global
                # `_job` (the extraction job) across this whole method and breaks
                # the /api/extract* handlers with an UnboundLocalError.
                def _sweep_job():
                    try:
                        results, err = _run_auto_limits(pkg, rel, step, mx)
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
                                package=None)
                threading.Thread(target=_run_extract, args=(target,),
                                 daemon=True).start()
                return self._send_json({"started": True})
            if path == "/api/extract/status":
                return self._send_json(_job)
            if path == "/api/open":
                target = (query.get("path") or [""])[0]
                try:
                    pkg, rel = _resolve_package(target)
                except ValueError as e:
                    return self._send_json({"error": str(e)}, 400)
                cls.pkg_dir, cls.urdf_rel = pkg, rel
                cls.robot_name = os.path.splitext(os.path.basename(rel))[0]
                print(f"[sw2robot.web] open: {cls.robot_name} ({pkg})")
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
                data = _export_zip(cls.pkg_dir, cls.robot_name, fmt)
                fname = (f"{cls.robot_name}_glb.zip" if fmt == "glb"
                         else f"{cls.robot_name}_description.zip")
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{fname}"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return None
            if path.startswith("/pkg/"):
                if not cls.pkg_dir:
                    return self.send_error(404, "no package open")
                full = self._resolve(cls.pkg_dir, path[len("/pkg/"):])
                if full is None or not os.path.isfile(full):
                    return self.send_error(404)
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
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                linv = _link_names_inverse(txt)   # display -> component
                applied, missed = [], []
                for lm in limits:
                    c = linv.get(lm.get("child"), lm.get("child"))
                    if not c:
                        continue
                    txt, k = _set_joint_limit(
                        txt, c, float(lm.get("lower", 0.0)),
                        float(lm.get("upper", 0.0)),
                        bool(lm.get("continuous")))
                    (applied if k else missed).append(c)
                if applied:
                    _snapshot(cls.pkg_dir, yml, f"auto limits x{len(applied)}")
                    with open(yml, "w", encoding="utf-8") as f:
                        f.write(txt)
                    from sw2robot.exporter.export import build
                    try:
                        build(cls.pkg_dir, config_path=yml)
                    except Exception as e:
                        return self._send_json(
                            {"error": f"rebuild failed: {e}"}, 500)
                print(f"[sw2robot.web] set_limits: {len(applied)} applied, "
                      f"{len(missed)} not matched")
                return self._send_json({"applied": applied, "missed": missed})
            if parsed.path == "/api/set_types":
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                changes = body.get("changes") or []
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
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
                # exclude a COMPONENT from the built URDF entirely (yaml
                # `exclude:` list), or restore: {name, on} / {clear: true}
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                name = body.get("name")
                if not cls.pkg_dir or (not name and not body.get("clear")):
                    return self._send_json({"error": "no package/name"}, 400)
                pkg = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, pkg + ".joints.yaml")
                if not os.path.exists(yml):
                    return self._send_json(
                        {"error": "joints.yaml not found"}, 400)
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                if name:                       # a renamed link -> component
                    name = _link_names_inverse(txt).get(name, name)
                _snapshot(cls.pkg_dir, yml,
                          "restore excluded" if body.get("clear")
                          else f"exclude {str(name)[:30]}")
                m = re.search(r"(?m)^exclude:\n((?:- .*\n)*)", txt)
                block = m.group(1) if m else ""
                if body.get("clear"):
                    block = ""
                else:
                    block = re.sub(r"(?m)^- " + re.escape(name)
                                   + r"\s*$\n?", "", block)
                    if body.get("on", True):
                        block += f"- {name}\n"
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
                print(f"[sw2robot.web] set_exclude: {name} "
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
                from sw2robot.exporter.export import build
                try:
                    build(cls.pkg_dir, config_path=yml)
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
                name = os.path.splitext(os.path.basename(cls.urdf_rel))[0]
                yml = os.path.join(cls.pkg_dir, name + ".joints.yaml")
                h = _hist(cls.pkg_dir)
                src = "undo" if parsed.path.endswith("undo") else "redo"
                dst = "redo" if src == "undo" else "undo"
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
            if parsed.path == "/api/convert":
                n = int(self.headers.get("Content-Length", 0))
                if not (0 < n < 200 * 1024 * 1024):
                    return self.send_error(400, "bad length")
                data = self.rfile.read(n)
                glb = _convert_3dxml_bytes(data)
                print(f"[sw2robot.web] /api/convert: {n}B 3dxml -> "
                      f"{len(glb)}B glb")
                return self._send_bytes(glb, "model/gltf-binary")
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


def serve(package_dir=None, root_dir=None, port=8090, cad_roots=None,
          open_browser=True):
    import atexit
    atexit.register(_shutdown_sw)     # close the warm session on exit
    threading.Thread(target=_keepalive_loop, daemon=True).start()
    _Handler.root_dir = os.path.abspath(root_dir or _default_root())
    _ensure_index(cad_roots or _default_cad_roots())
    if package_dir:
        pkg, rel = _resolve_package(package_dir)
        _Handler.pkg_dir, _Handler.urdf_rel = pkg, rel
        _Handler.robot_name = os.path.splitext(os.path.basename(rel))[0]
        print(f"[sw2robot.web] serving '{_Handler.robot_name}' from {pkg}")
    else:
        print(f"[sw2robot.web] no package yet -- pick one in the browser "
              f"(root: {_Handler.root_dir})")
    httpd = socketserver.ThreadingTCPServer(("", port), _Handler)
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


def main():
    # self-dispatch: the auto-limit sweep re-invokes this exe as a subprocess
    # (a frozen .exe has no `python -m` to call) with a sentinel first arg.
    if len(sys.argv) > 1 and sys.argv[1] == "__autolimits__":
        from ._autolimits_cli import main as _autolimits_main
        del sys.argv[1]                       # _autolimits_cli reads argv[1:]
        return _autolimits_main()

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("package_dir", nargs="?", default=None)
    ap.add_argument("--root", default=None,
                    help="directory scanned for /api/list and where new "
                         "extractions are written (default: ./output in a "
                         "source checkout, %TEMP%\\sw2robot\\output for the "
                         "frozen .exe)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--cad-roots", nargs="*", default=None,
                    help="directories indexed for drag&drop .sldasm lookup "
                         "(default: KXR + Mechanical Design shares)")
    ap.add_argument("--no-browser", action="store_true",
                    help="do not open the editor in the default browser on "
                         "startup")
    args = ap.parse_args()
    serve(args.package_dir, root_dir=args.root, port=args.port,
          cad_roots=args.cad_roots, open_browser=not args.no_browser)


if __name__ == "__main__":
    main()
