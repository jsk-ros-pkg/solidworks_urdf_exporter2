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
import io
import json
import os
import posixpath
import http.server
import socket
import socketserver
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(HERE)
PROJECT_ROOT = os.path.dirname(PACKAGE_ROOT)
WEB_DIR = os.path.join(HERE, "web")

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
    print(f"[cad2rc.web] converting {os.path.basename(src)} -> glb ...")
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
                        print(f"[cad2rc.web] preconvert {f}: {e!r}")
        if n:
            print(f"[cad2rc.web] preconverted {n} meshes to glb")
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
_DEFAULT_CAD_ROOTS = [
    r"G:\共有ドライブ\KXR\kxr-design",
    r"G:\共有ドライブ\Designs\Mechanical Design",
]
_SKIP_DIRS = {"backup", "backups", "old", "_old", "bak", "trash", ".git",
              "sandbox"}
_INDEX_FILE = os.path.join(PROJECT_ROOT, "_sldasm_index.json")
_INDEX_MAX_AGE_S = 24 * 3600
_index = {"byname": {}, "building": False, "count": 0}


def _build_index(roots):
    _index["building"] = True
    byname, n = {}, 0
    try:
        for root in roots:
            if not os.path.isdir(root):
                print(f"[cad2rc.web] index: skipping missing root {root}")
                continue
            print(f"[cad2rc.web] index: walking {root} ...")
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
            json.dump(byname, f)
        print(f"[cad2rc.web] index: {n} .sldasm files indexed")
    except Exception as e:
        print(f"[cad2rc.web] index build failed: {e!r}")
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
                _index["byname"] = json.load(f)
            _index["count"] = sum(len(v)
                                  for v in _index["byname"].values())
            print(f"[cad2rc.web] index: {_index['count']} entries loaded "
                  f"from cache")
            return
        except Exception:
            pass
    threading.Thread(target=_build_index, args=(roots,),
                     daemon=True).start()


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
    root = PROJECT_ROOT
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


def _export_zip(pkg_dir, robot_name, mesh_fmt="native"):
    """ZIP the module package (it already contains package.xml/CMakeLists).

    ``mesh_fmt='dae'`` (the portable ROS variant) emits a standalone
    ``<robot_name>_description`` package: ``package://`` mesh URLs + COLLADA
    ``.dae`` meshes (RViz/Gazebo-loadable, colours kept).
    ``mesh_fmt='stl'`` converts every mesh to STL and rewrites the URDF
    references -- also RViz-displayable but colours are lost.
    ``mesh_fmt='native'`` ships the raw 3DXML/GLB + relative paths for skrobot /
    native-mesh consumers."""
    import io as _io
    import zipfile
    buf = _io.BytesIO()
    if mesh_fmt == "dae":
        from sw2robot.exporter.ros_export import build_ros_description
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            for arc, data in build_ros_description(pkg_dir, robot_name):
                z.writestr(arc, data)
        return buf.getvalue()
    skip_suffix = (".part.glb", ".part.3dxml")
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for dirpath, _dirs, files in os.walk(pkg_dir):
            for f in files:
                full = os.path.join(dirpath, f)
                rel = os.path.relpath(full, pkg_dir).replace("\\", "/")
                low = f.lower()
                if any(low.endswith(s) for s in skip_suffix) \
                        or low.endswith(".3dxml.glb"):
                    continue
                if mesh_fmt == "stl":
                    if low.endswith((".3dxml", ".glb")):
                        import trimesh
                        mesh = trimesh.load(full, force="mesh")
                        if low.endswith(".3dxml"):
                            mesh.apply_scale(0.001)   # mm -> m
                        stem = rel.rsplit(".", 1)[0]
                        z.writestr(stem + ".stl",
                                   mesh.export(file_type="stl"))
                        continue
                    if low.endswith(".urdf"):
                        txt = open(full, encoding="utf-8").read()
                        txt = txt.replace(".3dxml", ".stl") \
                                 .replace(".glb", ".stl")
                        z.writestr(rel, txt)
                        continue
                z.write(full, rel)
    return buf.getvalue()


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
            print(f"[cad2rc.web] warm session ping failed "
                  f"({fails}/3) -- retrying before declaring it dead")
            continue
        print("[cad2rc.web] warm SolidWorks session died while idle; "
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
        print(f"[cad2rc.web] extract: {msg}")

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
        state = core.extract_and_import(sldasm, progress=progress, sw=sw)
        _job["package"] = str(state.package_dir)
        _preconvert_meshes(str(state.package_dir))
        progress(f"done -> {state.package_dir} (SolidWorks kept warm for "
                 f"the next extraction)")
    except Exception as e:
        _job["error"] = f"{type(e).__name__}: {e}"
        print(f"[cad2rc.web] extract FAILED: {e!r}")
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
        import trimesh
        from skrobot.models.urdf import RobotModelFromURDF
        from . import autoinit
        robot = RobotModelFromURDF(urdf_file=urdf_path)
        meshes = {}
        for l in robot.link_list:
            vm = getattr(l, "visual_mesh", None)
            ms = (vm if isinstance(vm, (list, tuple)) else [vm]) \
                if vm is not None else []
            ms = [m for m in ms
                  if hasattr(m, "vertices") and len(m.vertices)]
            if ms:
                meshes[l.name] = (trimesh.util.concatenate(ms)
                                  if len(ms) > 1 else ms[0])
        sc = autoinit.SelfCollision(robot, meshes)
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
        print(f"[cad2rc.web] collision model ready: {len(meshes)} hulls, "
              f"{len(sc.baseline)} baseline pairs")
    except Exception as e:
        with _coll_lock:
            if _coll["key"] == key:
                _coll.update(ctx=None, building=False, error=repr(e))
        print(f"[cad2rc.web] collision model FAILED: {e!r}")


# ---- auto joint limits: self-collision sweep over the live collision model.
# Coarse linear scan brackets the first new self-collision, then a bisection
# refines the boundary (the user's "binary search" idea -- far fewer queries
# than fine stepping, and more precise).  One job at a time; it holds the
# collision lock while it sweeps (it mutates joint angles), so the live drag
# check pauses for its duration.
_limjob = {"running": False, "log": [], "error": None, "results": None,
           "n": 0, "total": 0}
_limjob_lock = threading.Lock()


def _run_auto_limits(pkg_dir, urdf_rel, step_deg, max_deg):
    """Run the self-collision limit sweep in a SUBPROCESS and return
    ``(results_list, error)``.  A subprocess on purpose: the sweep is CPU-bound
    and releases the GIL (numpy / fcl), so running it inside the threaded HTTP
    server makes it thrash the GIL against the browser's idle keep-alive
    threads -- the CPython convoy -- which inflated an 8 s sweep to ~90 s just
    by having a page open.  A fresh process has its own GIL and no server
    threads, so it stays at the true ~8 s (+~3 s to load the model)."""
    import subprocess
    urdf = os.path.join(pkg_dir, urdf_rel)
    if not os.path.exists(urdf):
        return None, "URDF not found"
    _t0 = time.time()
    try:
        p = subprocess.run(
            [sys.executable, "-m", "sw2robot.editor._autolimits_cli",
             urdf, str(step_deg), str(max_deg)],
            capture_output=True, text=True, timeout=900,
            cwd=PROJECT_ROOT)
    except subprocess.TimeoutExpired:
        return None, "sweep timed out"
    if p.returncode != 0:
        tail = (p.stderr or "").strip().splitlines()[-3:]
        return None, "sweep failed: " + " | ".join(tail)
    try:
        out = json.loads(p.stdout)["results"]
    except Exception as e:
        return None, f"bad sweep output: {e}"
    print(f"[cad2rc.web] auto_limits sweep: {time.time() - _t0:.1f}s "
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
                import re
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
                    import re
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
                    for r in [_Handler.root_dir, *_DEFAULT_CAD_ROOTS]:
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
                print(f"[cad2rc.web] locate {name} ({size}B): "
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
                # SUBPROCESS sweep (avoids the GIL convoy; see
                # _run_auto_limits).  One job at a time; blocks ~10 s.
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                with _limjob_lock:
                    if _limjob["running"]:
                        return self._send_json(
                            {"error": "a limit sweep is already running"}, 409)
                    _limjob["running"] = True
                try:
                    step = float((query.get("step") or ["10"])[0])
                    mx = float((query.get("max") or ["360"])[0])  # ±2π
                    results, err = _run_auto_limits(
                        cls.pkg_dir, cls.urdf_rel, step, mx)
                finally:
                    _limjob["running"] = False
                if err:
                    return self._send_json({"error": err}, 409)
                return self._send_json({"results": results})
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
                print(f"[cad2rc.web] open: {cls.robot_name} ({pkg})")
                _preconvert_meshes(pkg)
                return self._send_json(self._info())
            if path == "/api/export/zip":
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                fmt = (query.get("meshes") or ["native"])[0]
                data = _export_zip(cls.pkg_dir, cls.robot_name, fmt)
                fname = (f"{cls.robot_name}_description.zip" if fmt == "dae"
                         else f"{cls.robot_name}_ros.zip")
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{fname}"')
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return None
            if path == "/api/export/rczip":
                cls = type(self)
                if not cls.pkg_dir:
                    return self._send_json({"error": "no package open"}, 400)
                from . import core
                import tempfile
                state = core.load_module(
                    os.path.join(cls.pkg_dir, cls.urdf_rel),
                    package_dir=cls.pkg_dir)
                out = os.path.join(tempfile.gettempdir(),
                                   cls.robot_name + "_configs.zip")
                core.export_ros_package(state, out)
                with open(out, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "application/zip")
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{cls.robot_name}'
                                 f'_configs.zip"')
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
            print(f"[cad2rc.web] {self.path}: {e!r}")
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
                if os.environ.get("CAD2RC_TIME_COLLISION"):
                    print(f"[cad2rc.web] /api/collision read={_t1-_t0:.3f}s "
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
                applied, missed = [], []
                for lm in limits:
                    c = lm.get("child")
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
                print(f"[cad2rc.web] set_limits: {len(applied)} applied, "
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
                import re
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
                applied, missed = [], []
                for ch in changes:
                    c, t = ch.get("child"), ch.get("type")
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
                print(f"[cad2rc.web] set_types: {len(applied)} applied, "
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
                import re
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
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
                print(f"[cad2rc.web] set_base: {new_root} "
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
                import re
                import numpy as np
                from sw2robot.exporter.geometry import (matrix_from_rpy,
                                              matrix_to_xyz_rpy)
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
                    z = np.asarray(zdir, float)
                    z = z / np.linalg.norm(z)
                    ez = np.array([0.0, 0.0, 1.0])
                    v = np.cross(ez, z)
                    c = float(ez @ z)
                    if np.linalg.norm(v) < 1e-9:
                        if c < 0:          # flipped: 180 deg about X
                            D[:3, :3] = np.diag([1.0, -1.0, -1.0])
                    else:
                        K = np.array([[0, -v[2], v[1]],
                                      [v[2], 0, -v[0]],
                                      [-v[1], v[0], 0]])
                        D[:3, :3] = (np.eye(3) + K
                                     + K @ K * (1.0 / (1.0 + c)))
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
                print(f"[cad2rc.web] set_root_pose: xyz={p.tolist()} "
                      f"zdir={zdir}")
                return self._send_json({"rpy": list(rpy_new),
                                        "xyz": list(xyz_new)})
            if parsed.path == "/api/client_log":
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                out = os.path.join(PROJECT_ROOT, "_client_report.json")
                with open(out, "w", encoding="utf-8") as f:
                    json.dump(body, f, ensure_ascii=False, indent=1)
                print(f"[cad2rc.web] CLIENT REPORT -> {out}")
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
                import re
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
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
                print(f"[cad2rc.web] set_exclude: {name} "
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
                import re
                with open(yml, encoding="utf-8") as f:
                    txt = f.read()
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
                print(f"[cad2rc.web] set_material: {link} -> {density}")
                return self._send_json({"link": link, "density": density})
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
                print(f"[cad2rc.web] {src}: {label}")
                return self._send_json(
                    {"done": src, "label": label,
                     "undo": len(h["undo"]), "redo": len(h["redo"])})
            if parsed.path == "/api/register":
                cls = type(self)
                n = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(n) or b"{}")
                reg = body.get("dir")
                if not cls.pkg_dir or not reg:
                    return self._send_json(
                        {"error": "no package open / no registry dir"}, 400)
                from . import core
                state = core.load_module(
                    os.path.join(cls.pkg_dir, cls.urdf_rel),
                    package_dir=cls.pkg_dir)
                dst = core.register_module(state, reg)
                print(f"[cad2rc.web] registered -> {dst}")
                return self._send_json({"registered": str(dst)})
            if parsed.path == "/api/convert":
                n = int(self.headers.get("Content-Length", 0))
                if not (0 < n < 200 * 1024 * 1024):
                    return self.send_error(400, "bad length")
                data = self.rfile.read(n)
                glb = _convert_3dxml_bytes(data)
                print(f"[cad2rc.web] /api/convert: {n}B 3dxml -> "
                      f"{len(glb)}B glb")
                return self._send_bytes(glb, "model/gltf-binary")
            return self.send_error(404)
        except BrokenPipeError:
            pass
        except Exception as e:
            print(f"[cad2rc.web] {self.path}: {e!r}")
            try:
                self.send_error(500, str(e))
            except Exception:
                pass

    def log_message(self, fmt, *args):   # quiet; errors print above
        pass


def serve(package_dir=None, root_dir=None, port=8090, cad_roots=None):
    import atexit
    atexit.register(_shutdown_sw)     # close the warm session on exit
    threading.Thread(target=_keepalive_loop, daemon=True).start()
    _Handler.root_dir = os.path.abspath(root_dir or "output")
    _ensure_index(cad_roots or _DEFAULT_CAD_ROOTS)
    if package_dir:
        pkg, rel = _resolve_package(package_dir)
        _Handler.pkg_dir, _Handler.urdf_rel = pkg, rel
        _Handler.robot_name = os.path.splitext(os.path.basename(rel))[0]
        print(f"[cad2rc.web] serving '{_Handler.robot_name}' from {pkg}")
    else:
        print(f"[cad2rc.web] no package yet -- pick one in the browser "
              f"(root: {_Handler.root_dir})")
    httpd = socketserver.ThreadingTCPServer(("", port), _Handler)
    httpd.daemon_threads = True
    print(f"[cad2rc.web] open http://localhost:{port}")
    httpd.serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("package_dir", nargs="?", default=None)
    ap.add_argument("--root", default="output",
                    help="directory scanned for /api/list (default: output)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--cad-roots", nargs="*", default=None,
                    help="directories indexed for drag&drop .sldasm lookup "
                         "(default: KXR + Mechanical Design shares)")
    args = ap.parse_args()
    serve(args.package_dir, root_dir=args.root, port=args.port,
          cad_roots=args.cad_roots)


if __name__ == "__main__":
    main()
