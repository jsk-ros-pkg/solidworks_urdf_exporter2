"""Self-update for the frozen sw2robot-web editor.

The editor ships as a single PyInstaller binary attached to a GitHub Release
(``sw2robot-web-<os>-<arch>-vX.Y.Z`` -- ``.exe`` on Windows, a zipped ``.app``
on macOS, a bare ELF on Linux).  This module lets the browser UI:

  * check whether a newer Release exists (:func:`check_for_update`), and
  * download it, swap the running binary in place and relaunch
    (:func:`start_update` + the background :func:`_run_update` job), all over
    stdlib ``urllib`` -- no new runtime dependency (the web server is
    deliberately third-party-free).

Swapping a RUNNING executable is the per-platform tricky part:

  * Windows / Linux (single file): a running binary may be *renamed* (not
    overwritten), so move the current exe aside (``<exe>.old``), move the
    freshly downloaded one into its place, relaunch it, exit.  The ``.old``
    image can't be deleted while it is the running process -- the next launch
    reaps it (:func:`reap_leftovers`).
  * macOS (``.app`` bundle): the same rename trick on the bundle *directory*
    (the running Mach-O stays mapped): rename ``Foo.app`` aside, unzip the new
    bundle into place, restore the exec bits ``zip`` drops, relaunch via
    ``open``, exit.

Self-update only makes sense for a frozen build; from a source checkout it is a
no-op that points the user at ``git pull`` / ``pip install -U``.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

from .. import __version__

REPO = "jsk-ros-pkg/solidworks_urdf_exporter2"
_API_LATEST = f"https://api.github.com/repos/{REPO}/releases/latest"
RELEASES_URL = f"https://github.com/{REPO}/releases/latest"
_UA = f"sw2robot-web/{__version__}"

# Cache the GitHub answer so repeated UI polls don't burn the unauthenticated
# 60-requests/hour rate limit; 15 min is plenty for a desktop tool.
_CHECK_TTL = 900
_cache = {"at": 0.0, "result": None}
_cache_lock = threading.Lock()

# The in-flight download/swap job the UI polls (mirrors webserver's _job).
_update = {"state": "idle", "pct": 0, "error": None, "version": None,
           "downloaded": 0, "total": 0}
_update_lock = threading.Lock()


def is_frozen() -> bool:
    return bool(getattr(sys, "frozen", False))


def platform_key() -> str:
    """``<os>-<arch>`` matching the ``-<os>-<arch>-`` infix the release matrix
    bakes into every asset name (see .github/workflows/release.yml)."""
    import platform
    if sys.platform == "win32":
        osname = "windows"
    elif sys.platform == "darwin":
        osname = "macos"
    else:
        osname = "linux"
    m = platform.machine().lower()
    if m in ("x86_64", "amd64"):
        arch = "x64"
    elif m in ("arm64", "aarch64"):
        arch = "arm64"
    else:
        arch = m or "x64"
    return f"{osname}-{arch}"


def _parse_version(tag):
    """``v1.2.3`` / ``1.2.3`` -> ``(1, 2, 3)``; ``()`` when nothing parses
    (treated as the oldest possible version)."""
    nums = re.findall(r"\d+", tag or "")
    return tuple(int(n) for n in nums) if nums else ()


def _is_newer(latest, current) -> bool:
    return _parse_version(latest) > _parse_version(current)


def _http_json(url, timeout):
    req = urllib.request.Request(url, headers={
        "User-Agent": _UA,
        "Accept": "application/vnd.github+json",
    })
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def _pick_asset(assets):
    """The release asset for THIS platform, matched by the ``-<os>-<arch>-``
    infix; None when the matrix didn't ship one for this OS.

    Falls back to the same-OS x64 build when the exact arch isn't published
    (the matrix ships only windows-x64 / linux-x64) -- on ARM64 Windows the
    distributed x64 exe runs under emulation and reports x64 anyway, but a
    native-ARM run would otherwise dead-end with 'no asset'."""
    osname = platform_key().split("-", 1)[0]
    exact = f"-{platform_key()}-"
    same_os_x64 = f"-{osname}-x64-"
    fallback = None
    for a in assets:
        name = a.get("name") or ""
        if exact in name:
            return a
        if same_os_x64 in name:
            fallback = a
    return fallback


def check_for_update(timeout=6, force=False):
    """``{current, latest, update_available, html_url, asset, frozen,
    platform, error}`` -- the GitHub 'latest release' compared to the running
    version.  Cached for ``_CHECK_TTL`` s; ``force`` bypasses the cache.  Never
    raises: a network / API failure comes back as ``error`` with
    ``update_available`` false."""
    now = time.time()
    with _cache_lock:
        if not force and _cache["result"] is not None \
                and now - _cache["at"] < _CHECK_TTL:
            return _cache["result"]
    cur = __version__
    out = {"current": cur, "latest": None, "update_available": False,
           "html_url": RELEASES_URL, "asset": None, "frozen": is_frozen(),
           "platform": platform_key(), "error": None}
    try:
        data = _http_json(_API_LATEST, timeout)
        tag = data.get("tag_name") or data.get("name")
        out["latest"] = tag
        out["html_url"] = data.get("html_url") or RELEASES_URL
        asset = _pick_asset(data.get("assets") or [])
        if asset:
            out["asset"] = {"name": asset.get("name"),
                            "url": asset.get("browser_download_url"),
                            "size": asset.get("size") or 0}
        out["update_available"] = bool(tag) and _is_newer(tag, cur)
    except Exception as e:
        out["error"] = f"{type(e).__name__}: {e}"
    with _cache_lock:
        _cache.update(at=now, result=out)
    return out


# --- applying an update ------------------------------------------------------

def _current_target():
    """What self-update replaces, as ``(kind, path)``:
      * ``('exe', <file>)``    -- Windows ``.exe`` / Linux ELF (single file)
      * ``('appdir', <.app>)`` -- macOS bundle directory
    Only meaningful when frozen."""
    exe = os.path.realpath(sys.executable)
    if sys.platform == "darwin":
        macos_dir = os.path.dirname(exe)        # .../Foo.app/Contents/MacOS
        contents = os.path.dirname(macos_dir)   # .../Foo.app/Contents
        bundle = os.path.dirname(contents)      # .../Foo.app
        if bundle.endswith(".app"):
            return "appdir", bundle
        return "exe", exe                       # --console-app bare binary
    return "exe", exe


def reap_leftovers():
    """Delete the ``<target>.old`` image a previous self-update left behind (it
    couldn't self-delete while it was the running process).  Best-effort; a
    no-op outside a frozen build."""
    if not is_frozen():
        return
    kind, path = _current_target()
    old = path + ".old"
    try:
        if kind == "appdir" and os.path.isdir(old):
            shutil.rmtree(old, ignore_errors=True)
        elif os.path.exists(old):
            os.remove(old)
    except OSError:
        pass


def _download(url, dest, on_progress=None, timeout=30):
    req = urllib.request.Request(url, headers={"User-Agent": _UA})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        total = int(r.headers.get("Content-Length") or 0)
        done = 0
        with open(dest, "wb") as f:
            while True:
                chunk = r.read(262144)
                if not chunk:
                    break
                f.write(chunk)
                done += len(chunk)
                if on_progress:
                    on_progress(done, total)
    return dest


def _popen_detached(argv):
    """Launch the replacement so it OUTLIVES this exiting process and (on
    Windows) gets its own console -- the original's console closes when the
    parent dies, and a console exe with no console is invisible."""
    kw = {"close_fds": True}
    if sys.platform == "win32":
        kw["creationflags"] = subprocess.CREATE_NEW_CONSOLE
    else:
        kw["start_new_session"] = True
    subprocess.Popen(argv, **kw)


def _current_bound_port():
    """The port the running web server actually bound (so the relaunch can
    reclaim the SAME one); None outside the web server."""
    try:
        from . import webserver
        return getattr(webserver, "BOUND_PORT", None)
    except Exception:
        return None


def _relaunch_argv(exe):
    """The new binary + the user's original args (package dir, --root ...), with
    three adjustments so the relaunch lands back in the SAME browser tab instead
    of leaving a dead one behind on a drifting port:

      * force ``--no-browser`` -- the page that triggered the update reloads
        itself once the new server is up (see index.html), so opening a second
        tab is just clutter that points at the old, now-exited instance;
      * pin ``--port`` to the port THIS server actually bound, so that reload
        reconnects at the same URL (without it the relaunch defaults to 8090 and
        may bind a different free port than the tab is open on);
      * pass ``--reclaim-port`` so the new instance briefly WAITS for that exact
        port to free up (the old one is mid-exit) rather than immediately
        walking forward to the next free port.
    The three flags are stripped from the inherited args first so they don't
    accumulate across successive self-updates."""
    args, skip = [], False
    for a in sys.argv[1:]:
        if skip:                                   # value of a prior "--port"
            skip = False
            continue
        if a == "--port":
            skip = True
            continue
        if a.startswith("--port=") or a in ("--no-browser", "--reclaim-port"):
            continue
        args.append(a)
    argv = [exe, *args, "--no-browser", "--reclaim-port"]
    port = _current_bound_port()
    if port:
        argv += ["--port", str(port)]
    return argv


def _replace_into(src, dst):
    """Move ``src`` onto ``dst``.  ``os.replace`` is atomic but only WITHIN one
    filesystem; the download now lives in %TEMP%, which may sit on a different
    volume than the exe (e.g. TEMP on C:, the exe on D:), where ``os.replace``
    raises EXDEV / WinError 17.  Fall back to copying onto a sibling of ``dst``
    (same volume) and atomically replacing from there, so ``dst`` is never seen
    half-written by a concurrent reader."""
    try:
        os.replace(src, dst)
    except OSError:
        tmp = dst + ".new"
        shutil.copyfile(src, tmp)
        os.replace(tmp, dst)            # same-dir -> atomic
        try:
            os.remove(src)
        except OSError:
            pass


def _swap_exe_and_relaunch(new_file, target):
    """Single-file swap (Windows / Linux): rename the running image aside, move
    the new one into its place, relaunch, return (the caller then exits)."""
    old = target + ".old"
    try:
        if os.path.exists(old):
            os.remove(old)              # stale .old from an interrupted update
    except OSError:
        pass
    os.replace(target, old)             # rename the running image (allowed)
    try:
        _replace_into(new_file, target)  # move the new binary into place
    except OSError:
        os.replace(old, target)         # roll the rename back on failure
        raise
    if sys.platform != "win32":
        os.chmod(target, 0o755)
    _popen_detached(_relaunch_argv(target))


def _swap_app_and_relaunch(new_zip, bundle):
    """macOS bundle swap: unzip the new ``.app`` next to the current one,
    restore exec bits, rename the running bundle aside, move the new one in,
    relaunch via ``open`` (the caller then exits)."""
    import zipfile
    workdir = bundle + ".new"
    shutil.rmtree(workdir, ignore_errors=True)
    os.makedirs(workdir, exist_ok=True)
    with zipfile.ZipFile(new_zip) as z:
        z.extractall(workdir)
    apps = [os.path.join(workdir, n) for n in os.listdir(workdir)
            if n.endswith(".app")]
    if not apps:
        shutil.rmtree(workdir, ignore_errors=True)
        raise RuntimeError("downloaded zip contains no .app bundle")
    new_app = apps[0]
    # zip extraction drops the unix exec bit -- restore it on the inner binaries
    macos = os.path.join(new_app, "Contents", "MacOS")
    if os.path.isdir(macos):
        for n in os.listdir(macos):
            try:
                os.chmod(os.path.join(macos, n), 0o755)
            except OSError:
                pass
    old = bundle + ".old"
    shutil.rmtree(old, ignore_errors=True)
    os.rename(bundle, old)
    try:
        shutil.move(new_app, bundle)
    except Exception:
        os.rename(old, bundle)          # roll back so the app still launches
        raise
    shutil.rmtree(workdir, ignore_errors=True)
    _popen_detached(["open", "-n", bundle])


def _set(**kw):
    with _update_lock:
        _update.update(**kw)


def update_status():
    with _update_lock:
        return dict(_update)


def _run_update(info):
    """Background worker: download the matching asset (reporting progress), swap
    the running binary, relaunch, and exit so the new instance takes over."""
    asset = info.get("asset") or {}
    url = asset.get("url")
    name = asset.get("name") or "sw2robot-web-update"
    kind, target = _current_target()
    # Download into a private, NON-synced temp dir -- NOT next to the exe.  The
    # exe often lives in a OneDrive/Dropbox-synced folder (e.g. the Desktop);
    # streaming a multi-100MB .part right beside a synced binary makes the sync
    # client fight us for the file the whole download.  We download to %TEMP%
    # and only move the finished file across at the very end (_replace_into
    # handles the cross-volume case).
    dldir = tempfile.mkdtemp(prefix="sw2robot-update-")
    part = os.path.join(dldir, name + ".part")
    final = os.path.join(dldir, name)
    try:
        _set(state="downloading", pct=0, error=None, version=info.get("latest"),
             downloaded=0, total=asset.get("size") or 0)

        def prog(done, total):
            tot = total or asset.get("size") or 0
            _set(pct=int(done * 100 / tot) if tot else 0,
                 downloaded=done, total=tot)

        _download(url, part, on_progress=prog)
        os.replace(part, final)
        _set(state="installing", pct=100)
        if kind == "appdir":
            _swap_app_and_relaunch(final, target)
        else:
            _swap_exe_and_relaunch(final, target)
        _set(state="restarting")
        # let the UI's status poll read 'restarting' once, then exit so the
        # relaunched instance can reclaim the (now-freed) port
        threading.Timer(1.0, lambda: os._exit(0)).start()
    except Exception as e:
        shutil.rmtree(dldir, ignore_errors=True)
        _set(state="error", error=f"{type(e).__name__}: {e}")


def start_update(timeout=6):
    """Kick off the download+swap in a background thread.  Returns a small dict
    the caller JSON-replies; refuses when not frozen, already running, or there
    is no applicable asset / no newer release."""
    if not is_frozen():
        return {"ok": False,
                "error": "self-update only applies to the packaged binary; "
                         "in a source checkout use git pull / pip install -U"}
    with _update_lock:
        if _update["state"] in ("downloading", "installing", "restarting"):
            return {"ok": False, "error": "an update is already in progress",
                    "state": _update["state"]}
    info = check_for_update(timeout=timeout, force=True)
    if info.get("error"):
        return {"ok": False, "error": info["error"]}
    if not info.get("update_available"):
        return {"ok": False, "error": "already up to date",
                "current": info.get("current")}
    if not (info.get("asset") or {}).get("url"):
        return {"ok": False,
                "error": f"no downloadable asset for {platform_key()} in "
                         f"release {info.get('latest')}"}
    threading.Thread(target=_run_update, args=(info,), daemon=True).start()
    return {"ok": True, "state": "downloading", "version": info.get("latest")}
