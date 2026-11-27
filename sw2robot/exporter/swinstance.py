"""Start, list and stop dedicated SolidWorks processes, one per runner/agent.

Several agents on one PC must not share a SolidWorks: ``Dispatch`` hands every
client the same SLDWORKS.exe, and the MCP tools act on its *active* document,
so one agent's sketch lands in another's part.  The fix is one SolidWorks per
runner::

    pid=$(sw2robot-solidworks start)       # waits its turn, prints the pid
    export SOLIDWORKS_PID=$pid             # every sw2robot session uses it
    ...                                    # the runner's work
    sw2robot-solidworks stop "$pid"

``start`` is safe to run many times at once.  SolidWorks itself is not: three
SLDWORKS.exe launched at the same moment left ONE running (the others exited
before registering in the ROT), while three launched one after another all ran
(SolidWorks 2024, measured).  So starting is serialized by a session-wide mutex:
one caller launches and waits until its process is in the Running Object Table,
the rest print a waiting line every 10 s and go next.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time

from .swcom import (
    SW_PID_ENV,
    SolidWorksUnavailable,
    _require_typelibs,
    rot_solidworks,
    safe_call,
    safe_prop,
    sldworks_pids,
    solidworks_exe,
    solidworks_startup_lock,
)

START_TIMEOUT_S = 300.0      # a cold start measured 25-27 s; licence checks vary
STOP_TIMEOUT_S = 120.0
POLL_S = 1.0
NOTE_S = 10.0


def _say(msg):
    print(f"[sw2robot-solidworks] {msg}", file=sys.stderr, flush=True)


def _com_init():
    import pythoncom
    try:
        pythoncom.CoInitialize()
    except Exception:
        pass


def launch_solidworks(progress=_say, timeout=START_TIMEOUT_S):
    """Start a new SLDWORKS.exe and return its pid once it is in the ROT.

    Parameters
    ----------
    progress : callable
        Receives waiting / starting messages.
    timeout : float
        Seconds to wait for the new process to register; it is killed and
        SolidWorksUnavailable raised when they run out.

    Returns
    -------
    int
        Process id; connect with ``SolidWorks(pid=...)`` or ``$SOLIDWORKS_PID``.
    """
    _com_init()
    _require_typelibs()
    exe = solidworks_exe()
    flags = (subprocess.DETACHED_PROCESS
             | subprocess.CREATE_NEW_PROCESS_GROUP)   # outlives this command
    with solidworks_startup_lock(progress=progress, notify_s=NOTE_S):
        proc = subprocess.Popen(
            [exe], creationflags=flags, close_fds=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        progress(f"starting SolidWorks pid {proc.pid} ...")
        t0 = time.monotonic()
        next_note = t0 + NOTE_S
        while rot_solidworks(proc.pid) is None:
            code = proc.poll()
            if code is not None:
                raise SolidWorksUnavailable(
                    f"SolidWorks pid {proc.pid} exited (code {code}) before it "
                    f"finished starting -- usually another SLDWORKS.exe was "
                    f"started at the same moment outside sw2robot-solidworks; "
                    f"run start again")
            now = time.monotonic()
            if now - t0 >= timeout:
                proc.kill()
                raise SolidWorksUnavailable(
                    f"SolidWorks pid {proc.pid} did not register in the ROT "
                    f"within {timeout:.0f}s (a licence or startup dialog?); "
                    f"it was killed")
            if now >= next_note:
                progress(f"SolidWorks pid {proc.pid} still starting "
                         f"({int(now - t0)}s) ...")
                next_note = now + NOTE_S
            time.sleep(POLL_S)
        progress(f"SolidWorks pid {proc.pid} ready after "
                 f"{time.monotonic() - t0:.0f}s")
        return proc.pid


def list_solidworks():
    """``[(pid, in_rot)]`` for every running SLDWORKS.exe.  ``in_rot`` False
    means it was started by COM (Dispatch) and cannot be pinned by pid."""
    _com_init()
    pids = sldworks_pids()
    if pids is None:
        raise SolidWorksUnavailable("could not list SLDWORKS.exe processes")
    return [(pid, rot_solidworks(pid) is not None) for pid in sorted(pids)]


def _taskkill(pid):
    subprocess.run(["taskkill", "/PID", str(pid), "/F", "/T"],
                   capture_output=True, timeout=30)


def _dirty_titles(app):
    out = []
    for doc in safe_call(app, "GetDocuments") or []:
        if safe_call(doc, "GetSaveFlag"):
            out.append(str(safe_prop(doc, "GetTitle")))
    return out


def stop_solidworks(pid, force=False, progress=_say, timeout=STOP_TIMEOUT_S):
    """Quit SolidWorks process ``pid``.

    Refuses while a document has unsaved changes -- closing discards them --
    unless ``force``.  ``force`` also kills a process that will not quit or is
    not reachable through the ROT.
    """
    _com_init()
    if pid not in (sldworks_pids() or set()):
        raise SolidWorksUnavailable(f"no SolidWorks process with pid {pid}")
    app = rot_solidworks(pid)
    if app is None:
        if not force:
            raise SolidWorksUnavailable(
                f"SolidWorks pid {pid} is not in the ROT (started by COM?); "
                f"use --force to kill it")
        _taskkill(pid)
        progress(f"killed SolidWorks pid {pid}")
        return
    dirty = _dirty_titles(app)
    if dirty and not force:
        raise SolidWorksUnavailable(
            f"SolidWorks pid {pid} has unsaved changes in {dirty}; save them "
            f"or use --force to discard them")
    try:
        app.CloseAllDocuments(True)
        app.ExitApp()
    except Exception as e:
        progress(f"ExitApp failed ({e!r})")
    del app
    t0 = time.monotonic()
    while pid in (sldworks_pids() or set()):
        if time.monotonic() - t0 >= timeout:
            if not force:
                raise SolidWorksUnavailable(
                    f"SolidWorks pid {pid} did not exit within {timeout:.0f}s; "
                    f"use --force to kill it")
            _taskkill(pid)
            progress(f"SolidWorks pid {pid} did not exit; killed it")
            return
        time.sleep(POLL_S)
    progress(f"stopped SolidWorks pid {pid}")


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="sw2robot-solidworks",
        description="Start/stop a dedicated SolidWorks per runner.  Export the "
                    f"pid as ${SW_PID_ENV} so sw2robot uses that process.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_start = sub.add_parser(
        "start", help="start one (waits while another is starting) and print "
                      "its pid on stdout")
    p_start.add_argument("--timeout", type=float, default=START_TIMEOUT_S)
    p_start.add_argument("--env", action="store_true",
                         help=f"print {SW_PID_ENV}=<pid> instead of the bare pid")
    p_stop = sub.add_parser("stop", help="quit one by pid")
    p_stop.add_argument("pid", type=int)
    p_stop.add_argument("--force", action="store_true",
                        help="discard unsaved changes / kill if it will not quit")
    sub.add_parser("list", help="running SolidWorks processes and whether each "
                                "can be pinned by pid (in the ROT)")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "start":
            pid = launch_solidworks(timeout=args.timeout)
            print(f"{SW_PID_ENV}={pid}" if args.env else pid, flush=True)
        elif args.cmd == "stop":
            stop_solidworks(args.pid, force=args.force)
        else:
            for pid, in_rot in list_solidworks():
                print(f"{pid}\t{'rot' if in_rot else 'com-started'}")
    except SolidWorksUnavailable as e:
        _say(f"error: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
