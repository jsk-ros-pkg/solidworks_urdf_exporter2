"""One SolidWorks per runner: $SOLIDWORKS_PID pinning and the serialized start.

Real SolidWorks is not needed: the pin is pure environment parsing plus lock
naming, and the launcher is driven with a stand-in executable that exits at
once -- the "another SLDWORKS.exe started at the same moment" failure."""
import sys
import threading

import pytest

from sw2robot.exporter import swcom

win32_only = pytest.mark.skipif(
    sys.platform != "win32" or not swcom._HAVE_WIN32,
    reason="named mutexes / SLDWORKS.exe are Win32 only")


@pytest.fixture(autouse=True)
def private_names(monkeypatch):
    tag = id(object())
    monkeypatch.setattr(swcom, "SW_JOB_MUTEX_NAME", f"Local\\sw2robot-tj-{tag}")
    monkeypatch.setattr(swcom, "SW_STARTUP_MUTEX_NAME",
                        f"Local\\sw2robot-ts-{tag}")
    monkeypatch.delenv(swcom.SW_PID_ENV, raising=False)


def test_dedicated_pid_parsing(monkeypatch):
    assert swcom.dedicated_pid() is None
    monkeypatch.setenv(swcom.SW_PID_ENV, " 4242 ")
    assert swcom.dedicated_pid() == 4242
    monkeypatch.setenv(swcom.SW_PID_ENV, "abc")
    with pytest.raises(ValueError):
        swcom.dedicated_pid()


@win32_only
def test_runners_with_their_own_solidworks_do_not_queue(monkeypatch):
    """The job lock is per pinned pid: runner A busy must not block runner B."""
    monkeypatch.setenv(swcom.SW_PID_ENV, "1111")
    held, release = threading.Event(), threading.Event()

    def runner_a():
        with swcom.solidworks_job_lock():
            held.set()
            release.wait(10)

    t = threading.Thread(target=runner_a)
    t.start()
    try:
        assert held.wait(10)
        monkeypatch.setenv(swcom.SW_PID_ENV, "2222")
        with swcom.solidworks_job_lock(timeout=0):
            pass                                   # B got in at once
        monkeypatch.setenv(swcom.SW_PID_ENV, "1111")
        with pytest.raises(swcom.SolidWorksBusy):  # but A's own is busy
            with swcom.solidworks_job_lock(timeout=0):
                pass
    finally:
        release.set()
        t.join(10)


@win32_only
def test_launch_reports_a_process_that_exits_while_starting(monkeypatch):
    from sw2robot.exporter import swinstance
    # sys.executable with stdin at DEVNULL exits immediately, like an
    # SLDWORKS.exe that lost the start-up race
    monkeypatch.setattr(swinstance, "solidworks_exe", lambda: sys.executable)
    monkeypatch.setattr(swinstance, "_require_typelibs", lambda: None)
    monkeypatch.setattr(swinstance, "rot_solidworks", lambda pid: None)
    msgs = []
    with pytest.raises(swcom.SolidWorksUnavailable, match="exited"):
        swinstance.launch_solidworks(progress=msgs.append, timeout=30)
    assert any("starting SolidWorks pid" in m for m in msgs)


@win32_only
def test_launch_waits_while_another_start_is_in_progress(monkeypatch):
    from sw2robot.exporter import swinstance
    monkeypatch.setattr(swinstance, "NOTE_S", 0.05)
    held, release = threading.Event(), threading.Event()

    def other_start():
        with swcom.solidworks_startup_lock():
            held.set()
            release.wait(10)

    t = threading.Thread(target=other_start)
    t.start()
    try:
        assert held.wait(10)
        msgs = []
        with pytest.raises(swcom.SolidWorksBusy):
            with swcom.solidworks_startup_lock(progress=msgs.append,
                                               timeout=0.3, poll_s=0.05):
                pass
        assert msgs and "another SolidWorks is still starting" in msgs[0]
    finally:
        release.set()
        t.join(10)
