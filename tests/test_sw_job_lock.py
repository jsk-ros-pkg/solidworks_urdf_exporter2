"""The machine-wide SolidWorks job lock: one job at a time, re-entrant per thread.

Dispatch hands every client the same SLDWORKS.exe, so two concurrent jobs share
one process and trample each other's spawn decision and saved settings.  The
lock is a named Win32 mutex, so threads of one process contend exactly like
two processes do -- which is what these tests use."""
import sys
import threading

import pytest

from sw2robot.exporter import swcom

pytestmark = pytest.mark.skipif(
    sys.platform != "win32" or not swcom._HAVE_WIN32,
    reason="the job lock is a Win32 named mutex")


@pytest.fixture(autouse=True)
def private_mutex(monkeypatch):
    """Use a test-only name so a real sw2robot job on this PC is unaffected."""
    monkeypatch.setattr(swcom, "SW_JOB_MUTEX_NAME",
                        f"Local\\sw2robot-test-{id(object())}")


def _hold_in_thread():
    """Take the lock in a worker thread; returns (release_event, thread)."""
    held = threading.Event()
    release = threading.Event()

    def worker():
        with swcom.solidworks_job_lock():
            held.set()
            release.wait(10)

    t = threading.Thread(target=worker)
    t.start()
    assert held.wait(10)
    return release, t


def test_reentrant_on_the_same_thread():
    with swcom.solidworks_job_lock():
        with swcom.solidworks_job_lock(timeout=0):
            pass
        # still held after the inner block: another thread must see it busy
        result = {}

        def probe():
            try:
                with swcom.solidworks_job_lock(timeout=0):
                    result["got"] = True
            except swcom.SolidWorksBusy:
                result["got"] = False

        t = threading.Thread(target=probe)
        t.start()
        t.join(10)
        assert result == {"got": False}


def test_busy_times_out_and_reports_waiting():
    release, t = _hold_in_thread()
    try:
        msgs = []
        with pytest.raises(swcom.SolidWorksBusy):
            with swcom.solidworks_job_lock(progress=msgs.append, timeout=0.3,
                                           poll_s=0.05):
                pass
        assert msgs and "waiting for another sw2robot job" in msgs[0]
    finally:
        release.set()
        t.join(10)


def test_check_can_abandon_the_wait():
    release, t = _hold_in_thread()

    class Cancelled(Exception):
        pass

    def check():
        raise Cancelled()

    try:
        with pytest.raises(Cancelled):
            with swcom.solidworks_job_lock(check=check, poll_s=0.05):
                pass
    finally:
        release.set()
        t.join(10)


def test_acquired_once_the_holder_releases():
    release, t = _hold_in_thread()
    release.set()
    t.join(10)
    with swcom.solidworks_job_lock(timeout=0):
        pass
