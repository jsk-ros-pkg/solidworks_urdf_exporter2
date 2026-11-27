"""Spawned-SolidWorks records are per editor, so one editor never kills another's.

With a single shared ``_sw_spawned_pids.txt``, starting (or closing) a second
editor taskkilled the SolidWorks a first, still-running editor was extracting
in.  These tests pin the per-editor record: startup reaps only records whose
editor is gone, shutdown reaps only the editor's own record."""
import os
import subprocess
import sys

import pytest

from sw2robot.editor import webserver as ws


@pytest.fixture
def pid_dir(tmp_path, monkeypatch):
    """Point the records at ``tmp_path`` and capture taskkill instead of running
    it.  Returns the list of killed SolidWorks PIDs."""
    me = os.getpid()
    monkeypatch.setattr(ws, "_DATA_DIR", str(tmp_path))
    monkeypatch.setattr(ws, "_SW_PID_FILE",
                        str(tmp_path / f"_sw_spawned_pids.{me}.txt"))
    monkeypatch.setattr(ws, "_SW_PID_FILE_LEGACY",
                        str(tmp_path / "_sw_spawned_pids.txt"))
    # every recorded SolidWorks counts as alive
    monkeypatch.setattr(ws, "_sldworks_pids", lambda: {101, 202, 303, 404})
    killed = []

    def fake_run(cmd, **_kw):
        assert cmd[0] == "taskkill"
        killed.append(int(cmd[cmd.index("/PID") + 1]))

    monkeypatch.setattr(subprocess, "run", fake_run)
    return tmp_path, killed


def _dead_pid():
    """PID of a process that has exited."""
    p = subprocess.Popen([sys.executable, "-c", "pass"])
    p.wait()
    return p.pid


def _write(path, *pids):
    path.write_text("".join(f"{p}\n" for p in pids), encoding="utf-8")


def test_process_alive():
    assert ws._process_alive(os.getpid())
    assert not ws._process_alive(_dead_pid())


def test_startup_keeps_a_live_editors_solidworks(pid_dir):
    d, killed = pid_dir
    live_editor = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        live = d / f"_sw_spawned_pids.{live_editor.pid}.txt"
        dead = d / f"_sw_spawned_pids.{_dead_pid()}.txt"
        legacy = d / "_sw_spawned_pids.txt"
        _write(live, 101)
        _write(dead, 202)
        _write(legacy, 303)
        ws._reap_orphaned_sw()
        assert sorted(killed) == [202, 303]
        assert live.exists()
        assert not dead.exists() and not legacy.exists()
    finally:
        live_editor.kill()
        live_editor.wait()


def test_shutdown_reaps_only_its_own_record(pid_dir):
    d, killed = pid_dir
    ws._record_spawned_pid(404)
    other = d / f"_sw_spawned_pids.{_dead_pid()}.txt"
    _write(other, 101)
    ws._reap_spawned_sw()
    assert killed == [404]
    assert not os.path.exists(ws._SW_PID_FILE)
    assert other.exists()


def test_recorded_pid_that_is_no_longer_solidworks_is_not_killed(pid_dir):
    d, killed = pid_dir
    _write(d / f"_sw_spawned_pids.{_dead_pid()}.txt", 999)   # not in SW pids
    ws._reap_orphaned_sw()
    assert killed == []
