"""SolidWorks connection robustness.

A dead/unresponsive SolidWorks COM proxy (an instance that can't check out a
license, or a warm session whose creating thread has ended ->
CO_E_OBJNOTCONNECTED) must be detected via a REAL call and dropped, never
reused, so the next extraction starts a fresh instance and recovers.
"""

from sw2robot.exporter.swcom import SolidWorks


class _AliveApp:
    Visible = True


class _DeadApp:
    @property
    def Visible(self):
        raise RuntimeError("rpc unavailable")

    def RevisionNumber(self):
        raise RuntimeError("rpc unavailable")


def _bare(app):
    sw = SolidWorks.__new__(SolidWorks)   # skip __init__ (needs live COM)
    sw.app = app
    return sw


def test_responds_true_for_a_live_app():
    assert _bare(_AliveApp())._responds() is True


def test_responds_false_for_a_dead_app():
    assert _bare(_DeadApp())._responds() is False
    assert _bare(None)._responds() is False


def test_warm_session_dropped_when_proxy_is_disconnected(monkeypatch):
    """A warm session whose COM proxy is disconnected (e.g. its creating thread
    ended -> CO_E_OBJNOTCONNECTED) must be detected as NOT alive, so the server
    starts a fresh instance instead of reusing a dead one."""
    from sw2robot.editor import webserver

    monkeypatch.setattr(webserver.time, "sleep", lambda *_: None)  # no retry wait
    monkeypatch.setitem(webserver._sw, "sess", _bare(_DeadApp()))
    got = webserver._warm_sw(lambda _m: None)
    assert got is None, "a disconnected warm session must not be reused"
    assert webserver._sw["sess"] is None
