"""SolidWorks connection robustness: warn + stop instead of hanging or wedging.

When SolidWorks cannot be reached -- not running, or (most often) launched but
unable to check out a license because the license server is down -- the
extractor must surface a clear ``SolidWorksUnavailable`` and never cache a dead
instance, so retries recover the moment the license is back.  The webserver's
launch is also time-bounded so an unreachable license server stops the job with
a warning rather than blocking forever.
"""

import time

import pytest

from sw2robot.exporter import swcom
from sw2robot.exporter.swcom import SolidWorks, SolidWorksUnavailable


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


def test_start_session_times_out_with_a_warning(monkeypatch):
    """A launch that never returns (license server hung) must stop promptly
    with SolidWorksUnavailable, not wait out the whole hang."""
    from sw2robot.editor import webserver

    class _Hang:
        def __init__(self, *a, **k):
            time.sleep(30)

    monkeypatch.setattr(swcom, "SolidWorks", _Hang)
    t0 = time.time()
    with pytest.raises(SolidWorksUnavailable):
        webserver._start_sw_session(lambda _m: None, timeout=1)
    assert time.time() - t0 < 5, "did not stop promptly on a hung launch"


def test_start_session_propagates_launch_failure(monkeypatch):
    from sw2robot.editor import webserver

    class _Fail:
        def __init__(self, *a, **k):
            raise SolidWorksUnavailable("no license")

    monkeypatch.setattr(swcom, "SolidWorks", _Fail)
    with pytest.raises(SolidWorksUnavailable):
        webserver._start_sw_session(lambda _m: None, timeout=5)
