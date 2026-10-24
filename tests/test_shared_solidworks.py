"""A private session must never quit a SolidWorks it did not start.

While a COM-launched SolidWorks is running, Dispatch("SldWorks.Application")
hands back THAT process instead of a new one.  Treating it as our own made
shutdown() close every document in it unsaved and ExitApp it.
"""
from types import SimpleNamespace

from sw2robot.exporter import swcom
from sw2robot.exporter.swcom import SolidWorks, _owns_new_process


def test_ownership_by_process_id():
    assert _owns_new_process(200, {100}) is True       # a new process
    assert _owns_new_process(100, {100}) is False      # the running one
    assert _owns_new_process(None, set()) is True      # nothing else running
    assert _owns_new_process(None, {100}) is False     # cannot tell -> shared
    assert _owns_new_process(200, None) is False       # cannot list -> shared


class _App:
    def __init__(self, pid):
        self.pid = pid
        self.Visible = True
        self.calls = []

    def GetProcessID(self):
        return self.pid

    def OpenDoc6(self, *a):
        pass

    def CloseAllDocuments(self, *a):
        self.calls.append("CloseAllDocuments")

    def ExitApp(self):
        self.calls.append("ExitApp")

    def CloseDoc(self, title):
        self.calls.append(("CloseDoc", title))


def _session(monkeypatch, pid, running):
    app = _App(pid)
    fake_client = SimpleNamespace(
        gencache=SimpleNamespace(EnsureDispatch=lambda _n: app),
        Dispatch=lambda _n: app)
    monkeypatch.setattr(swcom, "win32com",
                        SimpleNamespace(client=fake_client))
    monkeypatch.setattr(swcom, "pythoncom",
                        SimpleNamespace(CoInitialize=lambda: None))
    monkeypatch.setattr(swcom, "_require_typelibs", lambda: None)
    monkeypatch.setattr(swcom, "sldworks_pids", lambda: running)
    return SolidWorks(visible=False), app


def test_shared_process_is_left_running_and_visible(monkeypatch):
    sw, app = _session(monkeypatch, pid=100, running={100})
    assert app.Visible is True                 # not hidden from its owner
    sw.shutdown()
    assert "ExitApp" not in app.calls
    assert "CloseAllDocuments" not in app.calls


def test_own_new_process_is_shut_down(monkeypatch):
    sw, app = _session(monkeypatch, pid=200, running={100})
    assert app.Visible is False
    sw.shutdown()
    assert app.calls == ["CloseAllDocuments", "ExitApp"]
