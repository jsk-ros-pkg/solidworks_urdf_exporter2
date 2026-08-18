"""Choosing WHICH assembly configuration to extract, from the web editor.

An assembly configuration suppresses whole components and points instances at
other variants of a part, so it decides what the URDF contains.  Only the
`sw2urdf --configuration` CLI could pick one; the editor always took the file's
saved-active configuration, with no way to see -- let alone change -- that.
These cover the plumbing: /api/configurations lists the choices without loading
the document, and /api/extract carries the choice through to extract().
"""
import json
import socket
import threading
import urllib.error
import urllib.request

import pytest


@pytest.fixture
def server(tmp_path, monkeypatch):
    from sw2robot.editor import webserver

    s = socket.socket()
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    httpd, port = webserver._bind_free_port(webserver._Handler, port)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        yield f"http://127.0.0.1:{port}", webserver
    finally:
        httpd.shutdown()
        httpd.server_close()


def _get(base, path):
    try:
        with urllib.request.urlopen(base + path) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def test_configurations_lists_what_the_file_offers(server, tmp_path,
                                                   monkeypatch):
    base, webserver = server
    asm = tmp_path / "module.SLDASM"
    asm.write_bytes(b"not really CAD, the SolidWorks call is stubbed")
    monkeypatch.setattr(webserver, "_configurations_of",
                        lambda p: (["デフォルト",
                                    "alt_mount"], "warm"))
    code, r = _get(base, f"/api/configurations?path={asm}")
    assert code == 200
    assert r["configurations"] == ["デフォルト",
                                   "alt_mount"]
    assert r["source"] == "warm"


def test_configurations_rejects_a_non_cad_path(server, tmp_path):
    base, _ = server
    txt = tmp_path / "notes.txt"
    txt.write_text("hello", encoding="utf-8")
    code, r = _get(base, f"/api/configurations?path={txt}")
    assert code == 400
    assert "not a .sldasm/.sldprt file" in r["error"]


def test_no_solidworks_is_not_an_error(server, tmp_path, monkeypatch):
    # with nothing to ask, the picker must stay silent and the extract fall
    # back to the file's saved-active configuration -- not fail
    base, webserver = server
    asm = tmp_path / "module.SLDASM"
    asm.write_bytes(b"stub")
    monkeypatch.setattr(webserver, "_configurations_of",
                        lambda p: ([], "unavailable"))
    code, r = _get(base, f"/api/configurations?path={asm}")
    assert code == 200
    assert r == {"configurations": [], "source": "unavailable"}


def test_extract_forwards_the_chosen_configuration(server, tmp_path,
                                                   monkeypatch):
    base, webserver = server
    asm = tmp_path / "module.SLDASM"
    asm.write_bytes(b"stub")
    seen = {}
    done = threading.Event()

    def fake_run(sldasm, configuration=None):
        seen["path"], seen["configuration"] = sldasm, configuration
        with webserver._job_lock:
            webserver._job.update(running=False)
        webserver._prog_finish(result={"package": str(tmp_path)})
        done.set()

    monkeypatch.setattr(webserver, "_run_extract", fake_run)
    code, r = _get(base, f"/api/extract?path={asm}&configuration=alt_mount")
    assert code == 200 and r == {"started": True}
    assert done.wait(5)
    assert seen["configuration"] == "alt_mount"


def test_extract_without_a_choice_keeps_saved_active(server, tmp_path,
                                                     monkeypatch):
    # "" from the picker's default option must arrive as None (= don't switch),
    # never as an empty configuration name
    base, webserver = server
    asm = tmp_path / "module.SLDASM"
    asm.write_bytes(b"stub")
    seen = {}
    done = threading.Event()

    def fake_run(sldasm, configuration=None):
        seen["configuration"] = configuration
        with webserver._job_lock:
            webserver._job.update(running=False)
        webserver._prog_finish(result={"package": str(tmp_path)})
        done.set()

    monkeypatch.setattr(webserver, "_run_extract", fake_run)
    code, _ = _get(base, f"/api/extract?path={asm}&configuration=")
    assert code == 200
    assert done.wait(5)
    assert seen["configuration"] is None


def test_graph_state_records_the_configuration_it_came_from():
    # a package must be able to say WHICH variant it is, without SolidWorks
    from sw2robot.exporter.state import GraphState

    g = GraphState(robot_name="m", source_assembly="m.SLDASM",
                   configuration="alt_mount",
                   configurations=["デフォルト",
                                   "alt_mount"])
    assert json.loads(g.model_dump_json())["configuration"] == "alt_mount"
    # older extracts carry neither
    old = GraphState(robot_name="m", source_assembly="m.SLDASM")
    assert old.configuration is None and old.configurations == []


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
