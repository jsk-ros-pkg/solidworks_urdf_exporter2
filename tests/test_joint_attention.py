"""The joint-review worklist: why a joint came out that way, and whether to check it.

The classifier already explains itself -- `Joint.geo_note` -- but that reason
only ever reached a trailing comment in joints.yaml, so the editor showed the
user a type dropdown and no evidence.  These pin the server half: parsing the
reason back out, flagging the judgement calls, and acknowledging one.
"""
import json
import shutil
import threading
import urllib.request
from pathlib import Path

import pytest

from sw2robot.editor.webserver import _joint_notes

REPO = Path(__file__).resolve().parent.parent
FINGERTIP = REPO / "examples" / "fingertip"


def test_note_is_parsed_per_child_link():
    yml = (
        "joints:\n"
        "  - parent: a_1\n"
        "    child:  b_1\n"
        "    type:   fixed # mates: COINCIDENT | geo: fully constrained\n"
        "  - parent: b_1\n"
        "    child:  c_1\n"
        "    type:   revolute # mates: CONCENTRIC | geo: 1 DOF rotation\n"
    )
    notes = _joint_notes(yml)
    assert set(notes) == {"b_1", "c_1"}
    assert "fully constrained" in notes["b_1"]["note"]
    # neither of these is a judgement call, so neither is flagged
    assert notes["b_1"]["attention"] is None
    assert notes["c_1"]["attention"] is None


@pytest.mark.parametrize("note,expect", [
    ("geo: under-constrained (3 DOF) -> fixed; verify", "could not name"),
    ("fastener welded fixed", "NAME"),
    ("geo: fully constrained; some mates unmodelled", "cannot model"),
    ("geo: 1 DOF rotation (derived axis)", None),
    ("geo: fully constrained", None),
    ("reference axis 'knee' drawn in CAD", None),
])
def test_which_reasons_are_flagged(note, expect):
    yml = f"joints:\n  - parent: a_1\n    child:  b_1\n    type: fixed # {note}\n"
    got = _joint_notes(yml)["b_1"]["attention"]
    if expect is None:
        assert got is None, got
    else:
        assert got is not None and expect in got


def test_a_joint_with_no_comment_is_absent():
    assert _joint_notes("joints:\n  - parent: a\n    child:  b_1\n"
                        "    type:   fixed\n") == {}


# --- end to end through the server -----------------------------------------

def _free_port():
    import socket
    s = socket.socket()
    s.bind(("", 0))
    p = s.getsockname()[1]
    s.close()
    return p


@pytest.fixture
def server(tmp_path):
    if not (FINGERTIP / "graph.json").exists():
        pytest.skip("fingertip fixture not present")
    from sw2robot.editor import webserver
    from sw2robot.exporter.export import build

    pkg = tmp_path / "pkg"
    (pkg / "meshes").mkdir(parents=True)
    shutil.copy2(FINGERTIP / "graph.json", pkg / "graph.json")
    for f in (FINGERTIP / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, pkg / "meshes" / f.name)
    build(str(pkg))
    httpd, port = webserver._bind_free_port(webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        with urllib.request.urlopen(f"{base}/api/open?path={pkg}") as r:
            assert json.loads(r.read())["mode"] == "cad"
        yield base, pkg
    finally:
        httpd.shutdown()
        httpd.server_close()
        webserver._um["state"] = None


def _get(base, path):
    with urllib.request.urlopen(base + path) as r:
        return json.loads(r.read().decode())


def _post(base, path, body):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req) as r:
        return json.loads(r.read().decode())


SCREW = "screwlock_male_hard_jointbase_v4_1"
TIP = "fingertip_back_1"          # the fixture's one inferred revolute


def test_components_carries_the_reason(server):
    """Every joint reaches the browser with the reason it came out that way.

    This fixture happens to have no GUESSED joint -- both of its joints are
    confident calls -- so what is pinned here is the plumbing; which reasons
    raise a flag is covered by test_which_reasons_are_flagged above."""
    base, _pkg = server
    links = _get(base, "/api/components")["links"]
    assert "geo:" in (links[SCREW]["joint_note"] or "")
    assert links[SCREW]["joint_attention"] is None    # a confident call
    assert links[SCREW]["joint_reviewed"] is False


def test_reviewing_persists_without_a_rebuild(server):
    base, pkg = server
    assert _post(base, "/api/set_joint_reviewed",
                 {"link": SCREW, "reviewed": True})["reviewed"] is True
    links = _get(base, "/api/components")["links"]
    assert links[SCREW]["joint_reviewed"] is True
    assert links[SCREW]["joint_attention"] is None
    # the reason itself stays visible -- reviewing hides the badge, not the why
    assert "geo:" in links[SCREW]["joint_note"]
    assert "joint_reviewed:" in (pkg / "fingertip.joints.yaml").read_text(
        encoding="utf-8")

    _post(base, "/api/set_joint_reviewed", {"link": SCREW, "reviewed": False})
    assert _get(base, "/api/components")["links"][SCREW]["joint_reviewed"] \
        is False


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))


def test_note_and_review_survive_a_rename(server):
    """joints.yaml keys off the COMPONENT name; the browser sends the DISPLAY
    name.  Before the fix the read side looked the note up under the display
    name and the write side stored the display name, so renaming a link lost
    both the badge and any acknowledgement of it."""
    base, pkg = server
    assert _post(base, "/api/rename",
                 {"kind": "link", "old": SCREW, "new": "quick_connect"}
                 ).get("ok", True) is not False
    links = _get(base, "/api/components")["links"]
    assert "quick_connect" in links, sorted(links)
    assert "geo:" in (links["quick_connect"]["joint_note"] or "")

    _post(base, "/api/set_joint_reviewed",
          {"link": "quick_connect", "reviewed": True})
    links = _get(base, "/api/components")["links"]
    assert links["quick_connect"]["joint_reviewed"] is True
    # stored under the COMPONENT name, so a further rename keeps it
    assert SCREW in (pkg / "fingertip.joints.yaml").read_text(encoding="utf-8")


def test_the_note_survives_a_configured_rebuild(server):
    """The reason lives in the URDF, which every build rewrites.

    It is ALSO written into the joints.yaml template -- but only when the build
    has no --config, and the editor always rebuilds WITH one, so that copy
    freezes at the first build.  Before this, a configured build produced no
    reason at all (`_config_parent_map` dropped it), which is every build once a
    package has a joints.yaml.
    """
    from sw2robot.editor.webserver import _joint_notes_from_urdf
    from sw2robot.exporter.export import build

    base, pkg = server
    yml = pkg / "fingertip.joints.yaml"
    urdf_rel = "urdf/fingertip.urdf"
    before = _joint_notes_from_urdf(str(pkg), urdf_rel)
    assert before, "the built URDF carries no sw2robot-note comment"

    build(str(pkg), config_path=str(yml))          # a CONFIGURED rebuild
    after = _joint_notes_from_urdf(str(pkg), urdf_rel)
    assert set(after) == set(before), (sorted(before), sorted(after))
    assert all(after[k]["note"] for k in after)
    # and it reaches the API the browser reads
    links = _get(base, "/api/components")["links"]
    assert any(v.get("joint_note") for v in links.values())


def test_a_type_the_config_overrode_says_so(server):
    """Changing a type by hand is the user's decision, not a guess -- the note
    records both so the badge can stop nagging about it."""
    from sw2robot.editor.webserver import _joint_notes_from_urdf
    from sw2robot.exporter.export import build

    _base, pkg = server
    yml = pkg / "fingertip.joints.yaml"
    txt = yml.read_text(encoding="utf-8")
    lines, seen = [], False
    for line in txt.splitlines(True):
        if line.strip().startswith("child:") and TIP in line:
            seen = True
        elif seen and line.strip().startswith("type:"):
            line, seen = "    type:   fixed\n", False
        lines.append(line)
    yml.write_text("".join(lines), encoding="utf-8")
    build(str(pkg), config_path=str(yml))

    note = _joint_notes_from_urdf(str(pkg), "urdf/fingertip.urdf")[TIP]["note"]
    assert "config sets fixed" in note and "inference said revolute" in note
