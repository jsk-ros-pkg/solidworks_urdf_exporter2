"""Build-time sub-assembly expansion (_expand_subassemblies) on a synthetic
graph: a 'servo unit' sub-assembly whose horn turns inside, mounted on a
base plate.  Expansion must splice the children in, keep the internal
revolute, and re-attach the mounting mate to the owning child."""
import json
import socket
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest
from test_classify_geo import O, Z, coinc_planes, conc, dup

from sw2robot.exporter.model import build_model, from_graph
from sw2robot.exporter.state import (
    ComponentState,
    GraphState,
    MateEdge,
    MateGeo,
    SubGraph,
)


def _comp(name, link, xyz=(0, 0, 0), sub=False, path=None):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(name=name, link_name=link, part_path=path,
                          is_subassembly=sub,
                          world=[float(x) for x in w.flatten()])


def _edge(a, b, mates_, owners_of=None):
    geos = []
    for m in mates_:
        g = MateGeo(**m)
        if owners_of:
            g.owners = [owners_of.get(i, "") for i in range(len(m["points"]))]
        geos.append(g)
    ax = None
    for m in mates_:
        if m["type"] == "CONCENTRIC":
            ax = (m["points"][0], m["dirs"][0])
            break
    return MateEdge(a=a, b=b, types=[m["type"] for m in mates_],
                    axis_point=ax[0] if ax else None,
                    axis_dir=ax[1] if ax else None, mates=geos)


def make_graph():
    # sub-assembly internals (local frame): case (grounded) + horn, hinge
    sub = SubGraph(
        components=[_comp("case-1", "case_1"),
                    _comp("horn-1", "horn_1", xyz=(0, 0, 0.02))],
        edges=[_edge("case-1", "horn-1",
                     dup(conc(O, Z), coinc_planes([0, 0, 0.02], Z)))],
        ground=["case-1"])
    # top level: plate + servo instance shifted +0.1 in x
    inst = _comp("servo-1", "servo_1", xyz=(0.1, 0, 0), sub=True,
                 path="X:/fake/servo_unit.SLDASM")
    plate = _comp("plate-1", "plate_1")
    # mounting mate: bolt pair into the CASE (owners point inside the sub)
    mount = _edge("plate-1", "servo-1",
                  dup(conc([0.11, 0, 0], Z), conc([0.09, 0, 0], Z),
                      coinc_planes([0.1, 0, 0], Z)))
    for g in mount.mates:
        g.owners = ["plate-1", "servo-1/case-1"]
    return GraphState(robot_name="t", source_assembly="t.SLDASM",
                      components=[plate, inst], edges=[mount],
                      ground=["plate-1"],
                      subassemblies={"X:/fake/servo_unit.SLDASM": sub})


def _free_port():
    s = socket.socket()
    s.bind(("", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _post_json(base, path, body):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))


def _get_json(base, path):
    with urllib.request.urlopen(base + path) as response:
        return response.status, json.loads(response.read().decode("utf-8"))


def test_expand_splices_children():
    comps, adj, ground = from_graph(make_graph())
    names = {c.name for c in comps}
    assert names == {"plate-1", "servo-1/case-1", "servo-1/horn-1"}
    assert frozenset(("servo-1/case-1", "servo-1/horn-1")) in adj
    # mount re-attached to the owning child; the dead instance name must not
    # appear as an edge endpoint anywhere
    assert frozenset(("plate-1", "servo-1/case-1")) in adj
    assert not any("servo-1" in key for key in adj)


def test_expand_transforms_geometry():
    comps, adj, ground = from_graph(make_graph())
    horn = next(c for c in comps if c.name == "servo-1/horn-1")
    # local (0,0,0.02) + instance offset (0.1,0,0)
    assert np.allclose(horn.world[:3, 3], [0.1, 0, 0.02])
    rec = adj[frozenset(("servo-1/case-1", "servo-1/horn-1"))]
    p, d = rec["axis"]
    assert np.allclose(p, [0.1, 0, 0])      # hinge axis moved with instance
    assert np.allclose(d, [0, 0, 1])


def test_expanded_model_has_internal_revolute():
    model = build_model(make_graph())
    types = {j.name: j.jtype for j in model.joints}
    rev = [n for n, t in types.items() if t == "revolute"]
    assert len(rev) == 1 and "horn" in rev[0]
    fixed = [n for n, t in types.items() if t == "fixed"]
    assert len(fixed) == 1 and "case" in fixed[0]


def test_no_expand_override_keeps_instance():
    comps, adj, ground = from_graph(make_graph(), no_expand=["servo"])
    assert {c.name for c in comps} == {"plate-1", "servo-1"}


def test_no_expand_reuses_fully_expanded_directed_tree():
    cfg = {
        # This is the fully-expanded tree written by the rename/joints panel.
        # no_expand should collapse only the sub-assembly boundary, not discard
        # the saved tree and attach the preserved instance somewhere arbitrary.
        "base": "plate-1",
        "no_expand": ["servo"],
        "joints": [
            {"parent": "plate-1", "child": "servo-1/case-1",
             "type": "fixed"},
            {"parent": "servo-1/case-1", "child": "servo-1/horn-1",
             "type": "revolute"},
        ],
    }
    model = build_model(make_graph(), config=cfg)
    assert {c.name for c in model.components} == {"plate-1", "servo-1"}
    assert [(j.parent, j.child, j.jtype) for j in model.joints] == [
        ("plate_1", "servo_1", "fixed")
    ]


def test_no_expand_maps_expanded_base_hint_to_preserved_instance():
    cfg = {
        "base": "servo-1/case-1",
        "no_expand": ["servo"],
        "joints": [
            {"parent": "plate-1", "child": "servo-1/case-1",
             "type": "fixed"},
            {"parent": "servo-1/case-1", "child": "servo-1/horn-1",
             "type": "revolute"},
        ],
    }
    model = build_model(make_graph(), config=cfg)
    assert model.base_link == "servo_1"


def test_canonical_tree_payload_ignores_subassembly_modes():
    from sw2robot.editor.webserver import _canonical_tree_payload

    payload = _canonical_tree_payload(
        make_graph(), "base: plate-1\nno_expand:\n- servo\n")
    link_names = {x["link_name"] for x in payload["links"]}
    assert "servo_1" not in link_names
    assert {"servo_1__case_1", "servo_1__horn_1"} <= link_names

    sub = payload["subassemblies"][0]
    assert sub["name"] == "servo-1"
    assert set(sub["member_links"]) == {"servo_1__case_1",
                                        "servo_1__horn_1"}
    assert {j["child"] for j in sub["internal_joints"]} == {
        "servo_1__horn_1"}
    assert {j["child"] for j in sub["boundary_joints"]} == {
        "servo_1__case_1"}


def test_canonical_tree_payload_keeps_directed_tree_edits():
    from sw2robot.editor.webserver import _canonical_tree_payload

    txt = """
base: plate-1
no_expand:
- servo
joints:
  - parent: plate-1
    child:  servo-1/case-1
    type:   fixed
  - parent: servo-1/case-1
    child:  servo-1/horn-1
    type:   fixed
"""
    payload = _canonical_tree_payload(make_graph(), txt)
    sub = payload["subassemblies"][0]
    assert [(j["parent"], j["child"], j["type"])
            for j in sub["internal_joints"]] == [
        ("servo_1__case_1", "servo_1__horn_1", "fixed")
    ]


def test_collapse_preview_replaces_no_expand_subassembly_members():
    from sw2robot.editor.webserver import _collapse_preview_payload

    txt = "base: plate-1\nno_expand:\n- servo\n"
    payload = _collapse_preview_payload(make_graph(), txt)
    assert payload["canonical_counts"] == {"links": 3, "joints": 2}
    assert payload["preview_counts"] == {"links": 2, "joints": 1}
    assert {x["link_name"] for x in payload["links"]} == {
        "plate_1", "servo_1"}
    assert payload["collapsed_subassemblies"][0]["name"] == "servo-1"
    assert set(payload["collapsed_subassemblies"][0]["member_links"]) == {
        "servo_1__case_1", "servo_1__horn_1"}
    assert [(j["parent"], j["child"], j["type"])
            for j in payload["joints"]] == [
        ("plate_1", "servo_1", "fixed")
    ]
    assert [j["source_name"] for j in payload["dropped_internal_joints"]] == [
        "servo_1__case_1__servo_1__horn_1"
    ]
    assert [(r["link"], r["depth"], r["joint_type"], r["collapsed"])
            for r in payload["tree_rows"]] == [
        ("plate_1", 0, "root", False),
        ("servo_1", 1, "fixed", True),
    ]
    assert payload["tree_rows"][1]["member_links"] == [
        "servo_1__case_1", "servo_1__horn_1"]
    plan = payload["collapse_plan"]
    assert plan["ready_for_urdf"] is True
    assert plan["blocking_issue_count"] == 0
    assert len(plan["collapsed_subassemblies"]) == 1
    plan_sub = plan["collapsed_subassemblies"][0]
    assert {key: plan_sub[key] for key in (
        "name", "link", "member_links", "member_components",
        "selected_parent", "selected_origin_link",
    )} == {
        "name": "servo-1",
        "link": "servo_1",
        "member_links": ["servo_1__case_1", "servo_1__horn_1"],
        "member_components": ["servo-1/case-1", "servo-1/horn-1"],
        "selected_parent": "",
        "selected_origin_link": "",
    }
    assert plan_sub["selected_driver_joint"] == ""
    assert plan_sub["auto_driver_joint"] == \
        "servo_1__case_1__servo_1__horn_1"
    assert plan["link_replacements"] == [
        {"source_link": "servo_1__case_1", "collapsed_link": "servo_1"},
        {"source_link": "servo_1__horn_1", "collapsed_link": "servo_1"},
    ]
    assert [(j["source_joint"], j["decision"])
            for j in plan["dropped_joints"]] == [
        ("servo_1__case_1__servo_1__horn_1",
         "dropped_internal_to_collapsed_subassembly"),
    ]


def test_collapse_preview_uses_expanded_base_as_collapsed_origin():
    from sw2robot.editor.webserver import _collapse_preview_payload

    txt = """
base: servo-1/case-1
no_expand:
- servo
joints:
  - parent: plate-1
    child:  servo-1/case-1
    type:   fixed
  - parent: servo-1/case-1
    child:  servo-1/horn-1
    type:   revolute
"""
    payload = _collapse_preview_payload(make_graph(), txt)
    assert payload["base_link"] == "servo_1"

    collapsed = payload["collapsed_subassemblies"][0]
    assert collapsed["selected_origin_link"] == "servo_1__case_1"
    assert collapsed["selected_origin_source"] == "canonical_base"

    plan = payload["collapse_plan"]
    servo_link = next(l for l in plan["links"] if l["link"] == "servo_1")
    assert servo_link["selected_origin_link"] == "servo_1__case_1"
    assert servo_link["selected_origin_source"] == "canonical_base"


def test_collapse_preview_lists_subassembly_driver_joint_candidates():
    from sw2robot.editor.webserver import (
        _collapse_preview_payload,
        _collapsed_driver_joints,
        _set_collapsed_driver_joint_yaml,
    )

    graph = make_graph()
    txt = "base: plate-1\nno_expand:\n- servo\n"
    payload = _collapse_preview_payload(graph, txt)
    choices = payload["driver_joint_choices"]
    assert len(choices) == 1
    assert choices[0]["edge"] == "plate_1__servo_1"
    assert choices[0]["parent"] == "plate_1"
    assert choices[0]["child"] == "servo_1"
    assert choices[0]["auto_driver_joint"] == \
        "plate_1__servo_1"
    assert [(c["source_joint"], c["role"], c["type"],
             c["source_kind"])
            for c in choices[0]["candidates"]] == [
        ("plate_1__servo_1", "normal_exact_edge", "fixed", "normal"),
    ]

    txt = _set_collapsed_driver_joint_yaml(
        txt, "plate_1__servo_1", "plate_1__servo_1")
    assert _collapsed_driver_joints(txt) == {
        "plate_1__servo_1": "plate_1__servo_1"
    }
    payload = _collapse_preview_payload(graph, txt)
    assert payload["driver_joint_choices"][0]["selected_driver_joint"] == \
        "plate_1__servo_1"
    plan_joint = payload["collapse_plan"]["joints"][0]
    assert plan_joint["driver_source_joint"] == \
        "plate_1__servo_1"
    assert plan_joint["driver_type"] == "fixed"


def test_set_subassembly_driver_joint_validates_resets_and_saves(tmp_path):
    from sw2robot.editor import webserver

    graph = make_graph()
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    (pkg / "urdf").mkdir()
    graph.save(pkg / "graph.json")
    yml = pkg / "t.joints.yaml"
    yml.write_text("""
base: plate-1
no_expand:
- servo
subassembly_driver_joints:
  servo-1: removed_joint
""", encoding="utf-8")

    old_state = (
        webserver._Handler.pkg_dir,
        webserver._Handler.urdf_rel,
        webserver._Handler.robot_name,
        webserver._Handler.root_dir,
    )
    webserver._Handler.pkg_dir = str(pkg)
    webserver._Handler.urdf_rel = "urdf/t.urdf"
    webserver._Handler.robot_name = "t"
    webserver._Handler.root_dir = str(pkg)
    httpd, port = webserver._bind_free_port(
        webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{port}"
    try:
        code, payload = _get_json(base, "/api/collapse_preview")
        assert code == 200
        choice = payload["driver_joint_choices"][0]
        assert choice["selected_driver_joint"] == "removed_joint"
        stale = next(c for c in choice["candidates"]
                     if c["source_joint"] == "removed_joint")
        assert stale["role"] == "stale"

        code, payload = _post_json(
            base, "/api/set_subassembly_driver_joint",
            {"name": "servo-1", "source_joint": "not_a_candidate"})
        assert code == 400
        assert "not a driver joint candidate" in payload["error"]

        code, payload = _post_json(
            base, "/api/set_subassembly_driver_joint",
            {"name": "servo-1", "source_joint": "auto"})
        assert code == 200
        assert payload["source_joint"] == ""
        assert webserver._subassembly_driver_joints(
            yml.read_text(encoding="utf-8")) == {}

        selected = "servo_1__case_1__servo_1__horn_1"
        code, payload = _post_json(
            base, "/api/set_subassembly_driver_joint",
            {"name": "servo-1", "source_joint": selected})
        assert code == 200
        assert payload["source_joint"] == selected
        assert webserver._subassembly_driver_joints(
            yml.read_text(encoding="utf-8")) == {"servo-1": selected}
    finally:
        httpd.shutdown()
        httpd.server_close()
        (
            webserver._Handler.pkg_dir,
            webserver._Handler.urdf_rel,
            webserver._Handler.robot_name,
            webserver._Handler.root_dir,
        ) = old_state
def test_collapse_driver_joint_choices_are_unique_per_edge():
    from sw2robot.editor.webserver import _collapse_driver_joint_choices

    joints = [
        {"name": "parent__sub", "parent": "parent", "child": "sub",
         "type": "fixed", "source_name": "parent__sub__a",
         "decision": "kept_boundary"},
        {"name": "parent__sub", "parent": "parent", "child": "sub",
         "type": "fixed", "source_name": "parent__sub__b",
         "decision": "kept_boundary"},
    ]
    current = [
        {"name": "parent__sub", "parent": "parent", "child": "sub",
         "type": "revolute"},
    ]
    collapsed = [{
        "name": "sub-1",
        "link_name": "sub",
        "member_links": ["sub__a", "sub__b"],
    }]
    choices = _collapse_driver_joint_choices(
        joints, current, collapsed, {"sub__a": "sub", "sub__b": "sub"},
        {}, {})
    assert len(choices) == 1
    assert choices[0]["edge"] == "parent__sub"
    assert choices[0]["auto_driver_joint"] == "parent__sub"


def test_collapse_preview_keeps_expanded_override():
    from sw2robot.editor.webserver import _collapse_preview_payload

    txt = "base: plate-1\nexpand:\n- servo\n"
    payload = _collapse_preview_payload(make_graph(), txt)
    assert payload["collapsed_subassemblies"] == []
    assert payload["preview_counts"] == payload["canonical_counts"]
    assert [(r["link"], r["depth"], r["joint_type"], r["collapsed"])
            for r in payload["tree_rows"]] == [
        ("plate_1", 0, "root", False),
        ("servo_1__case_1", 1, "fixed", False),
        ("servo_1__horn_1", 2, "revolute", False),
    ]


def test_collapse_preview_applies_subassembly_parent_override():
    from sw2robot.editor.webserver import (
        _collapse_preview_payload,
        _set_subassembly_origin_link_yaml,
        _set_subassembly_parent_override_yaml,
        _tree_rows_from_collapse_plan,
    )

    graph = make_graph()
    graph.components.append(_comp("bracket-1", "bracket_1", xyz=(0, 0.1, 0)))
    txt = """
base: plate-1
no_expand:
- servo
joints:
  - parent: plate-1
    child:  servo-1/case-1
    type:   fixed
  - parent: bracket-1
    child:  servo-1/horn-1
    type:   fixed
  - parent: plate-1
    child:  bracket-1
    type:   fixed
"""
    payload = _collapse_preview_payload(graph, txt)
    choices = payload["parent_choices"]
    assert choices[0]["subassembly"] == "servo-1"
    assert [p["link"] for p in choices[0]["parents"]] == [
        "bracket_1", "plate_1"]
    assert {
        i["code"] for i in payload["validation"]["issues"]
    } >= {"multiple_parents", "multiple_boundary_parents",
          "disconnected_members"}

    txt = _set_subassembly_parent_override_yaml(
        txt, graph, "servo-1", "bracket_1")
    payload = _collapse_preview_payload(graph, txt)
    assert payload["parent_choices"][0]["selected_parent"] == "bracket_1"
    assert [(j["parent"], j["child"]) for j in payload["joints"]] == [
        ("bracket_1", "servo_1"),
        ("plate_1", "bracket_1"),
    ]
    assert "multiple_boundary_parents" not in {
        i["code"] for i in payload["validation"]["issues"]
    }
    plan = payload["collapse_plan"]
    assert plan["version"] == 1
    assert plan["base_link"] == payload["base_link"]
    assert plan["ready_for_urdf"] is False
    assert plan["blocking_issue_count"] == 1
    servo_link = next(l for l in plan["links"] if l["link"] == "servo_1")
    assert servo_link["kind"] == "collapsed_subassembly"
    assert servo_link["source_subassembly"] == "servo-1"
    assert servo_link["selected_parent"] == "bracket_1"
    assert servo_link["member_links"] == [
        "servo_1__case_1", "servo_1__horn_1"]
    assert [(j["parent"], j["child"], j["source_joint"], j["decision"])
            for j in plan["joints"]] == [
        ("bracket_1", "servo_1", "bracket_1__servo_1__horn_1",
         "kept_boundary"),
        ("plate_1", "bracket_1", "plate_1__bracket_1",
         "kept_expanded"),
    ]
    assert [(j["source_joint"], j["decision"])
            for j in plan["dropped_joints"]] == [
        ("plate_1__servo_1__case_1", "dropped_parent_override")
    ]
    assert payload["tree_rows"] == _tree_rows_from_collapse_plan(plan)
    assert payload["group_choices"][0]["subassembly"] == "servo-1"
    assert [g["origin_link"] for g in payload["group_choices"][0]["groups"]] == [
        "servo_1__case_1", "servo_1__horn_1"]
    assert "disconnected_members" in {
        i["code"] for i in payload["validation"]["issues"]
    }

    txt = _set_subassembly_origin_link_yaml(
        txt, graph, "servo-1", "servo_1__horn_1")
    payload = _collapse_preview_payload(graph, txt)
    assert payload["group_choices"][0]["selected_origin_link"] == \
        "servo_1__horn_1"
    anchored = next(
        i for i in payload["validation"]["issues"]
        if i["code"] == "disconnected_members")
    assert anchored["severity"] == "info"
    assert anchored["origin_link"] == "servo_1__horn_1"
    plan = payload["collapse_plan"]
    assert plan["ready_for_urdf"] is True
    assert plan["blocking_issue_count"] == 0


def test_collapse_preview_reports_stale_subassembly_origin_link():
    from sw2robot.editor.webserver import _collapse_preview_payload

    txt = """
base: plate-1
no_expand:
- servo
subassembly_origin_links:
  servo-1: missing_link
"""
    payload = _collapse_preview_payload(make_graph(), txt)
    choices = payload["group_choices"]
    assert choices[0]["subassembly"] == "servo-1"
    assert choices[0]["selected_origin_link"] == ""
    assert choices[0]["stale_origin_link"] == "missing_link"
    assert payload["collapsed_subassemblies"][0]["selected_origin_link"] == ""
    assert payload["collapsed_subassemblies"][0]["stale_origin_link"] == \
        "missing_link"
    issue = next(
        i for i in payload["validation"]["issues"]
        if i["code"] == "invalid_origin_link")
    assert issue["origin_link"] == "missing_link"


def test_set_subassembly_origin_link_rejects_non_candidate(tmp_path):
    from sw2robot.editor import webserver

    graph = make_graph()
    graph.components.append(_comp("bracket-1", "bracket_1", xyz=(0, 0.1, 0)))
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    (pkg / "urdf").mkdir()
    graph.save(pkg / "graph.json")
    yml = pkg / "t.joints.yaml"
    yml.write_text("""
base: plate-1
no_expand:
- servo
joints:
  - parent: plate-1
    child:  servo-1/case-1
    type:   fixed
  - parent: bracket-1
    child:  servo-1/horn-1
    type:   fixed
  - parent: plate-1
    child:  bracket-1
    type:   fixed
""", encoding="utf-8")

    old_state = (
        webserver._Handler.pkg_dir,
        webserver._Handler.urdf_rel,
        webserver._Handler.robot_name,
        webserver._Handler.root_dir,
    )
    webserver._Handler.pkg_dir = str(pkg)
    webserver._Handler.urdf_rel = "urdf/t.urdf"
    webserver._Handler.robot_name = "t"
    webserver._Handler.root_dir = str(pkg)
    httpd, port = webserver._bind_free_port(
        webserver._Handler, _free_port())
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        code, payload = _post_json(
            f"http://127.0.0.1:{port}",
            "/api/set_subassembly_origin_link",
            {"name": "servo-1", "origin_link": "not_a_member"})
        assert code == 400
        assert "not a member origin link candidate" in payload["error"]
        assert "subassembly_origin_links" not in yml.read_text(
            encoding="utf-8")
    finally:
        httpd.shutdown()
        httpd.server_close()
        (
            webserver._Handler.pkg_dir,
            webserver._Handler.urdf_rel,
            webserver._Handler.robot_name,
            webserver._Handler.root_dir,
        ) = old_state


def test_collapse_preview_ignores_stale_subassembly_parent_override():
    from sw2robot.editor.webserver import _collapse_preview_payload

    graph = make_graph()
    graph.components.append(_comp("bracket-1", "bracket_1", xyz=(0, 0.1, 0)))
    txt = """
base: plate-1
no_expand:
- servo
subassembly_parent_overrides:
  servo-1: missing_parent
joints:
  - parent: plate-1
    child:  servo-1/case-1
    type:   fixed
  - parent: bracket-1
    child:  servo-1/horn-1
    type:   fixed
  - parent: plate-1
    child:  bracket-1
    type:   fixed
"""
    payload = _collapse_preview_payload(graph, txt)
    assert payload["parent_choices"][0]["selected_parent"] == ""
    assert payload["collapsed_subassemblies"][0]["selected_parent"] == ""
    assert {
        (j["parent"], j["child"]) for j in payload["joints"]
    } >= {
        ("plate_1", "servo_1"),
        ("bracket_1", "servo_1"),
    }
    assert "multiple_boundary_parents" in {
        i["code"] for i in payload["validation"]["issues"]
    }
    plan = payload["collapse_plan"]
    assert plan["ready_for_urdf"] is False
    servo_link = next(l for l in plan["links"] if l["link"] == "servo_1")
    assert servo_link["selected_origin_link"] == ""


def test_collapse_preview_can_reset_subassembly_parent_override_to_auto():
    from sw2robot.editor.webserver import (
        _collapse_preview_payload,
        _set_subassembly_parent_override_yaml,
        _subassembly_parent_overrides,
    )

    graph = make_graph()
    graph.components.append(_comp("bracket-1", "bracket_1", xyz=(0, 0.1, 0)))
    txt = """
base: plate-1
no_expand:
- servo
joints:
  - parent: plate-1
    child:  servo-1/case-1
    type:   fixed
  - parent: bracket-1
    child:  servo-1/horn-1
    type:   fixed
  - parent: plate-1
    child:  bracket-1
    type:   fixed
"""
    txt = _set_subassembly_parent_override_yaml(
        txt, graph, "servo-1", "bracket_1")
    txt = _set_subassembly_parent_override_yaml(txt, graph, "servo-1", "")
    assert _subassembly_parent_overrides(txt) == {}

    payload = _collapse_preview_payload(graph, txt)
    assert payload["parent_choices"][0]["selected_parent"] == ""
    assert {
        (j["parent"], j["child"]) for j in payload["joints"]
    } >= {
        ("plate_1", "servo_1"),
        ("bracket_1", "servo_1"),
    }


def test_validate_collapsed_tree_reports_multiple_parents_and_cycles():
    from sw2robot.editor.webserver import _validate_collapsed_tree

    links = [{"link_name": n} for n in ("base", "alt", "arm")]
    joints = [
        {"name": "base__arm", "parent": "base", "child": "arm",
         "type": "fixed", "source_name": "j1"},
        {"name": "alt__arm", "parent": "alt", "child": "arm",
         "type": "fixed", "source_name": "j2"},
        {"name": "arm__base", "parent": "arm", "child": "base",
         "type": "fixed", "source_name": "j3"},
    ]
    payload = _validate_collapsed_tree("base", links, joints, [])
    codes = {i["code"] for i in payload["issues"]}
    assert {"multiple_parents", "cycle"} <= codes
    multi = next(i for i in payload["issues"]
                 if i["code"] == "multiple_parents")
    assert multi["parents"] == ["alt", "base"]
    assert multi["source_joints"] == ["j1", "j2"]
    cycle = next(i for i in payload["issues"] if i["code"] == "cycle")
    assert cycle["links"] == ["base", "arm", "base"]
    assert cycle["source_joints"] == ["j1", "j3"]
    assert cycle["candidates"] == [
        {"joint": "base__arm", "source_joint": "j1",
         "parent": "base", "child": "arm"},
        {"joint": "arm__base", "source_joint": "j3",
         "parent": "arm", "child": "base"},
    ]
    assert payload["ok"] is False


def test_collapse_preview_applies_cycle_break_before_choices(monkeypatch):
    from sw2robot.editor import webserver

    links = [{"link_name": n} for n in ("base", "arm")]
    joints = [
        {"name": "base__arm", "parent": "base", "child": "arm",
         "type": "fixed", "source_name": "j1"},
        {"name": "arm__base", "parent": "arm", "child": "base",
         "type": "fixed", "source_name": "j2"},
    ]
    monkeypatch.setattr(webserver, "_canonical_tree_payload", lambda *_: {
        "base_link": "base",
        "links": links,
        "joints": joints,
        "subassemblies": [],
    })
    monkeypatch.setattr(webserver, "_subassemblies_payload", lambda *_: {
        "subassemblies": [],
    })

    unresolved = webserver._collapse_preview_payload(object(), "")
    assert any(i["code"] == "cycle"
               for i in unresolved["validation"]["issues"])
    assert unresolved["collapse_plan"]["ready_for_urdf"] is False

    payload = webserver._collapse_preview_payload(
        object(), "subassembly_cycle_break_joints:\n- arm__base\n")

    assert [j["source_name"] for j in payload["joints"]] == ["base__arm"]
    assert not any(i["code"] == "cycle"
                   for i in payload["validation"]["issues"])
    assert payload["cycle_break_choices"] == [{
        "links": [],
        "joints": [],
        "source_joints": ["arm__base"],
        "selected_source_joint": "arm__base",
        "stale": True,
        "candidates": [{
            "joint": "arm__base",
            "source_joint": "arm__base",
            "parent": "",
            "child": "",
        }],
    }]
    assert payload["collapse_plan"]["ready_for_urdf"] is True
    assert [(j["source_joint"], j["decision"])
            for j in payload["collapse_plan"]["dropped_joints"]] == [
        ("arm__base", "dropped_cycle_break"),
    ]


def test_collapse_plan_rejects_unreachable_second_root(monkeypatch):
    from sw2robot.editor import webserver

    monkeypatch.setattr(webserver, "_canonical_tree_payload", lambda *_: {
        "base_link": "base",
        "links": [{"name": n, "link_name": n}
                  for n in ("base", "arm", "orphan")],
        "joints": [{
            "name": "base__arm", "parent": "base", "child": "arm",
            "type": "fixed",
        }],
        "subassemblies": [],
    })
    monkeypatch.setattr(webserver, "_subassemblies_payload", lambda *_: {
        "subassemblies": [],
    })

    payload = webserver._collapse_preview_payload(object(), "")

    issues = {i["code"]: i for i in payload["validation"]["issues"]}
    assert issues["invalid_roots"]["roots"] == ["base", "orphan"]
    assert issues["unreachable_links"]["links"] == ["orphan"]
    assert payload["collapse_plan"]["ready_for_urdf"] is False


def test_subassembly_cycle_break_yaml_adds_and_resets_source_joint():
    from sw2robot.editor.webserver import (
        _set_subassembly_cycle_break_joint_yaml,
        _subassembly_cycle_break_joints,
    )

    txt = _set_subassembly_cycle_break_joint_yaml("", "j2", True)
    assert _subassembly_cycle_break_joints(txt) == {"j2"}

    txt = _set_subassembly_cycle_break_joint_yaml(txt, "", False, "j2")
    assert _subassembly_cycle_break_joints(txt) == set()


def test_directed_cycle_reports_self_loop():
    from sw2robot.editor.webserver import _directed_cycle_reports

    reports = _directed_cycle_reports(
        "base", [{"link_name": "base"}], [{
            "name": "base__base", "parent": "base", "child": "base",
            "type": "fixed", "source_name": "self_joint",
        }])

    assert len(reports) == 1
    assert reports[0]["links"] == ["base", "base"]
    assert reports[0]["source_joints"] == ["self_joint"]


def test_cycle_break_shared_candidate_resolves_two_cycles():
    from sw2robot.editor.webserver import _directed_cycle_reports

    links = [{"link_name": n} for n in ("base", "arm", "tip")]
    joints = [
        {"name": "base__arm", "parent": "base", "child": "arm",
         "source_name": "shared"},
        {"name": "arm__base", "parent": "arm", "child": "base",
         "source_name": "short_return"},
        {"name": "arm__tip", "parent": "arm", "child": "tip",
         "source_name": "to_tip"},
        {"name": "tip__base", "parent": "tip", "child": "base",
         "source_name": "long_return"},
    ]

    reports = _directed_cycle_reports("base", links, joints)
    assert len(reports) == 2
    assert all("shared" in r["source_joints"] for r in reports)

    kept = [j for j in joints if j["source_name"] != "shared"]
    assert _directed_cycle_reports("base", links, kept) == []
def test_collapsed_preview_urdf_lumps_member_visuals():
    import xml.etree.ElementTree as ET

    from sw2robot.editor.webserver import _collapsed_preview_urdf_text

    urdf = """<?xml version="1.0"?>
<robot name="demo">
  <link name="base"/>
  <link name="sub_a">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/a.stl"/></geometry></visual>
  </link>
  <link name="sub_b">
    <visual><origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/b.stl"/></geometry></visual>
  </link>
  <link name="outside"/>
  <joint name="base__sub_a" type="fixed">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="sub_a"/>
  </joint>
  <joint name="sub_a__sub_b" type="revolute">
    <origin xyz="0 2 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <parent link="sub_a"/><child link="sub_b"/>
    <limit lower="-0.5" upper="0.5" effort="2" velocity="3"/>
  </joint>
  <joint name="sub_b__outside" type="fixed">
    <origin xyz="0 0 3" rpy="0 0 0"/>
    <parent link="sub_b"/><child link="outside"/>
  </joint>
</robot>
"""
    plan = {
        "version": 1,
        "base_link": "base",
        "ready_for_urdf": True,
        "links": [
            {"link": "base", "name": "base", "kind": "expanded_link"},
            {"link": "sub", "name": "sub", "kind": "collapsed_subassembly",
             "member_links": ["sub_a", "sub_b"],
             "selected_origin_link": "sub_a"},
            {"link": "outside", "name": "outside", "kind": "expanded_link"},
        ],
        "joints": [
            {"name": "base__sub", "parent": "base", "child": "sub",
             "type": "fixed", "source_joint": "base__sub_a",
             "driver_source_joint": "sub_a__sub_b",
             "driver_type": "revolute",
             "decision": "kept_boundary"},
            {"name": "sub__outside", "parent": "sub", "child": "outside",
             "type": "fixed", "source_joint": "sub_b__outside",
             "decision": "kept_boundary"},
        ],
        "dropped_joints": [
            {"name": "sub_a__sub_b", "parent": "sub", "child": "sub",
             "type": "fixed", "source_joint": "sub_a__sub_b",
             "decision": "dropped_internal_to_collapsed_subassembly"},
        ],
        "collapsed_subassemblies": [],
        "link_replacements": [
            {"source_link": "sub_a", "collapsed_link": "sub"},
            {"source_link": "sub_b", "collapsed_link": "sub"},
        ],
    }
    root = ET.fromstring(_collapsed_preview_urdf_text(urdf, plan))

    assert {x.get("name") for x in root.findall("link")} == {
        "base", "sub", "outside"}
    assert {x.get("name") for x in root.findall("joint")} == {
        "base__sub", "sub__outside"}
    sub = next(x for x in root.findall("link") if x.get("name") == "sub")
    visuals = sub.findall("visual")
    assert len(visuals) == 2
    meshes = [v.find(".//mesh").get("filename") for v in visuals]
    assert meshes == ["../meshes/a.stl", "../meshes/b.stl"]
    assert visuals[0].find("origin").get("xyz") == "0 0 0"
    assert visuals[1].find("origin").get("xyz") == "0 2 0.5"
    sub_out = next(j for j in root.findall("joint")
                   if j.get("name") == "sub__outside")
    assert sub_out.find("parent").get("link") == "sub"
    assert sub_out.find("child").get("link") == "outside"
    assert sub_out.find("origin").get("xyz") == "0 2 3"
    base_sub = next(j for j in root.findall("joint")
                    if j.get("name") == "base__sub")
    assert base_sub.get("type") == "revolute"
    assert base_sub.find("origin").get("xyz") == "1 0 0"
    assert base_sub.find("axis").get("xyz") == "0 0 1"
    assert base_sub.find("limit").get("lower") == "-0.5"


def test_collapsed_preview_urdf_maps_canonical_base_origin_to_urdf_root():
    import xml.etree.ElementTree as ET

    from sw2robot.editor.webserver import _collapsed_preview_urdf_text

    expanded_urdf = """<?xml version="1.0"?>
<robot name="demo">
  <link name="base_link">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/base.stl"/></geometry></visual>
  </link>
  <link name="sub_b">
    <visual><origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/b.stl"/></geometry></visual>
  </link>
  <joint name="base_link__sub_b" type="fixed">
    <origin xyz="0 2 0" rpy="0 0 0"/>
    <parent link="base_link"/><child link="sub_b"/>
  </joint>
</robot>
"""
    plan = {
        "version": 1,
        "base_link": "sub",
        "ready_for_urdf": True,
        "links": [
            {"link": "sub", "name": "sub", "kind": "collapsed_subassembly",
             "member_links": ["sub_a", "sub_b"],
             "selected_origin_link": "sub_a",
             "selected_origin_source": "canonical_base"},
        ],
        "joints": [],
        "dropped_joints": [],
        "collapsed_subassemblies": [],
        "link_replacements": [
            {"source_link": "sub_a", "collapsed_link": "sub"},
            {"source_link": "sub_b", "collapsed_link": "sub"},
        ],
    }

    root = ET.fromstring(_collapsed_preview_urdf_text(expanded_urdf, plan))
    link = next(x for x in root.findall("link")
                if x.get("name") == "base_link")
    visuals = link.findall("visual")
    assert [v.find(".//mesh").get("filename") for v in visuals] == [
        "../meshes/base.stl", "../meshes/b.stl"]
    assert visuals[0].find("origin").get("xyz") == "0 0 0"
    assert visuals[1].find("origin").get("xyz") == "0 2 0.5"


def test_collapsed_preview_urdf_uses_normal_driver_joint_source():
    import xml.etree.ElementTree as ET

    from sw2robot.editor.webserver import _collapsed_preview_urdf_text

    expanded_urdf = """<?xml version="1.0"?>
<robot name="demo">
  <link name="base"/>
  <link name="sub_a">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/a.stl"/></geometry></visual>
  </link>
  <link name="sub_b">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/b.stl"/></geometry></visual>
  </link>
  <joint name="base__sub_a" type="fixed">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="sub_a"/>
  </joint>
  <joint name="sub_a__sub_b" type="fixed">
    <origin xyz="0 2 0" rpy="0 0 0"/>
    <parent link="sub_a"/><child link="sub_b"/>
  </joint>
</robot>
"""
    normal_urdf = """<?xml version="1.0"?>
<robot name="demo">
  <link name="base"/>
  <link name="sub"/>
  <joint name="base__sub" type="revolute">
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <parent link="base"/><child link="sub"/>
    <limit lower="-1" upper="1" effort="4" velocity="5"/>
  </joint>
</robot>
"""
    plan = {
        "version": 1,
        "base_link": "base",
        "ready_for_urdf": True,
        "links": [
            {"link": "base", "name": "base", "kind": "expanded_link"},
            {"link": "sub", "name": "sub", "kind": "collapsed_subassembly",
             "member_links": ["sub_a", "sub_b"],
             "selected_origin_link": "sub_a"},
        ],
        "joints": [
            {"name": "base__sub", "parent": "base", "child": "sub",
             "type": "fixed", "source_joint": "base__sub_a",
             "driver_source_joint": "base__sub",
             "driver_type": "revolute",
             "decision": "kept_boundary"},
        ],
        "dropped_joints": [],
        "collapsed_subassemblies": [],
        "link_replacements": [
            {"source_link": "sub_a", "collapsed_link": "sub"},
            {"source_link": "sub_b", "collapsed_link": "sub"},
        ],
    }
    root = ET.fromstring(_collapsed_preview_urdf_text(
        expanded_urdf, plan, driver_urdf_text=normal_urdf))
    sub = next(x for x in root.findall("link") if x.get("name") == "sub")
    assert len(sub.findall("visual")) == 2
    joint = next(x for x in root.findall("joint")
                 if x.get("name") == "base__sub")
    assert joint.get("type") == "revolute"
    assert joint.find("axis").get("xyz") == "1 0 0"
    assert joint.find("limit").get("effort") == "4"


def test_collapse_preview_http_cache_reuses_identical_input(tmp_path, monkeypatch):
    from sw2robot.editor import webserver

    pkg = tmp_path / "pkg"
    pkg.mkdir()
    (pkg / "graph.json").write_text("{}", encoding="utf-8")
    yml = pkg / "robot.joints.yaml"
    yml.write_text("base: a\n", encoding="utf-8")
    calls = {"n": 0}

    def fake_payload(_graph, txt, current_joints=None):
        calls["n"] += 1
        assert current_joints is None
        return {"txt": txt, "calls": calls["n"]}

    monkeypatch.setattr(webserver, "_collapse_preview_payload", fake_payload)
    webserver._COLLAPSE_PREVIEW_CACHE.clear()

    first = webserver._collapse_preview_payload_cached(
        str(pkg), object(), str(yml), yml.read_text(encoding="utf-8"))
    second = webserver._collapse_preview_payload_cached(
        str(pkg), object(), str(yml), yml.read_text(encoding="utf-8"))

    assert calls["n"] == 1
    assert first == second == {"txt": "base: a\n", "calls": 1}

    yml.write_text("base: b\n", encoding="utf-8")
    third = webserver._collapse_preview_payload_cached(
        str(pkg), object(), str(yml), yml.read_text(encoding="utf-8"))
    assert calls["n"] == 2
    assert third == {"txt": "base: b\n", "calls": 2}


def test_collapse_preview_cache_reads_normal_urdf_joints(tmp_path, monkeypatch):
    from sw2robot.editor import webserver

    pkg = tmp_path / "pkg"
    urdf_dir = pkg / "urdf"
    urdf_dir.mkdir(parents=True)
    (pkg / "graph.json").write_text("{}", encoding="utf-8")
    yml = pkg / "robot.joints.yaml"
    yml.write_text("base: base\n", encoding="utf-8")
    urdf = urdf_dir / "robot.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="r">
  <link name="base"/>
  <link name="sub"/>
  <joint name="base__sub" type="revolute">
    <parent link="base"/><child link="sub"/>
  </joint>
</robot>
""", encoding="utf-8")

    seen = {}

    def fake_payload(_graph, txt, current_joints=None):
        seen["txt"] = txt
        seen["current_joints"] = current_joints
        return {"ok": True}

    monkeypatch.setattr(webserver, "_collapse_preview_payload", fake_payload)
    webserver._COLLAPSE_PREVIEW_CACHE.clear()

    payload = webserver._collapse_preview_payload_cached(
        str(pkg), object(), str(yml), yml.read_text(encoding="utf-8"),
        "urdf/robot.urdf")

    assert payload == {"ok": True}
    assert seen["current_joints"] == [{
        "name": "base__sub",
        "parent": "base",
        "child": "sub",
        "type": "revolute",
    }]


def test_validate_collapsed_tree_reports_disconnected_members():
    from sw2robot.editor.webserver import _validate_collapsed_tree

    collapsed = [{
        "name": "sub-1",
        "link_name": "sub_1",
        "member_links": ["sub_1__a", "sub_1__b"],
        "internal_joints": [],
        "boundary_joints": [],
    }]
    payload = _validate_collapsed_tree(
        "base",
        [{"link_name": "base"}, {"link_name": "sub_1"}],
        [{"name": "base__sub_1", "parent": "base", "child": "sub_1",
          "type": "fixed"}],
        collapsed)
    issue = payload["issues"][0]
    assert issue["code"] == "disconnected_members"
    assert issue["subassembly"] == "sub-1"
    assert issue["components"] == [["sub_1__a"], ["sub_1__b"]]


def test_subassemblies_payload_reports_expansion_state():
    from sw2robot.editor.webserver import _subassemblies_payload

    payload = _subassemblies_payload(make_graph())
    rows = payload["subassemblies"]
    assert len(rows) == 1
    assert rows[0]["name"] == "servo-1"
    assert rows[0]["children"] == 2
    assert rows[0]["internal_edges"] == 1
    assert rows[0]["movable"] is True
    assert rows[0]["expanded"] is True
    assert rows[0]["override"] == "auto"


def test_subassemblies_payload_reports_no_expand_override():
    from sw2robot.editor.webserver import _subassemblies_payload

    payload = _subassemblies_payload(make_graph(), "no_expand:\n- servo\n")
    rows = payload["subassemblies"]
    assert len(rows) == 1
    assert rows[0]["expanded"] is False
    assert rows[0]["override"] == "no_expand"


def test_set_subassembly_mode_yaml_round_trips_exact_name():
    from sw2robot.editor.webserver import (
        _set_subassembly_mode_yaml,
        _subassemblies_payload,
    )

    txt, members = _set_subassembly_mode_yaml("", make_graph(), "servo-1",
                                              "no_expand")
    assert members["expand"] == []
    assert members["no_expand"] == ["servo-1"]
    row = _subassemblies_payload(make_graph(), txt)["subassemblies"][0]
    assert row["expanded"] is False
    assert row["override"] == "no_expand"

    txt, members = _set_subassembly_mode_yaml(txt, make_graph(), "servo-1",
                                              "expand")
    assert members["expand"] == ["servo-1"]
    assert members["no_expand"] == []
    row = _subassemblies_payload(make_graph(), txt)["subassemblies"][0]
    assert row["expanded"] is True
    assert row["override"] == "expand"

    txt, members = _set_subassembly_mode_yaml(txt, make_graph(), "servo-1",
                                              "auto")
    assert members["expand"] == []
    assert members["no_expand"] == []
    row = _subassemblies_payload(make_graph(), txt)["subassemblies"][0]
    assert row["override"] == "auto"


def test_set_subassembly_mode_yaml_rejects_shared_substring_override():
    from sw2robot.editor.webserver import _set_subassembly_mode_yaml

    g = make_graph()
    inst2 = _comp("servo-2", "servo_2", xyz=(0.2, 0, 0), sub=True,
                  path="X:/fake/servo_unit.SLDASM")
    g.components.append(inst2)
    with pytest.raises(ValueError, match="also match other sub-assemblies"):
        _set_subassembly_mode_yaml("no_expand:\n- servo\n", g, "servo-1",
                                   "expand")


def test_rigid_subassembly_not_expanded():
    g = make_graph()
    # make the internals rigid: bolt pair instead of a hinge
    g.subassemblies["X:/fake/servo_unit.SLDASM"].edges = [
        _edge("case-1", "horn-1",
              dup(conc([0.005, 0, 0], Z), conc([-0.005, 0, 0], Z),
                  coinc_planes(O, Z)))]
    comps, adj, ground = from_graph(g)
    assert {c.name for c in comps} == {"plate-1", "servo-1"}


def test_old_graph_without_subassemblies_unchanged():
    g = make_graph()
    g.subassemblies = {}
    comps, adj, ground = from_graph(g)
    assert {c.name for c in comps} == {"plate-1", "servo-1"}


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))


def _conc_r(p, r, d=Z):
    m = conc(p, d)
    m["radii"] = [r, r]
    return m


def test_strict_small_radius_is_fastener():
    from sw2robot.exporter.model import classify_edge_geo
    jt, ax, note = classify_edge_geo(dup(_conc_r(O, 0.0011)), strict=True)
    assert jt == "fixed" and "fastener" in note


def test_strict_large_radius_stays_revolute():
    from sw2robot.exporter.model import classify_edge_geo
    jt, ax, note = classify_edge_geo(dup(_conc_r(O, 0.017)), strict=True)
    assert jt == "revolute"


def test_nonstrict_small_radius_unchanged():
    from sw2robot.exporter.model import classify_edge_geo
    jt, ax, note = classify_edge_geo(dup(_conc_r(O, 0.0011)), strict=False)
    assert jt == "revolute"   # top-level behaviour (feetech pins) unchanged


def test_nested_movable_triggers_parent_expansion():
    g = make_graph()
    sub = g.subassemblies["X:/fake/servo_unit.SLDASM"]
    # make the direct internals rigid...
    sub.edges = []
    sub.components[1].is_subassembly = True
    sub.components[1].part_path = "X:/fake/inner.SLDASM"
    # ...but give the nested grandchild a real hinge
    inner = SubGraph(
        components=[_comp("shaft-1", "shaft_1"),
                    _comp("wheel-1", "wheel_1")],
        edges=[_edge("shaft-1", "wheel-1",
                     dup(_conc_r(O, 0.01), coinc_planes(O, Z)))],
        ground=[])
    g.subassemblies["X:/fake/inner.SLDASM"] = inner
    comps, adj, ground = from_graph(g)
    names = {c.name for c in comps}
    assert "servo-1/horn-1/shaft-1" in names    # two levels expanded


def test_coaxial_duplicate_demoted_inside_instance():
    # servo unit: output bearing (r=10mm) + far-side flange (r=4mm) on the
    # SAME axis -> only the big one stays a joint
    g = make_graph()
    sub = g.subassemblies["X:/fake/servo_unit.SLDASM"]
    sub.components.append(_comp("flange-1", "flange_1", xyz=(0, 0, -0.01)))
    sub.edges = [
        _edge("case-1", "horn-1", dup(_conc_r(O, 0.010))),
        _edge("case-1", "flange-1", dup(_conc_r([0, 0, -0.01], 0.004))),
    ]
    model = build_model(g)
    rev = [j for j in model.joints if j.jtype == "revolute"]
    assert len(rev) == 1 and "horn" in rev[0].name
    notes = [j.geo_note or "" for j in model.joints if j.jtype == "fixed"]
    assert any("coaxial support bearing" in n for n in notes)


def test_flexible_instance_uses_deep_worlds():
    # flexible instance: the horn has TURNED 90deg about its own hinge axis
    # (motion along the joint DOF).  Expansion must adopt the as-posed world;
    # the hinge axis stays the same world line (a rotation maps its own axis
    # onto itself)
    g = make_graph()
    Rz = np.array([[0, -1, 0, 0], [1, 0, 0, 0],
                   [0, 0, 1, 0], [0, 0, 0, 1]], float)
    T_inst = np.eye(4); T_inst[:3, 3] = [0.1, 0, 0]
    local = np.eye(4); local[:3, 3] = [0, 0, 0.02]
    posed = T_inst @ Rz @ local
    g.deep_worlds = {"servo-1/horn-1": [float(x) for x in posed.flatten()]}
    comps, adj, ground = from_graph(g)
    horn = next(c for c in comps if c.name == "servo-1/horn-1")
    assert np.allclose(horn.world, posed)
    rec = adj[frozenset(("servo-1/case-1", "servo-1/horn-1"))]
    p, d = rec["axis"]
    assert np.allclose(np.abs(d), [0, 0, 1])       # axis still vertical
    assert np.allclose(p[:2], [0.1, 0])            # through the instance


def test_rigid_instance_keeps_composed_transform():
    g = make_graph()
    # deep world consistent with composed (instance offset 0.1x): no change
    Tw = np.eye(4); Tw[:3, 3] = [0.1, 0, 0.02]
    g.deep_worlds = {"servo-1/horn-1": [float(x) for x in Tw.flatten()]}
    comps, adj, ground = from_graph(g)
    horn = next(c for c in comps if c.name == "servo-1/horn-1")
    assert np.allclose(horn.world[:3, 3], [0.1, 0, 0.02])


def test_hidden_components_excluded():
    g = make_graph()
    g.hidden = ["servo-1/horn-1"]
    comps, adj, ground = from_graph(g)
    assert "servo-1/horn-1" not in {c.name for c in comps}
    g2 = make_graph()
    g2.hidden = ["plate-1"]
    comps2, _, _ = from_graph(g2)
    assert "plate-1" not in {c.name for c in comps2}


def test_exclude_applies_after_subassembly_expansion():
    # excluding a part that only exists AFTER sub-assembly expansion (what the
    # editor's Delete sends for a sub-assembly child) must remove it -- the
    # top-level exclude filter never sees it, so the post-expansion pass does.
    comps, adj, ground = from_graph(make_graph(), exclude=["servo-1/horn-1"])
    names = {c.name for c in comps}
    assert "servo-1/horn-1" not in names
    assert names == {"plate-1", "servo-1/case-1"}
    # the removed child no longer appears on any adjacency edge
    assert not any("horn-1" in n for key in adj for n in key)
    # a top-level exclude still works (regression guard)
    comps2, _, _ = from_graph(make_graph(), exclude=["plate-1"])
    assert "plate-1" not in {c.name for c in comps2}


def test_excluded_matches_component_or_link_name():
    # the editor's Delete may send the raw component name OR the sanitised link
    # name; both must match (they differ by '.'/'/'/'-' vs '_').  Real vial case.
    from sw2robot.exporter.model import _excluded
    comp = "vial_phi30_all.SLDPRT-2/vial_phi30-1"
    link = "vial_phi30_all_SLDPRT_2__vial_phi30_1"
    assert _excluded(comp, link, [link.lower()])      # link-name form
    assert _excluded(comp, link, [comp.lower()])      # component-name form
    assert not _excluded(comp, link, ["unrelated"])
    assert not _excluded(comp, link, ["vial_phi30_all_sldprt_1__vial_phi30_1"])


def test_snap_not_fooled_by_multi_mate_single_instance():
    # ONE cover mated to two boards: hole A on its origin axis (0mm), hole B
    # 20mm away -- legitimate design, must NOT be "corrected"
    import numpy as np

    from sw2robot.exporter.model import Component, _warn_unsolved_mates
    def comp(name, path, xyz):
        w = np.eye(4); w[:3, 3] = xyz
        return Component(name=name, link_name=name.replace("-", "_"),
                         part_path=path, is_subassembly=False,
                         world=w, fixed=False, dof=None)
    cover = comp("cover-1", "X:/cover.SLDPRT", (0, 0, 0))
    b1 = comp("board-1", "X:/board.SLDPRT", (0, 0, 0.01))
    b2 = comp("board-2", "X:/board.SLDPRT", (0.02, 0, 0.02))
    def edge(owner_pt):
        return {"types": ["CONCENTRIC"]*2,
                "axis": None,
                "mates": [{"type": "CONCENTRIC", "etypes": [4, 4],
                           "points": [list(owner_pt)]*2,
                           "dirs": [[0, 0, 1]]*2, "radii": [0.002]*2,
                           "owners": ["cover-1", ""]}]*2}
    adj = {frozenset(("cover-1", "board-1")): edge((0, 0, 0)),
           frozenset(("cover-1", "board-2")): edge((0.02, 0, 0))}
    flagged = _warn_unsolved_mates([cover, b1, b2], adj)
    assert "cover-1" not in flagged


def test_snap_still_fires_for_true_stale_sibling():
    import numpy as np

    from sw2robot.exporter.model import Component, _warn_unsolved_mates
    def comp(name, path, xyz):
        w = np.eye(4); w[:3, 3] = xyz
        return Component(name=name, link_name=name.replace("-", "_"),
                         part_path=path, is_subassembly=False,
                         world=w, fixed=False, dof=None)
    # two blades, each ONE mate; blade-2 sits 80mm off its axis
    bl1 = comp("blade-1", "X:/b.SLDPRT", (0.1, 0, 0))
    bl2 = comp("blade-2", "X:/b.SLDPRT", (0.18, 0.05, 0))
    arm1 = comp("arm-1", "X:/a.SLDPRT", (0.1, 0, 0))
    arm2 = comp("arm-2", "X:/a.SLDPRT", (0.1, 0.1, 0))
    def edge(owner, axis_pt):
        return {"types": ["CONCENTRIC"]*2, "axis": None,
                "mates": [{"type": "CONCENTRIC", "etypes": [4, 4],
                           "points": [list(axis_pt)]*2,
                           "dirs": [[0, 0, 1]]*2, "radii": [0.003]*2,
                           "owners": [owner, ""]}]*2}
    adj = {frozenset(("blade-1", "arm-1")): edge("blade-1", (0.1, 0, 0)),
           frozenset(("blade-2", "arm-2")): edge("blade-2", (0.1, 0.1, 0))}
    flagged = _warn_unsolved_mates([bl1, bl2, arm1, arm2], adj)
    assert "blade-2" in flagged and "blade-1" not in flagged


def _adj_rec(mates_):
    from sw2robot.exporter.model import _edge_rec
    return _edge_rec(_edge("a", "b", mates_))


def test_loop_locked_hinge_demoted():
    # A-B looks like a hinge, but A-C and B-C are HARD fixed -> the loop
    # locks the hinge (what SolidWorks drag shows)
    import numpy as np

    from sw2robot.exporter.model import Component, _auto_parent_map
    def comp(name, xyz):
        w = np.eye(4); w[:3, 3] = xyz
        return Component(name=name, link_name=name, part_path=None,
                         is_subassembly=False, world=w, fixed=False,
                         dof=None)
    A, B, C = comp("A", (0, 0, 0)), comp("B", (0.05, 0, 0)), comp("C", (0, 0.05, 0))
    hinge = dup(conc(O, Z), coinc_planes(O, Z))
    rigid = dup(conc([0.0, 0, 0], Z), conc([0.02, 0, 0], Z),
                coinc_planes(O, Z))     # bolt pattern: fully constrained
    from sw2robot.exporter.model import _edge_rec
    def rec(ms):
        e = _edge("x", "y", ms)
        return _edge_rec(e)
    adj = {frozenset(("A", "B")): rec(hinge),
           frozenset(("A", "C")): rec(rigid),
           frozenset(("B", "C")): rec(rigid)}
    parent_of, info = _auto_parent_map([A, B, C], adj, A)
    types = {k: v["type"] for k, v in info.items()}
    assert all(t == "fixed" for t in types.values())
    assert any("globally locked" in (v.get("note") or "")
               for v in info.values())


def test_loop_with_offset_fastener_also_locks():
    # the loop edge is a single small fastener on a DIFFERENT axis: two
    # non-collinear axes on one part -> SolidWorks drag cannot rotate it,
    # so the global solve demotes the hinge too
    import numpy as np

    from sw2robot.exporter.model import Component, _auto_parent_map
    def comp(name, xyz):
        w = np.eye(4); w[:3, 3] = xyz
        return Component(name=name, link_name=name, part_path=None,
                         is_subassembly=False, world=w, fixed=False,
                         dof=None)
    A, B, C = comp("A", (0, 0, 0)), comp("B", (0.05, 0, 0)), comp("C", (0, 0.05, 0))
    from sw2robot.exporter.model import _edge_rec
    def rec(ms, strict=False):
        r = _edge_rec(_edge("x", "y", ms))
        if strict: r["strict"] = True
        return r
    hinge = dup(conc(O, Z), coinc_planes(O, Z))
    rigid = dup(conc([0.0, 0, 0], Z), conc([0.02, 0, 0], Z),
                coinc_planes(O, Z))
    soft = rec(dup(_conc_r([0, 0.02, 0], 0.0012, d=[1, 0, 0])), strict=True)
    adj = {frozenset(("A", "B")): rec(hinge),
           frozenset(("A", "C")): rec(rigid),
           frozenset(("B", "C")): soft}
    parent_of, info = _auto_parent_map([A, B, C], adj, A)
    types = {k: v["type"] for k, v in info.items()}
    assert all(t == "fixed" for t in types.values())


def test_isolated_hinge_survives_global_solve():
    # a hinge with NO other path stays movable under the global solve
    import numpy as np

    from sw2robot.exporter.model import Component, _auto_parent_map
    def comp(name, xyz):
        w = np.eye(4); w[:3, 3] = xyz
        return Component(name=name, link_name=name, part_path=None,
                         is_subassembly=False, world=w, fixed=False,
                         dof=None)
    A, B = comp("A", (0, 0, 0)), comp("B", (0.05, 0, 0))
    from sw2robot.exporter.model import _edge_rec
    adj = {frozenset(("A", "B")):
           _edge_rec(_edge("x", "y", dup(conc(O, Z), coinc_planes(O, Z))))}
    parent_of, info = _auto_parent_map([A, B], adj, A)
    assert [v["type"] for v in info.values()] == ["revolute"]
