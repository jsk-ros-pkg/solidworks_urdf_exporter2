"""Build-time sub-assembly expansion (_expand_subassemblies) on a synthetic
graph: a 'servo unit' sub-assembly whose horn turns inside, mounted on a
base plate.  Expansion must splice the children in, keep the internal
revolute, and re-attach the mounting mate to the owning child."""
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
        _set_subassembly_parent_override_yaml,
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
    } >= {"multiple_parents", "multiple_boundary_parents"}

    txt = _set_subassembly_parent_override_yaml(
        txt, graph, "servo-1", "bracket_1")
    payload = _collapse_preview_payload(graph, txt)
    assert payload["parent_choices"][0]["selected_parent"] == "bracket_1"
    assert [(j["parent"], j["child"]) for j in payload["joints"]] == [
        ("bracket_1", "servo_1"),
        ("plate_1", "bracket_1"),
    ]
    assert [j["source_name"]
            for j in payload["dropped_parent_override_joints"]] == [
        "plate_1__servo_1__case_1"
    ]
    assert "multiple_boundary_parents" not in {
        i["code"] for i in payload["validation"]["issues"]
    }


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
    assert payload["ok"] is False


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
