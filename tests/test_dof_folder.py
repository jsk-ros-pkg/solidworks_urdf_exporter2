"""DOF-folder mode: only the mates inside a FeatureManager folder named 'dof'
become joints; every other mate edge is welded fixed (force_fixed).

These drive read_dof_edges / _apply_dof_filter against a fake SolidWorks feature
tree -- no COM.  as_iface is monkeypatched to identity, and safe_prop/safe_call
already operate on plain Python objects (getattr, then call if callable)."""
from itertools import pairwise
from types import SimpleNamespace

import pytest

from sw2robot.exporter import model
from sw2robot.exporter.model import (
    _apply_dof_filter,
    _edge_rec,
    _mate_edges,
    classify_edge_auto,
    read_dof_edges,
)
from sw2robot.exporter.state import MateEdge

CONCENTRIC = 1        # MATE_TYPES key


class _Comp:
    def __init__(self, name):
        self._n = name

    @property
    def Name2(self):
        return self._n


class _Ent:
    def __init__(self, comp):
        self._rc = _Comp(comp)

    @property
    def ReferenceComponent(self):
        return self._rc


class _Mate:
    def __init__(self, mtype, comps):
        self._t = mtype
        self._ents = [_Ent(c) for c in comps]

    @property
    def Type(self):
        return self._t

    def GetMateEntityCount(self):
        return len(self._ents)

    def MateEntity(self, i):
        return self._ents[i]


class _Feat:
    """A fake IFeature / sub-feature node (only the methods the code calls)."""

    def __init__(self, typename, name="", spec=None):
        self._tn = typename
        self._nm = name
        self._spec = spec
        self.next_sub = None
        self.next_feat = None
        self.first_sub = None

    def GetTypeName2(self):
        return self._tn

    def GetNextFeature(self):
        return self.next_feat

    def GetFirstSubFeature(self):
        return self.first_sub

    def GetNextSubFeature(self):
        return self.next_sub

    def GetSpecificFeature2(self):
        return self._spec

    @property
    def Name(self):
        return self._nm


class _Doc:
    def __init__(self, first):
        self._first = first

    def FirstFeature(self):
        return self._first


def _mategroup(subs):
    """A doc whose single top feature is a MateGroup holding ``subs`` in order."""
    for a, b in pairwise(subs):
        a.next_sub = b
    mg = _Feat("MateGroup")
    mg.first_sub = subs[0] if subs else None
    return _Doc(mg)


def _dof_doc():
    """Feature tree: a 'dof' folder wrapping one CONCENTRIC A-B mate."""
    start = _Feat("FtrFolder", name="dof")
    mate = _Feat("MateFeature", name="Concentric1",
                 spec=_Mate(CONCENTRIC, ["A", "B"]))
    end = _Feat("FtrFolder", name="dofEndTag___")
    return _mategroup([start, mate, end])


@pytest.fixture(autouse=True)
def _hermetic(monkeypatch):
    # no SolidWorks typelib marshalling, and a fixed folder name regardless of
    # any SW2URDF_DOF_FOLDER in the environment
    monkeypatch.setattr(model, "as_iface", lambda obj, iface: obj)
    monkeypatch.setattr(model, "DOF_FOLDER_NAME", "dof")


def test_read_dof_edges_finds_folder_and_edge():
    found, edges = read_dof_edges(_dof_doc(), {"A", "B", "C"})
    assert found is True
    assert edges == {frozenset({"A", "B"}): {"CONCENTRIC"}}


def test_read_dof_edges_absent_without_folder():
    # the same mate, but sitting OUTSIDE any 'dof' folder
    mate = _Feat("MateFeature", spec=_Mate(CONCENTRIC, ["A", "B"]))
    found, edges = read_dof_edges(_mategroup([mate]), {"A", "B"})
    assert found is False
    assert edges == {}


def test_apply_dof_filter_keeps_dof_and_welds_the_rest(monkeypatch):
    monkeypatch.setattr(model, "_DOF_ACTIVE", True)
    comps = [SimpleNamespace(name=n) for n in ("A", "B", "C")]
    ab = frozenset({"A", "B"})
    bc = frozenset({"B", "C"})
    adjacency = {
        ab: {"types": ["CONCENTRIC", "COINCIDENT"],
             "mates": [{"type": "CONCENTRIC"}, {"type": "COINCIDENT"}]},
        bc: {"types": ["CONCENTRIC"], "mates": [{"type": "CONCENTRIC"}]},
    }

    _apply_dof_filter(_dof_doc(), comps, adjacency)

    # the marked DOF keeps ONLY its rotation axis and is NOT welded
    assert "force_fixed" not in adjacency[ab]
    assert adjacency[ab]["types"] == ["CONCENTRIC"]
    assert [m["type"] for m in adjacency[ab]["mates"]] == ["CONCENTRIC"]

    # every other edge is welded fixed, and the classifier honors it
    assert adjacency[bc]["force_fixed"] is True
    jtype, _axis, _note = classify_edge_auto(adjacency[bc])
    assert jtype == "fixed"


def test_apply_dof_filter_is_noop_when_inactive(monkeypatch):
    monkeypatch.setattr(model, "_DOF_ACTIVE", False)
    ab = frozenset({"A", "B"})
    adjacency = {ab: {"types": ["CONCENTRIC"], "mates": []}}
    _apply_dof_filter(_dof_doc(), [SimpleNamespace(name="A")], adjacency)
    assert "force_fixed" not in adjacency[ab]


def test_force_fixed_survives_graph_round_trip():
    # extract-time force_fixed must reach the build via the MateEdge JSON
    rec = _edge_rec(MateEdge(a="A", b="B", types=["CONCENTRIC"],
                             force_fixed=True))
    assert rec["force_fixed"] is True
    jtype, _axis, _note = classify_edge_auto(rec)
    assert jtype == "fixed"

    # and back: _mate_edges lifts the adjacency flag onto the MateEdge
    adjacency = {frozenset({"A", "B"}): {"types": ["CONCENTRIC"], "mates": [],
                                         "force_fixed": True}}
    (edge,) = _mate_edges(adjacency)
    assert edge.force_fixed is True


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
