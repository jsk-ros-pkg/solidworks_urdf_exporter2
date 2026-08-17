"""refresh_frames: partial re-extract of ONLY the named frames.

Iterating on coordinate-frame selection (PR #178) used to require a full
re-extract -- a multi-minute assembly walk -- just to re-read a handful of
CoordSys/RefAxis features.  refresh_frames updates those fields of an existing
graph.json in place and must leave everything else (components, mates, deep
worlds, meshes) untouched.  SolidWorks is faked out; only the orchestration is
under test here.
"""
import numpy as np
import pytest

from sw2robot.exporter import export as export_mod
from sw2robot.exporter.state import (
    ComponentState,
    CoordinateSystemState,
    GraphState,
    ReferenceAxisState,
    SubGraph,
)

_EYE = [float(x) for x in np.eye(4).flatten()]


class FakeSW:
    """Stands in for swcom.SolidWorks: records open/close, returns a token."""

    def __init__(self):
        self.opened = []
        self.closed = []

    def open_copy(self, path):
        self.opened.append(path)
        return "DOC"

    def close_doc(self, doc):
        self.closed.append(doc)


def _make_package(tmp_path):
    pkg = tmp_path / "robo"
    pkg.mkdir()
    asm = tmp_path / "robo.sldasm"
    asm.write_bytes(b"cad")
    graph = GraphState(
        robot_name="robo", source_assembly=str(asm),
        components=[ComponentState(name="a-1", link_name="a", world=_EYE)],
        coordinate_systems=[CoordinateSystemState(
            name="stale", document_from_frame=_EYE)],
        reference_axes=[ReferenceAxisState(
            name="stale_axis", document_point=[0.0, 0.0, 0.0],
            document_direction=[1.0, 0.0, 0.0])],
        subassemblies={
            r"C:\cad\sub.sldasm": SubGraph(coordinate_systems=[
                CoordinateSystemState(name="sub_stale",
                                      document_from_frame=_EYE)]),
            r"C:\cad\unloaded.sldasm": SubGraph(coordinate_systems=[
                CoordinateSystemState(name="keep_me",
                                      document_from_frame=_EYE)]),
        })
    graph.save(str(pkg / "graph.json"))
    return pkg, asm


def _patch_readers(monkeypatch):
    fresh_cs = CoordinateSystemState(name="fresh", document_from_frame=_EYE)
    fresh_ax = ReferenceAxisState(name="fresh_axis",
                                  document_point=[0.0, 0.0, 0.0],
                                  document_direction=[0.0, 0.0, 1.0])
    monkeypatch.setattr(export_mod, "extract_coordinate_systems",
                        lambda doc, owners=False: [fresh_cs.model_copy()])
    monkeypatch.setattr(export_mod, "extract_reference_axes",
                        lambda doc: [fresh_ax.model_copy()])
    # sub.sldasm is loaded in the (fake) session, unloaded.sldasm is not
    monkeypatch.setattr(export_mod, "_live_subassembly_docs",
                        lambda doc: {"sub.sldasm": "SUBDOC"})


def test_refresh_updates_frames_only(tmp_path, monkeypatch):
    pkg, asm = _make_package(tmp_path)
    _patch_readers(monkeypatch)
    fake = FakeSW()

    out = export_mod.refresh_frames(str(pkg), sw=fake)

    assert out == str(pkg)
    assert fake.opened == [str(asm)]
    assert fake.closed == ["DOC"]           # doc released even on success
    graph = GraphState.load(str(pkg / "graph.json"))
    assert [c.name for c in graph.coordinate_systems] == ["fresh"]
    assert [a.name for a in graph.reference_axes] == ["fresh_axis"]
    # everything that is NOT a frame is byte-for-byte untouched
    assert [c.name for c in graph.components] == ["a-1"]
    assert graph.robot_name == "robo"
    # loaded sub-assembly refreshed; unloaded one keeps its cached frames
    subs = {k.split("\\")[-1]: v for k, v in graph.subassemblies.items()}
    assert [c.name for c in subs["sub.sldasm"].coordinate_systems] == ["fresh"]
    assert [c.name for c in subs["unloaded.sldasm"].coordinate_systems] == \
        ["keep_me"]


def test_refresh_accepts_assembly_path(tmp_path, monkeypatch):
    # the extract-style calling convention: assembly + out dir -> same pkg dir
    pkg, asm = _make_package(tmp_path)
    _patch_readers(monkeypatch)

    out = export_mod.refresh_frames(str(asm), out_dir=str(tmp_path),
                                    sw=FakeSW())

    assert out == str(pkg)
    graph = GraphState.load(str(pkg / "graph.json"))
    assert [c.name for c in graph.coordinate_systems] == ["fresh"]


def test_refresh_requires_prior_extract(tmp_path):
    # NO fallback to a full extract: a missing graph.json is a clear error
    asm = tmp_path / "robo.sldasm"
    asm.write_bytes(b"cad")
    with pytest.raises(FileNotFoundError, match=r"graph\.json"):
        export_mod.refresh_frames(str(asm), out_dir=str(tmp_path), sw=FakeSW())


def test_refresh_closes_doc_on_reader_failure(tmp_path, monkeypatch):
    pkg, asm = _make_package(tmp_path)

    def boom(doc, owners=False):
        raise RuntimeError("COM died")

    monkeypatch.setattr(export_mod, "extract_coordinate_systems", boom)
    fake = FakeSW()
    with pytest.raises(RuntimeError, match="COM died"):
        export_mod.refresh_frames(str(pkg), sw=fake)
    assert fake.closed == ["DOC"]
    # the half-refreshed graph was NOT saved
    graph = GraphState.load(str(pkg / "graph.json"))
    assert [c.name for c in graph.coordinate_systems] == ["stale"]
