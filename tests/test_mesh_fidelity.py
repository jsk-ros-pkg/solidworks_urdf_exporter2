"""Read every written mesh back and check it survived the round trip.

The export hands geometry to a third-party loader and never looks at it again,
so anything the loader drops or merges ships into the URDF silently.  That is
how two configuration variants of one part came to be collapsed into a single
shape, drawn superimposed, with the part COUNT still correct -- nothing in the
pipeline was in a position to notice.  Comparing what the file declares against
what the loader returns catches that whole class, not just the known cause.
"""
import zipfile

import pytest

from sw2robot.exporter.mesh import _declared_counts, verify_mesh, verify_meshes

MANIFEST = b'<Manifest><Root>NameSetTree.3dxml</Root></Manifest>'


def _doc(refs, reps, instances):
    """refs: [(id, name)]; reps: [ref_id owning geometry];
    instances: [ref_id being instanced]."""
    out = [b'<Model_3dxml xmlns="http://www.3ds.com/xsd/3DXML">']
    for i, n in refs:
        out.append(b'<Reference3D id="%d" name="%s"/>' % (i, n))
    for r in reps:
        out.append(b'<InstanceRep><IsAggregatedBy>%d</IsAggregatedBy>'
                   b'<IsInstanceOf>99</IsInstanceOf></InstanceRep>' % r)
    for r in instances:
        out.append(b'<Instance3D name="x-1"><IsInstanceOf>%d</IsInstanceOf>'
                   b'</Instance3D>' % r)
    out.append(b'</Model_3dxml>')
    return b"".join(out)


def _make(tmp_path, tree, name="m.3dxml"):
    p = tmp_path / name
    with zipfile.ZipFile(p, "w") as z:
        z.writestr("Manifest.xml", MANIFEST)
        z.writestr("NameSetTree.3dxml", tree)
    return p


def test_declared_counts_ignores_subassembly_nodes(tmp_path):
    # ref 1 is a sub-assembly (no InstanceRep) -- it draws nothing itself and
    # must not be counted as a shape
    p = _make(tmp_path, _doc(refs=[(1, b"asm"), (2, b"part")],
                             reps=[2], instances=[1, 2, 2]))
    assert _declared_counts(str(p)) == (1, 2)


def test_two_configs_of_one_part_count_as_two_shapes(tmp_path):
    p = _make(tmp_path, _doc(refs=[(1, b"D"), (2, b"D")],
                             reps=[1, 2], instances=[1, 2]))
    assert _declared_counts(str(p)) == (2, 2)


def test_a_loader_that_merges_two_shapes_is_reported(tmp_path, monkeypatch):
    # the bug as it happened: 2 declared shapes, 1 returned
    p = _make(tmp_path, _doc(refs=[(1, b"D"), (2, b"D")],
                             reps=[1, 2], instances=[1, 2]))
    _fake_loader(monkeypatch, geometries=1, nodes=2)
    msg = verify_mesh(str(p))
    assert msg and "declares 2 shape(s) but reads back as 1" in msg
    assert "merged or dropped" in msg


def test_a_loader_that_drops_an_instance_is_reported(tmp_path, monkeypatch):
    p = _make(tmp_path, _doc(refs=[(1, b"a")], reps=[1], instances=[1, 1, 1]))
    _fake_loader(monkeypatch, geometries=1, nodes=2)
    assert "declares 3 instance(s) but reads back as 2" in verify_mesh(str(p))


def test_a_single_part_export_is_not_a_false_alarm(tmp_path, monkeypatch):
    # a lone .SLDPRT has no Instance3D at all -- the root reference IS the
    # geometry -- yet the loader still yields one node.  Flagging that would
    # bury the real complaints under one per part file.
    p = _make(tmp_path, _doc(refs=[(1, b"part")], reps=[1], instances=[]))
    _fake_loader(monkeypatch, geometries=1, nodes=1)
    assert verify_mesh(str(p)) is None


def test_a_faithful_read_is_silent(tmp_path, monkeypatch):
    p = _make(tmp_path, _doc(refs=[(1, b"a"), (2, b"b")],
                             reps=[1, 2], instances=[1, 2, 2]))
    _fake_loader(monkeypatch, geometries=2, nodes=3)
    assert verify_mesh(str(p)) is None


def test_an_unreadable_mesh_is_reported_not_raised(tmp_path, monkeypatch):
    p = _make(tmp_path, _doc(refs=[(1, b"a")], reps=[1], instances=[1]))

    class Boom:
        @staticmethod
        def load(_):
            raise RuntimeError("nope")

    monkeypatch.setitem(__import__("sys").modules, "trimesh", Boom)
    assert "could not be read back" in verify_mesh(str(p))


def test_a_non_3dxml_file_is_skipped(tmp_path):
    p = tmp_path / "plain.3dxml"
    p.write_bytes(b"not a zip")
    assert _declared_counts(str(p)) is None
    assert verify_mesh(str(p)) is None      # nothing to check, not a failure


def test_verify_meshes_collects_every_complaint(tmp_path, monkeypatch):
    good = _make(tmp_path, _doc(refs=[(1, b"a")], reps=[1], instances=[1]),
                 name="good.3dxml")
    bad = _make(tmp_path, _doc(refs=[(1, b"D"), (2, b"D")],
                               reps=[1, 2], instances=[1, 2]),
                name="bad.3dxml")
    _fake_loader(monkeypatch, geometries=1, nodes=1)
    said = []
    problems = verify_meshes([str(good), str(bad)], say=said.append)
    assert len(problems) == 1 and "bad.3dxml" in problems[0]
    assert said and "1 PROBLEM" in said[0]


def _fake_loader(monkeypatch, geometries, nodes):
    """Stand in for trimesh so the check is tested, not the CAD reader."""
    class Graph:
        def __init__(self):
            self.nodes_geometry = ["n"] * nodes

    class Scene:
        def __init__(self):
            self.geometry = {str(i): None for i in range(geometries)}
            self.graph = Graph()

    class Fake:
        @staticmethod
        def load(_):
            return Scene()

    monkeypatch.setitem(__import__("sys").modules, "trimesh", Fake)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
