"""The 3DXML header must not publish the exporter's Windows login.

SolidWorks writes the Windows login into <Header><Author> and the exporter's
temp SaveAs name (~sw2robot) into <Title>; a package committed to a public
repository carries both.  stamp_3dxml_headers replaces them.
"""
import zipfile

import pytest

from sw2robot.exporter.mesh import (
    AUTHOR_ENV,
    _set_header,
    set_3dxml_header,
    stamp_3dxml_headers,
)

MANIFEST = b'<?xml version="1.0" encoding="UTF-8"?>\n' \
           b'<Manifest><Root>NameSetTree.3dxml</Root></Manifest>'
# as SOLIDWORKS 2024 SP5.0 writes it
HEADER = (b"<Header><SchemaVersion>4.2</SchemaVersion><Title>~sw2robot</Title>"
          b"<Author>abior</Author><Generator>SOLIDWORKS 2024 SP5.0</Generator>"
          b"<Created>2026-10-30</Created></Header>")
TREE = (b'<?xml version="1.0" encoding="UTF-8"?>'
        b'<Model_3dxml xmlns="http://www.3ds.com/xsd/3DXML">' + HEADER +
        b'<ProductStructure root="1"><Reference3D id="1" name="Author"/>'
        b'</ProductStructure></Model_3dxml>')


def _make(tmp_path, name="board_panel_1.3dxml", tree=TREE):
    p = tmp_path / name
    with zipfile.ZipFile(p, "w") as z:
        z.writestr("Manifest.xml", MANIFEST)
        z.writestr("NameSetTree.3dxml", tree)
        z.writestr("TessPart_2.3DRep", b"tessellation bytes")
    return p


def _tree(path):
    with zipfile.ZipFile(path) as z:
        return z.read("NameSetTree.3dxml")


def test_author_and_temp_title_are_replaced(tmp_path):
    p = _make(tmp_path)
    assert set_3dxml_header(str(p), author="Iori Yanokura")
    t = _tree(p)
    assert b"<Author>Iori Yanokura</Author>" in t
    assert b"<Title>board_panel_1</Title>" in t
    assert b"abior" not in t and b"~sw2robot" not in t
    # nothing outside the header moves
    assert b'name="Author"' in t and b"<Generator>SOLIDWORKS" in t
    with zipfile.ZipFile(p) as z:
        assert z.read("TessPart_2.3DRep") == b"tessellation bytes"


def test_none_keeps_the_author(tmp_path):
    p = _make(tmp_path)
    assert set_3dxml_header(str(p))          # the temp title still goes
    assert b"<Author>abior</Author>" in _tree(p)


def test_empty_author_clears_it():
    fixed, changed = _set_header(TREE, author="")
    assert changed and b"<Author></Author>" in fixed


def test_a_real_title_is_left_alone():
    tree = TREE.replace(b"~sw2robot", b"robot_assembly")
    fixed, changed = _set_header(tree, title="other")
    assert not changed and fixed == tree


def test_author_is_escaped_and_utf8():
    fixed, _ = _set_header(TREE, author="矢野倉 & <lab>")
    assert "<Author>矢野倉 &amp; &lt;lab&gt;</Author>".encode() in fixed


def test_missing_author_is_added():
    tree = TREE.replace(b"<Author>abior</Author>", b"")
    fixed, changed = _set_header(tree, author="x")
    assert changed and b"<Author>x</Author></Header>" in fixed


def test_already_stamped_file_is_not_rewritten(tmp_path):
    p = _make(tmp_path)
    set_3dxml_header(str(p), author="x")
    before = p.read_bytes()
    assert not set_3dxml_header(str(p), author="x")
    assert p.read_bytes() == before


def test_stamp_reads_the_environment(tmp_path, monkeypatch):
    a, b = _make(tmp_path, "a.3dxml"), _make(tmp_path, "b.3dxml")
    monkeypatch.setenv(AUTHOR_ENV, "Iori Yanokura")
    assert stamp_3dxml_headers([str(a), str(b)]) == 2
    assert b"<Author>Iori Yanokura</Author>" in _tree(b)


def test_stamp_without_the_environment_keeps_the_author(tmp_path, monkeypatch):
    monkeypatch.delenv(AUTHOR_ENV, raising=False)
    p = _make(tmp_path)
    stamp_3dxml_headers([str(p)])
    assert b"<Author>abior</Author>" in _tree(p)


def test_not_a_zip_is_survivable(tmp_path):
    p = tmp_path / "broken.3dxml"
    p.write_bytes(b"not a zip")
    assert not set_3dxml_header(str(p), author="x")
    assert p.read_bytes() == b"not a zip"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
