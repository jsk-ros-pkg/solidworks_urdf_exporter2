"""Two configurations of one part must not collapse into one shape.

SolidWorks writes a 3DXML with one tessellation per CONFIGURATION, so a part
used with two of them appears as two Reference3D nodes carrying the SAME name.
trimesh re-keys geometry by that name, so the second overwrites the first and
BOTH instances get drawn with whichever survived -- a mirrored bracket on the
wrong side, a spacer 8 mm short, while the part COUNT still looks right.
_unique_part_names disambiguates the names in the file we ship.
"""
import zipfile

import pytest

from sw2robot.exporter.mesh import _dedupe_reference_names, _unique_part_names

MANIFEST = b'<?xml version="1.0" encoding="UTF-8"?>\n' \
           b'<Manifest><Root>NameSetTree.3dxml</Root></Manifest>'


def _tree(*refs):
    body = b"".join(
        b'<Reference3D xsi:type="Reference3DType" id="%d" name="%s"/>'
        % (i + 1, n) for i, n in enumerate(refs))
    return (b'<?xml version="1.0" encoding="UTF-8"?>'
            b'<Model_3dxml xmlns="http://www.3ds.com/xsd/3DXML">'
            b'<ProductStructure root="1">' + body +
            b'</ProductStructure></Model_3dxml>')


def _make(tmp_path, tree, extra=None, root="NameSetTree.3dxml"):
    p = tmp_path / "m.3dxml"
    with zipfile.ZipFile(p, "w") as z:
        z.writestr("Manifest.xml", MANIFEST)
        z.writestr(root, tree)
        z.writestr("TessPart_5.3DRep", b"tessellation bytes")
        for k, v in (extra or {}).items():
            z.writestr(k, v)
    return p


def _names(path):
    with zipfile.ZipFile(path) as z:
        import re
        return re.findall(rb'name="([^"]*)"', z.read("NameSetTree.3dxml"))


def test_duplicate_names_are_disambiguated(tmp_path):
    p = _make(tmp_path, _tree(b"bracket", b"bracket", b"screw"))
    assert _unique_part_names(str(p)) == 1
    assert _names(p) == [b"bracket", b"bracket__2", b"screw"]


def test_first_occurrence_keeps_its_name(tmp_path):
    # renaming the survivor would churn every package for no reason
    p = _make(tmp_path, _tree(b"a", b"a", b"a"))
    assert _unique_part_names(str(p)) == 2
    assert _names(p) == [b"a", b"a__2", b"a__3"]


def test_unique_file_is_left_alone(tmp_path):
    p = _make(tmp_path, _tree(b"a", b"b", b"c"))
    before = p.read_bytes()
    assert _unique_part_names(str(p)) == 0
    assert p.read_bytes() == before          # not even rewritten


def test_other_entries_survive_the_rewrite(tmp_path):
    p = _make(tmp_path, _tree(b"a", b"a"),
              extra={"CATMaterialRef.3dxml": b"<materials/>"})
    assert _unique_part_names(str(p)) == 1
    with zipfile.ZipFile(p) as z:
        assert set(z.namelist()) == {"Manifest.xml", "NameSetTree.3dxml",
                                     "TessPart_5.3DRep",
                                     "CATMaterialRef.3dxml"}
        assert z.read("TessPart_5.3DRep") == b"tessellation bytes"
        assert z.read("CATMaterialRef.3dxml") == b"<materials/>"


def test_escaped_and_cjk_names_are_not_re_encoded(tmp_path):
    # the suffix goes onto the RAW attribute bytes, so an &amp; stays &amp;
    # and a CJK name keeps its exact encoding
    cjk = "ネジ".encode()
    p = _make(tmp_path, _tree(b"a&amp;b", b"a&amp;b", cjk, cjk))
    assert _unique_part_names(str(p)) == 2
    assert _names(p) == [b"a&amp;b", b"a&amp;b__2", cjk, cjk + b"__2"]


def test_impossible_zip_dates_do_not_lose_entries(tmp_path):
    # SolidWorks stamps 3DXML entries with day 0 / month 13; zipfile refuses to
    # pack those, and dropping the entry would mean dropping geometry
    p = _make(tmp_path, _tree(b"a", b"a"))
    src = p.read_bytes()
    bad = tmp_path / "bad.3dxml"
    with zipfile.ZipFile(p) as z, zipfile.ZipFile(bad, "w") as out:
        for info in z.infolist():
            zi = zipfile.ZipInfo(info.filename)
            zi.date_time = (1980, 1, 0, 0, 0, 0)      # day 0, as SolidWorks
            out.writestr(zi, z.read(info.filename))
    assert _unique_part_names(str(bad)) == 1
    with zipfile.ZipFile(bad) as z:
        assert z.read("TessPart_5.3DRep") == b"tessellation bytes"
    assert src                                        # sanity


def test_not_a_zip_is_survivable(tmp_path):
    p = tmp_path / "broken.3dxml"
    p.write_bytes(b"not a zip at all")
    assert _unique_part_names(str(p)) == 0
    assert p.read_bytes() == b"not a zip at all"


def test_missing_manifest_is_survivable(tmp_path):
    p = tmp_path / "m.3dxml"
    with zipfile.ZipFile(p, "w") as z:
        z.writestr("SomethingElse.xml", b"<x/>")
    assert _unique_part_names(str(p)) == 0


def test_dedupe_is_pure_on_the_bytes():
    raw = _tree(b"x", b"x")
    fixed, n = _dedupe_reference_names(raw)
    assert n == 1
    assert b'name="x"' in fixed and b'name="x__2"' in fixed
    assert _dedupe_reference_names(fixed)[1] == 0     # idempotent


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
