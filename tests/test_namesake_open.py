"""A same-named file from another folder held by the session must stop the
extract instead of standing in for the assembly's own file.

SolidWorks keeps one document per file name and resolves a reference to a
file it already holds before looking on disk.  With B\\robot.SLDASM open,
extracting A\\robot.SLDASM in that session read B's Base and Case -- geometry
and mass -- with no warning.  Two traps seen on SolidWorks 2024: B's parts
were loaded lightweight, so GetDocuments did not list them, and once resolved,
GetDocumentDependencies2 for A reported B's paths.
"""
import os

import pytest

from sw2robot.exporter import swcom
from sw2robot.exporter.swcom import NamesakeOpenError, SolidWorks


@pytest.fixture(autouse=True)
def _no_typelib(monkeypatch):
    monkeypatch.setattr(swcom, "as_iface", lambda obj, _name: obj)


class _Comp:
    def __init__(self, path):
        self._path = path

    def GetPathName(self):
        return self._path


class _Doc:
    def __init__(self, path, components=()):
        self._path = path
        self._comps = [_Comp(p) for p in components]

    def GetPathName(self):
        return self._path

    def GetTitle(self):
        return os.path.basename(self._path)

    def GetComponents(self, top_only):
        if not self._comps:
            raise AttributeError("a part has no components")
        return self._comps


class _App:
    def __init__(self, docs, deps):
        self._docs = docs
        self._deps = deps

    def GetDocuments(self):
        return self._docs

    def GetDocumentDependencies2(self, path, traverse, search, readonly):
        out = []
        for d in self._deps:
            out += [os.path.basename(d), d]
        return out


def _bare(docs, deps):
    sw = SolidWorks.__new__(SolidWorks)   # skip __init__ (needs live COM)
    sw.app = _App(docs, deps)
    return sw


@pytest.fixture
def folders(tmp_path):
    for sub in ("A", "B"):
        (tmp_path / sub).mkdir()
        for f in ("robot.SLDASM", "Base.SLDPRT", "Case.SLDPRT"):
            (tmp_path / sub / f).write_bytes(b"")
    return tmp_path


def _b_session(b):
    # B open with its parts LIGHTWEIGHT: only the assembly is a document
    return [_Doc(str(b / "robot.SLDASM"),
                 [str(b / "Base.SLDPRT"), str(b / "Case.SLDPRT")])]


def test_lightweight_namesake_with_polluted_dependency_records(folders):
    a, b = folders / "A", folders / "B"
    # the dependency records already point at B's in-memory parts
    sw = _bare(_b_session(b), [str(b / "Base.SLDPRT"), str(b / "Case.SLDPRT")])
    assert sorted(sw._namesake_conflicts(str(a / "robot.SLDASM"))) == [
        (str(b / "Base.SLDPRT"), str(a / "Base.SLDPRT")),
        (str(b / "Case.SLDPRT"), str(a / "Case.SLDPRT"))]


def test_the_assemblys_own_files_held_are_not_a_conflict(folders):
    a = folders / "A"
    sw = _bare([_Doc(str(a / "Base.SLDPRT"))], [str(a / "Base.SLDPRT")])
    assert sw._namesake_conflicts(str(a / "robot.SLDASM")) == []


def test_a_shared_library_part_is_judged_by_its_recorded_path(folders):
    lib1, lib2 = folders / "lib1", folders / "lib2"
    for d in (lib1, lib2):
        d.mkdir()
        (d / "Servo.SLDPRT").write_bytes(b"")
    a = folders / "A"
    sw = _bare([_Doc(str(lib1 / "Servo.SLDPRT"))], [str(lib1 / "Servo.SLDPRT")])
    assert sw._namesake_conflicts(str(a / "robot.SLDASM")) == []
    sw = _bare([_Doc(str(lib2 / "Servo.SLDPRT"))], [str(lib1 / "Servo.SLDPRT")])
    assert sw._namesake_conflicts(str(a / "robot.SLDASM")) == [
        (str(lib2 / "Servo.SLDPRT"), str(lib1 / "Servo.SLDPRT"))]


def test_open_copy_stops_before_opening(folders):
    a, b = folders / "A", folders / "B"
    sw = _bare(_b_session(b), [str(b / "Base.SLDPRT")])
    with pytest.raises(NamesakeOpenError) as e:
        sw.open_copy(str(a / "robot.SLDASM"))
    assert str(b / "Base.SLDPRT") in str(e.value)
    assert str(a / "Base.SLDPRT") in str(e.value)
    assert "Close" in str(e.value)
