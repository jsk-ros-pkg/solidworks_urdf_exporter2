"""Reusing the user's open document must pick the SAME file, not a namesake.

An attached session extracts from the assembly already open in SolidWorks.
Two folders often hold an assembly of the same name (a Pack and Go, a variant,
a backup), so matching on the title alone could extract the other one.
"""

from sw2robot.exporter.swcom import SolidWorks


class _Doc:
    def __init__(self, path, title):
        self._path = path
        self._title = title

    def GetPathName(self):
        return self._path

    def GetTitle(self):
        return self._title


class _App:
    def __init__(self, docs):
        self._docs = docs

    def GetDocuments(self):
        return self._docs


def _bare(docs):
    sw = SolidWorks.__new__(SolidWorks)   # skip __init__ (needs live COM)
    sw.app = _App(docs)
    return sw


def test_same_named_assembly_in_another_folder_is_not_reused():
    other = _Doc(r"C:\backup\robot.SLDASM", "robot.SLDASM")
    mine = _Doc(r"C:\work\robot.SLDASM", "robot.SLDASM")
    sw = _bare([other, mine])
    assert sw._find_open_doc(r"C:\work\robot.SLDASM") is mine
    assert sw._find_open_doc(r"C:\elsewhere\robot.SLDASM") is None


def test_path_match_ignores_case_and_separator():
    mine = _Doc(r"C:\Work\Robot.SLDASM", "Robot")
    assert _bare([mine])._find_open_doc("c:/work/robot.sldasm") is mine


def test_title_is_used_when_no_path_is_reported():
    doc = _Doc(None, "robot.SLDASM")
    assert _bare([doc])._find_open_doc(r"C:\work\robot.SLDASM") is doc
