"""_save_3dxml must absolutise its destination: SolidWorks resolves a relative
SaveAs path against the SLDWORKS.exe process cwd, not ours, so a relative
`-o output` package dir otherwise fails every 3DXML export.  Driven with a fake
model doc (no COM) whose SaveAs records the path and writes the file."""
import os

import pytest

from sw2robot.exporter import mesh


class _Ext:
    def __init__(self, nbytes):
        self._nbytes = nbytes
        self.saved_path = None

    def SaveAs(self, path, *args):
        self.saved_path = path
        with open(path, "wb") as f:      # emulate SolidWorks writing the file
            f.write(b"x" * self._nbytes)
        return True


class _Doc:
    def __init__(self, nbytes):
        self.Extension = _Ext(nbytes)


@pytest.fixture(autouse=True)
def _identity_as_iface(monkeypatch):
    monkeypatch.setattr(mesh, "as_iface", lambda obj, iface: obj)


def test_relative_out_path_is_absolutised_and_saved(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "meshes").mkdir()
    doc = _Doc(mesh._MIN_MESH_BYTES + 100)

    ok = mesh._save_3dxml(doc, os.path.join("meshes", "foo.3dxml"))  # relative

    assert ok is True
    assert os.path.isabs(doc.Extension.saved_path)   # SolidWorks got an abs path
    assert os.path.exists(tmp_path / "meshes" / "foo.3dxml")


def test_tiny_envelope_is_rejected(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "meshes").mkdir()
    doc = _Doc(10)                        # below _MIN_MESH_BYTES -> empty envelope

    ok = mesh._save_3dxml(doc, os.path.join("meshes", "foo.3dxml"))

    assert ok is False
    assert not os.path.exists(tmp_path / "meshes" / "foo.3dxml")   # not kept
    assert not os.path.exists(tmp_path / "meshes" / "foo.3dxml.part.3dxml")  # tmp cleaned
    assert "3dxml rejected" in capsys.readouterr().out


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
