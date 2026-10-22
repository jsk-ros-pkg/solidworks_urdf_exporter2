"""The per-part mesh cache must not hand back a mesh made with OLD texture images.

A texture appearance references its image by path and the .3dxml embeds a copy,
so changing the image on disk touches neither the part file nor the mesh: the
mtime gate passed and the export kept the old image.  The manifest records the
images each mesh was made with and their mtimes.
"""
import json
import os

from sw2robot.exporter import mesh
from sw2robot.exporter.mesh import (
    _CACHE_MANIFEST,
    _MIN_MESH_BYTES,
    _cache_textures_unchanged,
    _manifest_record,
)


def _mesh(meshes_dir, name="link.3dxml"):
    p = meshes_dir / name
    p.write_bytes(b"x" * _MIN_MESH_BYTES)
    return str(p)


def _image(tmp_path, name="skin.png"):
    p = tmp_path / name
    p.write_bytes(b"png")
    return str(p)


def test_unchanged_images_allow_reuse(tmp_path):
    cand, img = _mesh(tmp_path), _image(tmp_path)
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", None, textures=[img])
    assert _cache_textures_unchanged(str(tmp_path), cand) is True


def test_a_changed_image_forces_reexport(tmp_path):
    cand, img = _mesh(tmp_path), _image(tmp_path)
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", None, textures=[img])
    t = os.path.getmtime(img)
    os.utime(img, (t + 10, t + 10))
    assert _cache_textures_unchanged(str(tmp_path), cand) is False


def test_a_missing_image_forces_reexport(tmp_path):
    cand, img = _mesh(tmp_path), _image(tmp_path)
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", None, textures=[img])
    os.remove(img)
    assert _cache_textures_unchanged(str(tmp_path), cand) is False


def test_a_mesh_without_textures_is_reused(tmp_path):
    cand = _mesh(tmp_path)
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", None)
    assert _cache_textures_unchanged(str(tmp_path), cand) is True


def test_a_record_from_before_textures_were_kept_is_refused(tmp_path):
    cand = _mesh(tmp_path)
    (tmp_path / _CACHE_MANIFEST).write_text(
        json.dumps({"link.3dxml": {"source": "link.SLDPRT", "config": None}}), encoding="utf-8")
    assert _cache_textures_unchanged(str(tmp_path), cand) is False


def test_the_images_save_3dxml_noted_are_recorded(tmp_path, monkeypatch):
    # _save_3dxml notes the document's images under the mesh's path; the record
    # that follows takes them without being told
    cand, img = _mesh(tmp_path), _image(tmp_path)
    # noted under the absolute path, recorded through a relative one (as the export does)
    monkeypatch.setitem(mesh._TEXTURES_OF, mesh._texture_key(cand), [img])
    monkeypatch.chdir(tmp_path)
    _manifest_record(str(tmp_path), os.path.relpath(cand, tmp_path), "link.SLDPRT", None)
    with open(tmp_path / _CACHE_MANIFEST, encoding="utf-8") as f:
        assert list(json.load(f)["link.3dxml"]["textures"]) == [img]
    assert mesh._texture_key(cand) not in mesh._TEXTURES_OF
    os.utime(img, (os.path.getmtime(img) + 10,) * 2)
    assert _cache_textures_unchanged(str(tmp_path), cand) is False
