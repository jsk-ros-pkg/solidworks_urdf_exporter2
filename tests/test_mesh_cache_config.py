"""The per-part mesh cache must not hand back ANOTHER configuration's geometry.

meshes/<link>.3dxml is keyed on the link name, and a .3dxml records nothing
about the configuration it was tessellated from.  Switching a part (or the
assembly) to another configuration leaves the previous config's mesh sitting at
exactly the filename the new one would take, and it passes the mtime gate --
the .SLDPRT itself did not change -- so it was reused and the export came out
with geometry from two configurations mixed together.  A sidecar manifest
records (source, config) per mesh so the reuse is checkable.
"""
import json
import os

import pytest

from sw2robot.exporter.mesh import (
    _CACHE_MANIFEST,
    _MIN_MESH_BYTES,
    _cache_holds_config,
    _manifest_record,
)


def _mesh(meshes_dir, name="link.3dxml"):
    p = meshes_dir / name
    p.write_bytes(b"x" * _MIN_MESH_BYTES)
    return str(p)


def test_unrecorded_mesh_is_never_reused(tmp_path):
    # written by an older sw2robot (or by hand): its config is unknowable, so
    # re-export rather than risk mixing configurations
    cand = _mesh(tmp_path)
    assert _cache_holds_config(str(tmp_path), cand, "Default") is False


def test_recorded_config_matches(tmp_path):
    cand = _mesh(tmp_path)
    _manifest_record(str(tmp_path), cand, "C:/parts/link.SLDPRT", "Default")
    assert _cache_holds_config(str(tmp_path), cand, "Default") is True


def test_other_config_forces_reexport(tmp_path):
    # the reported bug: mesh cached as 'variant_b', assembly references 'Default'
    cand = _mesh(tmp_path)
    _manifest_record(str(tmp_path), cand, "C:/parts/link.SLDPRT", "variant_b")
    assert _cache_holds_config(str(tmp_path), cand, "Default") is False


def test_empty_config_is_the_same_as_none(tmp_path):
    cand = _mesh(tmp_path)
    _manifest_record(str(tmp_path), cand, "C:/parts/link.SLDPRT", None)
    assert _cache_holds_config(str(tmp_path), cand, "") is True
    assert _cache_holds_config(str(tmp_path), cand, None) is True
    assert _cache_holds_config(str(tmp_path), cand, "Default") is False


def test_records_accumulate_and_survive_rewrites(tmp_path):
    a, b = _mesh(tmp_path, "a.3dxml"), _mesh(tmp_path, "b.3dxml")
    _manifest_record(str(tmp_path), a, "a.SLDPRT", "left")
    _manifest_record(str(tmp_path), b, "b.SLDPRT", "right")
    assert _cache_holds_config(str(tmp_path), a, "left") is True
    assert _cache_holds_config(str(tmp_path), b, "right") is True
    with open(tmp_path / _CACHE_MANIFEST, encoding="utf-8") as f:
        man = json.load(f)
    assert man["a.3dxml"]["source"] == "a.SLDPRT"
    assert set(man) == {"a.3dxml", "b.3dxml"}


def test_reexport_on_the_same_name_updates_the_record(tmp_path):
    cand = _mesh(tmp_path)
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", "variant_b")
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", "Default")
    assert _cache_holds_config(str(tmp_path), cand, "Default") is True
    assert _cache_holds_config(str(tmp_path), cand, "variant_b") is False


def test_corrupt_manifest_is_ignored_not_fatal(tmp_path):
    cand = _mesh(tmp_path)
    (tmp_path / _CACHE_MANIFEST).write_text("{not json", encoding="utf-8")
    assert _cache_holds_config(str(tmp_path), cand, "Default") is False
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", "Default")
    assert _cache_holds_config(str(tmp_path), cand, "Default") is True


def test_manifest_is_written_next_to_the_meshes(tmp_path):
    cand = _mesh(tmp_path)
    _manifest_record(str(tmp_path), cand, "link.SLDPRT", "Default")
    assert os.path.exists(tmp_path / _CACHE_MANIFEST)
    # no half-written temp left behind
    assert not os.path.exists(tmp_path / (_CACHE_MANIFEST + ".part"))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
