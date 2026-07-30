"""Mesh-cache keying: one mesh per (part file, referenced configuration).  Two
instances of the same .SLDPRT that reference configs with different geometry (a
length-configured tube) must NOT share one mesh -- the by_path cache keys on
_mkey, so they get separate entries."""
import pytest

from sw2robot.exporter.mesh import _mkey


def test_same_file_different_config_are_distinct_keys():
    assert _mkey("tube.SLDPRT", "L120") != _mkey("tube.SLDPRT", "L75")


def test_same_file_same_config_collapses():
    assert _mkey("tube.SLDPRT", "L120") == _mkey("tube.SLDPRT", "L120")


def test_empty_config_normalises_to_none():
    # "" (no referenced config) and None must be the SAME key, so a single-config
    # part is not exported twice under two spellings of "default"
    assert _mkey("tube.SLDPRT", "") == _mkey("tube.SLDPRT", None)
    assert _mkey("tube.SLDPRT", "") == ("tube.SLDPRT", None)


def test_different_files_are_distinct():
    assert _mkey("a.SLDPRT", "L120") != _mkey("b.SLDPRT", "L120")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
