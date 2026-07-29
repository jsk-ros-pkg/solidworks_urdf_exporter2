"""resolve_lightweight_components / resolve_suppressed_components force-resolve
components that would otherwise export as holes (lightweight) or vanish entirely
(config-suppressed).  Driven with fake COM components -- as_iface is patched to
identity; safe_call/safe_prop already work on plain objects.

Suppression states follow swComponentSuppressionState_e: 0 = suppressed,
1 = lightweight, 2 = fully resolved."""
import pytest

from sw2robot.exporter import swcom
from sw2robot.exporter.swcom import (
    resolve_lightweight_components,
    resolve_suppressed_components,
)


class _Comp:
    def __init__(self, name, state, resolvable=True):
        self._name = name
        self._state = state
        self._resolvable = resolvable
        self.set_calls = []

    def GetSuppression(self):
        return self._state

    def SetSuppression2(self, v):
        self.set_calls.append(v)
        if not self._resolvable:                 # e.g. the part file is missing
            raise RuntimeError("cannot resolve")
        self._state = v

    @property
    def Name2(self):
        return self._name


class _Doc:
    def __init__(self, comps):
        self._comps = comps

    def GetComponents(self, top_only):
        return self._comps


@pytest.fixture(autouse=True)
def _identity_as_iface(monkeypatch):
    monkeypatch.setattr(swcom, "as_iface", lambda obj, iface: obj)


def test_resolve_lightweight_only_touches_lightweight():
    lw = _Comp("lw", 1)
    resolved = _Comp("already", 2)
    supp = _Comp("supp", 0)
    n = resolve_lightweight_components(_Doc([lw, resolved, supp]))
    assert n == 1
    assert lw.set_calls == [2]        # driven to fully-resolved
    assert resolved.set_calls == []   # left alone
    assert supp.set_calls == []       # not lightweight -> untouched


def test_resolve_suppressed_counts_and_names_recovered():
    board = _Comp("encoder_board-1", 0)
    lw = _Comp("lw-1", 1)
    n, names = resolve_suppressed_components(_Doc([board, lw]))
    assert n == 1
    assert names == ["encoder_board-1"]
    assert lw.set_calls == []          # only config-suppressed (state 0) touched


def test_resolve_suppressed_skips_unresolvable():
    # a suppressed part whose file cannot be found: SetSuppression2 raises and
    # it must NOT be counted as recovered
    missing = _Comp("missing-1", 0, resolvable=False)
    n, names = resolve_suppressed_components(_Doc([missing]))
    assert n == 0
    assert names == []


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
