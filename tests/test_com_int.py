"""_com_int tolerates SolidWorks enum properties that marshal as VARIANT.

Some mate entities come back late-bound, so ``ReferenceType`` / mate ``Type``
arrive as a win32com VARIANT instead of an int; ``int(VARIANT)`` then raised
``TypeError`` and aborted the whole extract.  _com_int pulls the value out."""
from sw2robot.exporter.model import _com_int


class _FakeVariant:
    """Stand-in for a win32com VARIANT wrapping an enum value."""
    def __init__(self, value):
        self.value = value

    def __int__(self):
        raise TypeError("int() argument must be ... not 'VARIANT'")


def test_plain_int():
    assert _com_int(1) == 1
    assert _com_int(0) == 0


def test_float_like():
    assert _com_int(4.0) == 4


def test_variant_with_value():
    assert _com_int(_FakeVariant(1)) == 1          # CONCENTRIC
    assert _com_int(_FakeVariant(20)) == 20        # LOCK


def test_none_and_garbage():
    assert _com_int(None) is None
    assert _com_int(_FakeVariant(None)) is None
    assert _com_int(_FakeVariant("nope")) is None
    assert _com_int(object()) is None
