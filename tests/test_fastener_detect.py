"""Standard-hardware detection (is_fastener_part) + edge welding.

A bolt threaded into a tapped hole is a concentric mate the joint classifier
reads as a hinge, so every screw/nut/washer/pin spawns a spurious revolute.
is_fastener_part flags them off their library folder + catalogue naming so the
build welds them rigid -- without touching real mechanism parts (bearings, the
harmonic drive, the motor shaft) or the in-house links (sheet metal, the
3D-printed AprilTag marker)."""
import pytest

from sw2robot.exporter.model import classify_edge_auto, is_fastener_part


def p(folder, name, ext="SLDPRT"):
    return rf"X\Parts\{folder}\{name}.{ext}"


# (folder, part-name, expected) drawn from a real ReSEnCHiMan joint module
CASES = [
    # --- fasteners: weld fixed ---
    ("Bolt", "hex-socket-head-cap_M3x6", True),
    ("Bolt", "hex_socket_head_cap_M4x10", True),
    ("clinching-nut", "FS-M3-1-3W", True),
    ("Pin", "Pint_fai6_L12", True),       # folder "Pin" is the tell
    ("Washer", "plain-washer-M3", True),
    ("Screw", "M2.5x8", True),            # size designation only
    # --- NOT fasteners: keep as real links ---
    ("3DPrinter", "DummyAprilTag", False),          # the AR marker -- must stay
    ("MetalSheet", "MotorFrame_2_Cover", False),
    ("HarmonicDrive", "CSD-20-160-Circular", False),  # the real joint
    ("Gear", "Pinion_gear", False),       # 'pin' must not match 'pinion'
    ("Motor", "Spindle_AT4130", False),   # 'pin' must not match 'spindle'
    ("Bearing", "NSK_MR126ZZ", False),
]


@pytest.mark.parametrize("folder,name,want", CASES)
def test_is_fastener_part(folder, name, want):
    assert is_fastener_part(name, p(folder, name)) is want


def test_config_overrides_keep_and_extra():
    # not_fastener vetoes a would-be match; fastener: adds a custom one
    assert is_fastener_part("hex-socket-head-cap_M3x6",
                            p("Bolt", "hex-socket-head-cap_M3x6"),
                            keep=["hex-socket-head-cap"]) is False
    assert is_fastener_part("widget_42", p("Custom", "widget_42"),
                            extra=["widget"]) is True


def test_no_part_path_uses_name_only():
    assert is_fastener_part("M3x6_bolt", None) is True
    assert is_fastener_part("some_link", None) is False


def test_fastener_rec_welds_fixed():
    # a flagged edge classifies fixed regardless of its (hinge-looking) mates
    rec = {"types": ["CONCENTRIC"], "axis": None, "mates": [], "fastener": True}
    jt, ax, note = classify_edge_auto(rec)
    assert jt == "fixed" and ax is None and "fastener" in note


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
