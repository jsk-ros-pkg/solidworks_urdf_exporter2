"""An explicitly requested port is honoured or refused -- never silently moved.

Drifting to the next free port is deliberate when nobody named one: it is what
lets a second editor instance come up instead of dying on "address in use".
But when ``--port`` IS given, drifting is a trap.  Whatever already owns the
requested port keeps answering there, so a script that starts a server on 8591
and then talks to 8591 reaches somebody else's server and measures it happily.
"""

import socket
import subprocess
import sys

import pytest

from sw2robot.editor import webserver


def _busy_port():
    """Bind an ephemeral port and keep it held for the duration of the test."""
    s = socket.socket()
    s.bind(("", 0))
    s.listen(1)
    return s, s.getsockname()[1]


def test_explicit_port_in_use_refuses_to_serve():
    held, port = _busy_port()
    try:
        # If this regresses, the server comes up on some OTHER port and runs
        # forever, so the failure is a timeout rather than a non-zero exit --
        # keep it short and report it as the failure it is, not as a hang.
        proc = subprocess.run(
            [sys.executable, "-m", "sw2robot.editor.webserver",
             "--port", str(port), "--no-browser"],
            capture_output=True, text=True, timeout=30,
        )
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode(errors="replace") \
            + (e.stderr or b"").decode(errors="replace")
        pytest.fail(f"still serving after 30s on a port that was taken:\n{out}")
    finally:
        held.close()

    out = proc.stdout + proc.stderr
    assert proc.returncode != 0, f"served anyway on a taken port:\n{out}"
    assert f"port {port} is already in use" in out, out
    # and it must not claim to be serving somewhere else
    assert "open http://localhost:" not in out, out


def _bind_on_busy(**kw):
    """Run _bind against a port that is held, and return (bound, exc)."""
    held, port = _busy_port()
    try:
        httpd, bound = webserver._bind(webserver._Handler, port, **kw)
    except OSError as e:
        return port, None, e
    finally:
        held.close()
    httpd.server_close()
    return port, bound, None


def test_no_port_named_walks_forward():
    """The behaviour we are keeping: nothing named -> take the next free one.

    This goes through _bind, not _bind_free_port: testing the helper alone
    leaves the branch that chooses it untested, and a mutation that made the
    default path strict survived exactly that way.
    """
    port, bound, exc = _bind_on_busy(explicit=False, reclaim_port=False)
    assert exc is None, exc
    assert bound != port


def test_named_port_in_use_raises():
    port, bound, exc = _bind_on_busy(explicit=True, reclaim_port=False)
    assert isinstance(exc, webserver.PortInUse), (bound, exc)
    assert f"port {port} is already in use" in str(exc)


def test_reclaim_still_walks_forward():
    """A self-update relaunch names its port but must still drift: the old
    instance it is replacing has not finished exiting."""
    port, bound, exc = _bind_on_busy(
        explicit=True, reclaim_port=True, wait_first=0.1)
    assert exc is None, exc
    assert bound != port


def test_serve_signature_defaults_to_no_port():
    """``port=None`` is what distinguishes "pick one" from "use this one"."""
    import inspect
    sig = inspect.signature(webserver.serve)
    assert sig.parameters["port"].default is None


def test_port_in_use_is_an_oserror():
    """Callers that already catch OSError around serve() keep working."""
    assert issubclass(webserver.PortInUse, OSError)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
