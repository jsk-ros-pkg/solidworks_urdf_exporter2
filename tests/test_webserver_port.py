"""_bind_free_port walks to the next free port instead of crashing.

A second editor instance used to die with WinError 10048 (address in use) on
the default port; now it just comes up on the next free one."""
import socket
import socketserver

import pytest

from sw2robot.editor.webserver import _bind_free_port


class _H(socketserver.BaseRequestHandler):
    pass


def _free_port():
    s = socket.socket()
    s.bind(("", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_uses_requested_port_when_free():
    p = _free_port()
    httpd, bound = _bind_free_port(_H, p)
    try:
        assert bound == p
    finally:
        httpd.server_close()


def test_advances_past_a_busy_port():
    p = _free_port()
    occupied, used = _bind_free_port(_H, p)      # takes p
    try:
        httpd, bound = _bind_free_port(_H, p)    # p busy -> must advance
        try:
            assert used == p
            assert bound > p
        finally:
            httpd.server_close()
    finally:
        occupied.server_close()


def test_raises_when_no_free_port_in_range():
    p = _free_port()
    occupied, _ = _bind_free_port(_H, p)
    try:
        with pytest.raises(OSError):
            _bind_free_port(_H, p, tries=1)      # only port p, which is taken
    finally:
        occupied.server_close()
