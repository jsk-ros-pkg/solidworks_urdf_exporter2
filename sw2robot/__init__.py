"""SolidWorks assembly -> URDF, plus a browser editor.

Sub-packages:
- ``sw2robot.exporter``  -- the exporter (SolidWorks .sldasm -> graph.json -> URDF)
- ``sw2robot.editor``   -- the browser editor on top of the graph
"""


def _detect_version() -> str:
    """The running version, single-sourced from ``pyproject.toml``.

    Order: installed package metadata (works in a wheel/pip install AND in the
    frozen .exe, which bundles the dist-info via build_exe.py's
    ``--copy-metadata sw2robot``) -> the repo's pyproject.toml (a bare source
    checkout that was never installed, e.g. the test run with pythonpath=".")
    -> a sentinel.  The self-update check (``sw2robot.editor.update``) compares
    this against the latest GitHub Release tag, so it must reflect the build."""
    try:
        from importlib.metadata import PackageNotFoundError, version
        try:
            return version("sw2robot")
        except PackageNotFoundError:
            pass
    except Exception:
        pass
    try:
        import os
        import tomllib
        pp = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                          "pyproject.toml")
        with open(pp, "rb") as f:
            return tomllib.load(f)["project"]["version"]
    except Exception:
        return "0.0.0+unknown"


__version__ = _detect_version()
