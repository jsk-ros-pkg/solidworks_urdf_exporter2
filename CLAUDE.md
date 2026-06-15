# CLAUDE.md

## Project
sw2robot: a SolidWorks assembly -> URDF exporter with a browser-based editor
(joint types, limits, root frame, ROS/glb export). The SolidWorks extract step
is Windows-only (pywin32 COM); view / edit / build / export run on any OS from a
cached `graph.json`.

## Read first when touching ...
- web editor UI  -> `sw2robot/editor/web/index.html` is one self-contained
  HTML+JS file; every user-facing string goes through the i18n `t()` layer.
- freeze / release -> `build_exe.py` and `.github/workflows/`.

## Commands
- Fast check (run after each change, <20s): `uv run --extra ui --with pytest pytest -q`
- Lint: `uvx ruff@0.15.17 check .`  (config in `pyproject.toml` `[tool.ruff]`)
- Run the editor: `uv run python -m sw2robot.editor.webserver <pkg_dir> --port 8090`
- Build the .exe (Windows): `python build_exe.py --editor-ui`
- Use Python 3.12. Don't run on 3.14: its argparse rejects the webserver's
  `%TEMP%` help string and the server won't start.

## Rules
- Don't translate filesystem paths, package names, or the Google "shared drives"
  mount name in the web UI. Instead translate display strings only, because those
  names carry real on-disk SolidWorks paths resolved server-side.
- Don't disable or fake an assertion to make a check pass. Instead write the real
  check or flag the gap, because green-but-fake tests hide regressions.
- Don't add a fallback that makes broken functionality look like it works.
  Instead surface the failure.
- Don't read, edit, commit, or transmit any `.env` or secret file.

## Conventions
- CI runs on PRs only (lint / test / build-check / e2e); a push to main is an
  already-tested merge. Releases are tag-driven: push a `vX.Y.Z` tag.
- A `feat`/`fix` change ships with its test in the same PR.
