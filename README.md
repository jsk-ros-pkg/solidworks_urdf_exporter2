# sw2robot

**SolidWorks → robot (URDF) converter.** Turn a SolidWorks assembly into a
URDF, then clean it up in the browser.

One import package, with two subpackages:

- **`sw2robot.exporter`** — the exporter. `extract` opens a throwaway copy of a `.sldasm`
  in a hidden SolidWorks instance and pulls the kinematic graph + per-link
  meshes into `graph.json` (slow, Windows + SolidWorks only). `build` turns that
  `graph.json` into a URDF (fast, headless, no SolidWorks). The original CAD file
  is never modified.
- **`sw2robot.editor`** — a single-page browser editor on top of the graph: re-root the
  tree, change joint types (incl. **Shift+drag box-select** to bulk-set a range),
  edit root frames, set materials/densities, see **live self-collision** as you
  drag, **auto joint limits** from a self-collision sweep, and export a ROS /
  robot-compiler package.

## Install

```bash
pip install -e .            # core: extract / build / web editor (view+edit)
pip install -e ".[ui]"      # + live collision highlight, auto joint-limits, viser GUI
```

`[ui]` adds `scikit-robot` (FK) and `python-fcl` (collision). The editor's
view / edit / extract / build work without it; collision and auto-limits just
report "not available" until it is installed.

## Use

**Extract a `.sldasm` -> URDF** (Windows, with SolidWorks installed):

```bash
python -m sw2robot.exporter.export path/to/assembly.sldasm -o output
```

**Open the browser editor** on an already-extracted package (no SolidWorks
needed — a sample is included):

```bash
python -m sw2robot.editor.webserver examples/fingertip --port 8090
# then open http://localhost:8090
```

From the editor you can also drag-drop a `.sldasm` onto the viewer to extract a
fresh one (it fingerprints the file on disk and drives SolidWorks for you).

**Headless build / edit / export** (no GUI):

```bash
python -m sw2robot.editor            # see the CLI
```

## Layout

```
sw2robot/                one import package (pip install sw2robot)
  exporter/              SolidWorks -> graph.json -> URDF
  editor/                the browser editor (server + single-page web/index.html)
    _vendor/rc_config/   vendored ROS/MoveIt/Gazebo config generators
examples/fingertip/      a small pre-extracted package to try the editor offline
tests/                   pytest (sw2robot.exporter classification) + tests/e2e (puppeteer UI suite)
```

## Tests

```bash
PYTHONPATH= pytest                       # sw2robot.exporter unit tests
cd tests/e2e && npm i && node run.mjs    # UI suite (needs a running sw2robot-web + Chrome)
```

Some pytest fixtures expect a cached `output/<pkg>/graph.json`; those skip when
absent.

## License

[Apache License 2.0](LICENSE) © 2026 Iori Yanokura
