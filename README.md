# sw2robot

**SolidWorks → robot (URDF) converter.** Turn a SolidWorks assembly into a
URDF, then clean it up in the browser.

https://github.com/user-attachments/assets/d821525c-d1e2-4a33-8bbf-fea42ba12434

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

## Quick start — download the Windows `.exe` (no Python)

If you just want to convert a SolidWorks assembly, you don't need to install
Python or clone this repo. Grab the prebuilt editor and run it:

1. **Download.** Go to the
   [latest release](https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2/releases/latest)
   and download `sw2robot-web-windows-x64-v<version>.exe` (under *Assets*).
   It's a single self-contained file — nothing to install.
2. **Run it.** Double-click the `.exe`. A console window opens, a local server
   starts, and your default browser opens the editor at
   `http://localhost:8090` automatically.
   - Windows SmartScreen may warn that the publisher is unknown (the binary is
     unsigned). Click **More info → Run anyway**.
   - To pick a different port or skip auto-opening the browser, run it from a
     terminal: `sw2robot-web-windows-x64-v<version>.exe --port 8090 --no-browser`.
3. **Extract a robot.** Open your `.sldasm` assembly with the in-app file
   picker — the **🗄 file browser** (lists SolidWorks' recent files for
   one-click access) or **📋 paste a full path**. SolidWorks needs the real
   on-disk path to resolve referenced parts, which a browser never exposes for
   a drag-and-dropped file, so the editor opens by path rather than by drop.
   The app drives a hidden SolidWorks instance to pull the kinematic graph +
   meshes. Your original CAD file is never modified. Extracted packages are
   written to `%TEMP%\sw2robot\output`.
   - **The extract step needs SolidWorks installed on the same machine** — the
     app talks to it over COM; it does not embed SolidWorks.
   - No SolidWorks? You can still open and edit an already-extracted package,
     and the build / export steps work without it.
4. **Clean it up.** In the editor: re-root the tree, set joint types
   (Shift+drag to bulk-set a range), edit the root frame, set materials and
   densities, watch live self-collision as you drag joints, and run the
   auto joint-limit sweep.
5. **Export.** Export a ROS / robot-compiler package (URDF + meshes + configs)
   from the editor, ready to drop into your workspace.

Closing the console window stops the server and tears down the SolidWorks
instance it spawned.

## Editor — features & keyboard shortcuts

Everything below edits the package server-side and rebuilds the URDF in place
(~0.5 s), so the viewer always shows the real exported result.

**Kinematics**

- **Make root** — pick any link and make it the base; the tree is re-rooted and
  every edge between the old and new root is flipped automatically.
- **Joint types** — toggle a joint fixed ↔ movable, or **Shift+drag** a box over
  the tree to bulk-set a whole range at once.
- **Flip axis** — reverse a joint's positive direction in one keystroke.
- **Mimic** — link follower joints to a master so they move together; set a
  **multiplier** and **offset** per follower (URDF `<mimic>`). Move the master
  and the followers track it live.
- **Delete subtree** — drop a link and everything below it.

**Coordinate frames**

- **Root frame** — rotate the root about its current axes or type exact numbers;
  or click a face to **align the root** to it (origin = face center). Written as
  `root_rpy` / `root_xyz` in `joints.yaml`.
- **⊕ Port (end-coords)** — click a face to drop a named, coordinate-only link
  there with **+Z = the face normal**, then nudge it with the gizmo
  (`g` move / `r` rotate) and **Place**. Handy for end-effector, sensor, or
  mount frames. Click a magenta marker to remove one. Stored under `ports:` in
  `joints.yaml`.

**Authoring aids** — live self-collision highlight as you drag a joint, an
**auto joint-limit** sweep, per-link materials/densities, a `tf` view (frame
triads + parent links), and a sizeable ground grid.

**Keyboard shortcuts** (with a link selected or hovered):

| Key | Action |
| --- | --- |
| `t` | toggle joint **fixed ↔ movable** |
| `f` | **flip** the joint axis / direction |
| `m` | start **mimic** linking (then click followers · `Enter`/`m` apply · `Esc` cancel) |
| `r` / `R` | **make root** at this link |
| `Del` / `Backspace` | delete the link **+ its subtree** |
| `0` / `Home` | reset **pose** (all joints to 0) |
| `c` | reset the **view** |
| `Esc` | clear selection / cancel the current mode |
| `click` · `dbl-click` · `Shift+drag` | select · jump to its tree row · box-select a range |
| mouse | left-drag orbit · right-drag pan · wheel zoom |

In a port/end-coords placement session the gizmo owns the keys: `g` move,
`r` rotate, `Esc` cancel.

## Install from source (developers)

Prefer this if you want to hack on sw2robot or run it on a non-Windows machine
(view / edit / build / export work anywhere; only *extract* needs Windows +
SolidWorks).

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

From the editor you can also extract a fresh package: open a `.sldasm` with the
in-app file picker (🗄 file browser of SolidWorks' recent files, or 📋 paste a
full path) and it drives SolidWorks for you. It opens by real on-disk path —
not by browser drag-and-drop, which never exposes the path SolidWorks needs to
resolve referenced parts.

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
