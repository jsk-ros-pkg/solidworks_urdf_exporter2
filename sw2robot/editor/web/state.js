// ---- cross-module shared state ------------------------------------------
// A LEAF module: it imports nothing, so ES module evaluation always finishes it
// before any importer runs.  That is what makes an evaluation-time read of one
// of these objects safe inside the split's import cycles.
//
// Every top-level `let` of the old inline module script that was ASSIGNED from
// more than one `// ----` section lives here.  Section banners were the finest
// structural unit in that block and every module boundary landed on one, so
// "assigned from two sections" is a superset of "assigned from two modules".
//
// Why it matters: as an ES module, such a `let` would have to be
// `export let`, and an import binding is READ-ONLY -- modules are always strict,
// so a cross-module assignment throws "TypeError: Assignment to constant
// variable".  Reads stay live either way; only writes break.  Collecting them
// into named `const` objects makes the write legal, and makes a use site read as
// "this is state that outlives the module".
//
// Field names are deliberately IDENTICAL to the old top-level `let` names, so
// the change that introduced them is one mechanical rewrite (`x` -> `group.x`)
// that can be checked line by line.  Renaming (e.g. mimicState.mimicMode ->
// mimicState.active) is a readability change and belongs in its own commit.
//
// Variables assigned from only ONE section stay plain `let` in the module that
// owns them, and `export let` is fine for the read-only cross-module reads
// (THREE, ecTool, boxSelected, faceOverlay, ...).

// The package currently open in the editor plus the sidecar metadata that came
// with it.  Written by the loader / the extractor; read by nearly every panel.
export const packageState = {
  dropMode: false,          // meshes come from dropped blobs, not /pkg/
  currentInfo: null,        // {name, urdf} of the open server package
  compMeta: {},             // link -> {material, density, name, override}
  linkColors: {},           // URDF/viewer link name -> '#rrggbb' override
  excludedList: [],         // component names excluded from the URDF
  massOnlyLinks: new Set(), // links flagged mass-only (weight kept, no mesh)
  rootBaseName: null,       // which COMPONENT is base_link right now
                            // (the URDF rename hides it)
};

// What the 3D view is currently showing.
export const viewState = {
  mergedView: false,        // viewer shows the fixed-joint-lumped URDF
  tfNodes: [],              // per-link TF triads + parent->child lines
  // The orbit BASE: null = robot bbox centre (default / after C); otherwise a link
  // whose frame origin we orbit. Set by double-click, cleared by C. Right-drag pan
  // rides on top of this (controls.target = base + pan); V removes the pan.
  orbitBaseLink: null,
  // Drag-to-articulate mode: false = left-drag orbits (default), true =
  // left-drag moves the joint under the cursor.  Promoted out of the drag-
  // mode section so the material-restore hover handler can read it from a
  // leaf instead of importing it back and closing a cycle.
  poseDrag: false,
};

// The sidebar tree / joint-list view controls.
export const treeState = {
  // when on, the joint list shows ONLY the joints the classifier flagged as
  // guesses (compMeta[...].joint_attention) -- the review worklist
  jointCheckOnly: false,
  jointFilter: '',          // substring filter: when set, the tree collapses
                            // to a FLAT list of joints whose link/joint name
                            // matches -- so select-all / Set / Delete scope
                            // to exactly the matches (see buildJointRows)
  treeViewMode: 'expanded', // expanded = editable URDF tree; subassembly =
                            // read-only collapse_preview tree
  robotViewMode: 'normal',  // normal | collapsed_preview
  pendingRobotViewMode: null,
  subasmTreeRequest: 0,     // discard stale async preview responses
  playMode: false,          // movable-joints-only panel active
};

// What the user has selected / hovered, and the UI hooks that follow it.
export const selectionState = {
  selectedLink: null,
  hoveredLink: null,
  hoveredAxis: null,        // joint name whose axis the cursor is near
  selVis: null,             // selection visuals: link frame triad + CoM marker
  selAxisJoint: null,       // joint whose axis is force-shown while its
                            // child link is selected (incl. fixed joints)
  // while the colour picker for a link is open we suppress that link's tint
  // (selection cyan etc.) so the user judges the TRUE colour, not the overlay
  colorPreviewLink: null,
  // the in-viewer joint editor's live-sync hook, set while #linkinfo shows a
  // movable joint: {name, set(nativeVal)} so the sidebar slider and the overlay
  // slider always show the same angle.
  jpSync: null,
  // link to re-select after the next edit-rebuild reload, so the joint panel
  // stays open on the same link after an immediate type change
  reselectAfterLoad: null,
};

// Live self-collision query, plus the CoACD collision-preview handshake that
// hands its results to the same tinting path.
export const collisionState = {
  collisionLinks: new Set(),   // links in a NEW (non-rest) self-collision
  colReady: false,
  colBusy: false,
  colQueued: false,
  colPoll: 0,
  collPreviewFinalized: false, // ran the post-generation hookup (export + collision)
};

// The mimic-linking session (master joint + follower joints).
export const mimicState = {
  mimicMode: false,
  mimicMaster: null,        // master joint name (the driver)
  mimicMasterChild: null,   // master's child link (can't follow itself)
};

// The edit-rebuild reload handshake: the viewer is torn down and reloaded, so
// what has to survive that gap lives here.
export const loadState = {
  pendingRestore: null,     // {angles} saved across an edit-rebuild reload
  loadCover: null,          // visual clone shown while the URDF reloads,
                            // so re-root / type edits never flash blank
};

// Replay (②) re-performs a recorded op() stream; the camera-track sampler and
// the recorders must not record the playback itself.
export const replayState = {
  _replaying: false,        // true while replayLog() re-performs a session
};
