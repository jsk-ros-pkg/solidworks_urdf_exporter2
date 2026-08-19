import { originOn, setAxesVisible } from './axis-markers.js';
import { markHelper } from './bootstrap.js';
import { boxSelected } from './box-select.js';
import { robotBox } from './camera-reroot.js';
import { originBtn, tfBtn, viewer } from './dom.js';
import { _colorTint } from './link-look.js';
import { mimicFollowers } from './selection.js';
import {
  collisionState, mimicState, selectionState, viewState,
} from './state.js';
import { THREE } from './three-setup.js';
// ---- RGB triads (red=X green=Y blue=Z) -----------------------------------
export function makeTriad(len, r) {
  const triad = new THREE.Group();
  const DIRS = [[[1, 0, 0], 0xe53935], [[0, 1, 0], 0x43a047],
                [[0, 0, 1], 0x1e88e5]];
  for (const [d, color] of DIRS) {
    const dir = new THREE.Vector3(...d);
    const mat = new THREE.MeshBasicMaterial({
      color, depthTest: false, transparent: true, opacity: 0.95 });
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(r, r, len, 10), mat);
    shaft.position.copy(dir.clone().multiplyScalar(len / 2));
    shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    shaft.renderOrder = 998;
    markHelper(shaft);
    const tip = new THREE.Mesh(
      new THREE.CylinderGeometry(0, r * 3, len * 0.14, 12), mat);
    tip.position.copy(dir.clone().multiplyScalar(len * 1.07));
    tip.quaternion.copy(shaft.quaternion);
    tip.renderOrder = 998;
    markHelper(tip);
    triad.add(shaft, tip);
  }
  return triad;
}

export let originTriad = null;

export function addOriginTriad(robot) {
  if (originTriad) { originTriad.removeFromParent(); originTriad = null; }
  const box = robotBox();
  const diag = box ? box.getSize(new THREE.Vector3()).length() : 0.3;
  originTriad = makeTriad(Math.max(diag * 0.32, 0.04),
                          Math.max(diag * 0.0035, 0.0008));
  robot.add(originTriad);           // root-link frame, follows the model
  originTriad.visible = originOn();
}

// ---- coordinate-only links (dummy_link ports): a mesh-less link is invisible,
// so draw a magenta sphere + a frame triad at each, and make the sphere a
// click target so a mis-placed port can be removed --------------------------
let portNodes = [];                 // marker groups, cleared each reload
let portPickTargets = [];           // the clickable spheres (raycast to remove)
const portSphereMat = new THREE.MeshBasicMaterial({
  color: 0xff3df0, depthTest: false, transparent: true, opacity: 0.9 });

export function clearPortMarkers() {
  portNodes.forEach(o => o.removeFromParent());
  portNodes = [];
  portPickTargets = [];
}

export function addPortMarkers(robot) {
  clearPortMarkers();
  if (!robot) { return; }
  const box = robotBox();
  const diag = box ? box.getSize(new THREE.Vector3()).length() : 0.3;
  const len = Math.max(diag * 0.1, 0.02);
  const r = Math.max(diag * 0.0035, 0.0008);
  for (const [name, link] of Object.entries(robot.links)) {
    if (!/^dummy_link/.test(name)) { continue; }    // robot-compiler port
    const g = new THREE.Group();
    g.add(makeTriad(len, r));       // RGB axes: +Z (blue) = outgoing connector
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(Math.max(diag * 0.012, 0.003), 16, 12),
      portSphereMat);
    sphere.renderOrder = 999;
    sphere.userData.portName = name;
    markHelper(sphere);             // ignored by link raycast / pose-drag
    g.add(sphere);
    markHelper(g);
    link.add(g);
    portNodes.push(g);
    portPickTargets.push(sphere);
  }
  if (portPickTargets.length) { viewer.redraw(); }
}

// the markers are markHelper'd (inert to the link raycast + pose-drag), so we
// can't raycast them; pick by screen-space proximity to each port's center
export function pickPort(ev) {
  if (!portPickTargets.length || !viewer.robot) { return null; }
  const rect = viewer.getBoundingClientRect();
  const mx = ev.clientX - rect.left, my = ev.clientY - rect.top;
  const v = new THREE.Vector3();
  let best = null, bestD = 18;          // px hit radius
  for (const s of portPickTargets) {
    s.getWorldPosition(v).project(viewer.camera);
    if (v.z > 1) { continue; }          // behind the camera
    const sx = (v.x * 0.5 + 0.5) * rect.width;
    const sy = (-v.y * 0.5 + 0.5) * rect.height;
    const d = Math.hypot(sx - mx, sy - my);
    if (d < bestD) { bestD = d; best = s.userData.portName; }
  }
  return best;
}

// ---- TF view: a small triad on EVERY link + parent->child lines, with the
// meshes dimmed (rviz-style) so the frame tree is point-at-able ------------
export function tfOn() { return tfBtn.classList.contains('active'); }
const dimmedMats = new Map();   // material -> original {transparent,opacity}

function buildTF(robot) {
  viewState.tfNodes.forEach(o => o.removeFromParent());
  viewState.tfNodes = [];
  const box = robotBox();
  const diag = box ? box.getSize(new THREE.Vector3()).length() : 0.3;
  const len = Math.max(diag * 0.045, 0.012);
  const r = Math.max(diag * 0.0014, 0.0004);
  const rootName = (() => {       // the link that is never a child
    const kids = new Set(Object.values(robot.joints).map(j =>
      [...(j.urdfNode?.children ?? [])]
        .find(el => el.tagName === 'child')?.getAttribute('link')));
    return Object.keys(robot.links).find(n => !kids.has(n));
  })();
  for (const [name, link] of Object.entries(robot.links)) {
    // the ROOT frame is the origin of the whole TF tree: draw it 2.5x
    const big = name === rootName;
    const t = makeTriad(len * (big ? 2.5 : 1), r * (big ? 2 : 1));
    link.add(t);
    viewState.tfNodes.push(t);
  }
  // parent -> child connectors WITH direction (cone at the child end),
  // so the tree visibly flows outward from the root
  const lineMat = new THREE.LineBasicMaterial({
    color: 0xbababa, depthTest: false, transparent: true, opacity: 0.8 });
  const coneMat = new THREE.MeshBasicMaterial({
    color: 0xffd24d, depthTest: false, transparent: true, opacity: 0.9 });
  for (const j of Object.values(robot.joints)) {
    if (!j.parent?.isURDFLink) { continue; }
    const to = j.position.clone();
    const d = to.length();
    if (d < 1e-6) { continue; }   // coincident frames: nothing to draw
    const geo = new THREE.BufferGeometry().setFromPoints(
      [new THREE.Vector3(0, 0, 0), to]);
    const line = new THREE.Line(geo, lineMat);
    line.renderOrder = 997;
    markHelper(line);
    j.parent.add(line);
    viewState.tfNodes.push(line);
    const cone = new THREE.Mesh(
      new THREE.CylinderGeometry(0, r * 3.2, Math.min(d * 0.25, len * 0.5),
                                 10), coneMat);
    const dir = to.clone().normalize();
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    cone.position.copy(to.clone().sub(
      dir.clone().multiplyScalar(Math.min(d * 0.125, len * 0.25))));
    cone.renderOrder = 997;
    markHelper(cone);
    j.parent.add(cone);
    viewState.tfNodes.push(cone);
  }
}

export function undimAll() {
  // materials are SHARED with the mesh memory cache, so a dim that is not
  // restored survives every reload as a permanently dark part -- always
  // restore before clearing
  dimmedMats.forEach((orig, m) => {
    m.transparent = orig.t;
    m.opacity = orig.o;
  });
  dimmedMats.clear();
}

export function setTF(on) {
  if (on && !viewState.tfNodes.length && viewer.robot) { buildTF(viewer.robot); }
  viewState.tfNodes.forEach(o => { o.visible = on; });
  if (on) {
    viewer.robot?.traverse(c => {
      if (c.isMesh && !c.userData.sw2robotMarker && c.material
          && !dimmedMats.has(c.material)) {
        dimmedMats.set(c.material, { t: c.material.transparent,
                                     o: c.material.opacity });
        c.material.transparent = true;
        c.material.opacity = 0.22;
      }
    });
  } else {
    undimAll();
  }
  viewer.redraw();
}
tfBtn.addEventListener('click', () => setTF(tfBtn.classList.toggle('active')));
originBtn.addEventListener('click', () => {
  const on = originBtn.classList.toggle('active');
  if (originTriad) { originTriad.visible = on; }
  viewer.redraw();
});
document.getElementById('axes').addEventListener('change',
  e => setAxesVisible(e.target.checked));

// link tinting: hover = amber glow, selected = cyan glow.  The ORIGINAL
// material is CLONED and given an emissive boost, so the part's own
// colours and shading stay visible (a flat replacement read as black)
export const tintClones = new Map();   // original material -> {hov, sel, col}
export const savedMats = new Map();    // mesh -> original material

const TINT_GLOW = { sel: 0x0e5d80, hov: 0x7a5500, col: 0x8a0e0e,
                    mim: 0x4a2e8a, mas: 0x267a3a };
const TINT_FLAT = { sel: 0x55d6ff, hov: 0xffc94d, col: 0xff3030,
                    mim: 0xb38cff, mas: 0x66e08a };

function _tintedClone(orig, kind) {
  let entry = tintClones.get(orig);
  if (!entry) { entry = {}; tintClones.set(orig, entry); }
  if (!entry[kind]) {
    const m = orig.clone();
    if ('emissive' in m) {
      m.emissive = new THREE.Color(TINT_GLOW[kind]);
      m.emissiveIntensity = kind === 'col' ? 1.4 : 1.0;
    } else {                      // e.g. MeshBasicMaterial: blend the color
      m.color = m.color.clone().lerp(
        new THREE.Color(TINT_FLAT[kind]), kind === 'col' ? 0.65 : 0.5);
    }
    entry[kind] = m;
  }
  return entry[kind];
}

// One of TWO top-level 'joint-mouseover' listeners on the viewer; the other
// (joint-axis markers) emphasises the hovered joint's axis.  Registration order
// is whatever the import graph produces once this block is split, and it does
// not decide anything because the two touch DISJOINT objects: this one walks
// the robot's LINK meshes and skips `userData.sw2robotMarker`, the other
// touches only marker meshes + the sidebar row.  That skip is the invariant.
viewer.addEventListener('joint-mouseover', e => {
  // the element just swapped every hovered mesh to its flat highlightMaterial
  // and stashed the real one in __origMaterial.  pose mode: replace it with an
  // emissive amber CLONE (glow + keep colours).  orbit mode (default): restore
  // the original outright, so a plain hover never lights the part up.  Walk the
  // whole robot by __origMaterial so it works regardless of e.detail.
  viewer.robot?.traverse(c => {
    if (c.isMesh && !c.userData.sw2robotMarker && c.__origMaterial) {
      c.material = viewState.poseDrag ? _tintedClone(c.__origMaterial, 'hov')
                            : c.__origMaterial;
    }
  });
  viewer.redraw();
});

// the tint a link should wear when neither hovered nor freshly (de)selected:
// collision red wins over the selection blue
export function baseTint(name) {
  if (mimicState.mimicMode) {                 // mimic session: master=green, follower=purple
    if (name === mimicState.mimicMasterChild) { return 'mas'; }
    if (mimicFollowers.has(name)) { return 'mim'; }
  }
  return collisionState.collisionLinks.has(name) ? 'col'
       : (name === selectionState.selectedLink || boxSelected.has(name) ? 'sel' : null);
}

export function _tintLink(linkName, kind) {   // kind: 'hov' | 'sel' | null
  const link = viewer.robot?.links?.[linkName];
  if (!link) { return; }
  (function walk(node) {
    for (const c of node.children) {
      if (c.isURDFJoint) { continue; }     // own meshes only, not subtree
      if (c.isMesh && !c.userData.sw2robotMarker) {
        // the element's drag-hover keeps the pre-hover material in
        // __origMaterial and restores it on mouseout.  While that dance is
        // in progress, OUR tint must read/write __origMaterial (not
        // c.material) or the true original gets lost and the tint sticks
        // forever after deselect (the classic select -> hover -> Esc bug)
        const hovering = c.__origMaterial !== undefined;
        if (kind) {
          if (hovering) {
            if (!savedMats.has(c)) { savedMats.set(c, c.__origMaterial); }
            c.__origMaterial = _tintedClone(savedMats.get(c), kind);
          } else if (c.material !== viewer.highlightMaterial) {
            if (!savedMats.has(c)) { savedMats.set(c, c.material); }
            c.material = _tintedClone(savedMats.get(c), kind);
          }
        } else if (savedMats.has(c)) {
          const orig = savedMats.get(c);
          if (hovering) {
            c.__origMaterial = orig;        // element restores it for us
          } else if (c.material !== viewer.highlightMaterial) {
            c.material = orig;
          }
          savedMats.delete(c);
        }
      }
      walk(c);
    }
  })(link);
  viewer.redraw();
}

// hover tint, but never repaint over the selection
export function highlightLink(linkName, on) {
  if (linkName === selectionState.selectedLink) { return; }
  _tintLink(linkName, on ? 'hov' : _colorTint(linkName));
}

