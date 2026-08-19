import { clearBoxSelection } from './box-select.js';
import { highlightJoint } from './bulk-edit.js';
import { pickAxis, pickHit, pickLink } from './camera-reroot.js';
import { alignBtn, hovertip, portBtn, viewer } from './dom.js';
import {
  alignOn, alignRootTo, clearFaceOverlay, faceOverlay, portOn, removePort,
  showFaceOverlay,
} from './face-pick.js';
import { highlightLink, pickPort } from './frames.js';
import { selectLink } from './link-info.js';
import { loadRobot } from './load.js';
import { mimicPick } from './mimic.js';
import { refreshHistory } from './root-frame.js';
import { op } from './session-log.js';
import { mimicState, packageState, selectionState } from './state.js';
import { THREE, TransformControls } from './three-setup.js';
// ===== Blender-style end-coords (port) placement gizmo =====================
// A port/end-coords is a fixed-joint dummy_link; the editor places it with a
// TransformControls gizmo so the user can freely set its offset + rotation and
// a name, then commits via /api/add_port.  The frame is parented UNDER the link
// object, so its LOCAL transform is exactly the port's xyz/rpy in the link frame
// (URDF convention) -- no Y-up<->Z-up juggling.
export let ecTool = null;                 // active placement session, or null

function quatToRpy(q) {             // quaternion -> URDF roll/pitch/yaw (XYZ fixed)
  const e = new THREE.Matrix4().makeRotationFromQuaternion(q).elements;
  const r00 = e[0], r10 = e[1], r20 = e[2], r21 = e[6], r22 = e[10];
  return [Math.atan2(r21, r22),
          Math.atan2(-r20, Math.hypot(r00, r10)),
          Math.atan2(r10, r00)];
}
function rpyToQuat(r, p, y) {
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
}

function startEndcoords(link, originWorld, zdirWorld) {
  cancelEndcoords();
  const lo = viewer.robot?.links?.[link];
  if (!lo) { log(t('port.linkNotFound', { link }), 'err'); return; }
  clearFaceOverlay();
  lo.updateMatrixWorld(true);
  const frame = new THREE.Object3D();
  lo.add(frame);
  frame.position.copy(lo.worldToLocal(originWorld.clone()));
  if (zdirWorld) {                 // orient +Z along the clicked face normal
    const zl = zdirWorld.clone()
      .applyQuaternion(lo.getWorldQuaternion(new THREE.Quaternion()).invert())
      .normalize();
    frame.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), zl);
  }
  const axes = new THREE.AxesHelper(0.05);
  axes.material.depthTest = false;
  axes.material.transparent = true;
  axes.renderOrder = 999;
  frame.add(axes);
  lo.updateMatrixWorld(true);
  const dom = viewer.renderer?.domElement || viewer.querySelector('canvas');
  const tc = new TransformControls(viewer.camera, dom);
  tc.setSize(0.85);
  tc.setSpace('local');
  tc.attach(frame);
  viewer.scene.add(tc);
  tc.addEventListener('dragging-changed',
    e => { viewer.controls.enabled = !e.value; });
  tc.addEventListener('objectChange',
    () => { ecSyncPanelFromFrame(); viewer.redraw?.(); });
  tc.addEventListener('change', () => viewer.redraw?.());
  ecTool = { link, lo, frame, axes, tc, mode: 'translate', space: 'local',
             snap: false, faceSnap: null };
  ecShowPanel();
  ecSyncPanelFromFrame();
  viewer.redraw?.();
  log(t('ec.started', { link }));
}

export function cancelEndcoords() {
  if (!ecTool) { return; }
  const { tc, frame, lo } = ecTool;
  try { tc.detach(); tc.dispose(); viewer.scene.remove(tc); } catch (e) { /**/ }
  try { lo.remove(frame); } catch (e) { /**/ }
  ecTool = null;
  ecHidePanel();
  viewer.controls.enabled = true;
  viewer.redraw?.();
}

async function commitEndcoords() {
  if (!ecTool) { return; }
  ecApplyPanelToFrame();                 // fold in any unsynced numeric edits
  const { link, frame } = ecTool;
  const xyz = frame.position.toArray();
  const rpy = quatToRpy(frame.quaternion);
  const name = ecPanel.querySelector('.ec-name').value.trim();
  const jname = ecPanel.querySelector('.ec-jname').value.trim();
  op('add_port');
  try {
    const resp = await fetch('/api/add_port', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ link, name, joint_name: jname, xyz, rpy }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('ec.placed', { name: r.name || r.parent }), 'ok');
    cancelEndcoords();
    portBtn.classList.remove('active');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('ec.placeFail', { e: e.message ?? e }), 'err');
  }
}

// ---- gizmo option toggles -------------------------------------------------
export function ecSetMode(m) {
  if (!ecTool) { return; }
  ecTool.mode = m;
  ecTool.tc.setMode(m);
  ecPanel.querySelectorAll('.ec-mode').forEach(b =>
    b.classList.toggle('on', b.dataset.m === m));
}
function ecSetSpace(s) {
  if (!ecTool) { return; }
  ecTool.space = s;
  ecTool.tc.setSpace(s);
  ecPanel.querySelectorAll('.ec-space').forEach(b =>
    b.classList.toggle('on', b.dataset.s === s));
}
function ecSetSnap(on) {
  if (!ecTool) { return; }
  ecTool.snap = on;
  ecTool.tc.setTranslationSnap(on ? 0.001 : null);          // 1 mm
  ecTool.tc.setRotationSnap(on ? THREE.MathUtils.degToRad(15) : null);  // 15°
  ecTool.tc.setScaleSnap(on ? 0.1 : null);
  ecPanel.querySelector('.ec-snap').classList.toggle('on', on);
}
function ecArmFaceSnap(kind) {
  if (!ecTool) { return; }
  ecTool.faceSnap = ecTool.faceSnap === kind ? null : kind;
  ecPanel.querySelectorAll('.ec-fsnap').forEach(b =>
    b.classList.toggle('on', b.dataset.k === ecTool.faceSnap));
  log(t(ecTool.faceSnap ? 'ec.snapArmed' : 'ec.snapOff',
        { kind: ecTool.faceSnap || '' }), 'wrn');
}

// snap the frame ORIGIN (and, for a face, +Z) to a clicked surface point
function ecDoFaceSnap(hit) {
  if (!ecTool || !hit?.face) { return; }
  const lo = ecTool.lo;
  let wPoint, nWorld = null;
  if (ecTool.faceSnap === 'vertex') {
    const g = hit.object.geometry, pos = g.attributes.position;
    const idx = [hit.face.a, hit.face.b, hit.face.c];
    let best = null, bd = Infinity;
    for (const i of idx) {
      const v = new THREE.Vector3().fromBufferAttribute(pos, i)
        .applyMatrix4(hit.object.matrixWorld);
      const d = v.distanceTo(hit.point);
      if (d < bd) { bd = d; best = v; }
    }
    wPoint = best;
  } else {                               // face: centre + normal
    if (faceOverlay) {
      wPoint = faceOverlay.host.localToWorld(faceOverlay.centroidLocal.clone());
      nWorld = faceOverlay.normalLocal.clone()
        .transformDirection(faceOverlay.host.matrixWorld);
    } else {
      wPoint = hit.point.clone();
      nWorld = hit.face.normal.clone().transformDirection(hit.object.matrixWorld);
    }
  }
  lo.updateMatrixWorld(true);
  ecTool.frame.position.copy(lo.worldToLocal(wPoint.clone()));
  if (nWorld) {
    const zl = nWorld.clone()
      .applyQuaternion(lo.getWorldQuaternion(new THREE.Quaternion()).invert())
      .normalize();
    ecTool.frame.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), zl);
  }
  ecTool.faceSnap = null;
  ecPanel.querySelectorAll('.ec-fsnap').forEach(b => b.classList.remove('on'));
  clearFaceOverlay();
  ecSyncPanelFromFrame();
  viewer.redraw?.();
}

// ---- panel ----------------------------------------------------------------
let ecPanel = null;
function ecBuildPanel() {
  if (ecPanel) { return ecPanel; }
  ecPanel = document.createElement('div');
  ecPanel.id = 'ecpanel';
  ecPanel.innerHTML =
    `<div class="ec-hd">${t('ec.title')}</div>` +
    `<div class="ec-parent"></div>` +
    `<label class="ec-l">${t('ec.name')}<input class="ec-name" ` +
      `placeholder="end_coords"></label>` +
    `<label class="ec-l">${t('ec.jname')}<input class="ec-jname" ` +
      `placeholder="(auto)"></label>` +
    `<div class="ec-seg">${t('ec.mode')}` +
      `<button class="ec-mode on" data-m="translate">${t('ec.move')}</button>` +
      `<button class="ec-mode" data-m="rotate">${t('ec.rot')}</button></div>` +
    `<div class="ec-seg">${t('ec.space')}` +
      `<button class="ec-space on" data-s="local">${t('ec.local')}</button>` +
      `<button class="ec-space" data-s="world">${t('ec.world')}</button></div>` +
    `<div class="ec-seg">` +
      `<button class="ec-snap">${t('ec.snapStep')}</button>` +
      `<button class="ec-fsnap" data-k="vertex">${t('ec.snapVert')}</button>` +
      `</div>` +
    `<div class="ec-num"><span>X</span><input class="ec-x" type="number" step="1">` +
      `<span>Y</span><input class="ec-y" type="number" step="1">` +
      `<span>Z</span><input class="ec-z" type="number" step="1"><b>mm</b></div>` +
    `<div class="ec-num"><span>R</span><input class="ec-r" type="number" step="1">` +
      `<span>P</span><input class="ec-p" type="number" step="1">` +
      `<span>Y</span><input class="ec-yaw" type="number" step="1"><b>°</b></div>` +
    `<div class="ec-act">` +
      `<button class="ec-place">${t('ec.place')}</button>` +
      `<button class="ec-cancel">${t('ec.cancel')}</button></div>`;
  // inside the viewer pane (left), not the far-right sidebar
  document.getElementById('viewwrap').appendChild(ecPanel);
  ecPanel.querySelectorAll('.ec-mode').forEach(b =>
    b.addEventListener('click', () => ecSetMode(b.dataset.m)));
  ecPanel.querySelectorAll('.ec-space').forEach(b =>
    b.addEventListener('click', () => ecSetSpace(b.dataset.s)));
  ecPanel.querySelector('.ec-snap').addEventListener('click',
    () => ecSetSnap(!ecTool?.snap));
  ecPanel.querySelectorAll('.ec-fsnap').forEach(b =>
    b.addEventListener('click', () => ecArmFaceSnap(b.dataset.k)));
  ecPanel.querySelector('.ec-place').addEventListener('click', commitEndcoords);
  ecPanel.querySelector('.ec-cancel').addEventListener('click', cancelEndcoords);
  ['x', 'y', 'z', 'r', 'p', 'yaw'].forEach(k =>
    ecPanel.querySelector('.ec-' + k).addEventListener('change',
      ecApplyPanelToFrame));
  return ecPanel;
}
function ecShowPanel() {
  ecBuildPanel();
  ecPanel.querySelector('.ec-parent').textContent =
    t('ec.parent', { link: ecTool.link });
  ecPanel.querySelector('.ec-name').value = '';
  ecPanel.querySelector('.ec-jname').value = '';
  ecSetMode('translate'); ecSetSpace('local'); ecSetSnap(false);
  ecPanel.style.display = 'block';
}
function ecHidePanel() { if (ecPanel) { ecPanel.style.display = 'none'; } }
function ecSyncPanelFromFrame() {
  if (!ecTool || !ecPanel) { return; }
  const f = ecTool.frame;
  const mm = v => (Math.abs(v) < 5e-7 ? 0 : v * 1000).toFixed(1);
  const dg = v => { let d = v * 180 / Math.PI;
    if (Math.abs(d) < 0.05) { d = 0; } return d.toFixed(1); };
  ecPanel.querySelector('.ec-x').value = mm(f.position.x);
  ecPanel.querySelector('.ec-y').value = mm(f.position.y);
  ecPanel.querySelector('.ec-z').value = mm(f.position.z);
  const [r, p, y] = quatToRpy(f.quaternion);
  ecPanel.querySelector('.ec-r').value = dg(r);
  ecPanel.querySelector('.ec-p').value = dg(p);
  ecPanel.querySelector('.ec-yaw').value = dg(y);
}
function ecApplyPanelToFrame() {
  if (!ecTool || !ecPanel) { return; }
  const num = c => parseFloat(ecPanel.querySelector('.ec-' + c).value) || 0;
  ecTool.frame.position.set(num('x') / 1000, num('y') / 1000, num('z') / 1000);
  const d2r = d => d * Math.PI / 180;
  ecTool.frame.quaternion.copy(
    rpyToQuat(d2r(num('r')), d2r(num('p')), d2r(num('yaw'))));
  ecTool.lo.updateMatrixWorld(true);
  viewer.redraw?.();
}

// clicking the view moves keyboard focus to it (so R works even after
// typing in the path box) and a no-drag click selects the link
viewer.setAttribute('tabindex', '-1');
let downAt = null;
let ecDownOnGizmo = false;        // did this press start on a gizmo handle?
viewer.addEventListener('pointerdown', ev => {
  viewer.focus();
  downAt = [ev.clientX, ev.clientY];
  // capture it NOW: TransformControls may clear .axis/.dragging before our
  // bubbling pointerup runs, so a no-drag handle click would otherwise re-fit
  ecDownOnGizmo = !!(ecTool && (ecTool.tc.dragging || ecTool.tc.axis));
});
viewer.addEventListener('pointerup', ev => {
  if (!downAt) { return; }
  const moved = Math.hypot(ev.clientX - downAt[0], ev.clientY - downAt[1]);
  downAt = null;
  if (moved > 8 || ev.button !== 0) { return; }    // it was a drag/orbit
  if (mimicState.mimicMode) { mimicPick(ev); return; }        // click = toggle a follower
  if (alignOn()) {
    const ph = pickHit(ev);
    alignBtn.classList.remove('active');
    if (ph?.hit?.face) { alignRootTo(ph.hit); }
    else { log(t('align.cancelNoFace'), 'wrn'); }
    return;
  }
  if (ecTool) {                                    // a placement session is open
    // clicking the gizmo itself (a handle) is for editing -- don't re-fit then
    if (ecDownOnGizmo || ecTool.tc.dragging) { return; }
    // clicking a FACE re-fits the frame to it (origin = face centre, +Z =
    // normal), just like the align tool; vertex if vertex-snap is armed
    const ph = pickHit(ev);
    if (ph?.hit?.face) { ecDoFaceSnap(ph.hit); }
    return;                                         // empty space: leave it be
  }
  if (portOn()) {                                  // click a face -> start gizmo
    const pn = pickPort(ev);                       // an existing port? remove it
    if (pn) { removePort(pn); return; }
    const ph = pickHit(ev);
    if (!ph?.hit?.face) { log(t('port.noFace'), 'wrn'); return; }
    showFaceOverlay(ph.hit);
    let wPoint, nWorld;
    if (faceOverlay) {
      wPoint = faceOverlay.host.localToWorld(faceOverlay.centroidLocal.clone());
      nWorld = faceOverlay.normalLocal.clone()
        .transformDirection(faceOverlay.host.matrixWorld);
    } else {
      wPoint = ph.hit.point.clone();
      nWorld = ph.hit.face.normal.clone()
        .transformDirection(ph.hit.object.matrixWorld);
    }
    startEndcoords(ph.link, wPoint, nWorld);
    return;
  }
  // a visible joint axis under the cursor wins over the mesh behind it, so you
  // can select an occluded joint by clicking its (always-on-top) axis line
  const ax = pickAxis(ev);
  const link = ax ? ax.child : pickLink(ev);
  if (link) { clearBoxSelection(); selectLink(link); }
  else { clearBoxSelection(); selectLink(null); }   // empty space: clear all
});

let lastPick = 0;
viewer.addEventListener('pointermove', ev => {
  if (ev.buttons) {                       // orbiting/dragging: no tooltip
    hovertip.style.display = 'none';
    return;
  }
  const now = performance.now();
  if (now - lastPick < 60) { return; }    // throttle the raycast
  lastPick = now;
  if (alignOn() || portOn()) {            // face-pick mode: no link tint
    const port = portOn() ? pickPort(ev) : null;   // hovering an existing port?
    if (port) {
      clearFaceOverlay();
      const rect = viewer.getBoundingClientRect();
      hovertip.innerHTML = t('hov.removePort', { port });
      hovertip.style.left = (ev.clientX - rect.left + 14) + 'px';
      hovertip.style.top = (ev.clientY - rect.top + 18) + 'px';
      hovertip.style.display = 'block';
      return;
    }
    const ph = pickHit(ev);
    if (ph?.hit?.face) {
      showFaceOverlay(ph.hit);
      const rect = viewer.getBoundingClientRect();
      hovertip.innerHTML = portOn()
        ? t('hov.addPort', { link: ph.link })
        : t('hov.alignFace', { link: ph.link });
      hovertip.style.left = (ev.clientX - rect.left + 14) + 'px';
      hovertip.style.top = (ev.clientY - rect.top + 18) + 'px';
      hovertip.style.display = 'block';
    } else {
      clearFaceOverlay();
      hovertip.style.display = 'none';
    }
    return;
  }
  // a visible joint axis near the cursor is clickable: emphasise it + name it,
  // so the always-on-top axis line reads as an affordance (clears on move-away)
  const ax = pickAxis(ev);
  if (ax) {
    if (selectionState.hoveredAxis !== ax.joint) {
      if (selectionState.hoveredAxis) { highlightJoint(selectionState.hoveredAxis, false); }
      selectionState.hoveredAxis = ax.joint;
      highlightJoint(ax.joint, true);
    }
    if (selectionState.hoveredLink) { selectionState.hoveredLink = null; }
    viewer.style.cursor = 'pointer';
    const rect = viewer.getBoundingClientRect();
    hovertip.innerHTML = t('hov.axis', { joint: ax.joint });
    hovertip.style.left = (ev.clientX - rect.left + 14) + 'px';
    hovertip.style.top = (ev.clientY - rect.top + 18) + 'px';
    hovertip.style.display = 'block';
    return;
  }
  if (selectionState.hoveredAxis) { highlightJoint(selectionState.hoveredAxis, false); selectionState.hoveredAxis = null;
                     viewer.style.cursor = ''; }
  // 3D hover highlighting is the ELEMENT's job (its drag-hover tints the
  // movable subtree); we only track the link name for the tooltip / R key
  const link = pickLink(ev);
  selectionState.hoveredLink = link;
  if (link) {
    const rect = viewer.getBoundingClientRect();
    hovertip.innerHTML = link === selectionState.selectedLink
      ? t('hov.selected', { link })
      : t('hov.link', { link });
    hovertip.style.left = (ev.clientX - rect.left + 14) + 'px';
    hovertip.style.top = (ev.clientY - rect.top + 18) + 'px';
    hovertip.style.display = 'block';
  } else {
    hovertip.style.display = 'none';
  }
});
viewer.addEventListener('pointerleave', () => {
  if (selectionState.hoveredLink) { highlightLink(selectionState.hoveredLink, false); selectionState.hoveredLink = null; }
  if (selectionState.hoveredAxis) { highlightJoint(selectionState.hoveredAxis, false); selectionState.hoveredAxis = null;
                     viewer.style.cursor = ''; }
  hovertip.style.display = 'none';
});

