import { markHelper } from './bootstrap.js';
import { boxSelected, clearBoxSelection } from './box-select.js';
import { deleteSelectedSubtree } from './bulk-edit.js';
import { reRoot, robotBox, setView } from './camera-reroot.js';
import { alignBtn, bulkbarEl, portBtn, viewer } from './dom.js';
import { cancelEndcoords, ecSetMode, ecTool } from './endcoords-gizmo.js';
import { clearFaceOverlay } from './face-pick.js';
import { _tintLink } from './frames.js';
import { rows } from './joint-rows.js';
import { flipTargetJointAxis, toggleJointFixed } from './joint-type.js';
import { selectLink } from './link-info.js';
import {
  applyLinkColor, pushRecentColor, resetLinkColor,
} from './link-look.js';
import { loadRobot, resetPose, resetView } from './load.js';
import { applyMimic, cancelMimic, enterMimicMode } from './mimic.js';
import { refreshHistory } from './root-frame.js';
import {
  mimicState, packageState, selectionState, viewState,
} from './state.js';
import { THREE } from './three-setup.js';
// ---- batch look (colour / material / reset) over the box selection ----------
// density presets shared with the per-link material picker
const BULK_MAT = [['PLA', '1240'], ['ABS', '1040'], ['PETG', '1270'],
  ['Nylon', '1140'], ['POM', '1410'], ['FR4', '1850'],
  ['Aluminum', '2700'], ['Iron/Steel', '7850'], ['Titanium', '4500'],
  ['custom', 'custom']];
const bulkMatSel = document.getElementById('bulk_mat');
if (bulkMatSel) {
  bulkMatSel.innerHTML =
    `<option value="">${t('ui.bulkMatPlaceholder')}</option>`
    + `<option value="__reset__">${t('ui.bulkMatReset')}</option>`
    + BULK_MAT.map(([label, val]) =>
      `<option value="${val}">${val === 'custom' ? label : label + ' ' + val}` +
      `</option>`).join('');
}

async function bulkSetColor(hex) {       // colour every selected link (no rebuild)
  const links = [...boxSelected];
  if (!links.length) { return; }
  for (const n of links) {
    applyLinkColor(n, hex);              // live
    try {
      const resp = await fetch('/api/set_color', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ link: n, color: hex }) });
      const r = await resp.json();
      if (resp.ok && !r.error) {
        packageState.linkColors[n] = r.color; if (packageState.compMeta[n]) { packageState.compMeta[n].color = r.color; }
      }
    } catch (_e) { /* per-link failure is non-fatal */ }
  }
  pushRecentColor(hex);
  for (const n of boxSelected) { _tintLink(n, 'sel'); }   // keep the box tint
  log(t('bulk.colorOk', { n: links.length, hex }), 'ok');
}

async function bulkResetColor() {        // clear the colour override on selection
  const links = [...boxSelected];
  if (!links.length) { return; }
  for (const n of links) {
    resetLinkColor(n);
    try {
      await fetch('/api/set_color', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ link: n, color: null }) });
    } catch (_e) { /* non-fatal */ }
    delete packageState.linkColors[n]; if (packageState.compMeta[n]) { packageState.compMeta[n].color = null; }
  }
  for (const n of boxSelected) { _tintLink(n, 'sel'); }
  log(t('bulk.colorReset', { n: links.length }), 'ok');
}

async function bulkSetDensity(d) {       // set material density on all (rebuilds)
  const links = [...boxSelected];
  if (!links.length || !packageState.currentInfo) { return; }
  const ctrls = bulkbarEl.querySelectorAll('button, select, input');
  ctrls.forEach(e => { e.disabled = true; });
  log(t('bulk.densitySet', { n: links.length,
                             d: d == null ? t('li.swValue') : d }));
  statusEl.textContent = t('status.rebuilding');
  try {
    for (const n of links) {
      const resp = await fetch('/api/set_material', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ link: n, density: d }) });
      const r = await resp.json();
      if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    }
    log(t('bulk.densityOk', { n: links.length }), 'ok');
    clearBoxSelection();
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('bulk.densityFail', { e: e.message ?? e }), 'err');
  } finally {
    ctrls.forEach(e => { e.disabled = false; });
  }
}

document.getElementById('bulk_color')?.addEventListener('change',
  e => bulkSetColor(e.target.value));
document.getElementById('bulk_resetcolor')?.addEventListener('click',
  bulkResetColor);
bulkMatSel?.addEventListener('change', () => {
  const v = bulkMatSel.value;
  bulkMatSel.selectedIndex = 0;          // reset the picker back to placeholder
  if (v === '') { return; }
  if (v === '__reset__') { bulkSetDensity(null); return; }
  if (v === 'custom') {
    const d = prompt(t('li.densityPrompt'));
    if (d) { bulkSetDensity(Number(d)); }
    return;
  }
  bulkSetDensity(Number(v));
});
window.addEventListener('keydown', ev => {
  // Esc closes the rename / mass-list overlays first -- even when focus is in
  // one of their fields (which would otherwise swallow the key below)
  if (ev.key === 'Escape') {
    const ov = document.getElementById('masslist')
            || document.getElementById('renamelist');
    if (ov) { ev.preventDefault(); ev.stopPropagation(); ov.remove(); return; }
  }
  const ae = document.activeElement;
  const inField = /INPUT|SELECT|TEXTAREA/.test(ae?.tagName)
      || ae?.isContentEditable;
  // a focused field swallows the command shortcuts (you're typing) -- EXCEPT a
  // "move" mode slider/field, where 't' still toggles that joint's type (the
  // slider never consumes the 't' key) so you can drag then press t.
  const playTKey = (ev.key === 't' || ev.key === 'T')
      && ae?.closest?.('#playpanel');
  if (inField && !playTKey) {
    return;                       // typing in a field, not a command
  }
  // Everything below is a bare-key command, so a Ctrl/Cmd chord used to run one
  // as a side effect of the browser's own: Ctrl+C reset the camera on a copy,
  // Ctrl+F EDITED the model.  Guard ctrl/meta only -- shift is deliberate input
  // here ('t' || 'T', and Shift+drag is box-select), nothing binds alt, and
  // Escape only ever cancels (the overlay branch above already takes it under
  // any modifier).
  if ((ev.ctrlKey || ev.metaKey) && ev.key !== 'Escape') { return; }
  if (ecTool) {                   // a placement session owns the shortcuts
    if (ev.key === 'Escape') { cancelEndcoords(); }
    else if (ev.key === 'g' || ev.key === 'G') { ecSetMode('translate'); }
    else if (ev.key === 'r' || ev.key === 'R') { ecSetMode('rotate'); }
    return;
  }
  if (mimicState.mimicMode) {                // a mimic-linking session owns the shortcuts
    if (ev.key === 'Escape') { cancelMimic(); }
    else if (ev.key === 'Enter' || ev.key === 'm' || ev.key === 'M') {
      applyMimic();
    }
    return;
  }
  const target = selectionState.selectedLink ?? selectionState.hoveredLink;
  if ((ev.key === 'Delete' || ev.key === 'Backspace') && selectionState.selectedLink) {
    ev.preventDefault();                // Backspace must not navigate back
    deleteSelectedSubtree();            // remove the link + its whole subtree
  } else if ((ev.key === 't' || ev.key === 'T')) {
    toggleJointFixed();                 // fixed <-> revolute on selected/hovered
  } else if ((ev.key === 'f' || ev.key === 'F')) {
    flipTargetJointAxis();              // reverse + direction on selected/hovered
  } else if ((ev.key === 'm' || ev.key === 'M')) {
    enterMimicMode();                   // start mimic linking from the selection
  } else if (ev.key === '0' || ev.key === 'Home') {
    resetPose();                        // all joints back to 0 (neutral pose)
  } else if (ev.key === 'c' || ev.key === 'C') {
    resetView();                        // whole viewer back to the loaded state
  } else if (ev.key === 'v' || ev.key === 'V') {
    recenterPan();                      // re-centre orbit on model (keep position)
  } else if ((ev.key === 'r' || ev.key === 'R') && target) {
    reRoot(target);
    selectLink(null);
  } else if (ev.key === 'Escape') {
    selectLink(null);              // also dismisses a stale selection bar
    clearBoxSelection();
    alignBtn.classList.remove('active');
    portBtn.classList.remove('active');
    clearFaceOverlay();
  }
});
viewer.addEventListener('dblclick', () => {
  if (!selectionState.hoveredLink) { return; }
  const rec = [...rows.values()].find(r => r.child === selectionState.hoveredLink);
  if (rec) {
    rec.row.scrollIntoView({ block: 'center' });
    rec.row.style.outline = '2px solid #ffe93d';
    setTimeout(() => { rec.row.style.outline = ''; }, 1200);
  }
  // Double-click sets the orbit BASE to this link's frame origin. Right-drag pan
  // then accumulates on top; V snaps back here. setOrbitCenter translates the rig
  // (no rotation) so the link lands dead-centre.
  if (viewer.robot?.links?.[selectionState.hoveredLink]) {
    viewState.orbitBaseLink = selectionState.hoveredLink;
    setOrbitCenter(orbitBasePoint());
    log(t('pivot.focused', { link: selectionState.hoveredLink }));
  }
});

function orbitBasePoint() {
  const link = viewState.orbitBaseLink && viewer.robot?.links?.[viewState.orbitBaseLink];
  if (link) { return link.getWorldPosition(new THREE.Vector3()); }
  const box = robotBox();                    // fallback: robot centre
  return box ? box.getCenter(new THREE.Vector3()) : new THREE.Vector3();
}

// Make a world point the orbit pivot WITHOUT rotating the camera. We translate
// the whole rig (camera + target) by the same delta, so the camera's orientation
// and zoom are untouched (no pitch/yaw) and the point lands exactly where the old
// centred pivot was -- screen centre (0.5,0.5). Requires no-auto-recenter (set
// above) or the target height is clobbered back each frame, which re-introduces
// the pitch. Shared by double-click focus and V.
function setOrbitCenter(p) {
  viewer.camera.position.add(p.clone().sub(viewer.controls.target));
  viewer.controls.target.copy(p);
  viewer.controls.update();
  viewer.redraw();
}

// V: reset the pan offset -- translate the rig back so the orbit BASE (the
// double-clicked link, or the robot centre if none) is dead-centre again. No
// rotation. Lighter than C, which fully reframes to iso.
function recenterPan() {
  if (!viewer.robot) { log(t('fit.emptyBox'), 'err'); return; }
  setOrbitCenter(orbitBasePoint());
  log(viewState.orbitBaseLink ? t('pivot.focused', { link: viewState.orbitBaseLink })
                    : t('pivot.recentered'), 'ok');
}

export function fitView() {
  const box = robotBox();
  if (!box) {
    log(t('fit.emptyBox'), 'err');
    return;
  }
  const size = box.getSize(new THREE.Vector3());
  log(t('fit.bbox', { x: size.x.toFixed(3), y: size.y.toFixed(3),
                      z: size.z.toFixed(3) }));
  if (size.length() > 100) {
    log(t('fit.bigBox'), 'wrn');
  }
  const diag = size.length();
  // tiny non-zero floor: let the camera dolly right inside the assembly to
  // inspect inner parts
  viewer.controls.minDistance = Math.max(diag / 1000, 1e-4);
  setView('iso');
  log(t('fit.ok'), 'ok');
}

// ---- ground grid with a "nice" pitch chosen from the model size ---------
const gridBtn = document.getElementById('grid');
const gridCap = document.getElementById('gridcap');
let gridObj = null;
let gridPitch = null;            // metres; null = auto from model size

const fmtPitch = c => c >= 1 ? `${c.toFixed(0)} m`
                             : `${(c * 1000).toFixed(0)} mm`;
{
  const opt = document.createElement('option');
  opt.value = 'auto';
  opt.textContent = 'auto';
  gridCap.appendChild(opt);
  for (const mm of [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]) {
    const o = document.createElement('option');
    o.value = String(mm);
    o.textContent = `${mm} mm`;
    gridCap.appendChild(o);
  }
}
gridCap.addEventListener('change', () => {
  gridPitch = gridCap.value === 'auto' ? null : Number(gridCap.value) / 1000;
  buildGrid();
});

export function buildGrid() {
  gridObj?.removeFromParent();
  gridObj = null;
  const box = robotBox();
  const diag = box ? box.getSize(new THREE.Vector3()).length() : 0.3;
  const raw = diag / 12;
  const pow = 10 ** Math.floor(Math.log10(raw));
  const autoCell = [1, 2, 5, 10].map(m => m * pow).find(v => v >= raw)
                   ?? 10 * pow;
  const cell = gridPitch ?? autoCell;
  const ext = cell * 30;
  const g = new THREE.Group();
  const minor = new THREE.GridHelper(ext, 30, 0x7d8590, 0xaab0b8);
  const major = new THREE.GridHelper(ext, 3, 0x4a5560, 0x6b7480);
  for (const h of [minor, major]) {
    h.material.transparent = true;
    h.material.opacity = h === major ? 0.8 : 0.45;
    markHelper(h);
    g.add(h);
  }
  viewer.scene.add(g);              // three Y-up scene: y=0 == urdf Z=0
  gridObj = g;
  gridObj.visible = gridBtn.classList.contains('active');
  gridCap.querySelector('option[value="auto"]').textContent =
    `auto (${fmtPitch(autoCell)})`;
  gridCap.value = gridPitch ? String(gridPitch * 1000) : 'auto';
  gridCap.style.display = gridObj.visible ? '' : 'none';
  viewer.redraw();
}
gridBtn.addEventListener('click', () => {
  const on = gridBtn.classList.toggle('active');
  if (gridObj) { gridObj.visible = on; }
  gridCap.style.display = on ? '' : 'none';
  viewer.redraw();
});

