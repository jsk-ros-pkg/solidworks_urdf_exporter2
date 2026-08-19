import { applyPoseDrag, ownLinkBox } from './axis-markers.js';
import { linkSubtree, setExclude } from './bulk-edit.js';
import { pickAxis, pickLink } from './camera-reroot.js';
import { bulkbarEl, viewer } from './dom.js';
import { _tintLink, baseTint } from './frames.js';
import { selectLink } from './link-info.js';
import { op } from './session-log.js';
import { packageState, selectionState } from './state.js';
import { THREE } from './three-setup.js';
// ---- Shift+drag box-select -> bulk joint-type change (Blender-style) -------
// The plain drag is orbit / joint-manipulation, so box-select is Shift+drag.
const boxselEl = document.getElementById('boxsel');
let boxing = null;                 // {x0, y0} while dragging the rectangle
export let boxSelected = new Set();       // link names picked by the box
let boxSelectMode = false;         // toolbar toggle: plain left-drag box-selects
const boxSelectBtn = document.getElementById('boxselect');
boxSelectBtn?.addEventListener('click', () => {
  boxSelectMode = boxSelectBtn.classList.toggle('active');
  viewer.style.cursor = boxSelectMode ? 'crosshair' : '';
});

function drawBox(x, y) {
  const r = viewer.getBoundingClientRect();
  boxselEl.style.left = (Math.min(boxing.x0, x) - r.left) + 'px';
  boxselEl.style.top = (Math.min(boxing.y0, y) - r.top) + 'px';
  boxselEl.style.width = Math.abs(x - boxing.x0) + 'px';
  boxselEl.style.height = Math.abs(y - boxing.y0) + 'px';
}
// capture phase: intercept BEFORE OrbitControls / the joint manipulator
viewer.addEventListener('pointerdown', ev => {
  if (!(ev.shiftKey || boxSelectMode) || ev.button !== 0 || !viewer.robot) {
    return;
  }
  ev.preventDefault();
  ev.stopImmediatePropagation();
  viewer.controls.enabled = false;
  if (viewer.dragControls) { viewer.dragControls.enabled = false; }
  boxing = { x0: ev.clientX, y0: ev.clientY };
  drawBox(ev.clientX, ev.clientY);
  boxselEl.style.display = 'block';
}, true);
window.addEventListener('pointermove', ev => {
  if (boxing) { drawBox(ev.clientX, ev.clientY); }
});
window.addEventListener('pointerup', ev => {
  if (!boxing) { return; }
  const { x0, y0 } = boxing;
  boxing = null;
  boxselEl.style.display = 'none';
  viewer.controls.enabled = true;
  applyPoseDrag();                 // restore the chosen drag mode (orbit/pose)
  if (Math.abs(ev.clientX - x0) < 4 && Math.abs(ev.clientY - y0) < 4) {
    // a Shift+CLICK (no drag): toggle the clicked link in the selection so
    // repeated clicks accumulate, same as a region box-select
    const ax = pickAxis(ev);
    toggleBoxLink(ax ? ax.child : pickLink(ev));
    return;
  }
  selectLinksInBox({ l: Math.min(x0, ev.clientX), r: Math.max(x0, ev.clientX),
                     t: Math.min(y0, ev.clientY), b: Math.max(y0, ev.clientY) });
});

// a link is selected if the centroid of its OWN meshes projects inside the box
export function selectLinksInBox(rect) {
  clearBoxSelection();
  selectLink(null);                          // box-select replaces single-sel
  const r = viewer.getBoundingClientRect();
  viewer.robot.updateMatrixWorld(true);
  const v = new THREE.Vector3();
  for (const [name, link] of Object.entries(viewer.robot.links)) {
    const box = ownLinkBox(link, new THREE.Box3());
    if (box.isEmpty()) { continue; }
    box.getCenter(v).project(viewer.camera);
    if (v.z < -1 || v.z > 1) { continue; }   // behind the camera
    const sx = r.left + (v.x * 0.5 + 0.5) * r.width;
    const sy = r.top + (-v.y * 0.5 + 0.5) * r.height;
    if (sx >= rect.l && sx <= rect.r && sy >= rect.t && sy <= rect.b) {
      boxSelected.add(name);
    }
  }
  for (const n of boxSelected) { _tintLink(n, 'sel'); }
  op('boxSelect', { n: boxSelected.size, names: [...boxSelected] });
  refreshBulkbar();
  log(boxSelected.size ? t('bulk.boxSelected', { n: boxSelected.size })
                       : t('bulk.boxNone'), boxSelected.size ? undefined : 'wrn');
}

// reflect the current box-selection count in the bulkbar (shared by the
// Shift+drag box and the Shift+click accumulation below)
export function refreshBulkbar() {
  if (boxSelected.size) {
    document.getElementById('bulkcount').textContent =
      t('bulk.count', { n: boxSelected.size });
    bulkbarEl.style.display = 'flex';
  } else {
    bulkbarEl.style.display = 'none';
  }
}

// Shift+CLICK a link to ADD/REMOVE it from the selection -- build a multi-select
// one link at a time into the SAME set + bulkbar as a Shift+drag box, so you can
// then bulk-set their joint type together.
export function toggleBoxLink(name) {
  if (!name) { return; }
  if (!boxSelected.size && selectionState.selectedLink) { selectLink(null); }  // box replaces single
  if (boxSelected.has(name)) {
    boxSelected.delete(name);
    _tintLink(name, baseTint(name));
  } else {
    boxSelected.add(name);
    _tintLink(name, 'sel');
  }
  op('boxToggle', { name, n: boxSelected.size });
  refreshBulkbar();
}

export function clearBoxSelection() {
  // clear the set FIRST: baseTint() now reports box-selected links as 'sel', so
  // restoring their tint must happen once they are no longer in the set
  const prev = boxSelected;
  boxSelected = new Set();
  for (const n of prev) {
    if (n !== selectionState.selectedLink) { _tintLink(n, baseTint(n)); }
  }
  bulkbarEl.style.display = 'none';
}
document.getElementById('bulkclose').addEventListener('click', clearBoxSelection);

// delete every box-selected link AND its subtree in one rebuild -- mirrors the
// sidebar 🗑 bulk delete, but over the 3D box selection.  Ctrl+Z restores.
document.getElementById('bulkbardel').addEventListener('click', () => {
  const sel = [...boxSelected];
  if (!sel.length || !packageState.currentInfo) { return; }
  const links = new Set();
  for (const c of sel) { for (const n of linkSubtree(c)) { links.add(n); } }
  const comps = [...new Set([...links].map(n => packageState.compMeta[n]?.name ?? n))];
  if (!confirm(t('bulk.delConfirm', { n: sel.length }))) { return; }
  clearBoxSelection();
  selectLink(null);
  setExclude({ names: comps, on: true }, t('bulk.deleting', { n: sel.length }));
});

