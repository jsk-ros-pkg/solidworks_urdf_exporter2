import { axisGlyphs, axisMarkers, dirsOn } from './axis-markers.js';
import { viewer } from './dom.js';
import { rows } from './joint-rows.js';
import { applyTypeChanges } from './joint-type.js';
import { selectLink } from './link-info.js';
import { loadRobot } from './load.js';
import { refreshHistory } from './root-frame.js';
import { _markerRestVisible } from './selection.js';
import { op } from './session-log.js';
import {
  packageState, selectionState, treeState, viewState,
} from './state.js';
import { buildJointRows, syncTreeModeControls } from './tree.js';
// 🗑 = REALLY remove from the built URDF (yaml exclude:), not just hide
export async function setExclude(body, what) {
  op('exclude', body);
  log(t('common.rebuilding', { what }));
  try {
    const resp = await fetch('/api/set_exclude', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('excl.now', { n: r.excluded.length }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('excl.fail', { e: e.message ?? e }), 'err');
  }
}
document.getElementById('selexclude').addEventListener('click', () => {
  if (!selectionState.selectedLink || !packageState.currentInfo) { return; }
  const comp = packageState.compMeta[selectionState.selectedLink]?.name ?? selectionState.selectedLink;
  const l = selectionState.selectedLink;
  selectLink(null);
  setExclude({ name: comp, on: true }, t('excl.excluding', { l }));
});
// the 🗑 chip opens a popover listing each excluded (deleted) link with an
// individual ↩ restore, plus a "restore all" -- so a single deletion can be
// undone without restoring everything (the list is loaded from joints.yaml, so
// persisted deletions show up even after reopening a cached package)
const _exclChip = document.getElementById('exclchip');
const _exclPop = document.getElementById('exclpop');
function showExclPopover() {
  if (!packageState.excludedList.length) { _exclPop.style.display = 'none'; return; }
  const esc = s => String(s).replace(/[&<>"]/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
  const rows = packageState.excludedList.map(n =>
    '<div style="display:flex;gap:6px;align-items:center;'
    + 'justify-content:space-between;padding:2px 0">'
    + `<span style="overflow:hidden;text-overflow:ellipsis;white-space:nowrap"`
    + ` title="${esc(n)}">${esc(n)}</span>`
    + `<button data-restore="${esc(n)}" style="flex:none">↩</button></div>`
  ).join('');
  _exclPop.innerHTML =
    `<div style="color:#8a93a3;margin-bottom:4px">`
    + `${t('excl.popTitle', { n: packageState.excludedList.length })}</div>${rows}`
    + '<div style="border-top:1px solid #456;margin-top:4px;padding-top:4px">'
    + `<button id="exclrestoreall" style="width:100%">`
    + `${t('excl.restoreAllBtn')}</button></div>`;
  const r = _exclChip.getBoundingClientRect();
  _exclPop.style.left = Math.round(r.left) + 'px';
  _exclPop.style.top = Math.round(r.bottom + 4) + 'px';
  _exclPop.style.display = 'block';
  _exclPop.querySelectorAll('button[data-restore]').forEach(b =>
    b.addEventListener('click', () => {
      _exclPop.style.display = 'none';
      setExclude({ names: [b.dataset.restore], on: false },
                 t('excl.restoringOne', { n: b.dataset.restore }));
    }));
  document.getElementById('exclrestoreall').addEventListener('click', () => {
    _exclPop.style.display = 'none';
    setExclude({ clear: true }, t('excl.restoring'));
  });
}
_exclChip.addEventListener('click', e => {
  e.stopPropagation();
  if (_exclPop.style.display === 'block') { _exclPop.style.display = 'none'; }
  else { showExclPopover(); }
});
document.addEventListener('click', e => {
  if (_exclPop.style.display === 'block'
      && !_exclPop.contains(e.target) && e.target !== _exclChip) {
    _exclPop.style.display = 'none';
  }
});

// a link + every descendant link under it (the subtree to delete together)
export function linkSubtree(linkName) {
  const root = viewer.robot?.links?.[linkName];
  const out = [];
  if (!root) { return out; }
  (function walk(link) {
    out.push(link.name);
    for (const j of link.children) {
      if (j.isURDFJoint) {
        for (const cl of j.children) { if (cl.isURDFLink) { walk(cl); } }
      } else if (j.isURDFLink) { walk(j); }   // defensive: directly nested link
    }
  })(root);
  return out;
}
// Delete key: remove the selected link AND its whole subtree from the URDF
// (children would otherwise dangle).  Excluded as one rebuild; Ctrl+Z restores.
export function deleteSelectedSubtree() {
  if (!selectionState.selectedLink || !packageState.currentInfo) { return; }
  const links = linkSubtree(selectionState.selectedLink);
  if (!links.length) { return; }
  const comps = [...new Set(links.map(n => packageState.compMeta[n]?.name ?? n))];
  const l = selectionState.selectedLink, n = links.length;
  selectLink(null);
  setExclude({ names: comps, on: true }, t('del.deleting', { l, n }));
}

// the arrow + rotation arc clutters the view if shown on every joint at once,
// so it rides hover (and selection): only the joint under the cursor gets it.
function showJointGlyph(name, on) {
  const gl = axisGlyphs.get(name);
  if (gl) { gl.visible = on || name === selectionState.selAxisJoint || dirsOn(); }
}

export function highlightJoint(name, on) {
  const m = axisMarkers.get(name);
  if (m && m.ghost) {
    // fixed joint: reveal the would-be axis while hovering (or kept on if
    // this joint's link is the current selection)
    m.mesh.visible = on || _markerRestVisible(name, m);
    m.mesh.material.color.set(on ? 0xffe93d : m.baseColor);
    m.mesh.scale.set(on ? 2.0 : 1, 1, on ? 2.0 : 1);
  } else if (m) {
    // show the axis while hovering even when the global toggle is off.
    // keep the rod its joint-type COLOUR (no yellow recolour): the +direction
    // glyph that appears on hover is the signal, and a yellow rod just competes
    // with it -- only thicken + solidify the rod a touch to locate it
    m.mesh.visible = on ? true : _markerRestVisible(name, m);
    m.mesh.material.color.set(m.baseColor);
    m.mesh.material.opacity = on ? 1.0 : 0.85;
    m.mesh.scale.set(on ? 1.8 : 1, 1, on ? 1.8 : 1);
  }
  showJointGlyph(name, on);          // reveal the +direction glyph while hovering
  const rec = rows.get(name);
  if (rec) {
    rec.row.style.outline = on ? '1px solid #ffe93d' : '';
    if (on) { rec.row.scrollIntoView({ block: 'nearest' }); }
  }
  viewer.redraw();
}
// hovering the MODEL emphasises the joint axis -- only in pose mode, where it
// signals "drag this to move the joint"; in orbit mode it would be a misleading
// affordance.  (Sidebar-row hover still calls highlightJoint directly.)
// pose mode: the full emphasis (rod brightens/fattens) signals "drag to move".
// orbit mode: skip that affordance, but still reveal the informational
// +direction glyph (arrow + arc) so hovering a joint reads its positive sense.
viewer.addEventListener('joint-mouseover',
  e => { if (viewState.poseDrag) { highlightJoint(e.detail, true); }
         else { showJointGlyph(e.detail, true); viewer.redraw(); } });
viewer.addEventListener('joint-mouseout',
  e => { if (viewState.poseDrag) { highlightJoint(e.detail, false); }
         else { showJointGlyph(e.detail, false); viewer.redraw(); } });

// joint filter: narrow the tree to a flat list of name-matching joints, so the
// select-all + Set/Delete below act on exactly those (e.g. all "flange" joints).
const _jfilterEl = document.getElementById('jfilter');
_jfilterEl.addEventListener('input', () => {
  treeState.jointFilter = _jfilterEl.value;
  treeState.jointCheckOnly = false;         // typing a name means "search", not "review"
  document.getElementById('selall').checked = false;
  if (viewer.robot) { buildJointRows(viewer.robot); }
});
document.getElementById('jcheckonly').addEventListener('click', () => {
  treeState.jointCheckOnly = !treeState.jointCheckOnly;
  if (treeState.jointCheckOnly) { treeState.jointFilter = ''; _jfilterEl.value = ''; }
  if (viewer.robot) { buildJointRows(viewer.robot); }
});
// Esc clears the filter (without bubbling to the global Escape handler)
_jfilterEl.addEventListener('keydown', e => {
  e.stopPropagation();
  if (e.key === 'Escape' && _jfilterEl.value) {
    e.preventDefault();
    _jfilterEl.value = '';
    treeState.jointFilter = '';
    document.getElementById('selall').checked = false;
    if (viewer.robot) { buildJointRows(viewer.robot); }
  }
});
const _treeModeEl = document.getElementById('treemode');
_treeModeEl?.addEventListener('change', () => {
  treeState.treeViewMode = _treeModeEl.value === 'subassembly'
    ? 'subassembly' : 'expanded';
  document.getElementById('selall').checked = false;
  if (viewer.robot) { buildJointRows(viewer.robot); }
});
syncTreeModeControls();

// bulk select / set
document.getElementById('selall').addEventListener('change', e => {
  rows.forEach(r => { r.pick.checked = e.target.checked; });
});
document.getElementById('bulkset').addEventListener('click', () => {
  const t = document.getElementById('bulktype').value;
  const changes = [];
  rows.forEach(r => {
    if (r.pick.checked && r.typeSel.value !== t) {
      changes.push({ name: r.joint.name, parent: r.parent,
                     child: r.child, type: t });
    }
  });
  if (!changes.length) {
    log(t('bulk.noneSelected'), 'wrn');
    return;
  }
  applyTypeChanges(changes);             // the whole batch in ONE rebuild
});

// bulk delete: remove every checked link AND its subtree in ONE rebuild
// (children would otherwise dangle).  Same exclude path as Del / 🗑 — Ctrl+Z
// restores.  Pairs with the filter: filter to "flange", select-all, Delete.
document.getElementById('bulkdel').addEventListener('click', () => {
  if (!packageState.currentInfo) { return; }
  const picked = [...rows.values()].filter(r => r.pick.checked);
  if (!picked.length) { log(t('bulk.delNone'), 'wrn'); return; }
  // expand each picked link to its whole subtree, mapped to component names
  const links = new Set();
  for (const r of picked) {
    for (const n of linkSubtree(r.child)) { links.add(n); }
  }
  const comps = [...new Set([...links].map(n => packageState.compMeta[n]?.name ?? n))];
  if (!confirm(t('bulk.delConfirm', { n: picked.length }))) { return; }
  selectLink(null);
  setExclude({ names: comps, on: true },
             t('bulk.deleting', { n: picked.length }));
});

