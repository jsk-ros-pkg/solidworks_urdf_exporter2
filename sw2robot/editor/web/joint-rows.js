import { viewer } from './dom.js';
import { selectLink } from './link-info.js';
import { loadRobot } from './load.js';
import { refreshHistory } from './root-frame.js';
import { packageState, selectionState } from './state.js';
// ---- joint rows: slider (movable) + type select + bulk checkbox ---------
export function fmt(v) { return (Math.abs(v) < 1e-10 ? 0 : v).toFixed(2); }
export const rows = new Map();        // joint name -> row record
export const playRows = new Map();    // joint name -> "move" mode slider record


// move a joint AND keep every UI that displays it in lock-step (the sidebar
// row + the in-viewer joint panel).  `val` is in the joint's native unit
// (radians for revolute/continuous, metres for prismatic).
export function previewJoint(name, val) {
  viewer.setJointValue(name, val);
  const rec = rows.get(name);
  if (rec) {
    if (rec.slider) { rec.slider.value = val; }
    if (rec.val) { rec.val.textContent = rec.fmtDisp ? rec.fmtDisp(val) : fmt(val); }
  }
  const pr = playRows.get(name);
  if (pr) { pr.slider.value = val; pr.val.value = pr.fmt(val); }
  if (selectionState.jpSync && selectionState.jpSync.name === name) { selectionState.jpSync.set(val); }
}

export function jointLinkOf(j, tag) {
  return [...(j?.urdfNode?.children ?? [])]
    .find(el => el.tagName === tag)?.getAttribute('link') ?? '';
}

function parentJointForLink(linkName) {
  const link = viewer.robot?.links?.[linkName];
  return link?.parent?.isURDFJoint ? link.parent : null;
}

export function previewJointForPlanRow(row) {
  // Prefer the actual parent joint of the displayed child link.  The collapsed
  // preview URDF may suffix duplicate joint names (e.g. "..._3"), and this is
  // the same path the in-viewer joint editor uses after a link click.
  const byChild = parentJointForLink(row?.child || '');
  if (byChild) { return byChild; }
  const byName = viewer.robot?.joints?.[row?.name || ''];
  if (byName) { return byName; }
  return Object.values(viewer.robot?.joints ?? {})
    .find(j => jointLinkOf(j, 'child') === row?.child) || null;
}

export const collapsed = new Set();    // link names folded in the tree (persists
                                // across rebuilds within the session)

// ---- rename a joint / link (dbl-click in tree/panel, or the ≣ 改名 list) ----
export async function doRename(kind, oldName, newName) {
  newName = (newName || '').trim();
  if (newName === oldName) { return false; }   // unchanged
  const reset = newName === '';                // empty -> back to default name
  try {
    const resp = await fetch('/api/rename', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ kind, old: oldName, new: newName }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(reset ? t('rename.resetOk', { kind, name: oldName })
              : t('rename.ok', { kind, old: oldName, new: newName }), 'ok');
    refreshHistory();
    // a link rename invalidates the current selection (its name changed); clear
    // it so the reload doesn't reference a link that no longer exists
    selectLink(null);
    loadRobot(packageState.currentInfo, { keepPose: true });   // rebuild with the new name
    return true;
  } catch (e) {
    log(t('rename.fail', { reset, e: e.message ?? e }), 'err');
    return false;
  }
}

// revert ALL joint/link names to their defaults (one rebuild)
export async function resetAllNames() {
  if (!packageState.currentInfo) { return; }
  if (!confirm(t('rename.confirmResetAll'))) { return; }
  try {
    const resp = await fetch('/api/reset_names', { method: 'POST' });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('rename.resetAllOk'), 'ok');
    refreshHistory();
    selectLink(null);
    document.getElementById('renamelist')?.remove();
    loadRobot(packageState.currentInfo, { keepPose: true });
  } catch (e) {
    log(t('rename.resetAllFail', { e: e.message ?? e }), 'err');
  }
}

// double-click a name span -> edit it in place -> Enter commits, Esc cancels
export function attachInlineRename(el, kind, getOld) {
  el.classList.add('renamable');
  if (!el.title) {
    el.title = t('rename.inlineTitle');
  }
  let editing = false, commit = true, oldVal = '';
  el.addEventListener('dblclick', ev => {
    ev.preventDefault(); ev.stopPropagation();
    if (editing) { return; }
    editing = true; commit = true; oldVal = getOld();
    el.contentEditable = 'true'; el.spellcheck = false; el.textContent = oldVal;
    el.focus();
    const sel = window.getSelection(), rng = document.createRange();
    rng.selectNodeContents(el); sel.removeAllRanges(); sel.addRange(rng);
  });
  el.addEventListener('keydown', e => {
    if (!editing) { return; }
    e.stopPropagation();
    if (e.key === 'Enter') { e.preventDefault(); el.blur(); }
    else if (e.key === 'Escape') { commit = false; el.blur(); }
  });
  el.addEventListener('blur', () => {
    if (!editing) { return; }
    editing = false;
    el.contentEditable = 'false';
    const v = el.textContent.trim();
    el.textContent = oldVal;          // revert; a successful (re)name reloads
    // empty -> reset to default; same value -> no-op
    if (commit && v !== oldVal) { doRename(kind, oldVal, v); }
  });
}

