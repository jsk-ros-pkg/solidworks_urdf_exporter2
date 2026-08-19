import { boxSelected, clearBoxSelection } from './box-select.js';
import { pickAxis, pickLink } from './camera-reroot.js';
import { bulkbarEl, mimicBar } from './dom.js';
import { _tintLink, baseTint } from './frames.js';
import { rows } from './joint-rows.js';
import { applyTypeChanges } from './joint-type.js';
import { loadRobot } from './load.js';
import { refreshHistory } from './root-frame.js';
import { mimicFollowers } from './selection.js';
import { op } from './session-log.js';
import { mimicState, packageState, selectionState } from './state.js';
// ---- mimic linking: select a MASTER joint, press m, then click other joints
// to make them follow it; one URDF rebuild on apply.  For fingers: one driven
// joint + several coupled phalanges. ----------------------------------------
export function jointRecForLink(child) {
  return [...rows.values()].find(r => r.child === child) || null;
}
function updateMimicBar() {
  document.getElementById('mimicbartext').innerHTML =
    t('mimic.bar', { master: mimicState.mimicMaster, n: mimicFollowers.size });
}
export function enterMimicMode() {
  const rec = selectionState.selectedLink && jointRecForLink(selectionState.selectedLink);
  if (!rec || rec.joint.jointType === 'fixed') {
    log(t('mimic.needMaster'), 'wrn');
    return;
  }
  mimicState.mimicMode = true;
  mimicState.mimicMaster = rec.joint.name;
  mimicState.mimicMasterChild = rec.child;
  mimicFollowers.clear();
  _tintLink(rec.child, 'mas');          // master = green
  mimicBar.style.display = 'flex';
  updateMimicBar();
  log(t('mimic.start', { master: mimicState.mimicMaster }));
}
// click a joint while in mimic mode: toggle it as a follower of the master
export function mimicPick(ev) {
  const ax = pickAxis(ev);
  const child = ax ? ax.child : pickLink(ev);
  if (!child || child === mimicState.mimicMasterChild) { return; }
  const rec = jointRecForLink(child);
  if (!rec || rec.joint.jointType === 'fixed') {
    log(t('mimic.followerFixed'), 'wrn');
    return;
  }
  if (mimicFollowers.has(child)) {
    mimicFollowers.delete(child);
    _tintLink(child, baseTint(child));
  } else {
    mimicFollowers.add(child);
    _tintLink(child, 'mim');            // follower = purple
  }
  updateMimicBar();
}
function exitMimicMode() {
  // drop session tints (a rebuild repaints from the URDF on apply anyway)
  const mc = mimicState.mimicMasterChild, fol = [...mimicFollowers];
  mimicState.mimicMode = false;
  mimicState.mimicMaster = mimicState.mimicMasterChild = null;
  mimicFollowers.clear();
  if (mc) { _tintLink(mc, baseTint(mc)); }
  for (const c of fol) { _tintLink(c, baseTint(c)); }
  mimicBar.style.display = 'none';
}
export function cancelMimic() { log(t('mimic.cancel'), 'wrn'); exitMimicMode(); }
export async function applyMimic() {
  if (!mimicFollowers.size) { cancelMimic(); return; }
  const master = mimicState.mimicMaster, followers = [...mimicFollowers];
  const changes = followers.map(child => ({
    child, master, multiplier: 1.0, offset: 0.0 }));
  op('setMimic', { master, n: changes.length });
  exitMimicMode();
  selectionState.reselectAfterLoad = followers[0];     // keep a follower's panel open after
  log(t('mimic.applying', { n: changes.length, master }));
  statusEl.textContent = t('status.rebuilding');
  try {
    const resp = await fetch('/api/set_mimic', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ changes }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (r.missed?.length) {
      log(t('mimic.missed', { list: r.missed.join(', ') }), 'wrn');
    }
    log(t('mimic.done', { n: r.applied?.length ?? 0 }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('mimic.fail', { e: e.message ?? e }), 'err');
  }
}
// unlink one follower (called from the joint panel of a mimic joint)
export async function clearMimic(child) {
  op('clearMimic', { child });
  selectionState.reselectAfterLoad = child;
  log(t('mimic.unlinking', { child }));
  statusEl.textContent = t('status.rebuilding');
  try {
    const resp = await fetch('/api/set_mimic', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ changes: [{ child, clear: true }] }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('mimic.unlinked'), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('mimic.fail', { e: e.message ?? e }), 'err');
  }
}
// change an existing mimic's multiplier/offset (from the joint panel)
export async function updateMimic(child, master, multiplier, offset) {
  selectionState.reselectAfterLoad = child;
  try {
    const resp = await fetch('/api/set_mimic', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ changes: [{ child, master, multiplier, offset }] }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('mimic.done', { n: 1 }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('mimic.fail', { e: e.message ?? e }), 'err');
  }
}
mimicBar.querySelector('#mimicapply').addEventListener('click', applyMimic);
mimicBar.querySelector('#mimiccancel').addEventListener('click', cancelMimic);

// apply one joint type to every box-selected link's controlling joint
export function bulkSetType(type) {
  const links = [...boxSelected];
  if (!links.length || !packageState.currentInfo) { return; }
  // route the 3D box-selection through the SAME optimistic path as the sidebar
  // checkbox bulk: map each selected child link to its joint, apply instantly,
  // persist in the background.  Root / unmatched links carry no joint -> skipped;
  // joints already at `type` are no-ops and dropped.
  const byChild = new Map([...rows.values()].map(r => [r.child, r]));
  const changes = [];
  for (const c of links) {
    const rec = byChild.get(c);
    if (!rec) { continue; }
    // compare against the DISPLAYED type so a mass-only link reads as 'mass_only'
    // (its joint is fixed) -- lets us skip no-ops and still clear it for a real type
    const cur = packageState.massOnlyLinks.has(rec.child) ? 'mass_only' : rec.joint.jointType;
    if (cur !== type) {
      changes.push({ name: rec.joint.name, parent: rec.parent,
                     child: rec.child, type });
    }
  }
  clearBoxSelection();
  if (!changes.length) { log(t('bulk.noneSelected'), 'wrn'); return; }
  log(t('bulk.applying', { n: changes.length, type }));
  applyTypeChanges(changes);          // instant + background persist (serialized)
}
bulkbarEl.querySelectorAll('button[data-bt]').forEach(b =>
  b.addEventListener('click', () => bulkSetType(b.dataset.bt)));

