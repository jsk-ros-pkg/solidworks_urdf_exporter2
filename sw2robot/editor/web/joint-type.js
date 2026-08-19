import { addAxisMarkers, axisMemory } from './axis-markers.js';
import {
  hideLoadbar, pollProgress, refreshCompMeta, setProgressStop, showLoadbar,
} from './capture-progress.js';
import { viewer } from './dom.js';
import { playRows, rows } from './joint-rows.js';
import { selectLink } from './link-info.js';
import { loadRobot } from './load.js';
import { buildPlayRows, withJointOpSuppressed } from './play-mode.js';
import { refreshHistory } from './root-frame.js';
import { op } from './session-log.js';
import { packageState, selectionState, treeState } from './state.js';
import { THREE } from './three-setup.js';
import { buildJointRows } from './tree.js';
// set joint type(s).  A fixed<->movable flip NEVER changes topology (same links,
// joint names and tree -- only the joint's type + <axis>/<limit> presence; the
// exporter even writes a ghost <axis> on fixed joints), so we apply it INSTANTLY
// in the live model and persist (patch joints.yaml + rebuild URDF) in the
// BACKGROUND -- no full reload / mesh re-parse on the happy path, hence no lag.
// Same trust-the-client pattern as root-frame edits.  The sidebar dropdown /
// box-select / panel all funnel through here.

// best-known axis [x,y,z] for a joint being (re)enabled: this session's memory
// (stored whenever it was movable) -> the URDF's <axis> tag (present even on
// fixed joints) -> the live loader axis.  null only if truly unknown.
function jointAxisArray(j) {
  const m = axisMemory.get(j.name);
  if (m && m.length === 3 && m.every(n => Number.isFinite(n))) { return m; }
  const el = j.urdfNode && [...j.urdfNode.children]
    .find(e => e.tagName === 'axis');
  if (el) {
    const a = (el.getAttribute('xyz') ?? '').trim().split(/\s+/).map(Number);
    if (a.length === 3 && a.every(n => Number.isFinite(n))) { return a; }
  }
  if (j.axis) { return j.axis.toArray(); }
  return null;
}

// mutate the live urdf-loader joint so the 3D model + drag behave as the new
// type, WITHOUT reloading the URDF (topology is identical across a type flip).
function patchJointTypeLive(j, type) {
  if (type === 'fixed') {
    if (j.jointType !== 'fixed') {
      axisMemory.set(j.name, j.axis.toArray());  // remember for un-fixing later
      withJointOpSuppressed(() => j.setJointValue(0));   // rest before locking
    }
    j.jointType = 'fixed';
    return;
  }
  // -> movable: ensure a usable axis (a freshly un-fixed joint may carry none)
  const arr = jointAxisArray(j) ?? [0, 0, 1];
  if (!j.axis) { j.axis = new THREE.Vector3(); }
  j.axis.set(arr[0], arr[1], arr[2]).normalize();
  j.jointType = type;
  if (type === 'revolute' || type === 'prismatic') {
    const lo = Number(j.limit?.lower), hi = Number(j.limit?.upper);
    // un-fixed joints have no <limit>; default to +-pi like the exporter does.
    // the background rebuild persists the authoritative range for next reload.
    if (!(hi > lo)) { j.limit = { lower: -Math.PI, upper: Math.PI }; }
  }
  withJointOpSuppressed(() => j.setJointValue(0));
}

// re-render everything that reads a joint's type WITHOUT touching the 3D scene
// or re-parsing meshes: sidebar tree, move-mode sliders, axis markers, and the
// open joint panel.  DOM-only -> this is what makes a type flip feel instant.
function refreshAfterTypeChange() {
  buildJointRows(viewer.robot);
  if (treeState.playMode) { buildPlayRows(viewer.robot); }
  addAxisMarkers(viewer.robot);
  if (selectionState.selectedLink && viewer.robot.links[selectionState.selectedLink]) {
    selectLink(selectionState.selectedLink);          // rebuild the panel (type / limit / axis)
  }
  viewer.redraw();
}

// serialize the background persists: the editor server is threaded, so two
// overlapping /api/set_types calls would race the shared joints.yaml + build()
export let typePersistChain = Promise.resolve();

async function persistTypeChanges(changes, { reload = false } = {}) {
  try {
    const resp = await fetch('/api/set_types', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ changes }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    refreshHistory();
    if (r.missed?.length) {
      log(t('types.notFound', { list: r.missed.join(', ') }), 'wrn');
    } else if (!r.applied?.length) {
      log(t('types.noneMatched'), 'err');
    } else {
      log(t('types.saved', { n: r.applied.length }), 'ok');
      // a mass-only toggle changes geometry (strip/restore + fold), so the
      // optimistic patch was skipped -- reload to show the new link set
      if (reload && packageState.currentInfo) {
        await refreshCompMeta();
        loadRobot(packageState.currentInfo, { keepPose: true });
      }
      return;
    }
    // anything the optimistic view changed but the file did NOT accept -> resync
    // the live model from the server's URDF so the editor never drifts.
    if (packageState.currentInfo) { loadRobot(packageState.currentInfo, { keepPose: true }); }
  } catch (e) {
    log(t('types.applyFail', { e: e.message ?? e }), 'err');
    if (packageState.currentInfo) { loadRobot(packageState.currentInfo, { keepPose: true }); }
  }
}

export function applyTypeChanges(changes) {
  if (!changes.length || !packageState.currentInfo || !viewer.robot) {
    selectionState.reselectAfterLoad = null;
    return false;
  }
  // mass-only toggles (to OR away from) change the built link set -- geometry is
  // stripped and the link folds into its parent -- which the optimistic in-place
  // patch can't represent.  Persist and reload the whole batch instead.
  if (changes.some(ch => ch.type === 'mass_only' || packageState.massOnlyLinks.has(ch.child))) {
    selectionState.reselectAfterLoad = null;
    log(t('types.applied', { n: changes.length }));
    typePersistChain = typePersistChain
      .then(() => persistTypeChanges(changes, { reload: true }));
    return true;
  }
  // 1. optimistic in-place patch (instant)
  const patched = [];
  for (const ch of changes) {
    const j = viewer.robot.joints[ch.name];
    if (!j) { continue; }
    patchJointTypeLive(j, ch.type);
    patched.push(ch);
  }
  selectionState.reselectAfterLoad = null;            // no reload -> nothing to reselect
  if (!patched.length) { return false; }
  refreshAfterTypeChange();
  log(t('types.applied', { n: patched.length }));
  // 2. persist + rebuild URDF in the background (serialized)
  typePersistChain = typePersistChain
    .then(() => persistTypeChanges(changes));
  return true;                         // the panel/rows were rebuilt in place
}

// write one joint's travel range to joints.yaml (lower/upper, in NATIVE units:
// rad for revolute, m for prismatic) and rebuild -- shares /api/set_limits with
// the auto-limit sweep, so a joint with no SolidWorks limit mate is editable too
export async function applyJointLimits(child, lowerNat, upperNat) {
  if (!packageState.currentInfo) {
    selectionState.reselectAfterLoad = null;
    return;
  }
  log(t('limits.applying'));
  statusEl.textContent = t('status.rebuilding');
  try {
    const resp = await fetch('/api/set_limits', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ limits: [{ child, lower: lowerNat,
                                        upper: upperNat, continuous: false }] }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (!r.applied?.length) {
      log(t('limits.noneMatched', { child }), 'err');
      selectionState.reselectAfterLoad = null;
      return;
    }
    log(t('limits.done'), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('limits.fail', { e: e.message ?? e }), 'err');
    selectionState.reselectAfterLoad = null;
  }
}

// `t` key: toggle the target joint between fixed and revolute -- the common
// "is this joint rigid or does it turn?" pass.  Target = the selected link's
// joint, else the axis currently under the cursor.  Non-fixed types
// (continuous/prismatic) collapse to fixed; fixed becomes revolute.
export function toggleJointFixed() {
  let rec = null;
  if (selectionState.selectedLink) {
    rec = [...rows.values()].find(r => r.child === selectionState.selectedLink);
  } else if (selectionState.hoveredAxis) {
    rec = rows.get(selectionState.hoveredAxis);
  } else {
    // "move" mode: target the joint whose slider/value field is focused
    const pr = document.activeElement?.closest?.('.prow');
    const entry = pr && [...playRows.values()].find(r => r.row === pr);
    if (entry) { rec = [...rows.values()].find(r => r.child === entry.child); }
  }
  if (!rec) { log(t('jtype.noTarget'), 'wrn'); return; }
  const cur = rec.joint.jointType;
  const next = cur === 'fixed' ? 'revolute' : 'fixed';
  selectionState.reselectAfterLoad = rec.child;        // keep selection/panel after the rebuild
  log(t('jtype.toggle', { joint: rec.joint.name, from: cur, to: next }));
  applyTypeChanges([{ name: rec.joint.name, parent: rec.parent,
                      child: rec.child, type: next }]);
}

// reverse a joint's + direction: negate its axis AND swap/negate its limits,
// patched straight into the served URDF (frame untouched; self-inverse -- press
// again to revert).  No yaml rebuild, so it works on any loaded URDF and the
// export reads it as-is.  Target = the selected link's joint, else the hovered
// axis (mirrors `t`).
export async function applyAxisFlip(jointName, child) {
  if (!packageState.currentInfo) { return; }
  log(t('flip.applying', { joint: jointName }));
  try {
    const resp = await fetch('/api/set_axis', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ joints: [jointName] }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (!r.applied?.length) { log(t('flip.noneMatched'), 'err'); return; }
    log(t('flip.done', { joint: jointName }), 'ok');
    selectionState.reselectAfterLoad = child;            // keep selection/panel after reload
    loadRobot(packageState.currentInfo, { keepPose: true });
  } catch (e) {
    log(t('flip.fail', { e: e.message ?? e }), 'err');
  }
}

export function flipTargetJointAxis() {
  let rec = null;
  if (selectionState.selectedLink) {
    rec = [...rows.values()].find(r => r.child === selectionState.selectedLink);
  } else if (selectionState.hoveredAxis) {
    rec = rows.get(selectionState.hoveredAxis);
  }
  if (!rec) { log(t('flip.noTarget'), 'wrn'); return; }
  if (rec.joint.jointType === 'fixed') { log(t('flip.notMovable'), 'wrn'); return; }
  applyAxisFlip(rec.joint.name, rec.child);
}

// ---- 🛠 auto joint limits via the self-collision sweep -------------------
const autolimitsBtn = document.getElementById('autolimits');
autolimitsBtn.addEventListener('click', async () => {
  if (!packageState.currentInfo || packageState.dropMode) {
    log(t('auto.needPkg'), 'wrn');
    return;
  }
  autolimitsBtn.disabled = true;
  op('autoLimits', {});
  log(t('auto.sweepStart'));
  // Start ASYNC, then watch the unified /api/progress panel (via pollProgress)
  // for per-joint progress.  (Safe now: the sweep runs in a child process, so
  // polling no longer steals its GIL the way the old in-process sweep did.)
  showLoadbar(t('auto.sweepBar'), { indet: true });
  try {
    const md = document.getElementById('marginDeg')?.value || '2';
    const mm = document.getElementById('marginMm')?.value || '2';
    const resp0 = await fetch(
      `/api/auto_limits?margin_deg=${encodeURIComponent(md)}`
      + `&margin_mm=${encodeURIComponent(mm)}`);
    const data = await resp0.json();
    if (!resp0.ok || data.error) { throw new Error(data.error ?? resp0.status); }
    // unified progress: the sweep reports stages (load model -> sweep joints)
    // and per-joint frac into _prog; pollProgress paints them + returns results
    setProgressStop(() => fetch('/api/auto_limits/cancel').catch(() => {}));
    const { result, cancelled } = await pollProgress();
    if (cancelled) {
      hideLoadbar();
      log(t('auto.cancelled'), 'wrn');
      autolimitsBtn.disabled = false;
      return;
    }
    const results = (result && result.results) || [];
    const limited = results.filter(r => !r.continuous);
    const cont = results.filter(r => r.continuous);
    log(t('auto.sweepDone', { l: limited.length, c: cont.length }), 'ok');
    if (!results.length) {
      log(t('auto.noRot'), 'wrn');
      hideLoadbar();
      autolimitsBtn.disabled = false;
      return;
    }
    showLoadbar(t('auto.writeBar'), { indet: true });
    const resp = await fetch('/api/set_limits', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ limits: results }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (r.missed?.length) {
      log(t('auto.notMatched', { list: r.missed.join(', ') }), 'wrn');
    }
    log(t('auto.applied', { n: r.applied.length }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    hideLoadbar();
    log(t('auto.fail', { e: e.message ?? e }), 'err');
  } finally {
    autolimitsBtn.disabled = false;
  }
});

