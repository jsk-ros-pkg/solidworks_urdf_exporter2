import { viewer } from './dom.js';
import { originTriad } from './frames.js';
import { typePersistChain } from './joint-type.js';
import { loadRobot } from './load.js';
import { op } from './session-log.js';
import { packageState } from './state.js';
import { THREE } from './three-setup.js';
// ---- root frame box: +-90 deg buttons + numeric rpy(deg)/xyz(mm) --------
// Root-frame edits NEVER reload the robot: the model's internal geometry is
// unchanged, so we re-pose the robot object so its local frame IS the new
// root frame, and move the camera by the same delta -- on screen the model
// stays pixel-fixed and only the origin triad / frame lines move.  The
// server rebuild (persisting the URDF) happens in the background.
const D2R = Math.PI / 180;
let rootAtLoad = new THREE.Matrix4();   // yaml root offset when URDF loaded
export let rootDelta = new THREE.Matrix4();    // load frame -> CURRENT root frame

function rootMat(rpy, xyz) {
  // URDF semantics: rotate, then translate in the rotated frame
  const m = new THREE.Matrix4().makeRotationFromEuler(
    new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));
  const t = new THREE.Vector3(...xyz)
    .applyMatrix4(new THREE.Matrix4().extractRotation(m));
  m.setPosition(t);
  return m;
}

function setRootFields(rpy, xyz) {
  ['r_r', 'r_p', 'r_y'].forEach((id, i) => {
    document.getElementById(id).value = (rpy[i] / D2R).toFixed(1);
  });
  ['r_x', 'r_yy', 'r_z'].forEach((id, i) => {
    document.getElementById(id).value = (xyz[i] * 1000).toFixed(1);
  });
}

export async function refreshRootBox({ resetBaseline = false } = {}) {
  try {
    const r = await (await fetch('/api/root_pose')).json();
    if (r.error) { return; }
    setRootFields(r.rpy, r.xyz);
    packageState.rootBaseName = r.base ?? null;
    const rc = document.querySelector('.joint.root .rootcomp');
    if (rc) {
      rc.textContent = packageState.rootBaseName ? `= ${packageState.rootBaseName}` : '';
    }
    if (resetBaseline) {
      rootAtLoad = rootMat(r.rpy, r.xyz);
      rootDelta.identity();
    }
  } catch { /* no package yet */ }
}

export function applyRootPose(rpy, xyz) {
  // model and camera stay EXACTLY where they are; only the origin triad
  // jumps to where the new root frame sits relative to the geometry
  rootDelta = rootAtLoad.clone().invert().multiply(rootMat(rpy, xyz));
  if (originTriad) {
    rootDelta.decompose(originTriad.position, originTriad.quaternion,
                        originTriad.scale);
    originTriad.scale.set(1, 1, 1);
    viewer.redraw();
  }
  setRootFields(rpy, xyz);
}

async function postRootPose(body, what) {
  op('rootPose', { what });
  if (!packageState.currentInfo) {
    log(t('root.needPkg'), 'wrn');
    return;
  }
  log(t('root.doing', { what }));
  try {
    const resp = await fetch('/api/set_root_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    applyRootPose(r.rpy, r.xyz);     // in-place: no reload, no flash
    refreshHistory();
    log(t('root.ok'), 'ok');
    statusEl.textContent = t('root.okStatus');
  } catch (e) {
    log(t('root.fail', { e: e.message ?? e }), 'err');
  }
}

document.querySelectorAll('#rootbox .rot').forEach(b =>
  b.addEventListener('click', () => {
    const rpy = [0, 0, 0];
    rpy[Number(b.dataset.ax)] = Number(b.dataset.s) * Math.PI / 2;
    postRootPose({ rpy }, t('root.rotating', { dir: b.textContent }));
  }));
// numeric fields apply LIVE (Enter / focus-out), no Set button needed
function applyRootFields() {
  const deg = id => (parseFloat(document.getElementById(id).value) || 0);
  postRootPose({ absolute: {
    rpy: [deg('r_r') * D2R, deg('r_p') * D2R, deg('r_y') * D2R],
    xyz: [deg('r_x') / 1000, deg('r_yy') / 1000, deg('r_z') / 1000] } },
    t('root.settingNumeric'));
}
['r_r', 'r_p', 'r_y', 'r_x', 'r_yy', 'r_z'].forEach(id =>
  document.getElementById(id).addEventListener('change', applyRootFields));

// ---- undo / redo (server-side yaml snapshots) ----------------------------
const undoBtn = document.getElementById('undo');
const redoBtn = document.getElementById('redo');

export async function refreshHistory() {
  try {
    const h = await (await fetch('/api/history')).json();
    undoBtn.disabled = !h.undo?.length;
    redoBtn.disabled = !h.redo?.length;
    undoBtn.title = h.undo?.length
      ? t('hist.undoTitle', { label: h.undo[h.undo.length - 1] })
      : t('hist.undoEmpty');
    redoBtn.title = h.redo?.length
      ? t('hist.redoTitle', { label: h.redo[h.redo.length - 1] })
      : t('hist.redoEmpty');
  } catch { /* no package yet */ }
}

export async function doHistory(which) {
  op(which);
  if (!packageState.currentInfo) { return; }
  // a type flip persists its yaml+rebuild in the BACKGROUND; let it finish
  // before undo/redo rebuilds, or the server runs two builds at once
  await typePersistChain.catch(() => {});
  try {
    const resp = await fetch('/api/' + which, { method: 'POST' });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('hist.done', { which, label: r.label }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshRootBox();
    refreshHistory();
  } catch (e) {
    log(t('hist.fail', { which, e: e.message ?? e }), 'wrn');
  }
}
undoBtn.addEventListener('click', () => doHistory('undo'));
redoBtn.addEventListener('click', () => doHistory('redo'));
window.addEventListener('keydown', ev => {
  if (/INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName)
      || document.activeElement?.isContentEditable) {
    return;
  }
  if ((ev.ctrlKey || ev.metaKey) && !ev.shiftKey
      && ev.key.toLowerCase() === 'z') {
    ev.preventDefault();
    doHistory('undo');
  } else if ((ev.ctrlKey || ev.metaKey)
             && (ev.key.toLowerCase() === 'y'
                 || (ev.shiftKey && ev.key.toLowerCase() === 'z'))) {
    ev.preventDefault();
    doHistory('redo');
  }
});

