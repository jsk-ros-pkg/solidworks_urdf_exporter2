import {
  boxSelected, clearBoxSelection, refreshBulkbar, toggleBoxLink,
} from './box-select.js';
import { _projToScreen } from './camera-reroot.js';
import { viewer } from './dom.js';
import { _tintLink } from './frames.js';
import { selectLink } from './link-info.js';
import { hiddenLinks, toggleLinkVisible } from './link-look.js';
import { resetPose } from './load.js';
import { jointRecForLink } from './mimic.js';
import { _joPending, withJointOpSuppressed } from './play-mode.js';
import { doHistory } from './root-frame.js';
import { _jointUnit } from './selection.js';
import { packageState, replayState, selectionState } from './state.js';
import { THREE } from './three-setup.js';
// ---- operation log + diagnostic dump: every user action is recorded, and
// window.sw2robot.report() ships the session state to the server, so a "now
// it's broken" moment in the USER's long-lived session can be inspected
// (and replayed in tests) instead of guessed at -------------------------
export const oplog = [];
const opSubs = [];               // live subscribers (keycast overlay, future recorders)
export function onOp(fn) { opSubs.push(fn); }   // subscribe to the semantic action stream
export function op(name, args = {}) {
  const entry = { t: Math.round(performance.now()), op: name, ...args };
  // while replaying we still light up the overlay, but we must NOT append the
  // re-performed actions back into the log we're reading from.
  if (!replayState._replaying) {
    oplog.push(entry);
    if (oplog.length > 400) { oplog.splice(0, 100); }
  }
  // notify live listeners; a bad subscriber must never break the action itself
  for (const fn of opSubs) { try { fn(entry); } catch { /* ignore */ } }
}

// ---- camera track: sampled SEPARATELY from op() (so it never touches the
// overlay / action log), throttled, so replay can move the view the way it
// moved during recording -- the "someone is looking around" feel. -----------
export const camlog = [];               // { t, op:'camera', pos:[x,y,z], tgt:[x,y,z] }
let _camLastT = 0;
viewer.controls.addEventListener('change', () => {
  if (replayState._replaying) { return; }                     // don't record our own playback
  const now = Math.round(performance.now());
  if (now - _camLastT < 100) { return; }          // ~10 keyframes/sec max
  _camLastT = now;
  const p = viewer.camera.position, g = viewer.controls.target;
  camlog.push({ t: now, op: 'camera',
                pos: [p.x, p.y, p.z], tgt: [g.x, g.y, g.z] });
  if (camlog.length > 4000) { camlog.splice(0, 1000); }
});
// actions + camera keyframes merged into one time-ordered stream for replay/export
export function fullTimeline() {
  return [...oplog, ...camlog].sort((a, b) => (a.t ?? 0) - (b.t ?? 0));
}
// a fresh session (new package / cleared viewer): the old actions + camera
// keyframes reference a robot/coordinate-frame that no longer exists, so
// replaying/exporting them would be garbage -- drop everything, timers included.
export function _resetSessionLogs() {
  oplog.length = 0;
  camlog.length = 0;
  _camLastT = 0;
  for (const p of _joPending.values()) { clearTimeout(p.timer); }
  _joPending.clear();
}

// Human-readable label for a semantic action from the op() stream -- the
// shared vocabulary used by the screencast overlay, the log export, and (later)
// replay.  View navigation (orbit/pan/zoom) never calls op(), so it can't
// appear here: the noise is gone by construction, not by filtering.  An unknown
// op falls back to a Title-cased name, so a newly-instrumented action shows up
// for free (add a nicer entry here when you want one).
const OP_LABEL = {
  select:      a => a.link ? 'Select: ' + a.link : 'Deselect',
  setColor:    a => 'Colour: ' + a.link,
  eye:         a => (a.hide ? 'Hide: ' : 'Show: ') + a.link,
  setMaterial: a => 'Material: ' + a.link,
  setInertial: a => 'Inertial: ' + a.link,
  exclude:     () => 'Exclude',
  autoLimits:  () => 'Auto limits',
  reRoot:      a => 'Make root: ' + a.link,
  align:       () => 'Align to face',
  remove_port: () => 'Remove end-coords',
  add_port:    () => 'Add end-coords',
  setJoint:    a => {
    const j = viewer.robot?.joints?.[a.joint];
    const um = j ? _jointUnit(j.jointType) : null;
    const disp = um ? um.toDisp(a.value).toFixed(um.dec) + um.unit
                    : (+a.value).toFixed(3);
    return `Joint ${a.joint} → ${disp}`;
  },
  resetPose:   () => 'Reset pose',
  boxSelect:   a => `Box select (${a.n})`,
  boxToggle:   a => 'Toggle: ' + a.name,
  setMimic:    a => 'Mimic → ' + a.master,
  clearMimic:  a => 'Clear mimic: ' + a.child,
  loadRobot:   a => 'Load' + (a.name ? ': ' + a.name : ''),
  rootPose:    a => 'Root pose: ' + a.what,
  clearViewer: () => 'Clear scene',
  undo:        () => 'Undo',
  redo:        () => 'Redo',
};
export function opLabel(e) {
  return OP_LABEL[e.op] ? OP_LABEL[e.op](e)
                        : e.op.charAt(0).toUpperCase() + e.op.slice(1);
}

// ---- action-log export (③): the same op() stream the overlay shows, written
// out for a bug report / tutorial notes.  Text is human-readable
// ("+12.3s  Select: base_link"); JSON is the raw oplog, which replay reads. ---
export function actionLogText() {
  const t0 = oplog.length ? oplog[0].t : 0;   // times relative to the first action
  return oplog.map(e => `${('+' + ((e.t - t0) / 1000).toFixed(1) + 's').padStart(8)}  `
                        + opLabel(e)).join('\n') + '\n';
}
export function downloadActionLog(kind = 'txt') {
  const raw = kind === 'json';
  const body = raw ? JSON.stringify(fullTimeline(), null, 2) : actionLogText();
  const pkg = (packageState.currentInfo?.name || 'sw2robot').replace(/[^\w.-]+/g, '_');
  const blob = new Blob([body], {
    type: raw ? 'application/json' : 'text/plain;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `${pkg}-actions.${raw ? 'json' : 'txt'}`;
  document.body.appendChild(a); a.click(); a.remove();   // in-DOM click for Firefox
  URL.revokeObjectURL(url);
  log(t('log.saved', { file: a.download, n: oplog.length }), 'ok');
}

// ---- replay (②): re-perform a recorded op() stream, with the original
// inter-action timing, driving the viewer -- so a recording plays itself back
// (joints move, links select) for a demo video.  Only a SAFE, non-destructive
// subset is replayable; anything else (rename/exclude/add_port/... which mutate
// server state) is skipped with a log line.  Add an entry here to grow it. ----
// glide a joint from its current angle to `to` (easeInOutQuad) instead of
// snapping, so a replayed drag looks like a hand moving it.  Non-blocking: the
// replay loop keeps its recorded timing while the joint eases in the background.
const _tweenGen = new Map();     // joint -> generation, so a newer tween wins
let _activeTweens = 0;           // eases still running (replayLog drains these)
export function _cancelTweens() {        // invalidate every running tween (bump all gens)
  for (const n of _tweenGen.keys()) { _tweenGen.set(n, (_tweenGen.get(n) || 0) + 1); }
}
function tweenJoint(name, to, dur = 500) {
  const j = viewer.robot?.joints?.[name];
  if (!j) { return; }
  op('setJoint', { joint: name, value: to });     // announce label once (not logged)
  const from = Number(j.angle) || 0;
  const gen = (_tweenGen.get(name) || 0) + 1;
  _tweenGen.set(name, gen);
  if (Math.abs(to - from) < 1e-6) {
    withJointOpSuppressed(() => viewer.setJointValue(name, to)); return;
  }
  const t0 = performance.now();
  _activeTweens += 1;
  let live = true;
  const finish = () => { if (live) { live = false; _activeTweens -= 1; } };
  const step = () => {
    // stop only when SUPERSEDED (a newer tween, or _cancelTweens on stop/reset/
    // reload).  NOT gated on _replaying: the last action's ease starts as the
    // replay loop ends and must finish (replayLog drains us before resolving).
    if (_tweenGen.get(name) !== gen) { finish(); return; }
    const k = Math.min(1, (performance.now() - t0) / dur);
    const e = k < 0.5 ? 2 * k * k : 1 - Math.pow(-2 * k + 2, 2) / 2;
    withJointOpSuppressed(() => viewer.setJointValue(name, from + (to - from) * e));
    if (k < 1) { requestAnimationFrame(step); } else { finish(); }
  };
  requestAnimationFrame(step);
}
const REPLAY = {
  select:    e => selectLink(e.link),
  setJoint:  e => tweenJoint(e.joint, e.value),
  resetPose: () => { _cancelTweens(); resetPose(); },   // stop any in-flight glide first
  eye:       e => {
    if (hiddenLinks.has(e.link) === !!e.hide) { return; }   // already in target state
    toggleLinkVisible(e.link, jointRecForLink(e.link)?.row);  // pass the row so its UI updates
    if (e.link === selectionState.selectedLink) {                           // keep the selbar eye in sync too
      document.getElementById('seleye').classList.toggle('off', hiddenLinks.has(e.link));
    }
  },
  boxSelect: e => {                                // reproduce the recorded set (not select-all)
    clearBoxSelection();                           // (the recorded select(null) handles deselect)
    for (const n of e.names ?? []) {
      if (viewer.robot?.links?.[n]) { boxSelected.add(n); _tintLink(n, 'sel'); }
    }
    refreshBulkbar();
  },
  boxToggle: e => toggleBoxLink(e.name),
  undo:      () => doHistory('undo'),
  redo:      () => doHistory('redo'),
  camera:    e => {                               // move the view to a keyframe
    if (!e.pos || !e.tgt) { return; }
    viewer.camera.position.set(e.pos[0], e.pos[1], e.pos[2]);
    viewer.controls.target.set(e.tgt[0], e.tgt[1], e.tgt[2]);
    viewer.controls.update();
    viewer.redraw();
  },
};

// ---- synthetic cursor: a fake pointer that glides to each action's on-screen
// spot and pulses on click-like ops, so an automated replay reads as "someone
// is operating it" rather than things happening by themselves. ---------------
const CLICK_OPS = new Set(['select', 'eye', 'reRoot', 'add_port', 'remove_port',
                           'boxToggle', 'setColor', 'setMaterial']);
const _cursorV = new THREE.Vector3();
const _cursorView = new THREE.Vector3();
function _actionScreenPos(e) {
  if (!viewer.robot || !viewer.camera) { return null; }
  const obj = e.op === 'setJoint' ? viewer.robot.joints?.[e.joint]
            : viewer.robot.links?.[e.link || e.child || e.name];
  if (!obj) { return null; }
  obj.getWorldPosition(_cursorV);
  // reject points BEHIND the camera: the perspective divide would mirror them
  // to a bogus on-screen spot (view space looks down -z, so front is z < 0)
  _cursorView.copy(_cursorV).applyMatrix4(viewer.camera.matrixWorldInverse);
  if (_cursorView.z >= 0) { return null; }
  return _projToScreen(_cursorV, viewer.getBoundingClientRect());
}
function _showCursor(on) {
  const c = document.getElementById('replaycursor');
  if (c) { c.style.display = on ? 'block' : 'none'; }
}
function _moveCursor(x, y, click) {
  const c = document.getElementById('replaycursor');
  if (!c) { return; }
  c.style.left = x + 'px'; c.style.top = y + 'px';
  if (click) { c.classList.remove('click'); void c.offsetWidth; c.classList.add('click'); }
}
export function replayCancel() { _cancelTweens(); replayState._replaying = false; }   // stop eases where they are
export async function replayLog(entries, { speed = 1, maxGap = 1500 } = {}) {
  if (replayState._replaying) { return; }                   // no re-entrancy
  if (!Array.isArray(entries) || !entries.length) {
    log(t('replay.empty'), 'wrn'); return;
  }
  _cancelTweens();                              // drop any ease left over from a prior run
  speed = speed > 0 ? speed : 1;               // guard against a 0 / negative speed
  replayState._replaying = true;
  document.getElementById('replay')?.classList.add('active');
  _showCursor(true);
  let done = 0, prev = entries[0].t ?? 0;
  const skippedOps = new Set();                // distinct non-replayable op names
  try {
    for (const e of entries) {
      const wait = Math.min(maxGap, Math.max(0, (e.t ?? prev) - prev)) / speed;
      prev = e.t ?? prev;
      const fn = REPLAY[e.op];
      // only track the cursor for ops that actually replay (camera has no
      // on-screen spot; a skipped op must NOT show a phantom click) -- start the
      // glide NOW so the pointer arrives as the action fires
      const pos = (fn && e.op !== 'camera') ? _actionScreenPos(e) : null;
      if (pos) { _moveCursor(pos.x, pos.y, false); }
      if (wait) { await new Promise(r => setTimeout(r, wait)); }
      if (!replayState._replaying) { break; }                // cancelled mid-run
      if (pos && CLICK_OPS.has(e.op)) { _moveCursor(pos.x, pos.y, true); }  // click pulse
      if (!fn) { skippedOps.add(e.op); continue; }
      try { fn(e); done += 1; }
      catch (err) { log(t('replay.stepFail', { op: e.op, e: err.message ?? err }), 'wrn'); }
    }
    // let the final ease(s) finish before resolving, so a caller that stops a
    // screen recording on our promise captures the full motion (bounded so a
    // stuck tween can't hang the await)
    const drainUntil = performance.now() + 800;
    while (replayState._replaying && _activeTweens > 0 && performance.now() < drainUntil) {
      await new Promise(r => setTimeout(r, 30));
    }
  } finally {
    replayState._replaying = false;
    document.getElementById('replay')?.classList.remove('active');
    _showCursor(false);
  }
  log(t('replay.done', { done, skipped: skippedOps.size }), 'ok');
  // not just a count: name the actions we couldn't replay, so a "done" that
  // silently dropped rename/exclude/add_port/... reads as the partial it is
  if (skippedOps.size) { log(t('replay.partial', { ops: [...skippedOps].join(', ') }), 'wrn'); }
}

