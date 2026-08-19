import { highlightJoint } from './bulk-edit.js';
import { viewer } from './dom.js';
import { highlightLink } from './frames.js';
import { fmt, playRows, previewJoint, rows } from './joint-rows.js';
import { selectLink } from './link-info.js';
import { _jointLimits, _jointUnit } from './selection.js';
import { op } from './session-log.js';
import { treeState } from './state.js';
// ---- "move" mode: a clean slider list of ONLY the movable joints ----------
// Same kinematic order as the edit tree (base -> tip) but stripped to one
// slider per movable, non-mimic joint -- so a 300-link assembly is reduced to
// the ~dozen things you actually drive.
const playRowsEl = document.getElementById('playrows');
const playCountEl = document.getElementById('playcount');
const playEmptyEl = document.getElementById('playempty');

function addPlayRow(j, child) {
  const um = _jointUnit(j.jointType);
  const [lo, hi] = _jointLimits(j);
  const fmt = v => {
    let d = um.toDisp(v);
    if (Math.abs(d) < Math.pow(10, -um.dec) / 2) { d = 0; }
    return d.toFixed(um.dec);
  };
  const clamp = v => Math.min(hi, Math.max(lo, v));
  const row = document.createElement('div');
  row.className = 'prow';
  row.innerHTML =
    `<div class="ptop">` +
      `<span class="pname" title="${child}">${child}</span>` +
      `<input class="pval" type="number" step="${um.pris ? 0.5 : 1}">` +
      `<span class="punit">${um.unit}</span>` +
      `<button class="preset" title="${t('play.resetTitle')}">↺</button>` +
    `</div>` +
    `<div class="pslide">` +
      `<span class="plim">${fmt(lo)}</span>` +
      `<input type="range" min="${lo}" max="${hi}" step="0.005" ` +
      `value="${Number(j.angle) || 0}">` +
      `<span class="plim hi">${fmt(hi)}</span>` +
    `</div>`;
  const slider = row.querySelector('input[type=range]');
  const valIn = row.querySelector('.pval');
  valIn.value = fmt(Number(j.angle) || 0);
  const drive = v => previewJoint(j.name, clamp(v));
  slider.addEventListener('input', () => drive(parseFloat(slider.value)));
  valIn.addEventListener('change', () => {
    const d = parseFloat(valIn.value);
    if (!Number.isNaN(d)) { drive(um.toNat(d)); }
  });
  // Shift+←/→ = coarse nudge (the native arrow keys already do a fine step)
  slider.addEventListener('keydown', ev => {
    if (ev.shiftKey && (ev.key === 'ArrowLeft' || ev.key === 'ArrowRight')) {
      ev.preventDefault();
      drive(parseFloat(slider.value)
        + (hi - lo) / 20 * (ev.key === 'ArrowRight' ? 1 : -1));
    }
  });
  row.querySelector('.preset').addEventListener('click', () => drive(0));
  row.querySelector('.pname').addEventListener('click', () => selectLink(child));
  row.addEventListener('mouseenter', () => {
    highlightJoint(j.name, true); highlightLink(child, true);
  });
  row.addEventListener('mouseleave', () => {
    highlightJoint(j.name, false); highlightLink(child, false);
  });
  playRows.set(j.name, { joint: j, row, slider, val: valIn, child, lo, hi, fmt });
  playRowsEl.appendChild(row);
}

export function buildPlayRows(robot) {
  playRowsEl.innerHTML = '';
  playRows.clear();
  if (!robot) { playCountEl.textContent = ''; return; }
  const linkOf = (j, tag) => [...(j.urdfNode?.children ?? [])]
    .find(el => el.tagName === tag)?.getAttribute('link') ?? '';
  const byParent = new Map();
  const childLinks = new Set();
  for (const j of Object.values(robot.joints)) {
    const p = linkOf(j, 'parent') || (j.parent?.name ?? '');
    childLinks.add(linkOf(j, 'child'));
    if (!byParent.has(p)) { byParent.set(p, []); }
    byParent.get(p).push(j);
  }
  byParent.forEach(list => list.sort(
    (a, b) => linkOf(a, 'child').localeCompare(linkOf(b, 'child'))));
  const seen = new Set();
  let n = 0;
  const walk = j => {
    if (seen.has(j.name)) { return; }
    seen.add(j.name);
    const child = linkOf(j, 'child');
    if (j.jointType !== 'fixed' && !j.mimicJoint) { addPlayRow(j, child); n += 1; }
    for (const k of byParent.get(child) ?? []) { walk(k); }
  };
  const roots = Object.keys(robot.links).filter(x => !childLinks.has(x)).sort();
  for (const r of roots) { for (const j of byParent.get(r) ?? []) { walk(j); } }
  playCountEl.textContent = t('play.count', { n });
  playEmptyEl.style.display = n ? 'none' : 'block';
}

const playBtn = document.getElementById('playmode');
const _panelEl = document.getElementById('panel');
playBtn.addEventListener('click', () => {
  treeState.playMode = playBtn.classList.toggle('active');
  _panelEl.classList.toggle('playmode', treeState.playMode);
  if (treeState.playMode) {
    buildPlayRows(viewer.robot);
  } else {
    playRows.clear();                 // drop refs to the now-hidden rows
    playRowsEl.innerHTML = '';
  }
  log(treeState.playMode ? t('play.on') : t('play.off'), 'ok');
});
document.getElementById('playhome').addEventListener('click', () => {
  let n = 0;
  withJointOpSuppressed(() => {
    for (const [name, r] of playRows) {
      previewJoint(name, Math.min(r.hi, Math.max(r.lo, 0)));
      n += 1;
    }
  });
  if (n) { op('resetPose', { n }); }   // one action, not n joint edits
});

viewer.addEventListener('angle-change', e => {
  const rec = rows.get(e.detail);
  if (rec && rec.slider) {
    rec.slider.value = Number(rec.joint.angle);
    rec.val.textContent = rec.fmtDisp
      ? rec.fmtDisp(Number(rec.joint.angle))
      : fmt(Number(rec.joint.angle));
  }
  const pr = playRows.get(e.detail);
  if (pr) {
    const a = Number(pr.joint.angle);
    pr.slider.value = a;
    pr.val.value = pr.fmt(a);
  }
  _recordJointEdit(e.detail);
});

// ---- record a joint move as ONE semantic op per settle --------------------
// angle-change fires every frame while a slider or a pose-drag moves, so a
// single drag would flood the action log (and the screencast overlay).  We
// debounce per joint: when a joint's value stops changing for a moment, emit
// one op('setJoint').  Programmatic moves (pose restore on reload, snap-to-rest
// on a type flip, reset-pose) run inside withJointOpSuppressed(), so the model
// updates without the move being logged as if the user did it by hand.
let _joSuppress = 0;
export function withJointOpSuppressed(fn) {
  _joSuppress += 1;
  try { return fn(); } finally { _joSuppress -= 1; }
}
export const _joPending = new Map();      // joint name -> { val, suppressed, timer }
function _recordJointEdit(name) {
  const j = viewer.robot?.joints?.[name];
  // only user-drivable joints: a mimic follows its master and a fixed joint
  // can't move, so an angle-change on either is a side effect, not an edit
  if (!j || j.mimicJoint || j.jointType === 'fixed') { return; }
  // stamp suppression at event time: a programmatic burst runs synchronously
  // while _joSuppress > 0, so its last event marks the entry suppressed even
  // though the debounce timer fires later, after the flag has been cleared.
  const suppressed = _joSuppress > 0;
  const prev = _joPending.get(name);
  if (prev) {
    clearTimeout(prev.timer);
    // a programmatic (suppressed) move must not erase a real pending edit that
    // hasn't fired yet -- flush it now so the user's drag isn't lost
    if (suppressed && !prev.suppressed) { op('setJoint', { joint: name, value: prev.val }); }
  }
  const val = Number(j.angle);
  const timer = setTimeout(() => {
    const p = _joPending.get(name);
    _joPending.delete(name);
    if (!p || p.suppressed) { return; }        // programmatic move: don't log
    op('setJoint', { joint: name, value: p.val });
  }, 350);
  _joPending.set(name, { val, suppressed, timer });
}

