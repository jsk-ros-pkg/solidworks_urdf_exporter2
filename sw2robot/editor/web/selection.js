import {
  axesOn, axisGlyphs, axisMarkers, ownLinkBox,
} from './axis-markers.js';
import { markHelper } from './bootstrap.js';
import { viewer } from './dom.js';
import { makeTriad } from './frames.js';
import { attachInlineRename, previewJoint, rows } from './joint-rows.js';
import {
  applyAxisFlip, applyJointLimits, applyTypeChanges,
} from './joint-type.js';
import { loadRobot } from './load.js';
import { clearMimic, updateMimic } from './mimic.js';
import { refreshHistory } from './root-frame.js';
import { packageState, selectionState } from './state.js';
import { THREE } from './three-setup.js';
// ---- selection visuals: link frame triad + centre-of-mass marker --------
// ---- mimic-linking session (master joint + follower joints) state -----------
export const mimicFollowers = new Set(); // follower child-link names chosen this run

// resting visibility of an axis marker (no hover): a fixed joint's ghost is
// normally hidden and a live axis follows the global toggle, BUT the selected
// link's joint axis is always shown -- so you can see where a fixed joint would
// turn and what `t` will toggle.
export function _markerRestVisible(name, m) {
  if (name === selectionState.selAxisJoint) { return true; }
  return m.ghost ? false : axesOn();
}
// reveal the selected link's joint axis (and restore the previously shown one).
// The rod + its always-on glyph track visibility together.
export function revealSelectedJointAxis(linkName) {
  const prev = selectionState.selAxisJoint;
  selectionState.selAxisJoint = null;
  if (prev) {
    const pm = axisMarkers.get(prev);
    if (pm) { pm.mesh.visible = _markerRestVisible(prev, pm); }
    const pg = axisGlyphs.get(prev);
    if (pg) { pg.visible = false; }   // glyph rides hover/selection only
  }
  if (linkName) {
    const rec = [...rows.values()].find(r => r.child === linkName);
    const m = rec && axisMarkers.get(rec.joint.name);
    if (m) {
      m.mesh.visible = true; selectionState.selAxisJoint = rec.joint.name;
      const gl = axisGlyphs.get(rec.joint.name);
      if (gl) { gl.visible = true; }
    }
  }
  viewer.redraw();
}

export function parseInertial(link) {
  const inert = [...(link.urdfNode?.children ?? [])]
    .find(e => e.tagName === 'inertial');
  if (!inert) { return null; }
  const get = tag => [...inert.children].find(e => e.tagName === tag);
  const num = (el, attr, d) => el ? parseFloat(el.getAttribute(attr) ?? d)
                                  : d;
  const org = get('origin');
  return {
    mass: num(get('mass'), 'value', 0),
    com: org ? org.getAttribute('xyz').split(/\s+/).map(Number) : [0, 0, 0],
    inertia: get('inertia')
      ? Object.fromEntries(['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
          .map(k => [k, parseFloat(get('inertia').getAttribute(k) ?? 0)]))
      : null,
  };
}

export function showSelectionVisuals(name) {
  selectionState.selVis?.removeFromParent();
  selectionState.selVis = null;
  const link = viewer.robot?.links?.[name];
  if (!link) { return; }
  const box = ownLinkBox(link, new THREE.Box3());
  const diag = box.isEmpty() ? 0.05
    : box.getSize(new THREE.Vector3()).length();
  const g = new THREE.Group();
  g.add(makeTriad(Math.max(diag * 0.55, 0.025),
                  Math.max(diag * 0.006, 0.0008)));
  const inert = parseInertial(link);
  if (inert && inert.mass > 0) {
    const com = new THREE.Mesh(
      new THREE.SphereGeometry(Math.max(diag * 0.035, 0.002), 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xff4dd2, depthTest: false,
                                    transparent: true, opacity: 0.9 }));
    com.position.set(...inert.com);
    com.renderOrder = 999;
    g.add(com);
  }
  markHelper(g);
  link.add(g);                        // link frame: follows FK
  selectionState.selVis = g;
  viewer.redraw();
}

// --- in-viewer joint editor: clicking a link surfaces its parent joint so the
// type and angle can be edited right on the model (no scrolling the sidebar) --
const JTYPES = ['revolute', 'continuous', 'prismatic', 'fixed'];
// the type dropdowns also offer "mass-only": a fixed joint whose child link
// keeps its weight but drops its geometry (value 'mass_only', shown 'mass-only')
const JTYPES_SEL = [...JTYPES, 'mass_only'];
const typeLabel = t => (t === 'mass_only' ? 'mass-only' : t);
export function typeOptionsHtml(cur) { return JTYPES_SEL.map(t =>
  `<option value="${t}" ${t === cur ? 'selected' : ''}>${typeLabel(t)}</option>`)
  .join(''); }

export function _jointUnit(jtype) {
  const pris = jtype === 'prismatic';
  return pris
    ? { pris, unit: 'mm', step: 0.5, dec: 1,
        toDisp: v => v * 1000, toNat: v => v / 1000 }
    : { pris, unit: '°', step: 1, dec: 0,
        toDisp: v => v * 180 / Math.PI, toNat: v => v * Math.PI / 180 };
}

// native [lo, hi] limits, falling back to a sensible range for continuous /
// unlimited joints (mirrors the sidebar slider's behaviour)
export function _jointLimits(j) {
  let lo = Number(j.limit?.lower), hi = Number(j.limit?.upper);
  const ang = j.jointType !== 'prismatic';
  if (j.jointType === 'continuous' || !(hi > lo)) {
    if (ang) { lo = -Math.PI; hi = Math.PI; }
    else { lo = -0.05; hi = 0.05; }
  }
  return [lo, hi];
}

// current joint physics read straight off the URDF node (source of truth on
// screen): <limit effort/velocity>, <dynamics>, <safety_controller>, <calibration>
function _readPhysics(j) {
  const n = j.urdfNode;
  const at = (sel, a) => n?.querySelector?.(sel)?.getAttribute?.(a) ?? '';
  return {
    effort: at('limit', 'effort'), velocity: at('limit', 'velocity'),
    damping: at('dynamics', 'damping'), friction: at('dynamics', 'friction'),
    soft_lower_limit: at('safety_controller', 'soft_lower_limit'),
    soft_upper_limit: at('safety_controller', 'soft_upper_limit'),
    k_position: at('safety_controller', 'k_position'),
    k_velocity: at('safety_controller', 'k_velocity'),
    cal_rising: at('calibration', 'rising'),
    cal_falling: at('calibration', 'falling'),
  };
}

// collapsible actuator/physics editor for a movable joint: effort/velocity go
// into <limit>, the rest into optional <dynamics>/<safety_controller>/
// <calibration> (blank clears).  POSTs /api/set_physics (rebuild + reload).
function physicsSectionHtml(j) {
  const p = _readPhysics(j);
  const num = (key, val) =>
    `<input type="number" class="jp-num jp-phys" data-key="${key}" ` +
    `step="any" value="${val}">`;
  const row = (lbl, ...inputs) =>
    `<div class="jp-row"><span class="jp-lbl">${lbl}</span>${inputs.join('')}</div>`;
  return (
    `<details class="jp-physics" open><summary>${t('jp.physics')}</summary>` +
      row(t('jp.effort'), num('effort', p.effort)) +
      row(t('jp.velocity'), num('velocity', p.velocity)) +
      `<div class="jp-sub">${t('jp.dynamics')}</div>` +
      row(t('jp.damping'), num('damping', p.damping)) +
      row(t('jp.friction'), num('friction', p.friction)) +
      `<div class="jp-sub">${t('jp.safety')}</div>` +
      row(t('jp.softLower'), num('soft_lower_limit', p.soft_lower_limit)) +
      row(t('jp.softUpper'), num('soft_upper_limit', p.soft_upper_limit)) +
      row(t('jp.kPosition'), num('k_position', p.k_position)) +
      row(t('jp.kVelocity'), num('k_velocity', p.k_velocity)) +
      `<div class="jp-sub">${t('jp.calibration')}</div>` +
      row(t('jp.calRising'), num('cal_rising', p.cal_rising)) +
      row(t('jp.calFalling'), num('cal_falling', p.cal_falling)) +
      `<div class="jp-row jp-physrow">` +
        `<button class="jp-phys-apply">${t('jp.physApply')}</button></div>` +
      `<div class="jp-hint">${t('jp.physHint')}</div>` +
    `</details>`);
}

// gather the physics inputs (blank -> null = clear) and POST /api/set_physics
async function applyPhysics(child, phys) {
  if (!packageState.currentInfo) { return; }
  log(t('phys.applying'));
  statusEl.textContent = t('status.rebuilding');
  try {
    const resp = await fetch('/api/set_physics', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ physics: [{ child, ...phys }] }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (!r.applied?.length) {
      log(t('limits.noneMatched', { child }), 'err');
      selectionState.reselectAfterLoad = null;
      return;
    }
    log(t('phys.done'), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('phys.fail', { e: e.message ?? e }), 'err');
    selectionState.reselectAfterLoad = null;
  }
}

export function jointPanelHtml(j, parentLink, childLink) {
  const movable = j.jointType !== 'fixed';
  const mimic = !!j.mimicJoint;
  const um = _jointUnit(j.jointType);
  const ang = !um.pris;
  const [lo, hi] = _jointLimits(j);
  const dLo = um.toDisp(lo), dHi = um.toDisp(hi);
  const cur = um.toDisp(Number(j.angle) || 0);
  // snap shortcuts: the cardinal angles (with the limit endpoints) for a
  // rotary joint, the endpoints + zero for a slide -- only those in range
  const cand = ang ? [dLo, -180, -90, 0, 90, 180, dHi] : [dLo, 0, dHi];
  const snaps = [...new Set(cand
    .filter(v => v >= dLo - 1e-6 && v <= dHi + 1e-6)
    .map(v => Math.round(v * 10) / 10))].sort((a, b) => a - b);
  const curType = packageState.massOnlyLinks.has(childLink) ? 'mass_only' : j.jointType;
  const typeOpts = typeOptionsHtml(curType);
  let body = '';
  if (movable && !mimic) {
    // continuous has no endpoints; revolute/prismatic get editable lo/hi that
    // POST to /api/set_limits (the same path the auto-limit sweep writes) so a
    // joint WITHOUT a SolidWorks limit mate -- e.g. the Z gantry -- can still
    // be given its travel range straight from the browser.
    const limRow = (j.jointType === 'continuous')
      ? `<div class="jp-lim">${t('jp.contLimit')}</div>`
      : `<div class="jp-row jp-limedit">` +
          `<span class="jp-lbl">${t('jp.range')}</span>` +
          `<input type="number" class="jp-num jp-lo" step="${um.step}" ` +
            `value="${dLo.toFixed(um.dec)}">` +
          `<span class="jp-dash">…</span>` +
          `<input type="number" class="jp-num jp-hi" step="${um.step}" ` +
            `value="${dHi.toFixed(um.dec)}">` +
          `<span class="jp-unit">${um.unit}</span>` +
          `<button class="jp-lim-apply">${t('jp.limApply')}</button>` +
        `</div>`;
    body =
      `<div class="jp-row">` +
        `<input type="range" class="jp-slider" min="${dLo}" max="${dHi}" ` +
          `step="${um.step}" value="${cur}">` +
        `<input type="number" class="jp-num" min="${dLo}" max="${dHi}" ` +
          `step="${um.step}" value="${cur.toFixed(um.dec)}">` +
        `<span class="jp-unit">${um.unit}</span>` +
      `</div>` +
      `<div class="jp-snaps">` +
        snaps.map(s => `<button data-v="${s}">${s}${um.unit}</button>`).join('') +
      `</div>` + limRow +
      `<div class="jp-row jp-fliprow">` +
        `<button class="jp-flip" title="${t('jp.flipTitle')}">` +
        `${t('jp.flip')}</button>` +
      `</div>` + physicsSectionHtml(j);
  } else if (mimic) {
    // read the live coupling from the URDF <mimic> element (the loader exposes
    // only the master link, not multiplier/offset)
    const mimEl = j.urdfNode?.querySelector?.('mimic');
    const mn = mimEl?.getAttribute('joint')
      ?? (j.mimicJoint?.name ?? j.mimicJoint);
    const mult = mimEl?.getAttribute('multiplier') ?? '1';
    const off = mimEl?.getAttribute('offset') ?? '0';
    body =
      `<div class="jp-lim">${t('jp.mimic', { mn })}</div>` +
      `<div class="jp-row"><span class="jp-lbl">${t('jp.mimicMult')}</span>` +
        `<input type="number" class="jp-num jp-mim-mult" step="0.01" ` +
          `value="${mult}" data-master="${mn}"></div>` +
      `<div class="jp-row"><span class="jp-lbl">${t('jp.mimicOff')}</span>` +
        `<input type="number" class="jp-num jp-mim-off" step="0.01" ` +
          `value="${off}"></div>` +
      `<div class="jp-row"><button class="jp-mim-unlink">` +
        `${t('jp.mimicUnlink')}</button></div>`;
  } else {
    body = `<div class="jp-lim">${t('jp.fixed')}</div>`;
  }
  return (
    `<div class="jpanel">` +
      `<div class="jp-title" title="${parentLink} → ${childLink}">` +
        t('jp.title') +
        (mimic ? ` <span class="mimictag">mimic</span>` : '') +
      `</div>` +
      // editable names (double-click): link = the child link, joint = the joint
      `<div class="jp-row"><span class="jp-lbl">${t('jp.lblLink')}</span>` +
        `<span class="jp-rename jp-name" data-kind="link" ` +
        `data-old="${childLink}">${childLink}</span></div>` +
      `<div class="jp-row"><span class="jp-lbl">${t('jp.lblJoint')}</span>` +
        `<span class="jp-rename jp-name" data-kind="joint" ` +
        `data-old="${j.name}">${j.name}</span></div>` +
      `<div class="jp-row">` +
        `<span class="jp-lbl">${t('jp.lblType')}</span>` +
        `<select class="jp-type t-${curType}">${typeOpts}</select>` +
      `</div>` + body +
    `</div>`);
}

export function wireJointPanel(el, j) {
  const panel = el.querySelector('.jpanel');
  if (!panel) { return; }
  // dbl-click a name to rename (link = child link, joint = the joint name)
  panel.querySelectorAll('.jp-rename').forEach(el =>
    attachInlineRename(el, el.dataset.kind, () => el.dataset.old));
  const typeSel = panel.querySelector('.jp-type');
  typeSel?.addEventListener('change', async () => {
    const t = typeSel.value;
    typeSel.className = 'jp-type t-' + t;
    const child = [...(j.urdfNode?.children ?? [])]
      .find(e => e.tagName === 'child')?.getAttribute('link') ?? '';
    // a focused single-joint edit applies immediately (rebuild + reload), so
    // the change actually takes effect (axis, export) without a separate Apply
    selectionState.reselectAfterLoad = child;        // keep this panel open on the new model
    typeSel.disabled = true;
    const reloading = await applyTypeChanges(
      [{ name: j.name, parent: '', child, type: t }]);
    if (!reloading && typeSel.isConnected) { typeSel.disabled = false; }
  });

  // mimic joint controls: edit multiplier/offset, or unlink the coupling
  const childLink = [...(j.urdfNode?.children ?? [])]
    .find(e => e.tagName === 'child')?.getAttribute('link') ?? '';
  const mimMult = panel.querySelector('.jp-mim-mult');
  const mimOff = panel.querySelector('.jp-mim-off');
  if (mimMult) {
    const apply = () => {
      mimMult.disabled = mimOff.disabled = true;
      updateMimic(childLink, mimMult.dataset.master,
                  parseFloat(mimMult.value) || 0,
                  parseFloat(mimOff.value) || 0);
    };
    mimMult.addEventListener('change', apply);
    mimOff.addEventListener('change', apply);
  }
  panel.querySelector('.jp-mim-unlink')?.addEventListener('click',
    () => clearMimic(childLink));

  // ⇄ reverse this joint's + direction (axis negated, limits swapped)
  panel.querySelector('.jp-flip')?.addEventListener('click', () =>
    applyAxisFlip(j.name, childLink));

  // actuator & physics: gather the inputs (blank -> null) and apply
  panel.querySelector('.jp-phys-apply')?.addEventListener('click', () => {
    const phys = {};
    panel.querySelectorAll('.jp-phys').forEach(inp => {
      const v = inp.value.trim();
      phys[inp.dataset.key] = v === '' ? null : parseFloat(v);
    });
    selectionState.reselectAfterLoad = childLink;      // keep this panel open after rebuild
    applyPhysics(childLink, phys);
  });

  const slider = panel.querySelector('.jp-slider');
  if (!slider) { return; }            // fixed / mimic: no angle control
  const num = panel.querySelector('.jp-num');
  const um = _jointUnit(j.jointType);
  const [lo, hi] = _jointLimits(j);
  const dLo = um.toDisp(lo), dHi = um.toDisp(hi);
  const snapBtns = [...panel.querySelectorAll('.jp-snaps button')];
  const clamp = v => Math.min(dHi, Math.max(dLo, v));
  const tol = um.pris ? 0.05 : 0.5;
  const markCur = disp => snapBtns.forEach(b =>
    b.classList.toggle('cur', Math.abs(parseFloat(b.dataset.v) - disp) < tol));
  const show = disp => {
    slider.value = disp;
    num.value = disp.toFixed(um.dec);
    markCur(disp);
  };
  const apply = disp => {
    disp = clamp(isNaN(disp) ? 0 : disp);
    show(disp);
    previewJoint(j.name, um.toNat(disp));   // moves the model + syncs sidebar
  };
  slider.addEventListener('input', () => apply(parseFloat(slider.value)));
  num.addEventListener('change', () => apply(parseFloat(num.value)));
  snapBtns.forEach(b =>
    b.addEventListener('click', () => apply(parseFloat(b.dataset.v))));
  markCur(parseFloat(slider.value));
  // external moves (the sidebar slider) refresh this panel without re-firing
  selectionState.jpSync = { name: j.name, set: nat => show(clamp(um.toDisp(nat))) };

  // editable travel range (revolute/prismatic): write lower/upper -> rebuild
  const limApply = panel.querySelector('.jp-lim-apply');
  if (limApply) {
    const loIn = panel.querySelector('.jp-lo');
    const hiIn = panel.querySelector('.jp-hi');
    limApply.addEventListener('click', () => {
      const dlo = parseFloat(loIn.value), dhi = parseFloat(hiIn.value);
      if (isNaN(dlo) || isNaN(dhi) || !(dhi > dlo)) {
        log(t('jp.limBad'), 'err');
        return;
      }
      limApply.disabled = true;
      selectionState.reselectAfterLoad = childLink;     // keep this panel open after rebuild
      applyJointLimits(childLink, um.toNat(dlo), um.toNat(dhi));
    });
  }
}

