import { ownLinkBox } from './axis-markers.js';
import { escAttr } from './bootstrap.js';
import { boxSelected, clearBoxSelection } from './box-select.js';
import { reRoot } from './camera-reroot.js';
import { refreshCompMeta } from './capture-progress.js';
import { viewer } from './dom.js';
import { _tintLink } from './frames.js';
import { playRows, rows } from './joint-rows.js';
import {
  _colorTint, applyLinkColor, beginColorPreview, hiddenLinks, recentColors,
  setLinkColor, toggleLinkVisible,
} from './link-look.js';
import { loadRobot } from './load.js';
import { matPickDensity, matSelectHtml } from './mass-editor.js';
import { refreshHistory } from './root-frame.js';
import {
  jointPanelHtml, parseInertial, revealSelectedJointAxis,
  showSelectionVisuals, wireJointPanel,
} from './selection.js';
import { op } from './session-log.js';
import { packageState, selectionState, treeState } from './state.js';
import { dockPanelBelowViews } from './subassembly-choices.js';
import { THREE } from './three-setup.js';
export function fillLinkInfo(name) {
  selectionState.jpSync = null;                  // the panel for the previous link is gone
  const el = document.getElementById('linkinfo');
  const link = viewer.robot?.links?.[name];
  if (!link) { el.style.display = 'none'; return; }
  // the selection bar (name + 👁/🗑/⌂/✕) is folded in as this panel's header so
  // it is ONE panel, not two overlapping ones.  Park it outside #linkinfo before
  // the innerHTML rebuild below so the element (and its listeners) survives.
  const selbar = document.getElementById('selbar');
  if (selbar && selbar.parentElement === el) {
    document.getElementById('viewwrap').appendChild(selbar);
  }
  const j = link.parent?.isURDFJoint ? link.parent : null;
  const box = ownLinkBox(link, new THREE.Box3());
  const size = box.isEmpty() ? null : box.getSize(new THREE.Vector3());
  let tris = 0, mat = null, meshFile = null;
  (function walk(n) {
    for (const c of n.children) {
      if (c.isURDFJoint || c.userData.sw2robotMarker) { continue; }
      if (c.isMesh) {
        const idx = c.geometry.index;
        tris += (idx ? idx.count : c.geometry.attributes.position.count) / 3;
        if (!mat) { mat = c.__origMaterial ?? c.material; }
      }
      walk(c);
    }
  })(link);
  const vis = [...(link.urdfNode?.children ?? [])]
    .find(e => e.tagName === 'visual');
  meshFile = vis?.querySelector('mesh')?.getAttribute('filename')
    ?.split('/').pop() ?? t('common.none');
  const inert = parseInertial(link);
  const mm = v => (v * 1000).toFixed(1);
  const rowsHtml = [];
  const add = (k, v) => rowsHtml.push(`<tr><td>${k}</td><td>${v}</td></tr>`);
  // the joint editor lives in its own panel (prepended below); here we only
  // add the static axis read-out (movable joints) or the root marker
  let jParentLink = '';
  if (j) {
    jParentLink = [...(j.urdfNode?.children ?? [])]
      .find(e => e.tagName === 'parent')?.getAttribute('link')
      ?? (j.parent?.name ?? '');
    if (j.jointType !== 'fixed' && j.axis) {
      add(t('li.jointAxis'), `(${[...j.axis.toArray()]
        .map(x => x.toFixed(2)).join(', ')})`);
    }
  } else {
    add(t('li.parentJoint'), t('li.rootLink'));
  }
  if (size) {
    add(t('li.bbox'), `${mm(size.x)} × ${mm(size.y)} × ${mm(size.z)} mm`);
  }
  // CAD packages show inertia read-only (it is derived from mesh + density);
  // a directly-opened URDF lets the user edit it (URDF-input mode).
  const urdfMode = packageState.currentInfo?.mode === 'urdf';
  if (inert && !urdfMode) {
    add(t('li.mass'), inert.mass >= 0.1
      ? `${inert.mass.toFixed(3)} kg` : `${(inert.mass * 1000).toFixed(1)} g`);
    add(t('li.com'), `(${inert.com.map(mm).join(', ')}) mm`);
    if (inert.inertia) {
      const ii = inert.inertia;
      add(t('li.inertia'), `ixx=${ii.ixx.toExponential(2)} ` +
          `iyy=${ii.iyy.toExponential(2)} izz=${ii.izz.toExponential(2)}`);
    }
  }
  if (urdfMode) {
    // editable inertial -- mass (kg), CoM origin (m), inertia tensor (kg·m²),
    // matching the URDF's own SI units; applied together via the button below
    const im = inert || { mass: 0.1, com: [0, 0, 0], inertia: null };
    const ii = im.inertia
      || { ixx: 1e-4, ixy: 0, ixz: 0, iyy: 1e-4, iyz: 0, izz: 1e-4 };
    const niSt = 'width:62px;background:#15171a;color:#ddd;border:1px solid #444;'
      + 'border-radius:3px;font-size:11px';
    const ni = (id, v) =>
      `<input type="number" step="any" id="${id}" value="${v}" style="${niSt}">`;
    add(t('li.editMass'), `${ni('li_mass', im.mass)} kg`);
    add(t('li.editCom'),
      `${ni('li_cx', im.com[0])}${ni('li_cy', im.com[1])}${ni('li_cz', im.com[2])} m`);
    add(t('li.editInertia'),
      `ixx${ni('li_ixx', ii.ixx)} iyy${ni('li_iyy', ii.iyy)} izz${ni('li_izz', ii.izz)}`
      + `<br>ixy${ni('li_ixy', ii.ixy)} ixz${ni('li_ixz', ii.ixz)} `
      + `iyz${ni('li_iyz', ii.iyz)}`);
    rowsHtml.push('<tr><td></td><td><button id="li_inertial_apply" '
      + 'style="font-size:11px;background:#15171a;color:#ddd;border:1px solid #444;'
      + `border-radius:3px;padding:2px 10px;cursor:pointer">${t('li.inertialApply')}`
      + '</button></td></tr>');
  }
  const c = mat?.color?.getHexString?.();
  // editable colour: the picker shows the persisted override if any, else the
  // mesh's own colour; the 'reset' button (active only when overridden) clears
  // the override back to the CAD colour
  const ov = packageState.linkColors[name];                         // '#rrggbb' or undefined
  const cur = (ov || (c ? '#' + c : '#cccccc')).toLowerCase();
  rowsHtml.push(
    `<tr><td>${t('li.color')}</td><td>` +
    `<input type="color" id="li_color" value="${cur}" ` +
    `style="vertical-align:middle;width:36px;height:20px;padding:0;` +
    `background:#15171a;border:1px solid #444;border-radius:3px;` +
    `cursor:pointer">` +
    ` <button id="li_color_reset" title="${t('li.colorResetTitle')}" ` +
    `style="font-size:10px;background:#15171a;color:#bbb;border:1px solid #444;` +
    `border-radius:3px;padding:1px 6px;cursor:pointer"` +
    `${ov ? '' : ' disabled'}>${t('li.colorReset')}</button>` +
    (ov ? ` <span style="color:#e8c468">${t('li.override')}</span>` : '') +
    `</td></tr>`);
  // reusable palette of recently-set colours: click a swatch to apply it
  if (recentColors.length) {
    const sw = recentColors.map(h =>
      // pointer-events:auto -- #linkinfo is pointer-events:none and only
      // re-enables select/button/input, so a bare <span> swatch would be unclickable
      `<span class="palsw" data-c="${h}" title="${h}" style="display:inline-block;` +
      `width:16px;height:16px;border-radius:3px;margin:1px;cursor:pointer;` +
      `pointer-events:auto;` +
      `vertical-align:middle;border:1px solid ${h === cur ? '#e8c468' : '#444'};` +
      `background:${h}"></span>`).join('');
    rowsHtml.push(`<tr><td>${t('li.recentColors')}</td><td>${sw}</td></tr>`);
  }
  // SolidWorks material + density (with web override), editable below
  const meta = packageState.compMeta[name];
  const matTxt = meta?.override
    ? `${meta.override} kg/m³ <span style="color:#e8c468">${t('li.override')}</span>`
    : meta?.material
      ? `${meta.material}${meta.density
          ? ` · ${meta.density.toFixed(0)} kg/m³` : ''}`
      : meta ? t('li.swUnset') : t('li.reextract');
  add(t('li.material'), matTxt);
  add(t('li.mesh'), `${meshFile} · ${Math.round(tris)} tris`);
  // mass source: a checkbox picks target-mass vs material/density, then only
  // the relevant control is shown (no disabled fields).  CAD mode only -- a
  // URDF-input link edits its inertial through the block above.
  const byMass = meta?.mass != null;
  // the same per-link switch the joint rows carry: export this link's shape and
  // weight, or emit it as a bare frame (CAD-only dummy parts)
  const frameOnly = !!meta?.frame_only;
  if (!urdfMode) {
    rowsHtml.push(
      `<tr><td>${t('li.exportShape')}</td><td>` +
      `<input type="checkbox" id="li_geo" ${frameOnly ? '' : 'checked'} ` +
      `title="${escAttr(t('row.geoTitle'))}"> ${t('li.exportShapeLabel')}` +
      (frameOnly
        ? `<div class="mass-note">${t('li.exportShapeOff')}</div>` : '') +
      `</td></tr>`);
  }
  if (!urdfMode) {
    rowsHtml.push(
      `<tr><td>${t('mass.thByMass')}</td><td>` +
      `<input type="checkbox" id="li_bm" ${byMass ? 'checked' : ''} ` +
      `title="${t('mass.byMassTitle')}"> ${t('mass.byMass')}</td></tr>`);
  }
  if (byMass && !urdfMode) {
    rowsHtml.push(
      `<tr><td>${t('mass.thTarget')}</td><td>` +
      `<input id="li_masst" type="number" step="any" min="0" ` +
      `placeholder="${meta?.current_mass != null ? meta.current_mass.toFixed(3) : ''}" ` +
      `value="${meta?.mass ?? ''}" style="width:80px;background:#15171a;color:#ddd;` +
      `border:1px solid #444;border-radius:3px;font-size:11px"> kg</td></tr>`);
  } else {
    rowsHtml.push(
      `<tr><td>${t('li.setMaterial')}</td><td>` +
      matSelectHtml('li_mat', { style: 'width:100%;',
        selected: meta?.override ?? meta?.density }) +
      (meta?.override ? `<div class="mass-note">${meta.override.toFixed(0)} kg/m³ `
        + `${t('li.override')}</div>` : '') +
      `</td></tr>`);
  }
  const jointHtml = j ? jointPanelHtml(j, jParentLink, name) : '';
  el.innerHTML = jointHtml + `<table>${rowsHtml.join('')}</table>`;
  // re-insert the selection bar as the panel header (its buttons keep their
  // listeners since the element was only moved, never recreated)
  if (selbar) {
    Object.assign(selbar.style, { position: 'static', border: 'none',
      background: 'none', padding: '0', margin: '0 0 6px', width: 'auto',
      flexWrap: 'wrap' });                 // let the buttons drop to a 2nd row
    // name takes the whole first row (truncated); the action buttons wrap below
    const nm = document.getElementById('selname');
    if (nm) {
      Object.assign(nm.style, { flexBasis: '100%', minWidth: '0',
        overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' });
    }
    el.insertBefore(selbar, el.firstChild);
  }
  if (j) { wireJointPanel(el, j); }
  // source toggle: check -> pin an explicit target mass at the current mass;
  // uncheck -> clear it, back to material / density
  const setMass = async (mass) => {
    try {
      const resp = await fetch('/api/set_masses', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ link: name, mass }) });
      const r = await resp.json();
      if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
      log(mass != null ? t('mass.setMassOk', { name, m: mass }) : t('mass.setOk'), 'ok');
      // refresh compMeta FIRST so the reselect below re-renders the popup in the
      // new state (checkbox / override), not the stale pre-edit one
      await refreshCompMeta();
      selectionState.reselectAfterLoad = name;   // reopen this popup when the reload completes
      loadRobot(packageState.currentInfo, { keepPose: true });
      refreshHistory();
    } catch (err) { log(t('mass.fail', { e: err.message ?? err }), 'err'); }
  };
  // export shape + weight, or emit a bare frame (same switch as the joint row).
  // Follows setMass' order: refresh the metadata BEFORE the reload so the panel
  // that reopens shows the new state, not the stale pre-edit one.
  el.querySelector('#li_geo')?.addEventListener('change', async e => {
    const off = !e.target.checked;
    try {
      const resp = await fetch('/api/set_frame_only', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ link: name, on: off }) });
      const r = await resp.json();
      if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
      log(t(off ? 'row.frameOnlyOn' : 'row.frameOnlyOff', { name }), 'ok');
      await refreshCompMeta();
      selectionState.reselectAfterLoad = name;   // reopen this popup when the reload completes
      loadRobot(packageState.currentInfo, { keepPose: true });
      refreshHistory();
    } catch (err) { log(t('mass.fail', { e: err.message ?? err }), 'err'); }
  });
  el.querySelector('#li_bm')?.addEventListener('change', e => {
    // `||` (not `??`): a 0 / missing current mass falls to 0.1 so the seed
    // POST is never rejected by the positive-mass check
    setMass(e.target.checked ? (meta?.current_mass || 0.1) : null);
  });
  const sel = el.querySelector('.li_mat');    // matSelectHtml emits a class (density mode only)
  sel?.addEventListener('change', async () => {
    const d = matPickDensity(sel.value);      // number, null (reset), or undefined
    if (d === undefined) { sel.selectedIndex = 0; return; }
    op('setMaterial', { link: name, density: d });
    log(t('li.settingDensity', { name, d: d ?? t('li.swValue') }));
    try {
      const resp = await fetch('/api/set_material', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ link: name, density: d }) });
      const r = await resp.json();
      if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
      log(t('li.densityOk'), 'ok');
      await refreshCompMeta();    // fresh meta so the reselect shows the override
      selectionState.reselectAfterLoad = name;   // reopen this popup when the reload completes
      loadRobot(packageState.currentInfo, { keepPose: true });
      refreshHistory();
    } catch (e) {
      log(t('li.densityFail', { e: e.message ?? e }), 'err');
    }
  });
  el.querySelector('#li_masst')?.addEventListener('change', (e) => {
    const v = e.target.value.trim();
    setMass(v ? Number(v) : null);
  });
  const colInp = el.querySelector('#li_color');
  if (colInp) {
    // focus strips the selection tint so the picker shows the TRUE colour;
    // 'input' previews live while dragging; 'change' persists once.  The preview
    // lingers (no blur restore) until another link is selected, so the applied
    // colour stays visible instead of being re-masked by the selection cyan.
    colInp.addEventListener('focus', () => beginColorPreview(name));
    colInp.addEventListener('input', () => applyLinkColor(name, colInp.value));
    colInp.addEventListener('change', () => setLinkColor(name, colInp.value));
  }
  const colReset = el.querySelector('#li_color_reset');
  if (colReset) {
    colReset.addEventListener('click', () => setLinkColor(name, null));
  }
  el.querySelectorAll('.palsw').forEach(s =>
    s.addEventListener('click', () => setLinkColor(name, s.dataset.c)));
  const inApply = el.querySelector('#li_inertial_apply');
  if (inApply) {
    inApply.addEventListener('click', async () => {
      const num = id => parseFloat(el.querySelector('#' + id).value);
      const mass = num('li_mass');
      const com = [num('li_cx'), num('li_cy'), num('li_cz')];
      const inertia = [num('li_ixx'), num('li_ixy'), num('li_ixz'),
                       num('li_iyy'), num('li_iyz'), num('li_izz')];
      if (![mass, ...com, ...inertia].every(Number.isFinite)) {
        log(t('li.inertialBadNum'), 'err');   // an empty field -> NaN; don't post
        return;
      }
      const body = { link: name, mass, com, inertia };
      op('setInertial', { link: name });
      log(t('li.settingInertial', { name }));
      try {
        const resp = await fetch('/api/set_inertial', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body) });
        const r = await resp.json();
        if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
        log(t('li.inertialOk'), 'ok');
        loadRobot(packageState.currentInfo, { keepPose: true });   // re-read the edited URDF
        refreshHistory();
        const reselect = name;
        setTimeout(() => selectLink(reselect), 1500);  // reopen panel after rebuild
      } catch (e) {
        log(t('li.inertialFail', { e: e.message ?? e }), 'err');
      }
    });
  }
  dockPanelBelowViews(el);      // sit under the viewer's button row (was 76px)
  el.style.display = 'block';   // '' would fall back to the stylesheet's
                                // display:none and never show
}

export function selectLink(linkName) {
  op('select', { link: linkName });
  if (linkName && boxSelected.size) { clearBoxSelection(); }  // single replaces box
  // moving to a different link ends any colour preview, so the link we leave
  // gets its normal resting tint back
  if (linkName !== selectionState.colorPreviewLink) { selectionState.colorPreviewLink = null; }
  const prev = selectionState.selectedLink;
  selectionState.selectedLink = linkName;
  if (prev) { _tintLink(prev, _colorTint(prev)); }
  const bar = document.getElementById('selbar');
  if (linkName) {
    _tintLink(linkName, _colorTint(linkName));   // red survives selection
    statusEl.textContent = t('sel.status', { name: linkName });
    document.getElementById('selname').textContent = '🔎 ' + linkName;
    document.getElementById('seleye').classList.toggle(
      'off', hiddenLinks.has(linkName));
    bar.style.display = 'flex';
    showSelectionVisuals(linkName);
    revealSelectedJointAxis(linkName);    // show this joint's axis (even fixed)
    fillLinkInfo(linkName);
    const rec = [...rows.values()].find(r => r.child === linkName);
    rec?.row.scrollIntoView({ block: 'nearest' });
    if (treeState.playMode) {                       // 3D click -> jump to the play slider
      for (const r of playRows.values()) { r.row.classList.remove('hl'); }
      const pr = [...playRows.values()].find(r => r.child === linkName);
      if (pr) { pr.row.classList.add('hl'); pr.row.scrollIntoView({ block: 'nearest' }); }
    }
  } else {
    if (treeState.playMode) {
      for (const r of playRows.values()) { r.row.classList.remove('hl'); }
    }
    bar.style.display = 'none';
    selectionState.selVis?.removeFromParent();
    selectionState.selVis = null;
    revealSelectedJointAxis(null);        // hide the de-selected joint's axis
    selectionState.jpSync = null;
    document.getElementById('linkinfo').style.display = 'none';
    viewer.redraw();
  }
}
document.getElementById('selroot').addEventListener('click', () => {
  if (selectionState.selectedLink) { const l = selectionState.selectedLink; selectLink(null); reRoot(l); }
});
document.getElementById('seldesel').addEventListener('click',
  () => selectLink(null));
document.getElementById('seleye').addEventListener('click', () => {
  if (!selectionState.selectedLink) { return; }
  const rec = [...rows.values()].find(r => r.child === selectionState.selectedLink);
  toggleLinkVisible(selectionState.selectedLink, rec?.row);
  document.getElementById('seleye').classList.toggle(
    'off', hiddenLinks.has(selectionState.selectedLink));
});

