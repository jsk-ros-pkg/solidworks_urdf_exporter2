import { escAttr, urdfInputMode } from './bootstrap.js';
import { highlightJoint } from './bulk-edit.js';
import { reRoot } from './camera-reroot.js';
import { refreshCompMeta } from './capture-progress.js';
import { jointsEl, viewer } from './dom.js';
import { highlightLink } from './frames.js';
import {
  attachInlineRename, collapsed, jointLinkOf, previewJoint,
  previewJointForPlanRow, rows,
} from './joint-rows.js';
import { applyTypeChanges } from './joint-type.js';
import { selectLink } from './link-info.js';
import { hiddenLinks, toggleLinkVisible } from './link-look.js';
import { applyLinkEdit, postArgs } from './mass-editor.js';
import { _jointLimits, _jointUnit, typeOptionsHtml } from './selection.js';
import { packageState, treeState } from './state.js';
import {
  applyCollapsedPreviewChoices, bindCollapsedJointAxisChoices,
  bindSubassemblyDriverJointChoices, bindSubassemblyFrameChoices,
  bindSubassemblyParentChoices, collapsedJointAxisChoicesHtml,
  driverJointChoicesHtml, htmlEsc, parentChoicesHtml,
  subassemblyFrameChoicesHtml, validationIssuesHtml,
} from './subassembly-choices.js';
import {
  bindSubassemblyCycleBreakChoices, cycleBreakChoicesHtml,
} from './subassembly-preview.js';
export function syncTreeModeControls() {
  const subassemblyModeAvailable = packageState.currentInfo?.mode !== 'urdf';
  const effectiveMode = treeState.robotViewMode === 'collapsed_preview'
    ? 'subassembly'
    : subassemblyModeAvailable ? treeState.treeViewMode : 'expanded';
  const readonly = effectiveMode === 'subassembly';
  const treeMode = document.getElementById('treemode');
  if (treeMode) {
    treeMode.querySelector('option[value="expanded"]').textContent =
      t('tree.modeExpanded');
    treeMode.querySelector('option[value="subassembly"]').textContent =
      t('tree.modeSubassembly');
    treeMode.value = effectiveMode;
    treeMode.disabled = treeState.robotViewMode === 'collapsed_preview'
      || !subassemblyModeAvailable;
  }
  for (const id of ['selall', 'bulktype', 'bulkset', 'bulkdel']) {
    const el = document.getElementById(id);
    if (el) { el.disabled = readonly; }
  }
}

export function effectiveTreeViewMode() {
  if (treeState.robotViewMode === 'collapsed_preview') { return 'subassembly'; }
  return packageState.currentInfo?.mode === 'urdf' ? 'expanded' : treeState.treeViewMode;
}

function previewTreeLinkSet(row) {
  const links = new Set([row.link, ...(row.member_links || [])]);
  return [...links].filter(n => viewer.robot?.links?.[n]);
}

function highlightPreviewTreeRow(row, on) {
  for (const link of previewTreeLinkSet(row)) {
    highlightLink(link, on);
  }
}

function collapsedPreviewSlidersHtml(plan) {
  if (treeState.robotViewMode !== 'collapsed_preview') { return null; }
  const seen = new Set();
  const joints = [];
  for (const row of (plan?.joints || [])) {
    const name = row.name || '';
    const j = previewJointForPlanRow(row);
    if (!j || j.jointType === 'fixed' || j.mimicJoint) { continue; }
    if (!name || seen.has(j.name)) { continue; }
    seen.add(j.name);
    joints.push({ row, joint: j });
  }
  const card = document.createElement('div');
  card.className = 'joint';
  card.style.cssText = 'border:1px solid #39465a;background:#1e2630;' +
    'border-radius:4px;padding:6px 8px;margin:8px 0';
  const title = document.createElement('div');
  title.style.cssText = 'color:#cfe3ff;font-size:12px;margin-bottom:2px';
  title.textContent = t('subasm.previewSlidersTitle');
  card.appendChild(title);
  const help = document.createElement('div');
  help.style.cssText = 'color:#8a93a3;font-size:11px;margin-bottom:6px';
  help.textContent = joints.length
    ? t('subasm.previewSlidersHelp')
    : t('subasm.previewSlidersEmpty');
  card.appendChild(help);
  for (const { row, joint } of joints) {
    const [lo, hi] = _jointLimits(joint);
    const um = _jointUnit(joint.jointType);
    const fmtDisp = v => {
      let d = um.toDisp(Number(v) || 0);
      if (Object.is(d, -0) || Math.abs(d) < Math.pow(10, -um.dec) / 2) { d = 0; }
      return d.toFixed(um.dec) + um.unit;
    };
    const item = document.createElement('div');
    item.style.cssText = 'margin-top:6px';
    const label = row.child || jointLinkOf(joint, 'child') || joint.name;
    const source = row.driver_source_joint || row.source_joint || '';
    const previewName = joint.name;
    const nativeStep = um.pris ? 0.0005 : 0.01;
    item.innerHTML =
      `<div class="row1">` +
      `<span class="caret">·</span>` +
      `<span class="jname" title="${htmlEsc(previewName)}">` +
      `${htmlEsc(label)}</span>` +
      `<span class="jtype">${htmlEsc(joint.jointType)}</span>` +
      `<span class="jval">${htmlEsc(fmtDisp(Number(joint.angle) || 0))}</span>` +
      `</div>` +
      `<div style="color:#6b7480;font-size:10px;overflow:hidden;` +
      `text-overflow:ellipsis;white-space:nowrap;margin-left:17px">` +
      `${htmlEsc('preview: ' + previewName)}` +
      `${source ? htmlEsc(' / source normal: ' + source) : ''}</div>` +
      `<div class="jslider" style="margin-left:17px">` +
      `<input type="range" min="${lo}" max="${hi}" step="${nativeStep}" ` +
      `value="${Number(joint.angle) || 0}"></div>`;
    const slider = item.querySelector('input[type=range]');
    const val = item.querySelector('.jval');
    slider.addEventListener('input', () => {
      previewJoint(joint.name, parseFloat(slider.value));
    });
    item.querySelector('.jname')?.addEventListener('click', () => {
      const child = jointLinkOf(joint, 'child') || row.child;
      if (child && viewer.robot?.links?.[child]) { selectLink(child); }
    });
    rows.set(joint.name, {
      joint, row: item, slider, val, fmtDisp,
      child: jointLinkOf(joint, 'child') || row.child,
    });
    card.appendChild(item);
  }
  return card;
}

async function renderSubassemblyTreeRows(robot, movable) {
  const req = ++treeState.subasmTreeRequest;
  rows.clear();
  jointsEl.innerHTML = `<div class="joint" style="color:#8a93a3">` +
    `${t('tree.subasmLoading')}</div>`;
  const countEl = document.getElementById('jfiltercount');
  if (countEl) { countEl.textContent = ''; }
  try {
    const resp = await fetch('/api/collapse_preview');
    const r = await resp.json();
    if (req !== treeState.subasmTreeRequest || effectiveTreeViewMode() !== 'subassembly') {
      return;
    }
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    const q = treeState.jointFilter.trim().toLowerCase();
    const allRows = r.tree_rows || [];
    const treeRows = q ? allRows.filter(row =>
      [row.link, row.joint, row.source_joint, row.joint_type]
        .some(x => String(x || '').toLowerCase().includes(q))) : allRows;
    jointsEl.innerHTML = '';
    if (countEl) {
      countEl.textContent = q
        ? t('jfilter.count', { n: treeRows.length, total: allRows.length })
        : t('tree.subasmCount', { n: treeRows.length });
    }
    const validationHtml = validationIssuesHtml(r.validation);
    if (validationHtml) {
      const warn = document.createElement('div');
      warn.innerHTML = validationHtml;
      jointsEl.appendChild(warn);
    }
    const choicesHtml = parentChoicesHtml(r.parent_choices);
    if (choicesHtml) {
      const choices = document.createElement('div');
      choices.innerHTML = choicesHtml;
      jointsEl.appendChild(choices);
      bindSubassemblyParentChoices(choices);
    }
    const frameChoices = subassemblyFrameChoicesHtml(r.frame_choices);
    if (frameChoices) {
      const frames = document.createElement('div');
      frames.innerHTML = frameChoices;
      jointsEl.appendChild(frames);
    }
    const driverChoices = driverJointChoicesHtml(r.driver_joint_choices);
    if (driverChoices) {
      const drivers = document.createElement('div');
      drivers.innerHTML = driverChoices;
      jointsEl.appendChild(drivers);
    }
    const axisChoices = collapsedJointAxisChoicesHtml(r.joint_axis_choices);
    if (axisChoices) {
      const axes = document.createElement('div');
      axes.innerHTML = axisChoices;
      jointsEl.appendChild(axes);
    }
    if (frameChoices || driverChoices || axisChoices) {
      const coordinateBar = document.createElement('div');
      coordinateBar.style.cssText = 'display:flex;margin:6px 0 10px';
      const applyCoordinates = document.createElement('button');
      applyCoordinates.className = 'subasm-preview-apply';
      applyCoordinates.textContent = t('subasm.coordinateApplyBtn');
      applyCoordinates.title = t('subasm.coordinateApplyTitle');
      applyCoordinates.disabled = true;
      applyCoordinates.addEventListener('click', () =>
        applyCollapsedPreviewChoices(jointsEl, null, async () => {
          if (viewer.robot && effectiveTreeViewMode() === 'subassembly') {
            buildJointRows(viewer.robot);
          }
        }));
      coordinateBar.appendChild(applyCoordinates);
      jointsEl.appendChild(coordinateBar);
      bindSubassemblyFrameChoices(jointsEl, r.frame_choices);
      bindSubassemblyDriverJointChoices(jointsEl);
      bindCollapsedJointAxisChoices(jointsEl, r.joint_axis_choices);
    }
    const cycleChoices = cycleBreakChoicesHtml(r.cycle_break_choices);
    if (cycleChoices) {
      const cycles = document.createElement('div');
      cycles.innerHTML = cycleChoices;
      jointsEl.appendChild(cycles);
      bindSubassemblyCycleBreakChoices(cycles);
    }
    const previewSliders = collapsedPreviewSlidersHtml(r.collapse_plan);
    if (previewSliders) {
      jointsEl.appendChild(previewSliders);
    }
    const hint = document.createElement('div');
    hint.className = 'joint';
    hint.style.cssText = 'color:#8a93a3;font-size:11px';
    hint.textContent = t('tree.subasmReadonly');
    jointsEl.appendChild(hint);
    for (const row of treeRows) {
      const el = document.createElement('div');
      el.className = 'joint subasmrow';
      el.style.marginLeft = (Math.max(0, Number(row.depth || 0)) * 14) + 'px';
      const marker = row.root ? '⌂' : '·';
      const memberLinks = row.member_links || [];
      const collapsedTag = row.collapsed
        ? `<span class="jtype" style="background:#4b3d24;color:#d6b36b">` +
          `${t('subasm.collapsedTag')}</span>` +
          `<span style="color:#6b7480;font-size:10px">` +
          `${t('tree.memberCount', { n: memberLinks.length })}</span>`
        : '';
      const title = row.collapsed
        ? `${row.link}\n${memberLinks.join('\n')}` : row.link;
      el.innerHTML =
        `<div class="row1">` +
        `<span class="caret">${marker}</span>` +
        `<span class="jname" title="${htmlEsc(title)}">${htmlEsc(row.link)}` +
        `</span>` +
        collapsedTag +
        `<span class="jtype">${htmlEsc(row.joint_type || '')}</span>` +
        `<span style="color:#6b7480;font-size:10px;overflow:hidden;` +
        `text-overflow:ellipsis;white-space:nowrap">` +
        `${htmlEsc(row.joint || '')}</span>` +
        `</div>`;
      const visibleLinks = previewTreeLinkSet(row);
      if (visibleLinks.length) {
        el.querySelector('.jname').style.cursor = 'pointer';
        el.querySelector('.jname').addEventListener('click', () =>
          selectLink(visibleLinks[0]));
        el.addEventListener('mouseenter', () => highlightPreviewTreeRow(row, true));
        el.addEventListener('mouseleave', () => highlightPreviewTreeRow(row, false));
      }
      jointsEl.appendChild(el);
    }
  } catch (e) {
    if (req !== treeState.subasmTreeRequest || effectiveTreeViewMode() !== 'subassembly') {
      return;
    }
    jointsEl.innerHTML = `<div class="joint" style="color:#ffb4b4">` +
      `${htmlEsc(t('tree.subasmUnavailable', { e: e.message ?? e }))}</div>`;
    if (countEl) { countEl.textContent = ''; }
  }
  return movable;
}

export function buildJointRows(robot) {
  jointsEl.innerHTML = '';
  rows.clear();
  syncTreeModeControls();
  // the child/parent link of a joint, read off its URDF node.  buildPlayRows
  // and the rename list each declare their own copy; this one was MISSING, so
  // the name filter below threw ReferenceError -- silently, because that path
  // only runs once the user types something into #jfilter.
  const linkOf = (j, tag) => [...(j.urdfNode?.children ?? [])]
    .find(el => el.tagName === tag)?.getAttribute('link') ?? '';
  const movable = Object.values(robot?.joints ?? {})
    .filter(j => j.jointType !== 'fixed').length;
  if (effectiveTreeViewMode() === 'subassembly') {
    renderSubassemblyTreeRows(robot, movable);
    return movable;
  }
  const byParent = new Map();
  const childLinks = new Set();
  for (const j of Object.values(robot.joints)) {
    const p = jointLinkOf(j, 'parent') || (j.parent?.name ?? '');
    childLinks.add(jointLinkOf(j, 'child'));
    if (!byParent.has(p)) { byParent.set(p, []); }
    byParent.get(p).push(j);
  }
  byParent.forEach(list => list.sort(
    (a, b) => jointLinkOf(a, 'child').localeCompare(jointLinkOf(b, 'child'))));
  const roots = Object.keys(robot.links)
    .filter(n => !childLinks.has(n));
  function jointRow(j, depth, flat) {
    const isMimic = !!j.mimicJoint;
    const isMov = j.jointType !== 'fixed';
    const parent = jointLinkOf(j, 'parent');
    const child = jointLinkOf(j, 'child');
    const kids = byParent.get(child) ?? [];
    const row = document.createElement('div');
    row.className = 'joint';
    row.style.marginLeft = (depth * 14) + 'px';
    let lo = Number(j.limit.lower), hi = Number(j.limit.upper);
    if (j.jointType === 'continuous' || (lo === 0 && hi === 0)) {
      lo = -3.14; hi = 3.14;
    }
    // show the angle in DEGREES (mm for a slide), and draw ruler ticks at the
    // cardinal values (0, ±45, ±90 …) inside the range so the slider can be
    // eyeballed to a round angle.
    const um = _jointUnit(j.jointType);
    const fmtDisp = v => {
      let d = um.toDisp(v);
      if (Object.is(d, -0) || Math.abs(d) < Math.pow(10, -um.dec) / 2) { d = 0; }
      return d.toFixed(um.dec) + um.unit;
    };
    const dLo = um.toDisp(lo), dHi = um.toDisp(hi);
    const tickDisp = um.pris ? [dLo, 0, dHi]
      : [-180, -135, -90, -45, 0, 45, 90, 135, 180];
    const ticks = [...new Set(tickDisp
      .filter(d => d >= dLo - 1e-6 && d <= dHi + 1e-6)
      .map(d => Math.round(d * 1e4) / 1e4))].sort((a, b) => a - b);
    // guide marks (label + tick) at the round values, positioned along the
    // track -- a visible version of an <input list> ruler (Chrome draws that one
    // almost invisibly).  cardinal angles get a label; ±45/±135 stay tick-only
    // so a tight row doesn't get crowded.
    const span = (dHi - dLo) || 1;
    const labelAt = d => um.pris || d === 0 || Math.abs(d) === 90
      || Math.abs(d) === 180 || d === dLo || d === dHi;
    // place each tick at the THUMB CENTRE for its value: the thumb centre travels
    // from thumb/2 to width-thumb/2, so left = f*100% + (0.5-f)*thumb  (the 12px
    // must match --thumb in the CSS).  This is what makes the marks line up
    // exactly under the thumb so the magnet feels snug.
    const THUMB = 12;
    const tickBar = (isMov && !isMimic && ticks.length)
      ? `<div class="jticks">` + ticks.map(d => {
          const f = (d - dLo) / span;                  // 0..1 along the track
          const off = ((0.5 - f) * THUMB).toFixed(2);  // thumb-inset compensation
          return `<span class="jtick${d === 0 ? ' zero' : ''}" ` +
            `style="left:calc(${(f * 100).toFixed(3)}% + ${off}px)">` +
            (labelAt(d) ? `<i>${d}${um.unit}</i>` : '') + `</span>`;
        }).join('') + `</div>` : '';
    row.innerHTML =
      `<div class="row1">` +
      `<span class="caret">${(!flat && kids.length)
        ? (collapsed.has(child) ? '▸' : '▾') : '·'}</span>` +
      `<input type="checkbox" class="pick">` +
      `<span class="jname" title="${t('row.jnameTitle',
        { name: j.name, parent, child })}">` +
      `${child}</span>` +
      (isMimic ? `<span class="mimictag">mimic</span>` : '') +
      (packageState.massOnlyLinks.has(child) ? `<span class="motag">mass</span>` : '') +
      (packageState.compMeta[child]?.frame_only
        ? `<span class="frameonlytag">frame</span>` : '') +
      // the classifier's own reason, and a badge when it is one of the
      // guesses worth checking (the server decides -- _JOINT_ATTENTION)
      (packageState.compMeta[child]?.joint_attention
        ? `<span class="chktag" title="${escAttr(packageState.compMeta[child].joint_attention)}` +
          `&#10;&#10;${escAttr(packageState.compMeta[child].joint_note)}` +
          `&#10;&#10;${escAttr(t('row.chkHint'))}">?</span>`
        : (packageState.compMeta[child]?.joint_reviewed
            ? `<span class="chktag done" title="${escAttr(t('row.chkDone'))}` +
              `&#10;&#10;${escAttr(packageState.compMeta[child].joint_note)}">✓</span>`
            : '')) +
      // a mass-only link reads as 'mass_only' (its joint is fixed underneath);
      // picking any real type clears the flag, picking mass_only sets it
      `<select class="jtypesel t-${packageState.massOnlyLinks.has(child)
        ? 'mass_only' : j.jointType}">` +
      typeOptionsHtml(packageState.massOnlyLinks.has(child) ? 'mass_only' : j.jointType) +
      `</select>` +
      `<span class="jval">${isMov ? fmtDisp(Number(j.angle)) : ''}</span>` +
      // export this link's shape + weight?  Unchecked = a CAD-only part
      // (dummy axis): the link stays, but exports as a bare frame.  CAD
      // packages only -- a URDF-input link has no graph component behind it.
      (urdfInputMode()
        ? ''
        : `<input type="checkbox" class="geo"` +
          `${packageState.compMeta[child]?.frame_only ? '' : ' checked'} ` +
          `title="${escAttr(t('row.geoTitle'))}">`) +
      // a mass-only / frame-only link has no geometry to hide/show
      `<button class="eye${hiddenLinks.has(child) ? ' off' : ''}" ` +
      (packageState.massOnlyLinks.has(child)
        ? `disabled title="${t('row.eyeMassOnly')}"`
        : packageState.compMeta[child]?.frame_only
          ? `disabled title="${t('row.eyeFrameOnly')}"`
          : `title="${t('row.eyeTitle')}"`) + `>👁</button>` +
      `<button class="mkroot" title="${t('row.mkrootTitle')}">⌂</button>` +
      `</div>` +
      (isMov && !isMimic
        ? `<div class="jslider">` +
          `<input type="range" min="${lo}" max="${hi}" step="0.01" ` +
          `value="${Number(j.angle)}">` + tickBar + `</div>` : '');
    const slider = row.querySelector('input[type=range]');
    if (slider) {
      // magnetic snap: when the thumb lands within a small tolerance of a guide
      // mark (0, ±45, ±90 …) stick to that exact round value.  For a finer value
      // inside the snap zone, use the in-viewer panel's number field.
      const snapTol = um.pris ? 2 : 8;            // display units (mm / °)
      slider.addEventListener('input', () => {
        let v = parseFloat(slider.value);
        const d = um.toDisp(v);
        let best = null, bd = snapTol;
        for (const td of ticks) {
          const dist = Math.abs(td - d);
          if (dist <= bd) { bd = dist; best = td; }
        }
        if (best !== null) { v = um.toNat(best); slider.value = v; }
        previewJoint(j.name, v);
      });
    }
    const rec = { joint: j, row, parent, child,
                  // origType is the DISPLAYED type so the change handler fires
                  // correctly when toggling mass-only on/off (its joint is fixed)
                  origType: packageState.massOnlyLinks.has(child) ? 'mass_only' : j.jointType,
                  typeSel: row.querySelector('.jtypesel'),
                  pick: row.querySelector('.pick'),
                  slider, val: row.querySelector('.jval'), fmtDisp };
    rec.typeSel.addEventListener('change', () => {
      const t = rec.typeSel.value;
      rec.typeSel.className = 'jtypesel t-' + t;
      if (t !== rec.origType) {           // apply instantly -- no Apply button
        applyTypeChanges([{ name: rec.joint.name, parent: rec.parent,
                            child: rec.child, type: t }]);
      }
    });
    const chk = row.querySelector('.chktag');
    if (chk) {
      chk.addEventListener('click', async ev => {
        ev.stopPropagation();
        const on = !packageState.compMeta[child]?.joint_reviewed;
        // acknowledgement only -- it changes nothing about the model, so
        // there is no rebuild (same contract as /api/set_mass_reviewed)
        await fetch('/api/set_joint_reviewed', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ link: child, reviewed: on }) });
        await refreshCompMeta();
        if (viewer.robot) { buildJointRows(viewer.robot); }
      });
    }
    const geo = row.querySelector('.geo');
    if (geo) {
      geo.addEventListener('click', ev => ev.stopPropagation());  // not a row click
      geo.addEventListener('change', ev => {
        const frameOnly = !ev.target.checked;
        applyLinkEdit(
          postArgs('/api/set_frame_only', { link: child, on: frameOnly }),
          { rebuild: true,
            okMsg: t(frameOnly ? 'row.frameOnlyOn' : 'row.frameOnlyOff',
                     { name: child }) });
      });
    }
    row.querySelector('.mkroot').addEventListener('click',
      () => reRoot(child));
    row.classList.toggle('hiddenlink', hiddenLinks.has(child));
    row.querySelector('.eye').addEventListener('click', ev => {
      ev.stopPropagation();
      toggleLinkVisible(child, row);
    });
    // clicking the link NAME selects it (same as clicking the part in 3D);
    // double-clicking renames the link
    const jnameEl = row.querySelector('.jname');
    jnameEl.style.cursor = 'pointer';
    jnameEl.addEventListener('click', () => selectLink(child));
    attachInlineRename(jnameEl, 'link', () => child);
    row.addEventListener('mouseenter', () => {
      highlightJoint(j.name, true);
      highlightLink(child, true);
    });
    row.addEventListener('mouseleave', () => {
      highlightJoint(j.name, false);
      highlightLink(child, false);
    });
    rows.set(j.name, rec);
    jointsEl.appendChild(row);
    const caret = row.querySelector('.caret');
    if (!flat && kids.length) {
      caret.style.cursor = 'pointer';
      caret.addEventListener('click', () => {
        collapsed.has(child) ? collapsed.delete(child)
                             : collapsed.add(child);
        buildJointRows(robot);            // re-render with new fold state
      });
    }
    if (!flat && !collapsed.has(child)) {
      for (const k of kids) { jointRow(k, depth + 1); }
    }
  }

  // filtered mode: drop the tree + root heads, render a FLAT list of just the
  // joints whose child-link or joint name matches the substring.  `rows` then
  // holds exactly the matches, so select-all / Set / Delete scope to them.
  const q = treeState.jointFilter.trim().toLowerCase();
  const countEl = document.getElementById('jfiltercount');
  // "needs check": the joints the server flagged as guesses
  const flagged = Object.values(robot?.joints ?? {}).filter(
    j => packageState.compMeta[linkOf(j, 'child')]?.joint_attention);
  const chkBtn = document.getElementById('jcheckonly');
  if (chkBtn) {
    chkBtn.style.display = flagged.length ? '' : 'none';
    chkBtn.textContent = t('jfilter.checkCount', { n: flagged.length });
    chkBtn.style.opacity = treeState.jointCheckOnly ? '1' : '.55';
  }
  if (treeState.jointCheckOnly && flagged.length) {
    for (const j of flagged.sort(
        (a, b) => linkOf(a, 'child').localeCompare(linkOf(b, 'child')))) {
      jointRow(j, 0, true);
    }
    if (countEl) { countEl.textContent = ''; }
    return movable;
  }
  if (q) {
    const matches = Object.values(robot.joints)
      .filter(j => linkOf(j, 'child').toLowerCase().includes(q)
                || j.name.toLowerCase().includes(q))
      .sort((a, b) => linkOf(a, 'child').localeCompare(linkOf(b, 'child')));
    for (const j of matches) { jointRow(j, 0, true); }
    if (countEl) {
      countEl.textContent = t('jfilter.count',
        { n: matches.length, total: Object.keys(robot.joints).length });
    }
    return movable;
  }
  if (countEl) { countEl.textContent = ''; }
  for (const rootName of roots.sort()) {
    const head = document.createElement('div');
    head.className = 'joint root';
    head.innerHTML = `<div class="row1"><span class="caret">⌂</span>` +
      `<span class="jname" style="color:#fff;font-weight:bold;` +
      `cursor:pointer" title="${t('row.rootLinkTitle')}">${rootName}</span>` +
      `<span class="rootcomp" style="color:#6b7480;font-size:10px;` +
      `overflow:hidden;text-overflow:ellipsis;white-space:nowrap">` +
      `${packageState.rootBaseName && rootName === 'base_link'
        ? '= ' + packageState.rootBaseName : ''}</span>` +
      `<span class="jtype" style="background:#2d4a35;color:#8fd99f">root` +
      `</span>` +
      (packageState.compMeta[rootName]?.frame_only
        ? `<span class="frameonlytag">frame</span>` : '') +
      // same "export shape + weight" switch as the joint rows below
      (urdfInputMode()
        ? ''
        : `<input type="checkbox" class="geo"` +
          `${packageState.compMeta[rootName]?.frame_only ? '' : ' checked'} ` +
          `title="${escAttr(t('row.geoTitle'))}">`) +
      `<button class="eye${hiddenLinks.has(rootName) ? ' off' : ''}" ` +
      (packageState.compMeta[rootName]?.frame_only
        ? `disabled title="${t('row.eyeFrameOnly')}"`
        : `title="${t('row.eyeTitle')}"`) + `>👁</button></div>`;
    head.classList.toggle('hiddenlink', hiddenLinks.has(rootName));
    head.querySelector('.eye').addEventListener('click', ev => {
      ev.stopPropagation();
      toggleLinkVisible(rootName, head);
    });
    const rootGeo = head.querySelector('.geo');
    if (rootGeo) {
      rootGeo.addEventListener('click', ev => ev.stopPropagation());
      rootGeo.addEventListener('change', ev => {
        const frameOnly = !ev.target.checked;
        applyLinkEdit(
          postArgs('/api/set_frame_only', { link: rootName, on: frameOnly }),
          { rebuild: true,
            okMsg: t(frameOnly ? 'row.frameOnlyOn' : 'row.frameOnlyOff',
                     { name: rootName }) });
      });
    }
    const rootNameEl = head.querySelector('.jname');
    rootNameEl.addEventListener('click', () => selectLink(rootName));
    attachInlineRename(rootNameEl, 'link', () => rootName);
    head.addEventListener('mouseenter',
      () => highlightLink(rootName, true));
    head.addEventListener('mouseleave',
      () => highlightLink(rootName, false));
    jointsEl.appendChild(head);
    for (const j of byParent.get(rootName) ?? []) { jointRow(j, 1); }
  }
  return movable;
}

