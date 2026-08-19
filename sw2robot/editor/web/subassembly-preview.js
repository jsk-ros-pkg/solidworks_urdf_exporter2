import { viewer } from './dom.js';
import { loadRobot } from './load.js';
import { refreshHistory } from './root-frame.js';
import { op } from './session-log.js';
import { packageState } from './state.js';
import {
  applyCollapsedPreviewChoices, bindCollapsedJointAxisChoices,
  bindSubassemblyDriverJointChoices, bindSubassemblyFrameChoices,
  bindSubassemblyParentChoices, collapsedJointAxisChoicesHtml,
  dockPanelBelowViews, driverJointChoicesHtml, htmlEsc, parentChoicesHtml,
  subassemblyFrameChoicesHtml, subassemblyFrameSummaryHtml,
  validationIssuesHtml,
} from './subassembly-choices.js';
import { buildJointRows, effectiveTreeViewMode } from './tree.js';
export function cycleBreakChoicesHtml(cycleChoices) {
  const choices = (cycleChoices || []).filter(c =>
    (c.candidates || []).length || c.selected_source_joint);
  if (!choices.length) { return ''; }
  const rows = choices.map((c, i) => {
    const selected = c.selected_source_joint || '';
    const stale = !!c.stale;
    const seen = new Set();
    const opts = [`<option value=""${selected ? '' : ' selected'}>` +
      `${t('subasm.cycleBreakAuto')}</option>`].concat((c.candidates || [])
      .filter(j => {
        const source = j.source_joint || j.joint || '';
        if (!source || seen.has(source)) { return false; }
        seen.add(source);
        return true;
      }).map(j => {
        const source = j.source_joint || j.joint || '';
        const edge = j.parent && j.child
          ? `${j.parent} → ${j.child}` : source;
        return `<option value="${htmlEsc(source)}"` +
          `${selected === source ? ' selected' : ''}>` +
          `drop: ${htmlEsc(edge)}</option>`;
      })).join('');
    const path = stale
      ? `<div style="color:#d6c18c">${t('subasm.cycleBreakStale')}</div>`
      : `<div style="color:#6b7480">links: ` +
        `${(c.links || []).map(htmlEsc).join(' → ')}</div>`;
    const joints = (c.candidates || []).map(j => {
      const source = j.source_joint || j.joint || '';
      const edge = j.parent && j.child ? `${j.parent} → ${j.child}` : '';
      return `<div style="color:#6b7480">${htmlEsc(source)}` +
        `${edge ? `: ${htmlEsc(edge)}` : ''}</div>`;
    }).join('');
    return `<div style="display:grid;grid-template-columns:minmax(80px,0.6fr) ` +
      `minmax(150px,1.8fr);gap:6px;align-items:start;margin-top:5px">` +
      `<div><div style="color:#cfe3ff">${stale ? 'saved drop' : `cycle ${i + 1}`}</div>` +
      path + `</div>` +
      `<div><select class="subasm-cycle-break-choice" ` +
      `data-prev="${htmlEsc(selected)}">` + opts +
      `</select>${joints}</div></div>`;
  }).join('');
  return `<div style="border:1px solid #5a3940;background:#271e22;` +
    `border-radius:4px;padding:6px 8px;margin:8px 0;font-size:11px">` +
    `<div style="color:#e2a6b1">${t('subasm.cycleBreakChoicesTitle')}</div>` +
    rows + `</div>`;
}

async function setSubassemblyCycleBreakChoice(sourceJoint, previousSourceJoint, ov) {
  const label = sourceJoint || t('subasm.cycleBreakAuto');
  const target = sourceJoint || previousSourceJoint;
  op('subassembly_cycle_break', {
    source_joint: sourceJoint,
    previous_source_joint: previousSourceJoint,
  });
  log(t('subasm.cycleBreakSetting', { sourceJoint: label }));
  try {
    const resp = await fetch('/api/set_subassembly_cycle_break', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        source_joint: sourceJoint,
        previous_source_joint: previousSourceJoint,
        drop: !!sourceJoint,
      }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('subasm.cycleBreakSetOk', { sourceJoint: target || label }), 'ok');
    refreshHistory();
    if (ov) {
      await openSubassemblyPreview(ov);
    } else {
      document.querySelectorAll('.subasm-cycle-break-choice')
        .forEach(x => { x.disabled = false; });
    }
    if (viewer.robot && effectiveTreeViewMode() === 'subassembly') {
      buildJointRows(viewer.robot);
    }
  } catch (e) {
    log(t('subasm.cycleBreakSetFail', { e: e.message ?? e }), 'err');
    if (ov) {
      await openSubassemblyPreview(ov);
    } else {
      document.querySelectorAll('.subasm-cycle-break-choice')
        .forEach(x => { x.disabled = false; });
      if (viewer.robot && effectiveTreeViewMode() === 'subassembly') {
        buildJointRows(viewer.robot);
      }
    }
  }
}

export function bindSubassemblyCycleBreakChoices(root, ov) {
  root.querySelectorAll('.subasm-cycle-break-choice').forEach(sel => {
    sel.addEventListener('change', () => {
      const sourceJoint = sel.value;
      const previousSourceJoint = sel.dataset.prev || '';
      root.querySelectorAll('.subasm-cycle-break-choice')
        .forEach(x => { x.disabled = true; });
      setSubassemblyCycleBreakChoice(sourceJoint, previousSourceJoint, ov);
    });
  });
}

async function loadCollapsedPreviewRobot() {
  if (!packageState.currentInfo) { log(t('common.openFirst'), 'wrn'); return; }
  log(t('subasm.previewRobotLoading'));
  try {
    const resp = await fetch('/api/collapsed_preview_urdf');
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    loadRobot({
      ...packageState.currentInfo,
      name: `${packageState.currentInfo.name} collapsed preview`,
      urdf: r.urdf,
    }, {
      keepPose: false,
      previewOnly: true,
      followCamera: true,
      restoreAngles: {},
    });
    const meshIssues = (r.mesh_report || []).filter(x =>
      Number(x.missing_count || 0) || Number(x.visuals || 0) === 0);
    for (const issue of meshIssues.slice(0, 5)) {
      log(`collapsed mesh: ${issue.link} visuals=${issue.visuals} ` +
          `missing=${issue.missing_count}`, 'wrn');
    }
    log(t('subasm.previewRobotOk'), 'ok');
  } catch (e) {
    log(t('subasm.previewRobotFail', { e: e.message ?? e }), 'err');
  }
}

function loadNormalRobot() {
  if (!packageState.currentInfo) { log(t('common.openFirst'), 'wrn'); return; }
  loadRobot(packageState.currentInfo, { keepPose: false });
  log(t('subasm.normalRobotOk'), 'ok');
}

function pathBase(p) {
  const s = String(p ?? '');
  return s.split(/[\\/]/).pop() || s;
}

function subasmOverrideLabel(v) {
  if (v === 'expand') { return t('subasm.overrideExpand'); }
  if (v === 'no_expand') { return t('subasm.overrideNoExpand'); }
  return t('subasm.overrideAuto');
}

async function setSubassemblyMode(name, mode, ov) {
  op('subassembly_mode', { name, mode });
  log(t('subasm.settingPreview', { name, mode }));
  try {
    const resp = await fetch('/api/set_subassembly_mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, mode }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('subasm.setPreviewOk', { name, mode }), 'ok');
    refreshHistory();
    await openSubassemblyList(ov);
    if (viewer.robot && effectiveTreeViewMode() === 'subassembly') {
      buildJointRows(viewer.robot);
    }
  } catch (e) {
    log(t('subasm.setFail', { e: e.message ?? e }), 'err');
    await openSubassemblyList(ov);
  }
}

export async function openSubassemblyList(ov) {
  if (!viewer.robot && !packageState.currentInfo) { log(t('common.openFirst'), 'wrn'); return; }
  document.getElementById('masslist')?.remove();
  if (!ov) { document.getElementById('renamelist')?.remove(); }
  const owned = ov || document.createElement('div');
  owned.id = 'renamelist';
  dockPanelBelowViews(owned);
  try {
    const resp = await fetch('/api/subassemblies');
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    const card = document.createElement('div');
    card.className = 'card';
    let html =
      '<div style="font-size:14px;margin-bottom:2px;color:#9fe0a8">' +
      t('subasm.title') + '</div>' +
      '<div style="font-size:11px;color:#8a93a3;margin-bottom:8px">' +
      t('subasm.help') + '</div>';
    const rows = r.subassemblies || [];
    if (r.mode === 'urdf') {
      html += '<div style="color:#d6b36b">' + t('subasm.urdfMode') + '</div>';
    } else if (!rows.length) {
      html += '<div style="color:#8a93a3">' + t('subasm.none') + '</div>';
    } else {
      html += '<table class="rntable"><thead><tr><th>' +
        t('subasm.thName') + '</th><th>' + t('subasm.thLink') +
        '</th><th>' + t('subasm.thChildren') + '</th><th>' +
        t('subasm.thEdges') + '</th><th>' + t('subasm.thState') +
        '</th><th>' + t('subasm.thMode') +
        '</th><th>' + t('subasm.thPath') + '</th></tr></thead><tbody>';
      for (const s of rows) {
        const state = s.expanded ? t('subasm.stateExpanded')
                                 : t('subasm.stateKept');
        const col = s.expanded ? '#9fe0a8' : '#d6b36b';
        const modeSource = s.mode_source === 'default'
          ? `, ${t('subasm.modeDefault')}` : '';
        const opt = v => `<option value="${v}"${s.override === v ? ' selected' : ''}>` +
          `${subasmOverrideLabel(v)}</option>`;
        html += `<tr title="${htmlEsc(s.reason)}">` +
          `<td>${htmlEsc(s.name)}</td>` +
          `<td style="color:#cfe3ff">${htmlEsc(s.link_name)}</td>` +
          `<td>${s.children ?? 0}</td>` +
          `<td>${s.internal_edges ?? 0}</td>` +
          `<td><span style="color:${col}">${state}</span>` +
          ` <span style="color:#8a93a3">` +
          `(${subasmOverrideLabel(s.override)}${htmlEsc(modeSource)})</span></td>` +
          `<td><select class="subasm-mode" data-name="${htmlEsc(s.name)}" ` +
          `data-prev="${htmlEsc(s.override)}" data-children="${s.children ?? 0}" ` +
          `data-edges="${s.internal_edges ?? 0}">` +
          opt('auto') + opt('expand') + opt('no_expand') + '</select></td>' +
          `<td title="${htmlEsc(s.path)}">${htmlEsc(pathBase(s.path))}</td></tr>`;
      }
      html += '</tbody></table>';
    }
    card.innerHTML = html;
    const bar = document.createElement('div');
    bar.style.cssText = 'display:flex;gap:8px;margin-top:10px';
    const preview = document.createElement('button');
    preview.textContent = t('subasm.previewBtn');
    preview.addEventListener('click', () => openSubassemblyPreview(owned));
    const close = document.createElement('button');
    close.textContent = t('common.close');
    close.addEventListener('click', () => owned.remove());
    bar.append(preview, close);
    card.appendChild(bar);
    owned.replaceChildren(card);
    card.querySelectorAll('.subasm-mode').forEach(sel => {
      sel.addEventListener('change', () => {
        const name = sel.dataset.name;
        const mode = sel.value;
        card.querySelectorAll('.subasm-mode').forEach(x => { x.disabled = true; });
        setSubassemblyMode(name, mode, owned);
      });
    });
    if (!owned.parentElement) {
      (document.getElementById('viewer').parentElement || document.body)
        .appendChild(owned);
    }
  } catch (e) {
    log(t('subasm.fail', { e: e.message ?? e }), 'err');
  }
}

export async function openSubassemblyPreview(ov, preparedPayload = null) {
  if (!viewer.robot && !packageState.currentInfo) { log(t('common.openFirst'), 'wrn'); return; }
  const owned = ov || document.getElementById('renamelist') ||
    document.createElement('div');
  owned.id = 'renamelist';
  try {
    let r = preparedPayload;
    if (!r) {
      const resp = await fetch('/api/collapse_preview');
      r = await resp.json();
      if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    }
    const card = document.createElement('div');
    card.className = 'card';
    const cc = r.canonical_counts || {};
    const pc = r.preview_counts || {};
    let html =
      '<div style="font-size:14px;margin-bottom:2px;color:#9fe0a8">' +
      t('subasm.previewTitle') + '</div>' +
      '<div style="font-size:11px;color:#8a93a3;margin-bottom:8px">' +
      t('subasm.previewHelp') + '</div>' +
      '<div style="margin-bottom:8px;color:#cfe3ff">' +
      t('subasm.previewCounts', {
        cl: cc.links ?? 0, cj: cc.joints ?? 0,
        pl: pc.links ?? 0, pj: pc.joints ?? 0,
      }) + '</div>' +
      validationIssuesHtml(r.validation) +
      parentChoicesHtml(r.parent_choices) +
      subassemblyFrameChoicesHtml(r.frame_choices) +
      driverJointChoicesHtml(r.driver_joint_choices) +
      collapsedJointAxisChoicesHtml(r.joint_axis_choices) +
      cycleBreakChoicesHtml(r.cycle_break_choices);
    const rows = r.collapsed_subassemblies || [];
    if (!rows.length) {
      html += '<div style="color:#8a93a3">' + t('subasm.previewNone') + '</div>';
    } else {
      html += '<table class="rntable"><thead><tr><th>' +
        t('subasm.thName') + '</th><th>' + t('subasm.thLink') +
        '</th><th>' + t('subasm.thMembers') + '</th><th>' +
        t('subasm.thDropped') + '</th><th>' +
        t('subasm.thOrigin') + '</th><th>' +
        t('subasm.thMode') + '</th></tr></thead><tbody>';
      for (const s of rows) {
        html += `<tr title="${htmlEsc(s.reason)}">` +
          `<td>${htmlEsc(s.name)}</td>` +
          `<td style="color:#cfe3ff">${htmlEsc(s.link_name)}</td>` +
          `<td>${s.member_links?.length ?? 0}</td>` +
          `<td>${s.internal_joints?.length ?? 0}</td>` +
          `<td>${subassemblyFrameSummaryHtml(s)}</td>` +
          `<td>${subasmOverrideLabel(s.override)}</td></tr>`;
      }
      html += '</tbody></table>';
    }
    const treeRows = r.tree_rows || [];
    if (treeRows.length) {
      html += '<div style="font-size:13px;margin:12px 0 4px;color:#9fe0a8">' +
        t('subasm.previewTreeTitle') + '</div>' +
        '<table class="rntable"><thead><tr><th>' +
        t('subasm.thTreeLink') + '</th><th>' + t('subasm.thTreeJoint') +
        '</th><th>' + t('rename.thType') + '</th></tr></thead><tbody>';
      for (const row of treeRows) {
        const indent = Math.max(0, Number(row.depth || 0)) * 14;
        const marker = row.root ? '⌂' : '·';
        const collapsed = row.collapsed
          ? ` <span style="color:#d6b36b">(${t('subasm.collapsedTag')})</span>`
          : '';
        html += `<tr><td><span style="display:inline-block;margin-left:${indent}px">` +
          `${marker} ${htmlEsc(row.link)}${collapsed}</span></td>` +
          `<td>${htmlEsc(row.joint || '')}</td>` +
          `<td>${htmlEsc(row.joint_type || '')}</td></tr>`;
      }
      html += '</tbody></table>';
    }
    card.innerHTML = html;
    bindSubassemblyParentChoices(card, owned);
    bindSubassemblyFrameChoices(card, r.frame_choices);
    bindSubassemblyDriverJointChoices(card);
    bindCollapsedJointAxisChoices(card, r.joint_axis_choices);
    bindSubassemblyCycleBreakChoices(card, owned);
    const bar = document.createElement('div');
    bar.style.cssText = 'display:flex;gap:8px;margin-top:10px';
    const back = document.createElement('button');
    back.textContent = t('subasm.backSubasm');
    back.addEventListener('click', () => openSubassemblyList(owned));
    const applyCoordinates = document.createElement('button');
    applyCoordinates.className = 'subasm-preview-apply';
    applyCoordinates.textContent = t('subasm.coordinateApplyBtn');
    applyCoordinates.title = t('subasm.coordinateApplyTitle');
    applyCoordinates.disabled = true;
    applyCoordinates.addEventListener('click', () =>
      applyCollapsedPreviewChoices(card, owned));
    const previewRobot = document.createElement('button');
    previewRobot.className = 'subasm-preview-robot';
    previewRobot.textContent = t('subasm.previewRobotBtn');
    previewRobot.addEventListener('click', () => loadCollapsedPreviewRobot());
    const normalRobot = document.createElement('button');
    normalRobot.textContent = t('subasm.normalRobotBtn');
    normalRobot.addEventListener('click', () => loadNormalRobot());
    const close = document.createElement('button');
    close.textContent = t('common.close');
    close.addEventListener('click', () => owned.remove());
    bar.append(back, applyCoordinates, previewRobot, normalRobot, close);
    card.appendChild(bar);
    owned.replaceChildren(card);
    if (!owned.parentElement) {
      (document.getElementById('viewer').parentElement || document.body)
        .appendChild(owned);
    }
  } catch (e) {
    log(t('subasm.fail', { e: e.message ?? e }), 'err');
  }
}

