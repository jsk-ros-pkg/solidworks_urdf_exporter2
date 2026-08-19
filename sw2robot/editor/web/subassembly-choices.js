import { viewer } from './dom.js';
import { refreshHistory } from './root-frame.js';
import { op } from './session-log.js';
import { openSubassemblyPreview } from './subassembly-preview.js';
import { buildJointRows, effectiveTreeViewMode } from './tree.js';
// dock a floating list panel (#renamelist / #masslist) just below the viewer's
// button row (#views), which may wrap to several rows, instead of overlapping it
export function dockPanelBelowViews(ov) {
  const v = document.getElementById('views');
  if (v) { ov.style.top = (v.offsetTop + v.offsetHeight + 6) + 'px'; }
}

export function htmlEsc(s) {
  return String(s ?? '').replace(/[&<>"]/g, c => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'
  }[c]));
}

export function validationIssuesHtml(validation) {
  const issues = validation?.issues || [];
  if (!issues.length) { return ''; }
  function previewList(values, limit = 5) {
    const arr = (values || []).filter(v => v !== null && v !== undefined);
    if (!arr.length) { return ''; }
    const head = arr.slice(0, limit).map(htmlEsc).join(', ');
    const more = arr.length > limit ? `, +${arr.length - limit} more` : '';
    return head + more;
  }
  function issueDetailHtml(issue) {
    const parts = [];
    const parents = previewList(issue.parents);
    if (parents) {
      parts.push(`<div>candidates: ${parents}</div>`);
    }
    const joints = previewList(issue.source_joints || issue.joints);
    if (joints) {
      parts.push(`<div>joints: ${joints}</div>`);
    }
    const links = previewList(issue.links, 8);
    if (links) {
      parts.push(`<div>links: ${links}</div>`);
    }
    const comps = issue.components || [];
    if (comps.length) {
      const rows = comps.slice(0, 3).map((comp, i) => {
        const label = previewList(comp, 4);
        return `<div>group ${i + 1} (${comp.length}): ${label}</div>`;
      });
      if (comps.length > 3) {
        rows.push(`<div>+${comps.length - 3} groups</div>`);
      }
      parts.push(rows.join(''));
    }
    if (!parts.length) { return ''; }
    return `<div style="color:#c6ad79;margin:2px 0 5px 12px">` +
      parts.join('') + `</div>`;
  }
  const rows = issues.slice(0, 6).map(issue =>
    `<div style="margin-top:2px">` +
    `<span style="color:#d6b36b">${htmlEsc(issue.code || 'warning')}</span>` +
    `: ${htmlEsc(issue.message || '')}</div>` +
    issueDetailHtml(issue)).join('');
  const more = issues.length > 6
    ? `<div style="color:#8a93a3;margin-top:2px">+${issues.length - 6}</div>`
    : '';
  return `<div style="border:1px solid #6b5730;background:#2b251b;` +
    `color:#f0d49a;border-radius:4px;padding:6px 8px;margin:8px 0;` +
    `font-size:11px">` +
    `<div>${t('tree.validationTitle', { n: issues.length })}</div>` +
    rows + more + `</div>`;
}

export function parentChoicesHtml(parentChoices) {
  const choices = (parentChoices || []).filter(c =>
    (c.parents || []).length > 1 || c.selected_parent);
  if (!choices.length) { return ''; }
  const rows = choices.map(c => {
    const selected = c.selected_parent || '';
    const opts = [`<option value=""${selected ? '' : ' selected'}>` +
      `${t('subasm.parentAuto')}</option>`].concat((c.parents || []).map(p =>
      `<option value="${htmlEsc(p.link)}"${selected === p.link ? ' selected' : ''}>` +
      `${htmlEsc(p.link)}</option>`)).join('');
    const joints = (c.parents || []).map(p => {
      const js = (p.joints || []).slice(0, 3).map(htmlEsc).join(', ');
      const more = (p.joints || []).length > 3
        ? `, +${(p.joints || []).length - 3} more` : '';
      return `<div style="color:#6b7480">${htmlEsc(p.link)}: ${js}${more}</div>`;
    }).join('');
    return `<div style="display:grid;grid-template-columns:minmax(110px,1fr) ` +
      `minmax(120px,1.4fr);gap:6px;align-items:start;margin-top:5px">` +
      `<div><div style="color:#cfe3ff">${htmlEsc(c.subassembly)}</div>` +
      `<div style="color:#6b7480">${htmlEsc(c.link_name)}</div></div>` +
      `<div><select class="subasm-parent-choice" ` +
      `data-name="${htmlEsc(c.subassembly)}">` +
      opts + `</select>${joints}</div></div>`;
  }).join('');
  return `<div style="border:1px solid #3c4654;background:#1f242c;` +
    `border-radius:4px;padding:6px 8px;margin:8px 0;font-size:11px">` +
    `<div style="color:#9fb7de">${t('subasm.parentChoicesTitle')}</div>` +
    rows + `</div>`;
}

async function setSubassemblyParentChoice(name, parent, ov) {
  const label = parent || t('subasm.parentAuto');
  op('subassembly_parent', { name, parent });
  log(t('subasm.parentSetting', { name, parent: label }));
  try {
    const resp = await fetch('/api/set_subassembly_parent', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, parent }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('subasm.parentSetOk', { name, parent: label }), 'ok');
    refreshHistory();
    if (ov) {
      await openSubassemblyPreview(ov);
    }
    if (viewer.robot && effectiveTreeViewMode() === 'subassembly') {
      buildJointRows(viewer.robot);
    }
  } catch (e) {
    log(t('subasm.parentSetFail', { e: e.message ?? e }), 'err');
    if (ov) {
      await openSubassemblyPreview(ov);
    } else if (viewer.robot && effectiveTreeViewMode() === 'subassembly') {
      buildJointRows(viewer.robot);
    }
  }
}

export function bindSubassemblyParentChoices(root, ov) {
  root.querySelectorAll('.subasm-parent-choice').forEach(sel => {
    sel.addEventListener('change', () => {
      const name = sel.dataset.name;
      const parent = sel.value;
      root.querySelectorAll('.subasm-parent-choice')
        .forEach(x => { x.disabled = true; });
      setSubassemblyParentChoice(name, parent, ov);
    });
  });
}

function subassemblyFrameSourceLabel(source) {
  if (source === 'subassembly_coordinate_system') {
    return t('subasm.frameSourceSubassembly');
  }
  if (source === 'top_level_coordinate_system') {
    return t('subasm.frameSourceTop');
  }
  return t('subasm.frameSourceOrigin');
}

function subassemblyFrameSelectionLabel(selection) {
  if (selection === 'manual') { return t('subasm.frameSelectionManual'); }
  if (selection === 'base_link') { return t('subasm.frameSelectionBase'); }
  return t('subasm.frameSelectionAuto');
}

function subassemblyFrameDiagnosticsHtml(choice) {
  const source = choice.selected_frame_source || choice.configured_source ||
    choice.configured_frame_source || 'origin_link';
  const name = choice.selected_frame_name || choice.configured_frame_name ||
    t('subasm.frameAutoRepresentative');
  const selection = choice.selection || choice.frame_selection || 'auto';
  let html = `<div style="color:#a9bdd8">${htmlEsc(t('subasm.frameUsed'))}: ` +
    `<span style="color:#cfe3ff">${htmlEsc(name)}</span> ` +
    `<span style="color:#6b7480">(` +
    `${htmlEsc(subassemblyFrameSourceLabel(source))}, ` +
    `${htmlEsc(subassemblyFrameSelectionLabel(selection))})</span></div>`;
  if (choice.error || choice.frame_error) {
    html += `<div style="color:#d68c8c">` +
      `${htmlEsc(choice.error || choice.frame_error)}</div>`;
  }
  const stale = choice.stale_origin_link || '';
  if (stale) {
    html += `<div style="color:#d68c8c">` +
      `${htmlEsc(t('subasm.originStaleFallback', { link: stale }))}</div>`;
  }
  return html;
}

export function subassemblyFrameChoicesHtml(frameChoices) {
  const choices = frameChoices || [];
  if (!choices.length) { return ''; }
  const rows = choices.map(choice => {
    const source = choice.configured_source || choice.selected_frame_source ||
      'origin_link';
    const origins = choice.origin_links || [];
    const localFrames = choice.subassembly_coordinate_systems || [];
    const topFrames = choice.top_level_coordinate_systems || [];
    const sourceOption = (value, label, available = true) =>
      `<option value="${value}"${source === value ? ' selected' : ''}` +
      `${available ? '' : ' disabled'}>${htmlEsc(label)}` +
      `${available ? '' : ` (${htmlEsc(t('subasm.frameNoneFound'))})`}` +
      `</option>`;
    const sourceOptions = [
      sourceOption('origin_link', t('subasm.frameSourceOrigin')),
      sourceOption('subassembly_coordinate_system',
        t('subasm.frameSourceSubassembly'), localFrames.length > 0),
      sourceOption('top_level_coordinate_system',
        t('subasm.frameSourceTop'), topFrames.length > 0),
    ].join('');

    let candidates = origins;
    let selected = choice.selected_origin_source === 'user' &&
      !choice.stale_origin_link ? (choice.configured_origin_link || '') : '';
    const nameOptions = [];
    if (source === 'origin_link') {
      nameOptions.push(`<option value=""${selected ? '' : ' selected'}>` +
        `${htmlEsc(t('subasm.frameAutoRepresentative'))}</option>`);
    } else {
      candidates = source === 'subassembly_coordinate_system'
        ? localFrames : topFrames;
      selected = choice.configured_frame_name || choice.selected_frame_name || '';
    }
    if (selected && !candidates.includes(selected)) {
      nameOptions.push(`<option value="${htmlEsc(selected)}" selected disabled>` +
        `${htmlEsc(selected)} (${htmlEsc(t('subasm.frameUnavailable'))})</option>`);
    }
    for (const name of candidates) {
      nameOptions.push(`<option value="${htmlEsc(name)}"` +
        `${selected === name ? ' selected' : ''}>${htmlEsc(name)}</option>`);
    }
    if (!nameOptions.length) {
      nameOptions.push(`<option value="" selected disabled>` +
        `${htmlEsc(t('subasm.frameNoneFound'))}</option>`);
    }
    return `<div class="subasm-frame-row" style="display:grid;` +
      `grid-template-columns:minmax(110px,1fr) minmax(170px,2fr);` +
      `gap:6px;align-items:start;margin-top:7px">` +
      `<div><div style="color:#cfe3ff">${htmlEsc(choice.subassembly)}</div>` +
      `<div style="color:#6b7480">${htmlEsc(choice.link_name)}</div></div>` +
      `<div><select class="subasm-frame-source" ` +
      `data-name="${htmlEsc(choice.subassembly)}">${sourceOptions}</select> ` +
      `<select class="subasm-frame-name" ` +
      `data-name="${htmlEsc(choice.subassembly)}">` +
      `${nameOptions.join('')}</select>` +
      `<div style="margin-top:3px">` +
      `${subassemblyFrameDiagnosticsHtml(choice)}</div></div></div>`;
  }).join('');
  return `<div style="border:1px solid #4d4536;background:#24231d;` +
    `border-radius:4px;padding:6px 8px;margin:8px 0;font-size:11px">` +
    `<div style="color:#d6c18c">${htmlEsc(t('subasm.frameChoicesTitle'))}</div>` +
    `<div style="color:#8a93a3;margin:2px 0 5px">` +
    `${htmlEsc(t('subasm.frameChoicesHelp'))}</div>${rows}</div>`;
}

function markCollapsedPreviewChoicesDirty(root, row) {
  root.dataset.previewChoicesDirty = '1';
  row.dataset.previewChoiceDirty = '1';
  const apply = root.querySelector('.subasm-preview-apply');
  if (apply) { apply.disabled = false; }
  const preview = root.querySelector('.subasm-preview-robot');
  if (preview) { preview.disabled = true; }
}

function replaceSelectOptions(select, rows, selected) {
  select.replaceChildren(...rows.map(row => {
    const option = document.createElement('option');
    option.value = row.value;
    option.textContent = row.label;
    option.disabled = !!row.disabled;
    return option;
  }));
  select.value = selected;
}

function populateSubassemblyFrameName(row, choice, source, preferred) {
  const select = row.querySelector('.subasm-frame-name');
  let candidates = choice.origin_links || [];
  const options = [];
  if (source === 'origin_link') {
    options.push({ value: '', label: t('subasm.frameAutoRepresentative') });
  } else if (source === 'subassembly_coordinate_system') {
    candidates = choice.subassembly_coordinate_systems || [];
  } else {
    candidates = choice.top_level_coordinate_systems || [];
  }
  for (const name of candidates) {
    options.push({ value: name, label: name });
  }
  if (!options.length) {
    options.push({ value: '', label: t('subasm.frameNoneFound'), disabled: true });
  }
  const selected = candidates.includes(preferred)
    ? preferred : (source === 'origin_link' ? '' : (candidates[0] || ''));
  replaceSelectOptions(select, options, selected);
  select.disabled = !options.some(option => !option.disabled);
  return selected;
}

export function bindSubassemblyFrameChoices(root, frameChoices) {
  const byName = new Map((frameChoices || []).map(choice =>
    [choice.subassembly, choice]));
  root.querySelectorAll('.subasm-frame-row').forEach(row => {
    const sourceSelect = row.querySelector('.subasm-frame-source');
    const nameSelect = row.querySelector('.subasm-frame-name');
    const name = sourceSelect?.dataset.name || '';
    const choice = byName.get(name) || {};
    const remembered = new Map([[sourceSelect.value, nameSelect.value]]);
    row.dataset.frameSource = sourceSelect.value;
    sourceSelect.addEventListener('change', () => {
      remembered.set(row.dataset.frameSource || 'origin_link', nameSelect.value);
      const source = sourceSelect.value;
      let preferred = remembered.get(source);
      if (preferred === undefined && source === 'origin_link') {
        preferred = choice.selected_origin_source === 'user'
          ? (choice.configured_origin_link || '') : '';
      } else if (preferred === undefined && choice.configured_source === source) {
        preferred = choice.configured_frame_name || '';
      }
      const selected = populateSubassemblyFrameName(
        row, choice, source, preferred || '');
      remembered.set(source, selected);
      row.dataset.frameSource = source;
      markCollapsedPreviewChoicesDirty(root, row);
    });
    nameSelect.addEventListener('change', () => {
      remembered.set(sourceSelect.value, nameSelect.value);
      markCollapsedPreviewChoicesDirty(root, row);
    });
  });
}

export function subassemblyFrameSummaryHtml(s) {
  return subassemblyFrameDiagnosticsHtml(s);
}

export function driverJointChoicesHtml(driverChoices) {
  const choices = (driverChoices || []).filter(c =>
    (c.candidates || []).length || c.selected_driver_joint);
  if (!choices.length) { return ''; }
  const rows = choices.map(c => {
    const selected = c.selected_driver_joint || '';
    const auto = c.auto_driver_joint || '';
    const ambiguous = !!c.requires_explicit_selection;
    const effective = c.effective_driver_joint || '';
    const edge = c.edge || '';
    const edgeLabel = c.parent && c.child
      ? `${c.parent} → ${c.child}` : edge;
    const opts = [`<option value=""${selected ? '' : ' selected'}>` +
      `${t('subasm.driverJointAuto')}${auto ? `: ${htmlEsc(auto)}` : ''}` +
      `</option>`].concat((c.candidates || []).map(j => {
        const source = j.source_joint || j.joint || '';
        const edge = j.source_parent && j.source_child
          ? `${j.source_parent} → ${j.source_child}`
          : (j.parent && j.child ? `${j.parent} → ${j.child}` : source);
        const kind = j.source_kind ? `${j.source_kind} ` : '';
        const label = `${kind}${j.role || 'joint'} ${j.type || ''}: ${edge}`;
        return `<option value="${htmlEsc(source)}"` +
          `${selected === source ? ' selected' : ''}>` +
          `${htmlEsc(label)}</option>`;
      })).join('');
    const details = (c.candidates || []).slice(0, 5).map(j => {
      const source = j.source_joint || j.joint || '';
      const edge = j.source_parent && j.source_child
        ? `${j.source_parent} → ${j.source_child}` : '';
      const kind = j.source_kind ? `${j.source_kind} ` : '';
      const mov = j.movable ? 'movable' : 'fixed';
      return `<div style="color:#6b7480">${htmlEsc(kind + (j.role || 'joint'))}: ` +
        `${htmlEsc(source)} ${htmlEsc(j.type || '')} ${htmlEsc(mov)}` +
        `${edge ? ` (${htmlEsc(edge)})` : ''}</div>`;
    }).join('');
    const more = (c.candidates || []).length > 5
      ? `<div style="color:#6b7480">+${(c.candidates || []).length - 5} more</div>`
      : '';
    const diag = `<div style="color:#a9bdd8;margin-top:3px">` +
      `${htmlEsc(t('subasm.driverEffective'))}: ` +
      `<span style="color:#cfe3ff">` +
      `${htmlEsc(effective || t('subasm.driverJointNotSelected'))}</span>` +
      `</div>` +
      (selected ? `<div style="color:#6b7480">` +
        `${htmlEsc(t('subasm.driverConfigured'))}: ${htmlEsc(selected)}</div>` : '');
    return `<div class="subasm-driver-row" style="display:grid;` +
      `grid-template-columns:minmax(110px,1fr) ` +
      `minmax(150px,1.7fr);gap:6px;align-items:start;margin-top:5px">` +
      `<div><div style="color:#cfe3ff">${htmlEsc(edgeLabel)}</div>` +
      `<div style="color:#6b7480">${htmlEsc(c.subassembly || c.link_name || edge)}</div></div>` +
      `<div><select class="subasm-driver-joint-choice" ` +
      `data-edge="${htmlEsc(edge)}">` +
      opts + `</select>` +
      `${ambiguous ? `<div style="color:#e0b45d;margin-top:3px">` +
        `${htmlEsc(t('subasm.driverJointAmbiguous'))}</div>` : ''}` +
      `${diag}${details}${more}</div></div>`;
  }).join('');
  return `<div style="border:1px solid #3d5244;background:#1f2a25;` +
    `border-radius:4px;padding:6px 8px;margin:8px 0;font-size:11px">` +
    `<div style="color:#9bd6ad">${t('subasm.driverJointChoicesTitle')}</div>` +
    `<div style="color:#8a93a3;margin:2px 0 5px">` +
    `${htmlEsc(t('subasm.driverJointChoicesHelp'))}</div>` +
    rows + `</div>`;
}

export function bindSubassemblyDriverJointChoices(root) {
  root.querySelectorAll('.subasm-driver-joint-choice').forEach(sel => {
    sel.addEventListener('change', () => {
      const row = sel.closest('.subasm-driver-row');
      if (row) { markCollapsedPreviewChoicesDirty(root, row); }
    });
  });
}

export function collapsedJointAxisChoicesHtml(axisChoices) {
  const choices = (axisChoices || []).filter(choice =>
    choice.applies || choice.configured_source);
  if (!choices.length) { return ''; }
  const rows = choices.map(choice => {
    const source = choice.configured_source ||
      choice.selected_axis_source || 'normal_joint';
    const axes = choice.top_level_reference_axes || [];
    const normal = choice.normal_source_joint ||
      choice.selected_axis_name || t('subasm.driverJointNotSelected');
    const sourceOptions = [
      `<option value="normal_joint"` +
        `${source === 'normal_joint' ? ' selected' : ''}>` +
        `${htmlEsc(t('subasm.axisSourceNormal'))}</option>`,
      `<option value="top_level_reference_axis"` +
        `${source === 'top_level_reference_axis' ? ' selected' : ''}` +
        `${axes.length ? '' : ' disabled'}>` +
        `${htmlEsc(t('subasm.axisSourceTop'))}` +
        `${axes.length ? '' : ` (${htmlEsc(t('subasm.axisNoneFound'))})`}` +
        `</option>`,
    ].join('');
    const selected = choice.configured_axis_name ||
      choice.selected_axis_name || '';
    const nameOptions = [];
    if (source === 'normal_joint') {
      nameOptions.push(`<option value="" selected>${htmlEsc(normal)}</option>`);
    } else {
      if (selected && !axes.includes(selected)) {
        nameOptions.push(`<option value="${htmlEsc(selected)}" selected disabled>` +
          `${htmlEsc(selected)} (${htmlEsc(t('subasm.axisUnavailable'))})</option>`);
      }
      for (const axis of axes) {
        nameOptions.push(`<option value="${htmlEsc(axis)}"` +
          `${selected === axis ? ' selected' : ''}>${htmlEsc(axis)}</option>`);
      }
      if (!nameOptions.length) {
        nameOptions.push(`<option value="" selected disabled>` +
          `${htmlEsc(t('subasm.axisNoneFound'))}</option>`);
      }
    }
    const edgeLabel = choice.parent && choice.child
      ? `${choice.parent} → ${choice.child}` : choice.edge;
    const used = choice.selected_axis_name || selected || normal;
    const selection = choice.selection === 'manual'
      ? t('subasm.axisSelectionManual') : t('subasm.axisSelectionDefault');
    const diagnostics = `<div style="color:#a9bdd8;margin-top:3px">` +
      `${htmlEsc(t('subasm.axisUsed'))}: ` +
      `<span style="color:#cfe3ff">${htmlEsc(used)}</span> ` +
      `<span style="color:#6b7480">(${htmlEsc(selection)})</span></div>` +
      (choice.error ? `<div style="color:#d68c8c">` +
        `${htmlEsc(choice.error)}</div>` : '');
    return `<div class="subasm-axis-row" style="display:grid;` +
      `grid-template-columns:minmax(110px,1fr) minmax(170px,2fr);` +
      `gap:6px;align-items:start;margin-top:7px">` +
      `<div><div style="color:#cfe3ff">${htmlEsc(edgeLabel)}</div>` +
      `<div style="color:#6b7480">${htmlEsc(choice.joint_type || '')}</div></div>` +
      `<div><select class="subasm-axis-source" ` +
      `data-edge="${htmlEsc(choice.edge)}">${sourceOptions}</select> ` +
      `<select class="subasm-axis-name" ` +
      `data-edge="${htmlEsc(choice.edge)}"` +
      `${source === 'normal_joint' ? ' disabled' : ''}>` +
      `${nameOptions.join('')}</select>${diagnostics}</div></div>`;
  }).join('');
  return `<div style="border:1px solid #46506a;background:#20242d;` +
    `border-radius:4px;padding:6px 8px;margin:8px 0;font-size:11px">` +
    `<div style="color:#aebfe8">${htmlEsc(t('subasm.axisChoicesTitle'))}</div>` +
    `<div style="color:#8a93a3;margin:2px 0 5px">` +
    `${htmlEsc(t('subasm.axisChoicesHelp'))}</div>${rows}</div>`;
}

function populateCollapsedJointAxisName(row, choice, source, preferred) {
  const select = row.querySelector('.subasm-axis-name');
  const axes = choice.top_level_reference_axes || [];
  if (source === 'normal_joint') {
    const normal = choice.normal_source_joint ||
      choice.selected_axis_name || t('subasm.driverJointNotSelected');
    replaceSelectOptions(select, [{ value: '', label: normal }], '');
    select.disabled = true;
    return '';
  }
  const options = axes.length
    ? axes.map(name => ({ value: name, label: name }))
    : [{ value: '', label: t('subasm.axisNoneFound'), disabled: true }];
  const selected = axes.includes(preferred) ? preferred : (axes[0] || '');
  replaceSelectOptions(select, options, selected);
  select.disabled = !axes.length;
  return selected;
}

export function bindCollapsedJointAxisChoices(root, axisChoices) {
  const byEdge = new Map((axisChoices || []).map(choice =>
    [choice.edge, choice]));
  root.querySelectorAll('.subasm-axis-row').forEach(row => {
    const sourceSelect = row.querySelector('.subasm-axis-source');
    const nameSelect = row.querySelector('.subasm-axis-name');
    const edge = sourceSelect?.dataset.edge || '';
    const choice = byEdge.get(edge) || {};
    const remembered = new Map([[sourceSelect.value, nameSelect.value]]);
    row.dataset.axisSource = sourceSelect.value;
    sourceSelect.addEventListener('change', () => {
      remembered.set(row.dataset.axisSource || 'normal_joint', nameSelect.value);
      const source = sourceSelect.value;
      let preferred = remembered.get(source);
      if (preferred === undefined && choice.configured_source === source) {
        preferred = choice.configured_axis_name || '';
      }
      const selected = populateCollapsedJointAxisName(
        row, choice, source, preferred || '');
      remembered.set(source, selected);
      row.dataset.axisSource = source;
      markCollapsedPreviewChoicesDirty(root, row);
    });
    nameSelect.addEventListener('change', () => {
      remembered.set(sourceSelect.value, nameSelect.value);
      markCollapsedPreviewChoicesDirty(root, row);
    });
  });
}

function collapsedPreviewChoicePayload(root) {
  const changed = '[data-preview-choice-dirty="1"]';
  const frames = [...root.querySelectorAll(`.subasm-frame-row${changed}`)]
    .map(row => {
      const source = row.querySelector('.subasm-frame-source');
      const name = row.querySelector('.subasm-frame-name');
      return {
        name: source?.dataset.name || '',
        source: source?.value || 'origin_link',
        frame_name: name?.value || '',
      };
    });
  const drivers = [
    ...root.querySelectorAll(`.subasm-driver-row${changed}`),
  ].map(row => {
    const select = row.querySelector('.subasm-driver-joint-choice');
    return {
      edge: select?.dataset.edge || '',
      source_joint: select?.value || '',
    };
  });
  const axes = [...root.querySelectorAll(`.subasm-axis-row${changed}`)]
    .map(row => {
      const source = row.querySelector('.subasm-axis-source');
      const name = row.querySelector('.subasm-axis-name');
      return {
        edge: source?.dataset.edge || '',
        source: source?.value || 'normal_joint',
        axis_name: name?.value || '',
      };
    });
  return { frames, drivers, axes };
}

function setCollapsedPreviewControlsDisabled(root, disabled) {
  root.querySelectorAll(
    '.subasm-frame-source, .subasm-driver-joint-choice, .subasm-axis-source')
    .forEach(select => { select.disabled = disabled; });
  root.querySelectorAll('.subasm-frame-name').forEach(select => {
    const available = [...select.options].some(option => !option.disabled);
    select.disabled = disabled || !available;
  });
  root.querySelectorAll('.subasm-axis-name').forEach(select => {
    const source = select.closest('.subasm-axis-row')
      ?.querySelector('.subasm-axis-source')?.value || 'normal_joint';
    const available = [...select.options].some(option => !option.disabled);
    select.disabled = disabled || source === 'normal_joint' || !available;
  });
  const apply = root.querySelector('.subasm-preview-apply');
  if (apply) { apply.disabled = disabled; }
}

export async function applyCollapsedPreviewChoices(root, ov, onApplied = null) {
  if (root.dataset.previewChoicesDirty !== '1') { return; }
  const choices = collapsedPreviewChoicePayload(root);
  op('collapsed_coordinate_choices', choices);
  log(t('subasm.coordinateApplySetting'));
  setCollapsedPreviewControlsDisabled(root, true);
  try {
    const resp = await fetch('/api/set_collapsed_preview_choices', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(choices),
    });
    const result = await resp.json();
    if (!resp.ok || result.error) {
      throw new Error(result.error ?? resp.status);
    }
    log(t('subasm.coordinateApplyOk', {
      frames: result.frames_applied ?? choices.frames.length,
      drivers: result.drivers_applied ?? choices.drivers.length,
      axes: result.axes_applied ?? choices.axes.length,
    }), 'ok');
    refreshHistory();
    if (onApplied) {
      await onApplied(result);
    } else {
      await openSubassemblyPreview(ov, result);
    }
    if (!onApplied && viewer.robot &&
        effectiveTreeViewMode() === 'subassembly') {
      buildJointRows(viewer.robot);
    }
  } catch (e) {
    log(t('subasm.coordinateApplyFail', { e: e.message ?? e }), 'err');
    setCollapsedPreviewControlsDisabled(root, false);
    const apply = root.querySelector('.subasm-preview-apply');
    if (apply) { apply.disabled = false; }
  }
}

