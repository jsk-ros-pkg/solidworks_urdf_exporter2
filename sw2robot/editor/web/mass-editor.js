import { refreshCompMeta } from './capture-progress.js';
import { viewer } from './dom.js';
import { loadRobot } from './load.js';
import { refreshHistory } from './root-frame.js';
import { packageState } from './state.js';
import { dockPanelBelowViews } from './subassembly-choices.js';
import { buildJointRows } from './tree.js';
// shared material/density presets (label -> density kg/m³; 'custom' = prompt),
// used by both the mass editor and the per-link select popup
const MAT_PRESETS = [['PLA', '1240'], ['ABS', '1040'], ['PETG', '1270'],
  ['Nylon', '1140'], ['POM', '1410'], ['FR4', '1850'], ['Aluminum', '2700'],
  ['Iron/Steel', '7850'], ['Titanium', '4500'], ['custom', 'custom']];

// a material picker <select>: reset + presets + custom.  `disabled` greys it out
// (used when a target mass is set -- mass and material are mutually exclusive).
export function matSelectHtml(cls, { disabled = false, style = '', selected = null } = {}) {
  // pre-select the preset matching the current density so the picker shows the
  // active material instead of resetting to the placeholder; if the density is
  // a non-preset (custom) value, fall through to the placeholder
  const sel = selected != null ? String(Math.round(Number(selected))) : null;
  const known = sel != null && MAT_PRESETS.some(([, v]) => v === sel);
  return `<select class="${cls}"${disabled ? ' disabled' : ''} style="${style}`
    + `background:#15171a;color:#ddd;border:1px solid #444;border-radius:3px;`
    + `font-size:11px">`
    + `<option value=""${known ? '' : ' selected'}>${t('li.densityPlaceholder')}</option>`
    + `<option value="__reset__">${t('li.densityReset')}</option>`
    + MAT_PRESETS.map(([l, v]) =>
        `<option value="${v}"${v === sel ? ' selected' : ''}>`
        + `${v === 'custom' ? l : l + ' ' + v}</option>`).join('')
    + `</select>`;
}

// map a material <select> value to a density: a number, null (reset to SW), or
// undefined (placeholder / cancelled custom prompt -> caller should no-op)
export function matPickDensity(val) {
  if (val === '__reset__') { return null; }
  if (val === 'custom') {
    const d = prompt(t('li.densityPrompt'));
    return d ? Number(d) : undefined;
  }
  return val ? Number(val) : undefined;
}

// the ⚖ 質量 list: per-link mass / density / mass-only, with default-mass ⚠
const _fmtMass = m => (m == null) ? '—'
  : (m >= 0.1 ? `${m.toFixed(3)} kg` : `${(m * 1000).toFixed(1)} g`);

export async function openMassList() {
  // Fully data-driven (keyed by the server's final link names): do NOT depend
  // on viewer.robot, so reopening right after an edit -- while the robot is
  // still reloading -- always repaints with the freshly-stored values instead
  // of leaving the stale panel up.
  let data;
  try {
    data = await (await fetch('/api/components')).json();
  } catch { log(t('common.openFirst'), 'wrn'); return; }
  if (data.error || !data.links || !Object.keys(data.links).length) {
    log(t('common.openFirst'), 'wrn'); return;
  }
  try {
    _renderMassList(data);
  } catch (e) {
    // never let a render error wedge the panel: drop any half-built one so the
    // button can always bring a fresh one back up
    document.getElementById('masslist')?.remove();
    log(t('mass.fail', { e: e.message ?? e }), 'err');
  }
}

// one per-link edit -> POST -> (optionally) rebuild the robot -> refresh the
// metadata and repaint both the mass popup and the joint tree.  Shared by the
// mass panel and the joint rows' frame-only checkbox.
export async function applyLinkEdit(fetchArgs, { rebuild, okMsg } = {}) {
  try {
    const resp = await fetch(...fetchArgs);
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(okMsg ?? t('mass.setOk'), 'ok');
    if (rebuild) {
      loadRobot(packageState.currentInfo, { keepPose: true });
      refreshHistory();
    }
    await refreshCompMeta();      // repaints the mass popup (see refreshCompMeta)
    // sync the main tree so a mass-only / frame-only toggle shows there too
    if (viewer.robot) { buildJointRows(viewer.robot); }
    return true;
  } catch (e) {
    log(t('mass.fail', { e: e.message ?? e }), 'err');
    return false;
  }
}
export function postArgs(path, body) { return [path, {
  method: 'POST', headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body) }]; }


function _renderMassList(data) {
  document.getElementById('renamelist')?.remove();  // the two panels are exclusive
  document.getElementById('masslist')?.remove();
  // link order: base link(s) first (no parent joint), then the rest, matching
  // the tree feel of the rename panel; parent_joint gates mass-only (fixed only)
  const links = data.links;
  const order = [...Object.keys(links)].sort((a, b) => {
    const ra = links[a].parent_joint ? 1 : 0, rb = links[b].parent_joint ? 1 : 0;
    return ra - rb || a.localeCompare(b);
  });
  const urdfMasses = data.urdf_masses ?? {};
  const massOnly = new Set(data.mass_only ?? []);
  const flaggedN = (data.default_mass_links ?? []).length;

  const ov = document.createElement('div');
  ov.id = 'masslist';
  dockPanelBelowViews(ov);          // sit under the viewer's button row
  const card = document.createElement('div');
  card.className = 'card';
  const esc = s => String(s).replace(/"/g, '&quot;');
  const rows = order.map(ln => {
    const m = data.links[ln] ?? {};
    const resolved = m.name != null;        // maps to a graph component
    const warn = m.default_mass && !m.reviewed;
    const matTxt = m.mass_overridden_in_sw ? t('mass.swOverride')
      : m.material ? m.material
      : resolved ? `<span class="mass-flag">${t('li.swUnset')}</span>`
      : '<span class="mass-note">—</span>';
    const cur = m.current_mass ?? urdfMasses[ln];
    const fixed = m.parent_joint === 'fixed' || massOnly.has(ln);
    // per-link source: a target mass OR a material/density -- a checkbox picks
    // which, and the value cell shows only that control (no disabled elements)
    const byMass = m.mass != null;
    const valueCell = byMass
      ? `<input class="rn-input mass-num mt" type="number" step="any" min="0" `
        + `placeholder="${cur != null ? cur.toFixed(3) : ''}" `
        + `value="${m.mass ?? ''}"> kg`
      : matSelectHtml('md', { style: 'width:9em;', selected: m.override ?? m.density })
        + (m.override ? `<div class="mass-note">${m.override.toFixed(0)} kg/m³ `
          + `${t('li.override')}</div>` : '');
    return `<tr class="${warn ? 'mass-warn' : ''}" data-link="${esc(ln)}">` +
      `<td class="mlink" title="${esc(ln)}">` +
      `${warn ? '<span class="mass-flag">⚠ </span>' : ''}${ln}</td>` +
      `<td class="mass-note">${matTxt}</td>` +
      `<td>${_fmtMass(cur)}</td>` +
      `<td style="text-align:center"><input type="checkbox" class="bm" ` +
      `${byMass ? 'checked' : ''} title="${t('mass.byMassTitle')}"></td>` +
      `<td>${valueCell}</td>` +
      `<td style="text-align:center">${fixed
        ? `<input type="checkbox" class="mo" ${massOnly.has(ln) ? 'checked' : ''}>`
        : `<span class="mass-note" title="${t('mass.massOnlyFixedOnly')}">—</span>`
      }</td>` +
      `<td style="text-align:center">${m.default_mass || m.reviewed
        ? `<input type="checkbox" class="rv" ${m.reviewed ? 'checked' : ''}>`
        : ''}</td>` +
      `</tr>`;
  }).join('');
  const summary = flaggedN
    ? `<div class="mass-summary warn">${t('mass.needReview', { n: flaggedN })}</div>`
    : `<div class="mass-summary ok">${t('mass.allGood')}</div>`;
  card.innerHTML =
    '<div style="font-size:14px;margin-bottom:2px;color:#9fe0a8">' +
    t('mass.listTitle') + '</div>' +
    // cap the help width so this long sentence wraps instead of stretching the
    // shrink-to-fit card wider than the table (which pushed columns off-screen)
    '<div style="font-size:11px;color:#8a93a3;margin-bottom:8px;max-width:640px">' +
    t('mass.listHelp') + '</div>' + summary +
    '<table class="rntable"><thead><tr>' +
    `<th>${t('mass.thLink')}</th><th>${t('mass.thMaterial')}</th>` +
    `<th>${t('mass.thMass')}</th><th>${t('mass.thByMass')}</th>` +
    `<th>${t('mass.thValue')}</th><th>${t('mass.thMassOnly')}</th>` +
    `<th>${t('mass.thReview')}</th>` +
    '</tr></thead><tbody>' + rows + '</tbody></table>';

  const apply = applyLinkEdit;
  const post = postArgs;

  card.querySelectorAll('tr[data-link]').forEach(tr => {
    const ln = tr.dataset.link;
    // the source toggle: check -> pin an explicit target mass at the current
    // computed mass (user then edits it); uncheck -> clear it, back to material
    tr.querySelector('.bm')?.addEventListener('change', e => {
      if (e.target.checked) {
        // `||` (not `??`): a 0 / missing current mass falls to 0.1 so the seed
        // POST is never rejected by the positive-mass check
        const seed = data.links[ln]?.current_mass || 0.1;
        apply(post('/api/set_masses', { link: ln, mass: seed }),
              { rebuild: true, okMsg: t('mass.setMassOk', { name: ln, m: seed }) });
      } else {
        apply(post('/api/set_masses', { link: ln, mass: null }), { rebuild: true });
      }
    });
    tr.querySelector('.md')?.addEventListener('change', e => {
      const d = matPickDensity(e.target.value);
      if (d === undefined) { e.target.value = ''; return; }   // placeholder / cancelled
      apply(post('/api/set_material', { link: ln, density: d }), { rebuild: true });
    });
    tr.querySelector('.mt')?.addEventListener('change', e => {
      const v = e.target.value.trim();
      apply(post('/api/set_masses', { link: ln, mass: v ? Number(v) : null }),
            { rebuild: true,
              okMsg: v ? t('mass.setMassOk', { name: ln, m: v }) : undefined });
    });
    tr.querySelector('.mo')?.addEventListener('change', e => {
      apply(post('/api/set_mass_only', { link: ln, on: e.target.checked }),
            { rebuild: true });
    });
    tr.querySelector('.rv')?.addEventListener('change', e => {
      apply(post('/api/set_mass_reviewed', { link: ln, reviewed: e.target.checked }),
            { rebuild: false });      // acknowledgement changes no geometry
    });
  });

  const bar = document.createElement('div');
  bar.style.cssText = 'display:flex;gap:8px;margin-top:10px';
  if (flaggedN) {
    const ackAll = document.createElement('button');
    ackAll.textContent = t('mass.ackAll');
    ackAll.addEventListener('click', async () => {
      for (const ln of (data.default_mass_links ?? [])) {
        await fetch(...post('/api/set_mass_reviewed', { link: ln, reviewed: true }));
      }
      log(t('mass.setOk'), 'ok');
      await refreshCompMeta();      // repaints this popup (see refreshCompMeta)
    });
    bar.appendChild(ackAll);
  }
  const close = document.createElement('button');
  close.textContent = t('common.close');
  close.addEventListener('click', () => ov.remove());
  bar.appendChild(close);
  card.appendChild(bar);
  ov.appendChild(card);
  (document.getElementById('viewer').parentElement || document.body)
    .appendChild(ov);
}
document.getElementById('masslistbtn').addEventListener('click', () => {
  // toggle: the button always responds -- close an open panel, else (re)open it
  const p = document.getElementById('masslist');
  if (p) { p.remove(); } else { openMassList(); }
});

