import { viewer } from './dom.js';
import { extractFlow } from './export-box.js';
import { doRename, resetAllNames } from './joint-rows.js';
import { loadRobot } from './load.js';
import { refreshHistory } from './root-frame.js';
import { op } from './session-log.js';
import { packageState } from './state.js';
import { dockPanelBelowViews } from './subassembly-choices.js';
import { openSubassemblyList } from './subassembly-preview.js';
// the ≣ 改名 list: every joint with its (editable) joint name + child link name
function openRenameList(ov = null) {
  if (!viewer.robot) { log(t('common.openFirst'), 'wrn'); return; }
  document.getElementById('masslist')?.remove();   // the two panels are exclusive
  if (!ov) { document.getElementById('renamelist')?.remove(); }
  const linkOf = (j, tag) => [...(j.urdfNode?.children ?? [])]
    .find(e => e.tagName === tag)?.getAttribute('link') ?? '';
  ov = ov || document.createElement('div');
  ov.id = 'renamelist';
  dockPanelBelowViews(ov);          // sit under the viewer's button row
  const card = document.createElement('div');
  card.className = 'card';
  const ttl = t('rename.emptyResetTitle');
  let html =
    '<div style="font-size:14px;margin-bottom:2px;color:#9fe0a8">' +
    t('rename.listTitle') + '</div>' +
    '<div style="font-size:11px;color:#8a93a3;margin-bottom:8px">' +
    t('rename.listHelp') +
    '</div>' +
    '<table class="rntable"><thead><tr><th>' + t('rename.thJoint') +
    '</th><th>' + t('rename.thParent') + '</th>' +
    '<th>' + t('rename.thChild') + '</th><th>' + t('rename.thType') +
    '</th></tr></thead><tbody>';
  for (const j of Object.values(viewer.robot.joints)) {
    const c = linkOf(j, 'child');
    html +=
      `<tr><td><input class="rn-input" data-kind="joint" title="${ttl}" ` +
      `data-old="${j.name}" value="${j.name}"></td>` +
      `<td style="color:#8a93a3">${linkOf(j, 'parent')}</td>` +
      `<td><input class="rn-input" data-kind="link" title="${ttl}" ` +
      `data-old="${c}" value="${c}"></td>` +
      `<td style="color:#9dc4ff">${j.jointType}</td></tr>`;
  }
  html += '</tbody></table>';
  card.innerHTML = html;
  const bar = document.createElement('div');
  bar.style.cssText = 'display:flex;gap:8px;margin-top:10px';
  const resetAll = document.createElement('button');
  resetAll.textContent = t('rename.resetAllBtn');
  resetAll.title = t('rename.resetAllBtnTitle');
  resetAll.addEventListener('click', resetAllNames);
  const close = document.createElement('button');
  close.textContent = t('common.close');
  close.addEventListener('click', () => ov.remove());
  bar.append(resetAll, close);
  card.appendChild(bar);
  ov.replaceChildren(card);
  card.querySelectorAll('.rn-input').forEach(inp => {
    inp.addEventListener('change', async () => {
      const v = inp.value.trim(), old = inp.dataset.old;
      if (v === old) { return; }       // unchanged; empty -> reset to default
      // a successful (re)name reloads the whole robot, so close the list
      if (await doRename(inp.dataset.kind, old, v)) { ov.remove(); }
      else { inp.value = old; }
    });
  });
  if (!ov.parentElement) {
    (document.getElementById('viewer').parentElement || document.body)
      .appendChild(ov);
  }
}
document.getElementById('renamelistbtn')
  .addEventListener('click', () => openRenameList());

export function syncSubassemblyAccessButton() {
  const button = document.getElementById('subassemblybtn');
  const hasPackage = !!packageState.currentInfo;
  const available = hasPackage && packageState.currentInfo.mode !== 'urdf';
  button.disabled = !available;
  button.style.display = hasPackage && !available ? 'none' : '';
}

document.getElementById('subassemblybtn')
  .addEventListener('click', () => openSubassemblyList());

// The SW2URDF migration toggle is only meaningful for a CAD package whose
// graph.json captured the classic add-in's embedded configuration, so it stays
// greyed out everywhere else -- with a title that says which reason applies.
export function syncSw2urdfButton() {
  const button = document.getElementById('sw2urdfbtn');
  if (!button) { return; }
  const info = packageState.currentInfo?.sw2urdf ?? null;
  const usable = !!packageState.currentInfo && packageState.currentInfo.mode !== 'urdf' && !!info?.payload;
  button.disabled = !usable;
  button.classList.toggle('active', usable && info.mode === 'sw2urdf_compat');
  button.title = !packageState.currentInfo ? t('sw2urdf.tNone')
    : packageState.currentInfo.mode === 'urdf' ? t('sw2urdf.tUrdf')
      : usable ? (info.mode === 'sw2urdf_compat' ? t('sw2urdf.tOn') : t('sw2urdf.tOff'))
        : info?.marker ? t('sw2urdf.tStale', { marker: info.marker })
          : t('sw2urdf.tNone');
  const wrap = document.getElementById('sw2urdfwrap');
  if (wrap) { wrap.title = button.title; }
}

// The mesh cache lives in the package's meshes/ and is normally self-managing:
// a CAD edit invalidates it by mtime, a configuration change by the cache
// manifest.  The button exists for the case those cannot cover -- ruling the
// cache out when an export still looks wrong -- so it only appears for a CAD
// package whose source assembly is still reachable to re-extract from.
export function syncClearMeshButton() {
  const btn = document.getElementById('clearmesh');
  if (!btn) { return; }
  const usable = !!packageState.currentInfo && packageState.currentInfo.mode === 'cad'
    && !!packageState.currentInfo.source_assembly;
  btn.style.display = usable ? '' : 'none';
  if (usable) { btn.title = t('t.clearmesh') + '\n' + packageState.currentInfo.source_assembly; }
}

export async function clearMeshCacheAndReextract() {
  if (!packageState.currentInfo?.source_assembly) { log(t('clearmesh.noSource'), 'err'); return; }
  const src = packageState.currentInfo.source_assembly;
  const cfg = packageState.currentInfo.configuration || '';
  if (!confirm(t('clearmesh.confirm', { name: packageState.currentInfo.name, src }))) { return; }
  let r;
  try {
    const resp = await fetch('/api/clear_mesh_cache', { method: 'POST' });
    r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
  } catch (e) {
    log(t('clearmesh.fail', { e: e.message ?? e }), 'err');
    return;
  }
  log(t('clearmesh.done',
        { n: r.removed, mb: (r.bytes / 1048576).toFixed(1) }), 'ok');
  // re-extract on the SAME configuration this package came from, so the only
  // thing that changed is that every mesh was rebuilt from scratch
  extractFlow(src, cfg);
}

async function toggleSw2urdfVerbatim() {
  const info = packageState.currentInfo?.sw2urdf;
  if (!packageState.currentInfo || !info?.payload) { return; }
  const mode = info.mode === 'sw2urdf_compat' ? 'auto' : 'sw2urdf_compat';
  op('sw2urdfMode', { mode });
  log(t('sw2urdf.start', { mode }));
  try {
    const resp = await fetch('/api/set_sw2urdf_mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    packageState.currentInfo.sw2urdf = r.sw2urdf ?? { ...info, mode };
    syncSw2urdfButton();
    log(t('sw2urdf.ok', { mode: packageState.currentInfo.sw2urdf.mode }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true, followCamera: true });
    refreshHistory();
  } catch (e) {
    log(t('sw2urdf.fail', { e: e.message ?? e }), 'err');
  }
}
document.getElementById('sw2urdfbtn')
  ?.addEventListener('click', toggleSw2urdfVerbatim);
syncSw2urdfButton();          // give it its "why am I grey" title before any load

