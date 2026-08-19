import {
  hideLoadbar, pollProgress, renderProgress, setProgressStop, showLoadbar,
} from './capture-progress.js';
import {
  collModeSel, cqualitySel, expColFmt, expLinks, expVisFmt, expmeshdir,
  exppkg, expurdf, mergeFixedBox, viewer,
} from './dom.js';
import { updateCollUI } from './export-names.js';
import { highlightLink } from './frames.js';
import { initCollision } from './link-look.js';
import { loadRobot } from './load.js';
import { openMassList } from './mass-editor.js';
import { collisionState, packageState, viewState } from './state.js';
import { GLTFLoader } from './three-setup.js';
// ---- CoACD collision preview: generate per-link convex parts in the
// background, popping each link's mesh into the viewer as it lands, and a toggle
// to overlay them (semi-transparent) on the visual meshes.
const collPreviewGenBtn = document.getElementById('coacdgen');
const collPreviewShow = document.getElementById('coacdshow');
const collPreviewProg = document.getElementById('coacdprog');       // inline summary

// blink the links currently being decomposed (several run in parallel), so it
// is obvious which parts are still computing -- toggle their hover tint on one
// shared timer
let collPreviewBlinkSet = new Set(), collPreviewBlinkTimer = null, collPreviewBlinkOn = false;
function collPreviewSetBlink(links) {
  const next = new Set(links || []);
  for (const l of collPreviewBlinkSet) {            // restore links no longer running
    if (!next.has(l)) { try { highlightLink(l, false); } catch (_e) { /* gone */ } }
  }
  collPreviewBlinkSet = next;
  if (next.size === 0) {
    if (collPreviewBlinkTimer) { clearInterval(collPreviewBlinkTimer); collPreviewBlinkTimer = null; }
    collPreviewBlinkOn = false;
    return;
  }
  if (!collPreviewBlinkTimer) {
    collPreviewBlinkTimer = setInterval(() => {
      collPreviewBlinkOn = !collPreviewBlinkOn;
      for (const l of collPreviewBlinkSet) {
        if (viewer.robot?.links?.[l]) {
          try { highlightLink(l, collPreviewBlinkOn); } catch (_e) { /* gone */ }
        }
      }
    }, 500);
  }
}
const collPreviewGroups = {};        // link name -> THREE.Object3D attached to the link
export let collPreviewPoll = null;
let collPreviewLastCurrent = null;   // last link the server reported as in-progress
const collPreviewLogged = new Set(); // links already logged as ready (avoid repeats)

export function collPreviewClearOverlay() {
  for (const [link, grp] of Object.entries(collPreviewGroups)) {
    grp.parent?.remove(grp);
    grp.traverse(o => { o.geometry?.dispose?.(); o.material?.dispose?.(); });
    delete collPreviewGroups[link];
  }
}
function collPreviewSetShown(on) {
  for (const grp of Object.values(collPreviewGroups)) { grp.visible = on; }
  viewer.redraw();          // on-demand renderer: toggling must trigger a frame
}
// When the overlay is globally HIDDEN, still reveal the collision mesh of any
// link that is currently self-colliding -- so even with "coll" off you can see
// WHICH part hit and where.  When the overlay is shown globally, the toggle
// owns visibility and this is a no-op.
export function collPreviewRevealColliding(links) {
  if (!collViewBtn || collViewBtn.classList.contains('active')) { return; }
  for (const [link, grp] of Object.entries(collPreviewGroups)) {
    grp.visible = !!(links && links.has && links.has(link));
  }
  viewer.redraw();
}
function collPreviewLoadLink(link, url) {
  if (collPreviewGroups[link]) { return; }          // already loaded this link
  const target = viewer.robot?.links?.[link];
  if (!target || !GLTFLoader) { return; }
  new GLTFLoader().load(url, g => {
    const grp = g.scene;
    grp.traverse(o => {
      if (!o.isMesh) { return; }
      o.renderOrder = 2000;                     // draw over the visual mesh
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      for (const m of mats) {
        if (!m) { continue; }
        m.transparent = true; m.opacity = 0.5;
        m.depthWrite = false; m.vertexColors = true;
      }
    });
    grp.visible = !!collPreviewShow?.checked;
    collPreviewGroups[link] = grp;
    target.add(grp);
  }, null, () => { /* a link mesh failing to load is non-fatal */ });
}
async function collPreviewTick() {
  let s;
  try { s = await (await fetch('/api/collision/preview/status')).json(); }
  catch (_e) { return; }
  // If the user switched collision mode while THIS status fetch was in flight,
  // collPreviewAbort() already nulled the poll -- bail before the finalize
  // block below can revert the selector to the old mode and re-apply its parts.
  if (!collPreviewPoll) { return; }
  const what = s.mode === 'hull' ? t('coll.hullword') : 'CoACD';
  // pop each newly-finished link into the viewer + log it once
  for (const [link, url] of Object.entries(s.parts || {})) {
    collPreviewLoadLink(link, url);
    if (!collPreviewLogged.has(link)) {
      collPreviewLogged.add(link);
      log(t('coacd.link', { done: collPreviewLogged.size, total: s.total || 0, link, what }),
          'ok');
    }
  }
  const total = s.total || 0, done = s.done || 0;
  // also surface the link currently being decomposed (the slow step), so a
  // multi-minute link does not look frozen
  if (s.running && s.current && s.current !== collPreviewLastCurrent
      && !collPreviewLogged.has(s.current)) {
    collPreviewLastCurrent = s.current;
    log(t('coacd.working', { done, total, link: s.current, what }));
  }
  // links the SERVER produced parts for (authoritative: the overlay GLBs load
  // asynchronously, so collPreviewGroups may still be empty right when the job ends)
  const produced = Object.keys(s.parts || {}).length;
  // blink ALL links currently being decomposed (they run in parallel)
  const inflight = s.running ? (s.inflight || []) : [];
  collPreviewSetBlink(inflight);
  if (s.running) {
    // paint the SAME unified panel as extract/export/sweep (issue #21) -- the
    // per-link blink/mesh-pop above stays in the viewport, unaffected
    const label = inflight.length
      ? inflight.slice(0, 2).join(', ')
        + (inflight.length > 2 ? ` +${inflight.length - 2}` : '')
      : (s.current || '');
    const msg = t('coacd.prog', { done, total, link: label, what });
    renderProgress({
      job: 'collision', running: true, done: false,
      stages: [{ name: 'build collision preview', state: 'active' }],
      frac: total ? done / total : null,
      label: 'build collision preview',
      sub: total ? msg : (label || t('coacd.start')),
      log: [],
    });
    collPreviewProg.textContent = msg;
  }
  if (!s.running) {
    hideLoadbar();
    clearInterval(collPreviewPoll); collPreviewPoll = null;
    if (collPreviewGenBtn) { collPreviewGenBtn.disabled = false; }
    if (s.error) {
      collPreviewProg.textContent = '';
      log(t('coacd.fail', { e: s.error }), 'err');
    } else if (s.cancelled) {
      collPreviewProg.textContent = t('coacd.cancelled', { n: produced });
      log(t('coacd.cancelled', { n: produced }), 'warn');
    } else {
      collPreviewProg.textContent = t('coacd.done', { n: produced });
      log(t('coacd.done', { n: produced }), 'ok');
    }
    if (produced > 0 && !collisionState.collPreviewFinalized) {
      collisionState.collPreviewFinalized = true;
      // 1) the URDF/ROS export should now ship these parts as <collision>:
      //    point the mode selector at whatever we just generated
      if (collModeSel && s.mode) { collModeSel.value = s.mode; updateCollUI(); }
      // 2) rebuild the live self-collision model so the red highlight uses the
      //    generated parts (the server dropped the old one when the job finished)
      initCollision();
      log(t('coacd.applied', { what }), 'ok');
    }
  }
}
// stop the preview (the unified panel's ■ stop registers this while it runs)
function collPreviewStop() {
  log(t('coacd.stopping'));
  fetch('/api/collision/preview/cancel').catch(() => { /* ignore */ });
}
// Abandon the in-flight generate job (the user switched collision mode mid-run).
// Stop polling FIRST so the now-cancelled job's completion can't reach the
// finalize block and revert the selector to the old mode; then tell the server
// to stop and reset the gen UI.  Unlike Stop, this discards the partial result.
export function collPreviewAbort() {
  if (collPreviewPoll) { clearInterval(collPreviewPoll); collPreviewPoll = null; }
  collPreviewSetBlink([]);                         // stop the per-link blink timer
  try { fetch('/api/collision/preview/cancel'); } catch (_e) { /* ignore */ }
  hideLoadbar();                                   // drop the unified panel
  if (collPreviewProg) { collPreviewProg.textContent = ''; }
  if (collPreviewGenBtn) { collPreviewGenBtn.disabled = false; }
}
collPreviewGenBtn?.addEventListener('click', async () => {
  if (collPreviewPoll) { return; }
  collPreviewClearOverlay();
  collPreviewLogged.clear(); collPreviewLastCurrent = null;
  collisionState.collPreviewFinalized = false;
  setCollisionShown(true);                       // watch it build by default
  collPreviewGenBtn.disabled = true;
  collPreviewProg.textContent = t('coacd.start');
  // show the unified panel immediately (same style/position as extract/export),
  // with a ■ stop control wired to cancel this job
  renderProgress({
    job: 'collision', running: true, done: false,
    stages: [{ name: 'build collision preview', state: 'active' }],
    frac: null, label: 'build collision preview', sub: t('coacd.start'), log: [],
  });
  setProgressStop(collPreviewStop);
  const mode = collModeSel?.value || 'coacd';
  const q = cqualitySel?.value || 'balanced';
  const what = mode === 'hull' ? t('coll.hullword') : 'CoACD';
  try {
    const r = await (await fetch(
      `/api/collision/preview/init?mode=${encodeURIComponent(mode)}`
      + `&quality=${encodeURIComponent(q)}`)).json();
    if (r.error) { throw new Error(r.error); }
    log(t('coacd.started', { q: mode === 'coacd' ? q : '', what }));
    collPreviewPoll = setInterval(collPreviewTick, 1000);
    collPreviewTick();
  } catch (e) {
    collPreviewGenBtn.disabled = false;
    collPreviewProg.textContent = '';
    hideLoadbar();
    log(t('coacd.fail', { e: e.message ?? e }), 'err');
  }
});
// one source of truth for "is the collision overlay shown", driven by EITHER
// the top-toolbar toggle or the export-panel checkbox (kept in sync)
const collViewBtn = document.getElementById('collview');
export function setCollisionShown(on) {
  if (collPreviewShow) { collPreviewShow.checked = on; }
  collViewBtn?.classList.toggle('active', on);
  collPreviewSetShown(on);
  // hiding the overlay shouldn't hide a CURRENT collision -- keep colliding
  // links' meshes visible
  if (!on) { collPreviewRevealColliding(collisionState.collisionLinks); }
}
collPreviewShow?.addEventListener('change', () => setCollisionShown(collPreviewShow.checked));
collViewBtn?.addEventListener('click',
  () => setCollisionShown(!collViewBtn.classList.contains('active')));

// "merge fixed" view: reload the robot from the fixed-joint-lumped URDF
// (?merged=1).  Reuses the normal load path, keeping the current pose.
const mergeViewBtn = document.getElementById('mergeview');
mergeViewBtn?.addEventListener('click', () => {
  if (!packageState.currentInfo) { return; }
  viewState.mergedView = mergeViewBtn.classList.toggle('active');
  log(t(viewState.mergedView ? 'merge.on' : 'merge.off'));
  loadRobot(packageState.currentInfo, { keepPose: true });
});
expLinks.forEach(a => a.addEventListener('click', async ev => {
  ev.preventDefault();
  // mesh format is orthogonal to ROS version: the button picks the version,
  // the selectors pick visual + collision format independently
  const ros = a.id === 'expros2' ? 2 : 1;
  const vfmt = expVisFmt?.value || 'dae';
  const cfmt = expColFmt?.value || 'stl';
  const name = (exppkg?.value || '').trim();
  const urdf = (expurdf?.value || '').trim();
  const meshdir = (expmeshdir?.value || '').trim().replace(/^\/+|\/+$/g, '');
  // collision mode + merge-fixed apply to every ROS package export (the
  // separate uniform-glb button is gone -- format is now a visual selector)
  const collMode = collModeSel?.value || 'copy';
  const mergeFixed = !!mergeFixedBox?.checked;
  const query = `ros=${ros}&meshes=${vfmt}&colfmt=${cfmt}`
    + (name ? `&name=${encodeURIComponent(name)}` : '')
    + (urdf ? `&urdf=${encodeURIComponent(urdf)}` : '')
    + (meshdir ? `&meshdir=${encodeURIComponent(meshdir)}` : '')
    + (collMode !== 'copy' ? `&collision=${collMode}` : '')
    + (collMode === 'coacd' ? `&cquality=${cqualitySel?.value || 'balanced'}` : '')
    + (mergeFixed ? '&mergefixed=1' : '');
  const what = ros === 2 ? 'ROS 2' : 'ROS 1';
  showLoadbar(t('export.bar', { what }), { indet: true });
  log(t('export.start', { what }));
  try {
    // ASYNC export: start the build (reports into _prog), watch the unified
    // progress panel live, then download the finished bytes
    let r0 = await (await fetch('/api/export/zip/start?' + query)).json();
    // default-mass gate: some links still use a guessed SolidWorks mass. Let the
    // user fix them in the mass editor, or knowingly export anyway (&ack=1).
    if (r0.default_mass_links && r0.default_mass_links.length) {
      const links = r0.default_mass_links;
      const msg = t('mass.exportBlockedTitle', { n: links.length })
        + '\n\n' + links.join('\n') + '\n\n' + t('mass.exportAnyway');
      if (!confirm(msg)) { openMassList(); return; }
      r0 = await (await fetch('/api/export/zip/start?' + query + '&ack=1')).json();
    }
    if (r0.error) { throw new Error(r0.error); }
    setProgressStop(() => {                     // ■ stop -> cooperative cancel
      log(t('export.cancelling'), 'wrn');
      fetch('/api/export/zip/cancel').catch(() => { /* ignore */ });
    });
    const { cancelled } = await pollProgress();
    if (cancelled) { log(t('export.cancelled'), 'wrn'); return; }
    const resp = await fetch('/api/export/zip/download');
    if (!resp.ok) {
      let msg = resp.status;
      try { msg = (await resp.json()).error ?? msg; } catch (_e) { /* not json */ }
      throw new Error(msg);
    }
    const blob = await resp.blob();
    const cd = resp.headers.get('Content-Disposition') || '';
    const m = cd.match(/filename="?([^"]+)"?/);
    const fn = m ? m[1] : `${name || 'package'}.zip`;
    const objurl = URL.createObjectURL(blob);
    const tmp = document.createElement('a');
    tmp.href = objurl; tmp.download = fn;
    document.body.appendChild(tmp); tmp.click(); tmp.remove();
    URL.revokeObjectURL(objurl);
    log(t('export.done', { file: fn }), 'ok');
  } catch (e) {
    log(t('export.fail', { e: e.message ?? e }), 'err');
  } finally {
    hideLoadbar();
  }
}));

