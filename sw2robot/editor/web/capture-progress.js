import { viewer } from './dom.js';
import { applyPersistedColors } from './link-look.js';
import { openMassList } from './mass-editor.js';
import { packageState } from './state.js';
import {
  ColladaLoader, GLTFLoader, OBJLoader, STLLoader, THREE, mergeGeometries,
} from './three-setup.js';
// ---- screen recording: capture the whole tab (all UI + the keycast overlay)
// to a .webm via getDisplayMedia + MediaRecorder.  The browser asks "share
// this tab" once per recording (the required user gesture); after that it is
// hands-off.  Pairs with replay: Shift-click records a replay end to end. ------
export let _rec = null;                 // { recorder, chunks, stream } while recording
let _recStarting = false;        // guards the async gap before _rec is assigned
export async function startScreenRecording() {
  if (_rec || _recStarting) { return _rec; }   // already recording / share dialog open
  if (!navigator.mediaDevices?.getDisplayMedia || !window.MediaRecorder) {
    log(t('rec.unsupported'), 'err'); return null;
  }
  let stream;
  _recStarting = true;
  try {
    stream = await navigator.mediaDevices.getDisplayMedia({
      video: { frameRate: 30 }, audio: false });
  } catch { log(t('rec.cancelled'), 'wrn'); return null; }
  finally { _recStarting = false; }
  const mime = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm']
    .find(m => MediaRecorder.isTypeSupported(m)) || 'video/webm';
  const chunks = [];
  const recorder = new MediaRecorder(stream, { mimeType: mime });
  recorder.ondataavailable = e => { if (e.data.size) { chunks.push(e.data); } };
  recorder.onstop = () => {
    stream.getTracks().forEach(tr => tr.stop());
    const blob = new Blob(chunks, { type: mime });
    const pkg = (packageState.currentInfo?.name || 'sw2robot').replace(/[^\w.-]+/g, '_');
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url; a.download = `${pkg}-demo.webm`;
    document.body.appendChild(a); a.click(); a.remove();   // in-DOM click for Firefox
    URL.revokeObjectURL(url);
    log(t('rec.saved', { file: a.download, kb: Math.round(blob.size / 1024) }), 'ok');
  };
  // the browser's own "Stop sharing" chip ends the track -> finalise cleanly
  stream.getVideoTracks()[0].addEventListener('ended', stopScreenRecording);
  recorder.start();
  _rec = { recorder, chunks, stream };
  document.getElementById('record')?.classList.add('active');
  log(t('rec.on'), 'ok');
  return _rec;
}
export function stopScreenRecording() {
  if (!_rec) { return; }
  const r = _rec; _rec = null;
  document.getElementById('record')?.classList.remove('active');
  if (r.recorder.state !== 'inactive') { r.recorder.stop(); }  // onstop downloads
}

export async function refreshCompMeta() {
  try {
    const r = await (await fetch('/api/components')).json();
    if (!r.error) {
      packageState.compMeta = r.links ?? {};
      packageState.linkColors = r.colors ?? {};   // keyed by viewer link name (composed too)
      applyPersistedColors();     // meta may arrive after geometry-loaded
      packageState.excludedList = r.excluded ?? [];
      // mass-only links (final URDF link names == the viewer's link names)
      packageState.massOnlyLinks = new Set(r.mass_only ?? []);
      const chip = document.getElementById('exclchip');
      chip.style.display = packageState.excludedList.length ? '' : 'none';
      chip.textContent = t('excl.chip', { n: packageState.excludedList.length });
      chip.title = packageState.excludedList.join('\n') + '\n' + t('excl.restoreAll');
      // keep an open mass-editor popup in sync with mass-only / material edits
      // made from the main panel (both read the same /api/components data)
      if (document.getElementById('masslist')) { openMassList(); }
    }
  } catch { /* no package yet */ }
}

// ---- centre loading bar --------------------------------------------------
const loadbarEl = document.getElementById('loadbar');
let loadbarTimer = null;

export function showLoadbar(text, { indet = false, delay = 0 } = {}) {
  clearTimeout(loadbarTimer);
  const show = () => {
    loadbarEl.querySelector('.lb-text').textContent = text;
    loadbarEl.querySelector('.lb-sub').textContent = '';
    // a plain loadbar has no stage checklist / log tail -- hide any left over
    // from a previous unified-progress run so it doesn't bleed through
    loadbarEl.querySelector('.lb-stages').style.display = 'none';
    loadbarEl.querySelector('.lb-log').style.display = 'none';
    clearProgressStop();          // a plain loadbar carries no cancel control
    loadbarEl.classList.toggle('indet', indet);
    if (indet) { loadbarEl.querySelector('.lb-fill').style.width = '30%'; }
    loadbarEl.style.display = 'block';
  };
  if (delay) { loadbarTimer = setTimeout(show, delay); }
  else { show(); }
}

function updateLoadbar(frac, sub, text) {
  if (loadbarEl.style.display === 'none') { return; }
  if (text != null) {
    loadbarEl.querySelector('.lb-text').textContent = text;
  }
  if (frac != null) {
    loadbarEl.classList.remove('indet');
    loadbarEl.querySelector('.lb-fill').style.width =
      `${Math.round(frac * 100)}%`;
  }
  if (sub != null) { loadbarEl.querySelector('.lb-sub').textContent = sub; }
}

export function hideLoadbar() {
  clearTimeout(loadbarTimer);
  loadbarTimer = null;
  clearProgressStop();
  loadbarEl.style.display = 'none';
}

// ---- unified progress: one bar + per-stage checklist + log tail (issue #21)
// Extract / export / limit-sweep all report into the server's single _prog
// object (GET /api/progress); renderProgress paints it into #loadbar and
// pollProgress drives one poll loop, resolving with the job's result.
const PROG_STAGE_KEY = {
  'connect SolidWorks': 'prog.s.connect',
  'extract assembly': 'prog.s.extract',
  'export meshes': 'prog.s.meshes',
  'build package': 'prog.s.build',
  'load model': 'prog.s.load',
  'sweep joints': 'prog.s.sweep',
  'collision': 'prog.s.collision',
  'convert meshes': 'prog.s.convert',
  'package + zip': 'prog.s.zip',
};
function stageLabel(name) {
  const k = PROG_STAGE_KEY[name];
  return k ? t(k) : name;
}
export function renderProgress(st) {
  loadbarEl.style.display = 'block';
  const q = s => loadbarEl.querySelector(s);
  // once stop is pressed, show a clear "cancelling" state until the job ends
  // (a step already running -- e.g. a CoACD part -- has to finish first)
  const cancelling = progressCancelling && st.running;
  q('.lb-text').textContent = cancelling ? t('prog.cancelling')
    : (st.label ? stageLabel(st.label) : t('prog.working'));
  if (cancelling || st.frac == null) {
    loadbarEl.classList.add('indet');
    q('.lb-fill').style.width = '30%';
  } else {
    loadbarEl.classList.remove('indet');
    q('.lb-fill').style.width = `${Math.round(st.frac * 100)}%`;
  }
  q('.lb-sub').textContent = st.sub || '';
  const stagesEl = q('.lb-stages');
  if (st.stages && st.stages.length) {
    stagesEl.style.display = 'block';
    stagesEl.replaceChildren();
    for (const s of st.stages) {
      const row = document.createElement('div');
      row.className = 'lb-stage ' + s.state;
      const ico = s.state === 'done' ? '✓' : s.state === 'active' ? '▸' : '○';
      const sp = document.createElement('span');
      sp.className = 'lb-ico'; sp.textContent = ico;
      row.append(sp, document.createTextNode(stageLabel(s.name)));
      stagesEl.append(row);
    }
  } else {
    stagesEl.style.display = 'none';
  }
  const logEl = q('.lb-log');
  const lines = st.log || [];
  if (lines.length) {
    logEl.style.display = 'block';
    logEl.replaceChildren();
    for (const ln of lines.slice(-8)) {
      const d = document.createElement('div');
      d.textContent = ln;
      logEl.append(d);
    }
  } else {
    logEl.style.display = 'none';
  }
}
let progressPoll = null;
// Poll /api/progress until the job ends.  onTick(st) runs each poll (e.g. to
// stream log lines into the console).  Resolves {result, cancelled}; rejects on
// a job error.  intervalMs defaults to 400.
export function pollProgress({ onTick, intervalMs = 400 } = {}) {
  return new Promise((resolve, reject) => {
    if (progressPoll) { clearInterval(progressPoll); }
    progressPoll = setInterval(async () => {
      let st;
      try { st = await (await fetch('/api/progress')).json(); }
      catch { return; }                         // transient: keep polling
      renderProgress(st);
      if (onTick) { try { onTick(st); } catch (_e) { /* non-fatal */ } }
      if (st.done) {
        clearInterval(progressPoll); progressPoll = null;
        if (st.error) { reject(new Error(st.error)); }
        else { resolve({ result: st.result, cancelled: !!st.cancelled }); }
      }
    }, intervalMs);
  });
}
// Shared cancel control on the unified panel: a cancellable job registers a stop
// callback (extract, collision preview); the #lbstop button runs it.  showLoadbar
// / hideLoadbar clear it, so it only shows while a cancellable job owns the panel.
const lbStop = document.getElementById('lbstop');
let progressStopFn = null;
let progressCancelling = false;      // stop pressed, job winding down
export function setProgressStop(fn) {
  progressStopFn = fn;
  progressCancelling = false;
  if (lbStop) { lbStop.disabled = false; lbStop.style.display = 'inline-block'; }
}
function clearProgressStop() {
  progressStopFn = null;
  progressCancelling = false;
  if (lbStop) { lbStop.style.display = 'none'; }
}
lbStop?.addEventListener('click', () => {
  if (!progressStopFn) { return; }
  lbStop.disabled = true;                    // one press; the job ends on its own
  // some steps can't stop instantly (CoACD is atomic per part), so show a clear
  // "cancelling" state right away rather than leaving the panel looking frozen
  progressCancelling = true;
  progressStopFn();
});

// ---- mesh loading with per-file progress -------------------------------
export const meshStats = { want: 0, done: 0, fail: 0 };
function meshTick(name, ok, detail) {
  if (ok) { meshStats.done += 1; } else { meshStats.fail += 1; }
  const i = meshStats.done + meshStats.fail;
  log(t('mesh.tick', { i, n: meshStats.want, mark: ok ? '✓' : '✗', name,
                       detail: detail ? ' — ' + detail : '' }),
      ok ? '' : 'err');
  statusEl.textContent = t('mesh.status', { i, n: meshStats.want,
    fail: meshStats.fail ? t('mesh.statusFail', { n: meshStats.fail }) : '' });
  updateLoadbar(i / (meshStats.want || 1),
                t('mesh.barText', { i, n: meshStats.want, name }),
                t('mesh.barSub'));
}

// NOTE: the element property is loadMeshFunc (NOT the loader's loadMeshCb)
// meshCache: a rebuild after a joint edit only changes the URDF, never the
// meshes -- serve repeat loads from memory (clone) so the reload is instant
const meshCache = new Map();
viewer.loadMeshFunc = (path, manager, done) => {
  const clean = path.split('?')[0];
  const name = clean.split('/').pop();
  const lower = clean.toLowerCase();
  // NEVER hand urdf-loaders a null object on failure: it calls .traverse() on
  // whatever we pass and a single null halts the ENTIRE model load.  A bad mesh
  // becomes an empty Group so the rest of the robot keeps loading.  Wrap done()
  // too, so a throw deeper in the loader can't take the whole load down either.
  const safeDone = obj => { try { done(obj); } catch (e) { /* keep loading */ } };
  if (meshCache.has(path)) {
    meshStats.want += 1; meshStats.done += 1;
    safeDone(meshCache.get(path).clone(true));
    return;
  }
  meshStats.want += 1;
  const ok = obj => {
    meshCache.set(path, obj);
    meshTick(name, true);
    safeDone(obj.clone(true));
  };
  const fail = err => {
    meshTick(name, false, err?.message ?? String(err));
    safeDone(new THREE.Group());      // empty placeholder, not null
  };
  if (lower.endsWith('.3dxml')) {
    const url = packageState.dropMode ? path : path + '?glb=1';
    new GLTFLoader(manager).load(url, g => ok(g.scene), null, fail);
  } else if (lower.endsWith('.glb') || lower.endsWith('.gltf')) {
    new GLTFLoader(manager).load(path, g => ok(g.scene), null, fail);
  } else if (lower.endsWith('.stl')) {
    new STLLoader(manager).load(path, geom => ok(new THREE.Mesh(geom,
      new THREE.MeshPhongMaterial({ color: 0x999999 }))), null, fail);
  } else if (lower.endsWith('.dae')) {
    new ColladaLoader(manager).load(path, dae => ok(dae.scene), null, fail);
  } else if (lower.endsWith('.obj')) {
    // urdf-loader applies the URDF <material> colour only when the loaded mesh
    // is a THREE.Mesh (URDFLoader: `obj instanceof THREE.Mesh`).  OBJLoader
    // returns a Group, so its sub-meshes would never receive the <material>
    // colour (e.g. PR2 <material name="White"/>, Fetch name="Grey") and the
    // robot renders flat gray.  Merge the OBJ into ONE THREE.Mesh -- now it is
    // coloured exactly like .stl / primitive visuals.  We don't read the .mtl;
    // the URDF material is the authoritative colour here (matches RViz).
    new OBJLoader(manager).load(path, grp => {
      grp.updateMatrixWorld(true);
      const geoms = [];
      grp.traverse(o => {
        if (!o.isMesh || !o.geometry) return;
        const g = o.geometry.clone();
        g.applyMatrix4(o.matrixWorld);          // bake any sub-mesh transform
        // mergeGeometries needs a uniform attribute set; we colour flat, so
        // keep position(+normal) only and drop uv/colour/etc.
        for (const k of Object.keys(g.attributes)) {
          if (k !== 'position' && k !== 'normal') g.deleteAttribute(k);
        }
        if (!g.attributes.normal) g.computeVertexNormals();
        geoms.push(g);
      });
      if (!geoms.length) { fail(new Error(t('mesh.unsupported'))); return; }
      const merged = geoms.length === 1 ? geoms[0]
                                        : mergeGeometries(geoms, false);
      if (!merged) { fail(new Error('OBJ merge failed')); return; }
      ok(new THREE.Mesh(merged,
        new THREE.MeshPhongMaterial({ color: 0x999999 })));
    }, null, fail);
  } else {
    fail(new Error(t('mesh.unsupported')));
  }
};

