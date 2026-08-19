import {
  hideLoadbar, pollProgress, setProgressStop, showLoadbar,
} from './capture-progress.js';
import { loadRobot } from './load.js';
// ---- export box ----------------------------------------------------------
export async function openServerPath(p) {
  await ensureNoExtraction();           // interrupt a running extraction first
  log(`/api/open ${p}`);
  const resp = await fetch('/api/open?path=' + encodeURIComponent(p));
  const info = await resp.json();
  if (!resp.ok || info.error) {
    log(t('open.fail', { e: info.error ?? resp.status }), 'err');
    return;
  }
  loadRobot(info);
}

// elapsed seconds -> readable "45秒" / "1分23秒" (the user asked for seconds,
// not "0.2 min")
function fmtSecs(s) {
  s = Math.round(s);
  return s < 60 ? t('time.sec', { s })
                : t('time.minSec', { m: Math.floor(s / 60),
                                     s: String(s % 60).padStart(2, '0') });
}

// extraction runs SolidWorks in a server-side thread that checks a cancel flag
// at every progress checkpoint (per phase / per mesh), so it can be interrupted
// part-way through a heavy file.
export let extracting = false;
export function cancelExtraction() {
  log(t('extract.cancelling'), 'wrn');
  return fetch('/api/extract/cancel').catch(() => {});
}
// stop any running extraction before starting/loading something new, and wait
// for it to actually settle (cooperative cancel lands within ~one mesh)
async function ensureNoExtraction() {
  if (!extracting) { return; }
  await cancelExtraction();
  for (let i = 0; i < 120 && extracting; i++) {
    await new Promise(r => setTimeout(r, 500));
  }
}

// Which assembly configuration to extract.  Resolves to a config name, '' for
// "the file's saved-active one", or null when the user backed out.  The picker
// only appears when the file genuinely offers a choice (>1 configuration and a
// SolidWorks session that could list them) -- a single-config assembly, or a
// machine with no SolidWorks running, goes straight through as before.
async function pickConfiguration(p) {
  let info;
  try {
    const r = await fetch('/api/configurations?path=' + encodeURIComponent(p));
    info = await r.json();
  } catch { return ''; }
  const names = (info && info.configurations) || [];
  if (names.length < 2) { return ''; }
  const modal = document.getElementById('cfgmodal');
  const sel = document.getElementById('cfgsel');
  document.getElementById('cfgnote').textContent =
    t('cfg.note', { name: p.split(/[\\/]/).pop(), n: names.length });
  sel.innerHTML = '';
  const dflt = document.createElement('option');
  dflt.value = '';
  dflt.textContent = t('cfg.saved');
  sel.appendChild(dflt);
  for (const n of names) {
    const o = document.createElement('option');
    o.value = n; o.textContent = n;       // a config name is a CAD identifier
    sel.appendChild(o);
  }
  modal.style.display = 'flex';
  return await new Promise(resolve => {
    const done = v => {
      modal.style.display = 'none';
      document.getElementById('cfggo').onclick = null;
      document.getElementById('cfgcancel').onclick = null;
      resolve(v);
    };
    document.getElementById('cfggo').onclick = () => done(sel.value);
    document.getElementById('cfgcancel').onclick = () => done(null);
  });
}

// SolidWorks extraction: start the job, stream its progress into the log
export async function extractFlow(p, configuration) {
  await ensureNoExtraction();           // a heavy job in flight? cancel it first
  if (configuration === undefined) {
    configuration = await pickConfiguration(p);
    if (configuration === null) { return; }      // user cancelled
  }
  log(t('extract.request', { p }), 'wrn');
  if (configuration) { log(t('cfg.chose', { c: configuration })); }
  statusEl.textContent = t('extract.statusRunning');
  const resp = await fetch('/api/extract?path=' + encodeURIComponent(p) +
    (configuration ? '&configuration=' + encodeURIComponent(configuration) : ''));
  const r = await resp.json();
  if (!resp.ok || r.error) {
    log(t('extract.startFail', { e: r.error ?? resp.status }), 'err');
    return;
  }
  log(t('extract.started'));
  extracting = true;
  showLoadbar(t('extract.bar'), { indet: true });
  setProgressStop(() => cancelExtraction());   // ■ stop on the unified panel
  const t0 = performance.now();
  let seen = 0;
  const backdrop = document.getElementById('loadbackdrop');
  try {
    // the server now owns the stage/frac mapping (see _prog_extract_stage) --
    // the client just streams new log lines; renderProgress paints the rest
    const { result, cancelled } = await pollProgress({
      intervalMs: 1000,
      onTick: st => {
        const lines = st.log || [];
        for (; seen < lines.length; seen++) { log(`  ${lines[seen]}`); }
        statusEl.textContent = t('extract.statusElapsed',
          { time: fmtSecs((performance.now() - t0) / 1000) });
      },
    });
    extracting = false;
    if (cancelled) {
      hideLoadbar();
      backdrop.style.display = 'none';
      statusEl.textContent = t('extract.cancelled');
      log(t('extract.cancelled'), 'wrn');
      return;
    }
    log(t('extract.done', { time: fmtSecs((performance.now() - t0) / 1000) }), 'ok');
    if (result && result.package) {
      openServerPath(result.package);   // its own loadbar takes over
    } else {
      hideLoadbar();
      backdrop.style.display = 'none';
    }
  } catch (e) {
    extracting = false;
    hideLoadbar();
    backdrop.style.display = 'none';
    log(t('extract.failed', { e: e.message ?? e }), 'err');
  }
}

export function openAny(p) {
  // a .SLDASM assembly OR a single .SLDPRT part goes through SolidWorks extract
  // (a lone part becomes a 1-link URDF); a package/.urdf just loads
  if (/\.(sldasm|sldprt)$/i.test(p)) { extractFlow(p); }
  else { openServerPath(p); }
}
