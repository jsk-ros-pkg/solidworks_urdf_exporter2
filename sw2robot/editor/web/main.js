import { extractFlow } from './export-box.js';
import { loadRobot } from './load.js';
import './state.js';
import './dom.js';
import './three-setup.js';
import './bootstrap.js';
import './session-log.js';
import './capture-progress.js';
import './joint-rows.js';
import './subassembly-choices.js';
import './subassembly-preview.js';
import './lists.js';
import './mass-editor.js';
import './tree.js';
import './play-mode.js';
import './axis-markers.js';
import './frames.js';
import './link-look.js';
import './selection.js';
import './link-info.js';
import './bulk-edit.js';
import './joint-type.js';
import './export-names.js';
import './coacd-preview.js';
import './camera-reroot.js';
import './face-pick.js';
import './endcoords-gizmo.js';
import './box-select.js';
import './mimic.js';
import './batch-look.js';
import './root-frame.js';
import './file-browser.js';
import './diagnostics.js';
import './keycast.js';
// ---- kick off -----------------------------------------------------------
// live SolidWorks session: only attachable when THIS SERVER was started
// from the user's own terminal (same login session); otherwise we show why
let _swStatus = null;          // last /api/swstatus payload, for re-rendering
export function renderSwStatus() {
  const st = _swStatus;
  if (!st) { return; }
  const el = document.getElementById('swstat');
  const btn = document.getElementById('useactive');
  if (st.active_assembly) {
    btn.style.display = '';
    btn.title = st.active_assembly;
    // the path/filename is a real on-disk identifier -- never translated
    el.textContent = (st.dirty ? t('sw.unsaved') : '') +
      t('sw.open', { name: st.active_assembly.split(/[\\/]/).pop() });
  } else if (st.running && !st.attachable) {
    el.textContent = t('sw.runningNotVisible');
  } else if (!st.running) {
    el.textContent = t('sw.notRunning');
  }
}
(async () => {
  try {
    _swStatus = await (await fetch('/api/swstatus')).json();
    renderSwStatus();
    if (_swStatus.active_assembly) {
      document.getElementById('useactive').addEventListener('click', () => {
        if (_swStatus.dirty) { log(t('sw.unsavedLog'), 'wrn'); }
        extractFlow(_swStatus.active_assembly);
      });
    }
  } catch { /* status is best-effort */ }
})();

const info = await (await fetch('/api/info')).json();
if (info.urdf) { loadRobot(info); }
else {
  statusEl.textContent = t('start.pick');
  document.getElementById('emptyprompt').style.display = 'block';
  log(t('start.noPkg'), 'wrn');
}
