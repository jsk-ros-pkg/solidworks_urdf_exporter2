import { axesOn, axisGlyphs } from './axis-markers.js';
import { boxSelected, selectLinksInBox } from './box-select.js';
import { reRoot } from './camera-reroot.js';
import {
  _rec, refreshCompMeta, startScreenRecording, stopScreenRecording,
} from './capture-progress.js';
import { viewer } from './dom.js';
import { extractFlow } from './export-box.js';
import { alignOn } from './face-pick.js';
import { savedMats, tfOn } from './frames.js';
import { fillLinkInfo, selectLink } from './link-info.js';
import { hiddenLinks } from './link-look.js';
import { syncSw2urdfButton } from './lists.js';
import { renderSwStatus } from './main.js';
import { bulkSetType } from './mimic.js';
import { doHistory, refreshHistory } from './root-frame.js';
import {
  actionLogText, camlog, downloadActionLog, fullTimeline, oplog,
  replayCancel, replayLog,
} from './session-log.js';
import {
  collisionState, loadState, packageState, replayState, selectionState,
} from './state.js';
import { buildJointRows } from './tree.js';
// ---- diagnostic dump: full session state, for the 🐞 button and for
// driving/inspecting the page from tests via window.sw2robot ----------------
function sw2robotDump() {
  const v = viewer;
  const rect = v.getBoundingClientRect();
  const eaters = [];
  for (const fx of [0.15, 0.5, 0.85]) {
    for (const fy of [0.15, 0.5, 0.85]) {
      const el = document.elementFromPoint(
        rect.left + rect.width * fx, rect.top + rect.height * fy);
      eaters.push(el ? (el.id || el.tagName) : null);
    }
  }
  let elementHover = 0;
  v.robot?.traverse(c => {
    if (c.__origMaterial !== undefined) { elementHover++; }
  });
  return {
    when: new Date().toISOString(),
    package: packageState.currentInfo?.name ?? null,
    selected: selectionState.selectedLink,
    hidden: [...hiddenLinks],
    controlsEnabled: v.controls?.enabled,
    dragManipulating: v.dragControls?.manipulating?.name ?? null,
    dragHovered: v.dragControls?.hovered?.name ?? null,
    coverPresent: !!loadState.loadCover,
    savedMats: savedMats.size,
    elementHoverMeshes: elementHover,
    links: Object.keys(v.robot?.links ?? {}).length,
    eventEaters: eaters,           // what HTML sits over the viewport
    align: alignOn(), tf: tfOn(), axes: axesOn(),
    logTail: [...document.querySelectorAll('#log div')]
      .slice(-15).map(d => d.textContent),
    oplog: oplog.slice(-80),
  };
}

window.sw2robot = {
  dump: sw2robotDump,
  collision: () => ({ ready: collisionState.colReady, links: [...collisionState.collisionLinks] }),
  extractFlow: (p, c) => extractFlow(p, c),   // for E2E (fetch is mocked there)
  boxSelect: rect => selectLinksInBox(rect ??
    { l: -1e9, t: -1e9, r: 1e9, b: 1e9 }),   // no rect => everything on screen
  boxSelected: () => [...boxSelected],
  bulkType: t => bulkSetType(t),
  select: n => selectLink(n),
  actionLogText,                         // ③ readable log (for tests / tooling)
  exportLog: kind => downloadActionLog(kind),
  oplog: () => oplog.slice(),            // ② a copy of the raw action stream
  timeline: () => fullTimeline(),        // actions + camera keyframes, time-ordered
  replay: (entries, opts) => replayLog(entries ?? fullTimeline(), opts),
  cancelReplay: replayCancel,
  record: () => startScreenRecording(),  // 🎥 screen recording (for tooling/tests)
  stopRecord: stopScreenRecording,
  isRecording: () => !!_rec,
  axisGlyphs: () => [...axisGlyphs.entries()].map(([name, g]) => ({
    name, visible: g.visible, children: g.children.length,
    inScene: !!g.parent })),
  reRoot: l => reRoot(l),
  undo: () => doHistory('undo'),
  redo: () => doHistory('redo'),
  report: async () => {
    const d = sw2robotDump();
    try {
      await fetch('/api/client_log', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(d) });
      log(t('bug.sent'), 'ok');
    } catch (e) {
      log(t('bug.fail', { e: e.message }), 'err');
    }
    return d;
  },
};
document.getElementById('bug').addEventListener('click',
  () => window.sw2robot.report());
document.getElementById('savelog').addEventListener('click',
  e => downloadActionLog(e.shiftKey ? 'json' : 'txt'));
document.getElementById('replay').addEventListener('click', e => {
  if (replayState._replaying) { replayCancel(); return; }          // click again = stop
  if (e.shiftKey) { replayLog(fullTimeline()); return; } // replay live session
  document.getElementById('replayfile').click();        // else load a saved .json
});
document.getElementById('replayfile').addEventListener('change', async ev => {
  const file = ev.target.files?.[0];
  ev.target.value = '';                                 // allow re-picking same file
  if (!file) { return; }
  try {
    const entries = JSON.parse(await file.text());
    replayLog(entries);
  } catch (e) {
    log(t('replay.badFile', { e: e.message ?? e }), 'err');
  }
});
document.getElementById('record').addEventListener('click', async e => {
  if (_rec) { stopScreenRecording(); return; }           // click again = stop
  // Shift = auto-record a replay -- but only when one isn't already running,
  // else replayLog() would return immediately (re-entrancy guard) and we'd
  // stop the capture at once, saving an empty video
  const wantReplay = e.shiftKey && !replayState._replaying && (oplog.length || camlog.length);
  const started = await startScreenRecording();          // asks to share the tab
  if (started && wantReplay) {                            // Shift = record a replay
    await replayLog(fullTimeline());
    stopScreenRecording();
  }
});

// re-render the dynamic (JS-built) UI in the new language when toggled.
// data-i18n static chrome is handled by applyStaticI18n in the loader script.
window.onLangChange = () => {
  refreshHistory();              // undo/redo titles
  renderSwStatus();             // SolidWorks status line
  refreshCompMeta();            // 🗑 excluded chip text
  syncSw2urdfButton();          // its title is built in JS, not data-i18n-title
  // #title shows the package name when one is open (language-neutral); only
  // the empty placeholder needs re-localizing.  #status is transient -- just
  // refresh the no-package case.
  if (!packageState.currentInfo) {
    document.getElementById('title').textContent = t('ui.title');
    if (!viewer.robot) { statusEl.textContent = t('start.pick'); }
  }
  if (viewer.robot) {
    buildJointRows(viewer.robot);
    if (selectionState.selectedLink) { fillLinkInfo(selectionState.selectedLink); }
  }
};
document.querySelectorAll('#langbar button').forEach(b =>
  b.addEventListener('click', () => setLang(b.dataset.lang)));

