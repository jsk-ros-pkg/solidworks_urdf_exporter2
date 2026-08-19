import {
  addAxisMarkers, applyPoseDrag, axisGlyphs, axisMarkers,
} from './axis-markers.js';
import { buildGrid, fitView } from './batch-look.js';
import { clearBoxSelection } from './box-select.js';
import { setView } from './camera-reroot.js';
import {
  hideLoadbar, meshStats, refreshCompMeta, showLoadbar,
} from './capture-progress.js';
import {
  alignBtn, jointsEl, mimicBar, portBtn, tfBtn, viewer,
} from './dom.js';
import { cancelEndcoords, ecTool } from './endcoords-gizmo.js';
import { cancelExtraction, extracting } from './export-box.js';
import { refreshExportName } from './export-names.js';
import { clearFaceOverlay } from './face-pick.js';
import {
  addOriginTriad, addPortMarkers, clearPortMarkers, savedMats, setTF, tfOn,
  undimAll,
} from './frames.js';
import { rows } from './joint-rows.js';
import { selectLink } from './link-info.js';
import {
  applyLinkVisibility, applyPersistedColors, hiddenLinks, initCollision,
} from './link-look.js';
import {
  syncClearMeshButton, syncSubassemblyAccessButton, syncSw2urdfButton,
} from './lists.js';
import { cancelMimic } from './mimic.js';
import { buildPlayRows, withJointOpSuppressed } from './play-mode.js';
import { refreshRootBox } from './root-frame.js';
import { mimicFollowers } from './selection.js';
import { _cancelTweens, _resetSessionLogs, op } from './session-log.js';
import {
  collisionState, loadState, mimicState, packageState, selectionState,
  treeState, viewState,
} from './state.js';
import { THREE } from './three-setup.js';
import { buildJointRows } from './tree.js';
viewer.addEventListener('urdf-processed', () => {
  if (treeState.pendingRobotViewMode) {
    treeState.robotViewMode = treeState.pendingRobotViewMode;
    treeState.pendingRobotViewMode = null;
  }
  const n = buildJointRows(viewer.robot);
  if (treeState.playMode) { buildPlayRows(viewer.robot); }   // keep the move panel in sync
  log(t('urdf.parsed', { n: Object.keys(viewer.robot.links).length, m: n }), 'ok');
});

function captureJointAngles(robot) {
  const angles = {};
  for (const [n, j] of Object.entries(robot?.joints ?? {})) {
    if (j.jointType !== 'fixed' && !j.mimicJoint) {
      angles[n] = Number(j.angle);
    }
  }
  return angles;
}

function removeLoadCover() {
  if (loadState.loadCover) {
    loadState.loadCover.removeFromParent();
    loadState.loadCover = null;
    viewer.redraw();
  }
}

viewer.addEventListener('geometry-loaded', () => {
  hideLoadbar();
  document.getElementById('loadbackdrop').style.display = 'none';  // new model in
  log(t('mesh.allLoaded'), 'ok');
  _cancelTweens();               // a replay ease must not keep driving the NEW robot
  const restore = loadState.pendingRestore;
  loadState.pendingRestore = null;
  requestAnimationFrame(() => {
    addAxisMarkers(viewer.robot);
    addOriginTriad(viewer.robot);
    addPortMarkers(viewer.robot);  // dummy_link ports (mesh-less, else unseen)
    applyPoseDrag();               // keep orbit/pose mode across reloads
    buildGrid();
    hiddenLinks.forEach(n => applyLinkVisibility(n));
    viewState.tfNodes = [];                  // old robot's TF nodes died with it
    undimAll();                    // restore shared (cached) materials!
    applyPersistedColors();        // re-paint server-saved colour overrides
    collisionState.collisionLinks = new Set();    // fresh meshes wear no stale red
    initCollision();
    if (tfOn()) { setTF(true); }
    if (restore) {
      // joint edit reload: keep the camera, put the pose back
      let n = 0;
      withJointOpSuppressed(() => {
        for (const [name, v] of Object.entries(restore.angles)) {
          const j = viewer.robot.joints[name];
          if (j && j.jointType !== 'fixed' && !j.mimicJoint) {
            viewer.setJointValue(name, v);
            n += 1;
          }
        }
      });
      if (restore.linkWorlds) {
        // the model jumped in scene space (re-root / root frame change):
        // move the CAMERA by the same rigid transform so the view looks
        // unchanged.  Any link present before and after gives it to us.
        viewer.robot.updateMatrixWorld(true);
        for (const [name, oldW] of restore.linkWorlds) {
          // 'base_link' names a DIFFERENT physical link after a re-root
          if (name === 'base_link') { continue; }
          const l = viewer.robot.links[name];
          if (!l) { continue; }
          const S = l.matrixWorld.clone()
            .multiply(oldW.clone().invert());
          viewer.camera.position.applyMatrix4(S);
          viewer.controls.target.applyMatrix4(S);
          const R = new THREE.Matrix4().extractRotation(S);
          viewer.camera.up.transformDirection(R);
          viewer.camera.updateProjectionMatrix();
          viewer.controls.update();
          log(t('reroot.camFollow'), 'ok');
          break;
        }
      }
      buildJointRows(viewer.robot);
      removeLoadCover();           // new robot is in place: drop the ghost
      viewer.redraw();
      log(t('pose.restored', { n }), 'ok');
    } else {
      removeLoadCover();
      fitView();
    }
    if (selectionState.reselectAfterLoad) {
      const rl = selectionState.reselectAfterLoad; selectionState.reselectAfterLoad = null;
      if (viewer.robot.links[rl]) { selectLink(rl); }   // keep the panel open
    }
  });
  statusEl.textContent = t('status.summary', {
    links: Object.keys(viewer.robot.links).length,
    joints: buildJointRows(viewer.robot),
    done: meshStats.done, want: meshStats.want,
    tail: meshStats.fail ? t('status.summaryFail', { n: meshStats.fail })
                         : t('status.summaryOk') });
});
viewer.addEventListener('urdf-error', e => {
  treeState.pendingRobotViewMode = null;
  removeLoadCover();
  hideLoadbar();
  document.getElementById('loadbackdrop').style.display = 'none';
  log(t('urdf.error', { detail: e.detail ?? t('urdf.seeConsole') }), 'err');
});

export function resetPose() {
  let n = 0;
  withJointOpSuppressed(() => {
    for (const j of Object.values(viewer.robot?.joints ?? {})) {
      if (j.jointType !== 'fixed' && !j.mimicJoint) {
        viewer.setJointValue(j.name, 0);
        n += 1;
      }
    }
  });
  if (n) { op('resetPose', { n }); }   // one action, not n joint edits
}
document.getElementById('reset').addEventListener('click', resetPose);

// reset the VIEWER to the just-loaded state -- neutral pose, default camera,
// nothing selected, every transient tool/overlay closed.  NOT a data reload:
// joint TYPE/limit edits persist (they live in the URDF); only the live view
// state is cleared.
export function resetView() {
  if (mimicState.mimicMode) { cancelMimic(); }
  if (ecTool) { cancelEndcoords(); }
  resetPose();
  selectLink(null);
  clearBoxSelection();
  alignBtn.classList.remove('active');
  portBtn.classList.remove('active');
  clearFaceOverlay();
  viewState.orbitBaseLink = null;          // orbit base back to the robot centre
  setView('iso');
  log(t('view.reset'), 'ok');
}
document.getElementById('resetview').addEventListener('click', resetView);

// Empty the viewer entirely (unlike resetView, which only returns the loaded
// robot to its initial pose/camera).  The server package stays open, so 🗄 /
// the recent list re-open it.  Markers (axis meshes, TF triads) are parented to
// the robot's joints/links, so removing the robot drops them too; we only have
// to undim the SHARED materials TF may have dimmed and clear the JS-side state.
function clearViewer() {
  if (extracting) { cancelExtraction(); return; }  // interrupt a running extract
  if (!viewer.robot) { return; }                 // already empty
  op('clearViewer', {});
  if (mimicState.mimicMode) { cancelMimic(); }
  if (ecTool) { cancelEndcoords(); }
  selectLink(null);
  clearBoxSelection();
  clearFaceOverlay();
  clearPortMarkers();
  alignBtn.classList.remove('active');
  portBtn.classList.remove('active');
  undimAll();                                    // restore TF-dimmed materials
  tfBtn.classList.remove('active');
  viewState.tfNodes = [];                                  // gone with the robot tree
  axisMarkers.clear();                           // gone with the robot tree
  axisGlyphs.clear();                            // gone with the robot tree
  clearTimeout(collisionState.colPoll); collisionState.colPoll = 0;            // stop self-collision polling
  collisionState.colReady = false; collisionState.colBusy = false; collisionState.colQueued = false;
  removeLoadCover();
  const r = viewer.robot;
  if (r && r.parent) { r.parent.remove(r); }
  viewer.robot = null;
  treeState.robotViewMode = 'normal';
  treeState.pendingRobotViewMode = null;
  jointsEl.innerHTML = '';
  rows.clear();
  document.getElementById('selbar').style.display = 'none';
  document.getElementById('linkinfo').style.display = 'none';
  hideLoadbar();
  document.getElementById('emptyprompt').style.display = '';
  packageState.currentInfo = null;
  syncSubassemblyAccessButton();
  syncSw2urdfButton();
  document.getElementById('title').textContent = t('ui.title');
  document.title = 'sw2robot';
  statusEl.textContent = t('view.cleared');
  viewer.redraw();
  _resetSessionLogs();           // emptied the viewer -> start a fresh recording
  log(t('view.cleared'), 'ok');
}
document.getElementById('clearview').addEventListener('click', clearViewer);

// ---- loading entry points -----------------------------------------------
export function loadRobot(info, { drop = false, keepPose = false,
                           followCamera = false, previewOnly = false,
                           restoreAngles = null } = {}) {
  // a keepPose reload is the SAME session (edit rebuild); anything else is a
  // fresh package -> drop the old actions/camera so they can't replay onto it
  if (!keepPose) { _resetSessionLogs(); }
  op('loadRobot', { name: info?.name, keepPose, drop, previewOnly });
  treeState.pendingRobotViewMode = previewOnly ? 'collapsed_preview' : 'normal';
  cancelEndcoords();        // a rebuild invalidates any open placement session
  document.getElementById('emptyprompt').style.display = 'none';
  if (((keepPose || followCamera) && viewer.robot) || restoreAngles) {
    const angles = restoreAngles || captureJointAngles(viewer.robot);
    let linkWorlds = null;
    if (followCamera) {
      viewer.robot.updateMatrixWorld(true);
      linkWorlds = new Map();
      for (const [n, l] of Object.entries(viewer.robot.links)) {
        linkWorlds.set(n, l.matrixWorld.clone());
      }
    }
    loadState.pendingRestore = { angles, linkWorlds };
  }
  packageState.dropMode = drop;
  if (drop) {
    packageState.currentInfo = null;
    viewer.urlModifierFunc = null;
  } else {
    viewer.urlModifierFunc = null;
    if (!previewOnly) {
      packageState.currentInfo = info;
    }
  }
  syncSubassemblyAccessButton();
  syncSw2urdfButton();
  syncClearMeshButton();
  refreshExportName();          // export field placeholder follows the open pkg
  // cover the reload gap with a display-only clone of the current robot
  // (same world pose -> the swap is pixel-invisible); geometry/materials
  // are shared so this costs milliseconds
  removeLoadCover();
  if (!drop && viewer.robot?.parent && keepPose) {
    loadState.loadCover = viewer.robot.clone(true);
    loadState.loadCover.traverse(c => { c.raycast = () => {}; });
    viewer.robot.parent.add(loadState.loadCover);
  }
  meshStats.want = meshStats.done = meshStats.fail = 0;
  jointsEl.innerHTML = '';
  rows.clear();
  savedMats.clear();              // old robot's meshes are discarded anyway
  selectionState.selectedLink = null;
  selectionState.jpSync = null;                  // the old #linkinfo panel died with robot
  selectionState.selVis = null;                  // died with the old robot
  selectionState.selAxisJoint = null;            // markers are rebuilt by addAxisMarkers
  if (mimicState.mimicMode) {                // a reload ends any open mimic session
    mimicState.mimicMode = false; mimicState.mimicMaster = mimicState.mimicMasterChild = null;
    mimicFollowers.clear(); mimicBar.style.display = 'none';
  }
  clearFaceOverlay();
  document.getElementById('selbar').style.display = 'none';
  document.getElementById('linkinfo').style.display = 'none';
  document.getElementById('title').textContent = info.name;
  document.title = `${info.name} — sw2robot`;
  if (!drop) {
    refreshRootBox({ resetBaseline: true });
    if (!previewOnly) { refreshCompMeta(); }
  }
  statusEl.textContent = t('status.loadingUrdf');
  log(t('urdf.loading', { urdf: info.urdf }));
  // centre bar only when the load is actually noticeable (300 ms grace:
  // cached edit-reloads finish before it ever appears)
  showLoadbar(t('urdf.loadingBar', { name: info.name }), { indet: true, delay: 300 });
  // cache-bust so a rebuilt URDF is never served stale by the browser
  viewer.urdf = drop ? info.urdf
    : info.urdf + '?v=' + Date.now() + (viewState.mergedView ? '&merged=1' : '');
}

