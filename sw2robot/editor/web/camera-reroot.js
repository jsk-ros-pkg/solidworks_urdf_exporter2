import { axisMarkers } from './axis-markers.js';
import { viewer } from './dom.js';
import { rows } from './joint-rows.js';
import { loadRobot } from './load.js';
import { refreshHistory, rootDelta } from './root-frame.js';
import { op } from './session-log.js';
import { packageState } from './state.js';
import { THREE } from './three-setup.js';
// ---- camera: bbox helpers, SolidWorks-style view presets, auto-fit ------
export function robotBox() {
  // bbox of the actual MODEL: visible meshes only, helpers excluded
  // (axis markers / triads poke far outside the geometry and were
  // inflating every camera fit and the grid pitch)
  const robot = viewer.robot;
  if (!robot) { return null; }
  robot.updateMatrixWorld(true);
  const box = new THREE.Box3();
  const b = new THREE.Box3();
  robot.traverse(c => {
    if (c.isMesh && !c.userData.sw2robotMarker && c.visible && c.geometry) {
      if (!c.geometry.boundingBox) { c.geometry.computeBoundingBox(); }
      b.copy(c.geometry.boundingBox).applyMatrix4(c.matrixWorld);
      box.union(b);
    }
  });
  return box.isEmpty() ? null : box;
}

// up="Z" rotates the world so URDF (x,y,z) renders as three (x, z, -y)
const U2T = v => new THREE.Vector3(v[0], v[2], -v[1]);
const VIEWS = {
  front:  [1, 0, 0], back: [-1, 0, 0],
  right:  [0, 1, 0], left: [0, -1, 0],
  top:    [0, 0, 1], bottom: [0, 0, -1],
  // isometric from the +X +Y +Z octant (camera at "(1,1,1)"): a robot-friendly
  // 3/4 view.  Any (±1,±1,±1) diagonal is isometric; this picks the all-positive
  // one so +Y (robot left) faces the viewer instead of away.
  iso:    [1, 1, 1],
};

// ⌖ origin: the WORLD origin is fixed (grid lives there); this moves the
// whole CAD so the current base_link frame coincides with it
function seatModel() {
  if (!viewer.robot) { return; }
  const pose = rootDelta.clone().invert();
  pose.decompose(viewer.robot.position, viewer.robot.quaternion,
                 viewer.robot.scale);
  viewer.robot.scale.set(1, 1, 1);
  viewer.robot.updateMatrixWorld(true);
  viewer.controls.target.set(0, 0, 0);   // look at the (fixed) origin
  viewer.controls.update();
  viewer.redraw();
  log(t('origin.seated'), 'ok');
}
document.getElementById('vorigin').addEventListener('click', seatModel);

export function setView(name) {
  const box = robotBox();
  if (!box) { return; }
  const center = box.getCenter(new THREE.Vector3());
  const diag = box.getSize(new THREE.Vector3()).length();
  const dir = U2T(VIEWS[name]).normalize();
  // distance that actually FILLS the screen: fit the extent along the
  // camera's UP axis to the vertical FOV and the extent along its RIGHT
  // axis to the horizontal FOV, plus the depth toward the camera
  const upVec = name === 'top' ? new THREE.Vector3(0, 0, -1)
    : name === 'bottom' ? new THREE.Vector3(0, 0, 1)
    : new THREE.Vector3(0, 1, 0);
  const right = new THREE.Vector3().crossVectors(dir, upVec).normalize();
  const upOrtho = new THREE.Vector3().crossVectors(right, dir).normalize();
  let rUp = 0, rRight = 0, rPar = 0;
  const corner = new THREE.Vector3();
  for (const xi of ['min', 'max']) {
    for (const yi of ['min', 'max']) {
      for (const zi of ['min', 'max']) {
        corner.set(box[xi].x, box[yi].y, box[zi].z).sub(center);
        rPar = Math.max(rPar, corner.dot(dir));
        rUp = Math.max(rUp, Math.abs(corner.dot(upOrtho)));
        rRight = Math.max(rRight, Math.abs(corner.dot(right)));
      }
    }
  }
  const tanV = Math.tan((viewer.camera.fov * Math.PI / 180) / 2);
  const tanH = tanV * viewer.camera.aspect;
  const dist = Math.max(
    Math.max(rUp / tanV, rRight / tanH) * 1.08 + rPar, 0.1);
  viewer.camera.position.copy(center.clone().add(dir.multiplyScalar(dist)));
  // looking straight down/up: keep "front" toward the screen bottom
  viewer.camera.up.set(0, 1, 0);
  if (name === 'top') { viewer.camera.up.set(0, 0, -1); }
  if (name === 'bottom') { viewer.camera.up.set(0, 0, 1); }
  // small near floor so dollying right up to a small inner part doesn't clip it
  viewer.camera.near = Math.max(diag / 1000, 1e-4);
  viewer.camera.far = Math.max(diag * 100, 50);
  viewer.camera.updateProjectionMatrix();
  viewer.controls.target.copy(center);
  viewer.controls.update();
  viewer.redraw();
}
document.querySelectorAll('#views button[data-v]').forEach(b =>
  b.addEventListener('click', () => setView(b.dataset.v)));
// ---- re-root: shared by the tree's ⌂ buttons and the 3D hover R key -----
export async function reRoot(link) {
  op('reRoot', { link });
  if (!packageState.currentInfo) {
    log(t('reroot.needPkg'), 'wrn');
    return;
  }
  log(t('reroot.start', { link }));
  statusEl.textContent = t('reroot.status');
  try {
    const resp = await fetch('/api/set_base', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ link }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('reroot.ok', { base: r.base, flipped: r.flipped }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true, followCamera: true });
    refreshHistory();
  } catch (e) {
    log(t('reroot.fail', { e: e.message ?? e }), 'err');
  }
}

async function redetectMimic() {
  if (!packageState.currentInfo) { log(t('reroot.needPkg'), 'wrn'); return; }
  log(t('redetect.start'));
  try {
    const resp = await fetch('/api/redetect_couplings', { method: 'POST' });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (!r.applied || !r.applied.length) { log(t('redetect.none'), 'wrn'); return; }
    log(t('redetect.ok', { n: r.applied.length }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true, followCamera: true });
    refreshHistory();
  } catch (e) {
    log(t('redetect.fail', { e: e.message ?? e }), 'err');
  }
}
document.getElementById('redetectmimic')
  ?.addEventListener('click', redetectMimic);

// one-liner that downloads the ROS 2 package zip, colcon-builds it and brings
// up display.launch.py on a ROS 2 machine (served by /api/launch_it.sh)
(function () {
  const el = document.getElementById('launchitcmd');
  if (!el) { return; }
  // Works in any shell (bash / zsh / fish) AND across WSL networking modes,
  // without the user caring: the outer shell only sees `bash -c '...' | bash`
  // (no process substitution), and the host detection runs inside that bash.
  // It tries the browser host, then localhost (mirrored WSL / same machine),
  // then the WSL2 NAT gateway; the first that responds wins, and launch_it.sh
  // derives its zip URL from that same host so the whole flow stays consistent.
  const port = location.port || '8090';
  const lh = 'localhost:' + port;
  const heads = (location.host === lh) ? [lh] : [location.host, lh];
  const hostList = heads.join(' ')
    + ` $(ip route show default 2>/dev/null | cut -d" " -f3):${port}`;
  const cmd = `bash -c 'for H in ${hostList}; do `
    + `curl -fs http://$H/api/launch_it.sh && break; done' | bash`;
  el.textContent = cmd;
  el.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(cmd);
      log(t('launchit.copied'), 'ok');
    } catch (e) { /* clipboard blocked -- user can still select the text */ }
  });
})();

// ---- 3D hover: name the link under the cursor, R = make it root,
// double-click = jump to its row in the tree -------------------------------
const raycaster = new THREE.Raycaster();

export function pickHit(ev) {
  if (!viewer.robot) { return null; }
  const rect = viewer.getBoundingClientRect();
  raycaster.setFromCamera({
    x: ((ev.clientX - rect.left) / rect.width) * 2 - 1,
    y: -((ev.clientY - rect.top) / rect.height) * 2 + 1 }, viewer.camera);
  const hit = raycaster.intersectObject(viewer.robot, true)
    .find(h => h.object.isMesh && !h.object.userData.sw2robotMarker);
  if (!hit) { return null; }
  let n = hit.object;
  while (n && !n.isURDFLink) { n = n.parent; }
  return { link: n?.name || null, hit };
}

export function pickLink(ev) {
  return pickHit(ev)?.link ?? null;
}

// ---- axis-line picking: the joint axis markers draw on top (depthTest off)
// but are inert to the mesh raycast, so an axis can be visible yet unclickable
// when a mesh sits in front.  Instead we pick in SCREEN space: if the cursor is
// within a few px of a visible axis line, select that joint -- even occluded.
const AXIS_PICK_PX = 12;          // how close (px) the cursor must get to a line
export function _projToScreen(v, rect) {
  const p = v.clone().project(viewer.camera);     // NDC; z>1 or <-1 = clipped
  if (p.z < -1 || p.z > 1) { return null; }       // behind camera / off depth
  return { x: rect.left + (p.x * 0.5 + 0.5) * rect.width,
           y: rect.top + (-p.y * 0.5 + 0.5) * rect.height };
}
function _segDistPx(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
  let t = l2 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}
const _axA = new THREE.Vector3(), _axB = new THREE.Vector3();
// nearest VISIBLE joint axis within AXIS_PICK_PX of the cursor, or null
export function pickAxis(ev) {
  if (!viewer.robot || !viewer.camera) { return null; }
  const rect = viewer.getBoundingClientRect();
  let best = null;
  for (const [joint, m] of axisMarkers) {
    if (!m.mesh.visible) { continue; }
    const h = (m.mesh.geometry.parameters?.height ?? 0) / 2;
    if (!h) { continue; }
    m.mesh.updateWorldMatrix(true, false);
    const sa = _projToScreen(_axA.set(0,  h, 0).applyMatrix4(m.mesh.matrixWorld),
                             rect);
    const sb = _projToScreen(_axB.set(0, -h, 0).applyMatrix4(m.mesh.matrixWorld),
                             rect);
    if (!sa || !sb) { continue; }
    const d = _segDistPx(ev.clientX, ev.clientY, sa.x, sa.y, sb.x, sb.y);
    if (d <= AXIS_PICK_PX && (!best || d < best.dist)) {
      const child = rows.get(joint)?.child ?? null;
      if (child) { best = { joint, child, dist: d }; }
    }
  }
  return best;
}

