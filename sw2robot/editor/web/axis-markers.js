import { markHelper } from './bootstrap.js';
import { originBtn, viewer } from './dom.js';
import { selectionState, viewState } from './state.js';
import { THREE } from './three-setup.js';
// ---- joint axis markers: depth-test OFF so they shine through the meshes;
// children of the joint Object3D, so they follow FK for free ---------------
const AXIS_COLORS = { revolute: 0xff5050, continuous: 0xffa050,
                      prismatic: 0x4090ff, mimic: 0xb38cff };
// rotation-arc accent: same hue family as the axis but brighter, so the joint
// type stays readable from the axis colour while the + turn direction pops
const ARC_COLORS  = { revolute: 0xffae1a, continuous: 0xffe000,
                      prismatic: 0x4090ff, mimic: 0xff5ecb };
export const axisMarkers = new Map();   // joint name -> {mesh, baseColor}
export const axisGlyphs = new Map();    // joint name -> Group (arrow + rotation arc),
                                 // shown for every MOVABLE joint, toggled with
                                 // the rod; tracks its rod's visibility

// bounding box of ONE link's own meshes (stop at nested joints, so a serial
// chain doesn't inflate the box) -- used to size each axis to its own link
export function ownLinkBox(link, out) {
  if (!link) { return out; }
  for (const c of link.children) {
    if (c.isURDFJoint || c.type === 'URDFJoint') { continue; }
    if (c.isMesh && c.geometry) {
      if (!c.geometry.boundingBox) { c.geometry.computeBoundingBox(); }
      const b = c.geometry.boundingBox.clone()
        .applyMatrix4(c.matrixWorld);
      out.union(b);
    }
    ownLinkBox(c, out);
  }
  return out;
}

// remembered axis directions per joint name, surviving rebuilds: lets us
// show a GHOST of "where the axis would be" after a joint is set to fixed
export const axisMemory = new Map();    // joint name -> [x,y,z] (joint-local)

function makeAxisMesh(j, axis, { ghost = false } = {}) {
  // span of the CHILD link's meshes along the axis line, in joint space:
  // the marker should pierce the mesh and poke out on both ends
  const childLink = j.children.find(c => c.isURDFLink
                                         || c.type === 'URDFLink');
  const box = ownLinkBox(childLink, new THREE.Box3());
  let tMin = -0.025, tMax = 0.025, girth = 0.05;
  if (!box.isEmpty()) {
    const inv = j.matrixWorld.clone().invert();
    tMin = Infinity; tMax = -Infinity;
    for (const xi of ['min', 'max']) {
      for (const yi of ['min', 'max']) {
        for (const zi of ['min', 'max']) {
          const corner = new THREE.Vector3(
            box[xi].x, box[yi].y, box[zi].z).applyMatrix4(inv);
          const t = corner.dot(axis);
          tMin = Math.min(tMin, t);
          tMax = Math.max(tMax, t);
        }
      }
    }
    girth = box.getSize(new THREE.Vector3()).length();
  }
  const margin = Math.max((tMax - tMin) * 0.18, 0.008);
  const len = (tMax - tMin) + 2 * margin;
  const mid = (tMax + tMin) / 2;
  const r = Math.min(Math.max(girth * 0.012, 0.0008), 0.004);
  const color = ghost ? 0xdddd66
    : (AXIS_COLORS[j.mimicJoint ? 'mimic' : j.jointType] ?? 0xcccccc);
  const geo = new THREE.CylinderGeometry(r, r, len, 12);
  const mat = new THREE.MeshBasicMaterial({
    color, depthTest: false, transparent: true,
    opacity: ghost ? 0.55 : 0.85 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.renderOrder = 999;            // draw after everything (always visible)
  markHelper(mesh);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
  mesh.position.copy(axis.clone().multiplyScalar(mid));
  j.add(mesh);
  return { mesh, baseColor: color, ghost };
}

// span of the CHILD link's meshes along the axis line + a girth estimate, in
// joint space (shared by the rod and the rich glyph so they size alike).
function axisSpan(j, axis) {
  const childLink = j.children.find(c => c.isURDFLink
                                         || c.type === 'URDFLink');
  const box = ownLinkBox(childLink, new THREE.Box3());
  let tMin = -0.025, tMax = 0.025, girth = 0.05;
  if (!box.isEmpty()) {
    const inv = j.matrixWorld.clone().invert();
    tMin = Infinity; tMax = -Infinity;
    for (const xi of ['min', 'max']) {
      for (const yi of ['min', 'max']) {
        for (const zi of ['min', 'max']) {
          const t = new THREE.Vector3(box[xi].x, box[yi].y, box[zi].z)
            .applyMatrix4(inv).dot(axis);
          tMin = Math.min(tMin, t); tMax = Math.max(tMax, t);
        }
      }
    }
    girth = box.getSize(new THREE.Vector3()).length();
  }
  return { tMin, tMax, girth };
}

// Rich directional glyph for the SELECTED joint: a cone arrowhead on the +axis
// end (= +translation for prismatic, the +rotation axis for rotary) and, for
// rotary joints, a curved arc + arrowhead sweeping the right-hand-rule positive
// turn (so it reads as "joint > 0 spins this way").  A child of the joint
// Object3D, so it follows FK like the rod.  Returns the Group (caller disposes).
function makeAxisGlyph(j, axis) {
  const { tMin, tMax, girth } = axisSpan(j, axis);
  const margin = Math.max((tMax - tMin) * 0.18, 0.008);
  const tipT = tMax + margin;                 // +axis end of the rod
  const span = (tMax - tMin) + 2 * margin;
  const key = j.mimicJoint ? 'mimic' : j.jointType;
  const axisColor = AXIS_COLORS[key] ?? 0xcccccc;   // = the rod's type colour
  const arcColor = ARC_COLORS[key] ?? axisColor;    // brighter same-hue accent
  // lit (Phong) so the cone + tube pick up shading and a glossy highlight and
  // read as solid 3D instead of a flat sticker; a touch of emissive keeps the
  // colour vivid under the bright ambient, depthTest off keeps them on top
  const glyphMat = c => new THREE.MeshPhongMaterial({
    color: c, emissive: c, emissiveIntensity: 0.18,
    specular: 0x555555, shininess: 80, depthTest: false });
  const axisMat = glyphMat(axisColor);
  const arcMat = glyphMat(arcColor);
  // sized to read clearly against the link mesh: a fat arrowhead and arc that
  // are noticeably bolder than the thin overview rod
  const coneLen = Math.min(Math.max(span * 0.18, 0.01), 0.05);
  const coneR = Math.min(Math.max(girth * 0.045, 0.003), 0.014);
  const g = new THREE.Group();

  // arrowhead on the +axis end -- shows which way the rod's + direction points
  const cone = new THREE.Mesh(new THREE.ConeGeometry(coneR, coneLen, 16), axisMat);
  cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);
  cone.position.copy(axis.clone().multiplyScalar(tipT + coneLen / 2));
  g.add(cone);

  // rotary joints: an arc curling in the positive (right-hand-rule) direction
  if (j.jointType === 'revolute' || j.jointType === 'continuous') {
    const radius = Math.min(Math.max(girth * 0.32, 0.018), 0.075);
    // (u, v) is a right-handed basis about `axis`: rotating u by +90deg = v, so
    // sweeping angle 0 -> + goes u -> v = the joint's positive rotation.
    const ref = Math.abs(axis.x) < 0.9 ? new THREE.Vector3(1, 0, 0)
                                       : new THREE.Vector3(0, 0, 1);
    const u = new THREE.Vector3().crossVectors(axis, ref).normalize();
    const v = new THREE.Vector3().crossVectors(axis, u).normalize();
    const a0 = -Math.PI * 0.5, a1 = Math.PI;     // ~270deg sweep, CCW about axis
    const N = 56;
    const pts = [];
    for (let i = 0; i <= N; i++) {
      const a = a0 + (a1 - a0) * i / N;
      pts.push(u.clone().multiplyScalar(Math.cos(a) * radius)
        .add(v.clone().multiplyScalar(Math.sin(a) * radius)));
    }
    const tubeR = Math.min(Math.max(girth * 0.015, 0.0014), 0.006);
    g.add(new THREE.Mesh(new THREE.TubeGeometry(
      new THREE.CatmullRomCurve3(pts), N, tubeR, 10, false), arcMat));
    // arrowhead at the leading (a1) end, pointing along +d/da (the + turn)
    const tip = u.clone().multiplyScalar(Math.cos(a1) * radius)
      .add(v.clone().multiplyScalar(Math.sin(a1) * radius));
    const tan = u.clone().multiplyScalar(-Math.sin(a1))
      .add(v.clone().multiplyScalar(Math.cos(a1))).normalize();
    const head = new THREE.Mesh(
      new THREE.ConeGeometry(coneR * 0.95, coneLen * 0.95, 14), arcMat);
    head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tan);
    head.position.copy(tip);
    g.add(head);
  }
  markHelper(g);
  g.traverse(o => { o.renderOrder = 1000; });   // after the rod, always on top
  j.add(g);
  return g;
}

export function addAxisMarkers(robot) {
  axisMarkers.forEach(m => m.mesh.removeFromParent());
  axisGlyphs.forEach(g => g.removeFromParent());
  axisMarkers.clear();
  axisGlyphs.clear();
  robot.updateMatrixWorld(true);
  let ghosts = 0;
  for (const j of Object.values(robot.joints)) {
    if (j.jointType !== 'fixed' && j.axis) {
      const ax = j.axis.clone().normalize();
      axisMemory.set(j.name, j.axis.toArray());
      axisMarkers.set(j.name, makeAxisMesh(j, ax));
      axisGlyphs.set(j.name, makeAxisGlyph(j, ax));   // arrow + arc, always-on
      continue;
    }
    // fixed: ghost axis from this session's memory, or from an <axis> tag
    // the URDF may still carry
    let a = axisMemory.get(j.name);
    if (!a && j.urdfNode) {
      const el = [...j.urdfNode.children]
        .find(e => e.tagName === 'axis');
      if (el) { a = el.getAttribute('xyz').split(/\s+/).map(Number); }
    }
    if (a) {
      const m = makeAxisMesh(
        j, new THREE.Vector3(...a).normalize(), { ghost: true });
      m.mesh.visible = false;        // only shown while hovering the row
      axisMarkers.set(j.name, m);
      ghosts += 1;
    }
  }
  setAxesVisible(axesOn());
  log(t('axes.drawn', { k: axisMarkers.size - ghosts })
      + (ghosts ? t('axes.ghostSuffix', { g: ghosts }) : ''), 'ok');
}

const axesBtn = document.getElementById('axes');
const dirsBtn = document.getElementById('dirs');
export function axesOn() { return axesBtn.classList.contains('active'); }
export function dirsOn() { return dirsBtn.classList.contains('active'); }
export function originOn() { return originBtn.classList.contains('active'); }

export function setAxesVisible(on) {
  // keep the selected link's joint axis shown even with the global toggle off
  axisMarkers.forEach((m, name) => {
    m.mesh.visible = (on && !m.ghost) || name === selectionState.selAxisJoint;
    // the rich glyph (arrow + arc): on every joint when the 'dir' toggle is on,
    // else only on the hovered / selected joint (cluttered shown everywhere)
    const gl = axisGlyphs.get(name);
    if (gl) { gl.visible = dirsOn() || name === selectionState.selAxisJoint; }
  });
  viewer.redraw();
}
// the 'dir' toggle flips between always-on-everywhere and hover/selection-only
dirsBtn.addEventListener('click', () => {
  dirsBtn.classList.toggle('active');
  axisGlyphs.forEach((gl, name) => {
    gl.visible = dirsOn() || name === selectionState.selAxisJoint;
  });
  viewer.redraw();
});
axesBtn.addEventListener('click', () => {
  setAxesVisible(axesBtn.classList.toggle('active'));
});

// ---- drag mode: orbit (default) vs pose (drag a link to move its joint) ----
// the in-viewer joint panel's sliders/snap buttons drive joints, so the
// element's drag-to-articulate (which steals left-drag from OrbitControls and
// amber-highlights links) is OFF by default -- left-drag just rotates the view.
// The flag itself lives in viewState (see state.js): it is READ at evaluation
// time by the material-restore hover handler, and keeping it in this section
// would make the viewer wiring import it back from here -- a cycle, and a
// binding that is not initialised yet when the cycle is traversed.
const poseBtn = document.getElementById('posedrag');
let _origOnHover, _origOnUnhover;     // the element's hover-highlight callbacks
let _origDragControls = null;
const _noopHover = () => {};
export function applyPoseDrag() {
  const dc = viewer.dragControls;
  if (!dc) { return; }
  dc.enabled = viewState.poseDrag;
  if (dc !== _origDragControls) {
    _origDragControls = dc;
    _origOnHover = dc.onHover; _origOnUnhover = dc.onUnhover;
    // urdf-loader's getPrismaticDelta projects the pointer onto the axis
    // transformed by joint.PARENT.matrixWorld -- it drops the joint's own
    // <origin rpy>, so a prismatic with a rotated origin drags the WRONG way
    // (getRevoluteDelta already uses joint.matrixWorld).  The real motion runs
    // the axis through the joint's own frame, so use joint.matrixWorld here too
    // and every slider drags the way the mouse moves, not just the rpy=0 ones.
    const _pdN = new THREE.Vector3();
    const _pdD = new THREE.Vector3();
    dc.getPrismaticDelta = (joint, startPoint, endPoint) => {
      _pdD.subVectors(endPoint, startPoint);
      _pdN.copy(joint.axis).transformDirection(joint.matrixWorld).normalize();
      return _pdD.dot(_pdN);
    };
    // urdf-loader grabs a joint on mousedown for ANY button (it never checks
    // e.button), so a middle- or right-drag manipulates the joint instead of
    // panning/orbiting the camera.  Only the LEFT button should drive a joint;
    // let the other buttons fall through to OrbitControls.
    const _el = viewer.renderer.domElement;
    const _origMouseDown = dc._mouseDown;
    _el.removeEventListener('mousedown', _origMouseDown);
    dc._mouseDown = e => { if (e.button === 0) { _origMouseDown(e); } };
    _el.addEventListener('mousedown', dc._mouseDown);
  } else {
    if (dc.onHover !== _noopHover && dc.onHover !== _origOnHover) {
      _origOnHover = dc.onHover;
    }
    if (dc.onUnhover !== _noopHover && dc.onUnhover !== _origOnUnhover) {
      _origOnUnhover = dc.onUnhover;
    }
  }
  // the element highlights the hovered link on EVERY pointermove (re-applying
  // it), so a one-shot handler can't undo it -- no-op the hover callbacks in
  // orbit mode instead, so a plain hover/click never lights the part up
  dc.onHover = viewState.poseDrag ? _origOnHover : _noopHover;
  dc.onUnhover = viewState.poseDrag ? _origOnUnhover : _noopHover;
  if (!viewState.poseDrag) {                  // clear anything left highlighted
    viewer.robot?.traverse(c => {
      if (c.isMesh && c.__origMaterial !== undefined) {
        c.material = c.__origMaterial;
        delete c.__origMaterial;
      }
    });
    viewer.redraw();
  }
}
poseBtn.addEventListener('click', () => {
  viewState.poseDrag = poseBtn.classList.toggle('active');
  applyPoseDrag();
  log(viewState.poseDrag ? t('pose.on') : t('pose.off'), 'ok');
});

