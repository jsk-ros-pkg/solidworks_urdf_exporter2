import { markHelper } from './bootstrap.js';
import { alignBtn, portBtn, viewer } from './dom.js';
import { cancelEndcoords } from './endcoords-gizmo.js';
import { loadRobot } from './load.js';
import { applyRootPose, refreshHistory, rootDelta } from './root-frame.js';
import { op } from './session-log.js';
import { packageState } from './state.js';
import { THREE } from './three-setup.js';
// ---- ⊕ align mode: hover highlights the PLANAR FACE under the cursor
// (triangles with the same normal on the same plane -- a B-rep-less
// approximation of SolidWorks face picking); click puts the root origin
// at the FACE CENTROID with +Z along the face normal ---------------------
export function alignOn() { return alignBtn.classList.contains('active'); }
const faceMat = new THREE.MeshBasicMaterial({
  color: 0xffe93d, transparent: true, opacity: 0.55,
  depthTest: false, side: THREE.DoubleSide });
export let faceOverlay = null;          // {mesh, centroidLocal, normalLocal, host}

export function clearFaceOverlay() {
  faceOverlay?.mesh.removeFromParent();
  faceOverlay = null;
}

export function showFaceOverlay(hit) {
  clearFaceOverlay();
  const host = hit.object;
  const geo = host.geometry;
  const pos = geo.attributes.position;
  const idx = geo.index;
  const n = hit.face.normal.clone().normalize();   // host-local
  const pLocal = host.worldToLocal(hit.point.clone());
  const planeD = pLocal.dot(n);
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  const a = new THREE.Vector3(), b = new THREE.Vector3(),
        c = new THREE.Vector3(), tn = new THREE.Vector3(),
        e1 = new THREE.Vector3(), e2 = new THREE.Vector3();
  const verts = [];
  let area = 0;
  const cx = new THREE.Vector3();
  for (let t = 0; t < triCount; t++) {
    const i0 = idx ? idx.getX(3 * t) : 3 * t;
    const i1 = idx ? idx.getX(3 * t + 1) : 3 * t + 1;
    const i2 = idx ? idx.getX(3 * t + 2) : 3 * t + 2;
    a.fromBufferAttribute(pos, i0);
    b.fromBufferAttribute(pos, i1);
    c.fromBufferAttribute(pos, i2);
    e1.copy(b).sub(a);
    e2.copy(c).sub(a);
    tn.copy(e1).cross(e2);
    const triArea2 = tn.length();
    if (triArea2 < 1e-14) { continue; }
    tn.divideScalar(triArea2);
    if (tn.dot(n) < 0.999) { continue; }           // same orientation
    if (Math.abs(a.dot(n) - planeD) > 3e-4) { continue; }  // same plane
    verts.push(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
    const w = triArea2 / 2;
    area += w;
    cx.addScaledVector(a, w / 3).addScaledVector(b, w / 3)
      .addScaledVector(c, w / 3);
  }
  if (!verts.length) { return; }
  cx.divideScalar(area);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  const m = new THREE.Mesh(g, faceMat);
  m.renderOrder = 996;
  markHelper(m);
  host.add(m);                     // same local frame as the host mesh
  faceOverlay = { mesh: m, centroidLocal: cx.clone(),
                  normalLocal: n.clone(), host, area };
  viewer.redraw();
}

alignBtn.addEventListener('click', () => {
  const on = alignBtn.classList.toggle('active');
  if (on) {
    portBtn.classList.remove('active');   // the two face modes are exclusive
    log(t('align.modeOn'), 'wrn');
  } else {
    clearFaceOverlay();
  }
});

// ---- ⊕ port mode: same face-pick as align, but click drops a coordinate-only
// link (robot-compiler dummy_link) on the clicked LINK at the face center,
// +Z along its normal.  Clicking an existing port marker removes it. ---------
export function portOn() { return portBtn.classList.contains('active'); }

portBtn.addEventListener('click', () => {
  const on = portBtn.classList.toggle('active');
  if (on) {
    alignBtn.classList.remove('active');  // the two face modes are exclusive
    clearFaceOverlay();
    log(t('port.modeOn'), 'wrn');
  } else {
    cancelEndcoords();                    // close any open placement session
    clearFaceOverlay();
  }
});

// --- Bore / cylinder axis fit (align snap-to-circle-centre) -----------------
// When the align click lands on a CYLINDRICAL face (a bore or a pin), snap the
// root origin to the CIRCLE CENTRE on that cylinder's axis rather than the
// clicked surface point.  Everything is computed in the clicked mesh's own
// frame (three.js host-local -> world), so there is no CAD/URDF frame matching.

// welded triangle adjacency (tessellated meshes split vertices at seams, so
// weld by rounded position or edges would never be shared); cached on the geo.
function _boreAdj(geo) {
  const pos = geo.attributes.position, idx = geo.index;
  // rebuild if the geometry's buffers were replaced/updated in place
  const sig = pos.count + ':' + pos.version + ':' +
              (idx ? idx.count + ':' + idx.version : 'n');
  if (geo.userData._boreAdj && geo.userData._boreAdj.sig === sig) {
    return geo.userData._boreAdj;
  }
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  const getI = (t, j) => (idx ? idx.getX(3 * t + j) : 3 * t + j);
  const vid = new Int32Array(pos.count), map = new Map();
  const wx = [], wy = [], wz = [];   // one representative position per welded id
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    const k = Math.round(x * 1e5) + ',' + Math.round(y * 1e5) + ',' + Math.round(z * 1e5);
    let id = map.get(k);
    if (id === undefined) { id = map.size; map.set(k, id); wx.push(x); wy.push(y); wz.push(z); }
    vid[i] = id;
  }
  const edge = new Map(), adj = Array.from({ length: triCount }, () => []);
  for (let t = 0; t < triCount; t++) {
    const v = [vid[getI(t, 0)], vid[getI(t, 1)], vid[getI(t, 2)]];
    for (let e = 0; e < 3; e++) {
      let a = v[e], b = v[(e + 1) % 3];
      if (a > b) { const z = a; a = b; b = z; }
      const k = a + '_' + b, arr = edge.get(k);
      if (arr) { for (const t2 of arr) { adj[t].push(t2); adj[t2].push(t); } arr.push(t); }
      else { edge.set(k, [t]); }
    }
  }
  geo.userData._boreAdj = { adj, getI, triCount, sig, vid, wx, wy, wz };
  return geo.userData._boreAdj;
}

// eigen-decomposition of a symmetric 3x3 (Jacobi), ascending eigenvalues.
function _eig3sym(a) {
  const A = [[a[0], a[1], a[2]], [a[1], a[3], a[4]], [a[2], a[4], a[5]]];
  const V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let iter = 0; iter < 60; iter++) {
    let p = 0, q = 1, mx = Math.abs(A[0][1]);
    if (Math.abs(A[0][2]) > mx) { mx = Math.abs(A[0][2]); p = 0; q = 2; }
    if (Math.abs(A[1][2]) > mx) { mx = Math.abs(A[1][2]); p = 1; q = 2; }
    if (mx < 1e-13) { break; }
    const phi = 0.5 * Math.atan2(2 * A[p][q], A[q][q] - A[p][p]);
    const c = Math.cos(phi), s = Math.sin(phi);
    for (let k = 0; k < 3; k++) { const kp = A[k][p], kq = A[k][q]; A[k][p] = c * kp - s * kq; A[k][q] = s * kp + c * kq; }
    for (let k = 0; k < 3; k++) { const pk = A[p][k], qk = A[q][k]; A[p][k] = c * pk - s * qk; A[q][k] = s * pk + c * qk; }
    for (let k = 0; k < 3; k++) { const kp = V[k][p], kq = V[k][q]; V[k][p] = c * kp - s * kq; V[k][q] = s * kp + c * kq; }
  }
  const vals = [A[0][0], A[1][1], A[2][2]];
  const vecs = [0, 1, 2].map(i => [V[0][i], V[1][i], V[2][i]]);
  const order = [0, 1, 2].sort((i, j) => vals[i] - vals[j]);
  return { vals: order.map(i => vals[i]), vecs: order.map(i => vecs[i]) };
}

// (wPoint, nWorld, radius, count) of the bore the click sits on, or null when
// the clicked face is not a clean cylinder (flat/sphere/cone -> caller falls back)
function fitBoreAxis(hit) {
  try {
    const host = hit.object, geo = host && host.geometry;
    if (hit.faceIndex == null || !geo || !geo.attributes.position) { return null; }
    const pos = geo.attributes.position;
    const { adj, getI, triCount } = _boreAdj(geo);
    const A = new THREE.Vector3(), B = new THREE.Vector3(), C = new THREE.Vector3(),
          e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), tn = new THREE.Vector3();
    const triN = (t) => {
      A.fromBufferAttribute(pos, getI(t, 0)); B.fromBufferAttribute(pos, getI(t, 1));
      C.fromBufferAttribute(pos, getI(t, 2));
      e1.copy(B).sub(A); e2.copy(C).sub(A); tn.copy(e1).cross(e2);
      const l = tn.length(); return l < 1e-14 ? null : tn.clone().divideScalar(l);
    };
    const triCen = (t) => {
      A.fromBufferAttribute(pos, getI(t, 0)); B.fromBufferAttribute(pos, getI(t, 1));
      C.fromBufferAttribute(pos, getI(t, 2));
      return A.clone().add(B).add(C).multiplyScalar(1 / 3);
    };
    const sN = triN(hit.faceIndex); if (!sN) { return null; }
    // smooth-patch flood: cross to a neighbour while the surface bends gently
    // (a curved cylinder wall) and STOP at the sharp edge to any flat face
    const COS = Math.cos(30 * Math.PI / 180);
    const seen = new Uint8Array(triCount); seen[hit.faceIndex] = 1;
    const stack = [[hit.faceIndex, sN]], patch = [], normals = [];
    while (stack.length) {
      const [t, nt] = stack.pop(); patch.push(t); normals.push(nt);
      if (patch.length > 6000) { break; }
      for (const u of adj[t]) {
        if (seen[u]) { continue; }
        const nu = triN(u); if (!nu) { continue; }
        if (nt.dot(nu) >= COS) { seen[u] = 1; stack.push([u, nu]); }
      }
    }
    if (patch.length < 10) {
      log('align bore-fit: patch too small (' + patch.length + ' facets) — using face/point', 'wrn');
      return null;
    }
    let m0 = 0, m1 = 0, m2 = 0, m3 = 0, m4 = 0, m5 = 0;
    for (const n of normals) {
      m0 += n.x * n.x; m1 += n.x * n.y; m2 += n.x * n.z;
      m3 += n.y * n.y; m4 += n.y * n.z; m5 += n.z * n.z;
    }
    const eig = _eig3sym([m0, m1, m2, m3, m4, m5]);
    const e0 = eig.vals[0], eMid = eig.vals[1], eMax = eig.vals[2];
    if (eMax < 1e-9) { return null; }
    // cylinder: a clear axis (e0 ~ 0) AND normals spread around it (eMid sizable);
    // a flat face has eMid ~ 0, a sphere/cone has e0 sizable -> both rejected
    if (e0 / eMax > 0.08 || eMid / eMax < 0.12) {
      log('align bore-fit: not a cylinder (axis=' + (e0 / eMax).toFixed(2) +
          ', spread=' + (eMid / eMax).toFixed(2) + ') — using face/point', 'wrn');
      return null;
    }
    const d = new THREE.Vector3(eig.vecs[0][0], eig.vecs[0][1], eig.vecs[0][2]).normalize();
    const t0 = Math.abs(d.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
    const ub = new THREE.Vector3().crossVectors(d, t0).normalize();
    const vb = new THREE.Vector3().crossVectors(d, ub).normalize();
    const P = patch.map(triCen);
    let mu = 0, mv = 0; const us = [], vs = [];
    for (const p of P) { const pu = p.dot(ub), pv = p.dot(vb); us.push(pu); vs.push(pv); mu += pu; mv += pv; }
    mu /= P.length; mv /= P.length;
    let Suu = 0, Svv = 0, Suv = 0, b1 = 0, b2 = 0;
    for (let i = 0; i < P.length; i++) {
      const x = us[i] - mu, y = vs[i] - mv;
      Suu += x * x; Svv += y * y; Suv += x * y;
      b1 += 0.5 * (x * x * x + x * y * y); b2 += 0.5 * (y * y * y + y * x * x);
    }
    const det = Suu * Svv - Suv * Suv;
    // RELATIVE conditioning (absolute 1e-18 is scale-dependent): reject a
    // near-collinear scatter -- a short/degenerate arc whose 2x2 has a tiny
    // minor axis, where Kasa returns a wild far-away centre
    const tr2 = Suu + Svv, disc = Math.sqrt(Math.max(0, tr2 * tr2 - 4 * det));
    const eMin2 = 0.5 * (tr2 - disc), eMax2 = 0.5 * (tr2 + disc);
    if (eMax2 < 1e-14 || eMin2 / eMax2 < 0.008) {
      log('align bore-fit: near-collinear arc (cond=' +
          (eMax2 > 0 ? (eMin2 / eMax2).toFixed(3) : '0') + ') — using face/point', 'wrn');
      return null;
    }
    const uc = (b1 * Svv - b2 * Suv) / det, vc = (b2 * Suu - b1 * Suv) / det;
    const r = Math.sqrt(uc * uc + vc * vc + (Suu + Svv) / P.length);
    // radius must be plausible vs how much of the circle we actually see: a
    // near-flat sliver otherwise fits a huge circle far from the clicked bore
    let uMin = Infinity, uMax = -Infinity, vMin = Infinity, vMax = -Infinity;
    for (let i = 0; i < P.length; i++) {
      if (us[i] < uMin) { uMin = us[i]; } if (us[i] > uMax) { uMax = us[i]; }
      if (vs[i] < vMin) { vMin = vs[i]; } if (vs[i] > vMax) { vMax = vs[i]; }
    }
    const patchDiag = Math.hypot(uMax - uMin, vMax - vMin);
    if (!(r > 1e-4) || !(patchDiag > 0) || r > 10 * patchDiag) {
      log('align bore-fit: implausible radius (r=' + (r * 1000).toFixed(1) +
          'mm vs patch ' + (patchDiag * 1000).toFixed(1) + 'mm) — using face/point', 'wrn');
      return null;
    }
    let resid = 0;
    for (let i = 0; i < P.length; i++) {
      const x = us[i] - mu - uc, y = vs[i] - mv - vc;
      resid += Math.abs(Math.sqrt(x * x + y * y) - r);
    }
    resid /= P.length;
    if (resid / r > 0.30) {
      log('align bore-fit: circle residual high (' + (resid / r).toFixed(2) + ') — using face/point', 'wrn');
      return null;
    }
    // axis point (host-local, in the projection plane).  Verify the surface is
    // a CYLINDER (constant radius along the axis) not a cone/draft face: the
    // radial distance to the axis must NOT track the axial coordinate.
    const axisPt = new THREE.Vector3().addScaledVector(ub, mu + uc).addScaledVector(vb, mv + vc);
    let sa = 0, sr = 0, saa = 0, srr = 0, sar = 0;
    for (const p of P) {
      const w = p.clone().sub(axisPt);
      const ax = w.dot(d);
      const rad = w.addScaledVector(d, -ax).length();
      sa += ax; sr += rad; saa += ax * ax; srr += rad * rad; sar += ax * rad;
    }
    const nP = P.length, ma = sa / nP, mr = sr / nP;
    const cov = sar / nP - ma * mr, va = saa / nP - ma * ma, vr = srr / nP - mr * mr;
    const corr = (va > 1e-12 && vr > 1e-12) ? cov / Math.sqrt(va * vr) : 0;
    const radStd = Math.sqrt(Math.max(0, vr));
    if (Math.abs(corr) > 0.9 || radStd / r > 0.35) {   // cone / not constant radius
      log('align bore-fit: not constant-radius (corr=' + corr.toFixed(2) +
          ', radStd/r=' + (radStd / r).toFixed(2) + ') — using face/point', 'wrn');
      return null;
    }
    // snap the origin to the circle centre at the CLICKED height along the axis
    const centerLocal = axisPt.clone();
    const hp = host.worldToLocal(hit.point.clone());
    const along = hp.clone().sub(centerLocal).dot(d);
    centerLocal.addScaledVector(d, along);
    const wPoint = host.localToWorld(centerLocal.clone());
    const nLocal = along < 0 ? d.clone().negate() : d.clone();  // +Z toward clicked end
    const nWorld = nLocal.transformDirection(host.matrixWorld).normalize();
    return { wPoint, nWorld, radius: r, count: nP };
  } catch (e) {
    return null;
  }
}

// plain Kasa (algebraic) circle fit of a subset of 2D points
function _kasa(us, vs, inl) {
  let mu = 0, mv = 0;
  for (const i of inl) { mu += us[i]; mv += vs[i]; }
  mu /= inl.length; mv /= inl.length;
  let Suu = 0, Svv = 0, Suv = 0, b1 = 0, b2 = 0;
  for (const i of inl) { const x = us[i] - mu, y = vs[i] - mv; Suu += x * x; Svv += y * y; Suv += x * y; b1 += 0.5 * (x * x * x + x * y * y); b2 += 0.5 * (y * y * y + y * x * x); }
  const det = Suu * Svv - Suv * Suv; if (Math.abs(det) < 1e-18) { return null; }
  const uc = (b1 * Svv - b2 * Suv) / det, vc = (b2 * Suu - b1 * Suv) / det;
  const cu = mu + uc, cv = mv + vc, r = Math.sqrt(uc * uc + vc * vc + (Suu + Svv) / inl.length);
  if (!(r > 1e-4)) { return null; }
  let resid = 0; for (const i of inl) { resid += Math.abs(Math.hypot(us[i] - cu, vs[i] - cv) - r); }
  return { cu, cv, r, resid: resid / inl.length, n: inl.length };
}

// RANSAC circle fit: a face's boundary loop mixes the true rim ARC with a
// notch's straight chord and other non-circular flange features -- often a big
// FRACTION of the points, which median-trimming cannot remove.  Sample circles
// through random point triples, keep the one with the most inliers within a
// tight band, then Kasa-refit on those inliers.  Recovers the true rim circle.
function _fitCircle2D(us, vs) {
  const n = us.length; if (n < 8) { return null; }
  let bx0 = Infinity, bx1 = -Infinity, by0 = Infinity, by1 = -Infinity;
  for (let i = 0; i < n; i++) { if (us[i] < bx0) bx0 = us[i]; if (us[i] > bx1) bx1 = us[i]; if (vs[i] < by0) by0 = vs[i]; if (vs[i] > by1) by1 = vs[i]; }
  const scale = Math.hypot(bx1 - bx0, by1 - by0), tol = Math.max(scale * 0.01, 2e-4);
  const circ3 = (i, j, k) => {
    const ax = us[i], ay = vs[i], bx = us[j], by = vs[j], cx = us[k], cy = vs[k];
    const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (Math.abs(d) < 1e-15) { return null; }
    const a2 = ax * ax + ay * ay, b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
    const ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    const uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    return { cu: ux, cv: uy, r: Math.hypot(ax - ux, ay - uy) };
  };
  let bestInl = [];
  for (let it = 0; it < 300; it++) {
    const i = (Math.random() * n) | 0, j = (Math.random() * n) | 0, k = (Math.random() * n) | 0;
    if (i === j || j === k || i === k) { continue; }
    const c = circ3(i, j, k); if (!c || !(c.r > 1e-4) || c.r > scale * 3) { continue; }
    const inl = [];
    for (let p = 0; p < n; p++) { if (Math.abs(Math.hypot(us[p] - c.cu, vs[p] - c.cv) - c.r) < tol) { inl.push(p); } }
    if (inl.length > bestInl.length) { bestInl = inl; }
  }
  if (bestInl.length < Math.max(8, n * 0.3)) { return null; }   // no dominant circle
  return _kasa(us, vs, bestInl);
}

// (wPoint, nWorld, radius) of the CIRCLE the click sits nearest on a FLAT face:
// fit a circle to the face's boundary-edge loops (outer rim / a hole rim).  A
// NOTCHED or partial circle still yields the true full-circle centre, where the
// area-weighted face centroid would be pulled off toward the solid side.
function fitFaceCircle(hit) {
  try {
    const host = hit.object, geo = host && host.geometry;
    if (hit.faceIndex == null || !geo || !geo.attributes.position) { return null; }
    const pos = geo.attributes.position;
    const { adj, getI, triCount, vid, wx, wy, wz } = _boreAdj(geo);
    const A = new THREE.Vector3(), B = new THREE.Vector3(), C = new THREE.Vector3(),
          e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), tn = new THREE.Vector3();
    const triN = (t) => {
      A.fromBufferAttribute(pos, getI(t, 0)); B.fromBufferAttribute(pos, getI(t, 1));
      C.fromBufferAttribute(pos, getI(t, 2));
      e1.copy(B).sub(A); e2.copy(C).sub(A); tn.copy(e1).cross(e2);
      const l = tn.length(); return l < 1e-14 ? null : tn.clone().divideScalar(l);
    };
    const nSeed = triN(hit.faceIndex); if (!nSeed) { return null; }
    A.fromBufferAttribute(pos, getI(hit.faceIndex, 0));
    const planeD = A.dot(nSeed);
    // flood a PLANAR patch: coplanar (same normal) AND on the same plane
    const seen = new Uint8Array(triCount); seen[hit.faceIndex] = 1;
    const stack = [hit.faceIndex], patch = [hit.faceIndex], P0 = new THREE.Vector3();
    while (stack.length) {
      const t = stack.pop();
      for (const u of adj[t]) {
        if (seen[u]) { continue; }
        const nu = triN(u); if (!nu || nu.dot(nSeed) < 0.999) { continue; }
        P0.fromBufferAttribute(pos, getI(u, 0));
        if (Math.abs(P0.dot(nSeed) - planeD) > 3e-4) { continue; }
        seen[u] = 1; stack.push(u); patch.push(u);
      }
      if (patch.length > 20000) { break; }
    }
    if (patch.length < 4) { return null; }
    // boundary edges: welded-id edges used by exactly ONE patch triangle
    const ecount = new Map();
    for (const t of patch) {
      const v = [vid[getI(t, 0)], vid[getI(t, 1)], vid[getI(t, 2)]];
      for (let e = 0; e < 3; e++) {
        let a = v[e], b = v[(e + 1) % 3]; if (a > b) { const z = a; a = b; b = z; }
        const k = a + '_' + b; ecount.set(k, (ecount.get(k) || 0) + 1);
      }
    }
    // union-find the boundary vertices into connected loops
    const parent = new Map();
    const find = (x) => { while (parent.get(x) !== x) { parent.set(x, parent.get(parent.get(x))); x = parent.get(x); } return x; };
    const add = (x) => { if (!parent.has(x)) { parent.set(x, x); } };
    const bverts = new Set();
    for (const [k, c] of ecount) {
      if (c !== 1) { continue; }
      const s = k.split('_'), a = +s[0], b = +s[1];
      add(a); add(b); bverts.add(a); bverts.add(b); parent.set(find(a), find(b));
    }
    if (!bverts.size) { return null; }
    const loops = new Map();
    for (const v of bverts) { const r = find(v); let arr = loops.get(r); if (!arr) { arr = []; loops.set(r, arr); } arr.push(v); }
    // face-plane basis
    const d = nSeed.clone();
    const t0 = Math.abs(d.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
    const ub = new THREE.Vector3().crossVectors(d, t0).normalize();
    const vb = new THREE.Vector3().crossVectors(d, ub).normalize();
    const cl = host.worldToLocal(hit.point.clone());
    const cKu = cl.dot(ub), cKv = cl.dot(vb);
    let best = null; const tmp = new THREE.Vector3();
    for (const ids of loops.values()) {
      if (ids.length < 8) { continue; }
      const us = [], vs = [];
      for (const id of ids) { tmp.set(wx[id], wy[id], wz[id]); us.push(tmp.dot(ub)); vs.push(tmp.dot(vb)); }
      const fit = _fitCircle2D(us, vs);
      if (!fit || fit.resid / fit.r > 0.05) { continue; }   // RANSAC inliers -> tight
      // prefer the largest circle that ENCLOSES the click (the outer rim of the
      // face you are on), so a nearby bolt hole never wins over the flange edge;
      // fall back to the largest circle when the click is outside them all
      const enc = Math.hypot(cKu - fit.cu, cKv - fit.cv) < fit.r;
      const better = !best || (enc && !best.enc) ||
        (enc === best.enc && fit.r > best.r);
      if (better) { best = { cu: fit.cu, cv: fit.cv, r: fit.r, enc, n: fit.n }; }
    }
    if (!best) { return null; }
    const centerLocal = new THREE.Vector3().addScaledVector(ub, best.cu).addScaledVector(vb, best.cv);
    centerLocal.addScaledVector(d, planeD - centerLocal.dot(d));   // lay on the clicked plane
    const wPoint = host.localToWorld(centerLocal.clone());
    const nWorld = d.clone().transformDirection(host.matrixWorld).normalize();
    return { wPoint, nWorld, radius: best.r, count: best.n };
  } catch (e) {
    return null;
  }
}

export async function alignRootTo(hit) {
  op('align');
  if (!packageState.currentInfo) {
    log(t('align.needPkg'), 'wrn');
    return;
  }
  // 1. snap to a bore/pin circle centre when the click is on a cylinder;
  // 2. else a circle fitted to the clicked FLAT face's edge (outer rim / hole);
  // 3. else the detected flat face centroid; 4. else the raw hit triangle
  let wPoint, nWorld;
  const bore = fitBoreAxis(hit);
  const circ = bore ? null : fitFaceCircle(hit);
  if (bore) {
    wPoint = bore.wPoint;
    nWorld = bore.nWorld;
    clearFaceOverlay();
    log('align: snapped to circle centre (r=' + (bore.radius * 1000).toFixed(1) +
        ' mm, ' + bore.count + ' facets)', 'ok');
  } else if (circ) {
    wPoint = circ.wPoint;
    nWorld = circ.nWorld;
    clearFaceOverlay();
    log('align: snapped to circle centre (edge fit, r=' + (circ.radius * 1000).toFixed(1) +
        ' mm, ' + circ.count + ' pts)', 'ok');
  } else if (faceOverlay) {
    wPoint = faceOverlay.host.localToWorld(
      faceOverlay.centroidLocal.clone());
    nWorld = faceOverlay.normalLocal.clone()
      .transformDirection(faceOverlay.host.matrixWorld);
    log(t('align.faceCenter', { area: (faceOverlay.area * 1e6).toFixed(0) }));
  } else {
    wPoint = hit.point.clone();
    nWorld = hit.face.normal.clone()
      .transformDirection(hit.object.matrixWorld);
  }
  clearFaceOverlay();
  // robot-local = LOAD-time root frame; convert into the CURRENT root
  // frame (in-place edits move the frame without reloading the URDF)
  const inv = rootDelta.clone().invert();
  const pLocal = viewer.robot.worldToLocal(wPoint).applyMatrix4(inv);
  const qInv = viewer.robot.getWorldQuaternion(
    new THREE.Quaternion()).invert();
  const nLocal = nWorld.applyQuaternion(qInv)
    .transformDirection(inv).normalize();
  log(t('align.aligning', {
    origin: `${pLocal.x.toFixed(4)}, ${pLocal.y.toFixed(4)}, ${pLocal.z.toFixed(4)}`,
    zdir: `${nLocal.x.toFixed(2)}, ${nLocal.y.toFixed(2)}, ${nLocal.z.toFixed(2)}` }));
  statusEl.textContent = t('align.status');
  try {
    const resp = await fetch('/api/set_root_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ xyz: pLocal.toArray(),
                             zdir: nLocal.toArray() }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('align.ok'), 'ok');
    applyRootPose(r.rpy, r.xyz);     // in-place: no reload, no flash
    refreshHistory();
  } catch (e) {
    log(t('align.fail', { e: e.message ?? e }), 'err');
  }
}

export async function removePort(name) {
  op('remove_port');
  if (!packageState.currentInfo) { return; }
  log(t('port.removing', { name }));
  statusEl.textContent = t('port.removeStatus');
  try {
    const resp = await fetch('/api/remove_port', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    log(t('port.removed', { name }), 'ok');
    loadRobot(packageState.currentInfo, { keepPose: true });
    refreshHistory();
  } catch (e) {
    log(t('port.removeFail', { e: e.message ?? e }), 'err');
  }
}

