// ---- viewer wiring: right-click, lighting, highlight material, markHelper --
import { viewer } from './dom.js';
import { packageState } from './state.js';
import { THREE } from './three-setup.js';
// suppress the browser's native right-click menu ("copy/save image", ...) on
// the viewport -- it hijacks the right-drag pan gesture mid-motion
viewer.addEventListener('contextmenu', ev => ev.preventDefault());

// every helper visual (axes, triads, TF, grid, face overlay) must be
// INVISIBLE TO RAYCASTS: the element's drag controls raycast the whole
// scene, and three.js hits Line objects within a 1 m (!) default
// threshold -- the grid was eating every drag attempt
export function markHelper(o) {
  o.userData.sw2robotMarker = true;
  o.raycast = () => {};
  o.children?.forEach(markHelper);
  return o;
}

// brighter than the element defaults (hemi 0.5 / dir PI), which read dark
// on small machined parts; the directional light FOLLOWS THE CAMERA
// (SolidWorks-style headlight) so no orbit angle ends up in shadow
viewer.ambientLight.intensity = 1.2;
viewer.directionalLight.intensity = Math.PI * 1.1;
viewer.directionalLight.castShadow = false;
function headlight() {
  const dl = viewer.directionalLight;
  dl.position.copy(viewer.camera.position);
  dl.target.position.copy(viewer.controls.target);
  dl.target.updateMatrixWorld();
}
viewer.controls.addEventListener('change', headlight);
headlight();
// right-drag pans in the camera's image plane (not parallel to the ground),
// so panning behaves the same regardless of how the model is oriented
viewer.controls.screenSpacePanning = true;
// urdf-viewer._updateEnvironment() runs EVERY frame and pins controls.target.y
// to the model centre. That clobber fights any pan/focus that moves the orbit
// point vertically -- the pivot snaps back to mid-height and the camera pitches
// on the next frame. The `no-auto-recenter` attribute is SUPPOSED to disable it
// but does not in urdf-loader 0.12.7 (verified: _updateEnvironment still fires
// ~every frame with the attribute set). So wrap it to preserve OUR target -- it
// still positions the shadow plane/light, which we don't use, so nothing else
// changes. We drive the camera ourselves (setView/fitView, double-click, V).
viewer.setAttribute('no-auto-recenter', '');     // harmless; in case a build honours it
const _origUpdateEnv = viewer._updateEnvironment.bind(viewer);
viewer._updateEnvironment = function () {
  const savedTarget = viewer.controls.target.clone();
  _origUpdateEnv();
  viewer.controls.target.copy(savedTarget);      // undo the target.y clobber
};

// the element's default highlight (Phong) renders black in this scene --
// replace it with the same material family as the GLB meshes, with an
// emissive so it glows amber under any lighting
// drag-hover highlight: the element swaps the subtree to its single flat
// highlightMaterial (which hides all surface detail; single-sided ones
// even rendered black on our flipped-normal GLBs).  Immediately after its
// swap (the joint-mouseover event fires synchronously, before any render)
// we replace each mesh with an emissive CLONE of its original material --
// the part glows amber but keeps its colours and shading.  The element
// restores the true originals itself on mouseout (__origMaterial).
viewer.highlightMaterial = new THREE.MeshBasicMaterial({
  color: 0xffb340, side: THREE.DoubleSide, toneMapped: false });
// a directly-opened URDF has no graph.json behind it, so the CAD-only per-link
// controls (frame-only, mass source, ...) do not apply
export function urdfInputMode() { return packageState.currentInfo?.mode === 'urdf'; }
// HTML-attribute escape for text that goes into a title="" (classifier notes
// carry quotes and angle brackets)
export function escAttr(v) { return String(v ?? '').replace(/[&<>"]/g,
  c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

