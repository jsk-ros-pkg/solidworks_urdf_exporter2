import {
  collPreviewAbort, collPreviewClearOverlay, collPreviewPoll,
  setCollisionShown,
} from './coacd-preview.js';
import {
  collModeSel, cqualitySel, expmeshdir, exppkg, exprobot, expurdf,
} from './dom.js';
import { collisionState, packageState } from './state.js';
// ---- export names: thread the chosen package + URDF + robot names into the
// ZIP links.  The ROS1/ROS2/glb downloads are plain <a download> links; on click
// we rewrite their ?name=&urdf=&robotname= from these fields so the exported
// package (and zip) carry them.  The names cascade when left empty: URDF stem ->
// package name, <robot name> -> URDF stem.
function exportDefaultName() {
  // mirror the server: sanitise the assembly name to a valid ROS package name
  // ('Assem1' -> 'assem1_description') so the placeholder matches what ships
  const n = packageState.currentInfo?.name;
  if (!n) { return 'robot_description'; }
  let base = n.toLowerCase().replace(/[^a-z0-9_]/g, '_').replace(/^_+|_+$/g, '');
  if (!base || !/[a-z]/.test(base[0])) { base = base ? 'robot_' + base : 'robot'; }
  return `${base}_description`;
}
function effectivePkgName() {
  return (exppkg?.value || '').trim() || exportDefaultName();
}
// the URDF stem defaults to the package name, and the <robot name> inside the
// URDF to that stem -- so renaming the package renames all three
function effectiveUrdfName() {
  return (expurdf?.value || '').trim() || effectivePkgName();
}
function refreshNamePlaceholders() {
  if (expurdf) { expurdf.placeholder = effectivePkgName(); }  // urdf = pkg name
  if (exprobot) { exprobot.placeholder = effectiveUrdfName(); }  // robot = urdf
}
export function refreshExportName() {
  if (exppkg) { exppkg.placeholder = exportDefaultName(); }
  refreshNamePlaceholders();
}
// keep the package field a valid ROS package name as the user types (lowercase,
// digits, underscore; must start with a letter) -- matches the server check
exppkg?.addEventListener('input', () => {
  const clean = exppkg.value.toLowerCase()
    .replace(/[^a-z0-9_]/g, '_').replace(/^[^a-z]+/, '');
  if (exppkg.value !== clean) { exppkg.value = clean; }
  refreshNamePlaceholders();
});
// the URDF stem is a filename: letters/digits/_/-/. , starting alphanumeric
expurdf?.addEventListener('input', () => {
  const clean = expurdf.value
    .replace(/[^A-Za-z0-9_.-]/g, '_').replace(/^[^A-Za-z0-9]+/, '');
  if (expurdf.value !== clean) { expurdf.value = clean; }
  refreshNamePlaceholders();
});
// the <robot name> takes the same character set as the URDF stem
exprobot?.addEventListener('input', () => {
  const clean = exprobot.value
    .replace(/[^A-Za-z0-9_.-]/g, '_').replace(/^[^A-Za-z0-9]+/, '');
  if (exprobot.value !== clean) { exprobot.value = clean; }
});
// the mesh dir is a package-relative path: letters/digits/_/-/. segments joined
// by '/'; drop other chars and any leading slash (must stay inside the package)
expmeshdir?.addEventListener('input', () => {
  const clean = expmeshdir.value
    .replace(/[^A-Za-z0-9_./-]/g, '_').replace(/^\/+/, '');
  if (expmeshdir.value !== clean) { expmeshdir.value = clean; }
});
// building a ROS package converts every mesh (3dxml -> dae/stl) and can take a
// few seconds, during which a plain <a download> gives NO feedback.  Drive it
// over fetch instead: show an "exporting ..." loadbar, then hand the finished
// blob to a throwaway download link.
// Collision geometry for the ROS packages: 'copy' (visual mesh), 'hull'  or '
// coacd' (convex decomposition into many parts).
const collGenRow = document.getElementById('collgenrow');

// quality only applies to CoACD; the Generate/preview row only to hull+coacd
export function updateCollUI() {
  const mode = collModeSel?.value || 'copy';
  if (cqualitySel) { cqualitySel.style.display = mode === 'coacd' ? '' : 'none'; }
  if (collGenRow) { collGenRow.style.display = mode === 'copy' ? 'none' : 'flex'; }
}
collModeSel?.addEventListener('change', () => {
  updateCollUI();
  // a generate job still in flight belongs to the PREVIOUS mode -- abandon it,
  // else its completion would finalize against the old mode and silently revert
  // this deliberate switch (re-applying the old parts to live + export).
  if (collPreviewPoll) { collPreviewAbort(); }
  // any generated preview belongs to the PREVIOUS mode, so switching type drops
  // it: nothing is shown until a preview is made for the newly-selected type
  // (copy has none; hull/coacd must be (re)generated).
  collPreviewClearOverlay();
  collisionState.collPreviewFinalized = false;
  setCollisionShown(false);
});
updateCollUI();

