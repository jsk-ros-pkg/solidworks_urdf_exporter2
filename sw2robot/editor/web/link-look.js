import { collPreviewRevealColliding } from './coacd-preview.js';
import { viewer } from './dom.js';
import { _tintLink, baseTint, savedMats, tintClones } from './frames.js';
import { selectLink } from './link-info.js';
import { op } from './session-log.js';
import { collisionState, packageState, selectionState } from './state.js';
import { THREE } from './three-setup.js';
// ---- per-link visual colour override -----------------------------------
// We repaint the link's OWN (non-marker) mesh materials -- the same materials
// the tint system saves in `savedMats`, so a tint overlays the override and
// deselect/dehover falls back to it (not the CAD colour).  The very first
// override of a material stashes its original colour so 'reset' can restore it.
// Materials are shared with the mesh cache, so this survives edit-rebuilds; on a
// fresh page load applyPersistedColors() re-applies from the server.
const origMatColor = new WeakMap();   // material -> THREE.Color before overrides
// the tint a link should currently wear, honouring colour-preview suppression
export function _colorTint(name) {
  return name === selectionState.colorPreviewLink ? null : baseTint(name);
}

// recently-set colours, newest first, shown as a reusable palette and kept in
// localStorage so they persist across sessions
const RECENT_KEY = 'sw2robot.recentColors';
const RECENT_MAX = 30;
export let recentColors = [];
try {
  const r = JSON.parse(localStorage.getItem(RECENT_KEY) || '[]');
  if (Array.isArray(r)) {
    recentColors = r.filter(h => /^#[0-9a-f]{6}$/.test(h)).slice(0, RECENT_MAX);
  }
} catch { /* corrupt/absent -> empty palette */ }
export function pushRecentColor(hex) {
  if (!hex) { return; }
  const h = hex.toLowerCase();
  if (!/^#[0-9a-f]{6}$/.test(h)) { return; }
  recentColors = [h, ...recentColors.filter(x => x !== h)].slice(0, RECENT_MAX);
  try { localStorage.setItem(RECENT_KEY, JSON.stringify(recentColors)); }
  catch { /* storage full/blocked -> keep in memory only */ }
}

// start previewing a link's true colour: strip its tint so the colour (picker
// or freshly-applied swatch) is judged without the selection-cyan overlay.  The
// preview persists until a DIFFERENT link is selected (selectLink clears it).
export function beginColorPreview(name) {
  selectionState.colorPreviewLink = name;
  _tintLink(name, null);
  viewer.redraw();
}

// the link's own mesh materials, reading through an in-progress drag-hover the
// same way _tintLink does (so we never lose the true original)
function _linkBaseMaterials(linkName) {
  const link = viewer.robot?.links?.[linkName];
  const out = [];
  if (!link) { return out; }
  (function walk(node) {
    for (const c of node.children) {
      if (c.isURDFJoint) { continue; }
      if (c.isMesh && !c.userData.sw2robotMarker
          && c.material !== viewer.highlightMaterial) {
        const m = savedMats.get(c)
          ?? (c.__origMaterial !== undefined ? c.__origMaterial : c.material);
        if (m && m.color) { out.push(m); }
      }
      walk(c);
    }
  })(link);
  return out;
}

export function applyLinkColor(linkName, hex) {
  const col = new THREE.Color(hex);
  for (const m of _linkBaseMaterials(linkName)) {
    // remember the true original so reset can restore it
    if (!origMatColor.has(m)) {
      origMatColor.set(m, { color: m.color.clone(), vc: m.vertexColors });
    }
    m.color.copy(col);
    // the CAD colours are baked as VERTEX colours (GLB COLOR_0); while those are
    // active the shader does material.color * vertexColour, so a colour change
    // is barely visible -- disable them so the override shows as a solid colour
    if (m.vertexColors) { m.vertexColors = false; m.needsUpdate = true; }
    tintClones.delete(m);            // stale tints must rebuild from new base
  }
  _tintLink(linkName, _colorTint(linkName));   // re-overlay tint unless previewing
  viewer.redraw();
}

export function resetLinkColor(linkName) {
  for (const m of _linkBaseMaterials(linkName)) {
    const orig = origMatColor.get(m);
    if (orig) {
      m.color.copy(orig.color);
      if (m.vertexColors !== orig.vc) { m.vertexColors = orig.vc; m.needsUpdate = true; }
    }
    tintClones.delete(m);
  }
  _tintLink(linkName, baseTint(linkName));
  viewer.redraw();
}

// re-apply every server-persisted override (compMeta.color) to the live robot;
// safe to call repeatedly (idempotent) once geometry + compMeta are both in
export function applyPersistedColors() {
  if (!viewer.robot) { return; }
  for (const [name, hex] of Object.entries(packageState.linkColors)) {
    if (hex && viewer.robot.links[name]) { applyLinkColor(name, hex); }
  }
}

// persist a colour (or null to clear) and apply it live; mirrors set_material's
// flow but needs NO rebuild -- the override is purely a viewer/export concern
export async function setLinkColor(name, hex) {
  op('setColor', { link: name, color: hex });
  selectionState.colorPreviewLink = name;          // keep the true colour visible after applying
  if (hex) { applyLinkColor(name, hex); } else { resetLinkColor(name); }
  try {
    const resp = await fetch('/api/set_color', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ link: name, color: hex }) });
    const r = await resp.json();
    if (!resp.ok || r.error) { throw new Error(r.error ?? resp.status); }
    if (packageState.compMeta[name]) { packageState.compMeta[name].color = r.color; }
    if (r.color) { packageState.linkColors[name] = r.color; } else { delete packageState.linkColors[name]; }
    if (r.color) { pushRecentColor(r.color); }   // remember for the palette
    log(t('li.colorOk'), 'ok');
    if (name === selectionState.selectedLink) { selectLink(name); }   // refresh reset-btn state
  } catch (e) {
    log(t('li.colorFail', { e: e.message ?? e }), 'err');
  }
}

// ---- live self-collision: NEW contacts (not present at the rest pose)
// turn the offending links red.  The hull model lives server-side
// (autoinit.SelfCollision); we POST the joint angles, debounced.
let lastActiveJoint = null;    // joint most recently moved (drag/slider)
const colstatEl = document.getElementById('colstat');

function applyCollision(links, pairs) {
  for (const n of collisionState.collisionLinks) {
    if (!links.has(n)) {
      _tintLink(n, n === selectionState.selectedLink ? 'sel' : null);
    }
  }
  for (const n of links) {
    if (!collisionState.collisionLinks.has(n)) { _tintLink(n, 'col'); }
  }
  collisionState.collisionLinks = links;
  collPreviewRevealColliding(links);   // show colliding links' collision mesh even
                                 // when the CoACD overlay is globally hidden
  if (pairs.length) {
    // "this angle collides": name the joint being moved + its angle
    const j = lastActiveJoint && viewer.robot?.joints?.[lastActiveJoint];
    let at = '';
    if (j) {
      const deg = Number(j.angle) * 180 / Math.PI;
      const u = j.jointType === 'prismatic'
        ? `${(Number(j.angle) * 1000).toFixed(1)} mm`
        : `${deg.toFixed(1)}°`;
      at = `${lastActiveJoint} = ${u} → `;
    }
    colstatEl.textContent = t('col.label', { at,
      list: pairs.map(p => p.join(' × ')).join(',  ') });
    colstatEl.style.display = 'block';
  } else {
    colstatEl.style.display = 'none';
  }
}

async function collisionCheck() {
  if (!collisionState.colReady || !viewer.robot) { return; }
  if (collisionState.colBusy) { collisionState.colQueued = true; return; }
  collisionState.colBusy = true;
  try {
    const angles = {};
    for (const [n, j] of Object.entries(viewer.robot.joints)) {
      if (j.jointType !== 'fixed') { angles[n] = Number(j.angle); }
    }
    const r = await (await fetch('/api/collision', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ angles }) })).json();
    if (r.ready) { applyCollision(new Set(r.links), r.pairs); }
  } catch { /* transient; next angle change retries */ }
  collisionState.colBusy = false;
  if (collisionState.colQueued) { collisionState.colQueued = false; collisionCheck(); }
}

// NOT debounced: a smooth drag fires angle-change ~60x/s, and a trailing
// debounce would reset every frame and never run until the drag STOPPED
// (that was the "collision feels slow / only shows after release" bug).
// colBusy/colQueued instead coalesce the burst down to the query rate
// (~30/s, one request in flight + one queued) so it updates live.
function scheduleCollision(e) {
  if (e?.detail) { lastActiveJoint = e.detail; }   // joint name (drag/slider)
  collisionCheck();
}
viewer.addEventListener('angle-change', scheduleCollision);

export async function initCollision() {
  collisionState.colReady = false;
  clearTimeout(collisionState.colPoll);
  applyCollision(new Set(), []);
  if (packageState.dropMode) { return; }       // server has no matching model to query
  try {
    const r = await (await fetch('/api/collision/init')).json();
    if (r.error) { log(t('col.modelErr', { e: r.error }), 'wrn'); return; }
    if (!r.ready) { collisionState.colPoll = setTimeout(initCollision, 2000); return; }
    collisionState.colReady = true;
    log(t('col.ready', { n: r.baseline }), 'ok');
    collisionCheck();
  } catch { /* server gone; harmless */ }
}

// ---- per-link visibility (the 👁 buttons) --------------------------------
export const hiddenLinks = new Set();

export function applyLinkVisibility(name) {
  const link = viewer.robot?.links?.[name];
  if (!link) { return; }
  const vis = !hiddenLinks.has(name);
  (function walk(n) {                     // own visuals only, not subtree
    for (const c of n.children) {
      if (c.isURDFJoint || c.userData.sw2robotMarker) { continue; }
      c.visible = vis;
      // three.js raycasts HIT invisible meshes: a hidden cover would still
      // grab drags (the element disables orbit while "hovering" its joint)
      // and steal clicks -- so hidden meshes go raycast-invisible too
      if (c.isMesh) {
        if (vis) { delete c.raycast; }
        else { c.raycast = () => {}; }
      }
      walk(c);
    }
  })(link);
  viewer.redraw();
}

export function toggleLinkVisible(name, row) {
  op('eye', { link: name, hide: !hiddenLinks.has(name) });
  hiddenLinks.has(name) ? hiddenLinks.delete(name) : hiddenLinks.add(name);
  applyLinkVisibility(name);
  row?.classList.toggle('hiddenlink', hiddenLinks.has(name));
  row?.querySelector('.eye')?.classList.toggle('off',
                                               hiddenLinks.has(name));
}

