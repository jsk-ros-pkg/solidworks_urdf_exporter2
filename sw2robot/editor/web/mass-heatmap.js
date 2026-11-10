import { viewer } from './dom.js';
import { applyLinkColor, applyPersistedColors, resetLinkColor } from './link-look.js';
import { selectLink } from './link-info.js';
import { fmtMass } from './mass-editor.js';
import { op } from './session-log.js';
import { packageState } from './state.js';
import { THREE } from './three-setup.js';
// ---- mass heat view ------------------------------------------------------
// Paint every link by how much it weighs, so "which part of this robot carries
// the mass" is a glance instead of a 60-row table read.  This is a VIEW: it
// borrows link-look's colour override (the same one the per-link picker uses)
// and hands it straight back on toggle-off via applyPersistedColors(), so no
// saved colour is lost and nothing is written to the package.
//
// Two deliberate choices:
//
//  * log scale.  Link masses on a real robot span 3-4 decades (a 20 kg torso
//    next to a 3 g bracket); on a linear ramp everything but the single
//    heaviest link comes out the same colour and the view says nothing.
//  * default-mass links are NOT on the scale.  A link the exporter fell back to
//    a SolidWorks default mass for has a made-up number; painting it "cold"
//    would present a placeholder as a measurement.  Those get a flat
//    out-of-scale violet and a legend line saying how many there are.

// Blue (light) -> red (heavy), following the standard advice for colouring a
// SHADED 3D SURFACE rather than a flat chart -- the two want opposite things:
//
//   "in a 3D scene, shading cues, which are themselves changes in brightness,
//    are vital to understanding shapes. Thus, you have to avoid having the
//    brightness changes in the color map interfere with the brightness changes
//    in shading."   -- Moreland, kennethmoreland.com/color-advice/
//
// So brightness is NOT allowed to carry the data here; hue is.  Measured
// luminance swing across this ramp is 0.056, against 0.588 for stock
// cool-warm, 0.738 for turbo and 0.763 for viridis -- an order of magnitude
// flatter, which leaves the renderer's shading free to describe the geometry.
// (A rainbow ramp is the worst of both: it swings hardest in brightness AND
// its yellow-green peak is the part that vanishes against this viewer's pale
// CAD background, which is what made the middle of the scale unreadable.)
//
// The path is matplotlib's `coolwarm` -- i.e. Moreland's diverging cool-warm,
// the map that advice recommends for 3D surfaces -- with its hue/chroma kept
// and its lightness re-seated into a narrow L* 49-55 band.  Stock cool-warm
// passes through near-white at the midpoint, which is invisible on a white-ish
// background (contrast 1.01); re-seated, the whole ramp sits at contrast
// 2.7-3.3.  Validated again as RENDERED through the lighting rather than as
// assigned -- worst case anywhere on the ramp is 2.35, on a fully lit facet of
// the grey-mauve middle -- because what the eye judges is the shaded pixel, not
// the value we hand the material.  (See _unmetal in capture-progress.js: until
// that landed, every one of these arrived on screen as a dark metal.)
//
// Cost of a diverging map on single-ended data, stated plainly: the middle
// (grey-mauve) is where two links are hardest to tell apart, and its position
// is just the log-midpoint of the range, not a meaningful threshold.  What it
// buys is that "heavy" and "negligible" are unmistakable at a glance.
const RAMP = [
  [0x44, 0x63, 0xff], [0x1b, 0x71, 0xf9], [0x28, 0x7b, 0xe3], [0x52, 0x81, 0xc2],
  [0x75, 0x85, 0x9b], [0x97, 0x7e, 0x70], [0xb5, 0x6e, 0x4d], [0xce, 0x56, 0x36],
  [0xe1, 0x35, 0x2d], [0xef, 0x00, 0x2f],
];

// Default-mass links: off the ramp entirely, separated by HUE at the same
// lightness as the ramp (L* 50.7) so it does not fight the shading either.
const FLAG_COLOR = '#a850d8';
// No <inertial> at all (frame-only links).  The one colour deliberately kept
// pale, so "carries no weight" recedes.  Checked as RENDERED, not as assigned:
// through the lighting this reads at contrast 1.41 on a fully lit facet and
// 3.10 on a shaded one -- faint, but never the invisible 1.04 that a lighter
// grey collapses to, and lower than the ramp at every equal lighting angle.
const ZERO_COLOR = '#a6acb4';

function rampColor(t) {
  const x = Math.min(1, Math.max(0, t)) * (RAMP.length - 1);
  const i = Math.min(RAMP.length - 2, Math.floor(x));
  const f = x - i;
  const hex = n => Math.round(n).toString(16).padStart(2, '0');
  return '#' + [0, 1, 2]
    .map(k => hex(RAMP[i][k] + (RAMP[i + 1][k] - RAMP[i][k]) * f))
    .join('');
}

// ---- which number the heat paints ----------------------------------------
// The same view answers three questions, switched from the legend:
//
//   mass     -- where the robot's weight sits (the original view).
//   bytes    -- where the LOAD cost sits: on-disk size of each link's visual
//               mesh files, i.e. editor start-up, package size, git weight.
//               Tracks `tris` for a uniform GLB package, but not across
//               formats (text COLLADA is several times GLB) or with textures.
//               The server measures it; the browser cannot see file sizes.
//   tris     -- where the RENDER cost sits: triangle count of each link's own
//               visual meshes, which is what makes the editor, RViz and MuJoCo
//               slow.  Big parts are expected to rank high here.
//   density  -- triangles per cm^2 of surface: over-tessellation, i.e. where
//               triangles buy no visible shape (threads, knurling, embossed
//               text, springs).  This is the "what is worth decimating" list,
//               which raw count cannot give -- a large link is heavy AND fine.
//
// Only <visual> meshes are counted.  The loader also builds every <collision>
// mesh (as a hidden URDFCollider, often the very same file), and the CoACD
// preview hangs under the link too; both are skipped, or a link whose collision
// reuses its visual would read as exactly twice its real triangle count.
export const HEAT_METRICS = ['mass', 'bytes', 'tris', 'density'];
export let heatMetric = 'mass';

// triangle count + LOCAL-space area per geometry.  Geometries are shared through
// the mesh cache, and area is a full pass over the triangles, so it is done once
// per geometry, not on every repaint.
const geomStats = new WeakMap();
const _a = [0, 0, 0], _b = [0, 0, 0], _c = [0, 0, 0];

function geometryStats(geom) {
  const hit = geomStats.get(geom);
  if (hit) { return hit; }
  const pos = geom.attributes?.position;
  const idx = geom.index;
  if (!pos) { return { tris: 0, area: 0 }; }
  const n = Math.floor((idx ? idx.count : pos.count) / 3);
  const at = (k, out) => {
    const v = idx ? idx.getX(k) : k;
    out[0] = pos.getX(v); out[1] = pos.getY(v); out[2] = pos.getZ(v);
  };
  let area = 0;
  for (let f = 0; f < n; f += 1) {
    at(3 * f, _a); at(3 * f + 1, _b); at(3 * f + 2, _c);
    const ux = _b[0] - _a[0], uy = _b[1] - _a[1], uz = _b[2] - _a[2];
    const vx = _c[0] - _a[0], vy = _c[1] - _a[1], vz = _c[2] - _a[2];
    const cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
    area += 0.5 * Math.sqrt(cx * cx + cy * cy + cz * cz);
  }
  const s = { tris: n, area };
  geomStats.set(geom, s);
  return s;
}

// a link's OWN visual meshes (not its subtree, colliders, markers or the
// CoACD preview): total triangles and world-space surface area in m^2.  The area is
// rescaled by the mesh's world scale, which is how millimetre CAD parts end up
// in metres (mesh scale 0.001).
const _scale = new THREE.Vector3();
function linkMeshStats(linkName) {
  const link = viewer.robot?.links?.[linkName];
  let tris = 0, area = 0;
  if (!link) { return { tris, area }; }
  (function walk(node) {
    for (const c of node.children) {
      if (c.isURDFJoint || c.isURDFCollider || c.userData.sw2robotMarker
          || c.userData.sw2robotCollPreview) { continue; }
      if (c.isMesh && c.geometry) {
        const g = geometryStats(c.geometry);
        c.getWorldScale(_scale);
        const s2 = Math.abs(_scale.x * _scale.y * _scale.z) ** (2 / 3);
        tris += g.tris;
        area += g.area * s2;
      }
      walk(c);
    }
  })(link);
  return { tris, area };
}

const fmtCount = n => (n >= 1e6 ? (n / 1e6).toFixed(2) + 'M'
  : n >= 1e3 ? (n / 1e3).toFixed(1) + 'k' : String(Math.round(n)));

// per-metric: the ranked [link, value] pairs on the scale, the placeholder
// set, and (mesh metrics) each link's raw stats for the legend.  Mass keeps its placeholder-mass flagging; the
// mesh metrics have no notion of a fake value, so nothing is flagged there.
function metricEntries(metric) {
  const robot = viewer.robot;
  const none = { entries: [], flagged: new Set(), stats: {}, missing: [] };
  if (!robot) { return none; }
  if (metric === 'mass') {
    const entries = heatEntries();
    return { ...none, entries, flagged: flaggedSet(entries) };
  }
  if (metric === 'bytes') {
    const entries = Object.entries(packageState.urdfMeshBytes ?? {})
      .filter(([ln, b]) => robot.links[ln] && typeof b === 'number' && b > 0)
      .sort((a, b) => b[1] - a[1]);
    const missing = (packageState.urdfMeshMissing ?? [])
      .filter(ln => robot.links[ln]);
    return { ...none, entries, missing };
  }
  const stats = {};
  const entries = [];
  for (const ln of Object.keys(robot.links)) {
    const s = linkMeshStats(ln);
    if (s.tris <= 0) { continue; }
    stats[ln] = s;
    if (metric === 'tris') {
      entries.push([ln, s.tris]);
    } else if (s.area > 0) {
      entries.push([ln, s.tris / (s.area * 1e4)]);   // per cm^2
    }
  }
  entries.sort((a, b) => b[1] - a[1]);
  return { ...none, entries, stats };
}

function fmtValue(metric, v) {
  if (metric === 'mass') { return fmtMass(v); }
  if (metric === 'bytes') {
    return v >= 1e6 ? (v / 1e6).toFixed(2) + ' MB'
      : v >= 1e3 ? (v / 1e3).toFixed(1) + ' kB' : `${v} B`;
  }
  if (metric === 'tris') { return t('heat.tris', { n: fmtCount(v) }); }
  return t('heat.dens', { d: v >= 100 ? v.toFixed(0) : v.toPrecision(2) });
}

export let massHeatOn = false;
let painted = [];                 // links we recoloured, so toggle-off is exact

// The links this view can speak about: present in the loaded robot AND carrying
// a mass in the built URDF.  Keyed by the viewer's link names, which is what
// `urdf_masses` uses, so no name mapping is needed.
function heatEntries() {
  const robot = viewer.robot;
  if (!robot) { return []; }
  return Object.entries(packageState.urdfMasses ?? {})
    .filter(([ln, m]) => robot.links[ln]
                         && typeof m === 'number' && isFinite(m) && m > 0)
    .sort((a, b) => b[1] - a[1]);
}

// which links carry a placeholder mass.  `default_mass_links` is keyed by the
// COMPONENT name in CAD mode, so match a display link both ways -- the same
// two-sided lookup the mass panel does.
function flaggedSet(entries) {
  const flagged = new Set(packageState.defaultMassLinks ?? []);
  const out = new Set();
  for (const [ln] of entries) {
    const comp = packageState.compMeta[ln];
    const acked = comp?.reviewed;
    if (acked) { continue; }        // user acknowledged it: treat as real
    if (flagged.has(ln) || (comp?.name && flagged.has(comp.name))
        || comp?.default_mass) {
      out.add(ln);
    }
  }
  return out;
}

function clearPaint() {
  for (const ln of painted) { resetLinkColor(ln); }
  painted = [];
}

// The log ramp, derived once from the links actually ON the scale (flagged ones
// are excluded so a fabricated 0.1 kg cannot stretch the bounds).  Shared by the
// paint pass and the legend so a swatch always matches the link it names.
function heatScale(entries, flagged) {
  const logs = entries.filter(([ln]) => !flagged.has(ln))
    .map(([, m]) => Math.log10(m));
  if (!logs.length) {
    return { lo: null, hi: null, color: () => FLAG_COLOR };
  }
  const lo = Math.min(...logs), hi = Math.max(...logs), span = hi - lo;
  return {
    lo: 10 ** lo, hi: 10 ** hi,
    // a single-value (or all-equal) robot has no ramp to place anything on:
    // paint the top of it rather than dividing by a zero span
    color: (ln, m) => (flagged.has(ln) ? FLAG_COLOR
      : rampColor(span > 1e-9 ? (Math.log10(m) - lo) / span : 1)),
  };
}


// (re)paint the whole robot; safe to call whenever the model, the masses or the
// metric changed.  A no-op unless the view is on.
export function refreshMassHeat() {
  if (!massHeatOn) { return; }
  clearPaint();
  const { entries, flagged, stats, missing } = metricEntries(heatMetric);
  if (entries.length) {
    const scale = heatScale(entries, flagged);
    for (const [ln, v] of entries) {
      applyLinkColor(ln, scale.color(ln, v));
      painted.push(ln);
    }
    // every link the ramp did NOT cover (no <inertial> / no mesh): grey, so
    // "nothing to measure" reads as its own state rather than as the bottom
    // of the ramp
    // a link whose mesh file was not found has no size to place: flag it
    // (violet, like a placeholder mass) instead of letting it read as light
    for (const ln of missing) {
      applyLinkColor(ln, FLAG_COLOR);
      painted.push(ln);
    }
    const done = new Set(painted);
    for (const ln of Object.keys(viewer.robot?.links ?? {})) {
      if (!done.has(ln)) {
        applyLinkColor(ln, ZERO_COLOR);
        painted.push(ln);
      }
    }
  }
  // the legend stays up even with nothing to paint: it holds the metric
  // switch, and dropping it would strand the user on an empty metric
  renderLegend(entries, flagged, stats, missing);
}

export function setMassHeat(on) {
  massHeatOn = !!on;
  document.getElementById('massheat')?.classList.toggle('active', massHeatOn);
  if (massHeatOn) {
    refreshMassHeat();
    if (!metricEntries(heatMetric).entries.length) {
      log(t(`heat.none.${heatMetric}`), 'wrn');
    }
  } else {
    clearPaint();
    applyPersistedColors();      // hand the user's own colours back
    document.getElementById('heatlegend')?.remove();
  }
  viewer.redraw();
}

export function setHeatMetric(metric) {
  if (!HEAT_METRICS.includes(metric)) {
    throw new Error(`unknown heat metric: ${metric}`);
  }
  heatMetric = metric;
  if (massHeatOn) {
    refreshMassHeat();
    viewer.redraw();
  }
}

// ---- legend --------------------------------------------------------------
// The gradient alone does not answer "how much is a lot"; the ramp ends and a
// top-5 list do, and each entry selects its link so the 3D view and the number
// stay tied together.
function renderLegend(entries, flagged, stats, missing) {
  document.getElementById('heatlegend')?.remove();
  if (!massHeatOn) { return; }
  const el = document.createElement('div');
  el.id = 'heatlegend';
  const metric = heatMetric;
  const switcher = '<div class="heatmetric">' + HEAT_METRICS.map(m =>
    `<button data-metric="${m}" class="${m === metric ? 'active' : ''}"`
    + ` title="${t(`heat.m.${m}.t`).replace(/"/g, '&quot;')}">${t(`heat.m.${m}`)}</button>`)
    .join('') + '</div>';
  let body;
  if (!entries.length) {
    body = `<div class="heatflag">${t(`heat.none.${metric}`)}</div>`;
  } else {
    const stops = RAMP.map((_, i) => rampColor(i / (RAMP.length - 1)));
    const { lo, hi, color } = heatScale(entries, flagged);
    // density is a ratio: summing it is meaningless, so its rows carry the
    // link's triangle count instead of a share of the total
    const total = metric === 'density' ? 0 : entries.reduce((a, [, v]) => a + v, 0);
    const note = (ln, v) => (metric === 'density'
      ? t('heat.tris', { n: fmtCount(stats[ln].tris) })
      : t('heat.share', { p: total ? (v / total * 100).toFixed(1) : '0.0' }));
    const top = entries.slice(0, 5).map(([ln, v]) =>
      `<div class="heatrow" data-link="${ln.replace(/"/g, '&quot;')}">`
      + `<span class="heatsw" style="background:${color(ln, v)}"></span>`
      + `<span class="heatname" title="${ln.replace(/"/g, '&quot;')}">${ln}</span>`
      + `<span class="heatval">${fmtValue(metric, v)}</span>`
      + `<span class="heatpct">${note(ln, v)}</span></div>`).join('');
    body =
      `<div class="heatramp" style="background:linear-gradient(90deg,${stops.join(',')})"></div>`
      + `<div class="heatends"><span>${lo == null ? '' : fmtValue(metric, lo)}</span>`
      + `<span>${hi == null ? '' : fmtValue(metric, hi)}</span></div>`
      + (metric === 'tris'
        ? `<div class="heattitle" style="margin-top:4px">${t('heat.total', { n: fmtCount(total) })}</div>` : '')
      + (metric === 'bytes'
        ? `<div class="heattitle" style="margin-top:4px">${t('heat.totalBytes', { s: fmtValue(metric, total) })}</div>` : '')
      + (flagged.size ? `<div class="heatflag">${t('heat.flagged', { n: flagged.size })}</div>` : '')
      + (missing.length ? `<div class="heatflag">${t('heat.missing', { n: missing.length })}</div>` : '')
      + `<div class="heattitle" style="margin-top:6px">${t(`heat.top.${metric}`)}</div>${top}`;
  }
  el.innerHTML = switcher
    + `<div class="heattitle">${t(`heat.legend.${metric}`)}</div>` + body;
  el.querySelectorAll('.heatmetric button').forEach(b =>
    b.addEventListener('click', () => {
      if (b.dataset.metric === heatMetric) { return; }
      op('heatMetric', { metric: b.dataset.metric });
      setHeatMetric(b.dataset.metric);
    }));
  el.querySelectorAll('.heatrow').forEach(r =>
    r.addEventListener('click', () => selectLink(r.dataset.link)));
  (document.getElementById('viewer').parentElement || document.body).appendChild(el);
}

document.getElementById('massheat')?.addEventListener('click', () => {
  op('massHeat', { on: !massHeatOn });
  setMassHeat(!massHeatOn);
});
