// sw2robot web editor E2E suite (puppeteer-core + system Chrome).
//
// Prereqs:
//   1. server running:  uv run python -m sw2robot.editor.webserver output/<pkg> --root output --port 8090
//   2. once:            cd tests/e2e && npm i
// Run:
//   node tests/e2e/run.mjs [url]
//
// The suite is package-agnostic (picks links from the live tree) and
// restores everything it changes (root, visibility) before exiting.
import puppeteer from 'puppeteer-core';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

// force Japanese so the suite's display-text assertions are deterministic
// regardless of the test machine's browser locale (?lang= overrides detection)
const BASE = process.argv[2] ?? 'http://localhost:8090';
const URL = BASE + (BASE.includes('?') ? '&' : '?') + 'lang=ja';

// Smoke mode (E2E_SMOKE=1, used by CI against the tiny bundled fingertip
// example): skip the sections that need a package with movable joints /
// many meshes (the auto-limits sweep), which the minimal fixture can't
// exercise.  Local full runs against a real package leave it unset.
const SMOKE = process.env.E2E_SMOKE === '1';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';

let failures = 0;
const check = (name, ok, detail = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`);
  if (!ok) { failures += 1; }
};
const sleep = ms => new Promise(r => setTimeout(r, ms));

// CI (headless Linux) needs --no-sandbox etc.; pass them via CHROME_ARGS so
// local runs keep their default launch flags untouched.
const EXTRA_ARGS = (process.env.CHROME_ARGS ?? '')
  .split(',').map(s => s.trim()).filter(Boolean);
const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new',
  args: ['--disable-gpu', ...EXTRA_ARGS] });
const page = await browser.newPage();
const pageErrors = [];
page.on('pageerror', e => pageErrors.push(e.message.split('\n')[0]));

// ---- 1. load --------------------------------------------------------------
// Don't gate on networkidle2: the live editor polls /api/collision every 2 s,
// so the network is never "idle" for long on a slow CI runner and the
// navigation would time out even though the page is fine.  Wait for the DOM,
// then for the page's OWN done-loading signal (the "camera fitted" log line),
// bounded by a generous cap -- if it never appears the check below fails
// clearly instead of the whole suite dying on a navigation timeout.
await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(
  () => [...document.querySelectorAll('#log div')]
    .some(d => d.textContent.includes('カメラを調整しました')),
  { timeout: 60000 }).catch(() => {});
await sleep(1500);              // let the first render settle
const logText = await page.evaluate(() =>
  [...document.querySelectorAll('#log div')].map(d => d.textContent).join('\n'));
check('load: camera fitted', logText.includes('カメラを調整しました'));
check('load: no page errors', pageErrors.length === 0, pageErrors[0] ?? '');

const rootPose = async () =>
  await page.evaluate(async () => (await (await fetch('/api/root_pose')).json()));
const origBase = (await rootPose()).base;

// pick the first non-root tree row's link
const target = await page.evaluate(() => {
  const row = [...document.querySelectorAll('.joint:not(.root)')][0];
  return row?.querySelector('.jname')?.textContent;
});
check('tree: has rows', !!target, `target=${target}`);

// ---- 2. selection: info card at the top-left overlay ----------------------
await page.evaluate(t => {
  [...document.querySelectorAll('.joint:not(.root)')]
    .find(r => r.querySelector('.jname')?.textContent === t)
    .querySelector('.jname').click();
}, target);
await sleep(700);
const sel = await page.evaluate(() => {
  const li = document.getElementById('linkinfo');
  const r = li.getBoundingClientRect();
  return { bar: document.getElementById('selbar').style.display === 'flex',
           shown: r.width > 50 && r.height > 30,
           x: Math.round(r.x), y: Math.round(r.y),
           text: li.textContent.length,
           // the card must be interactive so it can SCROLL to controls below the
           // fold (a tall per-link panel) -- pointer-events:none let scroll pass
           // through to the 3D view, stranding the colour/inertial editors
           pe: getComputedStyle(li).pointerEvents,
           overflow: getComputedStyle(li).overflowY };
});
check('select: selbar visible', sel.bar);
check('select: info card rendered top-left', sel.shown && sel.x < 50 && sel.y < 200,
      `rect=(${sel.x},${sel.y}) chars=${sel.text}`);
check('select: info card is scrollable (interactive)',
      sel.pe === 'auto' && sel.overflow === 'auto', `pe=${sel.pe} of=${sel.overflow}`);

// ---- 3. selbar eye toggles the link's meshes -------------------------------
const meshState = () => page.evaluate(t => {
  const link = document.getElementById('viewer').robot.links[t];
  let vis = 0, hid = 0;
  for (const c of link.children) {
    if (!c.isURDFJoint && !c.userData.sw2robotMarker) { c.visible ? vis++ : hid++; }
  }
  return { vis, hid };
}, target);
await page.click('#seleye');
await sleep(400);
const hidden = await meshState();
check('eye: hides meshes', hidden.vis === 0 && hidden.hid > 0,
      JSON.stringify(hidden));
await page.click('#seleye');
await sleep(400);
const shown = await meshState();
check('eye: restores meshes', shown.hid === 0);
await page.keyboard.press('Escape');

// ---- 4. make-root: no blank frames, undo/redo round trip -------------------
await page.evaluate(() => {
  const v = document.getElementById('viewer');
  window.__samples = [];
  window.__t = setInterval(() => {
    let m = 0;
    v.world.traverse(c => { if (c.isMesh && c.visible) m++; });
    window.__samples.push(m);
  }, 40);
});
await page.evaluate(t => {
  [...document.querySelectorAll('.joint:not(.root)')]
    .find(r => r.querySelector('.jname')?.textContent === t)
    .querySelector('.mkroot').click();
}, target);
await sleep(9000);
const samples = await page.evaluate(() => {
  clearInterval(window.__t);
  return window.__samples;
});
const newBase = (await rootPose()).base;
check('make-root: base changed', newBase === target, `base=${newBase}`);
check('make-root: no blank frames',
      Math.min(...samples) >= samples[0] * 0.95,
      `min=${Math.min(...samples)} baseline=${samples[0]}`);
const rootRow = await page.evaluate(() =>
  document.querySelector('.joint.root .rootcomp')?.textContent);
check('make-root: tree shows new component', rootRow?.includes(target),
      rootRow);

await page.keyboard.down('Control');
await page.keyboard.press('z');
await page.keyboard.up('Control');
await sleep(9000);
check('undo: base restored', (await rootPose()).base === origBase);
await page.keyboard.down('Control');
await page.keyboard.press('y');
await page.keyboard.up('Control');
await sleep(9000);
check('redo: base re-applied', (await rootPose()).base === target);
await page.evaluate(() => fetch('/api/undo', { method: 'POST' }));
await sleep(7000);
check('cleanup: base back to original', (await rootPose()).base === origBase);

// ---- 5. root frame: X+90 button + numeric reset, no reload ----------------
const rpyBefore = (await rootPose()).rpy;
await page.click('#rootbox .rot');           // first rot button = X+90
await sleep(3000);
const rpyAfter = (await rootPose()).rpy;
check('root X+90: rpy changed',
      Math.abs(rpyAfter[0] - rpyBefore[0]) > 1.0
      || Math.abs(rpyAfter[1] - rpyBefore[1]) > 1.0
      || Math.abs(rpyAfter[2] - rpyBefore[2]) > 1.0,
      JSON.stringify(rpyAfter));
await page.evaluate(() => fetch('/api/undo', { method: 'POST' }));
await sleep(5000);
const rpyRestored = (await rootPose()).rpy;
check('root X+90 undo: rpy restored',
      rpyRestored.every((v, i) => Math.abs(v - rpyBefore[i]) < 1e-6),
      JSON.stringify(rpyRestored));

// ---- 6. 📁 server-side file browser ---------------------------------------
await page.click('#fsbrowse');
await sleep(1200);
const fs1 = await page.evaluate(() => ({
  modal: document.getElementById('fsmodal').style.display,
  rows: document.querySelectorAll('#fslist div').length,
  path: document.getElementById('fspath').textContent,
}));
check('fs: modal opens with entries', fs1.modal === 'flex' && fs1.rows > 0,
      `rows=${fs1.rows} at "${fs1.path}"`);
const descended = await page.evaluate(async () => {
  const dir = [...document.querySelectorAll('#fslist div')]
    .find(d => d.textContent.startsWith('📁'));
  if (!dir) { return null; }
  dir.click();
  await new Promise(r => setTimeout(r, 1000));
  return document.getElementById('fspath').textContent;
});
if (descended === null) {
  console.log('SKIP  fs: descend into a folder (no sub-folder under the browse root)');
} else {
  check('fs: descend into a folder', descended !== fs1.path, `-> "${descended}"`);
}
await page.click('#fsclose');
const fsClosed = await page.evaluate(
  () => document.getElementById('fsmodal').style.display);
check('fs: close hides modal', fsClosed === 'none');
check('fs: empty prompt hidden while robot loaded', await page.evaluate(
  () => document.getElementById('emptyprompt').style.display !== 'block'));

// ---- 8. self-collision: NEW contacts tint the offending links red ----------
// reload to a clean server-package state before checking collisions.
// domcontentloaded (not networkidle2): the mesh stream keeps the network
// busy for >60 s on big packages, so networkidle2 would time out.
await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await sleep(15000);
let colReady = false;
for (let i = 0; i < 90 && !colReady; i++) {
  colReady = await page.evaluate(() => window.sw2robot?.collision().ready === true);
  if (!colReady) { await sleep(2000); }
}
check('collision: model ready', colReady);
if (colReady) {
  // scan the movable joints for a pose with a NEW collision
  const found = await page.evaluate(async () => {
    const v = document.getElementById('viewer');
    const names = Object.values(v.robot.joints)
      .filter(j => j.jointType !== 'fixed' && !j.mimicJoint)
      .map(j => j.name);
    for (const jn of names) {
      for (const deg of [45, 90, -45, -90, 135, 180]) {
        const r = await (await fetch('/api/collision', { method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ angles: { [jn]: deg * Math.PI / 180 } }),
        })).json();
        if (r.pairs?.length) {
          return { joint: jn, rad: deg * Math.PI / 180, links: r.links };
        }
      }
    }
    return null;
  });
  if (!found) {
    console.log('SKIP  collision red-tint (no colliding pose found by scan)');
  } else {
    const firstMesh = `(function find(n) {
      for (const ch of n.children) {
        if (ch.isURDFJoint) { continue; }
        if (ch.isMesh && !ch.userData.sw2robotMarker) { return ch; }
        const m = find(ch); if (m) { return m; }
      }
      return null;
    })`;
    await page.evaluate(f => {
      const v = document.getElementById('viewer');
      v.setJointValue(f.joint, f.rad);
    }, found);
    await sleep(1500);
    const red = await page.evaluate((f, fm) => {
      const link = document.getElementById('viewer').robot.links[f.links[0]];
      const mesh = eval(fm)(link);
      return { links: window.sw2robot.collision().links,
               badge: document.getElementById('colstat').style.display,
               banner: document.getElementById('colstat').textContent,
               emissive: mesh?.material.emissive?.getHex() };
    }, found, firstMesh);
    check('collision: offending links turn red',
          red.links.length > 0 && red.badge === 'block'
          && red.emissive === 0x8a0e0e,
          `joint=${found.joint} links=${red.links.length} ` +
          `emissive=${(red.emissive ?? 0).toString(16)}`);
    // setJointValue fires angle-change just like a drag does, so the banner
    // must name the moved joint and the angle it collides at ("この角度で")
    check('collision: banner shows the joint angle',
          red.banner.includes(found.joint) && /[-\d.]+°/.test(red.banner),
          red.banner.slice(0, 80));
    await page.evaluate(f => {
      document.getElementById('viewer').setJointValue(f.joint, 0);
    }, found);
    await sleep(1500);
    const clear = await page.evaluate((f, fm) => {
      const link = document.getElementById('viewer').robot.links[f.links[0]];
      const mesh = eval(fm)(link);
      return { links: window.sw2robot.collision().links,
               badge: document.getElementById('colstat').style.display,
               emissive: mesh?.material.emissive?.getHex() };
    }, found, firstMesh);
    check('collision: tint clears back at rest',
          clear.links.length === 0 && clear.badge === 'none'
          && clear.emissive !== 0x8a0e0e,
          `links=${clear.links.length}`);
  }
}

// ---- 9. auto joint limits (self-collision sweep) ---------------------------
// reuses the collision model from section 8 (already reloaded, non-drop).
// The flow does an async rebuild+reload (~20s for 126 meshes), so POLL for
// the expected joint state rather than fixed sleeps -- and only undo AFTER
// the apply-reload has landed, or the two reloads race.
if (SMOKE) {
  console.log('SKIP  auto-limits + box-select (smoke mode: minimal fixture has no movable joints)');
}
if (!SMOKE) {
const countContinuous = () => page.evaluate(() =>
  Object.values(document.getElementById('viewer').robot?.joints ?? {})
    .filter(j => j.jointType === 'continuous').length);
if (colReady) {
  const contBefore = await countContinuous();
  await page.click('#autolimits');
  let contAfter = contBefore, failed = false;
  for (let i = 0; i < 120; i++) {           // up to ~4 min (sweep+rebuild+load)
    await sleep(2000);
    contAfter = await countContinuous();
    if (contAfter > contBefore) { break; }
    failed = await page.evaluate(() => [...document.querySelectorAll('#log div')]
      .some(d => /自動リミットに失敗/.test(d.textContent)));
    if (failed) { break; }
  }
  check('auto-limits: free joints became continuous',
        contAfter > contBefore && !failed,
        `continuous ${contBefore} -> ${contAfter}`);
  // restore the package (revert the limit write so the suite is idempotent)
  await page.evaluate(() => window.sw2robot.undo());
  let contRestored = contAfter;
  for (let i = 0; i < 90; i++) {
    await sleep(2000);
    contRestored = await countContinuous();
    if (contRestored === contBefore) { break; }
  }
  check('auto-limits: undo restores joint types',
        contRestored === contBefore, `back to ${contRestored}`);
}

// ---- 9b. Shift+drag box-select -> bulk joint-type change -------------------
// (before the mock sections, so /api/set_types is real)
const movable = () => page.evaluate(() =>
  Object.values(document.getElementById('viewer').robot?.joints ?? {})
    .filter(j => j.jointType !== 'fixed' && !j.mimicJoint).length);
const movBefore = await movable();
// a real Shift+drag rubber band across the viewer must NOT orbit the camera
const camBefore = await page.evaluate(() =>
  document.getElementById('viewer').camera.position.toArray());
const vr = await page.evaluate(() => {
  const r = document.getElementById('viewer').getBoundingClientRect();
  return { x: r.left, y: r.top, w: r.width, h: r.height };
});
const cx = vr.x + vr.w / 2, cy = vr.y + vr.h / 2;
await page.keyboard.down('Shift');
await page.mouse.move(cx - vr.w * 0.35, cy - vr.h * 0.35);
await page.mouse.down();
for (let i = 1; i <= 6; i++) {
  await page.mouse.move(cx - vr.w * 0.35 + i * vr.w * 0.11,
                        cy - vr.h * 0.35 + i * vr.h * 0.11);
  await sleep(30);
}
await page.mouse.up();
await page.keyboard.up('Shift');
await sleep(400);
const drag = await page.evaluate(() => ({
  selected: window.sw2robot.boxSelected().length,
  bulkbar: getComputedStyle(document.getElementById('bulkbar')).display }));
const camAfter = await page.evaluate(() =>
  document.getElementById('viewer').camera.position.toArray());
const orbited = camBefore.some((v, i) => Math.abs(v - camAfter[i]) > 1e-3);
check('box-select: Shift+drag selects links without orbiting',
      drag.selected >= 1 && drag.bulkbar === 'flex' && !orbited,
      `selected=${drag.selected} orbited=${orbited}`);

// bulk-set every link to fixed -> 0 movable, then undo restores
await page.evaluate(() => { window.sw2robot.boxSelect(); window.sw2robot.bulkType('fixed'); });
let movAfter = movBefore;
for (let i = 0; i < 40; i++) { await sleep(2000); movAfter = await movable(); if (movAfter < movBefore) break; }
check('box-select: bulk "fixed" applies to all selected',
      movAfter < movBefore, `movable ${movBefore} -> ${movAfter}`);
await page.evaluate(() => window.sw2robot.undo());
let movRestored = movAfter;
for (let i = 0; i < 40; i++) { await sleep(2000); movRestored = await movable(); if (movRestored === movBefore) break; }
check('box-select: undo restores joint types',
      movRestored === movBefore, `back to ${movRestored}`);
}  // end if (!SMOKE) -- auto-limits + box-select block

// ---- 10. extraction loading UI (mock /api/extract*, no SolidWorks) ---------
// B: determinate bar during mesh export; C: cold-start reassurance; seconds.
await page.evaluate(() => {
  window._phase = 'start';
  const of = window.fetch.bind(window);
  window.fetch = async (u, opts) => {
    const url = typeof u === 'string' ? u : u?.url;
    const J = o => new Response(JSON.stringify(o),
      { headers: { 'Content-Type': 'application/json' } });
    if (url && url.includes('/api/extract?')) return J({ started: true });
    if (url && url.includes('/api/extract/status')) {
      if (window._phase === 'start') return J({ running: true, error: null, log: [
        'starting SolidWorks (this can take a minute) ...',
        '... still waiting (10s in this phase, 20s total, 1 SolidWorks process(es) alive)'] });
      if (window._phase === 'mesh') return J({ running: true, error: null,
        log: ['starting SolidWorks ...', 'exporting mesh 30/55: linkB_1'] });
      return J({ running: false, error: null, log: ['done'], package: 'output/x' });
    }
    return of(u, opts);
  };
  window.sw2robot.extractFlow('C:/fake/thing.SLDASM');
});
const lbar = () => page.evaluate(() => {
  const lb = document.getElementById('loadbar');
  return { indet: lb.classList.contains('indet'),
    text: lb.querySelector('.lb-text').textContent,
    sub: lb.querySelector('.lb-sub').textContent,
    fill: parseInt(lb.querySelector('.lb-fill').style.width) || 0 };
});
await sleep(2500);
const cold = await lbar();
check('extract-ui: cold-start indeterminate + alive + seconds',
      cold.indet && /SolidWorks を起動中/.test(cold.text)
      && /稼働中 ✓/.test(cold.sub) && /\d+秒/.test(cold.text)
      && !/min/.test(cold.text + cold.sub), JSON.stringify(cold));
await page.evaluate(() => { window._phase = 'mesh'; });
await sleep(1600);
const meshb = await lbar();
check('extract-ui: mesh export determinate bar (30/55) + seconds',
      !meshb.indet && meshb.fill >= 50 && meshb.fill <= 60
      && /メッシュ 30\/55/.test(meshb.sub) && /\d+秒/.test(meshb.text),
      JSON.stringify(meshb));
await page.evaluate(() => { window._phase = 'done'; });
await sleep(1500);

// ---- 11. 🗄 server browser: recent files + filter + paste-a-path extract ----
await page.evaluate(() => {
  const of = window.fetch.bind(window);
  window.fetch = async (u, opts) => {
    const url = typeof u === 'string' ? u : u?.url;
    const J = o => new Response(JSON.stringify(o),
      { headers: { 'Content-Type': 'application/json' } });
    if (url && url.includes('/api/recent'))
      return J(['G:/proj/recent_arm.SLDASM', 'G:/proj/recent_gripper.SLDASM']);
    if (url && url.includes('/api/list'))
      return J([{ name: 'built_pkg', path: 'C:/out/built_pkg' }]);
    if (url && url.includes('/api/fs'))           // server browser root
      return J({ path: '', parent: null, dirs: [], files: [] });
    if (url && url.includes('/api/extract?')) return J({ started: true });
    if (url && url.includes('/api/extract/status'))
      return J({ running: true, error: null, log: ['starting SolidWorks ...'] });
    return of(u, opts);
  };
});
// 🗄 surfaces SolidWorks recent files (⭐) AND built packages (⭐📦) at the root
await page.evaluate(() => document.getElementById('fsbrowse').click());
await sleep(400);
const recent = await page.evaluate(() => {
  const rows = [...document.querySelectorAll('#fslist > div')];
  return {
    star: rows.some(d => /⭐/.test(d.textContent) && /recent_arm/i.test(d.textContent)),
    pkg: rows.some(d => /built_pkg/.test(d.textContent)),
  };
});
check('server-browser: lists recent files (⭐) and built packages',
      recent.star && recent.pkg, JSON.stringify(recent));
// filter narrows the listing to matching rows
await page.evaluate(() => {
  const f = document.getElementById('fsfilter');
  f.value = 'gripper'; f.dispatchEvent(new Event('input'));
});
await sleep(120);
const filt = await page.evaluate(() => {
  const vis = [...document.querySelectorAll('#fslist > div')]
    .filter(d => d.style.display !== 'none').map(d => d.textContent);
  return { n: vis.length, hasGripper: vis.some(t => /gripper/i.test(t)),
           hasArm: vis.some(t => /recent_arm/i.test(t)) };
});
check('server-browser: filter narrows to matching rows',
      filt.hasGripper && !filt.hasArm, JSON.stringify(filt));
// the path bar IS the paste-path: type a .sldasm path + Enter -> extraction
await page.evaluate(() => {
  const el = document.getElementById('fspath');
  el.value = '  "C:\\\\drop\\\\x.SLDASM"  ';     // quoted + padded, like "Copy as path"
  el.dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true }));
});
await sleep(1200);
const paste = await page.evaluate(() => ({
  text: document.getElementById('loadbar').querySelector('.lb-text').textContent,
}));
check('server-browser: pasting a .sldasm path into the bar starts extraction',
      /SolidWorks|抽出/.test(paste.text), JSON.stringify(paste));

check('suite: no page errors at end', pageErrors.length === 0,
      pageErrors.join(' | '));
await browser.close();
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
