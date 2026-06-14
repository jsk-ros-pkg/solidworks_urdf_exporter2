// cad2rc web editor E2E suite (puppeteer-core + system Chrome).
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

const URL = process.argv[2] ?? 'http://localhost:8090';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';

let failures = 0;
const check = (name, ok, detail = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`);
  if (!ok) { failures += 1; }
};
const sleep = ms => new Promise(r => setTimeout(r, ms));

const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new', args: ['--disable-gpu'] });
const page = await browser.newPage();
const pageErrors = [];
page.on('pageerror', e => pageErrors.push(e.message.split('\n')[0]));

// ---- 1. load --------------------------------------------------------------
await page.goto(URL, { waitUntil: 'networkidle2', timeout: 60000 });
await sleep(9000);
const logText = await page.evaluate(() =>
  [...document.querySelectorAll('#log div')].map(d => d.textContent).join('\n'));
check('load: camera fitted', logText.includes('camera fitted'));
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
           text: li.textContent.length };
});
check('select: selbar visible', sel.bar);
check('select: info card rendered top-left', sel.shown && sel.x < 50 && sel.y < 200,
      `rect=(${sel.x},${sel.y}) chars=${sel.text}`);

// ---- 3. selbar eye toggles the link's meshes -------------------------------
const meshState = () => page.evaluate(t => {
  const link = document.getElementById('viewer').robot.links[t];
  let vis = 0, hid = 0;
  for (const c of link.children) {
    if (!c.isURDFJoint && !c.userData.cad2rcMarker) { c.visible ? vis++ : hid++; }
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
check('fs: descend into a folder', !!descended && descended !== fs1.path,
      `-> "${descended}"`);
await page.click('#fsclose');
const fsClosed = await page.evaluate(
  () => document.getElementById('fsmodal').style.display);
check('fs: close hides modal', fsClosed === 'none');
check('fs: empty prompt hidden while robot loaded', await page.evaluate(
  () => document.getElementById('emptyprompt').style.display !== 'block'));

// ---- 7. OS-dialog open buttons (hidden file inputs, robot-compiler style) --
// uploadFile / FileChooser.accept = exactly what the native dialog does.
const pkgName = await page.evaluate(async () =>
  (await (await fetch('/api/info')).json()).name);
const pkgDir = path.resolve(
  path.dirname(fileURLToPath(import.meta.url)), '../../output', pkgName);
const pkgUrdf = path.join(pkgDir, 'urdf', pkgName + '.urdf');
if (fs.existsSync(pkgUrdf)) {
  const logLen = () => page.evaluate(
    () => document.querySelectorAll('#log div').length);
  const logAfter = async n => page.evaluate(i =>
    [...document.querySelectorAll('#log div')].slice(i)
      .map(d => d.textContent).join('\n'), n);

  let mark = await logLen();
  await (await page.$('#openfileinput')).uploadFile(pkgUrdf);
  await sleep(4000);
  const fileLog = await logAfter(mark);
  check('open-file: picked .urdf reaches the drop pipeline',
        /dropped 1 files/.test(fileLog) && fileLog.includes('URDF parsed'),
        fileLog.split('\n')[0]);

  mark = await logLen();
  const [chooser] = await Promise.all([
    page.waitForFileChooser({ timeout: 10000 }),
    page.click('#openfolder'),
  ]);
  await chooser.accept([pkgDir]);     // a DIRECTORY (webkitdirectory input)
  await sleep(30000);
  const dirLog = await logAfter(mark);
  check('open-folder: package folder loads fully',
        /dropped \d+ files/.test(dirLog) && dirLog.includes('camera fitted'),
        (dirLog.match(/dropped \d+ files/) ?? ['?'])[0]);
} else {
  console.log(`SKIP  open-file/open-folder (no local ${pkgUrdf})`);
}

// ---- 8. self-collision: NEW contacts tint the offending links red ----------
// section 7 left a DROPPED robot loaded (collision is disabled for those);
// reload to get the server package back in the normal (non-drop) path.
// domcontentloaded (not networkidle2): the mesh stream keeps the network
// busy for >60 s on big packages, so networkidle2 would time out.
await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await sleep(15000);
let colReady = false;
for (let i = 0; i < 90 && !colReady; i++) {
  colReady = await page.evaluate(() => window.cad2rc?.collision().ready === true);
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
        if (ch.isMesh && !ch.userData.cad2rcMarker) { return ch; }
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
      return { links: window.cad2rc.collision().links,
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
      return { links: window.cad2rc.collision().links,
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
      .some(d => /auto-limits failed/.test(d.textContent)));
    if (failed) { break; }
  }
  check('auto-limits: free joints became continuous',
        contAfter > contBefore && !failed,
        `continuous ${contBefore} -> ${contAfter}`);
  // restore the package (revert the limit write so the suite is idempotent)
  await page.evaluate(() => window.cad2rc.undo());
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
  selected: window.cad2rc.boxSelected().length,
  bulkbar: getComputedStyle(document.getElementById('bulkbar')).display }));
const camAfter = await page.evaluate(() =>
  document.getElementById('viewer').camera.position.toArray());
const orbited = camBefore.some((v, i) => Math.abs(v - camAfter[i]) > 1e-3);
check('box-select: Shift+drag selects links without orbiting',
      drag.selected >= 1 && drag.bulkbar === 'flex' && !orbited,
      `selected=${drag.selected} orbited=${orbited}`);

// bulk-set every link to fixed -> 0 movable, then undo restores
await page.evaluate(() => { window.cad2rc.boxSelect(); window.cad2rc.bulkType('fixed'); });
let movAfter = movBefore;
for (let i = 0; i < 40; i++) { await sleep(2000); movAfter = await movable(); if (movAfter < movBefore) break; }
check('box-select: bulk "fixed" applies to all selected',
      movAfter < movBefore, `movable ${movBefore} -> ${movAfter}`);
await page.evaluate(() => window.cad2rc.undo());
let movRestored = movAfter;
for (let i = 0; i < 40; i++) { await sleep(2000); movRestored = await movable(); if (movRestored === movBefore) break; }
check('box-select: undo restores joint types',
      movRestored === movBefore, `back to ${movRestored}`);

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
  window.cad2rc.extractFlow('C:/fake/thing.SLDASM');
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

// ---- 11. drop-a-.sldasm UX: dim backdrop + spinner + candidate picker ------
await page.evaluate(() => {
  const of = window.fetch.bind(window);
  window.fetch = async (u, opts) => {
    const url = typeof u === 'string' ? u : u?.url;
    const J = o => new Response(JSON.stringify(o),
      { headers: { 'Content-Type': 'application/json' } });
    if (url && url.includes('/api/locate')) {
      await new Promise(r => setTimeout(r, 700));
      return J({ building: false, hits: [
        { path: 'G:/a/x.SLDASM', size: 1, size_match: false },
        { path: 'G:/b/x.SLDASM', size: 2, size_match: false }] });
    }
    if (url && url.includes('/api/extract?')) return J({ started: true });
    if (url && url.includes('/api/extract/status'))
      return J({ running: true, error: null, log: ['starting SolidWorks ...'] });
    return of(u, opts);
  };
  window.cad2rc.locateAndExtract('C:/drop/x.SLDASM', 12345);
});
await sleep(300);
const loc = await page.evaluate(() => ({
  backdrop: getComputedStyle(document.getElementById('loadbackdrop')).display,
  text: document.getElementById('loadbar').querySelector('.lb-text').textContent,
}));
check('drop-sldasm: dim + spinner during locate',
      loc.backdrop === 'block' && /検索中/.test(loc.text), JSON.stringify(loc));
await sleep(700);
const pick = await page.evaluate(() => {
  const ov = [...document.querySelectorAll('#viewwrap > div')].find(d =>
    d.style.zIndex === '9' && /複数見つかりました/.test(d.textContent));
  return { shown: !!ov,
    backdrop: getComputedStyle(document.getElementById('loadbackdrop')).display,
    btns: ov ? ov.querySelectorAll('button').length : 0 };
});
check('drop-sldasm: prominent candidate picker (dim stays)',
      pick.shown && pick.btns >= 3 && pick.backdrop === 'block',
      JSON.stringify(pick));
await page.evaluate(() => {
  [...document.querySelectorAll('#viewwrap > div')].find(d =>
    d.style.zIndex === '9' && /複数見つかりました/.test(d.textContent))
    .querySelector('button').click();
});
await sleep(1500);
const af = await page.evaluate(() => ({
  gone: ![...document.querySelectorAll('#viewwrap > div')].some(d =>
    d.style.zIndex === '9' && /複数見つかりました/.test(d.textContent)),
  text: document.getElementById('loadbar').querySelector('.lb-text').textContent,
}));
check('drop-sldasm: picking a candidate starts extraction',
      af.gone && /SolidWorks|抽出/.test(af.text), JSON.stringify(af));

check('suite: no page errors at end', pageErrors.length === 0,
      pageErrors.join(' | '));
await browser.close();
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
