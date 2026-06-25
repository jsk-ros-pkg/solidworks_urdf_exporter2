// Verify Shift+click accumulates link selection into the same set + bulkbar as
// a Shift+drag region box-select.
import puppeteer from 'puppeteer-core';
const URL = (process.argv[2] ?? 'http://localhost:8090') + '?lang=ja';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const sleep = ms => new Promise(r => setTimeout(r, ms));
let fail = 0;
const check = (n, ok, d = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${n}${d ? '  -- ' + d : ''}`); if (!ok) fail++; };

const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new', args: ['--disable-gpu'] });
const page = await browser.newPage();
const errs = []; page.on('pageerror', e => errs.push(e.message.split('\n')[0]));
await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(
  () => [...document.querySelectorAll('#log div')]
    .some(d => d.textContent.includes('カメラを調整しました')), { timeout: 60000 }).catch(() => {});
await sleep(1500);

const boxN = () => page.evaluate(() => window.sw2robot.boxSelected().length);
const bulkbarShown = () => page.evaluate(() =>
  getComputedStyle(document.getElementById('bulkbar')).display !== 'none');

// project each link's own-mesh centroid to screen coords (camera matrices),
// so we can fire real Shift+clicks at links.
const points = await page.evaluate(() => {
  const v = document.getElementById('viewer');
  const r = v.getBoundingClientRect();
  v.robot.updateMatrixWorld(true);
  // grab THREE from the module scope via a known object: viewer.camera is a
  // THREE.Camera; reuse its constructor's Vector3? Instead use project() helper.
  const cam = v.camera;
  const res = [];
  for (const [name, link] of Object.entries(v.robot.links)) {
    let cx = 0, cy = 0, cz = 0, k = 0;
    link.traverse(o => {
      if (o.isMesh && o.geometry) {
        o.geometry.computeBoundingBox?.();
        const bb = o.geometry.boundingBox; if (!bb) return;
        const c = { x: (bb.min.x + bb.max.x) / 2, y: (bb.min.y + bb.max.y) / 2,
                    z: (bb.min.z + bb.max.z) / 2 };
        o.updateMatrixWorld(true);
        const e = o.matrixWorld.elements;
        const wx = e[0]*c.x + e[4]*c.y + e[8]*c.z + e[12];
        const wy = e[1]*c.x + e[5]*c.y + e[9]*c.z + e[13];
        const wz = e[2]*c.x + e[6]*c.y + e[10]*c.z + e[14];
        cx += wx; cy += wy; cz += wz; k++;
      }
    });
    if (!k) continue;
    cx /= k; cy /= k; cz /= k;
    // project with camera matrices
    cam.updateMatrixWorld(true);
    const vp = cam.projectionMatrix.clone().multiply(cam.matrixWorldInverse);
    const m = vp.elements;
    const px = m[0]*cx + m[4]*cy + m[8]*cz + m[12];
    const py = m[1]*cx + m[5]*cy + m[9]*cz + m[13];
    const pw = m[3]*cx + m[7]*cy + m[11]*cz + m[15];
    if (pw <= 0) continue;
    const ndcx = px / pw, ndcy = py / pw;
    if (ndcx < -1 || ndcx > 1 || ndcy < -1 || ndcy > 1) continue;
    res.push({ name,
      x: r.left + (ndcx * 0.5 + 0.5) * r.width,
      y: r.top + (-ndcy * 0.5 + 0.5) * r.height });
  }
  return res;
});
check('projected at least 3 on-screen links', points.length >= 3,
  `got ${points.length}`);

// pick 3 spread-out points
points.sort((a, b) => a.x - b.x);
const probes = [points[0], points[Math.floor(points.length / 2)], points[points.length - 1]];

await page.keyboard.down('Shift');
const sizes = [];
for (const p of probes) {
  await page.mouse.move(p.x, p.y);
  await page.mouse.down(); await page.mouse.up();
  await sleep(120);
  sizes.push(await boxN());
}
check('shift+click accumulates selection (grows past 1)',
  sizes[sizes.length - 1] >= 2, `sizes=${JSON.stringify(sizes)}`);
check('bulkbar appears once links are shift-selected', await bulkbarShown(), '');

// re-click the LAST probe point -> toggles that link back off (count drops)
const beforeToggle = await boxN();
await page.mouse.move(probes[probes.length - 1].x, probes[probes.length - 1].y);
await page.mouse.down(); await page.mouse.up();
await sleep(150);
const afterToggle = await boxN();
check('shift+click again toggles a link OFF',
  afterToggle === beforeToggle - 1, `${beforeToggle} -> ${afterToggle}`);
await page.keyboard.up('Shift');

check('no page errors during shift-click selection', errs.length === 0, errs.join(' | '));
await browser.close();
console.log(`\n${fail === 0 ? 'ALL PASSED' : fail + ' FAILED'}`);
process.exit(fail === 0 ? 0 : 1);
