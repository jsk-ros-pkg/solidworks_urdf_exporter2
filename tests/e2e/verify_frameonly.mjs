// Browser E2E for the per-link "export shape + weight" switch (frame_only):
// the checkbox on every joint-tree row AND the one in the top-left link panel.
// Unchecking it must leave the link (and its joint) in the tree while the URDF
// gets a bare frame -- no visual, no collision, no inertial.
//
// Prereqs (needs a CAD package, i.e. one with a graph.json):
//   1. build once:  cp -r examples/fingertip <pkg> && \
//                   uv run python -m sw2robot.exporter.build <pkg>
//   2. server:      uv run python -m sw2robot.editor.webserver <pkg> --port 8092
//   3. once:        cd tests/e2e && npm i
// Run:
//   node tests/e2e/verify_frameonly.mjs [url]
//
// Restores the link it toggles before exiting.
import puppeteer from 'puppeteer-core';

const BASE = process.argv[2] ?? 'http://127.0.0.1:8092';
// force Japanese: the panel assertions below check localized strings
const URL = BASE + (BASE.includes('?') ? '&' : '?') + 'lang=ja';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const sleep = ms => new Promise(r => setTimeout(r, ms));
let fails = 0;
const check = (n, ok, d = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${n}${d ? '  -- ' + d : ''}`);
  if (!ok) { fails += 1; }
};

const EXTRA_ARGS = (process.env.CHROME_ARGS ?? '')
  .split(',').map(s => s.trim()).filter(Boolean);
const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new',
  args: ['--no-sandbox', ...EXTRA_ARGS] });
const page = await browser.newPage();
await page.setViewport({ width: 1500, height: 900 });
page.on('pageerror', e => { console.log('PAGEERROR', e.message); fails += 1; });
await page.goto(URL, { waitUntil: 'networkidle2', timeout: 60000 });
await page.waitForSelector('.joint .geo', { timeout: 30000 });

// state of one link's row in the joint tree
const rowState = link => page.evaluate(t => {
  const e = [...document.querySelectorAll('.joint')]
    .find(x => x.querySelector('.jname')?.textContent === t);
  return e && { checked: e.querySelector('.geo')?.checked,
                tag: !!e.querySelector('.frameonlytag'),
                eyeDisabled: e.querySelector('.eye')?.disabled };
}, link);
// state of the top-left link panel (open it by clicking the link name first)
const panelState = () => page.evaluate(() => {
  const cb = document.querySelector('#linkinfo #li_geo');
  const li = document.getElementById('linkinfo');
  return { present: !!cb, checked: cb?.checked,
           label: li.innerText.includes('形状・質量'),
           note: /座標フレームのみで出力/.test(li.innerText) };
});
const linkXml = async link => {
  const urdf = await page.evaluate(async () => {
    const i = await (await fetch('/api/info')).json();
    return await (await fetch(i.urdf)).text();
  });
  return urdf.split(/<link /).find(s => s.startsWith(`name="${link}"`)) ?? '';
};
const waitFor = async (fn, ms = 25000) => {
  for (let i = 0; i * 500 < ms; i++) {
    if (await fn()) { return true; }
    await sleep(500);
  }
  return false;
};

// --- every row carries the switch, and the root header row too --------------
const rows = await page.$$eval('.joint', els => els.map(e => ({
  link: e.querySelector('.jname')?.textContent,
  root: e.classList.contains('root'),
  hasGeo: !!e.querySelector('.geo'),
  checked: e.querySelector('.geo')?.checked })));
check('every link row has the switch', rows.length > 0 && rows.every(r => r.hasGeo),
      JSON.stringify(rows.filter(r => !r.hasGeo)));
check('the root row has it too', rows.some(r => r.root && r.hasGeo));
check('they start checked (shape + weight exported)',
      rows.every(r => r.checked === true));

// --- toggling from a joint row ----------------------------------------------
const TARGET = (rows.find(r => !r.root) ?? rows[0]).link;
await page.evaluate(t => [...document.querySelectorAll('.joint')]
  .find(x => x.querySelector('.jname')?.textContent === t)
  .querySelector('.geo').click(), TARGET);
const off = await waitFor(async () => {
  const s = await rowState(TARGET);
  return s && s.checked === false && s.tag;
});
check(`${TARGET}: row switched off`, off, JSON.stringify(await rowState(TARGET)));
check(`${TARGET}: eye button disabled (nothing to show)`,
      (await rowState(TARGET))?.eyeDisabled === true);
const xmlOff = await linkXml(TARGET);
check('served URDF: bare frame (no visual/collision/inertial)',
      xmlOff !== '' && !/<visual|<collision|<inertial/.test(xmlOff),
      xmlOff.slice(0, 80));

// --- the top-left link panel shows the SAME switch, in sync -----------------
await page.evaluate(t => [...document.querySelectorAll('.joint .jname')]
  .find(e => e.textContent === t).click(), TARGET);
await page.waitForSelector('#linkinfo #li_geo', { timeout: 15000 });
let p = await panelState();
check('link panel shows the switch, localized', p.present && p.label);
check('link panel agrees it is off', p.checked === false);
check('link panel says what is exported instead', p.note === true);

// --- turning it back on from the PANEL restores everything ------------------
await page.evaluate(() => document.querySelector('#linkinfo #li_geo').click());
const backOn = await waitFor(async () => {
  const s = await rowState(TARGET);
  return s && s.checked === true && !s.tag;
});
check(`${TARGET}: restored from the panel`, backOn);
const xmlOn = await linkXml(TARGET);
check('served URDF: geometry and inertial are back',
      /<visual/.test(xmlOn) && /<inertial/.test(xmlOn), xmlOn.slice(0, 80));

await browser.close();
console.log(fails ? `\n${fails} FAILURE(S)` : '\nall checks passed');
process.exit(fails ? 1 : 0);
