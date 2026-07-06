// Focused check for step-2 joint-edit recording (drives the real slider DOM):
//  - a user joint move emits exactly ONE op('setJoint') per settle (debounced)
//  - the settled value is carried
//  - the screencast overlay shows a "Joint … → …" keycap
//  - reset-pose logs ONE op('resetPose'), not N stray setJoint ops
//  - no page errors throughout
import puppeteer from 'puppeteer-core';

const BASE = process.argv[2] ?? 'http://localhost:8090';
const URL = BASE + (BASE.includes('?') ? '&' : '?') + 'lang=ja';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const EXTRA = (process.env.CHROME_ARGS ?? '').split(',').map(s => s.trim()).filter(Boolean);
const sleep = ms => new Promise(r => setTimeout(r, ms));
let failures = 0;
const check = (n, ok, d = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${n}${d ? '  -- ' + d : ''}`);
  if (!ok) failures++;
};
const countOp = (page, name) => page.evaluate(
  n => window.sw2robot.dump().oplog.filter(o => o.op === n).length, name);

const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new', args: ['--disable-gpu', ...EXTRA] });
const page = await browser.newPage();
const errs = [];
page.on('pageerror', e => errs.push(e.message.split('\n')[0]));

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.sw2robot, { timeout: 60000 });
await sleep(1500);

// screencast overlay ON (exercise the op -> overlay path too)
await page.click('#keycast');
// move mode builds a slider per movable joint
await page.click('#playmode');
await sleep(300);
const hasSlider = await page.evaluate(
  () => !!document.querySelector('#playrows .prow input[type=range]'));
check('setup: a movable-joint slider exists', hasSlider);

// simulate a drag: many rapid 'input' events, then let it settle past debounce
const before = await countOp(page, 'setJoint');
await page.evaluate(() => {
  const s = document.querySelector('#playrows .prow input[type=range]');
  const hi = parseFloat(s.max), lo = parseFloat(s.min);
  const target = lo + (hi - lo) * 0.7;
  for (let i = 1; i <= 20; i++) {
    s.value = String(lo + (target - lo) * i / 20);
    s.dispatchEvent(new Event('input', { bubbles: true }));
  }
  // the range input snaps to its step, so read the value it actually settled on
  window.__settled = parseFloat(s.value);
});
await sleep(600);   // > 350 ms debounce

const r = await page.evaluate(before1 => {
  const sj = window.sw2robot.dump().oplog.filter(o => o.op === 'setJoint');
  const caps = [...document.querySelectorAll('#keyoverlay .keycap')]
    .map(c => c.firstChild.textContent);
  return { delta: sj.length - before1, last: sj[sj.length - 1],
           settled: window.__settled, caps };
}, before);
check('drag: exactly ONE setJoint op (debounced)', r.delta === 1, `delta=${r.delta}`);
check('drag: settled value carried',
      r.last && Math.abs(r.last.value - r.settled) < 1e-4,
      `value=${r.last?.value} settled=${r.settled}`);
check('drag: overlay shows a Joint keycap',
      r.caps.some(c => /Joint .*→/.test(c)), JSON.stringify(r.caps));

// reset-pose = ONE op('resetPose'), no stray programmatic setJoint ops
const bReset = await countOp(page, 'resetPose');
const bSet = await countOp(page, 'setJoint');
await page.evaluate(() => document.getElementById('reset').click());
await sleep(600);
const aReset = await countOp(page, 'resetPose');
const aSet = await countOp(page, 'setJoint');
check('reset: one resetPose op', aReset - bReset === 1, `delta=${aReset - bReset}`);
check('reset: no stray setJoint from the programmatic move',
      aSet - bSet === 0, `delta=${aSet - bSet}`);

check('no page errors', errs.length === 0, errs.join(' | '));

await browser.close();
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
