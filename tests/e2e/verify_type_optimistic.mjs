// Focused verification of the OPTIMISTIC joint-type change (no full reload).
// Drives the real sidebar <select> and asserts the live model + UI reflect the
// flip synchronously, that the 3D robot object is NOT reloaded, and that the
// change still persists to joints.yaml in the background.
import puppeteer from 'puppeteer-core';

const URL = (process.argv[2] ?? 'http://localhost:8090') + '?lang=ja';
const YAML = process.argv[3];   // path to joints.yaml on disk to verify persist
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const sleep = ms => new Promise(r => setTimeout(r, ms));
let fail = 0;
const check = (n, ok, d = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${n}${d ? '  -- ' + d : ''}`);
  if (!ok) fail++;
};

const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new', args: ['--disable-gpu'] });
const page = await browser.newPage();
const errs = [];
page.on('pageerror', e => errs.push(e.message.split('\n')[0]));

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(
  () => [...document.querySelectorAll('#log div')]
    .some(d => d.textContent.includes('カメラを調整しました')),
  { timeout: 60000 }).catch(() => {});
await sleep(1500);

// helper run in the page: flip a joint via its sidebar <select> and report the
// SYNCHRONOUS post-dispatch state (proves "instant, no reload").
const flip = (kind, toType) => page.evaluate((kind, toType) => {
  const v = document.getElementById('viewer');
  const joints = Object.values(v.robot.joints);
  const childOf = j => [...(j.urdfNode?.children ?? [])]
    .find(e => e.tagName === 'child')?.getAttribute('link') ?? '';
  const want = kind === 'movable'
    ? joints.find(j => j.jointType !== 'fixed' && !j.mimicJoint)
    : joints.find(j => j.jointType === 'fixed');
  if (!want) { return { err: 'no ' + kind + ' joint' }; }
  const name = want.name, child = childOf(want);
  // find the row <select> whose row names this child link
  let sel = null;
  for (const row of document.querySelectorAll('.joint')) {
    const jn = row.querySelector('.jname');
    if (jn && jn.textContent.trim() === child) {
      sel = row.querySelector('.jtypesel'); break;
    }
  }
  if (!sel) { return { err: 'no row select for ' + child }; }
  const robotBefore = v.robot;
  const movBefore = joints.filter(j => j.jointType !== 'fixed').length;
  sel.value = toType;
  sel.dispatchEvent(new Event('change', { bubbles: true }));
  // read state SYNCHRONOUSLY right after the handler returned
  const j2 = v.robot.joints[name];
  const movAfter = Object.values(v.robot.joints)
    .filter(j => j.jointType !== 'fixed').length;
  return {
    name, child, toType,
    newType: j2.jointType,
    sameRobot: v.robot === robotBefore,   // false => a full reload happened
    movBefore, movAfter,
    axis: j2.axis ? j2.axis.toArray() : null,
  };
}, kind, toType);

// 1. movable -> fixed, instantly + no reload
const a = await flip('movable', 'fixed');
check('movable->fixed: type flips synchronously',
  a.newType === 'fixed', JSON.stringify(a));
check('movable->fixed: NO full robot reload (same object)',
  a.sameRobot === true, `sameRobot=${a.sameRobot}`);
check('movable->fixed: movable count drops instantly',
  a.movAfter === a.movBefore - 1, `${a.movBefore}->${a.movAfter}`);
const lockedName = a.name, lockedChild = a.child;

// 2. fixed -> movable (revolute), instantly with a usable axis
const b = await flip('fixed', 'revolute');
check('fixed->revolute: type flips synchronously',
  b.newType === 'revolute', JSON.stringify(b));
check('fixed->revolute: NO full robot reload (same object)',
  b.sameRobot === true, `sameRobot=${b.sameRobot}`);
const axLen = b.axis ? Math.hypot(...b.axis) : 0;
check('fixed->revolute: joint has a unit-length axis',
  b.axis && axLen > 0.9 && axLen < 1.1, `axis=${JSON.stringify(b.axis)} |a|=${axLen.toFixed(3)}`);

// 3. background persist reaches joints.yaml (poll the file)
let persisted = false;
if (YAML) {
  const fs = await import('node:fs');
  for (let i = 0; i < 40; i++) {
    await sleep(1000);
    const txt = fs.readFileSync(YAML, 'utf-8');
    // the joint we locked (child=lockedChild) should now read type: fixed,
    // and the joint we unfixed (child=b.child) should read type: revolute
    const lockOk = new RegExp('child:\\s*' + lockedChild +
      '\\s*\\n\\s*type:\\s*fixed').test(txt);
    const moveOk = new RegExp('child:\\s*' + b.child +
      '\\s*\\n\\s*type:\\s*revolute').test(txt);
    if (lockOk && moveOk) { persisted = true; break; }
  }
  check('background persist: joints.yaml shows both new types', persisted,
    `lockedChild=${lockedChild} unfixedChild=${b.child}`);
}

// 4. the page never threw
check('no page errors during type flips', errs.length === 0, errs.join(' | '));

await browser.close();
console.log(`\n${fail === 0 ? 'ALL PASSED' : fail + ' FAILED'}`);
process.exit(fail === 0 ? 0 : 1);
