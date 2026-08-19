// Net for the bare-key shortcut handler registered on window by the web editor.
//
// The handler matches BARE ev.key and returns early only for a focused field,
// so with focus on <body> every browser/OS chord whose letter it knows fires an
// editor command as a side effect: Ctrl+C reset the camera, Ctrl+V re-centred
// the pan, Ctrl+F flipped a joint axis (a MODEL EDIT).  The sibling keydown
// handler on window (undo/redo) already guards on (ctrlKey || metaKey).
//
// Both directions are checked for every key, because a "fix" that simply kills
// the shortcut also makes the bug go away:
//   * bare <key>   STILL runs the command   (the feature must survive)
//   * Ctrl+<key>   does NOT run the command (the bug must be gone)
//
// Observables, all language-independent:
//   c -> resetView()            : viewer.camera.position
//   v -> recenterPan()          : viewer.controls.target
//   f -> flipTargetJointAxis()  : the POST to /api/set_axis, seen on the wire
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

const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new', args: ['--disable-gpu', ...EXTRA] });
const page = await browser.newPage();
const errs = [];
page.on('pageerror', e => errs.push(e.message.split('\n')[0]));
// 'f' mutates the served URDF through this POST -- watch the wire, so the check
// does not depend on the reload that follows finishing in time.
let axisPosts = 0;
page.on('request', r => {
  if (r.method() === 'POST' && r.url().includes('/api/set_axis')) { axisPosts++; }
});

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.sw2robot && window.viewer?.robot,
                           { timeout: 60000 });
await sleep(1500);

// The handler returns early for INPUT/SELECT/TEXTAREA/contentEditable.  With
// focus in a field we would be testing that early return, not the missing
// modifier guard -- so pin focus on <body> and assert it.
await page.evaluate(() => { document.activeElement?.blur?.(); });
const active = await page.evaluate(() => document.activeElement?.tagName ?? null);
check('setup: focus is on <body>, not a field', active === 'BODY', `activeElement=${active}`);

const cam = () => page.evaluate(() => window.viewer.camera.position.toArray());
const tgt = () => page.evaluate(() => window.viewer.controls.target.toArray());
const far = (a, b) => a.some((v, i) => Math.abs(v - b[i]) > 1e-3);
const bare = async k => { await page.keyboard.press(k); await sleep(1200); };
const withMod = async (mod, k) => {
  await page.keyboard.down(mod);
  await page.keyboard.press(k);
  await page.keyboard.up(mod);
  await sleep(1200);
};
const ctrl = k => withMod('Control', k);
const shift = k => withMod('Shift', k);

// ---- c: resetView() -------------------------------------------------------
await bare('c');                       // land on the reset ("home") framing
const camHome = await cam();
const nudge = () => page.evaluate(() => {
  window.viewer.camera.position.x += 0.05;
  window.viewer.camera.position.y += 0.05;
  window.viewer.camera.position.z += 0.05;
  window.viewer.controls.update();
  window.viewer.redraw();
});
await nudge();
const camMoved = await cam();
check('setup: nudging the camera is observable', far(camHome, camMoved),
      JSON.stringify(camMoved.map(v => +v.toFixed(4))));

await ctrl('c');                       // Ctrl+C is COPY, not "reset the view"
const camAfterCtrl = await cam();
check('Ctrl+C does not reset the view', far(camAfterCtrl, camHome),
      `cam=${JSON.stringify(camAfterCtrl.map(v => +v.toFixed(4)))} `
      + `home=${JSON.stringify(camHome.map(v => +v.toFixed(4)))}`);

await bare('c');                       // ...but the bare shortcut still works
const camAfterBare = await cam();
check('bare c still resets the view', !far(camAfterBare, camHome),
      JSON.stringify(camAfterBare.map(v => +v.toFixed(4))));

// Shift is deliberately NOT part of the guard: the handler matches uppercase on
// purpose ('t' || 'T', 'c' || 'C'), so Shift+<letter> is real input, and
// Shift+drag is box-select.  Pin that down, or "guard shiftKey too" looks free.
await nudge();
const camShiftMoved = await cam();
await shift('c');
const camAfterShift = await cam();
check('Shift+C still resets the view', far(camShiftMoved, camAfterShift)
      && !far(camAfterShift, camHome),
      JSON.stringify(camAfterShift.map(v => +v.toFixed(4))));

// ---- v: recenterPan() -----------------------------------------------------
await bare('v');                       // land on the re-centred orbit target
const tgtHome = await tgt();
const pan = () => page.evaluate(() => {
  window.viewer.controls.target.x += 0.02;
  window.viewer.controls.target.y += 0.02;
  window.viewer.controls.target.z += 0.02;
  window.viewer.controls.update();
  window.viewer.redraw();
});
await pan();
const tgtMoved = await tgt();
check('setup: panning the orbit target is observable', far(tgtHome, tgtMoved),
      JSON.stringify(tgtMoved.map(v => +v.toFixed(4))));

await ctrl('v');                       // Ctrl+V is PASTE, not "re-centre"
const tgtAfterCtrl = await tgt();
check('Ctrl+V does not recentre the orbit pan', far(tgtAfterCtrl, tgtHome),
      `tgt=${JSON.stringify(tgtAfterCtrl.map(v => +v.toFixed(4)))} `
      + `home=${JSON.stringify(tgtHome.map(v => +v.toFixed(4)))}`);

await bare('v');
const tgtAfterBare = await tgt();
check('bare v still recentres the orbit pan', !far(tgtAfterBare, tgtHome),
      JSON.stringify(tgtAfterBare.map(v => +v.toFixed(4))));

// ---- Escape is exempt from the guard, on purpose --------------------------
// Every Escape branch in the handler only cancels/dismisses, and the overlay
// branch at the top of the handler already runs Escape under any modifier, so
// Ctrl+Escape keeps meaning "cancel" rather than becoming a no-op.
const firstLink = await page.evaluate(() => {
  const n = Object.keys(window.viewer.robot.links)[0];
  window.sw2robot.select(n);
  return n;
});
await sleep(600);
await page.evaluate(() => { document.activeElement?.blur?.(); });
const selBefore = await page.evaluate(() => window.sw2robot.dump().selected);
check('setup: a link is selected before the Escape check', selBefore === firstLink,
      `selected=${selBefore}`);
await ctrl('Escape');
const selAfter = await page.evaluate(() => window.sw2robot.dump().selected);
check('Ctrl+Escape still clears the selection (Escape is exempt)',
      selAfter === null, `selected=${selAfter}`);

// ---- f: flipTargetJointAxis() -- the one that EDITS THE MODEL -------------
const plan = await page.evaluate(() => {
  const j = Object.values(window.viewer.robot.joints)
    .find(x => x.jointType !== 'fixed' && !x.mimicJoint);
  const child = j && Object.values(window.viewer.robot.links)
    .find(l => l.parent === j);
  return { joint: j?.name ?? null, link: child?.name ?? null };
});
check('setup: a movable joint and its child link exist',
      !!plan.joint && !!plan.link, JSON.stringify(plan));
await page.evaluate(l => window.sw2robot.select(l), plan.link);
await sleep(600);
await page.evaluate(() => { document.activeElement?.blur?.(); });

axisPosts = 0;
await ctrl('f');                       // Ctrl+F is FIND, not "flip the axis"
check('Ctrl+F does not flip the joint axis', axisPosts === 0,
      `POST /api/set_axis x${axisPosts}`);

await bare('f');
await sleep(1500);
check('bare f still flips the joint axis', axisPosts === 1,
      `POST /api/set_axis x${axisPosts}`);

// /api/set_axis is self-inverse, so an EVEN number of flips is already back
// where we started: only an odd count needs one more to restore the fixture for
// a re-run (and for any script that follows this one).
if (axisPosts % 2 === 1) {
  await page.evaluate(j => fetch('/api/set_axis', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ joints: [j] }) }).then(r => r.json()), plan.joint);
  await sleep(800);
}

check('no page errors', errs.length === 0, errs.join(' | '));

await browser.close();
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
