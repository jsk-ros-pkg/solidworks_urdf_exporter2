// Step-2 check: replay re-performs a recorded op() stream on the viewer.
//  - select + setJoint + resetPose dispatch and actually change viewer state
//  - replay does NOT append the re-performed actions back into oplog
//  - an unknown/destructive op is skipped, not fatal
//  - no page errors
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

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.sw2robot, { timeout: 60000 });
await sleep(1500);

// pick a link (not the current selection) and a movable joint + target angle
const plan = await page.evaluate(() => {
  const links = Object.keys(window.viewer.robot.links);
  const j = Object.values(window.viewer.robot.joints)
    .find(x => x.jointType !== 'fixed' && !x.mimicJoint);
  return { link: links[links.length - 1], joint: j?.name ?? null };
});
check('setup: have a link and a movable joint', !!plan.link && !!plan.joint,
      JSON.stringify(plan));

// craft a recorded stream with realistic timestamps + one unknown op.
// setJoint now EASES to its value over ~500ms, so leave a wide gap before
// resetPose and sample on the plateau after the ease completes.
const entries = [
  { t: 0,    op: 'select',    link: plan.link },
  { t: 60,   op: 'camera',    pos: [1.25, 2.5, 3.75], tgt: [0, 0, 0] },
  { t: 120,  op: 'setJoint',  joint: plan.joint, value: 0.3 },
  { t: 300,  op: 'rename',    kind: 'link', old: 'x', new: 'y' },  // destructive -> skip
  { t: 1400, op: 'resetPose', n: 1 },
];

// snapshot the log length; replay must not grow it
const before = await page.evaluate(() => window.sw2robot.oplog().length);

// replay at real speed; sample on the setJoint plateau (~800ms: after the
// 120ms start + 500ms ease, before resetPose at 1400ms)
// NB: block body so evaluate does not auto-await the replay promise
await page.evaluate(e => { window.__rp = window.sw2robot.replay(e, { speed: 1 }); }, entries);
await sleep(820);
const mid = await page.evaluate(j => ({
  selected: window.sw2robot.dump().selected,
  angle: Number(window.viewer.robot.joints[j].angle),
  cam: window.viewer.camera.position.toArray(),
  cursor: getComputedStyle(document.getElementById('replaycursor')).display,
}), plan.joint);
check('replay: select was dispatched', mid.selected === plan.link,
      `selected=${mid.selected}`);
check('replay: setJoint eased the joint to 0.3', Math.abs(mid.angle - 0.3) < 1e-3,
      `angle=${mid.angle}`);
check('replay: camera moved to the keyframe',
      Math.hypot(mid.cam[0] - 1.25, mid.cam[1] - 2.5, mid.cam[2] - 3.75) < 1e-3,
      `cam=${mid.cam.map(n => n.toFixed(2))}`);
check('replay: synthetic cursor is visible while replaying', mid.cursor === 'block',
      `display=${mid.cursor}`);

await page.evaluate(() => window.__rp);   // wait for the run to finish
await sleep(200);
const after = await page.evaluate(() => ({
  len: window.sw2robot.oplog().length,
  angle: Number(window.viewer.robot.joints[
    Object.values(window.viewer.robot.joints)
      .find(x => x.jointType !== 'fixed' && !x.mimicJoint).name].angle),
}));
check('replay: did NOT append to oplog', after.len === before,
      `before=${before} after=${after.len}`);
check('replay: resetPose ran last (joint back to 0)', Math.abs(after.angle) < 1e-6,
      `angle=${after.angle}`);
check('replay: synthetic cursor hidden after replay ends',
      (await page.evaluate(() => getComputedStyle(
        document.getElementById('replaycursor')).display)) === 'none');

// ---- camera RECORDING: orbiting the view populates a camera track that
// fullTimeline() interleaves with the actions (throttled ~10/s) ---------------
const camRec = await page.evaluate(async () => {
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const before = window.sw2robot.timeline().filter(e => e.op === 'camera').length;
  for (let i = 1; i <= 4; i++) {                 // move + fire 'change', spaced past throttle
    window.viewer.camera.position.set(i, i, i);
    window.viewer.controls.dispatchEvent({ type: 'change' });
    await sleep(130);
  }
  const tl = window.sw2robot.timeline();
  return { added: tl.filter(e => e.op === 'camera').length - before,
           ordered: tl.every((e, i) => i === 0 || (e.t ?? 0) >= (tl[i - 1].t ?? 0)),
           sample: tl.find(e => e.op === 'camera') };
});
check('camera-record: orbiting adds camera keyframes', camRec.added >= 3,
      `added=${camRec.added}`);
check('camera-record: keyframe has pos+tgt',
      Array.isArray(camRec.sample?.pos) && Array.isArray(camRec.sample?.tgt));
check('camera-record: timeline stays time-ordered', camRec.ordered);

// ---- fix #1 + #1b: a replay ENDING on a setJoint reaches its target, AND the
// promise only resolves once the ease has settled -- so a caller that stops a
// recording on `await replayLog()` captures the full glide (no truncation) ----
await page.evaluate(j => { window.viewer.setJointValue(j, 0); }, plan.joint);
const atResolve = await page.evaluate(async j => {
  await window.sw2robot.replay([{ t: 0, op: 'setJoint', joint: j, value: 0.6 }], { speed: 1 });
  return Number(window.viewer.robot.joints[j].angle);   // sampled the instant the await returns
}, plan.joint);
check('fix#1b: replayLog resolves only after the ease settles (video not truncated)',
      Math.abs(atResolve - 0.6) < 1e-3, `angleAtResolve=${atResolve}`);

// ---- fix #2: boxSelect replays the RECORDED subset, not "everything on screen"
const box = await page.evaluate(() => {
  const links = Object.keys(window.viewer.robot.links);
  const subset = links.slice(0, 1);              // pretend the user box-picked just one
  window.__boxentry = { t: 0, op: 'boxSelect', n: subset.length, names: subset };
  return { total: links.length, subset };
});
await page.evaluate(() => window.sw2robot.replay([window.__boxentry], { speed: 1 }));
await sleep(200);
const boxSel = await page.evaluate(() => window.sw2robot.boxSelected());
check('fix#2: boxSelect replays the recorded subset, not select-all',
      boxSel.length === box.subset.length && boxSel[0] === box.subset[0],
      `selected=${JSON.stringify(boxSel)} (of ${box.total})`);

// ---- fix #4: a skipped (non-replayable) op must NOT fire a cursor click-pulse
const phantom = await page.evaluate(async () => {
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const link = Object.keys(window.viewer.robot.links)[0];
  let pulsed = false;
  document.getElementById('replaycursor').classList.remove('click');  // clear stale pulse
  // reRoot is in CLICK_OPS but NOT in REPLAY -> should be skipped with no pulse
  window.sw2robot.replay([{ t: 0, op: 'reRoot', link }], { speed: 1 });
  for (let i = 0; i < 8; i++) {
    if (document.getElementById('replaycursor').classList.contains('click')) { pulsed = true; }
    await sleep(40);
  }
  return pulsed;
});
check('fix#4: skipped op does not show a phantom cursor click', phantom === false);

// ---- fix #8: a non-replayable op makes replay report a PARTIAL run by name
const partialLog = await page.evaluate(async () => {
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const link = Object.keys(window.viewer.robot.links)[0];
  window.sw2robot.replay([{ t: 0, op: 'select', link },
                          { t: 50, op: 'exclude', body: 'x' }], { speed: 1 });   // exclude not replayable
  await sleep(300);
  return [...document.querySelectorAll('#log div')].map(d => d.textContent).join('\n');
});
check('fix#8: partial replay is reported with the skipped op name',
      /部分的|partial/.test(partialLog) && /exclude/.test(partialLog));

// ---- fix #5: clearing the viewer (fresh session) drops the action + camera logs
const beforeClear = await page.evaluate(() => window.sw2robot.timeline().length);
const afterClear = await page.evaluate(() => {
  document.getElementById('clearview').click();     // empties the viewer
  return window.sw2robot.timeline().length;
});
check('fix#5: clearing the viewer resets the session logs',
      beforeClear > 0 && afterClear === 0, `before=${beforeClear} after=${afterClear}`);

check('no page errors', errs.length === 0, errs.join(' | '));

await browser.close();
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
