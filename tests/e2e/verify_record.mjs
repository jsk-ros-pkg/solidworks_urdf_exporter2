// Video-recording check. getDisplayMedia is unavailable in headless Chrome, so
// we stub it to return a canvas.captureStream(), exercising the real
// MediaRecorder -> Blob path. We assert on the produced video Blob (via a
// URL.createObjectURL hook) rather than the downloaded file, because Chrome
// blocks *multiple* automatic downloads -- only the first would ever land on
// disk. In real use each stop is a user click, so every download is allowed.
//  - start -> isRecording() true, ⏺ button goes active
//  - stop  -> a non-empty video/webm blob is produced
//  - Shift+click auto-records a replay and stops itself, producing a 2nd blob
//  - no page errors
import puppeteer from 'puppeteer-core';
const sleep = ms => new Promise(r => setTimeout(r, ms));

const BASE = process.argv[2] ?? 'http://localhost:8090';
const URL = BASE + (BASE.includes('?') ? '&' : '?') + 'lang=ja';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const EXTRA = (process.env.CHROME_ARGS ?? '').split(',').map(s => s.trim()).filter(Boolean);
let failures = 0;
const check = (n, ok, d = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${n}${d ? '  -- ' + d : ''}`);
  if (!ok) failures++;
};
// video blobs the page produced (size + type), captured via createObjectURL
const videoBlobs = () => page.evaluate(() =>
  (window.__blobs || []).filter(b => b.type.startsWith('video/')));

const browser = await puppeteer.launch({
  executablePath: CHROME, headless: 'new', args: ['--disable-gpu', ...EXTRA] });
const page = await browser.newPage();
const errs = [];
page.on('pageerror', e => errs.push(e.message.split('\n')[0]));

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.sw2robot, { timeout: 60000 });
await sleep(1500);

// capture every Blob handed to createObjectURL (the .webm the recorder makes)
await page.evaluate(() => {
  window.__blobs = [];
  const orig = URL.createObjectURL.bind(URL);
  URL.createObjectURL = b => {
    if (b instanceof Blob) { window.__blobs.push({ size: b.size, type: b.type }); }
    return orig(b);
  };
});

// stub getDisplayMedia. Each call gets its OWN freshly-animated canvas so a
// second recording is not starved by the first stopping its capture track
// (real getDisplayMedia always returns a fresh live stream; only the stub needs
// this). The animation guarantees the MediaRecorder receives real frames.
await page.evaluate(() => {
  Object.defineProperty(navigator.mediaDevices, 'getDisplayMedia', {
    configurable: true, writable: true,
    value: async () => {
      const cv = document.createElement('canvas');
      cv.width = 320; cv.height = 240;
      const ctx = cv.getContext('2d');
      let f = 0;
      cv.__on = true;
      const draw = () => {
        ctx.fillStyle = `hsl(${(f++ * 7) % 360},70%,50%)`;
        ctx.fillRect(0, 0, 320, 240);
        if (cv.__on) { requestAnimationFrame(draw); }
      };
      draw();
      const stream = cv.captureStream(30);
      stream.getVideoTracks()[0].addEventListener('ended', () => { cv.__on = false; });
      return stream;
    },
  });
});

// ---- manual start / stop ---------------------------------------------------
await page.click('#record');
await sleep(300);
const recActive = await page.evaluate(() => ({
  rec: window.sw2robot.isRecording(),
  active: document.getElementById('record').classList.contains('active'),
}));
check('record: start -> isRecording() true', recActive.rec);
check('record: ⏺ button shows active state', recActive.active);

// keep the canvas producing frames, then stop
await sleep(700);
await page.click('#record');
await sleep(800);   // let onstop fire + the blob get built

const v1 = await videoBlobs();
check('record: a non-empty video/webm blob was produced',
      v1.length === 1 && v1[0].size > 0, JSON.stringify(v1));
check('record: isRecording() false after stop',
      !(await page.evaluate(() => window.sw2robot.isRecording())));

// ---- Shift+click = auto-record a replay (starts, replays, stops itself) -----
await page.evaluate(() => { const n = Object.keys(window.viewer.robot.links)[0];
  window.sw2robot.select(n); });                 // put one action in the log
await page.keyboard.down('Shift');
await page.click('#record');
await page.keyboard.up('Shift');
// replay clamps the load->select gap to maxGap (1500ms), then stops itself
await sleep(3500);
const v2 = await videoBlobs();
check('record: Shift+click auto-records a replay and self-stops',
      v2.length === 2 && v2[1].size > 0
      && !(await page.evaluate(() => window.sw2robot.isRecording())),
      `blobs=${v2.length} recording=${await page.evaluate(() => window.sw2robot.isRecording())}`);

check('no page errors', errs.length === 0, errs.join(' | '));

await browser.close();
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
