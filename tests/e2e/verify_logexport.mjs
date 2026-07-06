// Step-3 check: the action log exports as readable text and as raw JSON.
//  - actionLogText() renders "+<sec>s  <label>" lines from the op() stream
//  - a real action (select) shows up with its readable label
//  - the ⤓ button downloads a .txt (click) and a .json (shift+click)
//  - no page errors
import puppeteer from 'puppeteer-core';

const BASE = process.argv[2] ?? 'http://localhost:8090';
const URL = BASE + (BASE.includes('?') ? '&' : '?') + 'lang=ja';
const CHROME = process.env.CHROME_PATH
  ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const EXTRA = (process.env.CHROME_ARGS ?? '').split(',').map(s => s.trim()).filter(Boolean);
const sleep = ms => new Promise(r => setTimeout(r, ms));
import os from 'node:os';
import fs from 'node:fs';
import path from 'node:path';
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

// route downloads to a temp dir so we can assert the files land
const dlDir = fs.mkdtempSync(path.join(os.tmpdir(), 'sw2log-'));
const client = await page.target().createCDPSession();
await client.send('Page.setDownloadBehavior', { behavior: 'allow', downloadPath: dlDir });

await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 60000 });
await page.waitForFunction(() => window.sw2robot, { timeout: 60000 });
await sleep(1500);

// do a real, labelled action
const link = await page.evaluate(() => {
  const n = Object.keys(window.viewer.robot.links)[0];
  window.sw2robot.select(n);
  return n;
});

const txt = await page.evaluate(() => window.sw2robot.actionLogText());
check('actionLogText: renders "+<sec>s  <label>" lines',
      /\+\d+\.\d+s\s+\S/.test(txt), JSON.stringify(txt.split('\n')[0]));
check('actionLogText: includes the select action with its label',
      txt.includes('Select: ' + link), `link=${link}`);

// ⤓ click -> .txt, shift+click -> .json
await page.click('#savelog');
await sleep(400);
await page.keyboard.down('Shift');
await page.click('#savelog');
await page.keyboard.up('Shift');
await sleep(400);
const files = fs.readdirSync(dlDir).filter(f => !f.endsWith('.crdownload'));
check('download: a .txt file landed', files.some(f => f.endsWith('-actions.txt')),
      files.join(', '));
check('download: a .json file landed', files.some(f => f.endsWith('-actions.json')),
      files.join(', '));
const jf = files.find(f => f.endsWith('.json'));
if (jf) {
  const parsed = JSON.parse(fs.readFileSync(path.join(dlDir, jf), 'utf8'));
  check('download: .json is the raw oplog array', Array.isArray(parsed)
        && parsed.some(o => o.op === 'select'), `entries=${parsed.length}`);
}

check('no page errors', errs.length === 0, errs.join(' | '));

await browser.close();
fs.rmSync(dlDir, { recursive: true, force: true });
console.log(failures ? `\n${failures} FAILURE(S)` : '\nALL PASS');
process.exit(failures ? 1 : 0);
