// Verify MULTI-SELECT joint-type changes after the optimistic-update rework.
//  A) sidebar checkboxes + #bulkset  -> goes through applyTypeChanges (instant)
//  B) 3D box-select + bulkType()     -> bulkSetType (reload path, persist-guarded)
import puppeteer from 'puppeteer-core';
const URL = (process.argv[2] ?? 'http://localhost:8090') + '?lang=ja';
const YAML = process.argv[3];
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

const movct = () => page.evaluate(() => Object.values(
  document.getElementById('viewer').robot.joints)
  .filter(j => j.jointType !== 'fixed' && !j.mimicJoint).length);

// ---- A) checkbox multi-select -> bulkset 'fixed' (optimistic path) ----------
const a = await page.evaluate(() => {
  const v = document.getElementById('viewer');
  const robotBefore = v.robot;
  const movBefore = Object.values(v.robot.joints)
    .filter(j => j.jointType !== 'fixed' && !j.mimicJoint).length;
  let picked = 0; const targets = [];
  for (const row of document.querySelectorAll('.joint')) {
    const sel = row.querySelector('.jtypesel');
    if (sel && sel.value !== 'fixed') {
      const pick = row.querySelector('.pick');
      if (pick) {
        pick.checked = true;
        targets.push(row.querySelector('.jname').textContent.trim());
        picked++;
      }
    }
    if (picked >= 3) break;
  }
  document.getElementById('bulktype').value = 'fixed';
  document.getElementById('bulkset').click();           // synchronous handler
  const movAfter = Object.values(v.robot.joints)
    .filter(j => j.jointType !== 'fixed' && !j.mimicJoint).length;
  return { picked, movBefore, movAfter, same: v.robot === robotBefore, targets };
});
check('checkbox bulk->fixed: all picked joints flip synchronously',
  a.picked === 3 && a.movAfter === a.movBefore - a.picked, JSON.stringify(a));
check('checkbox bulk->fixed: NO full robot reload (same object)',
  a.same === true, `same=${a.same}`);

// persisted to yaml?
if (YAML) {
  const fs = await import('node:fs');
  let ok = false;
  for (let i = 0; i < 40; i++) {
    await sleep(1000);
    const txt = fs.readFileSync(YAML, 'utf-8');
    ok = a.targets.every(c => new RegExp('child:\\s*' + c +
      '\\s*\\n\\s*type:\\s*fixed').test(txt));
    if (ok) break;
  }
  check('checkbox bulk->fixed: all 3 persisted to joints.yaml', ok,
    a.targets.join(', '));
}

// ---- B) 3D box-select everything -> bulkType('fixed') (now optimistic too) --
const bsel = await page.evaluate(() => {
  const v = document.getElementById('viewer');
  const robotBefore = v.robot;
  const mv = () => Object.values(v.robot.joints)
    .filter(j => j.jointType !== 'fixed' && !j.mimicJoint).length;
  const movBefore = mv();
  window.sw2robot.boxSelect();
  window.sw2robot.bulkType('fixed');
  return { movBefore, movAfter: mv(), same: v.robot === robotBefore };
});
check('box-select bulk->fixed: flips synchronously to 0 movable',
  bsel.movAfter === 0, JSON.stringify(bsel));
check('box-select bulk->fixed: NO full robot reload (same object)',
  bsel.same === true, `same=${bsel.same}`);
await sleep(8000);   // let the box-select background persist + rebuild settle

check('no page errors during bulk type changes', errs.length === 0, errs.join(' | '));
await browser.close();
console.log(`\n${fail === 0 ? 'ALL PASSED' : fail + ' FAILED'}`);
process.exit(fail === 0 ? 0 : 1);
