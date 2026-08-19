import { viewer } from './dom.js';
import { openAny, openServerPath } from './export-box.js';
import { clearMeshCacheAndReextract } from './lists.js';
// ---- 📁 server-side file browser (the OS dialog can't give us a path) ---
const fsmodal = document.getElementById('fsmodal');
let fsCur = '';

const FS_LAST_DIR_KEY = 'sw2robot.fsdir';

// File-type marks for the browser rows + the filter checkboxes.  These are our
// OWN inline SVG glyphs -- NOT Dassault's proprietary SolidWorks icons -- that
// borrow only the familiar colour coding (green = part, red = assembly) so a
// row's type reads at a glance.  Inline (no external request) => CSP-safe.
const FS_ICON = {
  // a green isometric cube = a single part
  sldprt: '<svg width="13" height="13" viewBox="0 0 24 25" aria-hidden="true" '
        + 'style="vertical-align:-2px;flex:none">'
        + '<polygon points="12,2 21,7 12,12 3,7" fill="#8ccf8c"/>'
        + '<polygon points="3,7 3,17 12,22 12,12" fill="#4ea24e"/>'
        + '<polygon points="21,7 21,17 12,22 12,12" fill="#3c7d3c"/></svg>',
  // two red cubes = an assembly of parts
  sldasm: '<svg width="14" height="14" viewBox="0 0 26 27" aria-hidden="true" '
        + 'style="vertical-align:-2px;flex:none">'
        + '<polygon points="16,2 23,6 16,10 9,6" fill="#eba49c"/>'
        + '<polygon points="9,6 9,14 16,18 16,10" fill="#c9564b"/>'
        + '<polygon points="23,6 23,14 16,18 16,10" fill="#aa4238"/>'
        + '<polygon points="9,9 16,13 9,17 2,13" fill="#f2b0a8"/>'
        + '<polygon points="2,13 2,21 9,25 9,17" fill="#d75f53"/>'
        + '<polygon points="16,13 16,21 9,25 9,17" fill="#b8493f"/></svg>',
  // a blue robot head = a URDF (robot definition); blue is the natural third
  // colour next to the green part / red assembly (and SolidWorks' drawing hue)
  urdf: '<svg width="14" height="14" viewBox="0 0 24 24" aria-hidden="true" '
      + 'style="vertical-align:-2px;flex:none">'
      + '<circle cx="12" cy="2.7" r="1.5" fill="#8fb0e8"/>'
      + '<rect x="11.3" y="3.4" width="1.4" height="3" fill="#8fb0e8"/>'
      + '<rect x="3.5" y="6" width="17" height="13.5" rx="3.2" fill="#4f7fd0"/>'
      + '<rect x="7" y="10.4" width="3.3" height="3.3" rx="1" fill="#eaf1ff"/>'
      + '<rect x="13.7" y="10.4" width="3.3" height="3.3" rx="1" fill="#eaf1ff"/>'
      + '<rect x="8.5" y="15.6" width="7" height="1.6" rx="0.8" fill="#bcd2f5"/></svg>',
};

async function fsShow(path) {
  const r = await (await fetch(
    '/api/fs?path=' + encodeURIComponent(path ?? ''))).json();
  if (r.error) { log(t('fs.error', { e: r.error }), 'err'); return; }
  fsCur = r.path;
  try { localStorage.setItem(FS_LAST_DIR_KEY, r.path || ''); } catch { /**/ }
  document.getElementById('fspath').value = r.path || '';
  document.getElementById('fsup').disabled = !r.parent && !r.path;
  document.getElementById('fsup').dataset.parent = r.parent ?? '';
  document.getElementById('fsfilter').value = '';
  const list = document.getElementById('fslist');
  list.innerHTML = '';
  const row = (icon, name, cb, hint) => {
    const d = document.createElement('div');
    // icon is one of OUR constants (emoji or inline SVG) -> innerHTML is safe;
    // the file NAME is untrusted, so it goes in as a text node (never HTML)
    const ic = document.createElement('span');
    ic.style.cssText = 'display:inline-block;margin-right:4px';
    ic.innerHTML = icon;
    d.append(ic, document.createTextNode(
      `${name}${hint ? '   ' + hint : ''}`));
    d.dataset.fsname = String(name).toLowerCase();
    d.style.cssText = 'padding:3px 6px;border-radius:3px;cursor:pointer;' +
                      'white-space:nowrap;overflow:hidden;' +
                      'text-overflow:ellipsis';
    d.addEventListener('mouseenter', () => d.style.background = '#2a3340');
    d.addEventListener('mouseleave', () => d.style.background = '');
    d.addEventListener('click', cb);
    list.appendChild(d);
    return d;
  };
  // tag an input file row with its type so the .sldasm/.sldprt/.urdf checkboxes
  // can filter it (untagged rows -- folders, packages -- stay always shown)
  const tagCad = (d, name) => {
    const m = /\.(sldasm|sldprt|urdf)$/i.exec(name);
    if (m) { d.dataset.fstype = m[1].toLowerCase(); }
    return d;
  };
  if (!r.path) {           // root view: recent SolidWorks files + built packages
    const [recent, pkgs] = await Promise.all([
      fetch('/api/recent').then(x => x.json()).catch(() => []),
      fetch('/api/list').then(x => x.json()).catch(() => [])]);
    for (const p of (recent || []).slice(0, 10)) {
      const nm = p.split(/[\\/]/).pop();
      tagCad(row('⭐', nm, () => { fsmodal.style.display = 'none'; openAny(p); },
                 t('fs.recentHint')), nm);
    }
    for (const pk of (pkgs || []).slice(0, 10)) {
      row('⭐📦', pk.name, () => { fsmodal.style.display = 'none';
                                  openServerPath(pk.path); },
          t('fs.pkgRecentHint'));
    }
  }
  for (const dir of r.dirs) {
    if (dir.package) {
      row('📦', dir.name, () => { fsmodal.style.display = 'none';
                                  openServerPath(dir.path); },
          t('fs.packageHint'));
    } else {
      row('📁', dir.name, () => fsShow(dir.path));
    }
  }
  for (const f of r.files) {
    const lc = f.name.toLowerCase();
    const icon = lc.endsWith('.sldasm') ? FS_ICON.sldasm
               : lc.endsWith('.sldprt') ? FS_ICON.sldprt : FS_ICON.urdf;
    tagCad(row(icon, f.name,
               () => { fsmodal.style.display = 'none'; openAny(f.path); }),
           f.name);
  }
  if (!r.dirs.length && !r.files.length) {
    row('·', t('fs.empty'), () => {});
  }
  fsFilter();              // apply the .sldasm/.sldprt checkboxes (sldprt off by default)
  fsmodal.style.display = 'flex';
}

// narrow the current listing: the text box AND the .sldasm/.sldprt/.urdf
// checkboxes.  Rows with no fstype (folders, packages) are never hidden by the
// checkboxes; only input file rows are.
function fsFilter() {
  const q = document.getElementById('fsfilter').value.trim().toLowerCase();
  const showAsm = document.getElementById('fstype-sldasm').checked;
  const showPrt = document.getElementById('fstype-sldprt').checked;
  const showUrdf = document.getElementById('fstype-urdf').checked;
  for (const d of document.querySelectorAll('#fslist > div')) {
    const ty = d.dataset.fstype;
    const typeOk = ty === 'sldasm' ? showAsm
                 : ty === 'sldprt' ? showPrt
                 : ty === 'urdf' ? showUrdf : true;
    const textOk = !q || (d.dataset.fsname || '').includes(q);
    d.style.display = (typeOk && textOk) ? '' : 'none';
  }
}

// the path bar IS the old 📋: paste/edit a full path + Enter -> a
// .sldasm/.sldprt/.urdf is opened/extracted, anything else is treated as a
// directory to navigate to.
// Windows "Copy as path" wraps the path in quotes -- strip them.
function fsGo() {
  const p = document.getElementById('fspath').value.trim().replace(/^"+|"+$/g, '');
  if (!p) { return; }
  if (/\.(sldasm|sldprt|urdf)$/i.test(p)) { fsmodal.style.display = 'none'; openAny(p); }
  else { fsShow(p); }
}

function openFsBrowser() {
  let start = fsCur;
  try { start = start || localStorage.getItem(FS_LAST_DIR_KEY) || ''; }
  catch { /**/ }
  fsShow(start);
  setTimeout(() => document.getElementById('fspath').focus(), 50);
}
function closeFsBrowser() { fsmodal.style.display = 'none'; }
document.getElementById('fsbrowse').addEventListener('click', openFsBrowser);
document.getElementById('clearmesh')
  .addEventListener('click', clearMeshCacheAndReextract);

// Clicking the EMPTY viewer opens the file browser (no robot loaded -> the only
// thing to do is open one), like pressing 🗄.  While the browser is open, a
// click anywhere outside it (or Escape) dismisses it.
const viewwrapEl = document.getElementById('viewwrap');
viewwrapEl.addEventListener('click', e => {
  if (e.target.closest('#fsmodal')) { return; }     // clicks inside it: ignore
  if (fsmodal.style.display !== 'none') { closeFsBrowser(); return; }  // click-away
  // empty viewer surface -> open (the 3D click lands on the <canvas> inside the
  // viewer, not the element itself, so match anything that isn't a UI control)
  if (!viewer.robot && !e.target.closest('button, input, select, a')) {
    openFsBrowser();
  }
});
// capture phase so Escape closes the browser even while typing in its path/
// filter fields (the inputs' own handlers would otherwise swallow it)
document.addEventListener('keydown', e => {
  if (e.key === 'Escape' && fsmodal.style.display !== 'none') {
    closeFsBrowser();
    e.stopPropagation();
  }
}, true);
document.getElementById('fsgo').addEventListener('click', fsGo);
document.getElementById('fspath').addEventListener('keydown', e => {
  if (e.key === 'Enter') { e.preventDefault(); fsGo(); }
});
document.getElementById('fsfilter').addEventListener('input', fsFilter);
document.getElementById('fsfilter').addEventListener('keydown', e => {
  if (e.key !== 'Enter') { return; }     // Enter on a lone match activates it
  const vis = [...document.querySelectorAll('#fslist > div')]
    .filter(d => d.style.display !== 'none');
  if (vis.length === 1) { vis[0].click(); }
});

document.getElementById('fsclose').addEventListener('click',
  () => { fsmodal.style.display = 'none'; });
document.getElementById('fsup').addEventListener('click', ev =>
  fsShow(ev.target.dataset.parent || ''));

// .sldasm / .sldprt / .urdf type filter: .sldprt is OFF by default (its HTML
// checkbox is unchecked), the other two ON; a user's choice persists across
// sessions in localStorage.
(function () {
  // paint the checkbox marks from the same SVG constants the rows use
  for (const el of document.querySelectorAll('#fsmodal [data-fsicon]')) {
    el.innerHTML = FS_ICON[el.dataset.fsicon] || '';
  }
  const asm = document.getElementById('fstype-sldasm');
  const prt = document.getElementById('fstype-sldprt');
  const urdf = document.getElementById('fstype-urdf');
  const K_ASM = 'sw2robot.fs.showSldasm', K_PRT = 'sw2robot.fs.showSldprt',
        K_URDF = 'sw2robot.fs.showUrdf';
  try {
    const a = localStorage.getItem(K_ASM);
    const p = localStorage.getItem(K_PRT);
    const u = localStorage.getItem(K_URDF);
    if (a !== null) { asm.checked = a === '1'; }
    if (p !== null) { prt.checked = p === '1'; }
    if (u !== null) { urdf.checked = u === '1'; }
  } catch { /**/ }
  const onChange = () => {
    try {
      localStorage.setItem(K_ASM, asm.checked ? '1' : '0');
      localStorage.setItem(K_PRT, prt.checked ? '1' : '0');
      localStorage.setItem(K_URDF, urdf.checked ? '1' : '0');
    } catch { /**/ }
    fsFilter();
  };
  asm.addEventListener('change', onChange);
  prt.addEventListener('change', onChange);
  urdf.addEventListener('change', onChange);
})();

// ---- resizable right panel (link names get long; let the user widen it) ---
(function () {
  const panel = document.getElementById('panel');
  const resizer = document.getElementById('resizer');
  const KEY = 'sw2robot.panelWidth';
  const MIN = 240;
  const maxW = () => Math.max(MIN, window.innerWidth - 360);   // keep the viewer usable
  const apply = w => {
    panel.style.width = Math.round(Math.min(maxW(), Math.max(MIN, w))) + 'px';
  };
  try {
    const saved = parseFloat(localStorage.getItem(KEY));
    if (saved > 0) { apply(saved); }
  } catch { /**/ }
  let dragging = false;
  resizer.addEventListener('mousedown', e => {
    dragging = true; e.preventDefault();
    document.body.classList.add('resizing');
  });
  window.addEventListener('mousemove', e => {
    if (dragging) { apply(window.innerWidth - e.clientX); }  // panel hugs the right edge
  });
  window.addEventListener('mouseup', () => {
    if (!dragging) { return; }
    dragging = false;
    document.body.classList.remove('resizing');
    try { localStorage.setItem(KEY, parseFloat(panel.style.width)); } catch { /**/ }
  });
  resizer.addEventListener('dblclick', () => {       // reset to the default width
    panel.style.width = '';
    try { localStorage.removeItem(KEY); } catch { /**/ }
  });
})();

