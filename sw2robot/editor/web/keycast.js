import { onOp, opLabel } from './session-log.js';
// ---- keystroke overlay (screencast keys) for recording demo videos --------
// OFF by default; the ⌨ keys toolbar toggle enables it. We only *listen* to
// events (never preventDefault), so this can't interfere with normal editing.
(function setupKeycast() {
  const overlay = document.getElementById('keyoverlay');  // bottom-left layer
  const toggle  = document.getElementById('keycast');     // ⌨ keys toolbar btn
  let on = false;
  const LIFETIME = 1300;          // ms a keycap stays before fading out
  const FADE = 250;               // must match CSS transition

  // pretty names for non-printable keys
  const NAMES = {
    ' ': 'Space', 'Spacebar': 'Space', 'Escape': 'Esc', 'Enter': '⏎',
    'Backspace': '⌫', 'Delete': 'Del', 'Tab': 'Tab ⇥',
    'ArrowUp': '↑', 'ArrowDown': '↓', 'ArrowLeft': '←', 'ArrowRight': '→',
    'Control': 'Ctrl', 'Meta': 'Cmd',
  };
  const isMod = k => k === 'Control' || k === 'Shift' || k === 'Alt' ||
                     k === 'Meta';

  let last = null;                // { el, label, count, timer } for ×N merge

  function expire(entry) {
    entry.el.classList.add('fading');
    setTimeout(() => { entry.el.remove(); if (last === entry) last = null; },
               FADE);
  }

  function show(label, kind) {
    // merge a rapid repeat of the identical chord into a ×N counter
    if (last && last.label === label && !last.el.classList.contains('fading')) {
      const entry = last;
      entry.count++;
      entry.el.querySelector('.mult').textContent = '×' + entry.count;
      clearTimeout(entry.timer);
      entry.timer = setTimeout(() => expire(entry), LIFETIME);
      return;
    }
    const el = document.createElement('span');
    el.className = 'keycap' + (kind && kind !== 'key' ? ' ' + kind : '');
    el.append(document.createTextNode(label));
    const m = document.createElement('span');
    m.className = 'mult';
    el.appendChild(m);
    overlay.appendChild(el);
    // cap how many keycaps live at once
    while (overlay.children.length > 6) { overlay.firstChild.remove(); }
    const entry = { el, label, count: 1, timer: null };
    entry.timer = setTimeout(() => expire(entry), LIFETIME);
    last = entry;
  }

  function chord(e, key) {
    const mods = [];
    if (e.ctrlKey)  mods.push('Ctrl');
    if (e.altKey)   mods.push('Alt');
    if (e.shiftKey) mods.push('Shift');
    if (e.metaKey)  mods.push('Cmd');
    let main = NAMES[key] || key;
    if (main.length === 1) main = main.toUpperCase();
    // don't print e.g. "Shift + Shift" when the key itself is the modifier
    if (isMod(key) && mods.length &&
        mods[mods.length - 1] === (NAMES[key] || key)) {
      return mods.join(' + ');
    }
    return [...mods, main].join(' + ');
  }

  function onKey(e) {
    if (!on || e.repeat) return;
    show(chord(e, e.key), 'key');
  }

  // Semantic actions come from the op() stream, NOT from raw mouse/wheel
  // events -- so a click that only orbits/pans/zooms the camera produces
  // nothing, while a click that selects a link, adds a port, re-roots, etc.
  // shows its meaning ("Select: base_link").  Keyboard is still read raw
  // above, because a keypress is almost always an intentional command.
  onOp(e => { if (on) show(opLabel(e), 'op'); });

  // capture phase so we see the keypress before any field/control consumes it
  window.addEventListener('keydown', onKey, true);

  toggle.addEventListener('click', () => {
    on = toggle.classList.toggle('active');
    overlay.style.display = on ? 'flex' : 'none';
    if (!on) { overlay.replaceChildren(); last = null; }
  });
  overlay.style.display = 'none';
})();

