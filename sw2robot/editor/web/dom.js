// ---- the element handles more than one module needs ---------------------
// A LEAF module: it imports nothing.  `viewer` and `expLinks` in particular
// are read at EVALUATION time (listener registration, link wiring) by modules
// that sit in an import cycle with whoever used to own them, so owning them
// here is what keeps those reads from hitting an uninitialised binding.
// A handle exactly one module uses stays in that module.
export const viewer = document.getElementById('viewer');
export const jointsEl = document.getElementById('joints');
export const hovertip = document.getElementById('hovertip');
export const bulkbarEl = document.getElementById('bulkbar');
export const mimicBar = document.getElementById('mimicbar');
// toolbar toggles whose .active class IS the mode flag
export const originBtn = document.getElementById('origin');
export const tfBtn = document.getElementById('tf');
export const alignBtn = document.getElementById('alignmode');
export const portBtn = document.getElementById('portmode');
// the export box: names threaded into the ZIP links, and the collision
// controls the CoACD preview shares with them
export const exppkg = document.getElementById('exppkg');
export const expurdf = document.getElementById('expurdf');
export const exprobot = document.getElementById('exprobot');
export const expmeshdir = document.getElementById('expmeshdir');
export const expLinks = ['expdae', 'expros2']
  .map(id => document.getElementById(id)).filter(Boolean);
export const expVisFmt = document.getElementById('expvisfmt');
export const expColFmt = document.getElementById('expcolfmt');
export const collModeSel = document.getElementById('collmode');
export const cqualitySel = document.getElementById('cquality');
export const mergeFixedBox = document.getElementById('mergefixed');
