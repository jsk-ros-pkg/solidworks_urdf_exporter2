"""Every display language covers the whole UI catalogue.

English and Japanese live inline in ``web/i18n.js``; each other language is
its own ``web/i18n.<code>.js``.  A key a language lacks silently falls back to
English, so a UI change that adds a string without translating it would leave
that language half English with nothing failing.  This evaluates the files in
node and checks, per language: the same keys as English, a template where
English has one, reading the same ``args`` fields, and the file loaded before
``i18n.js`` so it is there when the page starts.
"""
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

WEB = Path(__file__).resolve().parents[1] / "sw2robot" / "editor" / "web"

_PROBE = r"""
const fs = require('fs');
const vm = require('vm');
const [web, ...codes] = process.argv.slice(1);
const src = fs.readFileSync(`${web}/i18n.js`, 'utf8');
const start = src.indexOf('const I18N = {');
const end = src.indexOf('\n  };', start);
const base = vm.runInNewContext(`(${src.slice(start + 'const I18N = '.length, end + 4)})`);

// call a template with args that record which fields it reads
const leaf = new Proxy(function () {}, {
  get: (t, p) => (p === Symbol.toPrimitive ? () => 1 : leaf),
  apply: () => leaf,
});
function shape(v) {
  if (typeof v !== 'function') {
    return { fn: false, ok: typeof v === 'string' && v.length > 0, args: [] };
  }
  const seen = new Set();
  const args = new Proxy({}, {
    get: (t, p) => { if (typeof p === 'string') { seen.add(p); } return leaf; },
  });
  let ok = true;
  try { ok = typeof v(args) === 'string'; } catch (e) { ok = false; }
  return { fn: true, ok, args: [...seen].sort() };
}

const out = { en: {}, langs: {} };
for (const [k, e] of Object.entries(base)) { out.en[k] = shape(e.en); }
for (const code of codes) {
  const ctx = { window: {} };
  vm.runInNewContext(fs.readFileSync(`${web}/i18n.${code}.js`, 'utf8'), ctx);
  const table = (ctx.window.SW2_I18N || {})[code] || {};
  out.langs[code] = {};
  for (const [k, v] of Object.entries(table)) { out.langs[code][k] = shape(v); }
}
console.log(JSON.stringify(out));
"""


def _extra_langs():
    src = (WEB / "i18n.js").read_text(encoding="utf-8")
    block = src[src.index("const LANGS = ["):]
    block = block[:block.index("];")]
    return [c for c in re.findall(r"code: '([^']+)'", block)
            if c not in ("en", "ja")]


@pytest.fixture(scope="module")
def catalog():
    node = shutil.which("node")
    if node is None:
        pytest.skip("node is not installed")
    res = subprocess.run([node, "-e", _PROBE, str(WEB), *_extra_langs()],
                         capture_output=True, text=True, encoding="utf-8",
                         check=True)
    return json.loads(res.stdout)


def test_there_is_a_language_beyond_en_ja():
    assert _extra_langs(), "LANGS lists no language file to check"


@pytest.mark.parametrize("code", _extra_langs())
def test_language_file_loads_before_i18n_js(code):
    html = (WEB / "index.html").read_text(encoding="utf-8")
    own = html.find(f'<script src="i18n.{code}.js"></script>')
    assert own != -1, f"index.html does not load i18n.{code}.js"
    assert own < html.find('<script src="i18n.js"></script>')


@pytest.mark.parametrize("code", _extra_langs())
def test_language_covers_every_key(catalog, code):
    en, lang = catalog["en"], catalog["langs"][code]
    assert sorted(set(en) - set(lang)) == [], f"{code}: untranslated keys"
    assert sorted(set(lang) - set(en)) == [], f"{code}: keys English lacks"


@pytest.mark.parametrize("code", _extra_langs())
def test_templates_match_english(catalog, code):
    en, lang = catalog["en"], catalog["langs"][code]
    bad = []
    for key, want in en.items():
        got = lang.get(key)
        if got is None:
            continue
        if not got["ok"]:
            bad.append(f"{key}: not a non-empty string")
        elif got["fn"] != want["fn"]:
            bad.append(f"{key}: template={got['fn']}, English={want['fn']}")
        elif got["args"] != want["args"]:
            bad.append(f"{key}: reads {got['args']}, English {want['args']}")
    assert bad == []
