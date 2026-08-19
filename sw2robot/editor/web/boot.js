  const logEl = document.getElementById('log');
  const statusEl = document.getElementById('status');
  const t0 = performance.now();
  function log(msg, cls) {
    const sec = ((performance.now() - t0) / 1000).toFixed(1);
    const line = document.createElement('div');
    line.innerHTML = `<span class="t">${sec}s</span>` +
                     `<span class="${cls || ''}"></span>`;
    line.lastChild.textContent = msg;
    logEl.appendChild(line);
    logEl.scrollTop = logEl.scrollHeight;
    if (cls === 'err') { statusEl.textContent = '❌ ' + msg; }
  }
  window.log = log;

  // Backend-down banner: when the local server stops (or the network drops),
  // every fetch throws `TypeError: Failed to fetch` -- cryptic on its own.
  // Wrap fetch so ANY failed request raises a clear, persistent banner, and any
  // successful response clears it (auto-recovers when the server is back).
  function setBackendDown(down) {
    const el = document.getElementById('backendDown');
    if (!el) { return; }
    if (down && el.style.display === 'none') {
      document.getElementById('backendDownMsg').textContent = t('backend.down');
    }
    el.style.display = down ? 'block' : 'none';
  }
  window.setBackendDown = setBackendDown;
  const isNetErr = r => (r instanceof TypeError)
    && /fetch|network|load failed/i.test(r.message || '');
  const _origFetch = window.fetch.bind(window);
  window.fetch = async (...args) => {
    try {
      const r = await _origFetch(...args);
      setBackendDown(false);            // got a response -> server is alive
      return r;
    } catch (e) {
      if (isNetErr(e)) { setBackendDown(true); }
      throw e;                          // keep existing error handling working
    }
  };
  const reloadBtn = document.getElementById('backendReload');
  if (reloadBtn) { reloadBtn.addEventListener('click', () => location.reload()); }

  // --- self-update -----------------------------------------------------
  // The editor ships as a single binary attached to a GitHub Release.  Ask the
  // backend (which talks to the GitHub API) whether a newer one exists; if so,
  // show a banner that downloads + swaps the running binary and relaunches.
  // From a source checkout the backend reports frozen:false and "Update now"
  // just opens the release page (there is nothing to swap).
  (function setupUpdate() {
    const bar = document.getElementById('updateBar');
    const chip = document.getElementById('verChip');
    const msg = document.getElementById('updateMsg');
    const nowBtn = document.getElementById('updateNow');
    const viewLink = document.getElementById('updateView');
    const laterBtn = document.getElementById('updateLater');
    let lastInfo = null;
    let pollTimer = null;

    function showBar(info) {
      msg.textContent = t('update.available', { v: info.latest, cur: info.current });
      nowBtn.textContent = info.frozen ? t('update.now') : t('update.manual');
      nowBtn.style.display = ''; nowBtn.disabled = false;
      viewLink.textContent = t('update.view');
      viewLink.href = info.html_url; viewLink.style.display = '';
      laterBtn.textContent = t('update.later'); laterBtn.style.display = '';
      bar.style.display = 'block';
    }

    function setChip(info) {
      if (!chip) { return; }
      const v = (info && info.current) || '?';
      chip.textContent = t('ver.current', { v });
      if (info && info.update_available) {
        chip.style.color = '#7fe6a0';
        chip.title = t('ver.update', { v: info.latest });
      } else {
        chip.style.color = '#8aa';
        chip.title = (info && info.error) ? info.error : t('ver.latest');
      }
    }

    async function check(force) {
      try {
        const info = await (await fetch('/api/version' + (force ? '?force=1' : '')))
          .json();
        lastInfo = info;
        setChip(info);
        if (info.update_available) { showBar(info); }
        return info;
      } catch (e) { return null; }   // offline / server down: stay quiet
    }

    async function apply() {
      const info = lastInfo;
      if (!info) { return; }
      if (!info.frozen) {            // source checkout -> just open the page
        window.open(info.html_url, '_blank', 'noopener'); return;
      }
      nowBtn.disabled = true;
      try {
        const r = await (await fetch('/api/update/apply', { method: 'POST' })).json();
        if (!r.ok) {
          msg.textContent = t('update.failed', { e: r.error || '?' });
          nowBtn.disabled = false; return;
        }
      } catch (e) {
        msg.textContent = t('update.failed', { e: String(e) });
        nowBtn.disabled = false; return;
      }
      poll();
    }

    function poll() {
      clearTimeout(pollTimer);
      pollTimer = setTimeout(async () => {
        let s;
        try { s = await (await fetch('/api/update/status')).json(); }
        catch (e) { poll(); return; }   // server may be mid-restart -- keep trying
        nowBtn.style.display = 'none';
        viewLink.style.display = 'none';
        laterBtn.style.display = 'none';
        if (s.state === 'downloading') {
          msg.textContent = t('update.downloading',
            { v: s.version || '', pct: s.pct || 0 });
          poll();
        } else if (s.state === 'installing') {
          msg.textContent = t('update.installing'); poll();
        } else if (s.state === 'restarting') {
          msg.textContent = t('update.restarting');
          // the old server is exiting; the relaunched instance rebinds the port
          // (and opens its own tab).  Reload to reconnect to the new version.
          setTimeout(() => location.reload(), 4000);
        } else if (s.state === 'error') {
          msg.textContent = t('update.failed', { e: s.error || '?' });
          nowBtn.style.display = ''; nowBtn.disabled = false;
          laterBtn.style.display = '';
        } else { poll(); }
      }, 600);
    }

    nowBtn.addEventListener('click', apply);
    laterBtn.addEventListener('click', () => { bar.style.display = 'none'; });
    if (chip) {
      chip.addEventListener('click', () => { chip.title = t('ver.checking'); check(true); });
    }
    check(false);
  })();

  window.addEventListener('error', e =>
    log(t('boot.jsError', { msg: e.message,
        file: (e.filename || '?').split('/').pop(), line: e.lineno }), 'err'));
  window.addEventListener('unhandledrejection', e => {
    // a dead backend surfaces here as "Failed to fetch" -- the banner already
    // explains it clearly, so don't also dump the raw TypeError on the user
    if (isNetErr(e.reason)) { setBackendDown(true); e.preventDefault(); return; }
    log(t('boot.promiseRejected', { reason: e.reason }), 'err');
  });
  // translate the static chrome + reflect the active language button now
  applyStaticI18n();
  // #title / #status are dynamic (set from JS), so localize their placeholders
  // here rather than via data-i18n (which would clobber them on a lang switch)
  document.getElementById('title').textContent = t('ui.title');
  statusEl.textContent = t('status.starting');
  document.querySelectorAll('#langbar button').forEach(b =>
    b.classList.toggle('active', b.dataset.lang === window.__lang));
  log(t('boot.pageLoaded'));
