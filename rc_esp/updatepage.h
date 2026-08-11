const char updatePage[] PROGMEM = R"rawliteral(











<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>{{OTA_NAME}}</title>
  <style>
    body {
      background: #f5f7fa;
      font-family: 'Segoe UI', Arial, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      margin: 0;
    }
    .container {
      background: #fff;
      margin-top: 60px;
      padding: 32px 28px 24px 28px;
      border-radius: 12px;
      box-shadow: 0 4px 24px rgba(0, 0, 0, 0.08);
      min-width: 320px;
      max-width: 420px;
      width: 100%;
      box-sizing: border-box;
    }
    h2 {
      margin-top: 0;
      color: #222;
      font-weight: 600;
      text-align: center;
    }
    label {
      display: block;
      margin-bottom: 6px;
      color: #333;
      font-size: 1em;
      font-weight: 500;
    }
    input[type="text"] {
      width: 100%;
      padding: 10px 8px;
      margin-bottom: 16px;
      border: 1px solid #ccc;
      border-radius: 6px;
      font-size: 1em;
      box-sizing: border-box;
    }
    input[type="file"] {
      width: 100%;
      margin-bottom: 16px;
      box-sizing: border-box;
    }
    h3 {
      margin: 18px 0 10px;
      color: #222;
      font-weight: 600;
      font-size: 1.02em;
    }
    button {
      background: #0078d7;
      color: #fff;
      border: none;
      border-radius: 6px;
      padding: 12px 24px;
      font-size: 1rem;
      cursor: pointer;
      margin-bottom: 8px;
      transition: background 0.2s;
    }
    button:disabled {
      background: #b0b0b0;
      cursor: not-allowed;
    }
    button:hover:enabled {
      background: #005fa3;
    }
    .hint {
      color: #888;
      font-size: 0.97em;
      text-align: center;
      margin-top: 8px;
    }

    /* Popup modal */
    #popup {
      display: none;
      position: fixed;
      left: 0; top: 0;
      width: 100vw; height: 100vh;
      background: rgba(0,0,0,0.25);
      z-index: 1000;
      align-items: center; justify-content: center;
    }
    #popup .box {
      background: #fff;
      border-radius: 10px;
      padding: 28px 22px;
      min-width: 260px;
      max-width: 90vw;
      box-shadow: 0 4px 24px rgba(0,0,0,0.12);
      text-align: center;
    }
    .loader {
      display: inline-block;
      width: 32px; height: 32px;
      border: 4px solid #0078d7;
      border-radius: 50%;
      border-top-color: transparent;
      animation: spin 1s linear infinite;
      margin: 14px 0;
    }
    @keyframes spin { 0%{transform:rotate(0deg)} 100%{transform:rotate(360deg)} }

    .btn-row { margin-top: 16px; display: flex; gap: 10px; justify-content: center; }
    .btn-red { background: #d32f2f; }
    .btn-red:hover:enabled { background: #b71c1c; }
    .btn-blue { background: #0078d7; }
    .btn-blue:hover:enabled { background: #005fa3; }
  </style>
</head>
<body>
  <div class="container">
    <h2>Firmware Update</h2>

    <h3>Update via URL</h3>
    <label for="urlInput">Update URL</label>
    <input id="urlInput" type="text" placeholder="http://host/firmware.bin" />
    <label for="md5Input">MD5 (opsional)</label>
    <input id="md5Input" type="text" placeholder="32-hex MD5 (optional)" />
    <button id="urlUpdateBtn">Upload via URL</button>
    <div class="hint">Pastikan URL langsung ke file .bin</div>

    <h3>Update via File</h3>
    <label for="fileInput">Firmware File (.bin)</label>
    <input id="fileInput" type="file" accept=".bin" />
    <button id="fileUpdateBtn">Update via File</button>

    <button onclick="location.href='/'">Homepage</button>
  </div>

  <div id="popup">
    <div class="box">
      <div id="popupTitle" style="font-size:1.1em; margin-bottom:8px;">Update</div>
      <div id="popupBody"></div>
      <div class="btn-row" id="popupButtons"></div>
    </div>
  </div>

  <script>
    const urlUpdateBtn = document.getElementById('urlUpdateBtn');
    const fileUpdateBtn = document.getElementById('fileUpdateBtn');
    const urlInput = document.getElementById('urlInput');
    const md5Input = document.getElementById('md5Input');
    const fileInput = document.getElementById('fileInput');
    const popup = document.getElementById('popup');
    const popupTitle = document.getElementById('popupTitle');
    const popupBody = document.getElementById('popupBody');
    const popupButtons = document.getElementById('popupButtons');

    let pollTimer = null;

    function showPopup(htmlBody, buttons = []) {
      popupBody.innerHTML = htmlBody;
      popupButtons.innerHTML = '';
      buttons.forEach(btn => popupButtons.appendChild(btn));
      popup.style.display = 'flex';
    }
    function hidePopup() {
      popup.style.display = 'none';
      popupBody.innerHTML = '';
      popupButtons.innerHTML = '';
    }
    function createButton(text, className, onClick) {
      const b = document.createElement('button');
      b.textContent = text;
      b.className = className;
      b.onclick = onClick;
      return b;
    }

    function setBusy(isBusy) {
      urlUpdateBtn.disabled = isBusy;
      fileUpdateBtn.disabled = isBusy;
    }

    async function postStartUpdate() {
      const url = urlInput.value.trim();
      const md5 = md5Input.value.trim();

      if (!url) {
        showPopup('<div style="color:#d32f2f;">URL wajib diisi.</div>', [
          createButton('OK', 'btn-blue', hidePopup)
        ]);
        return;
      }

      setBusy(true);

      try {
        const params = new URLSearchParams();
        params.append('url', url);
        if (md5) params.append('md5', md5);

        const res = await fetch('/startupdate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: params.toString()
        });

        if (!res.ok) {
          const txt = await res.text();
          showPopup(`<div style="color:#d32f2f;">Gagal mulai update: ${txt || res.status}</div>`, [
            createButton('OK', 'btn-blue', hidePopup)
          ]);
          setBusy(false);
          return;
        }

        // sukses → tampilkan loading + mulai polling
        popupTitle.textContent = 'Updating...';
        showPopup('<div class="loader"></div><div id="progressText">Loading...</div>');

        startPolling();
      } catch (e) {
        showPopup(`<div style="color:#d32f2f;">Error: ${e.message}</div>`, [
          createButton('OK', 'btn-blue', hidePopup)
        ]);
        setBusy(false);
      }
    }

    async function postUpdateViaFile() {
      const file = fileInput.files && fileInput.files[0];

      if (!file) {
        showPopup('<div style="color:#d32f2f;">File wajib dipilih.</div>', [
          createButton('OK', 'btn-blue', hidePopup)
        ]);
        return;
      }

      const fileName = (file.name || '').toLowerCase();
      if (!fileName.endsWith('.bin')) {
        showPopup('<div style="color:#d32f2f;">Extensi file harus <b>.bin</b>.</div>', [
          createButton('OK', 'btn-blue', hidePopup)
        ]);
        return;
      }

      setBusy(true);

      try {
        popupTitle.textContent = 'Uploading...';
        showPopup('<div class="loader"></div><div id="progressText">Loading...</div>');

        const form = new FormData();
        form.append('firmware', file, file.name);

        await new Promise((resolve, reject) => {
          const xhr = new XMLHttpRequest();
          xhr.open('POST', '/updateViaFile', true);
          xhr.setRequestHeader('X-File-Size', String(file.size));

          xhr.upload.onprogress = (evt) => {
            const progressText = document.getElementById('progressText');
            if (!progressText) return;
            if (evt.lengthComputable) {
              const percent = Math.floor((evt.loaded * 100) / evt.total);
              progressText.textContent = `Progress: ${percent}%`;
            } else {
              progressText.textContent = `Uploaded: ${Math.floor(evt.loaded / 1024)} KB`;
            }
          };

          xhr.onload = () => {
            if (xhr.status >= 200 && xhr.status < 300) {
              resolve();
            } else {
              reject(new Error(xhr.responseText || `HTTP ${xhr.status}`));
            }
          };
          xhr.onerror = () => reject(new Error('Network error'));
          xhr.send(form);
        });

        // upload selesai → lanjut polling status update (reuse UX yang sama)
        popupTitle.textContent = 'Updating...';
        startPolling();
      } catch (e) {
        if (pollTimer) {
          clearInterval(pollTimer);
          pollTimer = null;
        }
        showPopup(`<div style="color:#d32f2f;">Gagal update via file: ${e.message}</div>`, [
          createButton('OK', 'btn-blue', hidePopup)
        ]);
        setBusy(false);
      }
    }

    function startPolling() {
      if (pollTimer) clearInterval(pollTimer);
      pollTimer = setInterval(async () => {
        try {
          const res = await fetch('/updateinfo');
          if (!res.ok) throw new Error(`HTTP ${res.status}`);
          const data = await res.json();

          const percent = data.percent ?? 0;
          const done = data.done ?? 'wait';

          const progressText = document.getElementById('progressText');
          if (progressText) {
            progressText.textContent = `Progress: ${percent}%`;
          }

          if (done === 'wait') {
            // keep loading
            return;
          }

          clearInterval(pollTimer);
          pollTimer = null;

          if (done === 'ok') {
            popupTitle.textContent = 'Update Selesai';
            showPopup(
              '<div style="color:#28a745; margin-bottom:8px;">Device berhasil diupdate, klik reboot</div>',
              [
                createButton('Reboot', 'btn-red', async () => {
                  try { await fetch('/reboot', { method: 'POST' }); } catch (_) {}
                  hidePopup();
                })
              ]
            );
          } else if (done === 'fail') {
            popupTitle.textContent = 'Update Gagal';
            showPopup(
              '<div style="color:#d32f2f; margin-bottom:8px;">Update gagal silahkan coba lagi</div>',
              [createButton('OK', 'btn-blue', hidePopup)]
            );
          }
          setBusy(false);
        } catch (e) {
          clearInterval(pollTimer);
          pollTimer = null;
          popupTitle.textContent = 'Update Gagal';
          showPopup(
            `<div style="color:#d32f2f;">Gagal cek status: ${e.message}</div>`,
            [createButton('OK', 'btn-blue', hidePopup)]
          );
          setBusy(false);
        }
      }, 1000);
    }

    urlUpdateBtn.addEventListener('click', postStartUpdate);
    fileUpdateBtn.addEventListener('click', postUpdateViaFile);
  </script>
</body>
</html>


)rawliteral";


const char noUpdatePage[] PROGMEM = R"rawliteral(


<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>{{OTA_NAME}}</title>
  <style>
    body {
      background: #f5f7fa;
      font-family: 'Segoe UI', Arial, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      margin: 0;
    }
    .container {
      background: #fff;
      margin-top: 60px;
      padding: 32px 28px 24px 28px;
      border-radius: 12px;
      box-shadow: 0 4px 24px rgba(0, 0, 0, 0.08);
      min-width: 320px;
      max-width: 420px;
      width: 100%;
      box-sizing: border-box;
    }
    h2 {
      margin-top: 0;
      color: #222;
      font-weight: 600;
      text-align: center;
    }
    .hint {
      color: #666;
      font-size: 1em;
      text-align: center;
      margin-top: 8px;
      line-height: 1.4;
    }
    button {
      background: #0078d7;
      color: #fff;
      border: none;
      border-radius: 6px;
      padding: 12px 24px;
      font-size: 1rem;
      cursor: pointer;
      margin-top: 18px;
      transition: background 0.2s;
      display: block;
      width: 100%;
    }
    button:hover:enabled {
      background: #005fa3;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>Firmware Update</h2>
    <div class="hint">Belum terhubung ke wifi, silahkan setup wifi dulu</div>
    <button onclick="location.href='/'">Home</button>
  </div>
</body>
</html>

)rawliteral";