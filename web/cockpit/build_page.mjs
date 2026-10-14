// Aviation Dynamics — bundle the cockpit into one self-contained HTML file.
// SPDX-License-Identifier: MIT
// Usage: node web/cockpit/build_page.mjs [dist-dir]   (after web/build_wasi.sh)
// Inlines avd.wasm (base64), the loader, the aircraft JSON and cockpit.js.
import { readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const dist = path.resolve(process.argv[2] ?? path.join(here, '..', 'dist'));
const [html, wasm, loader, json, app] = await Promise.all([
  readFile(path.join(here, 'cockpit.html'), 'utf8'),
  readFile(path.join(dist, 'avd.wasm')),
  readFile(path.join(dist, 'avd.mjs'), 'utf8'),
  readFile(path.join(dist, 'tupi_t180.json'), 'utf8'),
  readFile(path.join(here, 'cockpit.js'), 'utf8'),
]);
const safe = (s) => s.replaceAll('</script', '<\\/script');
const scripts = [
  `<script type="application/json" id="avd-aircraft">${safe(json)}</script>`,
  `<script type="application/octet-stream" id="avd-wasm">${wasm.toString('base64')}</script>`,
  `<script>${safe(app)}</script>`,
  `<script type="module">\n${safe(loader.replace('export default async function createAvdModule', 'async function createAvdModule'))}
const b64 = document.getElementById('avd-wasm').textContent;
const bytes = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
window.AVD_BOOT(createAvdModule, bytes, document.getElementById('avd-aircraft').textContent);
</script>`,
].join('\n');
const out = path.join(dist, 'cockpit.html');
await writeFile(out, html.replace('<!--AVD_SCRIPTS-->', scripts));
console.log(`${out}: ${(Buffer.byteLength(html) / 1024 + wasm.length * 4 / 3 / 1024).toFixed(0)} KiB`);
