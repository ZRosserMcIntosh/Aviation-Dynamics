// Aviation Dynamics — headless-browser test of the bundled cockpit page.
// SPDX-License-Identifier: MIT
// Usage: node web/cockpit/browser_test.mjs [web/dist/cockpit.html]   (needs the `playwright` package)
// Loads the self-contained page in Chromium and checks that it boots, that the
// panel reads the C++ instruments, and that knobs and failures act on them.
import { createRequire } from 'node:module';
import { execSync } from 'node:child_process';
import { readFile, writeFile, mkdtemp } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
let playwright;
try { playwright = require('playwright'); } catch {
  playwright = require(path.join(execSync('npm root -g').toString().trim(), 'playwright'));
}
const here = path.dirname(fileURLToPath(import.meta.url));
const pagePath = path.resolve(process.argv[2] ?? path.join(here, '..', 'dist', 'cockpit.html'));
// The artifact host wraps the page body in a document skeleton; do the same.
const body = await readFile(pagePath, 'utf8');
const dir = await mkdtemp(path.join(tmpdir(), 'avd-cockpit-'));
const file = path.join(dir, 'page.html');
await writeFile(file, `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{margin:0}[hidden]{display:none!important}</style></head><body>${body}</body></html>`);

let failures = 0;
const check = (ok, what) => { console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}`); if (!ok) ++failures; };
const launch = {};
if (process.env.PLAYWRIGHT_CHROMIUM) launch.executablePath = process.env.PLAYWRIGHT_CHROMIUM;
const browser = await playwright.chromium.launch(launch);
try {
  const errors = [];
  const page = await browser.newPage({ viewport: { width: 1440, height: 1200 } });
  page.on('pageerror', (e) => errors.push(e.message));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  // Google Fonts may be unreachable in CI; serve nothing (the page has fallbacks).
  await page.route(/fonts\.(googleapis|gstatic)\.com/, (r) => r.fulfill({ status: 200, contentType: 'text/css', body: '' }));
  await page.goto('file://' + file);
  await page.waitForFunction(() => /Throttle/.test(document.getElementById('status')?.textContent ?? ''), null, { timeout: 15000 });
  check(true, 'page boots the WebAssembly flight model');
  check((await page.$$('#switches .switch')).length > 15, 'switch registry rendered from the C++ cockpit controls');

  // Truth table rows: [name, truth, shown, unit]
  const rows = async () => Object.fromEntries((await page.$$eval('#debug-body tr', (rs) =>
    rs.map((r) => Array.from(r.children).map((c) => c.textContent)))).map((r) => [r[0], r.slice(1)]));
  const num = (s) => parseFloat(s);
  await page.selectOption('#scenario', '3');
  await page.keyboard.press('KeyI');
  await page.waitForTimeout(6000);
  let t = await rows();
  const vs = t['Vertical speed · VSI'], alt = t['Altitude MSL · altimeter'], ias = t['Calibrated airspeed · ASI'];
  check(Math.abs(num(vs[0]) - num(vs[1])) < 400, `cruise: VSI ${vs[1]} vs true ${vs[0]} fpm`);
  check(Math.abs(num(alt[0]) - num(alt[1])) < 60, `cruise: altimeter ${alt[1]} vs true ${alt[0]} ft`);
  check(Math.abs(num(ias[0]) - num(ias[1])) < 6, `cruise: ASI ${ias[1]} vs CAS ${ias[0]} kt`);

  // Kollsman knob: hover it on the canvas and scroll ten detents (0.10 inHg ≈ 90-100 ft).
  const box = await (await page.$('#panel')).boundingBox();
  const s = Math.min(box.width / 1200, box.height / 600);
  await page.mouse.move(box.x + (box.width - 1200 * s) / 2 + 451 * s, box.y + (box.height - 600 * s) / 2 + 184 * s);
  check(/Kollsman/.test(await page.textContent('#interact')), 'Kollsman knob is discoverable under the pointer');
  const before = num((await rows())['Altitude MSL · altimeter'][1]) - num((await rows())['Altitude MSL · altimeter'][0]);
  for (let i = 0; i < 10; ++i) await page.mouse.wheel(0, -100);
  await page.waitForTimeout(800);
  t = await rows();
  const after = num(t['Altitude MSL · altimeter'][1]) - num(t['Altitude MSL · altimeter'][0]);
  check(after - before > 70 && after - before < 120, `Kollsman +0.10 inHg moves the altimeter ${(after - before).toFixed(0)} ft`);

  // Vacuum failure: the gyro rotors spin down (they do not snap to zero).
  await page.check('#fail-vac');
  await page.waitForTimeout(6000);
  t = await rows();
  const rotor = num(t['Gyro rotor: attitude'][0]);
  check(rotor < 99.5 && rotor > 50, `vacuum failure spins the attitude gyro down (${rotor}%)`);
  check((await page.$$eval('#log li', (l) => l.map((x) => x.textContent))).some((x) => /Vacuum pump: failed/.test(x)), 'failure is logged');

  // Final approach: localizer and glideslope are received on NAV 1.
  await page.selectOption('#scenario', '2');
  await page.waitForTimeout(1500);
  const nav = await page.evaluate(() => document.getElementById('nav-a').textContent);
  check(nav === '110.30', `NAV 1 active ${nav}`);

  // Phone width: no horizontal page scroll.
  await page.setViewportSize({ width: 400, height: 900 });
  await page.waitForTimeout(500);
  const sw = await page.evaluate(() => document.documentElement.scrollWidth);
  check(sw <= 400, `no horizontal scroll at 400 px (scrollWidth ${sw})`);

  check(errors.length === 0, `no page errors${errors.length ? ': ' + errors.join(' | ') : ''}`);
} finally {
  await browser.close();
}
process.exit(failures ? 1 : 0);
