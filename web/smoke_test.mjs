// Aviation Dynamics — Node smoke test of the WebAssembly build (web/dist).
// SPDX-License-Identifier: MIT
import { readFile } from 'node:fs/promises';
import createAvdModule from './dist/avd.mjs';

const A = await createAvdModule();
const json = await readFile(new URL('./dist/tupi_t180.json', import.meta.url), 'utf8');
const errors = A.load(json);
if (errors.length) { console.error(errors); process.exit(1); }
let failures = 0;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) failures++; };

A.avd_reset(1);                      // engine running on runway 12 at Campo de Marte
A.avd_set_assist(0);            // Arcade: wing leveler holds the torque roll
A.avd_set_input(0, 0, 0, 1.0, 0.85, 0, 0, 0, 0);
A.avd_cockpit_cycle(A.cockpit().find((c) => c.id === 'brakes.parking').index);
for (let i = 0; i < 25 * 60; ++i) {
  const s = A.state();
  const pitch = s[10] > 52 ? 0.35 : 0; // rotate at 52 KIAS
  A.avd_set_input(pitch, 0, 0, 1.0, 0.85, 0, 0, 0, 0);
  A.avd_advance(1 / 60);
}
const s = A.state();
check(s[28] === 0, `airborne after 25 s (AGL ${s[14].toFixed(0)} ft, ${s[10].toFixed(0)} KIAS)`);
check(s[30] === 1, 'engine running');
check(Math.abs(s[7]) < 20, `wings roughly level (roll ${s[7].toFixed(1)} deg)`);
check(A.cockpit().length > 8, `${A.cockpit().length} cockpit controls exposed`);
A.avd_reset(0);
check(A.state()[30] === 0, 'cold and dark reset');
A.avd_quick_start();
for (let i = 0; i < 20 * 60; ++i) A.avd_advance(1 / 60);
check(A.state()[30] === 1 && A.avd_procedure_status() === 2, 'Quick Start through the cockpit');
// Instruments through the ABI: on final for runway 12 the ILS needles are alive.
A.avd_reset(2);
for (let i = 0; i < 60; ++i) A.avd_advance(1 / 60);
let c = A.cockpitState();
check(c[37] === 1 && c[42] === 1, `localizer received (CDI ${c[38].toFixed(2)})`);
check(c[40] === 1, `glideslope received (${c[41].toFixed(2)})`);
check(Math.abs(c[0] - c[49]) < 6, `ASI ${c[0].toFixed(1)} KIAS vs CAS ${c[49].toFixed(1)}`);
const before = c[1];
A.avd_altimeter_adjust(100);
for (let i = 0; i < 60; ++i) A.avd_advance(1 / 60);
check(A.cockpitState()[1] - before > 800, 'Kollsman knob moves the altimeter');
check(A.markings()[6] === 163, 'VNE red line 163 KIAS from the panel data');
A.avd_reset(3);                      // airborne spawn: instruments seeded at the new position
A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
A.avd_advance(1);
c = A.cockpitState();
check(Math.abs(c[3] - c[53]) < 400 && Math.abs(c[1] - c[51]) < 40, `cruise spawn: VSI ${c[3].toFixed(0)} vs ${c[53].toFixed(0)} fpm, altimeter ${c[1].toFixed(0)} vs ${c[51].toFixed(0)} ft`);
// Trimmed final approach: hands-off it holds the 3 deg path on the ILS.
A.avd_reset(2);
A.avd_set_assist(2);
A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
A.avd_advance(0.25);
const vs0 = A.cockpitState()[3];
for (let i = 0; i < 40; ++i) A.avd_advance(0.25);  // 10 s (each call is clamped to 0.25 s)
c = A.cockpitState();
check(Math.abs(vs0 - c[53]) < 60, `final spawn: VSI reads the descent at once (${vs0.toFixed(0)} fpm)`);
check(Math.abs(c[0] - 65) < 2 && Math.abs(c[53] + 345) < 60, `final spawn trimmed: ${c[0].toFixed(1)} KIAS, ${c[53].toFixed(0)} fpm after 10 s hands-off`);
check(Math.abs(c[38]) < 0.1 && Math.abs(c[41]) < 0.2, `on the ILS: CDI ${c[38].toFixed(2)}, GS ${c[41].toFixed(2)}`);
// Spawn anywhere: over MASP, 600 m above the field, heading north at 100 KIAS.
A.avd_reset_air(0, 0, 600, 0, 100, 0);
A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
for (let i = 0; i < 40; ++i) A.avd_advance(0.25);  // 10 s (each call is clamped to 0.25 s)
c = A.cockpitState();
const st = A.state();
check(Math.abs(c[0] - 100) < 3 && Math.abs(c[53]) < 150, `spawn over MASP trimmed: ${c[0].toFixed(1)} KIAS, ${c[53].toFixed(0)} fpm after 10 s`);
check(Math.abs(st[0]) < 60 && st[1] > 400 && st[1] < 600 && Math.abs(st[14] - 600 / 0.3048) < 150,
  `spawn position: ${st[0].toFixed(0)} m E, ${st[1].toFixed(0)} m N, ${st[14].toFixed(0)} ft above the field`);
// A 135 deg roll: the attitude indicator follows (360 deg roll, no tumble) and is not stuck afterwards.
A.avd_set_assist(2);
A.avd_reset_air(0, 0, 900, 0, 110, 0);
for (let i = 0; i < 6; ++i) { A.avd_set_input(0, 1, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0); A.avd_advance(0.25); }
c = A.cockpitState();
check(c[6] === 0 && Math.abs(c[5] - c[55]) < 5, `steep roll: AI ${c[5].toFixed(0)} deg vs ${c[55].toFixed(0)} deg, not tumbled`);
A.avd_ai_cage();
check(A.cockpitState()[6] === 0, 'caging knob');

// ---- The Top G (turbofan, retractable gear, speedbrake) ----
const B = await createAvdModule();
const jetErrors = B.load(await readFile(new URL('./dist/top_g.json', import.meta.url), 'utf8'));
check(jetErrors.length === 0, `top_g.json loads${jetErrors.length ? ': ' + jetErrors.join('; ') : ''}`);
B.avd_set_assist(0);
B.avd_reset(1);
check(B.jet()[0] === 2 && B.jet()[13] === 1 && B.jet()[21] === 1, 'jet: two turbofans running at idle on the runway');
B.avd_cockpit_cycle(B.cockpit().find((x) => x.id === 'brakes.parking').index);
for (let i = 0; i < 40 * 4; ++i) {
  const js = B.state();
  B.avd_set_input(js[10] > 125 ? Math.max(-1, Math.min(1, 0.15 * (10 - js[8]))) : 0, 0, 0, 1, 1, 0, 0, 0, 0);
  B.avd_advance(0.25);
}
let j = B.jet(), js = B.state();
check(js[28] === 0 && js[14] > 200, `jet: airborne after 40 s at full thrust (AGL ${js[14].toFixed(0)} ft, N1 ${j[8].toFixed(0)} %)`);
B.avd_cockpit_cycle(B.cockpit().find((x) => x.id === 'gear.lever').index);
for (let i = 0; i < 40; ++i) B.avd_advance(0.25);
check(B.jet()[1] === 0, 'jet: gear retracts');
B.avd_set_assist(2);
B.avd_reset_air(0, 0, 3000, 0, 250, 0);
for (let i = 0; i < 40; ++i) { B.avd_set_input(0, 0, 0, B.avd_trim_throttle(), 1, 0, 0, 0, 0); B.avd_advance(0.25); }
c = B.cockpitState();
check(Math.abs(c[0] - 250) < 4 && Math.abs(c[53]) < 200 && B.jet()[1] === 0, `jet spawn trimmed: ${c[0].toFixed(1)} KIAS, ${c[53].toFixed(0)} fpm, gear up`);
B.avd_reset(0);
B.avd_quick_start();
for (let i = 0; i < 100 * 4; ++i) { B.avd_set_input(0, 0, 0, 0, 1, 0, 0, 0, 0); B.avd_advance(0.25); }
j = B.jet();
check(j[13] === 1 && j[21] === 1 && B.procedure().status === 2, `jet Quick Start: both engines running (N2 ${j[9].toFixed(0)} / ${j[17].toFixed(0)} %)`);
process.exit(failures ? 1 : 0);
