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
for (let i = 0; i < 10; ++i) A.avd_advance(1);
c = A.cockpitState();
check(Math.abs(vs0 - c[53]) < 60, `final spawn: VSI reads the descent at once (${vs0.toFixed(0)} fpm)`);
check(Math.abs(c[0] - 65) < 2 && Math.abs(c[53] + 345) < 60, `final spawn trimmed: ${c[0].toFixed(1)} KIAS, ${c[53].toFixed(0)} fpm after 10 s hands-off`);
check(Math.abs(c[38]) < 0.1 && Math.abs(c[41]) < 0.2, `on the ILS: CDI ${c[38].toFixed(2)}, GS ${c[41].toFixed(2)}`);
// Wind physics through the page API: a hands-off approach flown into the
// microburst on final loses its airspeed and sinks far faster than the
// 3 deg path; moderate turbulence shakes the airspeed.
A.avd_set_wind_hazard(1);
A.avd_reset(2);
A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
let minIas = 1e9, maxSink = 0;
for (let i = 0; i < 75 * 4; ++i) {
  A.avd_advance(0.25);
  c = A.cockpitState();
  minIas = Math.min(minIas, c[49]);
  maxSink = Math.max(maxSink, -c[53]);
}
check(minIas < 55 && maxSink > 900, `microburst on final: CAS down to ${minIas.toFixed(1)} kt, sink ${maxSink.toFixed(0)} fpm`);
A.avd_set_wind_hazard(0);
A.avd_set_wind_detail(2, 0, 1500);
A.avd_reset(3);
A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
let lo = 1e9, hi = 0;
for (let i = 0; i < 20 * 4; ++i) {
  A.avd_advance(0.25);
  c = A.cockpitState();
  lo = Math.min(lo, c[49]);
  hi = Math.max(hi, c[49]);
}
check(hi - lo > 4, `moderate turbulence: CAS ${lo.toFixed(1)}-${hi.toFixed(1)} kt`);
A.avd_set_wind_detail(0, 2, 1500);  // thermals: just runs
A.avd_advance(1);
check(Number.isFinite(A.cockpitState()[49]), 'thermals over the city');
process.exit(failures ? 1 : 0);
