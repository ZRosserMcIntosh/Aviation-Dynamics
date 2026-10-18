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

A.avd_set_airport(0);                // Campo de Marte runway 12 (Congonhas is the default; checked below)
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
// Auto-trim (straight-and-level hold): a climbing turn, then hands off with
// less power: it levels off and holds the altitude it captures.
A.avd_reset(3);
A.avd_set_assist(2);
A.avd_set_level_hold(1);
for (let i = 0; i < 120; ++i) {
  A.avd_set_input(i < 90 ? 0.15 : 0, i < 60 ? 0.25 : 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
  A.avd_advance(1 / 60);
}
A.avd_set_input(0, 0, 0, A.avd_trim_throttle() * 0.85, A.avd_trim_mixture(), 0, 0, 0, 0);
for (let i = 0; i < 30; ++i) A.avd_advance(1);
const h0 = A.state()[53];
for (let i = 0; i < 30; ++i) A.avd_advance(1);
const held = A.state();
check(held[55] === 1 && Math.abs(held[53] - h0) < 15 && Math.abs(held[7]) < 2, `auto-trim holds straight and level (${(held[53] - h0).toFixed(1)} ft in 30 s, bank ${held[7].toFixed(1)} deg)`);
A.avd_set_assist(1);
A.avd_advance(0.1);
check(A.state()[55] === 1, 'auto-trim kept across preset changes');
A.avd_set_level_hold(0);
A.avd_advance(0.1);
check(A.state()[55] === 0, 'auto-trim off');
check(A.state()[57] === 0 && A.state()[56] === 0, 'the Tupi has no tiller: Q/E stay on the pedals');
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
B.avd_set_input(0, 0, 1, 0, 1, 1, 1, 0, 0);  // full right Q/E input, brakes on, at rest
B.avd_advance(0.5);
check(B.state()[57] === 1 && B.state()[56] === 1, 'jet: Q/E turn the nose-wheel tiller at walking pace');
B.avd_set_input(0, 0, 0, 0, 1, 0, 0, 0, 0);
B.avd_advance(4);  // tiller centred again (20 deg/s actuator) before the take-off run
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
j = B.jet();
check(j[32] > 95 && j[32] < 140 && j[34] > 75 && j[34] < j[33] && j[33] < j[32] && Math.abs(j[36]) < 25 && j[40] > 100,
  `jet: low-speed data (Vs ${j[32].toFixed(0)}, Vs TO ${j[33].toFixed(0)}, Vs LDG ${j[34].toFixed(0)} kt, FPA ${j[36].toFixed(1)}, RA ${j[40].toFixed(0)} ft)`);
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
// Load sheet: a planned load (full tanks, full cabin) applies at the next spawn.
B.avd_set_tank(0, 1e6); B.avd_set_tank(1, 1e6); B.avd_set_station(1, 2200);
const plan = B.loadsheet(true);
check(plan.tanks[0].litres === plan.tanks[0].capacity && (plan.flags & 3) === 3, `load sheet: full tanks and cabin plan ${Math.round(plan.gross)} kg, over ramp and MTOW flagged`);
B.avd_reset(1);
check(Math.abs(B.state()[40] - plan.gross) < 1 && Math.abs(B.loadsheet(false).gross - plan.gross) < 1, `the next spawn carries it (${Math.round(B.state()[40])} kg)`);
// Crosswind on final: the spawn crabs into it and tracks the localizer; the
// smoothed wind, track and drift are reported.
B.avd_set_weather(15 * 0.5144, 60, 0, 0, 0.6);
B.avd_reset(2);
for (let i = 0; i < 40; ++i) { B.avd_set_input(0, 0, 0, B.avd_trim_throttle(), 1, 0, 0, 0, 0); B.avd_advance(0.25); }
const xw = B.state(), xc = B.cockpitState();
check(Math.abs(xc[38]) < 0.3 && xw[62] < -3 && Math.abs(xw[59] - 60) < 25 && xw[60] > 8,
  `crosswind final: CDI ${xc[38].toFixed(2)}, drift ${xw[62].toFixed(1)} deg, wind ${xw[59].toFixed(0)}/${xw[60].toFixed(0)} kt, track ${xw[61].toFixed(0)}`);
const wg = B.windGrid(15, 15, 8, 8, 6, 4);
check(wg.length === 3 * 8 * 6 * 4 && wg.every(Number.isFinite), `wind grid around the aircraft (${wg.length / 3} points)`);
B.avd_set_weather(0, 0, 0, 0, 0.6);
B.avd_set_runway_water(8);
check(B.state()[63] === 8, 'standing water on the runway (8 mm)');
B.avd_set_runway_water(0);
// The default airport is Congonhas (runway 35L), with its test ILS on NAV 1.
const ap = B.airport();
check(ap.icao === 'SBSP' && ap.runway === '35L', `default airport ${ap.icao} runway ${ap.runway}`);
B.avd_reset(2);
for (let i = 0; i < 60; ++i) B.avd_advance(1 / 60);
const cg = B.cockpitState();
check(cg[32] === 110.5 && cg[37] === 1 && cg[40] === 1 && Math.abs(cg[38]) < 0.1, `Congonhas 35L final: ILS 110.50 received (CDI ${cg[38].toFixed(2)}, GS ${cg[41].toFixed(2)})`);
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
// The wake: shed behind the aircraft, rolled up, exported in ENU.
A.avd_set_wind_detail(0, 0, 1500);
A.avd_reset(3);
A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0);
const t0w = performance.now();
for (let i = 0; i < 15 * 4; ++i) A.avd_advance(0.25);
const wakeMs = (performance.now() - t0w) / 15;
const wk = A.wake();
let oldest = 0, finite = true;
for (let k = 0; k < wk.n; ++k) { for (let q = 0; q < 9; ++q) finite = finite && Number.isFinite(wk.seg[k * 9 + q]); oldest = Math.max(oldest, wk.seg[k * 9 + 8]); }
check(wk.n > 30 && finite && wk.gmax > 5 && wk.gmax < 80 && oldest > 10,
  `wake: ${wk.n} segments, max circulation ${wk.gmax.toFixed(1)} m2/s, oldest ${oldest.toFixed(1)} s; ${wakeMs.toFixed(1)} ms per simulated s`);
const wkAt = A.state();
const box = A.windBox([wkAt[0], wkAt[1], wkAt[2]], [800, 800, 200], 8, 8, 4);
check(box.length === 768 && box.every(Number.isFinite), 'wind box around the aircraft');
A.avd_set_wake(0);
check(A.wake().n === 0, 'wake off: nothing shed');
A.avd_set_wake(1);
process.exit(failures ? 1 : 0);
