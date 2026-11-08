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
check(A.abiVersion === A.avd_abi_version() && A.layout.state === 64 && A.layout.flow === 1352, `ABI version ${A.abiVersion}, layout ${JSON.stringify(A.layout)}`);
check(A.buildId().length > 0, `build id "${A.buildId()}"`);
check(A.taws().cls === 1, 'TAWS: Class B in the 172');

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
// A G1000 172: the attitude and heading from an AHRS. Four minutes after a
// vacuum failure (its gyro would be down to a third of its speed and frozen)
// a 30 deg bank is shown right (rolling, within the AHRS's 0.05 s filter
// lag), no flag, no tumble; back to the data's gyros.
{
  A.avd_reset_air(0, 0, 900, 0, 110, 0);
  check(A.setAttitudeReference('ahrs') === 'ahrs' && A.attitude().ahrs && !A.attitude().attFlag, 'G1000 172: AHRS attitude, aligned');
  A.avd_failure(2, 1);
  A.avd_set_assist(2);
  for (let t = 0; t < 240; t += 0.25) { A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0); A.avd_advance(0.25); }
  A.avd_set_assist(0);
  for (let i = 0; i < 12 && Math.abs(A.cockpitState()[55]) < 30; ++i) { A.avd_set_input(0, 0.4, 0, A.avd_trim_throttle(), A.avd_trim_mixture(), 0, 0, 0, 0); A.avd_advance(0.25); }
  c = A.cockpitState();
  const at = A.attitude();
  check(Math.abs(c[5] - c[55]) < 2 && Math.abs(c[55]) > 15 && !at.attFlag && !at.tumbled && c[6] === 0,
    `G1000 172, vacuum failed 4 min: AI ${c[5].toFixed(1)} deg vs ${c[55].toFixed(1)} deg, no flag, no tumble`);
  A.avd_failure(2, 0);
  check(A.setAttitudeReference('data') === 'gyro' && !A.attitude().ahrs, 'back to the vacuum gyros');
}

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
B.avd_set_input(0, 0, 0, 0, 1, 1, 1, 0, 0);
B.setReverse(1);  // the reverse levers at idle on the ground: deployed, reverse N1
for (let t = 0; t < 8; t += 0.25) B.avd_advance(0.25);
{
  const jr = B.jet();
  // Deployed; at rest the N1 stays at reverse idle (no power-back).
  check(jr[52] > 0.99 && jr[53] === 4 && jr[54] > 0.99 && jr[55] === 4 && jr[8] < 30, `reverse levers: deployed (phase ${jr[53]}), N1 ${jr[8].toFixed(0)} % at rest`);
}
B.setReverse(0);
for (let t = 0; t < 8; t += 0.25) B.avd_advance(0.25);
check(B.jet()[52] === 0 && B.jet()[53] === 0, 'reverse levers down: stowed');
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
// Bluff bodies: none with the gear up and the speedbrake stowed; raised
// spoiler panels each one, and the flow behind them separated.
check(B.bodies().n === 0, 'jet: no bluff bodies clean');
const sbIdx = B.cockpit().find((x) => x.id === 'speedbrake.lever').index;
for (let k = 0; k < 2; ++k) B.avd_cockpit_cycle(sbIdx);
for (let i = 0; i < 12; ++i) B.avd_advance(0.25);
const bod = B.bodies(), fl = B.flow();
let spoilers = 0, cleanly = true, separated = 0;
for (let k = 0; k < bod.n; ++k) { spoilers += bod.b[k * 10 + 9] === 1; cleanly = cleanly && bod.b[k * 10 + 8] > 0 && bod.b[k * 10 + 6] > 0; }
for (let i = 0; i < (fl[0] | 0); ++i) separated += fl[16 + i * 20 + 16] < 0.6;
check(spoilers >= 4 && cleanly && separated >= 4, `jet speedbrake: ${spoilers} spoiler panels in the flow, ${separated} strips separated`);
B.avd_reset(0);
B.avd_quick_start();
for (let i = 0; i < 100 * 4; ++i) { B.avd_set_input(0, 0, 0, 0, 1, 0, 0, 0, 0); B.avd_advance(0.25); }
j = B.jet();
check(j[13] === 1 && j[21] === 1 && B.procedure().status === 2, `jet Quick Start: both engines running (N2 ${j[9].toFixed(0)} / ${j[17].toFixed(0)} %)`);
// Systems through the ABI: both generators on line, both hydraulic systems
// pressurised, every actuator powered, the accumulator full; one lever
// retarded splits the throttles (and avd_set_input joins them again).
check(j[60] === 1 && j[61] === 1 && j[62] > 2500 && j[63] > 2500 && j[64] > 0.99 && j[68] === 1 && j[69] === 6 && j[71] === 1,
  `jet systems: GEN ${j[60]}/${j[61]}, HYD A ${j[62].toFixed(0)} / B ${j[63].toFixed(0)} psi, accumulator ${j[69]}`);
B.setThrottle(1, 0.6);
for (let i = 0; i < 16; ++i) B.avd_advance(0.25);
j = B.jet();
check(j[58] === 0 && j[59] === 0.6 && j[16] > j[8] + 10 && j[74 + 4] > j[74] + 10,
  `split throttles: levers ${j[58]} / ${j[59]}, N1 indicated ${j[8].toFixed(0)} / ${j[16].toFixed(0)} %, truth ${j[74].toFixed(0)} / ${j[78].toFixed(0)} %`);
B.avd_set_input(0, 0, 0, 0, 1, 0, 0, 0, 0);
B.avd_advance(0.25);
check(B.jet()[59] === 0, 'avd_set_input joins the levers again');
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
// The default airport is Congonhas (runway 35L), with its ILS (ISO) on NAV 1.
const ap = B.airport();
check(ap.icao === 'SBSP' && ap.runway === '35L', `default airport ${ap.icao} runway ${ap.runway}`);
B.avd_reset(2);
for (let i = 0; i < 60; ++i) B.avd_advance(1 / 60);
const cg = B.cockpitState();
check(cg[32] === 109.7 && cg[37] === 1 && cg[40] === 1 && Math.abs(cg[38]) < 0.1, `Congonhas 35L final: ILS ISO 109.70 received (CDI ${cg[38].toFixed(2)}, GS ${cg[41].toFixed(2)})`);
// Destinations: Guarulhos 10L. On the departure runway the route starts
// there and NAV 1 is already on IUC; at the intermediate fix the localizer is
// centred and the glideslope above; the 172 flown hands-off down the final
// lands on Guarulhos' own elevation (750 m, not Congonhas' 802 m).
{
  const dl = B.destinations();
  const gi = dl.findIndex((d) => d.icao === 'SBGR');
  check(dl.length === 6 && gi >= 0 && dl[gi].ends[0].ils === 'IUC' && dl[gi].ends[0].khz === 110700, `destinations: ${dl.map((d) => d.icao).join(' ')}; SBGR 10L ${dl[gi].ends[0].ils} ${dl[gi].ends[0].khz}`);
  B.avd_set_destination(gi, 0);
  B.avd_reset(1);
  let r = B.route();
  check(r.wp[0].ident === 'RW35L' && r.wp.at(-1).ident === 'RW10L' && B.cockpitState()[32] === 110.7, `departure route ${r.wp.map((w) => w.ident).join(' ')}, NAV 1 ${B.cockpitState()[32]}`);
  B.avd_reset(4);
  for (let i = 0; i < 120; ++i) B.avd_advance(1 / 60);
  const c4 = B.cockpitState(), g4 = B.gps();
  r = B.route();
  // (The glideslope's 10 NM coverage starts just inside the IF.)
  check(c4[37] === 1 && c4[42] === 1 && Math.abs(c4[38]) < 0.15 && g4[0] === 1 && r.wp[g4[1]].ident === 'FF10L' && Math.abs(g4[4]) < 100,
    `SBGR ILS 10L at the IF: LOC CDI ${c4[38].toFixed(2)}, GS ${c4[40] ? c4[41].toFixed(2) : 'not yet'}; GPS ${r.wp.map((w) => w.ident).join(' ')} to ${r.wp[g4[1]]?.ident} ${(g4[7] / 1852).toFixed(1)} NM, XTK ${g4[4].toFixed(0)} m`);
  check(Math.abs(B.avd_ground_height(r.wp.at(-1).east, r.wp.at(-1).north) - 750.1) < 0.5, `SBGR ground at ${B.avd_ground_height(r.wp.at(-1).east, r.wp.at(-1).north).toFixed(1)} m`);
  B.avd_reset(5);
  let landed = false;
  for (let i = 0; i < 60 * 240 && !landed; ++i) {
    B.avd_set_input(0, 0, 0, i < 60 * 150 ? B.avd_trim_throttle() : 0, 1, 0, 0, 0, 0);
    B.avd_advance(1 / 60);
    landed = B.state()[28] === 1;
  }
  const s5 = B.state();
  // The CG about 3 m above the wheels at touchdown (the oleos at full extension).
  check(landed && !s5[48] && Math.abs(s5[2] - 750.1) < 4, `hands-off 172 on the SBGR 10L final: ${landed ? 'touched down' : 'airborne'} at ${s5[2].toFixed(1)} m, crashed ${s5[48]}`);
  B.avd_set_destination(-1, 0);
}
// Autoland: engaged on the Top G's Congonhas 35L final (no destination: the
// runway in use becomes it), it flies the GPS path, flares and stops on the
// centreline; refused on the ground.
{
  B.avd_set_airport(1);
  B.avd_reset(1);
  check(B.autoland(true) === -2, 'autoland refused on the ground');
  B.avd_reset(2);
  const engaged = B.autoland(true);
  let st = B.autolandStatus(), t = 0, sawFlare = false;
  for (; t < 400 && st.on && st.phase !== 4; t += 0.25) { B.avd_advance(0.25); st = B.autolandStatus(); sawFlare = sawFlare || st.phase === 2; }
  const s6 = B.state().slice(), rw = B.runway(), h = rw[3] * Math.PI / 180;
  const dx = s6[0] - rw[0], dy = s6[1] - rw[1];
  const along = dx * Math.sin(h) + dy * Math.cos(h), right = dx * Math.cos(h) - dy * Math.sin(h);
  check(engaged === 1 && st.phase === 4 && sawFlare && !s6[48] && Math.abs(right) < 4 && along > 300 && along < rw[4],
    `autoland: engaged ${engaged}, phase ${st.phase} after ${t.toFixed(0)} s, stopped ${along.toFixed(0)} m down 35L, ${right.toFixed(1)} m right`);
  check(B.autoland(false) === 0 && !B.autolandStatus().on, 'autoland off');
  // Handed back with the levers where it left them (idle), not where the host's were.
  B.avd_advance(0.25);
  check(B.state()[24] < 0.02, `autoland hand-back: the throttle at ${B.state()[24].toFixed(2)}`);
  // The dummy in the pilot's seat rode an ordinary landing: no limit near, nothing broken.
  const o = B.occupant();
  check(o.peakG > 1 && o.peakG < 3 && o.dri > 1 && o.dri < 6 && o.hic < 50 && !o.collapsed.some((c) => c) && !o.structural,
    `occupant after the autoland: peak ${o.peakG.toFixed(2)} g, DRI ${o.dri.toFixed(2)}, HIC ${o.hic.toFixed(1)}, lumbar ${o.lumbarN.toFixed(0)} N`);
}
// The region's terrain (data/world/terrain_sp.avt): Pico do Jaraguá rises
// from the blend between the fields; the runway in use stays at its elevation.
{
  const rw0 = B.runway(), field = B.avd_ground_height(rw0[0], rw0[1]), before = B.avd_ground_height(-11414, 11504);
  const bytes = await readFile(new URL('../data/world/terrain_sp.avt', import.meta.url));
  const n = B.terrain(bytes);
  const peak = B.avd_ground_height(-11414, 11504), after = B.avd_ground_height(rw0[0], rw0[1]);
  check(n === 1101 * 901 && peak > 950 && peak < 1140 && Math.abs(after - field) < 1e-6,
    `terrain: ${n} samples; Pico do Jaraguá ${peak.toFixed(0)} m (the blend gave ${before.toFixed(0)}), the runway ${after.toFixed(1)} m as before`);
  check(B.terrain(new Uint8Array(16)) === -1, 'terrain: a malformed file is refused');
  // The city's 50 m grid nested in it (data/world/terrain_sp_city.avt): the
  // summit itself (SRTM 1,132 m at -23.4592, -46.7669), the runway unchanged.
  const summitE = -11462, summitN = 11227, coarse = B.avd_ground_height(summitE, summitN);
  const city = await readFile(new URL('../data/world/terrain_sp_city.avt', import.meta.url));
  const nd = B.terrainDetail(city);
  const fine = B.avd_ground_height(summitE, summitN), runwayNow = B.avd_ground_height(rw0[0], rw0[1]);
  check(nd === 1001 * 1241 && fine > 1100 && fine > coarse + 40 && Math.abs(runwayNow - field) < 1e-6,
    `terrain detail: ${nd} samples; the Jaraguá summit ${fine.toFixed(0)} m (region grid ${coarse.toFixed(0)}), the runway as before`);
  check(B.terrainDetail(new Uint8Array(16)) === -1, 'terrain detail: a malformed file is refused');
}
// TAWS: the Top G (Class A) level at 1,067 m MSL toward the Jaraguá summit at
// 220 kt, on the autopilot's ROL and ALT: CAUTION TERRAIN, then TERRAIN AHEAD,
// PULL UP, each spoken (a new voiceCount with the phrase), tens of seconds out.
{
  const rw0 = B.runway(), field = B.avd_ground_height(rw0[0], rw0[1]);
  const summitE = -11462, summitN = 11227, hdg = 340 * Math.PI / 180;
  B.avd_reset_air(summitE - 12000 * Math.sin(hdg), summitN - 12000 * Math.cos(hdg), 1067 - field, 340, 220, 0);
  const t0 = B.taws();
  check(t0.cls === 2 && t0.active.length === 0, `TAWS: class ${t0.cls} (A), quiet at the spawn (radio altitude ${t0.raFt.toFixed(0)} ft)`);
  const alt = B.state()[13];
  B.autopilot({ lateral: 1, vertical: 2, altFt: alt });
  let caution = -1, warning = -1, impact = -1, count = t0.voiceCount;
  const said = [];
  for (let t = 0; t < 140 && impact < 0; t += 0.25) {
    B.avd_advance(0.25);
    const w = B.taws(), s = B.state();
    if (caution < 0 && w.active.includes(6)) caution = t;
    if (warning < 0 && w.active.includes(3)) warning = t;
    if (w.voiceCount !== count) { count = w.voiceCount; if (said.at(-1) !== w.say) said.push(w.say); }
    if (s[48] || B.avd_ground_height(s[0], s[1]) > s[2] - 10) impact = t;
  }
  check(caution > 0 && warning > caution && impact - caution > 25 && impact - warning > 15 && said.indexOf('CAUTION TERRAIN, CAUTION TERRAIN') >= 0 &&
        said.indexOf('TERRAIN AHEAD, PULL UP') > said.indexOf('CAUTION TERRAIN, CAUTION TERRAIN'),
    `TAWS: CAUTION TERRAIN ${(impact - caution).toFixed(0)} s and TERRAIN AHEAD PULL UP ${(impact - warning).toFixed(0)} s before the hill; said: ${said.join(' / ')}`);
  B.autopilot({});
  check(B.tawsConfig({ inhibit: true }) === 2 && B.taws().inhibit && B.tawsConfig({}) === 2 && !B.taws().inhibit, 'TAWS: terrain inhibit on and off');
  check(B.tawsVoiceText(3) === 'TERRAIN, TERRAIN' && B.tawsAlertText(9) === 'TOO LOW GEAR', 'TAWS: phrase and alert texts');
}
// Autopilot: refused on the ground; on the Top G's final, HDG on the bug 60 deg
// right, ALT on the present altitude, the autothrottle 10 kt faster.
{
  B.avd_reset(1);
  check(B.autopilot({ lateral: 2, vertical: 2 }) === -2, 'autopilot refused on the ground');
  B.avd_reset(2);
  const s0 = B.state().slice();
  const altFt = s0[13], hdg0 = s0[37];  // the altimeter, the heading indicator
  B.avd_heading_bug((hdg0 + 60) % 360);
  const ias = Math.round(s0[10]) + 10;
  const r = B.autopilot({ lateral: 2, vertical: 2, autothrottle: true, altFt, vsFpm: 0, iasKt: ias });
  let worstAlt = 0;
  for (let t = 0; t < 90; t += 0.25) { B.avd_advance(0.25); worstAlt = Math.max(worstAlt, Math.abs(B.state()[13] - altFt)); }
  const st = B.autopilotStatus(), s1 = B.state();
  const off = ((s1[37] - (hdg0 + 60) + 540) % 360) - 180;
  check(r === 1 && st.on && st.lateral === 2 && st.vertical === 2 && Math.abs(off) < 3 && worstAlt < 120 && Math.abs(s1[10] - ias) < 5,
    `autopilot: HDG ${off.toFixed(1)} deg off the bug after 90 s, altitude within ${worstAlt.toFixed(0)} ft, ${s1[10].toFixed(0)} kt on ${ias}`);
  check(B.autopilot({}) === 0 && !B.autopilotStatus().on, 'autopilot off');
}
// APR: the Top G 10 NM out on the Congonhas 35L final, level at the path's
// altitude, ILS ISO on NAV 1: the localizer and glideslope captured, flown to
// DA (200 ft above the field), where the autopilot disconnects (the
// autothrottle stays). The jet's attitude is its AHRS: no ATT flag.
{
  B.avd_reset(4);
  const s0 = B.state().slice();
  check(B.attitude().ahrs && !B.attitude().attFlag, 'Top G: AHRS attitude, aligned');
  B.avd_heading_bug(s0[37]);
  const set = B.autopilotApproach({ source: 'ils' });
  const r = B.autopilot({ lateral: 4, vertical: 2, autothrottle: true, altFt: s0[13], vsFpm: 0, iasKt: Math.round(s0[10]) });
  let st = B.autopilotStatus(), t = 0, sawLoc = false, sawGs = false, worstLoc = 0, worstGs = 0;
  for (; t < 400 && st.apr !== 4; t += 0.25) {
    B.avd_advance(0.25);
    st = B.autopilotStatus();
    sawLoc = sawLoc || st.apr === 2;
    sawGs = sawGs || (st.apr === 3 && st.vertical === 5);
    if (st.apr === 3) { worstLoc = Math.max(worstLoc, Math.abs(st.locDots)); worstGs = Math.max(worstGs, Math.abs(st.gsDots)); }
  }
  const above = B.state()[14];
  check(set === 1 && r === 1 && sawLoc && sawGs && st.apr === 4 && st.on && st.lateral === 0 && st.vertical === 0 && st.autothrottle &&
    worstLoc < 1 && worstGs < 1 && st.daFt > 2750 && st.daFt < 2900,  // Congonhas 2,631 ft + 200
    `APR: ILS 35L captured, coupled within ${worstLoc.toFixed(2)} / ${worstGs.toFixed(2)} dots, MINIMUMS at DA ${st.daFt.toFixed(0)} ft after ${t.toFixed(0)} s (${above.toFixed(0)} ft above the ground), A/T on`);
  check(B.autopilot({}) === 0 && B.autopilotStatus().apr === 0, 'autopilot off after minimums');
}
// Anywhere in Brazil (data/world/airports_br.json): Galeão 15's ILS (IGL)
// at its 10 NM fix, 360 km away; a region airport's record maps onto the
// region's own; Florianópolis (sea level, 490 km) takes a hands-off landing.
{
  const { readFile } = await import('node:fs/promises');
  const br = JSON.parse(await readFile(new URL('../data/world/airports_br.json', import.meta.url), 'utf8')).airports;
  const rec = (icao) => br.find((a) => a.icao === icao);
  const endIndex = (r, id) => r.rw.flatMap((w) => w.ends).indexOf(id);
  const gr = rec('SBGR');
  check(B.setDestinationRecord(gr, endIndex(gr, '28L')) === 2 && B.avd_destination_runway() === 3, `SBGR record maps onto the region's (${B.avd_destination()}, ${B.avd_destination_runway()})`);
  const gl = rec('SBGL');
  const gi = B.setDestinationRecord(gl, endIndex(gl, '15'));
  B.avd_reset(4);
  for (let i = 0; i < 120; ++i) B.avd_advance(1 / 60);
  const cg = B.cockpitState(), r = B.route();
  check(gi === 6 && cg[32] === 110.3 && cg[37] === 1 && cg[42] === 1 && Math.abs(cg[38]) < 0.15 && r.wp.at(-1).ident === 'RW15',
    `SBGL ILS 15 at the IF, ${(Math.hypot(B.state()[0], B.state()[1]) / 1000).toFixed(0)} km out: NAV 1 ${cg[32]} LOC CDI ${cg[38].toFixed(2)}`);
  const fl = rec('SBFL');
  B.setDestinationRecord(fl, 0);
  B.avd_reset(5);
  let landed = false;
  for (let i = 0; i < 60 * 240 && !landed; ++i) {
    B.avd_set_input(0, 0, 0, i < 60 * 150 ? B.avd_trim_throttle() : 0, 1, 0, 0, 0, 0);
    B.avd_advance(1 / 60);
    landed = B.state()[28] === 1;
  }
  const s = B.state();
  check(landed && !s[48] && Math.abs(s[2] - fl.elev * 0.3048) < 4, `hands-off 172 at Florianópolis ${fl.rw[0].ends[0]}: ${landed ? 'touched down' : 'airborne'} at ${s[2].toFixed(1)} m (field ${(fl.elev * 0.3048).toFixed(1)} m)`);
  B.avd_set_destination(-1, 0);
}
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

// Buildings in the physics: an AVB1 tile (the flight test bed's format) with a
// 60 m tower 300 m ahead of the aircraft, flown into at 30 m above its base.
{
  const at = A.state(), hdg = at[9] * Math.PI / 180;
  const te = at[0] + 300 * Math.sin(hdg), tn = at[1] + 300 * Math.cos(hdg);
  const ti = Math.floor(te / 1000), tj = Math.floor(tn / 1000);
  const x = Math.round((te - ti * 1000) * 10), y = Math.round((tn - tj * 1000) * 10), r = 300;  // dm; 60 m square
  const blk = new DataView(new ArrayBuffer(8 + 6 + 2 + 4 * 4));
  [65, 86, 66, 49].forEach((c, k) => blk.setUint8(k, c));
  blk.setUint32(4, 1, true);
  blk.setUint16(8, 600, true); blk.setUint16(10, 0, true); blk.setUint8(12, 3); blk.setUint8(13, 1); blk.setUint16(14, 4, true);
  [[x - r, y - r], [x + r, y - r], [x + r, y + r], [x - r, y + r]].forEach(([px, py], k) => { blk.setInt16(16 + 4 * k, px, true); blk.setInt16(18 + 4 * k, py, true); });
  const ground = A.avd_ground_height(te, tn);
  A.avd_reset_air(at[0], at[1], ground + 30 - (at[13] - at[14]) * 0.3048, at[9], 120, 0);
  const n = A.buildingsTile(ti, tj, new Uint8Array(blk.buffer));
  check(n === 1 && A.avd_buildings_count() === 1, `buildings: a tile of ${n} building in the physics`);
  let strike = '';
  for (let t = 0; t < 20 && !strike; t += 0.25) {
    A.avd_set_input(0, 0, 0, 0.8, 1, 0, 0, 0, 0); A.avd_advance(0.25);
    for (const e of A.events()) if (e.startsWith('Building strike')) strike = e;
  }
  check(strike !== '', `buildings: flown into, ${strike || 'no strike'}`);
  A.avd_buildings_drop(ti, tj);
  check(A.avd_buildings_count() === 0, 'buildings: the tile dropped');
}

// ---- The King Air 350 (turboprops: power, propeller and condition levers) ----
{
  const K = await createAvdModule();
  const kErrors = K.load(await readFile(new URL('./dist/king_air_350.json', import.meta.url), 'utf8'));
  check(kErrors.length === 0, `king_air_350.json loads${kErrors.length ? ': ' + kErrors.join('; ') : ''}`);
  K.avd_set_assist(0);
  K.avd_reset(1);
  let tp = K.turboprop();
  check(tp.count === 2 && tp.engines[0].running && tp.engines[1].running && tp.engines[0].condition === 1 && K.state()[30] === 1,
    `turboprop: two PT6As at low idle (Ng ${tp.engines[0].ng.toFixed(0)} %, Np ${tp.engines[0].np.toFixed(0)} rpm, ITT ${tp.engines[0].itt.toFixed(0)} C)`);
  check(K.setCondition(-1, 2) === 2, 'condition levers to HIGH IDLE');
  for (let t = 0; t < 4; t += 0.25) K.avd_advance(0.25);
  tp = K.turboprop();
  check(Math.abs(tp.engines[0].ng - 72) < 1 && tp.engines[1].condition === 2, `high idle: Ng ${tp.engines[0].ng.toFixed(1)} %`);
  K.setCondition(-1, 1);
  K.avd_cockpit_cycle(K.cockpit().find((x) => x.id === 'brakes.parking').index);  // brakes off
  K.avd_set_input(0, 0, 0, 1, 1, 0, 0, 0, 0);  // power levers to the stop
  let climbed = false;
  for (let i = 0; i < 45 * 4; ++i) {
    const ks = K.state();
    K.avd_set_input(ks[10] > 100 ? Math.max(-1, Math.min(1, 0.12 * (10 - ks[8]))) : 0, 0, 0, 1, 1, 0, 0, 0, 0);
    K.avd_advance(0.25);
    if (i === 20 * 4) {
      tp = K.turboprop();
      check(Math.abs(tp.engines[0].torque - 100) < 3 && Math.abs(tp.engines[0].np - 1700) < 20 && tp.engines[0].itt < 820,
        `take-off power: torque ${tp.engines[0].torque.toFixed(0)} % (${tp.engines[0].torqueFtLb.toFixed(0)} ft lbf), Np ${tp.engines[0].np.toFixed(0)}, ITT ${tp.engines[0].itt.toFixed(0)} C, ${tp.engines[0].fuelFlowLbH.toFixed(0)} lb/h`);
    }
    climbed = climbed || (ks[28] === 0 && ks[14] > 300);
  }
  let ks = K.state();
  check(climbed && ks[28] === 0 && ks[15] > 1000, `King Air: off the runway and climbing (AGL ${ks[14].toFixed(0)} ft, ${ks[15].toFixed(0)} fpm, ${ks[10].toFixed(0)} KIAS)`);
  K.avd_cockpit_cycle(K.cockpit().find((x) => x.id === 'gear.lever').index);
  K.setPropLever(0.2);  // 1,500 rpm
  for (let i = 0; i < 40; ++i) { ks = K.state(); K.avd_set_input(Math.max(-1, Math.min(1, 0.12 * (8 - ks[8]))), 0, 0, 0.8, 1, 0, 0, 0, 0); K.avd_advance(0.25); }
  tp = K.turboprop();
  check(K.jet()[1] === 0 && Math.abs(tp.engines[0].np - 1500) < 15 && Math.abs(tp.engines[1].np - 1500) < 15 && Math.abs(tp.npSelected - 1500) < 1,
    `gear up; propeller levers to ${tp.npSelected.toFixed(0)} rpm: Np ${tp.engines[0].np.toFixed(0)} / ${tp.engines[1].np.toFixed(0)}`);
  K.setCondition(0, 0);  // engine 1 cut off
  K.setFeather(0, 1);
  for (let i = 0; i < 60; ++i) { ks = K.state(); K.avd_set_input(Math.max(-1, Math.min(1, 0.12 * (6 - ks[8]))), 0, 0, 0.8, 1, 0, 0, 0, 0); K.avd_advance(0.25); }
  tp = K.turboprop();
  check(!tp.engines[0].running && tp.engines[0].feathered && tp.engines[0].np < 100 && tp.engines[1].running,
    `engine 1 shut down and feathered (blade ${tp.engines[0].blade.toFixed(0)} deg, Np ${tp.engines[0].np.toFixed(0)})`);
  // Cold and dark, then Quick Start: starter, light-off, idle, one engine after the other.
  K.avd_reset(0);
  tp = K.turboprop();
  check(!tp.engines[0].running && tp.engines[0].feathered && tp.engines[0].condition === 0, 'King Air cold and dark: feathered, cut off');
  K.avd_quick_start();
  for (let i = 0; i < 100 * 4; ++i) { K.avd_set_input(0, 0, 0, 0, 1, 0, 0, 0, 0); K.avd_advance(0.25); }
  tp = K.turboprop();
  check(tp.engines[0].running && tp.engines[1].running && !tp.engines[0].feathered && K.procedure().status === 2 && K.jet()[60] === 1,
    `King Air Quick Start: both running (Ng ${tp.engines[0].ng.toFixed(0)} / ${tp.engines[1].ng.toFixed(0)} %), generators on line`);
  // Beta and reverse on the ground: power levers idle, reverse lever up.
  K.avd_reset(1);
  K.setReverse(1);
  for (let i = 0; i < 16; ++i) K.avd_advance(0.25);
  tp = K.turboprop();
  check(tp.engines[0].beta && tp.engines[0].blade < 0 && tp.engines[0].thrustKn < -1, `reverse: blade ${tp.engines[0].blade.toFixed(0)} deg, ${tp.engines[0].thrustKn.toFixed(1)} kN`);
  K.setReverse(0);
}

// Weather: airframe ice, carburettor ice and carb heat, rain on the runway, a front.
{
  const C = await createAvdModule();
  C.load(json);
  C.avd_set_airport(0);
  const fly = (seconds, thr) => { for (let t = 0; t < seconds; t += 0.25) { C.avd_set_input(0, 0, 0, thr, C.avd_trim_mixture(), 0, 0, 0, 0); C.avd_advance(0.25); } };
  // The C172 in a supercooled stratus at -5 C (0.5 g/m^3): ice on the wing and tail, the stall speed up.
  C.avd_set_assist(2);
  C.avd_set_level_hold(1);
  C.avd_reset_air(0, 0, 900, 120, 100, 0);
  const alt = C.state()[53] * 0.3048;
  C.avd_set_weather(0, 0, 0, -5 - (15 - 0.0065 * alt), 0.9);
  C.avd_set_cloud(alt - 600, alt + 600, 0.5);
  C.avd_reset_air(0, 0, 900, 120, 100, 0);
  const thr = C.avd_trim_throttle();
  const vs0 = C.icing().vsKcas;
  fly(300, thr);
  const ic = C.icing(), wx = C.weather();
  check(wx.inCloud && Math.abs(wx.oat + 5) < 2 && wx.supercooledWater > 0.4, `icing layer: OAT ${wx.oat.toFixed(1)} C, ${wx.supercooledWater.toFixed(2)} g/m3 supercooled, visibility ${wx.visibility.toFixed(0)} m`);
  check(ic.surfaces.length === 3 && ic.surfaces[0].mm > 0.5 && ic.surfaces[1].clmaxLoss > ic.surfaces[0].clmaxLoss * 0.9 && ic.windshieldOpacity > 0,
    `5 min at -5 C: wing ${ic.surfaces[0].mm.toFixed(1)} mm (clmax -${(100 * ic.surfaces[0].clmaxLoss).toFixed(0)} %), tail ${ic.surfaces[1].mm.toFixed(1)} mm, windshield ${(100 * ic.windshieldOpacity).toFixed(0)} %, ${ic.rateMmPerMin.toFixed(2)} mm/min (class ${ic.intensity})`);
  check(ic.vsKcas > ic.vsCleanKcas + 3 && ic.vsKcas > vs0 + 3 && Math.abs(C.jet()[32] - ic.vsKcas) < 0.5 && ic.stallAngleLossDeg > 1,
    `iced stall speed ${ic.vsKcas.toFixed(1)} KCAS against ${ic.vsCleanKcas.toFixed(1)} clean (stall angle -${ic.stallAngleLossDeg.toFixed(1)} deg)`);
  check(!ic.iceDetector && ic.switches === 0 && !C.cockpit().some((k) => k.id.startsWith('anti_ice.')), 'the C172 has no ice protection to switch on');
  // Carburettor ice in clear humid air at +10 C, then carb heat.
  C.avd_set_cloud(0, 0, 0);
  C.avd_set_weather(0, 0, 0, 10 - (15 - 0.0065 * alt), 0.9);
  C.avd_reset_air(0, 0, 900, 120, 100, 0);
  const thr2 = C.avd_trim_throttle();
  const map0 = C.state()[20];
  fly(600, thr2);
  const ci = C.icing(), map1 = C.state()[20];
  check(ci.carburettor && ci.carbIceG > 0.5 && map1 < map0 - 0.7 && ci.carbTempC < 0,
    `carburettor ice: ${ci.carbIceG.toFixed(2)} g after 10 min at +10 C, 90 % RH (throat ${ci.carbTempC.toFixed(0)} C), MAP ${map0.toFixed(1)} -> ${map1.toFixed(1)} inHg`);
  const heat = C.cockpit().find((k) => k.id === 'engine1.carb_heat');
  C.avd_cockpit_cycle(heat.index);
  let rough = 0;
  for (let t = 0; t < 120; t += 0.25) { fly(0.25, thr2); rough = Math.max(rough, C.icing().roughness); }
  const ch = C.icing();
  check(C.cockpit().find((k) => k.id === 'engine1.carb_heat').position === 1 && ch.carbIceG < 0.05 && rough > 0.1 && ch.carbHeat > 0.9,
    `carb heat: ${ch.carbIceG.toFixed(3)} g left after 2 min, roughness peak ${rough.toFixed(2)}, MAP ${C.state()[20].toFixed(1)} inHg`);
  // Rain on the field: the runway wets, the visibility falls.
  C.avd_set_weather(0, 0, 0, 0, 0.9);
  C.avd_reset(1);
  C.setRain(10);
  for (let t = 0; t < 600; t += 0.25) C.avd_advance(0.25);
  const rw = C.weather();
  check(rw.rain === 10 && rw.visibility < 6000 && C.state()[63] > 0.4 && C.state()[63] < 1.0 && Math.abs(rw.runwayWaterMm - C.state()[63]) < 1e-9,
    `rain 10 mm/h: runway water ${C.state()[63].toFixed(2)} mm after 10 min, visibility ${rw.visibility.toFixed(0)} m`);
  C.setRain(0);
  // A cold front over the field: the QNH up, the OAT down.
  C.avd_reset(1);
  const w0 = C.weather();
  C.setFront({ kind: 'cold', etaMin: 3 });
  const w1 = C.weather();
  for (let t = 0; t < 900; t += 0.25) C.avd_advance(0.25);
  const w2 = C.weather();
  check(w1.frontEtaMin > 2.5 && w1.frontEtaMin < 3.5 && w2.frontBehind > 0.9 && w2.fieldQnh > w0.fieldQnh + 2 && w2.fieldOat < w0.fieldOat - 4,
    `cold front: field QNH ${w0.fieldQnh.toFixed(1)} -> ${w2.fieldQnh.toFixed(1)} hPa, OAT ${w0.fieldOat.toFixed(1)} -> ${w2.fieldOat.toFixed(1)} C (ETA ${w1.frontEtaMin.toFixed(1)} min)`);
  C.setFront({ kind: 'none' });
  check(Math.abs(C.weather().fieldQnh - w0.fieldQnh) < 1e-6 && C.weather().frontBehind === -1, 'front cleared');
  // A jet's ice protection: the CJ4's wing anti-ice (bleed) and tail boots.
  const cj = await readFile(new URL('./dist/cj4.json', import.meta.url), 'utf8');
  check(C.load(cj).length === 0, 'CJ4 loaded');
  C.avd_reset_air(0, 0, 3000, 120, 200, 0);
  const ids = C.cockpit().filter((k) => k.id.startsWith('anti_ice.')).map((k) => k.id);
  check(ids.includes('anti_ice.wing') && ids.includes('anti_ice.tail'), `CJ4 ice protection switches: ${ids.join(', ')}`);
  C.avd_cockpit_cycle(C.cockpit().find((k) => k.id === 'anti_ice.wing').index);
  for (let t = 0; t < 30; t += 0.25) { C.avd_set_input(0, 0, 0, C.avd_trim_throttle(), 1, 0, 0, 0, 0); C.avd_advance(0.25); }
  const ji = C.icing();
  check(ji.bleed && ji.protectionHeat > 0.6 && (ji.protectionMask & 1) === 1 && ji.iceDetector, `CJ4 wing anti-ice on: bleed drawn, leading edge heat ${ji.protectionHeat.toFixed(2)}`);
}
// ---- Main-thread cost of the calls the site makes (web/README.md, "Cost per call") ----
// Air starts give the trimmed state they always gave (recorded from the build
// before the CLmax sweep was memoised and cut short: state[0..18], [24..27],
// the trim throttle), in a few ms when the requested speed is clear of
// 1.25 Vs; a dense building tile stands in a few ms.
{
  const same = (x, y) => Math.abs(x - y) <= 1e-9 * Math.max(1, Math.abs(y));
  const expect = {
    tupi_t180: [
      { args: [0, 0, 600, 0, 100, 0], thr: 0.4878728939711703, s: [0.0003137481141562628, 1.269703530952686, 1401.7372134331613, -0.7070392982248496, -0.009772363724735535, 0.009772561982731028, -0.7070391988463139, -1.5957815197366648e-05, 1.5837458153080595, 7.83269604198722e-06, 99.70659428879755, 107.15215106014185, 107.15215106014185, 4595.709707013627, 2132.4283648198298, 9.387408897727227e-07, 1.5837458103467523, -1.7619767508677017e-05, 0.9973353115726316, 0.4878728939711703, 0.8592018243433936, 0, -0.06389293319588032] },
      { args: [0, 0, 300, 120, 55, 2], thr: 0.38439852002457436, s: [1.0230685466733025, -0.5910310357222176, 1101.7907095286928, -0.9651768029698438, 0.010191636549637261, 0.03803357585842507, 0.2586180903586859, -6.441961410824637e-05, 4.5132555520477275, 119.99996926421636, 56.56622977920672, 60.38052588004798, 60.38052588004796, 3611.5133378396913, 1148.3155703175942, 1.4231664764949355e-06, 4.5132555390523885, 0.0003010691629727664, 0.9947166569316727, 0.38439852002457436, 0.8720239870919145, 20, -0.08334648082559146] },
    ],
    top_g: [
      { args: [3200, -2200, 330, 305, 220, 0], thr: 0.3537548403782448, s: [3186.691602585516, -2190.6795485506423, 1133.2283741522974, 0.30057851207911895, 0.027746636784302472, -0.00874848395353631, 0.9533132466571406, -3.261351513371032e-07, 3.3343007747102984, 304.9999999889585, 219.65379534530334, 232.30820626167443, 232.30820626167443, 3705.30384924298, 1153.459075831133, 1.5831145158525734e-05, 3.3343007361536814, -6.577720772555314e-09, 0.9961141031658124, 0.3537548403782448, 1, 0, 0.20204447740865641] },
      { args: [0, 0, 600, 0, 80, 0], thr: 0.33400560024068476, s: [0.001483691027431538, 15.887526580581783, 1404.7981506346302, -0.7048895457831058, -0.05595290872703124, 0.05595291272070683, -0.7048895458217127, -3.2692834904333513e-07, 9.077046771260173, 359.9999999709109, 147.39968637301385, 158.54739952344187, 158.54739952344187, 4588.646066960745, 2128.677454908023, 4.556868561209648e-06, 9.07704675499935, -2.2769789776931885e-08, 0.9852225492186235, 0.33400560024068476, 1, 0, 0.6515810997896074] },
    ],
  };
  for (const [name, cases] of Object.entries(expect)) {
    const M = await createAvdModule();
    M.load(await readFile(new URL(`./dist/${name}.json`, import.meta.url), 'utf8'));
    for (const want of cases) {
      for (let rep = 0; rep < 2; ++rep) {
        const t0 = performance.now();
        M.avd_reset_air(...want.args);
        const ms = performance.now() - t0;
        const st = M.state(), got = [...st.slice(0, 19), ...st.slice(24, 28)];
        const ok = got.every((v, k) => same(v, want.s[k])) && same(M.avd_trim_throttle(), want.thr);
        // The first start at a speed below 1.25 Vs sweeps CLmax to the stall (once per flap): the repeat is cheap.
        const fast = ms < 150 || (rep === 0 && want.s[10] > want.args[4] + 1);
        check(ok && fast, `air start ${name} (${want.args.join(', ')})${rep ? ' again' : ''}: ${ms.toFixed(1)} ms, ${got[10].toFixed(2)} KIAS, the recorded trimmed state${ok ? '' : ' NOT'}`);
      }
    }
  }
  // A dense 1 km tile (6,000 buildings, the city's densest are ~7,000) on the city's hills.
  const M = await createAvdModule();
  M.load(await readFile(new URL('./dist/tupi_t180.json', import.meta.url), 'utf8'));
  M.terrain(await readFile(new URL('../data/world/terrain_sp.avt', import.meta.url)));
  M.terrainDetail(await readFile(new URL('../data/world/terrain_sp_city.avt', import.meta.url)));
  const count = 6000, bytes = [65, 86, 66, 49, count & 0xff, count >> 8, 0, 0];
  let seed = 7;
  const rnd = (lo, hi) => { seed = (seed * 1664525 + 1013904223) >>> 0; return lo + (hi - lo) * seed / 4294967296; };
  const u16 = (v) => bytes.push(v & 0xff, (v >> 8) & 0xff);
  for (let b = 0; b < count; ++b) {
    u16(Math.round(rnd(30, 600))); u16(0); bytes.push(1, 1); u16(5);
    const cx = rnd(200, 9800), cy = rnd(200, 9800);
    for (let k = 0; k < 5; ++k) { u16(Math.round(cx + 80 * Math.cos(k * 1.2566))); u16(Math.round(cy + 80 * Math.sin(k * 1.2566))); }
  }
  const blk = new Uint8Array(bytes);
  M.buildingsTile(3, 3, blk);  // the first call compiles the code path
  M.avd_buildings_drop(3, 3);
  const times = [];
  let n = 0;
  for (const [i, j] of [[7, 7], [6, 7], [7, 6]]) { const t0 = performance.now(); n = M.buildingsTile(i, j, blk); times.push(performance.now() - t0); }
  check(n === count && M.avd_buildings_count() === 3 * count && Math.max(...times) < 15,
    `buildings: dense tiles of ${n} stood in ${times.map((t) => t.toFixed(1)).join(', ')} ms`);
}
process.exit(failures ? 1 : 0);
