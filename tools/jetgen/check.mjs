// Flight checks of generated jets in the web build: load, cruise trim, stall
// speeds, a stabilised final approach, a take-off. node tools/jetgen/check.mjs [ids]
import { readFile } from 'node:fs/promises';
import createAvdModule from '../../web/dist/avd.mjs';
const ids = process.argv.slice(2).length ? process.argv.slice(2) : ['top_g', 'phenom300', 'cj4', 'learjet45', 'crj700', 'b738'];
const SPEC = { top_g: [470, 410], phenom300: [464, 300], cj4: [451, 300], learjet45: [465, 330], crj700: [473, 370], b738: [460, 350] };
// ISA: KTAS at a flight level -> KCAS (subsonic compressible).
function casFromTas(ktas, fl) {
  const h = fl * 30.48, T = 288.15 - 0.0065 * Math.min(h, 11000), p = 101325 * (Math.min(T, 288.15 - 0.0065 * 11000) / 288.15) ** 5.2559 * (h > 11000 ? Math.exp(-(h - 11000) / 6341.6) : 1);
  const a = Math.sqrt(1.4 * 287.05 * T), M = ktas * 0.514444 / a;
  const qc = p * ((1 + 0.2 * M * M) ** 3.5 - 1);
  return Math.sqrt(5 * ((qc / 101325 + 1) ** (2 / 7) - 1)) * 340.294 / 0.514444;
}
for (const id of ids) {
  const json = await readFile(new URL(`../../data/aircraft/${id}.json`, import.meta.url), 'utf8');
  const A = await createAvdModule();
  const errs = A.load(json);
  if (errs.length) { console.log(id, 'LOAD', errs.join('; ')); continue; }
  const out = [];
  // Cruise.
  const [ktas, fl] = SPEC[id], kcas = casFromTas(ktas, fl);
  A.avd_set_assist(2);
  A.avd_reset_air(0, 0, fl * 30.48 - 802, 0, kcas, 0);
  const thr = A.avd_trim_throttle();
  for (let i = 0; i < 80; ++i) { A.avd_set_input(0, 0, 0, Math.min(thr, 1), 1, 0, 0, 0, 0); A.avd_advance(0.25); }
  let s = A.state(), j = A.jet();
  out.push(`cruise FL${fl} ${kcas.toFixed(0)} KCAS: trim thr ${thr.toFixed(2)}, IAS ${s[10].toFixed(0)}, TAS ${s[11].toFixed(0)}, VS ${s[15].toFixed(0)}, a ${s[16].toFixed(1)}, M ${j[5].toFixed(3)}`);
  out.push(`Vs clean/TO/LDG ${j[32].toFixed(0)}/${j[33].toFixed(0)}/${j[34].toFixed(0)} kt at ${(j[35] / 1000).toFixed(1)} t`);
  // Final approach.
  A.avd_reset(2);
  for (let i = 0; i < 80; ++i) { A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), 1, 0, 0, 0, 0); A.avd_advance(0.25); }
  s = A.state(); j = A.jet();
  out.push(`final: IAS ${s[10].toFixed(0)}, VS ${s[15].toFixed(0)}, a ${s[16].toFixed(1)}, flap ${j[27].toFixed(0)}, thr ${A.avd_trim_throttle().toFixed(2)}, Vs LDG ${j[34].toFixed(0)}`);
  // Take-off at full thrust on the detent the take-off stall speed is for
  // (jet()[49]); rotate at 1.12 Vs(TO).
  A.avd_set_assist(0);
  A.avd_reset(1);
  A.avd_cockpit_cycle(A.cockpit().find((x) => x.id === 'brakes.parking').index);
  const fl1 = A.cockpit().find((x) => x.id === 'flaps.lever');
  const toFlap = Math.round(A.jet()[49]);
  const flapAt = () => { const c = A.cockpit()[fl1.index]; return parseFloat(c.positions[c.position]); };
  for (let k = 0; k < fl1.positions.length && flapAt() !== toFlap; ++k) A.avd_cockpit_cycle(fl1.index);
  let vr = 0, t = 0, lift = -1, x0 = A.state()[0], y0 = A.state()[1];
  for (let i = 0; i < 4 * 90; ++i) {
    s = A.state(); j = A.jet();
    if (!vr && j[33] > 0) vr = 1.12 * j[33];
    const pitch = s[10] > vr ? Math.max(-1, Math.min(1, 0.12 * (10 - s[8]))) : 0;
    A.avd_set_input(pitch, 0, 0, 1, 1, 0, 0, 0, 0);
    A.avd_advance(0.25); t += 0.25;
    if (lift < 0 && !A.state()[28] && t > 3) lift = Math.hypot(A.state()[0] - x0, A.state()[1] - y0) * 3.28084;
    if (A.state()[14] > 400) break;
  }
  s = A.state();
  out.push(`take-off: Vr ${vr.toFixed(0)}, lift-off roll ${lift.toFixed(0)} ft, ${t.toFixed(0)} s to ${s[14].toFixed(0)} ft AGL, crashed ${s[48]}`);
  console.log(`== ${id}\n  ` + out.join('\n  '));
}
