// Calibrates generated jets in the web build against what is published:
// the engine lapse so the maximum level speed at full thrust matches the
// published maximum cruise speed, and (where an approach speed is published)
// the flap lift so the landing-flap stall speed at that weight matches it.
// Writes tools/jetgen/calibration.json; jetgen.py bakes it in.
//   node tools/jetgen/calibrate.mjs [ids]
import { readFile, writeFile } from 'node:fs/promises';
import createAvdModule from '../../web/dist/avd.mjs';
const LB = 0.45359237;
// Published targets: maximum cruise KTAS at a flight level; approach speed
// (KIAS) at a weight (lb) with the factor that relates it to the stall speed
// (1.23 transport category, 1.3 commuter / Part 23).
const TARGETS = {
  phenom300: { cruise: [464, 300] },
  cj4: { cruise: [451, 300], vref: { kias: 112, lb: 15660, factor: 1.3, note: 'Vref 112 KIAS (Aerocorner; weight not stated, taken at MLW 15,660 lb; unverified), VSO = Vref / 1.3' } },
  learjet45: { cruise: [465, 330] },
  crj700: { cruise: [473, 370], vref: { kias: 135, lb: 67000, factor: 1.23, note: 'Vref flaps 45 135 KIAS at 67,000 lb (pilot forum quoting the CRJ-701ER handbook), VSR = Vref / 1.23' } },
  b738: { cruise: [460, 350] },
};
const ids = process.argv.slice(2).length ? process.argv.slice(2) : Object.keys(TARGETS);
const path = new URL('./calibration.json', import.meta.url);
let calib = {};
try { calib = JSON.parse(await readFile(path, 'utf8')); } catch { /* first run */ }

function casFromTas(ktas, fl) {
  const h = fl * 30.48, T = Math.max(288.15 - 0.0065 * h, 216.65);
  const p = h <= 11000 ? 101325 * (T / 288.15) ** 5.2559 : 22632 * Math.exp(-(h - 11000) / 6341.6);
  const M = ktas * 0.514444 / Math.sqrt(1.4 * 287.05 * T), qc = p * ((1 + 0.2 * M * M) ** 3.5 - 1);
  return Math.sqrt(5 * ((qc / 101325 + 1) ** (2 / 7) - 1)) * 340.294 / 0.514444;
}
async function sim(def) {
  const A = await createAvdModule();
  const e = A.load(JSON.stringify(def));
  if (e.length) throw new Error(e.join('; '));
  return A;
}
// Maximum level TAS at full thrust at a flight level (altitude held by the Easy assist).
async function maxLevel(def, fl, ktas) {
  const A = await sim(def);
  A.avd_set_assist(0); A.avd_set_level_hold(1);
  A.avd_reset_air(0, 0, fl * 30.48 - 802, 0, casFromTas(ktas * 0.95, fl), 0);
  let tas = 0;
  for (let i = 0; i < 2 * 360; ++i) { A.avd_set_input(0, 0, 0, 1, 1, 0, 0, 0, 0); A.avd_advance(0.5); }
  const s = A.state();
  tas = s[11];
  return { tas, alt: s[53], vs: s[15], mach: A.jet()[5] };
}
async function vsLanding(def, lb) {
  const A = await sim(def);
  A.avd_reset_air(0, 0, 1500, 0, 160, 0);
  A.avd_advance(0.25);
  const j = A.jet();
  return j[34] * Math.sqrt(lb * LB / j[35]);
}
async function bisect(f, lo, hi, target, iters, increasing) {
  for (let k = 0; k < iters; ++k) {
    const mid = (lo + hi) / 2, v = await f(mid);
    if ((v < target) === increasing) lo = mid; else hi = mid;
  }
  return (lo + hi) / 2;
}
for (const id of ids) {
  const base = JSON.parse(await readFile(new URL(`../../data/aircraft/${id}.json`, import.meta.url), 'utf8'));
  const T = TARGETS[id], out = { calibration: [] };
  // Flap lift first (it does not change the cruise).
  if (T.vref) {
    const want = T.vref.kias / T.vref.factor;
    const setFlap = (r) => {
      const d = structuredClone(base);
      for (const p of d.surfaces[0].panels) for (const c of p.controls || []) if (c.channel === 'flap') c.clmax_increment_ratio = r;
      return d;
    };
    const before = await vsLanding(base, T.vref.lb);
    const r = await bisect(async (x) => vsLanding(setFlap(x), T.vref.lb), 0.0, 1.4, want, 12, false);
    const after = await vsLanding(setFlap(r), T.vref.lb);
    out.flap_clmax_ratio = +r.toFixed(3);
    out.calibration.push({ field: 'surfaces[0].panels[*].controls[flap].clmax_increment_ratio', kind: 'calibrated', source: 'EST',
      note: `${r.toFixed(3)} so the landing-flap stall speed at ${T.vref.lb} lb is ${want.toFixed(1)} KCAS (${T.vref.note}); the Top G layout gave ${before.toFixed(1)}.` });
    console.log(id, `flap clmax ratio ${r.toFixed(3)}: Vs LDG ${before.toFixed(1)} -> ${after.toFixed(1)} (want ${want.toFixed(1)})`);
    base.surfaces[0].panels.forEach((p) => (p.controls || []).forEach((c) => { if (c.channel === 'flap') c.clmax_increment_ratio = r; }));
  }
  // Engine lapse to the maximum cruise speed.
  const [ktas, fl] = T.cruise;
  const setLapse = (x) => { const d = structuredClone(base); for (const e of d.turbofans) e.lapse_mach_coefficient = x; return d; };
  const lapse = await bisect(async (x) => (await maxLevel(setLapse(x), fl, ktas)).tas, 0.0, 0.9, ktas, 9, false);
  const r = await maxLevel(setLapse(lapse), fl, ktas);
  out.lapse = +lapse.toFixed(3);
  out.calibration.push({ field: 'turbofans[*].lapse_mach_coefficient', kind: 'calibrated', source: 'MATTINGLY',
    note: `${lapse.toFixed(3)} (Mattingly's lapse law) so the maximum level speed at full thrust at FL${fl} is the published maximum cruise ${ktas} KTAS (sim ${r.tas.toFixed(0)} KTAS, M ${r.mach.toFixed(3)}).` });
  console.log(id, `lapse ${lapse.toFixed(3)}: max level ${r.tas.toFixed(1)} KTAS at FL${fl} (want ${ktas}), alt ${r.alt.toFixed(0)} ft, VS ${r.vs.toFixed(0)}`);
  calib[id] = out;
  await writeFile(path, JSON.stringify(calib, null, 1) + '\n');
}
