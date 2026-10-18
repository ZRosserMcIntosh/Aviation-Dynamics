// Aviation Dynamics — loader for the wasi-sdk build (web/build_wasi.sh).
// SPDX-License-Identifier: MIT
// `const A = await createAvdModule(bytes?)` then A.avd_*() and helpers. The core
// does no I/O in the browser, so the WASI imports are minimal stubs.
export default async function createAvdModule(wasmBytes) {
  let bytes = wasmBytes;
  if (!bytes) {
    const url = new URL('./avd.wasm', import.meta.url);
    if (typeof process !== 'undefined' && process.versions?.node && url.protocol === 'file:') {
      const { readFile } = await import('node:fs/promises');
      bytes = await readFile(url);
    } else {
      bytes = await (await fetch(url)).arrayBuffer();
    }
  }
  let memory = null;
  const ENOSYS = 52;
  const wasi = new Proxy({
    proc_exit: (c) => { throw new Error('avd.wasm exited with code ' + c); },
    fd_write: (fd, iovs, n, nwritten) => {
      const v = new DataView(memory.buffer); let s = '', total = 0;
      for (let i = 0; i < n; ++i) { const p = v.getUint32(iovs + i * 8, true), l = v.getUint32(iovs + i * 8 + 4, true); s += new TextDecoder().decode(new Uint8Array(memory.buffer, p, l)); total += l; }
      v.setUint32(nwritten, total, true); if (s.trim()) (fd === 2 ? console.warn : console.log)(s.trimEnd()); return 0;
    },
    environ_sizes_get: (c, s) => { const v = new DataView(memory.buffer); v.setUint32(c, 0, true); v.setUint32(s, 0, true); return 0; },
    environ_get: () => 0,
    clock_time_get: (id, prec, out) => { new DataView(memory.buffer).setBigUint64(out, BigInt(Math.round(performance.now() * 1e6)), true); return 0; },
    random_get: (p, n) => { crypto.getRandomValues(new Uint8Array(memory.buffer, p, n)); return 0; },
  }, { get: (t, k) => t[k] ?? (() => ENOSYS) });
  const { instance } = await WebAssembly.instantiate(bytes, { wasi_snapshot_preview1: wasi });
  const ex = instance.exports;
  memory = ex.memory;
  ex._initialize?.();
  const A = { ...Object.fromEntries(Object.entries(ex).filter(([k]) => k.startsWith('avd_'))) };
  const enc = new TextEncoder(), dec = new TextDecoder();
  A.text = (len) => dec.decode(new Uint8Array(memory.buffer, ex.avd_text(0), len));
  A.load = (json) => {
    const b = enc.encode(json);
    new Uint8Array(memory.buffer, ex.avd_text(b.length + 1), b.length).set(b);
    const n = ex.avd_load(b.length);
    const errors = [];
    for (let i = 0; i < n; ++i) errors.push(A.text(ex.avd_error_text(i)));
    return errors;
  };
  A.state = () => new Float64Array(memory.buffer, ex.avd_state(), 64);
  A.cockpitState = () => new Float64Array(memory.buffer, ex.avd_cockpit(), 96);
  A.markings = () => Array.from(new Float64Array(memory.buffer, ex.avd_markings(), 96));
  A.compassCard = () => Array.from(new Float64Array(memory.buffer, ex.avd_compass_card(), 12));
  A.flow = () => new Float64Array(memory.buffer, ex.avd_flow(), 16 + 64 * 16 + 40);
  A.jet = () => new Float64Array(memory.buffer, ex.avd_jet(), 64);
  A.runway = () => Array.from(new Float64Array(memory.buffer, ex.avd_runway(), 6));
  A.windGrid = (hx, hy, hz, nx, ny, nz) => new Float64Array(memory.buffer, ex.avd_wind_grid(hx, hy, hz, nx, ny, nz), 3 * nx * ny * nz);
  A.airport = () => { const [icao, name, runway, reciprocal] = A.text(ex.avd_airport_text()).split('|'); return { icao, name, runway, reciprocal }; };
  // Weight and balance: planned (next spawn) or current. Masses in kg, x in
  // the structural frame (m, forward positive), fuel in litres.
  A.loadsheet = (planned = true) => {
    const b = Array.from(new Float64Array(memory.buffer, ex.avd_loadsheet(planned ? 1 : 0), 256));
    const [sn, tn, gn] = A.text(ex.avd_loadsheet_names()).split('|').map((x) => (x ? x.split(';') : []));
    let k = 24;
    const take = (n) => b.slice(k, (k += n));
    const stations = Array.from({ length: b[17] }, (_, i) => { const [kg, max, x] = take(3); return { name: sn[i], kg, max, x }; });
    const tanks = Array.from({ length: b[18] }, (_, i) => { const [litres, capacity, unusable, x] = take(4); return { name: tn[i], litres, capacity, unusable, x }; });
    const groups = Array.from({ length: b[19] }, (_, i) => { const [kg, max] = take(2); return { name: gn[i], kg, max }; });
    const fwd = Array.from({ length: b[20] }, () => take(2)), aft = Array.from({ length: b[21] }, () => take(2));
    return {
      basicEmpty: b[0], basicEmptyX: b[1], payload: b[2], zeroFuel: b[3], zeroFuelX: b[4], fuelTotal: b[5], fuelUsable: b[6],
      gross: b[7], cgX: b[8], cgFwdLimit: b[9], cgAftLimit: b[10], flags: b[11], trappedFuel: b[22],
      limits: { ramp: b[12], takeoff: b[13], landing: b[14], zeroFuel: b[15] }, density: b[16], stations, tanks, groups, fwd, aft,
    };
  };
  A.events = () => { const n = ex.avd_events(); return n ? A.text(n).split('\n').filter(Boolean) : []; };
  A.cockpit = () => {
    const out = [];
    for (let i = 0; i < ex.avd_cockpit_count(); ++i) {
      const [id, label, pos, cur] = A.text(ex.avd_cockpit_text(i)).split('|');
      out.push({ index: i, id, label, positions: pos.split(';'), position: +cur });
    }
    return out;
  };
  A.procedure = () => { const st = ex.avd_procedure_status(); const n = new Uint8Array(memory.buffer, ex.avd_text(0)).indexOf(0); return { status: st, message: A.text(n) }; };
  return A;
}
