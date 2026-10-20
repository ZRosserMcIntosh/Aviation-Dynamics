// Aviation Dynamics — loader for the wasi-sdk build (web/build_wasi.sh).
// SPDX-License-Identifier: MIT
// `const A = await createAvdModule(bytes?)` then A.avd_*() and helpers. The core
// does no I/O in the browser, so the WASI imports are minimal stubs.

// The ABI version this loader was written for: kAbiVersion in web/avd_web.cpp.
// A module with another version is refused (web/README.md, "ABI version").
export const EXPECTED_ABI = 1;
// avd_layout(which), in this order (abi::layout in avd_web.cpp).
export const LAYOUT_NAMES = [
  'state', 'cockpit', 'jet', 'loadsheet', 'flowHeader', 'flowStride', 'flowStrips', 'flowEngineStride', 'flowEngines', 'flow',
  'markings', 'compassCard', 'runway', 'region', 'route', 'routeMax', 'gps', 'wakeHeader', 'wakeStride', 'wakeMax',
  'bodiesMax', 'bodyStride', 'bodies', 'windBoxMax', 'windGridMax', 'textSlack', 'eventsMax',
];

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
  // ABI handshake: the module's layout version must be the one this file was written for.
  const abi = typeof ex.avd_abi_version === 'function' ? ex.avd_abi_version() : 0;
  if (abi !== EXPECTED_ABI) {
    throw new Error(`avd.wasm has ABI version ${abi}, this loader expects ${EXPECTED_ABI}: ` +
      'rebuild web/dist with web/build_wasi.sh from the same commit as avd_loader.mjs (web/README.md, "ABI version")');
  }
  // Every array's size, stride and count from the module, never from literals here.
  const L = Object.fromEntries(LAYOUT_NAMES.map((n, i) => [n, ex.avd_layout(i)]));
  if (Object.values(L).some((v) => !(v > 0))) throw new Error('avd_layout() returned an invalid size: ' + JSON.stringify(L));
  A.abiVersion = abi;
  A.layout = L;
  const enc = new TextEncoder(), dec = new TextDecoder();
  A.text = (len) => dec.decode(new Uint8Array(memory.buffer, ex.avd_text(0), len));
  // "<git short hash>[-dirty] <date>" baked by web/build_wasi.sh ("dev" otherwise).
  A.buildId = () => A.text(ex.avd_build_id());
  A.load = (json) => {
    const b = enc.encode(json);
    new Uint8Array(memory.buffer, ex.avd_text(b.length + 1), b.length).set(b);
    const n = ex.avd_load(b.length);
    const errors = [];
    for (let i = 0; i < n; ++i) errors.push(A.text(ex.avd_error_text(i)));
    return errors;
  };
  A.state = () => new Float64Array(memory.buffer, ex.avd_state(), L.state);
  A.cockpitState = () => new Float64Array(memory.buffer, ex.avd_cockpit(), L.cockpit);
  A.markings = () => Array.from(new Float64Array(memory.buffer, ex.avd_markings(), L.markings));
  A.compassCard = () => Array.from(new Float64Array(memory.buffer, ex.avd_compass_card(), L.compassCard));
  A.flow = () => new Float64Array(memory.buffer, ex.avd_flow(), L.flow);
  A.jet = () => new Float64Array(memory.buffer, ex.avd_jet(), L.jet);
  A.runway = () => Array.from(new Float64Array(memory.buffer, ex.avd_runway(), L.runway));
  A.windGrid = (hx, hy, hz, nx, ny, nz) => new Float64Array(memory.buffer, ex.avd_wind_grid(hx, hy, hz, nx, ny, nz), 3 * nx * ny * nz);
  // The wake's vortex segments: { n, gmax, ground, seg } with 9 numbers a
  // segment (a and b ENU m, circulation m^2/s, core radius m, age s). A view
  // into the module's memory: copy what you keep.
  A.wake = () => {
    const h = new Float64Array(memory.buffer, ex.avd_wake(), L.wakeHeader);
    const n = h[0] | 0;
    return { n, gmax: h[1], ground: h[2], seg: new Float64Array(memory.buffer, h.byteOffset + 8 * L.wakeHeader, n * L.wakeStride) };
  };
  // The wind (ENU m/s) on a grid over centre +- half size (ENU m), x fastest.
  A.windBox = (c, h, nx, ny, nz) => new Float64Array(memory.buffer, ex.avd_wind_box(c[0], c[1], c[2], h[0], h[1], h[2], nx, ny, nz), 3 * nx * ny * nz);
  // Bluff bodies (gear legs, spoiler panels): { n, b } with 10 numbers a body.
  A.bodies = () => { const h = new Float64Array(memory.buffer, ex.avd_bodies(), L.bodies); return { n: h[0] | 0, b: h.subarray(1) }; };
  A.airport = () => { const [icao, name, runway, reciprocal] = A.text(ex.avd_airport_text()).split('|'); return { icao, name, runway, reciprocal }; };
  // Weight and balance: planned (next spawn) or current. Masses in kg, x in
  // the structural frame (m, forward positive), fuel in litres.
  A.loadsheet = (planned = true) => {
    const b = Array.from(new Float64Array(memory.buffer, ex.avd_loadsheet(planned ? 1 : 0), L.loadsheet));
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
  // The region's airports: [{ icao, name, elevation, ends: [{ runway, ils, khz, synthetic }] }].
  A.destinations = () => A.text(ex.avd_destinations_text()).split('\n').filter(Boolean).map((l) => {
    const [icao, name, elev, ends] = l.split('|');
    return { icao, name, elevation: +elev, ends: ends.split(';').map((e) => { const [runway, ils, khz, syn] = e.split(':'); return { runway, ils: ils || null, khz: khz ? +khz : 0, synthetic: syn === '1' }; }) };
  });
  // Every runway end of the region (world frame) per airport, in avd_destinations order.
  A.region = () => {
    const b = new Float64Array(memory.buffer, ex.avd_region(), L.region), out = [];
    for (let k = 1, i = 0; i < b[0]; ++i) {
      const elevation = b[k++], n = b[k++], ends = [];
      for (let j = 0; j < n; ++j, k += 6) ends.push({ east: b[k], north: b[k + 1], heading: b[k + 2], length: b[k + 3], width: b[k + 4], displaced: b[k + 5] });
      out.push({ elevation, ends });
    }
    return out;
  };
  // The GPS route: { active, faf, wp: [{ ident, east, north, alt }] } (alt m, -1 none).
  A.route = () => {
    const b = new Float64Array(memory.buffer, ex.avd_route(), L.route), names = A.text(ex.avd_route_text()).split(';');
    return { active: b[1], faf: b[2], wp: Array.from({ length: b[0] }, (_, i) => ({ ident: names[i], east: b[3 + 3 * i], north: b[4 + 3 * i], alt: b[5 + 3 * i] })) };
  };
  // A destination anywhere in Brazil: a record of airports_br.json and the
  // index of the runway end in its order. Returns its index in destinations(), -1 if unusable.
  A.setDestinationRecord = (record, runway) => {
    const b = enc.encode(JSON.stringify(record));
    new Uint8Array(memory.buffer, ex.avd_text(b.length + 1), b.length).set(b);
    return ex.avd_set_destination_record(b.length, runway);
  };
  A.gps = () => new Float64Array(memory.buffer, ex.avd_gps(), L.gps);
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
