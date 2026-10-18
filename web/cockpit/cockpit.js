// Aviation Dynamics — browser cockpit for the Tupi T-180 (Cessna 172S reference).
// SPDX-License-Identifier: MIT
//
// Renders ONLY what the C++ instrument models report (avd_cockpit) and sends
// pilot controls back through the C ABI. No gauge computes flight state here.
// Entry point: window.AVD_BOOT(createAvdModule, wasmBytes, aircraftJson).
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const DEG = Math.PI / 180;
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  const store = {
    get(k, d) { try { const v = localStorage.getItem('avd-cockpit:' + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
    set(k, v) { try { localStorage.setItem('avd-cockpit:' + k, JSON.stringify(v)); } catch { /* storage unavailable */ } },
  };

  // Panel colours come from CSS tokens so the page and the canvas agree.
  const css = getComputedStyle(document.documentElement);
  const tok = (n) => css.getPropertyValue(n).trim();
  const C = {
    face: tok('--face'), bezel: tok('--bezel'), panel: tok('--panel'), mark: tok('--mark'), dim: tok('--mark-dim'),
    green: tok('--arc-green'), yellow: tok('--arc-yellow'), red: tok('--arc-red'), white: tok('--arc-white'),
    needle: tok('--needle'), sky: tok('--sky'), earth: tok('--earth'), amber: tok('--amber'), lcd: tok('--lcd'),
    flag: tok('--flag'), accent: tok('--accent'),
  };
  const FONT = '"Barlow Condensed", "Arial Narrow", sans-serif';
  const MONO = '"Share Tech Mono", ui-monospace, monospace';

  // Cockpit-state buffer layout (web/README.md).
  const K = {
    ias: 0, alt: 1, kollsman: 2, vsi: 3, aiPitch: 4, aiRoll: 5, aiTumbled: 6, dg: 7, bug: 8, tcDefl: 9, tcRate: 10,
    tcFlag: 11, ball: 12, compass: 13, rpm: 14, oilP: 15, oilT: 16, egt: 17, ff: 18, fuelL: 19, fuelR: 20, amps: 21,
    volts: 22, suction: 23, ann: 24, stall: 25, oat: 26, clock: 27, flaps: 28, trim: 29, panelLights: 30, dimmer: 31,
    navA: 32, navS: 33, comA: 34, comS: 35, obs: 36, navValid: 37, cdi: 38, toFrom: 39, gsValid: 40, gs: 41, loc: 42,
    avionics: 43, squawk: 44, xpdr: 45, repAlt: 46, altRep: 47,
    tTas: 48, tCas: 49, tGs: 50, tAlt: 51, tPalt: 52, tVs: 53, tPitch: 54, tRoll: 55, tHdg: 56, tMag: 57,
    tP: 58, tQ: 59, tR: 60, tSlip: 61, aiRotor: 62, dgRotor: 63, tcRotor: 64, tEas: 65, tDalt: 66, tAgl: 67, tAlpha: 68,
    fPitot: 69, fStatic: 70, fVac: 71, pitotIce: 72, pitotHeat: 73, cloudWater: 74,
  };

  window.AVD_BOOT = async function (createAvdModule, wasmBytes, aircraftJson) {
    let A;
    try {
      A = await createAvdModule(wasmBytes);
    } catch (e) {
      $('status').textContent = 'This browser could not start the WebAssembly flight model: ' + e.message;
      return;
    }
    const errors = A.load(aircraftJson);
    if (errors.length) { $('status').textContent = 'Aircraft data rejected: ' + errors[0]; return; }
    await (document.fonts?.ready ?? Promise.resolve());

    const M = A.markings();
    const marks = {
      white: [M[0], M[1]], green: [M[2], M[3]], yellow: [M[4], M[5]], vne: M[6], dialMin: M[7], dialMax: M[8],
      tachGreen: [M[9], M[10]], tachRed: M[11], tachMax: M[12], oilPGreen: [M[13], M[14]], oilPRedLo: M[15],
      oilPRedHi: M[16], oilTGreen: [M[17], M[18]], oilTRed: M[19], suctionGreen: [M[20], M[21]], fuelFull: M[22],
      vsiRange: M[23], stdRateDefl: M[27],
    };

    // ------------------------------------------------------------ inputs ----
    const input = { pitch: 0, roll: 0, yaw: 0, throttle: 0.0, mixture: 1.0, brake: 0, trimRate: 0, starter: false };
    const keys = new Set();
    let paused = false, showDebug = store.get('debug', false), hover = null, gamepadOn = false;
    const scenarioNames = ['Cold and dark, runway 12', 'Ready for take-off, runway 12', 'Final approach, runway 12 (3 km)', 'Cruising over the city'];

    const keyMap = {
      KeyW: 'pitchDown', ArrowUp: 'pitchDown', KeyS: 'pitchUp', ArrowDown: 'pitchUp',
      KeyA: 'rollLeft', ArrowLeft: 'rollLeft', KeyD: 'rollRight', ArrowRight: 'rollRight',
      KeyQ: 'yawLeft', KeyE: 'yawRight', Equal: 'throttleUp', NumpadAdd: 'throttleUp', PageUp: 'throttleUp',
      Minus: 'throttleDown', NumpadSubtract: 'throttleDown', PageDown: 'throttleDown',
      KeyM: 'mixRich', KeyN: 'mixLean', KeyB: 'brake', BracketLeft: 'trimDown', BracketRight: 'trimUp', KeyT: 'starter',
    };
    addEventListener('keydown', (e) => {
      if (e.target.closest?.('input, select, textarea')) return;
      if (keyMap[e.code]) { keys.add(keyMap[e.code]); e.preventDefault(); }
      if (e.repeat) return;
      if (e.code === 'KeyF' && hover) { act(hover); e.preventDefault(); }
      if (e.code === 'Space') { paused = !paused; e.preventDefault(); }
      if (e.code === 'KeyI') toggleDebug();
      if (e.code === 'Comma') cycleTo('flaps.lever', -1);
      if (e.code === 'Period') cycleTo('flaps.lever', +1);
      if (e.code === 'KeyP') A.avd_cockpit_cycle(controlIndex('brakes.parking'));
      if (e.code === 'KeyH') A.avd_dg_sync();
      if (e.code === 'KeyL') { levelHold.checked = !levelHold.checked; applyLevelHold(true); }
    });
    addEventListener('keyup', (e) => { if (keyMap[e.code]) keys.delete(keyMap[e.code]); });
    addEventListener('blur', () => keys.clear());

    function approach(v, target, rate, dt) { return v < target ? Math.min(v + rate * dt, target) : Math.max(v - rate * dt, target); }
    function readInputs(dt) {
      const k = (n) => keys.has(n);
      input.pitch = approach(input.pitch, k('pitchUp') ? 1 : k('pitchDown') ? -1 : 0, k('pitchUp') || k('pitchDown') ? 1.5 : 3, dt);
      input.roll = approach(input.roll, k('rollRight') ? 1 : k('rollLeft') ? -1 : 0, k('rollRight') || k('rollLeft') ? 2 : 4, dt);
      input.yaw = approach(input.yaw, k('yawRight') ? 1 : k('yawLeft') ? -1 : 0, 3, dt);
      if (k('throttleUp')) input.throttle = clamp(input.throttle + 0.4 * dt, 0, 1);
      if (k('throttleDown')) input.throttle = clamp(input.throttle - 0.4 * dt, 0, 1);
      if (k('mixRich')) input.mixture = clamp(input.mixture + 0.3 * dt, 0, 1);
      if (k('mixLean')) input.mixture = clamp(input.mixture - 0.3 * dt, 0, 1);
      input.brake = k('brake') ? 1 : 0;
      input.trimRate = k('trimUp') ? 1 : k('trimDown') ? -1 : 0;
      input.starter = k('starter') || startHeld;
      // Gamepad (standard mapping): sticks fly, triggers throttle, A brakes.
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      const gp = pads && Array.from(pads).find((p) => p && p.mapping === 'standard');
      if (gp) {
        gamepadOn = true;
        const dz = (v) => (Math.abs(v) < 0.08 ? 0 : v);
        if (dz(gp.axes[0]) || dz(gp.axes[1])) { input.roll = dz(gp.axes[0]); input.pitch = dz(gp.axes[1]); }
        if (dz(gp.axes[2])) input.yaw = dz(gp.axes[2]);
        input.throttle = clamp(input.throttle + ((gp.buttons[7]?.value || 0) - (gp.buttons[6]?.value || 0)) * 0.5 * dt, 0, 1);
        if (gp.buttons[0]?.pressed) input.brake = 1;
        if (gp.buttons[12]?.pressed) input.trimRate = -1;
        if (gp.buttons[13]?.pressed) input.trimRate = 1;
      }
    }

    // --------------------------------------------------------- controls ----
    let controls = A.cockpit();
    const controlIndex = (id) => controls.findIndex((c) => c.id === id);
    function cycleTo(id, dir) {
      const c = controls[controlIndex(id)];
      if (!c) return;
      const n = c.positions.length;
      const target = clamp(c.position + dir, 0, n - 1);
      for (let i = 0; i < n && A.cockpit()[c.index].position !== target; ++i) A.avd_cockpit_cycle(c.index);
    }
    const switchesEl = $('switches');
    function buildSwitches() {
      controls = A.cockpit();
      switchesEl.textContent = '';
      for (const c of controls) {
        const b = document.createElement('button');
        b.type = 'button';
        b.className = 'switch';
        b.id = 'sw-' + c.id.replaceAll('.', '-');
        b.dataset.id = c.id;
        b.innerHTML = `<span class="sw-label"></span><span class="sw-pos"></span>`;
        b.querySelector('.sw-label').textContent = shortLabel(c.label);
        b.title = c.label;
        b.addEventListener('click', () => { A.avd_cockpit_cycle(c.index); refreshSwitches(); });
        b.addEventListener('mouseenter', () => setHover({ kind: 'switch', id: c.id, label: c.label }));
        b.addEventListener('mouseleave', () => setHover(null));
        b.addEventListener('focus', () => setHover({ kind: 'switch', id: c.id, label: c.label }));
        switchesEl.append(b);
      }
      refreshSwitches();
    }
    function shortLabel(l) {
      return l.replace('Circuit Breaker: ', 'CB ').replace(' Switch', '').replace(' (hold START to crank)', '')
        .replace('Master', 'Mstr').replace(' (NAV/COM, transponder)', '').replace(' (engine and fuel gauges)', '');
    }
    function refreshSwitches() {
      controls = A.cockpit();
      for (const c of controls) {
        const b = $('sw-' + c.id.replaceAll('.', '-'));
        if (!b) continue;
        const pos = c.positions[c.position] ?? '';
        b.querySelector('.sw-pos').textContent = pos;
        const on = !/^(OFF|0 deg)$/.test(pos);
        b.classList.toggle('on', on);
        b.setAttribute('aria-pressed', on ? 'true' : 'false');
      }
    }
    function setHover(h) {
      hover = h;
      $('interact').textContent = h ? (h.kind === 'switch' ? `Press F to interact — ${h.label}` : h.label) : 'Point at a switch or knob: F or click operates switches; scroll or click the knob’s left/right half turns knobs.';
    }
    function act(h) {
      if (h.kind === 'switch') { A.avd_cockpit_cycle(controlIndex(h.id)); refreshSwitches(); }
      else if (h.kind === 'knob') h.turn(+1);
    }
    let startHeld = false;
    const startBtn = $('starter');
    const hold = (v) => () => { startHeld = v; startBtn.classList.toggle('on', v); };
    startBtn.addEventListener('pointerdown', hold(true));
    for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) startBtn.addEventListener(ev, hold(false));

    // Radios (HTML stack).
    const radioBtn = (id, fn) => $(id).addEventListener('click', fn);
    radioBtn('com-swap', () => A.avd_com_swap());
    radioBtn('com-up', () => A.avd_com_tune(1));
    radioBtn('com-dn', () => A.avd_com_tune(-1));
    radioBtn('com-up-mhz', () => A.avd_com_tune(40));
    radioBtn('com-dn-mhz', () => A.avd_com_tune(-40));
    radioBtn('nav-swap', () => A.avd_nav_swap());
    radioBtn('nav-up', () => A.avd_nav_tune(1));
    radioBtn('nav-dn', () => A.avd_nav_tune(-1));
    radioBtn('nav-up-mhz', () => A.avd_nav_tune(20));
    radioBtn('nav-dn-mhz', () => A.avd_nav_tune(-20));
    const xpdrModes = ['OFF', 'SBY', 'ON', 'ALT'];
    radioBtn('xpdr-mode', () => A.avd_xpdr_mode((A.cockpitState()[K.xpdr] + 1) % 4));
    let squawkEntry = '';
    document.querySelectorAll('[data-digit]').forEach((b) => b.addEventListener('click', () => {
      squawkEntry += b.dataset.digit;
      if (squawkEntry.length === 4) { A.avd_squawk(parseInt(squawkEntry, 10)); squawkEntry = ''; }
    }));
    radioBtn('vfr', () => { A.avd_squawk(1200); squawkEntry = ''; });

    // Scenario, assists, weather, failures.
    const scen = $('scenario');
    scenarioNames.forEach((n, i) => scen.add(new Option(n, String(i))));
    scen.value = String(store.get('scenario', 1));
    const assist = $('assist');
    assist.value = String(store.get('assist', 1));
    const levelHold = $('level-hold');
    levelHold.checked = store.get('levelHold', false);
    function applyLevelHold(announce) {
      A.avd_set_level_hold(levelHold.checked ? 1 : 0);
      store.set('levelHold', levelHold.checked);
      if (announce) log(levelHold.checked ? 'Auto-trim on: release the stick and it holds straight and level.' : 'Auto-trim off.');
    }
    function reset() {
      const mode = +scen.value;
      store.set('scenario', mode);
      applyWeather();
      applyCloud();
      A.avd_reset(mode);
      A.avd_set_assist(+assist.value);
      applyLevelHold(false);
      // Airborne spawns are trimmed by the C++ trim solver: start the levers
      // where it left them, so the airplane flies hands-off.
      input.throttle = mode >= 2 ? A.avd_trim_throttle() : 0.12;
      input.mixture = mode >= 2 ? A.avd_trim_mixture() : 1;
      for (const f of ['fail-pitot', 'fail-static', 'fail-vac', 'fail-alt']) $(f).checked = false;
      buildSwitches();
      compassCard = A.compassCard();
      log(`Reset: ${scenarioNames[mode]}.`);
    }
    $('reset').addEventListener('click', reset);
    scen.addEventListener('change', reset);
    assist.addEventListener('change', () => { A.avd_set_assist(+assist.value); store.set('assist', +assist.value); });
    levelHold.addEventListener('change', () => applyLevelHold(true));
    $('quickstart').addEventListener('click', () => { A.avd_quick_start(); log('Quick Start: working through the checklist…'); });
    function applyWeather() {
      A.avd_set_weather(+$('wind').value * 0.5144, +$('winddir').value, +$('turb').value, +$('temp').value, 0.6);
      $('wind-v').textContent = `${$('wind').value} kt from ${String($('winddir').value).padStart(3, '0')}°`;
      $('turb-v').textContent = (+$('turb').value).toFixed(1) + ' m/s';
      $('temp-v').textContent = 'ISA ' + (+$('temp').value >= 0 ? '+' : '') + $('temp').value + ' °C';
    }
    for (const id of ['wind', 'winddir', 'turb', 'temp']) $(id).addEventListener('input', applyWeather);
    // Icing layer: stratus from 1,500 to 3,600 ft above the field, 0.4 g/m^3 of
    // cloud water. Supercooled when the temperature slider makes it freezing.
    function applyCloud() {
      const on = $('cloud').checked, base = rw[2] + 450, top = rw[2] + 1100;
      A.avd_set_cloud(base, on ? top : base, on ? 0.4 : 0);
    }
    $('cloud').addEventListener('change', () => { applyCloud(); log($('cloud').checked ? 'Cloud layer 1,500-3,600 ft AGL (0.4 g/m³ liquid water).' : 'Cloud layer removed.'); });
    [['fail-pitot', 0], ['fail-static', 1], ['fail-vac', 2], ['fail-alt', 3]].forEach(([id, n]) =>
      $(id).addEventListener('change', (e) => { A.avd_failure(n, e.target.checked ? 1 : 0); log(`${e.target.labels[0].textContent.trim()}: ${e.target.checked ? 'failed' : 'restored'}.`); }));
    $('dimmer').addEventListener('input', (e) => A.avd_dimmer(+e.target.value));
    $('debug-toggle').addEventListener('click', toggleDebug);
    function toggleDebug() { showDebug = !showDebug; store.set('debug', showDebug); $('debug').hidden = !showDebug; $('debug-toggle').setAttribute('aria-pressed', showDebug); $('debug-toggle').textContent = showDebug ? 'Hide (I)' : 'Show (I)'; }
    $('debug').hidden = !showDebug;
    $('debug-toggle').setAttribute('aria-pressed', showDebug);
    $('debug-toggle').textContent = showDebug ? 'Hide (I)' : 'Show (I)';

    // Sound: stall-warning horn (reed horn) through WebAudio, enabled by a click.
    let audio = null, hornGain = null;
    $('sound').addEventListener('click', () => {
      if (audio) { audio.close(); audio = null; $('sound').setAttribute('aria-pressed', 'false'); $('sound').textContent = 'Sound off'; return; }
      audio = new AudioContext();
      const osc = audio.createOscillator(), osc2 = audio.createOscillator();
      osc.type = 'sawtooth'; osc.frequency.value = 2950; osc2.type = 'square'; osc2.frequency.value = 2905;
      hornGain = audio.createGain(); hornGain.gain.value = 0;
      const lp = audio.createBiquadFilter(); lp.type = 'bandpass'; lp.frequency.value = 2900; lp.Q.value = 4;
      osc.connect(lp); osc2.connect(lp); lp.connect(hornGain); hornGain.connect(audio.destination);
      osc.start(); osc2.start();
      $('sound').setAttribute('aria-pressed', 'true'); $('sound').textContent = 'Sound on';
    });

    const logEl = $('log');
    function log(msg) {
      const li = document.createElement('li');
      const t = A.state()[42];
      li.innerHTML = `<time></time> <span></span>`;
      li.querySelector('time').textContent = fmtTime(t);
      li.querySelector('span').textContent = msg;
      logEl.prepend(li);
      while (logEl.children.length > 40) logEl.lastChild.remove();
    }
    const fmtTime = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

    // ------------------------------------------------------------ canvases ----
    const view = $('view'), panel = $('panel');
    const vctx = view.getContext('2d'), pctx = panel.getContext('2d');
    const PW = 1200, PH = 600;  // panel design units
    function fit() {
      const dpr = Math.min(devicePixelRatio || 1, 2);
      for (const cv of [view, panel]) {
        const r = cv.getBoundingClientRect();
        cv.width = Math.max(1, Math.round(r.width * dpr));
        cv.height = Math.max(1, Math.round(r.height * dpr));
      }
    }
    addEventListener('resize', fit);
    fit();

    // Knob hot-spots in panel units: [x, y, r, label, turn(dir)]
    // Knobs sit at the lower-left/right of their instrument (panel units).
    const KNOB = { kollsman: [451, 184], hdgSet: [251, 384], hdgBug: [389, 384], obs: [658, 177] };
    const knobs = [
      { at: KNOB.kollsman, label: 'Altimeter setting knob (Kollsman) — scroll, or click its left/right half', turn: (d) => A.avd_altimeter_adjust(d) },
      { at: KNOB.hdgSet, label: 'Heading indicator SET knob — scroll to rotate the card; H syncs it to the compass', turn: (d) => A.avd_dg_adjust(d) },
      { at: KNOB.hdgBug, label: 'Heading bug knob — scroll, or click its left/right half', turn: (d) => A.avd_heading_bug((A.cockpitState()[K.bug] + d * 2 + 360) % 360) },
      { at: KNOB.obs, label: 'OBS knob — scroll, or click its left/right half', turn: (d) => A.avd_obs_adjust(d) },
    ].map((k) => ({ ...k, x: k.at[0], y: k.at[1], r: 18 }));
    function panelPoint(e) {
      const r = panel.getBoundingClientRect();
      const s = Math.min(r.width / PW, r.height / PH);
      const ox = (r.width - PW * s) / 2, oy = (r.height - PH * s) / 2;
      return { x: (e.clientX - r.left - ox) / s, y: (e.clientY - r.top - oy) / s };
    }
    const knobAt = (p) => knobs.find((k) => Math.hypot(p.x - k.x, p.y - k.y) < k.r + 6);
    panel.addEventListener('pointermove', (e) => {
      const k = knobAt(panelPoint(e));
      panel.style.cursor = k ? 'ew-resize' : 'default';
      if (k) setHover({ kind: 'knob', label: k.label, turn: k.turn });
      else if (hover?.kind === 'knob') setHover(null);
    });
    panel.addEventListener('pointerleave', () => { if (hover?.kind === 'knob') setHover(null); });
    panel.addEventListener('wheel', (e) => {
      const p = panelPoint(e), k = knobAt(p);
      if (!k) return;
      e.preventDefault();
      k.turn(e.deltaY < 0 ? 1 : -1);
    }, { passive: false });
    panel.addEventListener('click', (e) => {
      const p = panelPoint(e), k = knobAt(p);
      if (k) k.turn(p.x >= k.x ? 1 : -1);
    });

    // ---------------------------------------------------- outside view ----
    const rw = A.runway();  // [east, north, elev, heading, length, width]
    const landmarks = [
      ['MASP', -1, 1], ['Catedral da Sé', 2193, 1161], ['Copan', 1154, 1665], ['Estação da Luz', 2072, 2948],
      ['Ibirapuera', -194, -2904], ['Ponte Estaiada', -4320, -5722], ['Congonhas', -522, -7260], ['Campo de Marte', 1797, 5734],
    ];
    function drawView(s) {
      const w = view.width, h = view.height, ctx = vctx;
      // Camera: pilot's eye in body FLU (forward, left, up) from the datum.
      const qw = s[3], qx = s[4], qy = s[5], qz = s[6];
      const rot = (v) => {  // FLU -> ENU
        const tx = 2 * (qy * v[2] - qz * v[1]), ty = 2 * (qz * v[0] - qx * v[2]), tz = 2 * (qx * v[1] - qy * v[0]);
        return [v[0] + qw * tx + (qy * tz - qz * ty), v[1] + qw * ty + (qz * tx - qx * tz), v[2] + qw * tz + (qx * ty - qy * tx)];
      };
      const inv = (v) => {  // ENU -> FLU (conjugate)
        const tx = 2 * (-qy * v[2] + qz * v[1]), ty = 2 * (-qz * v[0] + qx * v[2]), tz = 2 * (-qx * v[1] + qy * v[0]);
        return [v[0] + qw * tx + (-qy * tz + qz * ty), v[1] + qw * ty + (-qz * tx + qx * tz), v[2] + qw * tz + (-qx * ty + qy * tx)];
      };
      const off = rot([-1.0, 0.35, 1.05]);
      const eye = [s[0] + off[0], s[1] + off[1], s[2] + off[2]];
      const f = h * 1.05, cx = w / 2, cy = h * 0.42;
      const cam = (p) => inv([p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]]);
      const proj = (c) => [cx - f * c[1] / c[0], cy - f * c[2] / c[0]];
      // Sky and ground: the horizon is where a view ray is level, i.e. the line
      // up[0] - up[1]*u - up[2]*v = 0 with u, v the normalised screen offsets.
      ctx.fillStyle = C.sky; ctx.fillRect(0, 0, w, h);
      const up = inv([0, 0, 1]);
      const level = (px, py) => up[0] + up[1] * (cx - px) / f + up[2] * (cy - py) / f;
      const corners = [[0, 0], [w, 0], [w, h], [0, h]], poly = [];
      for (let i = 0; i < 4; ++i) {
        const a = corners[i], b = corners[(i + 1) % 4], la = level(a[0], a[1]), lb = level(b[0], b[1]);
        if (la < 0) poly.push(a);
        if ((la < 0) !== (lb < 0)) { const t = la / (la - lb); poly.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]); }
      }
      if (poly.length > 2) {
        ctx.beginPath(); poly.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath();
        ctx.fillStyle = C.earth; ctx.fill();
      }
      // Ground grid: the São Paulo World 500 m tiles (edges at odd multiples of 250 m).
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(200,214,190,0.18)';
      const line3 = (a, b) => {
        let ca = cam(a), cb = cam(b);
        const near = 2;
        if (ca[0] < near && cb[0] < near) return;
        if (ca[0] < near) { const t = (near - ca[0]) / (cb[0] - ca[0]); ca = ca.map((v, i) => v + (cb[i] - v) * t); }
        if (cb[0] < near) { const t = (near - cb[0]) / (ca[0] - cb[0]); cb = cb.map((v, i) => v + (ca[i] - v) * t); }
        const pa = proj(ca), pb = proj(cb);
        ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]);
      };
      const groundZ = rw[2];
      const gx0 = Math.round(eye[0] / 500) * 500, gy0 = Math.round(eye[1] / 500) * 500, R = 6000;
      ctx.beginPath();
      for (let i = -R; i <= R; i += 500) {
        line3([gx0 + i + 250, gy0 - R, groundZ], [gx0 + i + 250, gy0 + R, groundZ]);
        line3([gx0 - R, gy0 + i + 250, groundZ], [gx0 + R, gy0 + i + 250, groundZ]);
      }
      ctx.stroke();
      // Runway 12/30.
      const hd = rw[3] * DEG, ce = Math.sin(hd), cn = Math.cos(hd), L = rw[4], W = rw[5] / 2;
      const P = (along, across) => [rw[0] + along * ce + across * cn, rw[1] + along * cn - across * ce, groundZ + 0.05];
      const quad = [P(0, -W), P(L, -W), P(L, W), P(0, W)].map(cam);
      if (quad.every((c) => c[0] > 2)) {
        ctx.beginPath();
        quad.map(proj).forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
        ctx.closePath(); ctx.fillStyle = '#3a3d40'; ctx.fill();
      } else {
        ctx.beginPath(); ctx.strokeStyle = '#55595c'; ctx.lineWidth = 2;
        line3(P(0, -W), P(L, -W)); line3(P(L, -W), P(L, W)); line3(P(L, W), P(0, W)); line3(P(0, W), P(0, -W)); ctx.stroke();
      }
      ctx.beginPath(); ctx.strokeStyle = 'rgba(240,240,232,0.85)'; ctx.lineWidth = 1.5;
      for (let a = 60; a < L - 60; a += 60) line3(P(a, 0), P(a + 30, 0));
      for (const end of [8, L - 38]) for (let k = -8; k <= 8; k += 2.6) line3(P(end, k * 2), P(end + 30, k * 2));
      ctx.stroke();
      // Landmarks.
      ctx.font = `${Math.max(11, h * 0.028)}px ${FONT}`;
      ctx.textAlign = 'center';
      for (const [name, e, n] of landmarks) {
        const base = cam([e, n, groundZ + 40]), top = cam([e, n, groundZ + 140]);
        if (base[0] < 20 || base[0] > 30000) continue;
        const pb = proj(base), pt = proj(top);
        ctx.strokeStyle = 'rgba(240,226,190,0.7)'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(pb[0], pb[1]); ctx.lineTo(pt[0], pt[1]); ctx.stroke();
        ctx.fillStyle = 'rgba(244,236,214,0.9)';
        ctx.fillText(name, pt[0], pt[1] - 4);
      }
      // Glareshield silhouette and the wet compass sits on it (drawn in panel).
      ctx.fillStyle = C.panel;
      ctx.beginPath(); ctx.moveTo(0, h); ctx.lineTo(0, h * 0.93); ctx.quadraticCurveTo(w / 2, h * 0.86, w, h * 0.93); ctx.lineTo(w, h); ctx.fill();
    }

    // ---------------------------------------------------------- panel ----
    function dial(ctx, x, y, r, title) {
      ctx.save();
      ctx.fillStyle = C.bezel;
      ctx.beginPath(); ctx.arc(x, y, r + 7, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = C.face;
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill();
      if (title) { ctx.fillStyle = C.dim; ctx.font = `600 ${r * 0.13}px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText(title, x, y + r * 0.42); }
      ctx.restore();
    }
    function glass(ctx, x, y, r) {
      const g = ctx.createLinearGradient(x - r, y - r, x + r, y + r);
      g.addColorStop(0, 'rgba(255,255,255,0.07)'); g.addColorStop(0.45, 'rgba(255,255,255,0.0)'); g.addColorStop(1, 'rgba(255,255,255,0.03)');
      ctx.fillStyle = g; ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill();
    }
    function needle(ctx, x, y, len, ang, w, tail = 0.18, color = C.needle) {
      ctx.save(); ctx.translate(x, y); ctx.rotate(ang);
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.moveTo(-w, len * tail); ctx.lineTo(-w * 0.55, -len); ctx.lineTo(0, -len - w * 1.6); ctx.lineTo(w * 0.55, -len); ctx.lineTo(w, len * tail); ctx.closePath(); ctx.fill();
      ctx.fillStyle = C.bezel; ctx.beginPath(); ctx.arc(0, 0, w * 1.5, 0, 2 * Math.PI); ctx.fill();
      ctx.restore();
    }
    function arcBand(ctx, x, y, r, a0, a1, color, width) {
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath(); ctx.arc(x, y, r, a0 - Math.PI / 2, a1 - Math.PI / 2); ctx.stroke();
    }
    function ticks(ctx, x, y, r, from, to, step, angle, len, width, label, every, fmt) {
      ctx.strokeStyle = C.mark; ctx.fillStyle = C.mark; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      let i = 0;
      for (let v = from; v <= to + 1e-9; v += step, ++i) {
        const a = angle(v) - Math.PI / 2, major = every && i % every === 0;
        ctx.lineWidth = major ? width * 1.6 : width;
        const l = major ? len * 1.6 : len;
        ctx.beginPath(); ctx.moveTo(x + Math.cos(a) * r, y + Math.sin(a) * r); ctx.lineTo(x + Math.cos(a) * (r - l), y + Math.sin(a) * (r - l)); ctx.stroke();
        if (major && label) { ctx.fillText(fmt ? fmt(v) : String(v), x + Math.cos(a) * (r - l - label), y + Math.sin(a) * (r - l - label)); }
      }
    }

    function drawAsi(ctx, x, y, r, c) {
      dial(ctx, x, y, r);
      const a = (kt) => {  // C172-style nonlinear spacing: compressed low end
        const k = clamp(kt, 0, marks.dialMax);
        return k <= marks.dialMin ? (k / marks.dialMin) * 22 * DEG : (22 + (k - marks.dialMin) / (marks.dialMax - marks.dialMin) * 318) * DEG;
      };
      arcBand(ctx, x, y, r * 0.93, a(marks.white[0]), a(marks.white[1]), C.white, r * 0.05);
      arcBand(ctx, x, y, r * 0.86, a(marks.green[0]), a(marks.green[1]), C.green, r * 0.07);
      arcBand(ctx, x, y, r * 0.86, a(marks.yellow[0]), a(marks.yellow[1]), C.yellow, r * 0.07);
      ctx.strokeStyle = C.red; ctx.lineWidth = r * 0.03;
      const rv = a(marks.vne) - Math.PI / 2;
      ctx.beginPath(); ctx.moveTo(x + Math.cos(rv) * r * 0.97, y + Math.sin(rv) * r * 0.97); ctx.lineTo(x + Math.cos(rv) * r * 0.76, y + Math.sin(rv) * r * 0.76); ctx.stroke();
      ctx.font = `600 ${r * 0.17}px ${FONT}`;
      ticks(ctx, x, y, r * 0.98, marks.dialMin, marks.dialMax, 10, a, r * 0.08, 1.4, r * 0.15, 2);
      ctx.fillStyle = C.mark; ctx.font = `600 ${r * 0.12}px ${FONT}`; ctx.textAlign = 'center';
      ctx.fillText('AIRSPEED', x, y - r * 0.3); ctx.fillText('KNOTS', x, y + r * 0.32);
      needle(ctx, x, y, r * 0.8, a(c[K.ias]), r * 0.045);
      glass(ctx, x, y, r);
    }

    function drawAi(ctx, x, y, r, c) {
      ctx.save();
      ctx.fillStyle = C.bezel; ctx.beginPath(); ctx.arc(x, y, r + 7, 0, 2 * Math.PI); ctx.fill();
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.clip();
      const pitch = c[K.aiPitch] * DEG, roll = c[K.aiRoll] * DEG, ppx = r * 0.032 / DEG;  // px per rad
      ctx.translate(x, y); ctx.rotate(-roll);
      ctx.translate(0, pitch * ppx);
      ctx.fillStyle = C.sky; ctx.fillRect(-3 * r, -6 * r, 6 * r, 6 * r);
      ctx.fillStyle = C.earth; ctx.fillRect(-3 * r, 0, 6 * r, 6 * r);
      ctx.strokeStyle = C.mark; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(-3 * r, 0); ctx.lineTo(3 * r, 0); ctx.stroke();
      ctx.lineWidth = 1.3; ctx.font = `600 ${r * 0.12}px ${FONT}`; ctx.fillStyle = C.mark; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      for (const p of [-20, -15, -10, -5, 5, 10, 15, 20]) {
        const yy = -p * DEG * ppx, l = p % 10 === 0 ? r * 0.28 : r * 0.14;
        ctx.beginPath(); ctx.moveTo(-l, yy); ctx.lineTo(l, yy); ctx.stroke();
        if (p % 10 === 0) ctx.fillText(String(Math.abs(p)), l + 4, yy);
      }
      ctx.restore();
      // Bank scale (fixed to the case) and the roll pointer (on the gyro).
      ctx.save(); ctx.translate(x, y);
      ctx.strokeStyle = C.mark; ctx.lineWidth = 2;
      for (const b of [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]) {
        const a = (b - 90) * DEG, l = Math.abs(b) % 30 === 0 ? r * 0.16 : r * 0.09;
        ctx.beginPath(); ctx.moveTo(Math.cos(a) * r, Math.sin(a) * r); ctx.lineTo(Math.cos(a) * (r - l), Math.sin(a) * (r - l)); ctx.stroke();
      }
      ctx.fillStyle = C.mark; ctx.beginPath(); ctx.moveTo(0, -r); ctx.lineTo(-r * 0.06, -r * 0.88); ctx.lineTo(r * 0.06, -r * 0.88); ctx.fill();
      ctx.rotate(-roll); ctx.fillStyle = C.amber;
      ctx.beginPath(); ctx.moveTo(0, -r * 0.84); ctx.lineTo(-r * 0.06, -r * 0.72); ctx.lineTo(r * 0.06, -r * 0.72); ctx.fill();
      ctx.restore();
      // Miniature airplane (fixed).
      ctx.strokeStyle = C.amber; ctx.lineWidth = r * 0.05; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(x - r * 0.5, y); ctx.lineTo(x - r * 0.16, y); ctx.lineTo(x - r * 0.08, y + r * 0.08); ctx.moveTo(x + r * 0.5, y); ctx.lineTo(x + r * 0.16, y); ctx.lineTo(x + r * 0.08, y + r * 0.08); ctx.stroke();
      ctx.fillStyle = C.amber; ctx.beginPath(); ctx.arc(x, y, r * 0.035, 0, 2 * Math.PI); ctx.fill();
      ctx.lineCap = 'butt';
      glass(ctx, x, y, r);
    }

    function drawAlt(ctx, x, y, r, c) {
      dial(ctx, x, y, r);
      const alt = c[K.alt];
      ctx.font = `600 ${r * 0.2}px ${FONT}`;
      ticks(ctx, x, y, r * 0.98, 0, 9.8, 0.2, (v) => (v / 10) * 2 * Math.PI, r * 0.08, 1.4, r * 0.17, 5, (v) => String(Math.round(v)));
      // Kollsman window.
      ctx.fillStyle = '#000'; ctx.fillRect(x + r * 0.28, y - r * 0.1, r * 0.48, r * 0.2);
      ctx.fillStyle = C.mark; ctx.font = `${r * 0.15}px ${MONO}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(c[K.kollsman].toFixed(2), x + r * 0.52, y + 1);
      ctx.font = `600 ${r * 0.1}px ${FONT}`; ctx.fillStyle = C.dim; ctx.fillText('ALT · FEET', x, y - r * 0.32);
      // 10,000 ft pointer, 1,000 ft hand, 100 ft hand.
      const a = (v, per) => ((((v % per) + per) % per) / per) * 2 * Math.PI;
      needle(ctx, x, y, r * 0.86, a(alt, 100000), r * 0.018, 0.05, C.mark);
      needle(ctx, x, y, r * 0.48, a(alt, 10000), r * 0.06, 0.15);
      needle(ctx, x, y, r * 0.8, a(alt, 1000), r * 0.04, 0.2);
      knob(ctx, ...KNOB.kollsman);
      glass(ctx, x, y, r);
    }
    function knob(ctx, x, y) {
      ctx.fillStyle = '#26292c'; ctx.beginPath(); ctx.arc(x, y, 14, 0, 2 * Math.PI); ctx.fill();
      ctx.strokeStyle = '#4a4f54'; ctx.lineWidth = 1.5;
      for (let i = 0; i < 12; ++i) { const a = i * Math.PI / 6; ctx.beginPath(); ctx.moveTo(x + Math.cos(a) * 10, y + Math.sin(a) * 10); ctx.lineTo(x + Math.cos(a) * 14, y + Math.sin(a) * 14); ctx.stroke(); }
    }

    function drawVsi(ctx, x, y, r, c) {
      dial(ctx, x, y, r);
      const range = marks.vsiRange;
      const a = (fpm) => -Math.PI / 2 + (clamp(fpm, -range * 1.08, range * 1.08) / range) * 170 * DEG;  // 0 at 9 o'clock
      ctx.font = `600 ${r * 0.18}px ${FONT}`;
      ticks(ctx, x, y, r * 0.98, -range, range, 100, a, r * 0.08, 1.3, r * 0.17, 5, (v) => String(Math.abs(v / 100) | 0));
      ctx.fillStyle = C.dim; ctx.font = `600 ${r * 0.1}px ${FONT}`; ctx.textAlign = 'center';
      ctx.fillText('UP', x - r * 0.35, y - r * 0.25); ctx.fillText('DOWN', x - r * 0.35, y + r * 0.27); ctx.fillText('100 FT/MIN', x + r * 0.2, y);
      needle(ctx, x, y, r * 0.8, a(c[K.vsi]), r * 0.045);
      glass(ctx, x, y, r);
    }

    function drawHi(ctx, x, y, r, c) {
      dial(ctx, x, y, r);
      ctx.save(); ctx.translate(x, y); ctx.rotate(-c[K.dg] * DEG);
      ctx.strokeStyle = C.mark; ctx.fillStyle = C.mark; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (let d = 0; d < 360; d += 5) {
        const a = (d - 90) * DEG, l = d % 10 === 0 ? r * 0.12 : r * 0.07;
        ctx.lineWidth = d % 30 === 0 ? 2 : 1.2;
        ctx.beginPath(); ctx.moveTo(Math.cos(a) * r * 0.96, Math.sin(a) * r * 0.96); ctx.lineTo(Math.cos(a) * (r * 0.96 - l), Math.sin(a) * (r * 0.96 - l)); ctx.stroke();
        if (d % 30 === 0) {
          ctx.save(); ctx.rotate(d * DEG); ctx.font = `600 ${r * 0.19}px ${FONT}`;
          ctx.fillText({ 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[d] ?? String(d / 10), 0, -r * 0.66); ctx.restore();
        }
      }
      // Heading bug.
      ctx.rotate(c[K.bug] * DEG); ctx.fillStyle = C.accent;
      ctx.fillRect(-r * 0.07, -r * 0.99, r * 0.14, r * 0.07);
      ctx.restore();
      // Lubber line and aircraft.
      ctx.fillStyle = C.amber; ctx.beginPath(); ctx.moveTo(x, y - r * 0.82); ctx.lineTo(x - r * 0.05, y - r * 0.97); ctx.lineTo(x + r * 0.05, y - r * 0.97); ctx.fill();
      ctx.strokeStyle = C.amber; ctx.lineWidth = r * 0.04; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(x, y - r * 0.3); ctx.lineTo(x, y + r * 0.3); ctx.moveTo(x - r * 0.28, y - r * 0.02); ctx.lineTo(x + r * 0.28, y - r * 0.02); ctx.moveTo(x - r * 0.12, y + r * 0.26); ctx.lineTo(x + r * 0.12, y + r * 0.26); ctx.stroke();
      ctx.lineCap = 'butt';
      knob(ctx, ...KNOB.hdgSet); knob(ctx, ...KNOB.hdgBug);
      ctx.fillStyle = C.dim; ctx.font = `600 11px ${FONT}`; ctx.textAlign = 'center';
      ctx.fillText('SET', KNOB.hdgSet[0], KNOB.hdgSet[1] + 24); ctx.fillText('HDG', KNOB.hdgBug[0], KNOB.hdgBug[1] + 24);
      glass(ctx, x, y, r);
    }

    function drawTc(ctx, x, y, r, c) {
      dial(ctx, x, y, r);
      ctx.fillStyle = C.mark; ctx.font = `600 ${r * 0.13}px ${FONT}`; ctx.textAlign = 'center';
      ctx.fillText('TURN COORDINATOR', x, y - r * 0.55);
      ctx.fillText('2 MIN', x, y + r * 0.75);
      ctx.fillText('L', x - r * 0.72, y + r * 0.42); ctx.fillText('R', x + r * 0.72, y + r * 0.42);
      // Standard-rate marks at the calibrated deflection, wings-level marks.
      ctx.strokeStyle = C.mark; ctx.lineWidth = 3;
      for (const a of [0, marks.stdRateDefl, -marks.stdRateDefl]) {
        for (const side of [-1, 1]) {
          const ang = (a * DEG) + (side < 0 ? Math.PI : 0);
          ctx.beginPath(); ctx.moveTo(x + Math.cos(ang) * r * 0.78, y + Math.sin(ang) * r * 0.78); ctx.lineTo(x + Math.cos(ang) * r * 0.94, y + Math.sin(ang) * r * 0.94); ctx.stroke();
        }
      }
      // Miniature airplane.
      ctx.save(); ctx.translate(x, y); ctx.rotate(c[K.tcDefl] * DEG);
      ctx.strokeStyle = C.mark; ctx.fillStyle = C.mark; ctx.lineWidth = r * 0.055; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(-r * 0.7, 0); ctx.lineTo(r * 0.7, 0); ctx.stroke();
      ctx.beginPath(); ctx.arc(0, 0, r * 0.1, 0, 2 * Math.PI); ctx.fill();
      ctx.beginPath(); ctx.moveTo(-r * 0.18, -r * 0.2); ctx.lineTo(r * 0.18, -r * 0.2); ctx.moveTo(0, -r * 0.08); ctx.lineTo(0, -r * 0.28); ctx.stroke();
      ctx.restore(); ctx.lineCap = 'butt';
      // Inclinometer: curved glass tube, ball, reference wires.
      const ty = y + r * 0.42, tr = r * 1.6, limit = 16;
      ctx.save();
      ctx.strokeStyle = '#d8d6cc'; ctx.lineWidth = r * 0.16; ctx.globalAlpha = 0.25;
      ctx.beginPath(); ctx.arc(x, ty - tr, tr, Math.PI / 2 - 0.2, Math.PI / 2 + 0.2); ctx.stroke();
      ctx.globalAlpha = 1;
      const ba = Math.PI / 2 - clamp(c[K.ball], -limit, limit) / limit * 0.18;
      ctx.fillStyle = '#0a0a0a'; ctx.strokeStyle = C.mark; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(x + Math.cos(ba) * tr, ty - tr + Math.sin(ba) * tr, r * 0.065, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.strokeStyle = C.mark; ctx.lineWidth = 1.5;
      for (const s of [-1, 1]) { ctx.beginPath(); ctx.moveTo(x + s * r * 0.085, ty - r * 0.09); ctx.lineTo(x + s * r * 0.085, ty + r * 0.09); ctx.stroke(); }
      ctx.restore();
      if (c[K.tcFlag]) flag(ctx, x - r * 0.48, y - r * 0.3, 'OFF');
      glass(ctx, x, y, r);
    }
    function flag(ctx, x, y, text) {
      ctx.fillStyle = C.flag; ctx.fillRect(x - 18, y - 9, 36, 18);
      ctx.fillStyle = '#fff'; ctx.font = `700 13px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(text, x, y + 1);
    }

    function drawNav(ctx, x, y, r, c) {
      dial(ctx, x, y, r);
      // OBS card (course at the top).
      ctx.save(); ctx.translate(x, y); ctx.rotate(-c[K.obs] * DEG);
      ctx.strokeStyle = C.mark; ctx.fillStyle = C.mark; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (let d = 0; d < 360; d += 5) {
        const a = (d - 90) * DEG, l = d % 10 === 0 ? r * 0.1 : r * 0.05;
        ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.moveTo(Math.cos(a) * r * 0.97, Math.sin(a) * r * 0.97); ctx.lineTo(Math.cos(a) * (r * 0.97 - l), Math.sin(a) * (r * 0.97 - l)); ctx.stroke();
        if (d % 30 === 0) { ctx.save(); ctx.rotate(d * DEG); ctx.font = `600 ${r * 0.15}px ${FONT}`; ctx.fillText({ 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[d] ?? String(d / 10), 0, -r * 0.74); ctx.restore(); }
      }
      ctx.restore();
      ctx.fillStyle = C.amber; ctx.beginPath(); ctx.moveTo(x, y - r * 0.86); ctx.lineTo(x - r * 0.05, y - r * 0.98); ctx.lineTo(x + r * 0.05, y - r * 0.98); ctx.fill();
      // Dots: 5 each side (2 deg/dot VOR, 0.5 deg LOC).
      ctx.fillStyle = C.mark;
      for (let i = -5; i <= 5; ++i) if (i) { ctx.beginPath(); ctx.arc(x + i * r * 0.11, y, r * 0.022, 0, 2 * Math.PI); ctx.fill(); }
      ctx.strokeStyle = C.mark; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.arc(x, y, r * 0.08, 0, 2 * Math.PI); ctx.stroke();
      const valid = c[K.navValid] > 0;
      const dx = valid ? clamp(c[K.cdi], -1, 1) * r * 0.55 : 0;
      ctx.strokeStyle = C.mark; ctx.lineWidth = r * 0.035;
      ctx.beginPath(); ctx.moveTo(x + dx, y - r * 0.55); ctx.lineTo(x + dx, y + r * 0.55); ctx.stroke();
      // Glideslope.
      if (c[K.loc] > 0) {
        for (let i = -4; i <= 4; ++i) if (i) { ctx.beginPath(); ctx.arc(x + r * 0.72, y + i * r * 0.12, r * 0.02, 0, 2 * Math.PI); ctx.fill(); }
        const gy = c[K.gsValid] ? -clamp(c[K.gs], -1, 1) * r * 0.48 : 0;
        ctx.beginPath(); ctx.moveTo(x - r * 0.55, y + gy); ctx.lineTo(x + r * 0.55, y + gy); ctx.stroke();
        if (!c[K.gsValid]) flag(ctx, x + r * 0.62, y - r * 0.55, 'GS');
      }
      // TO/FROM / NAV flag.
      if (!valid) flag(ctx, x - r * 0.45, y - r * 0.35, 'NAV');
      else if (c[K.toFrom] !== 0) {
        ctx.fillStyle = C.mark; ctx.font = `700 ${r * 0.13}px ${FONT}`; ctx.textAlign = 'center';
        ctx.fillText(c[K.toFrom] > 0 ? 'TO' : 'FR', x + r * 0.42, y - r * 0.32);
      }
      knob(ctx, ...KNOB.obs);
      ctx.fillStyle = C.dim; ctx.font = `600 11px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText('OBS', KNOB.obs[0], KNOB.obs[1] + 24);
      glass(ctx, x, y, r);
    }

    function smallGauge(ctx, x, y, r, title, value, min, max, bands, fmt, labels) {
      dial(ctx, x, y, r);
      const a = (v) => (-60 + (clamp(v, min, max) - min) / (max - min) * 120) * DEG;  // 120 deg sweep at the top
      for (const [lo, hi, col] of bands) arcBand(ctx, x, y + r * 0.35, r * 0.95, a(lo), a(hi), col, r * 0.09);
      ctx.fillStyle = C.mark; ctx.font = `600 ${r * 0.24}px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(title, x, y + r * 0.6);
      if (labels) { ctx.font = `600 ${r * 0.2}px ${FONT}`; ctx.fillStyle = C.dim; ctx.fillText(labels[0], x - r * 0.62, y - r * 0.15); ctx.fillText(labels[1], x + r * 0.62, y - r * 0.15); }
      needle(ctx, x, y + r * 0.35, r * 0.92, a(value), r * 0.05, 0.05);
      if (fmt) { ctx.fillStyle = C.lcd; ctx.font = `${r * 0.22}px ${MONO}`; ctx.fillText(fmt(value), x, y + r * 0.86); }
      glass(ctx, x, y, r);
    }

    function drawCompass(ctx, x, y, w, h, c) {
      ctx.fillStyle = '#121416'; ctx.fillRect(x - w / 2 - 6, y - h / 2 - 6, w + 12, h + 12);
      ctx.save(); ctx.beginPath(); ctx.rect(x - w / 2, y - h / 2, w, h); ctx.clip();
      ctx.fillStyle = '#e9e4d6'; ctx.fillRect(x - w / 2, y - h / 2, w, h);
      const hd = c[K.compass], ppd = w / 60;
      ctx.fillStyle = '#111'; ctx.strokeStyle = '#111'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      for (let d = Math.floor(hd - 40); d <= hd + 40; ++d) {
        if (d % 5) continue;
        const xx = x - (d - hd) * ppd, dd = ((d % 360) + 360) % 360;
        ctx.lineWidth = dd % 10 === 0 ? 1.5 : 1;
        ctx.beginPath(); ctx.moveTo(xx, y + h / 2); ctx.lineTo(xx, y + h / 2 - (dd % 10 === 0 ? h * 0.3 : h * 0.18)); ctx.stroke();
        if (dd % 30 === 0) { ctx.font = `700 ${h * 0.42}px ${FONT}`; ctx.fillText({ 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[dd] ?? String(dd / 10), xx, y - h * 0.12); }
      }
      ctx.restore();
      ctx.strokeStyle = C.red; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(x, y - h / 2 - 4); ctx.lineTo(x, y + h / 2 + 4); ctx.stroke();
      ctx.fillStyle = C.dim; ctx.font = `600 11px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText('MAGNETIC COMPASS', x, y + h / 2 + 18);
      // Correction card from the compass swing (C++), placarded beside it.
      const cy = y + h / 2 + 28, cw = 20, x0 = x - 3 * cw - 8;
      ctx.fillStyle = '#e9e4d6'; ctx.fillRect(x0 - 26, cy, 6 * cw + 34, 52);
      ctx.fillStyle = '#111'; ctx.font = `600 9px ${FONT}`;
      const names = ['N', '30', '60', 'E', '120', '150', 'S', '210', '240', 'W', '300', '330'];
      for (let r = 0; r < 2; ++r) {
        const ry = cy + 11 + r * 25;
        ctx.textAlign = 'left';
        ctx.fillText('FOR', x0 - 23, ry); ctx.fillText('STEER', x0 - 23, ry + 11);
        ctx.textAlign = 'center';
        for (let k = 0; k < 6; ++k) {
          const i = r * 6 + k, xx = x0 + 10 + k * cw;
          ctx.fillText(names[i], xx, ry);
          ctx.fillText(String(Math.round(compassCard[i]) % 360).padStart(3, '0'), xx, ry + 11);
        }
      }
    }

    let compassCard = A.compassCard();
    const annNames = [['OIL PRESS', 1], ['LOW VAC', 2], ['VOLTS', 4], ['L LOW FUEL', 8], ['R LOW FUEL', 16]];
    function drawPanel(c) {
      const ctx = pctx, w = panel.width, h = panel.height;
      const s = Math.min(w / PW, h / PH);
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.fillStyle = C.panel; ctx.fillRect(0, 0, w, h);
      ctx.setTransform(s, 0, 0, s, (w - PW * s) / 2, (h - PH * s) / 2);
      // Six-pack.
      drawAsi(ctx, 120, 115, 88, c);
      drawAi(ctx, 320, 115, 88, c);
      drawAlt(ctx, 520, 115, 88, c);
      drawTc(ctx, 120, 315, 88, c);
      drawHi(ctx, 320, 315, 88, c);
      drawVsi(ctx, 520, 315, 88, c);
      drawNav(ctx, 720, 115, 80, c);
      drawCompass(ctx, 720, 290, 150, 40, c);
      // Engine and fuel cluster.
      const G = C.green, Y = C.yellow, R = C.red, W = C.white;
      const gx = [870, 980, 1090], gy = [80, 210, 340];
      smallGauge(ctx, gx[0], gy[0], 50, 'FUEL L', c[K.fuelL], 0, marks.fuelFull, [[0, 1.5, R]], (v) => v.toFixed(1) + ' gal', ['E', 'F']);
      smallGauge(ctx, gx[1], gy[0], 50, 'FUEL R', c[K.fuelR], 0, marks.fuelFull, [[0, 1.5, R]], (v) => v.toFixed(1) + ' gal', ['E', 'F']);
      smallGauge(ctx, gx[2], gy[0], 50, 'FUEL FLOW', c[K.ff], 0, 19, [[0, 19, W]], (v) => v.toFixed(1) + ' gph');
      smallGauge(ctx, gx[0], gy[1], 50, 'OIL TEMP', c[K.oilT], 75, 250, [[marks.oilTGreen[0], marks.oilTGreen[1], G], [marks.oilTRed, 250, R]], (v) => v > 0 ? v.toFixed(0) + ' °F' : '—');
      smallGauge(ctx, gx[1], gy[1], 50, 'OIL PRESS', c[K.oilP], 0, 120, [[0, marks.oilPRedLo, R], [marks.oilPGreen[0], marks.oilPGreen[1], G], [marks.oilPRedHi, 120, R]], (v) => v.toFixed(0) + ' psi');
      smallGauge(ctx, gx[2], gy[1], 50, 'EGT', c[K.egt], 1000, 1700, [[1000, 1700, W]], (v) => v > 0 ? v.toFixed(0) + ' °F' : '—');
      smallGauge(ctx, gx[0], gy[2], 50, 'SUCTION', c[K.suction], 0, 7, [[marks.suctionGreen[0], marks.suctionGreen[1], G]], (v) => v.toFixed(1) + ' inHg');
      smallGauge(ctx, gx[1], gy[2], 50, 'AMPS', c[K.amps], -60, 60, [], (v) => (v >= 0 ? '+' : '') + v.toFixed(0) + ' A', ['−', '+']);
      // Tachometer.
      dial(ctx, 1090, 340, 50);
      const ta = (rpm) => (-130 + clamp(rpm, 0, marks.tachMax) / marks.tachMax * 260) * DEG;
      arcBand(ctx, 1090, 340, 44, ta(marks.tachGreen[0]), ta(marks.tachGreen[1]), G, 6);
      ctx.strokeStyle = R; ctx.lineWidth = 3; const tr = ta(marks.tachRed) - Math.PI / 2;
      ctx.beginPath(); ctx.moveTo(1090 + Math.cos(tr) * 50, 340 + Math.sin(tr) * 50); ctx.lineTo(1090 + Math.cos(tr) * 36, 340 + Math.sin(tr) * 36); ctx.stroke();
      ctx.font = `600 11px ${FONT}`; ticks(ctx, 1090, 340, 49, 0, marks.tachMax, 500, ta, 5, 1.2, 11, 2, (v) => String(v / 100));
      ctx.fillStyle = C.mark; ctx.font = `600 10px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText('RPM × 100', 1090, 362);
      ctx.fillStyle = C.lcd; ctx.font = `12px ${MONO}`; ctx.fillText(c[K.rpm].toFixed(0), 1090, 378);
      needle(ctx, 1090, 340, 42, ta(c[K.rpm]), 2.6);
      // Annunciators.
      ctx.font = `700 13px ${FONT}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      annNames.forEach(([n, bit], i) => {
        const x = 70 + i * 104, y = 450, lit = (c[K.ann] & bit) !== 0;
        ctx.fillStyle = lit ? C.amber : '#1c1e1f'; ctx.fillRect(x - 46, y - 14, 92, 28);
        ctx.fillStyle = lit ? '#1a1000' : '#45484a'; ctx.fillText(n, x, y + 1);
      });
      // Flap position, trim, throttle, mixture.
      ctx.fillStyle = C.dim; ctx.font = `600 12px ${FONT}`; ctx.textAlign = 'left';
      ctx.fillText('FLAPS', 30, 510);
      ctx.fillStyle = '#0c0d0e'; ctx.fillRect(80, 500, 160, 18);
      ctx.fillStyle = C.mark; ctx.fillRect(80 + clamp(c[K.flaps] / 30, 0, 1) * 152, 498, 8, 22);
      ctx.font = `600 11px ${FONT}`; ctx.fillStyle = C.dim; ['0', '10', '20', '30'].forEach((t, i) => ctx.fillText(t, 80 + i * 50.6, 534));
      ctx.fillText('TRIM', 270, 510);
      ctx.fillStyle = '#0c0d0e'; ctx.fillRect(310, 494, 18, 60);
      ctx.fillStyle = C.mark; ctx.fillRect(306, 524 - clamp(c[K.trim], -1, 1) * 28 - 2, 26, 4);
      ctx.fillStyle = C.dim; ctx.fillText('UP', 334, 500); ctx.fillText('DN', 334, 552);
      lever(ctx, 400, 'THROTTLE', levers.throttle, '#111');
      lever(ctx, 470, 'MIXTURE', levers.mixture, C.red);
      // Clock / OAT.
      ctx.fillStyle = '#0c0d0e'; ctx.fillRect(560, 470, 210, 74);
      ctx.fillStyle = C.lcd; ctx.font = `22px ${MONO}`; ctx.textAlign = 'left';
      ctx.fillText(fmtTime(c[K.clock]), 576, 497);
      ctx.fillText(`${c[K.oat].toFixed(0).padStart(3, ' ')}°C`, 676, 497);
      ctx.font = `600 11px ${FONT}`; ctx.fillStyle = C.dim; ctx.fillText('CLOCK', 576, 530); ctx.fillText('OAT', 676, 530);
      ctx.fillText(`XPDR ${String(c[K.squawk]).padStart(4, '0')} ${xpdrModes[c[K.xpdr]] ?? ''}${c[K.altRep] ? ' · ' + c[K.repAlt] + ' ft' : ''}`, 576, 556);
      // Stall horn (an audible reed; shown here for the deaf and the muted).
      if (c[K.stall]) { ctx.fillStyle = C.red; ctx.font = `700 15px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText('STALL HORN', 920, 470); }
      if (c[K.aiTumbled]) { ctx.fillStyle = C.amber; ctx.font = `700 12px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText('ATTITUDE GYRO TUMBLED', 320, 220); }
      // Night lighting: post lights tint the panel when the panel lights are on.
      if (c[K.panelLights]) { ctx.fillStyle = `rgba(255,190,120,${0.05 + 0.08 * c[K.dimmer]})`; ctx.fillRect(0, 0, PW, PH); }
    }
    function lever(ctx, x, label, v, knobColor) {
      ctx.fillStyle = '#0c0d0e'; ctx.fillRect(x, 480, 10, 90);
      const y = 565 - v * 80;
      ctx.fillStyle = knobColor; ctx.strokeStyle = C.mark; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x + 5, y, 14, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.fillStyle = C.dim; ctx.font = `600 11px ${FONT}`; ctx.textAlign = 'center'; ctx.fillText(label, x + 5, 590);
    }

    // --------------------------------------------------------- readouts ----
    function updateHtml(c, s) {
      $('com-a').textContent = c[K.avionics] ? c[K.comA].toFixed(3) : '';
      $('com-s').textContent = c[K.avionics] ? c[K.comS].toFixed(3) : '';
      $('nav-a').textContent = c[K.avionics] ? c[K.navA].toFixed(2) : '';
      $('nav-s').textContent = c[K.avionics] ? c[K.navS].toFixed(2) : '';
      $('xpdr-code').textContent = c[K.avionics] ? String(c[K.squawk]).padStart(4, '0') + (squawkEntry ? ' ← ' + squawkEntry : '') : '';
      $('xpdr-mode').textContent = xpdrModes[c[K.xpdr]];
      $('xpdr-alt').textContent = c[K.altRep] ? `ALT ${c[K.repAlt]} ft` : '';
      $('stack').classList.toggle('dark', !c[K.avionics]);
      const proc = A.procedure();
      $('status').textContent = replay ? 'Replay (read-only): “Back to live” returns to the simulation.' : paused ? 'Paused — Space resumes.' : proc.status === 1 ? `Quick Start: ${proc.message}` :
        `Throttle ${(input.throttle * 100).toFixed(0)} % · mixture ${(input.mixture * 100).toFixed(0)} % · ${gamepadOn ? 'gamepad connected' : 'keyboard'}${s[49] ? ' · parking brake SET' : ''}` +
        (s[55] ? ' · holding straight and level' : levelHold.checked ? ' · auto-trim armed' : '') + (s[56] ? ' · Q/E on the tiller' : '');
      if (showDebug) {
        const f = (v, d) => (v === null ? '—' : v.toFixed(d));
        const row = (n, truth, shown, unit, d = 0) => `<tr><th>${n}</th><td>${f(truth, d)}</td><td>${f(shown, d)}</td><td>${unit}</td></tr>`;
        $('debug-body').innerHTML =
          row('Airspeed: TAS · ASI', c[K.tTas], c[K.ias], 'kt') + row('Calibrated airspeed · ASI', c[K.tCas], c[K.ias], 'kt') +
          row('Equivalent airspeed', c[K.tEas], null, 'kt') + row('Ground speed', c[K.tGs], null, 'kt') +
          row('Altitude MSL · altimeter', c[K.tAlt], c[K.alt], 'ft') + row('Pressure altitude · encoder', c[K.tPalt], c[K.altRep] ? c[K.repAlt] : null, 'ft') +
          row('Density altitude', c[K.tDalt], null, 'ft') + row('Height above ground', c[K.tAgl], null, 'ft') +
          row('Vertical speed · VSI', c[K.tVs], c[K.vsi], 'fpm') + row('Pitch · attitude indicator', c[K.tPitch], c[K.aiPitch], '°', 1) +
          row('Bank · attitude indicator', c[K.tRoll], c[K.aiRoll], '°', 1) + row('Magnetic heading · heading indicator', c[K.tMag], c[K.dg], '°') +
          row('Magnetic heading · compass', c[K.tMag], c[K.compass], '°') + row('True heading', c[K.tHdg], null, '°') +
          row('Yaw rate · turn coordinator', c[K.tR], c[K.tcRate], '°/s', 1) + row('Sideslip · ball', c[K.tSlip], c[K.ball], '°', 1) +
          row('Angle of attack', c[K.tAlpha], null, '°', 1) + row('Roll rate', c[K.tP], null, '°/s', 1) + row('Pitch rate', c[K.tQ], null, '°/s', 1) +
          row('Gyro rotor: attitude', c[K.aiRotor] * 100, null, '%') + row('Gyro rotor: heading', c[K.dgRotor] * 100, null, '%') +
          row('Gyro rotor: turn coordinator', c[K.tcRotor] * 100, null, '%') + row('Bus voltage', c[K.volts], null, 'V', 1) +
          row('Ammeter', null, c[K.amps], 'A', 1) + row('Suction', null, c[K.suction], 'inHg', 1) +
          row('Cloud liquid water', c[K.cloudWater], null, 'g/m³', 2) + row('Pitot ice (of inlet-closing amount)', c[K.pitotIce] * 100, null, '%') +
          row('Pitot heater warm-up', c[K.pitotHeat] * 100, null, '%');
      }
    }

    // ----------------------------------------------------------- replay ----
    // Every 0.1 s of simulated time the live flight is recorded (last 5 min).
    // Replay draws recorded frames through the same panel; a CSV written by
    // `avdemo instruments csv` (InstrumentRecorder) replays a native flight.
    const REC_MAX = 3000;
    const recording = [];
    let lastRecT = -1, replay = null;  // replay: {frames, i, playing, speed, source}
    const levers = { throttle: 0, mixture: 1 };
    function record(s, c) {
      const t = s[42];
      if (t < lastRecT) { recording.length = 0; lastRecT = -1; }  // a reset restarted the clock
      if (t - lastRecT < 0.1 - 1e-9) return;
      lastRecT = t;
      recording.push({ t, s: Float64Array.from(s), c: Float64Array.from(c) });
      if (recording.length > REC_MAX) recording.shift();
    }
    const qmul = (a, b) => [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
    function enuFluFromEuler(roll, pitch, yaw) {  // as world::enuFluFromNedFrd(Quat::fromEuler)
      const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2), cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
      const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
      const q = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy];
      const r = Math.SQRT1_2;
      return qmul(qmul([0, r, r, 0], q), [0, 1, 0, 0]);
    }
    function framesFromCsv(text) {
      const lines = text.trim().split(/\r?\n/);
      const head = lines[0].split(',');
      const col = Object.fromEntries(head.map((h, i) => [h.trim(), i]));
      const need = ['time_s', 'ias_kt', 'alt_indicated_ft', 'pitch_ai_deg', 'roll_ai_deg', 'hdg_dg_deg'];
      const missing = need.filter((n) => !(n in col));
      if (missing.length) throw new Error('not an instrument CSV (missing ' + missing.join(', ') + ')');
      const map = { ias_kt: 0, alt_indicated_ft: 1, kollsman_inhg: 2, vsi_fpm: 3, pitch_ai_deg: 4, roll_ai_deg: 5, ai_tumbled: 6,
        hdg_dg_deg: 7, turn_rate_tc_dps: 10, tc_flag: 11, ball_deg: 12, hdg_compass_deg: 13, rpm_tach: 14, oil_psi: 15, oil_f: 16,
        egt_f: 17, fuel_gph: 18, fuel_l_gal: 19, fuel_r_gal: 20, ammeter_a: 21, bus_v: 22, suction_inhg: 23, annunciators: 24,
        stall_horn: 25, oat_c: 26, time_s: 27, nav_valid: 37, nav_cdi: 38, nav_tofrom: 39, gs_valid: 40, gs_dev: 41, squawk: 44,
        tas_kt: 48, cas_kt: 49, gs_kt: 50, alt_true_ft: 51, alt_pressure_ft: 52, vs_true_fpm: 53, pitch_true_deg: 54, roll_true_deg: 55,
        hdg_true_deg: 56, hdg_mag_deg: 57, p_dps: 58, q_dps: 59, r_dps: 60, slip_true_deg: 61, ai_rotor: 62, dg_rotor: 63, tc_rotor: 64, eas_kt: 65 };
      const frames = [];
      for (const line of lines.slice(1)) {
        const v = line.split(',').map(Number);
        const get = (n, d = 0) => (n in col && Number.isFinite(v[col[n]]) ? v[col[n]] : d);
        const c = new Float64Array(96), s = new Float64Array(64);
        for (const [name, idx] of Object.entries(map)) c[idx] = get(name);
        c[K.tcDefl] = get('turn_rate_tc_dps') / 3 * marks.stdRateDefl;
        const rep = get('reported_alt_ft', -99999);
        c[K.repAlt] = rep; c[K.altRep] = rep > -99999 ? 1 : 0;
        c[K.avionics] = 'nav_valid' in col ? 1 : 0; c[K.loc] = get('gs_valid'); c[K.xpdr] = 'squawk' in col ? 3 : 0;
        c[K.obs] = get('hdg_mag_deg'); c[K.bug] = get('hdg_dg_deg');
        // Outside view: world position and true attitude.
        s[0] = get('east_m'); s[1] = get('north_m'); s[2] = get('alt_true_ft') * 0.3048;
        const q = enuFluFromEuler(get('roll_true_deg') * DEG, get('pitch_true_deg') * DEG, get('hdg_true_deg') * DEG);
        s[3] = q[0]; s[4] = q[1]; s[5] = q[2]; s[6] = q[3];
        s[24] = NaN; s[25] = NaN; s[42] = get('time_s');
        frames.push({ t: s[42], s, c });
      }
      return frames;
    }
    const rp = { bar: $('replay-bar'), info: $('replay-info'), play: $('replay-play') };
    function enterReplay(frames, source) {
      if (!frames.length) { log('Nothing recorded yet.'); return; }
      replay = { frames, i: 0, playing: true, speed: 1, acc: 0, source };
      rp.bar.max = String(frames.length - 1);
      rp.bar.disabled = false;
      $('replay-live').disabled = false;
      rp.play.disabled = false;
      log(`Replay: ${source}, ${frames.length} frames (${((frames.at(-1).t - frames[0].t) / 60).toFixed(1)} min).`);
    }
    function leaveReplay() {
      replay = null;
      rp.bar.disabled = true; $('replay-live').disabled = true; rp.play.disabled = true;
      rp.info.textContent = 'Live. The last 5 minutes are recorded.';
      log('Back to live flight.');
    }
    $('replay-last').addEventListener('click', () => enterReplay(recording.slice(), 'this flight'));
    $('replay-live').addEventListener('click', leaveReplay);
    rp.play.addEventListener('click', () => { if (replay) replay.playing = !replay.playing; });
    $('replay-speed').addEventListener('change', (e) => { if (replay) replay.speed = +e.target.value; });
    rp.bar.addEventListener('input', () => { if (replay) { replay.i = +rp.bar.value; replay.playing = false; } });
    $('replay-csv').addEventListener('change', async (e) => {
      const f = e.target.files?.[0];
      if (!f) return;
      try { enterReplay(framesFromCsv(await f.text()), f.name); } catch (err) { log('CSV not loaded: ' + err.message); }
      e.target.value = '';
    });

    // ------------------------------------------------------------- loop ----
    reset();
    applyWeather();
    let last = performance.now(), htmlAcc = 0, hornLevel = 0;
    function frame(now) {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      let s, c;
      if (replay) {
        const fr = replay.frames;
        if (replay.playing) {
          replay.acc += dt * replay.speed;
          while (replay.i < fr.length - 1 && fr[replay.i + 1].t - fr[replay.i].t <= replay.acc) { replay.acc -= fr[replay.i + 1].t - fr[replay.i].t; ++replay.i; }
          if (replay.i >= fr.length - 1) replay.playing = false;
        }
        ({ s, c } = fr[replay.i]);
        rp.bar.value = String(replay.i);
        rp.play.textContent = replay.playing ? 'Pause' : 'Play';
        rp.info.textContent = `Replaying ${replay.source}: ${fmtTime(fr[replay.i].t - fr[0].t)} / ${fmtTime(fr.at(-1).t - fr[0].t)}`;
        levers.throttle = Number.isFinite(s[24]) ? s[24] : 0; levers.mixture = Number.isFinite(s[25]) ? s[25] : 1;
      } else {
        readInputs(dt);
        A.avd_set_input(input.pitch, input.roll, input.yaw, input.throttle, input.mixture, input.brake, input.brake, input.trimRate, input.starter ? 1 : 0);
        if (!paused) A.avd_advance(dt);
        s = Float64Array.from(A.state());
        c = Float64Array.from(A.cockpitState());
        record(s, c);
        levers.throttle = input.throttle; levers.mixture = input.mixture;
      }
      drawView(s);
      drawPanel(c);
      if (!replay) for (const e of A.events()) log(e);
      htmlAcc += dt;
      if (htmlAcc > 0.1) { htmlAcc = 0; updateHtml(c, s); refreshSwitches(); }
      if (audio && hornGain) {
        hornLevel += ((c[K.stall] ? 0.12 : 0) - hornLevel) * Math.min(1, dt * 20);
        hornGain.gain.value = hornLevel;
      }
      requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
    setHover(null);
  };
})();
