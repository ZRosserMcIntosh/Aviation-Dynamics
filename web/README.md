# Browser runtime

The flight core compiles to WebAssembly with wasi-sdk. The build has no
exceptions or RTTI, and the WASI imports are stubs because the core does no
I/O. The C ABI is in `avd_web.cpp`.

```sh
WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh   # → web/dist/avd.wasm, avd.mjs, tupi_t180.json, top_g.json
node web/smoke_test.mjs                        # take-off, Quick Start, ILS reception, instruments
node web/abi_check.mjs                         # the tables below against the module's avd_layout()
node web/cockpit/build_page.mjs                # → web/dist/cockpit.html (one self-contained file)
node web/cockpit/browser_test.mjs              # headless Chromium check of the bundled page (needs playwright)
```

`avd.mjs` exports `createAvdModule(bytes?)`, which returns the raw `avd_*`
exports plus these helpers:

- `load(json)`, which returns a list of errors;
- `state()` and `cockpitState()`, which return Float64Array views (copy them
  before the next call);
- `markings()`, `runway()`, `events()`, `cockpit()`, `procedure()`;
- `abiVersion`, `layout` (every array size and stride from `avd_layout()`)
  and `buildId()`; see *ABI version* below.

## ABI version

The module and the loader must agree on the layout of every array below.
`avd_abi_version()` returns the module's layout version (`kAbiVersion` in
`avd_web.cpp`, currently **1**); `createAvdModule` throws if it is not the
loader's `EXPECTED_ABI`, so a stale `avd.wasm` next to a newer `avd.mjs` (or
the reverse) fails at load instead of reading garbage. **Bump `kAbiVersion`
on any change to a size, a stride or the meaning of an index** in the tables
of this file, and `EXPECTED_ABI` in `avd_loader.mjs` with it; the
`static_assert`s at the end of `avd_web.cpp` and `node web/abi_check.mjs`
(CI) catch a layout that changed without the tables.

`avd_layout(which)` returns the sizes (doubles; bytes for the text entries)
the loader sizes its views with, so the page never carries a literal:

| which | Name | Value | Describes |
|---|---|---|---|
| 0 | `state` | 64 | `avd_state()` |
| 1 | `cockpit` | 96 | `avd_cockpit()` |
| 2 | `jet` | 64 | `avd_jet()` |
| 3 | `loadsheet` | 256 | `avd_loadsheet()` |
| 4 | `flowHeader` | 16 | `avd_flow()` header doubles |
| 5 | `flowStride` | 20 | doubles per lifting strip |
| 6 | `flowStrips` | 64 | strips at most |
| 7 | `flowEngineStride` | 14 | doubles per engine |
| 8 | `flowEngines` | 4 | engines at most |
| 9 | `flow` | 1,352 | `avd_flow()` total |
| 10 | `markings` | 96 | `avd_markings()` (31 fields used) |
| 11 | `compassCard` | 12 | `avd_compass_card()` |
| 12 | `runway` | 6 | `avd_runway()` |
| 13 | `region` | 1,024 | `avd_region()` |
| 14 | `route` | 51 | `avd_route()` (3 + 3 · 16) |
| 15 | `routeMax` | 16 | waypoints at most |
| 16 | `gps` | 16 | `avd_gps()` |
| 17 | `wakeHeader` | 8 | `avd_wake()` header doubles |
| 18 | `wakeStride` | 9 | doubles per vortex segment |
| 19 | `wakeMax` | 4,096 | segments at most |
| 20 | `bodiesMax` | 32 | `avd_bodies()` bodies at most |
| 21 | `bodyStride` | 10 | doubles per body |
| 22 | `bodies` | 321 | `avd_bodies()` total (1 + 32 · 10) |
| 23 | `windBoxMax` | 20,736 | `avd_wind_box()` largest grid (3 · 24 · 24 · 12) |
| 24 | `windGridMax` | 4,608 | `avd_wind_grid()` largest grid (3 · 16 · 12 · 8) |
| 25 | `textSlack` | 1,024 | `avd_text(n)` returns at least n + 1 bytes; the buffer grows by this |
| 26 | `eventsMax` | 4,096 | bytes `avd_events()` keeps at most |

`avd_build_id()` puts the build's identity in the text buffer and returns its
length (`A.buildId()`): `"<git short hash>[-dirty] <UTC date>"` as baked by
`web/build_wasi.sh` (`AVD_BUILD_ID` in the environment overrides it; a build
without it says `dev`). The site should show it where it shows its own
version, for example in the settings or about panel as
`flight core <buildId> · ABI <abiVersion>`, and send it with bug reports, so
a report can be matched to the commit that built the module. The cockpit
page's CSV replay still sizes its own buffers (`cockpit.js`); it should read
`A.layout.state` and `A.layout.cockpit` too.

## Lifecycle

```
load(json) → avd_set_weather(...) → avd_reset(mode) → loop { avd_set_input(...); avd_advance(dt); read state }
```

`avd_advance(seconds)` accumulates wall time and runs whole 1/240 s steps. Each
call is clamped to 0.25 s. It returns the number of steps run.

| Function | Meaning |
|---|---|
| `avd_reset(mode)` | Reset on the runway in use (`avd_set_airport`; Congonhas 35L by default): 0 cold and dark 30 m down the runway, 1 engine running there, 2 trimmed on the 3° final 3 km out (jets 6 km) at 65 KIAS with flaps 20 (jets: Vref + 5 KIAS, landing flap, gear down), exactly on the glide path (which starts at the glideslope antenna 300 m past the landing threshold), 3 trimmed level cruise 8 km before the runway, 850 m above the field (jets 1,500 m), 100 KIAS (jets 250), 4 trimmed at the destination's intermediate fix (10 NM out on the final course, level at the 3° path's altitude, 90 KIAS; jets 180 KIAS with flaps 1), 5 like 2 but on the destination's final; 4 and 5 use the runway in use when no destination is set. Every reset sets the altimeter to QNH, NAV 1 to the destination's ILS (else the runway in use's, else 113.70) with 113.70 in standby, the OBS to that course, the heading bug to the runway heading on the ground (else the course), COM standby 132.80 and the transponder to ALT; modes 1–5 seed the instruments to the truth |
| `avd_reset_air(east, north, heightAboveField, headingDeg, kias, flap)` | Trimmed, level, hands-off spawn anywhere in the world (ENU metres from the world anchor, height in m above the field elevation, true track (the spawn crabs into the mean wind to make it good), 55–350 KIAS, flap detent 0–3). Read the levers with `avd_trim_throttle()` / `avd_trim_mixture()` |
| `avd_set_weather(windMs, windFromDeg, turbulenceMs, isaDevC, humidity)` | Weather (rebuilds the field). The wind is the airfield report; it brings its own mechanical turbulence, weaker wind and stronger gusts over the city around SBMT, backing with height, and Cantareira ridge lift. `turbulenceMs` is an extra RMS on top |
| `avd_set_wind_detail(severity, wStarMs, ziM)` | Turbulence class (0 none, 1 light, 2 moderate, 3 severe; MIL-HDBK-1797) and thermals (w*, convective-layer depth m AGL; w* 0 = none) |
| `avd_set_wind_hazard(kind)` | 0 none, 1 microburst 2 km before the runway in use (12 m/s outflow), 2 gust front from the SW reaching the field in ~5 min; placed at the current sim time |
| `avd_set_input(pitch, roll, yaw, throttle, mixture, brakeL, brakeR, trimRate, starter)` | Raw pilot input in −1..1 or 0..1. Starter is held while 1. On an aircraft with a tiller, yaw turns the tiller on the ground below 5 kt |
| `avd_set_assist(p)` | 0 Arcade, 1 Sport, 2 Simulation |
| `avd_set_airport(id)` / `avd_airport_text()` | Spawn airport: 0 Campo de Marte (SBMT, runway 12), 1 Congonhas (SBSP, runway 35L, the default). Kept across aircraft loads; takes effect at the next reset. Text: `ICAO\|name\|runway\|reciprocal`. `avd_runway()` describes the runway in use |
| `avd_set_station(i, kg)` / `avd_set_tank(i, litres)` | Planned load, applied to every new spawn (trimmed spawns trim at that weight). No upper limit on stations (placards are reported, not enforced); tanks hold at most their capacity |
| `avd_loadsheet(planned)` / `avd_loadsheet_names()` | Weight and balance of the planned load (1) or the aircraft now (0); layout below. `A.loadsheet(planned)` returns it as an object |
| `avd_set_runway_water(mm)` | Water on the airfield: 0 dry, under 3 mm wet (14 CFR 25.109 wet braking), 3 mm and more standing water (hydroplaning above 9·√p kt) |
| `avd_wind_grid(hx, hy, hz, nx, ny, nz)` | Wind around the aircraft minus the wind at the CG, body axes, on an nx·ny·nz grid (≤ 16·12·8) over ±hx, ±hy, ±hz m: eddies, thermals and shear for the airflow view; each value averaged over the grid spacing (smaller eddies would alias). `A.windGrid(...)` |
| `avd_wind_box(cE, cN, cU, hE, hN, hU, nx, ny, nz)` | The wind (ENU m/s, the air's own velocity) on an nx·ny·nz grid (≤ 24·24·12) over the ENU box centre ± half sizes, x fastest; each value averaged over the grid spacing so smaller eddies do not alias. Moves the airflow view's smoke. `A.windBox(c, h, nx, ny, nz)` |
| `avd_set_wake(on)` | The aircraft's wake (default on): shed into the air and felt when flown into (its own gates older than 4 s, e.g. after a 360). Off: neither |
| `avd_wake()` | The wake's vortex segments: [0] count, [1] largest \|Γ\| m²/s, [2] ground height (ENU up m, −1e9 without), then from [8] nine doubles a segment: ends a and b (ENU m), circulation (m²/s, right-handed about a→b), Lamb–Oseen core radius m, age s; bound, attached, shed and ground-image segments, at most 4,096. `A.wake()` returns `{ n, gmax, ground, seg }` (a view: copy what you keep) |
| `avd_set_level_hold(on)` | Straight-and-level hold (auto-trim that keeps the airplane straight and level with the stick released), with any preset; kept when the preset changes |
| `avd_quick_start()` / `avd_procedure_status()` | Run the Quick Start checklist through the cockpit (turbofan aircraft: power up, then each engine's fuel control switch to RUN in turn). Status: 0 idle, 1 running, 2 done, 3 failed; the message is in the text buffer |
| `avd_cockpit_count()` / `avd_cockpit_text(i)` / `avd_cockpit_cycle(i)` | Switch registry: `id\|label\|pos0;pos1…\|current` |
| `avd_altimeter_adjust(hundredthsInHg)` | Kollsman knob |
| `avd_dg_adjust(deg)` / `avd_dg_sync()` / `avd_heading_bug(deg)` | Heading indicator knobs |
| `avd_obs_adjust(deg)` | OBS knob |
| `avd_nav_tune(steps50kHz)` / `avd_nav_swap()` / `avd_com_tune(channels)` / `avd_com_swap()` | Radio standby tuning and flip-flop |
| `avd_squawk(code)` / `avd_xpdr_mode(0–3)` | Transponder: OFF, SBY, ON, ALT |
| `avd_dimmer(0–1)` | Panel light dimmer |
| `avd_failure(which, on)` | 0 pitot blocked (drain open), 1 static blocked, 2 vacuum pump, 3 alternator |
| `avd_set_cloud(baseMsl, topMsl, lwc)` | One cloud layer (m MSL, liquid water g/m³); supercooled below 0 °C, so the pitot ices without pitot heat. top ≤ base clears it |
| `avd_trim_throttle()` / `avd_trim_mixture()` | Lever positions of the last trimmed spawn (modes 2 and 3), so the host starts hands-off |
| `avd_compass_card()` | 12 doubles: compass reading (deg) on magnetic headings 0, 30, … 330 from a compass swing (the FOR/STEER card) |
| `avd_events()` | Event lines since the last call (text buffer) |
| `avd_ai_cage()` | Attitude indicator caging knob: squares the gyro to the case and clears a tumble (use wings level) |
| `avd_jet()` | 64 doubles for turbofan aircraft: [0] engines, [1] gear position, [2] gear lever, [3] speedbrake position, [4] speedbrake detent, [5] Mach, [6] fuel kg, [7] fuel flow kg/h; per engine at 8 + 8i: N1 %, N2 %, ITT °C, fuel flow kg/h, thrust kN, running, starting, fuel switch; [24] MMO, [25] VMO kt, [26] TAS kt, [27] flap deg, [28] trim, [29] weight on wheels, [30–31] oil psi; [32] 1-g stall CAS kt (current flap, weight and Mach, from the trimmed CLmax), [33] the same at take-off flap, [34] at landing flap, [35] gross mass kg, [36] flight-path angle deg, [37] track true deg, [38] wind from true deg, [39] wind kt, [40] radio altitude ft, [41] SAT °C, [42] load factor, [43] α deg, [44] generators online, [45] bus volts, [46] stall warning, [47] VFE kt for the current flap (0 flaps up), [48] VLE kt, [49] take-off flap deg, [50] landing flap deg, [51] stick pusher active, [52–55] thrust reversers (position 0..1, phase: 0 stowed, 1 reverse thrust, 2 reverse idle, 3 stowing) engines 1 and 2, [56] ground spoilers out, [57] reversers fitted |
| `avd_flow()` | Airflow-visualisation snapshot (1,352 doubles): header [0] strips, [1] engines, [2] TAS m/s, [3] α, [4] β, [5] ρ, [6–8] CG (FRD), [9] speed of sound, [10–12] body rates p, q, r (rad/s); per lifting strip at 16 + 20i (up to 64): quarter-chord position, span axis, normal (FRD body), chord, area, cl, local α, dynamic pressure, stall α, surface index, attached-flow fraction (1 attached, below 0.5 separated: stalled or behind a raised spoiler), bound circulation Γ m²/s, zero-lift α (with the flaps), cd; per engine at 1296 + 14k: position, thrust axis, disc/nozzle radius, slipstream velocity increment, thrust N, kind (0 propeller, 1 turbofan), propeller rate rad/s (signed, right-handed about the thrust axis), blades, swirl m/s at 0.7 R, hub radius fraction (turbofans: thrust-reverser position, efflux speed m/s, cowl radius, cascades forward of the nozzle m) |
| `avd_bodies()` | Bluff bodies in the airflow: [0] count, then ten doubles a body (up to 32): position (structural frame m), long axis, size (wheel diameter or the panel's projected height, m), length along the axis m, drag area m², kind (0 gear leg or wheel, 1 spoiler panel, 2 strut or other fixed item). Extended retractable gear, raised spoiler panels, and drag items given a `body` shape (fixed gear, wing struts); the airflow view draws their wakes. `A.bodies()` returns `{ n, b }` (a view: copy what you keep) |

## `avd_state()`: 64 doubles

Indices and fields:

| Index | Field | Index | Field |
|---|---|---|---|
| 0–2 | position ENU (m) | 25 | mixture |
| 3–6 | attitude quaternion w, x, y, z (body FLU → world ENU) | 26 | flap (deg) |
| 7 | roll (deg) | 27 | trim |
| 8 | pitch (deg) | 28 | on ground |
| 9 | true heading (deg) | 29 | stall horn |
| 10 | IAS (kt) | 30 | engine running |
| 11 | TAS (kt) | 31 | volts |
| 12 | ground speed (kt) | 32 | alternator online |
| 13 | indicated altitude (ft) | 33 | suction (inHg) |
| 14 | height above field (ft) | 34 | battery state of charge |
| 15 | VSI (fpm) | 35 | AI roll (deg) |
| 16 | α (deg) | 36 | AI pitch (deg) |
| 17 | β (deg) | 37 | DG (deg) |
| 18 | load factor | 38 | TC rate (deg/s) |
| 19 | tachometer rpm | 39 | ball |
| 20 | MAP (inHg) | 40 | mass (kg) |
| 21 | fuel flow (gph) | 41 | CG x (m) |
| 22 | fuel gauge left (l) | 42 | time (s) |
| 23 | fuel gauge right (l) | 43 | stalled strips |
| 24 | throttle | 44 | oil pressure (psi) |

| Index | Field |
|---|---|
| 45–47 | aileron, elevator, rudder |
| 48 | faulted |
| 49 | parking brake |
| 50 | thrust (N) |
| 51 | Mach |
| 52 | fuel total (l) |
| 53 | altitude MSL (ft) |
| 54 | assist active |
| 55 | straight-and-level hold engaged |
| 56 | yaw input on the nose-wheel tiller (below 5 kt on the ground) |
| 57 | aircraft has a tiller |
| 58 | straight-and-level hold in underspeed protection (descending to keep flying speed) |
| 59, 60 | wind at the aircraft, smoothed over 3 s: from (deg true), speed (kt) |
| 61, 62 | ground track (deg true), drift (track − heading, deg, + right) |
| 63 | water on the airfield (mm) |

## `avd_loadsheet(planned)`: 256 doubles

Masses in kg, x in the structural frame (m, forward positive; the 172's POH
arm in inches is −x / 0.0254).

| Index | Field | Index | Field |
|---|---|---|---|
| 0 | basic empty mass (as published) | 12–15 | limits: max ramp, take-off, landing, zero fuel (0 = not published) |
| 1 | basic empty CG x | 16 | fuel density (kg/L) |
| 2 | payload | 17–19 | number of stations, tanks, combined limits |
| 3, 4 | zero-fuel mass, CG x | 20, 21 | points in the forward and aft CG limit tables |
| 5, 6 | fuel on board (tanks and lines), usable fuel | 22 | trapped fuel (unusable and lines) |
| 7, 8 | gross mass, CG x | | |
| 9, 10 | forward and aft CG limits at the gross mass | | |
| 11 | flags: 1 over ramp, 2 over take-off, 4 over landing, 8 over zero fuel, 16 CG forward of limit, 32 CG aft of limit, 64 a station over its placard, 128 a combined limit exceeded, 256 a tank over capacity | | |

From index 24: per station (kg, placard kg, x), per tank (litres, capacity,
unusable, x), per combined limit (kg, limit kg), then the forward and aft CG
limit tables as (mass, x) pairs.

## `avd_cockpit()`: 96 doubles

Display values (indices 0–47):

| Index | Field | Index | Field |
|---|---|---|---|
| 0 | IAS (kt) | 24 | annunciator bits (1 OIL, 2 VAC, 4 VOLTS, 8 L FUEL, 16 R FUEL) |
| 1 | indicated altitude (ft) | 25 | stall horn |
| 2 | Kollsman (inHg) | 26 | OAT (°C) |
| 3 | VSI (fpm) | 27 | clock (s) |
| 4 | AI pitch (deg) | 28 | flap (deg) |
| 5 | AI roll (deg) | 29 | trim |
| 6 | AI tumbled | 30 | panel lights |
| 7 | DG heading (deg) | 31 | dimmer |
| 8 | heading bug (deg) | 32 | NAV active (MHz) |
| 9 | TC deflection (deg) | 33 | NAV standby (MHz) |
| 10 | TC rate (deg/s) | 34 | COM active (MHz) |
| 11 | TC OFF flag | 35 | COM standby (MHz) |
| 12 | ball (deg) | 36 | OBS (deg) |
| 13 | compass (deg) | 37 | NAV valid |
| 14 | tachometer rpm | 38 | CDI (±1 full scale) |
| 15 | oil pressure (psi) | 39 | TO/FROM (+1 / −1 / 0) |
| 16 | oil temperature (°F) | 40 | GS valid |
| 17 | EGT (°F) | 41 | GS (±1) |
| 18 | fuel flow (gph) | 42 | localizer tuned |
| 19 | fuel left (gal) | 43 | avionics powered |
| 20 | fuel right (gal) | 44 | squawk |
| 21 | ammeter (A) | 45 | transponder mode |
| 22 | volts | 46 | reported altitude (ft) |
| 23 | suction (inHg) | 47 | altitude reported |

Truth (indices 48–68), for debugging only; a renderer must not draw gauges
from these:

| Index | Field | Index | Field |
|---|---|---|---|
| 48 | TAS (kt) | 59 | q (deg/s) |
| 49 | CAS (kt) | 60 | r (deg/s) |
| 50 | ground speed (kt) | 61 | sideslip (deg) |
| 51 | true altitude (ft) | 62 | AI rotor (fraction) |
| 52 | pressure altitude (ft) | 63 | DG rotor (fraction) |
| 53 | true vertical speed (fpm) | 64 | TC rotor (fraction) |
| 54 | pitch (deg) | 65 | EAS (kt) |
| 55 | roll (deg) | 66 | density altitude (ft) |
| 56 | true heading (deg) | 67 | height above field (ft) |
| 57 | magnetic heading (deg) | 68 | α (deg) |
| 58 | p (deg/s) | | |

Faults (indices 69–71): pitot fault, static fault, vacuum failed.
Icing (truth, indices 72–74): pitot ice as a fraction of the amount that closes the inlet,
pitot heater warm-up (0–1), cloud liquid water at the aircraft (g/m³).

## `avd_markings()`

All markings come from the aircraft's `panel` data:

| Index | Field |
|---|---|
| 0–8 | ASI: white low/high, green low/high, yellow low/high, red line, dial min, dial max |
| 9–12 | Tachometer: green low/high, red line, maximum |
| 13–16 | Oil pressure: green low/high, red low, red high |
| 17–19 | Oil temperature: green low/high, red line |
| 20–21 | Suction green low/high |
| 22 | Usable fuel per tank (gal) |
| 23 | VSI range |
| 24 | NAV/COM count |
| 25 | Glideslope installed |
| 26 | Transponder installed |
| 27 | TC standard-rate deflection |
| 28 | EGT installed |
| 29 | Fuel flow installed |
| 30 | Ammeter installed |

## Cockpit page

`cockpit/cockpit.html` (markup and styles) and `cockpit/cockpit.js` (rendering
and input) are bundled with the WASM module and the aircraft JSON into a
single file by `cockpit/build_page.mjs`. Every gauge reads `avd_cockpit()` and
the markings; the page computes no flight state. The outside view is a
deliberately simple 2D projection: a horizon, the 500 m world grid, runway
12/30 and labelled landmark posts. It is there for orientation only and is not
a renderer.

The page records the last 5 minutes of the flight (one frame per 0.1 s of
simulated time). **Replay** plays it back through the same panel at ¼×, 1×
or 4×, with scrubbing. **Load CSV** replays a native flight written by
`avdemo instruments csv` (the `InstrumentRecorder` channels, including
`east_m` and `north_m` for the outside view).
