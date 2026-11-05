# Browser runtime

The flight core compiles to WebAssembly with wasi-sdk. The build has no
exceptions or RTTI, and the WASI imports are stubs because the core does no
I/O. The C ABI is in `avd_web.cpp`.

```sh
WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh   # → web/dist/avd.wasm, avd.mjs and the aircraft JSON (king_air_350.json among them)
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
- `markings()`, `runway()`, `events()`, `cockpit()`, `procedure()`, `jet()`,
  `setThrottle(engine, v)`;
- turboprops: `turboprop()`, `setPropLever(v)`, `setCondition(engine, p)`,
  `setFeather(engine, on)` (see *Turboprops* below);
- `abiVersion`, `layout` (every array size and stride from `avd_layout()`)
  and `buildId()`; see *ABI version* below.

## ABI version

The module and the loader must agree on the layout of every array below.
`avd_abi_version()` returns the module's layout version (`kAbiVersion` in
`avd_web.cpp`, currently **3**); `createAvdModule` throws if it is not the
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
| 2 | `jet` | 96 | `avd_jet()` |
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
| 27 | `traffic` | 32 | `avd_traffic_state()` |
| 28 | `airPointsMax` | 4,096 | `avd_points(n)` / `avd_air_velocity(n)` points at most |

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
| `avd_reset(mode)` | Reset on the runway in use (`avd_set_airport`; Congonhas 35L by default): 0 cold and dark 30 m down the runway, 1 engine running there, 2 trimmed on the 3° final 3 km out (jets 6 km) at 65 KIAS with flaps 20 (jets: Vref + 5 KIAS, landing flap, gear down), exactly on the glide path (which starts at the glideslope antenna 300 m past the landing threshold), 3 trimmed level cruise 8 km before the runway, 850 m above the field (jets 1,500 m), 100 KIAS (jets 250; turboprops take the jets' spawns but cruise at 200 KIAS), 4 trimmed at the destination's intermediate fix (10 NM out on the final course, level at the 3° path's altitude, 90 KIAS; jets 180 KIAS with flaps 1), 5 like 2 but on the destination's final; 4 and 5 use the runway in use when no destination is set. Every reset sets the altimeter to QNH, NAV 1 to the destination's ILS (else the runway in use's, else 113.70) with 113.70 in standby, the OBS to that course, the heading bug to the runway heading on the ground (else the course), COM standby 132.80 and the transponder to ALT; modes 1–5 seed the instruments to the truth |
| `avd_reset_air(east, north, heightAboveField, headingDeg, kias, flap)` | Trimmed, level, hands-off spawn anywhere in the world (ENU metres from the world anchor, height in m above the field elevation, true track (the spawn crabs into the mean wind to make it good), 55–350 KIAS, flap detent 0–3). Read the levers with `avd_trim_throttle()` / `avd_trim_mixture()` |
| `avd_set_weather(windMs, windFromDeg, turbulenceMs, isaDevC, humidity)` | Weather (rebuilds the field). The wind is the airfield report; it brings its own mechanical turbulence, weaker wind and stronger gusts over the city around SBMT, backing with height, and Cantareira ridge lift. `turbulenceMs` is an extra RMS on top |
| `avd_set_wind_detail(severity, wStarMs, ziM)` | Turbulence class (0 none, 1 light, 2 moderate, 3 severe; MIL-HDBK-1797) and thermals (w*, convective-layer depth m AGL; w* 0 = none) |
| `avd_set_wind_hazard(kind)` | 0 none, 1 microburst 2 km before the runway in use (12 m/s outflow), 2 gust front from the SW reaching the field in ~5 min; placed at the current sim time |
| `avd_set_input(pitch, roll, yaw, throttle, mixture, brakeL, brakeR, trimRate, starter)` | Raw pilot input in −1..1 or 0..1. Starter is held while 1. On an aircraft with a tiller, yaw turns the tiller on the ground below 5 kt. `throttle` moves every engine's lever together (and ends a split) |
| `avd_set_throttle(engine, v)` | One engine's throttle lever (0-based; −1 or out of range: every lever). Splits the levers (asymmetric thrust, differential taxi) until the next `avd_set_input`, which sets them all again. `A.setThrottle(engine, v)` |
| `avd_set_reverse(v)` | The reverse thrust levers, 0 (stowed) to 1 (maximum reverse). Lifted with the throttles at idle and the main wheels on the ground, they deploy the reversers (2 s) and then set the reverse N1, from idle up to the reverser's reverse N1. Back to 0 stows them. Held until set again; while it is 0 the automatic landing-roll sequence runs. `jet()` reports reverser phase 4 while on the lever. `A.setReverse(v)` |
| `avd_set_prop_lever(v)` | Turboprops: the propeller levers, 0 (the minimum governed rpm, 1,450 on the King Air 350) to 1 (the rated rpm, 1,700), every engine; the governors set the blade angle to hold it. Held until set again. `A.setPropLever(v)` |
| `avd_set_condition(engine, p)` | Turboprops: condition lever of engine (0-based; −1 every engine): 0 CUT-OFF (fuel off: shut-down), 1 LOW IDLE, 2 HIGH IDLE. Returns the number of levers set. The cockpit control `engine<N>.condition` |
| `avd_set_feather(engine, on)` | Turboprops: the propeller lever's feather detent (1 feather, 0 back to the governing range); −1 every engine. Returns the number set. The cockpit control `engine<N>.feather` |
| `avd_turboprop()` | Turboprop engine indication, 48 doubles (layout in *Turboprops*). `A.turboprop()` returns it as an object |
| `avd_set_assist(p)` | 0 Arcade, 1 Sport, 2 Simulation |
| `avd_set_airport(id)` / `avd_airport_text()` | Spawn airport: 0 Campo de Marte (SBMT, runway 12), 1 Congonhas (SBSP, runway 35L, the default). Kept across aircraft loads; takes effect at the next reset. Text: `ICAO\|name\|runway\|reciprocal`. `avd_runway()` describes the runway in use |
| `avd_set_station(i, kg)` / `avd_set_tank(i, litres)` | Planned load, applied to every new spawn (trimmed spawns trim at that weight). No upper limit on stations (placards are reported, not enforced); tanks hold at most their capacity |
| `avd_loadsheet(planned)` / `avd_loadsheet_names()` | Weight and balance of the planned load (1) or the aircraft now (0); layout below. `A.loadsheet(planned)` returns it as an object |
| `avd_set_runway_water(mm)` | Water on the airfield: 0 dry, under 3 mm wet (14 CFR 25.109 wet braking), 3 mm and more standing water (hydroplaning above 9·√p kt) |
| `avd_wind_grid(hx, hy, hz, nx, ny, nz)` | Wind around the aircraft minus the wind at the CG, body axes, on an nx·ny·nz grid (≤ 16·12·8) over ±hx, ±hy, ±hz m: eddies, thermals and shear for the airflow view; each value averaged over the grid spacing (smaller eddies would alias). `A.windGrid(...)` |
| `avd_wind_box(cE, cN, cU, hE, hN, hU, nx, ny, nz)` | The wind (ENU m/s, the air's own velocity) on an nx·ny·nz grid (≤ 24·24·12) over the ENU box centre ± half sizes, x fastest; each value averaged over the grid spacing so smaller eddies do not alias. Moves the airflow view's smoke. `A.windBox(c, h, nx, ny, nz)` |
| `avd_set_wake(on)` | The aircraft's wake (default on): shed into the air and felt when flown into (its own gates older than 4 s, e.g. after a 360). Off: neither |
| `avd_wake()` | The wake's vortex segments: [0] count, [1] largest \|Γ\| m²/s, [2] ground height (ENU up m, −1e9 without), then from [8] nine doubles a segment: ends a and b (ENU m), circulation (m²/s, right-handed about a→b), Lamb–Oseen core radius m, age s; bound, attached, shed and ground-image segments, at most 4,096. `A.wake()` returns `{ n, gmax, ground, seg }` (a view: copy what you keep) |
| `avd_reset_final(distance, kias)` | As `avd_reset(2)` (trimmed on the 3° final for the runway in use, hands off), `distance` m (500–30,000) before the threshold, at `kias` (0: the type's approach speed) |
| `avd_traffic_load(len)` | Load a second aircraft's definition (JSON in the text buffer) for traffic; returns the error count (`avd_error_text`). `A.trafficLoad(json)` |
| `avd_traffic_spawn(e, n, h, track, path, kias, flap, gear)` | Spawn the traffic trimmed at world ENU (e, n) m, h m above the field, true track and flight path deg (− descending), IAS kt, flap detent, gear down 1 / up 0; a line pilot flies it down that line from there (wings and line held on the truth state, the speed on the throttle). It steps before the aircraft in every `avd_advance`, and the aircraft flies in its whole wake. Returns 1 when it flies |
| `avd_traffic_go_around(along)` | The traffic goes around (take-off thrust, a 5° climb, the course held): at once (along < 0), or once that far along its line (m) |
| `avd_traffic_clear()` | Remove the traffic and its wake |
| `avd_traffic_state()` | 32 doubles: [0] flying, [1–3] position ENU m, [4–7] attitude quaternion w, x, y, z (body FLU → world ENU), [8] roll, [9] pitch, [10] true heading deg, [11] IAS, [12] TAS kt, [13] vertical speed ft/min, [14] going around, [15] m along its line, [16] m right of it, [17] m above it, [18] flap deg, [19] gear position, [20] distance to the aircraft m, [21] horizontal, [22] vertical (+ traffic above), [23] closest so far m, [24] horizontal and [25] vertical then, [26] when (the aircraft's clock s), [27] its wake's largest pair circulation m²/s, [28] wing span m, [29] mass kg, [30] its clock s, [31] reserved. `A.traffic()` |
| `avd_traffic_wake()` | The traffic's vortex segments, as `avd_wake()`. `A.trafficWake()` |
| `avd_points(n)` / `avd_air_velocity(n)` | The air's own velocity (ENU m/s: the wind and every wake) at up to 4,096 points: write 3n ENU positions (m) to `avd_points(n)`, then `avd_air_velocity(n)` overwrites them with velocities (smoke advection). `A.airVelocity(pts)` |
| `avd_line_pilot(on, kias, pathDeg)` | The aircraft flown by a line pilot down its present track at `kias` (0: its present IAS) on a line `pathDeg` steep over the ground (− descending; 0: its present path over the ground, which a wind tilts), hands off; off returns it to `avd_set_input`. `A.linePilot(on, kias, pathDeg)` |
| `avd_autoland(on)` | The autoland: the blind landings' instrument-only approach pilot (`scenarios::ApproachPilot`) flies the GPS route to the destination runway (the runway in use if none is set: it becomes the destination). It joins the final when the 3° path can be flown from there (within 50 m of it), else it flies the whole procedure, level at the final approach fix's altitude. It lowers the gear, extends the flaps a detent at a time below each one's VFE, sets the altimeter to QNH, flares, lands and stops on the centreline; off returns the aircraft to `avd_set_input`. Returns 1 engaged, 0 off, −1 no approach, −2 on the ground. `A.autoland(on)` |
| `avd_autoland_status()` | `[on, phase (0 intercept, 1 final, 2 flare, 3 rollout, 4 stopped), m right of the course, m above the path, approach IAS kt, landing flap detent]`. `A.autolandStatus()` → `{ on, phase, right, above, kias, flap }` |
| `avd_autopilot(lateral, vertical, autothrottle, altFt, vsFpm, iasKt)` | The autopilot (`scenarios::Autopilot`), flown from the instruments. Lateral: 0 off, 1 ROL (wings level), 2 HDG (the heading bug, `avd_heading_bug`; at most 25° of bank or a rate-one turn), 3 NAV (the GPS route's active leg, intercepting at up to 30°). Vertical: 0 off, 1 PIT (the attitude at engagement), 2 ALT (the selected altitude), 3 VS (the selected rate to the selected altitude, captured and then held). Autothrottle: the selected IAS on the throttles; off, `avd_set_input`'s throttle stays in. It runs the pitch trim, passes the pilot's brakes, flaps and gear through, and disconnects on the ground; the autoland and the line pilot take over from it. Altitude on the altimeter (ft), vertical speed fpm, IAS kt. Lateral 4 is APR (the approach set by `avd_autopilot_approach`): armed it flies the heading bug (ILS intercept) or the GPS route and the vertical mode given (ALT, VS); it captures the localizer (GPS final course) inside full scale, then the glideslope (glidepath) within a quarter dot, from below or level, and flies both to the decision altitude, where lateral and vertical disconnect (MINIMUMS) and the autothrottle stays in SPD: the pilot lands. Re-sending a vertical mode with APR selected keeps a captured glideslope. Returns 1 engaged, 0 everything off, −2 on the ground, −3 NAV without a route, −4 APR without its guidance (NAV 1 not on a region ILS, no GPS route). `A.autopilot({ lateral, vertical, autothrottle, altFt, vsFpm, iasKt })` |
| `avd_autopilot_status()` | `[on, lateral (4 APR), vertical (4 ALT* while capturing, 5 GS/GP coupled), autothrottle, selected altitude ft, vertical speed fpm, IAS kt, heading bug deg, commanded bank deg, commanded pitch deg, commanded vertical speed fpm, APR state (0 off, 1 armed, 2 LOC/LNAV captured and GS/GP armed, 3 GS/GP captured, 4 MINIMUMS: disconnected at DA, also once the autopilot is off), APR source (0 ILS, 1 GPS), DA ft, lateral deviation dots, vertical deviation dots]` (dots: + fly right / up, 2 dots full scale). The array grew from 11 to 16 at the end. `A.autopilotStatus()` |
| `avd_autopilot_approach(source, daFt)` | APR's approach: source 0 the ILS on NAV 1 (its front course on the OBS; the field elevation and runway length from the region ILS tuned), 1 the GPS approach of the route to the destination (its LPV-style glidepath); the decision altitude in feet on the altimeter (≤ 0: 200 ft above the threshold). Returns 1, −1 when a default DA has no field. `A.autopilotApproach({ source: 'ils' \| 'gps', daFt })` |
| `avd_set_attitude_reference(mode)` | The attitude and heading reference: 0 as the aircraft's data fit them (the 172's vacuum gyros; the jets' AHRS), 1 a glass cockpit's AHRS and magnetometer (a G1000 172: no turn or erection errors, no tumble, no drift; the vacuum system drives nothing the pilot flies by). Applied at once, aligned; kept across `avd_reset`; `avd_load` starts at 0. Returns the reference in use (0 gyro, 1 AHRS), −1 without an aircraft. `A.setAttitudeReference('ahrs' \| 'data')` |
| `avd_attitude()` | `[reference (0 gyro, 1 AHRS), ATT flag (no valid attitude: an AHRS aligning, unpowered or failed; an electric gyro unpowered), alignment 0..1 (a gyro: its rotor speed), tumbled]`. `A.attitude()` |
| `avd_terrain(len)` | The region's terrain between the airfields: AVT1 bytes (`tools/terrain`, a 200 m SRTM grid) already written to `avd_text`. The fields stay flat at their elevations, blended into it over 2 km; off the grid the fields' blend as before. Kept across airport changes; `avd_ground_height` and everything on the ground (gear, buildings, hard points) use it. Returns the sample count, −1 if malformed. `A.terrain(bytes)` |
| `avd_terrain_detail(len)` | A finer grid nested in the region's: AVT1 bytes already written to `avd_text` (the city's: `data/world/terrain_sp_city.avt`, 50 m over 50 × 62 km, 2,484,526 bytes). It wins where it lies and blends into the region grid over its last 2 km inside its edge (no step); the fields stay flat. One with the same extent replaces it; `len` 0 removes every detail grid. Kept across airport changes and `avd_terrain` calls (which keep their meaning: the region grid). Returns the sample count (0 when cleared), −1 if malformed. `A.terrainDetail(bytes)` (no bytes: clear) |
| `avd_taws()` | TAWS / GPWS (`avd/taws.hpp`), 16 doubles: [0] class (0 off, 1 B, 2 A), [1] active alerts (bit `1 << (alert − 1)`), [2] the highest-priority active alert, [3] its level (0 none, 1 advisory, 2 caution: amber, 3 warning: red PULL UP), [4] the phrase being spoken (voice id, 0 none), [5] phrases begun (a new value: speak the phrase), [6] radio altitude ft (−1 no reading: above 2,500 ft), [7] terrain closure rate ft/min, [8] FLTA: s until the present path comes within the required terrain clearance (−1 none), [9] phase (0 en route, 1 terminal, 2 approach, 3 departure), [10] the required terrain clearance ft, [11] Mode 3 altitude loss ft, [12] take-off phase (0/1), [13] glideslope alert loud (0/1), [14] terrain inhibit (0/1), [15] decision height ft. Alerts in priority order: 1 PULL UP (sink rate), 2 PULL UP (terrain closure), 3 TERRAIN AHEAD PULL UP, 4 TERRAIN TERRAIN, 5 MINIMUMS, 6 CAUTION TERRAIN, 7 TOO LOW TERRAIN, 8 altitude callout, 9 TOO LOW GEAR, 10 TOO LOW FLAPS, 11 SINK RATE, 12 DON'T SINK, 13 GLIDESLOPE, 14 BANK ANGLE. Voices: 1 "PULL UP", 2 "TERRAIN AHEAD, PULL UP", 3 "TERRAIN, TERRAIN", 4 "MINIMUMS", 5 "CAUTION TERRAIN, CAUTION TERRAIN", 6 "TOO LOW, TERRAIN", 7 "TOO LOW, GEAR", 8 "TOO LOW, FLAPS", 9 "SINK RATE", 10 "DON'T SINK", 11 "GLIDESLOPE", 12 "BANK ANGLE, BANK ANGLE", 13–24 the callouts 2,500, 1,000, 500, 400, 300, 200, 100, 50, 40, 30, 20, 10. `A.taws()` → `{ cls, active: [alert ids], top, level, voice, voiceCount, say, alert, raFt, closureFpm, terrainTime, phase, rtcFt, altLossFt, takeoff, gsLoud, inhibit, dhFt }` (`say`: the phrase's text, `alert`: the top alert's annunciation) |
| `avd_taws_text(which, index)` | Text for the TAWS in the text buffer (returns its length): 0 the phrase being spoken, 1 the top alert's annunciation, 2 voice `index`'s phrase, 3 alert `index`'s annunciation. `A.tawsVoiceText(id)`, `A.tawsAlertText(id)` |
| `avd_taws_config(cls, dhFt, inhibit, landingDetent)` | TAWS settings, kept across aircraft loads and resets: class −1 the aircraft's own (A in the jets, B in the others), 0 off, 1 B, 2 A; decision height ft on the radio altimeter for MINIMUMS (0 none, < 0 keep; default 200); terrain inhibit 1 (FLTA off, the GPWS modes stay); the flap detent from which the flaps count as landing (−1 the last). Returns the class in force. `A.tawsConfig({ cls, dhFt, inhibit, landingDetent })` |
| `avd_occupant()` | The crash dummy in the pilot's seat and the damage, since the last reset or placement: `[peak g (resultant), peak spinal g, DRI, lumbar load N, HIC(36), collapsed gear legs (bit mask), structural failure (0/1)]` (`avd/occupant.hpp`; the seat limits HIC 1,000, lumbar 6,672 N, DRI 18). `A.occupant()` → `{ peakG, spinalG, dri, lumbarN, hic, collapsed: [bool per leg], structural }` |
| `avd_set_level_hold(on)` | Straight-and-level hold (auto-trim that keeps the airplane straight and level with the stick released), with any preset; kept when the preset changes |
| `avd_quick_start()` / `avd_procedure_status()` | Run the Quick Start checklist through the cockpit (turbofan aircraft: power up, then each engine's fuel control switch to RUN in turn; turboprops: power up, then each engine's starter, its condition lever to LOW IDLE at light-off speed, the starter released at idle). Status: 0 idle, 1 running, 2 done, 3 failed; the message is in the text buffer |
| `avd_cockpit_count()` / `avd_cockpit_text(i)` / `avd_cockpit_cycle(i)` | Switch registry: `id\|label\|pos0;pos1…\|current`. Electrical: `electrical.battery`, `electrical.alternator` (every generator), `electrical.generator<N>`, `electrical.avionics`, `electrical.load_shed` (NORMAL / SHED: the pilot's shed of the non-essential loads, held until reset), `electrical.load.<name>` breakers. Ice: `engine<N>.carb_heat` (COLD / HOT, carburetted engines: the T-180), `anti_ice.<name>` (OFF / ON: the jets' `anti_ice.wing` bleed-air anti-ice, the CJ4's `anti_ice.tail` boots; the Phenom's `anti_ice.wing` is boots). These come after every other control |
| `avd_altimeter_adjust(hundredthsInHg)` | Kollsman knob |
| `avd_dg_adjust(deg)` / `avd_dg_sync()` / `avd_heading_bug(deg)` | Heading indicator knobs |
| `avd_obs_adjust(deg)` | OBS knob |
| `avd_nav_tune(steps50kHz)` / `avd_nav_swap()` / `avd_com_tune(channels)` / `avd_com_swap()` | Radio standby tuning and flip-flop |
| `avd_squawk(code)` / `avd_xpdr_mode(0–3)` | Transponder: OFF, SBY, ON, ALT |
| `avd_dimmer(0–1)` | Panel light dimmer |
| `avd_failure(which, on)` | 0 pitot blocked (drain open), 1 static blocked, 2 vacuum pump, 3 alternator (every generator), 4 generator 1 only, 5 generator 2 only |
| `avd_set_cloud(baseMsl, topMsl, lwc)` | One cloud layer (m MSL, liquid water g/m³: stratus 0.1–0.5, cumulus to 1–2); supercooled below 0 °C (`avd_set_weather`'s ISA deviation sets the temperature): the pitot ices without pitot heat, and the airframe ices (`avd_icing`). top ≤ base clears it |
| `avd_set_rain(mmPerHour)` | Rain (0 none, 2.5 moderate, 10 heavy, 50 a downpour) from the cloud layer's top to the ground (everywhere without a layer). Below 0 °C it is freezing rain (fast glaze icing). The runways wet with it: a Gallaway water film (0.7 mm in 10 mm/h, 2.6 mm in 50, standing water from ~70) building over ~3 min, draining and drying after (`state()[63]`, the larger of it and `avd_set_runway_water`). `A.setRain(mm)` |
| `avd_set_front(kind, etaMin, dT, dQnhHpa, windKt, windFromDeg)` | A moving front reaching the middle of the runway in use `etaMin` minutes from now: kind 0 none, 1 cold (from the SW at 12 m/s, a 1.5 hPa trough on its line, the airmass changing over 10 km), 2 warm (from the NW at 6 m/s, over 30 km). Behind it the temperature is `dT` K different, the QNH `dQnhHpa`, the surface wind `windKt` from `windFromDeg` true. The OAT and the local QNH change as the aircraft crosses it; an altimeter set ahead of it reads wrong behind it. Re-anchored to a reset like the wind hazard. `A.setFront({ kind: 'cold' \| 'warm' \| 'none', etaMin, dT, dQnhHpa, windKt, windFromDeg })` (cold: −6 K, +3 hPa, 15 kt from 200; warm: +4 K, −2 hPa, 10 kt from 320) |
| `avd_weather()` | 24 doubles, the weather at the aircraft and at the field: [0] rain mm/h, [1] meteorological visibility m (clear 40 km, rain, cloud), [2] OAT °C, [3] local QNH hPa, [4] the field's QNH hPa, [5] the field's OAT °C, [6] front: 0 ahead of it .. 1 behind (−1 no front), [7] km to the front line (+ still to come), [8] runway water mm, [9] cloud liquid water g/m³, [10] supercooled water g/m³ (cloud + rain, below 0 °C), [11] in cloud, [12] rain water g/m³, [13] freezing rain, [14–15] cloud base and top m MSL (0 none), [16] minutes until the front reaches the field (−1 none; negative once past), [17–23] reserved. `A.weather()` |
| `avd_icing()` | 48 doubles, the ice on the aircraft (`avd/icing.hpp`): [0] surfaces n, [1–8] ice at each surface's leading edge mm (0 wing, 1 horizontal tail, 2 fin), [9–16] each surface's clmax loss (fraction), [17–24] glaze share (0 rime, 1 clear), [25] propeller ice mm, [26] propeller thrust loss, [27] carburettor ice g, [28] the share of the throttle's area it blocks, [29] carburettor throat °C, [30] carb heat 0..1, [31] engine roughness 0..1 (meltwater, a blocked carburettor), [32] carburettor fitted, [33] protection switches on (bit mask, `anti_ice.*` in cockpit order), [34] leading-edge heat 0..1, [35] boots inflating, [36] ICE annunciator, [37] ice detector fitted, [38] windshield ice mm, [39] windshield opacity 0..1, [40] accretion rate on the unprotected wing mm/min, [41] intensity 0 none, 1 trace, 2 light, 3 moderate, 4 severe, [42] 1-g stall CAS kt now (with the ice; `jet()[32]` follows it too), [43] the clean airframe's, [44] anti-ice bleed drawn (jets lose 4 % thrust), [45] protection switches n, [46] wing stall angle lost to ice deg (the stall-warning vane still trips at the clean angle), [47] reserved. `A.icing()` |
| `avd_trim_throttle()` / `avd_trim_mixture()` | Lever positions of the last trimmed spawn (modes 2 and 3), so the host starts hands-off |
| `avd_compass_card()` | 12 doubles: compass reading (deg) on magnetic headings 0, 30, … 330 from a compass swing (the FOR/STEER card) |
| `avd_events()` | Event lines since the last call (text buffer) |
| `avd_ai_cage()` | Attitude indicator caging knob: squares the gyro to the case and clears a tumble (use wings level). On an AHRS (the jets, or `avd_set_attitude_reference(1)`) its reset: it re-aligns, the ATT flag showing for 30 s |
| `avd_jet()` | 96 doubles for turbofan aircraft: [0] engines, [1] gear position, [2] gear lever, [3] speedbrake position, [4] speedbrake detent, [5] Mach, [6] fuel kg, [7] fuel flow kg/h; per engine at 8 + 8i (the engine indication's sensed channels: needle lag, blank without gauge power): N1 %, N2 %, ITT °C, fuel flow kg/h, thrust kN, running, starting, fuel switch; [24] MMO, [25] VMO kt, [26] TAS kt, [27] flap deg, [28] trim, [29] weight on wheels, [30–31] oil psi (sensed); [32] 1-g stall CAS kt (current flap, weight and Mach, from the trimmed CLmax), [33] the same at take-off flap, [34] at landing flap, [35] gross mass kg, [36] flight-path angle deg, [37] track true deg, [38] wind from true deg, [39] wind kt, [40] radio altitude ft, [41] SAT °C, [42] load factor, [43] α deg, [44] any generator online, [45] bus volts, [46] stall warning, [47] VFE kt for the current flap (0 flaps up), [48] VLE kt, [49] take-off flap deg, [50] landing flap deg, [51] stick pusher active, [52–55] thrust reversers (position 0..1, phase: 0 stowed, 1 reverse thrust, 2 reverse idle, 3 stowing, 4 on the reverse levers) engines 1 and 2, [56] ground spoilers out, [57] reversers fitted, [58–59] throttle levers engines 1 and 2 (0..1, as followed), [60–61] generators 1 and 2 on line, [62–63] hydraulic pressure systems A and B psi (0 without hydraulics), [64] gear, [65] flap, [66] speedbrake, [67] steering, [68] brake power available (supply fraction 0..1; 1 without hydraulics; brakes: 1 while pressurised or an accumulator application is backing them), [69] brake accumulator applications left (−1 without hydraulics), [70] gear emergency release pulled, [71] hydraulics fitted, [72] battery charge 0..1, [73] battery current A (+ discharge), [74 + 4i] truth N1 %, N2 %, ITT °C, fuel flow kg/h engines 1 and 2 (for the debug overlay); [82] load shed active (LOAD SHED / ESS BUS ONLY: the non-essential loads are off, by the bus controller or the pilot's `electrical.load_shed`), [83] the bus controller's automatic shed is on (no generator on line, battery alone past `load_shed_delay_s`), [84] a bus controller is fitted (`auto_load_shed`); [85–95] reserved |
| `avd_flow()` | Airflow-visualisation snapshot (1,352 doubles): header [0] strips, [1] engines, [2] TAS m/s, [3] α, [4] β, [5] ρ, [6–8] CG (FRD), [9] speed of sound, [10–12] body rates p, q, r (rad/s); per lifting strip at 16 + 20i (up to 64): quarter-chord position, span axis, normal (FRD body), chord, area, cl, local α, dynamic pressure, stall α, surface index, attached-flow fraction (1 attached, below 0.5 separated: stalled or behind a raised spoiler), bound circulation Γ m²/s, zero-lift α (with the flaps), cd; per engine at 1296 + 14k: position, thrust axis, disc/nozzle radius, slipstream velocity increment, thrust N, kind (0 propeller, 1 turbofan), propeller rate rad/s (signed, right-handed about the thrust axis), blades, swirl m/s at 0.7 R, hub radius fraction (turbofans: thrust-reverser position, efflux speed m/s, cowl radius, cascades forward of the nozzle m) |
| `avd_bodies()` | Bluff bodies in the airflow: [0] count, then ten doubles a body (up to 32): position (structural frame m), long axis, size (wheel diameter or the panel's projected height, m), length along the axis m, drag area m², kind (0 gear leg or wheel, 1 spoiler panel, 2 strut or other fixed item). Extended retractable gear, raised spoiler panels, and drag items given a `body` shape (fixed gear, wing struts); the airflow view draws their wakes. `A.bodies()` returns `{ n, b }` (a view: copy what you keep) |

## Turboprops

A turboprop's **power levers** are the throttles (`avd_set_input`'s
`throttle`, `avd_set_throttle(engine, v)`): 0 idle to 1 the forward stop,
where the fuel control's torque limiter (100 % torque, 1,050 shp on the
King Air 350) and ITT limiter (820 °C) hold, so the stop is take-off power
low down and the ITT-limited maximum higher up. Their travel **below idle**
is `avd_set_reverse(v)`: with the power levers at idle and the main wheels
on the ground, 0–0.4 takes the blades from the flight-idle stop to the
reverse stop at idle Ng (**beta**), 0.4–1 raises Ng into **reverse**; in the
air nothing moves (the flight-idle gate). The **propeller levers**
(`avd_set_prop_lever`) select the governed Np; the feather detent is
`avd_set_feather`. The **condition levers** (`avd_set_condition`) are fuel
cut-off, low idle and high idle. The **starter** is the cockpit control
`engine<N>.starter` (ON cranks the gas generator; light-off needs the
condition lever out of CUT-OFF above 13 % Ng; the switch goes OFF by itself
at stabilised idle), and `propeller.autofeather` (OFF / ARM) the autofeather:
armed with both power levers above 85 % and both engines above 24 % torque,
it feathers an engine whose torque falls below 12 %. A propeller feathers by
itself when the engine runs down on the ground (no governor oil); a
windmilling propeller in flight keeps its oil and windmills at the governed
speed until it is feathered. `avd_state()` [19] is engine 1's Np (rpm) and
[30] any engine running; `avd_jet()` carries the gear, flap, V-speed,
generator and battery fields for turboprops too ([0], its turbofan count,
stays 0).

`avd_turboprop()`, 48 doubles (the needles marked * read 0 without bus power):

| Index | Meaning |
|---|---|
| 0 | turboprops fitted |
| 1 | propeller lever 0..1 |
| 2 | selected Np, rpm |
| 3 | autofeather fitted |
| 4 | autofeather switch (1 ARM) |
| 5 | autofeather armed |
| 6 | reverse (beta) lever 0..1 |
| 7 | gauges powered |
| 8 + 20i | engine i (0, 1): torque % * |
| 9 + 20i | torque ft lbf * |
| 10 + 20i | ITT °C * |
| 11 + 20i | Ng % * |
| 12 + 20i | Np rpm * |
| 13 + 20i | fuel flow kg/h * |
| 14 + 20i | fuel flow lb/h * |
| 15 + 20i | oil pressure psi * |
| 16 + 20i | oil temperature °C * |
| 17 + 20i | blade angle deg at 0.75 R |
| 18 + 20i | feathered |
| 19 + 20i | beta (blade angle on the power lever: ground beta / reverse) |
| 20 + 20i | running |
| 21 + 20i | lit (burning; starting or running) |
| 22 + 20i | condition lever (0 cut-off, 1 low idle, 2 high idle) |
| 23 + 20i | starter switch |
| 24 + 20i | feather detent |
| 25 + 20i | thrust kN (− windmilling or reverse) |
| 26 + 20i | shaft power shp |
| 27 + 20i | feathered by the autofeather |

`A.turboprop()` → `{ count, propLever, npSelected, autofeather: { fitted, on,
armed }, reverse, powered, engines: [{ torque, torqueFtLb, itt, ng, np,
fuelFlowKgH, fuelFlowLbH, oilPsi, oilC, blade, feathered, beta, running, lit,
condition, starter, feather, thrustKn, shp, autofeathered }] }`.

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
| 63 | water on the airfield (mm): `avd_set_runway_water`, or the rain's film when deeper |

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
Electrical (index 75): load shed active (1 while the non-essential loads are off: the
pilot's `electrical.load_shed`, or the jet's bus controller on battery power); the
panel shows LOAD SHED / ESS BUS ONLY from it. Indices 76–95 reserved.

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
