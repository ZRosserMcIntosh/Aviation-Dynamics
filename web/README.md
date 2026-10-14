# Browser runtime

The flight core compiles to WebAssembly with wasi-sdk. The build has no
exceptions or RTTI, and the WASI imports are stubs because the core does no
I/O. The C ABI is in `avd_web.cpp`.

```sh
WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh   # → web/dist/avd.wasm, avd.mjs, tupi_t180.json
node web/smoke_test.mjs                        # take-off, Quick Start, ILS reception, instruments
node web/cockpit/build_page.mjs                # → web/dist/cockpit.html (one self-contained file)
node web/cockpit/browser_test.mjs              # headless Chromium check of the bundled page (needs playwright)
```

`avd.mjs` exports `createAvdModule(bytes?)`, which returns the raw `avd_*`
exports plus these helpers:

- `load(json)`, which returns a list of errors;
- `state()` and `cockpitState()`, which return Float64Array views (copy them
  before the next call);
- `markings()`, `runway()`, `events()`, `cockpit()`, `procedure()`.

## Lifecycle

```
load(json) → avd_set_weather(...) → avd_reset(mode) → loop { avd_set_input(...); avd_advance(dt); read state }
```

`avd_advance(seconds)` accumulates wall time and runs whole 1/240 s steps. Each
call is clamped to 0.25 s. It returns the number of steps run.

| Function | Meaning |
|---|---|
| `avd_reset(mode)` | Reset scenario: 0 cold and dark on runway 12, 1 engine running on runway 12, 2 on final for runway 12 (3 km), 3 cruising over the city. Sets NAV 1 to 110.30 active and 113.70 standby, OBS and bug 122, COM standby 132.80, transponder ALT |
| `avd_set_weather(windMs, windFromDeg, turbulenceMs, isaDevC, humidity)` | Weather (rebuilds the field) |
| `avd_set_input(pitch, roll, yaw, throttle, mixture, brakeL, brakeR, trimRate, starter)` | Raw pilot input in −1..1 or 0..1. Starter is held while 1 |
| `avd_set_assist(p)` | 0 Arcade, 1 Sport, 2 Simulation |
| `avd_quick_start()` / `avd_procedure_status()` | Run the Quick Start checklist through the cockpit. Status: 0 idle, 1 running, 2 done, 3 failed; the message is in the text buffer |
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
