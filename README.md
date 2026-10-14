# Aviation Dynamics

A physically grounded, data-driven flight simulation core in C++20 for the
São Paulo open world. It is renderer-free and engine-independent. Aircraft are
JSON definitions with per-field provenance. The same core runs natively and in
the browser as WebAssembly.

Status: **Milestone 1 (single-engine piston foundation)** and **Milestone 2a
(analog cockpit instrumentation)** are implemented. The roster has one
aircraft, the fictional **Tupi T-180**, whose simulation reference is the
Cessna 172S Skyhawk SP. Everything else in the long-term brief (more aircraft,
helicopters, jets, damage, multiplayer) is on the roadmap and is not claimed
here.

## What works today

| Area | What is implemented | Evidence |
|---|---|---|
| 6-DoF | Rigid body in a local NED tangent frame on WGS-84, quaternion attitude, RK4 at a fixed 240 Hz step, floating-origin rebase | `tests/test_foundations.cpp` |
| Atmosphere | ISA-1976 with temperature/QNH deviation, moist density, power-law boundary-layer wind, frozen von Kármán turbulence with spatial gradients | `tests/test_foundations.cpp` |
| Aerodynamics | Strip theory over lifting surfaces with a ±180° section polar, flaps, downwash lag, ground effect, prop slipstream and swirl, fuselage Munk moment, component drag build-up | `tests/test_components.cpp`, `tests/test_flight.cpp` |
| Propulsion | Fixed-pitch propeller from blade-element tables with swirl and tip compressibility; piston engine with manifold pressure, mixture, friction, magnetos, starter, temperatures and oil-pressure physics | `tests/test_components.cpp` |
| Systems | Fuel tanks/selector/shutoff with line volume, starvation and leaks; battery/alternator/bus with named loads and breakers; engine-driven vacuum | `tests/test_systems_scenarios.cpp` |
| Ground | Spring-damper struts, tyre bristle model, brakes, nose-wheel steering, hard-point contacts, runway surfaces at Campo de Marte (SBMT) | `tests/test_flight.cpp` |
| Instruments | Pitot-static (IAS/CAS/EAS/TAS, position error, blockages), VSI capillary lag, vacuum attitude and heading gyros, turn coordinator and ball, pendulous wet compass, engine and system gauges, annunciators, VOR/LOC/GS receiver, transponder encoder | `tests/test_instruments.cpp` (34 cases) |
| Validation | Against the C172S POH: stall, cruise, maximum speed, climb, take-off, landing | `avdemo validate`, `tests/test_validation.cpp` |
| Browser | WASM build (wasi-sdk, no exceptions or RTTI), Node smoke test, interactive 2D cockpit | `web/` |

Current results against the POH are in
[docs/validation/c172s-validation.md](docs/validation/c172s-validation.md).
That document also covers the two known discrepancies: the take-off ground roll
is about 12% short (investigated; the remaining gap is in the lift-off and
transition) and the landing roll with maximum braking is about 56% short.

## Build and run

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/tests/avd_tests            # 100 test cases
./build/avdemo validate            # POH comparison table
./build/avdemo flight              # SBMT circuit: quick start, take-off, stall, pattern, landing
./build/avdemo instruments [csv]   # truth versus display through a scripted flight
./build/avdemo starvation | shutdown | cockpit | bench
```

Options: `AVD_BUILD_TESTS`, `AVD_BUILD_TOOLS`, `AVD_WARNINGS_AS_ERRORS`.

The browser build needs [wasi-sdk](https://github.com/WebAssembly/wasi-sdk);
version 25 is tested.

```sh
WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh   # web/dist/avd.wasm, avd.mjs, tupi_t180.json
node web/smoke_test.mjs
node web/cockpit/build_page.mjs                # web/dist/cockpit.html (self-contained)
node web/cockpit/browser_test.mjs              # headless Chromium check (needs playwright)
```

## Repository layout

```
src/core/      avd_core: the engine-independent flight core (namespace avd)
src/scenarios/ trim, scripted test pilot, Campo de Marte and São Paulo navigation data
tools/avdemo/  headless demos and the validation report
tests/         dependency-free test harness and suites
data/aircraft/ aircraft definitions (JSON, with provenance)
web/           C ABI, WASM build, JS loader, smoke test, 2D cockpit
docs/          architecture, decisions (ADRs), aircraft, validation, fidelity matrix, roadmap
```

## Principles

The full statements are in [docs/architecture/overview.md](docs/architecture/overview.md).

- **No artificial limits.** Nothing caps speed, rate or load; the physics does.
- **Assistance is not physics.** Arcade and Sport assists act only through the
  pilot's controls ([ADR 0004](docs/decisions/0004-assistance-is-not-physics.md)).
- **Scientific honesty.** Every aircraft parameter is tagged `published`,
  `calibrated`, `estimated` or `gameplay`, with its source. A disagreement with
  reference data is documented, not tuned away.
- **Independent model.** JSBSim (LGPL) is used only as a published reference,
  never as code ([ADR 0001](docs/decisions/0001-independent-flight-model.md)).

## Licence

MIT. The Tupi T-180 is a fictional aircraft. Its performance reference comes
from public Cessna 172S data, and no manufacturer branding or artwork is used.
