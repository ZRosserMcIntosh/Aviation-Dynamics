# Fidelity matrix

What is modelled, how, and how far it is validated, as of October 2026. Rows
cover the master brief's areas, including the ones not started.

Legend: ✅ implemented and tested · 🟡 partial or simplified · ⛔ not started.

| Area | Status | Model today | Validation | Main gap |
|---|---|---|---|---|
| Rigid-body 6-DoF | ✅ | RK4, 240 Hz, quaternion, WGS-84 local frame, rebase | Determinism, convergence, rebase tests | Rotating-earth terms (Coriolis) ignored: fine below Mach 1 at this scale |
| Atmosphere | ✅ | ISA-1976, ΔT, QNH, humidity, one cloud layer with liquid water | Table checks against ISA; icing tests | Precipitation; airframe icing (wing, prop, induction) not modelled |
| Wind and turbulence | ✅ | Log-law boundary layer with surface roughness, Ekman veer, winds aloft; MIL-HDBK-1797 turbulence (height-dependent, anisotropic), mechanical turbulence from the actual wind, severity classes, 1-cosine gusts, Oseguera–Bowles microbursts, Allen thermals (stream function, life cycle, drift), Queney ridge lift and mountain waves, convective σ ([architecture/wind.md](architecture/wind.md)) | `tests/test_wind.cpp`: MIL σ/L values, realised RMS and anisotropy, gradient consistency | See the weather-field row |
| Weather field | ✅ | `SimpleWeather`: roughness patches (city vs airfield), wind stations interpolated in space and time, moving wind shifts and gust fronts, obstacle (building/hangar) wakes; São Paulo surroundings for SBMT | `tests/test_wind.cpp` spatial cases; web smoke test flies a microburst on final | No pressure/temperature fields (fronts shift wind only); internal boundary layers at roughness edges not modelled; trapped lee waves and rotors absent |
| Wake turbulence (aircraft) | ⛔ | — | — | Building wakes are in the weather field; aircraft wake vortices are not |
| Wing aerodynamics | ✅ | Strip theory, ±180° polar, flaps, ground effect, downwash lag | Stall speeds +1.6%/+2.6%; adverse yaw; stability signs | No vortex-lattice interference; no spin validation |
| Fuselage and drag | 🟡 | Munk moment, component drag build-up (calibrated) | Cruise −3.9%, maximum +0.4% | Calibrated scale factor |
| Compressibility | 🟡 | Korn/Lock drag rise | Unit test | Irrelevant to the C172; unvalidated for jets |
| Propeller | ✅ | BEM with Prandtl tip loss, swirl, element compressibility (2-D tables in advance angle and Mach), fixed pitch | Static rpm in range; climb −4.7%; static figure of merit in the measured range | Take-off roll −12.5% (in tolerance; residual in lift-off/transition, see validation) |
| Slipstream | 🟡 | Axial momentum plus swirl angle on washed strips | Tail-down take-off roll | Simplified contraction |
| Piston engine | ✅ | MAP, φ, fmep, temperatures, oil pressure, magnetos, starter | Rated power, altitude lapse, mixture | No carburettor icing, detonation, shock cooling |
| Turboprop / jet / rotor | ⛔ | — | — | Milestones 4–6 |
| Fuel system | ✅ | Tanks, selector, shutoff, line volume, starvation, leaks | Scenario A/C tests | No unporting in uncoordinated flight |
| Electrical | ✅ | Battery OCV/resistance, alternator, named loads and breakers | Depletion, VOLTS annunciator | No bus split, no avionics bus relay |
| Vacuum | ✅ | Pump suction against rpm | Unit test | No regulator failure modes beyond pump failure |
| Landing gear and ground | 🟡 | Struts, bristle tyres, brakes, steering, hard points | Static equilibrium, tip-over, brakes | Landing roll −19% with a modelled pilot technique; no surface wetness |
| Instruments (analog) | ✅ | Full six-pack, compass (with airframe deviation and a generated correction card), engine and system gauges, annunciators, pitot icing and heat | 37 tests | Static-port icing; instrument lighting photometry |
| Navigation radios | 🟡 | VOR, LOC, GS, radio horizon, synthetic facilities | Geometry tests | No real navaid data; no DME, ADF or marker |
| Glass cockpit (G1000) | ⛔ | — | — | Separate profile on the same `CockpitState` pattern |
| Autopilot | ⛔ | (assist wing leveller only, a gameplay aid) | — | — |
| Damage and failures | 🟡 | Injectable instrument and system failures; component loss flag in aero | Failure tests | No structural loads, no damage model |
| Fire, bailout | ⛔ | — | — | — |
| Assists | ✅ | Arcade, Sport, Simulation acting on controls | Assist tests | — |
| Procedures | ✅ | Quick Start through cockpit controls | Procedure tests | Only one checklist |
| World integration | 🟡 | ENU world frame at MASP; SBMT runway; `TiledWorldGround` (terrain, surfaces, buildings, wall strikes) with a documented tile format | Runway geometry; world-ground tests on a synthetic tile | Real São Paulo tile data not exported yet |
| Browser runtime | ✅ | wasi-sdk WASM (about 636 KB), JS loader, 2D cockpit | Node smoke test, headless Chromium check | No 3D cockpit |
| Multiplayer | ⛔ | — | — | ADR 0003: deterministic per platform only |
| Audio | 🟡 | Stall horn in the web cockpit | — | No engine, wind or ground audio model |
