# Fidelity matrix

What is modelled, how, and how far it is validated, as of October 2026. Rows
cover the master brief's areas, including the ones not started.

Legend: ✅ implemented and tested · 🟡 partial or simplified · ⛔ not started.

| Area | Status | Model today | Validation | Main gap |
|---|---|---|---|---|
| Rigid-body 6-DoF | ✅ | RK4, 240 Hz, quaternion, WGS-84 local frame, rebase | Determinism, convergence, rebase tests | Rotating-earth terms (Coriolis) ignored: fine below Mach 1 at this scale |
| Atmosphere | ✅ | ISA-1976, ΔT, QNH, humidity, boundary-layer wind, von Kármán turbulence | Table checks against ISA | No thermals, fronts, precipitation, icing |
| Weather field | 🟡 | `SimpleWeather`: uniform plus boundary layer and turbulence | — | No spatial weather or time evolution |
| Wake turbulence | ⛔ | — | — | — |
| Wing aerodynamics | ✅ | Strip theory, ±180° polar, flaps, ground effect, downwash lag | Stall speeds +1.6%/+2.6%; adverse yaw; stability signs | No vortex-lattice interference; no spin validation |
| Fuselage and drag | 🟡 | Munk moment, component drag build-up (calibrated) | Cruise −3.4%, maximum +0.8% | Calibrated scale factor |
| Compressibility | 🟡 | Korn/Lock drag rise | Unit test | Irrelevant to the C172; unvalidated for jets |
| Propeller | 🟡 | BEM tables, fixed pitch | Static rpm in range; climb −3.2% | **No swirl loss: take-off roll −20%** |
| Slipstream | 🟡 | Axial momentum plus swirl angle on washed strips | Tail-down take-off roll | Simplified contraction |
| Piston engine | ✅ | MAP, φ, fmep, temperatures, oil pressure, magnetos, starter | Rated power, altitude lapse, mixture | No carburettor icing, detonation, shock cooling |
| Turboprop / jet / rotor | ⛔ | — | — | Milestones 4–6 |
| Fuel system | ✅ | Tanks, selector, shutoff, line volume, starvation, leaks | Scenario A/C tests | No unporting in uncoordinated flight |
| Electrical | ✅ | Battery OCV/resistance, alternator, named loads and breakers | Depletion, VOLTS annunciator | No bus split, no avionics bus relay |
| Vacuum | ✅ | Pump suction against rpm | Unit test | No regulator failure modes beyond pump failure |
| Landing gear and ground | 🟡 | Struts, bristle tyres, brakes, steering, hard points | Static equilibrium, tip-over, brakes | **Landing roll −54% (braking technique)**; no surface wetness |
| Instruments (analog) | ✅ | Full six-pack, compass, engine and system gauges, annunciators | 34 tests | No icing physics, no compass deviation |
| Navigation radios | 🟡 | VOR, LOC, GS, radio horizon, synthetic facilities | Geometry tests | No real navaid data; no DME, ADF or marker |
| Glass cockpit (G1000) | ⛔ | — | — | Separate profile on the same `CockpitState` pattern |
| Autopilot | ⛔ | (assist wing leveller only, a gameplay aid) | — | — |
| Damage and failures | 🟡 | Injectable instrument and system failures; component loss flag in aero | Failure tests | No structural loads, no damage model |
| Fire, bailout | ⛔ | — | — | — |
| Assists | ✅ | Arcade, Sport, Simulation acting on controls | Assist tests | — |
| Procedures | ✅ | Quick Start through cockpit controls | Procedure tests | Only one checklist |
| World integration | 🟡 | ENU world frame at MASP; SBMT runway; landmarks in the web view | Runway geometry test | No terrain/buildings query from São Paulo World tiles yet |
| Browser runtime | ✅ | wasi-sdk WASM (about 636 KB), JS loader, 2D cockpit | Node smoke test, headless Chromium check | No 3D cockpit |
| Multiplayer | ⛔ | — | — | ADR 0003: deterministic per platform only |
| Audio | 🟡 | Stall horn in the web cockpit | — | No engine, wind or ground audio model |
