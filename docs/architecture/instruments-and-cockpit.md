# Instruments and cockpit: assessment and design

Status: Milestone 2a (October 2026). This document is the written plan the
instrumentation work follows, and the reference for how instruments interface
with the flight engine. ✅ implemented · 🟡 partial · 🧭 planned.

## 1. Assessment of the existing engine (before this milestone)

| Question | Finding |
|---|---|
| Flight-dynamics architecture | `avd::Aircraft` integrates a 6-DoF rigid body with RK4 at a fixed 240 Hz step (`src/core/src/aircraft.cpp`). State: CG position/velocity in a local NED tangent frame, attitude quaternion body(FRD)→NED, body rates, shaft speed, tyre bristles. SI internally. |
| Modelled variant | Cessna 172S Skyhawk SP (Lycoming IO-360-L2A, fuel injected, fixed-pitch prop), published as the fictional *Tupi T-180* (`data/aircraft/tupi_t180.json`). The definition did not yet state its panel/avionics fit. **Decision:** conventional analog ("steam gauge") C172S panel, vacuum attitude and heading indicators, electric turn coordinator, single NAV/COM with a VOR/LOC/GS indicator, transponder with altitude encoder. No GPS, ADF, DME or autopilot by default (these were options on the real airplane). All of it is configuration data. |
| Atmosphere | `StandardAtmosphere`/`SimpleWeather` give static pressure, temperature, moist density, speed of sound and wind with spatial gradient at the aircraft. |
| Engine state | `PistonEngineState`: rpm, MAP, φ, fuel flow, brake power; lumped EGT/CHT/oil temperature; oil pressure was a linear function of rpm (replaced, see §4). |
| Electrical / vacuum | `ElectricalSystem` (battery, alternator, bus, named loads with breakers), `VacuumSystem` (engine-driven pump, gyro rotor speed). |
| Existing instrumentation | `avd::Instruments`: pitot-static IAS/CAS with static-source position error and blockages, altimeter, a first-order VSI, simplified vacuum-gyro AI/HI, canted turn gyro, ball, fuel/engine gauges and a pneumatic stall vane. **Extended, not duplicated:** this milestone replaces its internals with explicit sensor + instrument-dynamics models and keeps its outputs. |
| Rendering / cockpit assets | None in this repository: the core is renderer-free by design. The game's existing browser path (São Paulo World driving page) loads the vehicle core as WebAssembly and renders with JavaScript. No 3D cockpit model exists. **Decision:** a functional 2D instrument panel in the browser, driven entirely by the C++ instrument state through WebAssembly (`web/`). A 3D cockpit later binds to the same `CockpitState`. |
| Update loop / timing | Fixed step inside `Aircraft::step`; hosts accumulate wall time (`avd_advance`). Instruments already updated in the fixed step. |
| Input bindings | `PilotInput` (axes, levers, brakes, trim) + cockpit-control registry (`setCockpitControl`, "Press F to interact"). |
| Tests / telemetry | Dependency-free `avd_tests`; `TelemetryRecorder` CSV. |
| Missing outputs for instruments | Air velocity at the pitot location (rotation effects), specific force at the instrument location (done at the CG), geodetic latitude (earth-rate drift), magnetic field, navigation facilities, oil pressure physics, ammeter/annunciator thresholds, lighting and pitot-heat loads, OAT probe. Added in this milestone. |

## 2. Data flow

```
Flight dynamics / engine / systems / environment      (authoritative physical state)
        │  truth: air state, air velocity at probes, attitude quaternion, body rates,
        │  specific force, position/lat, rpm, oil, fuel quantities, bus voltages, suction
        ▼
Physical sensors                                       (avd/instruments.hpp: SensorInputs)
        │  pitot total & static pressure (+ position error, blockage, heat),
        │  gyro rotor speeds (vacuum / electric), OAT probe (ram rise), fuel senders
        ▼
Instrument dynamics                                    (InstrumentPanel::update, fixed step)
        │  ASI capsule, altimeter, VSI capillary leak, vacuum AI (erection, tumble),
        │  DG (drift, gimbal error, sync), turn gyro + inclinometer, compass card,
        │  engine gauges, annunciators, NAV receiver, transponder encoder
        ▼
CockpitState  (plain struct, copyable, renderer-ready: needle angles & readouts)
        ▼
Rendering (2D web panel today, 3D cockpit later) — reads only CockpitState
```

Rules:
- Gauges never compute aircraft state. They receive measured quantities (pressures,
  rotor-relative attitudes, voltages) and add only physically justified instrument
  behaviour: lag, damping, calibration error, precession, mechanical limits,
  power-loss behaviour. Every parameter is configuration data (`panel` section of
  the aircraft JSON) with provenance.
- One update per fixed simulation step. Rendering reads a snapshot (`CockpitState`)
  and never feeds back except through explicit controls.
- Controls that change instrument state (Kollsman knob, DG sync, OBS, radio tuning,
  panel lighting) are instrument inputs; controls that change aircraft systems go
  through the existing cockpit-control registry. No event is emitted for a control
  set to its current value.

## 3. Reference data and what is verified

Aircraft-specific values come from the Cessna 172S POH (Revision 5, 2004), sections
2 (instrument markings, limitations) and 7 (systems), and FAA references: *Pilot's
Handbook of Aeronautical Knowledge* (FAA-H-8083-25B, ch. 8 "Flight Instruments"),
*Instrument Flying Handbook* (FAA-H-8083-15B), AC 43.13-1B. Where the POH figure
is recalled rather than read from a primary copy in this environment, the panel
data marks it `published (verify against the POH revision of the target airplane)`.
Instrument-internal constants that no POH publishes (gyro erection rates, needle
damping, capillary time constants) are labelled `estimated`, with the source of the
range (FAA handbooks, manufacturer data for typical instruments) in
`docs/aircraft/tupi-t180-panel.md`.

## 4. Implementation plan (this milestone)

1. ✅ `PanelDef` in `AircraftDefinition` (+ JSON `panel` section, validated): installed
   instruments, power sources (vacuum / named electrical load), markings, sensor and
   gyro parameters, annunciator thresholds, avionics fit.
2. ✅ Sensor models: pitot/static at their real positions (local airflow incl. ω×r and
   angle of attack), blockage and pitot heat; OAT probe with ram rise; fuel senders.
3. ✅ Instrument models (one class per instrument, shared second-order needle filter):
   ASI, altimeter (three hands, Kollsman 28.10–31.00 inHg / hPa), VSI (case-pressure
   capillary), vacuum AI (rotor-axis gyro with pendulous erection and tumble limits),
   DG (inertial spin axis, earth-rate and mechanical drift, gimbal error, sync knob,
   heading bug), turn coordinator (canted rate gyro, power flag), inclinometer
   (damped ball on apparent gravity), magnetic compass (pendulous card in the local
   field with dip: turning and acceleration errors emerge; deviation card).
4. ✅ Engine/system gauges: tachometer, oil pressure (pump + relief valve + viscosity),
   oil temperature, fuel quantity L/R, EGT/fuel flow, ammeter, suction, annunciators
   (OIL PRESS, LOW VAC, VOLTS, L/R LOW FUEL), clock, OAT; stall warning stays the
   pneumatic vane on the wing strip angle of attack.
5. ✅ Navigation: `NavDatabase` (VOR, localizer, glideslope, line-of-sight reception)
   loaded from JSON; NAV receiver with OBS, CDI, TO/FROM, GS, flags; COM tuning;
   transponder with Gillham-style 100 ft encoder. Only clearly labelled synthetic
   test facilities ship with the repository (no unverified real-world frequencies).
6. ✅ `CockpitState` snapshot + C ABI for the browser; 2D panel with outside view and a
   debug overlay comparing truth with display.
7. ✅ Tests: per-instrument unit tests and flight-scenario integration tests; CSV
   instrument telemetry.

Not in this milestone (documented in the fidelity matrix): G1000 profile, GPS/ADF/
DME/autopilot models, pitot/induction icing physics (blockage is injectable now),
instrument lighting photometry, 3D cockpit.
