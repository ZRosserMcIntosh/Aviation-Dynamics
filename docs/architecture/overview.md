# Architecture overview

## Layers

```
data/aircraft/*.json ──► aircraft_loader ──► AircraftDefinition (validated, SI)
                                                    │
               ┌────────────────────────────────────┴───────────────────────────┐
               ▼                                                                ▼
   Aircraft (avd/aircraft.hpp)                                    InstrumentPanel (avd/instruments.hpp)
   ├─ MassModel          mass, CG, inertia from stations + fuel     sensors ─► instrument dynamics ─► CockpitState
   ├─ AeroModel          strips, fuselage, drag items, slipstream          ▲
   ├─ Propeller + PistonEngine                                             │ SensorInputs (truth)
   ├─ FuelSystem / ElectricalSystem / VacuumSystem                         │
   ├─ Gear + contacts    over a GroundQuery                                │
   └─ RK4 integrator ────────────────── postStep ──────────────────────────┘
               ▲                                       │
   PilotInput (axes, levers)                AircraftTelemetry, events
   cockpit controls (switches)
               ▲
   Assist layer (Arcade / Sport / Simulation) and ProcedureRunner, both acting only on the controls
               ▲
   Host: avdemo, tests, the WASM C ABI (web/avd_web.cpp), the game later
```

The core (`avd_core`) has no renderer, engine, OS or file-system dependency
apart from the optional JSON file loader. Hosts provide three things: a
`GroundQuery` (terrain and runway surfaces), an `AtmosphereField` (weather) and
a wall-clock accumulator that calls `Aircraft::step(kDt, …)` at the fixed step.

## Frames and units (ADR 0002)

| Frame | Axes | Used for |
|---|---|---|
| Body | FRD: x forward, y right, z down; origin at the CG for dynamics, at the structural datum for geometry | forces, moments, rates |
| Local tangent | NED at the `LocalFrame` origin on WGS-84 | integration state |
| World (game) | ENU: x east, y north, z up; origin MASP (−23.5611, −46.6558) | rendering, São Paulo tiles |
| Structural | origin at the firewall face on the thrust line (the C172 datum), X forward, Y right, Z down | the aircraft JSON |

The attitude quaternion maps body to NED. `world_frame.hpp` converts NED/FRD
to the renderer's ENU/FLU. Units are SI everywhere inside the core. Knots,
feet, inHg and gallons exist only at the edges: in JSON field names with
explicit suffixes, the instrument displays and the reports.

`LocalFrame::rebase` moves the tangent origin, preserving the ECEF position,
velocity and attitude, so flights far from the origin keep metre-level
precision. A flat-earth game world uses the same frame without rebasing.

## Time step (ADR 0003)

`kDt = 1/240 s`, with classic RK4 on position, velocity, attitude, body rates
and shaft speed. Subsystems with their own state (fuel lines, battery, gyro
rotors, needles, tyre bristles) advance once per step after the rigid-body
update, in `postStep`, in a fixed order. Results do not depend on the host
frame rate (`tests/test_instruments.cpp`, "frame-rate independence"). The step
costs about 30 µs natively (`avdemo bench`).

## Flight model

- **Atmosphere.** ISA-1976 layers to 84.852 km geopotential with ΔT and QNH, Buck vapour
  pressure for moist density, a wind power law in the boundary layer, and
  frozen-field von Kármán turbulence summed from Fourier modes with an analytic
  gradient. The gradient gives the strip theory spatially varying gusts.
- **Lifting surfaces.** Each panel is split into strips. Section lift and drag
  come from a full ±180° polar: the published linear range, Viterna-Corrigan
  post-stall and reversed flow. Wings use a nonlinear lifting line
  (horseshoe vortices solved together, ground images, artificial viscosity
  after the stall) for the span loading; tails use the Helmbold 3D slope with
  a strip-local induced angle. Flaps change the zero-lift angle and clmax
  through τ. Compressibility uses Korn/Lock drag rise. Downwash at the tail is
  lagged by the transport time. Ground effect reduces induced angle near the
  ground.
- **Propeller and slipstream.** Thrust and torque come from blade-element
  momentum theory with Prandtl tip loss, axial and swirl (tangential)
  induction, and per-element compressibility (Prandtl-Glauert lift, Korn/Lock
  drag rise from a blade thickness distribution). It is solved once per
  propeller definition into tables over advance angle and Mach, and shared
  between aircraft. Slipstream velocity follows axial momentum
  theory, with a swirl angle applied to the strips inside the slipstream tube.
  Slipstream scrubbing drag acts on the drag items it washes.
- **Piston engine.** Airflow is set by manifold pressure and volumetric
  efficiency. Power comes from an equivalence-ratio table, indicated efficiency
  (calibrated to rated power) and friction mean effective pressure. The model
  also covers magnetos and the starter, lumped EGT/CHT/oil temperatures, and
  oil pressure from a pump, relief valve and temperature-dependent viscosity.
- **Ground.** Struts are springs with damping and a preload. Tyres use a
  bristle model with rolling resistance. The model includes
  differential brakes, nose-wheel steering, and hard points for wingtips, tail
  and propeller that report contact events. Steering follows the rudder
  pedals and, where a gear leg has one, a tiller (`tiller_steer_deg`); the two
  commands add, limited to the larger authority. A powered steering actuator
  has a slew-rate limit (`steer_rate_deg_per_s`); without one the wheel
  follows a mechanical linkage at once.
- **Wake.** `Wake` (`avd/wake.hpp`) is what the aircraft leaves in the air,
  outside the integrated state. The aircraft owns no reference to it. Each
  surface's trailing-edge circulation (Kutta–Joukowski from the strips' lift)
  is shed at intervals of about half a span. The shed vortices form "gates" of
  vortex rings that stay where they were shed. Each gate:
  - rolls up into one vortex per same-sign run (Betz) over the Spreiter–Sacks
    distance;
  - moves with the local wind plus its own induction and ground images;
  - grows Lamb–Oseen cores (Squire);
  - decays in two phases after Holzäpfel's P2P model.

  Older gates are thinned. `WakeField` wraps an `AtmosphereField` with wakes,
  so an aircraft flying into one feels it through the ordinary air sample. The
  sample uses the wake gradient averaged over the span. The host updates the
  wake after each step, and the web build passes `WakeField` to the step.
- **World.** `TiledWorldGround` provides the open-world ground: 500 m tiles
  with a terrain heightfield, surface classes and extruded buildings. Hard
  points inside a building are pushed out through the nearest wall, and the
  contact event reports the surface struck. See
  [world-ground.md](world-ground.md).

## Systems

| System | Model |
|---|---|
| Fuel | Tanks with position, unusable quantity, sender. Selector and shutoff valve. Line volume between valve and engine, so the engine runs on for seconds after starvation. Leaks. |
| Electrical | Lead-acid battery on its loaded plateau with a surface charge, internal resistance that rises at low charge, CC/CV charge taper; one alternator or declared per-engine generators in parallel (own switch, failure and cut-in speed), a definition-level bus minimum voltage. Named loads with breakers and minimum voltage; the fuel-pump breaker carries the pump's current only while it runs. |
| Hydraulics | Optional: systems pressurised by engine-driven pumps and/or an electric pump needing generator power; gear, flaps, speedbrake, steering and brakes name their systems and move at a rate scaled by the pressure; brake accumulator; gear free-fall release. |
| Vacuum | Engine-driven pump with a regulated suction against rpm. The gyro rotors belong to the instruments that use them. |
| Flaps and trim | Electric flap motor with a rate, detents and a load (or hydraulic, see above). Trim position with a rate. |

## Instruments

The design is in [instruments-and-cockpit.md](instruments-and-cockpit.md). The
pipeline runs FDM truth → `SensorInputs` → instrument dynamics →
`CockpitState`. Renderers read only `CockpitState`.

## Aircraft schema

An aircraft is one JSON file with these sections:

- metadata, `sources` and `provenance`;
- `mass` (empty mass, CG, inertia and stations), `reference`;
- `surfaces`, `fuselages`, `drag_items`;
- `engines` (engine, propeller and oil system);
- `fuel`, `electrical` (with optional `generators`), `hydraulics` (optional), `vacuum`;
- `gear`, `contacts`, `controls`;
- `pitot_static`, `stall_warning`, `panel`, `limits`.

`aircraft_loader` rejects missing or out-of-range fields with a path-qualified
message. It never throws, because the WASM build has no exceptions. The
`provenance` list tags fields and field groups with a kind of `published`,
`calibrated`, `estimated` or `gameplay`, plus a source and a note. The loader
parses and validates these entries, but it does not yet enforce that every
numeric field has one; completeness is checked by review. The Tupi T-180 file is the worked
example; see [docs/aircraft/tupi-t180.md](../aircraft/tupi-t180.md).

## Assistance and procedures

`AssistLayer` turns raw stick input into control positions, and never modifies
forces or state. It has three presets:

- **Arcade:** auto-mixture, auto-coordination, auto-trim, wing leveller,
  stall prevention (limits nose-up input near the critical angle of attack),
  and rudder-to-brake steering on the ground.
- **Sport:** auto-mixture, auto-coordination and brake steering.
- **Simulation:** none.

Two options sit outside the presets:

- **Straight-and-level hold** (`levelHold`, "Auto-trim: hold straight and
  level" in the browser cockpit, key L), on top of any preset. With the stick
  and trim released, more than 30 m above the ground, it levels the wings,
  arrests any climb or descent, captures the heading and altitude, and holds
  them. It works like a two-axis autopilot with auto-trim: a small stick input
  makes the fast correction and the trim wheel integrates it away, so a power
  change ends with the airplane retrimmed. The trim only runs once the
  altitude is captured, so it does not wind up while the climb or descent is
  being arrested. Underspeed protection: closer than about 4° to the wing's
  stall angle, the hold lowers the nose as far as it takes to keep that
  margin. It trades height for speed instead of stalling and climbs back once
  there is power. It never moves the throttle. Any stick or trim input from
  the player overrides it, and letting go captures new references.
- **Tiller handover** (`tillerHandover`, every preset). This is an input
  binding, not an assist. On the ground below 5 kt the yaw input (Q/E, the
  gamepad rudder axis) turns the nose-wheel tiller instead of the pedals,
  which is how crews steer heavy aircraft at walking pace. Above 5 kt it goes
  back to the pedals and rudder, and returns to the tiller below 4 kt. It does
  nothing on an aircraft without a tiller, such as the Tupi.

`ProcedureRunner` executes checklists such as the
Quick Start through the same cockpit controls a player would use.

## Error handling

Configuration errors are reported as lists of strings. Invariant violations
in the core call `AVD_FAIL`. It throws where C++ exceptions exist and aborts
with a message in the exception-free WASM build.
Non-finite state is detected after each step, and the aircraft is marked
`faulted()` instead of propagating NaN.
