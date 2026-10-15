# ADR 0003: Fixed 240 Hz step with RK4

Status: accepted (Milestone 1)

## Context

The stiffest dynamics in a light aircraft are the landing-gear struts and tyre
contact, with natural frequencies of about 10–20 Hz and short damping
transients. Next come the short-period and roll modes (about 1–3 Hz) and the
engine/propeller shaft. Hosts run at whatever frame rate the browser gives
them: 30–144 Hz, with hitches. Multiplayer and replays need determinism.

## Decision

- Use a fixed step of `kDt = 1/240 s`, independent of the host frame rate.
  Hosts accumulate wall time and run whole steps (`avd_advance`). Each call
  is clamped to 0.25 s, so a hitch slows the simulation instead of exploding
  it.
- Integrate the rigid body and shaft speed with classic RK4. Discrete
  subsystems (fuel lines, electrical, gyros, needles, bristles) advance once
  per step in a fixed order after the rigid-body update.
- Evaluate forces at each RK stage with the same context (atmosphere sample,
  ground planes). The context is rebuilt once per step.

## Consequences

- Tyre and strut contact are stable at 240 Hz with the published stiffness
  values. Tests check that a parked aircraft settles and stays still.
- The step costs about 30 µs natively, or about 7 ms of CPU per second of
  flight, which leaves room for several aircraft in the browser.
- The result is identical for 30 Hz and 144 Hz hosts (tested). Bit-identical
  results across platforms are not guaranteed: libm differs. Multiplayer will
  need authority or reconciliation rather than lock-step.
