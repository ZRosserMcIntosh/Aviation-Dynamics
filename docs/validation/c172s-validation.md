# Validation: Tupi T-180 against the Cessna 172S POH

Reproduce with `./build/avdemo validate` (add `--json file` for the
machine-readable report). The rows, their sources and tolerances are in
`src/scenarios/validation.cpp`; `tests/test_validation.cpp` asserts every
one of them. The tables below are regenerated from the test output by
`tools/update_validation_docs.py` and must not be edited by hand (CI fails
when they are stale). Conditions: 2,550 lb, mid CG, ISA, no wind, no
assists. The scripted test pilot (`src/scenarios/test_pilot.cpp`) flies the
POH technique; the steady points come from the free-air trim solver.

## Results

<!-- validation:begin table aircraft=tupi_t180 -->
| Quantity | Published | Simulation | Error | Band | Result | Source |
|---|---|---|---|---|---|---|
| Stall speed, flaps up, power off<br><small>1 kt/s deceleration at idle, wings level; clean section clmax is the published NACA 2412 value (not fitted)</small> | 53 KCAS | 53.1 KCAS | +0.3 % | ±6 % | within | C172S POH Section 5 |
| Stall speed, flaps 30, power off<br><small>the flap clmax increment is calibrated to this row</small> | 48 KCAS | 47.9 KCAS | -0.1 % | ±6 % | within | C172S POH Section 5 |
| Stall warning margin, flaps up<br><small>horn on at 55.2 KIAS, stall at 46.7 KIAS</small> | 5-10 kt above the stall | 8.48 kt | — | 5–10 kt | within | C172S POH Section 5 |
| Stall warning margin, flaps 30<br><small>horn on at 50.1 KIAS, stall at 42.2 KIAS</small> | 5-10 kt above the stall | 7.85 kt | — | 5–10 kt | within | C172S POH Section 5 |
| Maximum level speed, sea level<br><small>full throttle, best power; 2692 rpm</small> | 126 KTAS | 126 KTAS | +0.4 % | ±5 % | within | C172S POH Section 5 |
| Engine speed at the maximum level speed | at most 2,700 rpm (red line) | 2,692 rpm | — | 0–2,700 rpm | within | C172S POH Section 5 |
| Full-throttle level speed, 8,000 ft<br><small>73 % power, 2590 rpm, propeller efficiency 0.84, 10.4 gph at best power</small> | 124 KTAS (75 % power, 2,650 rpm) | 120 KTAS | -3.6 % | ±5 % | within | C172S POH Section 5, Figure 5-8 |
| Power at full throttle, 8,000 ft | 75 % | 72.9 % | -2.8 % | ±8 % | within | C172S POH Section 5, Figure 5-8 |
| Fuel flow at full throttle, 8,000 ft<br><small>trimmed at best power (richer than the handbook's recommended lean)</small> | about 10.5 gph (recommended lean) | 10.4 gph | — | 8.50–12 gph | within | C172S POH Section 5, Figure 5-8 |
| Rate of climb, sea level, Vy<br><small>74 KIAS, full throttle</small> | 730 fpm | 707 fpm | -3.1 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Rate of climb, 4000 ft, Vy<br><small>72-74 KIAS, full throttle, mixture leaned for best power</small> | 600 fpm | 542 fpm | -9.7 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Rate of climb, 8000 ft, Vy<br><small>72-74 KIAS, full throttle, mixture leaned for best power</small> | 455 fpm | 382 fpm | -16.0 % | ±15 % | **OUTSIDE** | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Rate of climb, 12000 ft, Vy<br><small>72-74 KIAS, full throttle, mixture leaned for best power</small> | 310 fpm | 225 fpm | -27.4 % | ±15 % | **OUTSIDE** | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Time to climb, sea level to 8,000 ft | 14 min | 15.2 min | +8.5 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance) |
| Fuel to climb, sea level to 8,000 ft<br><small>the handbook adds 1.4 gal for start, taxi and take-off, not counted here</small> | 3.0 gal | 2.53 gal | -15.6 % | ±15 % | **OUTSIDE** | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance) |
| Distance to climb, sea level to 8,000 ft<br><small>zero wind</small> | 19 NM | 19.7 NM | +3.5 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance) |
| Cruise speed, 2000 ft, 55 % power<br><small>2223 rpm, throttle 60 %, mixture 50 degF rich of peak</small> | 100 KTAS | 100 KTAS | +0.2 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 2000 ft, 55 % power<br><small>recommended lean</small> | 7.8 gph | 7.30 gph | -6.5 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 2000 ft, 65 % power<br><small>2368 rpm, throttle 72 %, mixture 50 degF rich of peak</small> | 109 KTAS | 108 KTAS | -0.5 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 2000 ft, 65 % power<br><small>recommended lean</small> | 9.1 gph | 8.58 gph | -5.8 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 2000 ft, 75 % power<br><small>2493 rpm, throttle 83 %, mixture 50 degF rich of peak</small> | 116 KTAS | 115 KTAS | -0.6 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 2000 ft, 75 % power<br><small>recommended lean</small> | 10.5 gph | 9.85 gph | -6.2 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 8000 ft, 55 % power<br><small>2334 rpm, throttle 78 %, mixture 50 degF rich of peak</small> | 106 KTAS | 105 KTAS | -1.4 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 8000 ft, 55 % power<br><small>recommended lean</small> | 7.9 gph | 7.42 gph | -6.1 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 8000 ft, 65 % power<br><small>2489 rpm, throttle 92 %, mixture 50 degF rich of peak</small> | 116 KTAS | 114 KTAS | -2.0 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 8000 ft, 65 % power<br><small>recommended lean</small> | 9.2 gph | 8.72 gph | -5.2 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 8000 ft, 75 % power<br><small>full throttle gives only 71 % here; 2568 rpm, throttle 100 %, mixture 50 degF rich of peak</small> | 124 KTAS | 118 KTAS | -4.6 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 8000 ft, 75 % power<br><small>recommended lean</small> | 10.5 gph | 9.51 gph | -9.4 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Best glide ratio at 68 KIAS<br><small>throttle closed, engine idling at 1524 rpm, 3,000 ft; sink 336 fpm</small> | about 9.1:1 (1.5 NM per 1,000 ft) | 21.4 :1 | +135.1 % | ±15 % | **OUTSIDE** | C172S POH Section 5 (Maximum Glide) and Section 3 (68 KIAS) |
| Glide distance per 1,000 ft<br><small>from the ratio above</small> | — | 3.52 NM | — | — | info | — |
| Propeller thrust in the glide (throttle closed)<br><small>engine brake power 27.4 hp at 1524 rpm; + is pulling</small> | — | 477 N | — | — | info | — |
| Airframe lift/drag at 68 KIAS, no propeller force<br><small>aero model alone, L = W</small> | — | 11.9 :1 | — | — | info | — |
| Take-off ground roll (short field)<br><small>flaps 10, lift off at 51 KIAS, climb at 56 KIAS</small> | 960 ft | 812 ft | -15.4 % | ±20 % | within | C172S POH Section 5 (short-field take-off, sea level, ISA) |
| Take-off distance over a 50 ft obstacle | 1,630 ft | 1,605 ft | -1.5 % | ±20 % | within | C172S POH Section 5 (short-field take-off, sea level, ISA) |
| Lift-off speed | 51 KIAS | 51.4 KIAS | — | 48–55 KIAS | within | C172S POH Section 5 |
| Landing ground roll, pilot braking technique<br><small>touchdown at 50 KCAS, 1 s to the pedals, 1 s ramp, 70 % sustained pedal (no anti-skid); see the discussion in the validation note</small> | 575 ft | 459 ft | -20.1 % | ±25 % | within | C172S POH Section 5 (short-field landing, sea level, ISA) |
| Landing roll, tyre-limited bound (full pedal at touchdown)<br><small>physics check, not a handbook figure</small> | — | 286 ft | — | — | info | — |
| Static rpm, full throttle | 2,300-2,420 rpm | 2,357 rpm | — | 2,300–2,420 rpm | within | C172S POH Section 4 (full-throttle static check) |
<!-- validation:end -->

<!-- validation:begin summary aircraft=tupi_t180 -->
34 compared rows, 4 outside their band (c172s.climb.8000, c172s.climb.12000, c172s.climb.to8000.fuel, c172s.glide.ratio).
<!-- validation:end -->

Aerodynamics II replaced independent strips with a nonlinear lifting line
(span loading solved, ground images). The wing's section clmax is the
published smooth NACA 2412 value (1.60 at Re ~3 x 10^6, Abbott & von
Doenhoff) instead of a value fitted to the stall speed: **the clean stall
speed is a validation, not a calibration**. Only the flap's clmax increment
(0.55) is calibrated, to the flaps-30 stall.

Hot and high, for information: at SBMT (722 m) at ISA+20 the ground roll is
about 1,210 ft, against about 810 ft at sea-level ISA.

## Sources

| Rows | Source | Note |
|---|---|---|
| Stall speeds, stall warning | C172S POH Section 5 (stall speeds, 2,550 lb, power off) and Section 4 (warning 5–10 kt above the stall) | |
| Maximum level speed, full-throttle cruise at 8,000 ft | C172S POH Section 5, Figure 5-8 (cruise performance) and Section 1 (126 KTAS) | |
| Rate of climb at sea level, 4,000, 8,000 and 12,000 ft; time, fuel and distance to 8,000 ft | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; standard temperature column: 730, 600, 455, 310 fpm; 14 min, 3.0 gal, 19 NM; no start, taxi and take-off allowance) | Transcribed from the handbook table, confirm against the printed page before relying on a second decimal; the task that added the rows quoted about 730, 590, 440 and 290 fpm from memory, within a few percent of these |
| Cruise table, 2,000 and 8,000 ft at 55, 65 and 75 % power | C172S POH Section 5, Figure 5-8 (recommended lean mixture: 50 °F rich of peak EGT, Section 4). The handbook tabulates by rpm; the rows are its nearest rows interpolated in % BHP: 100/109/116 KTAS and 7.8/9.1/10.5 gph at 2,000 ft, 106/116/124 KTAS and 7.9/9.2/10.5 gph at 8,000 ft | As above: transcribed, confirm against the page. The simulation trims at an equivalence ratio of 1.06 for the recommended lean |
| Best glide | C172S POH Section 3 (68 KIAS) and Section 5 (maximum glide: propeller windmilling, flaps up, zero wind, about 1.5 NM per 1,000 ft, 9.1:1) | |
| Take-off, landing | C172S POH Section 5 (short-field take-off and landing, sea level, ISA) | Landing: demonstrated figure with an unpublished braking technique; see below |
| Static rpm | C172S POH Section 4 (full-throttle static check 2,300–2,420 rpm) | |

## Rows outside their band (October 2026) and what they point to

Nothing in this section was tuned. The rows added in October 2026 (climb
lapse, cruise table, glide) are the first to probe the engine at part
throttle and at altitude, and they found three things.

**Rate of climb lapses too fast with altitude.** Sea level −3 %, 4,000 ft
−10 %, 8,000 ft −16 % (outside ±15 %), 12,000 ft −27 % (outside). The
time to 8,000 ft is +9 % (15.2 min) and the distance +4 %; the fuel to
8,000 ft is −16 % (outside), consistent with the cruise fuel flows below
being 5–8 % lean of the handbook. Sea-level power, static rpm and the
sea-level climb are right, so the candidates are the power lapse with
altitude (manifold pressure and the mixture model at the leaned best
power) and the propeller's efficiency at the lower density, both of which
the full-throttle 8,000 ft row already shows a little low (73 % power at
2,590 rpm against the handbook's 75 % at 2,650 rpm).

**Best glide: 21:1 against 9:1.** The airframe alone has a lift-to-drag
ratio of 11.9 at 68 KIAS (the `airframe_ld` row), which is where a clean
172 with no propeller drag belongs. The difference is the engine: with the
throttle closed the trim settles at 1,524 rpm with 27 hp of brake power and
**477 N of propeller thrust**, where a real idling O-360 behind a
windmilling propeller gives roughly zero or a little drag. The idle
manifold-pressure ratio (0.33 of ambient at closed throttle) delivers too
much power at a windmilling rpm. This is the row to close first: it
changes every power-off descent and the landing flare.

**Cruise table.** Speeds within −0.5 to −4.6 % at every point; fuel flows
−5 to −8 % at the recommended lean (within ±15 %). 75 % power at 8,000 ft
is not available (full throttle gives 73 %), so that row flies full
throttle: 118.3 KTAS against 124 (−4.6 %, inside the ±5 % band by a
little).

## Investigated comparisons

Every comparison above is within its tolerance; the test bounds are
symmetric. The two that once needed a "known discrepancy" note are recorded
here with what closed them.

**Take-off: ground roll −12.5%, air segment long.** The ground roll is
840 ft against 960 ft (it was −19.9% and carried a widened 0.75× lower test
bound). The distance from lift-off to 50 ft is about 880 ft against the
POH's 670 ft; the total over 50 ft is +5.8%, within tolerance.

*Investigation (Milestone 2b).* The original hypothesis was propeller swirl
loss. That was tested and **refuted**. Adding tangential induction to the BEM
solver let the engine turn faster at static (lower torque) for about the same
thrust, and the roll barely changed (−19.9% → −19.8%). Swirl is kept because
it is correct physics. Each blade element solves the axial and tangential
momentum balances together (dT = 4πrρF(V+v)v dr, dQ = 4πr³ρF(V+v)Ωa′ dr):
a bracketed root on the swirl, with the axial balance re-solved at each trial
value. This replaced an under-relaxed fixed point that left high-tip-Mach and
reverse-flow table entries unconverged; the operating envelope, and so every
number on this page, was unchanged. The investigation then established:

1. **The lift-off speed is right.** The sim lifts off at 51.4 KIAS = 55.1 KCAS,
   matching the POH technique (lift off at 51 KIAS, about 55 KCAS with the
   high-α position error).
2. **The propeller was too efficient at static.** A measured C172 static
   thrust (O-320, 75 × 53 in McCauley: 211 kgf, figure of merit about 0.37)
   scales to about 2400–2500 N for this airplane. The model gave 3056 N. The
   propeller elements had no compressibility, although the 76 in tips run at
   Mach 0.7–0.8. Prandtl-Glauert lift and Korn/Lock drag rise were added per
   element, with a blade thickness distribution, and the tables became 2-D
   in Mach. The pitch returned to the **published 60 in**, replacing the
   compensating 1.70 m. The chord was recalibrated to the published static
   rpm.
3. **Rolling resistance was an icy-runway value.** It went from 0.02 to 0.04,
   the middle of the textbook range for dry asphalt with brakes off.

Together these took the roll from −19.9% to −12.5%, with every other
comparison still in tolerance. No parameter was tuned to the take-off
number. The sweep showed that no plausible propeller or rolling-resistance
value closes the gap alone: adding propeller losses lengthens the roll but
costs climb and cruise one for one.

What remains points to the **lift-off and transition**: the sim leaves the
ground at the right speed but climbs away slowly. Candidates are the
scripted pilot's rotation and climb technique, flaps-10 drag in ground
effect, and the POH's data reduction. This is within tolerance and not
tuned; it is the first place to look if the take-off is refined further.

**Landing roll: closed to −19% by modelling the pilot, not the airplane.**
The POH figure (575 ft) is a demonstrated landing. The old test braked at
the tyre limit with all wheels down from the instant of touchdown, and
stopped in 254 ft (−56%). The airplane physics was not changed. The scenario
now models the technique: touchdown just above the flaps-30 stall (50 KCAS;
POH stall 48 KCAS), 1 s to lower the nose and reach the pedals, 1 s to build
pressure, and 70% sustained pedal (no anti-skid, so a pilot keeps margin
against a locked, flat-spotted tyre). These are human-factors estimates,
chosen before running the comparison and not solved for the POH number. The
result is 468 ft. The tyre-limited 254 ft stays as a physics bound, and a
test checks that softer braking lengthens the roll.

## Behavioural checks (tests)

- **Flight** (`tests/test_flight.cpp`, `tests/test_components.cpp`):
  - hands-off trim holds;
  - the short period is well damped and the phugoid is slow and lightly
    damped;
  - a 30° coordinated turn gives n = 1/cos φ;
  - the stall warning precedes the stall;
  - nothing caps speed (VNE exceedance is reported, not prevented);
  - runs are deterministic, and 120 Hz and 960 Hz runs converge;
  - wind changes the track, not the airspeed;
  - fuel burn moves the CG;
  - a frame rebase preserves the trajectory;
  - the ailerons show adverse yaw;
  - stall asymmetry and component loss work.
- **Ground:**
  - the airplane rests in static equilibrium;
  - the parking brake holds full power;
  - steering geometry and differential braking work;
  - hard steering at speed can overturn the airplane.
- **Systems and procedures:**
  - Scenario A, fuel exhaustion from real consumption, then a glide.
  - Scenario C, shutoff valve in cruise.
  - Magnetos and idle cut-off.
  - Battery master off in flight.
  - Quick Start from cold and dark, and its real failure modes (flat battery,
    valve shut).
  - Hot, high and humid take-offs.
- **Instruments:** <!-- validation:begin tests file=test_instruments.cpp -->40<!-- validation:end --> cases in `tests/test_instruments.cpp`. Examples:
  - IAS/CAS/TAS at altitude;
  - Kollsman setting;
  - hot-day altimeter error;
  - VSI lag;
  - AI turn error at most 3° in a standard-rate turn;
  - compass northerly turning error;
  - pitot, static, vacuum, alternator and total electrical failures;
  - VOR/LOC/GS geometry;
  - heading wrap;
  - frame-rate independence;
  - no repeated events;
  - airborne spawn seeding.

## Telemetry and golden traces

`TelemetryRecorder` and `InstrumentRecorder` (`avd/telemetry.hpp`) write CSV.
Run `avdemo flight csv` or `avdemo instruments csv`. Channels are listed in
[telemetry-channels.md](telemetry-channels.md). Four canonical flights of
the Tupi (take-off, stall and recovery, trimmed cruise, ILS approach to a
stop) are recorded as golden traces for the regression test
([golden-traces.md](golden-traces.md)).
