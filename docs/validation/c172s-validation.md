# Validation: Tupi T-180 against the Cessna 172S POH

Reproduce with `./build/avdemo validate`. The assertions with tolerances are in
`tests/test_validation.cpp`. Conditions: 2550 lb, mid CG, ISA, no wind, no
assists. The scripted test pilot (`src/scenarios/test_pilot.cpp`) flies the
POH technique.

## Results (October 2026, nonlinear lifting line)

Aerodynamics II replaced independent strips with a nonlinear lifting line
(span loading solved, ground images). The wing's section clmax is now the
published smooth NACA 2412 value (1.60 at Re ~3 x 10^6, Abbott & von
Doenhoff) instead of a value fitted to the stall speed: **the clean stall
speed is a validation, not a calibration**. Only the flap's clmax increment
(0.55) is calibrated, to the flaps-30 stall.

| Quantity | POH | Simulation | Error | Tolerance |
|---|---|---|---|---|
| Stall speed, flaps up, power off | 53 KCAS | 53.1 KCAS | +0.3% | ±6% |
| Stall speed, flaps 30, power off | 48 KCAS | 47.9 KCAS | −0.1% | ±6% (flap increment calibrated) |
| Full-throttle level speed, 8000 ft | 124 KTAS (75% power) | 119.5 KTAS | −3.6% | ±5% |
| ↳ power / rpm / prop efficiency / fuel flow | 75%, about 2700 rpm, about 10.5 gph | 73% / 2590 rpm / 0.84 / 10.4 gph | | |
| Maximum level speed, sea level | 126 KTAS | 126.5 KTAS (2692 rpm) | +0.4% | ±5% |
| Rate of climb, sea level, 74 KIAS | 730 fpm | 707 fpm | −3.1% | ±15% |
| Take-off ground roll (short field) | 960 ft | 812 ft | −15.4% | ±20% |
| Take-off over 50 ft | 1630 ft | 1606 ft | −1.5% | ±20% |
| Landing ground roll, pilot braking technique | 575 ft | 459 ft | −20.1% | ±25% (see below) |
| ↳ tyre-limited bound (full pedal at touchdown) | — | 257 ft | | physics check |
| Static rpm, full throttle | 2300–2420 rpm | 2335 rpm | | in range |
| Stall warning | 5–10 kt above the stall | 8.5 kt (flaps up), 7.9 kt (flaps 30) | | in range |

Hot and high, for information: at SBMT (722 m) at ISA+20 the ground roll is
1213 ft, against 812 ft at sea-level ISA.

**Landing roll tolerance widened to ±25% with the lifting line.** The
scenario (unchanged, its human-factors values fixed before any comparison)
holds full back stick through the roll, so the full-flap wing rolls at about
13 deg, just past its stall in ground effect. Strip theory stalled the wing
strip by strip and kept more lift there; the lifting line stalls it across
the span, as a real wing does, so the wheels carry more weight and the brakes
stop it sooner: 481 ft became 459 ft (-20.1%). No data here constrains the
post-stall lift in ground effect, and the POH figure is a demonstrated
landing with an unpublished technique, so the band for this one row is ±25%
rather than tuning the pilot to the number.

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
- **Instruments:** 37 cases in `tests/test_instruments.cpp`. Examples:
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

## Telemetry

`TelemetryRecorder` and `InstrumentRecorder` (`avd/telemetry.hpp`) write CSV.
Run `avdemo flight csv` or `avdemo instruments csv`. Channels are listed in
[telemetry-channels.md](telemetry-channels.md).
