# Validation: Tupi T-180 against the Cessna 172S POH

Reproduce with `./build/avdemo validate`. The assertions with tolerances are in
`tests/test_validation.cpp`. Conditions: 2550 lb, mid CG, ISA, no wind, no
assists. The scripted test pilot (`src/scenarios/test_pilot.cpp`) flies the
POH technique.

## Results (October 2026)

| Quantity | POH | Simulation | Error | Tolerance |
|---|---|---|---|---|
| Stall speed, flaps up, power off | 53 KCAS | 53.9 KCAS | +1.6% | ±6% |
| Stall speed, flaps 30, power off | 48 KCAS | 49.2 KCAS | +2.6% | ±6% |
| Full-throttle level speed, 8000 ft | 124 KTAS (75% power) | 119.8 KTAS | −3.4% | ±5% |
| ↳ power / rpm / prop efficiency / fuel flow | 75%, about 2700 rpm, about 10.5 gph | 71% / 2491 rpm / 0.87 / 10.0 gph | | |
| Maximum level speed, sea level | 126 KTAS | 127.0 KTAS | +0.8% | ±5% |
| Rate of climb, sea level, 74 KIAS | 730 fpm | 706 fpm | −3.2% | ±15% |
| Take-off ground roll (short field) | 960 ft | 769 ft | **−19.9%** | ±20% (known discrepancy) |
| Take-off over 50 ft | 1630 ft | 1622 ft | −0.5% | ±20% |
| Landing ground roll, maximum braking | 575 ft | 262 ft | **−54%** | reported only (known discrepancy) |
| Static rpm, full throttle | 2300–2420 rpm | 2378 rpm | | in range |
| Stall warning | 5–10 kt above the stall | 8.4 kt (flaps up), 7.6 kt (flaps 30) | | in range |

Hot and high, for information: at SBMT (722 m) at ISA+20 the ground roll is
1213 ft, against 769 ft at sea-level ISA.

## Known discrepancies

**Take-off ground roll is about 20% short**, while the distance over 50 ft is
within 1%. The roll is short even when rotation is moved to the POH lift-off
speed of 51 KIAS. The likely cause is optimistic static and low-speed thrust:
the blade-element propeller tables do not include swirl losses, and those
losses matter most at low advance ratio. Static rpm is in the POH range, so
the error is in thrust per rpm rather than in power.

Following the project rule, this was **not** tuned away. Reducing thrust
would break climb and cruise, which currently match. The planned fix is to add
swirl (tangential induction) to the BEM tables and re-validate.

**Landing roll with maximum braking is about 54% short.** The POH figure
reflects demonstrated, technique-limited braking: the airplane is not on the
edge of tyre friction for the whole roll. The simulation brakes at the tyre
limit (μ about 0.65 on dry asphalt) from touchdown. A faithful comparison
needs either a pilot braking model or the POH's own technique; both are
planned with the landing-performance work.

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
- **Instruments:** 34 cases in `tests/test_instruments.cpp`. Examples:
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
