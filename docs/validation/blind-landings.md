# Validation: blind landings (zero visibility)

The question: in fog with nothing visible outside, can each aircraft be
landed on the ILS or on a GPS (RNAV) approach using its instruments alone?
This is what CAT IIIb/c operations do, with an autoland or a head-up guidance
system.

**Answer: yes.** All seven aircraft, flown by an instrument-only pilot,
landed and stopped on the centreline in zero visibility, on both the ILS and
RNAV approaches. That held in calm air and in a light (6 kt) crosswind with
the turbulence it makes over the city. Every touchdown was inside the CAT III
limits below. Above that wind the larger jets begin to land firm (see
*Limits*).

## The pilot: `scenarios::ApproachPilot`

`src/scenarios/approach_pilot.{hpp,cpp}` flies the approach as a coupled
autopilot and autoland would. It sees only the cockpit (`CockpitState`):

- **Lateral and vertical guidance:**
  - the ILS: NAV 1's localizer and glideslope needles;
  - or the GPS: cross-track and the LPV-style vertical deviation.
- **Attitude, heading, airspeed and altimeter.** The vertical speed comes
  from the altimeter's rate, as an air-data computer gives it.
- **Height for the flare:**
  - the radio altimeter on the jets;
  - the barometric altimeter against the field elevation on the 172, which
    has no radio altimeter.
- **Weight on wheels.**

It is given nothing from the truth state: not the position, the wind or the
runway's direction beyond the charted course.

- **Lateral.** The needle deflections are turned into metres from the beam
  geometry. The distance comes from the height on the 3° path; past the
  threshold, it is the distance flown since. The pilot steers a track back to
  the centreline. The crab into the wind is a running estimate of the drift:
  the track (from the rate of change of the deviation) less the heading. A
  heading-rate term damps the loop.
  - Below 100 ft the rudder helps hold the heading.
  - Below 10 ft it decrabs: it puts the nose on the runway heading and puts a
    wing down into the wind.
- **Vertical.** The pilot flies a vertical speed: the 3° path's sink at the
  true airspeed, corrected by the deviation. The throttle holds the approach
  speed with an acceleration term, as an autothrottle does.
- **Flare and retard.** This is the autoland law, flown on the radio
  altimeter:
  - The flare starts about 5 s from the ground.
  - The sink rate is held in proportion to the height, plus a small
    touchdown sink.
  - While the sink is still too high, the pitch is never lowered more than
    1° below its highest flare attitude yet.
  - The thrust retards gradually to idle (over 4 s) below 20 ft, or as soon
    as the flare has arrested the descent.
- **Rollout.** Idle and brakes; the rudder and nose wheel steer the
  centreline on the localizer or GPS.

Approach speed is 1.23 VS (landing flap, actual weight) + 5 kt for the
jets; 65 KIAS for the 172.

## The test: `tests/test_blind_landing.cpp`

Each aircraft starts trimmed in the landing configuration, level at the
intermediate fix (10 NM out for the jets, 6 NM for the 172). It is then
flown to a full stop. The pass criteria:

| Criterion | Limit | Reference |
|---|---|---|
| Touchdown point | 60–914 m past the threshold | the touchdown zone (3,000 ft) |
| Touchdown lateral | within 8.2 m (27 ft) of the centreline | AC 120-28D, CAT III touchdown |
| Sink rate at touchdown | under 600 fpm | 10 ft/s, the landing-gear design sink rate |
| Stop | on the runway, main gear 3 m inside its edge | — |

## Results (October 2026)

ILS 10L at Guarulhos, calm, zero visibility:

| Aircraft | Touchdown | Off centreline | Sink | Stopped at |
|---|---|---|---|---|
| Tupi T-180 (172) | 426 m | 0.9 m | 122 fpm | 729 m |
| Top G | 520 m | 0.2 m | 432 fpm | 1,089 m |
| Phenom 300E | 469 m | 1.0 m | 328 fpm | 866 m |
| Citation CJ4 | 518 m | 0.7 m | 278 fpm | 953 m |
| Learjet 45 | 468 m | 0.6 m | 348 fpm | 868 m |
| CRJ700 | 647 m | 0.8 m | 441 fpm | 1,353 m |
| 737-800 | 532 m | 0.1 m | 162 fpm | 1,433 m |

ILS 35L at Congonhas (1,940 m), 6 kt from 060 (almost straight across),
with the mechanical turbulence the wind makes over the city:

| Aircraft | Touchdown | Off centreline | Sink | Stopped at |
|---|---|---|---|---|
| Tupi T-180 | 422 m | 0.5 m | 132 fpm | 730 m |
| Top G | 489 m | 2.7 m | 397 fpm | 1,040 m |
| CRJ700 | 694 m | 1.1 m | 487 fpm | 1,294 m |
| 737-800 | 484 m | 1.6 m | 292 fpm | 1,339 m |

RNAV (GPS) approaches, the GPS vertical path to the threshold:

| Aircraft | Approach | Touchdown | Off centreline | Sink | Stopped at |
|---|---|---|---|---|---|
| Tupi T-180 | São José dos Campos 16, calm | 442 m | 0.8 m | 118 fpm | 731 m |
| Top G | São José dos Campos 16, calm | 461 m | 0.4 m | 437 fpm | 1,089 m |
| CRJ700 | São José dos Campos 16, calm | 589 m | 1.8 m | 441 fpm | 1,285 m |
| 737-800 | Guarulhos 28L, 6 kt from 200 | 444 m | 0.7 m | 405 fpm | 1,281 m |

## What it took (found and fixed)

- **The localizer's beam width past the threshold.**
  - The distance to the localizer antenna was estimated from the height. That
    estimate stopped at the threshold.
  - In the flare, the deviation in metres was then overestimated by up to
    60 %.
  - Fix: past the threshold, the distance flown is integrated instead.
- **A crab from an integral.**
  - The crab into the wind was the integral of the deviation, which had to
    wind up to a 10° crab.
  - It held the airplane 30 m downwind for most of the approach.
  - It is now a drift estimate: track less heading.
- **A 25 s weave in any wind.**
  - The lateral-speed damping term fed the heading back on itself and
    stiffened the heading loop.
  - Calm air never excited it; the city's mechanical turbulence did.
  - A heading-rate term (scaled with speed, so the 172 does not limit-cycle)
    damps it.
- **Decrab too early.** Aligning with the runway at the flare (5–9 s up for a
  jet) let the crosswind carry a 737 30 m sideways. It now aligns below 10 ft.
- **A CRJ floating 800 m.**
  - The pitch floor in the flare levelled the CRJ off at 20 ft.
  - The thrust stayed up because the retard waited for 20 ft.
  - The floor now relaxes once the sink is arrested, and the retard also
    starts at level-off.
- **A soft 737 flare in a gust.** The flare's pitch authority was raised (to
  three times the approach loop's).

## Limits

- **Stronger crosswinds and turbulence are not passed yet.**
  - In 8 kt across Congonhas, the CRJ700 touched down at 748 fpm.
  - In 10–12 kt with added turbulence (σ 0.5–1 m/s on top of the city's
    mechanical turbulence), the 737-800 and CRJ700 landed long or firm in
    some runs.
  - The lateral tracking held within a few metres in those runs.
  - Real CAT III autoland is certified to 15–25 kt crosswind. This pilot's
    flare would need gust compensation (a speed additive, a flare on inertial
    vertical speed) to get there.
- **Only fixed approaches are tested.** These are single deterministic
  runs per case, not the Monte Carlo dispersion analysis AC 120-28D asks
  for.
- **Ideal signals.** The radio altimeter reads the true height above the
  ground beneath. The ILS has no beam bends or multipath. The GPS has no
  integrity or accuracy model beyond its CDI scaling.
- **No pilot in the loop.** In the browser, flying blind by hand is possible
  with the same instruments, but nothing measures it.
