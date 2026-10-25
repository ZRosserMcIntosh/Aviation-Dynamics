# Validation: blind landings (zero visibility)

The question: in fog with nothing visible outside, can each aircraft be
landed on the ILS or on a GPS (RNAV) approach using its instruments alone?
This is what CAT IIIb/c operations do, with an autoland or a head-up guidance
system.

**Answer: yes.** All seven aircraft, flown by an instrument-only pilot,
landed and stopped on the centreline in zero visibility, on both the ILS and
RNAV approaches. That held in calm air and in a light (6 kt) crosswind with
the turbulence it makes over the city; the CRJ700 and 737-800 also in 10 kt.
Every touchdown was inside the CAT III limits below. From about 12 kt the
jets start to fall outside them (see *Limits*).

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
  - Its sink rate is the radio altimeter's rate of change (the 172's, the
    altimeter's).
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
| The pilot's seat | gear intact, spinal DRI under 5 | DRI 18 is the ejection-seat design limit; the crash-test dummy of `avd/occupant.hpp` |

## Results (October 2026)

ILS 10L at Guarulhos, calm, zero visibility:

| Aircraft | Touchdown | Off centreline | Sink | Stopped at | Seat | DRI |
|---|---|---|---|---|---|---|
| Tupi T-180 (172) | 427 m | 1.2 m | 148 fpm | 668 m | 1.9 g | 1.9 |
| Top G | 498 m | 0.2 m | 399 fpm | 904 m | 1.8 g | 2.2 |
| Phenom 300E | 476 m | 0.4 m | 319 fpm | 779 m | 2.0 g | 2.2 |
| Citation CJ4 | 503 m | 0.8 m | 288 fpm | 816 m | 2.2 g | 2.4 |
| Learjet 45 | 467 m | 1.2 m | 346 fpm | 762 m | 1.9 g | 2.2 |
| CRJ700 | 579 m | 0.9 m | 347 fpm | 1,181 m | 2.0 g | 2.3 |
| 737-800 | 527 m | 0.1 m | 391 fpm | 1,178 m | 2.5 g | 2.9 |

ILS 35L at Congonhas (1,940 m), the wind from 060 (almost straight across),
with the mechanical turbulence it makes over the city:

| Aircraft | Wind | Touchdown | Off centreline | Sink | Stopped at | Seat | DRI |
|---|---|---|---|---|---|---|---|
| Tupi T-180 | 6 kt | 425 m | 0.7 m | 121 fpm | 652 m | 1.7 g | 1.8 |
| Top G | 6 kt | 476 m | 2.8 m | 417 fpm | 882 m | 2.1 g | 2.2 |
| CRJ700 | 6 kt | 585 m | 2.9 m | 387 fpm | 1,128 m | 1.9 g | 2.3 |
| 737-800 | 6 kt | 478 m | 2.5 m | 360 fpm | 1,121 m | 2.5 g | 2.7 |
| CRJ700 | 10 kt | 524 m | 2.2 m | 447 fpm | 1,050 m | 2.1 g | 2.2 |
| 737-800 | 10 kt | 496 m | 2.9 m | 363 fpm | 1,147 m | 2.2 g | 2.5 |

RNAV (GPS) approaches, the GPS vertical path to the threshold:

| Aircraft | Approach | Touchdown | Off centreline | Sink | Stopped at | Seat | DRI |
|---|---|---|---|---|---|---|---|
| Tupi T-180 | São José dos Campos 16, calm | 416 m | 1.1 m | 144 fpm | 652 m | 1.9 g | 1.9 |
| Top G | São José dos Campos 16, calm | 436 m | 0.4 m | 409 fpm | 839 m | 1.8 g | 2.2 |
| CRJ700 | São José dos Campos 16, calm | 519 m | 1.3 m | 354 fpm | 1,117 m | 2.0 g | 2.3 |
| 737-800 | Guarulhos 28L, 6 kt from 200 | 421 m | 0.2 m | 447 fpm | 1,011 m | 2.2 g | 2.6 |

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
- **A flare on the wrong vertical speed.**
  - The flare read its sink rate from the barometric altimeter.
  - The static source's error moves with the angle of attack, so as the
    flare raised the nose it read the CRJ sinking up to 25 % faster than it
    was. The law kept pulling, ballooned the CRJ to a climb at 20 ft,
    floated it at idle and dropped it in at 677 fpm.
  - The flare now uses the radio altimeter's rate, as an autoland does:
    387 fpm, and 8–10 kt crosswinds pass.
- **The nose let fall.**
  - On the rollout the pilot pushed the nose to its ground attitude at
    once; with the mains' spin-up and the ground spoilers pitching it down
    as well, the nose came down at 20°/s.
  - The crash-test dummy in the pilot's seat showed it: 3.9 g at a
    Learjet 45's cockpit (DRI 4.7) with 1.1 g at the CG.
  - The nose is now flown down at 2°/s, back pressure against its rate:
    1.9 g, DRI 2.2. What is left is the main gear's touchdown (the CG at
    2.2–2.8 g in a jet's 300–450 fpm arrival).

## The autoland in the browser

The same pilot is the flight test bed's autoland (`avd_autoland`). It flies the
GPS route to the destination from wherever it is engaged: it joins the final
if the 3° path can be flown from there, else it flies the whole procedure. It
configures the airplane itself (gear, flaps by their limit speeds). From a
cruise start 8 km out and well above the path, every aircraft landed and
stopped on Congonhas 35L and Guarulhos 10L: touchdowns 400–522 m past the
threshold, within 2 m of the centreline (`web/smoke_test.mjs` checks the Top G).

## Limits

- **Stronger crosswinds and turbulence are not passed yet.**
  - In 12 kt across Congonhas, the CRJ700 touched down 9.2 m off the
    centreline (the limit is 8.2 m) and the Top G at 581 fpm.
  - Added turbulence (σ 0.5–1 m/s on top of the city's mechanical
    turbulence) is not tested since the flare change.
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
