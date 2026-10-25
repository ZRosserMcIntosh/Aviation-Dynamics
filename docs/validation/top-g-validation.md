# Validation: Top G against published Gulfstream G650ER figures

The Top G is a fictional twin jet that flies as a G650ER. The published
figures it is compared with are the manufacturer's specification-sheet
values and class-typical numbers; none comes from a flight manual, so every
row here is **representative, estimated**, with a band to match. The rows
are computed in `src/scenarios/validation.cpp` (`topG*`), asserted by
`tests/test_validation.cpp` and printed by `./build/avdemo validate`; the
tables below are regenerated from the test output by
`tools/update_validation_docs.py` and must not be edited by hand.

Conditions: the data file's mid weight (33.7 t: 24.0 t empty, crew and cabin
as loaded, 8.96 t of fuel) unless the row says otherwise; ISA; no wind; the
free-air trim solver for the steady points, scripted pilots for the landing.
The climb rows fly on the FADEC's maximum-climb rating and the cruise rows
on maximum cruise (`PilotInput::thrustMode`; the take-off rating is
time-limited after lift-off), as the published figures are.

## Results

<!-- validation:begin table aircraft=top_g -->
| Quantity | Published | Simulation | Error | Band | Result | Source |
|---|---|---|---|---|---|---|
| Time to climb, sea level to FL410, mid weight<br><small>33.7 t at brake release; 250/300 KIAS, Mach 0.80; maximum-climb rating; rate of climb at FL410 1472 fpm</small> | about 14 min (derived from about 22 min at MTOW) | 10.9 min | -22.0 % | ±25 % | within | representative, estimated: the MTOW figure (about 22 min) scaled by the specific excess thrust at 33.7 t (12-15 min, see the note) |
| Fuel to climb to FL410, mid weight<br><small>integrated from the trims</small> | — | 675 kg | — | — | info | — |
| Distance to climb to FL410, mid weight<br><small>zero wind</small> | — | 75.1 NM | — | — | info | — |
| Rate of climb at sea level, 250 KIAS, maximum-climb rating, mid weight | — | 7,292 fpm | — | — | info | — |
| Time to climb, sea level to FL410, maximum take-off weight<br><small>46.5 t at brake release; 250/300 KIAS, Mach 0.80; maximum-climb rating; rate of climb at FL410 595 fpm, at sea level 5083 fpm</small> | about 22 min | 18.1 min | -17.9 % | ±25 % | within | representative, estimated (G650ER class: FL410 as the initial cruise altitude in about 20-25 min from a departure near MTOW) |
| Fuel to climb to FL410, maximum take-off weight<br><small>integrated from the trims</small> | — | 1,068 kg | — | — | info | — |
| Distance to climb to FL410, maximum take-off weight<br><small>zero wind</small> | — | 126 NM | — | — | info | — |
| Thrust needed for Mach 0.90 at FL410, mid weight (fraction of the maximum-cruise rating)<br><small>33.7 t; the rating allows 18.1 kN per engine here; fuel flow 2117 kg/h, pitch 1.4 deg</small> | available (high-speed cruise Mach 0.90) | 93.9 % | — | 0–98 % | within | Gulfstream G650ER published high-speed cruise Mach 0.90 |
| Fuel flow at Mach 0.85, FL450, mid weight<br><small>83 % of the maximum-cruise rating (14.5 kN per engine)</small> | about 2,600-3,400 lb/h (class figures) | 1,456 kg/h | — | 998–1,724 kg/h | within | published long-range cruise burn of the class |
| Vref (1.23 Vs1g, landing flap) at 27.5 t<br><small>sea level ISA; trimmed CLmax, flaps 39; the square-root-of-weight law, not an independent published point</small> | about 103 KIAS (published 121 kt at MLW scaled by sqrt(W)) | 102 KIAS | — | 100–110 KIAS | within | derived: brokers' MLW Vref (about 121 kt at 37.9 t) x sqrt(27.5 / 37.9) = 103 KIAS; +-5 kt |
| Vref (1.23 Vs1g, landing flap) at the maximum landing weight, 37.9 t<br><small>sea level ISA; trimmed CLmax, flaps 39</small> | 110-125 KIAS (representative; published values 110-121 kt) | 121 KIAS | — | 110–125 KIAS | within | representative, estimated (G650ER class) |
| Landing distance from 50 ft, maximum landing weight (37.9 t)<br><small>Vref 121 KIAS (121 at 50 ft), touchdown 450 m past the 50 ft point at 545 fpm; full wheel braking (1 s to the pedals, 1 s ramp), ground spoilers, no reverse credit, dry runway</small> | 3,000 ft (914 m), SL, ISA, MLW | 834 m | -8.8 % | ±15 % | within | Gulfstream G650ER specification sheet (landing distance, SL, ISA, MLW); brokers quote 2,450-3,000 ft |
<!-- validation:end -->

<!-- validation:begin summary aircraft=top_g -->
7 compared rows, 0 outside their band.
<!-- validation:end -->

## Sources and how each row is measured

| Row | Published figure | Source | Method here |
|---|---|---|---|
| Time to climb to FL410, MTOW | about 20–25 min to the initial cruise altitude (FL410) | Gulfstream's G650ER figures quote FL410 as the initial cruise altitude; the time is a class-typical value for a heavy departure, not a published table, hence the ±25 % band around 22 min | Trims every 1,000 ft at 250 KIAS to 10,000 ft, 300 KIAS to the crossover, then Mach 0.80, maximum-climb rating, from 46.5 t (the tanks full; MTOW is 47.0 t), the mass falling with the fuel; time, fuel and distance integrated |
| Time to climb to FL410, 33.7 t | about 14 min (12–15), derived | The MTOW figure scaled by the specific excess thrust (T − D)/W, which sets the rate of climb: the drag falls less than the weight (the parasite part not at all), so at FL250 / M0.72 (T − D)/W is about 0.063 at 47 t and 0.098 at 33.7 t (0.64× the time), and near FL410, where the heavy aeroplane crawls, the ratio is lower still; ±25 % | As above from 33.7 t |
| Mach 0.90 at FL410 | high-speed cruise Mach 0.90 (MMO 0.925) | Gulfstream G650ER specification sheet | Trim at FL410, Mach 0.90, 33.7 t: the thrust needed as a fraction of what the maximum-cruise rating allows there (must be below 98 %) |
| Fuel flow, Mach 0.85 at FL450 | about 2,600–3,400 lb/h long-range cruise burn | class figures for the G650 (Rolls-Royce BR725) | Trim at FL450, Mach 0.85, 33.7 t (the band is wider: 2,200–3,800 lb/h) |
| Vref at 27.5 t | about 103 KIAS (100–110), derived | The published MLW Vref (about 121 kt at 37.9 t) scaled by the square root of the weight: 121 × √(27.5/37.9) = 103. The 115–125 KIAS "at 25–30 t" band the row first carried was inconsistent with the MLW figure it came with and is withdrawn (see below) | 1.23 Vs1g with the landing flap (39°) from the trimmed CLmax the cockpit V-speeds use, sea level ISA |
| Vref at MLW (37.9 t) | 110–121 kt | brokers' and reference-site values (Vref about 110 kt; about 121 kt at MLW) | as above at the maximum landing weight |
| Landing distance | 3,000 ft (914 m), SL, ISA, MLW | Gulfstream G650ER specification sheet; brokers quote 2,450–3,000 ft for the same conditions | 14 CFR 25.125 style: from 50 ft over the threshold at Vref on a 3° path to a stop, maximum wheel braking (1 s to the pedals, 1 s ramp), ground spoilers, no reverse-thrust credit, dry runway, MLW. The instrument-only approach pilot flies the path and the flare |

## Thrust ratings and the climb (October 2026)

Nothing below was tuned to a row; the model is reported as it is.

**FADEC ratings.** The engine had one limit, the take-off thrust of the
Mattingly lapse law, at any time and in any phase, so the climb to FL410
was flown on take-off thrust throughout. `TurbofanDef` now carries the
ratings (`ratings` in the JSON, provenance "estimated": maximum continuous
0.92, maximum climb 0.88, maximum cruise 0.84 of the sea-level static
take-off thrust, take-off limited to 5 min after lift-off, 10 min with an
engine out; typical of the class) and the fuel control caps the commanded
N1 at the active rating's limit, the lever still commanding N1 below it.
`ThrustMode::Auto` selects take-off on the ground and for the rating's
time after lift-off (re-armed by a go-around: levers to the stop with the
gear or flaps out), then maximum continuous with an engine out, maximum
climb while climbing and maximum cruise after a minute without a climb;
`PilotInput::thrustMode` forces a rating. The ratings are modelled as
turbine-temperature limits: the flat-rated take-off thrust is held up to
the throttle ratio by raising T_t4, and each lower rating holds a fixed
fraction of the temperature-limited thrust (Mattingly's temperature branch
continued below TR). They therefore give the quoted fractions at sea level
and converge on the flat-rated line in the cold air at altitude, where the
take-off rating itself sits far below its temperature limit: at 250 KIAS
and 10,000 ft the climb rating is 0.91 of take-off, at FL410 the cruise
rating allows 97 % of the lapse law. This keeps the lapse coefficient's
earlier calibration (Mach 0.85 available to FL510 at mid weight, which is
a maximum-cruise condition) meaningful, and it is why the climb time
changed less than a constant 0.88 would suggest.

**Time to FL410: 10.9 min at 33.7 t, 18.0 min at 46.5 t.** The class
figure (about 22 min) is for a heavy departure; at 33.7 t the aeroplane
carries 40 % less with the same engines and a specific excess thrust 1.5–3
times higher through the climb, so the expectation there is derived from
the MTOW figure (about 14 min, 12–15: see the sources table) rather than
copied. Both rows are inside ±25 % (−22 % and −18 %) and both are on the
fast side: 7,300 fpm at sea level and 1,470 fpm still at FL410 at 33.7 t,
5,100 and 600 fpm at 46.5 t. With only class figures to compare against
this is as far as the rows can go; a published climb table would settle
whether the thrust at altitude (which the cruise rows below constrain from
the other side) or the drag at climb speeds is the difference.

**Mach 0.90 at FL410 needs 92 % of the maximum-cruise rating at 33.7 t
(18.1 kN per engine).** It is available, as published, but the margin is
thin: at the weights a G650ER typically cruises at (35–40 t) the same
model would not hold Mach 0.90 at FL410 on the cruise rating. Either the
drag rise toward MMO or the thrust at altitude is marginal; the Mach-drag
model is being revised separately and this row will show the effect.

**Vref at 27.5 t: the band was wrong, not the model.** The 115–125 KIAS
"at 25–30 t" band was inconsistent with the published MLW Vref (about 121
kt at 37.9 t) it was given with: Vref scales with the square root of the
weight, 121 × √(27.5/37.9) = 103 KIAS, which is what the model gives at
both weights (121.6 KIAS at MLW). The row now targets the published MLW
figure scaled to 27.5 t (100–110 KIAS); the MLW row checks the published
figure itself.

**Landing distance: 791 m (−13 %).** The table's previous 881 m was
recorded before the tyre model (friction ellipse, torque brakes, anti-skid)
was merged; the engine work did not move this row.

**Landing distance: 834 m (−9 %), braking as a pilot brakes.** The row had
drifted to 773 m (−15.4 %, just outside its band). That last step came when
the autoland flare moved to the radio altimeter's rate, which shortened the
air distance by 6 m. The cause underneath was the method, which put the
brakes at full in the first instant on the ground. They now come on as in
the 172's landing row: 1 s to the pedals, then a 1 s ramp to full. The
reference and band are unchanged.

## Behavioural checks

`tests/test_jet.cpp` (19 cases): spool lag and thrust lapse, cruise at
Mach 0.85 and FL450 with its fuel burn, approach speeds and elevator
authority on the approach, take-off at a mid weight and gear retraction,
engine start, speedbrakes and the drag rise past the critical Mach, the
double flame-out and relight, the stick pusher against the T-tail deep
stall, tiller and pedal steering, the straight-and-level hold at FL410, the
load sheet, wet-runway stops and hydroplaning, automatic ground spoilers and
thrust reversers, spoiler panels and gear drag. The blind-landing tests land
it on the ILS and on GPS approaches in calm air and a 6 kt crosswind
([blind-landings.md](blind-landings.md)).

## Golden traces

`topg_takeoff_sbsp`, `topg_cruise_fl410` and `topg_ils_sbgr` record a
take-off from Congonhas, three minutes of trimmed cruise at FL410 and an ILS
approach to Guarulhos at 10 Hz for the regression test
([golden-traces.md](golden-traces.md)).
