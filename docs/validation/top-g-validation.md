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

## Results

<!-- validation:begin table aircraft=top_g -->
| Quantity | Published | Simulation | Error | Band | Result | Source |
|---|---|---|---|---|---|---|
| Time to climb, sea level to FL410, mid weight<br><small>33.7 t at brake release; 250/300 KIAS, Mach 0.80; take-off thrust; rate of climb at FL410 1472 fpm</small> | about 22 min | 10.3 min | -53.1 % | ±25 % | **OUTSIDE** | representative, estimated (G650ER class: FL410 as the initial cruise altitude in about 20-25 min) |
| Fuel to climb to FL410<br><small>integrated from the trims</small> | — | 656 kg | — | — | info | — |
| Distance to climb to FL410<br><small>zero wind</small> | — | 71.5 NM | — | — | info | — |
| Rate of climb at sea level, 250 KIAS, take-off thrust | — | 9,222 fpm | — | — | info | — |
| Thrust needed for Mach 0.90 at FL410, mid weight (fraction of maximum)<br><small>33.7 t; fuel flow 2068 kg/h, pitch 1.3 deg</small> | available (high-speed cruise Mach 0.90) | 93.3 % | — | 0–98 % | within | Gulfstream G650ER published high-speed cruise Mach 0.90 |
| Fuel flow at Mach 0.85, FL450, mid weight<br><small>throttle 88 %</small> | about 2,600-3,400 lb/h (class figures) | 1,447 kg/h | — | 998–1,724 kg/h | within | published long-range cruise burn of the class |
| Vref (1.23 Vs1g, landing flap) at 27.5 t<br><small>sea level ISA; trimmed CLmax, flaps 39</small> | 115-125 KIAS at 25-30 t (representative) | 103 KIAS | — | 115–125 KIAS | **OUTSIDE** | representative, estimated (G650ER class) |
| Vref (1.23 Vs1g, landing flap) at the maximum landing weight, 37.9 t<br><small>sea level ISA; trimmed CLmax, flaps 39</small> | 110-125 KIAS (representative; published values 110-121 kt) | 122 KIAS | — | 110–125 KIAS | within | representative, estimated (G650ER class) |
| Landing distance from 50 ft, maximum landing weight (37.9 t)<br><small>Vref 122 KIAS (122 at 50 ft), touchdown 463 m past the 50 ft point at 380 fpm; full wheel braking from touchdown, ground spoilers, no reverse credit, dry runway</small> | 3,000 ft (914 m), SL, ISA, MLW | 881 m | -3.6 % | ±15 % | within | Gulfstream G650ER specification sheet (landing distance, SL, ISA, MLW); brokers quote 2,450-3,000 ft |
<!-- validation:end -->

<!-- validation:begin summary aircraft=top_g -->
6 compared rows, 2 outside their band (top_g.climb.fl410.time, top_g.vref.27500kg).
<!-- validation:end -->

## Sources and how each row is measured

| Row | Published figure | Source | Method here |
|---|---|---|---|
| Time to climb to FL410 | about 20–25 min to the initial cruise altitude (FL410) | Gulfstream's G650ER figures quote FL410 as the initial cruise altitude; the time is a class-typical value, not a published table, hence the ±25 % band around 22 min | Trims every 1,000 ft at 250 KIAS to 10,000 ft, 300 KIAS to the crossover, then Mach 0.80, take-off thrust, the mass falling with the fuel; time, fuel and distance integrated |
| Mach 0.90 at FL410 | high-speed cruise Mach 0.90 (MMO 0.925) | Gulfstream G650ER specification sheet | Trim at FL410, Mach 0.90, 33.7 t: the thrust needed as a fraction of the maximum (must be below 98 %) |
| Fuel flow, Mach 0.85 at FL450 | about 2,600–3,400 lb/h long-range cruise burn | class figures for the G650 (Rolls-Royce BR725) | Trim at FL450, Mach 0.85, 33.7 t (the band is wider: 2,200–3,800 lb/h) |
| Vref at 27.5 t | 115–125 KIAS at 25–30 t | representative figure given with the task; see the note below | 1.23 Vs1g with the landing flap (39°) from the trimmed CLmax the cockpit V-speeds use, sea level ISA |
| Vref at MLW (37.9 t) | 110–121 kt | brokers' and reference-site values (Vref about 110 kt; about 121 kt at MLW) | as above at the maximum landing weight |
| Landing distance | 3,000 ft (914 m), SL, ISA, MLW | Gulfstream G650ER specification sheet; brokers quote 2,450–3,000 ft for the same conditions | 14 CFR 25.125 style: from 50 ft over the threshold at Vref on a 3° path to a stop, maximum wheel braking from touchdown, ground spoilers, no reverse-thrust credit, dry runway, MLW. The instrument-only approach pilot flies the path and the flare |

## Rows outside their band (October 2026) and what they point to

Nothing below was tuned; the model is reported as it is.

**Time to climb to FL410: 10.3 min against a representative 22 min
(−53 %).** The integrated climb at 33.7 t reaches FL410 in half the typical
time, with 9,200 fpm at sea level and 1,470 fpm still available at FL410. Two
things add up: 33.7 t is light (the typical figure is quoted near the
maximum take-off weight of 47 t, where the same thrust lifts 40 % more), and
the turbofan's thrust lapse (Mattingly) is calibrated to the published
cruise point, not to a climb schedule, so the thrust in the climb between
is not constrained by any row here. A time to climb at the maximum take-off
weight, and a published climb table, would settle which. The row keeps its
weight and its band so the number is visible.

**Vref at 27.5 t: 103 KIAS against the 115–125 KIAS band.** The same
model gives 121.6 KIAS at the maximum landing weight (37.9 t), which is
the published MLW figure (about 121 kt), and the stall speed scales with
the square root of the weight: 121.6 × √(27.5 / 37.9) = 103.6 KIAS. The
115–125 KIAS band at 25–30 t is therefore not consistent with the MLW
figure it accompanies; either the band belongs to a heavier weight, or the
published Vref is for a lighter definition of the reference stall speed.
The row is left failing for the decision to be made on the band, not on the
model.

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
