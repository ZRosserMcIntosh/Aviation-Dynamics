# Validation: King Air 350 against published figures

The King Air 350 (`data/aircraft/king_air_350.json`) is the project's first
turboprop: two Pratt & Whitney Canada PT6A-60A free-turbine engines
(1,050 shp flat rated at 1,700 propeller rpm) driving Hartzell four-blade
105 in constant-speed, feathering and reversing propellers. The published
figures are the manufacturer's brochure and type-certificate values (the
`sources` of the data file); none comes from the flight manual's charts, so
the bands are wide and every row says what it is compared with. The rows are
asserted by `tests/test_turboprop.cpp`, which prints the values below (this
page is written from its output by hand: rerun `avd_tests "king air:"` and
`avd_tests "turboprop:"` after a change to the engine, propeller or data).

Conditions: ISA, no wind, the data file's load (12,560 lb: crew 170 kg,
cabin 300 kg, baggage 50 kg, 960 kg of fuel) unless the row says MTOW
(15,000 lb: the tanks at 900 l each, the cabin topped up); the free-air trim
solver for steady points; scripted pilots for the take-off.

## Engine and propeller

| Quantity | Published / expected | Simulation | Band | Result |
|---|---|---|---|---|
| Take-off power, sea level static ISA | 1,050 shp at 1,700 rpm (100 % torque, 3,244 ft lbf) | torque 99.8 %, Np 1,700 rpm | ±2 % torque, ±10 rpm | within |
| Take-off Ng, ITT | below 104 % Ng, 820 °C | 92.6 % Ng, ITT 759 °C | Ng 88–100 %, ITT 700–820 °C | within |
| Take-off fuel flow, SFC | SFC 0.548 lb/shp/h (575 lb/h) | 575 lb/h, SFC 0.548 | 520–640 lb/h | within |
| Flat rating | 1,050 shp held at ISA+20 at sea level and at 10,000 ft | 1,050 / 1,050 shp; 883 shp at 18,000 ft | flat to 10,000 ft | within |
| Low idle fuel flow | ~100 lb/h (PT6 class, estimated) | 100 lb/h at 62 % Ng | ±2 lb/h | within |
| Static thrust per engine at take-off power | 2.5–3.7 lbf/shp (class of a 105 in four-blade at 1,050 shp) | 15.5 / 15.2 kN (3,480 / 3,430 lbf; the left engine's in the right one's swirl), blade 23° | 11–17.5 kN | within |
| Cold start: starter on to stabilised idle | ~30 s, start ITT below 1,000 °C | 30 s, ITT peak 850 °C | 20–45 s, 650–1,000 °C | within |
| Governor: idle to take-off power at 180 KTAS | Np held (constant-speed) | 1,700–1,707 rpm | ±2 % | within |
| Governor: take-off to 20 % power | Np held | 1,684–1,700 rpm | ±2 % | within |
| Propeller lever to 1,500 rpm | 1,500 rpm | 1,501 rpm | ±8 rpm | within |
| Engine cut at 140 KTAS: windmilling drag area (governor at the zero-torque blade angle, Np 1,700) | — | 0.21 m² | ≥ 2 × feathered | within |
| The same, feathered | — | 0.07 m², Np 0 | — | info |
| Autofeather, engine failure at take-off power | feathers without the pilot | feathered 7.2 s after the failure | 2–15 s | within |
| Shut-down on the ground | the propeller feathers as the oil goes | feathered, Np < 50 rpm | — | within |

## Aircraft

| Quantity | Published | Simulation | Band | Result |
|---|---|---|---|---|
| Maximum cruise, FL240, mid weight, 1,500 rpm | ~303–312 KTAS (FL240–FL280 band, brochures) | 308 KTAS (12,530 lb), 745 shp/engine (80 % torque), ITT 820 °C, propeller efficiency 0.85 | 290–320 KTAS | within |
| Fuel flow at that cruise | ~320–360 lb/h per engine (class) | 397 lb/h per engine | 280–420 lb/h | within (high) |
| Rate of climb, sea level ISA, MTOW, 140 KIAS, take-off power | 2,731 fpm | 2,806 fpm | 2,300–3,200 fpm | within |
| 1-g stall, landing flap, MTOW (trimmed CLmax) | VSO ~81 KIAS | 80.4 KIAS (CLmax 2.21) | 75–88 KIAS | within |
| 1-g stall, clean, MTOW | ~99 KIAS class (the B200's 99 KIAS clean, scaled) | 98.1 KIAS (CLmax 1.48) | 92–110 KIAS | within |
| 1-g stall, approach flap, MTOW | between the two | 86.8 KIAS | below clean | within |
| Vmca (left engine feathered, right at take-off power, 5° bank, light) | 94 KIAS | ~100 KIAS (the lowest speed with a lateral balance inside the rudder's travel) | 84–104 KIAS | within (6 kt high) |
| All-engines take-off to 50 ft, MTOW, SL ISA, approach flap | inside the 3,300 ft take-off field length (which includes an engine failure at V1) | ground roll 1,565 ft, lift-off 102 KIAS, 2,193 ft to 50 ft | 1,600–3,000 ft | within |
| Ground beta and reverse, 90 kt, no brakes, 6 s | — | idle 79 kt, beta 49 kt, full reverse 39 kt (−0.44 g mean) | full reverse > 0.12 g | within |
| Trim across the envelope | — | converged from sea level 130–250 KIAS to 33,000 ft, approach and landing configurations | trim inside its travel, α −4° to 11° | within |

## Notes

- **Flat rating and the ITT limit.** The gas generator's power is a function
  of corrected speed at the compressor face (ram rise included); the torque
  limiter holds 100 % torque up to about 15,000 ft in the climb, above which
  the ITT limiter takes over. The thermodynamic rating (1,575 shp at 100 %
  corrected Ng, sea level) and the exponent of the ITT's temperature
  correction (0.15) are calibrated so that the FL240 maximum cruise lands in
  the published band; the cruise fuel flow then comes out ~15 % above the
  class figures (the corrected Willans line does not capture the better SFC
  of the cold intake at altitude).
- **Power lever.** The PT6A-60A has no FADEC: the pilot sets torque and
  watches the ITT. Here the lever's forward stop is the fuel control's
  torque and ITT limiters (the King Air 350's own torque limiter is not
  claimed), so a player at full power is at take-off torque low down and at
  the ITT limit high up, never over either.
- **Windmilling against feathered.** A free-turbine engine's windmilling
  propeller drives only the power turbine and gearbox: the governor holds Np
  at the zero-torque blade angle, where the blades' profile drag at the full
  tip speed is the drag (three times the feathered propeller's at 140 KTAS).
  A single-shaft turboprop's would be far larger; that is not modelled.
- **Vmca 100 against 94 KIAS.** The static lateral balance (side force,
  rolling and yawing moments) at 5° of bank is solved at each speed with the
  engines run to their state; the rudder runs out 6 kt above the published
  speed. The fin and rudder are estimated from three-views (no King Air
  tail data is published), the rudder boost is not modelled, and the live
  engine's static thrust is at the top of its band.
- **Stall.** The landing-flap CLmax is calibrated to the published VSO class
  (section clmax 1.42, flap clmax increment ratio 0.8); with landing flap the
  trimmed attitude in level flight is about 3° nose down (the King Air's
  flap-down attitude is nose low).
