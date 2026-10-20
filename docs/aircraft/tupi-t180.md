# Tupi T-180

Data: `data/aircraft/tupi_t180.json` · Instrument panel: [tupi-t180-panel.md](tupi-t180-panel.md)
· Validation: [../validation/c172s-validation.md](../validation/c172s-validation.md)

The Tupi T-180 is a fictional four-seat high-wing trainer from the fictional
"Aeronaves Paulistanas". Its **simulation reference is the Cessna 172S
Skyhawk SP**: a Lycoming IO-360-L2A rated 180 bhp at 2700 rpm, with a
fixed-pitch McCauley propeller, in the normal category at 2550 lb. The game
uses only public specifications and published performance from the reference.
The name, branding and artwork are original.

## Sources

| Id | Source |
|---|---|
| POH | Cessna 172S Pilot's Operating Handbook / AFM, Revision 5 (2004): sections 1, 2, 5, 6, 7 |
| TCDS | FAA Type Certificate Data Sheet 3A12 |
| ABBOTT | Abbott & von Doenhoff, *Theory of Wing Sections* (1959): NACA 2412, NACA 0012 |
| DATCOM | USAF Stability and Control DATCOM (1978) |
| ROSKAM | Roskam, *Airplane Design* Parts V and VI |
| EST | Aviation Dynamics engineering estimate (explained in the JSON `note`) |

POH values were taken from public knowledge of the handbook, not from a
primary copy in this repository. Each one is marked so that it can be checked
against the POH revision of a specific airplane.

## Provenance summary

| Group | Kind | Notes |
|---|---|---|
| Empty, maximum and landing mass; station arms; CG limits | published | POH section 6 |
| Empty CG height, inertia | estimated | Inertia was chosen so the gross-weight totals land near the commonly quoted C172 values (Ixx ≈ 1285, Iyy ≈ 1825, Izz ≈ 2667 kg m²) |
| Wing area, span, MAC | published | 174 ft², 36 ft 1 in |
| Wing planform split, twist, dihedral | estimated | Two panels matching area and span; +1.5° root / −1.5° tip; 1.73° dihedral |
| Wing section | calibrated | NACA 2412 slope and α₀ (Abbott). clmax 1.48 and span efficiency 0.86 calibrated to the POH stall speeds |
| Control travel | published | Aileron 20° up / 15° down, elevator 28/23, rudder ±17.7°, flaps 0–30° (TCDS) |
| Horizontal tail | estimated / calibrated | Span from the POH. Incidence −1° and slipstream exposure 0.6 calibrated so the tail stays down on the take-off roll |
| Vertical tail | estimated | — |
| Drag build-up | calibrated | Roskam component areas scaled to POH cruise (124 KTAS at 75%, 8000 ft) and maximum speed (126 KTAS, sea level) |
| Engine rating, displacement | published | 180 bhp at 2700 rpm, 361 in³ |
| Engine friction, efficiency, oil pump | calibrated / estimated | Mechanical fmep 130 kPa at 2,700 rpm so that friction plus the full-throttle pumping loss is the 13.2 % of rated power Lycoming's altitude curve (Gagg–Farrar) implies; volumetric efficiency 0.89 from the published take-off fuel flow, which with the rated power fixes the indicated efficiency (0.33); oil pump estimated |
| Engine induction (compression ratio, throttle and idle orifices, VE with speed) | published / calibrated | Compression ratio 8.5 published. Idle orifice sized for 0.29 × ambient at 650 rpm (ground idle 704 rpm at 8.3 inHg: the idle stop); throttle bore for a 3 % drop at rated rpm; port-flow loss 0.08 calibrated to the rise of Lycoming's full-throttle torque below rated rpm |
| Propeller | published / calibrated | 76 in diameter and 60 in geometric pitch published (pitch_m 1.524). Chord distribution calibrated to the POH static rpm; blade thickness estimated (drives tip compressibility) |
| Fuel | published | Two 28 gal tanks, 26.5 gal usable each, arm 48 in, LEFT/BOTH/RIGHT selector and shutoff |
| Fuel line volume | estimated | Sets the run-on time after a tank runs dry |
| Battery, alternator | published | 24 V 12.75 Ah; 28 V 60 A |
| Electrical load currents | estimated | Switches and breakers follow the POH electrical description |
| Gear | estimated | Track and wheelbase from the POH; stiffness for about 6 cm static deflection; nose steering ±10°; rolling resistance 0.04 (dry asphalt, textbook range 0.03–0.05) |
| Brake friction coefficient | calibrated | 0.65, so the brakes hold full static power as the POH run-up requires |
| Static-port position error | calibrated | Fitted to the POH airspeed calibration table, flaps up |
| Rigging (rudder tab, aileron) | calibrated | Zero yaw and roll moment at 105 KIAS, 1500 m, as a ground-adjustable tab would be set |
| Limits | published | VNE 163, VNO 129, VA 105, VFE 110 (flaps 10) / 85 (flaps above 10) KIAS; +3.8 / −1.52 g |

The JSON has one `provenance` entry per field or field group, with its
source and a note. There are no `gameplay` values in this aircraft.

## Calibration record

All calibration used the scenarios in `src/scenarios/scenarios.cpp` without
assists. Each change was made only after its physical cause was identified:

1. **Propeller (revised in Milestone 2b).** The first calibration set an
   effective pitch of 1.70 m and scaled the chord by 0.85. That compensated
   for missing physics. After swirl and tip compressibility were added, the
   pitch returned to the published 60 in (1.524 m). Only the chord was
   recalibrated (×1.15 of the first estimate), to the POH static rpm. The
   static figure of merit fell into the range measured on C172s.
2. **Drag.** Speeds were high at both ends. Slipstream scrubbing drag on the
   items inside the propeller tube was missing. Adding it brought cruise and
   maximum speed to within about 1%. The remaining scale factor is labelled
   `calibrated`.
3. **Stall speeds.** clmax 1.48, a flap clmax ratio of 0.70 and span
   efficiency 0.86 give +1.9% (flaps up) and +2.8% (flaps 30).
4. **Hands-off roll from engine torque.** Fixed with rigging (a rudder tab and
   aileron trim), solved numerically for cruise. The torque itself was not
   reduced.
5. **Tail lifting at 18 kt on the take-off roll.** Fixed with tail incidence
   −1° and slipstream exposure 0.6. Both are in the plausible range for the
   C172's fixed stabiliser.

Behaviour that was *not* tuned away is recorded under investigated
comparisons in the validation report.
