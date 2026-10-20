# Validation: Tupi T-180 against the Cessna 172S POH

Reproduce with `./build/avdemo validate` (add `--json file` for the
machine-readable report). The rows, their sources and tolerances are in
`src/scenarios/validation.cpp`; `tests/test_validation.cpp` asserts every
one of them. The tables below are regenerated from the test output by
`tools/update_validation_docs.py` and must not be edited by hand (CI fails
when they are stale). Conditions: 2,550 lb, mid CG, ISA, no wind, no
assists. The scripted test pilot (`src/scenarios/test_pilot.cpp`) flies the
POH technique; the steady points come from the free-air trim solver. The
climb is flown as the handbook says (full rich below 3,000 ft, leaned for
best power above); the cruise table at the recommended lean (φ 1.06); the
maximum glide with the engine failed (idle cut-off) and the propeller
windmilling, which is the chart's condition.

## Results

<!-- validation:begin table aircraft=tupi_t180 -->
| Quantity | Published | Simulation | Error | Band | Result | Source |
|---|---|---|---|---|---|---|
| Stall speed, flaps up, power off<br><small>1 kt/s deceleration at idle, wings level; clean section clmax is the published NACA 2412 value (not fitted)</small> | 53 KCAS | 53.3 KCAS | +0.6 % | ±6 % | within | C172S POH Section 5 |
| Stall speed, flaps 30, power off<br><small>the flap clmax increment is calibrated to this row</small> | 48 KCAS | 48.2 KCAS | +0.3 % | ±6 % | within | C172S POH Section 5 |
| Stall warning margin, flaps up<br><small>horn on at 55.8 KIAS, stall at 47.1 KIAS</small> | 5-10 kt above the stall | 8.66 kt | — | 5–10 kt | within | C172S POH Section 5 |
| Stall warning margin, flaps 30<br><small>horn on at 50.1 KIAS, stall at 42.9 KIAS</small> | 5-10 kt above the stall | 7.14 kt | — | 5–10 kt | within | C172S POH Section 5 |
| Maximum level speed, sea level<br><small>full throttle, best power; 2693 rpm</small> | 126 KTAS | 127 KTAS | +0.4 % | ±5 % | within | C172S POH Section 5 |
| Engine speed at the maximum level speed | at most 2,700 rpm (red line) | 2,693 rpm | — | 0–2,700 rpm | within | C172S POH Section 5 |
| Full-throttle level speed, 8,000 ft<br><small>74 % power, 2606 rpm, propeller efficiency 0.84, 11.0 gph at best power</small> | 124 KTAS (75 % power, 2,650 rpm) | 120 KTAS | -2.9 % | ±5 % | within | C172S POH Section 5, Figure 5-8 |
| Power at full throttle, 8,000 ft | 75 % | 74.2 % | -1.1 % | ±8 % | within | C172S POH Section 5, Figure 5-8 |
| Fuel flow at full throttle, 8,000 ft<br><small>trimmed at best power (richer than the handbook's recommended lean)</small> | about 10.5 gph (recommended lean) | 11.0 gph | — | 8.50–12 gph | within | C172S POH Section 5, Figure 5-8 |
| Rate of climb, sea level, Vy<br><small>74 KIAS, full throttle, full rich (POH: lean above 3,000 ft)</small> | 730 fpm | 739 fpm | +1.2 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Rate of climb, 4000 ft, Vy<br><small>72-74 KIAS, full throttle, mixture leaned for best power</small> | 600 fpm | 587 fpm | -2.2 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Rate of climb, 8000 ft, Vy<br><small>72-74 KIAS, full throttle, mixture leaned for best power</small> | 455 fpm | 425 fpm | -6.6 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Rate of climb, 12000 ft, Vy<br><small>72-74 KIAS, full throttle, mixture leaned for best power</small> | 310 fpm | 264 fpm | -14.8 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (2,550 lb, standard temperature) |
| Time to climb, sea level to 8,000 ft | 14 min | 14.1 min | +0.6 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance) |
| Fuel to climb, sea level to 8,000 ft<br><small>the handbook adds 1.4 gal for start, taxi and take-off, not counted here</small> | 3.0 gal | 2.66 gal | -11.3 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance) |
| Distance to climb, sea level to 8,000 ft<br><small>zero wind</small> | 19 NM | 18.2 NM | -4.1 % | ±15 % | within | C172S POH Section 5, Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance) |
| Cruise speed, 2000 ft, 55 % power<br><small>2225 rpm, throttle 45 %, mixture 50 degF rich of peak</small> | 100 KTAS | 100 KTAS | +0.3 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 2000 ft, 55 % power<br><small>recommended lean</small> | 7.8 gph | 7.87 gph | +0.9 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 2000 ft, 65 % power<br><small>2368 rpm, throttle 51 %, mixture 50 degF rich of peak</small> | 109 KTAS | 108 KTAS | -0.5 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 2000 ft, 65 % power<br><small>recommended lean</small> | 9.1 gph | 9.14 gph | +0.4 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 2000 ft, 75 % power<br><small>2493 rpm, throttle 60 %, mixture 50 degF rich of peak</small> | 116 KTAS | 115 KTAS | -0.6 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 2000 ft, 75 % power<br><small>recommended lean</small> | 10.5 gph | 10.4 gph | -0.9 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 8000 ft, 55 % power<br><small>2334 rpm, throttle 55 %, mixture 50 degF rich of peak</small> | 106 KTAS | 105 KTAS | -1.4 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 8000 ft, 55 % power<br><small>recommended lean</small> | 7.9 gph | 7.85 gph | -0.7 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 8000 ft, 65 % power<br><small>2489 rpm, throttle 70 %, mixture 50 degF rich of peak</small> | 116 KTAS | 114 KTAS | -2.0 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 8000 ft, 65 % power<br><small>recommended lean</small> | 9.2 gph | 9.14 gph | -0.6 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Cruise speed, 8000 ft, 75 % power<br><small>full throttle gives only 73 % here; 2586 rpm, throttle 100 %, mixture 50 degF rich of peak</small> | 124 KTAS | 119 KTAS | -3.8 % | ±5 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Fuel flow, 8000 ft, 75 % power<br><small>recommended lean</small> | 10.5 gph | 10.1 gph | -3.7 % | ±15 % | within | C172S POH Section 5, Figure 5-8 (interpolated in % BHP) |
| Best glide ratio at 68 KIAS, engine failed, propeller windmilling<br><small>throttle closed, mixture at idle cut-off, propeller windmilling the dead engine at 993 rpm, 3,000 ft; sink 748 fpm</small> | about 9.1:1 (1.5 NM per 1,000 ft) | 9.57 :1 | +5.2 % | ±15 % | within | C172S POH Section 5 (Maximum Glide: power off, propeller windmilling) and Section 3 (68 KIAS) |
| Glide distance per 1,000 ft<br><small>from the ratio above</small> | — | 1.58 NM | — | — | info | — |
| Propeller thrust in the glide (engine failed, windmilling)<br><small>the windmilling propeller drives the dead engine's friction and pumping, 9.2 hp at 993 rpm; + is pulling</small> | — | -219 N | — | — | info | — |
| Glide ratio at 68 KIAS with the engine idling (practice glide)<br><small>throttle closed, mixture rich: the idling engine turns 1153 rpm, -1.4 hp, propeller thrust -55 N; flatter than the engine-failure glide, as pilots find</small> | — | 11.1 :1 | — | — | info | — |
| Airframe lift/drag at 68 KIAS, no propeller force<br><small>aero model alone, L = W</small> | — | 11.9 :1 | — | — | info | — |
| Take-off ground roll (short field)<br><small>flaps 10, lift off at 51 KIAS, climb at 56 KIAS</small> | 960 ft | 781 ft | -18.6 % | ±20 % | within | C172S POH Section 5 (short-field take-off, sea level, ISA) |
| Take-off distance over a 50 ft obstacle | 1,630 ft | 1,529 ft | -6.2 % | ±20 % | within | C172S POH Section 5 (short-field take-off, sea level, ISA) |
| Lift-off speed | 51 KIAS | 51.5 KIAS | — | 48–55 KIAS | within | C172S POH Section 5 |
| Landing ground roll, pilot braking technique<br><small>touchdown at 50 KCAS, 1 s to the pedals, 1 s ramp, 50 % sustained pedal (no anti-skid), the throttle closed (the idling propeller drags a little); see the discussion in the validation note</small> | 575 ft | 566 ft | -1.6 % | ±25 % | within | C172S POH Section 5 (short-field landing, sea level, ISA) |
| Landing roll, tyre-limited bound (full pedal at touchdown)<br><small>physics check, not a handbook figure</small> | — | 311 ft | — | — | info | — |
| Static rpm, full throttle | 2,300-2,420 rpm | 2,383 rpm | — | 2,300–2,420 rpm | within | C172S POH Section 4 (full-throttle static check) |
<!-- validation:end -->

<!-- validation:begin summary aircraft=tupi_t180 -->
34 compared rows, 0 outside their band.
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
| Best glide | C172S POH Section 3 (68 KIAS) and Section 5 (maximum glide: power off, propeller windmilling, flaps up, zero wind, about 1.5 NM per 1,000 ft, 9.1:1) | The chart's "propeller windmilling" is the engine-failure glide: flown here with the mixture at idle cut-off. The idle-power glide is reported as information |
| Take-off, landing | C172S POH Section 5 (short-field take-off and landing, sea level, ISA) | Landing: demonstrated figure with an unpublished braking technique; see below |
| Static rpm | C172S POH Section 4 (full-throttle static check 2,300–2,420 rpm) | |

## The engine rows (October 2026): the glide, the climb lapse and the fuel

The rows added in October 2026 (climb lapse, cruise table, glide) were the
first to probe the engine at part throttle, at a windmilling rpm and at
altitude. Three things were found and fixed in the engine model; one row
was corrected; one row is left failing for the ground model. Nothing was
tuned to a row: every parameter changed is a physical quantity set from a
published engine figure, and the change is recorded in the JSON provenance.

**The glide (21:1 against 9.1:1) was two things.** First, the induction
model: the closed throttle gave a manifold pressure of 0.33 × ambient at
any rpm, so a propeller windmilling the engine at 1,500 rpm found 27 hp
and 477 N of thrust. The intake is now a butterfly plate (effective area
1 − cos of the plate angle) in series with the engine's pumping: at the
closed stop the idle orifice is choked, the air flow is fixed, and MAP
falls as 1/rpm toward the residual-gas floor (ambient / compression
ratio), the volumetric efficiency falls with the exhaust-to-intake ratio
(Heywood's (r_c − p_e/p_i)/(r_c − 1)) and the pumping loop (ambient − MAP)
is charged against the shaft. On the ground the engine idles at 704 rpm
and 8.3 inHg; windmilled at 1,200 rpm with the throttle closed it sees 6
inHg and absorbs power. Second, the row: the handbook's Maximum Glide chart
is for "power off, **propeller windmilling**" — the engine-failure glide of
Section 3, with a dead engine driven by the airstream against its friction
and pumping. The row now flies that (mixture at idle cut-off): the
propeller windmills the dead engine at 993 rpm, absorbing 9 hp and dragging
219 N, and the glide is **9.6:1 (+5 %)**. The idle-power glide (engine
firing, throttle closed) is reported as information: 11.1:1 at 1,153 rpm
with the propeller at about zero lift (−55 N) — the flatter "practice"
glide every instructor warns about. The airframe alone is 11.9:1.

**The climb lapse was the engine's torque at the climb rpm, not its
altitude lapse.** At any given rpm the model already reproduced the
Lycoming sea-level/altitude curve and Gagg–Farrar (P/P0 = 1.132 σ − 0.132)
to within 1 % (the friction mep was trimmed from 160 to 130 kPa so that
friction plus the full-throttle pumping loss is the 13.2 % of rated power
that curve implies). What was missing is that the fixed-pitch propeller
holds the climb at about 2,250 rpm, where Lycoming's full-throttle curve
has 9 % more torque than at 2,700 (about 180/166/155/143/131 hp at
2,700/2,400/2,200/2,000/1,800 rpm) and the model, with a constant
volumetric efficiency, had 4 %. The inlet-port flow loss, which grows with
the square of the flow (Heywood 6.2.4), is now in the model
(`volumetric_efficiency_speed_loss` 0.08, calibrated to that curve). The
climb rows are 739 / 587 / 425 / 264 fpm against 730 / 600 / 455 / 310
(+1 %, −2 %, −7 %, −15 %), the time to 8,000 ft 14.1 min against 14, the
distance 18.2 NM against 19. The 12,000 ft row sits at the edge of its
band. What remains is not in the engine: the POH's 310 fpm at 12,000 ft
needs about 88 hp of thrust power from the 106 hp the Gagg–Farrar-lapsed
engine makes at that rpm (a propeller efficiency of 0.83 at J ≈ 0.64, not
credible) unless the drag at 72 KIAS is about 10 % lower than the model's
(L/D 10.6 in the climb, with the slipstream and trim). The full-throttle
level row at 8,000 ft points the same way: the model turns the same
advance ratio as the handbook (2,606 rpm at 120 KTAS against 2,650 at
124) and makes the handbook's power (74 % against 75 %) but flies 3 %
slower, so the drag at 100–110 KCAS is a few percent high. That is the
wing and fuselage model's territory (induced drag and slipstream scrubbing
at CL 0.8–0.9), noted for it here.

**The fuel flows were 5–8 % lean of the handbook at every power.** The
indicated efficiency is calibrated from the rated power and the volumetric
efficiency, and 0.85 implied 0.342; Lycoming's published full-rich take-off
fuel flow (about 16 gph, 96 lb/h at φ ≈ 1.25–1.3) implies a volumetric
efficiency of about 0.89 at rated rpm and an indicated efficiency of 0.33.
With 0.89 the six cruise fuel flows are within −4 % to +1 % of the
handbook (five of them within 1 %), the fuel to 8,000 ft is 2.66 gal
against 3.0 (−11 %; the climb below 3,000 ft is now flown full rich, as
the handbook's technique says) and the take-off BSFC is 0.48 lb/hp/h.

**Side effects, all inside their bands.** The static rpm is 2,384 (was
2,357), the sea-level climb 739 fpm (707), the take-off roll 781 ft
(−19 %, was −15 %: more torque at the roll's 2,200–2,350 rpm; the lift-off
and transition residual discussed below is unchanged), the stall speeds
moved 0.2 kt with the idle thrust.

**Landing roll: 408 ft against 575 (−29 %), left failing.** The roll
shortened from 468 ft because the old idle pulled 477 N throughout it,
which is gone; the idling propeller now gives a slight drag at 40–50 kt, as
a real one does. The braking technique (1 s to the pedals, 1 s ramp, 50 %
sustained pedal) was chosen from the brakes' lock-up threshold under the
old model and is **not** re-solved for the handbook number here. The
tyre-limited bound (full pedal from touchdown) is 299 ft, half the
handbook's demonstrated "maximum braking" roll: the remaining gap is in
the brake torque capacity, the tyre's peak friction and the load transfer
of the ground model, not in the engine, and is flagged for that model's
owner. No tolerance was widened.

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

**Torque-limited brakes (ground-handling model): the sustained pedal became
50%.** The brakes were a force in proportion to the wheel load (0.65 N at
any pedal), so they did little while the full-flap wing still carried the
airplane. They are now a torque capacity (630 N·m per main, the old model's
value at the static load) acting on a spinning wheel, with the tyre sliding
at 0.75 of its peak friction once it locks. At 70% pedal that gave 2 kN per
main from the moment the wheels took any load and a 386 ft roll (−33%). A
pilot without anti-skid holds the pressure the wheels take without locking
once the wing has unloaded; with these brakes that is about half the pedal
(1.4 kN per main, which the wheels hold above ~1.8 kN of load), so the
technique's sustained effort is 0.5. The roll is 468 ft (−18.6%); the
tyre-limited bound (full pedal from touchdown, locking and sliding until
the wing unloads) is 315 ft. The effort was chosen from the lock-up
threshold, not solved for the POH number; 0.55 would give 439 ft and 0.4
gives 547 ft.

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
  - hard steering at speed can overturn the airplane;
  - (`tests/test_ground_handling.cpp`) braking and cornering share one
    friction ellipse; a full-brake stop on a wet runway locks the mains,
    which slide at 0.75 of the peak friction, stop 12% longer than with an
    anti-skid system and ground-loop without pedals; touchdown spins the
    wheels up in under 0.1 s with an aft force and a nose-down pitch; parked
    on a 2° slope it does not creep in 60 s, and the rolling threshold holds
    it brakes-off on 1° but not 4°; a 15 kt crosswind take-off roll stays
    on the centreline with the pedals and weathervanes without them; the
    castoring nose wheel turns toward its velocity in a brake turn and the
    bungee linkage yields under a dragged tyre.
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
