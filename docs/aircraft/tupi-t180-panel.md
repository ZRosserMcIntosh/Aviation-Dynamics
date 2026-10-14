# Tupi T-180 instrument panel (`c172s_analog`)

Design and data flow: [../architecture/instruments-and-cockpit.md](../architecture/instruments-and-cockpit.md).
Data: the `panel` section of `data/aircraft/tupi_t180.json`.

## Variant

This is the conventional analog ("steam gauge") panel of a Cessna 172S
Skyhawk SP. It has:

- a vacuum attitude indicator and a vacuum heading indicator;
- an electric turn coordinator;
- a pitot-static airspeed indicator, altimeter and VSI;
- a panel wet compass;
- the 172S engine cluster (tachometer, oil temperature and pressure, fuel
  quantity left and right, fuel flow, EGT, ammeter, suction);
- an annunciator panel;
- one NAV/COM with a VOR/LOC/GS indicator;
- a transponder with an altitude encoder;
- a digital clock and OAT.

The real airplane offered GPS, ADF, DME and an autopilot as options. **This
profile does not install them**, and the data says so (`gps: false`, …). A G1000
profile is a separate future profile, not a variation of this one.

## Instruments and parameters

| Instrument | Model | Parameters | Kind |
|---|---|---|---|
| Airspeed indicator | Diaphragm on pitot minus static pressure. Second-order needle. Pitot recovery falls beyond the insensitive flow angle | White 40–85, green 48–129, yellow 129–163, red line 163 KIAS; dial 40–200 | published (POH sec. 2) |
| | | Insensitive angle 17°; needle ωn 6 rad/s, ζ 1.0 | estimated |
| Static-source position error | ΔP as a function of α and flap, fitted to the POH calibration table | — | calibrated |
| Altimeter | Aneroid inverted through the ISA pressure–altitude law, referenced to the Kollsman setting. Three hands | Setting 28.10–31.00 inHg | estimated (generic sensitive altimeter) |
| VSI | Case pressure leaking through a capillary. The needle reads the case/static differential, so the indication lags | Time constant 3 s; dial ±2000 fpm | estimated |
| Attitude indicator | Spin axis held in NED. Pendulous erection toward the apparent vertical, so acceleration and turn errors emerge. Tumbles past its limits. Spins up and down on suction | Erection 6°/min; limits 70° pitch / 110° bank; spin-up 60 s, spin-down 200 s; useful above 70% rotor speed | estimated |
| Heading indicator | Inertial azimuth with earth-rate drift (from latitude) plus mechanical drift. Gimbal error in bank. Tumbles past the gimbal limit. Card offset by the SET knob; sync to the compass | Drift 6°/h; gimbal limit 55° | estimated |
| Turn coordinator | Electric rate gyro canted 30°, so it senses roll rate as well as yaw rate. OFF flag when unpowered. The rotor coasts down | Standard-rate mark at 20° deflection, calibrated at 16° bank; maximum 40° | estimated |
| Inclinometer | Ball angle atan2(−fy, −fz) from specific force, damped | Tube limit ±16° | estimated |
| Magnetic compass | Pendulous card in the local magnetic field with dip, so northerly turning and acceleration errors emerge. Card tilt limit | 18° | estimated |
| Engine gauges | Tachometer, oil pressure (pump, relief valve, viscosity), oil temperature, EGT, fuel flow, fuel quantity (damped senders), ammeter, suction | Tach green 2100–2700, red 2700; oil pressure green 50–90, red 20/115 psi; oil temperature green 100–245 °F, red 245; suction green 4.5–5.5 inHg | published (POH sec. 2) |
| Annunciators | OIL PRESS, LOW VAC, VOLTS, L LOW FUEL, R LOW FUEL. Powered from the bus | OIL < 20 psi, VAC < 3.0 inHg, VOLTS < 24.5 V, fuel < 5 gal per tank | published (POH sec. 7) |
| Stall warning | Pneumatic reed driven by a vane at the wing-strip angle of attack, not by an airspeed | Margin from the stall_warning section | calibrated (sounds 5–10 kt before the stall, per POH) |
| OAT | Probe with ram temperature rise and a lag | Recovery factor 0.8; time constant 10 s | estimated |
| NAV receiver | VOR: 10° full-scale CDI, TO/FROM, flags in the cone of confusion and abeam. Localizer: course-width scaling. Glideslope: ±0.7° full scale. Radio-horizon reception | — | standard ICAO Annex 10 geometry |
| Transponder | Modes OFF / SBY / ON / ALT. The encoder reports pressure altitude (29.92) in 100 ft steps | — | — |

## Navigation data

`src/scenarios/airports.cpp` ships only **synthetic** test facilities, each
labelled `synthetic` in its `source`:

| Facility | Frequency | Location and geometry |
|---|---|---|
| VOR TSP | 113.70 | At MASP |
| LOC ITMT | 110.30 | Runway 12 at SBMT. Antenna 300 m beyond the far end; course width giving ±106.7 m at the threshold |
| GS | — | 3.0°, 300 m in from the threshold and 120 m to the side |

These are not real-world navaids, and their frequencies are not real
assignments. The magnetic field is estimated at declination −21.5° and
inclination −38°. Real navaid data can be loaded through
`NavDatabase::loadJson`, which requires a `source` for every facility.

## Not implemented (honest list)

- Static-port icing (the pitot probe ices and is heated; see the
  `pitot_probe` data). Airframe icing (wings, propeller, induction) is not
  modelled.
- Instrument lighting photometry. The panel lights are an electrical load and
  tint the 2D panel; there is no light model.
- Altimeter mechanical hysteresis and scale error. CHT gauge and voltmeter are
  not installed on this profile.
- COM audio and ATC. The COM frequency is tuned and displayed only.
- Marker beacons, GPS, ADF, DME and autopilot are not installed and not
  modelled.
