# Telemetry channels

Both recorders write one CSV row per `interval` of simulated time (0.1 s by
default). Use `avdemo flight csv` and `avdemo instruments csv`.

## `TelemetryRecorder` (flight)

| Channel | Meaning |
|---|---|
| `time_s` | Simulated time |
| `north_m`, `east_m` | Position in the local NED frame |
| `alt_msl_ft`, `agl_m` | Altitude MSL; wing height above ground |
| `roll_deg`, `pitch_deg`, `heading_deg` | Euler angles (true heading) |
| `tas_kt`, `ias_kt`, `gs_kt` | True airspeed; indicated airspeed (instrument); ground speed |
| `vs_fpm` | True vertical speed |
| `alpha_deg`, `beta_deg`, `mach`, `nz_g` | Angle of attack, sideslip, Mach, load factor |
| `mass_kg`, `cg_x_m`, `fuel_kg` | Mass properties |
| `rpm`, `map_inhg`, `fuel_flow_gph`, `power_hp` | Engine |
| `thrust_n`, `prop_eff` | Propeller |
| `flap_deg`, `trim`, `elevator`, `aileron`, `rudder` | Surface and trim positions (normalised) |
| `on_ground`, `stall_warning`, `stalled_strips` | Flags and the count of stalled wing strips |
| `bus_v`, `battery_soc` | Electrical |

## `InstrumentRecorder` (truth versus display)

| Channel | Meaning |
|---|---|
| `tas_kt`, `cas_kt`, `eas_kt`, `gs_kt`, `ias_kt` | Truth speeds, then the ASI reading |
| `alt_true_ft`, `alt_pressure_ft`, `alt_indicated_ft`, `kollsman_inhg` | Geometric and pressure altitude; altimeter reading and setting |
| `vs_true_fpm`, `vsi_fpm` | True vertical speed and VSI |
| `pitch_true_deg`, `pitch_ai_deg`, `roll_true_deg`, `roll_ai_deg`, `ai_tumbled` | Attitude truth and attitude indicator |
| `hdg_true_deg`, `hdg_mag_deg`, `hdg_dg_deg`, `hdg_compass_deg` | Headings: true, magnetic, heading indicator, compass |
| `p_dps`, `q_dps`, `r_dps`, `turn_rate_tc_dps`, `tc_flag` | Body rates; turn coordinator rate and OFF flag |
| `slip_true_deg`, `ball_deg` | Sideslip; inclinometer |
| `static_meas_pa`, `total_meas_pa` | Measured static and total pressure, including position error and faults |
| `ai_rotor`, `dg_rotor`, `tc_rotor` | Gyro rotor speed (fraction of rated) |
| `rpm_tach`, `oil_psi`, `oil_f`, `egt_f`, `fuel_gph`, `fuel_l_gal`, `fuel_r_gal` | Engine and fuel gauges |
| `ammeter_a`, `bus_v`, `suction_inhg` | System gauges |
| `annunciators` | Bit mask: 1 OIL, 2 VAC, 4 VOLTS, 8 L LOW FUEL, 16 R LOW FUEL |
| `stall_horn`, `oat_c` | Stall warning; OAT gauge |
| `nav_valid`, `nav_cdi`, `nav_tofrom`, `gs_valid`, `gs_dev` | NAV 1. CDI and GS deflection are normalised to ±1 full scale |
| `squawk`, `reported_alt_ft` | Transponder. The value is −99999 when no altitude is reported |
