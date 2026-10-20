# Golden traces

A golden trace is a recorded flight of a canonical scenario, sampled at
10 Hz into a fixed set of channels and kept in `tests/golden/<name>.csv`.
`tests/test_golden.cpp` flies each scenario again on every test run and
compares it with the recording channel by channel; a difference beyond the
band means the aircraft now flies differently, which is either a physics
change somebody meant to make (then the golden is re-recorded in a commit
that says so) or a regression. The scenarios are in
`src/scenarios/golden.cpp`, the tool in `tools/avdtrace`.

```sh
./build/tools/avdtrace --list              # the scenarios and the golden directory
./build/tools/avdtrace --record            # fly every scenario and write tests/golden/<name>.csv
./build/tools/avdtrace --check             # replay and compare, with the full per-channel report
./build/tools/avdtrace --run c172_stall_3000ft out.csv   # one scenario to a file (or stdout)
./build/tests/avd_tests golden             # the regression test itself
```

Until a scenario has a golden file its test prints `SKIPPED` and passes, so
CI is green before the first recording. The policy and the recording steps
are in [tests/golden/README.md](../../tests/golden/README.md): **goldens
change only in a commit that explains why.**

## Scenarios

Calm air (no wind, no turbulence of any kind), ISA, the data file's default
load, 240 Hz, scripted pilots only. Each run takes a few seconds to a
minute; the seven together about fifty seconds in a Release build.

| Name | Flight | Ends |
|---|---|---|
| `c172_takeoff_sbsp` | Tupi T-180 at Congonhas 35L: flaps 10, full throttle, rotate at 55 KIAS to 8°, Vy 74 KIAS, flaps up at 300 ft | 500 ft above the field (about 70 s) |
| `c172_stall_3000ft` | Tupi T-180 trimmed at 70 KIAS at 3,000 ft, throttle closed, 1 kt/s deceleration to the break; recovery: back pressure released and full power, nose 5° below the horizon, climb away at 70 KIAS | 20 s after the break (about 56 s) |
| `c172_cruise_trim` | Tupi T-180 trimmed at 100 KIAS at 4,500 ft, stick free in pitch (the trimmed elevator in the trim wheel), wings held level and coordinated | 300 s |
| `c172_ils_sbsp` | Tupi T-180 on the ILS to Congonhas 35L from 6 NM, flown by the instrument-only `ApproachPilot` (flaps 30, 65 KIAS, barometric flare) to touchdown and a stop | stopped (about 343 s) |
| `topg_takeoff_sbsp` | Top G at Congonhas 35L: flaps 10, take-off thrust, rotate at 125 KIAS to 12°, gear up, flaps up at 1,000 ft, wings level | 1,500 ft above the field (about 55 s) |
| `topg_cruise_fl410` | Top G trimmed at Mach 0.85 at FL410, hands off | 180 s |
| `topg_ils_sbgr` | Top G on the ILS to Guarulhos from 10 NM (`ApproachPilot`, landing flap, Vref + 5, radio-altimeter flare, automatic ground spoilers and reversers) to touchdown and a stop | stopped (about 309 s) |

## Channels

30 columns per row; the band is the per-channel tolerance the comparison
applies (absolute, in the channel's unit; thrust also 2 % of the golden
value). Angles are compared on the circle.

| Column | Meaning | Band |
|---|---|---|
| `t_s` | simulated time | exact |
| `north_m`, `east_m`, `down_m` | position in the local NED frame | 10 m, 10 m, 3.05 m (10 ft) |
| `tas_kt`, `cas_kt`, `gs_kt` | true and calibrated airspeed, ground speed | 0.5 kt |
| `alt_msl_ft` | altitude | 10 ft |
| `vs_fpm` | vertical speed | 30 fpm |
| `alpha_deg`, `beta_deg` | angle of attack, sideslip | 0.3° |
| `roll_deg`, `pitch_deg`, `heading_deg` | attitude (true heading, 0–360) | 0.3° |
| `p_dps`, `q_dps`, `r_dps` | body rates | 0.5°/s |
| `nz` | load factor | 0.02 |
| `elevator`, `aileron`, `rudder` | control deflections as the aero model sees them | 0.02 |
| `throttle` | the pilot's lever | 0.02 |
| `thrust1_n`, `thrust2_n` | thrust per engine (propeller or turbofan) | 2 % + 5 N |
| `n1_1_pct`, `n1_2_pct` | turbofan N1 in %; a piston engine's rpm as % of its rated rpm | 1 |
| `fuel_kg` | fuel on board | 0.5 kg |
| `gear` | 0 up and locked .. 1 down and locked | 0.01 |
| `flap_deg` | flap angle | 0.3° |
| `on_ground` | weight on wheels | exact |

The report a mismatch prints lists, per channel, the worst delta, when it
occurred, the band applied there and whether the channel left its band,
and the time of the first divergence over all channels. A length mismatch
(the scenario ended earlier or later) is reported first and the overlap is
compared.

## Determinism

A replay on the machine and compiler that recorded a trace reproduces it
exactly (every channel delta 0). The bands leave room for another
compiler's or platform's last-bit arithmetic over a few minutes of flight,
not for a physics change. Traces are recorded with the project's release
compiler (CI's `linux-gcc`, Release); ADR 0003 (deterministic per platform)
applies.

## Reference platform

The traces are recorded and compared on Linux x86-64 with GCC in Release.
Another compiler, architecture or optimisation level (Apple clang on arm64,
MSVC, the sanitizer build) differs in the last bits of a few operations and
the contact-heavy scenarios amplify that past the bands, so `avd_tests golden`
skips there with a message; `AVD_GOLDEN_FORCE=1` compares anyway, and the
`avdtrace --check` CI step runs only in the linux-gcc job.
