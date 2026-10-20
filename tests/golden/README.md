# Golden traces

Recorded flights of the canonical scenarios in `src/scenarios/golden.cpp`,
one CSV per scenario at 10 Hz, that `tests/test_golden.cpp` replays and
compares channel by channel (`avd_tests golden`). They catch unintended
changes to the physics: a change in any of them means the aircraft now
flies differently.

## Recording

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && ninja -C build
./build/tools/avdtrace --record          # every scenario -> tests/golden/<name>.csv
./build/tools/avdtrace --record c172_cruise_trim   # one of them
./build/tools/avdtrace --check           # what the test does, with the full report
./build/tools/avdtrace --list            # the scenarios and the golden directory
```

Until a scenario's file exists its test prints `SKIPPED` and passes, so a
fresh checkout is green; record on the branch the physics is settled on.

## Policy

- **Goldens change only in a commit that explains why.** The commit that
  re-records a trace says which physics change moved it and quotes the
  report's worst deltas. A golden is never re-recorded to make a red test
  green without that explanation.
- A new scenario is added to `golden.cpp`, to `tests/test_golden.cpp` and to
  `docs/validation/golden-traces.md`, then recorded.
- Traces are deterministic per platform (ADR 0003). Record them with the
  release compiler the project ships with (CI's `linux-gcc`, Release); the
  tolerances in `golden::channels()` leave room for another compiler's
  last-bit differences but not for a physics change.

## Channels and tolerances

| Column | Unit | Band | Column | Unit | Band |
|---|---|---|---|---|---|
| `t_s` | s | exact | `p_dps`, `q_dps`, `r_dps` | deg/s | 0.5 |
| `north_m`, `east_m` | m | 10 | `nz` | g | 0.02 |
| `down_m` | m | 3.05 (10 ft) | `elevator`, `aileron`, `rudder` | deflection | 0.02 |
| `tas_kt`, `cas_kt`, `gs_kt` | kt | 0.5 | `throttle` | lever | 0.02 |
| `alt_msl_ft` | ft | 10 | `thrust1_n`, `thrust2_n` | N | 2 % + 5 N |
| `vs_fpm` | fpm | 30 | `n1_1_pct`, `n1_2_pct` | % (piston: rpm / rated) | 1 |
| `alpha_deg`, `beta_deg`, `pitch_deg` | deg | 0.3 | `fuel_kg` | kg | 0.5 |
| `roll_deg`, `heading_deg` | deg (on the circle) | 0.3 | `gear` | 0..1 | 0.01 |
| | | | `flap_deg` | deg | 0.3 |
| | | | `on_ground` | 0/1 | exact |

## Reference platform

The traces are recorded and compared on Linux x86-64 with GCC in Release.
Another compiler, architecture or optimisation level (Apple clang on arm64,
MSVC, the sanitizer build) differs in the last bits of a few operations and
the contact-heavy scenarios amplify that past the bands, so `avd_tests golden`
skips there with a message; `AVD_GOLDEN_FORCE=1` compares anyway, and the
`avdtrace --check` CI step runs only in the linux-gcc job.
