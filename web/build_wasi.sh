#!/bin/sh
# Aviation Dynamics — build the browser flight demo's WebAssembly module with wasi-sdk.
# SPDX-License-Identifier: MIT
# Usage: WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh [output-dir]   (default: web/dist)
# Tested with wasi-sdk 25. The core builds without exceptions or RTTI here.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT"
EXPORTS=""
for f in avd_text avd_load avd_error_text avd_set_weather avd_reset avd_set_input avd_set_assist \
         avd_quick_start avd_procedure_status avd_advance avd_events avd_cockpit_count avd_cockpit_text \
         avd_cockpit_cycle avd_runway avd_state avd_cockpit avd_markings avd_altimeter_adjust avd_dg_adjust \
         avd_dg_sync avd_heading_bug avd_obs_adjust avd_nav_tune avd_nav_swap avd_com_tune avd_com_swap avd_squawk \
         avd_xpdr_mode avd_dimmer avd_failure avd_trim_throttle avd_trim_mixture avd_compass_card avd_set_cloud avd_reset_air \
         avd_ai_cage avd_jet avd_flow avd_set_level_hold \
         avd_set_station avd_set_tank avd_loadsheet avd_loadsheet_names avd_set_airport avd_airport_text \
         avd_set_wind_detail avd_set_wind_hazard \
         avd_set_runway_water avd_wind_grid avd_set_wake avd_wake avd_wind_box avd_bodies \
         avd_set_destination avd_destination avd_destination_runway avd_destinations_text avd_region \
         avd_ground_height avd_route avd_route_text avd_gps avd_set_destination_record \
         avd_abi_version avd_layout avd_build_id; do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
# The build id avd_build_id() returns: "<git short hash>[-dirty] <UTC date>", or AVD_BUILD_ID from the environment.
if [ -z "${AVD_BUILD_ID:-}" ]; then
  HASH=$(git -C "$REPO" rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
  if [ "$HASH" != unknown ] && ! git -C "$REPO" diff --quiet 2>/dev/null; then HASH="$HASH-dirty"; fi
  AVD_BUILD_ID="$HASH $(date -u +%Y-%m-%d)"
fi
echo "build id: $AVD_BUILD_ID"
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra -mexec-model=reactor \
  -DAVD_BUILD_ID="\"$AVD_BUILD_ID\"" \
  -I"$REPO/src/core/include" -I"$REPO/src/scenarios" \
  "$HERE/avd_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/scenarios/airports.cpp" \
  "$REPO/src/scenarios/trim.cpp" "$REPO/src/scenarios/test_pilot.cpp" \
  $EXPORTS -Wl,--initial-memory=33554432 -Wl,--max-memory=67108864 -Wl,--strip-all \
  -o "$OUT/avd.wasm"
cp "$HERE/avd_loader.mjs" "$OUT/avd.mjs"
cp "$REPO/data/aircraft/tupi_t180.json" "$OUT/tupi_t180.json"
cp "$REPO/data/aircraft/top_g.json" "$OUT/top_g.json"
for t in phenom300 cj4 learjet45 crj700 b738; do cp "$REPO/data/aircraft/$t.json" "$OUT/$t.json"; done
ls -l "$OUT"
