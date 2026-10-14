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
         avd_xpdr_mode avd_dimmer avd_failure; do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/src/core/include" -I"$REPO/src/scenarios" \
  "$HERE/avd_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/scenarios/airports.cpp" \
  $EXPORTS -Wl,--initial-memory=33554432 -Wl,--max-memory=67108864 -Wl,--strip-all \
  -o "$OUT/avd.wasm"
cp "$HERE/avd_loader.mjs" "$OUT/avd.mjs"
cp "$REPO/data/aircraft/tupi_t180.json" "$OUT/tupi_t180.json"
ls -l "$OUT"
