#!/bin/sh
# Ballistic Dynamics — build the WebAssembly module with wasi-sdk.
# SPDX-License-Identifier: MIT
# Usage: WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh [output-dir]   (default: web/dist)
# Tested with wasi-sdk 25. The core builds without exceptions or RTTI here, and
# with -ffp-contract=off like the native builds (ADR 0004).
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
AVD="$REPO/extern/aviation-dynamics/src/core"
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
[ -f "$AVD/src/atmosphere.cpp" ] || { echo "extern/aviation-dynamics is missing: git submodule update --init" >&2; exit 1; }
mkdir -p "$OUT"
EXPORTS=""
for f in bd_text bd_text_length bd_load bd_resolve bd_error_text bd_catalog bd_station bd_wind bd_air bd_zero bd_fire bd_advance bd_round bd_state_at bd_table bd_determinism; do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O2 -fno-exceptions -fno-rtti -ffp-contract=off -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/src/core/include" -I"$REPO/src/aviation/include" -I"$AVD/include" \
  "$HERE/ballistic_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO"/src/aviation/src/*.cpp \
  "$AVD/src/atmosphere.cpp" "$AVD/src/wind.cpp" "$AVD/src/table.cpp" \
  $EXPORTS -Wl,--initial-memory=16777216 -Wl,--max-memory=1073741824 -Wl,--strip-all \
  -o "$OUT/ballistic.wasm"
cp "$HERE/ballistic_loader.mjs" "$OUT/ballistic.mjs"
# One library file for the page.
node -e '
const fs = require("fs"), d = process.argv[1];
const out = { drag_curves: [], projectiles: [], cartridges: [], rifles: [] };
for (const f of ["drag_standard", "projectiles", "cartridges", "rifles"]) {
  const j = JSON.parse(fs.readFileSync(`${d}/${f}.json`, "utf8"));
  for (const k of Object.keys(out)) if (j[k]) out[k].push(...j[k]);
}
fs.writeFileSync(process.argv[2], JSON.stringify(out));
' "$REPO/data" "$OUT/library.json"
ls -l "$OUT"
