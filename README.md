# Ballistic Dynamics

A projectile engine in C++20 for the São Paulo open world, where every round
fired is its own projectile with its own state, flying through the real air
at that place and moment, and hitting material, not hitboxes. It has no
renderer and no game-engine dependency; projectiles, cartridges, rifles and
drag curves are JSON with a provenance note on every number. The same core
runs natively and in the browser as WebAssembly, and gives the same bits on
both.

It is the fourth engine of the project, next to **Glass Dynamics**
(glazing fracture), **Aviation Dynamics** (flight, and the atmosphere this
engine flies through) and **OpenRoad Dynamics** (vehicles).

**Status: Milestone 1, the skeleton.** The air, the clock, the deterministic
maths, the data library and the browser build are in. **No trajectory yet**:
the point-mass model is Milestone 2. The [roadmap](docs/roadmap.md) has the
rest, and nothing beyond what is listed here is claimed.

## What works today

| Area | What is implemented | Evidence |
|---|---|---|
| Air | Aviation Dynamics' U.S. Standard Atmosphere 1976 with temperature and pressure deviations and humidity (moist density from the virtual temperature, speed of sound, density altitude) and its wind and turbulence field, compiled unchanged from a pinned submodule behind a host-facing `AirSource` interface ([ADR 0003](docs/decisions/0003-atmosphere-source.md)). A shooter's station reading (pressure, temperature, humidity, elevation) becomes Aviation's conditions exactly | `tests/test_air.cpp` |
| Wind on the range | Layers by distance along the line of fire with seeded gusts and blended joins, over any air; the shooter's call and the true wind as two sources | `tests/test_air.cpp` |
| Clock | Fixed 1 ms steps counted as integers, RK4; the host's frame rate does not change the result ([ADR 0002](docs/decisions/0002-fixed-step-rk4.md)) | `tests/test_foundations.cpp` (60 Hz, 144 Hz, one call and a jittery host end bit-identical) |
| Determinism | Elementary functions on correctly rounded operations only (fdlibm's algorithms), no FMA contraction, seeded xoshiro256** streams per round; a golden hash every platform must reproduce ([ADR 0004](docs/decisions/0004-determinism.md)) | `tests/test_foundations.cpp`, `web/smoke_test.mjs` |
| Data | JSON library of drag curves, projectiles, cartridges and rifles; units in the field names; a `published:` / `measured:` / `estimated:` note required for every number ([data-format.md](docs/data-format.md)) | `tests/test_data.cpp` |
| Frames | SI inside; ENU world as Aviation's; line of fire x downrange, y left, z up (ISO 8855's order, as OpenRoad) ([ADR 0001](docs/decisions/0001-frames-and-units.md)) | `tests/test_foundations.cpp` |
| Browser | WebAssembly build (wasi-sdk 25, no exceptions or RTTI, 190 KB) with a C ABI; Node smoke test | `web/`, `web/smoke_test.mjs` |

19 test cases, on Linux (gcc, clang, sanitizers), macOS, Windows and WASM in
CI. What each scenario gives against its reference is in
[docs/validation.md](docs/validation.md); how far each model goes, and its main
gap, in [docs/physics.md](docs/physics.md) and the
[fidelity matrix](docs/fidelity-matrix.md).

## Build and run

The atmosphere comes from Aviation Dynamics as a submodule:

```sh
git clone --recurse-submodules https://github.com/ZRosserMcIntosh/ballistic-dynamics
# or, in an existing clone: git submodule update --init
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/tests/bd_tests                  # 19 test cases
./build/tests/bd_tests "air:"           # a filter
./build/bdemo list                      # the data library, with provenance
./build/bdemo air 83400 30 0.2 1609     # the air from a station reading, and up the path
./build/bdemo determinism               # this build's determinism hash
```

Options: `BD_BUILD_TESTS`, `BD_BUILD_TOOLS`, `BD_WARNINGS_AS_ERRORS`.

The browser build needs [wasi-sdk](https://github.com/WebAssembly/wasi-sdk);
version 25 is tested.

```sh
WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh   # web/dist: ballistic.wasm, ballistic.mjs, library.json
node web/smoke_test.mjs
```

## Using it

```cpp
#include "bd/aviation_air.hpp"
#include "bd/library.hpp"

bd::Library lib;
lib.load(projectilesJson); lib.load(cartridgesJson); lib.load(riflesJson);
lib.resolve();

// The air at the firing point, from the shooter's kestrel; ENU origin at the station.
bd::StationAir air(bd::StationReading{83400.0, 303.15, 0.2, 1609.0});
// The true wind along a northward line of fire, and what the shooter called.
bd::RangeWind truth(air, {0, 0, 0}, 0.0, {{0, 3, 1.5 * bd::kPi, 0.8}, {400, 5, 1.4 * bd::kPi, 1.2}});
bd::RangeWind call(air, {0, 0, 0}, 0.0, {{0, 4, 1.5 * bd::kPi}});
bd::AirSample a = truth.sample({0, 650, 3}, 1.2);   // density, speed of sound, wind (ENU)
```

A host with its own weather wraps Aviation's field directly:
`bd::AviationAir air(aviationWeatherField, originAltitudeMsl);`

## Repository layout

```
src/core/       bd_core: the engine-independent projectile core (namespace bd)
src/aviation/   bd_aviation: the adapter to Aviation Dynamics' atmosphere
extern/         aviation-dynamics, pinned (git submodule)
tools/bdemo/    headless tool: the library, the air, the determinism hash
tests/          dependency-free test harness and suites
data/           projectiles, cartridges, rifles (JSON, with provenance)
web/            C ABI, WASM build, JS loader, smoke test
docs/           decisions (ADRs), physics, validation, fidelity matrix, roadmap, data format
```

## Principles

Shared with the sibling engines:

- **Nothing scripted.** Every shot will come out of the physics; the
  renderer only draws.
- **Identical everywhere.** The same seed gives the same shot bit for bit, on
  every machine and in every browser (ADR 0004 says exactly how far that
  holds today).
- **Scientific honesty.** Every value is tagged `published`, `measured` or
  `estimated` with its source or reasoning. A disagreement with a published
  number is written in [docs/validation.md](docs/validation.md), never tuned
  away silently.
- **One atmosphere.** The air is Aviation Dynamics', not a copy of it.

## Licence

MIT. Aviation Dynamics (submodule) is MIT. The elementary functions in
`src/core/src/dmath.cpp` follow fdlibm (Sun Microsystems, freely
redistributable with its notice, which the file carries). Product names
(Sierra MatchKing, Federal Gold Medal Match, M24) identify the published data
a record follows; nothing here is endorsed by their makers.
