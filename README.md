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

**Status: Milestone 4 and the shooter layer.** These are in:

- the air, the clock, the deterministic maths, the data library and the
  browser build;
- the point-mass trajectory: G1/G7 drag, zeroing, range cards, bullet-time
  interpolation;
- spin (Miller stability, spin drift, aerodynamic jump, the wobble, tumbling)
  with Coriolis and Eötvös;
- every round its own: velocity spread and powder temperature, BC spread, the
  barrel's throw, seeded per round;
- **the shooter layer** ([ADR 0005](docs/decisions/0005-shooter-layer.md)):
  the weapon's cycle on the 1 ms clock, recoil from momentum with an
  MW2-shaped kick, the hip cone, and the sight.

**Not yet:** dynamic (transonic) stability, scope cant and the solver API
(the rest of 4), and materials (5). The [roadmap](docs/roadmap.md) has the
rest, and nothing beyond what is listed here is claimed.

## What works today

| Area | What is implemented | Evidence |
|---|---|---|
| Point mass | The round in the world frame with drag from the BRL G1 or G7 table scaled by its (stepped) BC, or its own C_D(Mach); air and wind sampled along the path at every Runge-Kutta stage; zeroing and range cards (drop, drift, time, speed, energy, Mach) | `tests/test_point_mass.cpp`: six cartridges (.308 175 SMK, 6.5 CM 140 ELD-M, .338 LM 250 Scenar, .50 BMG 750 A-MAX, 5.56 M855, 9 mm 124 Gold Dot) agree with an independent solver to 0.06 % in drop, and with published figures to within 0.05 in to +4 %; Federal's 9 mm, .45 Auto and .357 Magnum velocity charts to 0.4 % ([validation](docs/validation.md)) |
| Spin and the Earth | Miller's stability with its velocity and air corrections, then Sg ∝ p²/(ρV²C_mα) downrange; the yaw of repose and its lift (spin drift); Litz's aerodynamic jump from a crosswind; the epicyclic wobble for drawing; tumbling below Sg 1; Coriolis and Eötvös at any latitude and direction of fire | `tests/test_spin.cpp`: spin drift 0.79–0.98 of Litz's measured-data formula for five rifle loads without tuning; Coriolis against closed forms and the integrated lateral equation to 0.3 % |
| Bullet time | The state at any instant of the flight, smooth between the 1 ms steps (cubic Hermite, within 20 µm of a 20× finer run), so a camera can follow the round at 1/1000 speed with the same physics ([bullet-time.md](docs/bullet-time.md)) | `tests/test_point_mass.cpp`, `web/smoke_test.mjs` |
| Shooter layer | The weapon on the 1 ms clock: the cyclic rate exact at any frame rate (the remainder carried), semi, auto, burst and manual actions, magazine, dry fire, reload, sprint-out; the shot on the first step of the click's frame. Free recoil from the momentum of the bullet and its powder gas; the view's kick `p h / (M k² + I_hold)` with a constant pull, a slowed return that stops at the aim (no overshoot), a braced first shot; a visual-only gun spring; a hip cone that firing, turning and moving open, none at the sight; the hold's wander. One seeded stream per round, its recoil included ([ADR 0005](docs/decisions/0005-shooter-layer.md), [physics §7](docs/physics.md)) | `tests/test_shooter.cpp`: exact rate at 60 Hz, 144 Hz, one call and a jittery host bit for bit; free-recoil energies in the commonly quoted ranges; the kick against its closed form; a golden string the browser reproduces |
| Air | Aviation Dynamics' U.S. Standard Atmosphere 1976 with temperature and pressure deviations and humidity (moist density from the virtual temperature, speed of sound, density altitude) and its wind and turbulence field, compiled unchanged from a pinned submodule behind a host-facing `AirSource` interface ([ADR 0003](docs/decisions/0003-atmosphere-source.md)). A shooter's station reading (pressure, temperature, humidity, elevation) becomes Aviation's conditions exactly | `tests/test_air.cpp` |
| Wind on the range | Layers by distance along the line of fire with seeded gusts and blended joins, over any air; the shooter's call and the true wind as two sources | `tests/test_air.cpp` |
| Clock | Fixed 1 ms steps counted as integers, RK4; the host's frame rate does not change the result ([ADR 0002](docs/decisions/0002-fixed-step-rk4.md)) | `tests/test_foundations.cpp` (60 Hz, 144 Hz, one call and a jittery host end bit-identical) |
| Determinism | Elementary functions on correctly rounded operations only (fdlibm's algorithms), no FMA contraction, seeded xoshiro256** streams per round; golden hashes of the maths and of a whole trajectory that every platform must reproduce ([ADR 0004](docs/decisions/0004-determinism.md)) | `tests/test_foundations.cpp`, `tests/test_point_mass.cpp`, `web/smoke_test.mjs` |
| Data | JSON library of drag curves, projectiles, cartridges and guns (rifles, and since this round handguns and sub-machine guns: Glock 17, 1911, S&W 686, Five-seveN, MP5, P90, Thompson); units in the field names; a `published:` / `measured:` / `estimated:` note required for every number ([data-format.md](docs/data-format.md)) | `tests/test_data.cpp` |
| Frames | SI inside; ENU world as Aviation's; line of fire x downrange, y left, z up (ISO 8855's order, as OpenRoad) ([ADR 0001](docs/decisions/0001-frames-and-units.md)) | `tests/test_foundations.cpp` |
| Browser | WebAssembly build (wasi-sdk 25, no exceptions or RTTI, 230 KB) with a C ABI: the air, wind layers, zeroing, firing, bullet-time state, range cards; Node smoke test | `web/`, `web/smoke_test.mjs` |

60 test cases, on Linux (gcc, clang, sanitizers), macOS, Windows and WASM in
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
./build/tests/bd_tests                  # 60 test cases
./build/tests/bd_tests "point mass"     # a filter
BD_VERBOSE=1 ./build/tests/bd_tests validation   # the comparison tables
./build/bdemo list                      # the data library, with provenance
./build/bdemo table fgmm_308_175        # a range card: drop, drift, speed, energy to 1,000 yd
./build/bdemo table hornady_65cm_140 G1 100 1.5 10 1000 50   # model, zero, sight, wind, range, step
./build/bdemo table fgmm_308_175 - 100 1.5 0 1000 100 - rifle=m24_sws lat=-23.55 az=90   # spin, Coriolis, Eötvös
./build/bdemo air 83400 30 0.2 1609     # the air from a station reading, and up the path
./build/bdemo shoot m16a2 federal_xm855 standing 1   # a three-round burst: kick, gun, cone, ms by ms
./build/bdemo shoot mp5a3 mp5_9mm_124_fmj prone 1 1000 0   # a second of full auto from the hip, prone
./build/bdemo determinism               # this build's determinism hashes
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
#include "bd/solver.hpp"

bd::Library lib;
lib.load(dragStandardJson); lib.load(projectilesJson); lib.load(cartridgesJson); lib.load(riflesJson);
lib.resolve();

// The air at the firing point, from the shooter's kestrel; ENU origin at the station.
bd::StationAir air(bd::StationReading{83400.0, 303.15, 0.2, 1609.0});
// The true wind along a northward line of fire, and what the shooter called.
bd::RangeWind truth(air, {0, 0, 0}, 0.0, {{0, 3, 1.5 * bd::kPi, 0.8}, {400, 5, 1.4 * bd::kPi, 1.2}});
bd::RangeWind call(air, {0, 0, 0}, 0.0, {{0, 4, 1.5 * bd::kPi}});
bd::AirSample a = truth.sample({0, 650, 3}, 1.2);   // density, speed of sound, wind (ENU)

// Zero a .308 at 100 m and fire it through the true wind.
const bd::ProjectileDef& p = *lib.projectile("smk_308_175");
std::string err;
bd::RangeSetup s;
s.drag = bd::dragModelFor(p, lib, "", err);          // its own curve, else G7, else G1
s.muzzleSpeed = lib.cartridge("fgmm_308_175")->muzzleVelocity;
s.sightHeight = 0.04;
s.zeroRange = 100;
const double elevation = bd::zeroElevation(s, air);
bd::PointMassOptions o;
o.keepTrace = true;                                  // to replay the flight (bullet time)
bd::PointMassRound round(s.drag, truth, {{0, 0, 0}, bd::frames::directionFromAzEl(0, elevation), s.muzzleSpeed}, o);
while (!round.stopped() && round.position().y < 1000) round.advance(round.time() + 1.0 / 60);
bd::FlightSample mid = round.stateAt(0.5 * round.time());   // any instant flown, smooth between steps
```

The shooter layer turns input into shots and camera offsets; the host flies the shots:

```cpp
#include "bd/shooter.hpp"
#include "bd/variation.hpp"

namespace sh = bd::shooter;
std::string err;
const bd::CartridgeDef& c = *lib.cartridge("federal_xm855");
sh::WeaponSpec w = sh::weaponSpecFor(*lib.rifle("m16a2"), c, *lib.projectile(c.projectile), err);
sh::Weapon gun(w, *lib.stance("standing"), /*seed*/ 42);
std::vector<sh::ShooterEvent> events;
// Every frame: where the player looks, what they press, then the steps due.
gun.setLook(cameraForward);
sh::ShooterInput in;
in.trigger = mouseDown;
in.ads = rightMouseDown;
in.turnRate = turnRate;
in.moveSpeed = speed;
in.mode = 1;  // burst
gun.advance(frameTime, in, events);
for (const auto& e : events)
    if (e.kind == sh::EventKind::Shot) {
        // Fly it with the round's own draw (z1..z4 of the same stream as its recoil).
        const bd::RoundDraw d = bd::drawRound(variation, e.seed, e.stream);
        // ... a PointMassRound from the muzzle along bd::thrownDirection(e.direction, d), at e.time
    }
camera.pitch += gun.viewPitch();  // the kick and the hold
camera.yaw += gun.viewYaw();
viewmodel.pitch += gun.gunPitch();  // the gun in the hands
```

A host with its own weather wraps Aviation's field directly:
`bd::AviationAir air(aviationWeatherField, originAltitudeMsl);`

## Repository layout

```
src/core/       bd_core: the engine-independent projectile core (namespace bd)
src/aviation/   bd_aviation: the adapter to Aviation Dynamics' atmosphere
src/shooter/    bd_shooter: the weapon, the recoil and the aim, on bd_core (ADR 0005)
extern/         aviation-dynamics, pinned (git submodule)
tools/bdemo/    headless tool: the library, range cards, the air, shot strings, the determinism hashes
tools/reference/ the independent-solver reference tables (tests/reference)
tests/          dependency-free test harness and suites
data/           G1/G7 drag tables, projectiles, cartridges, rifles, stances (JSON, with provenance)
web/            C ABI, WASM build, JS loader, smoke test
docs/           decisions (ADRs), physics, validation, fidelity matrix, roadmap, data format
```

## Principles

Shared with the sibling engines:

- **Nothing scripted.** Every shot comes out of the physics; the renderer
  only draws, even in slow motion.
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
