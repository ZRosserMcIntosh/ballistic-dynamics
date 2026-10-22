# ADR 0003: The air comes from Aviation Dynamics, through an interface

Status: accepted (Milestone 1)

## Context

A round needs the air along its path: density (drag), the speed of sound
(Mach, and so the drag curve and the transonic zone), temperature and
pressure, humidity, and the wind. Aviation Dynamics already has a tested
U.S. Standard Atmosphere 1976 with temperature and pressure deviations and
humidity (moist density from the virtual temperature, density altitude),
and a weather field with a boundary-layer wind, MIL-HDBK-1797 turbulence,
gusts and spatial variation (`src/core/include/avd/atmosphere.hpp`,
`wind.hpp`). The brief says to use it rather than write a second one, and
not to copy and fork its code. It offered two ways:

1. a small shared header both engines consume, or
2. a host-supplied air interface that Aviation's field can implement.

Option 1 means moving code out of Aviation into a new shared place and
changing Aviation to consume it: a change in another repository, for a
benefit (one header instead of three source files) that does not need it
yet. Option 2 alone leaves the engine's own tests, tools and browser build
without real air unless something supplies it.

## Decision

Both halves of option 2, with Aviation's own code as the supplier:

- **The core depends only on `bd::AirSource`** (`bd/air.hpp`): `sample(
  positionEnu, time)` returns pressure, temperature, moist density, speed of
  sound, humidity, density altitude and the ENU wind. The core owns no
  atmosphere model. Any host can implement it: Aviation's field, a mission
  script, a server that sends the weather.
- **Aviation's sources are compiled unchanged from a pinned git submodule**
  (`extern/aviation-dynamics`, `atmosphere.cpp`, `wind.cpp`, `table.cpp`;
  they depend only on Aviation's header-only maths). They build into
  `bd_avd_air`; nothing is copied into this repository.
- **The adapter `bd::aviation`** (`src/aviation/`) is the only code that
  sees both engines' types. `AviationAir` wraps any `avd::AtmosphereField`
  (NED → ENU, origin altitude); `StationAir` builds Aviation's
  `SimpleWeather` from a shooter's station reading (station pressure,
  temperature, humidity, elevation): it sets the temperature deviation from
  ISA at the station and the sea-level pressure whose hydrostatic profile,
  with that temperature, passes through the station pressure (exact: the
  profile is linear in the sea-level pressure).
- **Range wind is ballistics-specific and lives in the core**: `RangeWind`
  lays wind layers along the line of fire over any base `AirSource` (the
  way a shooter reads flags in stretches), with seeded gusts and a smooth
  blend at the joins, replacing or adding to the base wind. A mission feeds
  the shooter's call and the true wind as two `RangeWind`s over the same
  air.

## Consequences

- One atmosphere in the project. A fix in Aviation reaches this engine by
  bumping the submodule, a deliberate commit that CI tests.
- CI checks out submodules; a clone needs `git submodule update --init`.
  CMake stops with that instruction if the submodule is missing.
- The browser build compiles the same three Aviation files.
- Aviation's atmosphere calls the C library's `exp`, `pow` and `log`, which
  are not bit-identical across platforms. ADR 0004 says what that means for
  determinism and how Milestone 6 deals with it.
- If a third engine needs the air, or Aviation wants it, option 1 can still
  be done later: the adapter is the only place that would change.
