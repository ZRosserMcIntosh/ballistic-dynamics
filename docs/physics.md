# The physics

How far each model goes, and its main gap. Milestone 1 is the skeleton:
only the air, the clock and the deterministic maths are in. The sections for
the trajectory, spin, dispersion and materials are listed with what they will
hold, and are marked as not started; nothing below claims more than the code
does. Source: [`src/core`](../src/core), [`src/aviation`](../src/aviation).

## 1. The air

**What it is.** Aviation Dynamics' atmosphere and weather, compiled from the
pinned submodule ([ADR 0003](decisions/0003-atmosphere-source.md)):

- U.S. Standard Atmosphere 1976 to 86 km, with a temperature deviation that
  shifts the whole profile and a sea-level pressure; pressure integrated
  hydrostatically with the actual temperature.
- Humidity: vapour pressure e = RH · e_s(T) (Buck 1996), moist density
  ρ = (p − e)/(R_d T) + e/(R_v T), which is lighter than dry air at the same
  p and T (by 0.378 e/p, the virtual-temperature effect).
- Speed of sound a = √(γ R_d T_v) with the virtual temperature
  T_v = p / (ρ R_d) and γ = 1.4.
- Density altitude: the ISA altitude with this density, by inverting the
  standard profile.
- Wind: Aviation's log-law boundary layer over the surface roughness, Ekman
  veer, winds aloft, MIL-HDBK-1797 turbulence, gusts and the rest of its
  field, sampled at the round's position and time.

**What this engine adds.**

- `StationAir`: a shooter's kestrel reading (station pressure, temperature,
  humidity, elevation) becomes Aviation's conditions: ΔT from ISA at the
  station, and the sea-level pressure that makes the hydrostatic profile pass
  through the station pressure. The reading is reproduced exactly at the
  station (to 1e-12), and the air above and below follows the standard lapse.
- `RangeWind`: wind in stretches along the line of fire (the way flags are
  read), each with a speed, a direction, seeded gusts (RMS speed and a
  direction wander, three incommensurate sines around a period), blended
  with a smoothstep over 20 m at each join. It replaces or adds to the base
  air's wind. The shooter's call and the true wind are two `RangeWind`s.

**Main gap.** Range-wind gusts are smooth sines, not a turbulence spectrum;
for turbulence use Aviation's field (`AviationAir` over a `SimpleWeather`
with turbulence on) instead of or under the layers. No vertical wind in the
layers. No mirage model yet (it belongs to the range page, Milestone 7, as a
read of the near-ground wind). The cross-platform bit-identity of Aviation's
air is not yet measured ([ADR 0004](decisions/0004-determinism.md)).

## 2. Time and integration

Fixed 1 ms steps counted as integers, RK4 for the translational state
([ADR 0002](decisions/0002-fixed-step-rk4.md)). The step and the host's
frames are independent (tested bit for bit at 60 Hz, 144 Hz, one call, and a
jittery host).

## 3. Determinism

Elementary functions from fdlibm's algorithms on `+ − × ÷` only, no FMA
contraction, seeded xoshiro256** streams, and a golden hash that every CI
platform and the WebAssembly build must reproduce
([ADR 0004](decisions/0004-determinism.md)). Main gap: the air from Aviation
uses the C library.

## 4. Exterior ballistics — not started (Milestone 2–3)

Point mass with G1/G7 tables scaled by the ballistic coefficient, or a
bullet's own C_D(Mach); then McCoy's modified point mass: Miller stability
with its velocity and density corrections, yaw of repose and spin drift,
aerodynamic jump, the epicyclic swerve damping out, transonic instability,
Coriolis and Eötvös.

## 5. Every round unique — not started (Milestone 4)

Lot muzzle velocity and spread, powder temperature, barrel length, cold
bore; BC spread, imbalance, barrel vibration; scope, cant and the solver.

## 6. Materials — not started (Milestone 5)

The host's segment query, the material handoff interface, gelatine, wood,
drywall, sheet steel, soil, ricochet; the projectile's own deformation and
breakup.
