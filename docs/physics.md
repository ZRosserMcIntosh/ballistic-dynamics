# The physics

How far each model goes, and its main gap. Milestones 1 to 3 are in: the
air, the clock, the deterministic maths, the point-mass trajectory, spin and
the rotating Earth. The sections for dispersion and materials are listed
with what they will hold, and are marked as not started; nothing below
claims more than the code does. Source: [`src/core`](../src/core), [`src/aviation`](../src/aviation).

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

## 4. Exterior ballistics: the point mass (Milestone 2)

**The equations.** The round is a point with the bullet's mass, in the world
frame (ENU), integrated with RK4 at 1 ms ([ADR 0002](decisions/0002-fixed-step-rk4.md)):

```
dp/dt = v
dv/dt = −K(|u|, a) ρ |u| u + g,      u = v − w(p, t)
```

u is the velocity relative to the air; ρ, the speed of sound a and the wind
w are sampled from the `AirSource` at every Runge-Kutta stage, so the round
flies through air that changes along the path (and with height). g is
standard gravity, (0, 0, −9.80665).

**Drag.** K comes from a drag table, C_D against Mach M = |u| / a,
interpolated linearly:

- A standard table scaled by the ballistic coefficient. With BC = m / (i d²)
  (the published lb/in² × 703.07 = kg/m²) the bullet's C_D is i times the
  reference's, so K = π C_D,ref(M) / (8 BC). The tables are the BRL G1 (79
  points) and G7 (84 points) from Mach 0 to 5 (`data/drag_standard.json`).
  A stepped BC (Sierra's velocity bands) uses the value for the current
  air-relative speed.
- A bullet's own measured curve (Doppler radar): K = C_D(M) A / (2m),
  A = π d²/4. The loader and the model support it; no bullet curve is in the
  library yet.
- The library prefers a bullet's own curve, then G7, then G1.

**Zeroing and range tables.** `zeroElevation` finds the bore angle that
puts the round on the line of sight at the zero range (secant method on real
flights, to 0.1 µm); `rangeTable` flies one round and reports drop below the
line of sight, windage, time, speed, energy and Mach where it crosses each
range (bisection on the Hermite interpolant inside the step).

**Bullet time.** `stateAt(t)` returns the state at any instant already flown
by cubic Hermite interpolation of the step states, continuous in position
and velocity and fourth-order accurate (within 20 µm of a 20× finer run).
A slow-motion camera advances the engine one step ahead of what it draws
and samples in between; the physics is the same at any playback speed
([bullet-time.md](bullet-time.md)).

**Main gap.** The point mass alone has no attitude; section 4b adds what
spin does. Gravity is constant (no
latitude or height variation, 0.5 % at most). A BC-scaled standard table is
only as good as the match between the bullet and the reference shape: a
G1 BC drifts with velocity for boat-tail match bullets (which is why Sierra
steps it); a G7 BC or a bullet's own curve is closer. Stepped BCs jump at
their band edges, as published. Linear C_D interpolation between table
points.

## 4b. Spin and the Earth (Milestone 3)

Off by default (`PointMassOptions::spin`, `earthRotation`), and then the
round is the point mass above bit for bit. Source: `bd/spin.hpp`,
`point_mass.cpp`.

**Stability.** Miller's rule gives Sg at the muzzle from the bullet's mass,
diameter and length, the twist, the muzzle velocity and the air:

```
Sg = 30 m / (t² d³ l (1 + l²)) · (V/2800)^(1/3) · (T_F + 460)/519 · 29.92/P
```

(m grains, t calibres per turn, d inches, l length in calibres; Miller 2005,
with the corrections as in Courtney & Courtney, arXiv:1410.5340). Downrange
Sg follows the physics, Sg ∝ p² / (ρ V² C_mα): it grows as the round slows
(the spin decays more slowly than the speed), and the overturning moment's
rise toward Mach 1 (an estimated trend, `overturningRatio`) takes some of it
back through the transonic zone. Below Sg = 1 the round is gyroscopically
unstable: it is marked tumbling, loses its spin terms and its drag rises
fourfold (estimated).

**Spin.** p₀ = 2π V₀ / twist, signed (+ right-hand); it decays by roll
damping, I_x dp/dt = ½ ρ |u| S d² C_lp p, with I_x = 0.105 m d² and
C_lp = −0.012 (both estimated, McCoy's small-arms range). The roll angle is
integrated for drawing.

**Spin drift: the yaw of repose.** A spinning round whose path curves
(gravity, Coriolis) points its nose slightly to the side of the path (McCoy,
*Modern Exterior Ballistics*, ch. 10): α_R = −(2 I_x p / (ρ S d V⁴ C_mα))
(V × dV/dt). Taking C_mα from Sg itself (ρ S d C_mα = I_x² p² / (2 I_y Sg V²))
leaves

```
α_R = −(4 (I_y/I_x) Sg / p) (u × a_curve) / |u|²,     a_lift = ½ ρ |u|² S C_Lα α_R / m
```

which needs only the inertia ratio (estimated from the length:
0.7 × (⅔ l² + ½)) and the lift slope C_Lα = 2.5 per radian (estimated). For a
right-hand twist the nose sits right of the path and the round drifts right.

**Aerodynamic jump.** A crosswind at the muzzle yaws the round, and its first
swerve turns into a vertical deflection; Litz's published regression gives
it, Y = 0.01 Sg − 0.0024 l + 0.032 MOA per mph of crosswind, up for a wind
from the left with a right-hand twist. It is applied as a turn of the
launch direction.

**The wobble.** After the muzzle the nose circles the path in the epicycle
of a fast (nutation) and a slow (precession) arm,
ω_f,s = (I_x p / 2 I_y)(1 ± √(1 − 1/Sg)), starting from the crosswind yaw
and any tip-off rate, each arm damping out (estimated rates: a few
hundredths and a few tenths of a second). It is computed in closed form for
drawing (`noseAt`); its net effect on the path is the jump.

**The rotating Earth.** a = −2 Ω × v with Ω = ω (0, cos φ, sin φ) in ENU at
latitude φ: the horizontal part deflects right in the northern hemisphere
and left in the southern; the vertical part (Eötvös) lifts a round fired
east and drops one fired west. The centrifugal term is in g.

**Main gap.** C_Lα, the inertia, the roll damping and the C_mα trend are
estimated per bullet class, not measured per bullet; the spin drift lands
within 0.79–0.98 of Litz's measured-data formula without tuning
([validation.md](validation.md)). Dynamic stability (the damping of the
wobble, S_d) is not modelled: a round that is gyroscopically stable but
dynamically unstable through the transonic zone (the real cause of most
transonic trouble) is not caught yet. Litz's jump regression is fitted near
Sg ≈ 1.75. The bullet lengths are estimated.

## 4c. The wake: the trace

What a spotter sees as the trace is the round's wake: a thin tube of
stirred, slower, slightly warmer air left along the path, which bends the
light behind it. `wakeAt` (`bd/wake.hpp`) returns it at any instant as
stretches shed every half metre or so, each where the round was, aged since,
carried by the wind where it is, with the self-similar far wake of an
axisymmetric body (Pope, *Turbulent Flows*, §5.5): with the drag as a
momentum thickness θ = √(C_D A / 2π) and x = U·age the distance the body has
gone on,

```
r½(x) = a θ (x/θ)^(1/3)        u₀/U = b (x/θ)^(−2/3)
```

so a trace widens as the cube root of its age and fades as its two-thirds
power. **Main gap:** the coefficients a = 0.6 and b = 1.2 are estimated at
the order of the measured values; the near wake (the first few hundred
diameters) is held at the bullet's radius; the trace's optical strength is
taken as proportional to the velocity deficit, not computed from the
temperature and density in the wake; no wake vortex shedding or spin swirl;
the wind carrying it is sampled now at where it was shed.

## 5. Every round unique — not started (Milestone 4)

Lot muzzle velocity and spread, powder temperature, barrel length, cold
bore; BC spread, imbalance, barrel vibration; scope, cant and the solver.

## 6. Materials — not started (Milestone 5)

The host's segment query, the material handoff interface, gelatine, wood,
drywall, sheet steel, soil, ricochet; the projectile's own deformation and
breakup.
