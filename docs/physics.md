# The physics

How far each model goes, and its main gap. Milestones 1 to 3 are in: the
air, the clock, the deterministic maths, the point-mass trajectory, spin and
the rotating Earth, with round-to-round variation from Milestone 4 and the
shooter layer (section 7, ADR 0005). Materials are listed with what they will
hold and marked as not started. Nothing below claims more than the code does.
Source: [`src/core`](../src/core), [`src/aviation`](../src/aviation),
[`src/shooter`](../src/shooter).

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
flights, to within 10 µm); `rangeTable` flies one round and reports drop below the
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
it, Y = 0.01 Sg − 0.0024 l + 0.032 MOA per mph of crosswind. With a
right-hand twist it is down for a wind from the left and up for one from the
right: the nose yawed right of the airflow by a wind from the left is turned
by the overturning moment, and the spin precesses it clockwise as seen from
behind, to down; the lift on that yaw is the jump (the benchrest "10 o'clock
to 4" stringing). It is applied as a turn of the launch direction. (Before
the audit of October 2026 the sign was the wrong way round.)

**The wobble.** After the muzzle the nose circles the path in the epicycle
of a fast (nutation) and a slow (precession) arm,
ω_f,s = (I_x p / 2 I_y)(1 ± √(1 − 1/Sg)), both arms turning the way the
round spins (clockwise from behind for a right-hand twist), starting from the crosswind yaw
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
power. The width coefficient a = 0.6 is estimated at the order of measured
values; b follows from momentum (a Gaussian deficit profile carrying the
drag needs a²b = ln 2, so b = 1.93).

**What bends the light** is the wake's heat, not its slowness. The drag's
work stays in the wake as heat (ρ c_p ∫ΔT dA = D per metre, as
ρ U ∫u dA = D carries its momentum), so with the same profile the centreline
excess is

```
ΔT/T = (u₀/U) (γ − 1) M²          index = ΔT/T · ρ / 1.225
```

and the refractive index there falls by `index` × 2.9e-4 (n − 1 scales with
density). A fast round's wake is far hotter for its momentum: ΔT/T is 2.1×
the deficit for a .308 at Mach 2.3, 0.4× for a 9 mm at Mach 1, 0.26× for a
.45 at Mach 0.8. Above Mach 1 the shocks carry the wave drag off as sound
(the crack); the wake is given only the drag the shape has at Mach 0.8
(friction and base, estimated), C_D(0.8)/C_D(M) of it: about 0.4 for a
boat-tail at Mach 2.3.

**Main gap:** a and the wake's subsonic share of the drag are estimated;
the near wake (the first ~15–20 diameters, until the far-wake width passes
the bullet's) is held at the bullet's radius; the heat is taken to have the
deficit's profile; the shock waves' own (small, short-range) refraction is
not in the wake; no wake vortex shedding, spin swirl, buoyancy, or shredding
by the air's own turbulence; the wind carrying it is sampled now at where it
was shed.

## 5. Every round unique (Milestone 4, in part)

`bd/variation.hpp`. Each round draws its own muzzle velocity, BC and barrel
throw from its own seeded stream (seed, stream), so the same pair is the
same round on every machine (a shared link, a duel):

```
v0    = v_published + k (T_powder − T_reference) + σ_v z1
BC    = BC_published (1 + σ_BC z2)
bore += (σ_a z3 up, σ_a z4 right),   σ_a = group / 3.08
```

z1..z4 independent standard normals, always drawn in that order (switching
one cause off leaves the others' draws alone). σ_a comes from the rifle's
mean 5-shot group: for a circular normal the expected 5-shot extreme spread
is 3.08 σ (Grubbs). The zero stays at the published velocity (the zeroing
day); the powder's temperature moves the flight. `spreadAt` gives the
spread of impacts at a range by cause, linearised (a second flight with v0
+1 m/s, another with the BC +1 %, and the throw times the range): for a .308
175 SMK at 1,000 yd with 12 fps, 0.75 % BC and a 0.75 MOA rifle it predicts
6.83 in vertical and 2.55 in horizontal, and 400 flown rounds give 6.81 and
2.58 (`tests/test_variation.cpp`).

**Main gap:** every spread figure in the library is estimated (makers do not
publish them); the throw is a circular normal (no barrel-harmonic vertical
stringing, no cold-bore shot, no heating string); no lateral throw from
bullet imbalance beyond the rifle's group; the scope (cant, parallax,
tracking error) and the solver API are still to come.

## 6. Materials — not started (Milestone 5)

The host's segment query, the material handoff interface, gelatine, wood,
drywall, sheet steel, soil, ricochet; the projectile's own deformation and
breakup.

## 7. The shooter layer (ADR 0005)

`bd/shooter.hpp`, in its own library (`bd_shooter`). It covers what happens
between the trigger and the launch, and what the shooter feels after it. The
core still only flies rounds. The layer hands it shot events (the bore's
direction, plus the round's seed and stream) and gives the host the camera and
viewmodel offsets.

**The weapon.**

- **Clock.** States run on the 1 ms clock, counted as integers.
  - The action's next readiness is held in microseconds, and the remainder is
    carried. An automatic fires at exactly its cyclic rate on average, on the
    first step at or after each readiness, at any frame rate.
  - A press given with an advance call fires on that call's first step.
- **Fire modes:** semi (re-arms on release), auto, burst *n* (the burst
  completes after release), and manual (bolt, lever or pump, at its
  `manual_cycle_s`).
- **Magazine.** Pulling the trigger on an empty magazine clicks once and
  starts the reload. A reload takes `reload_s` and refills the magazine; the
  host keeps the reserve.
- **Sprint.** A sprint holds the gun down, and `sprint_out_s` must pass before
  it fires again.
- **Sight.** The sight fraction moves at 1/`ads_s` per second.

**Free recoil (physics).** The bullet and the powder gas leave with momentum

```
p = m_b v0 + m_c v_gas
```

and the free gun takes `V = p / M` and `E = ½ M V²`. `v_gas` follows the
free-recoil tables' convention: 4,700 fps for rifle cartridges here, and
4,000 fps estimated for handgun cartridges. Powder charges are estimated from
handloading practice, because factories don't publish them.

**The kick.** The recoil acts along the bore, a height `h` (`bore_lever_in`)
above where the shooter takes it. Its angular impulse `p h` turns the gun and
the arms and upper body moving with it:

```
ω0 = p h / (M k² + I_hold)
```

- `k` (`gyration_in`) is the gun's pitch radius of gyration about its centre
  of mass.
- `I_hold` is the stance's.

A straight-line stock (an AR: `h` ≈ 0.6 in) climbs less for its momentum than
a dropped one (a Thompson: about 1.9 in). That comes out of the geometry, not
a per-gun recoil table.

On each shot `ω0` is multiplied by:

- the hip factor, lerped by the sight fraction;
- a braced reduction of `1 − r s`, where `s` rises from 0 to 1 over
  `settle_s` after a shot (a settled shooter takes the first shot of a string
  better);
- `1 + σ z5` from the round's stream.

The yaw rate is `ω0 σ_yaw z6`.

The view then integrates, per axis and per step:

1. A constant pull toward the aim (`recenter`).
2. Motion back toward the aim is slowed to `return_percent` of the rate.
3. Passing the aim stops it dead.
4. It is clamped at `kick_clamp_deg`.

The rise from one shot is the closed form of a constant deceleration,
`ω0² / 2a` at `ω0 / a`. The test checks the integration against it to 2 %.

- **M24 at the sight, standing:** a .308 shot gives about 19°/s, peaks at
  1.2° after 126 ms, and is back on the aim with no overshoot.
- **M16A2 three-round burst** (`bdemo shoot m16a2 federal_xm855 standing 1`):
  climbs about 0.37° and settles in about 470 ms.

The gun's own motion in the hands is a visual-only spring (rate, pull, viscous
damping, cap) driven by the same shots. It never moves the camera or the
bore.

**Aim.** A hip accumulator, from 0 to 1:

- **Opens with:** each hip shot, every degree turned, every metre moved, and
  each second off the ground.
- **Closes:** at `bloom_recovery_per_s`, but only while nothing opens it.

The hip cone's half-angle runs from `hip_cone_min` to `hip_cone_max` with
it. Only *fully* at the sight (a binary gate, as in the IW engine) is the cone
just `hip_cone_max × bloom`. Nothing adds to the bloom there, so it drains to
zero: a laser.

A shot's place in the cone is `u9` (radius) and `u10` (angle) of its stream,
weighted to the centre.

The hold's wander at the sight is two incommensurate sines per axis near a
breath's period, scaled by the sight fraction. It is part of the camera
offset, so the bore goes where the camera shows.

**Draws.** A shot draws z5..z8, u9 and u10 from its round's stream, after
`bd::drawRound`'s z1..z4. One `(seed, stream)` is the whole round, its recoil
and its cone, on every machine.

**Main gap:**

- **The response is not measured.** The shooter's response (`I_hold`, the
  pull, the return, the hip factor, the braced reduction, every bloom and
  cone figure) is tuned for play and tagged `estimated:`. Muzzle rise against
  high-speed footage of real shooters is an open item.
- **The rigid-body lever model is simple.** It has no shoulder compliance, no
  translation of the shooter, and no time-resolved barrel recoil. It
  under-states handgun flip, where the wrist is the pivot.
- **No muzzle brakes or compensators.** The M107's 101 ft lb is unbraked.
- **No suppressors**, which change the gas term.
- **One hold inertia per stance for every gun.**
- **Gun masses and handling times** are estimated from commonly quoted
  figures, not checked against manuals from the build environment.
- **Hit feedback waits for Milestone 5's world query.**
