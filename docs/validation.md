# Validation

What the engine gives for each scenario, against the published reference.
When the two disagree the gap is written here; nothing is tuned silently.
The checks run in `tests/test_point_mass.cpp` and `tests/test_air.cpp`; the
numbers are from `./build/bdemo table` and `./build/bdemo air` (Linux, gcc)
and `BD_VERBOSE=1 ./build/tests/bd_tests validation`.

## Point mass: the six cartridges against an independent solver

The brief asks for "JBM-style solutions". The reference is
[py-ballisticcalc](https://github.com/o-murphy/py-ballisticcalc) 3.1.1, an
independent point-mass solver, run as an outside program by
`tools/reference/make_reference.py` into `tests/reference/point_mass.json`.
Conditions: ICAO sea level (15 °C, 1013.25 hPa, dry), 100 yd zero (9 mm:
25 yd), sight height 1.5 in (9 mm: 0.5 in), 10 mph full-value crosswind,
no spin drift or Coriolis. Worst disagreement over every 100 yd row (the
zero rows, where both are ~0, are left out of the relative figures):

| Case | Model, BC | MV fps | Out to | Drop | Wind | Speed | Time |
|---|---|---|---|---|---|---|---|
| .308 Win 175 SMK | G1 0.505 | 2600 | 1000 yd | 0.010 % (0.04 in) | 0.025 % | 0.026 % | 0.008 % |
| .308 Win 175 SMK | G7 0.243 | 2600 | 1000 yd | 0.004 % (0.02 in) | 0.011 % | 0.043 % | 0.003 % |
| 6.5 Creedmoor 140 ELD-M | G1 0.646 | 2710 | 1000 yd | 0.006 % (0.02 in) | 0.016 % | 0.005 % | 0.003 % |
| 6.5 Creedmoor 140 ELD-M | G7 0.326 | 2710 | 1000 yd | 0.005 % (0.01 in) | 0.017 % | 0.010 % | 0.003 % |
| .338 Lapua 250 Scenar | G1 0.648 | 2920 | 1500 yd | 0.007 % (0.06 in) | 0.018 % | 0.028 % | 0.007 % |
| .338 Lapua 250 Scenar | G7 0.320 | 2920 | 1500 yd | 0.005 % (0.02 in) | 0.014 % | 0.017 % | 0.003 % |
| .50 BMG 750 A-MAX | G1 1.050 | 2820 | 2000 yd | 0.007 % (0.10 in) | 0.027 % | 0.023 % | 0.006 % |
| 5.56 mm M855 | G1 0.304 | 3020 | 800 yd | 0.008 % (0.01 in) | 0.017 % | 0.034 % | 0.005 % |
| 9 mm 124 Gold Dot | G1 0.134 | 1150 | 100 yd | 0.058 % (0.01 in) | 0.34 % (0.01 in) | 0.033 % | 0.026 % |

The test bands are 0.1 % (drop), 0.5 % (wind), 0.1 % (speed), 0.05 % (time).
Two independent integrations of the same model agree to a few hundredths of
a per cent; what remains is each solver's air (py-ballisticcalc's ICAO
density ratio is 1.0004) and its step. This shows the engine solves the
point-mass model correctly. It does not show the model is right about real
bullets: that is the published-figures table below and Milestone 3.

## Point mass: published figures

| Load | Published | Source | Engine | Gap |
|---|---|---|---|---|
| 6.5 CM 140 ELD-M, G1 0.646, 2710 fps, 100 yd zero | −3.8 / −13.4 / −29.6 in at 200 / 300 / 400 yd | LoadDevelopment.com 6.5 Creedmoor ballistics chart | −3.77 / −13.42 / −29.65 | within 0.05 in |
| 6.5 CM 140 ELD-M, G7 0.315, 2710 fps, 1.8 in sight | 2.00 / 2.89 / 8.84 mil at 400 / 500 / 1000 yd | Long Range Science, 6.5 Creedmoor chart | 2.01 / 2.92 / 8.97 | +0.5 / +1.0 / +1.5 % |
| .308 175 SMK, G7 0.243, 2600 fps, 10 mph | 11.67 mil drop, 2.99 mil drift at 1000 yd | Long Range Science, .308 Winchester chart | 12.00 mil, 3.12 mil | **+2.8 %, +4 %** |
| .50 BMG 750 A-MAX, 2815 fps, 10 mph | 34 in drift at 1000 yd | Field Ethos, Hornady .50 BMG 750 A-MAX | 35.3 in (at 2820 fps) | +3.7 % |
| 9 mm 124 Gold Dot, 1150 fps | 969 fps at 100 yd | Speer 23618GD | 959 fps with Speer's reloading-data BC 0.134; 974 with the 0.151 Speer also lists | −1.0 % / +0.5 % |

**Open gaps.**

- The Long Range Science .308 chart: the engine has 3–4 % more drop and
  drift at 1,000 yd. The independent solver agrees with the engine to
  0.01 %, so the difference is most likely in the chart's inputs: it does
  not state its atmosphere or sight height. Not tuned; to be rechecked
  against Federal's own table when it can be read.
- The 9 mm: Speer lists two BCs for the same bullet; the published 100 yd
  velocity falls between them.
- The manufacturers' own pages (Federal, Hornady, Lapua, Speer) and JBM were
  not reachable from the build environment, so these figures come from
  secondary pages that quote or compute them; each row names its source.
  Reading the primary tables is an open item ([roadmap](roadmap.md)).
- Hornady now lists 0.610 G1 for the 140 ELD-M (its Doppler re-measurement)
  where the retailers' 81500 data says 0.646. The library keeps 0.646 with
  that note.

**Loads without a published comparison yet.** The .408 CheyTac 419 gr
solid (G1 0.949, 2752 fps, CheyTac) and the 6.5 Creedmoor 147 gr ELD Match
(G7 0.351, 2695 fps, Hornady) are in the library and fly on the same
validated model, but no published drop table for them has been checked. For
the .408 solid, Miller's rule (fitted to lead-core bullets) under-states the
stability of a longer monolithic bullet, and its length is estimated.

## Point mass: numerics

| Check | Expected | Engine |
|---|---|---|
| Vacuum, any angle | the exact parabola (RK4 is exact on a quadratic) | to 1e-9 m after 2 s |
| Constant C_D, no gravity | v = v₀ / (1 + k v₀ t), x = ln(1 + k v₀ t) / k | to 1e-7 m/s, 1e-6 m |
| 1 ms against 0.1 ms, .308 G7 at 1000 yd | converged | drop within 0.1 mm, time within 1 µs |
| Frame chopping: 60 Hz, 144 Hz, a jittery host, fired mid-frame | bit-identical | bit-identical |
| Bullet time: Hermite between 1 ms steps against a 0.05 ms run | smooth | within 20 µm and 0.05 m/s |
| Whole-trajectory golden hash | `0x2ca1ba49af3e55d1` on every platform | Linux gcc, clang, sanitizers, wasm32 equal; macOS and MSVC in CI |
| A BC-scaled G7 and the same bullet's own curve (i × G7) | the same force | to 1e-12 |

## Spin and the Earth (Milestone 3)

Spin drift against Litz's published empirical formula, 1.25 (Sg + 1.2)
TOF^1.83 in (fitted to measured rifle data). The engine's drift comes from
the yaw of repose and estimated C_Lα and inertia, not from the formula.
Standard day, 100 yd zero, no wind, firing north, no Earth rotation:

| Load, twist | Sg (Miller) | Range | TOF | Engine | Litz | Ratio |
|---|---|---|---|---|---|---|
| .308 175 SMK, 1:11.25 | 1.90 | 1000 yd | 1.791 s | 9.4 in | 11.2 in | 0.83 |
| 6.5 CM 140 ELD-M, 1:8 | 1.59 | 1000 yd | 1.496 s | 6.2 in | 7.3 in | 0.85 |
| .338 LM 250 Scenar, 1:10 | 1.92 | 1500 yd | 2.500 s | 19.3 in | 20.9 in | 0.92 |
| .50 BMG 750 A-MAX, 1:15 | 2.04 | 2000 yd | 3.095 s | 31.3 in | 32.0 in | 0.98 |
| 5.56 M855, 1:7 | 2.48 | 600 yd | 0.871 s | 2.8 in | 3.6 in | 0.79 |
| 9 mm 124 GDHP, 1:10 | 11.95 | 100 yd | 0.289 s | 0.7 in | 1.7 in | 0.39 |

**Gap.** The engine is 2–21 % under Litz for the rifles (test band 0.7–1.15)
and far under for the 9 mm, a short pistol bullet outside the formula's
data. The likely cause is the estimated C_Lα, inertia ratio and roll
damping; they are not tuned to close it. Measured aerodynamic coefficients
per bullet (Doppler or spark-range data) would.

| Check | Expected | Engine |
|---|---|---|
| Miller's rule at 2800 fps, 59 °F, 29.92 inHg | the bare rule | to 1e-12 |
| Twice the twist rate | four times Sg | exact |
| Spin at the muzzle, 2600 fps through 1:11.25 | 2,773 turns/s | exact |
| A left-hand twist | the same drift, to the left | to 1 µm |
| Spin decay over 1000 yd | slower than the speed | 2773 → 2112 turns/s (−24 %) while the speed falls 58 % |
| Aerodynamic jump, .308, 10 mph crosswind | Litz: 0.01 Sg − 0.0024 l + 0.032 MOA/mph | the formula, up for a wind from the left, down from the right, opposite for a left-hand twist; ≈ 4 in at 1000 yd |
| Coriolis, vacuum, flat fire north at 45° N | ω sin φ V T² to the right | to 0.1 % |
| Eötvös, vacuum, firing east / west | ± ω cos φ V T² | to 0.1 % |
| Coriolis with drag, .50 BMG at 2000 yd, 45° N | the lateral equation integrated along the plain flight | to 0.3 %: 10.2 in, less than the drag-free ω sin φ X T (11.5 in) |
| São Paulo (23.55° S) | deflected left | left |
| A 175 SMK from a 1:20 twist (Sg 0.6) | unstable | tumbles; much more drop, much slower |
| The wobble | starts at the crosswind yaw, gone in under a second; nose within 1° of the path | yes |
| Spin golden hash | `0x86ee6c5cb23ef28e` on every platform | gcc, clang, sanitizers, wasm32 equal; macOS and MSVC in CI |

## The air

| Scenario | Reference | Engine | Check |
|---|---|---|---|
| Standard day at sea level: 101,325 Pa, 15 °C, dry | ISA (USSA-1976): ρ 1.2250 kg/m³, a 340.29 m/s, density altitude 0 | ρ 1.22500, a 340.29, DA 0 m | ρ ± 0.0005, a ± 0.01, DA ± 1 m |
| 30 °C, 101,325 Pa, dry | Ideal gas: ρ = p/(R_d T) = 1.16439; a = √(1.4 R_d T) = 349.04 | ρ 1.16439, a 349.04, DA 525 m | exact to 1e-9 |
| 30 °C, 101,325 Pa, saturated | Buck: e_s = 4,246 Pa; ρ lower by 0.378 e/p = 1.584 % → 1.14595 | ρ 1.14595 (−1.584 %), a 351.84 m/s, DA 690 m | ratio ± 2e-4; sound faster, DA higher |
| Station 1,609 m, 83,400 Pa, 30 °C, 20 % RH (a hot day at Denver's elevation) | The reading itself; ISA ρ ≈ 0.955 near 2,500 m | p 83,400.0 Pa, T 30.00 °C; ρ 0.9547, DA 2,523 m | station reproduced to 1e-12; DA 2,450–2,650 m |
| The same, 100 m up | Hydrostatics: Δp = ρ g Δz ≈ 936 Pa; ISA lapse 0.65 K | Δp 935.1 Pa; ΔT 0.65 K | ± 5 Pa; ± 0.01 K |
| Aviation's 10 m/s wind from the north, at 10 m | Blows toward the south: ENU (0, −10, 0) | (0, −10.000, 0) | ± 1e-6 |

## Range wind

| Scenario | Expected | Engine |
|---|---|---|
| Firing north, 3 m/s from the west to 400 m, 5 m/s from the east after | +3 m/s east at 100 m; −5 m/s at 700 m; halfway (−1) at 400 m; full values 10 m either side | as expected, continuous through the join |
| Gusting layer, 6 m/s with 1.5 m/s RMS | mean 6, RMS 1.5 over a long record; same seed same bits | within ± 0.15 of both; identical per seed, different between seeds |

## Determinism

| Check | Reference | Engine |
|---|---|---|
| `bd::dm` vs glibc over their ranges | ≤ 1–2 ulp (pow ≤ 16 ulp) | exp 1, log 1, sin/cos ≤ 1 ulp of 1, atan 1, atan2 1, pow 11, cbrt 3 |
| Golden hash of the maths and the RNG | `0x7302405ad0a26843` on every platform | Linux gcc, Linux clang, gcc with sanitizers, wasm32 (wasi-sdk 25, Node 22): equal. macOS arm64 and Windows MSVC: checked by CI |
| The same body stepped at 60 Hz, 144 Hz, one call, a jittery host | Bit-identical end state | Bit-identical |
