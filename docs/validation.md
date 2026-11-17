# Validation

What the engine gives for each scenario, against the published reference.
When the two disagree the gap is written here; nothing is tuned silently.
Milestone 1 has only the air to check; drop, drift, time of flight and energy
tables arrive with Milestone 2. The checks run in `tests/test_air.cpp`; the
numbers below are from `./build/bdemo air` (Linux, gcc).

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
