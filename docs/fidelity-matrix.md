# Fidelity matrix

What is modelled, how, and how far it is validated, as of Milestone 1
(October 2026). Rows cover the whole brief, including what is not started.

Legend: ✅ implemented and tested · 🟡 partial or simplified · ⛔ not started.

| Area | Status | Model today | Validation | Main gap |
|---|---|---|---|---|
| Frames and units | ✅ | SI inside; ENU world; line of fire x downrange, y left, z up; true bearings; Earth rotation vector in ENU ([ADR 0001](decisions/0001-frames-and-units.md)) | `tests/test_foundations.cpp` frame cases | No Earth curvature (flat tangent plane) |
| Atmosphere | ✅ | Aviation Dynamics' USSA-1976 with ΔT, sea-level pressure and humidity, from the pinned submodule; station readings mapped to it exactly ([ADR 0003](decisions/0003-atmosphere-source.md)) | ISA sea level, ideal gas, Buck moist density, hydrostatics, density altitude ([validation.md](validation.md)) | Cross-platform bit-identity of Aviation's libm calls not yet measured |
| Wind along the path | 🟡 | Aviation's field (boundary layer, turbulence, gusts) through `AviationAir`; range-wind layers by distance with seeded gusts and blended joins; call versus truth as two sources | `tests/test_air.cpp` | Range-wind gusts are smooth sines, not a spectrum; no vertical wind in the layers; no mirage |
| Integrator and clock | ✅ | Fixed 1 ms steps as an integer count, RK4 ([ADR 0002](decisions/0002-fixed-step-rk4.md)) | Frame-chop independence bit for bit; fourth order | The 1 ms step is to be confirmed by Milestone 2's convergence test |
| Determinism | 🟡 | fdlibm-algorithm maths on correctly rounded operations, no contraction, seeded RNG streams, golden hash ([ADR 0004](decisions/0004-determinism.md)) | Golden hash on gcc, clang, sanitizers, wasm32 (and macOS, MSVC in CI) | Only the core's maths so far; whole-trajectory hashes come with Milestone 2 |
| Data and provenance | ✅ | JSON library (drag curves, projectiles, cartridges, rifles), units in field names, a tagged provenance note required for every number at any depth | `tests/test_data.cpp` | One projectile, cartridge and rifle so far; the bullet length is estimated |
| WebAssembly | ✅ | wasi-sdk 25, no exceptions or RTTI, C ABI, Node smoke test | `web/smoke_test.mjs` (library, air, golden hash) | No trajectory to run yet |
| Point mass (G1/G7, custom C_D) | ⛔ | — | — | Milestone 2 |
| Spin, stability, spin drift, jump | ⛔ | — | — | Milestone 3 |
| Coriolis and Eötvös | ⛔ | Rotation vector only | — | Milestone 3 |
| Transonic instability, tumbling | ⛔ | — | — | Milestone 3 |
| Round-to-round variation | ⛔ | RNG streams only | — | Milestone 4 |
| Scope, cant, solver | ⛔ | — | — | Milestone 4 |
| World query and material handoff | ⛔ | — | — | Milestone 5 |
| Gelatine, wood, steel, soil, ricochet | ⛔ | — | — | Milestone 5 |
| Scale (SoA, 5,000 rounds) | ⛔ | — | — | Milestone 6 |
| Browser range | ⛔ | — | — | Milestone 7 |
| Glass handoff | ⛔ | — | — | Milestone 8 |
