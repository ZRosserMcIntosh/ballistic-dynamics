# Fidelity matrix

What is modelled, how, and how far it is validated, as of Milestone 2
(October 2026). Rows cover the whole brief, including what is not started.

Legend: ✅ implemented and tested · 🟡 partial or simplified · ⛔ not started.

| Area | Status | Model today | Validation | Main gap |
|---|---|---|---|---|
| Frames and units | ✅ | SI inside; ENU world; line of fire x downrange, y left, z up; true bearings; Earth rotation vector in ENU ([ADR 0001](decisions/0001-frames-and-units.md)) | `tests/test_foundations.cpp` frame cases | No Earth curvature (flat tangent plane) |
| Atmosphere | ✅ | Aviation Dynamics' USSA-1976 with ΔT, sea-level pressure and humidity, from the pinned submodule; station readings mapped to it exactly ([ADR 0003](decisions/0003-atmosphere-source.md)) | ISA sea level, ideal gas, Buck moist density, hydrostatics, density altitude ([validation.md](validation.md)) | Cross-platform bit-identity of Aviation's libm calls not yet measured |
| Wind along the path | 🟡 | Aviation's field (boundary layer, turbulence, gusts) through `AviationAir`; range-wind layers by distance with seeded gusts and blended joins; call versus truth as two sources | `tests/test_air.cpp` | Range-wind gusts are smooth sines, not a spectrum; no vertical wind in the layers; no mirage |
| Integrator and clock | ✅ | Fixed 1 ms steps as an integer count, RK4; every round on its own clock from its launch ([ADR 0002](decisions/0002-fixed-step-rk4.md)) | Frame-chop independence bit for bit; fourth order; 1 ms within 0.1 mm of 0.1 ms at 1,000 yd | — |
| Determinism | 🟡 | fdlibm-algorithm maths on correctly rounded operations, no contraction, seeded RNG streams, golden hashes of the maths and of a whole trajectory ([ADR 0004](decisions/0004-determinism.md)) | Both hashes on gcc, clang, sanitizers, wasm32 (and macOS, MSVC in CI) | Aviation's air not yet measured across platforms |
| Data and provenance | ✅ | JSON library (drag curves, projectiles, cartridges, rifles), units in field names, a tagged provenance note required for every number at any depth | `tests/test_data.cpp` | Bullet lengths estimated; several figures from secondary pages (manufacturer sites unreachable from the build) |
| WebAssembly | ✅ | wasi-sdk 25, no exceptions or RTTI, C ABI (air, wind layers, zero, fire, advance, bullet-time state, range card), Node smoke test | `web/smoke_test.mjs`: library, both golden hashes, the .308 range card equal to native, bullet time at 1/1000 | — |
| Point mass (G1/G7, custom C_D) | ✅ | Point mass in ENU, drag from BRL G1/G7 scaled by a (stepped) BC or a bullet's own C_D(M), air and wind sampled at every RK stage; zeroing and range tables; bullet-time interpolation | Six cartridges (nine cases) against an independent solver within 0.06 % drop, 0.34 % wind, 0.05 % speed; published figures within 0.05 in to +4 % ([validation.md](validation.md)) | No attitude (Milestone 3); no bullet-specific Doppler curve in the library yet; LRS .308 chart 3–4 % off (inputs unstated) |
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
