# Roadmap

Milestones follow the brief; each ships with its tests and docs, and none is
skipped. Small PRs, one milestone or less each.

| Milestone | Scope | Status |
|---|---|---|
| 1 | Skeleton: repository, CMake, CI, ADRs (frames and units, integrator, atmosphere source, determinism), JSON loader with provenance, the air from Aviation Dynamics, WASM build | ✅ |
| 2 | Point mass: G1/G7 and custom drag curves; drop, drift, time of flight and energy against published tables for .308 Win 175 gr, 6.5 Creedmoor 140 gr, .338 Lapua 250 gr, .50 BMG 750 gr, 5.56 mm M855, 9 mm 124 gr; convergence of the 1 ms step; whole-trajectory determinism hash; bullet-time interpolation | ✅ |
| 3 | Spin and the Earth: stability, spin drift, aerodynamic jump, Coriolis and Eötvös, the epicycle damping out, transonic instability; against McCoy and published long-range data | ✅ except dynamic (transonic) stability, which is open below |
| 4 | Every round unique: muzzle velocity, BC, imbalance and barrel spread; group sizes; scope and cant; the solver API | ⛔ |
| 5 | World and materials: the segment-query interface, the material handoff ADR, gelatine, wood, steel, ricochet | ⛔ |
| 6 | Scale: structure-of-arrays storage, the 5,000-round benchmark, determinism across native builds and browsers (including Aviation's air) | ⛔ |
| 7 | The range at `/sao-paulo/ballistics/` in the Sao-Paulo repository, with the Sniper Elite-style bullet-time camera ([bullet-time.md](bullet-time.md)) | ⛔ (outside this repository: asked before); the engine side of bullet time is in (Milestone 2) |
| 8 | Glass handoff: the interface PR in Glass, then sniping through glass; Rupert's drop | ⛔ (a PR in the Glass repository: asked before) |

## Open items

- Read the manufacturers' own tables (Federal, Hornady, Lapua, Speer) and
  JBM, unreachable from the build environment so far, and replace the
  secondary sources in docs/validation.md; recheck the Long Range Science
  .308 gap against Federal's table.
- Add a bullet's own Doppler C_D(M) curve to the library (Lapua publishes
  them for the Scenar).

- Measure the bullet lengths (all estimated); Milestone 3 uses them for Sg.
- Dynamic stability (S_d) through the transonic zone, and measured
  aerodynamic coefficients (C_Lα, C_mα, C_lp) per bullet.
- Measure whether Aviation's air is bit-identical across platforms
  (ADR 0004); pick the remedy if not.
