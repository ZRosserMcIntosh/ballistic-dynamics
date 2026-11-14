# Data format

The library is JSON: `data/drag_standard.json` (the G1 and G7 tables),
`data/projectiles.json`, `data/cartridges.json`, `data/rifles.json`,
`data/stances.json`. A document holds any of the sections `drag_curves`,
`projectiles`, `cartridges`, `rifles` and `stances`, plus an optional
`comment`.

## Provenance

**Every number carries a note**, in the `provenance` object of the object
that holds it:

```json
{
  "id": "smk_308_175",
  "mass_gr": 175,
  "length_in": 1.24,
  "provenance": {
    "mass_gr": "published: Sierra Bullets catalogue, #2275",
    "length_in": "estimated: about 1.24 in as handloaders measure it; ..."
  }
}
```

A note starts with its kind and says more after it:

| Kind | Means | Must say |
|---|---|---|
| `published:` | from a published source | the source (catalogue, standard, paper, page) |
| `measured:` | measured by us or a named person | what, how, by whom |
| `estimated:` | reasoned, not measured or published | the reasoning |

The loader rejects a record (and keeps the others) when a number or an
array of numbers has no note, when a note has no kind or nothing after it,
or when a note names a field that is not there. Nested objects carry their
own `provenance`. Arrays share one note.

## Units

Units are in the field names and converted to SI on load:

| Suffix | Unit | SI |
|---|---|---|
| `_gr` | grain | kg (× 64.79891e-6) |
| `_in` | inch | m (× 0.0254) |
| `_fps` | foot per second | m/s (× 0.3048) |
| `bc_g1`, `bc_g7` | lb/in² | kg/m² (× 703.0696) |
| `_f` | degree Fahrenheit | K |
| `_fps_per_f` | ft/s per °F | m/s per K (× 0.3048 × 1.8) |
| `_percent` | per cent | fraction (× 0.01) |
| `_lb` | pound (mass) | kg (× 0.45359237) |
| `_rpm` | rounds per minute | rounds per second (÷ 60) |
| `_s` | second | s |
| `_deg`, `_deg_per_s2` | degree, degree per s² | rad, rad/s² |
| `_moa` | minute of angle | rad |
| `_kg_m2` | kg m² | kg m² |
| `_per_deg_turned` | per degree | per rad |
| `_per_m_moved`, `_per_s`, `_per_s_airborne`, `_per_shot` | per metre, per second, per shot | as is |

## Records

**drag_curves**: `id`, `name`, `family` (`G1`, `G7` for a standard
projectile scaled by a BC, `bullet` for one bullet's own curve), `mach` (strictly
ascending, not negative; below the first point the first C_D holds) and `cd` (positive), the same length, at least two points.

**projectiles**: `id`, `name`, `construction` (text), `diameter_in`,
`mass_gr`, `length_in`; at least one of `bc_g1` / `bc_g7` (one value, or
stepped values with `bc_g1_min_fps` descending to 0: each value applies at
and above its speed, as Sierra publishes them) or `drag_curve` (the id of a
`bullet` curve). Optional: `bc_sd_percent`, the BC's shot-to-shot standard
deviation.

**cartridges**: `id`, `name`, `projectile` (an id), `muzzle_velocity_fps`
and the `test_barrel_in` it was measured from. Optional: `velocity_sd_fps`
(shot-to-shot standard deviation), `temp_coefficient_fps_per_f` and
`temp_reference_f` (how the velocity follows the powder's temperature, and the
temperature the published figure is for; the coefficient needs the
reference). For the shooter layer (ADR 0005): `charge_gr` and
`gas_velocity_fps` together, the powder's mass and its gas's exit speed in
the free-recoil convention.

**rifles**: `id`, `name`, `twist_in` (inches per turn), `twist_direction`
(`right` or `left`, default right), `barrel_in`, `sight_height_in`
(optional), `group_moa` (optional: the mean 5-shot group, extreme spread in
MOA, of the gun with its load). For the shooter layer (ADR 0005), all
optional:

- `mass_lb`: the gun as fired, loaded, with its optic.
- `cyclic_rate_rpm`.
- `fire_modes`: an array of `semi`, `auto`, `burst2`–`burst4` or `manual`.
  Every mode but `manual` needs a cyclic rate; `manual` needs
  `manual_cycle_s`.
- `magazine_rounds`.
- `reload_s`, `ads_s`, `sprint_out_s`.
- `bore_lever_in`: the bore's height over where the shooter takes the recoil.
- `gyration_in`: the gun's pitch radius of gyration about its centre of mass.

**stances**: `id`, `name`, and the shooter's body in that position (all
required):

- **Taking the recoil:** `hold_inertia_kg_m2`, `recenter_deg_per_s2`,
  `return_percent`, `kick_clamp_deg`, `kick_pitch_sd_percent`,
  `kick_yaw_sd_percent`, `hip_kick_percent`, `braced_reduction_percent`,
  `settle_s`.
- **The gun in the hands:** `gun_kick_percent`, `gun_kick_accel_deg_per_s2`,
  `gun_kick_decay_per_s`, `gun_kick_max_deg`.
- **The hip cone:** `hip_cone_min_moa`, `hip_cone_max_moa`, `bloom_per_shot`,
  `bloom_per_deg_turned`, `bloom_per_m_moved`, `bloom_per_s_airborne`,
  `bloom_recovery_per_s`.
- **The hold at the sight:** `hold_wobble_moa`, `wobble_period_s`.

docs/physics.md section 7 says what each one does.

The standard tables must have the ids `g1` and `g7`; a projectile with a
`bc_g7` or `bc_g1` is flown on them.
