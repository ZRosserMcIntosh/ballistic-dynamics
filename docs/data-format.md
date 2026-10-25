# Data format

The library is JSON: `data/drag_standard.json` (the G1 and G7 tables),
`data/projectiles.json`, `data/cartridges.json`, `data/rifles.json`. A document holds any
of the sections `drag_curves`, `projectiles`, `cartridges`, `rifles`, plus an
optional `comment`.

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

## Records

**drag_curves**: `id`, `name`, `family` (`G1`, `G7` for a standard
projectile scaled by a BC, `bullet` for one bullet's own curve), `mach` (strictly
ascending, not negative; below the first point the first C_D holds) and `cd` (positive), the same length, at least two points.

**projectiles**: `id`, `name`, `construction` (text), `diameter_in`,
`mass_gr`, `length_in`; at least one of `bc_g1` / `bc_g7` (one value, or
stepped values with `bc_g1_min_fps` descending to 0: each value applies at
and above its speed, as Sierra publishes them) or `drag_curve` (the id of a
`bullet` curve).

**cartridges**: `id`, `name`, `projectile` (an id), `muzzle_velocity_fps`
and the `test_barrel_in` it was measured from.

**rifles**: `id`, `name`, `twist_in` (inches per turn), `twist_direction`
(`right` or `left`, default right), `barrel_in`, `sight_height_in`
(optional).

The standard tables must have the ids `g1` and `g7`; a projectile with a
`bc_g7` or `bc_g1` is flown on them.
