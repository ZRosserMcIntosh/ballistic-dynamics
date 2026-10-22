# ADR 0001: Frames and units

Status: accepted (Milestone 1)

## Context

Ballistic Dynamics sits next to three engines that already chose their
frames: Aviation Dynamics (body FRD, local NED, world ENU; its ADR 0002),
OpenRoad Dynamics (body ISO 8855: x forward, y left, z up) and Glass Dynamics
(pane mounts in both hosts' body frames). A round fired from a car or a
helicopter, through a windscreen, onto a São Paulo World tile has to cross
all of them without a sign slip.

## Decision

- **SI everywhere inside the core.** Conversions live in `bd/units.hpp` and
  happen only at the boundaries: JSON fields carry their unit in the name
  (`_gr`, `_in`, `_fps`, `_inHg`), and the solver API (Milestone 4) can
  report in the shooter's units (MOA, MRAD, inches, yards). Ballistic
  coefficients are stored as SI mass per area (kg/m²); the published lb/in²
  value is multiplied by 703.0696.
- **The world frame is ENU** (x east, y north, z up, metres), the same as
  Aviation's `world_frame.hpp` and the São Paulo World tiles. Positions,
  velocities and the wind that the host sees are ENU.
- **The line-of-fire frame** is x downrange (horizontal, along the azimuth),
  y left, z up. It has ISO 8855's axis order, so a rifle carried by an
  OpenRoad vehicle maps without sign changes. Drop is −z; drift to the
  right is −y. Reports turn these into the shooter's "drop" and "right"
  with the sign written out.
- **Bearings are true, clockwise from north**; elevation is up from the
  horizontal; a wind's direction is where it blows *from* (Aviation's
  convention, so a mission can pass the same weather to both engines).
- **The trajectory is integrated in the world frame on a flat local
  tangent plane.** The rotating-Earth terms (Coriolis, Eötvös, centrifugal) are added as
  accelerations with Earth's rotation vector expressed in ENU,
  Ω = ω (0, cos φ, sin φ), from `frames::earthRotationEnu` (Milestone 3).
  The plane leaves out the Earth's curvature: gravity keeps pointing down
  the same axis instead of turning toward the Earth's centre, and the ground
  does not fall away under the round (x²/2R: 0.08 m at 1 km, 0.71 m at
  3 km). That is below a 1,000 m shot's other uncertainties but not below a
  3 km one; a later milestone may add curvature for long shots, and this
  ADR will say so when it does.
- **The projectile's body frame** (Milestone 3) is x along the axis, nose
  forward, right-handed; spin is positive about +x for a right-hand twist.

## Consequences

- One frame for the host to deal with (ENU) and one documented frame for the
  shooter (line of fire).
- `tests/test_foundations.cpp` checks the frame maps (azimuth and elevation
  both ways, line of fire right-handed, east-firing axes) and the Earth's
  rotation vector; `tests/test_air.cpp` checks that Aviation's NED wind
  arrives in ENU with the right sign (a north wind blows toward −y).
