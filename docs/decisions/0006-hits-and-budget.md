# ADR 0006: Hits, the 25 m no-lead rule, and the budget

Status: accepted (November 2026)

## Context

A round in flight is physics. What it hits, and what the hit does, is partly
physics and partly game design. *Modern Warfare 2* resolves everything at
once: hitscan, capsule hitboxes and damage tables. That is crisp, cheap and
readable, but it has no flight, no drop, no wind and no material.

This engine flies every round. The questions were:

- where real flight earns its cost;
- where a hitbox and a table are simply better;
- what the whole thing may cost.

Flight time decides how a fight feels, more than drop does. A sprinting
target at 7 m/s moves its own width while a 5.56 round flies 50 m (56 ms),
so the shooter must lead it. A hitscan game never asks for that.

## Decision

- **Hit detection is the host's geometry.** The host answers one question
  (`WorldQuery::segment`): what does this stretch of path cross first?
  Characters and vehicles answer with hitboxes, and the world with its
  surfaces. The engine asks along the flown path and stops the round at the
  first hit (`TrackedRound`), reporting:
  - the time (interpolated inside its 1 ms step from the distance flown,
    within 0.2 µs);
  - the point;
  - the velocity;
  - the distance;
  - the host's body, part and material ids.
- **The 25 m no-lead rule.** While the round is within 25 m of the muzzle
  (path length), the host places the moving bodies where they were *at the
  shot*. Beyond 25 m it places them at the round's own time.
  - Up close, you hit what you aimed at, as in MW2. Farther out you lead, and
    drop and wind are real.
  - The world's static surfaces are the same at any time. Walls stop rounds
    honestly at every range.
  - For a networked server this is one rewind to the shot's time, which it
    already keeps for lag compensation.
  - `resolveNoLead` flies the first 25 m at once, so the frame of the click
    already knows a close hit. The impact keeps its true time (16 ms at 15 m
    for a 5.56).
- **Damage to bodies is a table, not tissue.** The engine hands over the
  impact's velocity and the body part. The host turns that into damage with a
  designer table:
  - base damage;
  - the body part's multiplier;
  - falloff with the energy the round still carries.

  Wound ballistics stay out of gameplay: they would make the time to kill
  noisy and unreadable. Gelatine remains a validation case in Milestone 5,
  not a game rule.
- **Materials are for the world.** Milestone 5's penetration (wood, drywall,
  sheet steel, glass through Glass Dynamics, armour plates) is where physics
  changes play. A wall shot is earned, not scripted.
- **The budget** is measured by `bdemo bench`:

  | Path | Cost |
  |---|---|
  | One RK4 step, exact air (Aviation's model) | about 1.1 µs, three quarters of it the air |
  | One RK4 step, the air column | about 0.29 µs |
  | A 25 m shot (column air) | about 7.5 µs |
  | A 300 m flight (column air) | about 0.11 ms |
  | A game second: 20 players on continuous full auto, every round to the ground or a backstop, steady range wind, world queries | about 33 ms of one core (3 %) |
  | A weapon for one second at 60 Hz | about 13 µs idle, 20 µs firing |

  The fast paths changed no physics:
  - **`ColumnAir`** tabulates the full atmosphere up a vertical column at
    1 m, with a worst density error of 9e-10. A 300 m flight through it lands
    within a micrometre of the exact model's.
  - **A bucket index on the drag tables** finds the same interval as the
    binary search, so the trajectory bits and golden hashes are unchanged.
  - **The world is asked once per 4 ms chord**, not every 1 ms step. The
    path sags under 0.02 mm off the chord. Chords are aligned to the round's
    own steps, so impacts don't depend on the frame rate.
- **Tracers follow the cartridge's family.** A gun's `origin` sets its colour:
  - **western** traces red, like NATO's M856 and M62;
  - **eastern** traces green, as Soviet-bloc tracers (T-45, 7T3) typically
    did.

  A gun can override it. One round in `tracer_every` is a tracer (5 by
  default, the 4 ball : 1 tracer belt mix). Each shot event says whether it
  is a tracer and its colour. The wake (`bd_wake`) is the trace a spotter
  sees, and a tracer adds the burning element on top of it.

## Consequences

- **Tests** (`tests/test_world.cpp`, `tests/test_shooter.cpp`):
  - the wall crossing against the flight's own crossing at 1-, 4- and
    16-step chords;
  - a 10 cm body crossing at 7 m/s: hit without lead at 20 m, missed without
    the rule, and missed at 50 m unless led;
  - the frame of the click resolving a 15 m hit;
  - impacts identical at 1, 60 and 144 Hz;
  - the column's accuracy;
  - the drag index bit for bit against the binary search;
  - tracer colours and the mix.
- **Fast versus exact air.** A game builds its air as `StationAir` (or
  Aviation's weather), then `ColumnAir`, then `RangeWind`. Validation and the
  range cards keep the exact `StationAir`. The browser build still flies the
  exact air, and a switch is an open item.
- **Open costs:**
  - **Gusting range winds** evaluate their sines on every sample, which adds
    about 0.38 µs a step. That is the next thing to make cheaper.
  - **Spin and Earth rotation** add about 0.3 µs a step, and are off by
    default.
  - **The host's raycast** is its own cost, once per 4 ms of flight per
    round.
- **No-lead and time sources.** The no-lead range is measured along the
  path, so a round fired from a moving vehicle uses its own path too. The
  shot time comes from the round's launch.
