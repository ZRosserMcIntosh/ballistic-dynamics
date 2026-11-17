# ADR 0002: Fixed 1 ms step with RK4

Status: accepted (Milestone 1); the step confirmed in Milestone 2 (below)

## Context

A rifle bullet leaves the muzzle at 700–950 m/s and decelerates at a few
hundred m/s² (a .308 175 gr at the muzzle: about 0.5 ρ v² C_D A / m ≈
600 m/s²), so its velocity changes on a time scale of about a second. Its
yaw dynamics are much faster: the fast (nutation) arm of the epicycle turns
at roughly 1–3 kHz near the muzzle and the slow (precession) arm at
50–200 Hz. Hosts run at whatever rate the browser gives them (30–144 Hz,
with hitches). Many rounds must be stepped at once (5,000 in the browser at
60 Hz is the target), and the result must not depend on the host.

Aviation Dynamics uses RK4 at a fixed 240 Hz (its ADR 0003); Glass Dynamics
picks its internal step from what is happening (50 ns in a bullet strike),
also independent of the host's frames.

## Decision

- **Fixed step `kDefaultStep` = 1 ms**, independent of the host's frame
  rate. Engine time is an integer step count times the step
  (`FixedClock`), never a sum of frame times, so 60 frames of 1/60 s and
  144 frames of 1/144 s end on the same step. Hosts call `advance(t)`, which
  runs the whole steps that fit before t; one call is capped so a hitch slows
  the engine down instead of freezing the host.
- **Classic RK4** (`bd::rk4`) for the translational state of the point-mass
  and modified-point-mass models. At 1 ms a 900 m/s round moves 0.9 m per
  step, and the drag time scale is about a thousand steps: the truncation
  error is far below a millimetre at 1,000 m. Milestone 2 measured it: a
  .308 175 gr G7 round at 1,000 yd lands within 0.1 mm and 1 µs of the same
  round flown at 0.1 ms (`tests/test_point_mass.cpp`).
- **The fast yaw motion is not integrated through at this step.** The
  modified point mass (McCoy, Milestone 3) carries the yaw of repose, which
  varies on the drag time scale; the epicyclic swerve after the muzzle and
  the crosswind jump are added from their closed-form solutions, which are
  exact for any step. A full 6-DoF option, if it comes, will sub-step inside
  the 1 ms frame and keep the same outer clock.
- **Material crossings** (Milestone 5) are not limited by the step: when a
  step's segment crosses a surface, the crossing is resolved at the exact
  point and time inside the step and the step finishes from there.

## Consequences

- `tests/test_foundations.cpp` ("clock: results do not depend on how the
  host chops its frames") steps a body with drag and gravity at 60 Hz,
  144 Hz, one 1 s call and a jittery host with hitches, and checks the four
  end states are equal bit for bit. A fourth-order test checks RK4.
- Rounds fired mid-frame keep their own clock from the launch time, so they
  are frame-independent too (tested at 60 Hz, 144 Hz and a jittery host).
- Rendering between steps (bullet time) uses the Hermite interpolant of the
  step states, not smaller steps (bullet-time.md).
- A round costs at most 1,000 RK4 steps per second of flight; Milestone 6
  measures what that costs for 5,000 rounds and adds the fast path for
  straight stretches between world queries.
