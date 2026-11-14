# ADR 0005: The shooter layer

Status: accepted (November 2026)

## Context

The core flies rounds, but a game needs more than that between the trigger
and the launch. It needs the weapon's cycle and magazine, the recoil, the
shooter's aim, and the cues the player feels on the frame they click. The
reference for how that should *feel* is the IW engine's weapon model (*Modern
Warfare 2*, 2009). We read it in the open-source IW4L rewrite to learn the
mechanisms. No code or data was taken; the shapes below are reimplemented
here from their description. Six of its choices make its gunplay crisp:

1. The shot leaves on the command of the click: no wind-up, and feedback on
   that frame.
2. Recoil is an angular-velocity impulse on the view. A constant pull brings
   it back, the motion back is slowed (to 6 % of the rate), and the return
   stops dead at the aim. It never overshoots or oscillates. A separate,
   visual-only spring moves the gun in the hands.
3. At the sight, firing, turning and moving add no spread. From the hip, an
   accumulator opens a cone between a minimum and a maximum. The gate is
   binary: only fully at the sight counts as aiming.
4. The first shots of a string kick less.
5. Animations are time-stretched to the gameplay timers.
6. Layered hit feedback.

Two IW choices are kept out on purpose:

- Its weapon timers drop the leftover milliseconds, so the rate of fire
  depends on the frame rate.
- In the rewrite we read, the shooter's own kick and fire cues wait for a
  server snapshot, a round trip after the click.

## Decision

- **A separate library, `bd_shooter`** (`src/shooter/`, namespace
  `bd::shooter`), on `bd_core`. The core stays a projectile engine. The
  shooter layer turns input into shot events: the bore's direction, plus the
  `(seed, stream)` of the round's own draws. The host flies those rounds with
  the core (`bd_fire_shot` in the browser).
- **The same clock.** It runs whole 1 ms steps counted as integers (ADR 0002).
  - **Exact fire rate.** The action's next readiness is kept in microseconds
    and carries its remainder. 700 rpm fires at 85 or 86 ms intervals
    averaging 85.714 ms, at any frame rate. Hosts that chop frames
    differently get the same shots bit for bit.
  - **Shots on the click frame.** Input given with an advance call holds for
    all of its steps, so a press fires on that call's first step. The shot
    and its kick belong to the frame of the click.
- **Recoil from momentum; the response from a person.**
  - **Free-recoil momentum:** `p = m_b v0 + m_c v_gas`. Muzzle brakes are not
    modelled.
  - **The view's pitch rate per shot:** `p h / (m k² + I_hold)`. `h` is the
    bore's height over where the shooter takes the recoil, `k` the gun's
    radius of gyration, and `I_hold` the stance's arms and upper body.
  - **Response shape:** constant pull, slowed return, stop at the aim, clamp
    (point 2 above). Per-stance values live in `data/stances.json`, tagged
    `estimated:` as tuned for play.
  - **Factors on each shot:**
    - a hip factor, lerped by the sight fraction;
    - a braced-shot reduction (a settled shooter takes the first shot better),
      which is point 4 as a recovery state rather than a bullet count;
    - spreads drawn from the round's stream.
- **Aim.**
  - **The binary gate (point 3).** At the sight the cone is only what remains
    of the bloom, and nothing adds to it there. Short of the sight it is the
    hip cone.
  - **The hold's wander:** two slow sines per axis at about a breath, scaled
    by the sight fraction. It is part of the view offset, so the camera shows
    what the bore does: what you see is where it goes, except for the hip
    cone.
- **One stream per round.** A shot's recoil and cone draws are z5..z8, u9 and
  u10 of its round's stream, after the z1..z4 that `bd::drawRound` takes.
  The round and its recoil are one draw everywhere, and existing draws do not
  move. The hold's phases come from a separate stream of the weapon's seed.
- **Data.** These are optional fields: guns that lack them still fly as
  before.
  - Rifles: `mass_lb`, `cyclic_rate_rpm`, `fire_modes`, `magazine_rounds`,
    `manual_cycle_s`, `reload_s`, `ads_s`, `sprint_out_s`, `bore_lever_in`,
    `gyration_in`.
  - Cartridges: `charge_gr` and `gas_velocity_fps`.
  - A new `stances` section.

## Consequences

- **Tests** (`tests/test_shooter.cpp`):
  - exact cyclic rate at 60 Hz, 144 Hz, one call and a jittery host, bit for
    bit;
  - the shot on the first step of the click's frame;
  - semi, burst and manual actions;
  - dry fire, reload and sprint-out;
  - the kick's closed form and no overshoot;
  - the braced, hip and stance factors;
  - no cone at the sight;
  - golden hashes: one native, and one the browser reproduces through the C
    ABI (`web/smoke_test.mjs`).
- **Validation.** Free-recoil energies sit in the commonly quoted ranges
  (docs/validation.md). The shooter's response is tuned, not measured. Muzzle
  rise against high-speed footage is an open item.
- **Hit feedback** (hitmarkers, impact sounds) needs where the round lands,
  which is Milestone 5's world query. The engine already flies the round at
  full speed, so the impact time is known before the next frame is drawn. A
  5.56 M855 reaches 25 m in 28 ms and 50 m in 56 ms: one to four frames at
  60 Hz.
- **Animation stretching (point 5) is the host's job.** The events carry
  exact times, and the timers are in the data.
