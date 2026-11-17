# Bullet time

The range page (Milestone 7) gets a bullet camera in the style of *Sniper
Elite*'s kill cam: the shot leaves the rifle in real time, the clock slows
until the round hangs in the air, the camera rides alongside it down the
range, and time comes back at the impact. This page is the plan, split into
what the engine already provides and what the page will do. Nothing on the
page side exists yet; it belongs to the `Sao-Paulo` repository and is built
there in Milestone 7.

## What the engine provides (Milestone 2, done)

- **The same physics at any playback speed.** The round always flies 1 ms
  steps on its own clock (ADR 0002). Slow motion is the page advancing the
  engine by less than real time: at 1/1000 speed a 60 fps page advances
  16.7 µs per frame. The flight is bit-identical to the real-time one, so
  the slow-motion shot *is* the shot, not a re-enactment.
- **Smooth between steps.** `stateAt(t)` (`bd_state_at` in the browser)
  interpolates the step states with a cubic Hermite, continuous in position
  and velocity and within 20 µm of a 20× finer run. A camera at 1/1000 speed
  sees the round move 13 mm per frame, never a 0.8 m step
  (`web/smoke_test.mjs` checks it). The page advances the engine one step
  ahead of the time it draws.
- **Replays.** With the trace kept, any moment of the flight can be shown
  again, forwards or backwards, at any speed. With the seed and the inputs,
  a share link re-flies the same shot bit for bit (ADR 0004).
- **What to draw on it.** Position and velocity at any instant; the air
  there (density, speed of sound, wind) from the same `AirSource`, hence the
  Mach number for the shock cone's half-angle asin(1/M) and the moment the
  round goes subsonic.

## What the page will do (Milestone 7, in `Sao-Paulo`)

1. **Fly first, then show.** On the trigger the page flies the whole shot
   at full speed (a 1,000 m shot is about 1,700 steps, well under a
   millisecond natively), so it knows where and when the round lands before
   the camera starts. It then decides whether to cut to the bullet camera
   (for example, a hit beyond a set range).
2. **Time ramp.** Real time through the muzzle blast. Ease down to about
   1/200 in the first tenth of a second, hold about 1/1000 down the range,
   and slow to about 1/5000 for the last few metres. Snap back to real time
   at the impact. The ramp is a curve of the page's clock against engine
   time; the engine does not know about it.
3. **Camera.** A chase camera a fixed distance behind and to the side of
   the round. It slowly orbits it, uses `stateAt` for the round and a
   smoothed look-ahead along the trace for its aim, and leads into the
   impact point for the last metres.
4. **What is seen.** The bullet, rotating at its real spin rate once
   Milestone 3 gives the spin: a 175 gr .308 from a 1:11.25 barrel turns
   about 2,800 times a second, so 2.8 turns a second at 1/1000. Also its
   trail, the shock cone while supersonic, the wind flags it passes, the
   mirage, and the arc it has flown.
5. **The impact.** What happens at the target comes from the material
   models (Milestone 5) and the Glass engine (Milestone 8): penetration,
   deflection, glass cracking in slow motion, tumbling behind it. Glass
   Dynamics already keeps crack birth times so a renderer can replay a
   fracture at any speed. The engine has no hitboxes and no bodies, so there
   is no X-ray anatomy view; the target engine owns its own damage.

## Checks for Milestone 7

- The slow-motion flight and the real-time flight land on the same point
  bit for bit, whatever the frame rate.
- At every playback speed the camera never sees the round jump more than a
  few centimetres between frames.
- A share link replays the same shot, with the same camera.
