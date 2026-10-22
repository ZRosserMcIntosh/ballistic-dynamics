# Browser build

`build_wasi.sh` compiles the core, the Aviation adapter, Aviation's three
atmosphere sources and `ballistic_web.cpp` (a C ABI) to `ballistic.wasm` with
wasi-sdk (no exceptions, no RTTI, reactor model, `-ffp-contract=off`), copies
the loader as `ballistic.mjs`, and merges `data/*.json` into `library.json`.

```js
import createBallisticModule from './ballistic.mjs';
const B = await createBallisticModule();
B.load(await (await fetch('library.json')).text());  // returns error strings
const cat = B.catalog();                             // projectiles, cartridges, rifles, with provenance
B.bd_station(83400, 303.15, 0.2, 1609);              // station Pa, K, RH 0..1, m MSL
const air = B.air(0, 0, 100);                        // ENU metres from the station
B.determinism();                                     // ['7302405ad0a26843', '2ca1ba49af3e55d1'] on every platform

// A shot: zero, wind along the range, fire, and watch it in bullet time.
B.wind(0, [{ fromRange: 0, speed: 4.5, fromBearing: Math.PI / 2, gust: 0.8 }]);
const e = B.bd_zero(cartridgeIndex, 91.44, 0.0381);   // 100 yd zero, 1.5 in sight
B.bd_fire(cartridgeIndex, 0, 0, 0, 0, e, 0);         // from the origin, north
let t = 0;
function frame() {                                    // 1/1000 speed
  t += 1 / 60 / 1000;
  B.bd_advance(t + 0.001, 0);                         // the engine runs one step ahead...
  const s = B.stateAt(t);                             // ...and the camera draws between steps
}
const card = B.table(cartridgeIndex, 91.44, 0.0381, 914.4, 91.44);  // range card, SI
```

| Call | Returns |
|---|---|
| `bd_text(n)` / `bd_text_length()` | the scratch text buffer (JSON in, text out) |
| `bd_load(n)`, `bd_resolve()` | load n bytes of JSON from the buffer, resolve references; the error count |
| `bd_error_text(i)` | error i |
| `bd_catalog()` | the library as JSON, SI values, provenance per field |
| `bd_station(p, T, RH, alt)` | sets the air from a station reading (clears the wind); rounds already fired keep the air they were fired into |
| `bd_air(x, y, z, t)` | 9 doubles: pressure, temperature, density, speed of sound, relative humidity, density altitude, wind east, north, up |
| `bd_wind(azimuth, n)` (`B.wind`) | wind layers along the line of fire: n quadruples (from range m, speed m/s, from bearing rad, gust RMS m/s) in the text buffer; for rounds fired after it |
| `bd_zero(i, zeroRange, sightHeight)` | the bore elevation (rad) that zeroes cartridge i, in the station's still air |
| `bd_fire(i, x, y, z, azimuth, elevation, t0)` | fires cartridge i (its preferred drag model) at engine time t0, keeping the trace; returns the round's id (from 1; 0 if it cannot fire) and selects it |
| `bd_select(id)` | selects round `id` for `bd_advance`, `bd_round`, `bd_nose_at`, `bd_state_at` and `bd_wake`; 1 if it exists. Any number of rounds fly at once, each exactly as it would alone |
| `bd_release(id)` | frees round `id` (0: all). Past 256 rounds the oldest is released |
| `bd_advance(t, maxSteps)` | whole 1 ms steps up to engine time t |
| `bd_round()` (`B.round`) | 8 doubles: time, position, velocity, stopped |
| `bd_state_at(t)` (`B.stateAt`) | bullet time: 7 doubles, the state at any flown instant, smooth between steps (advance one step ahead first) |
| `bd_table(i, zeroRange, sightHeight, maxRange, step)` (`B.table`) | a range card: range, drop, windage, time, speed, energy, Mach (SI) |
| `bd_determinism()` | the determinism hashes (maths, trajectory), 16 hex digits each |
