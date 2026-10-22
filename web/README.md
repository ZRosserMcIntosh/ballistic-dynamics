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
B.determinism();                                     // '7302405ad0a26843' on every platform
```

| Call | Returns |
|---|---|
| `bd_text(n)` / `bd_text_length()` | the scratch text buffer (JSON in, text out) |
| `bd_load(n)`, `bd_resolve()` | load n bytes of JSON from the buffer, resolve references; the error count |
| `bd_error_text(i)` | error i |
| `bd_catalog()` | the library as JSON, SI values, provenance per field |
| `bd_station(p, T, RH, alt)` | sets the air from a station reading |
| `bd_air(x, y, z, t)` | 9 doubles: pressure, temperature, density, speed of sound, relative humidity, density altitude, wind east, north, up |
| `bd_determinism()` | the determinism hash, 16 hex digits |
