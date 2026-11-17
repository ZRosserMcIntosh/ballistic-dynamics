// Ballistic Dynamics — Node smoke test of the WebAssembly build (web/dist).
// SPDX-License-Identifier: MIT
import { readFile } from 'node:fs/promises';
import createBallisticModule from './dist/ballistic.mjs';

// The same value as kDeterminismGolden in src/core/include/bd/determinism.hpp:
// the browser must compute the same bits as every native build.
const GOLDEN = '7302405ad0a26843';

const fail = (m) => { console.error('FAIL', m); process.exit(1); };
const B = await createBallisticModule();
const errors = B.load(await readFile(new URL('./dist/library.json', import.meta.url), 'utf8'));
if (errors.length) { console.error(errors); fail('library'); }
const cat = B.catalog();
console.log(`${cat.projectiles.length} projectiles, ${cat.cartridges.length} cartridges, ${cat.rifles.length} rifles`);
for (const p of cat.projectiles)
  for (const [k, v] of Object.entries(p.provenance))
    if (!['published', 'measured', 'estimated'].includes(v.kind) || !v.note) fail(`provenance ${p.id}.${k}`);

const h = B.determinism();
if (h !== GOLDEN) fail(`determinism hash ${h}, expected ${GOLDEN}`);

// A hot, high station: the air reproduces the reading and thins with height.
B.bd_station(83400, 303.15, 0.2, 1609);
const a0 = B.air(0, 0, 0), a1 = B.air(0, 0, 100);
if (Math.abs(a0.pressure - 83400) > 1e-6) fail('station pressure ' + a0.pressure);
if (!(a0.density > 0.945 && a0.density < 0.96)) fail('density ' + a0.density);
if (!(a0.densityAltitude > 2450 && a0.densityAltitude < 2650)) fail('density altitude ' + a0.densityAltitude);
if (!(a1.pressure < a0.pressure - 900)) fail('pressure with height');
console.log(`ok: hash ${h}; rho ${a0.density.toFixed(4)} kg/m3, a ${a0.speedOfSound.toFixed(1)} m/s, DA ${a0.densityAltitude.toFixed(0)} m`);
