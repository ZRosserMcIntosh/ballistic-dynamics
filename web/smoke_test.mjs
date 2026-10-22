// Ballistic Dynamics — Node smoke test of the WebAssembly build (web/dist).
// SPDX-License-Identifier: MIT
import { readFile } from 'node:fs/promises';
import createBallisticModule from './dist/ballistic.mjs';

// The same values as kDeterminismGolden and kTrajectoryGolden in
// src/core/include/bd/determinism.hpp: the browser must compute the same bits
// as every native build.
const GOLDEN = ['7302405ad0a26843', '2ca1ba49af3e55d1'];

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
if (h[0] !== GOLDEN[0]) fail(`maths hash ${h[0]}, expected ${GOLDEN[0]}`);
if (h[1] !== GOLDEN[1]) fail(`trajectory hash ${h[1]}, expected ${GOLDEN[1]}`);

// A hot, high station: the air reproduces the reading and thins with height.
B.bd_station(83400, 303.15, 0.2, 1609);
const a0 = B.air(0, 0, 0), a1 = B.air(0, 0, 100);
if (Math.abs(a0.pressure - 83400) > 1e-6) fail('station pressure ' + a0.pressure);
if (!(a0.density > 0.945 && a0.density < 0.96)) fail('density ' + a0.density);
if (!(a0.densityAltitude > 2450 && a0.densityAltitude < 2650)) fail('density altitude ' + a0.densityAltitude);
if (!(a1.pressure < a0.pressure - 900)) fail('pressure with height');

// The .308 at 1,000 yd on a standard day with 10 mph from the right, as natively.
B.bd_station(101325, 288.15, 0, 0);
const mph = 0.44704, yd = 0.9144, inch = 0.0254;
B.wind(0, [{ fromRange: 0, speed: 10 * mph, fromBearing: Math.PI / 2 }]);
const i308 = cat.cartridges.findIndex((c) => c.id === 'fgmm_308_175');
const rows = B.table(i308, 100 * yd, 1.5 * inch, 1000 * yd, 100 * yd);
const last = rows[rows.length - 1];
console.log(`.308 at 1000 yd: drop ${(last.drop / inch).toFixed(2)} in, drift ${(last.windage / inch).toFixed(2)} in, ${(last.speed / 0.3048).toFixed(0)} fps`);
if (Math.abs(last.drop / inch + 432.06) > 0.05) fail('1000 yd drop ' + last.drop / inch);
if (Math.abs(last.windage / inch + 112.16) > 0.05) fail('1000 yd drift ' + last.windage / inch);

// Fire one and watch it in bullet time: 1/1000 speed at 60 frames a second.
const e = B.bd_zero(i308, 100 * yd, 1.5 * inch);
if (!B.bd_fire(i308, 0, 0, 0, 0, e, 0)) fail('fire');
let wall = 0, frames = 0, maxJump = 0, prev = null;
while (wall < 0.05) {         // 50 ms of flight, 3,000 frames
  wall += 1 / 60 / 1000;
  B.bd_advance(wall + 0.001, 0);  // the engine runs one step ahead of what is drawn
  const s = B.stateAt(wall);
  if (prev) maxJump = Math.max(maxJump, Math.hypot(...s.position.map((x, k) => x - prev.position[k])));
  prev = s;
  ++frames;
}
// 790 m/s for 16.7 us is 13 mm per frame; never the 0.79 m of a whole step.
if (!(maxJump < 0.02)) fail('bullet time jumps ' + maxJump);
console.log(`ok: hashes ${h.join(' ')}; bullet time ${frames} frames, at most ${(maxJump * 1000).toFixed(1)} mm per frame`);
