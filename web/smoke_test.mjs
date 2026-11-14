// Ballistic Dynamics — Node smoke test of the WebAssembly build (web/dist).
// SPDX-License-Identifier: MIT
import { readFile } from 'node:fs/promises';
import createBallisticModule from './dist/ballistic.mjs';

// The same values as kDeterminismGolden and kTrajectoryGolden in
// src/core/include/bd/determinism.hpp: the browser must compute the same bits
// as every native build.
const GOLDEN = ['7302405ad0a26843', '2ca1ba49af3e55d1', 'f3b557df819cffed'];

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
if (h[2] !== GOLDEN[2]) fail(`spin hash ${h[2]}, expected ${GOLDEN[2]}`);

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
// Milestone 3: the M24's 1:11.25 right-hand twist drifts the .308 right; at
// 45 N, firing north, Coriolis adds to it; in Sao Paulo it takes some away.
B.wind(0, []);
B.bd_setup(11.25 * inch, 0, 1);
const sg = B.bd_stability(i308);
if (!(sg > 1.8 && sg < 2.0)) fail('Sg ' + sg);
const spinOnly = B.table(i308, 100 * yd, 1.5 * inch, 1000 * yd, 1000 * yd).at(-1).windage / inch;
B.bd_setup(11.25 * inch, 45 * Math.PI / 180, 3);
const north = B.table(i308, 100 * yd, 1.5 * inch, 1000 * yd, 1000 * yd).at(-1).windage / inch;
B.bd_setup(11.25 * inch, -23.55 * Math.PI / 180, 3);
const sp = B.table(i308, 100 * yd, 1.5 * inch, 1000 * yd, 1000 * yd).at(-1).windage / inch;
console.log(`.308 at 1000 yd, Sg ${sg.toFixed(2)}: spin drift ${spinOnly.toFixed(1)} in; with Coriolis at 45 N ${north.toFixed(1)}, at Sao Paulo ${sp.toFixed(1)}`);
if (!(spinOnly > 8 && spinOnly < 11)) fail('spin drift ' + spinOnly);
if (!(north > spinOnly && sp < spinOnly)) fail('Coriolis direction');
if (!B.bd_fire(i308, 0, 0, 0, 0, e, 0)) fail('fire with spin');
B.bd_advance(0.5, 0);
const r = B.round(), nose = B.noseAt(0.25);
if (!(r.spin > 15000 && r.stability > sg)) fail('spin state ' + JSON.stringify(r));
if (!(Math.abs(Math.hypot(...nose) - 1) < 1e-12)) fail('nose');
const wk = B.wake(0.4, 1.0, 0.3, 175 * 6.479891e-5, 0.308 * inch);
if (!(wk.length > 7 * 100 && wk.length % 7 === 0)) fail('wake ' + wk.length);
// The optical strength: hot (index > 0) and weaker with age.
if (!(wk[6] > 0 && wk[6] < wk[wk.length - 1])) fail('wake index ' + wk[6] + ' ' + wk[wk.length - 1]);

// Milestone 4: every round unique. The same (seed, stream) is the same round;
// another stream another; the powder's cold slows it.
B.bd_station(101325, 288.15, 0, 0);
B.wind(0, []);
B.bd_group(0.9);
const flyRound = (stream) => { B.bd_fire_round(0, 0, 0, 0, 0, 0.01, 0, 1234567, stream); B.bd_advance(0.5, 0); return B.round().position; };
const r1 = flyRound(3), r2 = flyRound(3), r3 = flyRound(4);
if (!(r1.every((v, i) => v === r2[i]) && r1.some((v, i) => v !== r3[i]))) fail('unique rounds ' + r1 + ' / ' + r2 + ' / ' + r3);
const d3 = (B.bd_fire_round(0, 0, 0, 0, 0, 0.01, 0, 1234567, 3), B.lastDraw());
if (!(Math.abs(d3.velocityOffset) < 0.3048 * 12 * 5 && d3.bcFactor !== 1 && d3.up !== 0)) fail('draw ' + JSON.stringify(d3));
B.bd_powder(273.15 - 10);
B.bd_fire(0, 0, 0, 0, 0, 0.01, 0);
const cold = B.lastDraw().speed;
B.bd_powder(0);
B.bd_fire(0, 0, 0, 0, 0, 0.01, 0);
if (!(cold < B.lastDraw().speed - 20)) fail('powder temperature ' + cold + ' vs ' + B.lastDraw().speed);
const spr = B.spread(0, 91.44, 0.0381, 914.4);
if (!(spr.vertical > 0.1 && spr.vertical < 0.4 && spr.horizontal > 0.05)) fail("spread " + JSON.stringify(spr));
B.bd_group(0);
console.log(`unique rounds: same seed same bits; -10 C powder ${(cold / 0.3048).toFixed(0)} fps; 1000 yd spread ${(spr.vertical / 0.0254).toFixed(1)} in vertical`);

// Many rounds at once: each flies exactly as it would alone, in the air and
// wind it was fired into, whatever is fired or changed after it.
B.bd_station(101325, 288.15, 0, 0);
B.wind(0, []);
B.bd_setup(11.25 * inch, 45 * Math.PI / 180, 3);
const soloId = B.bd_fire(i308, 0, 0, 0, 0, e, 0);
B.bd_advance(1.0, 0);
const solo = B.stateAt(0.8);
B.bd_release(soloId);
const ra = B.bd_fire(i308, 0, 0, 0, 0, e, 0);
B.wind(0, [{ fromRange: 0, speed: 10, fromBearing: Math.PI / 2, gust: 0 }]);  // for the next round only
const rb = B.bd_fire(i308, 0, 0, 0, 0, e, 0.3);
if (!(ra > 0 && rb > ra)) fail('round ids ' + ra + ' ' + rb);
for (let t = 0.01; t <= 1.31; t += 0.01) {  // interleaved, as a page draws them
  B.bd_select(ra); B.bd_advance(t + 0.001, 0);
  B.bd_select(rb); B.bd_advance(t + 0.001, 0);
}
B.bd_select(ra);
const both = B.stateAt(0.8);
if (both.position.some((x, k) => x !== solo.position[k]) || both.velocity.some((x, k) => x !== solo.velocity[k])) fail('a round in company flew differently');
B.bd_select(rb);
const blown = B.stateAt(1.1).position[0] - both.position[0];
if (!(blown < -0.3)) fail('the second round in its own wind ' + blown);
B.bd_release(ra);
if (B.bd_select(ra) || !B.bd_select(rb)) fail('release');
B.bd_release(0);
if (B.bd_select(rb)) fail('release all');

// bd_air reports the wind rounds fly through (the page's flags read it).
B.bd_station(101325, 288.15, 0, 0);
B.wind(0, [{ fromRange: 0, speed: 10, fromBearing: Math.PI / 2, gust: 0 }]);  // from the east
const w = B.air(0, 500, 0, 3).wind;
if (!(Math.abs(w[0] + 10) < 1e-9 && Math.abs(w[1]) < 1e-9)) fail('bd_air wind ' + w);
// A table fired later meets the gusts of that moment.
B.wind(0, [{ fromRange: 0, speed: 5, fromBearing: Math.PI / 2, gust: 2 }]);
const at0 = B.table(i308, 100 * yd, 1.5 * inch, 1000, 1000, 0, 0).at(-1).windage;
const at9 = B.table(i308, 100 * yd, 1.5 * inch, 1000, 1000, 0, 9).at(-1).windage;
if (at0 === at9) fail('table start time');

// The shooter layer (ADR 0005): the scripted string of tests/test_shooter.cpp
// ("the browser's scripted string") through the C ABI must hash the same bits.
const SHOOTER_GOLDEN = '5bcdfaece1f41fd9';
// bd::Hash64's start value, exactly as in src/core/include/bd/math.hpp.
const fnv = { h: 1469598103934665603n };
const dv = new DataView(new ArrayBuffer(8));
const hashReal = (x) => {
  dv.setFloat64(0, x, true);
  for (let k = 0; k < 8; ++k) fnv.h = ((fnv.h ^ BigInt(dv.getUint8(k))) * 0x100000001b3n) & 0xffffffffffffffffn;
};
const rIdx = (id) => cat.rifles.findIndex((r) => r.id === id);
const cIdx = (id) => cat.cartridges.findIndex((c) => c.id === id);
const sIdx = (id) => cat.stances.findIndex((s) => s.id === id);
if (!cat.stances || cat.stances.length < 3) fail('stances in the catalog');
if (!B.bd_weapon(rIdx('mp5a3'), cIdx('mp5_9mm_124_fmj'), sIdx('standing'), 2026, 100, 0)) fail('bd_weapon');
B.bd_weapon_look(0, 1, 0);
let shotCount = 0, firstShot = null;
for (let f = 1; f <= 120; ++f) {
  B.weaponInput({ trigger: f <= 60, ads: f >= 30 }, 0, 0, 1);
  for (const e of B.weaponAdvance(f / 60)) {
    if (e.kind !== 'shot') continue;
    if (!firstShot) firstShot = { frame: f, ...e };
    ++shotCount;
    e.direction.forEach(hashReal);
    hashReal(e.kick);
  }
  const st = B.weaponState();
  hashReal(st.viewPitch);
  hashReal(st.viewYaw);
}
const shooterHash = fnv.h.toString(16).padStart(16, '0');
if (shotCount !== 14) fail(`shooter: ${shotCount} shots, expected 14`);
if (shooterHash !== SHOOTER_GOLDEN) fail(`shooter hash ${shooterHash}, expected ${SHOOTER_GOLDEN}`);
if (firstShot.frame !== 1 || firstShot.step !== 0) fail('shooter: the first shot is not on the first step of the click\'s frame');
// Fly a shot from the M16A2: M855 reaches 50 m in about 56 ms (standard day,
// powder as published, no rifle group set, and no spin: the .308's 1:11.25
// twist set above would tumble the long M855, which wants 1:7).
B.bd_station(101325, 288.15, 0, 0);
B.wind(0, []);
B.bd_powder(0);
B.bd_group(0);
B.bd_setup(0, 0, 0);
B.bd_weapon(rIdx('m16a2'), cIdx('federal_xm855'), sIdx('standing'), 7, 0, 0);
B.bd_weapon_look(0, 1, 0);
B.weaponInput({ trigger: true, ads: true });
const shot = B.weaponAdvance(1 / 60).find((e) => e.kind === 'shot');
if (!shot) fail('shooter: no shot');
const id = B.bd_fire_shot(cIdx('federal_xm855'), 0, 0, 1.5, shot.index);
if (!id) fail('bd_fire_shot');
let tof = 0;
for (let t = 0.001; t < 0.5; t += 0.001) { B.bd_advance(t, 0); if (B.round().position[1] >= 50) { tof = B.round().time; break; } }
if (!(tof > 0.050 && tof < 0.062)) fail(`shooter: M855 to 50 m in ${tof} s`);
const rec = B.recoil(rIdx('m16a2'), cIdx('federal_xm855'), sIdx('standing'));
if (!(rec.energy > 4 && rec.energy < 6)) fail(`shooter: M16A2 free recoil ${rec.energy} J`);
console.log(`shooter: ${shotCount} shots, hash ${shooterHash}; M855 to 50 m in ${(tof * 1e3).toFixed(1)} ms; M16A2 free recoil ${rec.energy.toFixed(2)} J`);

console.log(`ok: hashes ${h.join(' ')}; bullet time ${frames} frames, at most ${(maxJump * 1000).toFixed(1)} mm per frame`);
