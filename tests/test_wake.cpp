// Ballistic Dynamics — the wake a round leaves (the trace).
// SPDX-License-Identifier: MIT
#include "bd/units.hpp"
#include "bd/wake.hpp"
#include "bd_test.hpp"

using namespace bd;

TEST_CASE("wake: the far wake grows as the cube root and fades as the two-thirds power") {
    const WakeParams p;
    const Real theta = wakeMomentumThickness(1.4e-5);  // a .308's Cd A near Mach 2
    CHECK_NEAR(theta, std::sqrt(1.4e-5 / kTwoPi), 1e-15);
    Real r1, d1, r8, d8;
    farWake(theta, 0.004, 50.0, p, r1, d1);
    farWake(theta, 0.004, 400.0, p, r8, d8);
    CHECK_NEAR(r8 / r1, 2.0, 1e-9);   // 8x the distance: twice as wide
    CHECK_NEAR(d1 / d8, 4.0, 1e-9);   // and a quarter as strong
    // A few centimetres wide after 50 m, as a trace looks.
    CHECK_RANGE(r1, 0.01, 0.05);
    // Near the body it is held at the bullet's radius.
    Real r0, d0;
    farWake(theta, 0.004, 1e-4, p, r0, d0);
    CHECK_NEAR(r0, 0.004, 0);
    CHECK(d0 <= 1.0);
}

TEST_CASE("wake: shed along the path, aged, and carried by the wind") {
    const Library& lib = bdtest::library();
    const ProjectileDef& smk = *lib.projectile("smk_308_175");
    std::string err;
    const DragModel drag = dragModelFor(smk, lib, "", err);
    AirSample a;
    a.wind = {3.0, 0.0, 0.0};  // 3 m/s blowing east, firing north
    const UniformAir air(a);
    Launch l;
    l.muzzleSpeed = units::fps(2600);
    PointMassOptions o;
    o.keepTrace = true;
    o.gravity = {};
    PointMassRound r(drag, air, l, o);
    r.advance(0.5);
    WakeParams p;
    p.spacing = 1.0;
    p.maxAge = 0.4;
    const auto w = wakeAt(r, smk.mass, smk.diameter, 0.45, p);
    CHECK(w.size() > 100);
    Real oldest = 0, prevAge = 1e9;
    bool ordered = true;
    for (const WakeElement& e : w) {
        CHECK(e.age >= 0 && e.age <= 0.4);
        // Where the round was when it shed it, carried east by the wind since.
        const FlightSample shedAt = r.stateAt(0.45 - e.age);
        CHECK_NEAR(e.position.x - shedAt.position.x, 3.0 * e.age, 1e-9);
        CHECK_NEAR(e.position.y, shedAt.position.y, 1e-6);
        ordered = ordered && e.age <= prevAge + 1e-12;
        prevAge = e.age;
        oldest = std::max(oldest, e.age);
    }
    CHECK(ordered);  // from the oldest (at the muzzle end) to the newest
    CHECK(oldest > 0.35);
    // Older wake is wider and weaker.
    CHECK(w.front().radius > w.back().radius);
    CHECK(w.front().deficit < w.back().deficit);
    // Nothing from the future, and the same bits every time.
    const auto again = wakeAt(r, smk.mass, smk.diameter, 0.45, p);
    CHECK(again.size() == w.size());
    CHECK(bitsOf(again[17].position.x) == bitsOf(w[17].position.x));
    CHECK(wakeAt(r, smk.mass, smk.diameter, 0.0, p).empty());
}

TEST_CASE("wake: its optical strength is its heat, and above Mach 1 the shocks take the wave drag") {
    // The drag's work stays in the wake as heat: dT/T = (u0/U) (gamma - 1) M^2.
    CHECK_NEAR(wakeHeat(1e-3, 2.0), 1.6e-3, 1e-15);
    CHECK_NEAR(wakeHeat(1e-3, 1.0) / wakeHeat(1e-3, 0.5), 4.0, 1e-12);
    const Library& lib = bdtest::library();
    std::string err;
    const DragModel g7 = dragModelFor(*lib.projectile("smk_308_175"), lib, "G7", err);
    const DragModel g1 = dragModelFor(*lib.projectile("smk_308_175"), lib, "G1", err);
    // Subsonic the wake has it all; at Mach 2.3 a boat-tail's wake keeps under half.
    CHECK_NEAR(wakeDragShare(g7, 0.6), 1.0, 0);
    CHECK_NEAR(wakeDragShare(g7, 0.8), 1.0, 0);
    CHECK_RANGE(wakeDragShare(g7, 2.3), 0.3, 0.5);
    CHECK_RANGE(wakeDragShare(g1, 2.3), 0.3, 0.5);
    // A .308 and a 9 mm fired side by side: the rifle's trace is hotter for
    // the same velocity deficit, and thin air weakens both.
    AirSample a;
    const UniformAir air(a);
    PointMassOptions o;
    o.keepTrace = true;
    o.gravity = {};
    // Returns index / deficit, and the Mach number it was shed at.
    auto indexOverDeficit = [&](const char* cartridge, const AirSource& through, Real* machOut = nullptr) {
        const CartridgeDef& c = *lib.cartridge(cartridge);
        const ProjectileDef& p = *lib.projectile(c.projectile);
        Launch l;
        l.muzzleSpeed = c.muzzleVelocity;
        PointMassRound r(dragModelFor(p, lib, "", err), through, l, o);
        r.advance(0.12);
        WakeParams wp;
        wp.maxAge = 0.1;
        const auto w = wakeAt(r, p.mass, p.diameter, 0.1, wp);
        CHECK(!w.empty());
        if (w.empty()) return 0.0;
        if (machOut) *machOut = length(r.stateAt(0.1 - w.back().age).velocity) / through.sample({}, 0).speedOfSound;
        return w.back().index / w.back().deficit;
    };
    Real mRifle = 0, mThin = 0;
    const Real rifle = indexOverDeficit("fgmm_308_175", air, &mRifle), pistol = indexOverDeficit("speer_9mm_124gd", air);
    CHECK_NEAR(rifle, 0.4 * mRifle * mRifle, 1e-4 * rifle);
    CHECK_RANGE(rifle, 1.6, 2.3);   // 0.4 M^2 near Mach 2.3
    CHECK_RANGE(pistol, 0.3, 0.45); // near Mach 1
    AirSample thin = a;
    thin.density = 0.6125;
    const UniformAir high(thin);
    // Half the density, half the index change (at the Mach it was shed at).
    const Real thinRatio = indexOverDeficit("fgmm_308_175", high, &mThin) / (0.4 * mThin * mThin);
    CHECK_NEAR(thinRatio, 0.5, 1e-4);
}
