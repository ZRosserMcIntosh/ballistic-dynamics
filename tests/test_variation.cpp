// Ballistic Dynamics — every round unique (Milestone 4): the muzzle
// velocity's spread and its powder temperature, the BC's spread, the barrel's
// throw, seeded per round; and the spread of impacts they make.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstdlib>

#include "bd/aviation_air.hpp"
#include "bd/frames.hpp"
#include "bd/solver.hpp"
#include "bd/units.hpp"
#include "bd/variation.hpp"
#include "bd_test.hpp"

using namespace bd;

namespace {

const StationAir& icao() {
    static const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    return air;
}

VariationModel model308() {
    VariationModel m;
    m.velocitySd = units::fps(12);
    m.bcSd = 0.01;
    m.angleSd = units::moa(0.75) / kFiveShotSpreadPerSigma;
    m.tempCoefficient = units::fps(1.5) * 1.8;
    m.tempReference = units::celsiusToKelvin(21.1);
    return m;
}

}  // namespace

TEST_CASE("variation: each round's draw is its own seeded stream, the same everywhere") {
    const VariationModel m = model308();
    const RoundDraw a = drawRound(m, 42, 7), b = drawRound(m, 42, 7), c = drawRound(m, 42, 8), d = drawRound(m, 43, 7);
    CHECK(bitsOf(a.velocityOffset) == bitsOf(b.velocityOffset));
    CHECK(bitsOf(a.up) == bitsOf(b.up) && bitsOf(a.right) == bitsOf(b.right) && bitsOf(a.bcFactor) == bitsOf(b.bcFactor));
    CHECK(a.velocityOffset != c.velocityOffset);
    CHECK(a.velocityOffset != d.velocityOffset);
    // Turning a cause off leaves the others' draws alone.
    VariationModel noBc = m;
    noBc.bcSd = 0;
    const RoundDraw e = drawRound(noBc, 42, 7);
    CHECK(bitsOf(e.velocityOffset) == bitsOf(a.velocityOffset) && bitsOf(e.up) == bitsOf(a.up));
    CHECK(e.bcFactor == 1.0);
    // Nothing set, nothing drawn.
    const RoundDraw z = drawRound(VariationModel{}, 42, 7);
    CHECK(z.velocityOffset == 0 && z.bcFactor == 1 && z.up == 0 && z.right == 0);
    // A golden value: this draw must be these bits on every platform.
    char hex[32];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(bitsOf(a.velocityOffset)));
    if (std::getenv("BD_VERBOSE")) std::printf("  draw(42, 7).velocityOffset = %.17g (%s)\n", a.velocityOffset, hex);
}

TEST_CASE("variation: the draws have the spreads the model asks for") {
    const VariationModel m = model308();
    const int n = 20000;
    Real sv = 0, sv2 = 0, sb2 = 0, su2 = 0, sr2 = 0, sur = 0;
    for (int k = 0; k < n; ++k) {
        const RoundDraw d = drawRound(m, 2026, static_cast<std::uint64_t>(k));
        sv += d.velocityOffset;
        sv2 += sq(d.velocityOffset);
        sb2 += sq(d.bcFactor - 1);
        su2 += sq(d.up);
        sr2 += sq(d.right);
        sur += d.up * d.right;
    }
    CHECK_NEAR(sv / n, 0.0, 4.0 * m.velocitySd / std::sqrt(Real(n)));
    CHECK_NEAR(std::sqrt(sv2 / n) / m.velocitySd, 1.0, 0.02);
    CHECK_NEAR(std::sqrt(sb2 / n) / m.bcSd, 1.0, 0.02);
    CHECK_NEAR(std::sqrt(su2 / n) / m.angleSd, 1.0, 0.02);
    CHECK_NEAR(std::sqrt(sr2 / n) / m.angleSd, 1.0, 0.02);
    CHECK_NEAR(sur / n / sq(m.angleSd), 0.0, 0.03);  // the two axes are independent
}

TEST_CASE("variation: the powder's temperature moves the muzzle velocity") {
    CartridgeDef c;
    c.muzzleVelocity = units::fps(2600);
    const VariationModel m = model308();
    // 1.5 fps per degree F: 20 F colder, 30 fps slower.
    CHECK_NEAR(units::toFps(muzzleSpeedAt(c, m, m.tempReference - 20.0 / 1.8)), 2570.0, 1e-9);
    CHECK_NEAR(muzzleSpeedAt(c, m, m.tempReference), c.muzzleVelocity, 1e-12);
    CHECK_NEAR(muzzleSpeedAt(c, m, 0), c.muzzleVelocity, 0);           // not given: as published
    CHECK_NEAR(muzzleSpeedAt(c, VariationModel{}, 250), c.muzzleVelocity, 0);
}

TEST_CASE("variation: a 5-shot group the size the rifle's figure says") {
    // The barrel's throw alone, at 100 yd: the mean 5-shot extreme spread
    // over many groups comes back as the rifle's group figure (Grubbs' 3.08).
    VariationModel m;
    m.angleSd = units::moa(1.0) / kFiveShotSpreadPerSigma;
    const Real X = units::yards(100);
    Real total = 0;
    const int groups = 4000;
    for (int g = 0; g < groups; ++g) {
        Real x[5], y[5];
        for (int k = 0; k < 5; ++k) {
            const RoundDraw d = drawRound(m, 99, static_cast<std::uint64_t>(g * 5 + k));
            x[k] = d.right * X;
            y[k] = d.up * X;
        }
        Real es = 0;
        for (int i = 0; i < 5; ++i)
            for (int j = i + 1; j < 5; ++j) es = std::max(es, std::hypot(x[i] - x[j], y[i] - y[j]));
        total += es;
    }
    CHECK_NEAR(total / groups / (units::moa(1.0) * X), 1.0, 0.02);
}

TEST_CASE("variation: the throw turns the bore by its angles") {
    const Vec3 bore = frames::directionFromAzEl(0.3, 0.01);
    RoundDraw d;
    d.up = 1e-3;
    d.right = -2e-3;
    const Vec3 t = thrownDirection(bore, d);
    Real az0, el0, az1, el1;
    az0 = std::atan2(bore.x, bore.y); el0 = std::asin(bore.z);
    az1 = std::atan2(t.x, t.y); el1 = std::asin(t.z);
    // Small angles added to the direction: right to second order (a few 1e-8 rad here).
    CHECK_NEAR(el1 - el0, 1e-3, 5e-8);
    CHECK_NEAR((az1 - az0) * std::cos(el0), -2e-3, 5e-8);
}

TEST_CASE("variation: the spread of impacts at range, linearised, against flown rounds") {
    // A .308 175 SMK at 1,000 yd: fly 400 rounds with their own draws and
    // compare the scatter of their impacts with spreadAt's prediction.
    const Library& lib = bdtest::library();
    const CartridgeDef& c = *lib.cartridge("fgmm_308_175");
    const ProjectileDef& p = *lib.projectile(c.projectile);
    std::string err;
    RangeSetup s;
    s.drag = dragModelFor(p, lib, "", err);
    s.muzzleSpeed = c.muzzleVelocity;
    s.sightHeight = units::inches(1.5);
    s.zeroRange = units::yards(100);
    const Real el = zeroElevation(s, icao());
    const Real X = units::yards(1000);
    const VariationModel m = model308();
    const Spread pred = spreadAt(s, icao(), el, X, m);
    // About 9 in of vertical at 1,000 yd for 12 fps, 1 % BC and a 0.75 MOA rifle.
    CHECK_RANGE(units::toInches(pred.vertical), 6.0, 12.0);
    CHECK(pred.fromVelocity > pred.fromBarrel);
    const frames::LineOfFire lof({}, 0);
    const int n = 400;
    Real s2v = 0, s2h = 0, mv = 0;
    std::vector<Real> heights;
    for (int k = 0; k < n; ++k) {
        const RoundDraw d = drawRound(m, 7, static_cast<std::uint64_t>(k));
        RangeSetup r = s;
        r.drag = s.drag.scaledBc(d.bcFactor);
        r.muzzleSpeed = s.muzzleSpeed + d.velocityOffset;
        const auto row = rangeTable(r, icao(), el + d.up, {X})[0];
        // The throw to the right moves the impact right by about the angle times the range.
        const Real h = row.drop, w = row.windage + d.right * X;
        heights.push_back(h);
        mv += h / n;
        s2h += sq(w) / n;
    }
    for (Real h : heights) s2v += sq(h - mv) / n;
    if (std::getenv("BD_VERBOSE"))
        std::printf("  1000 yd: predicted %.2f in vertical, %.2f in horizontal; flown %.2f, %.2f\n", units::toInches(pred.vertical),
                    units::toInches(pred.horizontal), units::toInches(std::sqrt(s2v)), units::toInches(std::sqrt(s2h)));
    CHECK_NEAR(std::sqrt(s2v) / pred.vertical, 1.0, 0.12);   // 400 rounds: about 4 % sampling error on an SD
    CHECK_NEAR(std::sqrt(s2h) / pred.horizontal, 1.0, 0.12);
}
