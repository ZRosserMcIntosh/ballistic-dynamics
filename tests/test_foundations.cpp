// Ballistic Dynamics — foundations: deterministic maths, random numbers,
// frames, units, the fixed-step clock, JSON.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <limits>

#include "bd/clock.hpp"
#include "bd/determinism.hpp"
#include "bd/dmath.hpp"
#include "bd/frames.hpp"
#include "bd/json.hpp"
#include "bd/rng.hpp"
#include "bd/units.hpp"
#include "bd_test.hpp"

using namespace bd;

namespace {

Real ulpsBetween(Real a, Real b) {
    if (a == b) return 0;
    const Real step = std::nextafter(b, std::numeric_limits<Real>::infinity()) - b;
    return std::abs(a - b) / step;
}

}  // namespace


TEST_CASE("dmath: elementary functions agree with the C library to a few ulps") {
    Real e = 0, l = 0, s = 0, c = 0, a = 0, a2 = 0, p = 0, cb = 0;
    for (int i = 0; i < 20000; ++i) {
        const Real t = i / 20000.0;
        const Real x = -700 + 1400 * t;
        e = std::max(e, ulpsBetween(dm::exp(x), std::exp(x)));
        const Real y = std::exp(-300 + 600 * t);
        l = std::max(l, ulpsBetween(dm::log(y), std::log(y)));
        const Real z = -100 + 200 * t;
        s = std::max(s, std::abs(dm::sin(z) - std::sin(z)));
        c = std::max(c, std::abs(dm::cos(z) - std::cos(z)));
        const Real w = 3 * std::tan(-1.5 + 3 * t);
        a = std::max(a, ulpsBetween(dm::atan(w), std::atan(w)));
        const Real u = 3 * std::sin(44 * t), v = 2 * std::cos(31 * t);
        a2 = std::max(a2, ulpsBetween(dm::atan2(u, v), std::atan2(u, v)));
        const Real q = 0.01 + 100 * t;
        p = std::max(p, ulpsBetween(dm::pow(q, 1.7), std::pow(q, 1.7)));
        cb = std::max(cb, ulpsBetween(dm::cbrt(z), std::cbrt(z)));
    }
    CHECK(e <= 2);
    CHECK(l <= 2);
    CHECK(s <= 2.3e-16);
    CHECK(c <= 2.3e-16);
    CHECK(a <= 2);
    CHECK(a2 <= 2);
    CHECK(p <= 16);  // exp(y log x): the error of y log x is amplified by |y log x| <= 8
    CHECK(cb <= 4);
    CHECK(dm::exp(0) == 1.0);
    CHECK(dm::log(1) == 0.0);
    CHECK(dm::sin(0) == 0.0);
    CHECK_NEAR(dm::atan2(1, -1), 0.75 * kPi, 1e-16);
    CHECK_NEAR(dm::atan2(-1, -1), -0.75 * kPi, 1e-16);
    CHECK_NEAR(dm::atan2(0.0, -1.0), kPi, 0);
    CHECK_NEAR(dm::asin(1.0), 0.5 * kPi, 1e-16);
    CHECK_NEAR(dm::acos(-1.0), kPi, 4e-16);
    CHECK_NEAR(dm::tanh(0.3), std::tanh(0.3), 1e-16);
    CHECK_NEAR(dm::tanh(0.01), std::tanh(0.01), 1e-18);
    CHECK(dm::exp(800) == std::numeric_limits<Real>::infinity());
    CHECK(dm::exp(-800) == 0.0);
    CHECK(dm::log(-1) != dm::log(-1));  // NaN
}

TEST_CASE("determinism: the deterministic maths give the same bits on every platform") {
    const std::uint64_t h = determinismProbe();
    if (h != kDeterminismGolden) {
        ++bdtest::failures();
        std::fprintf(stderr, "  dmath hash 0x%016llx, expected 0x%016llx\n", static_cast<unsigned long long>(h),
                     static_cast<unsigned long long>(kDeterminismGolden));
    }
}

TEST_CASE("rng: same seed and stream, same draws; streams are independent of order") {
    Rng a(42, 3), b(42, 3), c(42, 4), d(43, 3);
    bool same = true, diffStream = false, diffSeed = false;
    for (int i = 0; i < 1000; ++i) {
        const std::uint64_t x = a.next();
        same = same && x == b.next();
        diffStream = diffStream || x != c.next();
        diffSeed = diffSeed || x != d.next();
    }
    CHECK(same);
    CHECK(diffStream);
    CHECK(diffSeed);
    // Round 7's draws do not depend on whether rounds 0-6 were drawn first.
    Rng late(99, 7);
    for (int k = 0; k < 7; ++k) {
        Rng other(99, static_cast<std::uint64_t>(k));
        for (int i = 0; i < 50; ++i) other.normal();
    }
    Rng fresh(99, 7);
    CHECK(late.normal() == fresh.normal());
}

TEST_CASE("rng: uniform and normal deviates have the right moments") {
    Rng r(7);
    const int n = 200000;
    Real su = 0, su2 = 0, sn = 0, sn2 = 0, sn4 = 0;
    Real lo = 1, hi = 0;
    for (int i = 0; i < n; ++i) {
        const Real u = r.uniform();
        lo = std::min(lo, u);
        hi = std::max(hi, u);
        su += u;
        su2 += u * u;
        const Real z = r.normal();
        sn += z;
        sn2 += z * z;
        sn4 += z * z * z * z;
    }
    CHECK(lo >= 0);
    CHECK(hi < 1);
    CHECK_NEAR(su / n, 0.5, 0.003);
    CHECK_NEAR(su2 / n - 0.25, 1.0 / 12.0, 0.002);
    CHECK_NEAR(sn / n, 0.0, 0.01);
    CHECK_NEAR(sn2 / n, 1.0, 0.01);
    CHECK_NEAR(sn4 / n, 3.0, 0.06);  // Gaussian kurtosis
}

TEST_CASE("units: the exact definitions and the ballistic coefficient") {
    CHECK_NEAR(units::grains(7000), 0.45359237, 1e-15);
    CHECK_NEAR(units::grains(175), 0.011339809, 1e-9);
    CHECK_NEAR(units::fps(2600), 792.48, 1e-9);
    CHECK_NEAR(units::moa(1) * 100.0, 0.0290888, 1e-7);  // 1 MOA ~ 2.9 cm at 100 m
    CHECK_NEAR(units::kBcLbPerSqInToSi, 703.0696, 1e-4);
    CHECK_NEAR(units::bcFromSi(units::bcToSi(0.505)), 0.505, 1e-15);
    CHECK_NEAR(units::fahrenheitToKelvin(59), 288.15, 1e-12);
    CHECK_NEAR(units::inHg(29.92), 101320.8, 0.1);
}

TEST_CASE("frames: azimuth and elevation, the line of fire is x downrange, y left, z up") {
    const Vec3 north = frames::directionFromAzEl(0, 0);
    const Vec3 east = frames::directionFromAzEl(0.5 * kPi, 0);
    CHECK_NEAR(north.y, 1, 1e-15);
    CHECK_NEAR(east.x, 1, 1e-15);
    CHECK_NEAR(east.y, 0, 1e-15);
    const Vec3 up30 = frames::directionFromAzEl(1.0, kPi / 6);
    CHECK_NEAR(up30.z, 0.5, 1e-15);
    Real az, el;
    frames::azElFromDirection(frames::directionFromAzEl(4.0, -0.2), az, el);
    CHECK_NEAR(az, 4.0, 1e-14);
    CHECK_NEAR(el, -0.2, 1e-14);

    // Firing east: downrange is +E, left is +N, up is +U.
    const frames::LineOfFire lof({10, 20, 30}, 0.5 * kPi);
    const Vec3 l = lof.fromWorld({110, 25, 28});
    CHECK_NEAR(l.x, 100, 1e-12);
    CHECK_NEAR(l.y, 5, 1e-12);
    CHECK_NEAR(l.z, -2, 1e-12);
    const Vec3 back = lof.toWorld(l);
    CHECK_NEAR(back.x, 110, 1e-12);
    CHECK_NEAR(back.y, 25, 1e-12);
    // Right-handed: x cross y = z.
    const Vec3 zz = cross(lof.x, lof.y);
    CHECK_NEAR(zz.z, 1, 1e-15);

    // Earth's rotation: vertical component at the poles, horizontal north at the equator.
    const Vec3 eq = frames::earthRotationEnu(0), pole = frames::earthRotationEnu(0.5 * kPi);
    CHECK_NEAR(eq.y, frames::kEarthRotationRate, 1e-20);
    CHECK_NEAR(pole.z, frames::kEarthRotationRate, 1e-20);
    // São Paulo (23.55 S): the vertical component points down.
    CHECK(frames::earthRotationEnu(-0.411).z < 0);
}

namespace {
// A small test body for the clock: a point with quadratic drag and gravity.
struct Body {
    Vec3 p, v;
    Body operator+(const Body& o) const { return {p + o.p, v + o.v}; }
    Body operator*(Real s) const { return {p * s, v * s}; }
};
Body deriv(Real, const Body& b) {
    const Real k = 1.0e-3;  // 1/m
    return {b.v, b.v * (-k * length(b.v)) + Vec3{0, 0, -kStandardGravity}};
}
struct Sim {
    FixedClock clock;
    Body b{{0, 0, 0}, {800, 0, 10}};
    void advance(Real t) {
        clock.advance(t, [&](Real t0, Real h) { b = rk4(b, t0, h, deriv); });
    }
};
}  // namespace

TEST_CASE("clock: results do not depend on how the host chops its frames") {
    Sim a, b, c, d;
    for (int f = 1; f <= 60; ++f) a.advance(f / 60.0);
    for (int f = 1; f <= 144; ++f) b.advance(f / 144.0);
    c.advance(1.0);
    // A host with hitches and jitter.
    Rng r(5);
    Real t = 0;
    while (t < 1.0) {
        t = std::min(1.0, t + r.uniform(0.0, 0.05));
        d.advance(t);
    }
    CHECK(a.clock.steps() == 1000);
    CHECK(b.clock.steps() == a.clock.steps());
    CHECK(c.clock.steps() == a.clock.steps());
    CHECK(d.clock.steps() == a.clock.steps());
    // Bit for bit, not just close.
    CHECK(bitsOf(a.b.p.x) == bitsOf(b.b.p.x));
    CHECK(bitsOf(a.b.p.z) == bitsOf(c.b.p.z));
    CHECK(bitsOf(a.b.v.x) == bitsOf(d.b.v.x));
    CHECK(bitsOf(a.b.v.z) == bitsOf(d.b.v.z));
    // Never runs past the host's time, and a capped call leaves the rest owed.
    FixedClock k;
    int ran = 0;
    k.advance(0.0105, [&](Real, Real) { ++ran; });
    CHECK(ran == 10);
    CHECK(k.time() <= 0.0105);
    CHECK(k.advance(1.0, [](Real, Real) {}, 100) == 100);
    CHECK(k.due(1.0) == 890);
}

TEST_CASE("clock: RK4 is fourth order") {
    // dy/dt = -y: global error ~ h^4.
    auto run = [](Real h) {
        Real y = 1, t = 0;
        const int n = static_cast<int>(1.0 / h + 0.5);
        for (int i = 0; i < n; ++i) {
            y = rk4(y, t, h, [](Real, Real v) { return -v; });
            t += h;
        }
        return std::abs(y - dm::exp(-1.0));
    };
    const Real e1 = run(0.1), e2 = run(0.05);
    CHECK_RANGE(e1 / e2, 14.0, 18.0);
}

TEST_CASE("json: parses, reports line and column, rejects what it should") {
    const auto ok = json::parse(R"({"a": [1, 2.5e-3, -0, true, null], "s": "xé\n", "o": {}})");
    CHECK(ok.ok());
    CHECK(ok.value.find("a")->items().size() == 5);
    CHECK_NEAR(ok.value.find("a")->items()[1].asNumber(), 2.5e-3, 0);
    CHECK(ok.value.find("s")->asString() == "x\xc3\xa9\n");
    const auto dup = json::parse("{\"a\": 1,\n \"a\": 2}");
    CHECK(!dup.ok());
    CHECK(dup.error.find("line 2") != std::string::npos);
    CHECK(dup.error.find("duplicate") != std::string::npos);
    CHECK(!json::parse("[1, 2,]").ok());
    CHECK(!json::parse("{\"a\": 01}").ok());
    CHECK(!json::parse("[1e999]").ok());
    CHECK(!json::parse("\"abc").ok());
    CHECK(!json::parse("[1] x").ok());
    std::string deep(100, '[');
    deep += std::string(100, ']');
    CHECK(!json::parse(deep).ok());
}
