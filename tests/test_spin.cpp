// Ballistic Dynamics — spin and the Earth (Milestone 3): Miller stability,
// spin drift against Litz, aerodynamic jump, Coriolis and Eotvos against
// their closed forms, the wobble, tumbling, determinism.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstdlib>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/solver.hpp"
#include "bd/spin.hpp"
#include "bd/units.hpp"
#include "bd_test.hpp"

using namespace bd;

namespace {

const StationAir& icao() {
    static const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    return air;
}

struct Shot {
    DragModel drag;
    const CartridgeDef* cartridge = nullptr;
    const ProjectileDef* projectile = nullptr;
    Real elevation = 0;
};

Shot shotFor(const char* cartridge) {
    const Library& lib = bdtest::library();
    Shot s;
    s.cartridge = lib.cartridge(cartridge);
    s.projectile = lib.projectile(s.cartridge->projectile);
    std::string err;
    s.drag = dragModelFor(*s.projectile, lib, "", err);
    RangeSetup r;
    r.drag = s.drag;
    r.muzzleSpeed = s.cartridge->muzzleVelocity;
    r.sightHeight = units::inches(1.5);
    r.zeroRange = units::yards(100);
    s.elevation = zeroElevation(r, icao());
    return s;
}

// Flies a round north (or along `azimuth`) past a downrange distance.
PointMassRound flyRound(const Shot& s, const AirSource& air, const PointMassOptions& base, Real range, Real azimuth = 0) {
    PointMassOptions o = base;
    o.keepTrace = true;
    Launch l;
    l.direction = frames::directionFromAzEl(azimuth, s.elevation);
    l.muzzleSpeed = s.cartridge->muzzleVelocity;
    PointMassRound r(s.drag, air, l, o);
    const frames::LineOfFire lof({}, azimuth);
    while (!r.stopped() && lof.fromWorld(r.position()).x < range) r.advance(r.time() + 0.02);
    r.advance(r.time() + 0.002);
    return r;
}

FlightSample crossAt(const PointMassRound& r, Real range, Real azimuth = 0) {
    FlightSample c;
    crossing(r.trace(), frames::LineOfFire({}, azimuth), range, c);
    return c;
}

FlightSample flyTo(const Shot& s, const AirSource& air, const PointMassOptions& base, Real range) {
    return crossAt(flyRound(s, air, base, range), range);
}

PointMassOptions withSpin(const Shot& s, Real twist) {
    PointMassOptions o;
    o.spin = spinParamsFor(*s.projectile, twist, s.cartridge->muzzleVelocity, icao().sample({}, 0));
    return o;
}

}  // namespace

TEST_CASE("spin: Miller's rule, its corrections and the spin rate") {
    const ProjectileDef& smk = *bdtest::library().projectile("smk_308_175");
    // At 2800 fps, 59 F and 29.92 inHg the corrections are 1: the bare rule.
    const Real d = 0.308, l = 1.24 / 0.308, t = 11.25 / 0.308;
    const Real bare = 30.0 * 175 / (t * t * d * d * d * l * (1 + l * l));
    CHECK_NEAR(millerStability(smk.mass, units::inches(11.25), smk.diameter, smk.length, units::fps(2800),
                               units::fahrenheitToKelvin(59), units::inHg(29.92)),
               bare, 1e-12);
    // Twice the twist rate, four times the stability; thinner air, more.
    const Real sg10 = millerStability(smk.mass, units::inches(10), smk.diameter, smk.length, units::fps(2600), 288.15, 101325);
    const Real sg5 = millerStability(smk.mass, units::inches(5), smk.diameter, smk.length, units::fps(2600), 288.15, 101325);
    CHECK_NEAR(sg5 / sg10, 4.0, 1e-12);
    CHECK(millerStability(smk.mass, units::inches(10), smk.diameter, smk.length, units::fps(2600), 288.15, 80000) > sg10);
    // The M24's 1:11.25 with the 175 SMK at 2600 fps on a standard day.
    const SpinParams sp = spinParamsFor(smk, units::inches(11.25), units::fps(2600), icao().sample({}, 0));
    CHECK_RANGE(sp.sgMuzzle, 1.8, 2.0);
    // 2600 fps through 1:11.25 is 2,773 turns a second.
    CHECK_NEAR(sp.muzzleSpin() / kTwoPi, 2600.0 * 12.0 / 11.25, 1e-9);
    CHECK(spinParamsFor(smk, -units::inches(11.25), units::fps(2600), icao().sample({}, 0)).muzzleSpin() < 0);
}

TEST_CASE("spin: drift to the right for a right-hand twist, against Litz's published formula") {
    struct Case {
        const char* cartridge;
        Real twistIn, yd;
    };
    // Litz's empirical spin drift, 1.25 (Sg + 1.2) TOF^1.83 in, is fitted to
    // rifle bullets. The engine's yaw-of-repose lift (estimated CLa and
    // inertia ratio) lands at 0.79-0.98 of it without tuning
    // (docs/validation.md).
    const Case cases[] = {{"fgmm_308_175", 11.25, 1000}, {"hornady_65cm_140", 8, 1000}, {"lapua_338lm_250", 10, 1500},
                          {"hornady_50bmg_750", 15, 2000}, {"federal_xm855", 7, 600}};
    for (const Case& c : cases) {
        const Shot s = shotFor(c.cartridge);
        const PointMassOptions o = withSpin(s, units::inches(c.twistIn));
        const PointMassRound r = flyRound(s, icao(), o, units::yards(c.yd));
        const FlightSample at = crossAt(r, units::yards(c.yd));
        const Real drift = at.position.x;  // firing north: east is right
        const Real litz = litzSpinDrift(o.spin.sgMuzzle, at.time);
        if (std::getenv("BD_VERBOSE"))
            std::printf("  %-18s Sg %.2f  TOF %.3f s  drift %.1f in  Litz %.1f in  (%.2f)\n", c.cartridge, o.spin.sgMuzzle,
                        at.time, units::toInches(drift), units::toInches(litz), drift / litz);
        CHECK_RANGE(drift / litz, 0.7, 1.15);
        CHECK(!r.tumbling());
        CHECK(r.minStability() >= 1.0);
    }
    // A left-hand twist drifts left by the same amount.
    const Shot s = shotFor("fgmm_308_175");
    const Real right = flyTo(s, icao(), withSpin(s, units::inches(11.25)), units::yards(1000)).position.x;
    const Real left = flyTo(s, icao(), withSpin(s, -units::inches(11.25)), units::yards(1000)).position.x;
    CHECK_NEAR(left, -right, 1e-6);
    // Without spin, no drift at all.
    CHECK_NEAR(flyTo(s, icao(), PointMassOptions{}, units::yards(1000)).position.x, 0.0, 1e-12);
}

TEST_CASE("spin: the spin decays slowly and the round grows more stable as it slows") {
    const Shot s = shotFor("fgmm_308_175");
    const PointMassRound r = flyRound(s, icao(), withSpin(s, units::inches(11.25)), units::yards(1000));
    const Real p0 = r.trace().front().spin, p1 = r.trace().back().spin;
    const Real v0 = length(r.trace().front().velocity), v1 = length(r.trace().back().velocity);
    CHECK(p1 < p0);
    CHECK(p1 / p0 > v1 / v0);  // spin decays more slowly than the speed
    CHECK(r.stability() > r.options().spin.sgMuzzle);
    // The roll angle is the integral of the spin.
    Real integral = 0;
    for (std::size_t i = 1; i < r.trace().size(); ++i)
        integral += 0.5 * (r.trace()[i].spin + r.trace()[i - 1].spin) * (r.trace()[i].time - r.trace()[i - 1].time);
    CHECK_NEAR(r.roll() / integral, 1.0, 1e-5);
}

TEST_CASE("spin: aerodynamic jump from a crosswind (Litz)") {
    const Shot s = shotFor("fgmm_308_175");
    const PointMassOptions o = withSpin(s, units::inches(11.25));
    const Real mph = 0.44704;
    // 10 mph from the left (from the west, firing north: blowing east, to the right).
    const RangeWind fromLeft(icao(), {}, 0.0, {{0, 10 * mph, 1.5 * kPi}});
    const RangeWind fromRight(icao(), {}, 0.0, {{0, 10 * mph, 0.5 * kPi}});
    Launch l;
    l.direction = frames::directionFromAzEl(0, s.elevation);
    l.muzzleSpeed = s.cartridge->muzzleVelocity;
    const PointMassRound a(s.drag, fromLeft, l, o), b(s.drag, fromRight, l, o);
    const Real expected = (0.01 * o.spin.sgMuzzle - 0.0024 * o.spin.calibres() + 0.032) * 10 * units::kMoa;
    // Right-hand twist: down in a wind from the left, up in one from the right
    // (the nose yawed right precesses clockwise, seen from behind, to down).
    CHECK_NEAR(a.jumpAngle(), -expected, 1e-12);
    CHECK_NEAR(b.jumpAngle(), expected, 1e-12);
    // About 0.4 MOA for the .308 in 10 mph: 4 in at 1,000 yd.
    CHECK_RANGE(units::toInches(b.jumpAngle() * units::yards(1000)), 3.0, 5.5);
    // A left-hand twist turns it the other way; no spin, no jump.
    PointMassOptions lh = withSpin(s, -units::inches(11.25));
    CHECK_NEAR(PointMassRound(s.drag, fromLeft, l, lh).jumpAngle(), expected, 1e-12);
    CHECK(PointMassRound(s.drag, fromLeft, l, PointMassOptions{}).jumpAngle() == 0);
}

TEST_CASE("earth: Coriolis and Eotvos against their closed forms") {
    const DragModel none = DragModel::bullet(DragTable({0, 5}, {0, 0}), 0.01, 0.008);
    const UniformAir air;
    const Real lat = 45.0 * kPi / 180.0, w = frames::kEarthRotationRate, V = 800, T = 2.0;
    auto fly = [&](Real az, Real latitude) {
        PointMassOptions o;
        o.earthRotation = true;
        o.latitude = latitude;
        o.gravity = {};  // flat fire, isolate the rotation
        Launch l;
        l.direction = frames::directionFromAzEl(az, 0);
        l.muzzleSpeed = V;
        PointMassRound r(none, air, l, o);
        r.advance(T);
        return frames::LineOfFire({}, az).fromWorld(r.position());
    };
    // Firing north in the northern hemisphere: deflected right by w sin(lat) V T^2.
    const Vec3 n = fly(0, lat);
    CHECK_NEAR(-n.y / (w * dm::sin(lat) * V * T * T), 1.0, 1e-3);
    // Firing east: the Eotvos effect lifts the round by w cos(lat) V T^2.
    const Vec3 e = fly(0.5 * kPi, lat);
    CHECK_NEAR(e.z / (w * dm::cos(lat) * V * T * T), 1.0, 1e-3);
    // Firing west it falls by as much; at the equator, no horizontal deflection.
    CHECK_NEAR(fly(1.5 * kPi, lat).z, -e.z, 1e-3 * e.z);
    CHECK_NEAR(fly(0, 0).y, 0.0, 1e-9);
    // São Paulo (23.55 S): deflected left.
    CHECK(fly(0, -23.55 * kPi / 180.0).y > 0);
}

TEST_CASE("earth: Coriolis on a real long shot, with drag") {
    // Firing north, the sideways (east) Coriolis acceleration is
    // 2 w (sin(lat) V_n - cos(lat) V_up), and drag damps the sideways
    // velocity at the rate r = k|u| it damps the forward one (r = -V_n'/V_n).
    // The deflection is that linear equation, y'' = -r y' + 2 w (...),
    // integrated along the flight without the rotation: an independent check
    // of the engine's 3-D cross product. Without drag and drop it would be
    // the familiar w sin(lat) X T; with drag it is less (the late, slow part of
    // the flight weighs most).
    const Shot s = shotFor("hornady_50bmg_750");
    PointMassOptions o;
    o.earthRotation = true;
    o.latitude = 45.0 * kPi / 180.0;
    const Real X = units::yards(2000);
    const FlightSample with = flyTo(s, icao(), o, X);
    const PointMassRound plain = flyRound(s, icao(), PointMassOptions{}, X);
    const FlightSample without = crossAt(plain, X);
    const Real sl = dm::sin(o.latitude), cl = dm::cos(o.latitude), w = frames::kEarthRotationRate;
    Real y = 0, yp = 0;
    for (std::size_t i = 1; i < plain.trace().size(); ++i) {
        const FlightSample& a = plain.trace()[i - 1];
        const FlightSample& b = plain.trace()[i];
        if (a.time >= without.time) break;
        const Real dt = std::min(b.time, without.time) - a.time;
        const Real r = -(b.velocity.y - a.velocity.y) / (0.5 * (a.velocity.y + b.velocity.y) * (b.time - a.time));
        const Real f = 0.5 * ((sl * a.velocity.y - cl * a.velocity.z) + (sl * b.velocity.y - cl * b.velocity.z));
        const Real yp1 = yp + (-r * yp + 2.0 * w * f) * dt;
        y += 0.5 * (yp + yp1) * dt;
        yp = yp1;
    }
    const Real exact = y;
    const Real deflection = with.position.x - without.position.x;
    CHECK_NEAR(deflection / exact, 1.0, 0.003);
    const Real familiar = frames::kEarthRotationRate * dm::sin(o.latitude) * X * with.time;
    CHECK(deflection < familiar);
    // About 10 in to the right at 2,000 yd at 45 N.
    CHECK_RANGE(units::toInches(deflection), 9.0, 11.5);
}

TEST_CASE("spin: the wobble starts from the crosswind yaw and dies out; the nose follows the path") {
    const Shot s = shotFor("fgmm_308_175");
    PointMassOptions o = withSpin(s, units::inches(11.25));
    o.keepTrace = true;
    o.spin.tipOffRate = 3.0;  // rad/s: a little tip-off
    const RangeWind wind(icao(), {}, 0.0, {{0, 5, 1.5 * kPi}});
    Launch l;
    l.direction = frames::directionFromAzEl(0, s.elevation);
    l.muzzleSpeed = s.cartridge->muzzleVelocity;
    PointMassRound r(s.drag, wind, l, o);
    r.advance(1.0);
    const Yaw2 y0 = epicyclicYaw(o.spin, {5.0 / l.muzzleSpeed, 0}, 0);
    CHECK_NEAR(y0.right, 5.0 / l.muzzleSpeed, 1e-12);
    Real early = 0, late = 0;
    for (int k = 0; k < 200; ++k) {
        const Yaw2 a = epicyclicYaw(o.spin, {5.0 / l.muzzleSpeed, 0}, 0.0005 * k);
        early = std::max(early, std::hypot(a.right, a.up));
        const Yaw2 b = epicyclicYaw(o.spin, {5.0 / l.muzzleSpeed, 0}, 0.8 + 0.0005 * k);
        late = std::max(late, std::hypot(b.right, b.up));
    }
    CHECK(early > 0.005);         // a few mrad near the muzzle
    // A right-hand spin turns the nose clockwise seen from behind: from right
    // to down first; a left-hand spin from right to up.
    PointMassOptions still = o;
    still.spin.tipOffRate = 0;
    const Yaw2 q = epicyclicYaw(still.spin, {0.006, 0}, 0.0002);
    CHECK(q.up < 0);
    PointMassOptions lhw = withSpin(s, -units::inches(11.25));
    CHECK(epicyclicYaw(lhw.spin, {0.006, 0}, 0.0002).up > 0);
    CHECK(late < 0.02 * early);   // gone by 0.8 s
    // The nose stays within a degree of the air-relative path.
    for (Real t : {0.0005, 0.01, 0.3, 0.9}) {
        const FlightSample f = r.stateAt(t);
        const Vec3 u = normalized(f.velocity - wind.sample(f.position, t).wind);
        CHECK(dot(r.noseAt(t), u) > dm::cos(kPi / 180));
    }
}

TEST_CASE("spin: a round fired from too slow a twist is unstable and tumbles") {
    const Shot s = shotFor("fgmm_308_175");
    const PointMassOptions o = withSpin(s, units::inches(20));  // 1:20 for a 175 SMK
    CHECK(o.spin.sgMuzzle < 1.0);
    const PointMassRound r = flyRound(s, icao(), o, units::yards(300));
    const FlightSample bad = crossAt(r, units::yards(300));
    const FlightSample good = flyTo(s, icao(), withSpin(s, units::inches(11.25)), units::yards(300));
    CHECK(r.tumbling());
    CHECK(bad.position.z < good.position.z - 0.2);   // drops much more
    CHECK(length(bad.velocity) < 0.7 * length(good.velocity));
}

TEST_CASE("determinism: a spinning round on a rotating Earth gives the same bits everywhere") {
    const std::uint64_t h = spinProbe();
    if (h != kSpinGolden) {
        ++bdtest::failures();
        std::fprintf(stderr, "  spin hash 0x%016llx, expected 0x%016llx\n", static_cast<unsigned long long>(h),
                     static_cast<unsigned long long>(kSpinGolden));
    }
}
