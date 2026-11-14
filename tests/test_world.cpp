// Ballistic Dynamics — the round meets the world (ADR 0006): the host's
// segment query, the 25 m no-lead rule, the chord between queries, and the
// fast paths (the air column, the drag table's index).
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstdlib>

#include "bd/aviation_air.hpp"
#include "bd/rng.hpp"
#include "bd/world.hpp"
#include "bd_test.hpp"

using namespace bd;

namespace {

const StationAir& stationAir() {
    static const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    return air;
}

DragModel m855() {
    std::string err;
    const Library& lib = bdtest::library();
    return dragModelFor(*lib.projectile("m855_224_62"), lib, "", err);
}

Real m855Speed() { return bdtest::library().cartridge("federal_xm855")->muzzleVelocity; }

// A test world: a wall across the range at y = wallY (static), and one
// body, a sphere of `radius` crossing the line of fire at `speed` m/s east,
// at x = 0 at `t0`, centred at (0, bodyY, 1.5).
struct TestWorld final : WorldQuery {
    Real wallY = 1e9, bodyY = 1e9, radius = 0.1, speed = 0, t0 = 0;
    mutable int queries = 0;
    WorldHit segment(const Vec3& a, const Vec3& b, Real bodyTime) const override {
        ++queries;
        WorldHit best;
        best.fraction = 2;
        // The body: segment against a sphere.
        const Vec3 c{speed * (bodyTime - t0), bodyY, 1.5};
        const Vec3 d = b - a, m = a - c;
        const Real A = dot(d, d), B = dot(m, d), C = dot(m, m) - radius * radius;
        const Real disc = B * B - A * C;
        if (A > 0 && disc >= 0) {
            const Real f = (-B - std::sqrt(disc)) / A;
            if (f >= 0 && f <= 1) {
                best = {true, f, a + d * f, normalized(a + d * f - c), 7, 2, 1};
            }
        }
        // The wall.
        if ((a.y - wallY) * (b.y - wallY) <= 0 && a.y != b.y) {
            const Real f = (wallY - a.y) / (b.y - a.y);
            if (f < best.fraction) best = {true, f, a + d * f, {0, -1, 0}, 0, 0, 3};
        }
        if (!best.hit) best = {};
        return best;
    }
};

Launch level(Real t0 = 0) { return {{0, 0, 1.5}, {0, 1, 0}, m855Speed(), {}, t0}; }

}  // namespace

TEST_CASE("world: a round stops at the wall, where its own flight crosses it") {
    TestWorld w;
    w.wallY = 40;
    for (int k : {1, 4, 16}) {
        HitOptions h;
        h.querySteps = k;
        TrackedRound r(m855(), stationAir(), level(), {}, h);
        for (int f = 1; f <= 60 && !r.done(); ++f) r.advance(f / 60.0, w);
        CHECK(r.hit());
        const Impact& i = r.impact();
        CHECK(i.hit.material == 3 && i.hit.body == 0);
        CHECK_NEAR(i.position.y, 40.0, 1e-9);
        // Against the flight itself (kept trace, crossing interpolated).
        PointMassOptions o;
        o.keepTrace = true;
        PointMassRound ref(m855(), stationAir(), level(), o);
        while (ref.position().y < 40) ref.advance(ref.time() + 1e-3);
        const auto& tr = ref.trace();
        const FlightSample& b = tr.back();
        const FlightSample& a = tr[tr.size() - 2];
        const Real tc = a.time + (40 - a.position.y) / (b.position.y - a.position.y) * (b.time - a.time);
        CHECK_NEAR(i.time, tc, 2e-7);  // within 0.2 us, whatever the chord
        // The host's point is on the chord: off the path by at most its sag, g (k h)^2 / 8.
        const Real sag = kStandardGravity * sq(k * 1e-3) / 8;
        CHECK_NEAR(i.position.z, hermite(a, b, tc).position.z, sag + 1e-6);
        if (std::getenv("BD_VERBOSE"))
            std::printf("  wall at 40 m, %2d-step chords: time off %.2e s, height off %.2e m (sag %.1e)\n", k, i.time - tc,
                        i.position.z - hermite(a, b, tc).position.z, sag);
        CHECK_NEAR(i.velocity.y, hermite(a, b, tc).velocity.y, 0.05);
        CHECK_NEAR(i.distance, 40.0, 0.01);
        CHECK(!i.noLead);  // 40 m: beyond the no-lead range
    }
}

TEST_CASE("world: inside 25 m you hit what you aimed at; beyond it you must lead") {
    // A body 10 cm in radius crossing at 7 m/s (a sprint), aimed at where it
    // was at the shot (x = 0).
    TestWorld w;
    w.radius = 0.1;
    w.speed = 7.0;
    w.t0 = 0;
    {
        w.bodyY = 20;  // 22 ms away: it moves 15 cm, more than its radius
        TrackedRound r(m855(), stationAir(), level(), {}, {});
        for (int f = 1; f <= 30 && !r.done(); ++f) r.advance(f / 60.0, w);
        CHECK(r.hit() && r.impact().hit.body == 7 && r.impact().noLead);
        // Without the rule (no-lead range 0) the same shot passes behind it.
        HitOptions none;
        none.noLeadRange = 0;
        TrackedRound s(m855(), stationAir(), level(), {}, none);
        for (int f = 1; f <= 30 && !s.done(); ++f) s.advance(f / 60.0, w);
        CHECK(!s.hit() || s.impact().hit.body != 7);
    }
    {
        w.bodyY = 50;  // beyond the rule: aimed at x = 0, the round arrives after the body left
        TrackedRound r(m855(), stationAir(), level(), {}, {});
        for (int f = 1; f <= 30 && !r.done(); ++f) r.advance(f / 60.0, w);
        CHECK(!r.hit() || r.impact().hit.body != 7);
        // Led by the speed times the flight time, it hits.
        const Real tof = 0.0565;
        Launch l = level();
        l.direction = normalized(Vec3{w.speed * tof, 50, 0});
        TrackedRound s(m855(), stationAir(), l, {}, {});
        for (int f = 1; f <= 30 && !s.done(); ++f) s.advance(f / 60.0, w);
        CHECK(s.hit() && s.impact().hit.body == 7 && !s.impact().noLead);
    }
}

TEST_CASE("world: the frame of the click knows a close hit, and the rest carries on") {
    TestWorld w;
    w.bodyY = 15;
    w.radius = 0.3;
    TrackedRound r(m855(), stationAir(), level(0.5), {}, {});
    CHECK(r.resolveNoLead(w));                   // flown now, at the click
    CHECK(r.impact().time > 0.5 && r.impact().time < 0.52);  // its true time: about 16 ms after the shot
    // Nothing within 25 m: the round has flown the range, and the host's clock carries it on.
    TestWorld empty;
    TrackedRound s(m855(), stationAir(), level(0.5), {}, {});
    CHECK(!s.resolveNoLead(empty));
    CHECK(s.distance() >= 25.0 && s.distance() < 25.0 + 4 * 1.0);
    const Real ahead = s.round().time();
    CHECK(!s.advance(0.5 + 1 / 60.0, empty));     // the host's frame is behind the round: nothing runs
    CHECK(s.round().time() == ahead);
    s.advance(0.6, empty);
    CHECK(s.round().time() > ahead);
}

TEST_CASE("world: where a round lands does not depend on the frame rate") {
    TestWorld w;
    w.wallY = 180;
    const auto land = [&](int hz) {
        TrackedRound r(m855(), stationAir(), level(), {}, {});
        for (int f = 1; f <= hz && !r.done(); ++f) r.advance(static_cast<Real>(f) / hz, w);
        return r.impact();
    };
    const Impact a = land(60), b = land(144), c = land(1);
    CHECK(a.position == b.position && a.position == c.position);
    CHECK(bitsOf(a.time) == bitsOf(b.time) && bitsOf(a.time) == bitsOf(c.time));
}

TEST_CASE("world: the air column is the model to a part in 10^8; the drag index is the search") {
    const StationAir& s = stationAir();
    const ColumnAir col(s, {0, 0, 0}, 0, -50, 500);
    Rng r(5, 0);
    Real worst = 0;
    for (int i = 0; i < 2000; ++i) {
        const Real z = r.uniform(-50, 500);
        const AirSample a = s.sample({0, 0, z}, 0), b = col.sample({123, 456, z}, 7);
        worst = std::max(worst, std::abs(b.density / a.density - 1));
        CHECK_NEAR(b.speedOfSound, a.speedOfSound, 1e-6);
        CHECK_NEAR(b.pressure, a.pressure, 1e-3);
    }
    CHECK(worst < 1e-8);
    if (std::getenv("BD_VERBOSE")) std::printf("  column air: worst density error %.2e\n", worst);
    // Flown through the column, a 300 m flight lands within a micrometre of the model's.
    PointMassRound x(m855(), s, level()), y(m855(), col, level());
    while (x.position().y < 300) x.advance(x.time() + 1e-3);
    while (y.flightTime() < x.flightTime()) y.advance(y.time() + 1e-3);
    CHECK(length(x.position() - y.position()) < 1e-6);

    // The drag table's bucket index finds the binary search's interval:
    // every table point, and between them.
    const Library& lib = bdtest::library();
    for (const char* id : {"g1", "g7"}) {
        const DragCurveDef& d = *lib.dragCurve(id);
        const DragTable t(d);
        bool same = true;
        const auto bsearch = [&](Real m) {
            const auto& M = d.mach;
            const auto& C = d.cd;
            const std::size_t n = M.size();
            if (!(m > M[0])) return C[0];
            if (m >= M[n - 1]) return C[n - 1];
            std::size_t lo = 0, hi = n - 1;
            while (hi - lo > 1) {
                const std::size_t mid = (lo + hi) / 2;
                if (M[mid] <= m) lo = mid;
                else hi = mid;
            }
            const Real tt = (m - M[lo]) / (M[hi] - M[lo]);
            return C[lo] + tt * (C[hi] - C[lo]);
        };
        for (std::size_t k = 0; k < d.mach.size(); ++k) {
            for (Real e : {-1e-12, 0.0, 1e-12})
                same = same && bitsOf(t.cd(d.mach[k] + e)) == bitsOf(bsearch(d.mach[k] + e));
        }
        for (int i = 0; i < 100000; ++i) {
            const Real m = r.uniform(-0.1, 5.2);
            same = same && bitsOf(t.cd(m)) == bitsOf(bsearch(m));
        }
        CHECK(same);
    }
}
