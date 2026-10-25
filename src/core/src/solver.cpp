// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/solver.hpp"

#include <limits>

namespace bd {

namespace {

constexpr Real kNaN = std::numeric_limits<Real>::quiet_NaN();

std::vector<FlightSample> fly(const RangeSetup& s, const AirSource& air, Real elevation, Real maxRange) {
    const frames::LineOfFire lof(s.origin, s.azimuth);
    Launch l;
    l.position = s.origin;
    l.direction = frames::directionFromAzEl(s.azimuth, elevation);
    l.muzzleSpeed = s.muzzleSpeed;
    l.time = s.time;
    PointMassOptions o = s.physics;
    o.step = s.step;
    o.keepTrace = true;
    o.maxTime = 15.0;
    PointMassRound r(s.drag, air, l, o);
    while (!r.stopped() && lof.fromWorld(r.position()).x < maxRange) r.advance(r.time() + 0.05);
    // One more step past the last range so the crossing is bracketed.
    if (!r.stopped()) r.advance(r.time() + s.step);
    return r.trace();
}

}  // namespace

bool crossing(const std::vector<FlightSample>& trace, const frames::LineOfFire& lof, Real range, FlightSample& out) {
    for (std::size_t i = 1; i < trace.size(); ++i) {
        const Real x0 = lof.fromWorld(trace[i - 1].position).x, x1 = lof.fromWorld(trace[i].position).x;
        if (x0 <= range && x1 >= range) {
            // Bisection in time on the Hermite cubic (downrange is monotonic within a step).
            Real lo = trace[i - 1].time, hi = trace[i].time;
            for (int k = 0; k < 60; ++k) {
                const Real mid = 0.5 * (lo + hi);
                if (lof.fromWorld(hermite(trace[i - 1], trace[i], mid).position).x < range)
                    lo = mid;
                else
                    hi = mid;
            }
            out = hermite(trace[i - 1], trace[i], 0.5 * (lo + hi));
            return true;
        }
    }
    return false;
}

Real zeroElevation(const RangeSetup& s, const AirSource& air) {
    const frames::LineOfFire lof(s.origin, s.azimuth);
    // Height of the round above the line of sight at the zero range.
    auto miss = [&](Real e, bool& ok) {
        FlightSample c;
        ok = crossing(fly(s, air, e, s.zeroRange), lof, s.zeroRange, c);
        return ok ? lof.fromWorld(c.position).z - s.sightHeight : 0.0;
    };
    // Start from the vacuum estimate and walk by the secant method.
    const Real x = s.zeroRange;
    Real e0 = (s.sightHeight + 0.5 * kStandardGravity * sq(x / s.muzzleSpeed)) / x;
    Real e1 = e0 * 1.5 + 1e-4;
    bool ok0, ok1;
    Real m0 = miss(e0, ok0), m1 = miss(e1, ok1);
    if (!ok0 || !ok1) return kNaN;
    for (int i = 0; i < 30 && std::abs(m1) > 1e-7; ++i) {
        if (m1 == m0) break;
        const Real e2 = e1 - m1 * (e1 - e0) / (m1 - m0);
        e0 = e1;
        m0 = m1;
        e1 = e2;
        m1 = miss(e1, ok1);
        if (!ok1) return kNaN;
    }
    return std::abs(m1) <= 1e-5 ? e1 : kNaN;
}

std::vector<RangeRow> rangeTable(const RangeSetup& s, const AirSource& air, Real elevation,
                                 const std::vector<Real>& ranges) {
    const frames::LineOfFire lof(s.origin, s.azimuth);
    Real far = 0;
    for (Real r : ranges) far = r > far ? r : far;
    const std::vector<FlightSample> trace = fly(s, air, elevation, far);
    std::vector<RangeRow> rows;
    for (Real r : ranges) {
        RangeRow row;
        row.range = r;
        FlightSample c;
        if (r <= 0) {
            c = trace.front();
            row.reached = true;
        } else {
            row.reached = crossing(trace, lof, r, c);
        }
        if (row.reached) {
            const Vec3 l = lof.fromWorld(c.position);
            row.drop = l.z - s.sightHeight;
            row.windage = -l.y;
            row.time = c.time - trace.front().time;
            row.speed = length(c.velocity);
            row.energy = 0.5 * s.mass * row.speed * row.speed;
            const AirSample a = air.sample(c.position, c.time);
            row.mach = length(c.velocity - a.wind) / a.speedOfSound;
            row.spin = c.spin;
            if (s.physics.spin.enabled) {
                // Sg at the crossing, from a round with the same set-up.
                Launch muzzle;
                muzzle.muzzleSpeed = s.muzzleSpeed;
                const PointMassRound probe(s.drag, air, muzzle, s.physics);
                PointMassRound::State st{c.position, c.velocity, c.spin, c.roll};
                row.stability = probe.stabilityAt(st, a);
            }
        }
        rows.push_back(row);
    }
    return rows;
}

}  // namespace bd
