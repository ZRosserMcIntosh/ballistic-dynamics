// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/point_mass.hpp"

namespace bd {

FlightSample hermite(const FlightSample& a, const FlightSample& b, Real t) {
    const Real h = b.time - a.time;
    if (!(h > 0)) return a;
    const Real s = clamp((t - a.time) / h, 0.0, 1.0);
    const Real s2 = s * s, s3 = s2 * s;
    // Position: Hermite basis on positions and h * velocities.
    const Real h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
    // Velocity: the derivative of the same cubic.
    const Real d00 = (6 * s2 - 6 * s) / h, d10 = 3 * s2 - 4 * s + 1, d01 = (-6 * s2 + 6 * s) / h, d11 = 3 * s2 - 2 * s;
    FlightSample r;
    r.time = a.time + s * h;
    r.position = a.position * h00 + a.velocity * (h * h10) + b.position * h01 + b.velocity * (h * h11);
    r.velocity = a.position * d00 + a.velocity * d10 + b.position * d01 + b.velocity * d11;
    return r;
}

PointMassRound::PointMassRound(const DragModel& drag, const AirSource& air, const Launch& launch,
                               const PointMassOptions& options)
    : drag_(drag), air_(air), o_(options), clock_(options.step), launchTime_(launch.time) {
    s_.p = launch.position;
    s_.v = normalized(launch.direction) * launch.muzzleSpeed + launch.carrierVelocity;
    prev_ = s_;
    prevTime_ = launchTime_;
    if (o_.keepTrace) trace_.push_back({launchTime_, s_.p, s_.v});
    if (drag_.kind() == DragModel::Kind::None) stopped_ = true;
}

PointMassRound::State PointMassRound::derivative(Real t, const State& s) const {
    const AirSample a = air_.sample(s.p, t);
    return {s.v, drag_.acceleration(s.v - a.wind, a) + o_.gravity};
}

void PointMassRound::step(Real t0, Real h) {
    prev_ = s_;
    prevTime_ = launchTime_ + t0;
    s_ = rk4(s_, launchTime_ + t0, h, [this](Real t, const State& s) { return derivative(t, s); });
}

std::int64_t PointMassRound::advance(Real t, std::int64_t maxSteps) {
    if (stopped_) return 0;
    std::int64_t ran = 0;
    const std::int64_t due = clock_.due(t - launchTime_, maxSteps);
    for (std::int64_t i = 0; i < due && !stopped_; ++i) {
        clock_.stepOnce([this](Real t0, Real h) { step(t0, h); });
        ++ran;
        if (o_.keepTrace) trace_.push_back({time(), s_.p, s_.v});
        if (clock_.time() >= o_.maxTime || length(s_.v) < o_.minSpeed || !isFinite(s_.p) || !isFinite(s_.v))
            stopped_ = true;
    }
    return ran;
}

void PointMassRound::runToEnd(std::int64_t maxSteps) {
    while (!stopped_ && maxSteps > 0) {
        const std::int64_t n = advance(time() + 1.0, maxSteps);
        if (n == 0) break;
        maxSteps -= n;
    }
}

FlightSample PointMassRound::stateAt(Real t) const {
    const FlightSample now{time(), s_.p, s_.v};
    if (t >= now.time) return now;
    if (!trace_.empty() && t < prevTime_) {
        if (t <= trace_.front().time) return trace_.front();
        // Steps are uniform: index directly.
        const Real k = (t - launchTime_) / clock_.step();
        std::size_t i = static_cast<std::size_t>(k);
        if (i + 1 >= trace_.size()) i = trace_.size() - 2;
        return hermite(trace_[i], trace_[i + 1], t);
    }
    return hermite({prevTime_, prev_.p, prev_.v}, now, t);
}

}  // namespace bd
