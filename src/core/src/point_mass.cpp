// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/point_mass.hpp"

#include "bd/frames.hpp"

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
    r.roll = a.roll * h00 + a.spin * (h * h10) + b.roll * h01 + b.spin * (h * h11);
    r.spin = a.spin + s * (b.spin - a.spin);
    return r;
}

PointMassRound::PointMassRound(const DragModel& drag, const AirSource& air, const Launch& launch,
                               const PointMassOptions& options)
    : drag_(drag), air_(air), o_(options), clock_(options.step), launchTime_(launch.time) {
    Vec3 dir = normalized(launch.direction);
    if (o_.earthRotation) omega_ = frames::earthRotationEnu(o_.latitude);
    if (o_.spin.enabled) {
        // The crosswind at the muzzle: the jump turns the bore up or down, and
        // the round starts yawed into the relative wind (display wobble).
        // Both are set by the wind relative to the rifle (a moving rifle
        // makes its own crosswind). A vertical bore has no right or left:
        // east stands in for it.
        const AirSample a = air_.sample(launch.position, launch.time);
        Vec3 right = cross(dir, Vec3{0, 0, 1});
        right = dot(right, right) > 1e-18 ? normalized(right) : Vec3{1, 0, 0};
        const Vec3 up = cross(right, dir);
        const Real cw = dot(a.wind - launch.carrierVelocity, right);
        jump_ = aerodynamicJump(cw, o_.spin.sgMuzzle, o_.spin.calibres(), o_.spin.twist);
        Real sj, cj;
        dm::sincos(jump_, sj, cj);
        dir = dir * cj + up * sj;
        if (launch.muzzleSpeed > 0) yaw0_ = {cw / launch.muzzleSpeed, 0};
        s_.spin = o_.spin.muzzleSpin();
        minSg_ = o_.spin.sgMuzzle;
        // Unstable from the muzzle: it tumbles from the first step.
        if (o_.spin.sgMuzzle > 0 && o_.spin.sgMuzzle < 1.0) tumbling_ = true;
    }
    s_.p = launch.position;
    s_.v = dir * launch.muzzleSpeed + launch.carrierVelocity;
    prev_ = s_;
    prevTime_ = launchTime_;
    if (o_.keepTrace) trace_.push_back({launchTime_, s_.p, s_.v, s_.spin, s_.roll});
    if (drag_.kind() == DragModel::Kind::None) stopped_ = true;
}

Real PointMassRound::stabilityAt(const State& s, const AirSample& a) const {
    const SpinParams& sp = o_.spin;
    if (!sp.enabled || !(sp.sgMuzzle > 0)) return 0;
    const Real u = length(s.v - a.wind);
    const Real p0 = sp.muzzleSpin();
    if (!(u > 0) || p0 == 0 || !(a.density > 0)) return 0;
    const Real mach = a.speedOfSound > 0 ? u / a.speedOfSound : 0;
    // Sg ~ p^2 / (rho V^2 Cma): scaled from Miller's value at the muzzle.
    const Real ps = s.spin / p0, vs = sp.muzzleSpeed / u;
    return sp.sgMuzzle * ps * ps * vs * vs * (sp.muzzleDensity / a.density) *
           (overturningRatio(sp.muzzleMach) / overturningRatio(mach));
}

PointMassRound::State PointMassRound::derivative(Real t, const State& s) const {
    const AirSample a = air_.sample(s.p, t);
    const Vec3 u = s.v - a.wind;
    Vec3 dv = drag_.acceleration(u, a);
    if (tumbling_) dv *= o_.tumbleDragFactor;
    // The non-drag accelerations curve the path: gravity and, on a rotating
    // Earth, Coriolis (-2 Omega x v, whose vertical part is the Eotvos effect).
    Vec3 curve = o_.gravity;
    if (o_.earthRotation) curve += cross(omega_, s.v) * -2.0;
    dv += curve;
    State d{s.v, dv, 0, 0};
    const SpinParams& sp = o_.spin;
    if (sp.enabled) {
        const Real u2 = dot(u, u);
        const Real area = 0.25 * kPi * sp.diameter * sp.diameter;
        if (u2 > 0 && s.spin != 0 && !tumbling_) {
            // Yaw of repose and its lift: the spin drift.
            const Real sg = stabilityAt(s, a);
            const Vec3 alpha = cross(u, curve) * (-4.0 * sp.iyOverIx * sg / (s.spin * u2));
            d.v += alpha * (0.5 * a.density * u2 * area * sp.liftSlope / sp.mass);
        }
        // Roll damping: Ix dp/dt = 0.5 rho |u| S d^2 Clp p.
        d.spin = 0.5 * a.density * std::sqrt(u2) * area * sp.diameter * sp.diameter * sp.rollDamping * s.spin / sp.ix();
        d.roll = s.spin;
    }
    return d;
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
        if (o_.keepTrace) trace_.push_back({time(), s_.p, s_.v, s_.spin, s_.roll});
        if (o_.spin.enabled) {
            const AirSample a = air_.sample(s_.p, time());
            const Real sg = stabilityAt(s_, a);
            if (sg < minSg_) minSg_ = sg;
            if (sg < 1.0) tumbling_ = true;  // gyroscopically unstable: it tumbles
            const Real mach = length(s_.v - a.wind) / a.speedOfSound;
            if (transonicStart_ < 0 && mach < 1.2) transonicStart_ = flightTime();
            if (subsonicTime_ < 0 && mach < 1.0) subsonicTime_ = flightTime();
        }
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
    const FlightSample now{time(), s_.p, s_.v, s_.spin, s_.roll};
    if (t >= now.time) return now;
    if (!trace_.empty() && t < prevTime_) {
        if (t <= trace_.front().time) return trace_.front();
        // Steps are uniform: index directly.
        const Real k = (t - launchTime_) / clock_.step();
        std::size_t i = static_cast<std::size_t>(k);
        if (i + 1 >= trace_.size()) i = trace_.size() - 2;
        return hermite(trace_[i], trace_[i + 1], t);
    }
    return hermite({prevTime_, prev_.p, prev_.v, prev_.spin, prev_.roll}, now, t);
}

Vec3 PointMassRound::noseAt(Real t) const {
    const FlightSample f = stateAt(t);
    const AirSample a = air_.sample(f.position, f.time);
    const Vec3 u = f.velocity - a.wind;
    const Vec3 fwd = normalized(u);
    if (!o_.spin.enabled || tumbling_) return fwd;
    const Vec3 right = normalized(cross(fwd, Vec3{0, 0, 1}));
    const Vec3 up = cross(right, fwd);
    // The yaw of repose (as in derivative()) plus the wobble.
    State s{f.position, f.velocity, f.spin, f.roll};
    const Real u2 = dot(u, u);
    Vec3 repose;
    if (u2 > 0 && f.spin != 0) {
        Vec3 curve = o_.gravity;
        if (o_.earthRotation) curve += cross(omega_, f.velocity) * -2.0;
        repose = cross(u, curve) * (-4.0 * o_.spin.iyOverIx * stabilityAt(s, a) / (f.spin * u2));
    }
    const Yaw2 w = epicyclicYaw(o_.spin, yaw0_, f.time - launchTime_);
    return normalized(fwd + repose + right * w.right + up * w.up);
}

}  // namespace bd
