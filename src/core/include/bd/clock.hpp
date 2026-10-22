// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Fixed internal steps (ADR 0002). Engine time is an integer count of steps
// times the step, never a running sum of the host's frame times, so a host at
// 30 Hz, 60 Hz, 144 Hz or with hitches reaches the same state at the same time.
// `advance(t)` runs the whole steps that fit before t; the remainder waits for
// the next call. One call runs at most `maxSteps` steps: a long hitch slows the
// engine down instead of freezing the host.
//
// The classic fourth-order Runge-Kutta step for any state with +, * by a scalar.
#pragma once

#include <cstdint>

#include "bd/math.hpp"

namespace bd {

inline constexpr Real kDefaultStep = 1.0 / 1000.0;  // s (ADR 0002)

class FixedClock {
public:
    explicit FixedClock(Real step = kDefaultStep) : dt_(step > 0 ? step : kDefaultStep) {}
    Real step() const { return dt_; }
    std::int64_t steps() const { return n_; }
    Real time() const { return static_cast<Real>(n_) * dt_; }
    // Time of step k (the state after k steps).
    Real timeOf(std::int64_t k) const { return static_cast<Real>(k) * dt_; }
    // Steps owed to reach `t` (whole steps only; time() <= t after them),
    // capped at maxSteps.
    std::int64_t due(Real t, std::int64_t maxSteps = 1 << 20) const {
        // A tolerance of 1e-9 step absorbs host rounding (k/60 summed).
        const Real target = t / dt_ + 1e-9;
        if (!(target >= 0)) return 0;
        std::int64_t k = static_cast<std::int64_t>(target) - n_;
        if (k < 0) k = 0;
        return k < maxSteps ? k : maxSteps;
    }
    // Runs fn(timeAtStepStart, dt) for every step due; returns how many ran.
    template <class Fn>
    std::int64_t advance(Real t, Fn&& fn, std::int64_t maxSteps = 1 << 20) {
        const std::int64_t k = due(t, maxSteps);
        for (std::int64_t i = 0; i < k; ++i) {
            fn(time(), dt_);
            ++n_;
        }
        return k;
    }
    // Runs exactly one step.
    template <class Fn>
    void stepOnce(Fn&& fn) {
        fn(time(), dt_);
        ++n_;
    }

private:
    Real dt_;
    std::int64_t n_ = 0;
};

// y(t + h) for dy/dt = f(t, y). State must support State + State and
// State * Real.
template <class State, class F>
State rk4(const State& y, Real t, Real h, F&& f) {
    const State k1 = f(t, y);
    const State k2 = f(t + 0.5 * h, y + k1 * (0.5 * h));
    const State k3 = f(t + 0.5 * h, y + k2 * (0.5 * h));
    const State k4 = f(t + h, y + k3 * h);
    return y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
}

}  // namespace bd
