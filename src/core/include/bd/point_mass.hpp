// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// The point-mass round (Milestone 2): position and velocity in the world
// frame (ENU), drag against the local air (its density, speed of sound and
// wind sampled at every Runge-Kutta stage), and gravity:
//
//   dp/dt = v
//   dv/dt = a_drag(v - w(p, t), air(p, t)) + g
//
// It steps on its own fixed clock (ADR 0002), which starts at the launch
// time, so a round's flight does not depend on how the host chops its
// frames or on when other rounds were fired.
//
// Bullet time. A renderer that shows the round in slow motion (a bullet
// camera at 1/1000 speed) does not need a smaller step: stateAt(t) gives the
// state at any time inside the flown steps by cubic Hermite interpolation of
// the step states (positions and velocities at both ends: fourth-order
// accurate, continuous in position and velocity), so the camera glides
// between steps instead of jumping 0.9 m at a time. Interpolation needs the
// step after the drawn time, so a camera advances the round one step ahead of
// what it draws: advance(t + step), then stateAt(t). Keep the trace to replay
// the whole flight.
#pragma once

#include <cstdint>
#include <vector>

#include "bd/air.hpp"
#include "bd/clock.hpp"
#include "bd/drag.hpp"
#include "bd/math.hpp"

namespace bd {

struct Launch {
    Vec3 position;            // muzzle, ENU (m)
    Vec3 direction{0, 1, 0};  // bore axis (normalised here)
    Real muzzleSpeed = 0;     // m/s, relative to the rifle
    Vec3 carrierVelocity;     // m/s, of the rifle (a moving vehicle); added
    Real time = 0;            // engine time of the shot (s)
};

struct PointMassOptions {
    Real step = kDefaultStep;       // s
    Vec3 gravity{0, 0, -kStandardGravity};
    Real maxTime = 12.0;            // s of flight, then the round stops
    Real minSpeed = 10.0;           // m/s, below which the round stops
    bool keepTrace = false;         // keep every step state (bullet-time replay)
};

struct FlightSample {
    Real time = 0;  // engine time
    Vec3 position, velocity;
};

class PointMassRound {
public:
    PointMassRound(const DragModel& drag, const AirSource& air, const Launch& launch,
                   const PointMassOptions& options = {});

    // Runs the whole steps due before engine time t. Returns steps run.
    std::int64_t advance(Real t, std::int64_t maxSteps = 1 << 20);
    // Steps until stopped (or the step budget).
    void runToEnd(std::int64_t maxSteps = 1 << 24);

    bool stopped() const { return stopped_; }
    Real time() const { return launchTime_ + clock_.time(); }
    Real flightTime() const { return clock_.time(); }
    const Vec3& position() const { return s_.p; }
    const Vec3& velocity() const { return s_.v; }
    // The state at engine time t between the launch and time(): exact at step
    // boundaries, cubic Hermite between them. Needs the trace for times before
    // the last step; without it, t is clamped into the last step.
    FlightSample stateAt(Real t) const;
    const std::vector<FlightSample>& trace() const { return trace_; }
    const DragModel& drag() const { return drag_; }

    // The derivative at a state (for tests and other integrators).
    struct State {
        Vec3 p, v;
        State operator+(const State& o) const { return {p + o.p, v + o.v}; }
        State operator*(Real k) const { return {p * k, v * k}; }
    };
    State derivative(Real t, const State& s) const;

private:
    void step(Real t0, Real h);
    DragModel drag_;
    const AirSource& air_;
    PointMassOptions o_;
    FixedClock clock_;
    Real launchTime_;
    State s_, prev_;
    Real prevTime_ = 0;
    bool stopped_ = false;
    std::vector<FlightSample> trace_;
};

// Cubic Hermite between two samples at time t (a.time <= t <= b.time).
FlightSample hermite(const FlightSample& a, const FlightSample& b, Real t);

}  // namespace bd
