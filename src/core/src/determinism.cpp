// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/determinism.hpp"

#include "bd/dmath.hpp"
#include "bd/point_mass.hpp"
#include "bd/rng.hpp"

namespace bd {

std::uint64_t determinismProbe() {
    Hash64 h;
    for (int i = 0; i < 4096; ++i) {
        const Real t = (i + 0.5) / 4096.0;
        h.add(dm::exp(-40.0 + 80.0 * t));
        h.add(dm::log(1e-6 + 1e6 * t * t));
        h.add(dm::sin(-50.0 + 100.0 * t));
        h.add(dm::cos(-50.0 + 100.0 * t));
        h.add(dm::atan2(dm::sin(13.0 * t), dm::cos(7.0 * t) - 0.3));
        h.add(dm::pow(0.01 + 30.0 * t, 1.0 / 3.0));
        h.add(dm::cbrt(-5.0 + 10.0 * t));
        h.add(dm::tanh(-3.0 + 6.0 * t));
        h.add(dm::asin(-0.999 + 1.998 * t));
    }
    Rng r(12345, 7);
    for (int i = 0; i < 1024; ++i) h.add(r.normal());
    return h.h;
}

std::uint64_t trajectoryProbe() {
    // A G7-like shape: flat subsonic, the transonic rise, the supersonic fall.
    const DragTable table({0.0, 0.8, 0.9, 0.95, 1.0, 1.05, 1.2, 1.5, 2.0, 3.0},
                          {0.120, 0.121, 0.130, 0.170, 0.380, 0.400, 0.385, 0.340, 0.290, 0.230});
    BallisticCoefficient bc;
    bc.family = "G7";
    bc.values = {171.0};  // kg/m^2 (0.243 lb/in^2)
    bc.minSpeeds = {0};
    const DragModel drag = DragModel::standard(table, bc);
    AirSample a;
    a.density = 1.1;
    a.speedOfSound = 343.0;
    a.wind = {3.0, -1.5, 0.2};
    const UniformAir air(a);
    Launch l;
    l.position = {12.5, -3.0, 1.6};
    l.direction = {0.6, 0.8, 0.012};
    l.muzzleSpeed = 820.0;
    l.time = 0.3;
    PointMassOptions o;
    o.keepTrace = true;
    PointMassRound r(drag, air, l, o);
    r.advance(2.8);
    Hash64 h;
    for (const FlightSample& s : r.trace()) {
        h.add(s.position);
        h.add(s.velocity);
    }
    const FlightSample mid = r.stateAt(1.23456);
    h.add(mid.position);
    h.add(mid.velocity);
    return h.h;
}

}  // namespace bd
