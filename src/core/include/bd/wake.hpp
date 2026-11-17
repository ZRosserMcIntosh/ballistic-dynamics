// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// The wake a round leaves in the air: what a spotter sees as the "trace".
//
// Behind a body the air it has pushed aside forms a turbulent wake: a thin
// tube of slower, stirred, slightly warmer air along the path, whose refractive
// index differs from the air around it. Seen from behind the rifle, looking
// along the path, that tube bends the light of the mirage behind it, and the
// shooter sees the trace swirl down the range.
//
// The model is the self-similar far wake of an axisymmetric body (Pope,
// Turbulent Flows, ch. 5.5; Uberoi & Freymuth 1970): with the drag written as
// a momentum thickness, D = rho U^2 pi theta^2, i.e. theta = sqrt(Cd A / 2 pi),
// a stretch of wake shed a distance x behind the body has
//
//     half-width   r(x) = a theta (x / theta)^(1/3)
//     deficit      u0/U = b (x / theta)^(-2/3)
//
// (the wake grows as the cube root of the distance and fades as its two-
// thirds power). The coefficients a and b are estimated at the order of the
// measured values (docs/physics.md). Every stretch of wake is shed where the
// round was, with that moment's speed and drag, ages as the round flies on
// (x = U age: the temporal analogy of the far wake), and drifts with the wind
// where it is. Near the body the width is held at the bullet's radius.
#pragma once

#include <vector>

#include "bd/math.hpp"
#include "bd/point_mass.hpp"

namespace bd {

struct WakeElement {
    Vec3 position;    // ENU, m: where the stretch of wake is now
    Real radius = 0;  // m, half-width
    Real deficit = 0; // centreline velocity deficit u0/U (0..1): the trace's strength
    Real age = 0;     // s since it was shed
};

struct WakeParams {
    Real spacing = 0.5;       // m between stretches along the path
    Real maxAge = 0.6;        // s: older stretches are left out
    Real minDeficit = 2e-5;   // weaker stretches are left out
    // a is estimated; b then follows from momentum: for a Gaussian deficit
    // profile with half-deficit radius r, the wake's momentum deficit
    // U u0 pi r^2 / ln 2 must carry the drag, U^2 pi theta^2, so a^2 b = ln 2.
    Real widthCoefficient = 0.6;          // a (estimated)
    Real deficitCoefficient = 1.925409;   // b = ln 2 / a^2
};

// The momentum thickness of a body with drag area Cd A (m^2): sqrt(Cd A / 2 pi).
Real wakeMomentumThickness(Real dragArea);

// The far-wake half-width and deficit at a distance x behind a body of
// momentum thickness theta and radius `radius`.
void farWake(Real theta, Real radius, Real x, const WakeParams& p, Real& halfWidth, Real& deficit);

// The wake of a flown round (it must keep its trace) at engine time t; `mass`
// in kg gives the drag area from the round's drag model.
std::vector<WakeElement> wakeAt(const PointMassRound& round, Real mass, Real diameter, Real t,
                                const WakeParams& p = {});

}  // namespace bd
