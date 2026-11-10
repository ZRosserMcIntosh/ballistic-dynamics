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
// measured values (docs/physics.md).
//
// What bends the light is not the slower air but the warmer air: the drag's
// work stays in the wake as heat, so per metre of path rho cp Int dT dA =
// D_wake, as rho U Int u dA = D_wake carries its momentum. With the same
// profile the centreline excess is dT/T = (u0/U) (gamma - 1) M^2, and the
// refractive index falls by (n - 1) dT/T, n - 1 in proportion to the air's
// density. `index` is that: dT/T times rho / 1.225, the stretch's index
// change in units of sea-level n - 1 (2.9e-4). A fast round's wake is far
// hotter for its momentum than a slow one's: a .308 at Mach 2.3 leaves 2.1
// times its deficit as dT/T, a 9 mm at Mach 1 0.4, a .45 at Mach 0.8 0.26.
//
// Above Mach 1 part of the drag is wave drag, which the shocks carry away as
// sound (the crack) rather than leave in the wake. The wake keeps the rest,
// estimated as the drag the same shape has at Mach 0.8 (friction and base
// drag): the drag table's C_D(0.8) / C_D(M), about 0.4 for a rifle bullet at
// Mach 2. Every stretch of wake is shed where the
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
    Real deficit = 0; // centreline velocity deficit u0/U (0..1)
    Real age = 0;     // s since it was shed
    Real index = 0;   // centreline refractive-index change, in sea-level n - 1: the trace's optical strength
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
    Real subsonicMach = 0.8;              // the wake keeps the drag the shape has here (estimated)
};

constexpr Real kSeaLevelDensity = 1.225;  // kg/m^3, ICAO

// The wake's share of the drag at Mach `mach`: 1 below `subsonicMach`, else
// C_D(subsonicMach) / C_D(mach) from the round's own drag table (the rest is
// wave drag, carried off by the shocks).
Real wakeDragShare(const DragModel& drag, Real mach, const WakeParams& p = {});

// dT/T on the wake's centreline for a velocity deficit at a Mach number (gamma 1.4).
inline Real wakeHeat(Real deficit, Real mach) { return deficit * 0.4 * mach * mach; }

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
