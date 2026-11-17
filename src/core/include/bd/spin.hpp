// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Spin and stability (Milestone 3, docs/physics.md section 4b).
//
//  - Gyroscopic stability Sg by Miller's rule (Miller 2005; the form with
//    the velocity, temperature and pressure corrections in Courtney &
//    Courtney, arXiv:1410.5340):
//        Sg = 30 m / (t^2 d^3 l (1 + l^2)) (V/2800)^(1/3) (T_F + 460)/519 (29.92/P)
//    m grains, t twist in calibres per turn, d inches, l length in calibres,
//    V fps, T_F deg F, P inHg.
//  - Downrange, Sg follows the physics instead: Sg is proportional to
//    p^2 / (rho V^2 Cma), with the spin p decaying slowly by roll damping
//    and Cma changing with Mach (estimated trend), so the round usually gets
//    more stable as it slows, and can lose it through the transonic zone.
//  - The yaw of repose (McCoy, Modern Exterior Ballistics, ch. 10): a spinning
//    round whose path curves (gravity) points its nose slightly to the side of
//    the path, alpha_R = -(2 Ix p / (rho S d V^4 Cma)) (V x dV/dt). With Cma
//    taken from Sg itself (rho S d Cma = Ix^2 p^2 / (2 Iy Sg V^2)) this is
//        alpha_R = -(4 Iy Sg / (Ix p)) (u x a) / u^2,
//    which needs only the inertia ratio Iy/Ix besides Sg. The lift on that yaw,
//    0.5 rho u^2 S CLa alpha_R, is the spin drift.
//  - Aerodynamic jump from a crosswind at the muzzle (Litz, Applied Ballistics
//    for Long Range Shooting): Y = 0.01 Sg - 0.0024 l + 0.032 MOA per mph of
//    crosswind, up for a wind from the left with a right-hand twist.
//  - The epicyclic wobble (fast nutation, slow precession) after the muzzle,
//    damping out: computed in closed form for display (its net effect on the
//    path is the jump above).
#pragma once

#include "bd/air.hpp"
#include "bd/library.hpp"
#include "bd/math.hpp"

namespace bd {

struct SpinParams {
    bool enabled = false;
    Real twist = 0;           // m per turn; positive right-hand, negative left-hand
    Real mass = 0;            // kg
    Real diameter = 0;        // m
    Real length = 0;          // m
    Real ixCoefficient = 0.105;  // Ix = k m d^2 (estimated: a solid of revolution with an ogive)
    Real iyOverIx = 7.5;      // estimated from the length (spinParamsFor)
    Real liftSlope = 2.5;     // CLa, per rad (estimated, McCoy's small-arms range 2-3)
    Real rollDamping = -0.012;  // Clp (estimated, McCoy's small-arms range -0.01 to -0.02)
    Real sgMuzzle = 0;        // Miller Sg at the muzzle conditions (set by spinParamsFor)
    Real muzzleMach = 0;      // for the Cma(M) ratio
    Real muzzleDensity = 0;   // kg/m^3
    Real muzzleSpeed = 0;     // m/s
    Real tipOffRate = 0.0;    // rad/s, initial yaw rate from the muzzle (0: a perfect barrel; M4 draws it)

    // Spin rate at the muzzle, rad/s (signed by the twist).
    Real muzzleSpin() const { return twist != 0 ? kTwoPi * muzzleSpeed / twist : 0; }
    Real ix() const { return ixCoefficient * mass * diameter * diameter; }
    Real calibres() const { return diameter > 0 ? length / diameter : 0; }
};

// Miller's Sg in SI inputs (kg, m per turn, m, m, m/s, K, Pa).
Real millerStability(Real mass, Real twist, Real diameter, Real length, Real speed, Real temperature, Real pressure);

// Cma(M) / Cma(M_ref): the overturning moment's change with Mach (estimated
// trend for a spitzer boat tail: rising toward Mach 1, falling subsonic).
Real overturningRatio(Real mach);

// Spin parameters for a projectile from a rifle at the muzzle conditions.
SpinParams spinParamsFor(const ProjectileDef& p, Real twist, Real muzzleSpeed, const AirSample& muzzleAir);

// Litz's aerodynamic jump: vertical angle (rad) for a crosswind component
// (m/s, positive blowing to the right of the line of fire), Sg and length in
// calibres; signed by the twist (positive right-hand).
Real aerodynamicJump(Real crosswindToRight, Real sg, Real calibres, Real twistSign);

// Litz's spin drift estimate (published empirical formula), metres:
// 1.25 (Sg + 1.2) TOF^1.83 inches.
Real litzSpinDrift(Real sg, Real timeOfFlight);

// The epicyclic yaw after the muzzle (display): the nose's angle off the
// air-relative path at time t since launch, as (right, up) in radians, for
// an initial yaw (right, up), initial yaw rate, muzzle spin and Sg.
struct Yaw2 {
    Real right = 0, up = 0;
};
Yaw2 epicyclicYaw(const SpinParams& s, Yaw2 initialYaw, Real t);

}  // namespace bd
