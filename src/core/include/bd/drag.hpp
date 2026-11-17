// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Drag (docs/physics.md, section 4).
//
// A drag table is C_D against Mach number, interpolated linearly between its
// points (the standard tables are tabulated every 0.025-0.05 Mach where the
// curve bends, and linear interpolation is what JBM and the published
// calculators use). Below the first point the first value holds; above the
// last, the last.
//
// Two ways to use one:
//
//  - A standard table (G1, G7) scaled by the bullet's ballistic coefficient
//    BC = m / (i d^2) (kg/m^2 in SI): the bullet's C_D is i times the
//    reference's, so the deceleration is
//        a = rho v^2 C_D,ref(M) pi / (8 BC).
//    A stepped BC (Sierra publishes them) picks the value for the current
//    air-relative speed.
//  - A bullet's own curve: a = rho v^2 C_D(M) A / (2 m), A = pi d^2 / 4.
//
// v is the speed relative to the air and M = v / a_sound of the local air.
#pragma once

#include <string>
#include <vector>

#include "bd/air.hpp"
#include "bd/library.hpp"
#include "bd/math.hpp"

namespace bd {

class DragTable {
public:
    DragTable() = default;
    DragTable(std::vector<Real> mach, std::vector<Real> cd);
    explicit DragTable(const DragCurveDef& d) : DragTable(d.mach, d.cd) {}
    Real cd(Real mach) const;
    bool empty() const { return mach_.size() < 2; }
    const std::vector<Real>& mach() const { return mach_; }
    const std::vector<Real>& cds() const { return cd_; }

private:
    std::vector<Real> mach_, cd_;
};

class DragModel {
public:
    enum class Kind { None, StandardBc, BulletCurve };

    DragModel() = default;
    // A standard reference table scaled by a (possibly stepped) BC.
    static DragModel standard(const DragTable& reference, const BallisticCoefficient& bc);
    // A bullet's own C_D(M), with its mass and diameter.
    static DragModel bullet(const DragTable& curve, Real mass, Real diameter);

    Kind kind() const { return kind_; }
    // Drag deceleration (m/s^2) for an air-relative velocity in this air.
    Vec3 acceleration(const Vec3& airVelocity, const AirSample& air) const;
    // Scalar: |a| / (rho v^2), the "drag factor" at a speed and speed of sound.
    Real factor(Real speed, Real speedOfSound) const;
    // The BC in force at an air-relative speed (kg/m^2; 0 for a bullet curve).
    Real bcAt(Real speed) const;
    const DragTable& table() const { return table_; }

private:
    Kind kind_ = Kind::None;
    DragTable table_;
    BallisticCoefficient bc_;
    Real curveFactor_ = 0;  // A / (2 m)
};

// The drag model for a projectile from the library, by preference: its own
// curve, then G7, then G1. `standardTables` holds the G1 and G7 curves (ids
// "g1", "g7"). `prefer` may force "G1", "G7" or "bullet". Empty model
// (kind None) and an error when nothing fits.
DragModel dragModelFor(const ProjectileDef& p, const Library& lib, const std::string& prefer, std::string& error);

}  // namespace bd
