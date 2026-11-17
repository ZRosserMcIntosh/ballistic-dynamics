// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Frames (ADR 0001).
//
//  - World: ENU, x east, y north, z up, metres; the same as Aviation
//    Dynamics' world_frame.hpp and the São Paulo World tiles.
//  - Line of fire: x downrange (horizontal, along the azimuth), y left,
//    z up. It is right-handed and has OpenRoad Dynamics' ISO 8855 axis
//    order (x forward, y left, z up), so a rifle in a vehicle maps without
//    sign changes. Drop is -z, drift to the right is -y.
//  - Bearings are true, clockwise from north (azimuth 0 = north, pi/2 =
//    east); elevation is up from the horizontal.
//
// The trajectory is integrated in the world frame on a flat local tangent
// plane; Earth rotation enters as the Coriolis and centrifugal terms
// (Milestone 3), with the rotation vector below.
#pragma once

#include "bd/dmath.hpp"
#include "bd/math.hpp"

namespace bd::frames {

// Earth's sidereal rotation rate, rad/s (IERS conventions).
inline constexpr Real kEarthRotationRate = 7.2921150e-5;

// Unit ENU vector along an azimuth (true, clockwise from north) and elevation.
inline Vec3 directionFromAzEl(Real azimuth, Real elevation) {
    Real sa, ca, se, ce;
    dm::sincos(azimuth, sa, ca);
    dm::sincos(elevation, se, ce);
    return {ce * sa, ce * ca, se};
}

// Azimuth in [0, 2pi) and elevation of an ENU direction (any length > 0).
inline void azElFromDirection(const Vec3& d, Real& azimuth, Real& elevation) {
    Real a = dm::atan2(d.x, d.y);
    if (a < 0) a += kTwoPi;
    azimuth = a;
    elevation = dm::atan2(d.z, std::sqrt(d.x * d.x + d.y * d.y));
}

// The line-of-fire frame for a shot along `azimuth` from `origin`.
struct LineOfFire {
    Vec3 origin;
    Vec3 x, y, z;  // its axes in ENU

    LineOfFire() : x{0, 1, 0}, y{-1, 0, 0}, z{0, 0, 1} {}
    LineOfFire(const Vec3& o, Real azimuth) : origin(o) {
        Real s, c;
        dm::sincos(azimuth, s, c);
        x = {s, c, 0};
        z = {0, 0, 1};
        y = cross(z, x);  // left
    }
    // ENU point -> (downrange, left, up)
    Vec3 fromWorld(const Vec3& p) const {
        const Vec3 d = p - origin;
        return {dot(d, x), dot(d, y), dot(d, z)};
    }
    Vec3 toWorld(const Vec3& l) const { return origin + x * l.x + y * l.y + z * l.z; }
    // Directions (velocities, winds): no origin shift.
    Vec3 vectorFromWorld(const Vec3& v) const { return {dot(v, x), dot(v, y), dot(v, z)}; }
    Vec3 vectorToWorld(const Vec3& l) const { return x * l.x + y * l.y + z * l.z; }
};

// Earth's rotation vector in the local ENU frame at a geodetic latitude (rad).
inline Vec3 earthRotationEnu(Real latitude) {
    Real s, c;
    dm::sincos(latitude, s, c);
    return {0, kEarthRotationRate * c, kEarthRotationRate * s};
}

}  // namespace bd::frames
