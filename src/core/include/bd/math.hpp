// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Minimal linear algebra. Double precision throughout. Vectors in the world
// frame are ENU (x east, y north, z up, metres), as in Aviation Dynamics'
// world_frame.hpp; see docs/decisions/0001-frames-and-units.md.
//
// Only +, -, *, / and sqrt appear here: IEEE 754 rounds each of them exactly,
// so these are bit-identical on every platform (ADR 0004). Transcendental
// functions live in bd/dmath.hpp.
#pragma once

#include <cmath>
#include <cstdint>

namespace bd {

using Real = double;

inline constexpr Real kPi = 3.14159265358979323846;
inline constexpr Real kTwoPi = 2.0 * kPi;
inline constexpr Real kStandardGravity = 9.80665;  // m/s^2 (CGPM 1901)

struct Vec3 {
    Real x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
    constexpr bool operator==(const Vec3&) const = default;
};

constexpr Vec3 operator*(Real s, const Vec3& v) { return v * s; }
constexpr Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) {
    const Real l = length(v);
    return l > 1e-300 ? v / l : Vec3{};
}
inline bool isFinite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

constexpr Real clamp(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }
constexpr Real sq(Real v) { return v * v; }

// Bit pattern of a double, for determinism hashes.
std::uint64_t bitsOf(Real v);
// FNV-1a over the bit patterns of a sequence of doubles.
struct Hash64 {
    std::uint64_t h = 1469598103934665603ull;
    void add(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (v >> (8 * i)) & 0xffu;
            h *= 1099511628211ull;
        }
    }
    void add(Real v) { add(bitsOf(v)); }
    void add(const Vec3& v) { add(v.x); add(v.y); add(v.z); }
};

}  // namespace bd
