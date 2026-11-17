// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Unit conversions. The core is SI inside; conversions happen only at the
// boundaries: JSON fields carry their unit in the name (`_gr`, `_in`, `_fps`,
// `_inHg`), and the solver API can report in the shooter's units. Exact
// definitions where they exist (inch, foot, yard, pound, grain are exact by the
// 1959 international yard and pound agreement).
#pragma once

#include "bd/math.hpp"

namespace bd::units {

inline constexpr Real kInch = 0.0254;                  // m
inline constexpr Real kFoot = 0.3048;                  // m
inline constexpr Real kYard = 0.9144;                  // m
inline constexpr Real kPound = 0.45359237;             // kg
inline constexpr Real kGrain = kPound / 7000.0;        // kg (64.79891 mg)
inline constexpr Real kInchOfMercury = 3386.389;       // Pa (at 0 C, conventional)
inline constexpr Real kMillibar = 100.0;               // Pa
inline constexpr Real kMoa = kPi / (180.0 * 60.0);     // rad, one minute of angle
inline constexpr Real kMrad = 1e-3;                    // rad
inline constexpr Real kKelvinOffset = 273.15;

constexpr Real grains(Real gr) { return gr * kGrain; }
constexpr Real inches(Real in) { return in * kInch; }
constexpr Real feet(Real ft) { return ft * kFoot; }
constexpr Real yards(Real yd) { return yd * kYard; }
constexpr Real fps(Real v) { return v * kFoot; }
constexpr Real toFps(Real ms) { return ms / kFoot; }
constexpr Real toInches(Real m) { return m / kInch; }
constexpr Real toYards(Real m) { return m / kYard; }
constexpr Real toGrains(Real kg) { return kg / kGrain; }
constexpr Real fahrenheitToKelvin(Real f) { return (f - 32.0) * (5.0 / 9.0) + kKelvinOffset; }
constexpr Real celsiusToKelvin(Real c) { return c + kKelvinOffset; }
constexpr Real kelvinToFahrenheit(Real k) { return (k - kKelvinOffset) * 1.8 + 32.0; }
constexpr Real inHg(Real p) { return p * kInchOfMercury; }
constexpr Real moa(Real n) { return n * kMoa; }
constexpr Real mrad(Real n) { return n * kMrad; }

// A ballistic coefficient as published in the U.S. (lb/in^2) to SI (kg/m^2):
// BC = m / (i d^2). 1 lb/in^2 = 703.0696 kg/m^2.
inline constexpr Real kBcLbPerSqInToSi = kPound / (kInch * kInch);
constexpr Real bcToSi(Real bcLbPerSqIn) { return bcLbPerSqIn * kBcLbPerSqInToSi; }
constexpr Real bcFromSi(Real bcSi) { return bcSi / kBcLbPerSqInToSi; }

}  // namespace bd::units
