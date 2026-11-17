// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Every round unique (Milestone 4, docs/physics.md section 5).
//
// No two rounds of a load leave the same: the powder burns a little
// differently (the muzzle velocity scatters, and follows the powder's
// temperature), no two bullets have quite the same shape (the BC
// scatters), and the barrel throws each one a little off the bore line
// (the rifle's own group). A round's draw comes from its own seeded stream
// (bd/rng.hpp), so the same (seed, stream) gives the same round, bit for
// bit, on every machine: a shared link or a duel replays it exactly.
//
//   v0    = v_published + k (T_powder - T_reference) + sigma_v z1
//   BC    = BC_published (1 + sigma_BC z2)
//   bore += (sigma_a z3 up, sigma_a z4 right)
//
// with z1..z4 independent standard normals and sigma_a the angular standard
// deviation per axis from the rifle's mean 5-shot group: for a circular
// normal, the expected extreme spread of 5 shots is 3.08 sigma (Grubbs'
// tables), so sigma_a = group / 3.08.
#pragma once

#include <cstdint>

#include "bd/drag.hpp"
#include "bd/library.hpp"
#include "bd/solver.hpp"

namespace bd {

constexpr Real kFiveShotSpreadPerSigma = 3.08;

struct VariationModel {
    Real velocitySd = 0;       // m/s
    Real bcSd = 0;             // fraction
    Real angleSd = 0;          // rad, per axis
    Real tempCoefficient = 0;  // m/s per K
    Real tempReference = 0;    // K
};

// From the library (a missing figure is no variation of that kind).
VariationModel variationFor(const CartridgeDef& c, const ProjectileDef& p, const RifleDef* rifle);

// The published velocity moved to a powder temperature (K); <= 0: as published.
Real muzzleSpeedAt(const CartridgeDef& c, const VariationModel& m, Real powderTemperature);

struct RoundDraw {
    Real velocityOffset = 0;  // m/s
    Real bcFactor = 1;
    Real up = 0, right = 0;   // rad, the barrel's throw off the bore line
};

// One round's draw from stream (seed, stream).
RoundDraw drawRound(const VariationModel& m, std::uint64_t seed, std::uint64_t stream);

// The bore direction turned by a draw's throw (up about the right axis, then right).
Vec3 thrownDirection(const Vec3& bore, const RoundDraw& d);

// The spread of impacts at a range (standard deviations, m), by cause, from
// the linearised flight: d(height)/d(v0) and d(height)/d(BC) by flying the
// round again with each nudged, the barrel's throw times the range. The
// shooter's call of the wind is not in it.
struct Spread {
    Real fromVelocity = 0, fromBc = 0, fromBarrel = 0;  // m, vertical (barrel: each axis)
    Real vertical = 0, horizontal = 0;                  // m, combined
};
Spread spreadAt(const RangeSetup& s, const AirSource& air, Real elevation, Real range, const VariationModel& m);

}  // namespace bd
