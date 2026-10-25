// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Zeroing and range tables (the published "drop and drift" format). These fly
// real rounds through the point-mass model; nothing here is a separate
// approximation.
//
// Geometry: the muzzle at `origin`, firing along `azimuth`; the optic `sight
// Height` above the bore; the line of sight horizontal (a flat range). Drop
// is the round's height relative to the line of sight (negative below it);
// windage is to the right of it (positive right). Ranges are horizontal
// distances along the line of fire.
#pragma once

#include <vector>

#include "bd/air.hpp"
#include "bd/drag.hpp"
#include "bd/frames.hpp"
#include "bd/point_mass.hpp"

namespace bd {

struct RangeSetup {
    DragModel drag;
    Real muzzleSpeed = 0;   // m/s
    Real mass = 0;          // kg (for energy)
    Real sightHeight = 0;   // m
    Real zeroRange = 100;   // m
    Vec3 origin;            // ENU
    Real azimuth = 0;       // rad, true
    Real step = kDefaultStep;
    Real time = 0;          // s, engine time the table's round is fired at (gusts are seeded by time)
    // Milestone 3: spin (spinParamsFor) and the rotating Earth, applied to the
    // zero and to the table alike (step, trace and time limits are set here).
    PointMassOptions physics;
};

struct RangeRow {
    Real range = 0;     // m
    Real spin = 0;      // rad/s
    Real stability = 0; // Sg (0 without spin)
    Real drop = 0;      // m, relative to the line of sight (negative below)
    Real windage = 0;   // m, positive right
    Real time = 0;      // s
    Real speed = 0;     // m/s (ground)
    Real energy = 0;    // J
    Real mach = 0;      // air-relative
    bool reached = false;
};

// The bore elevation (rad) that puts the round on the line of sight at the
// zero range, in `air` (the zero is usually taken in still air). Returns NaN
// when the round cannot reach the zero range.
Real zeroElevation(const RangeSetup& s, const AirSource& air);

// Flies one round at `elevation` through `air` and reports it at each range.
std::vector<RangeRow> rangeTable(const RangeSetup& s, const AirSource& air, Real elevation,
                                 const std::vector<Real>& ranges);

// Where a flown trace crosses a downrange distance in a line-of-fire frame:
// the interpolated sample, false if it never gets there.
bool crossing(const std::vector<FlightSample>& trace, const frames::LineOfFire& lof, Real range, FlightSample& out);

}  // namespace bd
