// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// The air a round flies through, as the core sees it (ADR 0003). The core
// owns no atmosphere model: the host supplies an AirSource. Aviation
// Dynamics' standard atmosphere and weather field are wrapped as one by
// bd/aviation_air.hpp (target bd::aviation), which is what the tests, the
// tools and the browser build use.
//
// Positions and the wind are in the world frame, ENU (ADR 0001).
#pragma once

#include <cstdint>
#include <vector>

#include "bd/math.hpp"

namespace bd {

struct AirSample {
    Real pressure = 101325.0;        // Pa, static
    Real temperature = 288.15;       // K, dry-bulb
    Real density = 1.225;            // kg/m^3, moist air
    Real speedOfSound = 340.294;     // m/s, from the virtual temperature
    Real relativeHumidity = 0;       // 0..1
    Real densityAltitude = 0;        // m, ISA altitude with this density
    Vec3 wind;                       // m/s, ENU air velocity
};

// Implementations must be deterministic and safe for concurrent const calls
// (many rounds, several threads). `time` is the engine time in seconds.
class AirSource {
public:
    virtual ~AirSource() = default;
    virtual AirSample sample(const Vec3& positionEnu, Real time) const = 0;
};

// The same air everywhere: a fixed sample. For tests and for hosts that only
// know the conditions at the firing point.
class UniformAir final : public AirSource {
public:
    explicit UniformAir(const AirSample& s = {}) : s_(s) {}
    AirSample sample(const Vec3&, Real) const override { return s_; }
    const AirSample& state() const { return s_; }

private:
    AirSample s_;
};

// The horizontal wind (ENU, m/s) that blows FROM a bearing (rad, true,
// clockwise from north) at a speed.
Vec3 windFrom(Real speed, Real fromBearing);
// The speed and the bearing it blows from, of a horizontal ENU wind.
void windToFrom(const Vec3& wind, Real& speed, Real& fromBearing);

// One stretch of a range's wind: from `fromRange` metres downrange to the next
// layer's start. `gust` adds a smooth, seeded variation of that RMS (m/s) to
// the speed, with periods around `gustPeriod` seconds, and the direction
// wanders by `veer` (rad RMS) with it. A shooter reads wind flags in stretches
// like these; a mission feeds the shooter's call and the true wind as two
// RangeWinds over the same air.
struct WindLayer {
    Real fromRange = 0;     // m downrange from the origin
    Real speed = 0;         // m/s
    Real fromBearing = 0;   // rad, true
    Real gust = 0;          // m/s RMS
    Real gustPeriod = 8.0;  // s
    Real veer = 0;          // rad RMS
};

// Wind laid along a line of fire, over another source's air. Range is
// measured horizontally from `origin` along `azimuth` (rad, true). Between
// layers the wind blends over `blend` metres so a round does not see a step.
// mode Replace: the layers are the wind; Add: they add to the base wind.
class RangeWind final : public AirSource {
public:
    enum class Mode { Replace, Add };
    RangeWind(const AirSource& base, const Vec3& origin, Real azimuth, std::vector<WindLayer> layers,
              Mode mode = Mode::Replace, Real blend = 20.0, std::uint64_t seed = 1);
    AirSample sample(const Vec3& positionEnu, Real time) const override;
    // The layers' wind alone at a downrange distance and time.
    Vec3 windAt(Real range, Real time) const;
    Real rangeOf(const Vec3& positionEnu) const;
    const std::vector<WindLayer>& layers() const { return layers_; }

private:
    Vec3 layerWind(std::size_t i, Real time) const;
    const AirSource& base_;
    Vec3 origin_;
    Vec3 along_;
    std::vector<WindLayer> layers_;
    Mode mode_;
    Real blend_;
    struct Phases {
        Real p[6];
        Real f[3];
    };
    std::vector<Phases> phases_;
};

}  // namespace bd
