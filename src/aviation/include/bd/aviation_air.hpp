// Ballistic Dynamics — air from Aviation Dynamics (ADR 0003).
// SPDX-License-Identifier: MIT
//
// Wraps Aviation Dynamics' U.S. Standard Atmosphere 1976 and weather field
// (extern/aviation-dynamics, pinned by the submodule) as a bd::AirSource.
// Nothing here re-implements the atmosphere: thermodynamics, humidity,
// density altitude, the boundary-layer wind, gusts and turbulence are all
// Aviation's. This file only converts frames (ENU <-> NED) and turns a
// shooter's station readings into Aviation's AtmosphereConditions.
#pragma once

#include "avd/atmosphere.hpp"
#include "bd/air.hpp"

namespace bd {

// What a shooter's kestrel reads at the firing point: station (not sea-level)
// pressure, dry-bulb temperature, relative humidity, and the elevation.
struct StationReading {
    Real pressure = 101325.0;    // Pa, station (absolute) pressure
    Real temperature = 288.15;   // K
    Real relativeHumidity = 0;   // 0..1
    Real altitudeMsl = 0;        // m, of the station
};

// The deviations from the standard day that give exactly this reading at the
// station's altitude in Aviation's model: the temperature offset from ISA at
// that altitude, and the sea-level pressure whose hydrostatic profile (with
// that temperature) passes through the station pressure.
avd::AtmosphereConditions conditionsFromStation(const StationReading& r);

// Air from any Aviation field. The ENU world origin sits at `originAltitudeMsl`
// (Aviation's local frame is anchored the same way: NED at the same origin).
class AviationAir final : public AirSource {
public:
    AviationAir(const avd::AtmosphereField& field, Real originAltitudeMsl)
        : field_(field), originAltitude_(originAltitudeMsl) {}
    AirSample sample(const Vec3& positionEnu, Real time) const override;
    static AirSample fromAviation(const avd::AirSample& s);

private:
    const avd::AtmosphereField& field_;
    Real originAltitude_;
};

// A ready-made still-air source from a station reading (Aviation's
// SimpleWeather with no wind, so the thermodynamics still change with height
// along the path). Add wind on top with RangeWind, or pass a full
// SimpleWeatherParams to AviationAir for the boundary layer and turbulence.
class StationAir final : public AirSource {
public:
    explicit StationAir(const StationReading& r, Real originAltitudeMsl);
    explicit StationAir(const StationReading& r) : StationAir(r, r.altitudeMsl) {}
    StationAir(const StationAir&) = delete;  // air_ refers to weather_
    StationAir& operator=(const StationAir&) = delete;
    AirSample sample(const Vec3& positionEnu, Real time) const override { return air_.sample(positionEnu, time); }
    const avd::SimpleWeather& weather() const { return weather_; }

private:
    avd::SimpleWeather weather_;
    AviationAir air_;
};

}  // namespace bd
