// Ballistic Dynamics — air from Aviation Dynamics (ADR 0003).
// SPDX-License-Identifier: MIT
#include "bd/aviation_air.hpp"

#include "avd/world_frame.hpp"

namespace bd {

avd::AtmosphereConditions conditionsFromStation(const StationReading& r) {
    avd::AtmosphereConditions c;
    c.temperatureDeviation = r.temperature - avd::StandardAtmosphere::isaTemperature(r.altitudeMsl);
    c.relativeHumidity = r.relativeHumidity;
    // With the temperature profile fixed, every layer's pressure is
    // proportional to the sea-level pressure (hydrostatic integration is
    // linear in p0), so one evaluation scales it exactly; a second pass
    // removes the rounding of the first.
    for (int pass = 0; pass < 2; ++pass) {
        const Real p = avd::StandardAtmosphere(c).at(r.altitudeMsl).pressure;
        c.seaLevelPressure *= r.pressure / p;
    }
    return c;
}

AirSample AviationAir::fromAviation(const avd::AirSample& s) {
    AirSample a;
    a.pressure = s.state.pressure;
    a.temperature = s.state.temperature;
    a.density = s.state.density;
    a.speedOfSound = s.state.speedOfSound;
    a.relativeHumidity = s.state.relativeHumidity;
    a.densityAltitude = s.state.densityAltitude;
    const avd::Vec3 w = avd::world::enuFromNed(s.wind);
    a.wind = {w.x, w.y, w.z};
    return a;
}

AirSample AviationAir::sample(const Vec3& p, Real time) const {
    const avd::Vec3 ned = avd::world::nedFromEnu(avd::Vec3{p.x, p.y, p.z});
    return fromAviation(field_.sample(ned, originAltitude_ + p.z, time));
}

namespace {
avd::SimpleWeatherParams stillAir(const StationReading& r) {
    avd::SimpleWeatherParams w;
    w.conditions = conditionsFromStation(r);
    w.windSpeed = 0;
    w.groundElevation = r.altitudeMsl;
    w.turbulence.mechanical = false;
    return w;
}
}  // namespace

StationAir::StationAir(const StationReading& r, Real originAltitudeMsl)
    : weather_(stillAir(r)), air_(weather_, originAltitudeMsl) {}

}  // namespace bd
