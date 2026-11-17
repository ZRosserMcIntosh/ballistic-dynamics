// Ballistic Dynamics — the air connection: Aviation Dynamics' atmosphere
// through bd::AirSource, station readings, wind along the range.
// SPDX-License-Identifier: MIT
#include "bd/air.hpp"
#include "bd/aviation_air.hpp"
#include "bd/units.hpp"
#include "bd_test.hpp"

using namespace bd;

TEST_CASE("air: a standard day at sea level is the ISA sea-level state") {
    const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    const AirSample s = air.sample({0, 0, 0}, 0);
    CHECK_NEAR(s.pressure, 101325.0, 1e-6);
    CHECK_NEAR(s.temperature, 288.15, 1e-9);
    CHECK_NEAR(s.density, 1.225, 5e-4);
    CHECK_NEAR(s.speedOfSound, 340.294, 0.01);
    CHECK_NEAR(s.densityAltitude, 0.0, 1.0);
    CHECK_NEAR(length(s.wind), 0.0, 1e-12);
}

TEST_CASE("air: moist air is lighter (virtual temperature) and carries sound faster") {
    const Real T = units::celsiusToKelvin(30);
    const AirSample dry = StationAir(StationReading{101325.0, T, 0.0, 0.0}).sample({}, 0);
    const AirSample wet = StationAir(StationReading{101325.0, T, 1.0, 0.0}).sample({}, 0);
    // Saturation vapour pressure at 30 C is about 4.25 kPa (Buck); the moist
    // density is lower by 0.378 e / p, about 1.6 %.
    const Real e = avd::saturationVapourPressure(T);
    CHECK_NEAR(e, 4246.0, 15.0);
    CHECK_NEAR(wet.density / dry.density, 1.0 - 0.378 * e / 101325.0, 2e-4);
    CHECK(wet.speedOfSound > dry.speedOfSound);
    CHECK_NEAR(dry.speedOfSound, std::sqrt(1.4 * 287.05287 * T), 1e-9);
    CHECK(wet.densityAltitude > dry.densityAltitude + 100);
    // Dry air is the ideal gas.
    CHECK_NEAR(dry.density, 101325.0 / (287.05287 * T), 1e-9);
}

TEST_CASE("air: a station reading is reproduced at the station, and the air changes with height") {
    // A hot afternoon at a high range (about Denver: 1609 m).
    const StationReading r{83400.0, units::celsiusToKelvin(30), 0.2, 1609.0};
    const StationAir air(r);
    const AirSample s = air.sample({0, 0, 0}, 0);
    CHECK_NEAR(s.pressure / r.pressure, 1.0, 1e-12);
    CHECK_NEAR(s.temperature, r.temperature, 1e-9);
    CHECK_NEAR(s.relativeHumidity, 0.2, 1e-12);
    // rho ~ 0.955 kg/m^3: the ISA has that density near 2,500 m.
    CHECK_RANGE(s.density, 0.945, 0.960);
    CHECK_RANGE(s.densityAltitude, 2450.0, 2650.0);
    // 100 m higher, hydrostatics: dp = -rho g dz.
    const AirSample up = air.sample({0, 0, 100}, 0);
    CHECK_NEAR(s.pressure - up.pressure, s.density * kStandardGravity * 100.0, 5.0);
    CHECK_NEAR(s.temperature - up.temperature, 0.65, 0.01);  // the standard lapse rate holds the profile
    // The ENU origin may be anywhere: the same point given another way.
    const StationAir shifted(r, 1500.0);
    CHECK_NEAR(shifted.sample({0, 0, 109.0}, 0).pressure, s.pressure, 1e-6);
}

TEST_CASE("air: Aviation's wind arrives in ENU (a north wind blows south)") {
    avd::SimpleWeatherParams w;
    w.windSpeed = 10;
    w.windFromDirection = 0;  // from the north
    w.turbulence.mechanical = false;
    const avd::SimpleWeather field(w);
    const AviationAir air(field, 0.0);
    const AirSample s = air.sample({0, 0, 10}, 0);  // the reference height
    CHECK_NEAR(s.wind.x, 0.0, 1e-9);
    CHECK_NEAR(s.wind.y, -10.0, 1e-6);
    CHECK_NEAR(s.wind.z, 0.0, 1e-9);
    // Weaker near the ground (log law), stronger aloft.
    CHECK(length(air.sample({0, 0, 2}, 0).wind) < 10.0);
    CHECK(length(air.sample({0, 0, 50}, 0).wind) > 10.0);
    // Wind direction helpers agree with Aviation's convention.
    Real speed, from;
    windToFrom(s.wind, speed, from);
    CHECK_NEAR(speed, 10.0, 1e-6);
    CHECK(from < 1e-9 || from > kTwoPi - 1e-9);
    const Vec3 westerly = windFrom(5, 1.5 * kPi);
    CHECK_NEAR(westerly.x, 5.0, 1e-12);
}

TEST_CASE("air: turbulence and gusts from Aviation's field reach the round") {
    avd::SimpleWeatherParams w;
    w.windSpeed = 8;
    w.windFromDirection = 0.5 * kPi;
    w.turbulence.mechanical = true;
    w.surfaceRoughness = 0.5;
    const avd::SimpleWeather field(w);
    const AviationAir air(field, 0.0);
    Real lo = 1e9, hi = -1e9;
    for (int i = 0; i < 400; ++i) {
        const Real x = air.sample({0, 0, 3}, 0.05 * i).wind.x;
        lo = std::min(lo, x);
        hi = std::max(hi, x);
    }
    CHECK(hi - lo > 1.0);  // the wind at the muzzle is not steady
}

TEST_CASE("air: wind laid along the range in layers, blended at the joins") {
    const UniformAir still;
    // Firing north. 0-400 m: 3 m/s from the west (full value from the left);
    // past 400 m: 5 m/s from the east.
    const std::vector<WindLayer> layers{{0, 3, 1.5 * kPi}, {400, 5, 0.5 * kPi}};
    const RangeWind truth(still, {0, 0, 0}, 0.0, layers);
    CHECK_NEAR(truth.sample({0, 100, 1}, 0).wind.x, 3.0, 1e-12);
    CHECK_NEAR(truth.sample({0, 700, 1}, 0).wind.x, -5.0, 1e-12);
    // Range is measured along the azimuth: a point off to the side is still at 100 m.
    CHECK_NEAR(truth.sample({30, 100, 1}, 0).wind.x, 3.0, 1e-12);
    // Continuous through the join, and halfway at it.
    const Real a = truth.sample({0, 399.999, 0}, 0).wind.x, b = truth.sample({0, 400.001, 0}, 0).wind.x;
    CHECK_NEAR(a, b, 2e-3);  // smoothstep: at most 1.5 x 8 m/s over 20 m of slope
    CHECK_NEAR(truth.windAt(400, 0).x, -1.0, 1e-12);
    // Fully the next layer once past the blend.
    CHECK_NEAR(truth.windAt(410.1, 0).x, -5.0, 1e-12);
    CHECK_NEAR(truth.windAt(389.9, 0).x, 3.0, 1e-12);

    // The shooter's call is another RangeWind over the same air.
    const RangeWind call(still, {0, 0, 0}, 0.0, {{0, 4, 1.5 * kPi}});
    CHECK_NEAR(call.sample({0, 700, 1}, 0).wind.x, 4.0, 1e-12);
    CHECK(call.sample({0, 700, 1}, 0).wind.x != truth.sample({0, 700, 1}, 0).wind.x);
}

TEST_CASE("air: range wind gusts are seeded, have their RMS, and add to a base wind") {
    const UniformAir still;
    const std::vector<WindLayer> layers{{0, 6, 1.5 * kPi, 1.5, 6.0, 0.1}};
    const RangeWind a(still, {}, 0.0, layers, RangeWind::Mode::Replace, 20, 9);
    const RangeWind b(still, {}, 0.0, layers, RangeWind::Mode::Replace, 20, 9);
    const RangeWind c(still, {}, 0.0, layers, RangeWind::Mode::Replace, 20, 10);
    Real sum = 0, sum2 = 0;
    bool same = true, differs = false;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const Real t = 0.1 * i;
        Real speed, from;
        windToFrom(a.windAt(100, t), speed, from);
        sum += speed;
        sum2 += speed * speed;
        same = same && bitsOf(a.windAt(100, t).x) == bitsOf(b.windAt(100, t).x);
        differs = differs || a.windAt(100, t).x != c.windAt(100, t).x;
    }
    CHECK(same);
    CHECK(differs);
    const Real mean = sum / n, rms = std::sqrt(sum2 / n - mean * mean);
    CHECK_NEAR(mean, 6.0, 0.15);
    CHECK_NEAR(rms, 1.5, 0.15);
    // Add mode: on top of the base air's own wind.
    AirSample base;
    base.wind = {0, 2, 0};
    const UniformAir withWind(base);
    const RangeWind added(withWind, {}, 0.0, {{0, 3, 1.5 * kPi}}, RangeWind::Mode::Add);
    const AirSample s = added.sample({0, 50, 0}, 0);
    CHECK_NEAR(s.wind.x, 3.0, 1e-12);
    CHECK_NEAR(s.wind.y, 2.0, 1e-12);
}
