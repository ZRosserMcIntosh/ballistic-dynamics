// Ballistic Dynamics — WebAssembly bindings.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over the core so a web page can run it. The page only renders
// and reads input. Buffers returned by pointer are views into the module's
// memory, valid until the next call; web/README.md has the layouts.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/library.hpp"
#include "bd/solver.hpp"
#include "bd/units.hpp"

#define BD_EXPORT extern "C" __attribute__((visibility("default")))

using namespace bd;

namespace {

struct Sim {
    Library lib;
    std::vector<std::string> errors;
    std::unique_ptr<StationAir> air;
    std::unique_ptr<RangeWind> wind;  // over `air`; the round flies through this
    std::unique_ptr<PointMassRound> round;
    std::string text;
    std::vector<double> sample;
};

Sim* g = nullptr;
Sim& sim() {
    if (!g) g = new Sim;
    return *g;
}

const AirSource* flightAir() {
    Sim& S = sim();
    if (!S.air) S.air = std::make_unique<StationAir>(StationReading{101325.0, 288.15, 0.0, 0.0});
    if (S.wind) return S.wind.get();
    return S.air.get();
}

// The drag model of cartridge i (its preferred model: own curve, G7, G1).
bool cartridgeModel(int i, DragModel& drag, const CartridgeDef*& c, const ProjectileDef*& p) {
    const Library& L = sim().lib;
    if (i < 0 || i >= static_cast<int>(L.cartridges().size())) return false;
    c = &L.cartridges()[static_cast<std::size_t>(i)];
    p = L.projectile(c->projectile);
    if (!p) return false;
    std::string err;
    drag = dragModelFor(*p, L, "", err);
    return drag.kind() != DragModel::Kind::None;
}

void putText(const std::string& s) {
    Sim& S = sim();
    S.text = s;
    S.text.push_back('\0');
}

void jsonString(std::string& o, const std::string& s) {
    o += '"';
    for (const char c : s) {
        if (c == '"' || c == '\\') {
            o += '\\';
            o += c;
        } else if (c == '\n') {
            o += "\\n";
        } else if (static_cast<unsigned char>(c) < 0x20) {
            o += ' ';
        } else {
            o += c;
        }
    }
    o += '"';
}

void jsonNum(std::string& o, double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.10g", std::isfinite(v) ? v : 0.0);
    o += b;
}

const char* kindName(ProvenanceKind k) {
    switch (k) {
        case ProvenanceKind::Published: return "published";
        case ProvenanceKind::Measured: return "measured";
        default: return "estimated";
    }
}

void provenance(std::string& o, const ProvenanceMap& m) {
    o += "\"provenance\":{";
    bool first = true;
    for (const auto& [k, p] : m) {
        if (!first) o += ',';
        first = false;
        jsonString(o, k);
        o += ":{\"kind\":";
        jsonString(o, kindName(p.kind));
        o += ",\"note\":";
        jsonString(o, p.note);
        o += '}';
    }
    o += '}';
}

}  // namespace

// Scratch text buffer of at least n bytes (the page writes JSON into it).
BD_EXPORT char* bd_text(int n) {
    Sim& S = sim();
    if (n > 0) S.text.assign(static_cast<std::size_t>(n), '\0');
    return S.text.data();
}
BD_EXPORT int bd_text_length() { return static_cast<int>(std::string(sim().text.c_str()).size()); }

// Loads the n bytes of JSON in the text buffer; returns the error count so far.
BD_EXPORT int bd_load(int n) {
    Sim& S = sim();
    const std::string doc(S.text.data(), static_cast<std::size_t>(n));
    for (auto& e : S.lib.load(doc)) S.errors.push_back(std::move(e));
    return static_cast<int>(S.errors.size());
}
BD_EXPORT int bd_resolve() {
    Sim& S = sim();
    for (auto& e : S.lib.resolve()) S.errors.push_back(std::move(e));
    return static_cast<int>(S.errors.size());
}
BD_EXPORT const char* bd_error_text(int i) {
    Sim& S = sim();
    putText(i >= 0 && i < static_cast<int>(S.errors.size()) ? S.errors[static_cast<std::size_t>(i)] : "");
    return S.text.c_str();
}

// The library as JSON (SI values, with provenance).
BD_EXPORT const char* bd_catalog() {
    const Library& L = sim().lib;
    std::string o = "{\"projectiles\":[";
    for (std::size_t i = 0; i < L.projectiles().size(); ++i) {
        const auto& p = L.projectiles()[i];
        if (i) o += ',';
        o += "{\"id\":";
        jsonString(o, p.id);
        o += ",\"name\":";
        jsonString(o, p.name);
        o += ",\"mass\":";
        jsonNum(o, p.mass);
        o += ",\"diameter\":";
        jsonNum(o, p.diameter);
        o += ",\"length\":";
        jsonNum(o, p.length);
        o += ',';
        provenance(o, p.provenance);
        o += '}';
    }
    o += "],\"cartridges\":[";
    for (std::size_t i = 0; i < L.cartridges().size(); ++i) {
        const auto& c = L.cartridges()[i];
        if (i) o += ',';
        o += "{\"id\":";
        jsonString(o, c.id);
        o += ",\"name\":";
        jsonString(o, c.name);
        o += ",\"projectile\":";
        jsonString(o, c.projectile);
        o += ",\"muzzleVelocity\":";
        jsonNum(o, c.muzzleVelocity);
        o += ',';
        provenance(o, c.provenance);
        o += '}';
    }
    o += "],\"rifles\":[";
    for (std::size_t i = 0; i < L.rifles().size(); ++i) {
        const auto& r = L.rifles()[i];
        if (i) o += ',';
        o += "{\"id\":";
        jsonString(o, r.id);
        o += ",\"name\":";
        jsonString(o, r.name);
        o += ",\"twist\":";
        jsonNum(o, r.twist);
        o += ",\"rightHand\":";
        o += r.rightHandTwist ? "true" : "false";
        o += ',';
        provenance(o, r.provenance);
        o += '}';
    }
    o += "]}";
    putText(o);
    return sim().text.c_str();
}

// The air from a station reading (Pa, K, 0..1, m MSL); the ENU origin is the station.
// Clears the wind and any round in flight (they refer to the old air).
BD_EXPORT void bd_station(double pressure, double temperature, double relativeHumidity, double altitudeMsl) {
    Sim& S = sim();
    S.round.reset();
    S.wind.reset();
    S.air = std::make_unique<StationAir>(StationReading{pressure, temperature, relativeHumidity, altitudeMsl});
}

// Wind along a line of fire from the origin at `azimuth` (rad): one layer per
// (fromRange m, speed m/s, fromBearing rad, gust RMS m/s) quadruple in the
// text buffer as doubles; n layers. n = 0 removes the wind.
BD_EXPORT void bd_wind(double azimuth, int n) {
    Sim& S = sim();
    S.round.reset();
    flightAir();
    if (n <= 0) {
        S.wind.reset();
        return;
    }
    std::vector<double> v(static_cast<std::size_t>(4 * n));
    std::memcpy(v.data(), S.text.data(), v.size() * sizeof(double));
    std::vector<WindLayer> layers;
    for (int i = 0; i < n; ++i) {
        WindLayer L;
        L.fromRange = v[4 * i];
        L.speed = v[4 * i + 1];
        L.fromBearing = v[4 * i + 2];
        L.gust = v[4 * i + 3];
        layers.push_back(L);
    }
    S.wind = std::make_unique<RangeWind>(*S.air, Vec3{}, azimuth, layers);
}

// The bore elevation (rad) that zeroes cartridge i at zeroRange (m) with the
// optic sightHeight (m) above the bore, in the station's still air. NaN if not.
BD_EXPORT double bd_zero(int cartridge, double zeroRange, double sightHeight) {
    DragModel drag;
    const CartridgeDef* c;
    const ProjectileDef* p;
    if (!cartridgeModel(cartridge, drag, c, p)) return std::nan("");
    flightAir();
    RangeSetup s;
    s.drag = drag;
    s.muzzleSpeed = c->muzzleVelocity;
    s.mass = p->mass;
    s.sightHeight = sightHeight;
    s.zeroRange = zeroRange;
    return zeroElevation(s, *sim().air);
}

// Fires cartridge i from (x, y, z) ENU along azimuth / elevation (rad) at
// engine time t0, keeping its trace for bullet-time replay. 1 on success.
BD_EXPORT int bd_fire(int cartridge, double x, double y, double z, double azimuth, double elevation, double t0) {
    DragModel drag;
    const CartridgeDef* c;
    const ProjectileDef* p;
    if (!cartridgeModel(cartridge, drag, c, p)) return 0;
    Launch l;
    l.position = {x, y, z};
    l.direction = frames::directionFromAzEl(azimuth, elevation);
    l.muzzleSpeed = c->muzzleVelocity;
    l.time = t0;
    PointMassOptions o;
    o.keepTrace = true;
    sim().round = std::make_unique<PointMassRound>(drag, *flightAir(), l, o);
    return 1;
}

// Advances the round to engine time t (whole 1 ms steps; at most maxSteps).
// For bullet time, advance by less than real time: the steps are the same,
// only the clock the page shows them on is slower.
BD_EXPORT int bd_advance(double t, int maxSteps) {
    Sim& S = sim();
    return S.round ? static_cast<int>(S.round->advance(t, maxSteps > 0 ? maxSteps : (1 << 20))) : 0;
}

// 8 doubles: engine time, position x y z, velocity x y z, stopped.
BD_EXPORT const double* bd_round() {
    Sim& S = sim();
    S.sample.assign(8, 0.0);
    if (S.round) {
        const auto& r = *S.round;
        S.sample = {r.time(), r.position().x, r.position().y, r.position().z,
                    r.velocity().x, r.velocity().y, r.velocity().z, r.stopped() ? 1.0 : 0.0};
    }
    return S.sample.data();
}

// Bullet time: 7 doubles, the round's state at any engine time already flown
// (time, position, velocity), smooth between steps. Advance one step (1 ms)
// ahead of the time drawn: bd_advance(t + 0.001), then bd_state_at(t).
BD_EXPORT const double* bd_state_at(double t) {
    Sim& S = sim();
    S.sample.assign(7, 0.0);
    if (S.round) {
        const FlightSample f = S.round->stateAt(t);
        S.sample = {f.time, f.position.x, f.position.y, f.position.z, f.velocity.x, f.velocity.y, f.velocity.z};
    }
    return S.sample.data();
}

// A range card for cartridge i (preferred drag model) in the station air and
// wind, firing along azimuth 0 from the origin: JSON rows of range, drop,
// windage (m), time (s), speed (m/s), energy (J), Mach.
BD_EXPORT const char* bd_table(int cartridge, double zeroRange, double sightHeight, double maxRange, double step) {
    std::string o = "[";
    DragModel drag;
    const CartridgeDef* c;
    const ProjectileDef* p;
    if (cartridgeModel(cartridge, drag, c, p) && step > 0) {
        flightAir();
        RangeSetup s;
        s.drag = drag;
        s.muzzleSpeed = c->muzzleVelocity;
        s.mass = p->mass;
        s.sightHeight = sightHeight;
        s.zeroRange = zeroRange;
        const Real e = zeroElevation(s, *sim().air);
        std::vector<Real> ranges;
        for (Real r = 0; r <= maxRange + 1e-9; r += step) ranges.push_back(r);
        bool first = true;
        for (const RangeRow& r : rangeTable(s, *flightAir(), e, ranges)) {
            if (!r.reached) continue;
            if (!first) o += ',';
            first = false;
            o += '[';
            for (double v : {r.range, r.drop, r.windage, r.time, r.speed, r.energy, r.mach}) {
                jsonNum(o, v);
                o += ',';
            }
            o.back() = ']';
        }
    }
    o += ']';
    putText(o);
    return sim().text.c_str();
}

// 9 doubles: pressure, temperature, density, speed of sound, relative humidity,
// density altitude, wind east, north, up. Zeros before bd_station.
BD_EXPORT const double* bd_air(double x, double y, double z, double t) {
    Sim& S = sim();
    S.sample.assign(9, 0.0);
    if (S.air) {
        const AirSample a = S.air->sample({x, y, z}, t);
        S.sample = {a.pressure, a.temperature, a.density, a.speedOfSound, a.relativeHumidity, a.densityAltitude,
                    a.wind.x, a.wind.y, a.wind.z};
    }
    return S.sample.data();
}

// The determinism hashes (same as `bdemo determinism` and the native tests):
// the maths, then a whole trajectory, as 16 hex digits each.
BD_EXPORT const char* bd_determinism() {
    const std::uint64_t h = determinismProbe(), t = trajectoryProbe();
    char b[40];
    std::snprintf(b, sizeof b, "%016llx %016llx", static_cast<unsigned long long>(h), static_cast<unsigned long long>(t));
    putText(b);
    return sim().text.c_str();
}
