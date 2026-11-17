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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/library.hpp"
#include "bd/solver.hpp"
#include "bd/spin.hpp"
#include "bd/wake.hpp"
#include "bd/units.hpp"

#define BD_EXPORT extern "C" __attribute__((visibility("default")))

using namespace bd;

namespace {

// A round in flight, holding on to the air and wind it was fired into: new
// air or wind for the next shot leaves the rounds already flying alone.
struct Flight {
    std::shared_ptr<const StationAir> air;
    std::shared_ptr<const RangeWind> wind;
    std::unique_ptr<PointMassRound> round;
};
constexpr std::size_t kMaxFlights = 256;  // past this, the oldest round is released

struct Sim {
    Library lib;
    std::vector<std::string> errors;
    std::shared_ptr<StationAir> air;
    std::shared_ptr<RangeWind> wind;  // over `air`; new rounds fly through this
    std::map<int, Flight> flights;    // by id, from 1
    int current = 0;                  // the round the round calls read
    int nextId = 1;
    // The gun's physics for zero, fire and table (bd_setup).
    Real twist = 0;          // m per turn, signed; 0: no spin
    Real latitude = 0;       // rad
    int flags = 0;           // 1 spin, 2 Earth rotation
    std::string text;
    std::vector<double> sample;
    std::vector<float> wake;
};

Sim* g = nullptr;
Sim& sim() {
    if (!g) g = new Sim;
    return *g;
}

// The selected round, or null.
PointMassRound* current() {
    Sim& S = sim();
    const auto it = S.flights.find(S.current);
    return it == S.flights.end() ? nullptr : it->second.round.get();
}

const AirSource* flightAir() {
    Sim& S = sim();
    if (!S.air) S.air = std::make_shared<StationAir>(StationReading{101325.0, 288.15, 0.0, 0.0});
    if (S.wind) return S.wind.get();
    return S.air.get();
}

PointMassOptions physicsFor(const ProjectileDef& p, Real muzzleSpeed);

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

PointMassOptions physicsFor(const ProjectileDef& p, Real muzzleSpeed) {
    Sim& S = sim();
    PointMassOptions o;
    if ((S.flags & 1) && S.twist != 0)
        o.spin = spinParamsFor(p, S.twist, muzzleSpeed, S.air ? S.air->sample({}, 0) : AirSample{});
    if (S.flags & 2) {
        o.earthRotation = true;
        o.latitude = S.latitude;
    }
    return o;
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
// Clears the wind. Rounds already fired keep the air they were fired into.
BD_EXPORT void bd_station(double pressure, double temperature, double relativeHumidity, double altitudeMsl) {
    Sim& S = sim();
    S.wind.reset();
    S.air = std::make_shared<StationAir>(StationReading{pressure, temperature, relativeHumidity, altitudeMsl});
}

// Wind along a line of fire from the origin at `azimuth` (rad): one layer per
// (fromRange m, speed m/s, fromBearing rad, gust RMS m/s) quadruple in the
// text buffer as doubles; n layers. n = 0 removes the wind.
BD_EXPORT void bd_wind(double azimuth, int n) {
    Sim& S = sim();
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
    S.wind = std::make_shared<RangeWind>(*S.air, Vec3{}, azimuth, layers);
}

// The gun's physics for the calls below: twist (m per turn, + right-hand, 0
// none), latitude (rad, + north), flags (1 spin: stability, spin drift and
// jump; 2 Earth rotation: Coriolis and Eotvos).
BD_EXPORT void bd_setup(double twist, double latitude, int flags) {
    Sim& S = sim();
    S.twist = twist;
    S.latitude = latitude;
    S.flags = flags;
}

// The Miller stability Sg of cartridge i from the set-up twist in the
// station air (0 without a twist).
BD_EXPORT double bd_stability(int cartridge) {
    DragModel drag;
    const CartridgeDef* c;
    const ProjectileDef* p;
    if (!cartridgeModel(cartridge, drag, c, p) || sim().twist == 0) return 0;
    flightAir();
    return spinParamsFor(*p, sim().twist, c->muzzleVelocity, sim().air->sample({}, 0)).sgMuzzle;
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
    s.physics = physicsFor(*p, c->muzzleVelocity);
    return zeroElevation(s, *sim().air);
}

// Fires cartridge i from (x, y, z) ENU along azimuth / elevation (rad) at
// engine time t0, keeping its trace for bullet-time replay. Returns the new
// round's id (from 1; 0 if it cannot fire) and selects it: bd_advance,
// bd_round, bd_nose_at, bd_state_at and bd_wake read the selected round. Any
// number of rounds can be in flight at once (bd_select, bd_release).
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
    flightAir();
    PointMassOptions o = physicsFor(*p, c->muzzleVelocity);
    o.keepTrace = true;
    o.maxTime = 15.0;
    Sim& S = sim();
    Flight f;
    f.air = S.air;
    f.wind = S.wind;
    f.round = std::make_unique<PointMassRound>(drag, *flightAir(), l, o);
    const int id = S.nextId++;
    S.flights.emplace(id, std::move(f));
    if (S.flights.size() > kMaxFlights) S.flights.erase(S.flights.begin());
    S.current = id;
    return id;
}

// Selects round `id` for the round calls. 1 if it exists.
BD_EXPORT int bd_select(int id) {
    Sim& S = sim();
    S.current = id;
    return S.flights.count(id) ? 1 : 0;
}

// Releases round `id` (0: every round).
BD_EXPORT void bd_release(int id) {
    Sim& S = sim();
    if (id == 0)
        S.flights.clear();
    else
        S.flights.erase(id);
}

// Advances the round to engine time t (whole 1 ms steps; at most maxSteps).
// For bullet time, advance by less than real time: the steps are the same,
// only the clock the page shows them on is slower.
BD_EXPORT int bd_advance(double t, int maxSteps) {
    PointMassRound* r = current();
    return r ? static_cast<int>(r->advance(t, maxSteps > 0 ? maxSteps : (1 << 20))) : 0;
}

// 16 doubles: engine time, position x y z, velocity x y z, stopped, spin
// (rad/s), roll (rad), Sg now, tumbling, jump (rad), lowest Sg, time it went
// below Mach 1.2 and below Mach 1 (s of flight, -1 not yet).
BD_EXPORT const double* bd_round() {
    Sim& S = sim();
    S.sample.assign(16, 0.0);
    if (const PointMassRound* rp = current()) {
        const auto& r = *rp;
        S.sample = {r.time(), r.position().x, r.position().y, r.position().z,
                    r.velocity().x, r.velocity().y, r.velocity().z, r.stopped() ? 1.0 : 0.0,
                    r.spinRate(), r.roll(), r.stability(), r.tumbling() ? 1.0 : 0.0, r.jumpAngle(),
                    r.minStability(), r.transonicStart(), r.subsonicTime()};
    }
    return S.sample.data();
}

// The nose's direction (unit ENU) at a flown instant: the path turned by the
// yaw of repose and the wobble (display).
BD_EXPORT const double* bd_nose_at(double t) {
    Sim& S = sim();
    S.sample.assign(3, 0.0);
    if (const PointMassRound* r = current()) {
        const Vec3 n = r->noseAt(t);
        S.sample = {n.x, n.y, n.z};
    }
    return S.sample.data();
}

// Bullet time: 9 doubles, the round's state at any engine time already flown
// (time, position, velocity, spin, roll), smooth between steps. Advance one
// step (1 ms) ahead of the time drawn: bd_advance(t + 0.001), then bd_state_at(t).
BD_EXPORT const double* bd_state_at(double t) {
    Sim& S = sim();
    S.sample.assign(9, 0.0);
    if (const PointMassRound* r = current()) {
        const FlightSample f = r->stateAt(t);
        S.sample = {f.time, f.position.x, f.position.y, f.position.z, f.velocity.x, f.velocity.y, f.velocity.z,
                    f.spin, f.roll};
    }
    return S.sample.data();
}

// A range card for cartridge i (preferred drag model, the set-up physics) in
// the station air and wind, firing along `azimuth` from the origin: JSON rows
// of range, drop, windage (m), time (s), speed (m/s), energy (J), Mach, Sg.
BD_EXPORT const char* bd_table(int cartridge, double zeroRange, double sightHeight, double maxRange, double step,
                               double azimuth) {
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
        s.azimuth = azimuth;
        s.physics = physicsFor(*p, c->muzzleVelocity);
        const Real e = zeroElevation(s, *sim().air);
        std::vector<Real> ranges;
        for (Real r = 0; r <= maxRange + 1e-9; r += step) ranges.push_back(r);
        bool first = true;
        for (const RangeRow& r : rangeTable(s, *flightAir(), e, ranges)) {
            if (!r.reached) continue;
            if (!first) o += ',';
            first = false;
            o += '[';
            for (double v : {r.range, r.drop, r.windage, r.time, r.speed, r.energy, r.mach, r.stability}) {
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

// The round's wake (the trace) at engine time t: float32, the count, then 6
// per stretch: x, y, z (ENU), half-width (m), deficit (strength), age (s).
// Stretches every `spacing` m, up to `maxAge` s old; mass (kg) and diameter
// (m) of the bullet.
BD_EXPORT const float* bd_wake(double t, double spacing, double maxAge, double mass, double diameter) {
    Sim& S = sim();
    S.wake.assign(1, 0.0f);
    if (const PointMassRound* r = current()) {
        WakeParams p;
        p.spacing = spacing;
        p.maxAge = maxAge;
        for (const WakeElement& e : wakeAt(*r, mass, diameter, t, p))
            for (double v : {e.position.x, e.position.y, e.position.z, e.radius, e.deficit, e.age})
                S.wake.push_back(static_cast<float>(v));
        S.wake[0] = static_cast<float>((S.wake.size() - 1) / 6);
    }
    return S.wake.data();
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
// the maths, a whole trajectory, and a spinning round on a rotating Earth, as
// 16 hex digits each.
BD_EXPORT const char* bd_determinism() {
    const std::uint64_t h = determinismProbe(), t = trajectoryProbe(), s = spinProbe();
    char b[64];
    std::snprintf(b, sizeof b, "%016llx %016llx %016llx", static_cast<unsigned long long>(h),
                  static_cast<unsigned long long>(t), static_cast<unsigned long long>(s));
    putText(b);
    return sim().text.c_str();
}
