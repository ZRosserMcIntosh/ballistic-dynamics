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
#include "bd/shooter.hpp"
#include "bd/solver.hpp"
#include "bd/spin.hpp"
#include "bd/wake.hpp"
#include "bd/units.hpp"
#include "bd/variation.hpp"

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
    // Milestone 4 (bd_powder, bd_group): the powder's temperature (K, 0: the
    // published velocity) and the rifle's mean 5-shot group (MOA, 0: none).
    Real powder = 0;
    Real group = 0;
    RoundDraw lastDraw;
    Real lastSpeed = 0;
    std::string text;
    std::vector<double> sample;
    std::vector<float> wake;
    // The shooter layer (ADR 0005): one weapon in hand.
    std::unique_ptr<shooter::Weapon> weapon;
    shooter::ShooterInput input;
    std::vector<shooter::ShooterEvent> events;
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

// The cartridge's variation: its velocity spread and temperature, its
// bullet's BC spread, and the set rifle group.
VariationModel variationOf(const CartridgeDef& c, const ProjectileDef& p) {
    VariationModel m = variationFor(c, p, nullptr);
    if (sim().group > 0) m.angleSd = units::moa(sim().group) / kFiveShotSpreadPerSigma;
    return m;
}
// The muzzle velocity at the set powder temperature.
Real powderSpeed(const CartridgeDef& c, const ProjectileDef& p) { return muzzleSpeedAt(c, variationOf(c, p), sim().powder); }

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
        o += ",\"mass\":";
        jsonNum(o, r.mass);
        o += ",\"magazine\":";
        jsonNum(o, r.magazine);
        o += ",\"fireModes\":[";
        for (std::size_t k = 0; k < r.fireModes.size(); ++k) {
            if (k) o += ',';
            jsonString(o, r.fireModes[k]);
        }
        o += "],";
        provenance(o, r.provenance);
        o += '}';
    }
    o += "],\"stances\":[";
    for (std::size_t i = 0; i < L.stances().size(); ++i) {
        const auto& st = L.stances()[i];
        if (i) o += ',';
        o += "{\"id\":";
        jsonString(o, st.id);
        o += ",\"name\":";
        jsonString(o, st.name);
        o += ',';
        provenance(o, st.provenance);
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

namespace {
int fireAlong(int cartridge, const Vec3& from, const Vec3& bore, double t0, std::uint64_t seed, std::uint64_t stream) {
    DragModel drag;
    const CartridgeDef* c;
    const ProjectileDef* p;
    if (!cartridgeModel(cartridge, drag, c, p)) return 0;
    Sim& S = sim();
    S.lastDraw = seed ? drawRound(variationOf(*c, *p), seed, stream) : RoundDraw{};
    S.lastSpeed = powderSpeed(*c, *p) + S.lastDraw.velocityOffset;
    Launch l;
    l.position = from;
    l.direction = thrownDirection(bore, S.lastDraw);
    l.muzzleSpeed = S.lastSpeed;
    l.time = t0;
    flightAir();
    PointMassOptions o = physicsFor(*p, S.lastSpeed);
    o.keepTrace = true;
    o.maxTime = 15.0;
    Flight f;
    f.air = S.air;
    f.wind = S.wind;
    f.round = std::make_unique<PointMassRound>(drag.scaledBc(S.lastDraw.bcFactor), *flightAir(), l, o);
    const int id = S.nextId++;
    S.flights.emplace(id, std::move(f));
    if (S.flights.size() > kMaxFlights) S.flights.erase(S.flights.begin());
    S.current = id;
    return id;
}

int fire(int cartridge, double x, double y, double z, double azimuth, double elevation, double t0, std::uint64_t seed,
         std::uint64_t stream) {
    return fireAlong(cartridge, {x, y, z}, frames::directionFromAzEl(azimuth, elevation), t0, seed, stream);
}
}  // namespace

// Fires cartridge i from (x, y, z) ENU along azimuth / elevation (rad) at
// engine time t0, keeping its trace for bullet-time replay, at the powder's
// velocity (bd_powder). Returns the new round's id (from 1; 0 if it cannot
// fire) and selects it: bd_advance, bd_round, bd_nose_at, bd_state_at and
// bd_wake read the selected round. Any number of rounds can be in flight at
// once (bd_select, bd_release). Every round of a load is the same round;
// bd_fire_round draws each its own.
BD_EXPORT int bd_fire(int cartridge, double x, double y, double z, double azimuth, double elevation, double t0) {
    return fire(cartridge, x, y, z, azimuth, elevation, t0, 0, 0);
}

// Milestone 4: as bd_fire, with this round's own draw from stream (seed,
// stream): its muzzle velocity, BC and the barrel's throw (the load's
// spreads and the bd_group rifle). seed 0: no draw. seed and stream are
// integers up to 2^53 (exact in a double); the same pair gives the same
// round on every machine.
BD_EXPORT int bd_fire_round(int cartridge, double x, double y, double z, double azimuth, double elevation, double t0,
                            double seed, double stream) {
    return fire(cartridge, x, y, z, azimuth, elevation, t0, static_cast<std::uint64_t>(seed), static_cast<std::uint64_t>(stream));
}

// The powder's temperature (K) for the calls that follow: bd_fire, and the
// flights of bd_table and bd_spread (their zero stays at the published
// velocity, the zeroing day's). 0: the published velocity.
BD_EXPORT void bd_powder(double temperature) { sim().powder = temperature > 0 ? temperature : 0; }

// The rifle's mean 5-shot group (MOA) for bd_fire_round and bd_spread; 0: none.
BD_EXPORT void bd_group(double moa) { sim().group = moa > 0 ? moa : 0; }

// The last fired round's draw: muzzle velocity (m/s, as it left), its
// velocity offset (m/s), BC factor, the barrel's throw up and right (rad).
BD_EXPORT const double* bd_last_draw() {
    Sim& S = sim();
    S.sample = {S.lastSpeed, S.lastDraw.velocityOffset, S.lastDraw.bcFactor, S.lastDraw.up, S.lastDraw.right};
    return S.sample.data();
}

// The spread of impacts at `range` for cartridge i zeroed at zeroRange (the
// set physics, station air and wind, firing along `azimuth`): standard
// deviations (m) vertical, horizontal, and the vertical from the velocity,
// the BC and the barrel.
BD_EXPORT const double* bd_spread(int cartridge, double zeroRange, double sightHeight, double range, double azimuth) {
    Sim& S = sim();
    S.sample.assign(5, 0.0);
    DragModel drag;
    const CartridgeDef* c;
    const ProjectileDef* p;
    if (!cartridgeModel(cartridge, drag, c, p)) return S.sample.data();
    flightAir();
    RangeSetup s;
    s.drag = drag;
    s.muzzleSpeed = c->muzzleVelocity;
    s.mass = p->mass;
    s.sightHeight = sightHeight;
    s.zeroRange = zeroRange;
    s.azimuth = azimuth;
    s.physics = physicsFor(*p, c->muzzleVelocity);
    const Real e = zeroElevation(s, *S.air);
    s.muzzleSpeed = powderSpeed(*c, *p);
    s.physics = physicsFor(*p, s.muzzleSpeed);
    const Spread sp = spreadAt(s, *flightAir(), e, range, variationOf(*c, *p));
    S.sample = {sp.vertical, sp.horizontal, sp.fromVelocity, sp.fromBc, sp.fromBarrel};
    return S.sample.data();
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
// the station air and wind, firing along `azimuth` from the origin at engine
// time t0 (the gusts it meets): JSON rows of range, drop, windage (m), time
// (s), speed (m/s), energy (J), Mach, Sg.
BD_EXPORT const char* bd_table(int cartridge, double zeroRange, double sightHeight, double maxRange, double step,
                               double azimuth, double t0) {
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
        // Zeroed at the published velocity; flown at the powder's (bd_powder).
        s.muzzleSpeed = powderSpeed(*c, *p);
        s.physics = physicsFor(*p, s.muzzleSpeed);
        s.time = std::isfinite(t0) ? t0 : 0.0;
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

// The round's wake (the trace) at engine time t: float32, the count, then 7
// per stretch: x, y, z (ENU), half-width (m), velocity deficit, age (s), and
// the refractive-index change in sea-level n - 1 (the optical strength).
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
            for (double v : {e.position.x, e.position.y, e.position.z, e.radius, e.deficit, e.age, e.index})
                S.wake.push_back(static_cast<float>(v));
        S.wake[0] = static_cast<float>((S.wake.size() - 1) / 7);
    }
    return S.wake.data();
}

// 9 doubles: pressure, temperature, density, speed of sound, relative humidity,
// density altitude, wind east, north, up. Zeros before bd_station.
BD_EXPORT const double* bd_air(double x, double y, double z, double t) {
    Sim& S = sim();
    S.sample.assign(9, 0.0);
    if (S.air) {
        // The air with its wind (the layers and gusts rounds fly through).
        const AirSample a = flightAir()->sample({x, y, z}, t);
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

// ---- The shooter layer (ADR 0005) ----

namespace {

bool weaponParts(int rifle, int cartridge, int stance, shooter::WeaponSpec& w, const StanceDef*& st) {
    const Library& L = sim().lib;
    if (rifle < 0 || rifle >= static_cast<int>(L.rifles().size()) || cartridge < 0 ||
        cartridge >= static_cast<int>(L.cartridges().size()) || stance < 0 || stance >= static_cast<int>(L.stances().size()))
        return false;
    const CartridgeDef& c = L.cartridges()[static_cast<std::size_t>(cartridge)];
    const ProjectileDef* p = L.projectile(c.projectile);
    if (!p) return false;
    std::string err;
    w = shooter::weaponSpecFor(L.rifles()[static_cast<std::size_t>(rifle)], c, *p, err);
    st = &L.stances()[static_cast<std::size_t>(stance)];
    return err.empty();
}

}  // namespace

// Puts rifle i with cartridge j in the shooter's hands, in stance k, with its
// clock starting at engine time t0. Its shots draw from streams streamBase + 1,
// + 2, ... of seed (integers up to 2^53), the same streams bd_fire_shot flies
// them with. Returns 1, or 0 when the rifle lacks shooter data.
BD_EXPORT int bd_weapon(int rifle, int cartridge, int stance, double seed, double streamBase, double t0) {
    shooter::WeaponSpec w;
    const StanceDef* st = nullptr;
    Sim& S = sim();
    S.weapon.reset();
    S.events.clear();
    S.input = {};
    if (!weaponParts(rifle, cartridge, stance, w, st)) return 0;
    S.weapon = std::make_unique<shooter::Weapon>(w, *st, static_cast<std::uint64_t>(seed),
                                                 static_cast<std::uint64_t>(streamBase), t0);
    return 1;
}

// Changes stance (an index into the catalog's stances).
BD_EXPORT int bd_weapon_stance(int stance) {
    Sim& S = sim();
    const Library& L = S.lib;
    if (!S.weapon || stance < 0 || stance >= static_cast<int>(L.stances().size())) return 0;
    S.weapon->setStance(L.stances()[static_cast<std::size_t>(stance)]);
    return 1;
}

// Where the shooter looks (ENU), before the kick and the hold.
BD_EXPORT void bd_weapon_look(double x, double y, double z) {
    if (sim().weapon) sim().weapon->setLook({x, y, z});
}

// This frame's input, held for every step of the next bd_weapon_advance.
// flags: 1 trigger, 2 aim down the sight, 4 sprinting, 8 airborne, 16 reload.
// turnRate rad/s, moveSpeed m/s, mode an index into the rifle's fire modes.
BD_EXPORT void bd_weapon_input(int flags, double turnRate, double moveSpeed, int mode) {
    shooter::ShooterInput& in = sim().input;
    in.trigger = (flags & 1) != 0;
    in.ads = (flags & 2) != 0;
    in.sprinting = (flags & 4) != 0;
    in.airborne = (flags & 8) != 0;
    in.reload = (flags & 16) != 0;
    in.turnRate = turnRate;
    in.moveSpeed = moveSpeed;
    in.mode = mode;
}

// Runs the weapon's whole 1 ms steps up to engine time t. Returns how many
// events happened (read them with bd_weapon_event); a press given before this
// call fires on the first step of it, so its shot belongs to this frame.
BD_EXPORT int bd_weapon_advance(double t) {
    Sim& S = sim();
    S.events.clear();
    if (!S.weapon) return 0;
    S.weapon->advance(t, S.input, S.events);
    return static_cast<int>(S.events.size());
}

// Event i of the last advance: 13 doubles: kind (0 shot, 1 dry fire, 2 reload
// start, 3 reload end), time, step, and for a shot its number, seed, stream,
// the bore (x, y, z ENU), the hip cone's part (up, right, rad), the kick it
// gave (rad/s) and the rounds left.
BD_EXPORT const double* bd_weapon_event(int i) {
    Sim& S = sim();
    S.sample.assign(13, 0.0);
    if (i < 0 || i >= static_cast<int>(S.events.size())) return S.sample.data();
    const auto& e = S.events[static_cast<std::size_t>(i)];
    S.sample = {static_cast<double>(e.kind), e.time, static_cast<double>(e.step), static_cast<double>(e.shot),
                static_cast<double>(e.seed), static_cast<double>(e.stream), e.direction.x, e.direction.y, e.direction.z,
                e.coneUp, e.coneRight, e.kick, static_cast<double>(e.magazine)};
    return S.sample.data();
}

// Flies event i's shot: cartridge j from (x, y, z) ENU along that shot's bore,
// at its time, with its round's own draw (its seed and stream). Returns the
// round's id (0 if event i is not a shot), selected as bd_fire's is.
BD_EXPORT int bd_fire_shot(int cartridge, double x, double y, double z, int i) {
    Sim& S = sim();
    if (i < 0 || i >= static_cast<int>(S.events.size())) return 0;
    const auto& e = S.events[static_cast<std::size_t>(i)];
    if (e.kind != shooter::EventKind::Shot) return 0;
    return fireAlong(cartridge, {x, y, z}, e.direction, e.time, e.seed, e.stream);
}

// The weapon now, 12 doubles: add to the camera (pitch, yaw, rad: the kick
// and the hold), the kick alone (pitch, yaw), the gun in the hands (pitch,
// yaw, the viewmodel's only), the sight fraction (0 hip, 1 sight), the hip
// cone's half-angle (rad), its bloom (0..1), rounds in the magazine,
// reloading (0/1), shots fired.
BD_EXPORT const double* bd_weapon_state() {
    Sim& S = sim();
    S.sample.assign(12, 0.0);
    if (!S.weapon) return S.sample.data();
    const shooter::Weapon& g = *S.weapon;
    S.sample = {g.viewPitch(), g.viewYaw(), g.kickPitch(), g.kickYaw(), g.gunPitch(), g.gunYaw(), g.adsFraction(),
                g.cone(), g.bloom(), static_cast<double>(g.magazine()), g.reloading() ? 1.0 : 0.0,
                static_cast<double>(g.shotsFired())};
    return S.sample.data();
}

// Free recoil of rifle i with cartridge j, and the kick in stance k: 4
// doubles, momentum (N s), the free gun's velocity (m/s) and energy (J), the
// view's pitch rate per shot before the stance's factors (rad/s). Zeros when
// the rifle lacks shooter data.
BD_EXPORT const double* bd_recoil(int rifle, int cartridge, int stance) {
    Sim& S = sim();
    S.sample.assign(4, 0.0);
    shooter::WeaponSpec w;
    const StanceDef* st = nullptr;
    if (!weaponParts(rifle, cartridge, stance, w, st)) return S.sample.data();
    const shooter::FreeRecoil f = shooter::freeRecoil(w);
    S.sample = {f.momentum, f.velocity, f.energy, shooter::kickRate(w, *st)};
    return S.sample.data();
}
