// Ballistic Dynamics — WebAssembly bindings.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over the core so a web page can run it. The page only renders
// and reads input. Buffers returned by pointer are views into the module's
// memory, valid until the next call; web/README.md has the layouts.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/library.hpp"

#define BD_EXPORT extern "C" __attribute__((visibility("default")))

using namespace bd;

namespace {

struct Sim {
    Library lib;
    std::vector<std::string> errors;
    std::unique_ptr<StationAir> air;
    std::string text;
    std::vector<double> sample;
};

Sim* g = nullptr;
Sim& sim() {
    if (!g) g = new Sim;
    return *g;
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
BD_EXPORT void bd_station(double pressure, double temperature, double relativeHumidity, double altitudeMsl) {
    sim().air = std::make_unique<StationAir>(StationReading{pressure, temperature, relativeHumidity, altitudeMsl});
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

// The determinism hash (same as `bdemo determinism` and the native test), as
// 16 hex digits in the text buffer.
BD_EXPORT const char* bd_determinism() {
    const std::uint64_t h = determinismProbe();
    char b[24];
    std::snprintf(b, sizeof b, "%016llx", static_cast<unsigned long long>(h));
    putText(b);
    return sim().text.c_str();
}
