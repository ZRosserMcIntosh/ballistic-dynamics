// Ballistic Dynamics — headless tool: the library, the air at a station.
// SPDX-License-Identifier: MIT
//
//   bdemo list                         the data library, with provenance kinds
//   bdemo air P_Pa T_C RH alt_m        the air a station reading gives, and up the path
//   bdemo table cartridge [model ...]  a range card (drop, drift, speed, energy, time)
//   bdemo determinism                  the determinism hashes of this build
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/library.hpp"
#include "bd/solver.hpp"
#include "bd/spin.hpp"
#include "bd/units.hpp"

using namespace bd;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

const char* kindName(ProvenanceKind k) {
    switch (k) {
        case ProvenanceKind::Published: return "published";
        case ProvenanceKind::Measured: return "measured";
        default: return "estimated";
    }
}

void printProvenance(const ProvenanceMap& m) {
    for (const auto& [k, p] : m) std::printf("      %-22s %-9s %s\n", k.c_str(), kindName(p.kind), p.note.c_str());
}

int list() {
    Library lib;
    int bad = 0;
    for (const char* f : {"/drag_standard.json", "/projectiles.json", "/cartridges.json", "/rifles.json"})
        for (const auto& e : lib.load(readFile(std::string(BD_DATA_DIR) + f))) {
            std::fprintf(stderr, "%s\n", e.c_str());
            ++bad;
        }
    for (const auto& e : lib.resolve()) {
        std::fprintf(stderr, "%s\n", e.c_str());
        ++bad;
    }
    std::printf("projectiles\n");
    for (const auto& p : lib.projectiles()) {
        std::printf("  %-14s %s: %.1f gr, %.3f in\n", p.id.c_str(), p.name.c_str(), units::toGrains(p.mass),
                    units::toInches(p.diameter));
        printProvenance(p.provenance);
    }
    std::printf("cartridges\n");
    for (const auto& c : lib.cartridges()) {
        std::printf("  %-14s %s: %.0f fps from %.0f in\n", c.id.c_str(), c.name.c_str(), units::toFps(c.muzzleVelocity),
                    units::toInches(c.testBarrel));
        printProvenance(c.provenance);
    }
    std::printf("rifles\n");
    for (const auto& r : lib.rifles()) {
        std::printf("  %-14s %s: 1:%.2f in %s, %.0f in barrel\n", r.id.c_str(), r.name.c_str(), units::toInches(r.twist),
                    r.rightHandTwist ? "RH" : "LH", units::toInches(r.barrel));
        printProvenance(r.provenance);
    }
    return bad ? 1 : 0;
}

int air(int argc, char** argv) {
    if (argc < 6) {
        std::fprintf(stderr, "usage: bdemo air station_pressure_Pa temperature_C relative_humidity_0_1 altitude_m\n");
        return 2;
    }
    const StationReading r{std::atof(argv[2]), units::celsiusToKelvin(std::atof(argv[3])), std::atof(argv[4]),
                           std::atof(argv[5])};
    const StationAir a(r);
    std::printf("%8s %10s %8s %9s %8s %10s\n", "up_m", "p_Pa", "T_C", "rho", "a_m/s", "DA_m");
    for (Real z : {0.0, 10.0, 50.0, 100.0, 500.0}) {
        const AirSample s = a.sample({0, 0, z}, 0);
        std::printf("%8.0f %10.1f %8.2f %9.5f %8.2f %10.0f\n", z, s.pressure, s.temperature - 273.15, s.density,
                    s.speedOfSound, s.densityAltitude);
    }
    return 0;
}

int determinism() {
    const std::uint64_t h = determinismProbe(), t = trajectoryProbe(), sp = spinProbe();
    std::printf("maths      0x%016llx %s\n", static_cast<unsigned long long>(h),
                h == kDeterminismGolden ? "(golden)" : "(DIFFERS from the golden value)");
    std::printf("trajectory 0x%016llx %s\n", static_cast<unsigned long long>(t),
                t == kTrajectoryGolden ? "(golden)" : "(DIFFERS from the golden value)");
    std::printf("spin       0x%016llx %s\n", static_cast<unsigned long long>(sp),
                sp == kSpinGolden ? "(golden)" : "(DIFFERS from the golden value)");
    return h == kDeterminismGolden && t == kTrajectoryGolden && sp == kSpinGolden ? 0 : 1;
}

// A range card for a cartridge in the ICAO standard atmosphere (sea level,
// 15 C, dry), flat range, line of sight horizontal, wind from 3 o'clock.
int table(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: bdemo table cartridge [G1|G7|bullet|-] [zero_yd 100] [sight_in 1.5] [wind_mph 10] "
                             "[max_yd 1000] [step_yd 100] [bc override|-] [rifle=ID] [lat=DEG] [az=DEG]\n");
        return 2;
    }
    Library lib;
    for (const char* f : {"/drag_standard.json", "/projectiles.json", "/cartridges.json", "/rifles.json"})
        for (const auto& e : lib.load(readFile(std::string(BD_DATA_DIR) + f))) std::fprintf(stderr, "%s\n", e.c_str());
    const CartridgeDef* c = lib.cartridge(argv[2]);
    if (!c) {
        std::fprintf(stderr, "unknown cartridge %s\n", argv[2]);
        return 2;
    }
    const ProjectileDef* p = lib.projectile(c->projectile);
    ProjectileDef pd = *p;
    std::string prefer = argc > 3 && std::strcmp(argv[3], "-") ? argv[3] : "";
    // Named options: rifle=ID (spin from its twist), lat=DEG (Earth rotation), az=DEG.
    const RifleDef* rifle = nullptr;
    Real latDeg = 0, azDeg = 0;
    bool earth = false;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("rifle=", 0) == 0) rifle = lib.rifle(a.substr(6));
        if (a.rfind("lat=", 0) == 0) {
            latDeg = std::atof(a.c_str() + 4);
            earth = true;
        }
        if (a.rfind("az=", 0) == 0) azDeg = std::atof(a.c_str() + 3);
    }
    int positional = argc;
    for (int i = 3; i < argc; ++i)
        if (std::strchr(argv[i], '=')) {
            positional = i;
            break;
        }
    argc = positional;
    if (argc > 9 && std::strcmp(argv[9], "-")) {  // a BC override for the chosen family
        BallisticCoefficient b;
        b.family = prefer.empty() ? "G1" : prefer;
        b.values = {units::bcToSi(std::atof(argv[9]))};
        b.minSpeeds = {0};
        (b.family == "G7" ? pd.bcG7 : pd.bcG1) = b;
    }
    std::string err;
    RangeSetup s;
    s.drag = dragModelFor(pd, lib, prefer, err);
    if (!err.empty()) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 2;
    }
    s.muzzleSpeed = c->muzzleVelocity;
    s.mass = p->mass;
    s.zeroRange = units::yards(argc > 4 ? std::atof(argv[4]) : 100);
    s.sightHeight = units::inches(argc > 5 ? std::atof(argv[5]) : 1.5);
    const Real mph = argc > 6 ? std::atof(argv[6]) : 10;
    const Real maxYd = argc > 7 ? std::atof(argv[7]) : 1000, stepYd = argc > 8 ? std::atof(argv[8]) : 100;
    const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    s.azimuth = azDeg * kPi / 180.0;
    // Wind from 3 o'clock relative to the line of fire.
    const RangeWind wind(air, {}, s.azimuth, {{0, mph * 0.44704, s.azimuth + 0.5 * kPi}});
    if (rifle) s.physics.spin = spinParamsFor(*p, rifle->rightHandTwist ? rifle->twist : -rifle->twist, s.muzzleSpeed, air.sample({}, 0));
    if (earth) {
        s.physics.earthRotation = true;
        s.physics.latitude = latDeg * kPi / 180.0;
    }
    const Real e = zeroElevation(s, air);
    std::vector<Real> ranges;
    for (Real y = 0; y <= maxYd + 1e-9; y += stepYd) ranges.push_back(units::yards(y));
    const auto rows = rangeTable(s, wind, e, ranges);
    std::printf("%s, %s; zero %.0f yd, sight %.2f in, %.0f mph from 3 o'clock; ICAO sea level, dry\n", c->name.c_str(),
                prefer.empty() ? "preferred model" : prefer.c_str(), units::toYards(s.zeroRange),
                units::toInches(s.sightHeight), mph);
    if (rifle) std::printf("spin: %s, 1:%.2f in, Sg %.2f at the muzzle\n", rifle->name.c_str(), units::toInches(rifle->twist), s.physics.spin.sgMuzzle);
    if (earth) std::printf("Earth rotation at %.2f deg latitude, firing at azimuth %.0f deg\n", latDeg, azDeg);
    std::printf("%6s %9s %7s %7s %9s %7s %7s %7s %8s %6s %5s\n", "yd", "drop_in", "MOA", "mil", "wind_in", "MOA", "fps",
                "ft_lbf", "time_s", "mach", "Sg");
    for (const auto& r : rows) {
        if (!r.reached) continue;
        const Real ang = r.range > 0 ? 1.0 / r.range : 0;
        std::printf("%6.0f %9.2f %7.2f %7.2f %9.2f %7.2f %7.0f %7.0f %8.4f %6.3f %5.2f\n", units::toYards(r.range),
                    units::toInches(r.drop), r.drop * ang / units::kMoa, r.drop * ang / units::kMrad,
                    units::toInches(r.windage), r.windage * ang / units::kMoa, units::toFps(r.speed),
                    r.energy / 1.3558179483314004, r.time, r.mach, r.stability);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const char* cmd = argc > 1 ? argv[1] : "list";
    if (!std::strcmp(cmd, "list")) return list();
    if (!std::strcmp(cmd, "air")) return air(argc, argv);
    if (!std::strcmp(cmd, "determinism")) return determinism();
    if (!std::strcmp(cmd, "table")) return table(argc, argv);
    std::fprintf(stderr, "usage: bdemo list | air P T RH alt | table cartridge ... | determinism\n");
    return 2;
}
