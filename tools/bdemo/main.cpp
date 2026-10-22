// Ballistic Dynamics — headless tool: the library, the air at a station.
// SPDX-License-Identifier: MIT
//
//   bdemo list                         the data library, with provenance kinds
//   bdemo air P_Pa T_C RH alt_m        the air a station reading gives, and up the path
//   bdemo determinism                  the determinism hash of this build
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/library.hpp"
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
    for (const char* f : {"/projectiles.json", "/cartridges.json", "/rifles.json"})
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
    const std::uint64_t h = determinismProbe();
    std::printf("0x%016llx %s\n", static_cast<unsigned long long>(h), h == kDeterminismGolden ? "(golden)" : "(DIFFERS from the golden value)");
    return h == kDeterminismGolden ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    const char* cmd = argc > 1 ? argv[1] : "list";
    if (!std::strcmp(cmd, "list")) return list();
    if (!std::strcmp(cmd, "air")) return air(argc, argv);
    if (!std::strcmp(cmd, "determinism")) return determinism();
    std::fprintf(stderr, "usage: bdemo list | air P T RH alt | determinism\n");
    return 2;
}
