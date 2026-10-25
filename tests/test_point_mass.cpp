// Ballistic Dynamics — the point-mass trajectory (Milestone 2): drag
// tables, exact solutions, convergence, determinism, and the six cartridges
// against an independent solver and published figures.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstdlib>

#include "bd/aviation_air.hpp"
#include "bd/determinism.hpp"
#include "bd/drag.hpp"
#include "bd/json.hpp"
#include "bd/point_mass.hpp"
#include "bd/rng.hpp"
#include "bd/solver.hpp"
#include "bd/units.hpp"
#include "bd_test.hpp"

using namespace bd;

namespace {

bool verbose() { return std::getenv("BD_VERBOSE") != nullptr; }

const DragTable& standardTable(const char* id) {
    static DragTable g1(*bdtest::library().dragCurve("g1"));
    static DragTable g7(*bdtest::library().dragCurve("g7"));
    return std::string(id) == "g1" ? g1 : g7;
}

BallisticCoefficient singleBc(const char* family, Real lbPerSqIn) {
    BallisticCoefficient b;
    b.family = family;
    b.values = {units::bcToSi(lbPerSqIn)};
    b.minSpeeds = {0};
    return b;
}

// The ICAO standard day at sea level, dry, through Aviation's atmosphere.
const StationAir& icao() {
    static const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    return air;
}

}  // namespace

TEST_CASE("drag: the standard tables load and interpolate") {
    const DragTable& g1 = standardTable("g1");
    const DragTable& g7 = standardTable("g7");
    CHECK(g1.mach().size() == 79);
    CHECK(g7.mach().size() == 84);
    CHECK_NEAR(g1.cd(0.0), 0.2629, 1e-12);
    CHECK_NEAR(g7.cd(0.0), 0.1198, 1e-12);
    // Linear between points, clamped outside.
    const Real m0 = g7.mach()[30], m1 = g7.mach()[31];
    CHECK_NEAR(g7.cd(0.5 * (m0 + m1)), 0.5 * (g7.cds()[30] + g7.cds()[31]), 1e-15);
    CHECK_NEAR(g1.cd(9.0), g1.cds().back(), 0);
    CHECK_NEAR(g1.cd(-1.0), g1.cds().front(), 0);
    // The transonic rise: C_D at Mach 1.2 is more than twice its subsonic value.
    CHECK(g1.cd(1.2) > 2.0 * g1.cd(0.5));
    CHECK(g7.cd(1.2) > 2.5 * g7.cd(0.5));
}

TEST_CASE("drag: a BC-scaled standard and a bullet curve give the same force for the same bullet") {
    // A bullet whose own curve is exactly i times G7 (form factor i) has
    // BC = m / (i d^2): both models must give the same deceleration.
    const Real mass = units::grains(175), d = units::inches(0.308), i = 1.05;
    const DragTable& g7 = standardTable("g7");
    std::vector<Real> cd;
    for (Real c : g7.cds()) cd.push_back(i * c);
    const DragModel own = DragModel::bullet(DragTable(g7.mach(), cd), mass, d);
    BallisticCoefficient bc;
    bc.family = "G7";
    bc.values = {mass / (i * d * d)};
    bc.minSpeeds = {0};
    const DragModel scaled = DragModel::standard(g7, bc);
    AirSample air;
    for (Real v : {150.0, 330.0, 345.0, 500.0, 800.0}) {
        const Vec3 a = own.acceleration({0, v, 0}, air), b = scaled.acceleration({0, v, 0}, air);
        CHECK_NEAR(a.y / b.y, 1.0, 1e-12);
        CHECK(a.y < 0);
    }
    // A stepped BC switches at its speeds (Sierra 175 SMK: .505 / .496 / .485).
    const ProjectileDef* smk = bdtest::library().projectile("smk_308_175");
    const DragModel stepped = DragModel::standard(standardTable("g1"), smk->bcG1);
    CHECK_NEAR(units::bcFromSi(stepped.bcAt(units::fps(2900))), 0.505, 1e-12);
    CHECK_NEAR(units::bcFromSi(stepped.bcAt(units::fps(2000))), 0.496, 1e-12);
    CHECK_NEAR(units::bcFromSi(stepped.bcAt(units::fps(1500))), 0.485, 1e-12);
    std::string err;
    CHECK(dragModelFor(*smk, bdtest::library(), "", err).kind() == DragModel::Kind::StandardBc);
    CHECK(dragModelFor(*smk, bdtest::library(), "bullet", err).kind() == DragModel::Kind::None);
    CHECK(!err.empty());
    // The family name in any case.
    CHECK(dragModelFor(*smk, bdtest::library(), "g7", err).kind() == DragModel::Kind::StandardBc);
    CHECK(err.empty());
    // A single BC built in code without speed bands applies at any speed.
    BallisticCoefficient one;
    one.family = "G7";
    one.values = {units::bcToSi(0.243)};
    const DragModel bare = DragModel::standard(standardTable("g7"), one);
    CHECK_NEAR(units::bcFromSi(bare.bcAt(units::fps(100))), 0.243, 1e-12);
}

TEST_CASE("point mass: in a vacuum the flight is the exact parabola") {
    const DragModel none = DragModel::bullet(DragTable({0, 5}, {0, 0}), 0.01, 0.008);
    const UniformAir air;
    Launch l;
    l.direction = frames::directionFromAzEl(0.3, 0.2);
    l.muzzleSpeed = 800;
    PointMassRound r(none, air, l);
    r.advance(2.0);
    const Vec3 v0 = l.direction * 800.0;
    const Real t = r.flightTime();
    CHECK_NEAR(t, 2.0, 1e-12);
    CHECK_NEAR(r.position().x, v0.x * t, 1e-9);
    CHECK_NEAR(r.position().y, v0.y * t, 1e-9);
    CHECK_NEAR(r.position().z, v0.z * t - 0.5 * kStandardGravity * t * t, 1e-9);  // RK4 is exact on a quadratic
    CHECK_NEAR(r.velocity().z, v0.z - kStandardGravity * t, 1e-9);
}

TEST_CASE("point mass: quadratic drag without gravity matches v = v0 / (1 + k v0 t)") {
    // Constant C_D, no gravity: dv/dt = -k v^2, k = rho C_D A / (2 m).
    const Real mass = 0.01, d = 0.008, cd = 0.3;
    const DragModel m = DragModel::bullet(DragTable({0, 5}, {cd, cd}), mass, d);
    const UniformAir air;
    Launch l;
    l.muzzleSpeed = 900;
    PointMassOptions o;
    o.gravity = {};
    PointMassRound r(m, air, l, o);
    r.advance(1.5);
    const Real k = 1.225 * cd * (0.25 * kPi * d * d) / (2 * mass);
    const Real t = r.flightTime();
    CHECK_NEAR(length(r.velocity()), 900.0 / (1 + k * 900.0 * t), 1e-7);
    CHECK_NEAR(r.position().y, dm::log(1 + k * 900.0 * t) / k, 1e-6);
}

TEST_CASE("point mass: the 1 ms step has converged (against 0.1 ms) at 1,000 yd") {
    RangeSetup s;
    s.drag = DragModel::standard(standardTable("g7"), singleBc("G7", 0.243));
    s.muzzleSpeed = units::fps(2600);
    s.sightHeight = units::inches(1.5);
    s.zeroRange = units::yards(100);
    const Real e = zeroElevation(s, icao());
    const auto coarse = rangeTable(s, icao(), e, {units::yards(1000)});
    RangeSetup fine = s;
    fine.step = 1e-4;
    const auto finer = rangeTable(fine, icao(), e, {units::yards(1000)});
    CHECK(coarse[0].reached && finer[0].reached);
    CHECK_NEAR(coarse[0].drop, finer[0].drop, 1e-4);   // 0.1 mm of 11 m
    CHECK_NEAR(coarse[0].time, finer[0].time, 1e-6);
    CHECK_NEAR(coarse[0].speed, finer[0].speed, 1e-4);
}

TEST_CASE("point mass: results do not depend on how the host chops its frames") {
    const DragModel m = DragModel::standard(standardTable("g7"), singleBc("G7", 0.243));
    Launch l;
    l.direction = frames::directionFromAzEl(1.0, 0.003);
    l.muzzleSpeed = units::fps(2600);
    l.time = 0.4167;  // fired mid-frame
    auto fly = [&](Real hz, bool jitter) {
        PointMassRound r(m, icao(), l);
        Rng rng(3);
        Real t = 0;
        while (t < 2.5) {
            t = std::min(2.5, t + (jitter ? rng.uniform(0.0, 0.04) : 1.0 / hz));
            r.advance(t);
        }
        return r;
    };
    const PointMassRound a = fly(60, false), b = fly(144, false), c = fly(0, true);
    CHECK(a.flightTime() == b.flightTime());
    CHECK(bitsOf(a.position().x) == bitsOf(b.position().x));
    CHECK(bitsOf(a.position().z) == bitsOf(b.position().z));
    CHECK(bitsOf(a.velocity().y) == bitsOf(c.velocity().y));
}

TEST_CASE("point mass: bullet time, the state between steps is smooth and exact at the steps") {
    const DragModel m = DragModel::standard(standardTable("g7"), singleBc("G7", 0.326));
    Launch l;
    l.muzzleSpeed = units::fps(2710);
    l.direction = frames::directionFromAzEl(0, 0.01);
    PointMassOptions o;
    o.keepTrace = true;
    PointMassRound r(m, icao(), l, o);
    r.advance(1.0);
    // Exact at a step.
    const FlightSample at = r.stateAt(0.5);
    CHECK(bitsOf(at.position.y) == bitsOf(r.trace()[500].position.y));
    // Between steps, against a round flown with a 20x finer step.
    PointMassOptions f = o;
    f.step = 5e-5;
    PointMassRound fine(m, icao(), l, f);
    fine.advance(1.0);
    Real worst = 0, worstV = 0;
    for (int k = 0; k < 200; ++k) {
        const Real t = 0.0001 + 0.004987 * k;
        const FlightSample a = r.stateAt(t), b = fine.stateAt(t);
        worst = std::max(worst, length(a.position - b.position));
        worstV = std::max(worstV, length(a.velocity - b.velocity));
    }
    CHECK(worst < 2e-5);   // 20 micrometres: a bullet camera at 1/1000 speed sees no steps
    CHECK(worstV < 0.05);  // m/s
    // A 1/1000 slow-motion camera at 60 fps asks for a state every 16.7 us.
    const FlightSample p = r.stateAt(0.3), q = r.stateAt(0.3 + 1.0 / 60000);
    CHECK_NEAR(length(q.position - p.position) / (1.0 / 60000), length(p.velocity), 0.5);
}

TEST_CASE("determinism: a whole trajectory gives the same bits on every platform") {
    const std::uint64_t h = trajectoryProbe();
    if (h != kTrajectoryGolden) {
        ++bdtest::failures();
        std::fprintf(stderr, "  trajectory hash 0x%016llx, expected 0x%016llx\n", static_cast<unsigned long long>(h),
                     static_cast<unsigned long long>(kTrajectoryGolden));
    }
}

// ---- Against an independent solver (tests/reference/point_mass.json) ----

namespace {

struct Comparison {
    Real worstDrop = 0, worstWind = 0, worstSpeed = 0, worstTime = 0;  // relative, beyond 100 yd
    Real worstDropIn = 0;
    int rows = 0;
};

Comparison compareCase(const json::Value& c) {
    Comparison out;
    const std::string id = c.find("id")->asString();
    const std::string model = c.find("model")->asString();
    RangeSetup s;
    s.drag = DragModel::standard(standardTable(model == "G1" ? "g1" : "g7"),
                                 singleBc(model.c_str(), c.find("bc")->asNumber()));
    s.muzzleSpeed = units::fps(c.find("muzzle_velocity_fps")->asNumber());
    s.mass = units::grains(c.find("mass_gr")->asNumber());
    s.sightHeight = units::inches(c.find("sight_height_in")->asNumber());
    s.zeroRange = units::yards(c.find("zero_yd")->asNumber());
    const Real e = zeroElevation(s, icao());
    // 10 mph from 3 o'clock; firing north, so from the east.
    const RangeWind crosswind(icao(), {}, 0.0, {{0, 10 * 0.44704, 0.5 * kPi}});
    std::vector<Real> ranges;
    for (const auto& r : c.find("rows")->items()) ranges.push_back(units::yards(r.find("range_yd")->asNumber()));
    const auto rows = rangeTable(s, crosswind, e, ranges);
    if (verbose()) std::printf("  %s\n  %6s %9s %9s %8s %8s %8s %8s\n", id.c_str(), "yd", "drop", "ref", "wind", "ref", "fps", "ref");
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const json::Value& ref = c.find("rows")->items()[i];
        const Real yd = ref.find("range_yd")->asNumber();
        const Real drop = units::toInches(rows[i].drop), wind = units::toInches(rows[i].windage);
        const Real fps = units::toFps(rows[i].speed);
        const Real rDrop = ref.find("drop_in")->asNumber(), rWind = ref.find("windage_in")->asNumber();
        const Real rFps = ref.find("velocity_fps")->asNumber(), rTime = ref.find("time_s")->asNumber();
        if (verbose())
            std::printf("  %6.0f %9.2f %9.2f %8.2f %8.2f %8.1f %8.1f\n", yd, drop, rDrop, wind, rWind, fps, rFps);
        if (!rows[i].reached) {
            out.worstDrop = 1e9;
            continue;
        }
        ++out.rows;
        if (yd <= 0 || std::abs(rDrop) < 1.0) continue;  // at the zero both are ~0: no relative error
        out.worstDrop = std::max(out.worstDrop, std::abs(drop - rDrop) / std::abs(rDrop));
        out.worstDropIn = std::max(out.worstDropIn, std::abs(drop - rDrop));
        out.worstWind = std::max(out.worstWind, std::abs(wind - rWind) / std::abs(rWind));
        out.worstSpeed = std::max(out.worstSpeed, std::abs(fps - rFps) / rFps);
        out.worstTime = std::max(out.worstTime, std::abs(rows[i].time - rTime) / rTime);
    }
    return out;
}

}  // namespace

TEST_CASE("validation: the six cartridges against an independent point-mass solver") {
    const auto doc = json::parse(bdtest::readFile(std::string(BD_TEST_DIR) + "/reference/point_mass.json"));
    CHECK(doc.ok());
    if (!doc.ok()) return;
    int cases = 0;
    for (const auto& c : doc.value.find("cases")->items()) {
        const Comparison k = compareCase(c);
        ++cases;
        if (verbose())
            std::printf("  %-14s drop %.3f%% (%.2f in)  wind %.3f%%  speed %.3f%%  time %.3f%%\n",
                        c.find("id")->asString().c_str(), 100 * k.worstDrop, k.worstDropIn, 100 * k.worstWind,
                        100 * k.worstSpeed, 100 * k.worstTime);
        CHECK(k.rows == static_cast<int>(c.find("rows")->items().size()));
        // Measured agreement (docs/validation.md): drop within 0.01 %, wind
        // within 0.03 % (0.4 % for the 9 mm's few inches), speed and time
        // within 0.05 %. The bands leave a little room, not a lot.
        CHECK(k.worstDrop < 0.001);
        CHECK(k.worstWind < 0.005);
        CHECK(k.worstSpeed < 0.001);
        CHECK(k.worstTime < 0.0005);
    }
    CHECK(cases == 9);
}

// ---- Against published figures (docs/validation.md has the sources) ----

namespace {

RangeRow publishedRow(const char* cartridge, const char* model, Real bcOverride, Real zeroYd, Real sightIn,
                      Real windMph, Real atYd) {
    const Library& lib = bdtest::library();
    const CartridgeDef* c = lib.cartridge(cartridge);
    ProjectileDef p = *lib.projectile(c->projectile);
    if (bcOverride > 0) (std::string(model) == "G7" ? p.bcG7 : p.bcG1) = singleBc(model, bcOverride);
    std::string err;
    RangeSetup s;
    s.drag = dragModelFor(p, lib, model, err);
    s.muzzleSpeed = c->muzzleVelocity;
    s.mass = p.mass;
    s.zeroRange = units::yards(zeroYd);
    s.sightHeight = units::inches(sightIn);
    const RangeWind wind(icao(), {}, 0.0, {{0, windMph * 0.44704, 0.5 * kPi}});
    return rangeTable(s, wind, zeroElevation(s, icao()), {units::yards(atYd)})[0];
}

Real mil(const RangeRow& r, Real value) { return value / r.range / units::kMrad; }

}  // namespace

TEST_CASE("validation: published drop, drift and velocity figures") {
    // 6.5 Creedmoor 140 ELD Match, G1 0.646, 2710 fps, 100 yd zero
    // (LoadDevelopment.com chart): -3.8, -13.4, -29.6 in at 200, 300, 400 yd.
    CHECK_NEAR(units::toInches(publishedRow("hornady_65cm_140", "G1", 0, 100, 1.5, 0, 200).drop), -3.8, 0.1);
    CHECK_NEAR(units::toInches(publishedRow("hornady_65cm_140", "G1", 0, 100, 1.5, 0, 300).drop), -13.4, 0.1);
    CHECK_NEAR(units::toInches(publishedRow("hornady_65cm_140", "G1", 0, 100, 1.5, 0, 400).drop), -29.6, 0.1);
    // The same bullet, G7 0.315, 1.8 in sight (Long Range Science chart):
    // 2.00, 2.89, 8.84 mil at 400, 500, 1000 yd. Engine: +0.5 to +1.5 %.
    for (const auto& [yd, ref] : std::vector<std::pair<Real, Real>>{{400.0, 2.00}, {500.0, 2.89}, {1000.0, 8.84}}) {
        const RangeRow r = publishedRow("hornady_65cm_140", "G7", 0.315, 100, 1.8, 0, yd);
        CHECK_NEAR(-mil(r, r.drop) / ref, 1.0, 0.02);
    }
    // .308 Win 175 SMK, G7 0.243, 2600 fps (Long Range Science chart): 11.67
    // mil drop and 2.99 mil drift in 10 mph at 1,000 yd. Engine: 12.00 and
    // 3.12 mil, +2.8 % and +4 %: the chart's atmosphere is not stated
    // (documented gap, not tuned).
    const RangeRow far = publishedRow("fgmm_308_175", "G7", 0, 100, 1.5, 10, 1000);
    CHECK_NEAR(-mil(far, far.drop) / 11.67, 1.0, 0.04);
    CHECK_NEAR(-mil(far, far.windage) / 2.99, 1.0, 0.05);
    // .50 BMG 750 A-MAX (Field Ethos): 34 in of drift in a 10 mph crosswind at
    // 1,000 yd from 2815 fps. Engine (2820 fps): 35.3 in.
    CHECK_NEAR(units::toInches(-publishedRow("hornady_50bmg_750", "G1", 0, 100, 1.5, 10, 1000).windage) / 34.0, 1.0, 0.05);
    // 9 mm 124 Gold Dot (Speer 23618GD): 969 fps at 100 yd. Engine with
    // Speer's reloading-data BC 0.134: 959 fps (-1.0 %); with the 0.151 Speer
    // also lists: 974 (+0.5 %).
    CHECK_NEAR(units::toFps(publishedRow("speer_9mm_124gd", "G1", 0, 25, 0.5, 0, 100).speed) / 969.0, 1.0, 0.015);
}
