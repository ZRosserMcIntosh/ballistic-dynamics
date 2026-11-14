// Ballistic Dynamics — bdemo bench: what a round, a shot and a weapon cost on
// this machine (ADR 0006's budget). Wall-clock, best of several runs.
// SPDX-License-Identifier: MIT
#include "bench.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "bd/aviation_air.hpp"
#include "bd/library.hpp"
#include "bd/point_mass.hpp"
#include "bd/shooter.hpp"
#include "bd/spin.hpp"
#include "bd/units.hpp"
#include "bd/world.hpp"

using namespace bd;

namespace {

std::string slurp(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

template <class F>
double bestNs(int reps, F&& f) {
    double best = 1e300;
    for (int r = 0; r < reps; ++r) {
        const auto t0 = std::chrono::steady_clock::now();
        f();
        const auto t1 = std::chrono::steady_clock::now();
        const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        if (ns < best) best = ns;
    }
    return best;
}

volatile double gSink = 0;

// The ground at z = 0 and a backstop across the range at 300 m: what a round
// can hit in the game-second benchmark. A host's raycast costs what it costs;
// this one is a few nanoseconds.
struct Range final : WorldQuery {
    WorldHit segment(const Vec3& a, const Vec3& b, Real) const override {
        WorldHit h;
        if (b.z <= 0 && a.z > 0) h = {true, a.z / (a.z - b.z), {}, {0, 0, 1}, 0, 0, 1};
        if (b.y >= 300 && a.y < 300) {
            const Real f = (300 - a.y) / (b.y - a.y);
            if (!h.hit || f < h.fraction) h = {true, f, {}, {0, -1, 0}, 0, 0, 2};
        }
        if (h.hit) h.point = a + (b - a) * h.fraction;
        return h;
    }
};

}  // namespace

int bench(int, char**) {
    Library lib;
    for (const char* f : {"/drag_standard.json", "/projectiles.json", "/cartridges.json", "/rifles.json", "/stances.json"})
        lib.load(slurp(std::string(BD_DATA_DIR) + f));
    const CartridgeDef& c = *lib.cartridge("federal_xm855");
    const ProjectileDef& p = *lib.projectile(c.projectile);
    std::string err;
    const DragModel drag = dragModelFor(p, lib, "", err);
    const StationAir station(StationReading{101325.0, 288.15, 0.0, 0.0});
    const UniformAir uniform(station.sample({0, 0, 0}, 0));

    std::printf("sizes: PointMassRound %zu B, FlightSample %zu B (per kept step), Weapon %zu B\n",
                sizeof(PointMassRound), sizeof(FlightSample), sizeof(shooter::Weapon));

    const auto flight = [&](const AirSource& air, const PointMassOptions& o, Real range, long& steps) {
        PointMassRound r(drag, air, {{0, 0, 1.5}, {0, 1, 0}, c.muzzleVelocity, {}, 0}, o);
        while (!r.stopped() && r.position().y < range) r.advance(r.time() + 1e-3);
        steps = static_cast<long>(r.flightTime() * 1000 + 0.5);
        gSink = gSink + r.position().z;
    };
    const auto row = [&](const char* name, const AirSource& air, const PointMassOptions& o, Real range) {
        long steps = 0;
        const int n = 200;
        const double ns = bestNs(5, [&] {
            for (int i = 0; i < n; ++i) flight(air, o, range, steps);
        }) / n;
        std::printf("  %-44s %5ld steps  %9.1f us a round  %6.1f ns a step\n", name, steps, ns / 1e3, ns / static_cast<double>(steps));
    };
    std::printf("an M855 round (5.56), level, no trace:\n");
    PointMassOptions plain;
    row("to 25 m, station air (Aviation's model)", station, plain, 25);
    row("to 300 m, station air", station, plain, 300);
    row("to 300 m, uniform air (one sample held)", uniform, plain, 300);
    const ColumnAir column(station, {0, 0, 0}, 0, -100, 400);
    row("to 25 m, column air", column, plain, 25);
    row("to 300 m, column air", column, plain, 300);
    const RangeWind windy(column, {0, 0, 0}, 0, {{0, 4, 1.5 * kPi, 1.0}, {150, 6, 1.4 * kPi, 1.5}});
    row("to 300 m, column air + range wind (gusting)", windy, plain, 300);
    PointMassOptions spun = plain;
    const RifleDef& ar = *lib.rifle("m16a2");
    spun.spin = spinParamsFor(p, ar.rightHandTwist ? ar.twist : -ar.twist, c.muzzleVelocity, station.sample({0, 0, 0}, 0));
    spun.earthRotation = true;
    spun.latitude = -23.55 * kPi / 180;
    row("to 300 m, station air, spin and Earth", station, spun, 300);
    PointMassOptions traced = plain;
    traced.keepTrace = true;
    row("to 300 m, station air, trace kept", station, traced, 300);

    // The air itself.
    {
        const int n = 200000;
        const double ns = bestNs(5, [&] {
            double s = 0;
            for (int i = 0; i < n; ++i) s += station.sample({0, 0.5 * i, 1.5}, 1e-3 * i).density;
            gSink = gSink + s;
        }) / n;
        const double nc = bestNs(5, [&] {
            double s = 0;
            for (int i = 0; i < n; ++i) s += column.sample({0, 0.5 * i, 1.5 + 1e-3 * i}, 1e-3 * i).density;
            gSink = gSink + s;
        }) / n;
        std::printf("one air sample: station air %.1f ns, column air %.1f ns\n", ns, nc);
    }

    // A game second: 20 players firing full auto at 800 rpm for a second,
    // every round flown against the world until it hits the ground or the
    // backstop (aimed between 1 degree down and 1 up), through the air column
    // with a steady range wind, 4 ms chords.
    {
        const Range range;
        const RangeWind steady(column, {0, 0, 0}, 0, {{0, 4, 1.5 * kPi}});
        long rounds = 0, steps = 0;
        const double ns = bestNs(3, [&] {
            rounds = steps = 0;
            for (int player = 0; player < 20; ++player)
                for (int shot = 0; shot < 14; ++shot) {
                    const Real el = (-1.0 + 2.0 * ((player * 14 + shot) % 37) / 36.0) * kPi / 180;
                    const Launch l{{0, 0, 1.5}, {0, std::cos(el), std::sin(el)}, c.muzzleVelocity, {}, shot * 0.075};
                    TrackedRound r(drag, steady, l, {}, {});
                    r.advance(l.time + 3.0, range);
                    ++rounds;
                    steps += static_cast<long>(r.round().flightTime() * 1000 + 0.5);
                }
        });
        std::printf("a game second: %ld rounds, %ld steps, %.2f ms of one core (%.0f ns a step, world queries included)\n",
                    rounds, steps, ns / 1e6, ns / static_cast<double>(steps));
    }

    // The weapon: one second of idle, of full auto at the sight.
    std::string werr;
    const CartridgeDef& mc = *lib.cartridge("mp5_9mm_124_fmj");
    shooter::WeaponSpec w = shooter::weaponSpecFor(*lib.rifle("mp5a3"), mc, *lib.projectile(mc.projectile), werr);
    w.magazine = 1 << 20;
    const StanceDef& st = *lib.stance("standing");
    std::vector<shooter::ShooterEvent> ev;
    ev.reserve(64);
    const auto weaponSecond = [&](bool trigger) {
        const int n = 200;
        return bestNs(5, [&] {
            for (int i = 0; i < n; ++i) {
                shooter::Weapon g(w, st, 1);
                shooter::ShooterInput in;
                in.trigger = trigger;
                in.ads = true;
                in.mode = 1;
                for (int f = 1; f <= 60; ++f) {
                    ev.clear();
                    g.advance(f / 60.0, in, ev);
                }
                gSink = gSink + g.viewPitch();
            }
        }) / n;
    };
    const double idle = weaponSecond(false), firing = weaponSecond(true);
    std::printf("a weapon for one second at 60 Hz: idle %.1f us, full auto (800 rpm) %.1f us\n", idle / 1e3, firing / 1e3);
    return 0;
}
