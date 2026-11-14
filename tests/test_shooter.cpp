// Ballistic Dynamics — the shooter layer (ADR 0005): the weapon's cycle and
// magazine on the 1 ms clock, the recoil from momentum, the view's kick and
// the gun's, the hip cone and the sight.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "bd/aviation_air.hpp"
#include "bd/rng.hpp"
#include "bd/shooter.hpp"
#include "bd/solver.hpp"
#include "bd/units.hpp"
#include "bd/variation.hpp"
#include "bd_test.hpp"

using namespace bd;
using namespace bd::shooter;

namespace {

bool verbose() { return std::getenv("BD_VERBOSE") != nullptr; }

// Each gun with shooter data and the load it is shot with here.
struct Pairing {
    const char* rifle;
    const char* cartridge;
};
constexpr Pairing kPairings[] = {
    {"m16a2", "federal_xm855"},       {"mp5a3", "mp5_9mm_124_fmj"},       {"fn_p90", "fn_57_ss197sr_p90"},
    {"thompson_m1a1", "thompson_45acp_230"}, {"glock_17", "speer_9mm_124gd"}, {"colt_1911", "federal_45acp_230_fmj"},
    {"m24_sws", "fgmm_308_175"},      {"svd_pso1", "ussr_762x54r_7n1"},   {"m107_50bmg", "hornady_50bmg_750"},
};

WeaponSpec specOf(const char* rifle, const char* cartridge) {
    const Library& lib = bdtest::library();
    const RifleDef* r = lib.rifle(rifle);
    const CartridgeDef* c = lib.cartridge(cartridge);
    const ProjectileDef* p = c ? lib.projectile(c->projectile) : nullptr;
    CHECK(r && c && p);
    if (!r || !c || !p) return {};
    std::string err;
    WeaponSpec w = weaponSpecFor(*r, *c, *p, err);
    if (!err.empty()) std::fprintf(stderr, "  %s + %s: %s\n", rifle, cartridge, err.c_str());
    CHECK(err.empty());
    return w;
}

const StanceDef& stance(const char* id) {
    static const StanceDef none{};
    const StanceDef* s = bdtest::library().stance(id);
    CHECK(s != nullptr);
    return s ? *s : none;
}

int count(const std::vector<ShooterEvent>& ev, EventKind k) {
    int n = 0;
    for (const auto& e : ev) n += e.kind == k ? 1 : 0;
    return n;
}

std::vector<ShooterEvent> shotsOf(const std::vector<ShooterEvent>& ev) {
    std::vector<ShooterEvent> out;
    for (const auto& e : ev)
        if (e.kind == EventKind::Shot) out.push_back(e);
    return out;
}

constexpr Real kDeg = kPi / 180.0;

}  // namespace

TEST_CASE("shooter: free recoil is the momentum of the bullet and its powder gas") {
    // .308 Win 175 gr at 2,600 fps, 43 gr of powder at the 4,700 fps
    // convention, from a 15.8 lb M24: worked by hand.
    const Real mb = units::grains(175), v = units::fps(2600), mc = units::grains(43), vg = units::fps(4700);
    const Real m = 15.8 * units::kPound;
    const FreeRecoil r = freeRecoil(mb, v, mc, vg, m);
    CHECK_NEAR(r.momentum, mb * v + mc * vg, 1e-12);
    CHECK_NEAR(r.momentum, 12.98, 0.01);   // N s
    CHECK_NEAR(r.velocity, r.momentum / m, 1e-12);
    CHECK_NEAR(r.energy, 0.5 * r.momentum * r.momentum / m, 1e-9);
    // Every gun with shooter data resolves, and lands where free-recoil
    // energies are known to sit: under one ft lb for the 5.7 mm, a few for a
    // 5.56 carbine and the pistol calibres, around ten for a .308, about a
    // hundred for an unbraked .50 (the M107's muzzle brake is not modelled:
    // docs/physics.md section 7).
    const Real ftlb = units::kFoot * units::kPound * kStandardGravity;
    if (verbose()) std::printf("  %-14s %-22s %8s %8s %8s %9s\n", "rifle", "cartridge", "p N s", "V m/s", "E ft lb", "kick d/s");
    for (const auto& pr : kPairings) {
        const WeaponSpec w = specOf(pr.rifle, pr.cartridge);
        const FreeRecoil f = freeRecoil(w);
        const Real kick = kickRate(w, stance("standing")) / kDeg;
        if (verbose())
            std::printf("  %-14s %-22s %8.2f %8.2f %8.2f %9.1f\n", pr.rifle, pr.cartridge, f.momentum, f.velocity,
                        f.energy / ftlb, kick);
        CHECK_RANGE(f.energy / ftlb, 0.3, 120.0);
        CHECK(kick > 0);
    }
    CHECK_RANGE(freeRecoil(specOf("m16a2", "federal_xm855")).energy / ftlb, 3.0, 4.5);
    CHECK_RANGE(freeRecoil(specOf("m24_sws", "fgmm_308_175")).energy / ftlb, 7.0, 14.0);
    CHECK_RANGE(freeRecoil(specOf("m107_50bmg", "hornady_50bmg_750")).energy / ftlb, 80.0, 120.0);
}

TEST_CASE("shooter: the kick is the angular impulse over the gun's and the hold's inertia") {
    const WeaponSpec ar = specOf("m16a2", "federal_xm855"), bolt = specOf("m24_sws", "fgmm_308_175");
    const StanceDef& standing = stance("standing");
    const StanceDef& prone = stance("prone");
    const Real expected =
        freeRecoil(ar).momentum * ar.boreLever / (ar.gunMass * ar.gyration * ar.gyration + standing.holdInertia);
    CHECK_NEAR(kickRate(ar, standing), expected, 1e-15);
    CHECK(kickRate(bolt, standing) > 2 * kickRate(ar, standing));  // a .308 rises more than a 5.56
    CHECK(kickRate(ar, prone) < kickRate(ar, standing));            // prone holds more of the gun
    // A drop stock (the Thompson's) rises more for its momentum than a straight one.
    const WeaponSpec thompson = specOf("thompson_m1a1", "thompson_45acp_230");
    CHECK(thompson.boreLever > ar.boreLever);
}

TEST_CASE("shooter: the cyclic rate is exact, whatever the frame rate") {
    WeaponSpec w = specOf("thompson_m1a1", "thompson_45acp_230");  // 700 rpm: 85.714 ms, not a whole step
    w.magazine = 1000;
    const StanceDef& s = stance("standing");
    ShooterInput in;
    in.trigger = true;
    in.mode = 1;  // auto
    const Real T = bdtest::kOptimisedBuild ? 6.0 : 2.0;
    const auto run = [&](const std::vector<Real>& frames) {
        Weapon g(w, s, 7, 0);
        std::vector<ShooterEvent> ev;
        for (Real t : frames) g.advance(t, in, ev);
        return ev;
    };
    std::vector<Real> f60, f144, jitter;
    for (int i = 1; i <= static_cast<int>(T * 60 + 0.5); ++i) f60.push_back(i / 60.0);
    for (int i = 1; i <= static_cast<int>(T * 144 + 0.5); ++i) f144.push_back(i / 144.0);
    Rng r(3, 0);
    for (Real t = 0; t < T;) {
        t += r.uniform() < 0.05 ? 0.25 : r.uniform(0.004, 0.03);  // a hitch now and then
        jitter.push_back(t < T ? t : T);
    }
    const auto a = run(f60), b = run(f144), c = run({T}), d = run(jitter);
    const auto sa = shotsOf(a);
    CHECK(static_cast<int>(sa.size()) == static_cast<int>(std::ceil(T * 700 / 60)));  // shots at 0, 85.7, ... < T
    bool same = a.size() == b.size() && a.size() == c.size() && a.size() == d.size();
    for (std::size_t i = 0; same && i < a.size(); ++i)
        same = a[i].step == b[i].step && a[i].step == c[i].step && a[i].step == d[i].step &&
               a[i].direction == b[i].direction && a[i].direction == c[i].direction && a[i].direction == d[i].direction &&
               bitsOf(a[i].kick) == bitsOf(d[i].kick);
    CHECK(same);
    // The remainder is carried: the mean interval is the cyclic rate's, and
    // no interval is off by more than the step.
    const Real mean = static_cast<Real>(sa.back().step - sa.front().step) / static_cast<Real>(sa.size() - 1);
    CHECK_NEAR(mean, 60000.0 / 700.0, 1.0 / static_cast<Real>(sa.size() - 1));
    for (std::size_t i = 1; i < sa.size(); ++i) CHECK_RANGE(static_cast<Real>(sa[i].step - sa[i - 1].step), 85, 86);
}

TEST_CASE("shooter: the shot leaves on the first step of the frame of the click") {
    Weapon g(specOf("m16a2", "federal_xm855"), stance("standing"), 1);
    std::vector<ShooterEvent> ev;
    ShooterInput idle;
    for (int i = 1; i <= 60; ++i) g.advance(i / 60.0, idle, ev);
    CHECK(ev.empty());
    ShooterInput press;
    press.trigger = true;
    g.advance(61 / 60.0, press, ev);  // the frame the click is seen in
    CHECK(count(ev, EventKind::Shot) == 1);
    CHECK(!ev.empty() && ev[0].step == 1000);  // the first step of that frame, t = 1.000 s
    CHECK(g.kickPitch() > 0);                  // and the view has already kicked this frame
    CHECK(g.gunPitch() > 0);
}

TEST_CASE("shooter: semi waits for the release, a burst fires its three, a bolt its cycle") {
    const StanceDef& s = stance("standing");
    ShooterInput in;
    std::vector<ShooterEvent> ev;
    {  // M16A2, semi: a held trigger is one shot.
        Weapon g(specOf("m16a2", "federal_xm855"), s, 1);
        in.trigger = true;
        in.mode = 0;
        g.advance(1.0, in, ev);
        CHECK(count(ev, EventKind::Shot) == 1);
        in.trigger = false;
        g.advance(1.1, in, ev);
        in.trigger = true;
        g.advance(1.2, in, ev);
        CHECK(count(ev, EventKind::Shot) == 2);
    }
    ev.clear();
    {  // Burst: a 16 ms tap fires three at the cyclic rate, then stops.
        Weapon g(specOf("m16a2", "federal_xm855"), s, 1);
        in.mode = 1;
        in.trigger = true;
        g.advance(0.016, in, ev);
        in.trigger = false;
        g.advance(1.0, in, ev);
        const auto shots = shotsOf(ev);
        CHECK(shots.size() == 3);
        if (shots.size() == 3) CHECK(shots[1].step - shots[0].step == 75 && shots[2].step - shots[1].step == 75);
    }
    ev.clear();
    {  // M24: pressing every 100 ms fires only as fast as the bolt is worked.
        Weapon g(specOf("m24_sws", "fgmm_308_175"), s, 1);
        in.mode = 0;
        for (int i = 0; i < 40; ++i) {
            in.trigger = (i % 2) == 0;
            g.advance(0.05 * (i + 1), in, ev);
        }
        const auto shots = shotsOf(ev);
        CHECK(shots.size() == 3);  // at 0, 0.8 and 1.6 s in 2 s
        for (std::size_t i = 1; i < shots.size(); ++i) CHECK(shots[i].step - shots[i - 1].step >= 800);
    }
}

TEST_CASE("shooter: an empty magazine clicks and reloads; a sprint holds the gun down") {
    const WeaponSpec glock = specOf("glock_17", "speer_9mm_124gd");
    Weapon g(glock, stance("standing"), 1);
    std::vector<ShooterEvent> ev;
    ShooterInput in;
    Real t = 0;
    for (int i = 0; i < 2 * 18; ++i) {  // 18 presses
        in.trigger = (i % 2) == 0;
        t += 0.1;
        g.advance(t, in, ev);
    }
    CHECK(count(ev, EventKind::Shot) == 17);
    CHECK(count(ev, EventKind::DryFire) == 1);
    CHECK(count(ev, EventKind::ReloadStart) == 1);
    CHECK(g.reloading() && g.magazine() == 0);
    in.trigger = false;
    g.advance(t + glock.reloadTime + 0.01, in, ev);
    CHECK(count(ev, EventKind::ReloadEnd) == 1);
    CHECK(g.magazine() == 17 && !g.reloading());

    // Sprinting: no shot; after it, none until sprintOut has passed.
    ev.clear();
    Weapon h(glock, stance("standing"), 1);
    in.sprinting = true;
    in.trigger = true;
    h.advance(1.0, in, ev);
    in.sprinting = false;
    h.advance(1.0 + glock.sprintOut - 0.01, in, ev);
    CHECK(count(ev, EventKind::Shot) == 0);
    h.advance(1.0 + glock.sprintOut + 0.01, in, ev);
    CHECK(count(ev, EventKind::Shot) == 1);
}

TEST_CASE("shooter: the view kicks, comes back smoothly and never overshoots") {
    const StanceDef& s = stance("standing");
    Weapon g(specOf("m24_sws", "fgmm_308_175"), s, 5);
    g.setLook({0, 1, 0});
    std::vector<ShooterEvent> ev;
    ShooterInput in;
    in.ads = true;
    g.advance(1.0, in, ev);  // at the sight, settled
    in.trigger = true;
    g.advance(1.001, in, ev);
    in.trigger = false;
    CHECK(count(ev, EventKind::Shot) == 1);
    const Real kick = ev.empty() ? 0 : ev.back().kick;
    Real peak = 0, peakTime = 0, prev = 0;
    bool rising = true, monotone = true, neverBelow = true;
    for (int i = 2; i <= 3000; ++i) {
        g.advance(1.0 + i * 1e-3, in, ev);
        const Real p = g.kickPitch();
        neverBelow = neverBelow && p >= 0;
        if (rising && p < prev) rising = false;
        else if (!rising && p > prev) monotone = false;
        if (p > peak) {
            peak = p;
            peakTime = i * 1e-3;
        }
        prev = p;
    }
    CHECK(neverBelow && monotone);
    CHECK(g.kickPitch() == 0 && g.kickPitchRate() == 0);  // back on the aim and stopped there
    // The rise is the closed form of a constant pull: peak w^2 / 2a at w / a.
    CHECK_NEAR(peak, kick * kick / (2 * s.recenter), 0.02 * peak);
    CHECK_NEAR(peakTime, kick / s.recenter, 0.003);
    if (verbose())
        std::printf("  M24 standing at the sight: kick %.1f deg/s, peak %.2f deg at %.0f ms\n", kick / kDeg, peak / kDeg,
                    peakTime * 1e3);
}

TEST_CASE("shooter: the braced first shot, the hip and the stance scale the kick") {
    const WeaponSpec w = specOf("m16a2", "federal_xm855");
    const StanceDef& s = stance("standing");
    Weapon g(w, s, 9);
    std::vector<ShooterEvent> ev;
    ShooterInput in;
    in.ads = true;
    in.mode = 1;  // burst
    g.advance(1.0, in, ev);
    in.trigger = true;
    g.advance(1.2, in, ev);
    const auto shots = shotsOf(ev);
    CHECK(shots.size() == 3);
    if (shots.size() == 3) {
        const Real base = kickRate(w, s);
        // The first shot meets a settled shooter; the next, 75 ms later, one
        // still re-bracing (75 of 350 ms settled).
        const RecoilDraw d1 = drawRecoil(9, shots[0].stream), d2 = drawRecoil(9, shots[1].stream);
        CHECK_NEAR(shots[0].kick, base * (1 - s.bracedReduction) * (1 + s.kickPitchSd * d1.pitch), 1e-12);
        CHECK_NEAR(shots[1].kick, base * (1 - s.bracedReduction * (0.075 / s.settleTime)) * (1 + s.kickPitchSd * d2.pitch),
                   1e-9);
    }
    // From the hip the same shot kicks hipKick times as hard.
    Weapon hip(w, s, 9);
    std::vector<ShooterEvent> eh;
    ShooterInput h;
    h.trigger = true;
    hip.advance(0.01, h, eh);
    const auto hs = shotsOf(eh);
    CHECK(!hs.empty() && !shots.empty());
    if (!hs.empty() && !shots.empty()) CHECK_NEAR(hs[0].kick / shots[0].kick, s.hipKick, 1e-12);
}

TEST_CASE("shooter: at the sight, firing, turning and moving open no cone; at the hip they do") {
    WeaponSpec w = specOf("mp5a3", "mp5_9mm_124_fmj");
    w.magazine = 1000;
    const StanceDef& s = stance("standing");
    ShooterInput in;
    in.mode = 1;  // auto
    in.turnRate = 30 * kDeg;
    in.moveSpeed = 3.0;
    std::vector<ShooterEvent> ev;
    {
        Weapon g(w, s, 4);
        in.ads = true;
        g.advance(1.0, ShooterInput{false, true}, ev);  // into the sight, settled
        CHECK(g.adsFraction() == 1.0 && g.cone() == 0);
        in.trigger = true;
        g.advance(3.0, in, ev);
        const auto shots = shotsOf(ev);
        CHECK(shots.size() > 20);
        bool laser = true;
        for (const auto& e : shots) laser = laser && e.coneUp == 0 && e.coneRight == 0;
        CHECK(laser);
        CHECK(g.bloom() == 0 && g.cone() == 0);
    }
    ev.clear();
    {
        Weapon g(w, s, 4);
        in.ads = false;
        in.trigger = false;
        g.advance(0.5, in, ev);
        CHECK(g.cone() > s.hipConeMin);  // turning and moving alone open it
        in.trigger = true;
        g.advance(1.5, in, ev);
        CHECK(g.bloom() == 1.0);
        CHECK_NEAR(g.cone(), s.hipConeMax, 1e-15);
        // And standing still with the trigger released, it closes.
        ShooterInput still;
        g.advance(1.5 + 1.0 / s.bloomRecovery + 0.01, still, ev);
        CHECK_NEAR(g.cone(), s.hipConeMin, 1e-12);
    }
}

TEST_CASE("shooter: a shot's draws follow its round's, and the string is the same everywhere") {
    // z1..z4 are bd::drawRound's; the recoil takes the next ones from the same stream.
    Rng r(11, 3);
    for (int i = 0; i < 4; ++i) (void)r.normal();
    const RecoilDraw d = drawRecoil(11, 3);
    CHECK(bitsOf(d.pitch) == bitsOf(r.normal()) && bitsOf(d.yaw) == bitsOf(r.normal()));
    CHECK(bitsOf(d.gunPitch) == bitsOf(r.normal()) && bitsOf(d.gunYaw) == bitsOf(r.normal()));
    CHECK(bitsOf(d.coneRadius) == bitsOf(r.uniform()) && bitsOf(d.coneAngle) == bitsOf(r.uniform()));

    // A scripted string: hip fire while turning, then a burst at the sight,
    // standing then prone. Its hash must be these bits on every platform.
    WeaponSpec w = specOf("mp5a3", "mp5_9mm_124_fmj");
    Weapon g(w, stance("standing"), 2026, 100, 0.5);
    std::vector<ShooterEvent> ev;
    Hash64 h;
    for (int f = 1; f <= 400; ++f) {
        ShooterInput in;
        in.mode = 1;
        in.trigger = (f % 50) < 30;
        in.ads = f > 120;
        in.turnRate = f < 90 ? 0.4 : 0.0;
        in.moveSpeed = f < 60 ? 2.5 : 0.0;
        in.reload = f == 200;
        if (f == 160) g.setStance(stance("prone"));
        Real az = 0.3 + 0.002 * f;
        g.setLook(frames::directionFromAzEl(az, 0.01));
        g.advance(0.5 + f / 60.0, in, ev);
        h.add(g.viewPitch());
        h.add(g.viewYaw());
        h.add(g.gunPitch());
    }
    for (const auto& e : ev) {
        h.add(static_cast<std::uint64_t>(e.kind));
        h.add(static_cast<std::uint64_t>(e.step));
        h.add(e.direction);
        h.add(e.kick);
    }
    char hex[32];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(h.h));
    if (verbose()) {
        std::printf("  shooter golden hash %s, %zu events\n", hex, ev.size());
    }
    CHECK(count(ev, EventKind::ReloadStart) == 1 && count(ev, EventKind::ReloadEnd) == 1);
    CHECK(std::string(hex) == "535c7230c9b51317");
}

TEST_CASE("shooter: the round flies from the shot, and close shots land within frames") {
    // 5.56 M855 from the M16A2, fired level: time of flight to 25, 50, 100 m.
    const Library& lib = bdtest::library();
    const ProjectileDef& p = *lib.projectile("m855_224_62");
    std::string err;
    const DragModel drag = dragModelFor(p, lib, "", err);
    CHECK(err.empty());
    const StationAir air(StationReading{101325.0, 288.15, 0.0, 0.0});
    Weapon g(specOf("m16a2", "federal_xm855"), stance("standing"), 1);
    g.setLook({0, 1, 0});
    std::vector<ShooterEvent> ev;
    ShooterInput in;
    in.ads = true;
    g.advance(1.0, in, ev);
    in.trigger = true;
    g.advance(1.001, in, ev);
    CHECK(ev.size() == 1);
    if (ev.size() != 1) return;
    const ShooterEvent& e = ev[0];
    CHECK_NEAR(length(e.direction), 1.0, 1e-12);
    CHECK(e.direction.y > 0.9999);  // along the look, give or take the hold
    PointMassOptions o;
    o.keepTrace = true;
    PointMassRound round(drag, air, {{0, 0, 1.5}, e.direction, lib.cartridge("federal_xm855")->muzzleVelocity, {}, e.time}, o);
    const Real ranges[] = {25, 50, 100};
    const Real expected[] = {0.027, 0.055, 0.115};  // the times quoted in ADR 0005
    for (int i = 0; i < 3; ++i) {
        while (!round.stopped() && round.position().y < ranges[i]) round.advance(round.time() + 1e-3);
        const Real tof = round.time() - e.time;
        if (verbose()) std::printf("  M855 to %3.0f m: %.1f ms\n", ranges[i], tof * 1e3);
        CHECK_NEAR(tof, expected[i], 0.004);
    }
}

TEST_CASE("shooter: the browser's scripted string (web/smoke_test.mjs) has these bits") {
    // MP5A3 with its 124 gr ball, standing, seed 2026, streams from 100, looking
    // north: full auto from the hip for one second at 60 Hz, at the sight from
    // half a second in, then released. The smoke test drives the same string
    // through the C ABI and must hash the same.
    Weapon g(specOf("mp5a3", "mp5_9mm_124_fmj"), stance("standing"), 2026, 100, 0);
    g.setLook({0, 1, 0});
    std::vector<ShooterEvent> ev;
    Hash64 h;
    int shots = 0;
    for (int f = 1; f <= 120; ++f) {
        ShooterInput in;
        in.trigger = f <= 60;
        in.ads = f >= 30;
        in.mode = 1;
        ev.clear();
        g.advance(f / 60.0, in, ev);
        for (const auto& e : ev) {
            if (e.kind != EventKind::Shot) continue;
            ++shots;
            h.add(e.direction);
            h.add(e.kick);
        }
        h.add(g.viewPitch());
        h.add(g.viewYaw());
    }
    char hex[32];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(h.h));
    if (verbose()) std::printf("  web string: %d shots, hash %s\n", shots, hex);
    CHECK(shots == 14);  // 800 rpm for 1 s: shots at 0, 75, ..., 975 ms
    CHECK(std::string(hex) == "5bcdfaece1f41fd9");
}
