// Ballistic Dynamics — the shooter layer (ADR 0005).
// SPDX-License-Identifier: MIT
#include "bd/shooter.hpp"

#include "bd/clock.hpp"
#include "bd/dmath.hpp"
#include "bd/rng.hpp"

namespace bd::shooter {

namespace {

constexpr std::int64_t kStepUs = 1000;  // the 1 ms step (ADR 0002), in microseconds
constexpr Real kStep = kDefaultStep;
// The hold's wander is its own stream of the weapon's seed, apart from the shots'.
constexpr std::uint64_t kWobbleSalt = 0x5ca1ab1ec0ffee11ull;

ShooterEvent eventOf(EventKind kind, Real time, std::int64_t step) {
    ShooterEvent e;
    e.kind = kind;
    e.time = time;
    e.step = step;
    return e;
}

std::int64_t toSteps(Real seconds) {
    if (!(seconds > 0)) return 0;
    return static_cast<std::int64_t>(seconds / kStep + 0.5);
}

Real signOf(Real v) { return v < 0 ? -1.0 : (v > 0 ? 1.0 : 0.0); }

}  // namespace

FreeRecoil freeRecoil(Real bulletMass, Real muzzleSpeed, Real chargeMass, Real gasSpeed, Real gunMass) {
    FreeRecoil r;
    r.momentum = bulletMass * muzzleSpeed + chargeMass * gasSpeed;
    if (gunMass > 0) {
        r.velocity = r.momentum / gunMass;
        r.energy = 0.5 * gunMass * r.velocity * r.velocity;
    }
    return r;
}

FreeRecoil freeRecoil(const WeaponSpec& w) {
    return freeRecoil(w.bulletMass, w.muzzleSpeed, w.charge, w.gasSpeed, w.gunMass);
}

bool parseFireMode(const std::string& text, ModeDef& out) {
    if (text == "semi") out = {FireMode::Semi, 0};
    else if (text == "auto") out = {FireMode::Auto, 0};
    else if (text == "manual") out = {FireMode::Manual, 0};
    else if (text == "burst2") out = {FireMode::Burst, 2};
    else if (text == "burst3") out = {FireMode::Burst, 3};
    else if (text == "burst4") out = {FireMode::Burst, 4};
    else return false;
    return true;
}

WeaponSpec weaponSpecFor(const RifleDef& rifle, const CartridgeDef& cartridge, const ProjectileDef& projectile,
                         std::string& error) {
    WeaponSpec w;
    error.clear();
    const auto need = [&](bool ok, const char* what) {
        if (!ok && error.empty()) error = std::string(what) + " is missing";
    };
    w.gunMass = rifle.mass;
    w.cyclicRate = rifle.cyclicRate;
    w.manualCycle = rifle.manualCycle;
    w.reloadTime = rifle.reloadTime;
    w.adsTime = rifle.adsTime;
    w.sprintOut = rifle.sprintOut;
    w.boreLever = rifle.boreLever;
    w.gyration = rifle.gyration;
    w.magazine = rifle.magazine;
    for (const auto& m : rifle.fireModes) {
        ModeDef d;
        if (parseFireMode(m, d)) w.modes.push_back(d);
        else if (error.empty()) error = "unknown fire mode \"" + m + "\"";
    }
    w.bulletMass = projectile.mass;
    w.muzzleSpeed = cartridge.muzzleVelocity;
    w.charge = cartridge.charge;
    w.gasSpeed = cartridge.gasSpeed;
    need(w.gunMass > 0, "rifle mass_lb");
    need(!w.modes.empty(), "rifle fire_modes");
    need(w.magazine > 0, "rifle magazine_rounds");
    need(w.adsTime > 0, "rifle ads_s");
    need(w.gyration > 0, "rifle gyration_in");
    need(w.charge > 0, "cartridge charge_gr");
    need(w.bulletMass > 0 && w.muzzleSpeed > 0, "the bullet's mass and speed");
    for (const auto& m : w.modes) {
        if (m.mode == FireMode::Manual) need(w.manualCycle > 0, "rifle manual_cycle_s");
        else need(w.cyclicRate > 0, "rifle cyclic_rate_rpm");
    }
    return w;
}

Real kickRate(const WeaponSpec& w, const StanceDef& s) {
    const Real inertia = w.gunMass * w.gyration * w.gyration + s.holdInertia;
    if (!(inertia > 0)) return 0;
    return freeRecoil(w).momentum * w.boreLever / inertia;
}

RecoilDraw drawRecoil(std::uint64_t seed, std::uint64_t stream) {
    Rng rng(seed, stream);
    for (int i = 0; i < 4; ++i) (void)rng.normal();  // z1..z4: bd::drawRound's (an even count: no spare left)
    RecoilDraw d;
    d.pitch = rng.normal();
    d.yaw = rng.normal();
    d.gunPitch = rng.normal();
    d.gunYaw = rng.normal();
    d.coneRadius = rng.uniform();
    d.coneAngle = rng.uniform();
    return d;
}

Weapon::Weapon(const WeaponSpec& spec, const StanceDef& stance, std::uint64_t seed, std::uint64_t streamBase,
               Real startTime)
    : spec_(spec), stance_(stance), seed_(seed), streamBase_(streamBase), startTime_(startTime) {
    magazine_ = spec_.magazine;
    Rng rng(seed ^ kWobbleSalt, streamBase);
    for (auto& p : wobblePhase_) p = kTwoPi * rng.uniform();
}

Real Weapon::time() const { return startTime_ + static_cast<Real>(step_) * kStep; }

std::int64_t Weapon::advance(Real t, const ShooterInput& in, std::vector<ShooterEvent>& events,
                             std::int64_t maxSteps) {
    // Whole steps due before t on the weapon's own clock (as FixedClock::due).
    const Real target = (t - startTime_) / kStep + 1e-9;
    if (!(target >= 0)) return 0;
    std::int64_t k = static_cast<std::int64_t>(target) - step_;
    if (k < 0) k = 0;
    if (k > maxSteps) k = maxSteps;
    for (std::int64_t i = 0; i < k; ++i) stepOnce(in, events);
    return k;
}

void Weapon::stepOnce(const ShooterInput& in, std::vector<ShooterEvent>& events) {
    const Real dt = kStep;
    const std::int64_t nowUs = step_ * kStepUs;

    // The trigger re-arms when released (semi, manual, and between bursts).
    if (!in.trigger) armed_ = true;

    // Sprinting lowers the gun; coming out of it takes sprintOut.
    if (in.sprinting) sprintBlock_ = toSteps(spec_.sprintOut);
    else if (sprintBlock_ > 0) --sprintBlock_;

    // Reloading.
    if (reloadLeft_ > 0 && --reloadLeft_ == 0) {
        magazine_ = spec_.magazine;
        events.push_back(eventOf(EventKind::ReloadEnd, time(), step_));
    }
    const auto startReload = [&] {
        if (reloadLeft_ > 0 || magazine_ >= spec_.magazine) return;
        reloadLeft_ = toSteps(spec_.reloadTime);
        if (reloadLeft_ <= 0) reloadLeft_ = 1;
        burstLeft_ = 0;
        events.push_back(eventOf(EventKind::ReloadStart, time(), step_));
    };
    if (in.reload && burstLeft_ == 0) startReload();

    // Toward the sight, or away from it.
    const bool wantAds = in.ads && !in.sprinting && reloadLeft_ == 0;
    const Real adsStep = spec_.adsTime > 0 ? dt / spec_.adsTime : 1.0;
    if (wantAds) ads_ = ads_ + adsStep >= 1.0 ? 1.0 : ads_ + adsStep;
    else ads_ = ads_ - adsStep <= 0.0 ? 0.0 : ads_ - adsStep;

    // The trigger.
    const int modeIndex = spec_.modes.empty() ? -1
                          : (in.mode >= 0 && in.mode < static_cast<int>(spec_.modes.size()) ? in.mode : 0);
    if (modeIndex >= 0) {
        const ModeDef& mode = spec_.modes[static_cast<std::size_t>(modeIndex)];
        const bool canFire = reloadLeft_ == 0 && sprintBlock_ == 0 && !in.sprinting && nowUs >= nextShotUs_;
        bool want = false;
        switch (mode.mode) {
            case FireMode::Auto: want = in.trigger; break;
            case FireMode::Semi:
            case FireMode::Manual: want = in.trigger && armed_; break;
            case FireMode::Burst: want = burstLeft_ > 0 || (in.trigger && armed_); break;
        }
        if (want && canFire) {
            if (magazine_ <= 0) {
                burstLeft_ = 0;
                if (armed_ || mode.mode == FireMode::Auto) {
                    events.push_back(eventOf(EventKind::DryFire, time(), step_));
                    armed_ = false;
                    startReload();
                }
            } else {
                fire(mode, events);
            }
        }
    }

    // The hip cone: firing, turning, moving and leaving the ground open it;
    // it closes when nothing does. At the sight nothing opens it.
    if (ads_ < 1.0) {
        const Real open = (stance_.bloomPerTurn * std::abs(in.turnRate) + stance_.bloomPerMetre * in.moveSpeed +
                           (in.airborne ? stance_.bloomAirborne : 0.0)) * dt;
        bloom_ = open > 0 ? bloom_ + open : bloom_ - stance_.bloomRecovery * dt;
    } else {
        bloom_ -= stance_.bloomRecovery * dt;
    }
    bloom_ = clamp(bloom_, 0.0, 1.0);

    // The shooter settles, the view comes back, the gun settles in the hands.
    settled_ = stance_.settleTime > 0 ? (settled_ + dt / stance_.settleTime >= 1.0 ? 1.0 : settled_ + dt / stance_.settleTime)
                                      : 1.0;
    for (auto& k : kick_) stepKick(k, dt);
    for (auto& g : gun_) stepGun(g, dt);
    ++step_;
}

void Weapon::fire(const ModeDef& mode, std::vector<ShooterEvent>& events) {
    const std::int64_t nowUs = step_ * kStepUs;
    ++shots_;
    --magazine_;
    const std::uint64_t stream = streamBase_ + static_cast<std::uint64_t>(shots_);
    const RecoilDraw d = drawRecoil(seed_, stream);

    // The bore now: the look, turned by the kick so far and the hold, then
    // the hip cone. This round's own recoil comes after it has left.
    Vec3 dir = offsetDirection(viewPitch(), viewYaw());
    const Real r = cone() * d.coneRadius;
    Real sa, ca;
    dm::sincos(kTwoPi * d.coneAngle, sa, ca);
    const Real up = r * ca, right = r * sa;
    if (r > 0) {
        Vec3 rt = cross(dir, Vec3{0, 0, 1});
        rt = dot(rt, rt) > 1e-18 ? normalized(rt) : Vec3{1, 0, 0};
        const Vec3 u = cross(rt, dir);
        dir = normalized(dir + u * up + rt * right);
    }

    // The recoil: the momentum's angular impulse, met by this stance's hold.
    const Real hip = stance_.hipKick + (1.0 - stance_.hipKick) * ads_;
    const Real braced = 1.0 - stance_.bracedReduction * settled_;
    const Real base = kickRate(spec_, stance_) * hip * braced;
    const Real pitchFactor = 1.0 + stance_.kickPitchSd * d.pitch;
    const Real kick = base * (pitchFactor > 0 ? pitchFactor : 0.0);
    kick_[0].rate += kick;
    kick_[1].rate += base * stance_.kickYawSd * d.yaw;
    const Real gunPitchFactor = 1.0 + stance_.kickPitchSd * d.gunPitch;
    gun_[0].speed += base * stance_.gunKick * (gunPitchFactor > 0 ? gunPitchFactor : 0.0);
    gun_[1].speed += base * stance_.gunKick * stance_.kickYawSd * d.gunYaw;
    settled_ = 0;
    if (ads_ < 1.0) bloom_ = clamp(bloom_ + stance_.bloomPerShot, 0.0, 1.0);

    // The action's next readiness, carrying the remainder so the rate is exact
    // (a trigger held through the cycle fires on the first step at or after it).
    const std::int64_t cycleUs =
        mode.mode == FireMode::Manual
            ? static_cast<std::int64_t>(spec_.manualCycle * 1e6 + 0.5)
            : (spec_.cyclicRate > 0 ? static_cast<std::int64_t>(1e6 / spec_.cyclicRate + 0.5) : kStepUs);
    const std::int64_t base0 = nowUs - nextShotUs_ < kStepUs ? nextShotUs_ : nowUs;
    nextShotUs_ = base0 + (cycleUs > 0 ? cycleUs : kStepUs);

    if (mode.mode == FireMode::Burst) {
        if (burstLeft_ == 0) burstLeft_ = mode.burst;
        --burstLeft_;
    }
    if (mode.mode != FireMode::Auto) armed_ = false;

    ShooterEvent e;
    e.kind = EventKind::Shot;
    e.time = time();
    e.step = step_;
    e.shot = shots_;
    e.seed = seed_;
    e.stream = stream;
    e.direction = dir;
    e.coneUp = up;
    e.coneRight = right;
    e.kick = kick;
    e.magazine = magazine_;
    events.push_back(e);
}

void Weapon::stepKick(KickAxis& k, Real dt) const {
    if (k.angle == 0 && k.rate == 0) return;
    // A constant pull back toward the aim; motion back toward it is slowed to
    // returnFraction, so the kick is fast and the return smooth.
    if (k.angle != 0) k.rate -= signOf(k.angle) * stance_.recenter * dt;
    Real change = k.rate * dt;
    if (k.angle * change < 0) change *= stance_.returnFraction;
    const Real next = k.angle + change;
    if (next * k.angle < 0) {  // through the aim: stop there, no overshoot
        k.angle = 0;
        k.rate = 0;
        return;
    }
    k.angle = next;
    if (k.angle == 0) {
        k.rate = 0;
    } else if (std::abs(k.angle) > stance_.kickClamp) {
        k.angle = signOf(k.angle) * stance_.kickClamp;
        k.rate = 0;
    }
}

void Weapon::stepGun(GunAxis& g, Real dt) const {
    if (g.offset == 0 && g.speed == 0) return;
    g.offset += g.speed * dt;
    if (std::abs(g.offset) > stance_.gunKickMax) {
        g.offset = signOf(g.offset) * stance_.gunKickMax;
        if (g.speed * g.offset > 0) g.speed = 0;
    }
    g.speed -= signOf(g.offset) * stance_.gunKickAccel * dt;
    g.speed -= g.speed * stance_.gunKickDecay * dt;
    // At rest within a micro-radian and a milliradian a second.
    if (std::abs(g.offset) < 1e-6 && std::abs(g.speed) < 1e-3) {
        g.offset = 0;
        g.speed = 0;
    }
}

Real Weapon::wobble(int axis) const {
    if (!(stance_.wobblePeriod > 0) || stance_.holdWobble == 0) return 0;
    const Real w = kTwoPi * (static_cast<Real>(step_) * kStep) / stance_.wobblePeriod;
    // Two incommensurate sines per axis: a breath and a quicker tremor.
    if (axis == 0)
        return stance_.holdWobble * (0.8 * dm::sin(w + wobblePhase_[0]) + 0.2 * dm::sin(2.7 * w + wobblePhase_[1]));
    return 0.6 * stance_.holdWobble * (0.8 * dm::sin(0.71 * w + wobblePhase_[2]) + 0.2 * dm::sin(2.3 * w + wobblePhase_[3]));
}

Real Weapon::viewPitch() const { return kick_[0].angle + ads_ * wobble(0); }
Real Weapon::viewYaw() const { return kick_[1].angle + ads_ * wobble(1); }

Real Weapon::cone() const {
    // At the sight the cone is only what is left of the bloom, which drains:
    // a laser once settled. Anywhere short of it, the hip cone.
    if (ads_ >= 1.0) return stance_.hipConeMax * bloom_;
    return stance_.hipConeMin + (stance_.hipConeMax - stance_.hipConeMin) * bloom_;
}

Vec3 Weapon::offsetDirection(Real pitch, Real yaw) const {
    Vec3 f = normalized(look_);
    if (dot(f, f) == 0) f = {0, 1, 0};
    Vec3 right = cross(f, Vec3{0, 0, 1});
    right = dot(right, right) > 1e-18 ? normalized(right) : Vec3{1, 0, 0};
    const Vec3 up = cross(right, f);
    Real sp, cp, sy, cy;
    dm::sincos(pitch, sp, cp);
    dm::sincos(yaw, sy, cy);
    return normalized(f * (cp * cy) + up * sp + right * (cp * sy));
}

Vec3 Weapon::boreDirection() const { return offsetDirection(viewPitch(), viewYaw()); }

}  // namespace bd::shooter
