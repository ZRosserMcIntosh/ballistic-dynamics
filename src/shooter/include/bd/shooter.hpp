// Ballistic Dynamics — the shooter layer (ADR 0005).
// SPDX-License-Identifier: MIT
//
// What happens between the trigger and the launch, and what the shooter
// feels afterwards: the weapon's cycle, magazine and reload; the recoil, sized
// by the momentum the round and its powder gas carry away; the view's kick and
// the gun's own motion in the hands; the hip-fire cone and the hold at the
// sight. It turns input into shot events (the bore's direction and the round's
// seeded stream) that the host flies with bd_core, and into the camera and
// viewmodel offsets the host draws.
//
// It runs on the same fixed 1 ms clock as the rounds (ADR 0002), counted as
// integers, so a weapon fires at its exact cyclic rate at any frame rate, and
// two hosts that feed it the same input at the same times get the same shots
// bit for bit (ADR 0004). A press reaches the trigger on the first step of
// the frame it is given in, so the shot, its kick and its sound belong to the
// frame of the click, never to a later one.
//
// Physics and play. The recoil impulse is physics (conservation of momentum).
// The shooter's response to it, the hip cone and the hold are a model of a
// person, tuned for play and tagged `estimated:` in data/stances.json; the
// shape of the response (a fast kick, a constant pull back, a slower return
// that stops at the aim and never overshoots) follows the IW engine's weapon
// model, which is the reference for crisp gunplay (ADR 0005).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "bd/library.hpp"
#include "bd/math.hpp"

namespace bd::shooter {

// ---- Free recoil (physics) ----

struct FreeRecoil {
    Real momentum = 0;  // N s carried off by the bullet and the powder gas
    Real velocity = 0;  // m/s of the free gun
    Real energy = 0;    // J of the free gun
};

// p = m_bullet v0 + m_charge v_gas; the gun takes -p.
FreeRecoil freeRecoil(Real bulletMass, Real muzzleSpeed, Real chargeMass, Real gasSpeed, Real gunMass);

// ---- The weapon ----

enum class FireMode { Semi, Auto, Burst, Manual };

struct ModeDef {
    FireMode mode = FireMode::Semi;
    int burst = 0;  // rounds per press, for Burst
};
// "semi", "auto", "burst2".."burst4", "manual".
bool parseFireMode(const std::string& text, ModeDef& out);

// A gun with its load, resolved to SI.
struct WeaponSpec {
    Real gunMass = 0;        // kg
    Real cyclicRate = 0;     // rounds/s
    Real manualCycle = 0;    // s
    Real reloadTime = 0;     // s
    Real adsTime = 0;        // s
    Real sprintOut = 0;      // s
    Real boreLever = 0;      // m
    Real gyration = 0;       // m
    int magazine = 0;
    std::vector<ModeDef> modes;
    Real bulletMass = 0;     // kg
    Real muzzleSpeed = 0;    // m/s
    Real charge = 0;         // kg
    Real gasSpeed = 0;       // m/s
};

// From the library. On a missing figure, error names it and the spec is not usable.
WeaponSpec weaponSpecFor(const RifleDef& rifle, const CartridgeDef& cartridge, const ProjectileDef& projectile,
                         std::string& error);
FreeRecoil freeRecoil(const WeaponSpec& w);

// The pitch rate (rad/s) one shot gives the view, before the stance's hip,
// braced and random factors: the angular impulse p h about the gun's centre
// of mass over the gun's own pitch inertia plus the hold's.
Real kickRate(const WeaponSpec& w, const StanceDef& s);

// A shot's own draws from its round's stream (seed, stream), after the four
// that bd::drawRound takes (velocity, BC, throw up, throw right): z5..z8
// normal, u9, u10 uniform. So the round and its recoil are one stream.
struct RecoilDraw {
    Real pitch = 0, yaw = 0;        // z5, z6: the view kick's spread
    Real gunPitch = 0, gunYaw = 0;  // z7, z8: the gun's
    Real coneRadius = 0;            // u9 in [0, 1): where in the hip cone
    Real coneAngle = 0;             // u10 in [0, 1): around it
};
RecoilDraw drawRecoil(std::uint64_t seed, std::uint64_t stream);

// What the shooter does this frame (held for every step of an advance call).
struct ShooterInput {
    bool trigger = false;
    bool ads = false;        // aiming down the sight (wanted; it takes adsTime to get there)
    bool sprinting = false;
    bool airborne = false;
    bool reload = false;     // asks for a reload when the magazine is not full
    Real turnRate = 0;       // rad/s the view is turning
    Real moveSpeed = 0;      // m/s over the ground
    int mode = 0;            // index into the weapon's fire modes
};

enum class EventKind { Shot, DryFire, ReloadStart, ReloadEnd };

struct ShooterEvent {
    EventKind kind = EventKind::Shot;
    Real time = 0;            // engine time of the step it happened on
    std::int64_t step = 0;    // that step's index on the weapon's clock
    // Shots:
    int shot = 0;             // 1, 2, ... since the weapon was made
    std::uint64_t seed = 0, stream = 0;  // the round's stream (bd::drawRound, bd_fire_round)
    Vec3 direction;           // the bore at the shot, ENU (unit)
    Real coneUp = 0, coneRight = 0;  // rad: the hip cone's part of it
    Real kick = 0;            // rad/s: the pitch rate this shot gave the view
    int magazine = 0;         // rounds left after it
};

class Weapon {
public:
    // Shots draw from streams streamBase + 1, + 2, ... of seed.
    Weapon(const WeaponSpec& spec, const StanceDef& stance, std::uint64_t seed, std::uint64_t streamBase = 0,
           Real startTime = 0);

    void setStance(const StanceDef& s) { stance_ = s; }
    // Where the shooter is looking (ENU, any length): the camera before the
    // offsets below. The bore is this turned by the kick and the hold.
    void setLook(const Vec3& look) { look_ = look; }

    // Runs the whole 1 ms steps due before engine time t with this input,
    // appending what happened. Returns the steps run.
    std::int64_t advance(Real t, const ShooterInput& in, std::vector<ShooterEvent>& events,
                         std::int64_t maxSteps = 1 << 20);

    // Add to the camera (rad, + up, + right): the kick and the hold's wander.
    Real viewPitch() const;
    Real viewYaw() const;
    // The kick alone, and its rates.
    Real kickPitch() const { return kick_[0].angle; }
    Real kickYaw() const { return kick_[1].angle; }
    Real kickPitchRate() const { return kick_[0].rate; }
    // The gun's motion in the hands, for the viewmodel only (rad).
    Real gunPitch() const { return gun_[0].offset; }
    Real gunYaw() const { return gun_[1].offset; }
    // 0 at the hip, 1 at the sight.
    Real adsFraction() const { return ads_; }
    // The hip cone's half-angle now (rad), and how far it is opened (0..1).
    Real cone() const;
    Real bloom() const { return bloom_; }
    // Where a shot would go now, without the hip cone.
    Vec3 boreDirection() const;

    int magazine() const { return magazine_; }
    bool reloading() const { return reloadLeft_ > 0; }
    int shotsFired() const { return shots_; }
    Real time() const;
    std::int64_t steps() const { return step_; }
    const WeaponSpec& spec() const { return spec_; }
    const StanceDef& stance() const { return stance_; }

private:
    struct KickAxis {
        Real angle = 0, rate = 0;
    };
    struct GunAxis {
        Real offset = 0, speed = 0;
    };
    void stepOnce(const ShooterInput& in, std::vector<ShooterEvent>& events);
    void fire(const ModeDef& mode, std::vector<ShooterEvent>& events);
    void stepKick(KickAxis& k, Real dt) const;
    void stepGun(GunAxis& g, Real dt) const;
    Real wobble(int axis) const;
    Vec3 offsetDirection(Real pitch, Real yaw) const;

    WeaponSpec spec_;
    StanceDef stance_;
    std::uint64_t seed_, streamBase_;
    Real startTime_;
    Real wobblePhase_[4] = {};
    Vec3 look_{0, 1, 0};

    std::int64_t step_ = 0;
    std::int64_t nextShotUs_ = 0;     // microseconds on the weapon's clock: the action is ready
    std::int64_t reloadLeft_ = 0;     // ms
    std::int64_t sprintBlock_ = 0;    // ms
    int magazine_ = 0;
    int shots_ = 0;
    int burstLeft_ = 0;
    bool armed_ = true;               // the trigger has been released since the last shot
    Real ads_ = 0;
    Real bloom_ = 0;
    Real settled_ = 1;                // 1: braced and settled; 0 right after a shot
    KickAxis kick_[2];                // pitch, yaw
    GunAxis gun_[2];
};

}  // namespace bd::shooter
