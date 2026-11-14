// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// The round meets the world (ADR 0006). The host answers one question: what
// does this segment of the path cross first? It owns the geometry (the
// world's surfaces, and the bodies' hitboxes). The engine asks it along the
// flown path and stops the round at the first hit.
//
// Hits on bodies follow the 25 m no-lead rule. While the round is within
// `noLeadRange` of the muzzle (path length), the host is asked to place the
// moving bodies where they were at the shot. Up close you hit what you aimed
// at, as in a hitscan shooter. Beyond that range they are placed at the
// round's own time, so a moving target must be led. The world's static
// surfaces are the same at any time.
//
// Light. The host is asked once per `querySteps` steps (4 ms by default),
// along the chord between those step states. Over 4 ms the flown path sags
// off its chord by g h^2 / 8 = 0.02 mm, far inside any hitbox or surface
// tolerance. The queries are aligned to the round's own step count, not to
// the host's frames, so where a round lands does not depend on the frame
// rate (ADR 0002).
#pragma once

#include <cstdint>

#include "bd/point_mass.hpp"

namespace bd {

struct WorldHit {
    bool hit = false;
    Real fraction = 0;           // along the queried segment, 0..1
    Vec3 point, normal;          // ENU; normal against the round
    std::uint32_t body = 0;      // 0: static world; otherwise the host's body id (a character, a vehicle)
    std::uint32_t part = 0;      // the host's hitbox or surface part (head, torso, door...)
    std::uint32_t material = 0;  // the host's material id
};

class WorldQuery {
public:
    virtual ~WorldQuery() = default;
    // The first thing the segment a -> b crosses, with moving bodies placed
    // at `bodyTime`: the shot's time inside the no-lead range, the segment's
    // mid-time beyond it. Must be deterministic and safe for concurrent const
    // calls if rounds are flown on several threads.
    virtual WorldHit segment(const Vec3& a, const Vec3& b, Real bodyTime) const = 0;
};

struct HitOptions {
    Real noLeadRange = 25.0;  // m of path from the muzzle: bodies at the shot's time (ADR 0006)
    int querySteps = 4;       // steps per world query (chord); 1 for every step
};

struct Impact {
    Real time = 0;          // engine time of the hit (along the chord)
    Vec3 position;          // the hit point the host returned
    Vec3 velocity;          // the round's velocity there (Hermite)
    Real distance = 0;      // m of path flown from the muzzle
    bool noLead = false;    // resolved against the bodies at the shot's time
    WorldHit hit;
};

// A round flown against the world: a PointMassRound that asks the host
// along its path and stops at the first hit.
class TrackedRound {
public:
    TrackedRound(const DragModel& drag, const AirSource& air, const Launch& launch,
                 const PointMassOptions& options = {}, const HitOptions& hits = {});

    // Runs the whole steps due before engine time t, asking `world` along
    // the way. True when the round hit something in this call (impact()).
    bool advance(Real t, const WorldQuery& world, std::int64_t maxSteps = 1 << 20);
    // Flies now, ahead of the host's clock, until the round has gone
    // noLeadRange or hit something. Everything in the no-lead range is
    // resolved against the bodies at the shot's time, so the frame of the
    // click already knows a close hit. Later advance() calls carry on from
    // there (they run no steps until the host's time catches up).
    bool resolveNoLead(const WorldQuery& world);

    bool hit() const { return hit_; }
    const Impact& impact() const { return impact_; }
    bool done() const { return hit_ || round_.stopped(); }
    Real distance() const { return distance_; }
    Real shotTime() const { return shotTime_; }
    const PointMassRound& round() const { return round_; }

private:
    bool stepAndQuery(const WorldQuery& world);
    bool query(const WorldQuery& world);
    PointMassRound round_;
    HitOptions o_;
    Real shotTime_;
    Real distance_ = 0;        // path length flown
    std::int64_t steps_ = 0;
    // The last queried state: the chord starts here.
    FlightSample from_;
    Real fromDistance_ = 0;
    // The states inside the current chord, for the impact's velocity (a ring
    // of querySteps + 1, so no trace is needed).
    static constexpr int kMaxChord = 32;
    FlightSample chord_[kMaxChord + 1];
    Real chordDistance_[kMaxChord + 1] = {};  // path length at each
    int chordLen_ = 0;
    bool hit_ = false;
    Impact impact_;
};

}  // namespace bd
