// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/world.hpp"

namespace bd {

namespace {

FlightSample sampleOf(const PointMassRound& r) {
    return {r.time(), r.position(), r.velocity(), r.spinRate(), r.roll()};
}

}  // namespace

TrackedRound::TrackedRound(const DragModel& drag, const AirSource& air, const Launch& launch,
                           const PointMassOptions& options, const HitOptions& hits)
    : round_(drag, air, launch, options), o_(hits), shotTime_(launch.time) {
    if (o_.querySteps < 1) o_.querySteps = 1;
    if (o_.querySteps > kMaxChord) o_.querySteps = kMaxChord;
    from_ = sampleOf(round_);
    chord_[0] = from_;
    chordDistance_[0] = 0;
    chordLen_ = 1;
}

bool TrackedRound::advance(Real t, const WorldQuery& world, std::int64_t maxSteps) {
    if (done()) return false;
    // Whole steps due before t on the round's clock, one at a time.
    const std::int64_t due = [&] {
        const Real target = (t - shotTime_) / round_.options().step + 1e-9;
        if (!(target >= 0)) return std::int64_t{0};
        std::int64_t k = static_cast<std::int64_t>(target) - steps_;
        if (k < 0) k = 0;
        return k < maxSteps ? k : maxSteps;
    }();
    for (std::int64_t i = 0; i < due; ++i)
        if (stepAndQuery(world)) return true;
    return false;
}

bool TrackedRound::resolveNoLead(const WorldQuery& world) {
    // Bounded: a round that never gets anywhere stops on its own (minSpeed, maxTime).
    while (!done() && fromDistance_ < o_.noLeadRange) {
        const std::int64_t before = steps_;
        if (stepAndQuery(world)) return true;
        if (steps_ == before) break;
    }
    return false;
}

bool TrackedRound::stepAndQuery(const WorldQuery& world) {
    const Vec3 before = round_.position();
    // Exactly one step (its clock counts whole steps; the cap keeps it to one).
    if (round_.advance(round_.time() + round_.options().step, 1) == 0) return false;
    ++steps_;
    const Vec3 d = round_.position() - before;
    distance_ += std::sqrt(dot(d, d));
    chordDistance_[chordLen_] = distance_;
    chord_[chordLen_++] = sampleOf(round_);
    if (steps_ % o_.querySteps == 0 || round_.stopped()) return query(world);
    return false;
}

bool TrackedRound::query(const WorldQuery& world) {
    const FlightSample& to = chord_[chordLen_ - 1];
    // Inside the no-lead range (where the chord starts), bodies are where they
    // were at the shot; beyond it, where they are at the chord's mid-time.
    const bool noLead = fromDistance_ < o_.noLeadRange;
    const Real bodyTime = noLead ? shotTime_ : 0.5 * (from_.time + to.time);
    const WorldHit h = world.segment(from_.position, to.position, bodyTime);
    if (h.hit) {
        const Real f = clamp(h.fraction, 0.0, 1.0);
        // The hit's distance along the path (the chord is the path to well
        // under a micrometre in length), then its step, then its time inside
        // that step: linear in distance over 1 ms is within 0.2 us.
        impact_.distance = fromDistance_ + f * (distance_ - fromDistance_);
        int k = 1;
        while (k < chordLen_ - 1 && chordDistance_[k] < impact_.distance) ++k;
        const Real span = chordDistance_[k] - chordDistance_[k - 1];
        const Real g = span > 0 ? clamp((impact_.distance - chordDistance_[k - 1]) / span, 0.0, 1.0) : 0.0;
        impact_.time = chord_[k - 1].time + g * (chord_[k].time - chord_[k - 1].time);
        impact_.position = h.point;
        impact_.noLead = noLead;
        impact_.hit = h;
        impact_.velocity = hermite(chord_[k - 1], chord_[k], impact_.time).velocity;
        hit_ = true;
        return true;
    }
    from_ = to;
    fromDistance_ = distance_;
    chord_[0] = to;
    chordDistance_[0] = distance_;
    chordLen_ = 1;
    return false;
}

}  // namespace bd
