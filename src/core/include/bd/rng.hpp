// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Seeded random numbers (ADR 0004). xoshiro256** (Blackman & Vigna 2018)
// seeded through SplitMix64, with normal deviates by Box-Muller on the
// deterministic bd::dm functions, so the same seed gives the same draws bit for
// bit on every platform and compiler (std::normal_distribution does not: its
// algorithm is implementation-defined).
//
// Streams. Every round draws from its own stream, keyed by (seed, stream id),
// so a round's muzzle velocity does not depend on how many rounds were fired
// before it or in what order the host created them.
#pragma once

#include <cstdint>

#include "bd/dmath.hpp"

namespace bd {

inline std::uint64_t splitMix64(std::uint64_t& s) {
    std::uint64_t z = (s += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

class Rng {
public:
    explicit Rng(std::uint64_t seed = 1, std::uint64_t stream = 0) {
        std::uint64_t s = seed ^ (stream * 0xd1342543de82ef95ull + 0x2545f4914f6cdd1dull);
        for (auto& w : s_) w = splitMix64(s);
    }
    std::uint64_t next() {
        const std::uint64_t r = rotl(s_[1] * 5, 7) * 9;
        const std::uint64_t t = s_[1] << 17;
        s_[2] ^= s_[0];
        s_[3] ^= s_[1];
        s_[1] ^= s_[2];
        s_[0] ^= s_[3];
        s_[2] ^= t;
        s_[3] = rotl(s_[3], 45);
        return r;
    }
    // Uniform in [0, 1): the top 53 bits.
    Real uniform() { return static_cast<Real>(next() >> 11) * 0x1.0p-53; }
    Real uniform(Real lo, Real hi) { return lo + (hi - lo) * uniform(); }
    // Standard normal (Box-Muller; the second deviate is kept for the next call).
    Real normal() {
        if (haveSpare_) {
            haveSpare_ = false;
            return spare_;
        }
        Real u = uniform();
        while (u <= 0) u = uniform();
        const Real v = uniform();
        const Real r = std::sqrt(-2.0 * dm::log(u));
        Real s, c;
        dm::sincos(kTwoPi * v, s, c);
        spare_ = r * s;
        haveSpare_ = true;
        return r * c;
    }
    Real normal(Real mean, Real sigma) { return mean + sigma * normal(); }

private:
    static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
    std::uint64_t s_[4]{};
    Real spare_ = 0;
    bool haveSpare_ = false;
};

}  // namespace bd
