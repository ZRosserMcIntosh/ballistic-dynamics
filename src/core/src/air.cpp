// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/air.hpp"

#include <algorithm>

#include "bd/dmath.hpp"
#include "bd/rng.hpp"

namespace bd {

Vec3 windFrom(Real speed, Real fromBearing) {
    Real s, c;
    dm::sincos(fromBearing, s, c);
    return {-speed * s, -speed * c, 0};
}

void windToFrom(const Vec3& wind, Real& speed, Real& fromBearing) {
    speed = std::sqrt(wind.x * wind.x + wind.y * wind.y);
    Real b = speed > 0 ? dm::atan2(-wind.x, -wind.y) : 0.0;
    if (b < 0) b += kTwoPi;
    fromBearing = b;
}

RangeWind::RangeWind(const AirSource& base, const Vec3& origin, Real azimuth, std::vector<WindLayer> layers, Mode mode,
                     Real blend, std::uint64_t seed)
    : base_(base), origin_(origin), layers_(std::move(layers)), mode_(mode), blend_(blend > 0 ? blend : 0) {
    Real s, c;
    dm::sincos(azimuth, s, c);
    along_ = {s, c, 0};
    std::stable_sort(layers_.begin(), layers_.end(),
                     [](const WindLayer& a, const WindLayer& b) { return a.fromRange < b.fromRange; });
    for (std::size_t i = 0; i < layers_.size(); ++i) {
        Rng r(seed, 0x5749u + i);  // one stream per layer: adding a layer leaves the others alone
        Phases ph;
        for (Real& p : ph.p) p = kTwoPi * r.uniform();
        // Three incommensurate periods around the nominal one: 0.6, 1, 1.7 x.
        ph.f[0] = 1.0 / 0.6;
        ph.f[1] = 1.0;
        ph.f[2] = 1.0 / 1.7;
        phases_.push_back(ph);
    }
}

Real RangeWind::rangeOf(const Vec3& p) const {
    const Vec3 d = p - origin_;
    return d.x * along_.x + d.y * along_.y;
}

Vec3 RangeWind::layerWind(std::size_t i, Real time) const {
    const WindLayer& L = layers_[i];
    Real speed = L.speed, bearing = L.fromBearing;
    if (L.gust > 0 || L.veer > 0) {
        const Phases& ph = phases_[i];
        const Real w = kTwoPi / (L.gustPeriod > 0 ? L.gustPeriod : 8.0);
        // Sum of three unit-variance-normalised sines: RMS of each sine is
        // 1/sqrt2, three of them sqrt(3/2); scaled back to RMS 1.
        constexpr Real kNorm = 0.81649658092772603;  // sqrt(2/3)
        Real g = 0, v = 0;
        for (int k = 0; k < 3; ++k) {
            g += dm::sin(w * ph.f[k] * time + ph.p[k]);
            v += dm::sin(w * ph.f[k] * time + ph.p[3 + k]);
        }
        speed = std::max(0.0, speed + L.gust * kNorm * g);
        bearing += L.veer * kNorm * v;
    }
    return windFrom(speed, bearing);
}

Vec3 RangeWind::windAt(Real range, Real time) const {
    if (layers_.empty()) return {};
    std::size_t i = 0;
    while (i + 1 < layers_.size() && range >= layers_[i + 1].fromRange) ++i;
    Vec3 w = layerWind(i, time);
    // Blend into the next layer over the last `blend` metres before it.
    if (blend_ > 0 && i + 1 < layers_.size()) {
        const Real toNext = layers_[i + 1].fromRange - range;
        if (toNext < 0.5 * blend_) {
            const Real t = 0.5 - toNext / blend_;  // 0 at blend/2 before, 0.5 at the boundary
            const Real s = t * t * (3.0 - 2.0 * t);
            w = w * (1.0 - s) + layerWind(i + 1, time) * s;
        }
    }
    if (blend_ > 0 && i > 0) {
        const Real past = range - layers_[i].fromRange;
        if (past < 0.5 * blend_) {
            const Real t = 0.5 + past / blend_;  // 0.5 at the boundary, 1 at blend/2 after
            const Real s = t * t * (3.0 - 2.0 * t);
            w = layerWind(i - 1, time) * (1.0 - s) + w * s;
        }
    }
    return w;
}

AirSample RangeWind::sample(const Vec3& p, Real time) const {
    AirSample a = base_.sample(p, time);
    const Vec3 w = windAt(rangeOf(p), time);
    if (mode_ == Mode::Replace)
        a.wind = w;
    else
        a.wind += w;
    return a;
}

}  // namespace bd
