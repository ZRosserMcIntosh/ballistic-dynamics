// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/variation.hpp"

#include "bd/frames.hpp"
#include "bd/rng.hpp"
#include "bd/units.hpp"

namespace bd {

VariationModel variationFor(const CartridgeDef& c, const ProjectileDef& p, const RifleDef* rifle) {
    VariationModel m;
    m.velocitySd = c.velocitySd;
    m.bcSd = p.bcSd;
    m.tempCoefficient = c.tempCoefficient;
    m.tempReference = c.tempReference;
    if (rifle && rifle->groupMoa > 0) m.angleSd = units::moa(rifle->groupMoa) / kFiveShotSpreadPerSigma;
    return m;
}

Real muzzleSpeedAt(const CartridgeDef& c, const VariationModel& m, Real powderTemperature) {
    if (!(powderTemperature > 0) || !(m.tempReference > 0)) return c.muzzleVelocity;
    return c.muzzleVelocity + m.tempCoefficient * (powderTemperature - m.tempReference);
}

RoundDraw drawRound(const VariationModel& m, std::uint64_t seed, std::uint64_t stream) {
    // Four normals in a fixed order, whatever is switched on, so turning one
    // cause off does not reshuffle the others.
    Rng rng(seed, stream);
    const Real z1 = rng.normal(), z2 = rng.normal(), z3 = rng.normal(), z4 = rng.normal();
    RoundDraw d;
    d.velocityOffset = m.velocitySd * z1;
    d.bcFactor = 1.0 + m.bcSd * z2;
    if (d.bcFactor < 0.5) d.bcFactor = 0.5;  // a 50-sigma tail; never reached in practice
    d.up = m.angleSd * z3;
    d.right = m.angleSd * z4;
    return d;
}

Vec3 thrownDirection(const Vec3& bore, const RoundDraw& d) {
    const Vec3 f = normalized(bore);
    Vec3 right = cross(f, Vec3{0, 0, 1});
    right = dot(right, right) > 1e-18 ? normalized(right) : Vec3{1, 0, 0};
    const Vec3 up = cross(right, f);
    // Small angles: tan is the angle to well under a part in a million at a few MOA.
    return normalized(f + up * d.up + right * d.right);
}

Spread spreadAt(const RangeSetup& s, const AirSource& air, Real elevation, Real range, const VariationModel& m) {
    Spread out;
    const auto height = [&](const RangeSetup& rs) {
        const auto rows = rangeTable(rs, air, elevation, {range});
        return rows.empty() || !rows[0].reached ? 0.0 : rows[0].drop;
    };
    const Real h0 = height(s);
    if (m.velocitySd > 0) {
        RangeSetup v = s;
        const Real dv = 1.0;  // m/s: linear enough for spreads of a few m/s
        v.muzzleSpeed += dv;
        out.fromVelocity = std::abs((height(v) - h0) / dv) * m.velocitySd;
    }
    if (m.bcSd > 0) {
        RangeSetup b = s;
        const Real k = 0.01;
        b.drag = s.drag.scaledBc(1.0 + k);
        out.fromBc = std::abs((height(b) - h0) / k) * m.bcSd;
    }
    out.fromBarrel = m.angleSd * range;
    out.vertical = std::sqrt(sq(out.fromVelocity) + sq(out.fromBc) + sq(out.fromBarrel));
    out.horizontal = out.fromBarrel;
    return out;
}

}  // namespace bd
