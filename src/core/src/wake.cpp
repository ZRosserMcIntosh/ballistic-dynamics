// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/wake.hpp"

#include "bd/dmath.hpp"

namespace bd {

Real wakeMomentumThickness(Real dragArea) { return std::sqrt(dragArea > 0 ? dragArea / kTwoPi : 0); }

void farWake(Real theta, Real radius, Real x, const WakeParams& p, Real& halfWidth, Real& deficit) {
    if (!(theta > 0) || !(x > 0)) {
        halfWidth = radius;
        deficit = 0;
        return;
    }
    const Real s = x / theta;
    const Real c = dm::cbrt(s);
    halfWidth = p.widthCoefficient * theta * c;
    if (halfWidth < radius) halfWidth = radius;
    deficit = p.deficitCoefficient / (c * c);
    if (deficit > 1) deficit = 1;
}

std::vector<WakeElement> wakeAt(const PointMassRound& r, Real mass, Real diameter, Real t, const WakeParams& p) {
    std::vector<WakeElement> out;
    const auto& tr = r.trace();
    if (tr.size() < 2 || !(p.spacing > 0)) return out;
    const AirSource& air = r.air();
    Real along = 0, next = 0;
    for (std::size_t i = 1; i < tr.size(); ++i) {
        const FlightSample& a = tr[i - 1];
        const FlightSample& b = tr[i];
        if (a.time > t) break;
        const Real seg = length(b.position - a.position);
        while (next <= along + seg) {
            const Real f = seg > 0 ? (next - along) / seg : 0;
            const Real te = a.time + f * (b.time - a.time);
            const Real age = t - te;
            next += p.spacing;
            if (age < 0 || age > p.maxAge) continue;
            const FlightSample at = hermite(a, b, te);
            const AirSample shed = air.sample(at.position, te);
            const Real U = length(at.velocity - shed.wind);
            // Drag area from the model: |a| = factor rho U^2 = (1/2) rho U^2 Cd A / m.
            const Real dragArea = 2.0 * mass * r.drag().factor(U, shed.speedOfSound);
            WakeElement e;
            farWake(wakeMomentumThickness(dragArea), 0.5 * diameter, U * age, p, e.radius, e.deficit);
            if (e.deficit < p.minDeficit) continue;
            // It drifts with the wind where it is (sampled now, at where it was shed).
            e.position = at.position + air.sample(at.position, t).wind * age;
            e.age = age;
            out.push_back(e);
        }
        along += seg;
    }
    return out;
}

}  // namespace bd
