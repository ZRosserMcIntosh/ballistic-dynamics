// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/drag.hpp"

namespace bd {

DragTable::DragTable(std::vector<Real> mach, std::vector<Real> cd) : mach_(std::move(mach)), cd_(std::move(cd)) {
    if (mach_.size() != cd_.size()) {
        mach_.clear();
        cd_.clear();
    }
}

Real DragTable::cd(Real m) const {
    const std::size_t n = mach_.size();
    if (n == 0) return 0;
    if (!(m > mach_[0])) return cd_[0];
    if (m >= mach_[n - 1]) return cd_[n - 1];
    // Binary search for the bracketing interval.
    std::size_t lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        const std::size_t mid = (lo + hi) / 2;
        if (mach_[mid] <= m)
            lo = mid;
        else
            hi = mid;
    }
    const Real t = (m - mach_[lo]) / (mach_[hi] - mach_[lo]);
    return cd_[lo] + t * (cd_[hi] - cd_[lo]);
}

DragModel DragModel::standard(const DragTable& reference, const BallisticCoefficient& bc) {
    DragModel d;
    if (reference.empty() || bc.empty()) return d;
    d.kind_ = Kind::StandardBc;
    d.table_ = reference;
    d.bc_ = bc;
    // One BC per speed band. Bands left unset (a single BC built in code)
    // mean "at any speed": no band can be read past its end.
    if (d.bc_.minSpeeds.size() != d.bc_.values.size()) d.bc_.minSpeeds.assign(d.bc_.values.size(), 0.0);
    return d;
}

DragModel DragModel::bullet(const DragTable& curve, Real mass, Real diameter) {
    DragModel d;
    if (curve.empty() || !(mass > 0) || !(diameter > 0)) return d;
    d.kind_ = Kind::BulletCurve;
    d.table_ = curve;
    d.curveFactor_ = (0.25 * kPi * diameter * diameter) / (2.0 * mass);
    return d;
}

Real DragModel::bcAt(Real speed) const {
    if (kind_ != Kind::StandardBc) return 0;
    for (std::size_t i = 0; i < bc_.values.size(); ++i)
        if (speed >= bc_.minSpeeds[i]) return bc_.values[i];
    return bc_.values.back();
}

Real DragModel::factor(Real speed, Real speedOfSound) const {
    const Real mach = speedOfSound > 0 ? speed / speedOfSound : 0;
    switch (kind_) {
        case Kind::StandardBc: return table_.cd(mach) * kPi / (8.0 * bcAt(speed));
        case Kind::BulletCurve: return table_.cd(mach) * curveFactor_;
        default: return 0;
    }
}

Vec3 DragModel::acceleration(const Vec3& vAir, const AirSample& air) const {
    const Real speed = length(vAir);
    if (!(speed > 0)) return {};
    return vAir * (-factor(speed, air.speedOfSound) * air.density * speed);
}

DragModel dragModelFor(const ProjectileDef& p, const Library& lib, const std::string& prefer, std::string& error) {
    error.clear();
    // "g1", "G1", "g7", "Bullet": the family names, in any case.
    std::string want = prefer;
    for (char& ch : want) ch = static_cast<char>(ch >= 'a' && ch <= 'z' ? ch - 32 : ch);
    if (want == "BULLET") want = "bullet";
    const bool any = want.empty();
    if ((any || want == "bullet") && !p.dragCurve.empty()) {
        if (const DragCurveDef* c = lib.dragCurve(p.dragCurve)) return DragModel::bullet(DragTable(*c), p.mass, p.diameter);
        error = "projectile \"" + p.id + "\": unknown drag curve \"" + p.dragCurve + "\"";
        return {};
    }
    if ((any || want == "G7") && !p.bcG7.empty()) {
        if (const DragCurveDef* g7 = lib.dragCurve("g7")) return DragModel::standard(DragTable(*g7), p.bcG7);
        error = "the library has no standard G7 table (id \"g7\")";
        return {};
    }
    if ((any || want == "G1") && !p.bcG1.empty()) {
        if (const DragCurveDef* g1 = lib.dragCurve("g1")) return DragModel::standard(DragTable(*g1), p.bcG1);
        error = "the library has no standard G1 table (id \"g1\")";
        return {};
    }
    error = "projectile \"" + p.id + "\" has no drag data for \"" + (any ? std::string("any model") : prefer) + "\"";
    return {};
}

}  // namespace bd
