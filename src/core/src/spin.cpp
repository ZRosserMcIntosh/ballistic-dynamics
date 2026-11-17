// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/spin.hpp"

#include "bd/dmath.hpp"
#include "bd/units.hpp"

namespace bd {

Real millerStability(Real mass, Real twist, Real diameter, Real length, Real speed, Real temperature, Real pressure) {
    const Real d = units::toInches(diameter);
    if (!(d > 0) || twist == 0 || !(length > 0)) return 0;
    const Real m = units::toGrains(mass);
    const Real t = units::toInches(twist < 0 ? -twist : twist) / d;  // calibres per turn
    const Real l = units::toInches(length) / d;
    const Real fps = units::toFps(speed);
    const Real tempF = units::kelvinToFahrenheit(temperature);
    const Real inHg = pressure / units::kInchOfMercury;
    const Real base = 30.0 * m / (t * t * d * d * d * l * (1.0 + l * l));
    return base * dm::cbrt(fps / 2800.0) * ((tempF + 460.0) / 519.0) * (29.92 / inHg);
}

Real overturningRatio(Real mach) {
    // Estimated: the overturning-moment slope of a spitzer boat tail rises by
    // about a third from Mach 2.5 to its peak just above Mach 1 and falls
    // again subsonic (the trend of McCoy's small-arms tables). Linear between
    // points; flat outside.
    static const Real m[] = {0.0, 0.7, 0.9, 1.0, 1.1, 1.5, 2.0, 2.5, 5.0};
    static const Real r[] = {1.05, 1.10, 1.25, 1.33, 1.35, 1.18, 1.08, 1.00, 1.00};
    if (mach <= m[0]) return r[0];
    for (int i = 1; i < 9; ++i)
        if (mach <= m[i]) return r[i - 1] + (mach - m[i - 1]) / (m[i] - m[i - 1]) * (r[i] - r[i - 1]);
    return r[8];
}

SpinParams spinParamsFor(const ProjectileDef& p, Real twist, Real muzzleSpeed, const AirSample& air) {
    SpinParams s;
    s.enabled = twist != 0 && p.length > 0 && p.diameter > 0;
    s.twist = twist;
    s.mass = p.mass;
    s.diameter = p.diameter;
    s.length = p.length;
    // A solid cylinder of the bullet's length has Iy/Ix = (2/3) l^2 + 1/2; the
    // ogive and boat tail take mass away from the ends: about 0.7 of that
    // (estimated; McCoy's 7.62 mm M80 data is near it).
    const Real l = s.calibres();
    s.iyOverIx = 0.7 * ((2.0 / 3.0) * l * l + 0.5);
    s.muzzleSpeed = muzzleSpeed;
    s.muzzleDensity = air.density;
    s.muzzleMach = air.speedOfSound > 0 ? muzzleSpeed / air.speedOfSound : 0;
    s.sgMuzzle = millerStability(p.mass, twist, p.diameter, p.length, muzzleSpeed, air.temperature, air.pressure);
    return s;
}

Real aerodynamicJump(Real crosswindToRight, Real sg, Real calibres, Real twistSign) {
    const Real mph = crosswindToRight / 0.44704;
    const Real moaPerMph = 0.01 * sg - 0.0024 * calibres + 0.032;
    // Right-hand twist: a wind from the left (blowing right) jumps the round up.
    return (twistSign >= 0 ? 1.0 : -1.0) * moaPerMph * mph * units::kMoa;
}

Real litzSpinDrift(Real sg, Real timeOfFlight) {
    if (!(timeOfFlight > 0)) return 0;
    return units::inches(1.25 * (sg + 1.2) * dm::pow(timeOfFlight, 1.83));
}

Yaw2 epicyclicYaw(const SpinParams& s, Yaw2 y0, Real t) {
    if (!s.enabled || !(s.sgMuzzle > 1.0)) return {};
    const Real p = s.muzzleSpin();
    const Real base = p / (2.0 * s.iyOverIx);
    const Real root = std::sqrt(1.0 - 1.0 / s.sgMuzzle);
    const Real wf = base * (1.0 + root), ws = base * (1.0 - root);
    // Estimated damping: the fast (nutation) arm dies in a few hundredths of
    // a second, the slow (precession) arm in a few tenths.
    const Real lf = -40.0, ls = -6.0;
    // xi = KF e^{(lf + i wf) t} + KS e^{(ls + i ws) t}, xi(0) = y0, xi'(0) = tip-off rate.
    const Real r0 = y0.right, i0 = y0.up;
    const Real dr = s.tipOffRate, di = 0;
    // KF = (xi'0 - sS xi0) / (sF - sS), sJ = lJ + i wJ.
    const Real nr = dr - (ls * r0 - ws * i0), ni = di - (ls * i0 + ws * r0);
    const Real er = lf - ls, ei = wf - ws;
    const Real den = er * er + ei * ei;
    if (!(den > 0)) return {};
    const Real kfr = (nr * er + ni * ei) / den, kfi = (ni * er - nr * ei) / den;
    const Real ksr = r0 - kfr, ksi = i0 - kfi;
    Real sf, cf, ss, cs;
    dm::sincos(wf * t, sf, cf);
    dm::sincos(ws * t, ss, cs);
    const Real af = dm::exp(lf * t), as = dm::exp(ls * t);
    Yaw2 out;
    out.right = af * (kfr * cf - kfi * sf) + as * (ksr * cs - ksi * ss);
    out.up = af * (kfr * sf + kfi * cf) + as * (ksr * ss + ksi * cs);
    return out;
}

}  // namespace bd
