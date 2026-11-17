// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
#include "bd/determinism.hpp"

#include "bd/dmath.hpp"
#include "bd/rng.hpp"

namespace bd {

std::uint64_t determinismProbe() {
    Hash64 h;
    for (int i = 0; i < 4096; ++i) {
        const Real t = (i + 0.5) / 4096.0;
        h.add(dm::exp(-40.0 + 80.0 * t));
        h.add(dm::log(1e-6 + 1e6 * t * t));
        h.add(dm::sin(-50.0 + 100.0 * t));
        h.add(dm::cos(-50.0 + 100.0 * t));
        h.add(dm::atan2(dm::sin(13.0 * t), dm::cos(7.0 * t) - 0.3));
        h.add(dm::pow(0.01 + 30.0 * t, 1.0 / 3.0));
        h.add(dm::cbrt(-5.0 + 10.0 * t));
        h.add(dm::tanh(-3.0 + 6.0 * t));
        h.add(dm::asin(-0.999 + 1.998 * t));
    }
    Rng r(12345, 7);
    for (int i = 0; i < 1024; ++i) h.add(r.normal());
    return h.h;
}

}  // namespace bd
