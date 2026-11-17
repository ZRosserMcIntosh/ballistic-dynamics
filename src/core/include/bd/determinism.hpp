// Ballistic Dynamics — engine-independent projectile core.
// SPDX-License-Identifier: MIT
//
// Determinism probe (ADR 0004): a hash of the deterministic maths and the
// random numbers over a fixed set of arguments. Every build (gcc, clang,
// MSVC, Apple clang on arm64, wasm32) must give kDeterminismGolden; the
// native tests and the WebAssembly smoke test both check it. If a compiler
// fuses a multiply-add, or a libm call slips into bd::dm, it changes.
#pragma once

#include <cstdint>

namespace bd {

// Change only on purpose (a function added to the probe, a coefficient fixed),
// together with web/smoke_test.mjs, and say why in the commit.
inline constexpr std::uint64_t kDeterminismGolden = 0x7302405ad0a26843ull;

std::uint64_t determinismProbe();

// A whole trajectory: a round with a transonic drag table (built in, so the
// probe needs no data files) flown 2.5 s through uniform air with wind, every
// step state hashed. Golden value as above.
inline constexpr std::uint64_t kTrajectoryGolden = 0x2ca1ba49af3e55d1ull;

std::uint64_t trajectoryProbe();

// Milestone 3: the same kind of flight with spin (Miller's Sg, the yaw of
// repose, roll damping, the jump in a crosswind) on a rotating Earth.
inline constexpr std::uint64_t kSpinGolden = 0x86ee6c5cb23ef28eull;

std::uint64_t spinProbe();

}  // namespace bd
