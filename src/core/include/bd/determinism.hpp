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

}  // namespace bd
