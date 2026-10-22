# ADR 0004: Determinism

Status: accepted (Milestone 1)

## Context

The brief asks for thousands of rounds in flight, every one real, and
*identical on every machine*: the same seed gives the same shot, bit for
bit, natively and in every browser. Two things break that in ordinary C++:

- **The C library's transcendental functions** (`exp`, `log`, `sin`,
  `atan2`, `pow`) are not specified to the last bit, and glibc, Apple's
  libm, the MSVC CRT and wasi-libc give different last bits. A drag curve
  looked up through `pow`, a Box-Muller `log`, or a rotation through `sin`,
  repeated over a thousand steps, is enough to move a round.
- **Floating-point contraction**: compilers may fuse `a * b + c` into one
  fused multiply-add (Apple clang on arm64 does by default), which rounds
  once instead of twice.

`+`, `−`, `×`, `÷` and `sqrt` are correctly rounded by IEEE 754 on every
target we build for (x86-64 SSE2, arm64, wasm32).

## Decision

- **The core's maths uses only correctly rounded operations.** Elementary
  functions come from `bd::dm` (`bd/dmath.hpp`): `exp`, `log`, `sin`, `cos`,
  `atan`, `atan2`, `asin`, `acos`, `pow`, `cbrt`, `tanh`, built from fdlibm's
  algorithms and minimax coefficients with `+ − × ÷` and exact scaling
  (`ldexp`, `frexp`). They agree with glibc within 1–2 ulp (`pow` within 16
  ulp, see `tests/test_foundations.cpp`) and give the same bits everywhere.
  `std::sqrt` is allowed (correctly rounded); `std::exp` and friends are not
  used in `src/core`.
- **No contraction, no fast-math.** Every target is compiled with
  `-ffp-contract=off -fno-fast-math` (gcc, clang, wasi-sdk) or `/fp:precise`
  (MSVC, which does not contract under it).
- **Seeded random numbers with streams.** `bd::Rng` is xoshiro256** seeded by
  SplitMix64 from (seed, stream); normal deviates by Box-Muller on `bd::dm`.
  Each round will draw from its own stream (seed, round id), so its muzzle
  velocity and its imperfections do not depend on how many rounds were drawn
  before it or in which order. `std::` distributions are not used: their
  algorithms are implementation-defined.
- **Fixed steps** (ADR 0002): the state depends on the step count, not on the
  host's frames.
- **A golden hash checked everywhere.** `determinismProbe()` hashes the bit
  patterns of every `bd::dm` function over thousands of arguments and 1,024
  normal deviates. Its value, `kDeterminismGolden`
  (`bd/determinism.hpp`), is checked by the native test on Linux gcc, Linux
  clang, the sanitizer build, macOS arm64 (Apple clang) and Windows MSVC,
  and by `web/smoke_test.mjs` in WebAssembly. Milestones 2 and 6 add the same
  kind of check over whole trajectories.
- **Host inputs are inputs.** The guarantee is: the same inputs give the same
  bits. The air is an input (ADR 0003). Aviation's atmosphere uses the C
  library's functions, so the *same weather* can produce a density that
  differs in the last bit between, say, glibc and the browser.

## Consequences

- Bit-identical results on every platform for everything inside the core,
  proven by the golden hash in CI from Milestone 1 on.
- The weather is the open seam. Milestone 6 measures it: the same
  `SimpleWeather` sampled on every CI platform, hashed. If the hashes
  differ, the options, in order of preference, are (a) Aviation adopts
  deterministic maths for its atmosphere (a change in Aviation, asked for
  first), (b) the host samples the air once per shot or per region on one
  machine and ships the samples (the natural design for multiplayer anyway:
  one authority for the weather), (c) the core quantises the air it
  receives. Until then this repository claims bit-identity across platforms
  only for the core with identical air samples.
- The deterministic functions cost about as much as libm's (they are the
  same algorithms); there is no performance reason to bypass them.
