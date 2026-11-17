// Ballistic Dynamics — minimal dependency-free test harness (same style as Glass Dynamics' gls_test.hpp,
// OpenRoad Dynamics' ord_test.hpp and Aviation Dynamics' avd_test.hpp).
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace bdtest {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
inline int& failures() {
    static int f = 0;
    return f;
}
struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

// Long scenarios run at full length in optimised builds and shortened in
// Debug/sanitizer builds (same invariants, fewer simulated seconds).
inline constexpr bool kOptimisedBuild =
#ifdef NDEBUG
    true;
#else
    false;
#endif

}  // namespace bdtest

#define BD_CAT2(a, b) a##b
#define BD_CAT(a, b) BD_CAT2(a, b)
#define TEST_CASE(name)                                                                                 \
    static void BD_CAT(bd_test_fn_, __LINE__)();                                                      \
    static bdtest::Registrar BD_CAT(bd_test_reg_, __LINE__)(name, &BD_CAT(bd_test_fn_, __LINE__)); \
    static void BD_CAT(bd_test_fn_, __LINE__)()

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            ++bdtest::failures();                                                         \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #cond << "\n"; \
        }                                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                  \
    do {                                                                                                       \
        const double bd_a_ = (a), bd_b_ = (b), bd_t_ = (tol);                                               \
        if (!(std::abs(bd_a_ - bd_b_) <= bd_t_)) {                                                          \
            ++bdtest::failures();                                                                             \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| <= " #tol << "  (" \
                      << bd_a_ << " vs " << bd_b_ << ")\n";                                                  \
        }                                                                                                      \
    } while (0)

#define CHECK_RANGE(v, lo, hi)                                                                                     \
    do {                                                                                                           \
        const double bd_v_ = (v);                                                                                 \
        if (!(bd_v_ >= (lo) && bd_v_ <= (hi))) {                                                                 \
            ++bdtest::failures();                                                                                 \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #v " = " << bd_v_ << " not in [" << (lo) \
                      << ", " << (hi) << "]\n";                                                                    \
        }                                                                                                          \
    } while (0)

// ---- Shared fixtures ----
#include <fstream>
#include <sstream>

#include "bd/library.hpp"

namespace bdtest {

inline std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

inline const bd::Library& library() {
    static bd::Library lib = [] {
        bd::Library l;
        for (const char* f : {"/projectiles.json", "/cartridges.json", "/rifles.json"}) {
            const std::string text = readFile(std::string(BD_DATA_DIR) + f);
            if (text.empty()) {
                std::cerr << "  library: cannot read " << f << "\n";
                ++failures();
            }
            for (const auto& e : l.load(text)) {
                std::cerr << "  library: " << e << "\n";
                ++failures();
            }
        }
        for (const auto& e : l.resolve()) {
            std::cerr << "  library: " << e << "\n";
            ++failures();
        }
        return l;
    }();
    return lib;
}

}  // namespace bdtest
