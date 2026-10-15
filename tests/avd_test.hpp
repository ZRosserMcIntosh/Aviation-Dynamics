// Aviation Dynamics — minimal dependency-free test harness (same style as
// OpenRoad Dynamics' ord_test.hpp).
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace avdtest {

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

}  // namespace avdtest

#define AVD_CAT2(a, b) a##b
#define AVD_CAT(a, b) AVD_CAT2(a, b)
#define TEST_CASE(name)                                                                                 \
    static void AVD_CAT(avd_test_fn_, __LINE__)();                                                      \
    static avdtest::Registrar AVD_CAT(avd_test_reg_, __LINE__)(name, &AVD_CAT(avd_test_fn_, __LINE__)); \
    static void AVD_CAT(avd_test_fn_, __LINE__)()

#define CHECK(cond)                                                                        \
    do {                                                                                   \
        if (!(cond)) {                                                                     \
            ++avdtest::failures();                                                         \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #cond << "\n"; \
        }                                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                  \
    do {                                                                                                       \
        const double avd_a_ = (a), avd_b_ = (b), avd_t_ = (tol);                                               \
        if (!(std::abs(avd_a_ - avd_b_) <= avd_t_)) {                                                          \
            ++avdtest::failures();                                                                             \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| <= " #tol << "  (" \
                      << avd_a_ << " vs " << avd_b_ << ")\n";                                                  \
        }                                                                                                      \
    } while (0)

#define CHECK_RANGE(v, lo, hi)                                                                                     \
    do {                                                                                                           \
        const double avd_v_ = (v);                                                                                 \
        if (!(avd_v_ >= (lo) && avd_v_ <= (hi))) {                                                                 \
            ++avdtest::failures();                                                                                 \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #v " = " << avd_v_ << " not in [" << (lo) \
                      << ", " << (hi) << "]\n";                                                                    \
        }                                                                                                          \
    } while (0)
