// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Construction-time failures (invalid definitions) throw where C++ exceptions
// are enabled. Builds without exceptions (e.g. wasi-sdk WebAssembly) abort
// with the message instead; such hosts must validate definitions first
// (AircraftDefinition::validate / loadAircraftFromJson) — the step itself
// never fails this way.
#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>

namespace avd::detail {
[[noreturn]] inline void fatal(const std::string& msg) {
    std::fprintf(stderr, "avd: %s\n", msg.c_str());
    std::abort();
}
}  // namespace avd::detail

#if defined(__cpp_exceptions) || defined(__EXCEPTIONS) || defined(_CPPUNWIND)
#define AVD_FAIL(ExceptionType, message) throw ExceptionType(message)
#else
#define AVD_FAIL(ExceptionType, message) ::avd::detail::fatal(message)
#endif
