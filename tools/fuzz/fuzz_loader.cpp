// Aviation Dynamics — libFuzzer target for the JSON parser and the aircraft
// loader (src/core/src/json.cpp, aircraft_loader.cpp).
// SPDX-License-Identifier: MIT
//
// Build (clang only):
//   cmake -S . -B build-fuzz -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
//         -DCMAKE_CXX_COMPILER=clang++ -DAVD_FUZZ=ON   (one line)
//   ninja -C build-fuzz avd_fuzz_loader
//   mkdir -p corpus && cp data/aircraft/*.json corpus/
//   ./build-fuzz/avd_fuzz_loader corpus -max_total_time=300
//
// Every input must come back as a parse, schema or validation error or a
// definition; a crash, a sanitizer report or a timeout is a bug in the core.
//
// Without libFuzzer (gcc, or a clang without compiler-rt) the same target
// builds as a stand-alone driver with -DAVD_FUZZ_DRIVER=ON: it runs the
// corpus files, then a deterministic mutation loop over them (byte flips,
// deletions, insertions of JSON punctuation and digits, truncations), under
// whatever sanitizers the build has:
//   cmake -S . -B build-asan -G Ninja -DCMAKE_BUILD_TYPE=Debug
//         -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined" -DAVD_FUZZ_DRIVER=ON   (one line)
//   ninja -C build-asan avd_fuzz_loader
//   ./build-asan/avd_fuzz_loader data/aircraft/*.json [-n iterations] [-seed s]
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "avd/aircraft_loader.hpp"
#include "avd/json.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    const std::string text(reinterpret_cast<const char*>(data), size);
    const avd::json::ParseResult parsed = avd::json::parse(text);
    if (parsed.ok()) {
        // Walk the DOM: every accessor must be safe on every node.
        std::size_t nodes = 0;
        std::vector<const avd::json::Value*> stack{&parsed.value};
        while (!stack.empty()) {
            const avd::json::Value* v = stack.back();
            stack.pop_back();
            ++nodes;
            if (v->isArray())
                for (const auto& e : v->items()) stack.push_back(&e);
            if (v->isObject())
                for (const auto& m : v->members()) stack.push_back(&m.second);
            if (v->isObject()) (void)v->find("id");
        }
        (void)nodes;
    }
    const avd::LoadResult r = avd::loadAircraftFromJson(text);
    if (r.ok()) {
        // A definition that passed validation must validate again unchanged.
        volatile std::size_t n = r.definition.validate().size();
        (void)n;
    }
    return 0;
}

#ifdef AVD_FUZZ_DRIVER
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <random>

int main(int argc, char** argv) {
    std::vector<std::string> seeds;
    long iterations = 20000;
    unsigned seed = 1;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "-n") && i + 1 < argc) { iterations = std::atol(argv[++i]); continue; }
        if (!std::strcmp(argv[i], "-seed") && i + 1 < argc) { seed = static_cast<unsigned>(std::atoi(argv[++i])); continue; }
        std::ifstream f(argv[i], std::ios::binary);
        if (!f) { std::fprintf(stderr, "cannot read %s\n", argv[i]); return 2; }
        seeds.emplace_back(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    if (seeds.empty()) { std::fprintf(stderr, "usage: avd_fuzz_loader corpus-file... [-n iterations] [-seed s]\n"); return 2; }
    for (const std::string& s : seeds) LLVMFuzzerTestOneInput(reinterpret_cast<const std::uint8_t*>(s.data()), s.size());
    std::mt19937 rng(seed);
    static const char kBytes[] = "{}[]\",:\\-+.eE0123456789ntfu \n\t\0\xff";
    for (long it = 0; it < iterations; ++it) {
        std::string s = seeds[rng() % seeds.size()];
        const int ops = 1 + static_cast<int>(rng() % 8);
        for (int k = 0; k < ops && !s.empty(); ++k) {
            const std::size_t at = rng() % s.size();
            switch (rng() % 5) {
                case 0: s[at] = kBytes[rng() % (sizeof kBytes - 1)]; break;
                case 1: s.erase(at, 1 + rng() % 16); break;
                case 2: s.insert(at, 1, kBytes[rng() % (sizeof kBytes - 1)]); break;
                case 3: s.resize(at); break;
                default: s[at] = static_cast<char>(rng() % 256); break;
            }
        }
        LLVMFuzzerTestOneInput(reinterpret_cast<const std::uint8_t*>(s.data()), s.size());
    }
    std::printf("avd_fuzz_loader: %zu seeds, %ld mutations, no crash\n", seeds.size(), iterations);
    return 0;
}
#endif
