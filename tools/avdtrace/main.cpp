// Aviation Dynamics — golden-trace recorder and checker.
// SPDX-License-Identifier: MIT
//
//   avdtrace --list                 the canonical scenarios
//   avdtrace --record [name ...]    fly each scenario and write tests/golden/<name>.csv
//   avdtrace --check  [name ...]    fly each scenario and compare with its golden file
//   avdtrace --run <name> [out.csv] fly one scenario and write the CSV (stdout by default)
//   avdtrace --dir <path> ...       use another golden directory (also: AVD_GOLDEN_DIR)
//
// Goldens change only in a commit that explains why: tests/golden/README.md.
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "golden.hpp"

namespace g = avd::scenarios::golden;

namespace {

int usage() {
    std::cerr << "usage: avdtrace [--dir path] --list | --record [name ...] | --check [name ...] | --run name [out.csv]\n";
    return 2;
}

std::vector<std::string> pick(const std::vector<std::string>& names) {
    return names.empty() ? g::scenarioNames() : names;
}

}  // namespace

int main(int argc, char** argv) {
    std::string mode;
    std::vector<std::string> names;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--dir" && i + 1 < argc) {
            setenv("AVD_GOLDEN_DIR", argv[++i], 1);
        } else if (a.rfind("--", 0) == 0) {
            if (!mode.empty()) return usage();
            mode = a;
        } else {
            names.push_back(a);
        }
    }
    if (mode.empty()) return usage();
    try {
        if (mode == "--list") {
            for (const std::string& n : g::scenarioNames()) std::cout << n << "  " << g::describe(n) << "\n";
            std::cout << "golden directory: " << g::goldenDirectory() << "\n";
            return 0;
        }
        if (mode == "--run") {
            if (names.empty() || names.size() > 2) return usage();
            const g::Trace t = g::run(names[0]);
            if (names.size() == 2) {
                std::ofstream f(names[1]);
                g::writeCsv(f, t);
                std::cerr << names[0] << ": " << t.rows.size() << " rows -> " << names[1] << "\n";
            } else {
                g::writeCsv(std::cout, t);
            }
            return 0;
        }
        if (mode == "--record") {
            for (const std::string& n : pick(names)) {
                const g::Trace t = g::run(n);
                const std::string path = g::goldenPath(n);
                std::ofstream f(path);
                if (!f) {
                    std::cerr << "cannot write " << path << "\n";
                    return 1;
                }
                g::writeCsv(f, t);
                std::cout << n << ": " << t.rows.size() << " rows (" << (t.rows.empty() ? 0.0 : t.rows.back()[0]) << " s) -> " << path << "\n";
            }
            std::cout << "Recorded. Commit the files with a message that says why they changed (tests/golden/README.md).\n";
            return 0;
        }
        if (mode == "--check") {
            int bad = 0, missing = 0;
            for (const std::string& n : pick(names)) {
                std::ifstream f(g::goldenPath(n));
                if (!f) {
                    std::cout << n << ": no golden file at " << g::goldenPath(n) << " (record with avdtrace --record)\n";
                    ++missing;
                    continue;
                }
                g::Trace golden;
                std::string err;
                if (!g::readCsv(f, golden, err)) {
                    std::cout << n << ": unreadable golden: " << err << "\n";
                    ++bad;
                    continue;
                }
                const g::Trace fresh = g::run(n);
                const g::Comparison c = g::compare(golden, fresh);
                std::cout << n << ": " << (c.ok ? "matches" : "DIFFERS") << "\n";
                if (!c.ok) {
                    g::writeReport(std::cout, c);
                    ++bad;
                }
            }
            return bad ? 1 : (missing ? 2 : 0);
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return usage();
}
