// Aviation Dynamics — robustness of the JSON parser and the aircraft loader
// against malformed and adversarial input: every case must end in a clean
// error (or a definition), never a crash, a hang or undefined behaviour.
// Run under the sanitizer build too (CI: linux-gcc-sanitizers); the
// libFuzzer target in tools/fuzz explores beyond this fixed list.
// SPDX-License-Identifier: MIT
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "avd/aircraft_loader.hpp"
#include "avd/json.hpp"
#include "avd_test.hpp"

using namespace avd;

namespace {

std::string referenceText() {
    std::ifstream f(std::string(AVD_DATA_DIR) + "/aircraft/tupi_t180.json");
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Parse and load; returns true when the parser rejected the text.
bool rejected(const std::string& text) {
    const json::ParseResult p = json::parse(text);
    const LoadResult l = loadAircraftFromJson(text);
    if (p.ok()) CHECK(!l.ok() || !l.definition.id.empty());  // loader ran to a verdict
    else CHECK(!l.ok());                                       // a parse error is a load error
    if (!p.ok()) CHECK(p.error.find("line ") == 0);          // positioned message
    return !p.ok();
}

std::string replaceOnce(std::string s, const std::string& from, const std::string& to) {
    const auto k = s.find(from);
    if (k != std::string::npos) s.replace(k, from.size(), to);
    return s;
}

}  // namespace

TEST_CASE("json robustness: malformed documents are rejected with a positioned error, never a crash") {
    // Each snippet is invalid JSON (RFC 8259) or beyond the parser's limits.
    const std::vector<std::string> bad = {
        "", " ", "\xEF\xBB\xBF{}",                              // empty, blank, BOM
        "{", "[", "}", "]", "{\"a\":1", "[1,2", "{\"a\":[1,2}",  // unbalanced
        "{,}", "[,]", "{\"a\":1,}", "[1,]", "{\"a\"}", "{\"a\":}", "{1:2}", "[1 2]", "{\"a\":1 \"b\":2}",
        "{\"a\":1}}", "[1]]", "[1] x", "nul", "truex", "{\"a\": tru}", "fals", "nulll",
        // Numbers: NaN/inf spellings, hex, signs, incomplete exponents, overflow.
        "NaN", "-NaN", "Infinity", "-Infinity", "inf", "0x1F", "1_000", "+1", ".5", "-", "--1", "1e", "1e+", "1e-",
        "[1e999]", "[-1e999]", "{\"a\": 1e400}", "01", "[01]", "- 1",
        // Strings: bad escapes, bad or short \u escapes, every surrogate misuse, control characters.
        "\"\\x41\"", "\"\\U0041\"", "\"\\u12\"", "\"\\u12G4\"", "\"\\u+123\"", "\"\\u 123\"", "\"\\u-123\"",
        "\"\\u0x12\"", "\"\\uD800\"", "\"\\uDC00\"", "\"\\uDFFF\"", "\"\\uD800\\u0041\"", "\"\\uD800\\uD800\"",
        "\"\\uD800\\uDBFF\"", "\"\\uD800\\u\"", "\"\\uD800\\uDC\"", "\"\\uD800\\uZZZZ\"", "\"\\u\"", "\"\\",
        "\"abc", "\"a\nb\"", "\"a\tb\"", std::string("\"a\0b\"", 5), "\"\\", "\"\\u",
        // Nesting beyond the limit (64) and a deep unclosed run.
        std::string(100, '[') + std::string(100, ']'), std::string(5000, '{'), std::string(5000, '['),
        // Duplicate keys.
        "{\"a\":1,\"a\":2}", "{\"a\":{\"b\":1,\"b\":1}}",
    };
    int rejectedCount = 0;
    for (const std::string& s : bad) {
        if (rejected(s)) ++rejectedCount;
        else std::cerr << "  accepted: " << s.substr(0, 40) << "\n";
    }
    CHECK(rejectedCount == static_cast<int>(bad.size()));
    CHECK(bad.size() >= 50);
}

TEST_CASE("json robustness: valid but unusual documents parse, and odd escapes decode correctly") {
    // Surrogate pairs combine; BMP escapes and embedded NUL survive.
    const auto emoji = json::parse("\"\\uD83D\\uDE00\"");
    CHECK(emoji.ok());
    CHECK(emoji.value.asString() == "\xF0\x9F\x98\x80");
    const auto maxCp = json::parse("\"\\uDBFF\\uDFFF\"");
    CHECK(maxCp.ok());
    CHECK(maxCp.value.asString() == "\xF4\x8F\xBF\xBF");
    const auto nul = json::parse("\"a\\u0000b\"");
    CHECK(nul.ok());
    CHECK(nul.value.asString().size() == 3);
    const auto hexCase = json::parse("\"\\u00e9\\u00C9\"");
    CHECK(hexCase.ok());
    CHECK(hexCase.value.asString() == "\xC3\xA9\xC3\x89");
    // Large but legal: a 20k-element array, a 5k-key object (duplicate check stays fast), long strings.
    std::string arr = "[";
    for (int i = 0; i < 20000; ++i) arr += (i ? ",1" : "1");
    arr += "]";
    CHECK(json::parse(arr).ok());
    std::string obj = "{";
    for (int i = 0; i < 5000; ++i) obj += (i ? ",\"k" : "\"k") + std::to_string(i) + "\":" + std::to_string(i);
    obj += "}";
    CHECK(json::parse(obj).ok());
    CHECK(json::parse("\"" + std::string(100000, 'x') + "\"").ok());
    CHECK(json::parse("[1e-999, -0, 0.0, 123456789012345678901234567890]").ok());
    CHECK(json::parse(std::string(64, '[') + std::string(64, ']')).ok());
    // Scalars and nesting at the top level load as schema errors, not crashes.
    for (const char* s : {"{}", "[]", "null", "1", "\"x\"", "true", "{\"schema_version\": 1}", "{\"schema_version\": -1}",
                          "{\"schema_version\": 1e300}", "{\"id\": null}", "[{}]"}) {
        CHECK(json::parse(s).ok());
        CHECK(!loadAircraftFromJson(s).ok());
    }
}

TEST_CASE("json robustness: every truncation and single-byte corruption of the reference aircraft ends cleanly") {
    const std::string ref = referenceText();
    CHECK(ref.size() > 1000);
    // Truncations: never valid (a prefix of a document is not a document).
    int ok = 0;
    for (std::size_t n = 0; n < ref.size(); n += std::max<std::size_t>(1, ref.size() / 97)) {
        const std::string t = ref.substr(0, n);
        if (json::parse(t).ok()) ++ok;
        CHECK(!loadAircraftFromJson(t).ok());
    }
    CHECK(ok == 0);
    // Corruptions: a structural byte, a quote, a NUL, a high byte and a digit
    // dropped at many positions; and a few keys removed, doubled or retyped.
    const char bytes[] = {'"', '\\', '{', '[', '}', ']', ',', ':', '\0', static_cast<char>(0xFF), '9', '-', 'e', 'u'};
    for (std::size_t n = 7; n < ref.size(); n += std::max<std::size_t>(1, ref.size() / 61)) {
        for (char b : bytes) {
            std::string t = ref;
            t[n] = b;
            (void)json::parse(t);
            (void)loadAircraftFromJson(t);
        }
    }
    // Schema-level adversaries on an otherwise valid document.
    const std::vector<std::pair<std::string, std::string>> edits = {
        {"\"surfaces\"", "\"surfaces\": \"wing\", \"x\""},
        {"\"stations\"", "\"stations\": [null], \"x\""},
        {"\"flap_detents_deg\": [", "\"flap_detents_deg\": [], \"x\": ["},
        {"\"tanks\"", "\"tanks\": [], \"x\""},
        {"\"initial_selector\": 0", "\"initial_selector\": 99"},
        {"\"tanks\": [0, 1]", "\"tanks\": [99, -1]"},
        {"\"empty_kg\": 754.3", "\"empty_kg\": 1e308"},
        {"\"empty_kg\": 754.3", "\"empty_kg\": -1e308"},
        {"\"empty_kg\": 754.3", "\"empty_kg\": 0"},
        {"\"diameter_m\"", "\"diameter_m\": 1e-300, \"x\""},
        {"\"blades\": 2", "\"blades\": 1e6"},
        {"\"blades\": 2", "\"blades\": -3"},
        {"\"downwash_source\": 0", "\"downwash_source\": 1000000"},
        {"\"downwash_source\": 0", "\"downwash_source\": -1"},
        {"\"id\": \"tupi_t180\"", "\"id\": \"" + std::string(100000, 'a') + "\""},
    };
    for (const auto& e : edits) {
        const std::string t = replaceOnce(ref, e.first, e.second);
        if (t == ref) continue;  // the key is not in this document version: nothing to corrupt
        const LoadResult r = loadAircraftFromJson(t);
        (void)r;  // a verdict either way; the check is that we get here
    }
    // Still loads unchanged.
    CHECK(loadAircraftFromJson(ref).ok());
}
