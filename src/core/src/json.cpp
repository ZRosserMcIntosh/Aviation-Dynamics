// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/json.hpp"

#include <cmath>
#include <cstdlib>
#include <unordered_set>

namespace avd::json {

const Value* Value::find(const std::string& key) const {
    for (const auto& m : obj_)
        if (m.first == key) return &m.second;
    return nullptr;
}

namespace {

class Parser {
public:
    explicit Parser(const std::string& t) : t_(t) {}

    ParseResult run() {
        ParseResult r;
        ws();
        r.value = value(0);
        ws();
        if (ok() && i_ != t_.size()) fail("unexpected trailing characters");
        r.error = error_;
        if (!ok()) r.value = Value();
        return r;
    }

private:
    // Exception-free error handling (the core also builds without
    // exceptions): the first error is recorded and every routine unwinds.
    bool ok() const { return error_.empty(); }
    Value fail(const std::string& m) {
        if (!ok()) return Value();
        std::size_t line = 1, col = 1;
        for (std::size_t k = 0; k < i_ && k < t_.size(); ++k) {
            if (t_[k] == '\n') { ++line; col = 1; } else { ++col; }
        }
        error_ = "line " + std::to_string(line) + ", column " + std::to_string(col) + ": " + m;
        i_ = t_.size();
        return Value();
    }
    char peek() const { return i_ < t_.size() ? t_[i_] : '\0'; }
    void ws() {
        while (i_ < t_.size()) {
            const char c = t_[i_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++i_;
            else break;
        }
    }
    bool expect(char c) {
        if (peek() != c) { fail(std::string("expected '") + c + "'"); return false; }
        ++i_;
        return true;
    }
    bool literal(const char* s) {
        std::size_t n = 0;
        while (s[n]) ++n;
        if (t_.compare(i_, n, s) == 0) { i_ += n; return true; }
        return false;
    }

    Value value(int depth) {
        if (!ok()) return Value();
        if (depth > 64) return fail("nesting too deep");
        const char c = peek();
        if (c == '{') return object(depth);
        if (c == '[') return array(depth);
        if (c == '"') return Value::string(string());
        if (c == 't') return literal("true") ? Value::boolean(true) : fail("invalid literal");
        if (c == 'f') return literal("false") ? Value::boolean(false) : fail("invalid literal");
        if (c == 'n') return literal("null") ? Value() : fail("invalid literal");
        if (c == '-' || (c >= '0' && c <= '9')) return number();
        return fail("unexpected character");
    }

    Value object(int depth) {
        if (!expect('{')) return Value();
        Value v = Value::object();
        ws();
        if (peek() == '}') { ++i_; return v; }
        std::unordered_set<std::string> keys;  // duplicate check in O(1) per key (find() is linear)
        while (ok()) {
            ws();
            if (peek() != '"') return fail("expected object key");
            std::string k = string();
            if (!ok()) break;
            if (!keys.insert(k).second) return fail("duplicate key \"" + k + "\"");
            ws();
            if (!expect(':')) break;
            ws();
            v.members().emplace_back(std::move(k), value(depth + 1));
            ws();
            if (peek() == ',') { ++i_; continue; }
            expect('}');
            return v;
        }
        return Value();
    }

    Value array(int depth) {
        if (!expect('[')) return Value();
        Value v = Value::array();
        ws();
        if (peek() == ']') { ++i_; return v; }
        while (ok()) {
            ws();
            v.items().push_back(value(depth + 1));
            ws();
            if (peek() == ',') { ++i_; continue; }
            expect(']');
            return v;
        }
        return Value();
    }

    std::string string() {
        if (!expect('"')) return {};
        std::string s;
        while (ok()) {
            if (i_ >= t_.size()) { fail("unterminated string"); break; }
            const char c = t_[i_++];
            if (c == '"') return s;
            if (static_cast<unsigned char>(c) < 0x20) { fail("control character in string"); break; }
            if (c != '\\') { s += c; continue; }
            if (i_ >= t_.size()) { fail("unterminated string"); break; }
            const char e = t_[i_++];
            switch (e) {
                case '"': s += '"'; break;
                case '\\': s += '\\'; break;
                case '/': s += '/'; break;
                case 'b': s += '\b'; break;
                case 'f': s += '\f'; break;
                case 'n': s += '\n'; break;
                case 'r': s += '\r'; break;
                case 't': s += '\t'; break;
                case 'u': {
                    // Exactly four hex digits (strtoul would also take a
                    // sign, blanks or "0x", and read past a short escape).
                    unsigned cp = 0;
                    if (!hex4(i_, cp)) { fail("bad \\u escape"); return {}; }
                    i_ += 4;
                    // Encode as UTF-8. A high surrogate must be followed by a
                    // low one (\\uDC00-\\uDFFF) and combined; a lone or
                    // misordered surrogate is an error, not a code point (the
                    // old `lo - 0xDC00` underflowed on a bad low surrogate).
                    if (cp >= 0xD800 && cp <= 0xDBFF) {
                        unsigned lo = 0;
                        if (t_.compare(i_, 2, "\\u") != 0 || !hex4(i_ + 2, lo) || lo < 0xDC00 || lo > 0xDFFF) {
                            fail("bad surrogate pair in \\u escape");
                            return {};
                        }
                        i_ += 6;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
                        fail("lone low surrogate in \\u escape");
                        return {};
                    }
                    if (cp < 0x80) s += static_cast<char>(cp);
                    else if (cp < 0x800) { s += static_cast<char>(0xC0 | (cp >> 6)); s += static_cast<char>(0x80 | (cp & 0x3F)); }
                    else if (cp < 0x10000) {
                        s += static_cast<char>(0xE0 | (cp >> 12));
                        s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        s += static_cast<char>(0x80 | (cp & 0x3F));
                    } else {
                        s += static_cast<char>(0xF0 | (cp >> 18));
                        s += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
                        s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        s += static_cast<char>(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default: fail("bad escape"); return {};
            }
        }
        return {};
    }

    // Four hex digits at `at` into `out`; false if fewer or not hex.
    bool hex4(std::size_t at, unsigned& out) const {
        if (at + 4 > t_.size()) return false;
        unsigned v = 0;
        for (std::size_t k = at; k < at + 4; ++k) {
            const char c = t_[k];
            unsigned d;
            if (c >= '0' && c <= '9') d = static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') d = static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') d = static_cast<unsigned>(c - 'A' + 10);
            else return false;
            v = (v << 4) | d;
        }
        out = v;
        return true;
    }

    Value number() {
        const std::size_t start = i_;
        if (peek() == '-') ++i_;
        // RFC 8259 grammar: no leading zeros, digits after the point and
        // the exponent (strtod alone would take "01", "1." and "1e").
        auto digit = [&] { return peek() >= '0' && peek() <= '9'; };
        if (!digit()) return fail("bad number");
        if (peek() == '0') ++i_;
        else while (digit()) ++i_;
        if (digit()) return fail("bad number (leading zero)");
        if (peek() == '.') {
            ++i_;
            if (!digit()) return fail("bad number (digit expected after '.')");
            while (digit()) ++i_;
        }
        if (peek() == 'e' || peek() == 'E') {
            ++i_;
            if (peek() == '+' || peek() == '-') ++i_;
            if (!digit()) return fail("bad number (exponent digits expected)");
            while (digit()) ++i_;
        }
        const double d = std::strtod(t_.substr(start, i_ - start).c_str(), nullptr);
        if (!std::isfinite(d)) return fail("number out of range");
        return Value::number(d);
    }

    const std::string& t_;
    std::size_t i_ = 0;
    std::string error_;
};

}  // namespace

ParseResult parse(const std::string& text) { return Parser(text).run(); }

}  // namespace avd::json
