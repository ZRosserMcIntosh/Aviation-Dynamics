// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/json.hpp"

#include <cmath>
#include <cstdlib>

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
        try {
            ws();
            r.value = value(0);
            ws();
            if (i_ != t_.size()) fail("unexpected trailing characters");
        } catch (const Error& e) {
            r.error = e.msg;
        }
        return r;
    }

private:
    struct Error { std::string msg; };

    [[noreturn]] void fail(const std::string& m) const {
        std::size_t line = 1, col = 1;
        for (std::size_t k = 0; k < i_ && k < t_.size(); ++k) {
            if (t_[k] == '\n') { ++line; col = 1; } else { ++col; }
        }
        throw Error{"line " + std::to_string(line) + ", column " + std::to_string(col) + ": " + m};
    }
    char peek() const { return i_ < t_.size() ? t_[i_] : '\0'; }
    void ws() {
        while (i_ < t_.size()) {
            const char c = t_[i_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++i_;
            else break;
        }
    }
    void expect(char c) {
        if (peek() != c) fail(std::string("expected '") + c + "'");
        ++i_;
    }
    bool literal(const char* s) {
        std::size_t n = 0;
        while (s[n]) ++n;
        if (t_.compare(i_, n, s) == 0) { i_ += n; return true; }
        return false;
    }

    Value value(int depth) {
        if (depth > 64) fail("nesting too deep");
        const char c = peek();
        if (c == '{') return object(depth);
        if (c == '[') return array(depth);
        if (c == '"') return Value::string(string());
        if (c == 't') { if (literal("true")) return Value::boolean(true); fail("invalid literal"); }
        if (c == 'f') { if (literal("false")) return Value::boolean(false); fail("invalid literal"); }
        if (c == 'n') { if (literal("null")) return Value(); fail("invalid literal"); }
        if (c == '-' || (c >= '0' && c <= '9')) return number();
        fail("unexpected character");
    }

    Value object(int depth) {
        expect('{');
        Value v = Value::object();
        ws();
        if (peek() == '}') { ++i_; return v; }
        while (true) {
            ws();
            if (peek() != '"') fail("expected object key");
            std::string k = string();
            if (v.find(k)) fail("duplicate key \"" + k + "\"");
            ws();
            expect(':');
            ws();
            v.members().emplace_back(std::move(k), value(depth + 1));
            ws();
            if (peek() == ',') { ++i_; continue; }
            expect('}');
            return v;
        }
    }

    Value array(int depth) {
        expect('[');
        Value v = Value::array();
        ws();
        if (peek() == ']') { ++i_; return v; }
        while (true) {
            ws();
            v.items().push_back(value(depth + 1));
            ws();
            if (peek() == ',') { ++i_; continue; }
            expect(']');
            return v;
        }
    }

    std::string string() {
        expect('"');
        std::string s;
        while (true) {
            if (i_ >= t_.size()) fail("unterminated string");
            const char c = t_[i_++];
            if (c == '"') return s;
            if (static_cast<unsigned char>(c) < 0x20) fail("control character in string");
            if (c != '\\') { s += c; continue; }
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
                    if (i_ + 4 > t_.size()) fail("bad \\u escape");
                    unsigned cp = static_cast<unsigned>(std::strtoul(t_.substr(i_, 4).c_str(), nullptr, 16));
                    i_ += 4;
                    // Encode as UTF-8 (surrogate pairs are combined when present).
                    if (cp >= 0xD800 && cp <= 0xDBFF && t_.compare(i_, 2, "\\u") == 0) {
                        const unsigned lo = static_cast<unsigned>(std::strtoul(t_.substr(i_ + 2, 4).c_str(), nullptr, 16));
                        i_ += 6;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
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
                default: fail("bad escape");
            }
        }
    }

    Value number() {
        const std::size_t start = i_;
        if (peek() == '-') ++i_;
        if (!(peek() >= '0' && peek() <= '9')) fail("bad number");
        while (peek() >= '0' && peek() <= '9') ++i_;
        if (peek() == '.') { ++i_; while (peek() >= '0' && peek() <= '9') ++i_; }
        if (peek() == 'e' || peek() == 'E') {
            ++i_;
            if (peek() == '+' || peek() == '-') ++i_;
            while (peek() >= '0' && peek() <= '9') ++i_;
        }
        const double d = std::strtod(t_.substr(start, i_ - start).c_str(), nullptr);
        if (!std::isfinite(d)) fail("number out of range");
        return Value::number(d);
    }

    const std::string& t_;
    std::size_t i_ = 0;
};

}  // namespace

ParseResult parse(const std::string& text) { return Parser(text).run(); }

}  // namespace avd::json
