// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/table.hpp"

namespace avd {

namespace {
std::string checkAxis(const std::vector<Real>& b, const char* name) {
    if (b.empty()) return std::string(name) + ": no breakpoints";
    for (std::size_t i = 0; i < b.size(); ++i) {
        if (!std::isfinite(b[i])) return std::string(name) + ": non-finite breakpoint";
        if (i > 0 && !(b[i] > b[i - 1])) return std::string(name) + ": breakpoints must be strictly increasing";
    }
    return {};
}
}  // namespace

std::string Table1D::validate() const {
    if (x_.size() != y_.size()) return "x and y have different lengths";
    if (auto e = checkAxis(x_, "x"); !e.empty()) return e;
    for (Real v : y_)
        if (!std::isfinite(v)) return "non-finite value";
    return {};
}

std::string Table2D::validate() const {
    if (auto e = checkAxis(r_, "rows"); !e.empty()) return e;
    if (auto e = checkAxis(c_, "cols"); !e.empty()) return e;
    if (v_.size() != r_.size() * c_.size()) return "value count != rows x cols";
    for (Real v : v_)
        if (!std::isfinite(v)) return "non-finite value";
    return {};
}

void Table2D::bracket(const std::vector<Real>& b, Real x, std::size_t& i0, std::size_t& i1, Real& t) {
    const std::size_t n = b.size();
    if (n == 1 || x <= b[0]) { i0 = i1 = 0; t = 0; return; }
    if (x >= b[n - 1]) { i0 = i1 = n - 1; t = 0; return; }
    std::size_t lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        const std::size_t mid = (lo + hi) / 2;
        (b[mid] <= x ? lo : hi) = mid;
    }
    i0 = lo; i1 = hi; t = (x - b[lo]) / (b[hi] - b[lo]);
}

Real Table2D::operator()(Real row, Real col) const {
    if (v_.empty()) return 0;
    std::size_t r0, r1, c0, c1;
    Real tr, tc;
    bracket(r_, row, r0, r1, tr);
    bracket(c_, col, c0, c1, tc);
    const std::size_t nc = c_.size();
    const Real a = lerp(v_[r0 * nc + c0], v_[r0 * nc + c1], tc);
    const Real b = lerp(v_[r1 * nc + c0], v_[r1 * nc + c1], tc);
    return lerp(a, b, tr);
}

}  // namespace avd
