// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Interpolation tables for aerodynamic and performance data. Lookups clamp to
// the table ends: that is extrapolation policy, not a physical limit, and
// tables that matter (e.g. section polars) span the full domain (-180..180 deg)
// so clamping never engages in practice. Allocation happens only at
// construction; lookups are allocation-free.
#pragma once

#include <string>
#include <vector>

#include "avd/math.hpp"

namespace avd {

class Table1D {
public:
    Table1D() = default;
    Table1D(std::vector<Real> x, std::vector<Real> y) : x_(std::move(x)), y_(std::move(y)) {}

    bool empty() const { return x_.empty(); }
    std::size_t size() const { return x_.size(); }
    const std::vector<Real>& xs() const { return x_; }
    const std::vector<Real>& ys() const { return y_; }

    // Returns an error message, or empty if the table is well formed
    // (equal lengths, >= 1 point, strictly increasing finite x, finite y).
    std::string validate() const;

    Real operator()(Real x) const {
        const std::size_t n = x_.size();
        if (n == 0) return 0;
        if (n == 1 || x <= x_[0]) return y_[0];
        if (x >= x_[n - 1]) return y_[n - 1];
        // Binary search: tables are small but lookups are hot.
        std::size_t lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            const std::size_t mid = (lo + hi) / 2;
            (x_[mid] <= x ? lo : hi) = mid;
        }
        const Real t = (x - x_[lo]) / (x_[hi] - x_[lo]);
        return y_[lo] + t * (y_[hi] - y_[lo]);
    }

private:
    std::vector<Real> x_, y_;
};

// Bilinear table: z(row, col) with row breakpoints r and column breakpoints c.
class Table2D {
public:
    Table2D() = default;
    Table2D(std::vector<Real> rows, std::vector<Real> cols, std::vector<Real> valuesRowMajor)
        : r_(std::move(rows)), c_(std::move(cols)), v_(std::move(valuesRowMajor)) {}

    bool empty() const { return v_.empty(); }
    std::string validate() const;

    Real operator()(Real row, Real col) const;

private:
    static void bracket(const std::vector<Real>& b, Real x, std::size_t& i0, std::size_t& i1, Real& t);
    std::vector<Real> r_, c_, v_;
};

}  // namespace avd
