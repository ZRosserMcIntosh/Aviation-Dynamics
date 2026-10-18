// Aviation Dynamics — wind and turbulence tests: boundary layer, MIL-HDBK-1797
// turbulence statistics, field consistency.
// SPDX-License-Identifier: MIT
#include "avd/atmosphere.hpp"
#include "avd/units.hpp"
#include "avd/wind.hpp"
#include "avd_test.hpp"

using namespace avd;
using namespace avd::units;

namespace {

// Wind blowing FROM `fromDeg` with speed `v`: the air-velocity vector.
Real speedOf(const Vec3& w) { return std::sqrt(w.x * w.x + w.y * w.y); }
Real fromDirection(const Vec3& w) { return std::atan2(-w.y, -w.x); }

// Sample gradient against a central finite difference of the sampled wind.
Real gradientError(const AtmosphereField& f, const Vec3& p, Real groundMsl, Real t) {
    auto at = [&](const Vec3& x) { return f.sample(x, groundMsl - x.z, t).wind; };
    const AirSample s = f.sample(p, groundMsl - p.z, t);
    const Real h = 1e-3;
    Real err = 0, scale = 1e-9;
    const Vec3 e[3] = {{h, 0, 0}, {0, h, 0}, {0, 0, h}};
    for (int j = 0; j < 3; ++j) {
        const Vec3 d = (at(p + e[j]) - at(p - e[j])) / (2 * h);
        const Real dv[3] = {d.x, d.y, d.z};
        for (int i = 0; i < 3; ++i) {
            err = std::max(err, std::abs(dv[i] - s.windGradient.m[i][j]));
            scale = std::max(scale, std::abs(dv[i]));
        }
    }
    return err / scale;
}

}  // namespace

// ============================================================ boundary layer ==

TEST_CASE("wind: log-law boundary layer follows the surface roughness") {
    BoundaryLayerParams p;
    p.windSpeed = 10;
    p.surfaceRoughness = 0.03;  // open airfield
    const BoundaryLayer open(p);
    CHECK_NEAR(speedOf(open.wind(10, 10)), 10, 1e-9);
    CHECK_NEAR(speedOf(open.wind(100, 100)) / 10, std::log(100 / 0.03) / std::log(10 / 0.03), 1e-9);
    p.surfaceRoughness = 1.0;  // city centre: same 10 m wind, much stronger shear
    const BoundaryLayer city(p);
    CHECK_NEAR(speedOf(city.wind(100, 100)) / 10, 2.0, 1e-9);
    // Friction velocity (u* = kappa U / ln(z/z0)) is larger over the city.
    CHECK_NEAR(open.frictionVelocity(), 0.4 * 10 / std::log(10 / 0.03), 1e-12);
    CHECK(city.frictionVelocity() > 2.4 * open.frictionVelocity());
    // Constant above the boundary layer when no winds aloft are given.
    CHECK_NEAR(speedOf(open.wind(2000, 2000)), speedOf(open.wind(600, 600)), 1e-9);
    // Held below a couple of roughness lengths, never negative or NaN.
    CHECK(speedOf(city.wind(0, 0)) > 0);
    CHECK(speedOf(city.wind(-5, -5)) == speedOf(city.wind(0, 0)));
}

TEST_CASE("wind: Ekman veer turns the wind with height; winds aloft interpolate the short way") {
    BoundaryLayerParams p;
    p.windSpeed = 8;
    p.windFromDirection = degToRad(120);
    p.veer = degToRad(-25);  // southern hemisphere: backing with height
    p.top = 600;
    p.aloft = {{3000, 30, degToRad(10)}, {1500, 20, degToRad(350)}};  // unordered on purpose
    const BoundaryLayer bl(p);
    CHECK_NEAR(radToDeg(fromDirection(bl.wind(10, 10))), 120 - 25 * (1 - sq(1 - 10.0 / 600)), 1e-6);
    CHECK_NEAR(radToDeg(fromDirection(bl.wind(600, 600))), 95, 1e-6);
    // Most of the turning is in the lower half of the layer.
    CHECK(std::abs(radToDeg(fromDirection(bl.wind(300, 300))) - 120) > 0.7 * 25);
    // 1500 -> 3000 m: 350 -> 10 deg through north, not through south.
    const Vec3 mid = bl.wind(2250, 2250);
    CHECK_NEAR(radToDeg(wrapPi(fromDirection(mid))), 0, 1e-6);
    CHECK_NEAR(speedOf(mid), 25, 1e-9);
    CHECK_NEAR(speedOf(bl.wind(9000, 9000)), 30, 1e-9);
    // Between the boundary-layer top and the first level: blended.
    const Real s1000 = speedOf(bl.wind(1000, 1000));
    CHECK(s1000 > speedOf(bl.wind(600, 600)) && s1000 < 20);
    // Winds aloft are by MSL: the same height over high ground is a higher level.
    CHECK(speedOf(bl.wind(1000, 2000)) > s1000);
}

TEST_CASE("wind: sampled shear gradient matches the profile") {
    SimpleWeather::Params p;
    p.windSpeed = 12;
    p.windFromDirection = degToRad(200);
    p.veer = degToRad(20);
    p.turbulence.mechanical = false;
    const SimpleWeather w(p);
    for (Real h : {5.0, 40.0, 250.0}) CHECK(gradientError(w, {0, 0, -h}, 0, 0) < 1e-4);
}

// ================================================================ turbulence ==

TEST_CASE("turbulence: MIL-HDBK-1797 statistics by height") {
    // No wind, no severity: calm air.
    CHECK(!AtmosphericTurbulence(TurbulenceParams{}, 0, 0).active());
    // Mechanical turbulence from a 10 m/s wind over an airfield.
    BoundaryLayerParams bp;
    bp.windSpeed = 10;
    const BoundaryLayer bl(bp);
    const AtmosphericTurbulence mech(TurbulenceParams{}, 0, bl.frictionVelocity());
    const Real us = bl.frictionVelocity();
    const TurbulenceStatistics s100 = mech.statistics(100 * kFoot, 100 * kFoot);
    CHECK_NEAR(s100.sigma.z, 1.25 * us, 1e-9);
    CHECK_NEAR(s100.sigma.x / s100.sigma.z, 1 / std::pow(0.177 + 0.0823, 0.4), 1e-9);
    CHECK_NEAR(s100.length.z, 100 * kFoot, 1e-9);                                    // L_w = h
    CHECK_NEAR(s100.length.x, 100 / std::pow(0.177 + 0.0823, 1.2) * kFoot, 1e-9);    // L_u
    // Same as MIL's sigma_w = 0.1 W20 over open terrain, within ~10%.
    CHECK_NEAR(s100.sigma.z, 0.1 * speedOf(bl.wind(20 * kFoot, 0)), 0.1 * s100.sigma.z);
    // Isotropic at 1000 ft, gone above the boundary layer (2000 ft AGL).
    const TurbulenceStatistics s1000 = mech.statistics(1000 * kFoot, 1000 * kFoot);
    CHECK_NEAR(s1000.sigma.x, s1000.sigma.z, 1e-3);
    CHECK_NEAR(mech.statistics(3000 * kFoot, 3000 * kFoot).sigma.z, 0, 1e-12);
    // Vertical gusts vanish at the surface.
    CHECK_NEAR(mech.statistics(0, 0).sigma.z, 0, 1e-12);
    // Severity classes: W20 = 15/30/45 kt at low altitude; aloft from the
    // exceedance curves (moderate at 7500 ft: 10.1 ft/s).
    TurbulenceParams mod;
    mod.severity = TurbulenceSeverity::Moderate;
    mod.mechanical = false;
    const AtmosphericTurbulence m(mod, 0, 0);
    CHECK_NEAR(m.statistics(200 * kFoot, 200 * kFoot).sigma.z, 0.1 * 30 * kKnot, 1e-9);
    const TurbulenceStatistics hi = m.statistics(7500 * kFoot, 7500 * kFoot);
    CHECK_NEAR(hi.sigma.z, 10.1 * kFoot, 1e-9);
    CHECK_NEAR(hi.length.x, 2500 * kFoot, 1e-9);
    TurbulenceParams sev = mod;
    sev.severity = TurbulenceSeverity::Severe;
    CHECK(AtmosphericTurbulence(sev, 0, 0).statistics(7500 * kFoot, 7500 * kFoot).sigma.z > 2 * hi.sigma.z);
    // Convective boundary layer adds vertical energy mid-layer.
    TurbulenceParams conv;
    conv.mechanical = false;
    conv.convectiveVelocity = 2;
    conv.convectiveDepth = 1500;
    const AtmosphericTurbulence c(conv, 0, 0);
    CHECK(c.active());
    CHECK_NEAR(c.statistics(500, 500).sigma.z,
               2 * std::sqrt(1.8 * std::pow(1 / 3.0, 2.0 / 3.0)) * (1 - 0.8 / 3), 1e-9);
}

TEST_CASE("turbulence: realised field has the specified anisotropy and scales") {
    // A long horizontal traverse through the frozen field at 30 m AGL in a
    // 12 m/s wind over the city (rough): realised RMS per component against
    // the statistics, and the vertical gusts shorter than the longitudinal.
    SimpleWeather::Params p;
    p.windSpeed = 12;
    p.windFromDirection = 0;  // u axis points south
    p.surfaceRoughness = 1.0;
    const SimpleWeather w(p);
    const Real h = 30;
    const TurbulenceStatistics st = w.turbulence().statistics(h, h);
    const int n = 20000;
    const Real dx = 2.0;
    std::vector<Vec3> u(n);
    const Vec3 mean = w.meanWindAt(h);
    Real s2[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i) {
        u[i] = w.sample({i * dx * 0.7, i * dx * 0.7, -h}, h, 0).wind - mean;
        s2[0] += u[i].x * u[i].x;  // along the wind (north-south)
        s2[1] += u[i].y * u[i].y;
        s2[2] += u[i].z * u[i].z;
    }
    CHECK_NEAR(std::sqrt(s2[0] / n), st.sigma.x, 0.35 * st.sigma.x);
    CHECK_NEAR(std::sqrt(s2[1] / n), st.sigma.y, 0.35 * st.sigma.y);
    CHECK_NEAR(std::sqrt(s2[2] / n), st.sigma.z, 0.35 * st.sigma.z);
    CHECK(st.sigma.x > 1.5 * st.sigma.z);  // near the ground: horizontal gusts dominate
    // Autocorrelation at a lag equal to L_w (30 m): w has decorrelated much
    // more than u, whose scale is ~ 4x longer.
    auto rho = [&](int comp, int lag) {
        Real num = 0, den = 0;
        for (int i = 0; i + lag < n; ++i) {
            const Real a[3] = {u[i].x, u[i].y, u[i].z}, b[3] = {u[i + lag].x, u[i + lag].y, u[i + lag].z};
            num += a[comp] * b[comp];
            den += a[comp] * a[comp];
        }
        return num / den;
    };
    const int lag = static_cast<int>(std::lround(st.length.z / (dx * 0.7 * std::sqrt(2.0))));
    CHECK(rho(0, lag) > rho(2, lag) + 0.15);
}

TEST_CASE("turbulence: gradient is consistent with the field, including the height dependence") {
    SimpleWeather::Params p;
    p.windSpeed = 9;
    p.windFromDirection = degToRad(130);
    p.turbulence.severity = TurbulenceSeverity::Light;
    const SimpleWeather w(p);
    for (Real h : {6.0, 25.0, 120.0, 450.0, 1200.0}) CHECK(gradientError(w, {137, -59, -h}, 0, 3.5) < 2e-3);
}

TEST_CASE("turbulence: frozen, shared and advected with the wind") {
    SimpleWeather::Params p;
    p.windSpeed = 10;
    p.windFromDirection = kPi / 2;  // from the east: air moves west (-y)
    const SimpleWeather a(p), b(p);
    const Vec3 x{100, 200, -50};
    CHECK(a.sample(x, 50, 7).wind == b.sample(x, 50, 7).wind);
    // Taylor's hypothesis: the gust at x at t=0 is found 10 m/s x 3 s
    // downwind at t=3 (plus the mean-wind difference, none at equal height).
    const Vec3 adv = a.meanWindAt(p.referenceHeight);
    const Vec3 w0 = a.sample(x, 50, 0).wind, w3 = a.sample(x + adv * 3.0, 50, 3).wind;
    CHECK_NEAR(length(w0 - w3), 0, 1e-9);
    // Rougher surface, same 10 m wind: stronger turbulence.
    SimpleWeather::Params city = p;
    city.surfaceRoughness = 1.0;
    CHECK(SimpleWeather(city).turbulence().statistics(50, 50).sigma.z >
          2 * a.turbulence().statistics(50, 50).sigma.z);
    // Mechanical turbulence can be switched off for steady-wind studies.
    SimpleWeather::Params steady = p;
    steady.turbulence.mechanical = false;
    CHECK(!SimpleWeather(steady).turbulence().active());
}
