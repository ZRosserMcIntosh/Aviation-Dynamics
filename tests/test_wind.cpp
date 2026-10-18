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

// =========================================================== discrete events ==

TEST_CASE("gust: 1-cosine and ramp-and-hold shapes, frozen in the moving air") {
    DiscreteGust g;
    g.heading = 0;        // front travels north
    g.frontSpeed = 0;     // stationary front at the origin: air south of it gusts
    g.length = 100;       // H
    g.amplitude = {0, 0, -6};  // 6 m/s updraft
    CHECK_NEAR(g.velocity({10, 0, 0}, 1, {}).z, 0, 1e-12);       // ahead of the front
    CHECK_NEAR(g.velocity({-100, 0, 0}, 1, {}).z, -6, 1e-12);    // peak at s = H
    CHECK_NEAR(g.velocity({-50, 0, 0}, 1, {}).z, -3, 1e-12);
    CHECK_NEAR(g.velocity({-200.1, 0, 0}, 1, {}).z, 0, 1e-12);   // over after 2H
    CHECK_NEAR(g.velocity({-100, 0, 0}, -1, {}).z, 0, 1e-12);    // before it starts
    DiscreteGust r = g;
    r.shape = DiscreteGust::Shape::RampAndHold;
    r.holdLength = 500;
    CHECK_NEAR(r.velocity({-400, 0, 0}, 1, {}).z, -6, 1e-12);
    CHECK_NEAR(r.velocity({-650, 0, 0}, 1, {}).z, -3, 1e-12);
    // Frozen gust: carried by a 10 m/s northerly-moving airmass.
    DiscreteGust f = g;
    f.frontSpeed = -1;
    CHECK_NEAR(f.velocity({-100 + 50, 0, 0}, 5, {10, 0, 0}).z, -6, 1e-12);
}

TEST_CASE("microburst: Oseguera-Bowles outflow, downdraft, continuity and life cycle") {
    Microburst m;
    m.radius = 1000;
    m.peakOutflow = 15;
    m.startTime = 0;
    m.growthTime = 60;
    m.matureTime = 300;
    m.decayTime = 120;
    const Real zm = m.peakOutflowHeight();
    CHECK_RANGE(zm, 80, 110);  // observed outflow maxima are ~50-100 m AGL
    // Peak radial wind at r = 1.12 R, z = z_m equals the specification, and
    // is the maximum over a scan.
    const Real t = 100;
    CHECK_NEAR(m.velocity({1120.9, 0, 0}, zm, t).x, 15, 0.001);
    Real scanMax = 0;
    for (Real r = 0; r < 4000; r += 50)
        for (Real z = 5; z < 600; z += 5) scanMax = std::max(scanMax, m.velocity({r, 0, 0}, z, t).x);
    CHECK(scanMax <= 15.0001);
    // Outflow points away from the core; downdraft (NED +z) in the core,
    // vanishing at the ground.
    CHECK(m.velocity({0, 1500, 0}, 50, t).y > 5);
    CHECK(m.velocity({-1500, 0, 0}, 50, t).x < -5);
    CHECK(m.velocity({0, 0, 0}, 300, t).z > 5);
    CHECK_NEAR(m.velocity({0, 0, 0}, 0, t).z, 0, 1e-12);
    // Continuity: the sampled gradient is trace-free (incompressible).
    SimpleWeather::Params p;
    p.microbursts.push_back(m);
    const SimpleWeather w(p);
    for (const Vec3& x : {Vec3{300, 400, -60}, Vec3{-900, 200, -150}, Vec3{1500, -100, -20}}) {
        const Mat3 g = w.sample(x, -x.z, t).windGradient;
        CHECK(std::abs(g.m[0][0] + g.m[1][1] + g.m[2][2]) < 1e-5);
        CHECK(gradientError(w, x, 0, t) < 1e-4);
    }
    // Life cycle: nothing before, full when mature, gone after.
    CHECK_NEAR(m.strength(-1), 0, 1e-12);
    CHECK_NEAR(m.strength(30), 0.5, 1e-12);
    CHECK_NEAR(m.strength(200), 1, 1e-12);
    CHECK_NEAR(m.strength(1000), 0, 1e-12);
    // Drifting core.
    Microburst d = m;
    d.driftEast = 5;
    CHECK_NEAR(d.velocity({0, 500, 0}, 300, 100).z, m.velocity({0, 0, 0}, 300, 100).z, 1e-9);
}

TEST_CASE("microburst: hazardous wind-shear F-factor on a low-level crossing") {
    // TDWR/FAA hazard index along a straight path at 300 ft through the core
    // of a 15 m/s microburst at 55 m/s airspeed: F > 0.1 is hazardous.
    Microburst m;
    m.peakOutflow = 15;
    const Real v = 55, z = 300 * kFoot, dx = 5;
    Real fMax = 0;
    for (Real x = -3000; x < 3000; x += dx) {
        // Flying north: tailwind = +x air velocity.
        const Real wx0 = m.velocity({x, 0, 0}, z, 100).x, wx1 = m.velocity({x + dx, 0, 0}, z, 100).x;
        const Real up = -m.velocity({x, 0, 0}, z, 100).z;
        fMax = std::max(fMax, windShearHazardFactor((wx1 - wx0) / dx * v, up, v));
    }
    CHECK(fMax > 0.1);
    // A weak one (5 m/s) stays below the alert threshold.
    Microburst weak = m;
    weak.peakOutflow = 5;
    Real fWeak = 0;
    for (Real x = -3000; x < 3000; x += dx) {
        const Real wx0 = weak.velocity({x, 0, 0}, z, 100).x, wx1 = weak.velocity({x + dx, 0, 0}, z, 100).x;
        fWeak = std::max(fWeak, windShearHazardFactor((wx1 - wx0) / dx * v, -weak.velocity({x, 0, 0}, z, 100).z, v));
    }
    CHECK(fWeak < 0.1);
}
