// Aviation Dynamics — wind and turbulence tests: boundary layer, MIL-HDBK-1797
// turbulence statistics, field consistency.
// SPDX-License-Identifier: MIT
#include "airports.hpp"
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

// ============================================================== vertical air ==

TEST_CASE("thermals: Allen profile, zero net mass flux, exact continuity") {
    ThermalParams tp;
    tp.convectiveVelocity = 2.5;
    tp.depth = 1500;
    tp.field = false;
    tp.thermals.push_back({0, 0, 1.0, -1e9, 0});
    const ThermalField f(tp, {});
    const Real z = 600;
    // Allen: w_avg = w* (z/zi)^(1/3) (1 - 1.1 z/zi), rounded at the surface only.
    CHECK_NEAR(f.meanUpdraft(z), 2.5 * std::cbrt(0.4) * (1 - 0.44), 0.01);
    CHECK_NEAR(f.meanUpdraft(1400), 0, 1e-12);
    // Mean updraft over r < r2 reproduces w_avg; zero at r2; sink outside.
    const Real r2 = f.outerRadius(z);
    Real flux = 0, area = 0, fluxAll = 0;
    const int n = 400;
    for (int i = 0; i < n; ++i) {
        const Real r = (i + 0.5) * 6 * r2 / n, dr = 6 * r2 / n;
        const Real w = -f.velocity({r, 0, 0}, z, 0).z;
        fluxAll += w * kTwoPi * r * dr;
        if (r < r2) { flux += w * kTwoPi * r * dr; area += kTwoPi * r * dr; }
    }
    CHECK_NEAR(flux / area, f.meanUpdraft(z), 0.01 * f.meanUpdraft(z));
    CHECK_NEAR(-f.velocity({r2, 0, 0}, z, 0).z, 0, 1e-6);
    CHECK(-f.velocity({1.6 * r2, 0, 0}, z, 0).z < -0.05);
    CHECK(std::abs(fluxAll) < 0.002 * flux);  // the sink ring returns the air
    CHECK_RANGE(-f.velocity({0, 0, 0}, z, 0).z / f.meanUpdraft(z), 2.5, 3.2);
    // Inflow below, outflow aloft.
    CHECK(f.velocity({80, 0, 0}, 100, 0).x < 0);
    CHECK(f.velocity({80, 0, 0}, 1300, 0).x > 0);
    // Continuity (central-difference gradient of the field itself).
    for (const Vec3& x : {Vec3{30, 40, -300}, Vec3{-90, 10, -800}, Vec3{0.001, 0, -500}, Vec3{150, -20, -50}}) {
        Vec3 v;
        Mat3 g;
        sampleWithGradient([&](const Vec3& y) { return f.velocity(y, -y.z, 0); }, x, v, g, 0.01);
        CHECK(std::abs(g.m[0][0] + g.m[1][1] + g.m[2][2]) < 1e-4);
    }
}

TEST_CASE("thermals: random field -- strong cores, mass balance, drift and life cycle") {
    SimpleWeather::Params p;
    p.windSpeed = 6;
    p.windFromDirection = kPi;  // from the south: air moves north
    p.thermals.convectiveVelocity = 2.0;
    p.thermals.depth = 1500;
    const SimpleWeather w(p);
    CHECK(w.turbulence().params().convectiveVelocity == 2.0);  // convective turbulence comes along
    const ThermalField& f = w.thermals();
    // A 60 km traverse at 0.5 zi: strong cores, net vertical flux ~ 0.
    Real sum = 0, maxUp = 0;
    int up = 0;
    const int n = 60000;
    for (int i = 0; i < n; ++i) {
        const Real wz = -f.velocity({i * 1.0, 0.37 * i, 0}, 750, 300).z;
        sum += wz;
        maxUp = std::max(maxUp, wz);
        up += wz > 0.5;
    }
    CHECK(maxUp > 2.0);  // cores of ~ w* and more
    CHECK(std::abs(sum / n) < 0.05);
    CHECK_RANGE(static_cast<Real>(up) / n, 0.02, 0.3);  // lift covers a minority of the sky
    // Thermals live and die: at a fixed point of the drifting airmass the
    // field changes over a lifetime.
    const Vec3 drift = w.boundaryLayer().wind(750, 750);
    CHECK(drift.x > 6);
    Real change = 0;
    for (int i = 0; i < 200; ++i) {
        const Vec3 y{i * 37.0, i * 11.0, 0};
        change += std::abs(f.velocity(y, 750, 0).z - f.velocity(y + drift * 600.0, 750, 600).z);
    }
    CHECK(change > 1.0);
    // A steady explicit thermal is carried with the drift, unchanged.
    ThermalParams tp = p.thermals;
    tp.field = false;
    tp.thermals.push_back({0, 0, 1.0, -1e9, 0});
    const ThermalField one(tp, drift);
    CHECK_NEAR(length(one.velocity({20, 30, 0}, 750, 0) - one.velocity(Vec3{20, 30, 0} + drift * 90.0, 750, 90)), 0, 1e-9);
}

TEST_CASE("ridge: slope lift at the surface, lee sink, mountain wave aloft") {
    Ridge r;
    r.axis = kPi / 2;   // crest runs east-west
    r.height = 400;
    r.halfWidth = 1500;
    r.stability = 0.01;
    r.length = 1e6;
    const Vec3 wind{10, 0, 0};  // 10 m/s from the south (air moves north)
    const Real a = r.halfWidth, h0 = r.height;
    // At the surface the flow follows the terrain: w = U dh/dx.
    for (Real xc : {-3000.0, -1000.0, -200.0, 500.0, 2500.0}) {
        const Real dhdx = -2 * h0 * a * a * xc / sq(xc * xc + a * a);
        CHECK_NEAR(-r.velocity({xc, 0, 0}, 0, wind).z, 10 * dhdx, 1e-9);
    }
    CHECK(-r.velocity({-1000, 0, 0}, 0, wind).z > 1.5);  // windward lift
    CHECK(-r.velocity({1000, 0, 0}, 0, wind).z < -1.5);  // lee sink
    // Vertical wavelength 2 pi U / N: the pattern repeats.
    const Real lz = kTwoPi * 10 / 0.01;
    for (Real xc : {-2000.0, 0.0, 1800.0})
        CHECK_NEAR(r.velocity({xc, 0, 0}, 700, wind).z, r.velocity({xc, 0, 0}, 700 + lz, wind).z, 1e-9);
    // Half a vertical wavelength up the pattern is inverted: the lift sits
    // over the lee slope (wave lift), the sink over the windward one.
    CHECK_NEAR(-r.velocity({1000, 0, 0}, lz / 2, wind).z, -(-r.velocity({1000, 0, 0}, 0, wind).z), 1e-9);
    CHECK(-r.velocity({1000, 0, 0}, lz / 2, wind).z > 1.5);
    // The phase tilts upwind with height: at a quarter wavelength the
    // streamlines are displaced up upwind of the crest, down over it.
    CHECK(-r.velocity({-2 * a, 0, 0}, lz / 4, wind).z > 0);
    CHECK(-r.velocity({0, 0, 0}, lz / 4, wind).z < -1.5);
    // 2-D continuity.
    for (const Vec3& x : {Vec3{-800, 0, -300}, Vec3{1200, 0, -2500}}) {
        Vec3 v;
        Mat3 g;
        sampleWithGradient([&](const Vec3& y) { return r.velocity(y, -y.z, wind); }, x, v, g, 0.01);
        CHECK(std::abs(g.m[0][0] + g.m[1][1] + g.m[2][2]) < 1e-6);
    }
    // Wind along the crest, a calm day, or high above the absorbing layer: nothing.
    CHECK(length(r.velocity({-1000, 0, 0}, 0, {0, 10, 0})) < 1e-12);
    CHECK(length(r.velocity({-1000, 0, 0}, 0, {0.5, 0, 0})) < 1e-12);
    CHECK(length(r.velocity({-1000, 0, 0}, 12000, wind)) < 1e-12);
    // Wind from the other side: lift moves to the other slope.
    CHECK(-r.velocity({1000, 0, 0}, 0, {-10, 0, 0}).z > 1.5);
    // Strongly stable, weak wind: blocked, the height is limited (N h0 / U <= 0.8).
    Ridge b = r;
    b.stability = 0.03;
    CHECK(std::abs(r.velocity({-1000, 0, 0}, 0, {4, 0, 0}).z) > std::abs(b.velocity({-1000, 0, 0}, 0, {4, 0, 0}).z));
}

// ========================================================= spatial variation ==

TEST_CASE("spatial: a city around the airfield -- weaker low wind, stronger turbulence, same wind aloft") {
    SimpleWeather::Params p;
    p.windSpeed = 10;                      // reported at the airfield
    p.surfaceRoughness = 0.03;
    p.roughnessPatches.push_back({0, 0, 20000, 500, 1.0});   // city
    p.roughnessPatches.push_back({0, 0, 800, 200, 0.03});    // the airfield inside it
    const SimpleWeather w(p);
    CHECK_NEAR(w.roughnessAt(0, 0), 0.03, 1e-12);
    CHECK_NEAR(w.roughnessAt(5000, 0), 1.0, 1e-9);
    const Real airfield10 = speedOf(w.meanWindAt({0, 0, -10}, 10, 0));
    const Real city10 = speedOf(w.meanWindAt({5000, 0, -10}, 10, 0));
    CHECK_NEAR(airfield10, 10, 1e-9);
    // Same wind at the top of the layer, log profiles below it.
    const Real expected = std::log(600 / 0.03) / std::log(10 / 0.03) * std::log(10 / 1.0) / std::log(600 / 1.0) * 10;
    CHECK_NEAR(city10, expected, 1e-9);
    CHECK_NEAR(speedOf(w.meanWindAt({5000, 0, -600}, 600, 0)), speedOf(w.meanWindAt({0, 0, -600}, 600, 0)), 1e-9);
    // City friction velocity -- and so the mechanical turbulence -- is larger.
    const BoundaryLayer& bl = w.boundaryLayer();
    CHECK(bl.frictionVelocity({10, 0, 1.0}) > 1.4 * bl.frictionVelocity({10, 0, 0.03}));
    Real s2City = 0, s2Field = 0;
    for (int i = 0; i < 4000; ++i) {
        const Vec3 c{5000 + i * 3.1, i * 1.7, -40}, f{i * 0.1, i * 0.13, -40};
        s2City += sq(w.sample(c, 40, 0).wind.z - w.meanWindAt(c, 40, 0).z);
        s2Field += sq(w.sample(f, 40, 0).wind.z - w.meanWindAt(f, 40, 0).z);
    }
    CHECK(s2City > 1.5 * s2Field);
    CHECK(gradientError(w, {700, 30, -25}, 0, 0) < 3e-3);  // across the airfield edge
}

TEST_CASE("spatial: wind stations interpolate in space and time") {
    SimpleWeather::Params p;
    p.turbulence.mechanical = false;
    p.profile = WindProfile::Uniform;
    p.stationInfluence = 1000;
    WindStation a{0, 0, {0, 600}, {5, 15}, {degToRad(350), degToRad(20)}};
    WindStation b{20000, 0, {0}, {10}, {degToRad(180)}};
    p.stations = {a, b};
    const SimpleWeather w(p);
    // At a station its own report dominates.
    const SurfaceWind s0 = w.surfaceWindAt(0, 0, 0);
    CHECK_NEAR(s0.speed, 5, 0.5);
    // Time: halfway through, 10 m/s from 005 (through north).
    Real sp, from;
    a.at(300, sp, from);
    CHECK_NEAR(sp, 10, 1e-9);
    CHECK_NEAR(radToDeg(wrapPi(from)), 5, 1e-9);
    a.at(1e6, sp, from);
    CHECK_NEAR(sp, 15, 1e-9);
    // Space: near b the southerly wins.
    CHECK_NEAR(radToDeg(wrapPi(w.surfaceWindAt(20000, 0, 0).fromDirection - kPi)), 0, 3);
    // Smooth in between: gradient consistent.
    CHECK(gradientError(w, {9000, 400, -50}, 0, 200) < 1e-4);
}

TEST_CASE("spatial: a gust front passes -- wind shift, depth, nose updraft") {
    SimpleWeather::Params p;
    p.turbulence.mechanical = false;
    p.windSpeed = 4;
    p.windFromDirection = degToRad(90);   // light easterly ahead of it
    WindShift f;
    f.north = -5000;                      // 5 km south at t = 0, moving north at 10 m/s
    f.heading = 0;
    f.speed = 10;
    f.newSpeed = 18;
    f.newFromDirection = degToRad(200);
    f.depth = 800;
    f.noseUpdraft = 3;
    p.windShifts.push_back(f);
    const SimpleWeather w(p);
    const Vec3 site{0, 0, -10};
    CHECK_NEAR(speedOf(w.meanWindAt(site, 10, 0)), 4, 0.01);         // before
    CHECK_NEAR(speedOf(w.meanWindAt(site, 10, 1000)), 18, 0.05);     // after (passes at 500 s)
    CHECK_NEAR(radToDeg(fromDirection(w.meanWindAt(site, 10, 1000))) + 360, 200, 1);
    // Above the outflow the old wind continues.
    CHECK_NEAR(fromDirection(w.meanWindAt({0, 0, -1500}, 1500, 1000)), degToRad(90), 1e-6);
    // The head lifts the air just ahead of the front.
    Real maxUp = 0;
    for (Real t = 400; t < 600; t += 2) maxUp = std::max(maxUp, -w.meanWindAt({0, 0, -400}, 400, t).z);
    CHECK_RANGE(maxUp, 2.5, 3.01);
    // Stronger wind behind it: more mechanical turbulence there (u*).
    SimpleWeather::Params q = p;
    q.turbulence.mechanical = true;
    const SimpleWeather wq(q);
    CHECK(wq.turbulence().active());
    CHECK(gradientError(w, {200, 30, -60}, 0, 495) < 1e-3);
}

TEST_CASE("spatial: a hangar's wake -- deficit and turbulence downwind only") {
    // 20 x 40 m hangar, 12 m tall, 10 m/s westerly (air moves east).
    const Obstacle h = Obstacle::fromFootprint({{-20, -10}, {20, -10}, {20, 10}, {-20, 10}}, 0, 12);
    CHECK_NEAR(h.east, 0, 1e-9);
    CHECK_NEAR(h.north, 0, 1e-9);
    CHECK_NEAR(h.width, 2 * std::sqrt(800 / kPi), 1e-9);
    CHECK_NEAR(h.height, 12, 1e-12);
    SimpleWeather::Params p;
    p.windSpeed = 10;
    p.windFromDirection = degToRad(270);
    p.turbulence.mechanical = false;
    SimpleWeather::Params q = p;
    q.obstacles.push_back(h);
    const SimpleWeather open(p), withHangar(q);
    auto east = [&](const SimpleWeather& w, Real e, Real n, Real z) { return w.meanWindAt({n, e, -z}, z, 0).y; };
    const Real free6 = east(open, 30, 0, 6);
    CHECK(east(withHangar, 30, 0, 6) < 0.3 * free6);           // in the cavity behind it
    CHECK_NEAR(east(withHangar, -60, 0, 6), free6, 1e-9);       // upwind: untouched
    CHECK_NEAR(east(withHangar, 30, 80, 6), free6, 0.02 * free6);  // to the side: almost untouched
    CHECK(east(withHangar, 240, 0, 6) > 0.85 * free6);         // 20 H: recovered
    CHECK(east(withHangar, 30, 0, 6) < east(withHangar, 90, 0, 6));
    Real extra;
    h.velocity({0, 30, 0}, 6, {0, 10, 0}, extra);
    CHECK(extra > 1.0);  // gusty in the wake
    CHECK(gradientError(withHangar, {3, 40, -5}, 0, 0) < 1e-3);
}

TEST_CASE("spatial: Campo de Marte -- airfield report, city around it, Cantareira lift in a southerly") {
    const LocalFrame frame = scenarios::saoPauloWorldFrame();
    const scenarios::Airport sbmt = scenarios::campoDeMarte(frame);
    SimpleWeather::Params p;
    scenarios::addSaoPauloWindEnvironment(p, sbmt);
    p.windSpeed = 8;
    p.windFromDirection = kPi;  // southerly: onto the ridge's south face
    p.turbulence.mechanical = false;
    const SimpleWeather w(p);
    const RunwayDef& rw = sbmt.runways[0];
    const Vec3 field = scenarios::runwayPoint(rw, 0.5 * rw.length);
    const Real e = sbmt.elevation;
    auto at = [&](const Vec3& c, Real agl) { return w.meanWindAt({c.x, c.y, field.z - agl}, e + agl, 0); };
    CHECK_NEAR(speedOf(at(field, 10)), 8, 1e-6);                                   // the report
    CHECK(speedOf(at(field + Vec3{0, 6000, 0}, 10)) < 0.7 * 8);                  // the city, 6 km east
    // South face of the Cantareira (crest 10 km north): lift near the slope.
    const Real crestMsl = e + 30 + 350;
    const Vec3 slope = field + Vec3{10000 - 1500, 0, 0};
    const Real up = -w.sample({slope.x, slope.y, -(crestMsl - 150)}, crestMsl - 150, 0).wind.z;
    CHECK(up > 0.4);
}
