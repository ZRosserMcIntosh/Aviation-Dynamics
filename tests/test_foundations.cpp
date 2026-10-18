// Aviation Dynamics — foundation tests: math, geodesy, atmosphere, mass, tables.
// SPDX-License-Identifier: MIT
#include "avd/atmosphere.hpp"
#include "avd/geodesy.hpp"
#include "avd/mass.hpp"
#include "avd/table.hpp"
#include "avd/units.hpp"
#include "avd/world_frame.hpp"
#include "avd_test.hpp"

using namespace avd;
using namespace avd::units;

// ---------------------------------------------------------------- math ----

TEST_CASE("math: euler angles round-trip through the quaternion") {
    for (Real r : {-2.5, -0.3, 0.0, 0.7, 3.0})
        for (Real p : {-1.2, -0.2, 0.0, 0.5, 1.3})
            for (Real y : {-3.0, -1.0, 0.0, 1.5, 3.1}) {
                const Quat q = Quat::fromEuler(r, p, y);
                CHECK_NEAR(q.roll(), r, 1e-9);
                CHECK_NEAR(q.pitch(), p, 1e-9);
                CHECK_NEAR(q.yaw(), y, 1e-9);
            }
}

TEST_CASE("math: aerospace rotation conventions (FRD body, NED)") {
    // Yaw 90 deg: body nose (x) points east.
    const Vec3 nose = Quat::fromEuler(0, 0, kPi / 2).rotate({1, 0, 0});
    CHECK_NEAR(nose.x, 0, 1e-12);
    CHECK_NEAR(nose.y, 1, 1e-12);
    // Pitch up 30 deg: nose has a negative down component.
    const Vec3 up = Quat::fromEuler(0, kPi / 6, 0).rotate({1, 0, 0});
    CHECK_NEAR(up.z, -0.5, 1e-12);
    // Roll right 90 deg: right wing (y) points down.
    const Vec3 wing = Quat::fromEuler(kPi / 2, 0, 0).rotate({0, 1, 0});
    CHECK_NEAR(wing.z, 1, 1e-12);
    // Matrix agrees with rotate().
    const Quat q = Quat::fromEuler(0.3, -0.4, 2.0);
    const Vec3 v{0.2, -1.3, 0.7};
    const Vec3 a = q.rotate(v), b = q.toMatrix() * v;
    CHECK_NEAR(a.x, b.x, 1e-12);
    CHECK_NEAR(a.y, b.y, 1e-12);
    CHECK_NEAR(a.z, b.z, 1e-12);
}

TEST_CASE("math: quaternion kinematics integrate a constant body rate exactly") {
    Quat q;
    const Vec3 w{0.0, 0.0, 0.5};  // yaw rate
    const Real dt = 0.001;
    for (int i = 0; i < 2000; ++i) {  // RK4 on dq/dt = 0.5 q (x) w
        const Quat k1 = quatDerivative(q, w);
        const Quat k2 = quatDerivative(q + k1 * (dt / 2), w);
        const Quat k3 = quatDerivative(q + k2 * (dt / 2), w);
        const Quat k4 = quatDerivative(q + k3 * dt, w);
        q = q + (k1 + k2 * 2 + k3 * 2 + k4) * (dt / 6);
        q.normalize();
    }
    CHECK_NEAR(q.yaw(), 1.0, 1e-9);
}

TEST_CASE("math: matrix inverse and unit conversions") {
    const Mat3 m = inertiaTensor(1285, 1825, 2667, 50);
    const Mat3 i = m * m.inverse();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) CHECK_NEAR(i.m[r][c], r == c ? 1.0 : 0.0, 1e-12);
    CHECK_NEAR(ktToMs(1), 0.514444, 1e-6);
    CHECK_NEAR(ftToM(1000), 304.8, 1e-9);
    CHECK_NEAR(lbToKg(2550), 1156.66, 0.01);
    CHECK_NEAR(galToM3(1) * 1000, 3.785411784, 1e-9);
    CHECK_NEAR(inHgToPa(29.92), 101320.8, 1.0);
    CHECK_NEAR(hpToW(180), 134225.98, 0.05);
}

TEST_CASE("table: interpolation, end policy and validation") {
    const Table1D t({0, 1, 3}, {0, 10, 30});
    CHECK_NEAR(t(0.5), 5, 1e-12);
    CHECK_NEAR(t(2), 20, 1e-12);
    CHECK_NEAR(t(-5), 0, 1e-12);
    CHECK_NEAR(t(9), 30, 1e-12);
    CHECK(t.validate().empty());
    CHECK(!Table1D({0, 0}, {1, 2}).validate().empty());
    CHECK(!Table1D({0, 1}, {1}).validate().empty());
    const Table2D t2({0, 1}, {0, 10}, {0, 10, 1, 11});
    CHECK_NEAR(t2(0.5, 5), 5.5, 1e-12);
    CHECK(t2.validate().empty());
}

// ------------------------------------------------------------- geodesy ----

TEST_CASE("geodesy: WGS-84 round trip and known points") {
    const Geodetic sp{degToRad(-23.5611), degToRad(-46.6558), 812.0};
    const Geodetic back = wgs84::ecefToGeodetic(wgs84::geodeticToEcef(sp));
    CHECK_NEAR(back.latitude, sp.latitude, 1e-12);
    CHECK_NEAR(back.longitude, sp.longitude, 1e-12);
    CHECK_NEAR(back.height, sp.height, 1e-6);
    const Vec3 eq = wgs84::geodeticToEcef({0, 0, 0});
    CHECK_NEAR(eq.x, 6378137.0, 1e-6);
    // Normal gravity: equator 9.7803 m/s^2, pole 9.8322 m/s^2.
    CHECK_NEAR(wgs84::normalGravity(0, 0), 9.7803253, 1e-6);
    CHECK_NEAR(wgs84::normalGravity(kPi / 2, 0), 9.8321850, 1e-6);
    // Free-air gradient ~ -3.086e-6 /s^2 per m.
    CHECK_NEAR(wgs84::normalGravity(0.4, 1000) - wgs84::normalGravity(0.4, 0), -3.086e-3, 2e-5);
}

TEST_CASE("geodesy: local NED frame geometry and rebasing rotation") {
    const LocalFrame f({degToRad(-23.5611), degToRad(-46.6558), 0});
    // 1 km north along the meridian.
    const Geodetic north = f.geodeticFromNed({1000, 0, 0});
    CHECK(north.latitude > f.origin().latitude);
    const Vec3 back = f.nedFromGeodetic(north);
    CHECK_NEAR(back.x, 1000, 1e-6);
    CHECK_NEAR(back.y, 0, 1e-6);
    // Earth curvature: a point 20 km away on the ellipsoid is ~31 m below the tangent plane.
    const Vec3 far = f.nedFromGeodetic(f.geodeticFromNed({20000, 0, 0}));
    CHECK_NEAR(far.z, 0, 1e-6);  // round trip exact
    Geodetic g = f.geodeticFromNed({20000, 0, 0});
    g.height = 0;
    CHECK_RANGE(f.nedFromGeodetic(g).z, 28.0, 34.0);
    // rotationTo is orthonormal.
    const LocalFrame f2({degToRad(-27.6), degToRad(-48.5), 0});  // Florianópolis
    const Mat3 r = f.rotationTo(f2);
    const Mat3 rrt = r * r.transposed();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) CHECK_NEAR(rrt.m[i][j], i == j ? 1.0 : 0.0, 1e-12);
    CHECK_NEAR(r.determinant(), 1.0, 1e-12);
    const Quat q = quatFromMatrix(r);
    const Vec3 v{1, 2, 3};
    const Vec3 a = r * v, b = q.rotate(v);
    CHECK_NEAR(a.x, b.x, 1e-12);
    CHECK_NEAR(a.y, b.y, 1e-12);
    CHECK_NEAR(a.z, b.z, 1e-12);
}

TEST_CASE("world frame: NED/ENU and FRD/FLU conversions; São Paulo tiles") {
    const Vec3 ned{100, 200, -50};
    const Vec3 enu = world::enuFromNed(ned);
    CHECK_NEAR(enu.x, 200, 0);
    CHECK_NEAR(enu.y, 100, 0);
    CHECK_NEAR(enu.z, 50, 0);
    CHECK(world::nedFromEnu(enu) == ned);
    // Quaternion conversion agrees with the vector conversions.
    const Quat q = Quat::fromEuler(0.2, -0.3, 1.1);
    const Quat qe = world::enuFluFromNedFrd(q);
    const Vec3 vFrd{0.3, 0.5, -0.2};
    const Vec3 a = qe.rotate(world::fluFromFrd(vFrd));
    const Vec3 b = world::enuFromNed(q.rotate(vFrd));
    CHECK_NEAR(a.x, b.x, 1e-12);
    CHECK_NEAR(a.y, b.y, 1e-12);
    CHECK_NEAR(a.z, b.z, 1e-12);
    // MASP is tile T_0_0; Catedral da Sé (2193, 1161) is tile T_p4_p2.
    CHECK(world::tileIndex(-1) == 0);
    CHECK(world::tileIndex(2193) == 4);
    CHECK(world::tileIndex(1161) == 2);
    CHECK(world::tileIndex(-4320) == -9);
}

// ---------------------------------------------------------- atmosphere ----

TEST_CASE("atmosphere: U.S. Standard Atmosphere 1976 reference values") {
    const StandardAtmosphere a;
    const AirState sl = a.at(0);
    CHECK_NEAR(sl.pressure, 101325.0, 1e-6);
    CHECK_NEAR(sl.temperature, 288.15, 1e-9);
    CHECK_NEAR(sl.density, 1.225, 1e-4);
    CHECK_NEAR(sl.speedOfSound, 340.294, 1e-3);
    CHECK_NEAR(sl.dynamicViscosity, 1.7894e-5, 1e-8);
    // Geopotential 11 km (tropopause), 20 km, 32 km (USSA-1976 tables).
    const Real z11 = isa::geometricFromGeopotential(11000);
    CHECK_NEAR(a.at(z11).temperature, 216.65, 1e-9);
    CHECK_NEAR(a.at(z11).pressure, 22632.06, 0.05);
    CHECK_NEAR(a.at(z11).density, 0.363918, 2e-5);
    CHECK_NEAR(a.at(z11).speedOfSound, 295.070, 2e-3);
    CHECK_NEAR(a.at(isa::geometricFromGeopotential(20000)).pressure, 5474.889, 0.02);
    CHECK_NEAR(a.at(isa::geometricFromGeopotential(32000)).pressure, 868.019, 0.01);
    CHECK_NEAR(a.at(isa::geometricFromGeopotential(47000)).pressure, 110.906, 0.01);
    // Geometric 5000 m (table value 54048 Pa, 255.68 K, 0.73643 kg/m^3).
    CHECK_NEAR(a.at(5000).pressure, 54048, 5);
    CHECK_NEAR(a.at(5000).temperature, 255.68, 0.01);
    CHECK_NEAR(a.at(5000).density, 0.73643, 2e-4);
}

TEST_CASE("atmosphere: pressure and density altitude") {
    const StandardAtmosphere isaDay;
    for (Real z : {0.0, 1500.0, 8000.0, 11000.0, 15000.0, 25000.0}) {
        const AirState s = isaDay.at(z);
        CHECK_NEAR(s.pressureAltitude, z, 0.01);
        CHECK_NEAR(s.densityAltitude, z, 0.01);
    }
    // ISA+20 at sea level: density altitude ~ 2,300-2,400 ft (rule of thumb 120 ft/degC).
    AtmosphereConditions hot;
    hot.temperatureDeviation = 20;
    const AirState h = StandardAtmosphere(hot).at(0);
    CHECK_NEAR(mToFt(h.densityAltitude), 2350, 120);
    CHECK_NEAR(h.pressureAltitude, 0, 0.01);
    // Low QNH raises pressure altitude at the field: 1000 hPa vs 1013.25 hPa ~ 110 m.
    AtmosphereConditions low;
    low.seaLevelPressure = 100000;
    CHECK_NEAR(StandardAtmosphere(low).at(0).pressureAltitude, 110.9, 1.0);
}

TEST_CASE("atmosphere: humidity reduces density by its physical contribution only") {
    CHECK_NEAR(saturationVapourPressure(celsiusToK(20)), 2338.8, 2.0);
    CHECK_NEAR(saturationVapourPressure(celsiusToK(30)), 4245, 10);
    AtmosphereConditions dry, wet;
    dry.temperatureDeviation = wet.temperatureDeviation = 15;  // 30 degC at sea level
    wet.relativeHumidity = 1.0;
    const Real rd = StandardAtmosphere(dry).at(0).density;
    const Real rw = StandardAtmosphere(wet).at(0).density;
    // Ideal-gas mixture: (1 - 0.378 e/p) ~ 1.6% lighter at 30 degC saturated.
    CHECK_NEAR((rd - rw) / rd, 0.378 * 4245 / 101325, 0.001);
    CHECK(StandardAtmosphere(wet).at(0).speedOfSound > StandardAtmosphere(dry).at(0).speedOfSound);
    // Humidity does not change pressure.
    CHECK_NEAR(StandardAtmosphere(wet).at(0).pressure, StandardAtmosphere(dry).at(0).pressure, 1e-9);
}

TEST_CASE("atmosphere: boundary-layer wind profile and direction convention") {
    SimpleWeather::Params p;
    p.windSpeed = 10;
    p.windFromDirection = 0;  // from the north
    const SimpleWeather w(p);
    const Vec3 v10 = w.meanWindAt(10);
    CHECK_NEAR(v10.x, -10, 1e-9);  // air moves south
    CHECK_NEAR(v10.y, 0, 1e-9);
    CHECK(w.meanWindAt(300).x < -14);  // stronger aloft (log law over z0 = 0.03 m: ~15.9 m/s)
    CHECK(w.meanWindAt(2).x > -10);
    const AirSample s = w.sample({0, 0, -100}, 100, 0);
    CHECK(s.windGradient.m[0][2] > 0);  // wind magnitude grows upward (-z)
}

TEST_CASE("atmosphere: world-space turbulence is coherent, divergence-free and calibrated") {
    TurbulenceField::Params tp;
    tp.intensity = 2.0;
    tp.seed = 42;
    const TurbulenceField f(tp);
    Real sum2[3] = {0, 0, 0}, mean[3] = {0, 0, 0}, maxDiv = 0;
    const int n = 4000;
    for (int i = 0; i < n; ++i) {
        const Vec3 x{i * 37.3, std::fmod(i * 91.7, 5000.0), -(i % 50) * 23.0};
        Vec3 u;
        Mat3 g;
        f.evaluate(x, u, g);
        sum2[0] += u.x * u.x; sum2[1] += u.y * u.y; sum2[2] += u.z * u.z;
        mean[0] += u.x; mean[1] += u.y; mean[2] += u.z;
        maxDiv = std::max(maxDiv, std::abs(g.m[0][0] + g.m[1][1] + g.m[2][2]));
    }
    const Real rms = std::sqrt((sum2[0] + sum2[1] + sum2[2]) / (3.0 * n));
    CHECK_NEAR(rms, 2.0, 0.6);
    CHECK_NEAR(mean[0] / n, 0, 0.4);
    CHECK(maxDiv < 1e-9);  // incompressible by construction
    // Deterministic and shared: same place, same air.
    Vec3 a, b;
    Mat3 ga, gb;
    f.evaluate({123, 456, -789}, a, ga);
    TurbulenceField(tp).evaluate({123, 456, -789}, b, gb);
    CHECK(a == b);
    // Gradient matches a central finite difference.
    Vec3 up, um;
    Mat3 gu;
    f.evaluate({123 + 1e-3, 456, -789}, up, gu);
    f.evaluate({123 - 1e-3, 456, -789}, um, gu);
    CHECK_NEAR((up.y - um.y) / 2e-3, ga.m[1][0], 1e-6);
}

// ---------------------------------------------------------------- mass ----

TEST_CASE("mass: parallel-axis aggregation, CG and inertia") {
    std::vector<MassItem> items;
    items.push_back({"a", 10, {1, 0, 0}, Mat3::zero()});
    items.push_back({"b", 10, {-1, 0, 0}, Mat3::zero()});
    const MassProperties p = combine(items);
    CHECK_NEAR(p.mass, 20, 1e-12);
    CHECK_NEAR(p.cg.x, 0, 1e-12);
    CHECK_NEAR(p.inertia.m[0][0], 0, 1e-12);   // on the x axis
    CHECK_NEAR(p.inertia.m[1][1], 20, 1e-12);  // 2 * 10 * 1^2
    CHECK_NEAR(p.inertia.m[2][2], 20, 1e-12);
    // Products of inertia for an offset mass.
    const Mat3 pi = pointInertia(2, {1, 0, 1});
    CHECK_NEAR(pi.m[0][2], -2, 1e-12);
    CHECK_NEAR(pi.m[1][1], 4, 1e-12);
    // Removing an item moves the CG toward the remaining mass.
    items.pop_back();
    CHECK_NEAR(combine(items).cg.x, 1, 1e-12);
}
