// Aviation Dynamics — tests: the open-world ground (terrain tiles, buildings).
// SPDX-License-Identifier: MIT
#include <cmath>
#include <sstream>
#include <string>

#include "airports.hpp"
#include "avd/units.hpp"
#include "avd/world_frame.hpp"
#include "avd/world_ground.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& ref() {
    static const AircraftDefinition d = loadReference();
    return d;
}

// Synthetic tile T_p0_p0 (centre at the world anchor): terrain rising 5 m per
// 100 m to the east from 760 m, a 20 x 20 m, 40 m tall building centred at
// (100, 0), asphalt in the western half. Test data, not São Paulo.
std::string syntheticTile(int i = 0, int j = 0, const std::string& extra = "") {
    const int n = 11;
    const Real size = 500, x0 = i * size - 250;
    std::ostringstream z;
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c) z << (r || c ? "," : "") << 760 + 0.05 * (x0 + c * 50.0 + 250);
    std::ostringstream s;
    s << R"({"format": "avd-world-tile/1", "tile": "T_test", "i": )" << i << R"(, "j": )" << j
      << R"(, "size_m": 500, "source": "synthetic test tile", "synthetic": true,
        "terrain": {"n": 11, "z_m": [)" << z.str() << R"(]},
        "default_surface": "grass",
        "surfaces": {"n": 2, "legend": ["asphalt", "grass"], "cells": [0, 1, 0, 1]},
        "buildings": [{"id": "block", "footprint": [[90, -10], [110, -10], [110, 10], [90, 10]], "height_m": 40}])"
      << extra << "}";
    return s.str();
}

Vec3 ned(Real east, Real north, Real up) { return world::nedFromEnu({east, north, up}); }

}  // namespace

TEST_CASE("world ground: tiles give terrain, surfaces, roofs and walls in the world frame") {
    TiledWorldGround g(700);
    const auto err = g.loadTileJson(syntheticTile());
    CHECK(err.empty());
    CHECK(g.tileCount() == 1);
    // Planar terrain is reproduced exactly by the bilinear heightfield.
    CHECK_NEAR(g.terrainHeight(-200, 30), 760 + 0.05 * 50, 1e-9);
    CHECK_NEAR(g.terrainHeight(0, 0), 772.5, 1e-9);
    CHECK(g.surfaceAt(-100, 100).id == surfaces::kAsphalt.id);
    CHECK(g.surfaceAt(100, 100).id == surfaces::kGrass.id);
    // Below a point over open terrain: the slope, normal tilted to the west.
    const GroundHit t = g.below(ned(0, 50, 900));
    CHECK(t.hit);
    CHECK_NEAR(-t.point.z, 772.5, 1e-6);
    const Vec3 nEnu = world::enuFromNed(t.normal);
    CHECK(nEnu.x < 0 && nEnu.z > 0.99);
    // Above the building: its roof (walls start at the lowest terrain under it).
    const Real base = g.terrainHeight(90, 0), roof = base + 40;
    const GroundHit r = g.below(ned(100, 0, roof + 20));
    CHECK_NEAR(-r.point.z, roof, 1e-6);
    CHECK(r.surface.id == surfaces::kBuilding.id);
    // Raycasts: straight down onto the roof, horizontally into the west wall.
    const GroundHit down = g.raycast(ned(100, 0, roof + 50), {0, 0, 1}, 200);
    CHECK(down.hit);
    CHECK_NEAR(down.distance, 50, 1e-6);
    const GroundHit wall = g.raycast(ned(0, 0, base + 10), world::nedFromEnu({1, 0, 0}), 500);
    CHECK(wall.hit);
    CHECK_NEAR(wall.distance, 90, 1e-6);
    CHECK_NEAR(world::enuFromNed(wall.normal).x, -1, 1e-9);  // faces the ray
    // A shallow ray into the rising terrain finds the slope.
    const GroundHit slope = g.raycast(ned(-240, 100, 800), normalized(world::nedFromEnu({1, 0, -0.1})), 2000);
    CHECK(slope.hit);
    CHECK_NEAR(-slope.point.z, g.terrainHeight(world::enuFromNed(slope.point).x, 100), 1e-3);
    // Inside the building: the nearest exit (west wall at 2 m).
    GroundHit c;
    CHECK(g.solidAt(ned(92, 0, base + 10), c));
    CHECK_NEAR(world::enuFromNed(c.point).x, 90, 1e-9);
    CHECK_NEAR(world::enuFromNed(c.normal).x, -1, 1e-9);
    CHECK(!g.solidAt(ned(92, 0, roof + 1), c));
    CHECK(!g.solidAt(ned(50, 0, base + 10), c));
    // Off the loaded tiles: the outside plane.
    CHECK_NEAR(-g.below(ned(5000, 5000, 900)).point.z, 700, 1e-9);
}

TEST_CASE("world ground: tile files are validated, with provenance required") {
    TiledWorldGround g;
    CHECK(!g.loadTileJson("{}").empty());
    CHECK(!g.loadTileJson(R"({"format": "avd-world-tile/1", "i": 0, "j": 0,
        "terrain": {"n": 2, "z_m": [0, 0, 0, 0]}})").empty());  // no source
    CHECK(!g.loadTileJson(R"({"format": "avd-world-tile/1", "i": 0, "j": 0, "source": "x",
        "terrain": {"n": 3, "z_m": [0, 0, 0, 0]}})").empty());  // wrong sample count
    CHECK(!g.loadTileJson(R"({"format": "avd-world-tile/1", "i": 0, "j": 0, "source": "x",
        "terrain": {"n": 2, "z_m": [0, 0, 0, 0]}, "default_surface": "lava"})").empty());
    CHECK(!g.loadTileJson(R"({"format": "avd-world-tile/1", "i": 0, "j": 0, "source": "x",
        "terrain": {"n": 2, "z_m": [0, 0, 0, 0]}, "buildings": [{"footprint": [[0, 0], [1, 0]], "height_m": 5}]})").empty());
    CHECK(g.tileCount() == 0);
    CHECK(g.loadTileJson(R"({"format": "avd-world-tile/1", "i": 0, "j": 0, "source": "x",
        "terrain": {"n": 2, "z_m": [0, 0, 0, 0]}})").empty());
    CHECK(g.tileCount() == 1);
}

TEST_CASE("world ground: an aircraft parks on sloping terrain and a wingtip strikes a building") {
    TiledWorldGround g(700);
    CHECK(g.loadTileJson(syntheticTile()).empty());
    SimpleWeather atm;
    {
        // Parked across a 5% slope with the parking brake set: it settles on
        // the gear (no hard-point contact) and does not slide away.
        Aircraft ac(ref(), saoPauloWorldFrame());
        const Vec3 p = ned(-150, 100, 0);
        ac.placeOnGround(p.x, p.y, 0, g, atm);  // nose north, slope rises to the east
        ac.setCockpitControl("brakes.parking", 1);
        PilotInput in;
        for (int i = 0; i < 6 * 240; ++i) ac.step(1.0 / 240, in, g, atm);
        const Vec3 moved = ac.datumPosition() - p;
        CHECK(std::hypot(moved.x, moved.y) < 0.5);
        CHECK(std::abs(ac.telemetry().roll) > degToRad(1.5));  // banked by the slope
        bool hardPoint = false;
        for (const FlightEvent& e : ac.drainEvents()) hardPoint |= e.type == FlightEventType::HardPointContact;
        CHECK(!hardPoint);
    }
    {
        // Flying east at 70 kt, 20 m above the building's base, fuselage 3 m
        // north of its north wall: the right wing (pointing south, ~5.5 m)
        // passes through it and the right wingtip hard point strikes it.
        Aircraft ac(ref(), saoPauloWorldFrame());
        ac.setEngineRunning(true, 2300);
        RigidBodyState s;
        const Real base = g.terrainHeight(90, 0);
        s.position = ned(40, 13, base + 20);
        s.velocity = world::nedFromEnu({ktToMs(70), 0, 0});
        s.attitude = Quat::fromEuler(0, 0, kPi / 2);  // heading east
        ac.setFlightState(s);
        ac.prepareContext(PilotInput{}, g, atm);
        ac.settleDownwash();
        PilotInput in;
        in.throttle = 0.6;
        in.mixture = 1;
        bool building = false;
        for (int i = 0; i < 3 * 240 && !building; ++i) {
            ac.step(1.0 / 240, in, g, atm);
            for (const FlightEvent& e : ac.drainEvents())
                building |= e.type == FlightEventType::HardPointContact && static_cast<int>(e.value) == surfaces::kBuilding.id;
        }
        CHECK(building);
    }
}
