// Aviation Dynamics — tests: the open-world ground (terrain tiles, buildings).
// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

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

TEST_CASE("building layer: buildings on another ground -- roofs, walls, rays, AVB1 tiles, a wingtip strike") {
    // The region's ground (each field at its own elevation) with streamed buildings on it.
    const auto region = makeRegionGround(saoPauloRegionAirports());
    BuildingLayer layer(*region);
    // An AVB1 block for tile (0, 0): one 20 x 20 m, 40 m building with its SW corner
    // at (90, -10) from the tile's corner, and a 10 m one with a 4 x 4 m courtyard.
    std::vector<std::uint8_t> blk = {'A', 'V', 'B', '1', 2, 0, 0, 0};
    auto u16 = [&](int v) { blk.push_back(static_cast<std::uint8_t>(v & 0xff)); blk.push_back(static_cast<std::uint8_t>(v >> 8)); };
    auto ring = [&](std::initializer_list<std::pair<int, int>> pts) {
        u16(static_cast<int>(pts.size()));
        for (auto [x, y] : pts) { u16(x & 0xffff); u16(y & 0xffff); }
    };
    u16(400); u16(0); blk.push_back(2); blk.push_back(1);
    ring({{900, -100}, {1100, -100}, {1100, 100}, {900, 100}});
    u16(100); u16(0); blk.push_back(1); blk.push_back(2);
    ring({{3000, 3000}, {3200, 3000}, {3200, 3200}, {3000, 3200}});
    ring({{3080, 3080}, {3080, 3120}, {3120, 3120}, {3120, 3080}});
    std::vector<WorldBuilding> list;
    CHECK(layer.parseAvb1(blk.data(), blk.size(), 0, 0, list));
    CHECK(list.size() == 2);
    const Real ground = -region->below(ned(100, 0, 1e4)).point.z;
    CHECK_NEAR(list[0].top, ground + 40, 0.01);
    CHECK_NEAR(list[0].base, ground - 0.6, 0.01);
    CHECK(list[1].footprint.size() == 4);  // the courtyard is open air, not a wall
    std::vector<std::uint8_t> bad(blk.begin(), blk.end() - 3);
    std::vector<WorldBuilding> none;
    CHECK(!layer.parseAvb1(bad.data(), bad.size(), 0, 0, none));
    layer.add(7, list);
    CHECK(layer.buildingCount() == 2);

    // On the roof, beside it, inside it.
    CHECK_NEAR(-layer.below(ned(100, 0, ground + 60)).point.z, ground + 40, 1e-6);
    CHECK_NEAR(-layer.below(ned(150, 0, ground + 60)).point.z, -region->below(ned(150, 0, 1e4)).point.z, 1e-6);
    GroundHit c;
    CHECK(layer.solidAt(ned(91, 0, ground + 20), c));  // 1 m inside the west wall
    CHECK_NEAR(c.point.x, ned(90, 0, 0).x, 1e-6);
    CHECK_NEAR(c.point.y, ned(90, 0, 0).y, 1e-6);
    CHECK(!layer.solidAt(ned(85, 0, ground + 20), c));
    CHECK(!layer.solidAt(ned(100, 0, ground + 41), c));
    // A level ray east hits the west wall; a ray down hits the roof.
    const GroundHit wall = layer.raycast(ned(0, 0, ground + 20), world::nedFromEnu({1, 0, 0}), 500);
    CHECK(wall.hit);
    CHECK_NEAR(wall.distance, 90, 1e-6);
    const GroundHit roof = layer.raycast(ned(95, 5, ground + 100), world::nedFromEnu({0, 0, -1}), 500);
    CHECK_NEAR(roof.distance, 60, 1e-6);

    // Flying east at 70 kt, 20 m up, 3 m north of the north wall: the right wingtip strikes it.
    SimpleWeather atm;
    Aircraft ac(ref(), saoPauloWorldFrame());
    ac.setEngineRunning(true, 2300);
    RigidBodyState s;
    s.position = ned(40, 13, ground + 20);
    s.velocity = world::nedFromEnu({ktToMs(70), 0, 0});
    s.attitude = Quat::fromEuler(0, 0, kPi / 2);
    ac.setFlightState(s);
    ac.prepareContext(PilotInput{}, layer, atm);
    ac.settleDownwash();
    PilotInput in;
    in.throttle = 0.6;
    in.mixture = 1;
    bool building = false;
    for (int i = 0; i < 3 * 240 && !building; ++i) {
        ac.step(1.0 / 240, in, layer, atm);
        for (const FlightEvent& e : ac.drainEvents())
            building |= e.type == FlightEventType::HardPointContact && static_cast<int>(e.value) == surfaces::kBuilding.id;
    }
    CHECK(building);
    // Groups come and go on their own: a second tile's building stays when the first is dropped.
    std::vector<WorldBuilding> other(1);
    other[0].footprint = {{1100, 0}, {1120, 0}, {1120, 20}, {1100, 20}};
    other[0].base = ground - 1;
    other[0].top = ground + 30;
    layer.add(8, other);
    CHECK(layer.buildingCount() == 3);
    layer.remove(7);
    CHECK(layer.buildingCount() == 1);
    CHECK(layer.groupCount() == 1);
    CHECK(!layer.solidAt(ned(91, 0, ground + 20), c));
    CHECK(layer.solidAt(ned(1110, 10, ground + 10), c));
    layer.add(8, {});  // replacing a group drops its old buildings
    CHECK(!layer.solidAt(ned(1110, 10, ground + 10), c));
}

TEST_CASE("map projection: airports in the São Paulo world sit on the map's UTM 23S grid") {
    // References from PROJ (pyproj, EPSG:4326 -> EPSG:31983) less the map origin.
    Real e = 0, n = 0;
    wgs84::transverseMercator(Geodetic{degToRad(-23.5611), degToRad(-46.6558), 0}, degToRad(-45.0), 0.9996, e, n);
    CHECK_NEAR(e + 500000.0, 331004.4497, 1e-3);
    CHECK_NEAR(n + 10000000.0, 7393388.1322, 1e-3);
    wgs84::transverseMercator(Geodetic{degToRad(-20.0), degToRad(-44.0), 0}, degToRad(-45.0), 0.9996, e, n);
    CHECK_NEAR(e + 500000.0, 604609.3238, 1e-3);
    CHECK_NEAR(n + 10000000.0, 7788206.4439, 1e-3);

    const Vec3 masp = saoPauloMapNed(-23.5611, -46.6558);
    CHECK_NEAR(masp.x, 77.412, 1e-3);
    CHECK_NEAR(masp.y, 18.450, 1e-3);
    // Congonhas 35L's surveyed threshold, on the map (OSM centreline within 2 m).
    const Airport cgh = congonhas(saoPauloWorldFrame());
    const Vec3 thr35 = landingThreshold(cgh.runways[0]);
    CHECK_NEAR(thr35.x, -8023.309, 0.01);
    CHECK_NEAR(thr35.y, 581.571, 0.01);
    // In another frame the tangent plane is used: the threshold is the same
    // geodetic point.
    const LocalFrame other(Geodetic{degToRad(-23.6), degToRad(-46.65), 0});
    const Vec3 t2 = landingThreshold(congonhas(other).runways[0]);
    const Geodetic g = other.geodeticFromNed({t2.x, t2.y, 0});
    CHECK_NEAR(radToDeg(g.latitude), -23.6343, 1e-6);
    CHECK_NEAR(radToDeg(g.longitude), -46.6512, 1e-6);
}
