// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// World anchoring and airports for the São Paulo World map.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "avd/atmosphere.hpp"
#include "avd/geodesy.hpp"
#include "avd/gps.hpp"
#include "avd/json.hpp"
#include "avd/ground.hpp"
#include "avd/navigation.hpp"

namespace avd::scenarios {

// The São Paulo World tile grid uses metres east (x) / north (y) with the
// origin at MASP (landmark at (-1, 1) in site_data.json). Anchor: MASP,
// 23.5611 S, 46.6558 W (Wikipedia infobox), height 0 = mean sea level (geoid
// undulation ~ -5 m ignored). The frame's tangent plane is the physics'
// (gravity, the Earth's rotation); where on it an airport goes is the map's
// projection, saoPauloMapNed below.
LocalFrame saoPauloWorldFrame();

// The São Paulo World map's projection: EPSG:31983 (SIRGAS 2000 / UTM 23S)
// less the map's origin, as the SaoPauloWorld tiles, the site's photo map and
// its OpenStreetMap buildings use (fitted to OSM street centrelines: no
// rotation, 0.3 m residual; tools/osm_buildings). The world's north is UTM
// grid north, ~0.66 deg east of true north here. Airports in the São Paulo
// world are placed with it (NED: x north, y east; z = 0).
inline constexpr Real kMapOriginEast = 330986.00, kMapOriginNorth = 7393310.72;  // EPSG:31983 (m), 15.8 m E and 45.3 m N of MASP
Vec3 saoPauloMapNed(Real latDeg, Real lonDeg);

// An ILS on one runway end: localizer along the centreline, its antenna
// 300 m beyond the far end; glideslope antenna 300 m past the landing
// threshold, 120 m to the side; 3 deg. Antenna sites are estimated from the
// runway (not surveyed); identifier and frequency are the published ones
// unless `synthetic`.
struct IlsDef {
    int runway = 0;  // index into Airport::runways
    int khz = 0;
    std::string ident;
    bool synthetic = false;
    std::string source;
};

struct Airport {
    std::string icao, name;
    Real elevation = 0;          // m MSL
    Real magneticVariation = 0;  // rad, east positive
    // Runway ends in pairs (an end, then its reciprocal); [0] is the runway in
    // use (spawns, approaches).
    std::vector<RunwayDef> runways;
    int ilsKhz = 0;              // ILS for runways[0], 0 = none
    std::vector<IlsDef> ils;
    std::string source;          // provenance of the runway data
};

// Campo de Marte (SBMT), São Paulo. Runway 12/30, 1600 x 45 m asphalt,
// elevation 722 m. Threshold coordinates from SkyVector (S23 30.50 W46 38.55 /
// S23 30.62 W46 37.88); the paved ends are 300 m / 150 m beyond the displaced
// thresholds. Approximate; not for real-world navigation.
Airport campoDeMarte(const LocalFrame& frame);

// Congonhas (SBSP / CGH), São Paulo: runway 17R/35L, 1940 x 45 m asphalt,
// field elevation 2631 ft (802 m). Threshold coordinates from SkyVector (35L
// S23 38.06 W46 39.07, 17R S23 37.26 W46 39.63); displaced thresholds 61 m (35L)
// and 126 m (17R) put the paved ends 1940 m apart. In use: 35L. The runway
// falls ~12 m toward the north (end elevations 2630 / 2590 ft); modelled flat at
// the field elevation until the world terrain arrives. Runway 17L/35R is not
// modelled. Approximate; not for real-world navigation.
Airport congonhas(const LocalFrame& frame);

// The airports of the São Paulo region a flight can go to: [0] Congonhas,
// [1] Campo de Marte (as above), then Guarulhos (SBGR), Viracopos (SBKP),
// São José dos Campos (SBSJ) and Jundiaí (SBJD). Runway ends, widths,
// displaced thresholds and elevations from OurAirports (public domain,
// 2026); ILS identifiers and frequencies from the published listings named
// in each IlsDef. Runways are modelled flat at the field elevation.
// Approximate; not for real-world navigation.
const std::vector<Airport>& saoPauloRegionAirports();

// An airport from one record of data/world/airports_br.json
// (tools/navdata/brazil.py): its paved runways (from their ends' coordinates
// when the record has them, else centred on the airport reference point on
// their true heading, or the runway number's with the regional variation;
// parallel L/R runways 400 m apart) and its ILSs. Runways flat at the field
// elevation. Unusable records give an airport with no runways.
Airport airportFromRecord(const json::Value& record, const LocalFrame& frame);

// The ILS navaids of `a` (IlsDef) into `db`: localizer antenna 300 m beyond
// the far end, glideslope 300 m past the landing threshold and 120 m to the
// side; 3 degrees.
void addIlsNavaids(NavDatabase& db, const Airport& a, const LocalFrame& frame);

// Flat airfield ground (grass) at the airport elevation with its runways.
std::unique_ptr<AirfieldGround> makeAirfieldGround(const Airport& a, const LocalFrame& frame);

// Every airport of the region on one ground, each field at its own
// elevation, the terrain between them blended (RegionGround).
std::unique_ptr<RegionGround> makeRegionGround(const std::vector<Airport>& airports);
// The São Paulo region's terrain (data/world/terrain_sp.avt, tools/terrain:
// SRTM 1", 200 m grid), read once; nullptr if the file is missing. Give it to
// a RegionGround with setTerrain for the hills between the fields.
std::shared_ptr<const ElevationGrid> saoPauloTerrain();

// The landing threshold of runway end `r` (past any displaced threshold).
Vec3 landingThreshold(const RunwayDef& r);

// A GPS approach route to runway end `runway` of `dest`: an initial fix (IAF)
// 5 NM abeam the intermediate fix when the route arrives from more than 60
// deg off the final course (and not already on the final), the intermediate fix (IF) 10 NM and the final
// approach fix (FF) 5 NM out on the extended centreline at the 3 deg path's
// altitudes, then the landing threshold (RW). `from` (NED) is where the
// route starts; when `departure` is given, the route begins on that runway
// end and 3 NM straight out. Waypoint names are generated, not the
// procedure's published fixes. Returns the route; `faf` gets the FAF index.
std::vector<RouteWaypoint> approachRoute(const Airport& dest, int runway, const Vec3& from, int& faf,
                                         const RunwayDef* departure = nullptr);

// Navigation environment for the São Paulo World map: local magnetic field
// (declination ~21.5 deg W, inclination ~38 deg up -- ESTIMATED, to be
// replaced by a World Magnetic Model evaluation), the region airports' ILSs
// (published identifiers and frequencies, antenna sites estimated from the
// runways) and two SYNTHETIC test facilities that do not exist in reality: a
// test VOR at MASP and a test ILS on Campo de Marte runway 12 (SBMT has no ILS).
const NavEnvironment& saoPauloNavEnvironment();

// Point at `along` metres down runway `r` and `across` metres right of the centreline (NED).
Vec3 runwayPoint(const RunwayDef& r, Real along, Real across = 0);

// The wind surroundings of Campo de Marte for SimpleWeather: the city of
// São Paulo (z0 ~ 1 m) around the open airfield (z0 0.03 m, where the wind
// is reported), southern-hemisphere backing of the wind with height (-20
// deg through the boundary layer), and the Serra da Cantareira ridge
// ~10 km north (crest ~1100 m MSL, running roughly east-west). ESTIMATED
// geometry, for wind physics only (the ground model does not include it).
void addSaoPauloWindEnvironment(SimpleWeatherParams& p, const Airport& a);

}  // namespace avd::scenarios
