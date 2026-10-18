// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// World anchoring and airports for the São Paulo World map.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "avd/geodesy.hpp"
#include "avd/ground.hpp"
#include "avd/navigation.hpp"

namespace avd::scenarios {

// The São Paulo World tile grid uses metres east (x) / north (y) with the
// origin at MASP (landmark at (-1, 1) in site_data.json). Anchor: MASP,
// 23.5611 S, 46.6558 W (Wikipedia infobox), height 0 = mean sea level (geoid
// undulation ~ -5 m ignored). Accuracy of the world-to-geodetic mapping is
// ~100 m pending confirmation from the world build tools (see
// docs/architecture/world-integration.md).
LocalFrame saoPauloWorldFrame();

struct Airport {
    std::string icao, name;
    Real elevation = 0;          // m MSL
    Real magneticVariation = 0;  // rad, east positive
    std::vector<RunwayDef> runways;  // [0] is the runway in use (spawns, approaches)
    int ilsKhz = 0;              // ILS for runways[0] (synthetic test facilities), 0 = none
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

// Flat airfield ground (grass) at the airport elevation with its runways.
std::unique_ptr<AirfieldGround> makeAirfieldGround(const Airport& a, const LocalFrame& frame);

// Navigation environment for the São Paulo World map: local magnetic field
// (declination ~21.5 deg W, inclination ~38 deg up -- ESTIMATED, to be
// replaced by a World Magnetic Model evaluation) and SYNTHETIC test radio
// facilities: a test VOR at MASP and test ILSs for Campo de Marte runway 12
// and Congonhas runway 35L. These facilities do not exist in reality (SBMT has
// no ILS; SBSP has none on 35L); they exercise the
// receivers until authoritative facility data is loaded (NavDatabase::loadJson).
const NavEnvironment& saoPauloNavEnvironment();

// Point at `along` metres down runway `r` and `across` metres right of the centreline (NED).
Vec3 runwayPoint(const RunwayDef& r, Real along, Real across = 0);

}  // namespace avd::scenarios
