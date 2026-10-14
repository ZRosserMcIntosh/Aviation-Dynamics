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
    std::vector<RunwayDef> runways;
};

// Campo de Marte (SBMT), São Paulo. Runway 12/30, 1600 x 45 m asphalt,
// elevation 722 m. Threshold coordinates from SkyVector (S23 30.50 W46 38.55 /
// S23 30.62 W46 37.88); the paved ends are 300 m / 150 m beyond the displaced
// thresholds. Approximate; not for real-world navigation.
Airport campoDeMarte(const LocalFrame& frame);

// Flat airfield ground (grass) at the airport elevation with its runways.
std::unique_ptr<AirfieldGround> makeAirfieldGround(const Airport& a, const LocalFrame& frame);

// Point at `along` metres down runway `r` and `across` metres right of the centreline (NED).
Vec3 runwayPoint(const RunwayDef& r, Real along, Real across = 0);

}  // namespace avd::scenarios
