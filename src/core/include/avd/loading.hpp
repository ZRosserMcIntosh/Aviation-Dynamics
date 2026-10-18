// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Weight and balance: the load sheet of the aircraft as it is loaded now,
// built from the same mass items the dynamics use. Limits come from the
// definition (POH/AFM). Nothing here stops an overloaded or out-of-balance
// aircraft from flying ("no artificial limits"): the sheet only reports the
// exceedances, and the physics shows their consequences.
#pragma once

#include <vector>

#include "avd/aircraft.hpp"

namespace avd {

// Exceedance flags.
enum LoadSheetFlag : unsigned {
    kOverMaxRamp = 1u << 0,
    kOverMaxTakeoff = 1u << 1,
    kOverMaxLanding = 1u << 2,
    kOverMaxZeroFuel = 1u << 3,
    kCgForwardOfLimit = 1u << 4,
    kCgAftOfLimit = 1u << 5,
    kStationOverPlacard = 1u << 6,
    kGroupOverPlacard = 1u << 7,
    kTankOverCapacity = 1u << 8,
};

struct LoadSheet {
    // As published: the empty mass and CG including unusable fuel when the
    // definition's empty mass does (emptyIncludesUnusableFuel).
    Real basicEmptyMass = 0;
    Vec3 basicEmptyCg;
    Real payload = 0;                  // all stations
    Real zeroFuelMass = 0;             // basic empty + payload (+ unusable fuel)
    Vec3 zeroFuelCg;
    Real fuelTotal = 0;                // kg on board: tanks (unusable included) and lines
    Real fuelUsable = 0;
    Real grossMass = 0;                // what is on the wheels now
    Vec3 cg;
    // CG limits (x, structural) at the gross mass; NaN when not published.
    Real cgForwardLimit = 0, cgAftLimit = 0;
    unsigned flags = 0;
    std::vector<int> stationsOverPlacard, groupsOverPlacard;
};

LoadSheet computeLoadSheet(const Aircraft& ac);

}  // namespace avd
