// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/loading.hpp"

#include <cmath>
#include <limits>

namespace avd {

namespace {

struct Sum {
    Real m = 0;
    Vec3 mx;
    void add(Real mass, const Vec3& p) {
        m += mass;
        mx = mx + p * mass;
    }
    Vec3 cg() const { return m > 0 ? mx * (1.0 / m) : Vec3{}; }
};

}  // namespace

LoadSheet computeLoadSheet(const Aircraft& ac) {
    const AircraftDefinition& d = ac.definition();
    const Real rho = d.fuel.density;
    LoadSheet s;

    // Basic empty as published; the unusable fuel is part of it when the
    // definition says so, otherwise it is counted with the fuel.
    Sum empty;
    empty.add(d.emptyMass, d.emptyCg);
    if (!d.emptyIncludesUnusableFuel)
        for (std::size_t i = 0; i < d.fuel.tanks.size(); ++i) {
            const FuelTankDef& t = d.fuel.tanks[i];
            empty.add(std::min(ac.fuel().tankQuantity(static_cast<int>(i)), t.unusable) * rho, t.position);
        }
    s.basicEmptyMass = empty.m;
    s.basicEmptyCg = empty.cg();

    Sum zf = empty;
    for (std::size_t i = 0; i < d.stations.size(); ++i) {
        const Real m = ac.stationMass(static_cast<int>(i));
        s.payload += m;
        zf.add(m, d.stations[i].position);
        if (m > d.stations[i].maxMass + 1e-9) s.stationsOverPlacard.push_back(static_cast<int>(i));
    }
    for (std::size_t g = 0; g < d.stationGroupLimits.size(); ++g) {
        Real m = 0;
        for (int i : d.stationGroupLimits[g].stations) m += ac.stationMass(i);
        if (m > d.stationGroupLimits[g].maxMass + 1e-9) s.groupsOverPlacard.push_back(static_cast<int>(g));
    }
    s.zeroFuelMass = zf.m;
    s.zeroFuelCg = zf.cg();

    Sum gross = zf;
    for (std::size_t i = 0; i < d.fuel.tanks.size(); ++i) {
        const FuelTankDef& t = d.fuel.tanks[i];
        const Real q = ac.fuel().tankQuantity(static_cast<int>(i));
        s.fuelTotal += q * rho;
        s.fuelUsable += std::max(0.0, q - t.unusable) * rho;
        // Unusable fuel is already in the basic empty mass.
        gross.add(std::max(0.0, q - t.unusable) * rho, t.position);
        if (q > t.capacity * (1 + 1e-9)) s.flags |= kTankOverCapacity;
    }
    // Fuel in the lines between the tanks and the engine(s): trapped, but carried.
    const Real line = ac.fuel().lineQuantity() * rho;
    s.fuelTotal += line;
    gross.add(line, d.fuel.tanks.empty() ? Vec3{} : d.fuel.tanks[0].position);
    s.grossMass = gross.m;
    s.cg = gross.cg();

    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    s.cgForwardLimit = d.cgForwardLimitVsMass.empty() ? nan : d.cgForwardLimitVsMass(s.grossMass);
    s.cgAftLimit = d.cgAftLimitVsMass.empty() ? nan : d.cgAftLimitVsMass(s.grossMass);
    // x is forward-positive: forward of the limit means a larger x.
    if (s.cg.x > s.cgForwardLimit) s.flags |= kCgForwardOfLimit;
    if (s.cg.x < s.cgAftLimit) s.flags |= kCgAftOfLimit;
    if (d.maxRampMass > 0 && s.grossMass > d.maxRampMass) s.flags |= kOverMaxRamp;
    if (s.grossMass > d.maxTakeoffMass) s.flags |= kOverMaxTakeoff;
    if (d.maxLandingMass > 0 && s.grossMass > d.maxLandingMass) s.flags |= kOverMaxLanding;
    if (d.maxZeroFuelMass > 0 && s.zeroFuelMass > d.maxZeroFuelMass) s.flags |= kOverMaxZeroFuel;
    if (!s.stationsOverPlacard.empty()) s.flags |= kStationOverPlacard;
    if (!s.groupsOverPlacard.empty()) s.flags |= kGroupOverPlacard;
    return s;
}

}  // namespace avd
