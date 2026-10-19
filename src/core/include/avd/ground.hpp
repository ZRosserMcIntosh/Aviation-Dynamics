// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Ground query interface (local NED frame). The core never talks to a world or
// physics engine directly: the host implements GroundQuery (e.g. from the game
// world's collision geometry via world_frame.hpp conversions), tests use the
// analytic implementations below. Same shape as OpenRoad Dynamics'
// ord::GroundQuery so a single host adapter can serve both vehicle types.
#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "avd/math.hpp"

namespace avd {

struct SurfaceProperties {
    Real friction = 0.8;            // peak tyre-surface friction coefficient (braking)
    Real rollingResistance = 1.0;   // multiplier on the gear's rolling-resistance coefficient
    Real waterDepth = 0;            // m of water on it: 0 dry, < 3 mm wet, >= 3 mm standing water
    int id = 0;                     // 0 asphalt, 1 concrete, 2 grass, 3 dirt, 4 water ...
};

namespace surfaces {
inline constexpr SurfaceProperties kAsphalt{0.8, 1.0, 0, 0};
inline constexpr SurfaceProperties kConcrete{0.8, 1.0, 0, 1};
// Short dry grass: rolling resistance roughly 2-3x paved (FAA AC 150/5320-6
// class values; Estimated), braking friction ~0.4.
inline constexpr SurfaceProperties kGrass{0.4, 2.5, 0, 2};
}  // namespace surfaces

// Peak tyre-surface friction with water on the surface, at tyre ground
// speed `speed` (m/s) and inflation pressure `tyrePressure` (Pa; 0 = unknown,
// taken as 50 psi):
// - wet paved surface (< 3 mm): the 14 CFR 25.109(c)(1) maximum wet braking
//   coefficient, a cubic in V/100 (V true ground speed, kt) for 50, 100 and
//   200 psi, interpolated linearly in pressure (held at the 200 psi curve
//   above it), never more than the dry value;
// - standing water (>= 3 mm): additionally halved toward the dynamic
//   hydroplaning speed V_P = 9 sqrt(p) kt (Horne, NASA TN D-2056) and 0.05
//   at and above it, where the tyre rides on the water (estimated shape);
// - wet unpaved surfaces: 60% of dry (estimated).
// Anti-skid efficiency is not applied here: the brake and tyre models stop
// short of the peak by themselves.
Real wetTyreFriction(const SurfaceProperties& s, Real speed, Real tyrePressure);
// Dynamic hydroplaning speed (m/s) for a tyre pressure (Pa).
Real hydroplaningSpeed(Real tyrePressure);

struct GroundHit {
    bool hit = false;
    Real distance = 0;
    Vec3 point;                 // NED
    Vec3 normal{0, 0, -1};      // NED, pointing away from the ground (up)
    SurfaceProperties surface;
};

class GroundQuery {
public:
    virtual ~GroundQuery() = default;
    // `dir` is unit length (NED). Must be thread-safe for concurrent const calls.
    virtual GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const = 0;
    // Terrain height (m, NED down coordinate of the surface) below a point.
    virtual GroundHit below(const Vec3& point) const { return raycast(point - Vec3{0, 0, 1000}, {0, 0, 1}, 1e5); }
    // Solid geometry the point is inside (e.g. a building): fills `contact`
    // with the nearest exit point and outward normal and returns true. Used
    // for the airframe hard points (wingtips, tail, propeller). Terrain-only
    // grounds have none.
    virtual bool solidAt(const Vec3& point, GroundHit& contact) const {
        (void)point;
        (void)contact;
        return false;
    }
};

// Infinite plane at NED down-coordinate `downOfSurface` (surface at altitude
// origin.height - downOfSurface).
class FlatGround final : public GroundQuery {
public:
    explicit FlatGround(Real downOfSurface = 0, SurfaceProperties s = surfaces::kAsphalt)
        : d_(downOfSurface), s_(s) {}
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;

private:
    Real d_;
    SurfaceProperties s_;
};

// A runway in the local frame: a sloped paved rectangle in a flat field.
struct RunwayDef {
    std::string ident;                // e.g. "12"
    Vec3 threshold;                   // NED of the runway-end centreline point
    Real headingTrue = 0;             // rad, direction of travel for this runway
    Real length = 1000, width = 30;
    Real slope = 0;                   // rise per metre along the heading (+ uphill)
    Real displaced = 0;               // m from the paved end to the landing threshold
    SurfaceProperties surface = surfaces::kAsphalt;
};

class AirfieldGround final : public GroundQuery {
public:
    AirfieldGround(Real fieldDown, SurfaceProperties field = surfaces::kGrass) : fieldDown_(fieldDown), field_(field) {}
    void addRunway(const RunwayDef& r) { runways_.push_back(r); }
    const std::vector<RunwayDef>& runways() const { return runways_; }
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;
    // Surface down-coordinate and properties at a horizontal position.
    void surfaceAt(Real north, Real east, Real& down, Vec3& normal, SurfaceProperties& s) const;
    // Water on the whole airfield (m): rain, standing water.
    void setWaterDepth(Real m) { waterDepth_ = std::max(m, 0.0); }
    Real waterDepth() const { return waterDepth_; }

private:
    Real waterDepth_ = 0;
    Real fieldDown_;
    SurfaceProperties field_;
    std::vector<RunwayDef> runways_;
};

// Several airfields in one region, each at its own elevation: flat within
// `radius` of its centre (with its runways), and between the fields a smooth
// blend of their elevations (inverse-distance weighting, power 2, on the
// distance beyond each field's flat area). Grass off the runways. Not real
// terrain: the hills and valleys between the fields are absent.
class RegionGround final : public GroundQuery {
public:
    struct Field {
        Real north = 0, east = 0;  // centre (NED m)
        Real radius = 2000;        // flat within this distance of the centre
        Real down = 0;             // NED down coordinate of the field
        std::vector<RunwayDef> runways;
    };
    void addField(const Field& f) { fields_.push_back(f); }
    const std::vector<Field>& fields() const { return fields_; }
    void setWaterDepth(Real m) { waterDepth_ = std::max(m, 0.0); }
    Real waterDepth() const { return waterDepth_; }
    // The terrain's down coordinate at a horizontal position (runways lie on it).
    Real terrainDown(Real north, Real east) const;
    void surfaceAt(Real north, Real east, Real& down, Vec3& normal, SurfaceProperties& s) const;
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;

private:
    Real waterDepth_ = 0;
    std::vector<Field> fields_;
};

}  // namespace avd
