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
#include <cstddef>
#include <memory>
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
// This is the peak; the tyre model (avd/aircraft.hpp) slides at a fraction of
// it once a wheel locks or skids, and a leg's anti-skid holds its efficiency's
// fraction of it.
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

// Ground heights on a regular grid (tools/terrain: the AVT1 file), in the
// local NED frame's horizontal axes: x north, y east, heights m above the
// frame's origin (MSL for the São Paulo world). Bilinear between samples.
class ElevationGrid {
public:
    // AVT1 bytes: b"AVT1", int32 nx, ny, float64 east0, north0, spacing,
    // scale, then nx * ny int16 (rows from the south, west to east), all
    // little-endian. False (and left empty) on anything malformed.
    bool parse(const unsigned char* data, std::size_t size);
    bool empty() const { return heights_.empty(); }
    // Height at (north, east); NaN off the grid.
    Real heightAt(Real north, Real east) const;
    // Distance inside the grid's edge (m; negative outside).
    Real inside(Real north, Real east) const;
    Real north0() const { return north0_; }
    Real east0() const { return east0_; }
    Real spacing() const { return spacing_; }
    int nx() const { return nx_; }
    int ny() const { return ny_; }

private:
    int nx_ = 0, ny_ = 0;
    Real east0_ = 0, north0_ = 0, spacing_ = 1;
    std::vector<float> heights_;
};

// Several airfields in one region, each at its own elevation: flat within
// `radius` of its centre (with its runways). Between the fields, the
// elevation grids if any are set (the region's real hills and valleys,
// blended to each field's elevation over 2 km beyond its flat area, and to
// the fields' blend over the region grid's last 5 km inside its edge);
// without one, or off them, a smooth blend of the fields' elevations
// (inverse-distance weighting, power 2, on the distance beyond each field's
// flat area). Grass off the runways.
//
// Grids nest: a finer grid (a smaller spacing) over part of the region wins
// where it lies, blended into the coarser ones over its own last `blend`
// metres inside its edge, so its edge has no step (setTerrain: the region
// grid; addTerrain: a detail grid such as the city's, data/world/
// terrain_sp_city.avt).
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
    // The region's terrain between the fields (nullptr: the blend alone),
    // blended into the fields' blend over its last 5 km. Replaces every grid.
    void setTerrain(std::shared_ptr<const ElevationGrid> grid);
    // A nested grid (finer than those it lies over), blended into them over
    // its last `blend` m inside its edge. One with the same origin, spacing
    // and size as one already set replaces it.
    void addTerrain(std::shared_ptr<const ElevationGrid> grid, Real blend = 2000);
    // The coarsest grid (the region's; nullptr without one), and all of them
    // from the coarsest to the finest.
    const ElevationGrid* terrain() const { return grids_.empty() ? nullptr : grids_.front().grid.get(); }
    int terrainCount() const { return static_cast<int>(grids_.size()); }
    const ElevationGrid* terrain(int i) const { return grids_[static_cast<std::size_t>(i)].grid.get(); }
    // The grids' height at a horizontal position (m above the frame's
    // origin, before any blending into the fields) and how far it is the
    // grids' (1 well inside the region grid, 0 off every grid: then the
    // fields' blend alone).
    Real gridHeight(Real north, Real east, Real& cover) const;
    // The terrain's down coordinate at a horizontal position (runways lie on it).
    Real terrainDown(Real north, Real east) const;
    void surfaceAt(Real north, Real east, Real& down, Vec3& normal, SurfaceProperties& s) const;
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;

private:
    Real blendDown(Real north, Real east) const;
    struct Grid {
        std::shared_ptr<const ElevationGrid> grid;
        Real blend = 5000;  // m inside its edge over which it blends into what lies under it
    };
    Real waterDepth_ = 0;
    std::vector<Field> fields_;
    std::vector<Grid> grids_;  // coarsest first
};

}  // namespace avd
