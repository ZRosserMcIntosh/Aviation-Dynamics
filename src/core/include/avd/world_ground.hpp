// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Ground for the open world: São Paulo World-style square tiles (500 m by
// default) in the game's ENU frame (X east, Y north, Z up = metres MSL, origin
// at the world anchor). Each tile carries a terrain heightfield, a grid of
// surface classes and building footprints extruded to their heights. The
// aircraft sees it through GroundQuery (local NED, whose origin must be the
// world anchor at height 0, as scenarios::saoPauloWorldFrame() is).
//
// Tile file format ("avd-world-tile/1", JSON): docs/architecture/world-ground.md.
#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "avd/ground.hpp"

namespace avd {

namespace surfaces {
inline constexpr SurfaceProperties kDirt{0.6, 1.8, 0, 3};
// Water: no tyre grip; landing gear is not a float (ditching is a crash).
inline constexpr SurfaceProperties kWater{0.05, 20.0, 1, 4};
inline constexpr SurfaceProperties kBuilding{0.7, 1.0, 0, 5};
}  // namespace surfaces

// Surface class names used by tile files.
bool surfaceFromName(const std::string& name, SurfaceProperties& out);

struct WorldBuilding {
    std::string id;
    std::vector<std::pair<Real, Real>> footprint;  // ENU (east, north), m; simple polygon, either winding
    Real base = 0;                                 // m MSL (bottom of the walls)
    Real top = 0;                                  // m MSL (roof)
};

struct WorldTile {
    std::string name;            // e.g. "T_m10_m1"
    int i = 0, j = 0;            // tile indices: centre at (i * size, j * size)
    Real size = 500;             // m
    int n = 2;                   // terrain samples per side (corners included), >= 2
    std::vector<Real> height;    // n*n, m MSL, row-major from the south-west corner (rows go north)
    int surfaceN = 0;            // surface cells per side (0: whole tile uses `defaultSurface`)
    std::vector<SurfaceProperties> surface;  // surfaceN*surfaceN, same ordering as `height`
    SurfaceProperties defaultSurface = surfaces::kGrass;
    std::vector<WorldBuilding> buildings;
    std::string source;          // provenance of the data (required in files)
    bool synthetic = false;
};

class TiledWorldGround final : public GroundQuery {
public:
    // `outside`: what lies beyond the loaded tiles (a flat plane at this MSL
    // height). Aircraft over unloaded tiles still have a ground.
    explicit TiledWorldGround(Real outsideHeightMsl = 0, SurfaceProperties outside = surfaces::kGrass)
        : outsideHeight_(outsideHeightMsl), outside_(outside) {}

    void addTile(WorldTile tile);
    // Parses an "avd-world-tile/1" document; returns errors (empty on success).
    std::vector<std::string> loadTileJson(const std::string& text);
    std::size_t tileCount() const { return tiles_.size(); }

    // ENU helpers (m MSL).
    Real terrainHeight(Real east, Real north) const;
    SurfaceProperties surfaceAt(Real east, Real north) const;

    // GroundQuery (local NED). `below` returns the highest surface (terrain or
    // roof) at or below the point (0.5 m tolerance): an aircraft can stand on
    // a roof. A point inside a building is reported by `solidAt` instead.
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override;
    GroundHit below(const Vec3& point) const override;
    bool solidAt(const Vec3& point, GroundHit& contact) const override;

private:
    struct Tile {
        WorldTile data;
        Real x0 = 0, y0 = 0;  // south-west corner, ENU
        static constexpr int kBuckets = 8;
        std::vector<int> buckets[kBuckets * kBuckets];  // building indices by footprint bounds
        std::vector<Real> bbox;  // per building: minX, minY, maxX, maxY
    };
    const Tile* tileAt(Real east, Real north) const;
    bool terrainAt(Real east, Real north, Real& z, Vec3& normalEnu) const;
    template <class F>
    void forBuildingsAt(const Tile& t, Real east, Real north, F&& f) const;

    std::map<std::pair<int, int>, Tile> tiles_;
    Real outsideHeight_;
    SurfaceProperties outside_;
};

}  // namespace avd
