# World ground: terrain tiles and buildings

`TiledWorldGround` (`avd/world_ground.hpp`) is the aircraft's ground in the
open world. It follows the São Paulo World tiling:

- square tiles of 500 m;
- tile `T_{i}_{j}` is centred at (i × 500, j × 500) m in the world ENU frame
  (x east, y north, z up = metres MSL), with the origin at the world anchor
  (MASP, −23.5611, −46.6558).

The aircraft's local NED frame must have its origin at that anchor at height
0. `scenarios::saoPauloWorldFrame()` does this, so ENU and NED differ only by
the axis swap in `world_frame.hpp`.

## What the aircraft gets from it

| Query | Used by | Behaviour |
|---|---|---|
| `below(p)` | gear, hard points, wing height for ground effect | Highest surface at or below the point (0.5 m tolerance): terrain, or a roof. An aircraft can stand on a roof. |
| `solidAt(p)` | hard points (wingtips, tail, propeller, belly) | Inside a building: the nearest exit (a wall or the roof) with its outward normal. The hard-point spring pushes the point out, and the aircraft raises a `HardPointContact` (or `PropStrike`) event whose `value` is the surface id (5 = building). |
| `raycast(o, d, max)` | renderers, sensors, future radar altimeter | Terrain by safe-step marching and bisection; buildings by exact ray/prism intersection (roof and walls). |

Beyond the loaded tiles there is a flat plane at a configurable MSL height,
so an aircraft flying off the data still has a ground. Per-step cost is
O(1): a bilinear lookup plus an 8 × 8 bucket index of building footprints per
tile.

## Tile file: `avd-world-tile/1`

```json
{
  "format": "avd-world-tile/1",
  "tile": "T_m10_m1", "i": -10, "j": -1, "size_m": 500,
  "source": "SaoPauloWorld export <commit>; terrain: <DEM and licence>; buildings: GeoSampa/OSM (ODbL)",
  "synthetic": false,
  "terrain": {"n": 51, "z_m": [ ... n*n heights, m MSL ... ]},
  "default_surface": "grass",
  "surfaces": {"n": 50, "legend": ["asphalt", "grass", "water"], "cells": [ ... n*n legend indices ... ]},
  "buildings": [
    {"id": "way/123", "footprint": [[east, north], ...], "height_m": 42.0},
    {"id": "way/456", "footprint": [[east, north], ...], "base_m": 760.0, "top_m": 810.0}
  ]
}
```

- **Required:** `format`, `i`, `j`, `terrain`, and `source`. The source must
  give the data's provenance; the loader rejects tiles without one.
- **`terrain.z_m`:** `n × n` samples including the tile's edges, row-major
  from the south-west corner, with rows running north (spacing
  `size_m / (n − 1)`).
- **`surfaces`:** optional. It is a coarser `n × n` grid of indices into
  `legend`. The classes are `asphalt`, `concrete`, `grass`, `dirt`, `water`
  and `building`, with friction and rolling-resistance values from
  `avd/ground.hpp` and `avd/world_ground.hpp`.
- **`buildings`:** footprints in world ENU metres (either winding). Give
  `height_m` (walls then start at the lowest terrain under the footprint), or
  explicit `base_m` / `top_m`.
- **Size:** all tiles share one `size_m`.

## Status

The integration layer is implemented and tested (`tests/test_world.cpp`):

- exact planar terrain;
- slope normals;
- roofs and walls by query and raycast;
- the loader's validation;
- an aircraft parked across a 5% slope settling on its gear;
- a wingtip striking a building.

**No real São Paulo terrain tiles ship here yet.** The terrain geometry is
produced by the SaoPauloWorld project, which is not in this repository. The
next step for terrain is an `avd-world-tile/1` exporter in that project. The
tests use a synthetic tile marked `"synthetic": true`.

## Buildings on another ground: `BuildingLayer`

The city's buildings come from OpenStreetMap (`tools/osm_buildings`): 2.07
million footprints with their mapped heights, in 1 km "AVB1" tiles in the site
frame. The browser flight test bed streams them around the aircraft.
`BuildingLayer` puts them on any other ground (in the browser, the region's
`RegionGround`):

- **Groups.** Buildings come and go in groups under a key (one tile each).
  Each group is indexed on its own in a 100 m grid, so streaming a tile does
  not reindex the others.
- **Parsing.** `parseAvb1` reads a tile and stands each building on the inner
  ground under its centroid and first corner. It sinks the base 0.6 m, as the
  renderer does. Courtyards are open air.
- **Queries.**
  - `below`: the higher of the inner ground and a roof at or below the point.
  - `solidAt`: the nearest wall or the roof, for the hard points (as
    `TiledWorldGround`).
  - `raycast`: the inner ground's hit, or a nearer roof or wall met along the
    ray.
- **Browser ABI.**
  - `avd_buildings_tile(i, j, len)`: the tile's bytes go in the text buffer,
    as `A.buildingsTile` does it.
  - `avd_buildings_drop(i, j)` and `avd_buildings_count()`.
  - A hard point meeting a building reports `Building strike (<point>)`.
- **Streaming.** The site hands over the tiles within about 1.5 km of the
  aircraft and drops them past 2.5 km.

Tested in `tests/test_world.cpp`:

- a tile parsed onto the region's ground: its tops, bases and courtyards;
- roofs, walls and rays;
- groups added, replaced and dropped on their own;
- a wingtip strike.

The web smoke test flies into a streamed tower.
