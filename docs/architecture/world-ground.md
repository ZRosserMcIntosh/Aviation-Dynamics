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

No `avd-world-tile/1` terrain tiles ship here: their geometry would come
from the SaoPauloWorld project, which is not in this repository, and the
tests use a synthetic tile marked `"synthetic": true`. The region's real
terrain reaches the aircraft another way, below.

## The region's terrain: `ElevationGrid` on `RegionGround`

`tools/terrain/make_terrain.py` builds `data/world/terrain_sp.avt`:

- **Source.** SRTM 1 arc-second (NASA/USGS, public domain), from the AWS
  Terrain Tiles mirror.
- **Grid.** 200 m over 220 × 180 km around MASP (1101 × 901 samples), in
  the map's projection (EPSG:31983 less the map origin, as the airports and
  buildings), heights in decimetres: 2 MB.
- **Ground, not roofs.** SRTM is a surface model; each cell takes the 20th
  percentile of the ~40 one-arc-second samples inside it (the open ground
  between buildings and trees). Voids are filled from their neighbours, the
  sea is 0.

**AVT1** (little-endian): `"AVT1"`, int32 nx, int32 ny, float64 east0, north0
(the south-west sample), float64 spacing, float64 scale (m per unit), then
nx × ny int16, rows from the south, each row west to east.

`RegionGround::setTerrain` takes the grid (`scenarios::saoPauloTerrain()`
natively, `avd_terrain` in the browser):

| Where | Ground |
|---|---|
| A field's flat area (its runways and 1.5 km around them) | the field's elevation, as before |
| The next 2 km | the grid, blended into the field's elevation (smoothstep) |
| Elsewhere on the grid | the grid, bilinear |
| The grid's last 5 km, and off it | blended into the fields' inverse-distance blend, as before |

Normals come from central differences; vertical rays (`below`) read it
directly; oblique rays march in steps of half the height above the ground
(at least 5 m) and bisect the crossing. The buildings (`BuildingLayer`) stand
on it, and the browser's ground mesh samples it.

## Buildings on another ground: `BuildingLayer`

The city's buildings come from the Overture Maps buildings theme
(`tools/osm_buildings`): 2.07 million OpenStreetMap footprints with their
mapped heights, and 2.38 million Google and Microsoft footprints where OSM has
none, their heights estimated. They are cut into 1 km "AVB1" tiles in the site
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
