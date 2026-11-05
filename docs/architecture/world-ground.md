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
| `raycast(o, d, max)` | renderers, sensors | Terrain by safe-step marching and bisection; buildings by exact ray/prism intersection (roof and walls). |

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

`tools/terrain/make_terrain.py` builds two grids from SRTM 1 arc-second
(NASA/USGS, public domain, from the AWS Terrain Tiles mirror), in the map's
projection (EPSG:31983 less the map origin, as the airports and buildings),
heights in decimetres:

| File | Grid | Ground filter | Size |
|---|---|---|---|
| `data/world/terrain_sp.avt` (the region) | 200 m over 220 × 180 km around MASP (1101 × 901) | the 20th percentile of the ~40 one-arc-second samples in each cell (the open ground between buildings and trees) | 2.0 MB |
| `data/world/terrain_sp_city.avt` (the city, `--detail`) | 50 m over 50 × 62 km (east −20 to +30 km, north −38 to +24 km of MASP; 1001 × 1241): the São Paulo municipality to the Billings and Guarapiranga reservoirs, the Pico do Jaraguá, the Serra da Cantareira, Guarulhos | a morphological ground filter (below) | 2.5 MB |

**The city's ground filter.** A 50 m cell holds only two or three SRTM
samples, so a percentile has nothing to choose from. On a 25 m resampling
of the model, its opening (the local minimum, then the local maximum, over
a 400 m square) keeps planar slopes and valleys as they are and takes off
what stands above the ground over less than that width: blocks of
buildings and stands of trees, but also the tops of hills. So the
correction (the surface less its opening, at most 10 m, the surface model's
bias over the high-rises) is applied in full only where the terrain is
gentle and fades out between 8 % and 20 % slope (the steepest of the
smoothed model within 400 m): a summit or a ridge, with steep ground all
round it, keeps its height. The correction, not the ground, is smoothed over
150 m, so the peaks stay sharp. Voids are filled from their neighbours, the
sea is 0.

| Landmark (SRTM / reference) | Region grid (200 m) | City grid (50 m) |
|---|---|---|
| Pico do Jaraguá summit (SRTM 1,132 m; 1,135 m surveyed) | 1,033 m | 1,118 m |
| Cantareira, highest (SRTM 1,195 m) | — | 1,194 m |
| Pedra Grande, Cantareira (~1,010 m) | — | 1,031 m |
| MASP on the Paulista ridge (~820 m) | 821 m | 818 m |
| Pinheiros floor (~720 m) | 728 m | 726 m |
| Tietê floor at the Ponte das Bandeiras (~720 m) | — | 724 m |
| Congonhas (field 802 m; flat at it in `RegionGround`) | — | 793 m at its reference point |
| Campo de Marte (field 722 m) | — | 720 m |

**AVT1** (little-endian): `"AVT1"`, int32 nx, int32 ny, float64 east0, north0
(the south-west sample), float64 spacing, float64 scale (m per unit), then
nx × ny int16, rows from the south, each row west to east.

`RegionGround::setTerrain` takes the region grid (`scenarios::saoPauloTerrain()`
natively, `avd_terrain` in the browser) and replaces any grids set before;
`addTerrain(grid, blend = 2000)` nests a finer one in it
(`scenarios::saoPauloCityTerrain()`, `avd_terrain_detail`;
`scenarios::makeSaoPauloGround()` is the region's ground with both). The
grids are kept coarsest first and the finest wins:

| Where | Ground |
|---|---|
| A field's flat area (its runways and 1.5 km around them) | the field's elevation, as before |
| The next 2 km | the grids, blended into the field's elevation (smoothstep) |
| Inside a nested grid | that grid, bilinear; over its last 2 km inside its edge blended (smoothstep) into the coarser grid under it, so its edge has no step |
| Elsewhere on the region grid | the region grid, bilinear |
| The region grid's last 5 km, and off it | blended into the fields' inverse-distance blend, as before |

Normals come from central differences; vertical rays (`below`) read it
directly; oblique rays march in steps of half the height above the ground
(at least 5 m) and bisect the crossing. The buildings (`BuildingLayer`) stand
on it, the browser's ground mesh samples it (`avd_ground_height`), the radio
altimeter reads it and the TAWS looks ahead along it (`avd/taws.hpp`).

Tested in `tests/test_world.cpp` ("region terrain"): the landmarks on both
grids (the city grid keeps the Jaraguá summit 85 m above the region grid's
rounded cell), the city grid inside it and the region's well off it, no
step across the city grid's edge (just inside and just outside agree within
0.06 m at 1,020 crossings; through the blend no faster than the grids' own
slopes plus 1 m per 25 m), every field flat with no cliff at its edge, a
level ray at 1,110 m meets the Jaraguá summit only with the city grid.

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
