# Terrain: the São Paulo region's ground between the airfields

`make_terrain.py` writes the ground heights the core's `RegionGround` uses
between the region's airfields (format and use:
[docs/architecture/world-ground.md](../../docs/architecture/world-ground.md)):

- `data/world/terrain_sp.avt`, the region: 200 m over 220 × 180 km around
  MASP (2.0 MB);
- `data/world/terrain_sp_city.avt`, the city (`--detail`): 50 m over 50 × 62 km
  (the São Paulo municipality to the Billings and Guarapiranga reservoirs, the
  Pico do Jaraguá, the Serra da Cantareira, Guarulhos; 2.5 MB), nested in the
  region's (`RegionGround::addTerrain`, `avd_terrain_detail` in the browser).

```sh
python3 tools/terrain/make_terrain.py            # fetches 9 SRTM tiles (~100 MB) into tools/terrain/.cache
python3 tools/terrain/make_terrain.py --detail   # the city grid from the same tiles
python3 tools/terrain/make_terrain.py --spacing 100 --east -50000 50000 --north -40000 40000 --out /tmp/t.avt
```

- **Data.** SRTM 1 arc-second, NASA / USGS (public domain), read from the AWS
  Terrain Tiles ("skadi") mirror. Only numpy is needed.
- **Projection.** EPSG:31983 (SIRGAS 2000 / UTM 23S) less the map origin
  (330986.00, 7393310.72), the frame the airports, buildings and the site's
  map share (`src/scenarios/airports.hpp`).
- **Ground, not roofs: the region.** Each output cell is the 20th percentile
  of a 7 × 7 sample of the elevation model inside it, which takes most of
  the city's buildings and the forest canopy off the surface model. It
  rounds the peaks: Pico do Jaraguá is 1,067 m in its 200 m cell (1,033 m at
  the summit's own position) against 1,135 m surveyed.
- **Ground, not roofs: the city.** A 50 m cell holds two or three samples,
  so a percentile cannot work. On a 25 m resampling: the model's
  morphological opening over 400 m (the local minimum, then the local
  maximum) is taken away from it, at most 10 m, where the terrain is gentle
  (in full below 8 % slope, none above 20 %, the steepest within 400 m):
  blocks of towers and stands of trees come down to the ground around them,
  while a summit or a ridge, steep all round, keeps its height. The
  correction is smoothed over 150 m before it is taken off, so the peaks
  stay sharp. Pico do Jaraguá 1,118 m (SRTM's summit 1,132 m), the
  Cantareira's top 1,194 m, MASP 818 m, the Pinheiros floor 726 m, the Tietê
  724 m, Campo de Marte 720 m (field 722 m), Congonhas' plateau 793 m
  (field 802 m) (`tests/test_world.cpp`). The constants are at the top of
  the script (`OPEN_M`, `CAP_M`, `SLOPE0`, `SLOPE1`, `SMOOTH_M`).
- **Rebuilt by the same script.** `make_terrain.py` without `--detail`
  writes `terrain_sp.avt` byte for byte as before.
