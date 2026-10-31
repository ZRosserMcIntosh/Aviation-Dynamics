# Terrain: the São Paulo region's ground between the airfields

`make_terrain.py` writes `data/world/terrain_sp.avt`, the ground heights the
core's `RegionGround` uses between the region's airfields (format and use:
[docs/architecture/world-ground.md](../../docs/architecture/world-ground.md)).

```sh
python3 tools/terrain/make_terrain.py            # fetches 9 SRTM tiles (~100 MB) into tools/terrain/.cache
python3 tools/terrain/make_terrain.py --spacing 100 --east -50000 50000 --north -40000 40000
```

- **Data.** SRTM 1 arc-second, NASA / USGS (public domain), read from the AWS
  Terrain Tiles ("skadi") mirror. Only numpy is needed.
- **Projection.** EPSG:31983 (SIRGAS 2000 / UTM 23S) less the map origin
  (330986.00, 7393310.72), the frame the airports, buildings and the site's
  map share (`src/scenarios/airports.hpp`).
- **Ground, not roofs.** Each output cell is the 20th percentile of a 7 × 7
  sample of the elevation model inside it, which takes most of the city's
  buildings and the forest canopy off the surface model. Checked against
  Pico do Jaraguá (1,067 m in its 200 m cell; the summit is 1,135 m), MASP
  (821 m), the Pinheiros floor (728 m) and the port of Santos (4 m)
  (`tests/test_world.cpp`).
