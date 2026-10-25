# OpenStreetMap buildings for the flight test bed

The city's buildings in the São Paulo flight test bed
(zrossermcintosh.com/sao-paulo/flying) are OpenStreetMap footprints and heights.
OSM's own servers are not reachable from the build machines, so the data is
taken from the [Overture Maps](https://overturemaps.org) buildings theme. That
theme carries every OSM building with its source and its `height` and
`building:levels` tags. Where OSM has no building, the theme has the footprints
traced from imagery by Google (Open Buildings) and Microsoft (ML Buildings);
`tile.py --ml` adds them, with estimated heights (below).

Building data © OpenStreetMap contributors, Microsoft, Google Open Buildings,
ODbL 1.0 (the Overture buildings theme's licence). The tiles are a derived
database under the same licence; the site credits all three wherever the
buildings are shown.

## Pipeline

```sh
# 1. The row groups whose bbox statistics meet the region (footers only, HTTP range reads).
python3 scan.py 2026-10-21.0 -46.83 -23.75 -46.36 -23.38 scan.json
# 2. The buildings of those row groups, into the site frame (one JSON line each).
python3 extract.py scan.json -46.83 -23.75 -46.36 -23.38 buildings.jsonl
# 3. 1 km binary tiles (gzipped), the tall-building overview and index.json;
#    --ml adds the machine-traced footprints where OSM has none.
python3 tile.py buildings.jsonl <sao-paulo repo>/sao-paulo/flying/buildings --ml
```

Requirements: `pyarrow`, `shapely`, `pyproj`, `numpy`. `httpfile.py` reads
Parquet over HTTP range requests (through `HTTPS_PROXY`).

## The site frame

The site, and the SaoPauloWorld project it comes from, uses EPSG:31983
(SIRGAS 2000 / UTM 23S) less an origin near MASP. `fit_frame.py` fits
`site_data.json`'s street centrelines to the same-named OSM roads (`roads.py`
fetches them) by ICP:

| Frame | Rotation | Scale | Translation (m) | Residual median / p90 |
|---|---|---|---|---|
| EPSG:31983 less MASP (-46.6558, -23.5611) | 0.0001° | 1.000002 | (15.76, 45.26) | 0.30 / 0.57 m |
| Tangent plane at MASP | 0.659° | 0.99994 | (15.73, 44.98) | 0.41 / 0.87 m |

```sh
python3 scan.py 2026-10-21.0 -46.70 -23.60 -46.60 -23.52 scan_roads.json transportation/type=segment
python3 roads.py scan_roads.json -46.70 -23.60 -46.60 -23.52 roads.json
python3 fit_frame.py <sao-paulo repo>/sao-paulo/site_data.json roads.json
```

So the site origin is EPSG:31983 (330986.00, 7393310.72) and there is no
rotation. A tangent plane would be 0.66° off, about 115 m at 10 km.

## Tiles

`b_<i>_<j>.bin.gz` holds every building whose centroid lies in the 1 km
square from (i·1000, j·1000) east and north. Each tile is a gzipped "AVB1"
block (the browser inflates it; gzip takes the tiles to ~70 %), little-endian:

```
"AVB1", u32 count, then per building:
  u16 height (dm), u16 min_height (dm), u8 kind, u8 rings,
  per ring: u16 n, n × (i16 x, i16 y) in dm from the tile's south-west corner
  (the first ring is the outline, the rest courtyards)
```

`tall.bin.gz` is the skyline overview: "AVBT", u32 groups, a table of
(i16 i, i16 j, u32 bytes), then one AVB1 block per tile with the buildings
35 m and taller.

Heights come from the OSM `height` tag (95 % of the OSM buildings have one,
most from the GeoSampa import of São Paulo municipality), else from
`building:levels` × 3.1 m + 1 m. Where neither is mapped, the height is
estimated from the footprint (4–12 m). Tiles are simplified to 0.3 m.

The machine-traced footprints have no heights. Each takes the median mapped
height of the OSM buildings of its size class (under 150 m², under 600 m²,
larger) in the 750 m square around it, when there are 8 or more: at most
12 m for a house-sized footprint, 45 m for the others. A footprint over
1,500 m² (a shed, a mall, a hangar), or one with too few mapped neighbours,
takes the footprint estimate. A machine-traced footprint overlapping an OSM
building by more than 10 % of its area is dropped (none is: Overture
conflates them).

The flight core reads the same blocks: `BuildingLayer::parseAvb1` (and
`avd_buildings_tile` in the browser) stands each building on the ground under
it and makes its walls and roofs solid.

Release 2026-10-21.0, region lon −46.83…−46.36, lat −23.75…−23.38, with
`--ml`: 4,447,982 buildings in 2,001 tiles (93 MB gzipped):

- OpenStreetMap: 2,070,013 (1,968,323 with a mapped height, 11,519 from
  levels, 90,192 estimated).
- Google Open Buildings: 2,109,798; Microsoft ML Buildings: 268,171
  (506,140 heights from their neighbours, 1,871,829 from the footprint).

24,250 buildings 35 m and taller are in the overview (1.0 MB).
