"""OSM buildings (via Overture) -> the flight test bed's building tiles.

Input: buildings.jsonl (site frame: EPSG:31983 less the site origin, m).
Output, in OUT/:
  b_<i>_<j>.bin   every building of 1 km tile (i, j) (its centroid in [i*1000, i*1000+1000) east, same north)
  tall.bin        the buildings 35 m and taller, the whole region (the skyline from afar), grouped by tile
  index.json      tile list with counts, the overview's tile groups, sources and licence

Binary (little-endian): "AVB1", u32 count, then per building:
  u16 height (dm), u16 min_height (dm), u8 kind, u8 rings, then per ring: u16 n, n x (i16 x, i16 y) in dm
  from the tile's south-west corner (the first ring is the outline, the rest courtyards).
"""
import json, math, struct, sys, collections, os
import numpy as np
from shapely.geometry import Polygon
src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
TS = 1000.0
KIND = {'house': 1, 'residential': 1, 'detached': 1, 'semidetached_house': 1, 'apartments': 2, 'commercial': 3, 'office': 3,
        'retail': 3, 'industrial': 4, 'warehouse': 4, 'hangar': 4, 'church': 5, 'cathedral': 5, 'school': 6, 'university': 6,
        'hospital': 6, 'transportation': 7, 'train_station': 7, 'roof': 8, 'garage': 9, 'garages': 9}
tiles = collections.defaultdict(list); seen = set(); stats = collections.Counter()
def height_of(r, area):
    h = r.get('h'); nf = r.get('nf')
    if h and h > 0: stats['h'] += 1; return float(h)
    if nf and nf > 0: stats['floors'] += 1; return 3.1 * nf + 1.0
    stats['estimated'] += 1
    # No height in OSM: by kind and footprint (a house of 1-2 floors; a large footprint a shed or a block).
    k = r.get('cls') or ''
    if k == 'roof': return 4.0
    if area < 60: return 4.0
    if area < 250: return 6.5
    if area < 1500: return 9.0
    return 12.0
for line in open(src):
    try: r = json.loads(line)
    except json.JSONDecodeError: continue
    if 'OpenStreetMap' not in r['src']: continue
    key = (r['id'], round(r['ring'][0][0], 1), round(r['ring'][0][1], 1))
    if key in seen: continue  # a building split over row groups
    seen.add(key)
    try:
        p = Polygon(r['ring'], r['holes'] or None)
        if not p.is_valid: p = p.buffer(0)
        if p.geom_type != 'Polygon' or p.area < 8: continue
        p = p.simplify(0.3, preserve_topology=True)
    except Exception:
        continue
    c = p.centroid
    i, j = math.floor(c.x / TS), math.floor(c.y / TS)
    h = height_of(r, p.area); mh = float(r.get('mh') or 0.0)
    if h <= mh + 0.5: continue
    kind = KIND.get(r.get('cls') or '', 0)
    tiles[(i, j)].append((p, h, mh, kind))
    stats['kept'] += 1
def encode(i, j, items):
    x0, y0 = i * TS, j * TS
    b = bytearray(b'AVB1') + struct.pack('<I', len(items))
    for p, h, mh, kind in items:
        rings = [p.exterior] + list(p.interiors)
        rings = [r for r in rings if len(r.coords) >= 4][:255]
        b += struct.pack('<HHBB', min(int(round(h * 10)), 65535), min(int(round(mh * 10)), 65535), kind, len(rings))
        for rg in rings:
            pts = np.array(rg.coords)[:-1]
            q = np.round((pts - [x0, y0]) * 10).astype(np.int64)
            q = np.clip(q, -32768, 32767)
            b += struct.pack('<H', len(q)) + q.astype('<i2').tobytes()
    return bytes(b)
index = {'format': 'AVB1', 'tile_m': TS, 'frame': 'site frame: EPSG:31983 (SIRGAS 2000 / UTM 23S) less (330986.00, 7393310.72)',
         'source': 'OpenStreetMap buildings via the Overture Maps buildings theme, release 2026-10-21.0 (OSM-sourced footprints only)',
         'licence': 'Data (c) OpenStreetMap contributors, ODbL 1.0 (opendatacommons.org/licenses/odbl); Overture Maps Foundation',
         'tiles': {}, 'tall': {}}
tall = []; total = 0
for (i, j), items in sorted(tiles.items()):
    data = encode(i, j, items)
    open(os.path.join(out, f'b_{i}_{j}.bin'), 'wb').write(data)
    index['tiles'][f'{i},{j}'] = [len(items), len(data)]
    total += len(data)
    tl = [it for it in items if it[1] >= 35]
    if tl: tall.append((i, j, tl))
# The tall overview: groups per tile, each an AVB1 block, behind a small table.
tb = bytearray(b'AVBT') + struct.pack('<I', len(tall))
blocks = []
for i, j, tl in tall:
    blk = encode(i, j, tl); blocks.append(blk); tb += struct.pack('<hhI', i, j, len(blk))
for blk in blocks: tb += blk
open(os.path.join(out, 'tall.bin'), 'wb').write(tb)
index['tall'] = {'buildings': sum(len(t[2]) for t in tall), 'bytes': len(tb)}
index['stats'] = dict(stats)
json.dump(index, open(os.path.join(out, 'index.json'), 'w'), separators=(',', ':'))
print(dict(stats), 'tiles', len(tiles), 'bytes', total, 'tall', index['tall'])
