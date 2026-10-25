"""OSM buildings (via Overture) -> the flight test bed's building tiles.

Input: buildings.jsonl (site frame: EPSG:31983 less the site origin, m).
With --ml, the machine-traced footprints (Google Open Buildings, Microsoft
ML Buildings) are added where no OpenStreetMap building overlaps them, their
heights estimated from the mapped OSM heights around them.
Output, in OUT/ (each binary gzipped, as .bin.gz: the browser inflates it):
  b_<i>_<j>.bin   every building of 1 km tile (i, j) (its centroid in [i*1000, i*1000+1000) east, same north)
  tall.bin        the buildings 35 m and taller, the whole region (the skyline from afar), grouped by tile
  index.json      tile list with counts, the overview's tile groups, sources and licence

Binary (little-endian): "AVB1", u32 count, then per building:
  u16 height (dm), u16 min_height (dm), u8 kind, u8 rings, then per ring: u16 n, n x (i16 x, i16 y) in dm
  from the tile's south-west corner (the first ring is the outline, the rest courtyards).
"""
import gzip, json, math, struct, sys, collections, os
import numpy as np
from shapely.geometry import Polygon
from shapely.strtree import STRtree
args = [a for a in sys.argv[1:] if not a.startswith('--')]
src, out = args[0], args[1]
ML = '--ml' in sys.argv
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
def polygon(r):
    try:
        p = Polygon(r['ring'], r['holes'] or None)
        if not p.is_valid: p = p.buffer(0)
        if p.geom_type != 'Polygon' or p.area < 8: return None
        return p.simplify(0.3, preserve_topology=True)
    except Exception:
        return None
# Mapped OSM heights by 250 m cell and footprint size (for the machine-traced
# footprints' estimate): small < 150 m2, medium < 600, large.
CELL = 250.0
size_class = lambda a: 0 if a < 150 else 1 if a < 600 else 2
mapped = collections.defaultdict(list)
pending = []  # machine-traced footprints, placed after every OSM building is known
for line in open(src):
    try: r = json.loads(line)
    except json.JSONDecodeError: continue
    osm = 'OpenStreetMap' in r['src']
    if not osm and not ML: continue
    key = (r['id'], round(r['ring'][0][0], 1), round(r['ring'][0][1], 1))
    if key in seen: continue  # a building split over row groups
    seen.add(key)
    p = polygon(r)
    if p is None: continue
    if not osm:
        pending.append((p, 'Microsoft' if 'Microsoft ML Buildings' in r['src'] else 'Google'))
        continue
    c = p.centroid
    i, j = math.floor(c.x / TS), math.floor(c.y / TS)
    h = height_of(r, p.area); mh = float(r.get('mh') or 0.0)
    if h <= mh + 0.5: continue
    kind = KIND.get(r.get('cls') or '', 0)
    tiles[(i, j)].append((p, h, mh, kind))
    stats['kept'] += 1
    if r.get('h') and mh == 0:
        mapped[(math.floor(c.x / CELL), math.floor(c.y / CELL), size_class(p.area))].append(h)
def estimate(c, a):
    """A machine-traced footprint's height: the median mapped height of the
    OSM buildings of its size within ~375 m (8 or more of them), else by its
    footprint (as an unmapped OSM building's)."""
    ci, cj, k = math.floor(c.x / CELL), math.floor(c.y / CELL), size_class(a)
    hs = [h for di in (-1, 0, 1) for dj in (-1, 0, 1) for h in mapped.get((ci + di, cj + dj, k), ())]
    # Capped by its size: a house-sized footprint is no tower (12 m), and a big
    # one (over 1,500 m2: a shed, a mall, a hangar) keeps the footprint's estimate,
    # however tall the blocks around it.
    if len(hs) >= 8 and a <= 1500:
        stats['ml_neighbours'] += 1
        return float(min(max(np.median(hs), 3.0), 12.0 if k == 0 else 45.0))
    stats['ml_footprint'] += 1
    return 4.0 if a < 60 else 6.5 if a < 250 else 9.0 if a < 1500 else 12.0
# Each machine-traced footprint where no OSM building overlaps it (Overture
# conflates them so already; checked here, more than 10 % of its area).
osm_tree = {k: STRtree([it[0] for it in v]) for k, v in tiles.items()}
osm_polys = {k: [it[0] for it in v] for k, v in tiles.items()}
for p, who in pending:
    c = p.centroid
    i, j = math.floor(c.x / TS), math.floor(c.y / TS)
    clash = False
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            t = osm_tree.get((i + di, j + dj))
            if t is None: continue
            for q in t.query(p):
                o = osm_polys[(i + di, j + dj)][q]
                if o.intersects(p) and o.intersection(p).area > 0.1 * p.area: clash = True; break
            if clash: break
        if clash: break
    if clash:
        stats['ml_overlapping_osm'] += 1
        continue
    tiles[(i, j)].append((p, estimate(c, p.area), 0.0, 0))
    stats['ml_' + who.lower()] += 1
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
         'source': ('OpenStreetMap buildings, and where OSM has none the Google Open Buildings and Microsoft ML Buildings footprints '
                    '(heights estimated), via the Overture Maps buildings theme, release 2026-10-21.0' if ML else
                    'OpenStreetMap buildings via the Overture Maps buildings theme, release 2026-10-21.0 (OSM-sourced footprints only)'),
         'licence': ('Building data (c) OpenStreetMap contributors, Microsoft, Google Open Buildings; ODbL 1.0 '
                     '(opendatacommons.org/licenses/odbl); Overture Maps Foundation' if ML else
                     'Data (c) OpenStreetMap contributors, ODbL 1.0 (opendatacommons.org/licenses/odbl); Overture Maps Foundation'),
         'tiles': {}, 'tall': {}}
tall = []; total = 0
for (i, j), items in sorted(tiles.items()):
    data = encode(i, j, items)
    gz = gzip.compress(data, 9, mtime=0)
    open(os.path.join(out, f'b_{i}_{j}.bin.gz'), 'wb').write(gz)
    index['tiles'][f'{i},{j}'] = [len(items), len(gz)]
    total += len(gz)
    tl = [it for it in items if it[1] >= 35]
    if tl: tall.append((i, j, tl))
# The tall overview: groups per tile, each an AVB1 block, behind a small table.
tb = bytearray(b'AVBT') + struct.pack('<I', len(tall))
blocks = []
for i, j, tl in tall:
    blk = encode(i, j, tl); blocks.append(blk); tb += struct.pack('<hhI', i, j, len(blk))
for blk in blocks: tb += blk
tgz = gzip.compress(bytes(tb), 9, mtime=0)
open(os.path.join(out, 'tall.bin.gz'), 'wb').write(tgz)
index['tall'] = {'buildings': sum(len(t[2]) for t in tall), 'bytes': len(tgz)}
index['stats'] = dict(stats)
json.dump(index, open(os.path.join(out, 'index.json'), 'w'), separators=(',', ':'))
print(dict(stats), 'tiles', len(tiles), 'bytes', total, 'tall', index['tall'])
