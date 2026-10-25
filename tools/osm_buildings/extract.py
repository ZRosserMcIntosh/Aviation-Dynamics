"""Overture buildings in a lon/lat box -> site frame (EPSG:31983 less the site origin), one JSON line per building."""
import json, sys, concurrent.futures as cf
import pyarrow.parquet as pq, shapely, numpy as np
from pyproj import Transformer
from httpfile import HttpFile
scan = json.load(open(sys.argv[1])); W, S, E, N = map(float, sys.argv[2:6]); out = sys.argv[6]
BASE = 'https://overturemaps-us-west-2.s3.amazonaws.com/'
X0, Y0 = 330986.00, 7393310.72   # the site origin in EPSG:31983 (fitted to its OSM street centrelines, 0.3 m)
T = Transformer.from_crs(4326, 31983, always_xy=True)
COLS = ['id', 'geometry', 'bbox', 'height', 'num_floors', 'min_height', 'min_floor', 'roof_shape', 'roof_height',
        'class', 'subtype', 'sources', 'names', 'is_underground', 'has_parts']
def job(k, g):
    pf = pq.ParquetFile(HttpFile(BASE + k, block=1 << 20))
    have = set(pf.schema_arrow.names)
    t = pf.read_row_group(g, columns=[c for c in COLS if c in have]).to_pylist()
    rows = []
    for r in t:
        b = r['bbox']
        if b['xmax'] < W or b['xmin'] > E or b['ymax'] < S or b['ymin'] > N: continue
        if r.get('is_underground'): continue
        geom = shapely.from_wkb(r['geometry'])
        polys = list(geom.geoms) if geom.geom_type == 'MultiPolygon' else [geom]
        src = sorted({s.get('dataset') for s in (r.get('sources') or []) if s})
        for p in polys:
            if p.geom_type != 'Polygon' or p.is_empty: continue
            ring = np.array(p.exterior.coords)
            x, y = T.transform(ring[:, 0], ring[:, 1])
            holes = []
            for h in p.interiors:
                hr = np.array(h.coords); hx, hy = T.transform(hr[:, 0], hr[:, 1])
                holes.append([[round(a - X0, 2), round(c - Y0, 2)] for a, c in zip(hx, hy)])
            rows.append(dict(id=r['id'], ring=[[round(a - X0, 2), round(c - Y0, 2)] for a, c in zip(x, y)], holes=holes,
                             h=r.get('height'), nf=r.get('num_floors'), mh=r.get('min_height'), mf=r.get('min_floor'),
                             roof=r.get('roof_shape'), rh=r.get('roof_height'), cls=r.get('class'), sub=r.get('subtype'),
                             src=src, name=((r.get('names') or {}).get('primary')), parts=r.get('has_parts')))
    return rows
tasks = [(k, g) for k, gs in scan.items() for g in gs]
n = 0
with open(out, 'w') as f, cf.ThreadPoolExecutor(12) as ex:
    for i, rows in enumerate(ex.map(lambda a: job(*a), tasks)):
        for r in rows: f.write(json.dumps(r, separators=(',', ':')) + '\n')
        n += len(rows)
        if i % 20 == 0: print(i, '/', len(tasks), n, flush=True)
print('buildings', n)
