"""Overture road segments (name, lon/lat line) in a box, from the scanned row groups."""
import json, sys
import pyarrow.parquet as pq, shapely
from httpfile import HttpFile
scan = json.load(open(sys.argv[1])); W, S, E, N = map(float, sys.argv[2:6]); out = sys.argv[6]
BASE = 'https://overturemaps-us-west-2.s3.amazonaws.com/'
res = []
for k, groups in scan.items():
    pf = pq.ParquetFile(HttpFile(BASE + k))
    for g in groups:
        t = pf.read_row_group(g, columns=['geometry', 'bbox', 'names', 'subtype', 'class'])
        bb = t.column('bbox').to_pylist(); nm = t.column('names').to_pylist(); geo = t.column('geometry').to_pylist()
        st = t.column('subtype').to_pylist()
        for i in range(t.num_rows):
            b = bb[i]
            if b['xmax'] < W or b['xmin'] > E or b['ymax'] < S or b['ymin'] > N or st[i] != 'road': continue
            n = (nm[i] or {}).get('primary')
            if not n: continue
            res.append([n, [list(c) for c in shapely.from_wkb(geo[i]).coords]])
json.dump(res, open(out, 'w')); print(len(res))
