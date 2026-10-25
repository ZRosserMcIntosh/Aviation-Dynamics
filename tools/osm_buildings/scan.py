"""Find the Overture files and row groups (buildings, or the theme/type given) whose bbox statistics meet a lon/lat box."""
import json, re, sys, urllib.request, concurrent.futures as cf
import pyarrow.parquet as pq
from httpfile import HttpFile
REL = sys.argv[1]; W, S, E, N = map(float, sys.argv[2:6]); out = sys.argv[6]; THEME = sys.argv[7] if len(sys.argv) > 7 else 'buildings/type=building'
BASE = 'https://overturemaps-us-west-2.s3.amazonaws.com/'
t = urllib.request.urlopen(f'{BASE}?list-type=2&prefix=release/{REL}/theme={THEME}/&max-keys=1000', timeout=60).read().decode()
keys = re.findall(r'<Key>([^<]*)</Key>', t)
def scan(k):
    f = HttpFile(BASE + k, block=1 << 16)
    md = pq.ParquetFile(f).metadata
    names = [md.schema.column(i).path for i in range(md.num_columns)]
    ix = {n: names.index(n) for n in ('bbox.xmin', 'bbox.xmax', 'bbox.ymin', 'bbox.ymax')}
    hits = []
    for g in range(md.num_row_groups):
        rg = md.row_group(g)
        st = {n: rg.column(i).statistics for n, i in ix.items()}
        if any(s is None or not s.has_min_max for s in st.values()): hits.append(g); continue
        if st['bbox.xmin'].min > E or st['bbox.xmax'].max < W or st['bbox.ymin'].min > N or st['bbox.ymax'].max < S: continue
        hits.append(g)
    return k, hits, md.num_row_groups
res = {}
with cf.ThreadPoolExecutor(16) as ex:
    for k, hits, n in ex.map(scan, keys):
        if hits: res[k] = hits; print(k.split('/')[-1][:12], len(hits), '/', n, flush=True)
json.dump(res, open(out, 'w'))
print('files', len(res), 'row groups', sum(len(v) for v in res.values()))
