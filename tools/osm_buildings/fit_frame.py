"""The site frame against OpenStreetMap: site_data.json's street centrelines
(sao-paulo repo) fitted by ICP to the same-named OSM roads (roads.py), under
EPSG:31983 less MASP and under a local tangent plane at MASP. Prints each
fit's rotation, scale, translation and residual.

    python3 fit_frame.py <site_data.json> <roads.json>
"""
import sys
import json, math
import numpy as np
from pyproj import Transformer
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree
site = json.load(open(sys.argv[1]))['streets']
roads = json.load(open(sys.argv[2]))
t = Transformer.from_crs(4326, 31983, always_xy=True)
MASP = (-46.6558, -23.5611); mx, my = t.transform(*MASP); lat0 = math.radians(MASP[1])
def utm(lon, lat): x, y = t.transform(lon, lat); return x - mx, y - my
def enu(lon, lat):
    e2 = 0.00669438; R = 6378137.0; s = math.sin(lat0)
    return (math.radians(lon - MASP[0]) * R * math.cos(lat0) / math.sqrt(1 - e2 * s * s), math.radians(lat - MASP[1]) * R * (1 - e2) / (1 - e2 * s * s) ** 1.5)
byname = {}
for n, pts in roads: byname.setdefault(n, []).append(pts)
SP = [(s[0], x, y) for s in site for x, y in s[3] if s[0] in byname and abs(x) < 6000 and abs(y) < 6000]
for proj in (utm, enu):
    lines = {n: [LineString([proj(*p) for p in pl]) for pl in pls] for n, pls in byname.items()}
    trees = {n: STRtree(v) for n, v in lines.items()}
    s, Rm, tr = 1.0, np.eye(2), np.zeros(2)
    for thr in (120, 80, 50, 30, 15, 10, 8, 8):
        # site point -> mapped into proj frame by the inverse of the current fit
        P, Q = [], []
        inv = lambda p: np.linalg.solve(s * Rm, np.array(p) - tr)
        for n, x, y in SP:
            q0 = inv((x, y)); pt = Point(*q0); ln = lines[n][trees[n].nearest(pt)]; q = ln.interpolate(ln.project(pt))
            if math.hypot(q.x - q0[0], q.y - q0[1]) < thr: P.append((x, y)); Q.append((q.x, q.y))
        A, B = np.array(Q), np.array(P); ma, mb = A.mean(0), B.mean(0); A0, B0 = A - ma, B - mb
        U, S_, Vt = np.linalg.svd(A0.T @ B0); Rm = (U @ Vt).T; s = S_.sum() / (A0 ** 2).sum(); tr = mb - s * Rm @ ma
        res = np.hypot(*(B - (s * (Rm @ A.T).T + tr)).T)
    rot = math.degrees(math.atan2(Rm[1, 0], Rm[0, 0]))
    print(f'{proj.__name__}: {len(P)}/{len(SP)} pts, rot {rot:.4f} deg, scale {s:.6f}, t ({tr[0]:.2f}, {tr[1]:.2f}); resid median {np.median(res):.2f} m, p90 {np.percentile(res, 90):.2f}')
