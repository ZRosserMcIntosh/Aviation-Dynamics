#!/usr/bin/env python3
# Aviation Dynamics — the São Paulo region's terrain for RegionGround.
# SPDX-License-Identifier: MIT
"""
Builds data/world/terrain_sp.avt: ground heights (m MSL) on a regular grid in
the São Paulo world frame (EPSG:31983, SIRGAS 2000 / UTM 23S, less the map
origin; x east, y north) from the SRTM 1 arc-second elevation model (NASA /
USGS, public domain), read from the AWS Terrain Tiles "skadi" mirror.

    python3 tools/terrain/make_terrain.py [--cache DIR] [--spacing 200]

SRTM is a surface model: in the city its radar return comes from part-way up
the buildings and trees. Each grid cell takes the 20th percentile of the
~40 one-arc-second samples inside it (the open ground between them), which
takes most of that off; voids are filled from their neighbours; the sea is 0.

Format AVT1 (little-endian): b"AVT1", int32 nx, int32 ny, float64 east0,
float64 north0 (the south-west sample, world metres), float64 spacing (m),
float64 scale (m per unit), then nx * ny int16 heights, rows from the south,
each row west to east.
"""
import argparse
import gzip
import math
import os
import struct
import sys
import urllib.request

import numpy as np

ORIGIN_E, ORIGIN_N = 330986.00, 7393310.72   # the map origin (EPSG:31983), as airports.hpp
A_WGS, F_WGS = 6378137.0, 1 / 298.257222101  # GRS80 (SIRGAS 2000)
K0, LON0 = 0.9996, math.radians(-45.0)


def utm_inverse(e, n):
    """EPSG:31983 easting/northing (arrays) -> latitude, longitude (deg). Krüger series to n^4."""
    nn = F_WGS / (2 - F_WGS)
    A = A_WGS / (1 + nn) * (1 + nn ** 2 / 4 + nn ** 4 / 64)
    beta = [nn / 2 - 2 * nn ** 2 / 3 + 37 * nn ** 3 / 96 - nn ** 4 / 360,
            nn ** 2 / 48 + nn ** 3 / 15 - 437 * nn ** 4 / 1440,
            17 * nn ** 3 / 480 - 37 * nn ** 4 / 840,
            4397 * nn ** 4 / 161280]
    xi = (n - 10000000.0) / (K0 * A)
    eta = (e - 500000.0) / (K0 * A)
    xp, ep = xi.copy(), eta.copy()
    for j, b in enumerate(beta, 1):
        xp -= b * np.sin(2 * j * xi) * np.cosh(2 * j * eta)
        ep -= b * np.cos(2 * j * xi) * np.sinh(2 * j * eta)
    chi = np.arcsin(np.sin(xp) / np.cosh(ep))
    lon = LON0 + np.arctan2(np.sinh(ep), np.cos(xp))
    # Conformal latitude -> geodetic (series in n).
    d = [2 * nn - 2 * nn ** 2 / 3 - 2 * nn ** 3 + 116 * nn ** 4 / 45,
         7 * nn ** 2 / 3 - 8 * nn ** 3 / 5 - 227 * nn ** 4 / 45,
         56 * nn ** 3 / 15 - 136 * nn ** 4 / 35,
         4279 * nn ** 4 / 630]
    lat = chi + sum(dj * np.sin(2 * j * chi) for j, dj in enumerate(d, 1))
    return np.degrees(lat), np.degrees(lon)


def tile(cache, lat0, lon0):
    """The SRTM tile whose south-west corner is (lat0, lon0) integers, as an int16 array (row 0 north)."""
    name = f"{'S' if lat0 < 0 else 'N'}{abs(lat0):02d}{'W' if lon0 < 0 else 'E'}{abs(lon0):03d}"
    path = os.path.join(cache, name + '.hgt.gz')
    if not os.path.exists(path):
        url = f"https://s3.amazonaws.com/elevation-tiles-prod/skadi/{name[:3]}/{name}.hgt.gz"
        print('fetching', url, file=sys.stderr)
        urllib.request.urlretrieve(url, path + '.part')
        os.replace(path + '.part', path)
    raw = gzip.open(path).read()
    n = math.isqrt(len(raw) // 2)
    return np.frombuffer(raw, '>i2').reshape(n, n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cache', default=os.path.join(os.path.dirname(__file__), '.cache'))
    ap.add_argument('--spacing', type=float, default=200.0)
    ap.add_argument('--east', type=float, nargs=2, default=[-110000.0, 110000.0])
    ap.add_argument('--north', type=float, nargs=2, default=[-90000.0, 90000.0])
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', '..', 'data', 'world', 'terrain_sp.avt'))
    a = ap.parse_args()
    os.makedirs(a.cache, exist_ok=True)
    sp = a.spacing
    xs = np.arange(a.east[0], a.east[1] + sp / 2, sp)
    ys = np.arange(a.north[0], a.north[1] + sp / 2, sp)
    nx, ny = len(xs), len(ys)
    # Each cell: a 7 x 7 sub-grid of samples across it (about the 1" spacing
    # at 200 m), a band of rows at a time.
    sub = 7
    off = (np.arange(sub) + 0.5) / sub - 0.5
    tiles = {}
    cell = np.full((ny, nx), np.nan)
    band = 40
    for y0 in range(0, ny, band):
        yb = ys[y0:y0 + band]
        E = xs[None, :, None, None] + off[None, None, None, :] * sp
        N = yb[:, None, None, None] + off[None, None, :, None] * sp
        E, N = np.broadcast_arrays(E, N)
        lat, lon = utm_inverse(E + ORIGIN_E, N + ORIGIN_N)
        h = np.full(lat.shape, np.nan)
        for la in range(int(math.floor(lat.min())), int(math.floor(lat.max())) + 1):
            for lo in range(int(math.floor(lon.min())), int(math.floor(lon.max())) + 1):
                m = (lat >= la) & (lat < la + 1) & (lon >= lo) & (lon < lo + 1)
                if not m.any():
                    continue
                if (la, lo) not in tiles:
                    tiles[(la, lo)] = tile(a.cache, la, lo)
                t = tiles[(la, lo)]
                n = t.shape[0] - 1
                fr = (la + 1 - lat[m]) * n   # rows from the north edge
                fc = (lon[m] - lo) * n
                r0 = np.clip(np.floor(fr).astype(int), 0, n - 1)
                c0 = np.clip(np.floor(fc).astype(int), 0, n - 1)
                u, v = fr - r0, fc - c0
                q = [t[r0, c0].astype(float), t[r0, c0 + 1].astype(float), t[r0 + 1, c0].astype(float), t[r0 + 1, c0 + 1].astype(float)]
                void = (q[0] < -1000) | (q[1] < -1000) | (q[2] < -1000) | (q[3] < -1000)
                val = q[0] * (1 - u) * (1 - v) + q[1] * (1 - u) * v + q[2] * u * (1 - v) + q[3] * u * v
                val[void] = np.nan
                h[m] = val
        cell[y0:y0 + len(yb)] = np.nanpercentile(h.reshape(len(yb), nx, sub * sub), 20, axis=2)
        print(f'  rows {y0 + len(yb)}/{ny}', file=sys.stderr, end='\r')
    # Voids: fill from the neighbours, growing inward.
    for _ in range(50):
        bad = np.isnan(cell)
        if not bad.any():
            break
        p = np.pad(cell, 1, constant_values=np.nan)
        nb = np.stack([p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
        fill = np.nanmean(np.where(np.isnan(nb), np.nan, nb), axis=0)
        cell[bad] = fill[bad]
    cell = np.nan_to_num(cell, nan=0.0)
    cell = np.maximum(cell, 0.0)  # the sea
    scale = 0.1
    q = np.round(cell / scale).astype('<i2')
    with open(a.out, 'wb') as f:
        f.write(b'AVT1' + struct.pack('<ii', nx, ny) + struct.pack('<dddd', xs[0], ys[0], sp, scale))
        f.write(q.tobytes())
    print(f'wrote {a.out}: {nx} x {ny} at {sp:.0f} m, {cell.min():.0f} to {cell.max():.0f} m', file=sys.stderr)


if __name__ == '__main__':
    with np.errstate(invalid='ignore'):
        main()
