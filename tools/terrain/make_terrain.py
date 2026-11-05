#!/usr/bin/env python3
# Aviation Dynamics — the São Paulo region's terrain for RegionGround.
# SPDX-License-Identifier: MIT
"""
Builds the ground heights (m MSL) RegionGround uses, on regular grids in the
São Paulo world frame (EPSG:31983, SIRGAS 2000 / UTM 23S, less the map
origin; x east, y north), from the SRTM 1 arc-second elevation model (NASA /
USGS, public domain), read from the AWS Terrain Tiles "skadi" mirror.

    python3 tools/terrain/make_terrain.py [--cache DIR]            # data/world/terrain_sp.avt
    python3 tools/terrain/make_terrain.py --detail [--cache DIR]   # data/world/terrain_sp_city.avt

SRTM is a surface model: in the city its radar return comes from part-way up
the buildings and trees. Two ground filters take most of that off:

- the region grid (200 m over 220 x 180 km): each cell takes the 20th
  percentile of the ~40 one-arc-second samples inside it (the open ground
  between them);
- the city grid (--detail: 50 m over the São Paulo municipality, the Pico do
  Jaraguá and the Serra da Cantareira), where a 50 m cell holds only two or
  three samples, so a percentile has nothing to choose from. Instead, on a
  25 m resampling of the model: its morphological opening (the local
  minimum, then the local maximum, over a square of OPEN_M) keeps planar
  slopes and valleys as they are and takes off what stands above the ground
  over less than that width -- blocks of buildings and stands of trees, but
  also the tops of hills. So the correction (the surface less its opening,
  at most CAP_M) is applied in full only where the terrain is gentle: it
  fades out between SLOPE0 and SLOPE1, the steepest slope of the smoothed
  model within OPEN_M, and a summit or a ridge, with steep ground all round
  it, keeps its height. The correction (not the ground) is smoothed over
  SMOOTH_M before it is taken off, so the peaks stay sharp; the result is
  sampled at 50 m.

Voids are filled from their neighbours; the sea is 0.

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
HERE = os.path.dirname(__file__)
WORLD = os.path.join(HERE, '..', '..', 'data', 'world')

# The city grid's ground filter (see the module docstring).
FINE_M = 25.0                 # resampling of the surface model for the filter (m)
OPEN_M = 400.0                # width of the opening's square (m)
CAP_M = 10.0                  # the most taken off (m): the surface model's bias over the high-rises
SLOPE0, SLOPE1 = 0.08, 0.20   # the correction in full below SLOPE0, none above SLOPE1
SMOOTH_M = 150.0              # the correction's smoothing (box, m)


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


def dem(cache, tiles, E, N):
    """The surface model (m) at world points (arrays, map metres), bilinear; NaN in voids."""
    lat, lon = utm_inverse(E + ORIGIN_E, N + ORIGIN_N)
    h = np.full(lat.shape, np.nan)
    for la in range(int(math.floor(lat.min())), int(math.floor(lat.max())) + 1):
        for lo in range(int(math.floor(lon.min())), int(math.floor(lon.max())) + 1):
            m = (lat >= la) & (lat < la + 1) & (lon >= lo) & (lon < lo + 1)
            if not m.any():
                continue
            if (la, lo) not in tiles:
                tiles[(la, lo)] = tile(cache, la, lo)
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
    return h


def fill_voids(cell):
    """Voids: fill from the neighbours, growing inward; the sea at 0."""
    for _ in range(50):
        bad = np.isnan(cell)
        if not bad.any():
            break
        p = np.pad(cell, 1, constant_values=np.nan)
        nb = np.stack([p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
        fill = np.nanmean(np.where(np.isnan(nb), np.nan, nb), axis=0)
        cell[bad] = fill[bad]
    cell = np.nan_to_num(cell, nan=0.0)
    return np.maximum(cell, 0.0)


def write_avt(path, xs, ys, sp, cell):
    scale = 0.1
    q = np.round(cell / scale).astype('<i2')
    with open(path, 'wb') as f:
        f.write(b'AVT1' + struct.pack('<ii', len(xs), len(ys)) + struct.pack('<dddd', xs[0], ys[0], sp, scale))
        f.write(q.tobytes())
    print(f'wrote {path}: {len(xs)} x {len(ys)} at {sp:.0f} m, {cell.min():.0f} to {cell.max():.0f} m, '
          f'{os.path.getsize(path)} bytes', file=sys.stderr)


def region(a, xs, ys):
    """The region grid: the 20th percentile of a 7 x 7 sample of each cell."""
    sp = a.spacing
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
        h = dem(a.cache, tiles, E, N)
        cell[y0:y0 + len(yb)] = np.nanpercentile(h.reshape(len(yb), nx, sub * sub), 20, axis=2)
        print(f'  rows {y0 + len(yb)}/{ny}', file=sys.stderr, end='\r')
    return fill_voids(cell)


def _running(a, r, axis, fn):
    """fn (np.min / np.max / np.mean) over a window of 2r + 1 along `axis`, edges replicated."""
    pad = [(0, 0)] * a.ndim
    pad[axis] = (r, r)
    p = np.pad(a, pad, mode='edge')
    w = np.lib.stride_tricks.sliding_window_view(p, 2 * r + 1, axis=axis)
    return fn(w, axis=-1)


def square(a, r, fn):
    return _running(_running(a, r, 0, fn), r, 1, fn)


def ground_filter(s, step):
    """The ground under surface model `s` (m, a grid at `step` m): the opening's correction where the terrain is gentle."""
    r = int(round(OPEN_M / 2 / step))
    opened = square(square(s, r, np.min), r, np.max)
    # The terrain's slope, from the model smoothed over the window (the
    # buildings' edges are gone from it), the steepest within the window:
    # round a summit or along a ridge the slopes are steep.
    gy, gx = np.gradient(square(s, r, np.mean), step)
    slope = square(np.hypot(gx, gy), r, np.max)
    w = np.clip((SLOPE1 - slope) / (SLOPE1 - SLOPE0), 0.0, 1.0)
    c = np.minimum(s - opened, CAP_M) * w
    return s - square(c, max(1, int(round(SMOOTH_M / 2 / step))), np.mean)


def detail(a, xs, ys):
    """The city grid: the ground filter on a FINE_M resampling, then sampled at the grid spacing."""
    sp = a.spacing
    m = int(round(OPEN_M / FINE_M)) + 4   # margin so the window sees beyond the grid's edge
    fx = np.arange(xs[0] - m * FINE_M, xs[-1] + (m + 0.5) * FINE_M, FINE_M)
    fy = np.arange(ys[0] - m * FINE_M, ys[-1] + (m + 0.5) * FINE_M, FINE_M)
    tiles = {}
    s = np.full((len(fy), len(fx)), np.nan)
    band = 200
    for y0 in range(0, len(fy), band):
        E, N = np.meshgrid(fx, fy[y0:y0 + band])
        s[y0:y0 + E.shape[0]] = dem(a.cache, tiles, E, N)
        print(f'  rows {y0 + E.shape[0]}/{len(fy)}', file=sys.stderr, end='\r')
    s = fill_voids(s)
    g = ground_filter(s, FINE_M)
    # Bilinear onto the output grid.
    u = (xs - fx[0]) / FINE_M
    v = (ys - fy[0]) / FINE_M
    i = np.clip(np.floor(u).astype(int), 0, len(fx) - 2)
    j = np.clip(np.floor(v).astype(int), 0, len(fy) - 2)
    du, dv = (u - i)[None, :], (v - j)[:, None]
    J, I = j[:, None], i[None, :]
    out = (g[J, I] * (1 - du) + g[J, I + 1] * du) * (1 - dv) + (g[J + 1, I] * (1 - du) + g[J + 1, I + 1] * du) * dv
    return np.maximum(out, 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cache', default=os.path.join(HERE, '.cache'))
    ap.add_argument('--detail', action='store_true',
                    help='the city grid (50 m, morphological ground filter) instead of the region grid')
    ap.add_argument('--spacing', type=float)
    ap.add_argument('--east', type=float, nargs=2)
    ap.add_argument('--north', type=float, nargs=2)
    ap.add_argument('--out')
    a = ap.parse_args()
    if a.detail:
        # The São Paulo municipality as far south as the Billings and
        # Guarapiranga reservoirs, the Pico do Jaraguá and the Serra da
        # Cantareira: 50 x 62 km.
        defaults = dict(spacing=50.0, east=[-20000.0, 30000.0], north=[-38000.0, 24000.0], out='terrain_sp_city.avt')
    else:
        defaults = dict(spacing=200.0, east=[-110000.0, 110000.0], north=[-90000.0, 90000.0], out='terrain_sp.avt')
    for k, v in defaults.items():
        if getattr(a, k) is None:
            setattr(a, k, os.path.join(WORLD, v) if k == 'out' else v)
    os.makedirs(a.cache, exist_ok=True)
    sp = a.spacing
    xs = np.arange(a.east[0], a.east[1] + sp / 2, sp)
    ys = np.arange(a.north[0], a.north[1] + sp / 2, sp)
    cell = detail(a, xs, ys) if a.detail else region(a, xs, ys)
    write_avt(a.out, xs, ys, sp, cell)


if __name__ == '__main__':
    with np.errstate(invalid='ignore'):
        main()
