#!/usr/bin/env python3
"""Rasterize the rooms.world geometry into a Nav2 map (PGM + YAML)."""
import math, os
RES = 0.05
SIZE_M = 12.2
ORIGIN = (-6.1, -6.1)
N = int(SIZE_M / RES)
FREE, OCC, UNK = 254, 0, 205

# rectangles: (cx, cy, sx, sy)
segs = [(-6, -3.7), (-2.3, 2.3), (3.7, 6)]
inner = []
for a, b in segs:
    inner.append((0, (a + b) / 2, 0.2, b - a))
    inner.append(((a + b) / 2, 0, b - a, 0.2))
rects = inner + [(0, 6, 12.2, .2), (0, -6, 12.2, .2), (6, 0, .2, 12.2), (-6, 0, .2, 12.2),
                 (-4.5, 4.5, .8, .8), (3, 4, 1.0, .6), (-3, -4, .8, 1.2),
                 (3.5, -4, 1.0, 1.0), (-1.2, 2.8, .4, .4)]
circles = [(-2, 4.2, .4), (4.8, 2, .35), (-5, -2, .4), (4.8, -1.8, .3)]

grid = [[UNK] * N for _ in range(N)]          # row 0 = top (max y)
for r in range(N):
    for c in range(N):
        x = ORIGIN[0] + (c + .5) * RES
        y = ORIGIN[1] + (N - r - .5) * RES
        v = FREE
        for cx, cy, sx, sy in rects:
            if abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2:
                v = OCC
        for cx, cy, rad in circles:
            if math.hypot(x - cx, y - cy) <= rad:
                v = OCC
        grid[r][c] = v

out = os.path.join(os.path.dirname(__file__), '..', 'maps')
with open(os.path.join(out, 'rooms_map.pgm'), 'wb') as f:
    f.write(f'P5\n{N} {N}\n255\n'.encode())
    for row in grid:
        f.write(bytes(row))
with open(os.path.join(out, 'rooms_map.yaml'), 'w') as f:
    f.write(f"""image: rooms_map.pgm
mode: trinary
resolution: {RES}
origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")
print('map written', N, 'x', N)
