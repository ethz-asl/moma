#!/usr/bin/env python

# Core logic file: sweep_search_core.py

import numpy as np
from math import hypot

def find_valid_sweeps_45(grid, min_len=1, max_len=16, width=4):
    h, w = grid.shape
    valid_sweeps = []
    total_checked = 0

    directions = [
        (1, 0),   # right
        (0, 1),   # down
        (1, 1),   # diagonal down-right
        (1, -1),  # diagonal up-right
    ]

    for y0 in range(h):
        for x0 in range(w):
            for dx, dy in directions:
                for l in range(min_len, max_len + 1):
                    x1 = x0 + dx * (l - 1)
                    y1 = y0 + dy * (l - 1)

                    # Check bounds
                    if not (0 <= x1 < w and 0 <= y1 < h):
                        continue

                    # Get perpendicular vector for effector width
                    perp = np.array([-dy, dx])
                    for offset in range(-(width // 2), (width + 1) // 2):
                        sx = x0 + perp[0] * offset
                        sy = y0 + perp[1] * offset
                        ex = x1 + perp[0] * offset
                        ey = y1 + perp[1] * offset

                        # Bounds check for effector swath
                        if not (0 <= sx < w and 0 <= sy < h and 0 <= ex < w and 0 <= ey < h):
                            break
                    else:
                        # All effector lines are in bounds — now check non-zero height at start
                        start_vals = []
                        for offset in range(-(width // 2), (width + 1) // 2):
                            sx = x0 + perp[0] * offset
                            sy = y0 + perp[1] * offset
                            start_vals.append(grid[sy, sx])

                        if np.any(start_vals):
                            valid_sweeps.append({
                                'start': (x0, y0),
                                'end': (x1, y1),
                                'direction': (dx, dy),
                                'length': l,
                                'namespace': f"length_{l}"
                            })
                    total_checked += 1
    return valid_sweeps, total_checked

def find_valid_sweeps(grid, min_len=1, max_len=16, width=4):
    h, w = grid.shape
    valid_sweeps = []
    total_checked = 0

    for y0 in range(h):
        for x0 in range(w):
            for y1 in range(h):
                for x1 in range(w):
                    if (x0, y0) == (x1, y1):
                        continue

                    dx = x1 - x0
                    dy = y1 - y0
                    dist = hypot(dx, dy)
                    if dist < min_len or dist > max_len:
                        continue

                    steps = int(np.ceil(dist)) + 1
                    line_x = np.linspace(x0, x1, steps)
                    line_y = np.linspace(y0, y1, steps)

                    # Perpendicular vector
                    norm = np.array([-dy, dx], dtype=float)
                    norm_len = np.linalg.norm(norm)
                    if norm_len == 0:
                        continue
                    perp = norm / norm_len

                    half_w = width / 2.0

                    # Check swath bounds
                    in_bounds = True
                    for t in np.linspace(-half_w + 0.5, half_w - 0.5, width):
                        offset_x = perp[0] * t
                        offset_y = perp[1] * t

                        sx = int(round(x0 + offset_x))
                        sy = int(round(y0 + offset_y))
                        ex = int(round(x1 + offset_x))
                        ey = int(round(y1 + offset_y))

                        if not (0 <= sx < w and 0 <= sy < h and 0 <= ex < w and 0 <= ey < h):
                            in_bounds = False
                            break
                    if not in_bounds:
                        continue

                    # Check non-zero height at start
                    start_vals = []
                    for t in np.linspace(-half_w + 0.5, half_w - 0.5, width):
                        offset_x = perp[0] * t
                        offset_y = perp[1] * t
                        sx = int(round(x0 + offset_x))
                        sy = int(round(y0 + offset_y))
                        start_vals.append(grid[sy, sx])

                    if np.any(start_vals):
                        valid_sweeps.append({
                            'start': (x0, y0),
                            'end': (x1, y1),
                            'direction': (dx, dy),
                            'length': dist,
                            'namespace': f"length_{int(round(dist))}"
                        })

                    total_checked += 1

    return valid_sweeps, total_checked
