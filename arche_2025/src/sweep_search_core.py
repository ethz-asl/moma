#!/usr/bin/env python

# Core logic file: sweep_search_core.py

import numpy as np

def find_valid_sweeps(grid, min_len=1, max_len=16, width=4):
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
