#!/usr/bin/env python3
import pandas as pd
import numpy as np
import sys

if len(sys.argv) < 3:
    print("Usage: compute_diagnostics.py rtk_converted.csv vehicle_localization.csv")
    sys.exit(1)

rtk_file = sys.argv[1]
loc_file = sys.argv[2]

rtk = pd.read_csv(rtk_file, sep=';')
loc = pd.read_csv(loc_file, sep=';')

# time sync
rtk_time_base = rtk['timestamp'].iloc[0]
rtk['t'] = rtk['timestamp'] - rtk_time_base
loc_time_base = loc['timestamp'].iloc[0]
loc['t'] = (loc['timestamp'] - loc_time_base) / 1000.0

# interp loc to rtk times
rtk_t = rtk['t'].values
if len(loc['t'].values) > 1:
    loc_x = np.interp(rtk_t, loc['t'].values, loc['x'].values)
    loc_y = np.interp(rtk_t, loc['t'].values, loc['y'].values)
else:
    loc_x = loc['x'].values[0] * np.ones_like(rtk_t)
    loc_y = loc['y'].values[0] * np.ones_like(rtk_t)

rtk_x = rtk['x'].values
rtk_y = rtk['y'].values

dx = loc_x - rtk_x
dy = loc_y - rtk_y
err = np.sqrt(dx*dx + dy*dy)

mean_pos_err = np.mean(err)
mean_dx = np.mean(dx)
mean_dy = np.mean(dy)

# regression loc_x ~ a * rtk_x + b
A = np.vstack([rtk_x, np.ones_like(rtk_x)]).T
# use numpy.linalg.lstsq compatible call
ab, *_ = np.linalg.lstsq(A, loc_x, rcond=None)
a, b = ab[0], ab[1]

# cumulative dist along RTK
rtk_points = np.vstack([rtk_x, rtk_y]).T
dists = np.linalg.norm(np.diff(rtk_points, axis=0), axis=1)
cumdist = np.concatenate(([0.0], np.cumsum(dists)))
if len(cumdist) == len(dx):
    slope_dx_vs_dist = np.polyfit(cumdist, dx, 1)[0]
else:
    # interpolate dx onto cumdist length if mismatch
    slope_dx_vs_dist = np.nan

print(f"pairs: {len(rtk_x)}")
print(f"mean_pos_err={mean_pos_err:.6f} m")
print(f"mean_dx={mean_dx:.6f} m, mean_dy={mean_dy:.6f} m")
print(f"loc_x ~ {a:.6f} * rtk_x + {b:.6f}")
print(f"dx vs dist slope = {slope_dx_vs_dist}")

# save minimal diagnostics csv
out = pd.DataFrame({
    'time': rtk_t,
    'rtk_x': rtk_x,
    'rtk_y': rtk_y,
    'loc_x': loc_x,
    'loc_y': loc_y,
    'dx': dx,
    'dy': dy,
    'err': err,
})
out.to_csv('diagnostic_pairs.csv', index=False)
print('wrote diagnostic_pairs.csv')
