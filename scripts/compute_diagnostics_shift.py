#!/usr/bin/env python3
import pandas as pd
import numpy as np
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('rtk_file')
parser.add_argument('loc_file')
parser.add_argument('--shift', type=float, default=0.0)
parser.add_argument('--out', default='diagnostic_pairs_shifted.csv')
args = parser.parse_args()

rtk = pd.read_csv(args.rtk_file, sep=';')
loc = pd.read_csv(args.loc_file, sep=';')

# time sync
rtk_time_base = rtk['timestamp'].iloc[0]
rtk['t'] = rtk['timestamp'] - rtk_time_base
loc_time_base = loc['timestamp'].iloc[0]
loc['t'] = (loc['timestamp'] - loc_time_base) / 1000.0

# apply shift (in seconds)
loc['t'] = loc['t'] + args.shift

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
ab, *_ = np.linalg.lstsq(A, loc_x, rcond=None)
a, b = ab[0], ab[1]

# cumulative dist along RTK
rtk_points = np.vstack([rtk_x, rtk_y]).T
dists = np.linalg.norm(np.diff(rtk_points, axis=0), axis=1)
cumdist = np.concatenate(([0.0], np.cumsum(dists)))
if len(cumdist) == len(dx):
    slope_dx_vs_dist = np.polyfit(cumdist, dx, 1)[0]
else:
    slope_dx_vs_dist = np.nan

print(f"pairs: {len(rtk_x)}")
print(f"mean_pos_err={mean_pos_err:.6f} m")
print(f"mean_dx={mean_dx:.6f} m, mean_dy={mean_dy:.6f} m")
print(f"loc_x ~ {a:.6f} * rtk_x + {b:.6f}")
print(f"dx vs dist slope = {slope_dx_vs_dist}")

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
out.to_csv(args.out, index=False)
print('wrote', args.out)
