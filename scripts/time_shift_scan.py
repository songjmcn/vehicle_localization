#!/usr/bin/env python3
import pandas as pd
import numpy as np
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('rtk_file')
parser.add_argument('loc_file')
parser.add_argument('--min', type=float, default=-1.0)
parser.add_argument('--max', type=float, default=1.0)
parser.add_argument('--step', type=float, default=0.1)
parser.add_argument('--out', default='time_shift_scan.csv')
args = parser.parse_args()

rtk = pd.read_csv(args.rtk_file, sep=';')
loc = pd.read_csv(args.loc_file, sep=';')

# time bases
rtk_time_base = rtk['timestamp'].iloc[0]
rtk_t = rtk['timestamp'] - rtk_time_base
# loc in seconds
loc_time_base = loc['timestamp'].iloc[0]
loc_t = (loc['timestamp'] - loc_time_base) / 1000.0

rtk_x = rtk['x'].values
rtk_y = rtk['y'].values

results = []
shifts = np.arange(args.min, args.max + args.step/2.0, args.step)
for s in shifts:
    loc_t_shifted = loc_t + s
    # interp loc to rtk times (rtk_t in original units: if rtk timestamps are seconds or ms?)
    # rtk_t currently in raw units (timestamp - base). If rtk timestamps are seconds, keep as-is.
    # Normalize rtk_t to seconds if large (>1e9)
    if np.mean(rtk_t) > 1e6:
        rtk_t_s = (rtk['timestamp'] - rtk_time_base)  # likely seconds
    else:
        rtk_t_s = rtk_t
    try:
        if len(loc_t_shifted) > 1:
            loc_x_interp = np.interp(rtk_t_s, loc_t_shifted, loc['x'].values)
            loc_y_interp = np.interp(rtk_t_s, loc_t_shifted, loc['y'].values)
        else:
            loc_x_interp = loc['x'].values[0] * np.ones_like(rtk_x)
            loc_y_interp = loc['y'].values[0] * np.ones_like(rtk_y)
    except Exception as e:
        print('interp failed for shift', s, e)
        continue
    dx = loc_x_interp - rtk_x
    dy = loc_y_interp - rtk_y
    err = np.sqrt(dx*dx + dy*dy)
    mean_err = np.mean(err)
    mean_dx = np.mean(dx)
    mean_dy = np.mean(dy)
    results.append((s, mean_err, mean_dx, mean_dy))

df = pd.DataFrame(results, columns=['shift_s', 'mean_pos_err', 'mean_dx', 'mean_dy'])
df.to_csv(args.out, index=False)
best = df.loc[df['mean_pos_err'].idxmin()]
print('wrote', args.out)
print('best shift:', best['shift_s'], 'mean_pos_err=', best['mean_pos_err'])
