from asammdf import MDF
from bisect import bisect_left
import statistics

MF4_PATH = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_ParallelParkingInLeft_5kph_001.mf4"
SIGNALS = ["PosLat2","PosLon2","AngleHeading","GpsWeek","GpsTime","system_state","satellite_status","GpsAge","PosAlt"]
DELTA = 0.005


def find_nearest_index(ts_list, target):
    if not ts_list:
        return None, None
    i = bisect_left(ts_list, target)
    candidates = []
    if i < len(ts_list):
        candidates.append(i)
    if i > 0:
        candidates.append(i-1)
    best_idx = None
    best_dt = None
    for ci in candidates:
        dt = abs(ts_list[ci] - target)
        if best_dt is None or dt < best_dt:
            best_dt = dt
            best_idx = ci
    return best_idx, best_dt


def main():
    mdf = MDF(MF4_PATH)
    sigs = mdf.select(SIGNALS)
    # gather timestamps and samples lists
    ts_lists = [s.timestamps.tolist() for s in sigs]
    samples = [s.samples.tolist() for s in sigs]

    print("Signal counts and time ranges:")
    for name, ts in zip(SIGNALS, ts_lists):
        if ts:
            print(f"  {name}: count={len(ts)}, first={ts[0]:.6f}, last={ts[-1]:.6f}")
        else:
            print(f"  {name}: count=0")

    heading_ts = ts_lists[2]
    n_heading = len(heading_ts)
    print(f"\nHeading samples: {n_heading}, delta_time={DELTA}s")

    # For each required signal compute dt distribution relative to heading
    required_idx = [0,1,3,4,5,6,7]  # indices of required signals
    stats = {i: [] for i in required_idx}
    exceed_counts = {i:0 for i in required_idx}
    dropped_by_reason = 0

    for j, h_ts in enumerate(heading_ts):
        ok = True
        for i in required_idx:
            idx, dt = find_nearest_index(ts_lists[i], h_ts)
            if idx is None:
                ok = False
                # record a big dt
                stats[i].append(float('inf'))
                exceed_counts[i] += 1
            else:
                stats[i].append(dt)
                if dt > DELTA:
                    exceed_counts[i] += 1
                    ok = False
        if not ok:
            dropped_by_reason += 1

    print("\nPer-signal time-difference stats relative to Heading (seconds):")
    for i in required_idx:
        vals = [v for v in stats[i] if v != float('inf')]
        infs = sum(1 for v in stats[i] if v == float('inf'))
        total = len(stats[i])
        if vals:
            print(f"  {SIGNALS[i]}: mean={statistics.mean(vals):.6f}, median={statistics.median(vals):.6f}, max={max(vals):.6f}, inf_missing={infs}, exceed={exceed_counts[i]}")
        else:
            print(f"  {SIGNALS[i]}: all missing (count={total})")

    print(f"\nHeading groups: {n_heading}, potential dropped (any required missing or dt>delta): {dropped_by_reason}")

    # show histogram-like summary of exceed counts
    print("\nSignals with high exceed counts ( >0 ):")
    for i in required_idx:
        if exceed_counts[i] > 0:
            print(f"  {SIGNALS[i]}: exceed_count={exceed_counts[i]} / {n_heading}")


if __name__ == '__main__':
    main()
