#!/usr/bin/env python3
"""
Y方向误差分析脚本 - 修正版
分析vehicle_localization.csv和rtk_converted.csv中y方向误差的原因
"""

import csv
import statistics

# 加载定位数据
print("[步骤1] 加载定位数据...")
localization = {}
with open('vehicle_localization.csv', 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    for row in reader:
        # vehicle_localization.csv的时间戳是毫秒级整数
        ts_ms = int(row['timestamp'])
        ts_s = ts_ms / 1000.0  # 转换为秒
        localization[ts_s] = {
            'x': float(row['x']),
            'y': float(row['y']),
            'heading': float(row['heading']),
            'steering': float(row['steering angle']),
            'gear': row['gear']
        }

print(f"   加载了 {len(localization)} 条定位数据")
print(f"   时间范围: {min(localization.keys()):.2f} - {max(localization.keys()):.2f}")

# 加载RTK数据
print("[步骤2] 加载RTK数据...")
rtk = {}
with open('rtk_converted.csv', 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    for row in reader:
        # rtk_converted.csv的时间戳已经是秒级浮点数
        ts_s = float(row['timestamp'])
        rtk[ts_s] = {
            'x': float(row['x']),
            'y': float(row['y']),
            'heading': float(row['heading'])
        }

print(f"   加载了 {len(rtk)} 条RTK数据")
print(f"   时间范围: {min(rtk.keys()):.2f} - {max(rtk.keys()):.2f}")

# 时间戳对齐 - 找到最接近的时间戳（允许误差0.1秒）
print("\n[步骤3] 时间戳对齐（允许误差0.1秒）...")
matched_pairs = []
tolerance = 0.1

for rtk_ts in sorted(rtk.keys()):
    # 找最接近的定位时间戳
    closest_ts = None
    min_diff = float('inf')
    
    for loc_ts in localization.keys():
        diff = abs(loc_ts - rtk_ts)
        if diff < tolerance and diff < min_diff:
            closest_ts = loc_ts
            min_diff = diff
    
    if closest_ts is not None:
        matched_pairs.append((closest_ts, rtk_ts))

print(f"   匹配对数: {len(matched_pairs)}")

# 计算误差
print("\n[步骤4] 计算Y方向误差...")
errors = []
for loc_ts, rtk_ts in matched_pairs:
    error_y = rtk[rtk_ts]['y'] - localization[loc_ts]['y']
    errors.append({
        'loc_ts': loc_ts,
        'rtk_ts': rtk_ts,
        'y_loc': localization[loc_ts]['y'],
        'y_rtk': rtk[rtk_ts]['y'],
        'error_y': error_y,
        'x_loc': localization[loc_ts]['x'],
        'heading': localization[loc_ts]['heading'],
        'steering': localization[loc_ts]['steering'],
        'gear': localization[loc_ts]['gear']
    })

if len(errors) == 0:
    print("ERROR: 没有匹配的数据点！")
    exit(1)

# 统计分析
errors_list = [e['error_y'] for e in errors]
errors_abs = [abs(e['error_y']) for e in errors]

print("\n" + "="*80)
print("Y方向误差统计分析")
print("="*80)

print(f"\n数据点总数: {len(errors)}")
print(f"均值 (mean):        {statistics.mean(errors_list):12.6f} m")
print(f"标准差 (stdev):     {statistics.stdev(errors_list):12.6f} m")
print(f"最小值 (min):       {min(errors_list):12.6f} m")
print(f"最大值 (max):       {max(errors_list):12.6f} m")
print(f"中位数 (median):    {statistics.median(errors_list):12.6f} m")
print(f"绝对误差均值:       {statistics.mean(errors_abs):12.6f} m")

# 分析误差分布
pos_errors = [e for e in errors_list if e > 0]
neg_errors = [e for e in errors_list if e < 0]
zero_errors = [e for e in errors_list if abs(e) < 1e-6]

print(f"\n误差分布:")
print(f"  正误差 (y_rtk > y_loc): {len(pos_errors)} 个 ({100*len(pos_errors)/len(errors):.1f}%)")
print(f"  负误差 (y_rtk < y_loc): {len(neg_errors)} 个 ({100*len(neg_errors)/len(errors):.1f}%)")
print(f"  零误差 (差异<1e-6):     {len(zero_errors)} 个 ({100*len(zero_errors)/len(errors):.1f}%)")

# 排序找最大误差
sorted_errors = sorted(errors, key=lambda x: abs(x['error_y']), reverse=True)

print(f"\n最大误差的15个时刻:")
print("-" * 150)
print(f"{'#':<3} {'RTK时刻':<15} {'error_y':<12} {'y_loc':<12} {'y_rtk':<12} {'x_loc':<10} {'heading':<10} {'steering':<10} {'gear':<8}")
print("-" * 150)

for i, e in enumerate(sorted_errors[:15], 1):
    print(f"{i:<3} {e['rtk_ts']:<15.4f} {e['error_y']:<12.6f} {e['y_loc']:<12.6f} {e['y_rtk']:<12.6f} {e['x_loc']:<10.6f} {e['heading']:<10.6f} {e['steering']:<10.6f} {e['gear']:<8}")

# 分析特定时段
print(f"\n\n分析前进 (Drive) 和倒车 (Reverse) 阶段:")
print("-" * 80)

drive_errors = [abs(e['error_y']) for e in errors if e['gear'] == 'Drive']
reverse_errors = [abs(e['error_y']) for e in errors if e['gear'] == 'Reverse']
neutral_errors = [abs(e['error_y']) for e in errors if e['gear'] == 'Neutral']

if drive_errors:
    print(f"前进阶段 (Drive):")
    print(f"  数据点: {len(drive_errors)} 个")
    print(f"  绝对误差均值: {statistics.mean(drive_errors):.6f} m")
    print(f"  绝对误差最大: {max(drive_errors):.6f} m")

if reverse_errors:
    print(f"\n倒车阶段 (Reverse):")
    print(f"  数据点: {len(reverse_errors)} 个")
    print(f"  绝对误差均值: {statistics.mean(reverse_errors):.6f} m")
    print(f"  绝对误差最大: {max(reverse_errors):.6f} m")

if neutral_errors:
    print(f"\n空档阶段 (Neutral):")
    print(f"  数据点: {len(neutral_errors)} 个")
    print(f"  绝对误差均值: {statistics.mean(neutral_errors):.6f} m")
    print(f"  绝对误差最大: {max(neutral_errors):.6f} m")

# 保存详细结果到CSV
print(f"\n\n保存详细结果到 y_error_analysis.csv...")
with open('y_error_analysis.csv', 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=[
        'rtk_timestamp', 'error_y', 'y_loc', 'y_rtk', 'x_loc',
        'heading', 'steering', 'gear', 'abs_error'
    ], delimiter=';')
    writer.writeheader()
    
    for e in sorted_errors:
        writer.writerow({
            'rtk_timestamp': f"{e['rtk_ts']:.6f}",
            'error_y': f"{e['error_y']:.6f}",
            'y_loc': f"{e['y_loc']:.6f}",
            'y_rtk': f"{e['y_rtk']:.6f}",
            'x_loc': f"{e['x_loc']:.6f}",
            'heading': f"{e['heading']:.6f}",
            'steering': f"{e['steering']:.6f}",
            'gear': e['gear'],
            'abs_error': f"{abs(e['error_y']):.6f}"
        })

print("✓ 分析完成！\n")
print("="*80)
