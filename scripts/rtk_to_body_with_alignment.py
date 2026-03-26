#!/usr/bin/env python3
"""
RTK 坐标转换示例：与 localization 系统时间对齐

关键处理步骤：
1. 从 localization 输出读取时间戳和速度数据（秒为单位）
2. 为每个 RTK 点找到最近的 localization 速度（时间窗口 100ms）
3. 使用对齐的速度进行 RTK 坐标转换
4. 倒车时自动反转 X 坐标以匹配 localization 的有符号速度语义
"""

import pandas as pd
import numpy as np
from bisect import bisect_left
from utils.rtk import RTKUtils


def find_nearest_velocity(rtk_timestamp, loc_timestamps, loc_velocities, max_time_diff=0.1):
    """
    根据 RTK 时间戳，找到最近的 localization velocity
    
    Args:
        rtk_timestamp: RTK 时间戳 (s)
        loc_timestamps: localization 时间戳数组 (s)
        loc_velocities: localization 速度数组 (m/s)
        max_time_diff: 最大时间差 (s)，默认 0.1s = 100ms
    
    Returns:
        最近的 velocity (m/s) 或 None
    """
    # 使用二分查找找到最近的时间戳
    idx = bisect_left(loc_timestamps, rtk_timestamp)
    
    # 检查前后两个点
    candidates = []
    if idx > 0:
        candidates.append((idx - 1, abs(loc_timestamps[idx - 1] - rtk_timestamp)))
    if idx < len(loc_timestamps):
        candidates.append((idx, abs(loc_timestamps[idx] - rtk_timestamp)))
    
    if not candidates:
        return None
    
    # 选择最近的
    best_idx, min_diff = min(candidates, key=lambda x: x[1])
    
    # 检查时间差是否在阈值内
    if min_diff > max_time_diff:
        return None
    
    return loc_velocities[best_idx]


def convert_rtk_with_aligned_velocity(rtk_list, loc_data, origin_lat, origin_lon, 
                                      origin_alt, origin_heading, max_time_diff=0.1):
    """
    批量转换 RTK，使用对齐的 localization 速度
    
    Args:
        rtk_list: RTK 点列表
        loc_data: localization DataFrame (需包含 'timestamp' 和 'velocity' 列)
        origin_lat, origin_lon, origin_alt: 原点坐标
        origin_heading: 原点航向 (度)
        max_time_diff: 最大时间差阈值 (s)，默认 0.1s
    
    Returns:
        DataFrame 包含转换后的坐标和对齐的速度
    """
    loc_timestamps = loc_data['timestamp'].values  # 单位：s
    loc_velocities = loc_data['velocity'].values    # 单位：m/s
    
    results = []
    unmatched_count = 0
    
    for idx, rtk_point in enumerate(rtk_list):
        # 找最近的速度
        velocity = find_nearest_velocity(
            rtk_point.timestamp,
            loc_timestamps,
            loc_velocities,
            max_time_diff=max_time_diff
        )
        
        if velocity is None:
            unmatched_count += 1
            continue  # 跳过无法匹配的点
        
        # 转换坐标（传入速度，倒车时自动反转 X 坐标）
        x, y, z, heading = rtk_point.to_body(
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            origin_alt=origin_alt,
            origin_heading=origin_heading,
            antenna_offset=None,
            body_z_up=True,
            velocity=velocity  # m/s，正值=前进，负值=倒车
        )
        
        results.append({
            'timestamp': rtk_point.timestamp,
            'aligned_velocity': velocity,
            'is_reverse': velocity < 0,
            'body_x': x,
            'body_y': y,
            'body_z': z,
            'body_heading': heading,
            'rtk_heading_deg': rtk_point.heading
        })
    
    total = len(rtk_list)
    matched = len(results)
    print(f"[INFO] 成功匹配 {matched}/{total} 个 RTK 点")
    if unmatched_count > 0:
        print(f"[WARN] 未匹配 {unmatched_count} 个点（时间差超过 {max_time_diff}s）")
    
    return pd.DataFrame(results)


def main():
    """主程序：完整的 RTK 转换流程"""
    
    print("=" * 70)
    print("RTK 倒车处理：Y 轴对齐验证")
    print("=" * 70)
    
    # 1. 读取 RTK body 坐标数据
    print("\n[Step 1] 读取 RTK body 坐标...")
    try:
        rtk_body = pd.read_csv('data/Lev01_R19_Stragight__backward_0_5kph_001/rtk_trajectory.csv')
        print(f"  ✓ 读取 {len(rtk_body)} 个 RTK body 坐标点")
        print(f"  ✓ RTK Y 轴范围: {rtk_body['y'].min():.3f} ~ {rtk_body['y'].max():.3f} m")
        print(f"  ✓ RTK 航向范围: {rtk_body['heading_deg'].min():.1f}° ~ {rtk_body['heading_deg'].max():.1f}°")
    except Exception as e:
        print(f"  ✗ 读取 RTK 失败: {e}")
        return
    
    # 2. 读取 localization 输出
    print("\n[Step 2] 读取 Localization 输出...")
    try:
        loc_data = pd.read_csv('data/Lev01_R19_Stragight__backward_0_5kph_001/vehicle_localization.csv')
        print(f"  ✓ 读取 {len(loc_data)} 个 Localization 点")
        print(f"  ✓ 时间范围: {loc_data['timestamp'].min():.2f}s ~ {loc_data['timestamp'].max():.2f}s")
        print(f"  ✓ 速度范围: {loc_data['velocity'].min():.3f} ~ {loc_data['velocity'].max():.3f} m/s")
        
        # 检查倒车时间
        reverse_mask = loc_data['velocity'] < -0.01
        if reverse_mask.any():
            reverse_time = loc_data[reverse_mask]
            print(f"  ℹ 倒车时间: {len(reverse_time)} 帧 ({len(reverse_time)/len(loc_data)*100:.1f}%)")
            print(f"    倒车速度范围: {reverse_time['velocity'].min():.3f} ~ {reverse_time['velocity'].max():.3f} m/s")
    except Exception as e:
        print(f"  ✗ 读取 Localization 失败: {e}")
        return
    
    # 3. 时间对齐合并数据
    print("\n[Step 3] 时间对齐合并数据...")
    
    # 添加 timestamp 列（如果没有的话）
    if 'timestamp' not in rtk_body.columns:
        print("  ⚠ RTK 数据缺少 timestamp 列，使用索引作为时间轴")
        rtk_body['timestamp'] = range(len(rtk_body))
    
    # 按时间戳对齐
    rtk_body = rtk_body.sort_values('timestamp').reset_index(drop=True)
    loc_data = loc_data.sort_values('timestamp').reset_index(drop=True)
    
    print(f"  ✓ RTK 时间范围: {rtk_body['timestamp'].min():.2f}s ~ {rtk_body['timestamp'].max():.2f}s")
    print(f"  ✓ Loc 时间范围: {loc_data['timestamp'].min():.2f}s ~ {loc_data['timestamp'].max():.2f}s")
    
    # 4. 统计分析
    print("\n[Step 4] 统计分析...")
    
    # Localization 中的倒车段
    reverse_mask = loc_data['velocity'] < -0.01
    forward_mask = loc_data['velocity'] >= -0.01
    
    print(f"  ✓ 前进段: {forward_mask.sum()} 帧")
    print(f"  ✓ 倒车段: {reverse_mask.sum()} 帧")
    
    # 前进段统计
    if forward_mask.sum() > 0:
        forward_loc = loc_data[forward_mask]
        print(f"\n  前进段 (Localization):")
        print(f"    Y 轴范围: {forward_loc['y'].min():.3f} ~ {forward_loc['y'].max():.3f} m")
        print(f"    Y 轴变化: {forward_loc['y'].max() - forward_loc['y'].min():.3f} m")
        print(f"    速度范围: {forward_loc['velocity'].min():.3f} ~ {forward_loc['velocity'].max():.3f} m/s")
    
    # 倒车段统计
    if reverse_mask.sum() > 0:
        reverse_loc = loc_data[reverse_mask]
        print(f"\n  倒车段 (Localization):")
        print(f"    Y 轴范围: {reverse_loc['y'].min():.3f} ~ {reverse_loc['y'].max():.3f} m")
        print(f"    Y 轴变化: {reverse_loc['y'].max() - reverse_loc['y'].min():.3f} m")
        print(f"    速度范围: {reverse_loc['velocity'].min():.3f} ~ {reverse_loc['velocity'].max():.3f} m/s")
    
    # RTK 统计
    print(f"\n  RTK body 坐标:")
    print(f"    Y 轴范围: {rtk_body['y'].min():.3f} ~ {rtk_body['y'].max():.3f} m")
    print(f"    Y 轴变化: {rtk_body['y'].max() - rtk_body['y'].min():.3f} m")
    
    # 5. 保存结果
    print("\n[Step 5] 保存结果...")
    
    # 合并 RTK 和 Localization 数据用于对比
    # 使用插值方法时间对齐
    merged_data = pd.merge_asof(
        loc_data.sort_values('timestamp'),
        rtk_body[['timestamp', 'x', 'y', 'heading_deg']].sort_values('timestamp'),
        on='timestamp',
        direction='nearest',
        suffixes=('_loc', '_rtk')
    )
    
    output_file = 'rtk_vs_localization_aligned.csv'
    merged_data.to_csv(output_file, index=False)
    print(f"  ✓ 结果已保存: {output_file}")
    
    # 6. 验证数据对齐 - Y 轴相关性分析
    print("\n[Step 6] Y 轴对齐验证...")
    
    # 过滤有效数据（两边都不是 NaN）
    valid_mask = ~(merged_data['y_loc'].isna() | merged_data['y_rtk'].isna())
    
    if valid_mask.sum() > 0:
        # 总体相关性
        correlation = merged_data[valid_mask]['y_loc'].corr(merged_data[valid_mask]['y_rtk'])
        print(f"  ✓ Y 轴总体相关系数: {correlation:.4f} (应接近 1.0)")
        
        # 前进时 Y 轴相关性
        forward_mask = (merged_data['velocity'] > 0.1) & valid_mask
        if forward_mask.sum() > 10:
            forward_corr = merged_data[forward_mask]['y_loc'].corr(merged_data[forward_mask]['y_rtk'])
            print(f"  ✓ 前进时 Y 轴相关系数: {forward_corr:.4f} (共 {forward_mask.sum()} 帧)")
        
        # 倒车时 Y 轴相关性
        reverse_mask = (merged_data['velocity'] < -0.1) & valid_mask
        if reverse_mask.sum() > 10:
            reverse_corr = merged_data[reverse_mask]['y_loc'].corr(merged_data[reverse_mask]['y_rtk'])
            print(f"  ✓ 倒车时 Y 轴相关系数: {reverse_corr:.4f} (共 {reverse_mask.sum()} 帧)")
        
        # Y 轴差值统计
        y_diff = merged_data[valid_mask]['y_loc'] - merged_data[valid_mask]['y_rtk']
        print(f"\n  Y 轴差值统计:")
        print(f"    平均差值: {y_diff.mean():.4f} m")
        print(f"    标准差: {y_diff.std():.4f} m")
        print(f"    MAE: {y_diff.abs().mean():.4f} m")
    
    print("\n" + "=" * 70)
    print("✓ 处理完成！")
    print("=" * 70)



if __name__ == "__main__":
    main()
