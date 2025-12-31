#!/usr/bin/env python3
"""
简单的轨迹叠加显示脚本
将rtk_converted.csv和vehicle_localization.csv的轨迹进行对比和叠加显示
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.interpolate import interp1d

def main():
    # 文件路径
    base_dir = '/Users/alex/workspace/code_review/vehicle_localization'
    rtk_file = os.path.join(base_dir, 'rtk_converted.csv')
    loc_file = os.path.join(base_dir, 'vehicle_localization.csv')
    output_dir = os.path.join(base_dir, 'scripts/trajectory_comparison')

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 70)
    print("轨迹叠加对比系统")
    print("=" * 70)
    print(f"\n📂 加载数据...")

    # 加载数据
    rtk_df = pd.read_csv(rtk_file, sep=';')
    loc_df = pd.read_csv(loc_file, sep=';')

    print(f"✅ RTK转换数据: {len(rtk_df)} 条")
    print(f"✅ 定位数据: {len(loc_df)} 条")

    # 时间同步
    rtk_time_base = rtk_df['timestamp'].iloc[0]
    rtk_df['time_sync'] = rtk_df['timestamp'] - rtk_time_base

    loc_time_base = loc_df['timestamp'].iloc[0]
    loc_df['time_sync'] = (loc_df['timestamp'] - loc_time_base) / 1000.0

    print(f"\n⏱️  时间同步:")
    print(f"   RTK: {rtk_df['time_sync'].min():.2f}s ~ {rtk_df['time_sync'].max():.2f}s")
    print(f"   定位: {loc_df['time_sync'].min():.2f}s ~ {loc_df['time_sync'].max():.2f}s")

    # 计算误差
    print(f"\n📐 计算定位误差...")
    rtk_x = rtk_df['x'].values
    rtk_y = rtk_df['y'].values
    rtk_time = rtk_df['time_sync'].values

    loc_x = loc_df['x'].values
    loc_y = loc_df['y'].values
    loc_time = loc_df['time_sync'].values

    # 插值对齐
    f_loc_x = interp1d(loc_time, loc_x, kind='linear',
                       bounds_error=False, fill_value='extrapolate')
    f_loc_y = interp1d(loc_time, loc_y, kind='linear',
                       bounds_error=False, fill_value='extrapolate')

    loc_x_interp = f_loc_x(rtk_time)
    loc_y_interp = f_loc_y(rtk_time)

    dx = loc_x_interp - rtk_x
    dy = loc_y_interp - rtk_y
    error_distance = np.sqrt(dx**2 + dy**2)

    print(f"   误差统计:")
    print(f"   • 均值误差: {error_distance.mean():.6f} m")
    print(f"   • 最大误差: {error_distance.max():.6f} m")
    print(f"   • 最小误差: {error_distance.min():.6f} m")
    print(f"   • 标准差: {error_distance.std():.6f} m")
    print(f"   • RMS误差: {np.sqrt(np.mean(error_distance**2)):.6f} m")

    # 绘制轨迹叠加图
    print(f"\n📊 生成轨迹叠加图...")
    fig, ax = plt.subplots(figsize=(14, 10))

    # RTK轨迹（蓝色）
    ax.plot(rtk_df['x'], rtk_df['y'], 'b-', linewidth=2.5,
            label='RTK轨迹 (车体坐标系)', alpha=0.8, zorder=2)
    ax.plot(rtk_df['x'].iloc[0], rtk_df['y'].iloc[0], 'bo', markersize=12,
            label='RTK起点', zorder=5)
    ax.plot(rtk_df['x'].iloc[-1], rtk_df['y'].iloc[-1], 'bs', markersize=12,
            label='RTK终点', zorder=5)

    # 定位轨迹（红色虚线）
    ax.plot(loc_df['x'], loc_df['y'], 'r--', linewidth=2,
            label='EKF定位轨迹', alpha=0.7, zorder=2)
    ax.plot(loc_df['x'].iloc[0], loc_df['y'].iloc[0], 'ro', markersize=12,
            label='定位起点', zorder=5)
    ax.plot(loc_df['x'].iloc[-1], loc_df['y'].iloc[-1], 'rs', markersize=12,
            label='定位终点', zorder=5)

    # 误差向量（橙色）
    step = max(1, len(error_distance) // 30)
    for i in range(0, len(error_distance), step):
        if error_distance[i] > 0.05:
            ax.arrow(rtk_x[i], rtk_y[i], dx[i]*0.8, dy[i]*0.8,
                    head_width=0.3, head_length=0.2, fc='orange', ec='orange',
                    alpha=0.5, zorder=3)

    ax.set_xlabel('X坐标 (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y坐标 (m)', fontsize=12, fontweight='bold')
    ax.set_title('RTK参考轨迹 vs EKF定位轨迹叠加对比\n(ISO 8855车体坐标系)',
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.legend(loc='best', fontsize=11, framealpha=0.95)
    ax.set_aspect('equal')

    # 添加误差统计信息框
    error_text = f"""误差统计:
━━━━━━━━━━━━━━━━━━━━━
均值: {error_distance.mean():.6f} m
最大: {error_distance.max():.6f} m
最小: {error_distance.min():.6f} m
标准差: {error_distance.std():.6f} m
RMS: {np.sqrt(np.mean(error_distance**2)):.6f} m
━━━━━━━━━━━━━━━━━━━━━"""

    ax.text(0.02, 0.98, error_text, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, pad=0.8))

    plt.tight_layout()
    filepath = os.path.join(output_dir, '01_trajectory_overlay.png')
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    print(f"✅ 轨迹叠加图: {filepath}")
    plt.close()

    # 绘制误差分布热力图
    print(f"📊 生成误差热力图...")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # 误差距离热力图
    scatter1 = ax1.scatter(rtk_df['x'], rtk_df['y'],
                           c=error_distance, cmap='RdYlGn_r',
                           s=50, alpha=0.6, edgecolors='black', linewidth=0.5)
    ax1.plot(rtk_df['x'], rtk_df['y'], 'b-', linewidth=1, alpha=0.3, zorder=1)
    ax1.set_xlabel('X坐标 (m)', fontsize=11)
    ax1.set_ylabel('Y坐标 (m)', fontsize=11)
    ax1.set_title('定位误差分布（距离热力图）', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    cbar1 = plt.colorbar(scatter1, ax=ax1)
    cbar1.set_label('误差距离 (m)', fontsize=10)

    # 误差向量场
    step = max(1, len(error_distance) // 40)
    scatter2 = ax2.quiver(rtk_df['x'][::step], rtk_df['y'][::step],
                          dx[::step], dy[::step],
                          error_distance[::step], cmap='RdYlGn_r',
                          scale=20, scale_units='inches', alpha=0.7, width=0.003)
    ax2.plot(rtk_df['x'], rtk_df['y'], 'b-', linewidth=1, alpha=0.3, zorder=1)
    ax2.set_xlabel('X坐标 (m)', fontsize=11)
    ax2.set_ylabel('Y坐标 (m)', fontsize=11)
    ax2.set_title('定位误差分布（误差向量场）', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    cbar2 = plt.colorbar(scatter2, ax=ax2)
    cbar2.set_label('误差距离 (m)', fontsize=10)

    plt.tight_layout()
    filepath = os.path.join(output_dir, '02_error_heatmap.png')
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    print(f"✅ 误差热力图: {filepath}")
    plt.close()

    # 绘制时间序列对比
    print(f"📊 生成时间序列图...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    # X坐标对比
    axes[0].plot(rtk_df['time_sync'], rtk_df['x'], 'b-', linewidth=2, label='RTK X坐标')
    axes[0].plot(loc_df['time_sync'], loc_df['x'], 'r--', linewidth=2, label='定位 X坐标')
    axes[0].set_ylabel('X坐标 (m)', fontsize=11)
    axes[0].set_title('X坐标对比', fontsize=12, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=10)

    # Y坐标对比
    axes[1].plot(rtk_df['time_sync'], rtk_df['y'], 'b-', linewidth=2, label='RTK Y坐标')
    axes[1].plot(loc_df['time_sync'], loc_df['y'], 'r--', linewidth=2, label='定位 Y坐标')
    axes[1].set_ylabel('Y坐标 (m)', fontsize=11)
    axes[1].set_title('Y坐标对比', fontsize=12, fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=10)

    # 误差距离时间序列
    axes[2].plot(rtk_time, error_distance, 'g-', linewidth=2, label='定位误差')
    axes[2].fill_between(rtk_time, error_distance, 0, alpha=0.3, color='green')
    axes[2].axhline(y=error_distance.mean(), color='r', linestyle='--',
                    alpha=0.7, label=f'平均误差: {error_distance.mean():.6f}m')
    axes[2].set_ylabel('误差距离 (m)', fontsize=11)
    axes[2].set_xlabel('时间 (s)', fontsize=11)
    axes[2].set_title('定位误差随时间变化', fontsize=12, fontweight='bold')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(fontsize=10)

    plt.tight_layout()
    filepath = os.path.join(output_dir, '03_timeseries.png')
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    print(f"✅ 时间序列图: {filepath}")
    plt.close()

    # 保存误差数据到CSV
    print(f"📊 保存误差数据...")
    error_df = pd.DataFrame({
        'time': rtk_time,
        'rtk_x': rtk_x,
        'rtk_y': rtk_y,
        'loc_x': loc_x_interp,
        'loc_y': loc_y_interp,
        'error_x': dx,
        'error_y': dy,
        'error_distance': error_distance
    })
    csv_file = os.path.join(output_dir, 'error_analysis.csv')
    error_df.to_csv(csv_file, index=False)
    print(f"✅ 误差数据: {csv_file}")

    print(f"\n✨ 对比完成！")
    print(f"📁 输出目录: {output_dir}/")
    print(f"\n生成的文件:")
    print(f"  • 01_trajectory_overlay.png - 轨迹叠加图")
    print(f"  • 02_error_heatmap.png - 误差热力图")
    print(f"  • 03_timeseries.png - 时间序列对比")
    print(f"  • error_analysis.csv - 误差数据表")

if __name__ == '__main__':
    main()

