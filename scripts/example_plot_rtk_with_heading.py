#!/usr/bin/env python3
"""
示例：使用 plot_rtk.py 显示轨迹和航向

演示如何使用 RTKTrajectoryPlotter 的新功能：
1. plot_trajectory() - 原始的详细分析图表
2. plot_trajectory_with_heading_markers() - 新增方法，显示航向箭头（每20cm一次）
"""

import sys
import os

# 添加脚本路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'scripts'))

from plot_rtk import RTKTrajectoryPlotter

def main():
    # 创建绘图器实例
    plotter = RTKTrajectoryPlotter()
    
    # 方法1：从 CSV 文件加载 RTK 数据
    if os.path.exists('rtk_data.csv'):
        print("=" * 70)
        print("示例 1: 从 CSV 文件加载并绘制轨迹（航向每20cm显示一次）")
        print("=" * 70)
        
        # 加载数据
        plotter.load_from_csv('rtk_data.csv', is_latlon=True)
        plotter.load_from_converted_csv('rtk_converted.csv', is_latlon=True)
        
        # 检测稳定点
        stable_idx, stable_info = plotter.detect_stable_first_point()
        
        # 转换为相对坐标
        plotter.convert_to_relative_coords()
        
        # 打印统计信息
        plotter.print_statistics()
        
        # 使用新方法：显示航向箭头的轨迹图（每20cm显示一次）
        print("\n【绘制轨迹，航向每20cm显示一次】")
        output_file = 'rtk_trajectory_with_heading.png'
        try:
            plotter.plot_trajectory_with_heading_markers(
                save_path=output_file,
                distance_interval=0.2  # 20cm 间隔
            )
            print(f"✓ 图表已保存到: {output_file}")
        except Exception as e:
            print(f"✗ 绘图失败: {e}")
        
        # 还可以尝试其他间隔
        print("\n【尝试其他间隔】")
        
        # 50cm 间隔
        output_file_50 = 'rtk_trajectory_heading_50cm.png'
        try:
            plotter.plot_trajectory_with_heading_markers(
                save_path=output_file_50,
                distance_interval=0.5  # 50cm 间隔
            )
            print(f"✓ 50cm间隔的图表已保存到: {output_file_50}")
        except Exception as e:
            print(f"✗ 绘图失败: {e}")
        
        # 也可以使用原始的详细分析图表
        print("\n【绘制详细分析图表】")
        output_file_detailed = 'rtk_trajectory_detailed.png'
        try:
            plotter.plot_trajectory(save_path=output_file_detailed)
            print(f"✓ 详细分析图表已保存到: {output_file_detailed}")
        except Exception as e:
            print(f"✗ 绘图失败: {e}")
        
    else:
        print("✗ 找不到 rtk_data.csv 文件")
        print("\n【使用示例代码】")
        print("""
# 如果有 CSV 文件：
plotter = RTKTrajectoryPlotter()
plotter.load_from_csv('rtk_data.csv')
plotter.detect_stable_first_point()
plotter.convert_to_relative_coords()

# 方法 A: 简单的航向显示图表（新增功能）
plotter.plot_trajectory_with_heading_markers(
    distance_interval=0.2  # 每20cm显示一次航向
)

# 方法 B: 详细的分析图表（原有功能）
plotter.plot_trajectory()

# 也可以自定义间隔距离
plotter.plot_trajectory_with_heading_markers(
    distance_interval=0.5,  # 每50cm显示一次
    save_path='output.png'
)
        """)

if __name__ == '__main__':
    main()
