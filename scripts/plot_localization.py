#!/usr/bin/env python3
"""
车辆定位轨迹可视化脚本
使用pandas读取CSV数据，matplotlib进行多维度可视化
支持ISO 8855坐标系绘制
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import matplotlib
import platform
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch

# 检测系统并设置中文字体
system = platform.system()
if system == 'Darwin':  # macOS
    # macOS中文字体优先级
    font_list = [
        'Arial Unicode MS',  # macOS系统字体
        'Songti SC',         # 宋体
        'Heiti SC',          # 黑体
        'PingFang SC',       # 苹方体
        'SimHei',            # 黑体（备选）
        'DejaVu Sans'        # 英文备选
    ]
elif system == 'Windows':  # Windows
    # Windows中文字体优先级
    font_list = [
        'SimHei',            # 黑体
        'Microsoft YaHei',   # 微软雅黑
        'SimSun',            # 宋体
        'Arial',
        'DejaVu Sans'
    ]
else:  # Linux
    # Linux中文字体优先级
    font_list = [
        'DejaVu Sans',
        'WenQuanYi Micro Hei',
        'SimHei',
        'Arial'
    ]

# 设置字体和负号
plt.rcParams['font.sans-serif'] = font_list
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 10

# 尝试清除字体缓存（如果方法存在）
try:
    import matplotlib.font_manager
    if hasattr(matplotlib.font_manager, '_rebuild'):
        matplotlib.font_manager._rebuild()
    elif hasattr(matplotlib.font_manager, 'rebuild'):
        matplotlib.font_manager.rebuild()
except Exception:
    pass  # 如果缓存清理失败，继续执行，matplotlib会自动使用配置的字体

class VehicleLocalizationVisualizer:
    """车辆定位数据可视化类"""

    # ISO 8855坐标系配置
    COORDINATE_SYSTEM = {
        'name': 'ISO 8855 (后轴中心原点)',
        'origin': '后轮轴中心 (RAC)',
        'x_axis': '纵向（前向为正）',
        'y_axis': '横向（左侧为正）',
        'z_axis': '竖向（向上为正）',
        'description': '车身固定坐标系'
    }

    def __init__(self, csv_file):
        """初始化并加载数据"""
        self.csv_file = csv_file
        self.df = None
        self.load_data()

    def load_data(self):
        """加载CSV数据"""
        print(f"📂 加载文件: {self.csv_file}")
        # 使用分号作为分隔符
        self.df = pd.read_csv(self.csv_file, sep=';')

        # 数据信息
        print(f"✅ 数据加载完成")
        print(f"   - 总数据点: {len(self.df)}")
        print(f"   - 列名: {list(self.df.columns)}")

        # 打印坐标系信息
        print(f"\n📐 坐标系信息 (ISO 8855):")
        print(f"   - 原点: {self.COORDINATE_SYSTEM['origin']}")
        print(f"   - X轴: {self.COORDINATE_SYSTEM['x_axis']}")
        print(f"   - Y轴: {self.COORDINATE_SYSTEM['y_axis']}")
        print(f"   - 类型: {self.COORDINATE_SYSTEM['description']}")

        # 计算轨迹长度
        distances = np.sqrt(np.diff(self.df['x'])**2 + np.diff(self.df['y'])**2)
        total_distance = np.sum(distances)
        print(f"\n📏 轨迹统计:")
        print(f"   - X范围: {self.df['x'].min():.2f} ~ {self.df['x'].max():.2f} m")
        print(f"   - Y范围: {self.df['y'].min():.2f} ~ {self.df['y'].max():.2f} m")
        print(f"   - 航向角范围: {np.degrees(self.df['heading'].min()):.2f}° ~ {np.degrees(self.df['heading'].max()):.2f}°")
        print(f"   - 转向角范围: {np.degrees(self.df['steering angle'].min()):.2f}° ~ {np.degrees(self.df['steering angle'].max()):.2f}°")
        print(f"   - 总行驶距离: {total_distance:.2f} m")

    def plot_trajectory(self, figsize=(12, 8)):
        """绘制车辆轨迹 (ISO 8855坐标系)"""
        fig, ax = plt.subplots(figsize=figsize)

        # 绘制轨迹线（Y为水平轴，X为垂直轴）
        ax.plot(self.df['y'], self.df['x'], 'b-', linewidth=1.5, label='车辆轨迹', alpha=0.7)

        # 标记起点和终点
        ax.plot(self.df['y'].iloc[0], self.df['x'].iloc[0], 'go',
                markersize=10, label='起点', zorder=5)
        ax.plot(self.df['y'].iloc[-1], self.df['x'].iloc[-1], 'r^',
                markersize=10, label='终点', zorder=5)

        # 添加方向箭头（每N个点添加一个）
        step = max(1, len(self.df) // 20)
        for i in range(0, len(self.df), step):
            if i + step < len(self.df):
                y1, x1 = self.df['y'].iloc[i], self.df['x'].iloc[i]
                y2, x2 = self.df['y'].iloc[i+step], self.df['x'].iloc[i+step]
                dy, dx = y2 - y1, x2 - x1
                if dx != 0 or dy != 0:
                    ax.arrow(y1, x1, dy*0.3, dx*0.3, head_width=0.8, head_length=0.5,
                             fc='cyan', ec='cyan', alpha=0.5, zorder=3)

        # 绘制坐标系标注
        self._draw_coordinate_system(ax, origin=(self.df['y'].min(), self.df['x'].min()), scale=0.8)

        # 设置标签和标题（Y为水平轴，X为垂直轴）
        ax.set_xlabel('Y 横向坐标 (m) - ISO 8855', fontsize=12, fontweight='bold')
        ax.set_ylabel('X 纵向坐标 (m) - ISO 8855', fontsize=12, fontweight='bold')
        ax.set_title('车辆定位轨迹 (ISO 8855 后轴中心坐标系)', fontsize=14, fontweight='bold')
        
        # 添加坐标系说明
        cs_info = f"坐标原点: {self.COORDINATE_SYSTEM['origin']}\n基准: {self.COORDINATE_SYSTEM['description']}"
        ax.text(0.02, 0.98, cs_info, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=10)
        ax.set_aspect('equal')

        # 美化
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        return fig

    def _draw_coordinate_system(self, ax, origin=(0, 0), scale=1.0, arrow_width=2):
        """
        在图表上绘制坐标系箭头
        
        参数:
            ax: matplotlib axes对象
            origin: 原点位置 (y, x) - 注意：已转换为(y_min, x_min)
            scale: 箭头长度缩放因子
            arrow_width: 箭头宽度
        """
        # 获取轴的当前范围
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]
        
        # 箭头长度（轴长度的10%）
        arrow_len = max(x_range, y_range) * 0.1 * scale
        
        # Y轴箭头在水平方向（红色）- 因为水平轴表示Y坐标
        arrow_y = FancyArrowPatch(origin, (origin[0] + arrow_len, origin[1]),
                                arrowstyle='->', mutation_scale=20, linewidth=arrow_width,
                                color='red', zorder=10)
        ax.add_patch(arrow_y)
        ax.text(origin[0] + arrow_len*1.1, origin[1], 'Y (横向)', fontsize=10, color='red', fontweight='bold')
        
        # X轴箭头在垂直方向（绿色）- 因为垂直轴表示X坐标
        arrow_x = FancyArrowPatch(origin, (origin[0], origin[1] + arrow_len),
                                arrowstyle='->', mutation_scale=20, linewidth=arrow_width,
                                color='green', zorder=10)
        ax.add_patch(arrow_x)
        ax.text(origin[0], origin[1] + arrow_len*1.1, 'X (纵向)', fontsize=10, color='green', fontweight='bold')
        
        # 原点标记
        ax.plot(origin[0], origin[1], 'k+', markersize=15, markeredgewidth=2, zorder=10)

    def plot_trajectory_with_heading(self, figsize=(14, 10)):
        """绘制轨迹并用颜色表示航向角和转向角 (ISO 8855坐标系)"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)

        # 左图：轨迹（用航向角着色）
        scatter = ax1.scatter(self.df['y'], self.df['x'], c=np.degrees(self.df['heading']),
                              cmap='hsv', s=20, alpha=0.6, vmin=-180, vmax=180)
        ax1.plot(self.df['y'], self.df['x'], 'gray', alpha=0.3, linewidth=0.5)
        ax1.plot(self.df['y'].iloc[0], self.df['x'].iloc[0], 'go', markersize=10, label='起点')
        ax1.plot(self.df['y'].iloc[-1], self.df['x'].iloc[-1], 'r^', markersize=10, label='终点')

        self._draw_coordinate_system(ax1, origin=(self.df['y'].min(), self.df['x'].min()), scale=0.8)

        ax1.set_xlabel('Y 横向坐标 (m) - ISO 8855', fontsize=11, fontweight='bold')
        ax1.set_ylabel('X 纵向坐标 (m) - ISO 8855', fontsize=11, fontweight='bold')
        ax1.set_title('车辆轨迹（按航向角着色）- ISO 8855', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        ax1.set_aspect('equal')

        cbar = plt.colorbar(scatter, ax=ax1)
        cbar.set_label('航向角 (°)', fontsize=10)

        # 右图：轨迹（用转向角着色）
        scatter2 = ax2.scatter(self.df['y'], self.df['x'], c=np.degrees(self.df['steering angle']),
                               cmap='RdYlBu_r', s=20, alpha=0.6)
        ax2.plot(self.df['y'], self.df['x'], 'gray', alpha=0.3, linewidth=0.5)
        ax2.plot(self.df['y'].iloc[0], self.df['x'].iloc[0], 'go', markersize=10, label='起点')
        ax2.plot(self.df['y'].iloc[-1], self.df['x'].iloc[-1], 'r^', markersize=10, label='终点')

        self._draw_coordinate_system(ax2, origin=(self.df['y'].min(), self.df['x'].min()), scale=0.8)

        ax2.set_xlabel('Y 横向坐标 (m) - ISO 8855', fontsize=11, fontweight='bold')
        ax2.set_ylabel('X 纵向坐标 (m) - ISO 8855', fontsize=11, fontweight='bold')
        ax2.set_title('车辆轨迹（按转向角着色）- ISO 8855', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best')
        ax2.set_aspect('equal')

        cbar2 = plt.colorbar(scatter2, ax=ax2)
        cbar2.set_label('转向角 (°)', fontsize=10)

        plt.tight_layout()
        return fig

    def plot_time_series(self, figsize=(14, 10)):
        """绘制时间序列图 (ISO 8855坐标系)"""
        # 计算时间步（相对时间）
        t = (self.df['timestamp'] - self.df['timestamp'].iloc[0]) / 1000  # 转换为秒

        fig, axes = plt.subplots(3, 2, figsize=figsize)

        # X位置
        axes[0, 0].plot(t, self.df['x'], 'b-', linewidth=1.5)
        axes[0, 0].set_ylabel('X 纵向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[0, 0].set_title('X 纵向位置随时间变化', fontsize=11, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3)

        # Y位置
        axes[0, 1].plot(t, self.df['y'], 'g-', linewidth=1.5)
        axes[0, 1].set_ylabel('Y 横向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[0, 1].set_title('Y 横向位置随时间变化', fontsize=11, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3)

        # 航向角
        heading_deg = np.degrees(self.df['heading'])
        axes[1, 0].plot(t, heading_deg, 'r-', linewidth=1.5)
        axes[1, 0].set_ylabel('航向角 (°)', fontsize=10)
        axes[1, 0].set_title('航向角随时间变化', fontsize=11, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)

        # 转向角
        steering_deg = np.degrees(self.df['steering angle'])
        axes[1, 1].plot(t, steering_deg, 'orange', linewidth=1.5)
        axes[1, 1].set_ylabel('转向角 (°)', fontsize=10)
        axes[1, 1].set_title('转向角随时间变化', fontsize=11, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)

        # 速度（通过位置差分估算）
        distances = np.sqrt(np.diff(self.df['x'])**2 + np.diff(self.df['y'])**2)
        time_diff = np.diff(t)
        velocity = distances / time_diff
        axes[2, 0].plot(t[:-1], velocity, 'purple', linewidth=1.5)
        axes[2, 0].set_ylabel('速度 (m/s)', fontsize=10)
        axes[2, 0].set_title('纵向速度随时间变化', fontsize=11, fontweight='bold')
        axes[2, 0].grid(True, alpha=0.3)

        # 加速度
        accel = np.diff(velocity) / time_diff[:-1]
        axes[2, 1].plot(t[:-2], accel, 'brown', linewidth=1.5)
        axes[2, 1].set_ylabel('加速度 (m/s²)', fontsize=10)
        axes[2, 1].set_title('纵向加速度随时间变化', fontsize=11, fontweight='bold')
        axes[2, 1].grid(True, alpha=0.3)

        # 统一x轴标签
        for ax in axes.flat:
            ax.set_xlabel('时间 (s)', fontsize=10)

        plt.tight_layout()
        return fig

    def plot_phase_diagram(self, figsize=(12, 8)):
        """绘制相位图 (ISO 8855坐标系)"""
        fig, axes = plt.subplots(2, 2, figsize=figsize)

        # X-Y 轨迹（Y为水平轴，X为垂直轴）
        axes[0, 0].plot(self.df['y'], self.df['x'], 'b-', linewidth=1.5, alpha=0.7)
        axes[0, 0].scatter(self.df['y'].iloc[0], self.df['x'].iloc[0], c='g', s=100, marker='o', label='起点', zorder=5)
        axes[0, 0].scatter(self.df['y'].iloc[-1], self.df['x'].iloc[-1], c='r', s=100, marker='^', label='终点', zorder=5)
        self._draw_coordinate_system(axes[0, 0], origin=(self.df['y'].min(), self.df['x'].min()), scale=0.5)
        axes[0, 0].set_xlabel('Y 横向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[0, 0].set_ylabel('X 纵向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[0, 0].set_title('X-Y 轨迹 (ISO 8855)', fontsize=11, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 0].set_aspect('equal')

        # 航向角 vs 转向角
        scatter = axes[0, 1].scatter(np.degrees(self.df['heading']),
                                     np.degrees(self.df['steering angle']),
                                     c=range(len(self.df)), cmap='viridis', s=10, alpha=0.6)
        axes[0, 1].set_xlabel('航向角 (°)', fontsize=10)
        axes[0, 1].set_ylabel('转向角 (°)', fontsize=10)
        axes[0, 1].set_title('航向角 vs 转向角相位图', fontsize=11, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3)

        # X vs 航向角
        axes[1, 0].scatter(self.df['x'], np.degrees(self.df['heading']),
                           c=range(len(self.df)), cmap='plasma', s=10, alpha=0.6)
        axes[1, 0].set_xlabel('X 纵向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[1, 0].set_ylabel('航向角 (°)', fontsize=10)
        axes[1, 0].set_title('X vs 航向角 (ISO 8855)', fontsize=11, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)

        # Y vs 航向角
        axes[1, 1].scatter(self.df['y'], np.degrees(self.df['heading']),
                           c=range(len(self.df)), cmap='inferno', s=10, alpha=0.6)
        axes[1, 1].set_xlabel('Y 横向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[1, 1].set_ylabel('航向角 (°)', fontsize=10)
        axes[1, 1].set_title('Y vs 航向角 (ISO 8855)', fontsize=11, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        return fig

    def plot_statistics(self, figsize=(14, 8)):
        """绘制统计信息 (ISO 8855坐标系)"""
        fig, axes = plt.subplots(2, 3, figsize=figsize)

        # X位置直方图
        axes[0, 0].hist(self.df['x'], bins=30, color='blue', alpha=0.7, edgecolor='black')
        axes[0, 0].set_xlabel('X 纵向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[0, 0].set_ylabel('频数', fontsize=10)
        axes[0, 0].set_title('X 位置分布', fontsize=11, fontweight='bold')
        axes[0, 0].axvline(self.df['x'].mean(), color='red', linestyle='--', label=f"均值: {self.df['x'].mean():.2f}")
        axes[0, 0].legend()

        # Y位置直方图
        axes[0, 1].hist(self.df['y'], bins=30, color='green', alpha=0.7, edgecolor='black')
        axes[0, 1].set_xlabel('Y 横向坐标 (m) - ISO 8855', fontsize=10, fontweight='bold')
        axes[0, 1].set_ylabel('频数', fontsize=10)
        axes[0, 1].set_title('Y 位置分布', fontsize=11, fontweight='bold')
        axes[0, 1].axvline(self.df['y'].mean(), color='red', linestyle='--', label=f"均值: {self.df['y'].mean():.2f}")
        axes[0, 1].legend()

        # 航向角直方图
        heading_deg = np.degrees(self.df['heading'])
        axes[0, 2].hist(heading_deg, bins=36, color='red', alpha=0.7, edgecolor='black')
        axes[0, 2].set_xlabel('航向角 (°)', fontsize=10)
        axes[0, 2].set_ylabel('频数', fontsize=10)
        axes[0, 2].set_title('航向角分布', fontsize=11, fontweight='bold')
        axes[0, 2].axvline(heading_deg.mean(), color='darkred', linestyle='--', label=f"均值: {heading_deg.mean():.2f}°")
        axes[0, 2].legend()

        # 转向角直方图
        steering_deg = np.degrees(self.df['steering angle'])
        axes[1, 0].hist(steering_deg, bins=30, color='orange', alpha=0.7, edgecolor='black')
        axes[1, 0].set_xlabel('转向角 (°)', fontsize=10)
        axes[1, 0].set_ylabel('频数', fontsize=10)
        axes[1, 0].set_title('转向角分布', fontsize=11, fontweight='bold')
        axes[1, 0].axvline(steering_deg.mean(), color='darkorange', linestyle='--', label=f"均值: {steering_deg.mean():.2f}°")
        axes[1, 0].legend()

        # 速度分布
        distances = np.sqrt(np.diff(self.df['x'])**2 + np.diff(self.df['y'])**2)
        t = (self.df['timestamp'] - self.df['timestamp'].iloc[0]) / 1000
        time_diff = np.diff(t)
        velocity = distances / time_diff
        axes[1, 1].hist(velocity, bins=30, color='purple', alpha=0.7, edgecolor='black')
        axes[1, 1].set_xlabel('速度 (m/s)', fontsize=10)
        axes[1, 1].set_ylabel('频数', fontsize=10)
        axes[1, 1].set_title('速度分布', fontsize=11, fontweight='bold')
        axes[1, 1].axvline(np.mean(velocity), color='darkviolet', linestyle='--', label=f"均值: {np.mean(velocity):.4f}")
        axes[1, 1].legend()

        # 显示统计信息文本和坐标系说明
        stats_text = f"""数据统计信息 (ISO 8855)
────────────────────
总数据点: {len(self.df)}

X 纵向坐标:
  均值: {self.df['x'].mean():.2f} m
  范围: {self.df['x'].min():.2f}~{self.df['x'].max():.2f} m

Y 横向坐标:
  均值: {self.df['y'].mean():.2f} m
  范围: {self.df['y'].min():.2f}~{self.df['y'].max():.2f} m

航向角 (相对北向):
  均值: {heading_deg.mean():.2f}°
  范围: {heading_deg.min():.2f}~{heading_deg.max():.2f}°

转向角 (前轮):
  均值: {steering_deg.mean():.2f}°
  范围: {steering_deg.min():.2f}~{steering_deg.max():.2f}°

速度 (纵向):
  均值: {np.mean(velocity):.4f} m/s
  范围: {np.min(velocity):.4f}~{np.max(velocity):.4f} m/s

坐标系: {self.COORDINATE_SYSTEM['name']}
原点: {self.COORDINATE_SYSTEM['origin']}"""

        axes[1, 2].text(0.05, 0.5, stats_text, fontsize=8, family='monospace',
                        verticalalignment='center',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        axes[1, 2].axis('off')

        plt.tight_layout()
        return fig

    def save_all_plots(self, output_dir='vehicle_localization_plots'):
        """保存所有图表"""
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"📁 创建输出目录: {output_dir}")

        print(f"\n📊 生成可视化图表 (ISO 8855坐标系)...")

        # 绘制并保存各个图表
        plots = [
            ('01_trajectory_iso8855.png', self.plot_trajectory, '轨迹图 (ISO 8855)'),
            ('02_trajectory_with_color_iso8855.png', self.plot_trajectory_with_heading, '着色轨迹图 (ISO 8855)'),
            ('03_time_series.png', self.plot_time_series, '时间序列图'),
            ('04_phase_diagram_iso8855.png', self.plot_phase_diagram, '相位图 (ISO 8855)'),
            ('05_statistics_iso8855.png', self.plot_statistics, '统计图表 (ISO 8855)'),
        ]

        for filename, plot_func, description in plots:
            try:
                fig = plot_func()
                filepath = os.path.join(output_dir, filename)
                fig.savefig(filepath, dpi=150, bbox_inches='tight')
                print(f"  ✅ {description}: {filepath}")
                plt.close(fig)
            except Exception as e:
                print(f"  ❌ {description} 失败: {e}")
                import traceback
                traceback.print_exc()
                plt.close('all')

        print(f"\n✨ 所有图表已保存到: {os.path.abspath(output_dir)}/")
        
        # 生成坐标系说明文档
        self._save_coordinate_system_doc(output_dir)

    def _save_coordinate_system_doc(self, output_dir):
        """生成坐标系说明文档"""
        doc_path = os.path.join(output_dir, '00_ISO8855_COORDINATE_SYSTEM.txt')
        
        doc_content = f"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                       ISO 8855 车辆坐标系说明                                ║
╚══════════════════════════════════════════════════════════════════════════════╝

📐 坐标系信息
════════════════════════════════════════════════════════════════════════════════
坐标系名称:   {self.COORDINATE_SYSTEM['name']}
坐标系原点:   {self.COORDINATE_SYSTEM['origin']}
坐标系类型:   {self.COORDINATE_SYSTEM['description']}

📍 轴定义
════════════════════════════════════════════════════════════════════════════════
X 轴 (纵向):  {self.COORDINATE_SYSTEM['x_axis']}
  - 正向: 车辆前进方向
  - 负向: 车辆后退方向
  - 单位: 米 (m)

Y 轴 (横向):  {self.COORDINATE_SYSTEM['y_axis']}
  - 正向: 车身左侧（驾驶员左手方向）
  - 负向: 车身右侧
  - 单位: 米 (m)

Z 轴 (竖向):  {self.COORDINATE_SYSTEM['z_axis']}
  - 正向: 竖直向上
  - 单位: 米 (m)

🔧 关键参数定义
════════════════════════════════════════════════════════════════════════════════
航向角 (Heading/Yaw):
  - 定义: 车身相对于参考方向的旋转角度
  - 参考: 北向（0°）或初始方向
  - 范围: -180° 到 180°
  - 正向: 逆时针旋转
  - 负向: 顺时针旋转

转向角 (Steering Angle):
  - 定义: 前轮相对于车身中心线的转向角
  - 正向: 左转
  - 负向: 右转
  - 单位: 弧度 (rad) 或 度 (°)

速度 (Velocity):
  - 定义: 车辆沿X轴（纵向）的速度
  - 正向: 前进运动
  - 负向: 后退运动
  - 单位: 米/秒 (m/s)

加速度 (Acceleration):
  - 定义: 速度的时间导数
  - 正向: 加速
  - 负向: 减速
  - 单位: 米/平方秒 (m/s²)

📊 数据统计
════════════════════════════════════════════════════════════════════════════════
数据点总数:     {len(self.df):,}

X 坐标范围:     {self.df['x'].min():.2f} ~ {self.df['x'].max():.2f} m
  - 平均值:     {self.df['x'].mean():.2f} m
  - 标准差:     {self.df['x'].std():.2f} m

Y 坐标范围:     {self.df['y'].min():.2f} ~ {self.df['y'].max():.2f} m
  - 平均值:     {self.df['y'].mean():.2f} m
  - 标准差:     {self.df['y'].std():.2f} m

航向角范围:     {np.degrees(self.df['heading'].min()):.2f}° ~ {np.degrees(self.df['heading'].max()):.2f}°
  - 平均值:     {np.degrees(self.df['heading'].mean()):.2f}°

转向角范围:     {np.degrees(self.df['steering angle'].min()):.2f}° ~ {np.degrees(self.df['steering angle'].max()):.2f}°
  - 平均值:     {np.degrees(self.df['steering angle'].mean()):.2f}°

📈 运动学模型
════════════════════════════════════════════════════════════════════════════════
本系统采用Ackermann转向模型：
  
  - 后轴中心为参考点（RAC: Rear Axle Center）
  - 不考虑车身侧滑（假设无侧向滑动）
  - 前轮转向角为控制输入
  - 车身长度为2.7m（定标准车辆）

运动方程：
  dx/dt = v * cos(ψ)
  dy/dt = v * sin(ψ)
  dψ/dt = (v/L) * tan(δ)
  
其中：
  (x, y)   = 车辆在ISO 8855坐标系中的位置
  v        = 车辆纵向速度
  ψ        = 车身航向角
  δ        = 前轮转向角
  L        = 车辆轴距 = 2.7 m

🎯 使用说明
════════════════════════════════════════════════════════════════════════════════
1. 图表坐标轴
   - X轴表示车辆的纵向位置（前后移动）
   - Y轴表示车辆的横向位置（左右移动）
   - 原点为后轮轴中心的初始位置

2. 颜色编码
   - 轨迹线: 蓝色表示车辆行驶路径
   - 着色轨迹: 不同颜色表示航向角或转向角的变化
   - 起点: 绿色圆点 (●)
   - 终点: 红色三角形 (▲)

3. 箭头符号
   - 红色箭头: X轴正向（纵向前进）
   - 绿色箭头: Y轴正向（横向左侧）
   - 十字符号: 坐标系原点

📝 注意事项
════════════════════════════════════════════════════════════════════════════════
- 所有坐标均基于ISO 8855后轴中心坐标系
- 时间序列数据基于传感器时间戳生成
- 速度和加速度通过位置微分计算
- 所有角度均以弧度为基本单位，但显示时转换为度
- Y坐标的符号约定：左为正，右为负

✅ 验证信息
════════════════════════════════════════════════════════════════════════════════
坐标系标准:     ISO 8855:1991（标准更新至ISO 8855:2011）
实现日期:       2025年12月29日
数据时间戳:     基于车载传感器系统时间
坐标原点固定:   后轮轴中心（车辆初始位置）

"""
        
        try:
            with open(doc_path, 'w', encoding='utf-8') as f:
                f.write(doc_content)
            print(f"\n📄 坐标系说明已保存: {doc_path}")
        except Exception as e:
            print(f"⚠️  坐标系说明保存失败: {e}")


def main():
    """主函数"""
    csv_file = 'vehicle_localization.csv'

    # 检查文件是否存在
    if not os.path.exists(csv_file):
        print(f"❌ 文件不存在: {csv_file}")
        print(f"   请检查路径是否正确")
        return

    # 创建可视化对象
    visualizer = VehicleLocalizationVisualizer(csv_file)

    # 保存所有图表
    visualizer.save_all_plots()

    # 显示所有图表
    print(f"\n🎨 显示图表...")
    try:
        plt.show()
    except Exception as e:
        print(f"⚠️  显示图表失败: {e}")
        print(f"   但图表已保存到文件，可直接打开查看")


if __name__ == '__main__':
    main()
