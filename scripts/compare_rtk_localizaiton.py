#!/usr/bin/env python3
"""
RTK轨迹对比与叠加显示脚本

支持三种数据源的轨迹对比：
1. rtk_converted.csv - RTK转换后的坐标（ISO 8855车体坐标系）
2. vehicle_localization.csv - EKF定位结果
3. rtk_can.csv - 原始RTK数据（经纬度，可选）

功能：
- 轨迹叠加显示
- 误差热力图
- 误差时间序列分析
- 轨迹细节对比
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.interpolate import interp1d
import seaborn as sns
import matplotlib
import platform

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
    if hasattr(matplotlib.font_manager, '_rebuild'):
        matplotlib.font_manager._rebuild()
    elif hasattr(matplotlib.font_manager, 'rebuild'):
        matplotlib.font_manager.rebuild()
except Exception:
    pass  # 如果缓存清理失败，继续执行，matplotlib会自动使用配置的字体

# ========== 🔧 C代码配置参数（与src/localization_system.c同步）==========
# 来源：src/localization_system.c line 129-133
# 
# 轮速计配置
WHEEL_CONFIG = {
    'pulses_per_wheel_rev': 96,      # 编码器：每轮每圈96脉冲（正确的原始值）
    'wheel_radius': 0.35,             # 轮半径 (m)
    'low_speed_threshold': 0.3,       # 低速阈值 (m/s)，默认 0.3
    'pulse_timeout_ms': 500,          # 脉冲超时 (ms)，默认 500
    'max_count': 4096,                # 脉冲计数器最大值+1 (4095+1)，用于处理溢出
}

# 计算轮周长（用于距离计算验证）
WHEEL_CIRCUMFERENCE = 2.0 * np.pi * WHEEL_CONFIG['wheel_radius']  # ≈ 2.199 m

# 融合算法参数（与src/localization_system.c同步）
FUSION_MODE_BASIC = 0           # 基础融合：轮速 + IMU加速度 + 转向角
FUSION_MODE_IMU_STEERING = 1    # IMU转向融合：添加IMU角速度用于转向增强

class TrajectoryComparison:
    """RTK与定位轨迹对比类"""

    def __init__(self, rtk_converted_file, localization_file, rtk_raw_file=None):
        """初始化并加载数据

        Args:
            rtk_converted_file: RTK转换后的数据文件(rtk_converted.csv)
            localization_file: 定位数据文件(vehicle_localization.csv)
            rtk_raw_file: 原始RTK数据文件(rtk_can.csv，可选)
        """
        self.rtk_converted_file = rtk_converted_file
        self.localization_file = localization_file
        self.rtk_raw_file = rtk_raw_file

        self.rtk_df = None          # RTK转换后的数据（已在车体坐标系）
        self.rtk_raw_df = None      # 原始RTK数据（可选）
        self.loc_df = None
        self.errors = None

        self.load_data()

    def load_data(self):
        """加载数据"""
        print(f"📂 加载RTK转换数据: {self.rtk_converted_file}")
        self.rtk_df = pd.read_csv(self.rtk_converted_file, sep=';')
        print(f"   ✅ RTK转换数据点: {len(self.rtk_df)}")

        print(f"📂 加载定位数据: {self.localization_file}")
        self.loc_df = pd.read_csv(self.localization_file, sep=';')
        print(f"   ✅ 定位数据点: {len(self.loc_df)}")

        # 可选：加载原始RTK数据
        if self.rtk_raw_file and os.path.exists(self.rtk_raw_file):
            print(f"📂 加载原始RTK数据: {self.rtk_raw_file}")
            self.rtk_raw_df = pd.read_csv(self.rtk_raw_file, sep=';')
            print(f"   ✅ 原始RTK数据点: {len(self.rtk_raw_df)}")

        # 🔧 打印C代码的关键参数（与src/localization_system.c同步）
        self._print_config_info()
        
        # 时间同步
        self.sync_timestamps()

    def _print_config_info(self):
        """打印C代码的配置参数信息"""
        print(f"\n🔧 系统参数信息（来自src/localization_system.c）:")
        print(f"   └─ 轮速计配置:")
        print(f"      • pulses_per_wheel_rev: {WHEEL_CONFIG['pulses_per_wheel_rev']} (每圈脉冲数)")
        print(f"      • wheel_radius: {WHEEL_CONFIG['wheel_radius']} m (轮半径)")
        print(f"      • wheel_circumference: {WHEEL_CIRCUMFERENCE:.4f} m (轮周长 = 2πr)")
        print(f"      • max_count: {WHEEL_CONFIG['max_count']} (脉冲计数器上限，处理溢出)")
        print(f"      • low_speed_threshold: {WHEEL_CONFIG['low_speed_threshold']} m/s")
        print(f"      • pulse_timeout_ms: {WHEEL_CONFIG['pulse_timeout_ms']} ms")
        print(f"   └─ 融合算法:")
        print(f"      • FUSION_MODE_BASIC: 轮速 + IMU加速度 + 转向角")
        print(f"      • FUSION_MODE_IMU_STEERING: 基础 + IMU角速度转向增强")

    def sync_timestamps(self):
        """同步两个数据源的时间戳"""
        # RTK转换数据使用 timestamp (秒级，浮点)
        rtk_time_base = self.rtk_df['timestamp'].iloc[0]
        self.rtk_df['time_sync'] = self.rtk_df['timestamp'] - rtk_time_base

        # 定位数据使用 timestamp (毫秒级，整数)
        loc_time_base = self.loc_df['timestamp'].iloc[0]
        self.loc_df['time_sync'] = (self.loc_df['timestamp'] - loc_time_base) / 1000.0

        print(f"\n⏱️  时间同步:")
        print(f"   RTK时间跨度: {self.rtk_df['time_sync'].min():.2f}s ~ {self.rtk_df['time_sync'].max():.2f}s")
        print(f"   定位时间跨度: {self.loc_df['time_sync'].min():.2f}s ~ {self.loc_df['time_sync'].max():.2f}s")

    def calculate_errors(self):
        """计算定位误差"""
        print(f"\n📐 计算定位误差...")

        # 使用RTK数据作为参考
        rtk_x = self.rtk_df['x'].values
        rtk_y = self.rtk_df['y'].values
        rtk_time = self.rtk_df['time_sync'].values

        loc_x = self.loc_df['x'].values
        loc_y = self.loc_df['y'].values
        loc_time = self.loc_df['time_sync'].values

        # 对定位数据进行插值，与RTK时间对齐
        if len(loc_time) > 1:
            f_loc_x = interp1d(loc_time, loc_x, kind='linear',
                               bounds_error=False, fill_value='extrapolate')
            f_loc_y = interp1d(loc_time, loc_y, kind='linear',
                               bounds_error=False, fill_value='extrapolate')

            # 在RTK时间点评估定位数据
            loc_x_interp = f_loc_x(rtk_time)
            loc_y_interp = f_loc_y(rtk_time)
        else:
            loc_x_interp = loc_x[0] * np.ones_like(rtk_x)
            loc_y_interp = loc_y[0] * np.ones_like(rtk_y)

        # 计算误差
        dx = loc_x_interp - rtk_x
        dy = loc_y_interp - rtk_y
        error_distance = np.sqrt(dx**2 + dy**2)

        self.errors = pd.DataFrame({
            'time': rtk_time,
            'rtk_x': rtk_x,
            'rtk_y': rtk_y,
            'loc_x': loc_x_interp,
            'loc_y': loc_y_interp,
            'error_x': dx,
            'error_y': dy,
            'error_distance': error_distance
        })

        # 打印误差统计
        print(f"   误差统计:")
        print(f"   • 均值误差: {self.errors['error_distance'].mean():.4f} m")
        print(f"   • 最大误差: {self.errors['error_distance'].max():.4f} m")
        print(f"   • 最小误差: {self.errors['error_distance'].min():.4f} m")
        print(f"   • 标准差: {self.errors['error_distance'].std():.4f} m")
        print(f"   • 中位数误差: {self.errors['error_distance'].median():.4f} m")
        print(f"   • RMS误差: {np.sqrt(np.mean(self.errors['error_distance']**2)):.4f} m")
        
        # 🔧 脉冲计数验证（基于wheel_odometry_processor.c的计算）
        self._validate_pulse_counts()

    def _validate_pulse_counts(self):
        """验证脉冲计数与距离是否匹配（基于wheel_odometry_processor.c）
        
        公式来源：src/wheel_odometry_processor.c line 232-237
        distance_delta = (double)pulse_delta / pulses_per_wheel_rev 
                       * wheel_circumference
        """
        print(f"\n✓ 脉冲计数验证（与src/wheel_odometry_processor.c同步）:")
        
        # 计算定位系统输出的总距离
        if len(self.loc_df) > 1:
            total_distance_loc = self.loc_df['distance'].iloc[-1]
            print(f"   • 定位系统总距离: {total_distance_loc:.2f} m")
            
            # 计算RTK参考距离
            rtk_points = np.array([self.rtk_df['x'].values, self.rtk_df['y'].values]).T
            segments = np.diff(rtk_points, axis=0)
            segment_distances = np.linalg.norm(segments, axis=1)
            total_distance_rtk = np.sum(segment_distances)
            print(f"   • RTK参考总距离: {total_distance_rtk:.2f} m")
            
            # 计算误差百分比
            if total_distance_rtk > 0:
                error_percent = abs(total_distance_loc - total_distance_rtk) / total_distance_rtk * 100
                print(f"   • 距离误差: {error_percent:.2f}%")
                
                # 如果误差小于2%，输出成功信息
                if error_percent < 2.0:
                    print(f"   ✅ 脉冲计数与距离计算正确（误差 < 2%）")
                else:
                    print(f"   ⚠️  距离误差较大，可能存在：")
                    print(f"      • 脉冲计数器溢出处理问题")
                    print(f"      • 异常CAN帧影响脉冲累计")
                    print(f"      • 检查max_count: {WHEEL_CONFIG['max_count']}")


    def plot_trajectory_overlay(self, figsize=(14, 10)):
        """绘制轨迹叠加图 - RTK转换坐标 vs 定位轨迹
        
        坐标系说明（ISO 8855车体坐标系，与src/localization_system.c同步）：
        • X轴：纵向（前进方向为正）- 在图表中为纵轴
        • Y轴：横向（左为正，右为负）- 在图表中为横轴，使用 -y 处理
        • Z轴：竖直向上为正
        
        重新定义坐标轴显示：
        • 横轴（横向）：y坐标取反显示，左正右负（驾驶员视角）
        • 纵轴（纵向）：x坐标直接显示，上正下负
        """
        fig, ax = plt.subplots(figsize=figsize)

        # 绘制RTK转换轨迹（已在车体坐标系）
        # 坐标轴重新定义：y轴为横轴（左正右负），x轴为纵轴
        ax.plot(-self.rtk_df['y'], self.rtk_df['x'],
                'b-', linewidth=2.5, label='RTK轨迹（车体坐标系）', alpha=0.8, zorder=2)
        ax.plot(-self.rtk_df['y'].iloc[0], self.rtk_df['x'].iloc[0],
                'bo', markersize=12, label='RTK起点', zorder=5)
        ax.plot(-self.rtk_df['y'].iloc[-1], self.rtk_df['x'].iloc[-1],
                'bs', markersize=12, label='RTK终点', zorder=5)

        # 绘制定位轨迹
        ax.plot(-self.loc_df['y'], self.loc_df['x'],
                'r--', linewidth=2, label='EKF定位轨迹', alpha=0.7, zorder=2)
        ax.plot(-self.loc_df['y'].iloc[0], self.loc_df['x'].iloc[0],
                'ro', markersize=12, label='定位起点', zorder=5)
        ax.plot(-self.loc_df['y'].iloc[-1], self.loc_df['x'].iloc[-1],
                'rs', markersize=12, label='定位终点', zorder=5)

        # 绘制误差向量（每N个点绘制一个）
        if self.errors is not None:
            step = max(1, len(self.errors) // 30)
            for i in range(0, len(self.errors), step):
                if self.errors['error_distance'].iloc[i] > 0.05:
                    ax.arrow(-self.errors['rtk_y'].iloc[i], self.errors['rtk_x'].iloc[i],
                             -self.errors['error_y'].iloc[i]*0.8, self.errors['error_x'].iloc[i]*0.8,
                             head_width=0.3, head_length=0.2, fc='orange', ec='orange',
                             alpha=0.5, zorder=3)

        ax.set_xlabel('Y 坐标 (m) [左正右负]', fontsize=12)
        ax.set_ylabel('X 坐标 (m) [纵轴]', fontsize=12)
        ax.set_title('RTK轨迹 vs EKF定位轨迹叠加对比\n（ISO 8855车体坐标系，重新定义坐标轴）',
                     fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=11)
        ax.set_aspect('equal')
        
        # 🔧 设置坐标轴范围：最小值±10，最大值由轨迹范围决定
        # 横轴（Y坐标）范围
        y_min = min(-self.rtk_df['y'].min(), -self.loc_df['y'].min())
        y_max = max(-self.rtk_df['y'].max(), -self.loc_df['y'].max())
        y_range = y_max - y_min
        
        # 如果范围小于20，则强制设定为-10到+10
        if y_range < 20:
            xlim_min, xlim_max = -10, 10
        else:
            xlim_min, xlim_max = y_min - y_range*0.1, y_max + y_range*0.1
        
        # 纵轴（X坐标）范围
        x_min = min(self.rtk_df['x'].min(), self.loc_df['x'].min())
        x_max = max(self.rtk_df['x'].max(), self.loc_df['x'].max())
        x_range = x_max - x_min
        
        # 如果范围小于20，则强制设定为-10到+10
        if x_range < 20:
            ylim_min, ylim_max = -10, 10
        else:
            ylim_min, ylim_max = x_min - x_range*0.1, x_max + x_range*0.1
        
        ax.set_xlim(xlim_min, xlim_max)
        ax.set_ylim(ylim_min, ylim_max)
        ax.invert_xaxis()  # 🔧 反转x轴：左正右负

        # 添加误差信息文本
        if self.errors is not None:
            error_text = f"""误差统计:
均值: {self.errors['error_distance'].mean():.4f} m
最大: {self.errors['error_distance'].max():.4f} m
最小: {self.errors['error_distance'].min():.4f} m
RMS: {np.sqrt(np.mean(self.errors['error_distance']**2)):.4f} m"""
            ax.text(0.02, 0.98, error_text, transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    family='monospace')

        plt.tight_layout()
        return fig

    def plot_trajectory_overlay_with_raw_rtk(self, figsize=(14, 10)):
        """绘制三轨迹叠加图（包含原始RTK数据）
        
        三个轨迹数据源：
        1. rtk_converted.csv - RTK转换坐标（ISO 8855车体坐标系）
        2. vehicle_localization.csv - EKF融合定位结果（轮速 + IMU + 转向）
        3. rtk_can.csv - 原始RTK数据（经纬度，可选）
        
        融合算法源码：src/localization_system.c, src/kalman_filter.c
        """
        if self.rtk_raw_df is None:
            print("⚠️ 原始RTK数据不可用，使用两轨迹对比")
            return self.plot_trajectory_overlay(figsize)

        fig, ax = plt.subplots(figsize=figsize)

        # 绘制原始RTK轨迹（经纬度转换后）
        if 'X' in self.rtk_raw_df.columns and 'Y' in self.rtk_raw_df.columns:
            ax.plot(-self.rtk_raw_df['Y'], self.rtk_raw_df['X'],
                    'g:', linewidth=2, label='原始RTK轨迹（未处理）', alpha=0.6, zorder=1)

        # 绘制RTK转换轨迹
        ax.plot(-self.rtk_df['y'], self.rtk_df['x'],
                'b-', linewidth=2.5, label='RTK转换轨迹', alpha=0.8, zorder=2)
        ax.plot(-self.rtk_df['y'].iloc[0], self.rtk_df['x'].iloc[0],
                'bo', markersize=10, zorder=5)

        # 绘制定位轨迹
        ax.plot(-self.loc_df['y'], self.loc_df['x'],
                'r--', linewidth=2, label='EKF定位轨迹', alpha=0.7, zorder=2)
        ax.plot(-self.loc_df['y'].iloc[0], self.loc_df['x'].iloc[0],
                'ro', markersize=10, zorder=5)

        ax.set_xlabel('Y 坐标 (m) [左正右负]', fontsize=12)
        ax.set_ylabel('X 坐标 (m) [纵轴]', fontsize=12)
        ax.set_title('三轨迹对比：原始RTK vs RTK转换 vs EKF定位', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=11)
        ax.set_aspect('equal')
        
        # 🔧 设置坐标轴范围：最小值±10，最大值由轨迹范围决定
        # 计算所有轨迹的范围
        y_values = [-self.rtk_df['y'].values, -self.loc_df['y'].values]
        if 'Y' in self.rtk_raw_df.columns:
            y_values.append(-self.rtk_raw_df['Y'].values)
        y_all = np.concatenate(y_values)
        
        x_values = [self.rtk_df['x'].values, self.loc_df['x'].values]
        if 'X' in self.rtk_raw_df.columns:
            x_values.append(self.rtk_raw_df['X'].values)
        x_all = np.concatenate(x_values)
        
        # 横轴（Y坐标）范围
        y_min, y_max = np.nanmin(y_all), np.nanmax(y_all)
        y_range = y_max - y_min
        
        if y_range < 20:
            xlim_min, xlim_max = -10, 10
        else:
            xlim_min, xlim_max = y_min - y_range*0.1, y_max + y_range*0.1
        
        # 纵轴（X坐标）范围
        x_min, x_max = np.nanmin(x_all), np.nanmax(x_all)
        x_range = x_max - x_min
        
        if x_range < 20:
            ylim_min, ylim_max = -10, 10
        else:
            ylim_min, ylim_max = x_min - x_range*0.1, x_max + x_range*0.1
        
        ax.set_xlim(xlim_min, xlim_max)
        ax.set_ylim(ylim_min, ylim_max)
        ax.invert_xaxis()  # 🔧 反转x轴：左正右负

        plt.tight_layout()
        return fig

    def plot_error_map(self, figsize=(14, 10)):
        """绘制误差热力图
        
        分析定位误差的空间分布：
        • 左图：定位误差热力图（相对于RTK参考轨迹）
        • 右图：定位误差向量场（显示误差的大小和方向）
        
        误差计算公式（src/localization_system.c）：
        error = sqrt((loc_x - rtk_x)² + (loc_y - rtk_y)²)
        
        热力图颜色：
        • 绿色：误差小（< 0.05m）
        • 黄色：误差中等（0.05-0.15m）
        • 红色：误差大（> 0.15m）
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)

        # 左图：误差距离热力图
        scatter1 = ax1.scatter(-self.errors['rtk_y'], self.errors['rtk_x'],
                               c=self.errors['error_distance'], cmap='RdYlGn_r',
                               s=50, alpha=0.6, edgecolors='black', linewidth=0.5)
        ax1.plot(-self.rtk_df['y'], self.rtk_df['x'],
                 'b-', linewidth=1, alpha=0.3, zorder=1)
        ax1.set_xlabel('Y 坐标 (m) [左正右负]', fontsize=11)
        ax1.set_ylabel('X 坐标 (m) [纵轴]', fontsize=11)
        ax1.set_title('定位误差分布（距离热力图）', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        
        # 🔧 设置坐标轴范围
        y_min = -self.errors['rtk_y'].max()
        y_max = -self.errors['rtk_y'].min()
        y_range = y_max - y_min
        if y_range < 20:
            xlim_min, xlim_max = -10, 10
        else:
            xlim_min, xlim_max = y_min - y_range*0.1, y_max + y_range*0.1
        
        x_min = self.errors['rtk_x'].min()
        x_max = self.errors['rtk_x'].max()
        x_range = x_max - x_min
        if x_range < 20:
            ylim_min, ylim_max = -10, 10
        else:
            ylim_min, ylim_max = x_min - x_range*0.1, x_max + x_range*0.1
        
        ax1.set_xlim(xlim_min, xlim_max)
        ax1.set_ylim(ylim_min, ylim_max)
        ax1.invert_xaxis()  # 🔧 反转x轴：左正右负
        
        cbar1 = plt.colorbar(scatter1, ax=ax1)
        cbar1.set_label('误差距离 (m)', fontsize=10)

        # 右图：X和Y方向误差向量场
        step = max(1, len(self.errors) // 40)
        scatter2 = ax2.quiver(-self.errors['rtk_y'][::step], self.errors['rtk_x'][::step],
                              -self.errors['error_y'][::step], self.errors['error_x'][::step],
                              self.errors['error_distance'][::step], cmap='RdYlGn_r',
                              scale=20, scale_units='inches', alpha=0.7, width=0.003)
        ax2.plot(-self.rtk_df['y'], self.rtk_df['x'],
                 'b-', linewidth=1, alpha=0.3, zorder=1)
        ax2.set_xlabel('Y 坐标 (m) [左正右负]', fontsize=11)
        ax2.set_ylabel('X 坐标 (m) [纵轴]', fontsize=11)
        ax2.set_title('定位误差分布（误差向量场）', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')
        
        # 🔧 设置坐标轴范围（与左图相同）
        ax2.set_xlim(xlim_min, xlim_max)
        ax2.set_ylim(ylim_min, ylim_max)
        ax2.invert_xaxis()  # 🔧 反转x轴：左正右负
        
        cbar2 = plt.colorbar(scatter2, ax=ax2)
        cbar2.set_label('误差距离 (m)', fontsize=10)

        plt.tight_layout()
        return fig

    def plot_error_timeseries(self, figsize=(14, 8)):
        """绘制误差时间序列"""
        fig, axes = plt.subplots(3, 1, figsize=figsize)

        t = self.errors['time']

        # X方向误差
        axes[0].plot(t, self.errors['error_x'], 'r-', linewidth=1.5, label='X方向误差')
        axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[0].fill_between(t, self.errors['error_x'], 0, alpha=0.3, color='red')
        axes[0].set_ylabel('X误差 (m)', fontsize=11)
        axes[0].set_title('定位误差时间序列', fontsize=12, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        # Y方向误差
        axes[1].plot(t, self.errors['error_y'], 'g-', linewidth=1.5, label='Y方向误差')
        axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[1].fill_between(t, self.errors['error_y'], 0, alpha=0.3, color='green')
        axes[1].set_ylabel('Y误差 (m)', fontsize=11)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()

        # 总误差距离
        axes[2].plot(t, self.errors['error_distance'], 'b-', linewidth=2, label='总误差距离')
        axes[2].fill_between(t, self.errors['error_distance'], 0, alpha=0.3, color='blue')
        axes[2].axhline(y=self.errors['error_distance'].mean(), color='r',
                        linestyle='--', alpha=0.7, label=f"平均误差: {self.errors['error_distance'].mean():.4f}m")
        axes[2].set_ylabel('误差距离 (m)', fontsize=11)
        axes[2].set_xlabel('时间 (s)', fontsize=11)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()

        for ax in axes:
            ax.set_xlabel('时间 (s)', fontsize=11)

        plt.tight_layout()
        return fig

    def plot_error_statistics(self, figsize=(14, 8)):
        """绘制误差统计分析"""
        fig, axes = plt.subplots(2, 2, figsize=figsize)

        # 误差距离直方图
        axes[0, 0].hist(self.errors['error_distance'], bins=50, color='blue', alpha=0.7, edgecolor='black')
        axes[0, 0].axvline(self.errors['error_distance'].mean(), color='red',
                           linestyle='--', linewidth=2, label=f"均值: {self.errors['error_distance'].mean():.4f}m")
        axes[0, 0].axvline(self.errors['error_distance'].median(), color='green',
                           linestyle='--', linewidth=2, label=f"中位数: {self.errors['error_distance'].median():.4f}m")
        axes[0, 0].set_xlabel('误差距离 (m)', fontsize=11)
        axes[0, 0].set_ylabel('频数', fontsize=11)
        axes[0, 0].set_title('误差距离分布', fontsize=12, fontweight='bold')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        # X误差分布
        axes[0, 1].hist(self.errors['error_x'], bins=50, color='red', alpha=0.7, edgecolor='black')
        axes[0, 1].axvline(self.errors['error_x'].mean(), color='darkred',
                           linestyle='--', linewidth=2, label=f"均值: {self.errors['error_x'].mean():.4f}m")
        axes[0, 1].set_xlabel('X误差 (m)', fontsize=11)
        axes[0, 1].set_ylabel('频数', fontsize=11)
        axes[0, 1].set_title('X方向误差分布', fontsize=12, fontweight='bold')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        # Y误差分布
        axes[1, 0].hist(self.errors['error_y'], bins=50, color='green', alpha=0.7, edgecolor='black')
        axes[1, 0].axvline(self.errors['error_y'].mean(), color='darkgreen',
                           linestyle='--', linewidth=2, label=f"均值: {self.errors['error_y'].mean():.4f}m")
        axes[1, 0].set_xlabel('Y误差 (m)', fontsize=11)
        axes[1, 0].set_ylabel('频数', fontsize=11)
        axes[1, 0].set_title('Y方向误差分布', fontsize=12, fontweight='bold')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        # 误差统计信息
        stats_text = f"""误差统计详情
────────────────────
总数据点: {len(self.errors)}

距离误差:
  均值: {self.errors['error_distance'].mean():.4f} m
  中位数: {self.errors['error_distance'].median():.4f} m
  最大: {self.errors['error_distance'].max():.4f} m
  最小: {self.errors['error_distance'].min():.4f} m
  标准差: {self.errors['error_distance'].std():.4f} m
  RMS: {np.sqrt(np.mean(self.errors['error_distance']**2)):.4f} m

X误差:
  均值: {self.errors['error_x'].mean():.4f} m
  范围: [{self.errors['error_x'].min():.4f}, {self.errors['error_x'].max():.4f}] m

Y误差:
  均值: {self.errors['error_y'].mean():.4f} m
  范围: [{self.errors['error_y'].min():.4f}, {self.errors['error_y'].max():.4f}] m"""

        axes[1, 1].text(0.05, 0.5, stats_text, fontsize=9, family='monospace',
                        verticalalignment='center',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        axes[1, 1].axis('off')

        plt.tight_layout()
        return fig

    def plot_comparison_detail(self, figsize=(14, 8)):
        """绘制轨迹细节对比"""
        fig, axes = plt.subplots(2, 2, figsize=figsize)

        t_rtk = self.rtk_df['time_sync']
        t_loc = self.loc_df['time_sync']

        # X位置对比
        axes[0, 0].plot(t_rtk, self.rtk_df['x'], 'b-', linewidth=2, label='RTK X')
        axes[0, 0].plot(t_loc, self.loc_df['x'], 'r--', linewidth=2, label='定位 X')
        axes[0, 0].set_ylabel('X 位置 (m)', fontsize=11)
        axes[0, 0].set_title('X位置对比', fontsize=12, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()

        # Y位置对比
        axes[0, 1].plot(t_rtk, self.rtk_df['y'], 'b-', linewidth=2, label='RTK Y')
        axes[0, 1].plot(t_loc, self.loc_df['y'], 'r--', linewidth=2, label='定位 Y')
        axes[0, 1].set_ylabel('Y 位置 (m)', fontsize=11)
        axes[0, 1].set_title('Y位置对比', fontsize=12, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()

        # 航向角对比
        axes[1, 0].plot(t_rtk, self.rtk_df['RTK_heading'], 'b-', linewidth=2, label='RTK 航向角')
        axes[1, 0].plot(t_loc, np.degrees(self.loc_df['heading']), 'r--', linewidth=2, label='定位 航向角')
        axes[1, 0].set_ylabel('航向角 (°)', fontsize=11)
        axes[1, 0].set_title('航向角对比', fontsize=12, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()

        # 误差距离随时间
        axes[1, 1].plot(self.errors['time'], self.errors['error_distance'], 'g-', linewidth=2)
        axes[1, 1].fill_between(self.errors['time'], self.errors['error_distance'], 0, alpha=0.3, color='green')
        axes[1, 1].set_ylabel('误差距离 (m)', fontsize=11)
        axes[1, 1].set_title('定位误差随时间变化', fontsize=12, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)

        for ax in axes.flat:
            ax.set_xlabel('时间 (s)', fontsize=11)

        plt.tight_layout()
        return fig

    def save_comparison_report(self, output_dir='trajectory_comparison'):
        """保存对比报告和图表"""
        os.makedirs(output_dir, exist_ok=True)
        print(f"\n📊 生成对比图表...")

        # 计算误差
        if self.errors is None:
            self.calculate_errors()

        plots = [
            ('01_trajectory_overlay.png', self.plot_trajectory_overlay, '轨迹叠加'),
            ('02_error_map.png', self.plot_error_map, '误差热力图'),
            ('03_error_timeseries.png', self.plot_error_timeseries, '误差时间序列'),
            ('04_error_statistics.png', self.plot_error_statistics, '误差统计'),
            ('05_comparison_detail.png', self.plot_comparison_detail, '详细对比'),
        ]

        # 如果有原始RTK数据，添加三轨迹对比
        if self.rtk_raw_df is not None:
            plots.insert(0, ('00_three_trajectory_overlay.png', self.plot_trajectory_overlay_with_raw_rtk, '三轨迹叠加'))

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

        # 保存误差数据为CSV
        if self.errors is not None:
            error_csv = os.path.join(output_dir, 'error_analysis.csv')
            self.errors.to_csv(error_csv, index=False)
            print(f"  ✅ 误差数据: {error_csv}")

        # 保存对比报告
        report_file = os.path.join(output_dir, 'comparison_report.txt')
        self.save_report_text(report_file)
        print(f"  ✅ 对比报告: {report_file}")

        print(f"\n✨ 所有对比结果已保存到: {os.path.abspath(output_dir)}/")

    def save_report_text(self, filepath):
        """保存文本格式的对比报告"""
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("RTK参考轨迹(rtk_can.csv) vs EKF定位轨迹(vehicle_licalization.csv)对比报告\n")
            f.write("=" * 70 + "\n\n")

            f.write("1. 数据信息\n")
            f.write("-" * 70 + "\n")
            f.write(f"RTK数据文件: {self.rtk_converted_file}\n")
            f.write(f"RTK数据点数: {len(self.rtk_df)}\n")
            f.write(f"RTK时间跨度: {self.rtk_df['time_sync'].min():.2f}s ~ {self.rtk_df['time_sync'].max():.2f}s\n\n")

            f.write(f"定位数据文件: {self.localization_file}\n")
            f.write(f"定位数据点数: {len(self.loc_df)}\n")
            f.write(f"定位时间跨度: {self.loc_df['time_sync'].min():.2f}s ~ {self.loc_df['time_sync'].max():.2f}s\n\n")

            # 可选：原始RTK数据报告
            if self.rtk_raw_df is not None:
                f.write(f"原始RTK数据文件: {self.rtk_raw_file}\n")
                f.write(f"原始RTK数据点数: {len(self.rtk_raw_df)}\n")
                f.write(f"原始RTK时间跨度: {self.rtk_raw_df['timestamp'].min():.2f}s ~ {self.rtk_raw_df['timestamp'].max():.2f}s\n")
                # 检查是否有已转换的坐标列
                if 'X' in self.rtk_raw_df.columns and 'Y' in self.rtk_raw_df.columns:
                    f.write(f"原始RTK坐标范围: X[{self.rtk_raw_df['X'].min():.4f} ~ {self.rtk_raw_df['X'].max():.4f}] m, "
                            f"Y[{self.rtk_raw_df['Y'].min():.4f} ~ {self.rtk_raw_df['Y'].max():.4f}] m\n\n")
                else:
                    # 原始数据使用经纬度
                    f.write(f"原始RTK坐标范围: 纬度[{self.rtk_raw_df['RTK_x'].min():.8f}° ~ {self.rtk_raw_df['RTK_x'].max():.8f}°], "
                            f"经度[{self.rtk_raw_df['RTK_y'].min():.8f}° ~ {self.rtk_raw_df['RTK_y'].max():.8f}°]\n\n")

            f.write("2. RTK参考轨迹统计\n")
            f.write("-" * 70 + "\n")
            f.write(f"X范围: {self.rtk_df['x'].min():.4f} ~ {self.rtk_df['x'].max():.4f} m\n")
            f.write(f"Y范围: {self.rtk_df['y'].min():.4f} ~ {self.rtk_df['y'].max():.4f} m\n")
            rtk_dist = np.sum(np.sqrt(np.diff(self.rtk_df['x'])**2 +
                                      np.diff(self.rtk_df['y'])**2))
            f.write(f"总路程: {rtk_dist:.4f} m\n\n")

            f.write("3. 定位轨迹统计\n")
            f.write("-" * 70 + "\n")
            f.write(f"X范围: {self.loc_df['x'].min():.4f} ~ {self.loc_df['x'].max():.4f} m\n")
            f.write(f"Y范围: {self.loc_df['y'].min():.4f} ~ {self.loc_df['y'].max():.4f} m\n")
            loc_dist = np.sum(np.sqrt(np.diff(self.loc_df['x'])**2 +
                                      np.diff(self.loc_df['y'])**2))
            f.write(f"总路程: {loc_dist:.4f} m\n\n")

            f.write("4. 定位误差分析\n")
            f.write("-" * 70 + "\n")
            f.write(f"误差均值: {self.errors['error_distance'].mean():.6f} m\n")
            f.write(f"误差中位数: {self.errors['error_distance'].median():.6f} m\n")
            f.write(f"误差最大值: {self.errors['error_distance'].max():.6f} m\n")
            f.write(f"误差最小值: {self.errors['error_distance'].min():.6f} m\n")
            f.write(f"误差标准差: {self.errors['error_distance'].std():.6f} m\n")
            f.write(f"误差均方根(RMS): {np.sqrt(np.mean(self.errors['error_distance']**2)):.6f} m\n\n")

            f.write(f"X方向误差均值: {self.errors['error_x'].mean():.6f} m\n")
            f.write(f"X方向误差范围: [{self.errors['error_x'].min():.6f}, {self.errors['error_x'].max():.6f}] m\n")
            f.write(f"X方向误差标准差: {self.errors['error_x'].std():.6f} m\n\n")

            f.write(f"Y方向误差均值: {self.errors['error_y'].mean():.6f} m\n")
            f.write(f"Y方向误差范围: [{self.errors['error_y'].min():.6f}, {self.errors['error_y'].max():.6f}] m\n")
            f.write(f"Y方向误差标准差: {self.errors['error_y'].std():.6f} m\n\n")

            f.write("5. 定位性能评估\n")
            f.write("-" * 70 + "\n")

            # CEP (Circular Error Probable) - 95%误差圆
            error_95 = np.percentile(self.errors['error_distance'], 95)
            f.write(f"95%误差圆半径(CEP): {error_95:.6f} m\n")

            # 超过阈值的点数
            threshold_01 = np.sum(self.errors['error_distance'] > 0.1)
            threshold_05 = np.sum(self.errors['error_distance'] > 0.5)
            threshold_10 = np.sum(self.errors['error_distance'] > 1.0)

            f.write(f"超过0.1m的点数: {threshold_01} ({100*threshold_01/len(self.errors):.2f}%)\n")
            f.write(f"超过0.5m的点数: {threshold_05} ({100*threshold_05/len(self.errors):.2f}%)\n")
            f.write(f"超过1.0m的点数: {threshold_10} ({100*threshold_10/len(self.errors):.2f}%)\n\n")

            f.write("=" * 70 + "\n")


def main():
    """主函数 - RTK与EKF融合定位系统对比分析
    
    系统架构（与C代码同步）：
    
    1️⃣  轮速计处理 (src/wheel_odometry_processor.c):
        • 输入：后轮左右脉冲计数 + 时间戳
        • 处理：脉冲计数溢出检测 + 异常数据过滤
        • 输出：纵向速度、距离积分、运动方向
        
        关键参数：
        ├─ pulses_per_wheel_rev: 96 (每圈脉冲数)
        ├─ wheel_radius: 0.35 m
        ├─ max_count: 4096 (脉冲计数器上限，处理溢出)
        └─ wheel_circumference: 2.199 m (= 2πr)
    
    2️⃣  扩展卡尔曼滤波 (src/kalman_filter.c):
        • 观测量：轮速 + IMU加速度 + 转向角 (+ IMU角速度可选)
        • 状态量：位置(x,y) + 速度(vx,vy) + 航向角
        • 融合模式：
          ├─ BASIC: 3D观测（轮速+IMU+转向）
          └─ IMU_STEERING: 4D观测（+IMU角速度）
    
    3️⃣  转向映射 (src/steering_model.c):
        • 物理模型：转向角与轮速差的非线性关系
        • 支持多种特性曲线（线性、渐进式等）
        • 基于Ackermann模型
    
    4️⃣  坐标系定义 (ISO 8855车体坐标系):
        • X轴：纵向（前进为正）
        • Y轴：横向（左为正，右为负）
        • Z轴：竖直向上
        • 原点：后轮轴中心
    
    本脚本功能：
    ✓ 加载RTK参考数据和定位算法输出
    ✓ 对齐时间戳并计算定位误差
    ✓ 可视化轨迹对比（2D叠加图）
    ✓ 绘制误差热力图和向量场
    ✓ 生成详细的误差统计报告
    ✓ 验证脉冲计数与距离计算的一致性
    
    性能指标：
    • 定位精度：< 0.1m (95%CEP)
    • 航向精度：±1.8° (IMU融合模式)
    • 低速性能：> 0.3 m/s 可用
    """
    # 文件路径
    rtk_converted_file = '/Users/alex/workspace/code_review/vehicle_localization/rtk_converted.csv'
    localization_file = '/Users/alex/workspace/code_review/vehicle_localization/vehicle_localization.csv'
    rtk_raw_file = '/Users/alex/workspace/code_review/vehicle_localization/rtk_data.csv'
    output_dir = '/Users/alex/workspace/code_review/vehicle_localization/scripts/trajectory_comparison'

    # 检查文件
    if not os.path.exists(rtk_converted_file):
        print(f"❌ RTK转换文件不存在: {rtk_converted_file}")
        return
    if not os.path.exists(localization_file):
        print(f"❌ 定位文件不存在: {localization_file}")
        return

    # 创建对比对象
    print("🚗 轨迹叠加对比系统")
    print("=" * 70)
    print(f"RTK转换数据: rtk_converted.csv (ISO 8855车体坐标系)")
    print(f"定位数据: vehicle_localization.csv (卡尔曼滤波结果)")
    print("=" * 70)

    comparator = TrajectoryComparison(rtk_converted_file, localization_file, rtk_raw_file)

    # 生成对比报告
    comparator.save_comparison_report(output_dir)

    print(f"\n✨ 对比完成！")
    print(f"\n📁 输出目录: {output_dir}")

    # 显示图表
    print(f"\n🎨 显示对比图表...")
    try:
        plt.show()
    except Exception as e:
        print(f"⚠️  显示图表失败: {e}")
        print(f"   但图表已保存到文件")


if __name__ == '__main__':
    main()
