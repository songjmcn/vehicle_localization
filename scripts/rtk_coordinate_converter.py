#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
转换RTK坐标到定位系统坐标系统

功能：
  将rtk_can.csv中的GPS坐标（地理坐标）转换到vehicle_localization.csv的
  ISO 8855车体坐标系（以第一个点为原点）

输入：
  - rtk_can.csv: RTK原始数据（地理坐标：纬度、经度、航向角）
  - vehicle_localization.csv: 定位系统输出（参考坐标系）

输出：
  - rtk_converted.csv: 转换后的RTK坐标（与vehicle_localization.csv同坐标系）

坐标系说明：
  - RTK原始数据：WGS84地理坐标系（纬度、经度）
  - 目标坐标系：ISO 8855车体固定坐标系
    * 原点：第一个RTK数据点
    * X轴：纵向（前进方向为正）
    * Y轴：横向（左侧为正）
    * 参考点：后轮轴中心
"""

import csv
import math
import sys
from pathlib import Path

# 常数定义
EARTH_RADIUS = 6371000.0  # 地球半径（米）
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


class CoordinateConverter:
    """坐标系转换器"""
    
    def __init__(self):
        self.origin_lat = None
        self.origin_lon = None
        self.origin_heading = None
        self.initialized = False
    
    def set_origin(self, lat, lon, heading):
        """设置原点（以第一个RTK点为原点）"""
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_heading = heading
        self.initialized = True
        
        print(f"✓ 坐标系原点设置完成")
        print(f"  - 纬度: {lat:.8f}°")
        print(f"  - 经度: {lon:.8f}°")
        print(f"  - 航向角: {heading:.2f}°")
    
    def wgs84_to_local(self, lat, lon, heading):
        """
        将WGS84地理坐标转换为ISO 8855车体坐标系

        ISO 8855坐标系定义：
        - 原点：参考点（后轮轴中心）
        - X轴：车辆前进方向（前向为正）
        - Y轴：垂直于X轴，左侧为正
        - 参考方向：由origin_heading定义（相对北方的角度）

        转换步骤：
        1. 计算相对于原点的地理差值（经纬度差）
        2. 将地理差值转换为ENU坐标（东北天）
        3. 通过旋转矩阵将ENU坐标转换到车体坐标系

        Args:
            lat: 纬度 (度)
            lon: 经度 (度)
            heading: 航向角 (度)
        
        Returns:
            (x, y, heading_relative): 车体坐标系下的坐标和相对航向角
        """
        if not self.initialized:
            return None, None, None
        
        # Step 1: 计算地理差值
        dlat = (lat - self.origin_lat) * DEG_TO_RAD
        dlon = (lon - self.origin_lon) * DEG_TO_RAD
        
        # Step 2: 大地测量投影 - 将经纬度差转换为米
        # 纬度每度约111320米
        dy_enu = dlat * EARTH_RADIUS  # 北向距离（ENU坐标系中的北）

        # 经度每度的米数取决于纬度
        dx_enu = dlon * EARTH_RADIUS * math.cos(self.origin_lat * DEG_TO_RAD)  # 东向距离（ENU坐标系中的东）

        # Step 3: 从ENU坐标系转换到车体坐标系
        # 车辆的航向角origin_heading是相对于北方的角度（顺时针为正）
        # 需要将ENU坐标旋转到车体坐标系

        # 注意：在北天地坐标系中：
        # - 北 = +y方向
        # - 东 = +x方向
        # 而在车体坐标系中：
        # - 前 = +x方向
        # - 左 = +y方向

        # 旋转角度：从地理北转到车体前向
        # 如果heading=0°，车指向北；heading=90°，车指向东
        theta = self.origin_heading * DEG_TO_RAD

        # 旋转矩阵（从ENU到车体坐标）：
        # [x_body]   [sin(θ)   cos(θ) ] [x_enu (东)]
        # [y_body] = [cos(θ)  -sin(θ)] [y_enu (北)]

        # 推导：车体X轴指向航向角theta方向
        # 在ENU中，theta角对应的方向向量是(sin(θ), cos(θ))
        # 因此车体X轴在ENU中的分量是(sin(θ), cos(θ))
        # 车体Y轴垂直于X轴且指向左侧，对应(cos(θ), -sin(θ))

        x = dx_enu * math.sin(theta) + dy_enu * math.cos(theta)  # 车体前向
        y = dx_enu * math.cos(theta) + dy_enu * math.sin(theta)  # 车体左向（修正：与车辆本地化一致）

        # Step 4: RTK heading（地理北为0，逆时针为正，0~360）→ 车辆heading（车头为0，左正右负，-180~180）
        heading_vehicle = 90.0 - heading  # 车辆X轴为正东，Y轴为正北
        while heading_vehicle > 180:
            heading_vehicle -= 360
        while heading_vehicle <= -180:
            heading_vehicle += 360
        return x, y, heading_vehicle


def read_rtk_data(rtk_file):
    """读取rtk_can.csv文件"""
    data = []
    try:
        with open(rtk_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f, delimiter=',')
            for i, row in enumerate(reader):
                try:
                    entry = {
                        'index': i,
                        'lat': float(row['RTK_x']),  # 纬度
                        'lon': float(row['RTK_y']),  # 经度
                        'z': float(row['RTK_z']),    # 高度
                        'heading': float(row['RTK_heading']),  # 航向角（度）
                        'quality': int(row['RTK_qualit']),  # 质量指标
                        'timestamp': float(row['timestamp'])  # 时间戳
                    }
                    data.append(entry)
                except (ValueError, KeyError) as e:
                    print(f"⚠ 警告：第{i+2}行数据有误，跳过")
                    continue
    except FileNotFoundError:
        print(f"❌ 错误：文件 {rtk_file} 不存在")
        return None
    except Exception as e:
        print(f"❌ 错误：读取RTK文件失败 - {e}")
        return None
    
    print(f"✓ 成功读取 {len(data)} 条RTK数据")
    return data


def read_vehicle_localization_data(vl_file):
    """读取vehicle_localization.csv以了解时间戳对应关系"""
    data = []
    try:
        with open(vl_file, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f, delimiter=',')
            for i, row in enumerate(reader):
                try:
                    entry = {
                        'index': i,
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'heading': float(row['heading']),
                        'steering_angle': float(row['steering angle']),
                        'timestamp': int(row['timestamp'])
                    }
                    data.append(entry)
                except (ValueError, KeyError) as e:
                    continue
    except FileNotFoundError:
        print(f"⚠ 警告：文件 {vl_file} 不存在，将仅使用RTK数据")
        return None
    except Exception as e:
        print(f"⚠ 警告：读取vehicle_localization文件失败 - {e}")
        return None
    
    print(f"✓ 成功读取 {len(data)} 条定位系统数据（用于参考）")
    return data


def convert_rtk_data(rtk_data):
    """
    转换RTK数据到车体坐标系
    
    Args:
        rtk_data: RTK原始数据列表
    
    Returns:
        converted_data: 转换后的数据列表
    """
    if not rtk_data or len(rtk_data) == 0:
        print("❌ 错误：没有RTK数据可转换")
        return None
    
    converter = CoordinateConverter()
    
    # 使用第一条有效数据作为原点
    first_data = rtk_data[0]
    converter.set_origin(first_data['lat'], first_data['lon'], first_data['heading'])
    
    print("\n开始坐标转换...")
    print("=" * 70)
    
    converted_data = []
    for entry in rtk_data:
        x, y, heading = converter.wgs84_to_local(
            entry['lat'], entry['lon'], entry['heading']
        )
        
        if x is not None:
            converted_entry = {
                'x': x,
                'y': y,
                'heading': heading,
                'heading_rad': heading * DEG_TO_RAD,  # 将heading从度转化为弧度
                'z': entry['z'],
                'quality': entry['quality'],
                'timestamp': entry['timestamp']
            }
            converted_data.append(converted_entry)
    
    print(f"✓ 成功转换 {len(converted_data)} 条数据")
    return converted_data


def save_converted_data(converted_data, output_file):
    """保存转换后的数据到CSV文件"""
    try:
        with open(output_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(
                f,
                fieldnames=['x', 'y', 'heading', 'heading_rad', 'z', 'quality', 'timestamp'],
                delimiter=','
            )
            writer.writeheader()
            writer.writerows(converted_data)
        
        print(f"\n✓ 转换结果已保存到: {output_file}")
        return True
    except Exception as e:
        print(f"❌ 错误：保存文件失败 - {e}")
        return False


def print_statistics(rtk_data, converted_data):
    """打印转换统计信息"""
    if not converted_data:
        return
    
    print("\n" + "=" * 70)
    print("转换统计信息")
    print("=" * 70)
    
    # 统计坐标范围
    x_values = [d['x'] for d in converted_data]
    y_values = [d['y'] for d in converted_data]
    heading_values = [d['heading'] for d in converted_data]
    heading_rad_values = [d['heading_rad'] for d in converted_data]

    print(f"数据点数：{len(converted_data)}")
    print(f"\nX坐标范围：{min(x_values):.3f} ~ {max(x_values):.3f} 米")
    print(f"Y坐标范围：{min(y_values):.3f} ~ {max(y_values):.3f} 米")
    print(f"航向角范围（度）：{min(heading_values):.2f} ~ {max(heading_values):.2f} 度")
    print(f"航向角范围（弧度）：{min(heading_rad_values):.4f} ~ {max(heading_rad_values):.4f} 弧度")

    # 时间戳范围
    timestamps = [d['timestamp'] for d in converted_data]
    print(f"\n时间戳范围：{min(timestamps):.1f} ~ {max(timestamps):.1f}")
    print(f"时间跨度：{max(timestamps) - min(timestamps):.1f} 秒")
    
    # RTK质量统计
    qualities = [d['quality'] for d in converted_data]
    quality_dist = {}
    for q in qualities:
        quality_dist[q] = quality_dist.get(q, 0) + 1
    
    print(f"\nRTK质量分布：")
    for q in sorted(quality_dist.keys()):
        count = quality_dist[q]
        percentage = (count / len(converted_data)) * 100
        print(f"  质量等级 {q}: {count} 个 ({percentage:.1f}%)")


def main():
    """主函数"""
    print("=" * 70)
    print("RTK坐标转换工具")
    print("=" * 70)
    print("\n功能：将rtk_can.csv中的GPS坐标转换到vehicle_localization.csv的坐标系")
    print("坐标系：ISO 8855车体固定坐标系（原点为第一个RTK数据点）\n")
    
    # 路径设置
    script_dir = Path(__file__).parent
    project_dir = script_dir.parent
    
    rtk_file = project_dir / 'rtk_data.csv'
    vl_file = project_dir / 'vehicle_localization.csv'
    output_file = project_dir / 'rtk_converted.csv'
    
    # 读取数据
    print("读取输入文件...")
    rtk_data = read_rtk_data(str(rtk_file))
    if rtk_data is None:
        return 1
    
    vl_data = read_vehicle_localization_data(str(vl_file))
    
    # 转换数据
    print("\n转换坐标系...")
    converted_data = convert_rtk_data(rtk_data)
    if converted_data is None:
        return 1
    
    # 保存结果
    print("\n保存转换结果...")
    if not save_converted_data(converted_data, str(output_file)):
        return 1
    
    # 打印统计
    print_statistics(rtk_data, converted_data)
    
    print("\n" + "=" * 70)
    print("✓ 转换完成")
    print("=" * 70)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
