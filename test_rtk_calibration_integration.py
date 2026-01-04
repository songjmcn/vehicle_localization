#!/usr/bin/env python3
"""
RTK校准集成测试 - 使用实际CSV数据验证Phase 1增强版

测试目标：
1. 从CSV加载真实的ODO和RTK数据
2. 运行改进的校准算法
3. 验证结果的合理性
4. 生成诊断报告
"""

import csv
import sys
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class Point:
    """数据点"""
    time: float
    odo_x: float
    odo_y: float
    odo_heading: float
    odo_v: float
    odo_steering: float
    rtk_x: float
    rtk_y: float
    rtk_heading: float
    rtk_quality: int

class RTKCalibrationSimulator:
    """RTK校准模拟器（Python版本）"""
    
    def __init__(self):
        self.points: List[Point] = []
        self.outliers = []
        self.rejected = []
        
    def add_point(self, point: Point, validate: bool = True) -> bool:
        """添加标定点"""
        if validate and len(self.points) > 0:
            prev = self.points[-1]
            
            # 验证1：速度加速度
            dv = abs(point.odo_v - prev.odo_v)
            max_dv = 5.0 * 0.1  # 5 m/s²
            if dv > max_dv:
                self.rejected.append(('accel', len(self.points), dv))
                return False
            
            # 验证2：转向角变化
            steering_change = abs(point.odo_steering - prev.odo_steering)
            if steering_change > 1.0:
                self.rejected.append(('steering', len(self.points), steering_change))
                return False
            
            # 验证3：轨迹连续性
            rtk_jump = math.sqrt((point.rtk_x - prev.rtk_x)**2 + 
                                (point.rtk_y - prev.rtk_y)**2)
            if rtk_jump > 10.0:
                self.rejected.append(('rtk_jump', len(self.points), rtk_jump))
                return False
            
            odo_jump = math.sqrt((point.odo_x - prev.odo_x)**2 + 
                                (point.odo_y - prev.odo_y)**2)
            if odo_jump > 10.0:
                self.rejected.append(('odo_jump', len(self.points), odo_jump))
                return False
            
            # 验证4：最小轨迹长度
            min_length = 0.01
            if odo_jump < min_length and rtk_jump < min_length:
                self.rejected.append(('no_movement', len(self.points), 0.0))
                return False
        
        # 质量检查
        if point.rtk_quality < 2:
            self.rejected.append(('quality', len(self.points), point.rtk_quality))
            return False
        
        self.points.append(point)
        return True
    
    def compute_calibration(self) -> dict:
        """计算校准参数"""
        if len(self.points) < 2:
            return {'wheel_factor': 1.0, 'outliers': 0, 'valid': False}
        
        # 计算轨迹段的距离
        segments = []
        for i in range(len(self.points) - 1):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            
            odo_dist = math.sqrt((p2.odo_x - p1.odo_x)**2 + 
                                (p2.odo_y - p1.odo_y)**2)
            rtk_dist = math.sqrt((p2.rtk_x - p1.rtk_x)**2 + 
                                (p2.rtk_y - p1.rtk_y)**2)
            
            segments.append({
                'odo_dist': odo_dist,
                'rtk_dist': rtk_dist,
                'quality': p2.rtk_quality,
                'ratio': odo_dist / rtk_dist if rtk_dist > 0.01 else 1.0
            })
        
        # 计算初始估计（简单均值）
        if len(segments) == 0:
            return {'wheel_factor': 1.0, 'outliers': 0, 'valid': False}
        
        sum_odo = sum(s['odo_dist'] for s in segments)
        sum_rtk = sum(s['rtk_dist'] for s in segments)
        initial_factor = sum_odo / sum_rtk if sum_rtk > 0 else 1.0
        
        # 检测离群点（MAD方法）
        residuals = [s['odo_dist'] - initial_factor * s['rtk_dist'] 
                    for s in segments]
        
        sorted_res = sorted(residuals)
        n = len(sorted_res)
        median = sorted_res[n//2] if n % 2 == 1 else (sorted_res[n//2-1] + sorted_res[n//2]) / 2
        
        deviations = sorted([abs(r - median) for r in residuals])
        mad = deviations[n//2] if n % 2 == 1 else (deviations[n//2-1] + deviations[n//2]) / 2
        
        threshold = 2.5 * mad if mad > 1e-6 else 2.0
        threshold = max(threshold, 2.0)
        
        is_outlier = [abs(residuals[i] - median) > threshold for i in range(len(residuals))]
        outlier_count = sum(is_outlier)
        
        # 加权最小二乘法
        max_quality = max(s['quality'] for s in segments)
        
        numerator = 0.0
        denominator = 0.0
        
        for i, seg in enumerate(segments):
            if not is_outlier[i]:
                q_norm = min(seg['quality'] / max_quality, 1.0)
                q_weight = q_norm * q_norm
                t_weight = 1.0 + math.log(1.0 + seg['rtk_dist'])
                u_factor = 1.0 / (q_weight + 0.1)
                weight = q_weight * t_weight / u_factor
                
                numerator += weight * seg['odo_dist'] * seg['rtk_dist']
                denominator += weight * seg['rtk_dist'] * seg['rtk_dist']
        
        if denominator < 1e-6:
            return {'wheel_factor': 1.0, 'outliers': outlier_count, 'valid': False}
        
        wheel_factor = numerator / denominator
        wheel_factor = max(0.8, min(1.2, wheel_factor))
        
        # 计算R²
        mean_odo = sum(s['odo_dist'] for i, s in enumerate(segments) 
                      if not is_outlier[i]) / (len(segments) - outlier_count) if len(segments) > outlier_count else 0
        
        ss_res = sum((s['odo_dist'] - wheel_factor * s['rtk_dist'])**2 
                    for i, s in enumerate(segments) if not is_outlier[i])
        ss_tot = sum((s['odo_dist'] - mean_odo)**2 
                    for i, s in enumerate(segments) if not is_outlier[i])
        
        r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 1e-6 else 0.0
        r_squared = max(0.0, min(1.0, r_squared))
        
        return {
            'wheel_factor': wheel_factor,
            'outliers': outlier_count,
            'total_segments': len(segments),
            'r_squared': r_squared,
            'valid': outlier_count < len(segments) * 0.3  # 少于30%离群点
        }

def load_csv_data(filename: str, max_points: Optional[int] = None) -> List[Point]:
    """从CSV文件加载数据"""
    points = []
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for i, row in enumerate(reader):
                if max_points and i >= max_points:
                    break
                
                try:
                    point = Point(
                        time=float(row.get('timestamp', row.get('time', i))),
                        odo_x=float(row.get('odo_x', row.get('x', 0))),
                        odo_y=float(row.get('odo_y', row.get('y', 0))),
                        odo_heading=float(row.get('odo_heading', row.get('heading', 0))),
                        odo_v=float(row.get('odo_v', row.get('v', row.get('velocity', 0)))),
                        odo_steering=float(row.get('steering_angle', row.get('steering', 0))),
                        rtk_x=float(row.get('rtk_x', row.get('gps_x', row.get('lat', 0)))),
                        rtk_y=float(row.get('rtk_y', row.get('gps_y', row.get('lon', 0)))),
                        rtk_heading=float(row.get('rtk_heading', row.get('gps_heading', 0))),
                        rtk_quality=int(row.get('rtk_quality', row.get('quality', 4)))
                    )
                    points.append(point)
                except (ValueError, KeyError) as e:
                    print(f"⚠️ 跳过第{i}行: {e}")
                    continue
        
        return points
    except Exception as e:
        print(f"❌ 无法加载文件 {filename}: {e}")
        return []

def test_real_data(csv_file: str, max_points: int = 500):
    """使用真实CSV数据进行测试"""
    print(f"\n{'='*60}")
    print(f"RTK校准集成测试 - {csv_file}")
    print(f"{'='*60}\n")
    
    # 加载数据
    print(f"📂 加载CSV数据: {csv_file}")
    points = load_csv_data(csv_file, max_points)
    
    if not points:
        print("❌ 无法加载数据")
        return False
    
    print(f"✓ 成功加载 {len(points)} 个数据点\n")
    
    # 创建校准器
    simulator = RTKCalibrationSimulator()
    
    # 添加数据点
    print("📊 验证并添加数据点...")
    added = 0
    for i, point in enumerate(points):
        if simulator.add_point(point, validate=(i > 0)):
            added += 1
    
    print(f"✓ 已添加 {added} 个有效点")
    print(f"⚠️  拒绝 {len(simulator.rejected)} 个点")
    
    if simulator.rejected:
        reject_types = {}
        for rtype, idx, val in simulator.rejected:
            reject_types[rtype] = reject_types.get(rtype, 0) + 1
        print(f"   原因统计: {reject_types}\n")
    
    # 计算校准参数
    print("🔬 计算校准参数...")
    result = simulator.compute_calibration()
    
    print(f"✓ 校准完成\n")
    
    # 打印结果
    print("【校准结果】")
    print(f"  轮速计校准因子: {result['wheel_factor']:.6f}")
    print(f"  拟合优度 (R²): {result['r_squared']:.6f}")
    print(f"  总轨迹段数: {result.get('total_segments', 0)}")
    print(f"  检测离群点: {result['outliers']}")
    print(f"  校准有效: {'✅ 是' if result['valid'] else '❌ 否'}")
    
    # 数据统计
    if simulator.points:
        qualities = [p.rtk_quality for p in simulator.points]
        print(f"\n【质量统计】")
        print(f"  质量等级分布:")
        for q in range(6):
            count = sum(1 for qv in qualities if qv == q)
            if count > 0:
                print(f"    等级{q}: {count}个 ({100*count/len(qualities):.1f}%)")
    
    print(f"\n{'='*60}\n")
    return result['valid']

if __name__ == '__main__':
    # 测试可用的CSV文件
    test_files = [
        'rtk_trajectory_processed.csv',
        'rtk_converted.csv',
        'calibration_test.csv',
        'diagnostic_rtk.csv'
    ]
    
    print("\n" + "="*60)
    print("Phase 1 (P0+) RTK校准集成测试")
    print("="*60)
    
    results = {}
    for csv_file in test_files:
        try:
            valid = test_real_data(csv_file, max_points=500)
            results[csv_file] = valid
        except Exception as e:
            print(f"❌ {csv_file}: {e}\n")
            results[csv_file] = False
    
    # 总结
    print("\n" + "="*60)
    print("测试总结")
    print("="*60)
    for csv_file, valid in results.items():
        status = "✅ 通过" if valid else "⚠️ 需关注"
        print(f"  {csv_file}: {status}")
    
    print("="*60 + "\n")
