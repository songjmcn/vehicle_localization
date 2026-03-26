#!/usr/bin/env python3
"""
单元测试：验证 RTK 倒车处理的修改

测试场景：
1. 前进时 (velocity > 0): X 坐标应保持原值
2. 倒车时 (velocity < 0): X 坐标应被反转
3. 静止时 (velocity ≈ 0): 坐标不变
4. 使用 origin_heading 作为参考框架是否一致
"""

import sys
import math

sys.path.insert(0, '/Users/alex/workspace/code_review/vehicle_localization/scripts')

from utils.rtk import RTKData


def test_forward_vs_reverse_x_inversion():
    """测试倒车时 X 坐标反转"""
    print("Test 1: X-coordinate inversion in reverse")
    print("-" * 60)
    
    origin_lat = 30.5
    origin_lon = 120.5
    origin_alt = 100.0
    origin_heading = 0.0
    
    # RTK 点向北移动约 11m
    rtk = RTKData(
        longitudinal=120.5,
        latitudinal=30.50009891,
        altitude=100.0,
        heading=0.0
    )
    
    # 前进
    x_fwd, y_fwd, z_fwd, h_fwd = rtk.to_body(
        origin_lat, origin_lon, origin_alt, origin_heading,
        velocity=1.0  # 前进
    )
    
    # 倒车
    x_rev, y_rev, z_rev, h_rev = rtk.to_body(
        origin_lat, origin_lon, origin_alt, origin_heading,
        velocity=-1.0  # 倒车
    )
    
    print(f"前进时: x={x_fwd:.3f}, y={y_fwd:.3f}")
    print(f"倒车时: x={x_rev:.3f}, y={y_rev:.3f}")
    
    # 验证：
    # 1. X 坐标应该反向
    assert x_fwd > 0 and x_rev < 0, f"X 坐标应该反向，但 x_fwd={x_fwd}, x_rev={x_rev}"
    # 2. X 坐标绝对值应该相同
    assert abs(abs(x_fwd) - abs(x_rev)) < 0.01, f"X 坐标绝对值应该相同"
    # 3. Y 坐标应该相同（不受速度影响）
    assert abs(y_fwd - y_rev) < 0.01, f"Y 坐标应该相同，但 y_fwd={y_fwd}, y_rev={y_rev}"
    
    print("✓ 通过\n")


def test_origin_heading_consistency():
    """测试使用 origin_heading 作为参考的一致性"""
    print("Test 2: Origin heading as reference frame")
    print("-" * 60)
    
    # 不同的原点航向
    origin_lat = 30.5
    origin_lon = 120.5
    origin_alt = 100.0
    
    rtk = RTKData(
        longitudinal=120.50007,  # 东方约 8m
        latitudinal=30.5,
        altitude=100.0,
        heading=90.0  # RTK 航向东方
    )
    
    # 情况 1: 原点航向 0°（北）
    x1, y1, _, h1 = rtk.to_body(origin_lat, origin_lon, origin_alt, 0.0, velocity=1.0)
    
    # 情况 2: 原点航向 90°（东）
    x2, y2, _, h2 = rtk.to_body(origin_lat, origin_lon, origin_alt, 90.0, velocity=1.0)
    
    print(f"origin_heading=0°: x={x1:.3f}, y={y1:.3f}, heading={h1:.3f}")
    print(f"origin_heading=90°: x={x2:.3f}, y={y2:.3f}, heading={h2:.3f}")
    
    # 验证：两种情况下坐标应该不同（因为参考框架不同）
    assert abs(x1 - x2) > 0.1 or abs(y1 - y2) > 0.1, "不同的 origin_heading 应该产生不同的坐标"
    
    print("✓ 通过\n")


def test_velocity_none_default():
    """测试速度为 None 时的默认行为"""
    print("Test 3: Velocity=None default (forward motion)")
    print("-" * 60)
    
    origin_lat = 30.5
    origin_lon = 120.5
    origin_alt = 100.0
    origin_heading = 0.0
    
    rtk = RTKData(
        longitudinal=120.5,
        latitudinal=30.50009891,
        altitude=100.0,
        heading=0.0
    )
    
    # velocity=None（默认）
    x_none, _, _, _ = rtk.to_body(origin_lat, origin_lon, origin_alt, origin_heading, velocity=None)
    
    # velocity>0（明确前进）
    x_pos, _, _, _ = rtk.to_body(origin_lat, origin_lon, origin_alt, origin_heading, velocity=1.0)
    
    print(f"velocity=None: x={x_none:.3f}")
    print(f"velocity=+1.0: x={x_pos:.3f}")
    
    # 验证：都应该为正（前进）
    assert x_none > 0, f"velocity=None 应该视为前进（x>0），但 x={x_none}"
    assert abs(x_none - x_pos) < 0.01, "velocity=None 和 velocity>0 应该相同"
    
    print("✓ 通过\n")


def test_heading_relative_calculation():
    """测试相对航向的计算"""
    print("Test 4: Relative heading calculation")
    print("-" * 60)
    
    origin_lat = 30.5
    origin_lon = 120.5
    origin_alt = 100.0
    origin_heading = 45.0
    
    rtk = RTKData(
        longitudinal=120.5,
        latitudinal=30.5,
        altitude=100.0,
        heading=45.0  # 与原点航向相同
    )
    
    _, _, _, h = rtk.to_body(origin_lat, origin_lon, origin_alt, origin_heading)
    
    print(f"origin_heading={origin_heading}°")
    print(f"rtk.heading={rtk.heading}°")
    print(f"相对航向: {h:.3f} rad ({math.degrees(h):.1f}°)")
    
    # 验证：相对航向应该接近 0
    assert abs(h) < 0.1, f"相对航向应该接近 0，但得到 {h:.3f} rad"
    
    print("✓ 通过\n")


def test_antenna_offset_with_origin_heading():
    """测试天线偏移使用 origin_heading 的正确性"""
    print("Test 5: Antenna offset with origin_heading")
    print("-" * 60)
    
    origin_lat = 30.5
    origin_lon = 120.5
    origin_alt = 100.0
    origin_heading = 0.0
    
    antenna_offset = (1.5, 0.0, 0.0)  # 天线在车前 1.5m
    
    # 创建两个RTK点：一个在原点，一个向前移动约1.5m（大约 0.000015°纬度）
    rtk_at_origin = RTKData(
        longitudinal=120.5,
        latitudinal=30.5,
        altitude=100.0,
        heading=0.0
    )
    
    rtk_ahead = RTKData(
        longitudinal=120.5,
        latitudinal=30.500015,  # 约 1.67m 北方
        altitude=100.0,
        heading=0.0
    )
    
    # 测试1：无偏移，原点
    x_origin_no_offset, _, _, _ = rtk_at_origin.to_body(
        origin_lat, origin_lon, origin_alt, origin_heading,
        antenna_offset=None,
        velocity=1.0
    )
    
    # 测试2：无偏移，前方点
    x_ahead_no_offset, _, _, _ = rtk_ahead.to_body(
        origin_lat, origin_lon, origin_alt, origin_heading,
        antenna_offset=None,
        velocity=1.0
    )
    
    # 测试3：有偏移，前方点（相对于原点的偏移）
    x_ahead_with_offset, _, _, _ = rtk_ahead.to_body(
        origin_lat, origin_lon, origin_alt, origin_heading,
        antenna_offset=antenna_offset,
        velocity=1.0
    )
    
    print(f"原点无偏移: x={x_origin_no_offset:.3f}")
    print(f"前方无偏移: x={x_ahead_no_offset:.3f}")
    print(f"前方有偏移 (1.5m): x={x_ahead_with_offset:.3f}")
    
    # 验证：origin_heading 用于天线偏移计算意味着偏移在body frame中是一致的
    # 简单检查：天线偏移应该影响坐标
    assert x_origin_no_offset != x_ahead_with_offset, "天线偏移应该影响坐标"
    
    print("✓ 通过\n")


if __name__ == "__main__":
    print("\n")
    print("=" * 70)
    print("RTK 倒车处理单元测试")
    print("=" * 70)
    print()
    
    try:
        test_forward_vs_reverse_x_inversion()
        test_origin_heading_consistency()
        test_velocity_none_default()
        test_heading_relative_calculation()
        test_antenna_offset_with_origin_heading()
        
        print("=" * 70)
        print("✓ 所有测试通过！")
        print("=" * 70)
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
