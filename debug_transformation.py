#!/usr/bin/env python3
"""
诊断RTK坐标转换问题
"""
import csv
import math

EARTH_RADIUS = 111320  # 地球半径（米）
DEG_TO_RAD = math.pi / 180

# 读取RTK原始数据前几条
print("="*70)
print("RTK原始数据分析")
print("="*70)

with open('rtk_can.csv', 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    rtk_data = []
    for i, row in enumerate(reader):
        if i >= 10:  # 只读前10行
            break
        rtk_data.append({
            'lat': float(row['RTK_x']),
            'lon': float(row['RTK_y']),
            'heading': float(row['RTK_heading']),
        })

print(f"\nRTK原点：")
print(f"  lat = {rtk_data[0]['lat']:.8f}°")
print(f"  lon = {rtk_data[0]['lon']:.8f}°")
print(f"  heading = {rtk_data[0]['heading']:.2f}°")

print(f"\n前5条RTK记录：")
for i, data in enumerate(rtk_data[:5]):
    print(f"  [{i}] lat={data['lat']:.8f}, lon={data['lon']:.8f}, heading={data['heading']:.2f}")

# 现在手动计算第1条记录的转换
print("\n" + "="*70)
print("手动转换计算 - 第1条记录")
print("="*70)

origin_lat = rtk_data[0]['lat']
origin_lon = rtk_data[0]['lon']
origin_heading = rtk_data[0]['heading']  # 123.69°

point_lat = rtk_data[1]['lat']
point_lon = rtk_data[1]['lon']
point_heading = rtk_data[1]['heading']

print(f"\n原点设置：")
print(f"  origin_lat = {origin_lat:.8f}°")
print(f"  origin_lon = {origin_lon:.8f}°")
print(f"  origin_heading = {origin_heading:.2f}°")

print(f"\n待转换点：")
print(f"  point_lat = {point_lat:.8f}°")
print(f"  point_lon = {point_lon:.8f}°")
print(f"  point_heading = {point_heading:.2f}°")

# Step 1: 计算地理差值
dlat = (point_lat - origin_lat) * DEG_TO_RAD
dlon = (point_lon - origin_lon) * DEG_TO_RAD

print(f"\nStep 1: 地理差值")
print(f"  dlat = ({point_lat} - {origin_lat}) * π/180 = {dlat:.10f} rad = {dlat * 180 / math.pi:.8f}°")
print(f"  dlon = ({point_lon} - {origin_lon}) * π/180 = {dlon:.10f} rad = {dlon * 180 / math.pi:.8f}°")

# Step 2: ENU坐标计算
dy_enu = dlat * EARTH_RADIUS
dx_enu = dlon * EARTH_RADIUS * math.cos(origin_lat * DEG_TO_RAD)

print(f"\nStep 2: ENU坐标（地球中心坐标）")
print(f"  dy_enu (北向) = {dlat:.10f} rad * {EARTH_RADIUS} m/rad = {dy_enu:.10f} m")
print(f"  dx_enu (东向) = {dlon:.10f} rad * {EARTH_RADIUS} * cos({origin_lat:.8f}°)")
print(f"               = {dlon:.10f} * {EARTH_RADIUS} * {math.cos(origin_lat * DEG_TO_RAD):.8f}")
print(f"               = {dx_enu:.10f} m")

# Step 3: 旋转变换
theta = -origin_heading * DEG_TO_RAD
print(f"\nStep 3: 旋转变换")
print(f"  theta = -origin_heading * π/180")
print(f"        = -{origin_heading:.2f}° * π/180")
print(f"        = {theta:.10f} rad = {theta * 180 / math.pi:.8f}°")
print(f"  sin(theta) = {math.sin(theta):.10f}")
print(f"  cos(theta) = {math.cos(theta):.10f}")

x_body = dx_enu * math.sin(theta) + dy_enu * math.cos(theta)
y_body = -dx_enu * math.cos(theta) + dy_enu * math.sin(theta)

print(f"\nStep 3a: 车体X（前向）")
print(f"  x_body = dx_enu * sin(theta) + dy_enu * cos(theta)")
print(f"         = {dx_enu:.10f} * {math.sin(theta):.10f} + {dy_enu:.10f} * {math.cos(theta):.10f}")
print(f"         = {dx_enu * math.sin(theta):.10f} + {dy_enu * math.cos(theta):.10f}")
print(f"         = {x_body:.10f} m")

print(f"\nStep 3b: 车体Y（左向）")
print(f"  y_body = -dx_enu * cos(theta) + dy_enu * sin(theta)")
print(f"         = -{dx_enu:.10f} * {math.cos(theta):.10f} + {dy_enu:.10f} * {math.sin(theta):.10f}")
print(f"         = {-dx_enu * math.cos(theta):.10f} + {dy_enu * math.sin(theta):.10f}")
print(f"         = {y_body:.10f} m")

# Step 4: 相对heading
heading_relative = point_heading - origin_heading
while heading_relative > 180:
    heading_relative -= 360
while heading_relative < -180:
    heading_relative += 360

print(f"\nStep 4: 相对Heading")
print(f"  heading_relative = {point_heading:.2f}° - {origin_heading:.2f}° = {heading_relative:.2f}°")

print(f"\n最终转换结果：")
print(f"  x = {x_body:.10f} m")
print(f"  y = {y_body:.10f} m")
print(f"  heading = {heading_relative:.2f}°")

# 现在读取实际转换文件
print("\n" + "="*70)
print("实际转换输出文件验证")
print("="*70)

with open('rtk_converted.csv', 'r') as f:
    reader = csv.DictReader(f, delimiter=';')
    actual = []
    for i, row in enumerate(reader):
        if i >= 2:  # 只读前2条
            break
        actual.append({
            'x': float(row['x']),
            'y': float(row['y']),
            'heading': float(row['heading']),
        })

print(f"\n实际转换文件中的前2条：")
for i, data in enumerate(actual):
    print(f"  [{i}] x={data['x']:.10f}, y={data['y']:.10f}, heading={data['heading']:.10f}")

print(f"\n对比：")
print(f"  手动计算 [1] vs 文件 [1]:")
if len(actual) > 1:
    print(f"    x: {x_body:.10f} vs {actual[1]['x']:.10f} - {'✓ 匹配' if abs(x_body - actual[1]['x']) < 1e-8 else '✗ 不匹配！'}")
    print(f"    y: {y_body:.10f} vs {actual[1]['y']:.10f} - {'✓ 匹配' if abs(y_body - actual[1]['y']) < 1e-8 else '✗ 不匹配！'}")

# 关键诊断：检查坐标系是否正确
print("\n" + "="*70)
print("关键诊断：坐标系检查")
print("="*70)

print(f"\n分析：")
print(f"1. dx_enu = {dx_enu:.10f} m (GPS向东移动)")
print(f"2. dy_enu = {dy_enu:.10f} m (GPS向北移动)")
print(f"3. origin_heading = {origin_heading:.2f}° (车面向东偏南)")
print(f"4. 旋转角 theta = {theta * 180 / math.pi:.2f}°")

print(f"\n当GPS向东(+dx_enu)移动时，在ISO 8855中应该如何解释？")
print(f"- 车面向东偏南(123.69°)")
print(f"- GPS东向移动应该...等等")

print(f"\n实际问题分析：")
print(f"- RTK输出 X = {x_body:.10f}m (非常小）")
print(f"- RTK输出 Y = {y_body:.10f}m (非常小)")
print(f"- 车物理距离应该是 sqrt(dx_enu² + dy_enu²) = {math.sqrt(dx_enu**2 + dy_enu**2):.10f}m")
print(f"- RTK计算距离 = sqrt(x_body² + y_body²) = {math.sqrt(x_body**2 + y_body**2):.10f}m")
print(f"- 距离是否保持？{'✓ 是' if abs(math.sqrt(dx_enu**2 + dy_enu**2) - math.sqrt(x_body**2 + y_body**2)) < 1e-8 else '✗ 否！'}")

print(f"\n向量旋转检查：")
angle_before = math.atan2(dy_enu, dx_enu) * 180 / math.pi
angle_after = math.atan2(y_body, x_body) * 180 / math.pi
print(f"- ENU向量角度 = {angle_before:.2f}° (相对北方)")
print(f"- 车体向量角度 = {angle_after:.2f}° (相对车体前向)")
print(f"- 旋转量 = {angle_before - angle_after:.2f}°")
print(f"- 预期旋转量(θ) = {theta * 180 / math.pi:.2f}°")
