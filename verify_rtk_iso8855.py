#!/usr/bin/env python3
"""
ISO 8855坐标系验证脚本
验证RTK转换后的数据是否符合ISO 8855标准
"""

import csv
import math
import sys

def main():
    print("="*70)
    print("RTK坐标系验证：ISO 8855标准")
    print("="*70)
    
    # 读取RTK转换后的数据
    with open('rtk_converted.csv', 'r') as f:
        rtk_reader = csv.DictReader(f, delimiter=';')
        rtk_data = list(rtk_reader)
    
    print(f"\n读取RTK数据：{len(rtk_data)} 条记录")
    
    # 提取数据
    rtk_x = [float(r['x']) for r in rtk_data]
    rtk_y = [float(r['y']) for r in rtk_data]
    rtk_h = [float(r['heading']) * math.pi / 180 for r in rtk_data]  # 转换为弧度
    
    # 检查1：数据范围
    print(f"\n[检查1] 坐标范围")
    print(f"  X范围: {min(rtk_x):.3f} ~ {max(rtk_x):.3f} m")
    print(f"  Y范围: {min(rtk_y):.3f} ~ {max(rtk_y):.3f} m")
    print(f"  heading范围: {min(rtk_h)*180/math.pi:.3f}° ~ {max(rtk_h)*180/math.pi:.3f}°")
    
    # 检查2：数据连续性
    print(f"\n[检查2] 轨迹连续性")
    large_jumps = []
    for i in range(1, len(rtk_data)):
        dx = rtk_x[i] - rtk_x[i-1]
        dy = rtk_y[i] - rtk_y[i-1]
        dist = math.sqrt(dx**2 + dy**2)
        if dist > 0.1:  # 超过10cm认为是跳跃
            large_jumps.append((i, dist))
    
    if large_jumps:
        print(f"  发现 {len(large_jumps)} 个大跳跃（>0.1m）")
        for i, dist in large_jumps[:5]:
            print(f"    [{i}] 距离 = {dist:.4f} m")
        if len(large_jumps) > 5:
            print(f"    ... 还有 {len(large_jumps)-5} 个")
    else:
        print(f"  ✓ 轨迹连续，无跳跃")
    
    # 检查3：ISO 8855运动模型
    print(f"\n[检查3] ISO 8855运动模型验证（dy/dx = tan(heading)）")
    
    # 计算总体轨迹
    x_start, x_end = rtk_x[0], rtk_x[-1]
    y_start, y_end = rtk_y[0], rtk_y[-1]
    h_avg = sum(rtk_h) / len(rtk_h)
    
    dx_total = x_end - x_start
    dy_total = y_end - y_start
    
    if abs(dx_total) > 0.01:
        dy_dx_actual = dy_total / dx_total
        dy_dx_expected = math.tan(h_avg)
        
        print(f"  总体位移：Δx={dx_total:.3f}m, Δy={dy_total:.3f}m")
        print(f"  实际斜率 dy/dx = {dy_dx_actual:.6f}")
        print(f"  期望斜率 tan(heading) = {dy_dx_expected:.6f}")
        print(f"  相对误差 = {abs(dy_dx_actual - dy_dx_expected)/max(abs(dy_dx_expected), 0.001)*100:.1f}%")
        
        if abs(dy_dx_actual - dy_dx_expected) < 0.05:
            print(f"  ✓ 符合ISO 8855运动模型")
        else:
            print(f"  ✗ 不符合ISO 8855运动模型（误差较大）")
    
    # 检查4：heading连续性
    print(f"\n[检查4] heading连续性")
    heading_jumps = []
    for i in range(1, len(rtk_data)):
        dh = rtk_h[i] - rtk_h[i-1]
        # 规范化到[-π, π]
        while dh > math.pi:
            dh -= 2*math.pi
        while dh < -math.pi:
            dh += 2*math.pi
        if abs(dh) > 0.1:  # 超过5.7度
            heading_jumps.append((i, dh*180/math.pi))
    
    if heading_jumps:
        print(f"  发现 {len(heading_jumps)} 个heading大跳跃（>5.7°）")
        for i, dh in heading_jumps[:5]:
            print(f"    [{i}] Δheading = {dh:.2f}°")
    else:
        print(f"  ✓ heading连续，无跳跃")
    
    # 检查5：heading与Y坐标的一致性
    print(f"\n[检查5] heading与Y坐标的一致性")
    
    # 计算分段
    segment_size = max(1, len(rtk_data) // 5)
    consistent_count = 0
    
    for seg_idx in range(5):
        start_idx = seg_idx * segment_size
        end_idx = min((seg_idx+1) * segment_size, len(rtk_data))
        
        if end_idx - start_idx > 2:
            y_delta = rtk_y[end_idx-1] - rtk_y[start_idx]
            h_avg_seg = sum(rtk_h[start_idx:end_idx]) / (end_idx - start_idx)
            
            # 在该分段内，Y应该在heading方向
            # heading > 0 意味着右转（应该产生正的Δy）
            # heading < 0 意味着左转（应该产生负的Δy）
            
            sign_consistent = (y_delta >= 0 and h_avg_seg >= 0) or (y_delta <= 0 and h_avg_seg <= 0)
            if sign_consistent:
                consistent_count += 1
            
            status = "✓" if sign_consistent else "✗"
            print(f"  [{status}] 分段{seg_idx}: Δy={y_delta:+.4f}m, h_avg={h_avg_seg*180/math.pi:+.2f}°")
    
    print(f"  一致性: {consistent_count}/5分段")
    
    # 总体结论
    print(f"\n" + "="*70)
    print(f"总体验证结果")
    print(f"="*70)
    
    if (abs(dy_dx_actual - dy_dx_expected) < 0.05 and 
        not large_jumps and 
        consistent_count >= 4):
        print(f"✓ RTK数据符合ISO 8855标准")
        print(f"  - 坐标系转换正确")
        print(f"  - heading定义正确（逆时针为正）")
        print(f"  - 运动模型一致")
        return 0
    else:
        print(f"✗ RTK数据仍有问题")
        if abs(dy_dx_actual - dy_dx_expected) >= 0.05:
            print(f"  - 运动模型不一致（dy/dx误差超过5%）")
        if large_jumps:
            print(f"  - 轨迹有跳跃")
        if consistent_count < 4:
            print(f"  - heading与Y坐标不一致")
        return 1

if __name__ == '__main__':
    sys.exit(main())
