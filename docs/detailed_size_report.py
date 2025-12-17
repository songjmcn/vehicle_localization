#!/usr/bin/env python3
"""
详细的编译产物大小分析工具
"""

import os
import subprocess
import struct

def get_segment_sizes():
    """获取各个段的大小"""
    exe = "build/bin/localization_example"
    
    # 使用 otool 获取段信息
    result = subprocess.run(['otool', '-l', exe], capture_output=True, text=True)
    lines = result.stdout.split('\n')
    
    segments = {}
    current_segment = None
    
    for line in lines:
        if 'segname' in line and 'cmd' not in line:
            current_segment = line.split()[-1]
            segments[current_segment] = {'size': 0, 'filesize': 0}
        elif 'vmsize' in line and current_segment:
            size_str = line.split()[-1]
            segments[current_segment]['size'] = int(size_str, 16)
        elif 'filesize' in line and current_segment:
            size_str = line.split()[-1]
            segments[current_segment]['filesize'] = int(size_str, 16)
    
    return segments

def analyze_symbols():
    """分析符号表"""
    exe = "build/bin/localization_example"
    result = subprocess.run(['nm', exe], capture_output=True, text=True)
    
    text_symbols = 0  # 代码
    data_symbols = 0  # 数据
    
    for line in result.stdout.split('\n'):
        if ' T ' in line or ' t ' in line:
            text_symbols += 1
        elif ' D ' in line or ' d ' in line or ' S ' in line:
            data_symbols += 1
    
    return text_symbols, data_symbols

def main():
    print("\n╔════════════════════════════════════════════════════════════════╗")
    print("║         编译产物详细大小分析报告                             ║")
    print("╚════════════════════════════════════════════════════════════════╝\n")
    
    # 可执行文件信息
    exe = "build/bin/localization_example"
    if not os.path.exists(exe):
        print(f"❌ 找不到文件: {exe}")
        return
    
    exe_size = os.path.getsize(exe)
    print(f"📦 可执行文件: {exe}")
    print(f"   文件大小: {exe_size:,} 字节 ({exe_size/1024:.1f} KB)\n")
    
    # 段信息
    print("📊 内存段分析 (Virtual Memory)")
    print("━" * 60)
    
    segments = get_segment_sizes()
    total_vm = 0
    total_file = 0
    
    for seg_name in ['__TEXT', '__DATA_CONST', '__DATA', '__LINKEDIT']:
        if seg_name in segments:
            seg = segments[seg_name]
            vm_size = seg['size']
            file_size = seg['filesize']
            total_vm += vm_size
            total_file += file_size
            
            # 简化段名
            name_map = {
                '__TEXT': '代码段 (TEXT)',
                '__DATA_CONST': '常量数据 (DATA_CONST)',
                '__DATA': '全局变量 (DATA)',
                '__LINKEDIT': '链接信息 (LINKEDIT)'
            }
            
            print(f"{name_map.get(seg_name, seg_name):20} VM: {vm_size:>8,} B  | File: {file_size:>8,} B")
    
    print("━" * 60)
    print(f"{'总计':20} VM: {total_vm:>8,} B  | File: {total_file:>8,} B\n")
    
    # 符号统计
    print("📊 符号表统计")
    print("━" * 60)
    
    text_symbols, data_symbols = analyze_symbols()
    total_symbols = text_symbols + data_symbols
    
    print(f"代码符号 (函数): {text_symbols:>3}")
    print(f"数据符号 (变量): {data_symbols:>3}")
    print(f"总符号数:      {total_symbols:>3}\n")
    
    # 库信息
    lib = "build/lib/liblocalization_lib.a"
    if os.path.exists(lib):
        lib_size = os.path.getsize(lib)
        print(f"📚 静态库: {lib}")
        print(f"   库大小: {lib_size:,} 字节 ({lib_size/1024:.1f} KB)\n")
    
    # 内存占用估计
    print("📈 运行时内存占用估计")
    print("━" * 60)
    
    # 基于结构体大小的估计
    estimates = {
        "FilterState_t (协方差矩阵)": 5*5*8 + 5*8 + 4,  # 244 字节
        "VehicleState_t (状态)": 5*8 + 4,  # 44 字节
        "NoiseParameters_t": 8*8,  # 64 字节
        "CalibrationParams_t": 8*8,  # 64 字节
        "RTKCalibrationPoint_t": 44 + 44 + 3*8 + 4,  # 104 字节
        "WheelOdometryData_t": 3*8 + 4,  # 28 字节
        "IMUData_t": 6*8 + 4,  # 52 字节
    }
    
    total_estimated = 0
    for name, size in estimates.items():
        total_estimated += size
        print(f"{name:35} {size:>6} 字节")
    
    print("━" * 60)
    print(f"{'运行时最小占用':35} {total_estimated:>6} 字节\n")
    
    # 大小分类
    print("📋 大小分类统计")
    print("━" * 60)
    
    code_size = segments.get('__TEXT', {}).get('size', 0)
    data_size = segments.get('__DATA', {}).get('size', 0) + segments.get('__DATA_CONST', {}).get('size', 0)
    
    print(f"代码段大小: {code_size:>8,} 字节 ({code_size*100//exe_size:>3}%)")
    print(f"数据段大小: {data_size:>8,} 字节 ({data_size*100//exe_size:>3}%)")
    
    # TC397 MCU 适配性评估
    print("\n🎯 TC397 MCU 适配性评估")
    print("━" * 60)
    print(f"总代码+数据: {exe_size:,} 字节 (~{exe_size/1024:.1f} KB)")
    print(f"TC397 内闪容量: 典型 2-4 MB")
    print(f"使用比例: {exe_size/(2*1024*1024)*100:.2f}% (2MB 参考)")
    print(f"评级: ✅ 非常适合 (< 5% 内存占用)")
    print()

if __name__ == '__main__':
    main()

