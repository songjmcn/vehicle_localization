#!/usr/bin/env python3
"""
convert_mapping_to_c.py - 将CSV映射表转换为C代码

用途：将generate_steering_mapping生成的CSV文件转换为C语言数据定义，
便于直接编译到程序中（静态映射表）
"""

import sys
import csv
import argparse
from pathlib import Path

def convert_csv_to_c(csv_file, output_file=None, array_name="steering_mapping"):
    """
    将CSV文件转换为C代码
    
    Args:
        csv_file: 输入CSV文件路径
        output_file: 输出C文件路径（默认为input_name.c）
        array_name: C数组名称前缀
    
    Returns:
        0 成功，-1 失败
    """
    
    try:
        # 读取CSV文件
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # 跳过头行
            rows = list(reader)
        
        if len(rows) < 2:
            print(f"Error: CSV must have at least 2 data rows")
            return -1
        
        # 提取数据
        num_points = len(rows)
        steering_wheel = []
        left_wheel = []
        right_wheel = []
        
        for row in rows:
            try:
                steering_wheel.append(float(row[0]))
                left_wheel.append(float(row[2]))
                right_wheel.append(float(row[4]))
            except (ValueError, IndexError) as e:
                print(f"Error parsing row: {row}")
                return -1
        
        # 确定输出文件名
        if output_file is None:
            output_file = Path(csv_file).stem + ".c"
        
        # 生成C代码
        c_code = generate_c_code(
            steering_wheel, left_wheel, right_wheel, 
            array_name, Path(csv_file).name
        )
        
        # 写入输出文件
        with open(output_file, 'w') as f:
            f.write(c_code)
        
        print(f"Successfully converted to C code: {output_file}")
        print(f"  - Points: {num_points}")
        print(f"  - Array name: {array_name}_steering_wheel, etc.")
        print(f"\nUsage in C:")
        print(f"  steering_mapping_init(")
        print(f"      {array_name}_steering_wheel,")
        print(f"      {array_name}_left_wheel,")
        print(f"      {array_name}_right_wheel,")
        print(f"      {num_points}")
        print(f"  );")
        
        return 0
        
    except IOError as e:
        print(f"Error: Cannot read file {csv_file}: {e}")
        return -1
    except Exception as e:
        print(f"Unexpected error: {e}")
        return -1


def generate_c_code(steering_wheel, left_wheel, right_wheel, array_name, source_file):
    """
    生成C代码
    """
    
    num_points = len(steering_wheel)
    
    code = f"""/**
 * @file {Path(array_name).name}.c
 * @brief 转向映射表定义
 * 
 * 这个文件由 convert_mapping_to_c.py 从 {source_file} 自动生成
 * 共包含 {num_points} 个映射点
 * 
 * 生成方式：
 *   python3 convert_mapping_to_c.py {source_file}
 */

#include <stdint.h>

/* 方向盘转角数组 [rad] */
static const double {array_name}_steering_wheel[] = {{
"""
    
    # 添加steering_wheel数据
    for i, val in enumerate(steering_wheel):
        if i % 5 == 0:
            code += "    "
        code += f"{val:12.6f},"
        if (i + 1) % 5 == 0 or i == len(steering_wheel) - 1:
            code += "\n"
        else:
            code += " "
    
    code += f"""}};

/* 前左轮转角数组 [rad] */
static const double {array_name}_left_wheel[] = {{
"""
    
    # 添加left_wheel数据
    for i, val in enumerate(left_wheel):
        if i % 5 == 0:
            code += "    "
        code += f"{val:12.6f},"
        if (i + 1) % 5 == 0 or i == len(left_wheel) - 1:
            code += "\n"
        else:
            code += " "
    
    code += f"""}};

/* 前右轮转角数组 [rad] */
static const double {array_name}_right_wheel[] = {{
"""
    
    # 添加right_wheel数据
    for i, val in enumerate(right_wheel):
        if i % 5 == 0:
            code += "    "
        code += f"{val:12.6f},"
        if (i + 1) % 5 == 0 or i == len(right_wheel) - 1:
            code += "\n"
        else:
            code += " "
    
    code += f"""}};

/* 映射表点数 */
static const uint32_t {array_name}_num_points = {num_points};

/**
 * @brief 初始化映射表
 * 
 * 在主程序中调用此函数来加载映射表
 */
void init_{array_name}(void) {{
    steering_mapping_init(
        {array_name}_steering_wheel,
        {array_name}_left_wheel,
        {array_name}_right_wheel,
        {array_name}_num_points
    );
}}
"""
    
    return code


def print_usage(prog_name):
    """打印使用说明"""
    print(f"""Usage: {prog_name} <csv_file> [options]

Convert a steering mapping CSV file to C code

Arguments:
  csv_file          Input CSV file path

Options:
  -o, --output FILE Output C file path (default: input_name.c)
  -n, --name NAME   Array name prefix (default: steering_mapping)
  -h, --help        Show this help

Examples:
  {prog_name} steering_mapping.csv
  {prog_name} steering_mapping.csv -o my_mapping.c -n mymap
  {prog_name} progressive_21pts.csv -o mapping_progressive.c
""")


def main():
    """主函数"""
    
    parser = argparse.ArgumentParser(
        description="Convert steering mapping CSV to C code",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  %(prog)s steering_mapping.csv
  %(prog)s steering_mapping.csv -o custom.c -n custom_map
  %(prog)s data/progressive_mapping.csv --output bin/mapping.c"""
    )
    
    parser.add_argument('csv_file', help='Input CSV file')
    parser.add_argument('-o', '--output', help='Output C file path')
    parser.add_argument('-n', '--name', default='steering_mapping',
                       help='Array name prefix (default: steering_mapping)')
    
    args = parser.parse_args()
    
    # 验证输入文件
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: File not found: {args.csv_file}")
        return 1
    
    if not csv_path.suffix.lower() == '.csv':
        print(f"Warning: File doesn't have .csv extension")
    
    # 转换
    result = convert_csv_to_c(str(csv_path), args.output, args.name)
    
    return 0 if result == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
