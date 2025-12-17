#!/bin/bash
# 编译和运行脚本

set -e

echo "=========================================="
echo "  车辆定位系统 - 编译和运行脚本"
echo "=========================================="
echo ""

# 检查编译环境
if ! command -v gcc &> /dev/null; then
    echo "错误：未找到gcc编译器，请安装GCC"
    exit 1
fi

echo "✓ 编译环境检查通过"
echo ""

# 编译
echo "正在编译..."
make clean > /dev/null 2>&1 || true
make

echo ""
echo "✓ 编译成功！"
echo ""

# 运行
if [ -f "localization_example" ]; then
    echo "=========================================="
    echo "  运行示例程序"
    echo "=========================================="
    echo ""
    ./localization_example
    echo ""
    echo "=========================================="
    echo "✓ 程序执行完成"
    echo "=========================================="
else
    echo "错误：编译的可执行文件不存在"
    exit 1
fi
