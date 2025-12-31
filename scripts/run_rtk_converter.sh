#!/bin/bash
# RTK坐标转换脚本运行器

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================================================"
echo "RTK坐标转换工具 - 启动脚本"
echo "========================================================================"
echo ""

# 检查Python版本
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误：未找到python3"
    echo "   请先安装Python 3.6或更高版本"
    exit 1
fi

echo "✓ Python版本："
python3 --version
echo ""

# 检查输入文件
if [ ! -f "$PROJECT_DIR/rtk_can.csv" ]; then
    echo "❌ 错误：输入文件不存在"
    echo "   位置：$PROJECT_DIR/rtk_can.csv"
    exit 1
fi

echo "✓ 输入文件检查："
echo "   - rtk_can.csv: 存在"
if [ -f "$PROJECT_DIR/vehicle_localization.csv" ]; then
    echo "   - vehicle_localization.csv: 存在"
else
    echo "   - vehicle_localization.csv: 不存在（将仅使用RTK数据）"
fi
echo ""

# 运行转换脚本
echo "开始坐标转换..."
echo ""

cd "$PROJECT_DIR"
python3 "$SCRIPT_DIR/rtk_coordinate_converter.py"

RESULT=$?

echo ""
if [ $RESULT -eq 0 ]; then
    echo "========================================================================"
    echo "✓ 转换完成"
    echo "========================================================================"
    
    if [ -f "$PROJECT_DIR/rtk_converted.csv" ]; then
        echo ""
        echo "输出文件信息："
        echo "  - 位置：$(pwd)/rtk_converted.csv"
        wc -l "$PROJECT_DIR/rtk_converted.csv" | awk '{print "  - 行数：" $1}'
        ls -lh "$PROJECT_DIR/rtk_converted.csv" | awk '{print "  - 大小：" $5}'
    fi
    exit 0
else
    echo "========================================================================"
    echo "❌ 转换失败"
    echo "========================================================================"
    exit 1
fi
