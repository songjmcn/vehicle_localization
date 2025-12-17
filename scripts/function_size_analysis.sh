#!/bin/bash

echo "=== 函数大小分析 ===" 
echo

# 提取所有函数并按大小排序
echo "📊 所有函数大小（按大小排序）"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

nm -n build/bin/localization_example | grep -E "^[0-9a-f]+ [tT] " | while read addr type name; do
  # 从符号表获取大小信息
  printf "%-40s %s\n" "$name" "$addr"
done | sort -r | head -30

echo
echo "📊 主要模块函数统计"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 卡尔曼滤波器函数
echo "卡尔曼滤波器函数:"
nm build/bin/localization_example | grep kalman | head -10

echo
echo "运动模型函数:"
nm build/bin/localization_example | grep motion | head -10

echo
echo "轮速计处理器函数:"
nm build/bin/localization_example | grep wheel | head -10

