#!/bin/bash

echo "=== 编译产物详细分析 ===" 
echo
echo "1. 可执行文件大小"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ls -lh build/bin/localization_example
stat -f "文件大小: %z 字节" build/bin/localization_example

echo
echo "2. 符号统计 (符号表)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# macOS 使用 nm
echo "代码符号 (函数):"
nm -n build/bin/localization_example | grep -E "[tT] " | wc -l
echo "函数总数"

echo
echo "数据符号 (变量):"
nm -n build/bin/localization_example | grep -E "[dD] " | wc -l
echo "全局变量总数"

echo
echo "3. 段大小详细分析"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
otool -l build/bin/localization_example | grep -E "segname|vmsize|filesize" | awk '
  /segname/ { seg=$2 }
  /vmsize/ { vm=$2; getline; print seg ": " vm }
'

echo
echo "4. 库文件分析"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "库文件大小:"
ls -lh build/lib/liblocalization_lib.a
stat -f "大小: %z 字节" build/lib/liblocalization_lib.a

echo
echo "库中包含的对象文件:"
ar -t build/lib/liblocalization_lib.a

echo
echo "5. 各模块大小估计"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "模块 | 代码大小 | 数据大小 | 总计"
echo "──────────────────────────────────"

for module in kalman_filter motion_model wheel_odometry_processor rtk_calibration localization_system example_usage; do
  if [ -f "build/CMakeFiles/localization_lib.dir/${module}.c.o" ]; then
    size=$(stat -f "%z" "build/CMakeFiles/localization_lib.dir/${module}.c.o" 2>/dev/null || echo "0")
    echo "$module | ? | ? | $size 字节"
  fi
done

