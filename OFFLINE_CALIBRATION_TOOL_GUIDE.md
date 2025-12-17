# 离线RTK参数标定工具使用指南

## 概述

本工具用于从CSV格式的RTK和整车数据进行离线参数标定，包括：
- **CSV数据生成工具** - 生成模拟的标定数据
- **离线标定工具** - 读取CSV数据并计算最优参数
- **完整的数据分析** - 位置误差、航向角误差等统计

## 快速开始

### 1. 编译工具

```bash
cd /Users/alex/workspace/code_review/vehicle_localization
./cmake_build.sh build
```

编译后生成的工具位置：
- `build/bin/generate_calibration_csv` - CSV生成工具
- `build/bin/offline_calibration` - 离线标定工具

### 2. 生成示例数据

```bash
./build/bin/generate_calibration_csv calibration_data.csv
```

**输出：**
- `calibration_data.csv` - 包含3000行的示例标定数据

### 3. 执行离线标定

```bash
./build/bin/offline_calibration calibration_data.csv
```

**输出文件：**
- `calibration_result_params.txt` - 标定参数
- `calibration_result_report.txt` - 详细报告
- `calibration_result_data.csv` - 标定数据（含误差）

### 4. 查看结果

```bash
cat calibration_result_params.txt
cat calibration_result_report.txt
```

## CSV文件格式

### 输入CSV格式

CSV文件需要包含以下列（必须有标题行）：

| 列名 | 说明 | 单位 | 范围 |
|-----|------|------|------|
| timestamp | 时间戳 | ms | 正整数 |
| odo_x | 里程计X位置 | m | 实数 |
| odo_y | 里程计Y位置 | m | 实数 |
| odo_theta | 里程计航向角 | rad | [-π, π] |
| odo_v | 里程计速度 | m/s | 实数 |
| odo_delta | 里程计转向角 | rad | 实数 |
| rtk_x | RTK X位置 | m | 实数 |
| rtk_y | RTK Y位置 | m | 实数 |
| rtk_heading | RTK航向角 | rad | [-π, π] |
| rtk_quality | RTK质量 | 0-5 | 整数 |

### 最小标定点数

- **最小值**：5个点
- **推荐值**：50-100个点
- **高精度**：200+个点

### RTK质量要求

只有质量值 ≥ 2 的数据才会被使用：
- 0-1：低质量（跳过）
- 2：浮点定位（接受）
- 3-4：PPP/RTK定位（接受）
- 5：精密定位（最佳）

## 数据收集建议

### 场景覆盖

为了获得更好的标定效果，应该收集多种场景的数据：

1. **直行运动**
   - 直线段走过不同长度（如50m, 100m, 200m）
   - 不同速度（0.5m/s, 1.0m/s, 2.0m/s）

2. **转向运动**
   - 不同转向角度（15°, 30°, 45°, 60°, 90°）
   - 不同速度下的转向

3. **航向角覆盖**
   - 尽可能覆盖 360° 的航向角范围
   - 向前、向后、左转、右转等

4. **速度覆盖**
   - 低速 (0.5 m/s)
   - 中速 (1.0 m/s)
   - 高速 (2.0 m/s)

### 质量检查

```bash
# 查看数据统计
grep "rtk_quality" calibration_data.csv | awk -F',' '{print $10}' | sort | uniq -c

# 查看有效行数
awk -F',' '$10 >= 2 {count++} END {print count}' calibration_data.csv
```

## 输出文件说明

### 1. _params.txt - 标定参数文件

包含计算出的最优标定参数：

```
wheel_calibration_factor = 0.99800000
steering_angle_offset = 0.020000
steering_angle_gain = 1.00000000
imu_accel_bias_x = 0.050000
imu_accel_bias_y = -0.030000
imu_gyro_bias_z = 0.002000
imu_accel_scale_x = 1.00500000
imu_accel_scale_y = 0.99700000
```

**参数说明：**
- `wheel_calibration_factor` - 轮速计校准因子（通常接近1.0）
- `steering_angle_offset` - 转向角零偏
- `steering_angle_gain` - 转向角增益
- `imu_accel_bias_*` - IMU加速度零偏
- `imu_gyro_bias_z` - IMU陀螺仪零偏
- `imu_accel_scale_*` - IMU加速度尺度因子

### 2. _report.txt - 详细报告

包含标定过程的详细统计信息：

```
## 数据统计
- 总行数: 3000
- 有效标定点: 2850
- 跳过行数(质量差): 150

## 位置误差
- 平均位置误差: 0.1234 m
- 最大位置误差: 0.5678 m
- 最小位置误差: 0.0012 m

## 航向角误差
- 平均航向角误差: 0.0123 rad (0.70°)
- 最大航向角误差: 0.0456 rad (2.61°)

...
```

### 3. _data.csv - 标定数据（含误差）

包含所有有效的标定数据点和计算的误差：

```
timestamp,odo_x,odo_y,odo_theta,odo_v,odo_delta,rtk_x,rtk_y,rtk_heading,rtk_quality,pos_error,heading_error
1000,0.000000,0.000000,0.000000,1.0000,0.000000,0.000000,0.000050,0.000000,4,0.000050,0.000000
...
```

## 精度评价标准

| 误差范围 | 精度评级 | 说明 |
|---------|--------|------|
| < 0.2m | 优秀 | 适合精确定位 |
| 0.2-0.5m | 良好 | 适合自动泊车 |
| 0.5-1.0m | 可接受 | 基本可用 |
| > 1.0m | 不足 | 需要重新标定 |

## 故障排除

### 问题1：标定点不足

```
标定点不足 (N < 5)
```

**原因**：CSV文件中有效数据太少

**解决方案**：
- 增加RTK质量检查的宽松度
- 收集更长时间的数据
- 检查CSV格式是否正确

### 问题2：标定精度差

```
平均位置误差: 1.2345 m  # 超过1.0m
```

**原因**：
- 里程计噪声大
- 轮胎打滑或偏磨
- 传感器安装不正确
- 数据中异常值过多

**解决方案**：
1. 检查轮胎状态
2. 验证传感器对齐
3. 过滤异常数据点
4. 在更优越的条件下重新收集

### 问题3：数据点分布不均匀

```
航向角覆盖范围: 0.5 rad (28°)  # 太小
```

**原因**：只收集了某些方向的数据

**解决方案**：
- 确保前进、后退、左转、右转等全方向数据
- 增加转向弧度
- 完成整个360°的旋转运动

## 高级用法

### 批量标定多个数据集

```bash
#!/bin/bash
for csv in calibration_*.csv; do
    prefix="${csv%.csv}"
    ./build/bin/offline_calibration "$csv" "$prefix"_result
done
```

### 对比多个标定结果

```bash
echo "对比标定结果:"
for file in *_params.txt; do
    echo "=== $file ==="
    grep "wheel_calibration_factor" "$file"
    grep "steering_angle_offset" "$file"
done
```

### 提取特定精度范围的数据

```bash
# 提取位置误差 < 0.5m 的数据
awk -F',' '$11 < 0.5 {print}' calibration_result_data.csv > filtered_data.csv
```

## 集成到生产系统

### 步骤1：离线生成标定参数

```bash
./build/bin/offline_calibration collected_data.csv prod_calibration
```

### 步骤2：验证标定质量

```bash
cat prod_calibration_report.txt | grep "平均位置误差"
```

### 步骤3：应用参数

参数可以通过以下方式应用到系统：

```c
// 方法1：直接编译进系统
const CalibrationParams_t production_calibration = {
    .wheel_calibration_factor = 0.998,
    .steering_angle_offset = 0.020,
    // ...
};

// 方法2：从参数文件读取并应用
localization_system_load_calibration(&sys, "calibration.dat");
```

## 性能指标

| 指标 | 值 |
|------|-----|
| 处理速度 | ~10000行/秒 |
| 内存占用 | ~5MB（10000点） |
| 标定计算时间 | <1秒 |
| CSV读取时间 | <100ms（1000行） |

## 相关文档

- [RTK离线标定工作流程](RTK_OFFLINE_CALIBRATION.md)
- [参数标定详解](PARAMETER_CALIBRATION_GUID.md)
- [集成指南](INTEGRATION_GUIDE.md)

## 许可和支持

本工具作为车辆定位系统的一部分提供。

如有问题或改进建议，请参考项目文档。

---

**版本**：1.0  
**更新日期**：2024年  
**状态**：支持CSV离线标定
