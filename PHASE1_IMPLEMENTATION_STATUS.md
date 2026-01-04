# Phase 1 (P0) RTK 标定实现状态报告

## 完成情况

### 1. 结构体扩展 ✅
- 在 `rtk_calibration.h` 中添加新字段到 `RTKCalibrationManager_t`：
  - `outlier_count`: 离群点计数
  - `r_squared`: 拟合优度 (0-1)
  - `weighted_std_dev`: 加权残差标准差
  - `rejected_count`: 验证失败的点数
  - `low_quality_skip`: 质量低而跳过的点数

### 2. 核心算法实现 ✅
- **加权最小二乘法** (`estimate_wheel_calibration_weighted`)
  - 权重计算：quality × trajectory_length
  - 鲁棒初始估计：trimmed mean (0.5-2.0范围)
  - WLS参数求解
  - R²和加权标准差计算

- **辅助函数**
  - `calculate_mean()`: 计算均值
  - `calculate_std_dev()`: 计算标准差
  - `mark_outliers()`: 离群点检测（绝对阈值）

### 3. 数据验证框架 ✅
- `rtk_calibration_add_point_with_validation()` 函数实现
  - 时间同步检查：速度加速度不超过5 m/s²
  - 轨迹连续性检查：最大10m跳跃
  - 质量检查：跳过质量 < 2的点

### 4. 单元测试 ✅
- 创建 `test_rtk_calibration_p0.c` 包含7个测试用例：
  1. 离群点检测
  2. 加权最小二乘法 (不同质量)
  3. 含10%离群点的合成数据集
  4. 轨迹连续性检查
  5. 低质量点跳过
  6. 拟合优度指标 (R²和标准差)
  7. 清除功能

## 已知问题

### 算法问题
1. **离群点检测阈值调整中**
   - 当前使用绝对残差阈值：max(3×std_dev, 5.0)米
   - σ-3准则在包含极端离群点时不够鲁棒
   - 可能需要迭代式离群点检测或RANSAC方法

2. **加权LS求解**
   - 公式验证中：factor = Σ(w × odo_dist × rtk_dist) / Σ(w × rtk_dist²)
   - 需要进一步测试和调整权重策略

### 测试问题
- 某些测试场景下calibration_factor不符合预期
- 需要用实际CSV数据进行集成测试

## 改进方向（后续阶段）

### 立即可改进（P0优化）
```
1. 使用中位数绝对偏差 (MAD) 替代标准差作为鲁棒度量
2. 实现两次迭代的离群点检测
3. 添加诊断日志输出拟合过程细节
4. 用Python验证算法数值正确性
```

### 后续阶段
- P1: 转向角增益标定
- P2: 时间同步补偿
- P3: IMU标定
- P4: 在线自适应更新

## 编译和运行

```bash
# 编译
cd tools
gcc -I../src -o test_rtk_calibration_p0 test_rtk_calibration_p0.c rtk_calibration.c -lm

# 运行
./test_rtk_calibration_p0
```

## 代码位置
- 头文件: [tools/rtk_calibration.h](../tools/rtk_calibration.h)
- 实现: [tools/rtk_calibration.c](../tools/rtk_calibration.c)
- 测试: [tools/test_rtk_calibration_p0.c](../tools/test_rtk_calibration_p0.c)

## 提交信息
```
feat: Phase 1 (P0) RTK calibration implementation with weighted least squares and validation
```

## 统计信息
- 新增代码：~600行 (rtk_calibration.c)
- 新增测试：~395行 (test_rtk_calibration_p0.c)
- 文件修改：2个 (rtk_calibration.h/c)

---
更新时间: 2026-01-04
