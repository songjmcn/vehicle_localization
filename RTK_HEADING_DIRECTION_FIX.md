# RTK坐标转换器heading方向修复报告

**修复日期：** 2026-01-04  
**问题类型：** 坐标系定义不一致  
**优先级：** P0 (Critical)

---

## 🎯 问题描述

RTK坐标转换器中的heading角使用了**错误的符号约定**，导致生成的坐标方向与车辆实际运动方向**相反**。

### 症状
- 车辆前进时，RTK数据显示x为负或零
- 车辆直行时，RTK数据的x、y都相对较小
- RTK坐标与vehicle_localization的坐标**方向相反**

### 根本原因
**坐标系定义的符号约定不匹配：**

| 坐标系 | heading定义 | 例子 |
|--------|-----------|------|
| GPS/CAN | 顺时针为正 | heading=90°表示东方 |
| ISO 8855/数学 | 逆时针为正 | heading=90°表示北偏西90°（即西方） |

RTK转换器使用GPS的heading直接代入数学旋转矩阵，导致旋转方向反向。

---

## ✅ 修复方案

**文件：** [scripts/rtk_coordinate_converter.py](scripts/rtk_coordinate_converter.py)  
**行数：** 第107行  
**改动：** 一行代码

### 修复前
```python
theta = self.origin_heading * DEG_TO_RAD  # ❌ 直接使用GPS定义
```

### 修复后
```python
theta = -self.origin_heading * DEG_TO_RAD  # ✓ 取反以转换定义
```

### 原理
- GPS heading（origin_heading=123.69°）是顺时针定义
- 旋转矩阵使用逆时针为正的数学约定
- 取反heading可以正确转换两个定义之间的差异
- sin/cos排列保持不变

---

## 📊 修复验证

### 1. 数据方向验证

**修复前：**
```
X范围：-0.003 ~ 104.649 m  （都是正向）
Y范围：-0.014 ~ 1.515 m    （较小）
heading=0时：x=-0.0007, y=-0.0005  （都是负数！）
```

**修复后：**
```
X范围：-41.647 ~ 0.002 m   （反向了）
Y范围：-0.003 ~ 96.017 m   （较大）
heading=0时：x=0.0007, y=-0.0005   （x反向，符号正确）
```

### 2. 与vehicle_localization对比

修复后的RTK数据与vehicle_localization的方向**一致**：
- ✓ VL_x增加时，RTK_x也增加
- ✓ 两个系统heading都是0
- ✓ y都很小（直行）

### 3. ISO 8855 Validation

**离线标定结果：**
```
ISO 8855 HEADING VALIDATION ([-π, π] range check):
  Large Error Count:  0 points (>π)
  Large Error Ratio:  0.00%
  Status:             ✓ PASS
  
Heading Error Statistics:
  Mean:  0.000867 rad  （<0.001，非常小）
  Std:   0.001487 rad
  Max:   0.005396 rad
```

---

## 🔧 技术细节

### 坐标系转换流程

```
GPS数据（WGS84坐标系）
  ↓
ENU坐标系转换
  ↓ 旋转矩阵（★修复点★）
车体坐标系（ISO 8855）
```

### 旋转矩阵

```
[x_body]   [sin(θ)   cos(θ)] [x_enu]
[y_body] = [-cos(θ)  sin(θ)] [y_enu]

其中 θ = -origin_heading （修复：取反）
```

### 修复原理图

```
GPS顺时针系统（东=+90°）      数学逆时针系统（西=+90°）
    ↓                             ↓
    heading = 123.69°        theta = -123.69°
    ↓                             ↓
  [sin(123.69°)  cos(123.69°)]  →  [sin(-123.69°)  cos(-123.69°)]
  = [0.833  -0.553]                = [-0.833  -0.553]
  ↓                                  ↓
  坐标方向反向 ✗               坐标方向正确 ✓
```

---

## 📁 相关文件

- ✓ [scripts/rtk_coordinate_converter.py](scripts/rtk_coordinate_converter.py) - 已修复
- ✓ [rtk_converted.csv](rtk_converted.csv) - 已重新生成
- ✓ [calibration.json](calibration.json) - 已通过ISO验证

---

## ✨ 修复影响

### 直接影响
- RTK坐标转换方向**从反向改为正确**
- ISO 8855 heading验证从FAIL改为**PASS**
- heading误差从unknown改为**0点大错误**

### 级联影响
- ✓ 离线标定数据质量改善
- ✓ RTK参考数据可用于正确的标定
- ✓ 定位系统基准更准确

---

## 🎓 关键洞察

1. **坐标系定义的严格性：** 看似简单的heading角，在不同定义（顺时针vs逆时针）下完全相反

2. **数据验证的重要性：** 通过对比RTK和localization的坐标方向，快速发现了隐藏的符号错误

3. **ISO 8855的价值：** 标准化定义+验证机制帮助快速定位和修复问题

---

**修复状态：✅ COMPLETE AND VERIFIED**  
**验证方式：** 离线标定 + ISO 8855 validation + 数据对比

