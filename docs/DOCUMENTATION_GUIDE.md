# 📚 定位系统文档导航指南

本项目包含三个层级的文档，适合不同的学习需求。

---

## 📖 文档层级与用途

### 🟢 第1层：快速参考卡（5-10分钟阅读）

**文件：** [COORDINATE_SYSTEM_QUICK_REFERENCE.md](COORDINATE_SYSTEM_QUICK_REFERENCE.md)

**用途：** 快速查阅和错误检查

**内容：**
- ✅ 核心概念（全局 vs 车体坐标系）
- ✅ 坐标系对比表
- ✅ 关键转换公式
- ✅ 常见错误及修复方法
- ✅ 数据结构速查
- ✅ 检查清单

**适合人群：**
- 💬 需要快速理解系统的开发者
- 🐛 调试代码时需要参考
- 🔍 代码审查人员
- 📌 集成工程师

**何时使用：**
```
"这个转向角应该怎么用？" → 查这个文件
"为什么后退时结果不对？" → 查这个文件
"坐标系到底怎么转换？" → 查这个文件
```

---

### 🟡 第2层：详细规范（30-60分钟深入阅读）

**文件：** [COORDINATE_SYSTEM_SPECIFICATION.md](COORDINATE_SYSTEM_SPECIFICATION.md)

**用途：** 理解系统设计和数学原理

**内容：**
- ✅ 全局坐标系详细定义
- ✅ 车体坐标系详细定义
- ✅ 传感器坐标系定义
- ✅ 完整数学转换推导
- ✅ 运动模型方程说明
- ✅ 数据流图表
- ✅ 实现细节和代码示例
- ✅ 应用示例

**适合人群：**
- 🎓 想深入理解系统的工程师
- 🏗️ 系统架构设计者
- 📊 数据分析和优化人员
- 🔧 需要修改算法的开发者

**何时使用：**
```
"我想理解卡尔曼滤波器的工作原理" → 查这个文件
"如何修改运动模型？" → 查这个文件
"新项目采用这个系统需要了解什么？" → 查这个文件
```

---

### 🔵 第3层：实现报告（15-30分钟了解系统现状）

**文件：** [SCHEME_B_IMPLEMENTATION_REPORT.py](SCHEME_B_IMPLEMENTATION_REPORT.py)

**用途：** 了解优化方案和系统性能

**内容：**
- ✅ 优化问题描述
- ✅ 方案对比分析
- ✅ 性能指标
- ✅ 测试结果
- ✅ 内存分析
- ✅ 部署建议

**适合人群：**
- 👨‍💼 项目经理
- 📈 性能优化人员
- ✅ 质量保证人员
- 🚀 系统集成人员

**何时使用：**
```
"这个系统的性能怎么样？" → 查这个文件
"内存占用是多少？" → 查这个文件
"为什么选择混合精度方案？" → 查这个文件
```

---

## 🗂️ 文件结构总览

```
vehicle_localization/
│
├─ README.md (系统概述)
│
├─ 📋 文档（按阅读顺序）
│  ├─ COORDINATE_SYSTEM_QUICK_REFERENCE.md     ← 🟢 先读这个
│  ├─ COORDINATE_SYSTEM_SPECIFICATION.md       ← 🟡 然后读这个
│  └─ SCHEME_B_IMPLEMENTATION_REPORT.py        ← 🔵 最后了解这个
│
├─ 📂 核心源代码
│  ├─ kalman_filter.c/h                        (扩展卡尔曼滤波器)
│  ├─ motion_model.c/h                         (单轨运动模型)
│  ├─ rtk_calibration.c/h                      (RTK标定工具)
│  └─ sensor_fusion.c/h                        (传感器融合)
│
├─ 📂 类型定义
│  ├─ localization_types.h                     (外部API - float)
│  ├─ localization_types_internal.h            (内部实现 - double)
│  └─ localization_adapter.h                   (float ↔ double 转换)
│
├─ 📂 测试
│  ├─ test_float_conversion.c                  (单元测试 5个)
│  ├─ test_fixtures/                           (测试数据)
│  └─ Makefile                                 (编译脚本)
│
└─ 📂 示例
   └─ example_usage.c                          (使用示例)
```

---

## 🚀 快速开始路径

### 场景1：我是新加入的开发者

```
1️⃣  阅读 README.md (5分钟)
    ↓
2️⃣  阅读 COORDINATE_SYSTEM_QUICK_REFERENCE.md (10分钟)
    ↓
3️⃣  查看 example_usage.c (10分钟)
    ↓
4️⃣  准备好开发了！ ✅
```

**总耗时：** 25分钟

---

### 场景2：我要修改算法

```
1️⃣  阅读 COORDINATE_SYSTEM_SPECIFICATION.md (60分钟)
    ↓
2️⃣  深入研究相关源代码:
    - kalman_filter.c (运动模型部分)
    - motion_model.c (坐标转换部分)
    ↓
3️⃣  运行测试验证修改:
    - make test
    - 检查 test_float_conversion.c
    ↓
4️⃣  准备好修改了！ ✅
```

**总耗时：** 120分钟

---

### 场景3：我在调试坐标系问题

```
1️⃣  快速查阅：
    - COORDINATE_SYSTEM_QUICK_REFERENCE.md
    - 检查"常见错误"部分
    ↓
2️⃣  查看相关代码:
    - motion_model.c (转换公式)
    - 具体的传感器处理部分
    ↓
3️⃣  问题解决！ ✅
```

**总耗时：** 15分钟

---

### 场景4：我需要优化性能

```
1️⃣  阅读 SCHEME_B_IMPLEMENTATION_REPORT.py (30分钟)
    ↓
2️⃣  查看 localization_types.h 中的数据结构 (15分钟)
    ↓
3️⃣  审查 localization_adapter.h 中的转换逻辑 (20分钟)
    ↓
4️⃣  运行性能测试:
    - make clean && make
    - 检查编译输出大小
    ↓
5️⃣  准备优化方案了！ ✅
```

**总耗时：** 65分钟

---

## 📊 文档对比总结

| 需求 | 推荐文档 | 时间 | 深度 |
|------|---------|------|------|
| 快速查阅概念 | 快速参考卡 🟢 | 5分钟 | ⭐⭐ |
| 理解工作原理 | 详细规范 🟡 | 30分钟 | ⭐⭐⭐⭐ |
| 修改算法 | 详细规范 🟡 | 60分钟 | ⭐⭐⭐⭐⭐ |
| 调试问题 | 快速参考卡 🟢 | 10分钟 | ⭐⭐ |
| 性能优化 | 实现报告 🔵 | 30分钟 | ⭐⭐⭐ |
| 系统集成 | 实现报告 🔵 | 20分钟 | ⭐⭐⭐ |

---

## 🎯 关键概念导航

### 坐标系概念

| 概念 | 快速参考 | 详细规范 | 位置 |
|------|---------|---------|------|
| 全局坐标系 | [查看](COORDINATE_SYSTEM_QUICK_REFERENCE.md#-核心要点) | [查看](COORDINATE_SYSTEM_SPECIFICATION.md#全局world坐标系) | L1-L20 |
| 车体坐标系 | [查看](COORDINATE_SYSTEM_QUICK_REFERENCE.md#-核心要点) | [查看](COORDINATE_SYSTEM_SPECIFICATION.md#车体body坐标系) | L30-L50 |
| 转换公式 | [查看](COORDINATE_SYSTEM_QUICK_REFERENCE.md#-关键转换公式) | [查看](COORDINATE_SYSTEM_SPECIFICATION.md#坐标系转换推导) | L60-L100 |
| 运动模型 | [查看](COORDINATE_SYSTEM_QUICK_REFERENCE.md#-单轨模型可视化) | [查看](COORDINATE_SYSTEM_SPECIFICATION.md#单轨bicycle-model) | L120-L180 |

### 实现细节

| 主题 | 源代码 | 文档参考 |
|------|--------|---------|
| 卡尔曼滤波 | [kalman_filter.c](kalman_filter.c) | [详细规范](COORDINATE_SYSTEM_SPECIFICATION.md#卡尔曼滤波器实现) |
| 运动模型 | [motion_model.c](motion_model.c) | [快速参考](COORDINATE_SYSTEM_QUICK_REFERENCE.md#️-单轨模型可视化) |
| 类型转换 | [localization_adapter.h](localization_adapter.h) | [实现报告](SCHEME_B_IMPLEMENTATION_REPORT.py) |
| RTK标定 | [rtk_calibration.c](rtk_calibration.c) | [README.md](README.md) |

---

## ✅ 文档质量检查

- ✅ **准确性**：所有公式已验证，与源代码一致
- ✅ **完整性**：涵盖从概念到实现的全面内容
- ✅ **可读性**：采用分层次、渐进式设计
- ✅ **可维护性**：文档与代码同步更新
- ✅ **可检索性**：完整的导航和交叉引用

---

## 💡 最佳实践

### 📖 首次上手

1. 从 README.md 开始了解系统功能
2. 阅读 COORDINATE_SYSTEM_QUICK_REFERENCE.md 理解坐标系
3. 查看 example_usage.c 学习实际用法
4. 编译并运行 make test 验证环境

### 🔍 深入学习

1. 阅读 COORDINATE_SYSTEM_SPECIFICATION.md 理解数学原理
2. 对应阅读源代码：kalman_filter.c, motion_model.c
3. 研究 localization_adapter.h 了解类型转换策略
4. 分析测试代码了解使用场景

### 🛠️ 开发维护

1. 保持文档与代码同步
2. 每个重要函数添加坐标系注释
3. 新算法修改时更新对应文档
4. 定期review文档和代码的一致性

---

## 📞 文档维护

**最后更新：** 2025-12-17  
**版本：** 1.0  
**维护者：** Vehicle Localization Team

---

**快速导航：**
- 🟢 [快速参考卡](COORDINATE_SYSTEM_QUICK_REFERENCE.md) - 3分钟入门
- 🟡 [详细规范](COORDINATE_SYSTEM_SPECIFICATION.md) - 深入理解
- 🔵 [实现报告](SCHEME_B_IMPLEMENTATION_REPORT.py) - 性能数据

