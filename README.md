# 车辆定位系统 | Vehicle Localization System

> 面向自动泊车场景的高精度多传感器融合定位解决方案

## 📖 项目概览

这是一个完整的**多传感器融合定位系统**，针对自动泊车场景（0.5~2.0 m/s）优化设计。系统采用扩展卡尔曼滤波（EKF）融合轮速计、IMU和转向角等多源传感器信息，支持前进/后退运动，并提供离线RTK校准工具用于参数优化。

### 🎯 核心特性

| 特性 | 说明 |
|-----|------|
| **多传感器融合** | 轮速计 + IMU + 转向角实时融合定位 |
| **低速优化** | 针对0.5~2.0 m/s自动泊车场景专优化 |
| **前后退支持** | 运动模型支持双向运动，适应复杂泊车轨迹 |
| **RTK离线标定** | 基于高精度RTK的离线参数标定工具链 |
| **自适应滤波** | 轮滑检测与自适应噪声参数调整 |
| **生产就绪** | C99实现，嵌入式友好，经过编译验证 |

## �️ 项目结构

```
vehicle_localization/
│
├─ 📘 文档库 (docs/)
│  ├─ 🆕 最近修复 (2025年12月 重要！)
│  │  ├── DECEMBER_2025_FIX_SUMMARY.md         ⭐ 修复总结与快速导航（必读）
│  │  ├── JACOBIAN_COVARIANCE_FIX.md          🔴 关键修复：雅可比矩阵与协方差初值问题
│  │  ├── REVERSE_MOTION_BUG_ANALYSIS.md      🔴 倒车运动Y坐标为0问题根源分析
│  │  ├── KALMAN_FILTER_DEBUGGING.md          🔧 卡尔曼滤波诊断与调试完整指南
│  │  └── COORDINATE_AXES_VISUALIZATION.md    📊 坐标系重新定义与轨迹可视化
│  │
│  ├─ 快速导航
│  │  ├── COORDINATE_ORIGIN_QUICK_VIEW.md      ⭐ 3秒速查：坐标系原点
│  │  ├── CSV_CALIBRATION_QUICKSTART.md        ⭐ 快速标定：3步完成
│  │  └── ISO8855_EXECUTIVE_SUMMARY.md         ISO 8855标准速览
│  │
│  ├─ 坐标系规范 (📐 6份文档)
│  │  ├── COORDINATE_SYSTEM_SPECIFICATION.md   完整规范
│  │  ├── COORDINATE_ORIGIN_ANALYSIS.md        原点分析
│  │  └── ISO8855_COORDINATE_SYSTEM.md         ISO标准实现
│  │
│  ├─ 算法与集成 (⚙️ 8份文档)
│  │  ├── low_speed_localization_guide.md      理论基础
│  │  ├── C_IMPLEMENTATION_GUIDE.md            代码实现
│  │  ├── INTEGRATION_GUIDE.md                 系统集成
│  │  └── MULTI_ALGORITHM_IMPLEMENTATION.md    算法方案
│  │
│  ├─ 参数标定 (🔧 5份文档)
│  │  ├── PARAMETER_CALIBRATION_GUID.MD        ⭐⭐⭐ 2500行完整指南
│  │  ├── RTK_OFFLINE_CALIBRATION.md           离线标定详解
│  │  └── OFFLINE_CALIBRATION_TOOL_GUIDE.md    工具使用说明
│  │
│  ├─ 转向模型 (🛣️ 6份文档 - 2025年12月重构)
│  │  ├── STEERING_MODEL_QUICK_REFERENCE.txt   ⭐ 快速参考卡片
│  │  ├── STEERING_MODEL_REFACTORING_COMPLETE.md ⭐ 重构完成报告
│  │  ├── STEERING_MODEL_USAGE_GUIDE.md        详细使用指南
│  │  └── STEERING_MODEL_IMPLEMENTATION_STATUS.txt 状态总结
│  │
│  ├─ 构建系统 (🏗️ CMake + Makefile)
│  │  ├── CMAKE_QUICK_REFERENCE.md             ⭐ 5分钟速查表
│  │  ├── CMAKE_BUILD_GUIDE.md                 完整构建指南
│  │  └── CMAKE_vs_MAKEFILE.md                 对比分析
│  │
│  └─ 项目文档 (📄 验收、总结、清单)
│     ├── ACCEPTANCE_CHECKLIST.md              验收检查
│     ├── PROJECT_STATUS.md                    项目状态
│     └── IMPLEMENTATION_SUMMARY.md            实现总结
│
├─ 🧮 核心模块 (src/)
│  ├── localization_types.h              # 数据类型与常量
│  ├── localization_system.h/c           # 高层系统接口 (EKF融合)
│  ├── motion_model.h/c                  # 运动学模型 (Ackermann单轨模型)
│  ├── kalman_filter.h/c                 # 扩展卡尔曼滤波 (EKF核心)
│  ├── steering_model.h/c                # 转向角处理 (2025年重构版)
│  └── localization_adapter.h            # 系统适配层
│
├─ 🛠️ 工具与标定 (tools/)
│  ├── rtk_calibration.h/c               # RTK标定模块
│  ├── wheel_odometry_processor.h/c      # 轮脉冲处理
│  ├── offline_calibration.c             # 离线参数计算
│  └── generate_calibration_csv.c        # CSV生成工具
│
├─ 📝 示例与测试
│  ├── example_usage.c                   # 完整使用示例
│  ├── example_mapping_integration.c     # 转向映射集成示例
│  └── test_steering_mapping.c           # 转向模型测试
│
├─ 📊 脚本工具 (scripts/)
│  ├── cmake_build.sh                    # CMake构建脚本
│  ├── localization_system.py            # Python参考实现
│  └── 其他分析工具 (二进制、性能分析等)
│
├─ 📈 数据与配置
│  ├── *.csv                             # 标定数据、轨迹数据
│  ├── dbc/                              # CAN DBC文件 (LEV、RTK)
│  └── build/                            # 编译输出目录
│
└─ 🔧 构建配置
   ├── CMakeLists.txt                   # CMake构建配置 (推荐)
   ├── Makefile                         # 传统Make配置 (向后兼容)
   └── README.md                        # 本文件
```

## 🚀 快速开始

### 方案 A：Python 快速验证（算法演示）

```bash
# 查看Python参考实现
cat scripts/localization_system.py

# 查看理论指南
cat docs/low_speed_localization_guide.md
```

### 方案 B：C 编译运行（CMake - 推荐）⭐

```bash
# 快速开始（一行命令）
./scripts/cmake_build.sh build && ./scripts/cmake_build.sh run

# 分步操作
./scripts/cmake_build.sh build         # 编译 Release 版本
./scripts/cmake_build.sh build-debug   # 编译 Debug 版本
./scripts/cmake_build.sh run           # 运行程序
./scripts/cmake_build.sh clean         # 清理编译产物
./scripts/cmake_build.sh rebuild       # 清除后重新编译
./scripts/cmake_build.sh help          # 查看所有命令
```

📚 详见：[CMake 快速参考](docs/CMAKE_QUICK_REFERENCE.md)

### 方案 C：传统 Makefile（向后兼容）

```bash
make clean && make && make run     # 一键编译运行
make test_motion                   # 仅编译运动模型
make test_filter                   # 仅编译滤波模块
```

📚 详见：[CMake vs Makefile](docs/CMAKE_vs_MAKEFILE.md)

## 📊 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                 定位系统（LocalizationSystem）           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │ 轮速计   │  │   IMU    │  │ 转向角   │             │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘             │
│       │             │             │                   │
│       └─────────────┼─────────────┘                   │
│                     │ 传感器数据融合                   │
│                     ▼                                  │
│        ┌────────────────────────────┐                │
│        │  扩展卡尔曼滤波 (EKF)       │                │
│        │  - 预测步骤                │                │
│        │  - 更新步骤                │                │
│        │  - 自适应噪声              │                │
│        └────────────┬───────────────┘                │
│                     │                                 │
│          ┌──────────┴──────────┐                     │
│          ▼                     ▼                      │
│    ┌──────────────┐    ┌──────────────┐            │
│    │ 位置估计     │    │ RTK标定模块  │            │
│    │(x, y, θ)    │    │ - 收集点      │            │
│    └──────────────┘    │ - 校准参数    │            │
│                        │ - 在线更新    │            │
│                        └──────────────┘            │
│                                                     │
└─────────────────────────────────────────────────────────┘
```

## 💡 核心算法

### 运动模型（支持前后退）

**位置更新**：
$$x_{k+1} = x_k + |v| \cos(\theta_k + \text{sign}(v)\delta) \Delta t$$
$$y_{k+1} = y_k + |v| \sin(\theta_k + \text{sign}(v)\delta) \Delta t$$

**航向角更新**（Ackermann单轨模型）：
$$\theta_{k+1} = \theta_k + \frac{\text{sign}(v)|v|\tan(\delta)}{L} \Delta t$$

其中：
- $v$ = 轮速计速度
- $\delta$ = 转向角
- $L$ = 轴距 (通常 2.7 m)
- $\Delta t$ = 时间步长

### 扩展卡尔曼滤波（EKF）

**状态向量**：$\mathbf{x} = [x, y, \theta, v, \delta]^T$

**融合过程**：
1. **预测**：基于运动模型预测下一状态
2. **更新**：融合传感器测量进行校正
3. **自适应**：根据轮滑检测调整噪声参数

### 转向角处理（2025年重构）

✨ 新特性（重构版本）：
- ✅ 消除 3 个全局变量 → 支持多实例
- ✅ 线程安全：每实例独立配置结构体
- ✅ 15 个函数重构，API保持兼容性升级

详见：[转向模型重构完成报告](docs/STEERING_MODEL_REFACTORING_COMPLETE.md)
## 🔧 最近修复说明（2025年12月31日）

### 问题背景

系统在**倒车运动时Y坐标始终为0**，导致轨迹无法正确累积。经过深入分析，发现了**两个独立的符号错误**共同作用引发此问题。

### 修复内容

#### 修复1：去除heading计算中的triple-reversal（运动模型）
- **文件**：`src/motion_model.c` 第145-177行、第170-171行
- **问题**：`get_effective_steering_angle()` 对倒车速度进行了不必要的符号反转
- **修复**：直接使用原始steering角，不再调用该函数
- **影响**：heading角能够正确响应转向输入

#### 修复2：修正雅可比矩阵F[0][3]和F[1][3]（卡尔曼滤波）
- **文件**：`src/motion_model.c` 第274-276行
- **问题**：在位置关于速度的偏导数中错误地添加了`sign_v`因子
- **修复**：移除`sign_v`，因为速度本身已经是有符号的
- **影响**：位置更新的卡尔曼增益计算正确

#### 修复3：调整协方差矩阵初值（卡尔曼滤波）
- **文件**：`src/kalman_filter.c` 第252-254行
- **问题**：所有状态的初始协方差都是0.1，过于乐观，导致卡尔曼滤波被锁定
- **修复**：根据物理意义设置合理初值（位置10m，航向1rad，速度0.5m/s）
- **影响**：滤波器能够正确响应传感器更新

### 验证结果

| 场景 | 修复前 | 修复后 | 改进 |
|-----|--------|--------|------|
| 倒车直线 | Y=0 ❌ | Y=0 ✓ | 不变（正确） |
| 倒车转向 | Y=0 ❌ | Y=3.07m ✓ | **已修复！** |
| 前进转向 | Y=3.07m ✓ | Y=3.07m ✓ | 不变（正确） |
| heading响应 | 无反应 ❌ | -19.1° ✓ | **已修复！** |

### 相关文档

- 📖 [JACOBIAN_COVARIANCE_FIX.md](docs/JACOBIAN_COVARIANCE_FIX.md) - 详细修复说明（数学推导）
- 📖 [REVERSE_MOTION_BUG_ANALYSIS.md](docs/REVERSE_MOTION_BUG_ANALYSIS.md) - 问题根源追踪与调试过程
- 📖 [KALMAN_FILTER_DEBUGGING.md](docs/KALMAN_FILTER_DEBUGGING.md) - 卡尔曼滤波诊断与调试指南
- 📖 [COORDINATE_AXES_VISUALIZATION.md](docs/COORDINATE_AXES_VISUALIZATION.md) - 轨迹可视化坐标轴重定义

### 快速验证

```bash
# 编译修复后的代码
cmake -B build && cmake --build build

# 运行定位示例
./build/bin/localization_example

# 查看结果（应该看到Y坐标在倒车时正常累积）
grep "^[0-9]" log.txt | tail -10
```
## � 使用示例

### C 语言集成

```c
#include "localization_system.h"

int main() {
    // 1. 初始化系统
    LocalizationSystem_t sys;
    VehicleState_t init_state = {.x=0, .y=0, .theta=0};
    localization_system_init(&sys, &init_state);
    
    // 2. 主处理循环
    for (int i = 0; i < num_samples; i++) {
        // 读取传感器数据
        SensorReadings_t readings = get_sensor_data();
        
        // 执行定位估计
        VehicleState_t estimated_state;
        localization_system_process(&sys, &readings, &estimated_state);
        
        // 输出估计状态
        printf("Pos: (%.2f, %.2f) m, Heading: %.2f°\n", 
               estimated_state.x, estimated_state.y, 
               RAD_TO_DEG(estimated_state.theta));
    }
    
    // 3. 清理资源
    localization_system_free(&sys);
    return 0;
}
```

### Python 参考实现

```python
from scripts.localization_system import LowSpeedLocalizationSystem, SensorReading

# 创建定位系统实例
system = LowSpeedLocalizationSystem()

# 处理传感器数据
reading = SensorReading(
    wheel_velocity=1.5,      # m/s
    imu_accel_x=0.05,        # m/s²
    imu_accel_y=0.05,        # m/s²
    imu_gyro_z=0.01,         # rad/s
    steering_angle=0.1,      # rad
    timestamp=1000           # ms
)

# 执行一步定位估计
state = system.run_step(reading)
print(f"位置: ({state.x:.2f}, {state.y:.2f}) m")
print(f"航向: {state.theta*180/3.14159:.2f}°")
```

## 📈 性能指标

| 指标 | 目标值 | 说明 |
|------|-------|------|
| **位置精度** | < 0.1 m | 1秒内累计误差 |
| **航向精度** | < 2° | 航向角误差 |
| **响应延迟** | < 100 ms | 转向响应 |
| **更新频率** | 20-50 Hz | 推荐频率 |
| **静止漂移** | < 0.05 m/min | 长期静止 |
| **RTK校准后** | < 0.05 m | 高精度 |

## 🎓 详细文档

### � 详细文档

### 🏗️ 构建系统（新增CMake）

**CMake 现代构建系统文档**：
- [docs/CMAKE_QUICK_REFERENCE.md](docs/CMAKE_QUICK_REFERENCE.md) - 快速参考卡片（5分钟）
- [docs/CMAKE_STRUCTURE.md](docs/CMAKE_STRUCTURE.md) - 工程结构说明（15分钟）
- [docs/CMAKE_BUILD_GUIDE.md](docs/CMAKE_BUILD_GUIDE.md) - 详细使用指南（20分钟）✨
- [docs/CMAKE_vs_MAKEFILE.md](docs/CMAKE_vs_MAKEFILE.md) - 工具对比分析（10分钟）
- [docs/CMAKE_CHECKLIST.md](docs/CMAKE_CHECKLIST.md) - 完成检查清单（10分钟）
- [docs/CMAKE_OVERVIEW.md](docs/CMAKE_OVERVIEW.md) - 总体概览（5分钟）

**推荐阅读顺序**：
1. 新手 → CMAKE_QUICK_REFERENCE.md
2. IDE开发者 → CMAKE_STRUCTURE.md §IDE集成
3. 深入学习 → CMAKE_BUILD_GUIDE.md

### 📖 定位系统核心文档

#### 理论基础
- 启程模式分析
- 卡尔曼滤波原理
- 低速特异性处理
- 参考文献

👉 详见 [docs/low_speed_localization_guide.md](docs/low_speed_localization_guide.md)

#### C实现详解
- 数据结构设计
- 算法实现细节
- 矩阵运算
- 内存优化
- 实时性考虑

👉 详见 [docs/C_IMPLEMENTATION_GUIDE.md](docs/C_IMPLEMENTATION_GUIDE.md)

#### 集成部署
- 硬件接口
- 传感器集成
- 校准流程
- 性能调优
- 故障诊断

👉 详见 [docs/INTEGRATION_GUIDE.md](docs/INTEGRATION_GUIDE.md)

### �️ 转向模型文档 (2025年12月重构)

#### 快速入门
- **新手必读** 👉 [docs/STEERING_MODEL_QUICK_REFERENCE.txt](docs/STEERING_MODEL_QUICK_REFERENCE.txt) (快速参考卡片)
  - API 速查表
  - 代码示例
  - 迁移检查清单
  - 性能数据

#### 详细使用指南
- **完整 API 文档** 👉 [docs/STEERING_MODEL_USAGE_GUIDE.md](docs/STEERING_MODEL_USAGE_GUIDE.md)
  - 快速开始（5分钟）
  - 完整 API 参考
  - 高级用法（多实例、线程安全）
  - 单元测试示例
  - 常见问题解答

#### 重构报告 (全局变量消除)
- **实现完成报告** 👉 [docs/STEERING_MODEL_REFACTORING_COMPLETE.md](docs/STEERING_MODEL_REFACTORING_COMPLETE.md) ⭐
  - 改动详解（15个函数）
  - 函数签名对比表
  - 编译验证结果
  - 完整迁移路径
  - 后续升级建议
  
- **实现状态总结** 👉 [docs/STEERING_MODEL_IMPLEMENTATION_STATUS.txt](docs/STEERING_MODEL_IMPLEMENTATION_STATUS.txt)
  - 改动统计（3个全局变量删除）
  - 架构改进指标
  - 验证清单
  - 已知限制说明

#### 现有文档
- [docs/STEERING_MODEL_IMPLEMENTATION_SUMMARY.md](docs/STEERING_MODEL_IMPLEMENTATION_SUMMARY.md) - 原始实现总结
- [docs/STEERING_MODEL_INTEGRATION_GUIDE.md](docs/STEERING_MODEL_INTEGRATION_GUIDE.md) - 集成指南

**关键改进**：
| 特性 | 改进 |
|------|------|
| 全局变量 | 3 → 0（100%消除）|
| 多实例支持 | ❌ → ✅ |
| 线程安全 | ❌ → ✅ |
| 编译状态 | ✅ 零错误 |

### 🛞 轮速处理与方向积分 (2025年12月新增)

#### 运动方向积分修复 ⭐

**关键改进**：实现了在轮速脉冲处理中考虑运动方向，使得**后退时速度为负，距离减少**

- **快速参考** 👉 [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)
  - 一句话总结
  - 修改内容速览
  - 快速验证方法
  - 常见问题解答

- **详细修复说明** 👉 [docs/DIRECTION_INTEGRATION_FIX.md](docs/DIRECTION_INTEGRATION_FIX.md)
  - 修复前后对比
  - 逻辑详解
  - 系统集成影响
  - 注意事项

- **完整测试用例** 👉 [docs/DIRECTION_INTEGRATION_TEST.md](docs/DIRECTION_INTEGRATION_TEST.md)
  - 6个基础功能测试
  - 3个集成场景测试
  - 4个代码审查检查点
  - 性能验证指标

- **完成报告** 👉 [docs/DIRECTION_INTEGRATION_COMPLETED.md](docs/DIRECTION_INTEGRATION_COMPLETED.md) ⭐
  - 实现详情
  - 修复成果
  - 部署步骤
  - 关键数据统计

**修改统计**：
| 指标 | 值 |
|------|-----|
| 修改文件 | wheel_odometry_processor.c |
| 修改函数 | 2 |
| 新增代码 | ~78行 |
| 时间复杂度 | O(1) |
| 向后兼容 | ✅ 完全 |

**修复效果**：
```
前进：velocity > 0，distance 增加 ✓
后退：velocity < 0，distance 减少 ✓ （关键修复）
停止：velocity = 0，distance 不变 ✓
```

### 🔧 参数标定和调参指南（重要！）⭐

#### 离线CSV标定工具 🆕

**快速标定工具链**（包含数据生成和参数计算）：

- [docs/CSV_CALIBRATION_QUICKSTART.md](docs/CSV_CALIBRATION_QUICKSTART.md) - **快速开始指南** 
  - 3步完成标定
  - CSV格式说明
  - 工具使用示例

- **工具组件**：
  - `generate_calibration_csv` - 生成模拟标定数据
  - `offline_calibration` - 离线参数计算

#### 完整的参数标定指南

- [docs/PARAMETER_CALIBRATION_GUID.MD](docs/PARAMETER_CALIBRATION_GUID.MD) - **2500行详细指南** ✨
  
  包含5个完整部分：
  
  **Part 1：参数体系总览 + 转向过滤参数**
  - 22个参数分类
  - 优先级划分
  - filter_alpha（转向滤波系数）详解
  - jitter_threshold（抖动检测）详解
  
  **Part 2：卡尔曼滤波过程噪声参数（Q矩阵）**
  - q_x, q_y：位置过程噪声
  - q_theta：航向过程噪声
  - q_v：速度过程噪声（低速关键参数）
  - q_delta：转向过程噪声
  - 3个场景推荐配置
  
  **Part 3：卡尔曼滤波测量噪声参数（R矩阵）⭐必须标定**
  - r_wheel：轮速测量噪声
  - r_imu：IMU加速度测量噪声
  - r_steering：转向角测量噪声
  - 3种标定方法对比
  
  **Part 4：传感器校准参数**
  - wheel_calibration_factor：轮速校准
  - steering_angle_offset/gain：转向校准
  - imu_accel_*_bias/scale：IMU加速度校准
  - imu_gyro_z_bias：陀螺仪零偏
  - imu_mag_*_bias：磁强计校准
  
  **Part 5：调参决策树 + 实际工作流程**
  - 故障症状诊断树
  - 3阶段标定流程（快速→精细→验证）
  - 22参数完整速查表
  - 3种场景推荐参数集合
  - 调参优先级速查表

**快速使用建议**：
- 首次标定：按 Part 5 的快速标定流程（30分钟）
- 精细调参：参考 Part 2-4 的调参方法
- 故障诊断：查询 Part 5 的决策树
- 参数查阅：使用 Part 5 的速查表

## 💾 传感器要求

### 轮速计（Wheel Odometer）
- 频率：20-50 Hz
- 精度：±2%
- 输出：速度 (m/s)

### IMU（Inertial Measurement Unit）
- 频率：>100 Hz
- 加速度计：±10 g
- 陀螺仪：±500 °/s

### 转向角传感器
- 频率：50 Hz
- 精度：±0.5°
- 范围：±35°

### RTK GNSS（可选，用于校准）
- 精度：< 0.1 m
- 频率：1-10 Hz
- 质量指示：0-5（推荐≥2）

## 📋 RTK校准工作流程

```
1. 数据收集阶段
   运行车辆在典型轨迹上移动
   同时收集RTK数据（≥10个点）
   ↓
2. 离线计算
   计算轮速计校准因子
   估计转向角零偏
   ↓
3. 应用校准
   更新定位系统的校准参数
   验证校准质量
   ↓
4. 验证结果
   比对RTK和里程计位置
   目标误差：< 0.1 m
```

## 🛠 编译选项

```bash
# 使用GCC编译
CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99 -lm

# 支持的选项
make              # 编译所有
make run          # 编译并运行示例
make clean        # 清理构建文件
make test_motion  # 编译运动模型测试
make test_filter  # 编译滤波器测试
make test_rtk     # 编译RTK模块测试
```

## 📱 硬件要求

### 最低配置
- **处理器**：ARM Cortex-M4 (168 MHz)
- **内存**：RAM > 20 KB，Flash > 100 KB
- **实时性**：支持50 Hz 运行

### 推荐配置
- **处理器**：ARM Cortex-A9 或更高
- **内存**：RAM > 100 MB
- **系统**：Linux / RTOS

## 📊 代码统计

| 模块 | 代码量 | 说明 |
|------|--------|------|
| 运动模型 | ~200 行 | 单轨模型实现 |
| 卡尔曼滤波 | ~600 行 | EKF核心算法 |
| RTK标定 | ~400 行 | 标定管理器 |
| 系统集成 | ~300 行 | 高层接口 |
| 示例程序 | ~300 行 | 完整示例 |
| **总计** | **~1800 行** | C代码 |

## 🐛 故障排查

### 位置持续漂移
→ 检查RTK标定、增加过程噪声、验证IMU零偏

### 转向响应慢
→ 减少转向角测量噪声、增加过程噪声

### 后退不准确  
→ 收集后退RTK数据、调整运动模型参数

👉 详见 [docs/INTEGRATION_GUIDE.md](docs/INTEGRATION_GUIDE.md) 中的故障诊断部分

## 📚 相关资源

### 经典文献
- Thrun, S., et al. (2005) "Probabilistic Robotics"
- Bar-Shalom, Y., et al. (2001) "Estimation with Applications"

### 标准规范
- SAE J2945: Vehicle-to-Everything (V2X)
- ISO 19141: Road vehicles - Automated driving

## 🎯 应用场景

- ✅ 自动泊车系统（APS）
- ✅ 高级驾驶辅助（ADAS）
- ✅ 自主配送车辆
- ✅ 无人搬运车（AGV）
- ✅ 车库管理系统

## 📝 版本和维护

- **当前版本**：1.0
- **最后更新**：2025年12月31日
- **维护者**：Vehicle Localization Team
- **最近修复**：倒车Y坐标问题、雅可比矩阵符号错误、协方差初值优化

## 📄 许可证

本项目提供参考实现，可自由使用和修改。

---

## 🤝 快速导航

### 最近修复概览

| 内容 | 阅读 | 时长 |
|-----|------|------|
| ⭐ **修复汇总与导航** | [DECEMBER_2025_FIX_SUMMARY.md](docs/DECEMBER_2025_FIX_SUMMARY.md) | 10分钟 |

### 问题修复与调试

| 需求 | 推荐阅读 | 时长 |
|-----|--------|------|
| 🔴 倒车Y坐标为0问题 | [REVERSE_MOTION_BUG_ANALYSIS.md](docs/REVERSE_MOTION_BUG_ANALYSIS.md) | 20分钟 |
| 🔴 雅可比矩阵与协方差修复 | [JACOBIAN_COVARIANCE_FIX.md](docs/JACOBIAN_COVARIANCE_FIX.md) | 25分钟 |
| 🔧 卡尔曼滤波诊断 | [KALMAN_FILTER_DEBUGGING.md](docs/KALMAN_FILTER_DEBUGGING.md) | 30分钟 |
| 📊 轨迹可视化坐标系 | [COORDINATE_AXES_VISUALIZATION.md](docs/COORDINATE_AXES_VISUALIZATION.md) | 15分钟 |

### 系统集成与开发

| 需求 | 推荐阅读 | 时长 |
|-----|--------|------|
| 了解理论 | [docs/low_speed_localization_guide.md](docs/low_speed_localization_guide.md) | 20分钟 |
| Python原型 | [scripts/localization_system.py](scripts/localization_system.py) | 10分钟 |
| C集成开发 | [docs/INTEGRATION_GUIDE.md](docs/INTEGRATION_GUIDE.md) | 30分钟 |
| 代码详解 | [docs/C_IMPLEMENTATION_GUIDE.md](docs/C_IMPLEMENTATION_GUIDE.md) | 40分钟 |
| 快速示例 | [example_usage.c](example_usage.c) | 5分钟 |

---

**开始使用**: 
- 🐛 **遇到问题？** 先查看上方"问题修复与调试"部分的相关文档
- 🚀 **开发应用？** 参考下方"系统集成与开发"部分选择合适的文档
- 📚 **完整学习？** 按推荐阅读顺序逐个阅读
