# 车辆定位系统 (Vehicle Localization System)

## 📋 项目简介

一套完整的车辆定位解决方案，专门针对**自动泊车场景**（0.5 m/s ~ 2.0 m/s）设计。采用多传感器融合（轮速计、IMU、转向角）和扩展卡尔曼滤波算法，并集成RTK高精度校准功能。

## 🎯 核心特性

- ✅ **多传感器融合**：轮速计 + IMU + 转向角（实时定位）
- ✅ **低速优化**：专为自动泊车场景优化（0.5-2.0 m/s）
- ✅ **前后退支持**：运动模型支持前进和后退场景
- ✅ **RTK离线标定**：基于RTK的离线参数标定功能（非实时融合）
- ✅ **自适应滤波**：轮滑动检测和自适应噪声调整
- ✅ **生产级代码**：C语言实现，嵌入式友好

## 📁 文件结构

```
vehicle_localization/
│
├─ 📘 文档 (docs/)
│  ├─ 快速开始
│  │  ├── COORDINATE_ORIGIN_QUICK_VIEW.md      # 坐标原点 (3秒速查)
│  │  ├── ISO8855_EXECUTIVE_SUMMARY.md         # ISO 8855 执行摘要
│  │  ├── SCHEME_B_IMPLEMENTATION_REPORT.py    # 方案B实现报告
│  │  └── CSV_CALIBRATION_QUICKSTART.md        # CSV标定快速开始
│  │
│  ├─ 协调系统文档
│  │  ├── COORDINATE_SYSTEM_SPECIFICATION.md   # 坐标系规范
│  │  ├── COORDINATE_SYSTEM_QUICK_REFERENCE.md # 坐标系快速参考
│  │  ├── COORDINATE_SYSTEM_DOCUMENTATION_SUMMARY.md
│  │  ├── COORDINATE_ORIGIN_ANALYSIS.md        # 坐标原点详细分析
│  │  └── DOCUMENTATION_GUIDE.md               # 文档导航指南
│  │
│  ├─ ISO 8855标准集成
│  │  ├── ISO8855_COORDINATE_SYSTEM.md         # ISO 8855坐标系定义
│  │  ├── ISO8855_IMPLEMENTATION_PLAN.md       # ISO 8855实施方案
│  │  └── ISO8855_NAVIGATION.md                # ISO 8855文档导航
│  │
│  ├─ 详细技术文档
│  │  ├── low_speed_localization_guide.md      # 低速定位原理
│  │  ├── C_IMPLEMENTATION_GUIDE.md            # C代码实现指南
│  │  ├── INTEGRATION_GUIDE.md                 # 集成部署指南
│  │  ├── WHEEL_PULSE_INTEGRATION.md           # 轮脉冲集成指南
│  │  ├── RTK_OFFLINE_CALIBRATION.md           # RTK离线标定
│  │  ├── OFFLINE_CALIBRATION_TOOL_GUIDE.md    # 离线工具指南
│  │  ├── PARAMETER_CALIBRATION_GUID.md        # 参数标定指南
│  │  └── RTK_REFACTORING_SUMMARY.md           # RTK重构总结
│  │
│  ├─ CMake & 构建
│  │  ├── CMAKE_OVERVIEW.md                    # CMake概述
│  │  ├── CMAKE_QUICK_REFERENCE.md             # CMake快速参考
│  │  ├── CMAKE_BUILD_GUIDE.md                 # 构建指南
│  │  ├── CMAKE_CHECKLIST.md                   # 构建检查清单
│  │  ├── CMAKE_STRUCTURE.md                   # CMake结构
│  │  └── CMAKE_vs_MAKEFILE.md                 # CMake vs Makefile
│  │
│  ├─ 其他文档
│  │  ├── FILE_MANIFEST.md                     # 文件清单
│  │  ├── PROJECT_COMPLETION_SUMMARY.md        # 项目完成总结
│  │  └── INTEGRATION_GUIDE.md                 # 集成指南
│  │
├─ 🐍 Python参考实现（原型开发）
│  └── localization_system.py          # Python EKF实现
│
├─ 🔧 C生产实现（嵌入式部署）
│  ├── localization_types.h            # 数据类型定义
│  ├── motion_model.h/c                # 运动学模型
│  ├── kalman_filter.h/c               # 扩展卡尔曼滤波
│  ├── rtk_calibration.h/c             # RTK标定模块
│  ├── localization_system.h/c         # 高层系统接口
│  ├── wheel_odometry_processor.h/c    # 轮脉冲处理
│  ├── example_usage.c                 # 使用示例程序
│  └── Makefile                        # 编译配置
│
├─ 🛠️ 离线标定工具（CSV处理）
│  ├── generate_calibration_csv.c      # 测试数据生成工具
│  └── offline_calibration.c           # 离线参数计算工具
│
└─ 📚 其他
   └── localization_system_private.h   # 私有头文件
```

## 🚀 快速开始

### 方案A：Python快速原型开发

```bash
# 检查Python实现
cat localization_system.py

# 查看理论指南
cat low_speed_localization_guide.md
```

### 方案B：C生产级部署（CMake - 推荐）⭐

```bash
# 一行命令快速开始
./cmake_build.sh build && ./cmake_build.sh run

# 或分步骤操作
./cmake_build.sh build         # 编译（Release）
./cmake_build.sh build-debug   # Debug编译（带调试符号）
./cmake_build.sh run           # 运行程序
./cmake_build.sh clean         # 清除编译产物
./cmake_build.sh rebuild       # 清除后重新编译
./cmake_build.sh help          # 查看所有命令
```

**CMake优势**：
- ✅ 跨平台支持（macOS/Linux/Windows）
- ✅ IDE完整集成（VS Code、CLion、Visual Studio）
- ✅ 灵活的编译配置
- ✅ 自动并行编译

👉 详见 [docs/CMAKE_QUICK_REFERENCE.md](docs/CMAKE_QUICK_REFERENCE.md) 快速参考卡片

### 方案C：传统Makefile（向后兼容）

```bash
# 编译系统
make clean
make

# 运行示例（模拟J形泊车轨迹）
make run

# 仅编译特定模块
make test_motion
make test_filter
make test_rtk

# 清理构建
make clean
```

👉 CMake 和 Makefile 详细对比见 [docs/CMAKE_vs_MAKEFILE.md](docs/CMAKE_vs_MAKEFILE.md)

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

## 🧮 核心算法

### 运动模型（支持前后退）

$$x_{k+1} = x_k + |v| \cos(\theta_k + \text{sign}(v)\delta) \Delta t$$
$$y_{k+1} = y_k + |v| \sin(\theta_k + \text{sign}(v)\delta) \Delta t$$
$$\theta_{k+1} = \theta_k + \frac{\text{sign}(v)|v|\tan(\delta)}{L} \Delta t$$

### 状态向量
$$\mathbf{x} = [x, y, \theta, v, \delta]^T$$
- x, y：车辆位置
- θ：航向角
- v：速度
- δ：转向角

### 卡尔曼滤波
- **预测**：基于运动模型预测下一状态
- **更新**：融合传感器测量进行校正

## 🔧 使用示例

### C语言集成

```c
#include "localization_system.h"

int main() {
    // 1. 初始化系统
    LocalizationSystem_t sys;
    VehicleState_t init_state = {.x=0, .y=0, .theta=0};
    localization_system_init(&sys, &init_state);
    
    // 2. 主循环
    while (1) {
        // 读取传感器
        SensorReadings_t readings = get_sensor_data();
        
        // 执行定位
        VehicleState_t state;
        localization_system_process(&sys, &readings, &state);
        
        printf("Pos: (%.2f, %.2f), Heading: %.2f°\n", 
               state.x, state.y, RAD_TO_DEG(state.theta));
        
        // RTK离线校准（可选）
        // 收集里程计和RTK数据进行离线标定
        if (has_rtk_data()) {
            RTKData_t rtk = read_rtk();
            VehicleState_t odometry = localization_system_get_state(&sys);  // 获取里程计估计
            localization_system_rtk_calibration_add_point(&sys, &odometry, &rtk);
        }
    }
    
    // 3. 清理
    localization_system_free(&sys);
    return 0;
}
```

### Python算法验证

```python
from localization_system import LowSpeedLocalizationSystem, SensorReading

# 初始化
system = LowSpeedLocalizationSystem()

# 处理传感器数据
reading = SensorReading(
    wheel_velocity=1.0,
    imu_accel_x=0.05,
    imu_accel_y=0.05,
    imu_gyro_z=0.01,
    steering_angle=0.1,
    timestamp=1000
)

state = system.run_step(reading)
print(f"位置: ({state.x:.2f}, {state.y:.2f}) m")
print(f"航向: {state.theta:.2f} rad")
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
- [CMAKE_QUICK_REFERENCE.md](CMAKE_QUICK_REFERENCE.md) - 快速参考卡片（5分钟）
- [CMAKE_STRUCTURE.md](CMAKE_STRUCTURE.md) - 工程结构说明（15分钟）
- [CMAKE_BUILD_GUIDE.md](CMAKE_BUILD_GUIDE.md) - 详细使用指南（20分钟）✨
- [CMAKE_vs_MAKEFILE.md](CMAKE_vs_MAKEFILE.md) - 工具对比分析（10分钟）
- [CMAKE_CHECKLIST.md](CMAKE_CHECKLIST.md) - 完成检查清单（10分钟）
- [CMAKE_OVERVIEW.md](CMAKE_OVERVIEW.md) - 总体概览（5分钟）

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

👉 详见 [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)

### 🔧 参数标定和调参指南（重要！）⭐

#### 离线CSV标定工具 🆕

**快速标定工具链**（包含数据生成和参数计算）：

- [CSV_CALIBRATION_QUICKSTART.md](CSV_CALIBRATION_QUICKSTART.md) - **快速开始指南** 
  - 3步完成标定
  - CSV格式说明
  - 工具使用示例

- **工具组件**：
  - `generate_calibration_csv` - 生成模拟标定数据
  - `offline_calibration` - 离线参数计算

#### 完整的参数标定指南

- [PARAMETER_CALIBRATION_GUID.MD](PARAMETER_CALIBRATION_GUID.md) - **2500行详细指南** ✨
  
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

👉 详见 [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) 中的故障诊断部分

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
- **最后更新**：2024年12月
- **维护者**：Vehicle Localization Team

## 📄 许可证

本项目提供参考实现，可自由使用和修改。

---

## 🤝 快速导航

| 需求 | 推荐阅读 |
|-----|--------|
| 了解理论 | [low_speed_localization_guide.md](low_speed_localization_guide.md) |
| Python原型 | [localization_system.py](localization_system.py) |
| C集成开发 | [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) |
| 代码详解 | [C_IMPLEMENTATION_GUIDE.md](C_IMPLEMENTATION_GUIDE.md) |
| 快速示例 | [example_usage.c](example_usage.c) |

---

**开始使用**: 根据你的需求选择Python或C实现，参考对应的集成指南进行开发。
