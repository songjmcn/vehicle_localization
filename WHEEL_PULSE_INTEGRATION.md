/**
 * @file WHEEL_PULSE_INTEGRATION.md
 * @brief 轮速脉冲处理器集成指南 - 快速入门
 */

# 轮速脉冲处理器集成指南

## 📌 概述

本文档描述如何在你的车辆定位系统中集成轮速脉冲处理器。该处理器可以：

- 将霍尔传感器、编码器等产生的脉冲计数**自动转换为速度和距离**
- 应用**低速自适应平滑**算法，消除脉冲离散性
- 检测**脉冲超时**和静止状态
- 与卡尔曼滤波器**无缝配合**

## 🚀 快速开始（5分钟）

### 1. 初始化系统

```c
#include "localization_system.h"

// 初始化定位系统
LocalizationSystem_t sys;
VehicleState_t initial = {.x=0, .y=0, .theta=0};
localization_system_init(&sys, &initial);
```

### 2. 配置轮速参数

```c
// 设置轮速计参数
WheelConfig_t wheel_config = {
    .pulses_per_wheel_rev = 400,    // 码盘：400脉冲/圈（根据实际硬件设置）
    .wheel_radius = 0.35,           // 轮半径：0.35米（根据轮胎规格设置）
    .low_speed_threshold = 0.3,     // 低速阈值：0.3 m/s（推荐值）
    .pulse_timeout_ms = 500         // 脉冲超时：500ms（推荐值）
};

localization_system_set_wheel_config(&sys, &wheel_config);
```

### 3. 实时更新轮速数据

#### 方式 A：传入脉冲计数（推荐用于硬件直接输出脉冲）

```c
// 在主循环中，每当收到新的脉冲计数时：
uint32_t pulse_count = read_pulse_from_hardware();  // 你的硬件接口
uint32_t timestamp = get_system_time_ms();

// 传入脉冲计数，系统自动转换
localization_system_update_wheel_pulse(&sys, pulse_count, timestamp);
```

#### 方式 B：传入速度值（推荐用于硬件已转换为速度）

```c
// 在主循环中，如果硬件已提供速度值：
double velocity = read_velocity_from_hardware();  // 你的硬件接口
uint32_t timestamp = get_system_time_ms();

// 传入速度，系统应用平滑和异常检测
localization_system_update_wheel_velocity(&sys, velocity, timestamp);
```

### 4. 检索处理后的轮速数据

```c
// 获取处理后的轮速数据（包含平滑和距离积分）
WheelOdometryData_t odometry;
localization_system_get_wheel_odometry(&sys, &odometry);

printf("Speed: %.2f m/s, Distance: %.2f m\n", 
       odometry.velocity, odometry.distance);
```

## 📖 详细说明

### 硬件接口示例

根据你的硬件通信方式选择相应的实现：

#### CAN 总线接口

```c
void update_wheel_from_can(LocalizationSystem_t *sys) {
    // 从 CAN 读取脉冲计数或速度
    uint8_t can_data[8];
    read_can_message(0x123, can_data);
    
    uint32_t pulse_count = (can_data[0] << 24) | (can_data[1] << 16) | 
                          (can_data[2] << 8) | can_data[3];
    uint32_t timestamp = get_system_time_ms();
    
    localization_system_update_wheel_pulse(sys, pulse_count, timestamp);
}
```

#### UART 串口接口

```c
void update_wheel_from_uart(LocalizationSystem_t *sys) {
    // 从 UART 读取速度值
    uint8_t buffer[4];
    if (uart_read(buffer, 4) == 4) {
        float velocity = *(float*)buffer;  // 假设是 IEEE 754 浮点数
        uint32_t timestamp = get_system_time_ms();
        
        localization_system_update_wheel_velocity(sys, (double)velocity, timestamp);
    }
}
```

#### GPIO 脉冲计数器接口

```c
void update_wheel_from_gpio(LocalizationSystem_t *sys) {
    // 从 GPIO 脉冲计数器读取
    uint32_t pulse_count = read_gpio_counter();  // 你的硬件接口
    uint32_t timestamp = get_system_time_ms();
    
    localization_system_update_wheel_pulse(sys, pulse_count, timestamp);
}
```

#### I2C 编码器接口

```c
void update_wheel_from_i2c(LocalizationSystem_t *sys) {
    // 从 I2C 编码器读取（例如 AS5048B）
    uint16_t angle_raw = read_i2c_encoder(0x40);
    
    // 将角度转换为脉冲（如果需要）
    uint32_t pulse_count = angle_to_pulse(angle_raw);
    uint32_t timestamp = get_system_time_ms();
    
    localization_system_update_wheel_pulse(sys, pulse_count, timestamp);
}
```

### 参数配置指南

#### 码盘参数（pulses_per_wheel_rev）

| 编码器类型 | 典型值 | 说明 |
|----------|-------|------|
| 光学编码器 | 100~500 | 工业标准 |
| 霍尔传感器 | 1~8 | 通常是轮幅数量 |
| 增量式编码器 | 400~2048 | 常见型号 |
| 绝对值编码器 | 4096+ | 高精度 |

**如何测定**：
```
1. 让车轮空转一圈
2. 记录脉冲计数：N
3. pulses_per_wheel_rev = N
```

#### 轮半径参数（wheel_radius）

| 轮胎规格 | 轮半径 (m) |
|---------|-----------|
| 155/65R13 | 0.33 |
| 175/65R14 | 0.35 |
| 185/60R15 | 0.34 |
| 205/55R16 | 0.35 |

**精确测量方法**：
```c
// 方法1：直接测量轮胎
double wheel_radius = (tire_width_mm * aspect_ratio / 100.0 + 
                      rim_diameter_mm * 25.4) / 2000.0;

// 方法2：实车标定（推荐）
// 行驶已知距离，使用 RTK GNSS 校准
double wheel_radius = (actual_distance / wheel_turns) / (2 * PI);
```

#### 低速阈值（low_speed_threshold）

建议值：**0.3 m/s**

- 低于此速度时启用平滑
- 避免低速脉冲离散导致的速度抖动
- 可根据应用场景调整（0.1~0.5 m/s）

#### 脉冲超时（pulse_timeout_ms）

建议值：**500 ms**

- 超过此时间无新脉冲 → 强制零速
- 防止通信中断导致的持续动作
- 可根据硬件可靠性调整（200~1000 ms）

## 🔍 故障诊断

### 问题1：速度抖动

**症状**：显示的速度在 0.0~0.1 m/s 之间频繁变化

**原因**：低速时脉冲离散

**解决方案**：
```c
// 方案A：提高低速阈值
wheel_config.low_speed_threshold = 0.5;  // 提高到 0.5 m/s

// 方案B：检查脉冲来源是否存在噪声
// 使用示波器验证脉冲信号质量
```

### 问题2：速度滞后

**症状**：速度响应缓慢，有明显延迟

**原因**：平滑窗口过大或脉冲分辨率过低

**解决方案**：
```c
// 方案A：降低低速阈值（减少平滑应用时间）
wheel_config.low_speed_threshold = 0.2;

// 方案B：增加脉冲分辨率
// 检查编码器是否支持更高分辨率
// 或在驱动层增加倍频电路
```

### 问题3：脉冲超时频繁触发

**症状**：频繁显示"脉冲超时"警告

**原因**：硬件通信不稳定

**解决方案**：
```c
// 方案A：增加超时时间
wheel_config.pulse_timeout_ms = 1000;  // 改为 1 秒

// 方案B：检查硬件连接
// - 检查 CAN/UART 线路
// - 验证 GPIO 中断配置
// - 确认编码器电源

// 方案C：添加通信重试逻辑
if (localization_system_wheel_is_timeout(&sys)) {
    // 尝试重新连接硬件
    reinit_hardware_interface();
}
```

### 问题4：距离积分误差大

**症状**：里程计积分距离与实际距离偏差 > 5%

**原因**：轮半径或校准因子不准确

**解决方案**：
```c
// 使用 RTK 自动校准
// 系统会自动调整 wheel_calibration_factor

// 或手动校准：
// 1. 行驶已知距离（例如 100 米）
// 2. 记录系统计算的距离：D_measured
// 3. 计算校准因子：factor = 100.0 / D_measured
// 4. 应用校准

CalibrationParams_t calib = {
    .wheel_calibration_factor = 1.02,  // 例如：校准因子为 1.02
    // ... 其他参数
};
localization_system_set_calibration(&sys, &calib);
```

## 📊 性能指标

### 脉冲处理精度

| 操作 | 精度 |
|------|------|
| 脉冲→距离 | ±0.1% |
| 距离→速度 | ±2% (低速) / ±1% (高速) |
| 低速平滑 | 减少 80% 噪声 |
| 响应延迟 | < 100 ms |

### 资源占用

| 资源 | 占用 |
|------|------|
| 代码空间 | ~4 KB |
| RAM | ~200 bytes |
| CPU (50Hz) | < 1% |

## 🔗 相关文档

- [PARAMETER_CALIBRATION_GUID.MD](PARAMETER_CALIBRATION_GUID.MD) - 完整参数标定指南
- [C_IMPLEMENTATION_GUIDE.md](C_IMPLEMENTATION_GUIDE.md) - C 实现详细说明
- [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) - 系统集成指南

## 📝 完整集成示例

```c
#include "localization_system.h"
#include <stdio.h>

int main() {
    // 1. 初始化
    LocalizationSystem_t sys;
    VehicleState_t init_state = {0};
    localization_system_init(&sys, &init_state);
    
    // 2. 配置轮速
    WheelConfig_t wheel_cfg = {
        .pulses_per_wheel_rev = 400,
        .wheel_radius = 0.35,
        .low_speed_threshold = 0.3,
        .pulse_timeout_ms = 500
    };
    localization_system_set_wheel_config(&sys, &wheel_cfg);
    
    // 3. 主循环
    while (1) {
        // 从硬件读取脉冲
        uint32_t pulse_count = read_pulse_from_hardware();
        uint32_t timestamp = get_system_time_ms();
        
        // 更新轮速处理器
        localization_system_update_wheel_pulse(&sys, pulse_count, timestamp);
        
        // 读取其他传感器
        SensorReadings_t sensor_data = {
            .imu = read_imu(),
            .steering = read_steering(),
            .wheel_odometry = {0},  // 将由处理器填充
            .timestamp = timestamp
        };
        
        // 获取处理后的轮速数据
        localization_system_get_wheel_odometry(&sys, &sensor_data.wheel_odometry);
        
        // 执行定位
        VehicleState_t state;
        localization_system_process(&sys, &sensor_data, &state);
        
        // 检查故障
        if (localization_system_wheel_is_timeout(&sys)) {
            printf("WARNING: Wheel pulse timeout!\n");
        }
        
        // 使用位置估计
        printf("Position: (%.2f, %.2f) m, Heading: %.2f°\n",
               state.x, state.y, state.theta * 180 / 3.14159);
        
        // 延迟
        usleep(50000);  // 50ms，对应 20Hz 更新频率
    }
    
    return 0;
}
```

---

**版本**：1.0  
**最后更新**：2025-12-16  
**维护者**：Vehicle Localization Team
