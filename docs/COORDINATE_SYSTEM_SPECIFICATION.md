/**
 * @file COORDINATE_SYSTEM_SPECIFICATION.md
 * @brief 车辆定位系统 - 坐标系详细说明
 * @date 2025-12-17
 * 
 * 本文档提供车辆定位系统中使用的所有坐标系的完整定义、
 * 转换关系、数据流向和应用示例。
 */

# 车辆定位系统坐标系规范

## 📋 目录

1. [全局坐标系 (Global/World Frame)](#1-全局坐标系--globalworld-frame)
2. [车体坐标系 (Body Frame)](#2-车体坐标系--body-frame)
3. [传感器坐标系 (Sensor Frame)](#3-传感器坐标系--sensor-frame)
4. [坐标系转换关系](#4-坐标系转换关系)
5. [数据流向](#5-数据流向)
6. [实现细节](#6-实现细节)
7. [应用示例](#7-应用示例)

---

## 1. 全局坐标系 (Global/World Frame)

### 定义

全局坐标系是**固定在地球表面**的参考坐标系，用于描述车辆的绝对位置和姿态。

```
          Y (北向)
          ↑
          |
    ------+------ X (东向)
          |
          O
```

### 坐标系特征

| 属性 | 说明 |
|------|------|
| **原点** | 初始化时的车辆位置 (默认: 0,0) |
| **X轴** | 东向（向右） |
| **Y轴** | 北向（向上） |
| **Z轴** | 垂直向上（本系统为2D，忽略） |
| **角度原点** | 北向 (θ = 0 表示指向北方) |
| **角度方向** | 逆时针为正 |

### 数据范围

```c
/* 在 localization_types.h 中定义 */
typedef struct {
    float x;      /* 全局坐标系 X 位置 (m) */
    float y;      /* 全局坐标系 Y 位置 (m) */
    float theta;  /* 全局坐标系中的航向角 (rad) 
                     范围: [-π, π]
                     0   = 北向
                     π/2 = 东向
                     π   = 南向
                    -π/2 = 西向 */
    float v;      /* 前进方向的速度 (m/s) */
    float delta;  /* 前轮转向角 (rad) */
    uint32_t timestamp;
} VehicleState_t;
```

### 角度规范化

```c
/**
 * @brief 角度规范化函数
 * 将任意角度转换到 [-π, π] 范围内
 */
static inline double normalize_angle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}
```

**例子：**
- `θ = 3π` → 规范化后 `θ = π`
- `θ = 0.5π` 保持不变
- `θ = -2.5π` → 规范化后 `θ = -0.5π`

---

## 2. 车体坐标系 (Body Frame)

### 定义

车体坐标系**固定在车辆上**，随车辆运动而旋转。

```
        车前方 (前进方向)
          ↑
          |  x_body (车纵向)
    ------+------
          |
         车中心
         
      y_body (车横向) →
```

### 坐标系特征

| 属性 | 说明 |
|------|------|
| **原点** | 车辆的中心 (通常是后轮轴中心) |
| **X轴** | 车纵向（指向车前方） |
| **Y轴** | 车横向（指向车左方） |
| **角速率** | 绕Z轴旋转 (yaw rate) |

### 车辆参数

```c
/* 在 localization_types.h 中定义 */
#define WHEELBASE 2.7  /* 轴距 (m) - 前后轮间距 */
```

### 传感器在车体坐标系中的位置

| 传感器 | 位置 |
|--------|------|
| 后轮轮速计 | X = 0, Y = 0 (基准点) |
| 前轮转向角 | X = WHEELBASE (2.7 m 前方) |
| IMU | 通常在车中心，提供加速度和角速率 |

---

## 3. 传感器坐标系 (Sensor Frame)

### 3.1 IMU坐标系

IMU 通常在车体坐标系中安装，输出加速度和角速率。

```c
typedef struct {
    float accel_x;    /* 车纵向加速度 (m/s²) */
    float accel_y;    /* 车横向加速度 (m/s²) */
    float accel_z;    /* 竖直加速度 (m/s²) - 通常 ≈ 9.81 */
    float gyro_x;     /* 横滚角速率 (rad/s) */
    float gyro_y;     /* 俯仰角速率 (rad/s) */
    float gyro_z;     /* 偏航角速率 (rad/s) - **关键** */
    uint32_t timestamp;
} IMUData_t;
```

**重要：** 系统主要使用 `gyro_z`（偏航角速率），用于估计车辆的角速度。

### 3.2 轮速计坐标系

轮速计提供的是沿车纵向的速度（前进/后退）。

```c
typedef struct {
    float velocity;   /* 车纵向速度 (m/s) 
                         正值：前进
                         负值：后退 */
    float distance;   /* 累积行驶距离 (m) */
    uint32_t timestamp;
} WheelOdometryData_t;
```

### 3.3 转向角传感器

转向角传感器测量前轮相对于车纵向的转角。

```c
typedef struct {
    float steering_angle;  /* 前轮转向角 (rad)
                              正值：左转 (逆时针)
                              负值：右转 (顺时针) */
    float steering_rate;   /* 转向角速率 (rad/s) */
    uint32_t timestamp;
} SteeringData_t;
```

---

## 4. 坐标系转换关系

### 4.1 车体到全局坐标系的转换

**原理：** 将车体坐标系中的位移转换到全局坐标系。

```c
/**
 * @brief 单轨运动模型 - 车体运动方程
 * 
 * 状态向量 (全局坐标系):
 *   x_global[k+1] = x_global[k] + v * cos(θ + δ_eff) * dt
 *   y_global[k+1] = y_global[k] + v * sin(θ + δ_eff) * dt
 *   θ[k+1] = θ[k] + (v * tan(δ_eff) / L) * dt
 *   v[k+1] = v[k]
 *   δ[k+1] = δ[k]
 *
 * 其中:
 *   v           = 速度 (m/s)
 *   θ           = 全局航向角 (rad)
 *   δ           = 转向角 (rad)
 *   δ_eff       = sign(v) * δ (后退时转向反向)
 *   L           = WHEELBASE (轴距)
 *   dt          = 时间间隔 (s)
 *   sign(v)     = +1 (前进) 或 -1 (后退)
 */

/* 代码实现 */
VehicleState_t motion_model_predict(const VehicleState_t *state,
                                    double velocity,
                                    double steering_angle,
                                    double dt) {
    VehicleState_t new_state = *state;
    
    double sign_v = sign_func(velocity);
    double abs_v = fabs(velocity);
    double effective_delta = sign_v * steering_angle;
    
    /* 位置更新 (全局坐标系) */
    double cos_term = cos(state->theta + effective_delta);
    double sin_term = sin(state->theta + effective_delta);
    
    new_state.x = state->x + abs_v * cos_term * dt;
    new_state.y = state->y + abs_v * sin_term * dt;
    
    /* 航向更新 */
    double tan_delta = tan(effective_delta);
    new_state.theta = state->theta + (sign_v * abs_v * tan_delta / WHEELBASE) * dt;
    new_state.theta = normalize_angle(new_state.theta);
    
    return new_state;
}
```

### 4.2 IMU测量到全局坐标系的转换

IMU 在车体坐标系中测量加速度，需要转换到全局坐标系：

```
全局加速度 = 旋转矩阵 × 车体加速度

[a_x_global]   [cos(θ)  -sin(θ)] [a_x_body]
[a_y_global] = [sin(θ)   cos(θ)] [a_y_body]

其中 θ 是当前航向角
```

**代码中的应用：** 卡尔曼滤波器中的测量模型。

### 4.3 转向角约束

单轨模型假设：
- 前轮转向，后轮固定
- 转向角范围限制：`|δ| ≤ 0.628 rad ≈ 36°`

```c
#define STEERING_ANGLE_MAX 0.628  /* 最大转向角 */

/* 检查转向角是否有效 */
if (fabs(steering_angle) > STEERING_ANGLE_MAX) {
    steering_angle = clamp(steering_angle, -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX);
}
```

---

## 5. 数据流向

### 5.1 传感器数据流

```
┌─────────────────────────────────────────┐
│         硬件传感器                      │
├─────────────────────────────────────────┤
│ • 轮速计 → velocity (车体坐标系)       │
│ • IMU → accel_x, accel_y, gyro_z       │
│ • 转向角传感器 → steering_angle        │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│  传感器数据融合 (SensorReadings_t)     │
├─────────────────────────────────────────┤
│ • 时间同步                             │
│ • 数据验证                             │
│ • 异常检测                             │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│  卡尔曼滤波器 (EKF)                    │
├─────────────────────────────────────────┤
│ 1. 预测步 (motion model):              │
│    • 使用运动模型预测状态              │
│    • 基于轮速计速度                   │
│                                        │
│ 2. 更新步 (measurement update):        │
│    • 融合 IMU、转向角测量              │
│    • 计算卡尔曼增益                   │
│    • 估计最优状态                     │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│  估计状态 (VehicleState_t)            │
├─────────────────────────────────────────┤
│ • x, y: 全局位置 (全局坐标系)         │
│ • theta: 全局航向角                   │
│ • v: 前进速度                         │
│ • delta: 转向角                       │
└─────────────────────────────────────────┘
```

### 5.2 数据坐标系说明

| 数据 | 坐标系 | 说明 |
|------|--------|------|
| `velocity` | 车体 | 沿车纵向 (+前/-后) |
| `accel_x`, `accel_y` | 车体 | 车纵向/横向加速度 |
| `gyro_z` | 车体 | 绕 Z 轴角速率 (偏航) |
| `steering_angle` | 车体 | 前轮转角 |
| `x`, `y` | 全局 | 绝对位置 (东/北) |
| `theta` | 全局 | 绝对航向角 (北向=0) |

---

## 6. 实现细节

### 6.1 运动模型的关键代码

见 [motion_model.c](motion_model.c) 中的 `motion_model_predict()` 函数。

**关键点：**
1. **单轨模型** - 假设前轮驱动/后轮固定
2. **支持前后退** - 通过 `sign(velocity)` 处理
3. **转向反向** - 后退时 `δ_eff = -δ`

### 6.2 卡尔曼滤波中的坐标系

见 [kalman_filter.c](kalman_filter.c)。

**状态向量：** 全局坐标系
```
x_hat = [x, y, theta, v, delta]^T
```

**测量向量：** 传感器坐标系 → 转换到全局
```
z = [v_measured, a_measured, delta_measured]^T
```

**雅可比矩阵：** 描述全局状态对车体运动的敏感性
```
F[5][5] = ∂x_global / ∂x_state
```

### 6.3 坐标系规范化

```c
/**
 * @brief 将角度限制在 [-π, π] 范围
 */
static inline double normalize_angle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

/**
 * @brief 将值限制在 [min, max] 范围
 */
static inline double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
```

---

## 7. 应用示例

### 7.1 泊车轨迹示例

```
起始位置：(0, 0), θ = 0°

阶段 1：直行 100 m (北向)
  • v = 1 m/s, δ = 0
  • 结果：(0, 100), θ = 0°

阶段 2：向左转向后退 30 m
  • v = -0.5 m/s (后退), δ = π/6 (向左)
  • δ_eff = -π/6 (后退时转向反向)
  • 结果：(约 -15, 85), θ ≈ π/2

阶段 3：停止
  • v = 0
```

### 7.2 代码使用示例

```c
#include "localization_system.h"

int main() {
    // 初始化在原点，航向北向 (θ=0)
    VehicleState_t init_state = {
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,  // 北向
        .v = 0.0f,
        .delta = 0.0f
    };
    
    LocalizationSystem_t sys;
    localization_system_init(&sys, &init_state);
    
    // 获取当前状态 (全局坐标系)
    VehicleState_t state;
    localization_system_get_state(&sys, &state);
    
    printf("全局位置: (%.2f, %.2f) m\n", state.x, state.y);
    printf("全局航向: %.2f° (北向 = 0°)\n", RAD_TO_DEG(state.theta));
    printf("速度: %.2f m/s\n", state.v);
    printf("转向角: %.2f°\n", RAD_TO_DEG(state.delta));
    
    return 0;
}
```

### 7.3 坐标转换验证

```c
/**
 * 验证：从全局坐标恢复车体运动
 */

// 已知：全局位移 Δx_global, Δy_global, Δθ_global
// 求解：对应的车体运动 v, δ

// 逆向计算
double distance_global = sqrt(dx_global*dx_global + dy_global*dy_global);
double angle_global = atan2(dy_global, dx_global);

// 相对于车前进方向的转角
double relative_angle = angle_global - current_theta;

// 如果 relative_angle 接近 0，则直线运动
// 如果 relative_angle != 0，则进行了转向

printf("全局运动：向%.1f°方向移动%.2f m\n",
       RAD_TO_DEG(angle_global), distance_global);
```

---

## 📐 坐标系总结表

### 常见坐标系转换

| 转换 | 方法 | 用途 |
|------|------|------|
| 车体 → 全局 | 旋转矩阵 (R(θ)) | 位置和速度 |
| 全局 → 车体 | 反向旋转矩阵 (R(θ)^T) | 检测车体相对位移 |
| 角度标准化 | 模 2π 运算 | 保持角度在 [-π, π] |
| 速度分解 | v_x = v·cos(θ), v_y = v·sin(θ) | 全局速度分量 |

### 单位约定

| 量 | 单位 | 范围 |
|----|------|------|
| 位置 (x, y) | 米 (m) | ≥ 0 (任意距离) |
| 航向角 (θ) | 弧度 (rad) | [-π, π] |
| 速度 (v) | 米/秒 (m/s) | [-2.0, 2.0] |
| 转向角 (δ) | 弧度 (rad) | [-0.628, 0.628] |
| 加速度 | 米/平方秒 (m/s²) | ±限制 |
| 时间 | 毫秒 (ms) | ≥ 0 |

---

## 🔗 相关文件

- [localization_types.h](localization_types.h) - 数据类型定义
- [motion_model.c](motion_model.c) - 运动模型实现
- [kalman_filter.c](kalman_filter.c) - 卡尔曼滤波实现
- [example_usage.c](example_usage.c) - 完整使用示例

---

**文档版本：** 1.0  
**最后更新：** 2025-12-17  
**作者：** 车辆定位系统开发团队

