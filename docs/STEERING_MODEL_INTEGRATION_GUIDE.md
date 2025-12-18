# 转向角转换模块集成指南

## 概述

转向模块 (`steering_model.h`/`steering_model.c`) 负责将方向盘转角（steering wheel angle）转换为前轮转向角（front wheel steering angle），并计算车辆航向角速率（yaw rate）。

## 模块架构

```
┌─────────────────────────────────────────────┐
│   驾驶员输入或传感器数据                     │
│   (方向盘转角 steering_wheel_angle)         │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────┐
│    转向模型 (steering_model.c)               │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │ 步骤1: 齿比转换                        │ │
│  │ δ_front = δ_wheel / steering_ratio    │ │
│  └────────────────────────────────────────┘ │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │ 步骤2: 零偏和增益补偿                  │ │
│  │ δ_comp = (δ_front - offset) * gain    │ │
│  └────────────────────────────────────────┘ │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │ 步骤3: 低通滤波                        │ │
│  │ δ_filtered = α*δ_prev + (1-α)*δ_comp │ │
│  └────────────────────────────────────────┘ │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │ 步骤4: 航向角速率计算                  │ │
│  │ ω = v * tan(δ) / L                    │ │
│  └────────────────────────────────────────┘ │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────┐
│   运动模型 (motion_model.c)                   │
│                                              │
│   ▼ 前轮转向角 δ_filtered                    │
│   ▼ 航向角速率 ω                             │
│                                              │
│   用于二轮运动学模型预测                     │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────┐
│   卡尔曼滤波器 (kalman_filter.c)              │
│                                              │
│   预测步骤中使用转向角和航向角速率           │
└──────────────────────────────────────────────┘
```

## 关键功能

### 1. 转向角转换

```c
/* 方向盘转角转前轮转向角 */
double steering_wheel_to_front_wheel(double steering_wheel_angle);

/* 前轮转向角转方向盘转角 */
double front_wheel_to_steering_wheel(double front_wheel_angle);
```

**转换公式：**
- 前轮转角 = 方向盘转角 ÷ 齿比（通常 16:1 或 20:1）
- 转向齿比是关键参数，不同车型值不同

### 2. 航向角速率计算

```c
/* 基于二轮运动学模型计算偏航角速率 */
double calculate_yaw_rate(double velocity, double steering_angle);
```

**公式：**
$$\omega = \frac{v \cdot \tan(\delta)}{L}$$

其中：
- $\omega$ = 航向角速率（偏航角速率）rad/s
- $v$ = 纵向速度 m/s
- $\delta$ = 前轮转向角 rad
- $L$ = 轴距 m

### 3. 转向角更新和滤波

```c
/* 更新转向角，返回滤波后的值 */
double steering_model_update(double steering_wheel_angle, uint32_t timestamp);
```

**功能：**
- 齿比转换
- 零偏和增益补偿
- 一阶低通滤波（可选）
- 转向角速率计算
- 时间戳跟踪

### 4. 参数标定和补偿

```c
/* 应用零偏和增益补偿 */
double steering_model_apply_calibration(double angle);

/* 更新配置参数 */
int steering_model_update_config(const SteeringConfig_t *config);
```

## 配置参数

### SteeringConfig_t 结构

```c
typedef struct {
    float steering_ratio;           /* 方向盘齿比（16-20 通常）*/
    float max_steering_wheel_angle; /* 最大方向盘转角（±π）*/
    float max_front_wheel_angle;    /* 最大前轮转角（±0.628 rad）*/
    float wheelbase;                /* 车轴距（m）*/
    float steering_angle_offset;    /* 转向零偏（标定值）*/
    float steering_angle_gain;      /* 转向增益（标定值）*/
    bool enable_steering_filter;    /* 是否启用低通滤波 */
    float steering_filter_alpha;    /* 低通滤波系数（0-1）*/
} SteeringConfig_t;
```

### 典型值

| 参数 | 典型值 | 范围 | 说明 |
|------|--------|------|------|
| steering_ratio | 16.0 | 14-20 | 方向盘转圈数/轮转角度 |
| max_steering_wheel_angle | 3.14159 | - | ±180° |
| max_front_wheel_angle | 0.628 | 0.436-0.785 | ±25°-45° |
| wheelbase | 2.7 | 2.5-3.5 | 根据车型 |
| steering_angle_offset | 0.0 | ±0.1 | 标定得出 |
| steering_angle_gain | 1.0 | 0.95-1.05 | 标定得出 |
| steering_filter_alpha | 0.15 | 0.1-0.3 | 滤波强度 |

## 集成步骤

### 1. 初始化阶段

```c
/* 在 motion_model_init() 中自动初始化 */
motion_model_init();

/* 或者在系统初始化时手动初始化 */
SteeringConfig_t config = {
    .steering_ratio = 16.0f,
    .wheelbase = 2.7f,
    /* 其他参数... */
};
steering_model_init(&config);
```

### 2. 运动模型中使用

```c
/* 在预测函数中，转向角被用于二轮模型 */
VehicleState_t new_state = motion_model_predict(
    &current_state,
    velocity,
    steering_angle,  /* 这是前轮转向角 */
    dt
);
```

### 3. 实时转向角更新

```c
/* 当接收到驾驶员方向盘输入时 */
double steering_wheel_angle = get_steering_wheel_angle();
uint32_t timestamp = get_system_time_ms();

/* 更新转向模型，获得滤波后的前轮转角 */
double filtered_front_wheel_angle = steering_model_update(
    steering_wheel_angle, 
    timestamp
);

/* 计算航向角速率 */
double yaw_rate = calculate_yaw_rate(velocity, filtered_front_wheel_angle);

/* 传递给卡尔曼滤波器使用 */
kalman_filter_predict(&state, velocity, filtered_front_wheel_angle, dt);
```

### 4. 获取转向状态

```c
/* 获取滤波后的前轮转向角 */
double filtered_angle = steering_model_get_filtered_angle();

/* 获取转向角速率 */
double steering_rate = steering_model_get_steering_rate();
```

## 流程示例

### 单步循环中的处理流程

```
主循环周期 (dt = 0.01s)
│
├─ 读取传感器数据
│  ├─ 车速: v = 0.5 m/s
│  ├─ 方向盘角: steering_wheel = 0.314 rad (18°)
│  └─ 时间戳: t = 1000ms
│
├─ 转向角处理
│  ├─ 齿比转换: δ_front = 0.314 / 16 = 0.01963 rad (1.125°)
│  ├─ 补偿: 无 (offset=0, gain=1)
│  ├─ 滤波: α=0.15
│  │  δ_filtered = 0.15*δ_prev + 0.85*0.01963
│  └─ 结果: δ_filtered ≈ 0.0167 rad (0.957°)
│
├─ 航向角速率计算
│  ├─ ω = v * tan(δ) / L
│  ├─ ω = 0.5 * tan(0.0167) / 2.7
│  └─ ω ≈ 0.00308 rad/s (很小，典型停车场场景)
│
├─ 运动模型预测
│  ├─ Δx = 0.5 * cos(θ + 0.0167) * 0.01
│  ├─ Δy = 0.5 * sin(θ + 0.0167) * 0.01
│  ├─ Δθ = 0.00308 * 0.01
│  └─ 更新位置和姿态
│
└─ 卡尔曼滤波器预测
   └─ 使用更新后的状态和转向角
```

## 标定指南

### 转向比标定

1. **静态测试**
   - 方向盘转至极限位置（左/右）
   - 测量前轮转向角
   - 计算: 齿比 = 方向盘转角 / 前轮转角

2. **动态验证**
   - 在停车场做圆圈测试
   - 验证转向半径是否符合预期
   - 调整转向比如有偏差

### 零偏标定

1. **直线走行**
   - 在标记的直线上行驶（无任何转向输入）
   - 如果车辆偏离直线，则存在零偏

2. **补偿计算**
   - offset = 观测到的偏转角
   - 使其恢复直线

### 增益标定

1. **角度对应关系**
   - 多个不同方向盘角度下测试
   - 验证前轮角是否与预期成正比

2. **增益调整**
   - 如果前轮转角过大：gain < 1
   - 如果前轮转角过小：gain > 1

## 常见场景

### 场景1: 停车场精确定位

```c
/* 低速，转向角小，需要高精度 */
SteeringConfig_t config = {
    .steering_filter_alpha = 0.15f,  /* 较弱滤波，保留细节 */
    .enable_steering_filter = true,
};

/* 高频更新转向角 */
steering_model_update(steering_wheel_angle, get_time_ms());
```

### 场景2: 高速转向

```c
/* 高速，转向输入可能嘈杂，需要滤波 */
SteeringConfig_t config = {
    .steering_filter_alpha = 0.25f,  /* 较强滤波，平滑输入 */
    .enable_steering_filter = true,
};
```

### 场景3: 转向角直接应用

```c
/* 无需滤波，信号已过滤 */
SteeringConfig_t config = {
    .enable_steering_filter = false,
};

double front_angle = steering_wheel_to_front_wheel(wheel_angle);
```

## 测试验证

### 单元测试

```c
/* test_steering_model.c */

/* 测试1: 齿比转换 */
assert(steering_wheel_to_front_wheel(16.0) ≈ 1.0);

/* 测试2: 航向角速率 */
double yaw_rate = calculate_yaw_rate(1.0, 0.1);
assert(yaw_rate > 0);

/* 测试3: 低通滤波 */
steering_model_update(0.1, 0);
double filtered1 = steering_model_get_filtered_angle();
steering_model_update(0.2, 10);
double filtered2 = steering_model_get_filtered_angle();
assert(filtered2 > filtered1 && filtered2 < 0.2);
```

### 集成测试

在实际车辆上验证：
1. 直线行驶测试（验证无零偏）
2. 圆圈测试（验证转向比）
3. S形路线（验证转向平滑性）
4. 停车精度测试（验证最终定位精度）

## 性能考虑

### 计算复杂度

- `steering_wheel_to_front_wheel()`: O(1)
- `calculate_yaw_rate()`: O(1)，包含一个 tan() 调用
- `steering_model_update()`: O(1)

### 存储需求

- SteeringConfig_t: ~40 字节
- SteeringModelState_t: ~40 字节
- 总计: ~80 字节

### 实时性

所有函数都是实时安全的（无动态分配、无浮点异常处理）

## 错误处理

### 返回值

- `steering_model_init()`: 0 成功，-1 参数错误
- `steering_model_update_config()`: 0 成功，-1 参数错误

### 验证

```c
/* 调用前检查参数有效性 */
if (velocity == 0) {
    /* 低速或静止时，转向角变化不影响航向 */
    yaw_rate = 0;
} else {
    yaw_rate = calculate_yaw_rate(velocity, steering_angle);
}
```

## 与其他模块的交互

### 运动模型 (motion_model.c)

- **输入**: 转向模型计算的前轮转角和航向角速率
- **输出**: 位置和姿态预测
- **集成点**: `motion_model_predict()` 中使用转向角参数

### 卡尔曼滤波器 (kalman_filter.c)

- **输入**: 运动模型的雅可比矩阵中的转向角导数
- **输出**: 状态预测协方差
- **集成点**: 状态转移过程中的线性化

### 定位系统 (localization_system.c)

- **输入**: 驾驶员方向盘角度输入
- **输出**: 滤波后的前轮转角用于定位计算
- **集成点**: 系统初始化和周期更新

## 扩展建议

### 1. 非线性转向特性

某些车型转向特性非线性，可添加查表法：

```c
double steering_wheel_to_front_wheel_nonlinear(double angle) {
    /* 使用查表和插值 */
}
```

### 2. 转向角速率限制

```c
double apply_steering_rate_limit(double current, double previous, double dt) {
    /* 限制转向角变化速率 */
}
```

### 3. 自适应滤波

```c
void steering_model_adaptive_filter_update(double velocity) {
    /* 根据速度调整滤波系数 */
}
```

---

**文件位置**: `/src/steering_model.h`, `/src/steering_model.c`

**依赖关系**: 仅依赖标准C库 (`math.h`, `stdlib.h`, `string.h`)

**集成状态**: ✓ 已集成到运动模型 (motion_model.c)
