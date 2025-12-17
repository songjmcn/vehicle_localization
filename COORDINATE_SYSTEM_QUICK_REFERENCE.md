/**
 * @file COORDINATE_SYSTEM_QUICK_REFERENCE.md
 * @brief 坐标系快速参考卡 - 开发人员速查手册
 * @date 2025-12-17
 */

# 坐标系快速参考卡

## 🎯 核心要点（必读）

### ✅ 系统使用全局坐标系（东北向）

```
全局坐标系原点 = 初始化时的车辆位置

      N (北)
      ↑
      | Y轴
------O------- E (东) X轴 →
      |
      
θ = 0°: 北向 (正北)
θ = 90°: 东向 (正东)
θ = 180°或-180°: 南向 (正南)
θ = -90°: 西向 (正西)
```

### ✅ 所有位置 (x, y) 都在**全局坐标系**中

```c
VehicleState_t state;
// state.x = 全局东向位置 (m)
// state.y = 全局北向位置 (m)
// state.theta = 全局航向角 (rad) [-π, π]
```

### ✅ 所有传感器测量都在**车体坐标系**中

```c
SensorReadings_t sensor;
// sensor.wheel_odometry.velocity = 车纵向速度 (m/s)
// sensor.imu.accel_x = 车纵向加速度 (m/s²)
// sensor.imu.gyro_z = 车偏航角速率 (rad/s)
// sensor.steering.steering_angle = 前轮转向角 (rad)
```

---

## 📐 坐标系对比表

| 特性 | 全局坐标系 | 车体坐标系 |
|------|----------|---------|
| **参考** | 地面（固定） | 车辆（动态） |
| **用途** | 绝对定位 | 局部测量 |
| **X轴** | 东向 | 车前方 |
| **Y轴** | 北向 | 车左方 |
| **θ=0** | 北向 | 车纵向 |
| **示例数据** | (x, y, θ) | (v, a, gyro_z) |
| **主要结构** | VehicleState_t | SensorReadings_t |

---

## 🔄 关键转换公式

### 1️⃣ 车体速度 → 全局速度分量

```
全局东向速度:  v_x = v * cos(θ)
全局北向速度:  v_y = v * sin(θ)

其中:
  v   = 车纵向速度 (m/s)
  θ   = 全局航向角 (rad)
```

### 2️⃣ 车体加速度 → 全局加速度

```
[a_x_global]   [cos(θ)  -sin(θ)] [a_x_body]
[a_y_global] = [sin(θ)   cos(θ)] [a_y_body]
```

### 3️⃣ 单轨运动模型（最重要）

```
状态更新方程 (全局坐标系):

x[k+1]     = x[k] + v*cos(θ + sign(v)*δ)*dt
y[k+1]     = y[k] + v*sin(θ + sign(v)*δ)*dt
θ[k+1]     = θ + sign(v)*v*tan(δ)/L*dt
v[k+1]     = v
δ[k+1]     = δ

其中:
  x, y        = 全局位置 (m)
  θ           = 全局航向角 (rad)
  v           = 车纵向速度 (m/s)
  δ           = 前轮转向角 (rad)
  sign(v)     = +1 (前进) 或 -1 (后退)
  L           = 轴距 = 2.7 m
  dt          = 时间间隔 (s)
```

### 4️⃣ 角度规范化

```c
// 所有角度自动规范化到 [-π, π]

例子：
  θ = 2π   → θ = 0
  θ = 3π   → θ = π
  θ = -2π  → θ = 0
  θ = 0.5π → θ = 0.5π (无变化)
```

---

## 🚗 单轨模型可视化

### 前进直线运动

```
初始:                   运动后:
  ↑ θ=0°                 ↑ θ=0°
  |                      |
  ●───→ v>0      →       ●───→ v>0
   δ=0
   
结果: (0, y) → (0, y+v*dt)
      y 北向移动
```

### 左转前进

```
初始:                   运动后:
  ↑ θ=0°                    ↑ θ=δ
  |                         /
  ●───→ v>0    →       ●───→ v>0
   δ>0 (左转)         δ>0
   
结果: 曲线轨迹，位置和航向都改变
```

### 后退转向（**关键：转向反向**）

```
初始:                   运动后:
  ↑ θ=0°                 ↑ θ≈-δ (反向!)
  |                      |
  ●←── v<0      →        ●←── v<0
   δ>0 (转向)      δ_eff=-δ (反向)
   
后退时转向反向！
这是单轨模型的重要特性
```

---

## 💾 数据结构速查

### VehicleState_t (全局坐标系)

```c
typedef struct {
    float x;        // ← 全局位置 (东向)
    float y;        // ← 全局位置 (北向)
    float theta;    // ← 全局航向 (rad) [-π,π]
    float v;        // 速度
    float delta;    // 转向角
    uint32_t timestamp;
} VehicleState_t;

// 使用示例
printf("车辆在全局坐标 (%.2f, %.2f) 处\n", state.x, state.y);
printf("航向 %.2f° (北=0°, 东=90°)\n", RAD_TO_DEG(state.theta));
```

### IMUData_t (车体坐标系)

```c
typedef struct {
    float accel_x;   // ← 车纵向加速度
    float accel_y;   // ← 车横向加速度
    float accel_z;   // 竖直
    float gyro_x;    // 横滚
    float gyro_y;    // 俯仰
    float gyro_z;    // ← 偏航角速率 (重要)
    uint32_t timestamp;
} IMUData_t;

// 关键：gyro_z 用于估计车辆转向速率
```

### WheelOdometryData_t (车体坐标系)

```c
typedef struct {
    float velocity;  // ← 车纵向速度 (前+/后-)
    float distance;  // 累积距离
    uint32_t timestamp;
} WheelOdometryData_t;
```

### SteeringData_t (车体坐标系)

```c
typedef struct {
    float steering_angle;  // ← 前轮转向角 (左+/右-)
    float steering_rate;   // 转向速率
    uint32_t timestamp;
} SteeringData_t;
```

---

## ⚙️ 常用常数

```c
#define PI 3.14159265359
#define WHEELBASE 2.7           // 轴距 (m)
#define V_STATIC_THRESHOLD 0.1  // 静止阈值 (m/s)
#define STEERING_ANGLE_MAX 0.628 // 最大转向角 ≈ 36°

/* 角度转换 */
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)

/* 角度规范化到 [-π, π] */
double normalize_angle(double theta);

/* 值限制 */
double clamp(double value, double min, double max);
```

---

## 🐛 常见错误

### ❌ 错误 1：混淆坐标系

```c
// 错误：直接将车体速度作为全局速度
float v_global_x = sensor.wheel_odometry.velocity;  // ❌ 错误！

// 正确：需要通过旋转矩阵转换
float v_global_x = sensor.wheel_odometry.velocity * cos(state.theta);
float v_global_y = sensor.wheel_odometry.velocity * sin(state.theta);
```

### ❌ 错误 2：忽视后退转向反向

```c
// 错误：后退时不改变转向角
double effective_delta = steering_angle;  // ❌ 错误！

// 正确：后退时转向反向
double effective_delta = sign(velocity) * steering_angle;
```

### ❌ 错误 3：角度超出范围

```c
// 错误：直接累加角度
theta += angular_change;  // ❌ 可能超过 2π

// 正确：规范化到 [-π, π]
theta += angular_change;
theta = normalize_angle(theta);
```

### ❌ 错误 4：轴距混淆

```c
// 轴距是从后轮到前轮的距离，不是两轮半径和
#define WHEELBASE 2.7  // 正确的轴距值
```

---

## 📊 坐标系转换决策树

```
我要处理的数据是什么？
│
├─ 位置或航向 (x, y, θ)
│  └─ 使用全局坐标系
│     示例: VehicleState_t
│
├─ 速度、加速度、角速率、转向角
│  └─ 使用车体坐标系
│     示例: SensorReadings_t
│
└─ 需要相互转换？
   │
   ├─ 车体 → 全局
   │  └─ 使用旋转矩阵 R(θ)
   │
   └─ 全局 → 车体
      └─ 使用反向旋转矩阵 R(θ)^T
```

---

## 🔗 相关函数

### 在 motion_model.c 中

```c
// 预测下一时刻状态 (全局坐标系输入输出)
VehicleState_t motion_model_predict(
    const VehicleState_t *state,
    double velocity,             // 车体系
    double steering_angle,       // 车体系
    double dt
);

// 计算雅可比矩阵（全局对车体的敏感性）
void motion_model_jacobian(...);
```

### 在 kalman_filter.c 中

```c
// 初始化卡尔曼滤波器
int kalman_filter_init(...);

// 预测步 (运动模型)
void kalman_filter_predict(
    KalmanFilter_t *kf,
    double velocity,       // 车体系
    double steering_angle, // 车体系
    uint32_t timestamp
);

// 更新步 (测量模型)
void kalman_filter_update(
    KalmanFilter_t *kf,
    const SensorReadings_t *sensor_data  // 车体系
);

// 获取估计状态 (全局坐标系)
void kalman_filter_get_state(
    const KalmanFilter_t *kf,
    VehicleState_t *state  // 全局系
);
```

---

## ✅ 检查清单

在使用坐标系时，请确认：

- [ ] 我明确知道我正在处理的是**全局坐标系**还是**车体坐标系**
- [ ] 位置和航向都在**全局坐标系**中
- [ ] 传感器测量都在**车体坐标系**中
- [ ] 我已经正确应用了坐标系转换
- [ ] 后退时我改变了转向角的符号
- [ ] 所有角度都规范化到 [-π, π]
- [ ] 轴距值为 2.7 m
- [ ] 时间单位为毫秒 (ms) 或秒 (s)

---

## 📚 更多资源

- 详细文档: [COORDINATE_SYSTEM_SPECIFICATION.md](COORDINATE_SYSTEM_SPECIFICATION.md)
- 示例代码: [example_usage.c](example_usage.c)
- 运动模型: [motion_model.c](motion_model.c)
- 卡尔曼滤波: [kalman_filter.c](kalman_filter.c)

---

**版本：** 1.0  
**最后更新：** 2025-12-17  
**用途：** 快速参考和错误检查

