# 车辆定位系统 - 参数标定和调参指南 (Part 1/5)

## 📖 本系列文档概览

本指南分为5个部分，详细说明22个参数的标定和调参方法：

- **Part 1**（本部分）：参数体系总览 + 转向过滤参数
- **Part 2**：卡尔曼滤波过程噪声参数(Q矩阵)
- **Part 3**：卡尔曼滤波测量噪声参数(R矩阵)
- **Part 4**：传感器校准参数
- **Part 5**：调参决策树+实际工作流程+快速参考

---

## 一、参数体系总览

### 1.1 参数分类统计

整个系统共涉及 **22个参数**，分为四大类：

| 分类 | 参数数量 | 必须标定 | 可调参 | 说明 |
|------|--------|--------|------|------|
| **转向过滤** | 2 | ✓建议 | ✓✓✓推荐 | 新增功能，针对传感器抖动 |
| **噪声参数Q** | 5 | ○可选 | ✓✓推荐 | 过程噪声，影响滤波平滑度 |
| **噪声参数R** | 3 | ✓✓✓必须 | ○可选 | 测量噪声，**必须通过标定获得** |
| **传感器校准** | 12 | ✓✓✓必须 | ○可选 | 零偏和尺度因子校正，**必须标定** |
| **总计** | 22 | 14个 | 16个 | - |

### 1.2 参数优先级划分

```
优先级1 ⭐⭐⭐（绝对必须标定，影响最大）
├─ imu_gyro_bias_z        航向漂移源头，30秒就能漂移57°
├─ wheel_calibration_factor 位置精度基础
├─ r_wheel, r_imu, r_steering 三个测量噪声

优先级2 ⭐⭐（强烈建议标定，影响显著）
├─ steering_angle_offset   直接影响航向精度
├─ steering_angle_gain     影响转向灵敏度
└─ imu_accel_bias_* 、imu_accel_scale_* IMU各项参数

优先级3 ⭐（微调参数，影响性能优化）
├─ filter_alpha, jitter_threshold 转向过滤
└─ q_x, q_y, q_theta, q_v, q_delta 噪声参数
```

### 1.3 参数与功能映射

```
┌─────────────────────────────────────────┐
│         系统架构与参数映射               │
├─────────────────────────────────────────┤
│ 转向角处理                              │
│  ├─ 原始值 → [过滤器] → 低通平滑        │
│  │   └─ filter_alpha (0.2)             │
│  │   └─ jitter_threshold (0.03)        │
│  └─ 输出：过滤后的转向角               │
│                                         │
│ 卡尔曼滤波（EKF）                       │
│  ├─ 预测阶段 [Q矩阵]                   │
│  │   └─ q_x, q_y, q_theta, q_v, q_delta│
│  │                                      │
│  ├─ 更新阶段 [R矩阵]                   │
│  │   └─ r_wheel, r_imu, r_steering    │
│  │                                      │
│  └─ 输出：位置、速度、航向估计         │
│                                         │
│ 传感器校准                              │
│  ├─ 轮速计：wheel_calibration_factor  │
│  ├─ IMU：imu_*_bias, imu_*_scale     │
│  └─ 转向：steering_angle_offset/gain │
└─────────────────────────────────────────┘
```

---

## 二、转向角抖动过滤器参数（2个）

转向角过滤是新增功能，用于消除转向传感器的高频噪声，改进航向估计。

### 2.1 filter_alpha - 低通滤波系数 ⭐ 可调参

#### 参数信息

| 项目 | 内容 |
|------|------|
| **参数名** | `SteeringJitterFilter_t.filter_alpha` |
| **代码位置** | `localization_types.h` 第176行 |
| **结构体** | `SteeringJitterFilter_t` |
| **功能** | 控制转向角低通滤波的平滑程度 |
| **初始值** | 0.2 |
| **调参范围** | 0.05 ~ 0.8 |
| **单位** | 无（归一化系数）|
| **推荐值** | 0.2（自动泊车）|

#### 数学原理

低通滤波递推公式：

```
filtered_angle(n) = α × raw_angle(n) + (1-α) × filtered_angle(n-1)
```

**解释：**
- 当 α = 0.2 时：新值占20%，上一帧占80%
- 当 α 越小：更依赖历史值，更平滑但延迟大
- 当 α 越大：更依赖新值，响应快但保留噪声

#### 调参指南表

| α值范围 | 平滑程度 | 响应延迟 | 适用场景 | 特点 |
|---------|--------|--------|--------|------|
| **0.05-0.1** | 超强平滑 | 很大 | 极低速(< 0.3 m/s)，抖动严重 | 延迟明显 |
| **0.15-0.25** | 强平滑 | 中等 | **自动泊车(0.5-2.0 m/s)，推荐** | 平衡方案 |
| **0.3-0.5** | 中等平滑 | 小 | 低速(2-5 m/s)，转向快速变化 | 响应快 |
| **0.6-0.8** | 弱平滑 | 很小 | 中等速度(5+ m/s)，高质量传感器 | 接近原始值 |

#### 实际效果对比示例

假设转向角序列：10° → 20° → 10° → 20°

```
原始值:     10   20   10   20
            │    │    │    │

α=0.1:      10 → 11 → 12 → 14  (超平滑，延迟3步)
            

α=0.2:      10 → 12 → 14 → 15  (强平滑，延迟2-3步)


α=0.5:      10 → 15 → 13 → 16  (中等，延迟1步)
```

#### 代码调参示例

```c
// 场景1：极端低速（泊车入库，< 0.3 m/s）
localization_system_set_steering_filter_params(&sys, 0.03, 0.1);
// 效果：超强平滑，消除所有高频噪声，可接受延迟

// 场景2：自动泊车（推荐配置，0.5-2.0 m/s）
localization_system_set_steering_filter_params(&sys, 0.03, 0.2);
// 效果：平衡响应和平滑，最适合自动泊车

// 场景3：一般低速行驶（2-5 m/s）
localization_system_set_steering_filter_params(&sys, 0.03, 0.3);
// 效果：更快响应，保留部分高频信息

// 场景4：中等速度（> 5 m/s）
localization_system_set_steering_filter_params(&sys, 0.03, 0.5);
// 效果：快速跟踪转向，最小延迟

// 场景5：动态调整（根据实时速度）
if (velocity < 0.3) {
    alpha = 0.1;  // 极低速
} else if (velocity < 2.0) {
    alpha = 0.2;  // 自动泊车
} else if (velocity < 5.0) {
    alpha = 0.3;  // 低速
} else {
    alpha = 0.5;  // 中速
}
localization_system_set_steering_filter_params(&sys, 0.03, alpha);
```

#### 调参要点

- **增加α（提高）**：响应速度↑，平滑效果↓，抖动增加
- **减少α（降低）**：响应速度↓，平滑效果↑，延迟增加
- **自动泊车最优**：α = 0.2（平衡点）

---

### 2.2 jitter_threshold - 抖动检测阈值 ⭐ 可调参

#### 参数信息

| 项目 | 内容 |
|------|------|
| **参数名** | `SteeringJitterFilter_t.jitter_threshold` |
| **代码位置** | `localization_types.h` 第175行 |
| **功能** | 识别高频抖动的方差阈值 |
| **初始值** | 0.03 rad² |
| **调参范围** | 0.001 ~ 0.1 rad² |
| **单位** | rad²（弧度平方）|
| **推荐值** | 0.03（标准传感器）|

#### 单位转换参考

```
角度与方差的对应关系（标准差）：

0.001 rad² → σ = 0.032 rad = 1.8°   （极敏感）
0.01  rad² → σ = 0.10 rad = 5.7°    （很敏感）
0.03  rad² → σ = 0.17 rad = 10°     （中等敏感，推荐）
0.05  rad² → σ = 0.22 rad = 12.6°   （不敏感）
0.1   rad² → σ = 0.32 rad = 18°     （很不敏感）
```

#### 调参指南表

| 阈值范围 | 检测灵敏度 | 传感器精度 | 零偏更新频率 | 适用情况 |
|---------|----------|----------|----------|---------|
| **0.001-0.01** | 极敏感 | ±0.5°高精度 | 非常频繁 | 易过度反应 |
| **0.02-0.03** | 中等敏感 | ±1-2°标准 | 适中 | **推荐值** |
| **0.05-0.1** | 不敏感 | ±5°低质量 | 很少 | 降低误报 |

#### 传感器选择指南

```
根据您的转向传感器选择阈值：

高精度传感器（如LVDT）
  精度：±0.5°
  推荐threshold = 0.01
  更新零偏：很频繁

标准传感器（如电位计）
  精度：±1-2°
  推荐threshold = 0.03  ✓
  更新零偏：适中

低质量传感器（如磁感应）
  精度：±5°
  推荐threshold = 0.05
  更新零偏：很少
```

#### 代码调参示例

```c
// 方法1：根据传感器类型选择
// 高精度LVDT传感器
localization_system_set_steering_filter_params(&sys, 0.01, 0.2);

// 标准电位计传感器（推荐）
localization_system_set_steering_filter_params(&sys, 0.03, 0.2);

// 低精度磁感应传感器
localization_system_set_steering_filter_params(&sys, 0.05, 0.2);


// 方法2：根据运行环境动态调整
double threshold;
if (temperature > 50) {
    // 高温，传感器漂移大
    threshold = 0.04;
} else if (temperature < 0) {
    // 低温，传感器迟滞大
    threshold = 0.04;
} else {
    // 常温，正常工作
    threshold = 0.03;
}
localization_system_set_steering_filter_params(&sys, threshold, 0.2);
```

#### 调参要点

- **阈值↑（提高）**：抖动检测频率↓，敏感度↓，误报↓
- **阈值↓（降低）**：抖动检测频率↑，敏感度↑，误报↑
- **平衡点**：0.03 rad²（适合标准传感器）

---

## 三、推荐配置方案总结

### 3.1 自动泊车场景（0.5-2.0 m/s，最常用）

```c
// 转向过滤参数设置（推荐）
.jitter_threshold = 0.03 rad²    // 中等灵敏度
.filter_alpha = 0.2              // 强平滑，响应均衡

// 调用代码
localization_system_set_steering_filter_params(&sys, 0.03, 0.2);

// 效果
// ✓ 消除转向传感器的高频抖动
// ✓ 保留转向变化的快速响应
// ✓ 航向估计更准确
```

### 3.2 极低速场景（< 0.3 m/s，泊车入库）

```c
.jitter_threshold = 0.02 rad²
.filter_alpha = 0.1              // 超强平滑
```

### 3.3 一般低速场景（2-5 m/s）

```c
.jitter_threshold = 0.05 rad²
.filter_alpha = 0.3              // 更快响应
```

### 3.4 中等速度场景（> 5 m/s）

```c
.jitter_threshold = 0.05 rad²
.filter_alpha = 0.5              // 快速跟踪
```

---

## 四、总结（Part 1）

### 核心概念

转向过滤参数是在**响应速度**和**平滑效果**之间的权衡：

| 参数 | 含义 | 调整方向 | 影响范围 |
|------|------|--------|--------|
| **filter_alpha** | 平滑强度 | 小→大：平滑↓响应↑ | 全局 |
| **jitter_threshold** | 抖动灵敏度 | 小→大：敏感↓误报↓ | 零偏更新 |

### 快速查表（自动泊车推荐）

```
系统状态                参数设置                      说明
─────────────────────────────────────────────────────
极低速（< 0.3 m/s）   α=0.1, th=0.02    超平滑
自动泊车（0.5-2 m/s） α=0.2, th=0.03    推荐 ✓
一般低速（2-5 m/s）   α=0.3, th=0.05    平衡
中等速度（> 5 m/s）   α=0.5, th=0.05    快速
```

### 下一步行动

1. **立即应用推荐值**：使用 α=0.2, threshold=0.03
2. **观察运行效果**：监测航向角稳定性和响应延迟
3. **细微调整**：根据实际表现微调参数
4. **过渡到Part 2**：学习卡尔曼滤波噪声参数

---

## Part 2：卡尔曼滤波过程噪声参数（Q矩阵）

过程噪声反映了**系统对运动模型的信任程度**。Q值越大，滤波器越相信传感器测量值；Q值越小，滤波器越相信运动模型预测。

### 参数概述

| 参数 | 代码位置 | 默认值 | 范围 | 功能 |
|------|--------|------|------|------|
| **q_x** | localization_system.c 第27行 | 0.005 | 0.001~0.05 | 位置X轴过程噪声 |
| **q_y** | localization_system.c 第28行 | 0.005 | 0.001~0.05 | 位置Y轴过程噪声 |
| **q_theta** | localization_system.c 第29行 | 0.01 | 0.001~0.05 | 航向角过程噪声 |
| **q_v** | localization_system.c 第30行 | 0.02 | 0.01~0.1 | 速度过程噪声（低速关键） |
| **q_delta** | localization_system.c 第31行 | 0.02 | 0.01~0.05 | 转向角过程噪声 |

---

## 一、q_x, q_y - 位置过程噪声

### 1.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.q_x`, `NoiseParameters_t.q_y` |
| **代码位置** | `localization_system.c` 第27-28行 |
| **功能** | 位置预测模型的不确定性（里程计积分误差） |
| **默认值** | 0.005 |
| **可调范围** | 0.001 ~ 0.05 |
| **推荐值** | 0.005（自动泊车） |

### 1.2 调参指南

| 值 | 含义 | 场景 | 表现 |
|----|------|------|------|
| **0.001-0.003** | 高度相信模型 | 直线行驶为主 | 平滑，对异常不敏感 |
| **0.005** | 平衡信任 | **自动泊车推荐** | 好的稳定性和响应 |
| **0.01-0.05** | 容纳模型误差 | 频繁转向、复杂路线 | 快速响应，可能抖动 |

### 1.3 标定方法

```
实验步骤：
1. 选择已知精确距离的直线测试路段（推荐100米以上）
2. 用恒定速度行驶一次，完整记录里程表数据
3. 对比实际距离与里程计距离：
   Δs = |实际距离 - 里程计距离|

判断标准：
- 若 Δs < 0.05m（优秀） → q_x, q_y = 0.001-0.003
- 若 Δs = 0.05-0.1m（良好） → q_x, q_y = 0.005（推荐）
- 若 Δs = 0.1-0.2m（一般） → q_x, q_y = 0.01
- 若 Δs > 0.2m（差） → q_x, q_y = 0.02-0.03

现象判断：
- 位置出现振荡 → 减小q_x, q_y
- 位置跟踪延迟 → 增大q_x, q_y
```

### 1.4 代码调整

```c
// 默认配置（自动泊车）
NoiseParameters_t noise = {
    .q_x = 0.005,    // 相同的X轴
    .q_y = 0.005,    // 相同的Y轴
    ...
};

// 手动设置
noise.q_x = 0.01;    // 里程计精度不好，增大
noise.q_y = 0.01;    // 允许更快响应
```

---

## 二、q_theta - 航向角过程噪声

### 2.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.q_theta` |
| **代码位置** | `localization_system.c` 第29行 |
| **功能** | 航向角预测模型的不确定性（转向动力学延迟） |
| **默认值** | 0.01 rad² |
| **可调范围** | 0.001 ~ 0.05 rad² |
| **推荐值** | 0.01（通用） |

### 2.2 调参指南

| 值 | 特性 | 场景 | 表现 |
|----|------|------|------|
| **0.001-0.005** | 相信转向模型 | 转向模型准确，动力学延迟小 | 航向平滑，可能滞后 |
| **0.01** | 平衡 | **一般场景推荐** | 响应和平滑均衡 |
| **0.02-0.05** | 容纳动力学误差 | 转向滞后大，齿轮间隙多 | 快速响应，可能抖动 |

### 2.3 标定方法

```
实验步骤：
1. 静止状态，转向盘固定在某个角度
2. 以恒定速度（推荐1 m/s）直线行驶
3. 监测航向角估计值与RTK航向的偏差

航向漂移检测：
- 记录10秒内的航向角变化量 Δθ
- 计算漂移率：rate = Δθ / 10 （单位：rad/s）

判断标准：
- 若 rate < 0.001 rad/s（< 0.06°/s） → 模型准确，q_theta = 0.01
- 若 rate = 0.001-0.01 rad/s → q_theta = 0.015
- 若 rate > 0.01 rad/s（> 0.57°/s） → q_theta = 0.02-0.03

现象判断：
- 航向漂移 > 2°/10s → 增大q_theta
- 航向振荡明显 → 减小q_theta
```

### 2.4 代码调整

```c
// 调试代码示例
noise.q_theta = 0.01;  // 基础值

if (heading_drift_rate > 0.01) {
    // 漂移严重，增加过程噪声
    noise.q_theta = 0.02;
    printf("航向漂移严重，已增大q_theta\n");
}
```

---

## 三、q_v - 速度过程噪声 ⭐ 低速关键参数

### 3.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.q_v` |
| **代码位置** | `localization_system.c` 第30行 |
| **功能** | 速度预测的模型不确定性（轮滑动、加速度延迟） |
| **默认值** | 0.02 m²/s⁴ |
| **可调范围** | 0.01 ~ 0.1 |
| **推荐值** | 0.02（自动泊车），0.03（极低速） |
| **⭐重要性** | **低速场景最关键参数** |

### 3.2 低速场景特殊性

```
为什么q_v在低速时关键？

低速（0.5-2.0 m/s）特点：
- 轮滑动频繁（静摩擦-动摩擦转换）
- 加速度响应非线性
- 传动系统间隙和滞后明显
- 传感器噪声相对值较大

因此需要更大的q_v来容纳这些误差
```

### 3.3 调参指南表

| 值 | 特性 | 场景 | 表现 |
|----|------|------|------|
| **0.01** | 光滑加速 | 恒速或缓慢加速，轮滑少 | 速度平滑，响应慢 |
| **0.02** | 平衡 | **自动泊车推荐** | 平衡响应和平滑 |
| **0.03** | 快速响应 | **极低速推荐** | 容纳轮滑，快速调整 |
| **0.05-0.1** | 敏捷跟踪 | 频繁加减速 | 快速响应，可能抖动 |

### 3.4 标定方法

```
标定步骤：

步骤1：静止采集（基础噪声）
- 车辆完全静止10秒
- 采集轮速传感器读数100个样本
- 计算标准差：σ_v = std(velocity)
- q_v_base ≈ σ_v²

步骤2：恒速采集（模型误差）
- 以恒定速度行驶（推荐0.8 m/s）
- 记录速度估计值和实际轮速
- 计算差异：err = |estimated_v - real_v|
- q_v_model ≈ mean(err²)

步骤3：制动响应（延迟评估）
- 恒速行驶，突然制动
- 记录速度下降延迟时间：Δt
  - Δt < 0.5s → q_v = 0.01（好）
  - Δt = 0.5-1.0s → q_v = 0.02-0.03（正常）
  - Δt > 1.0s → q_v = 0.03-0.05（调整）

步骤4：低速轮滑测试
- 极低速行驶（0.3 m/s）
- 快速转向观察速度估计
- 若出现跳跃 → 增大q_v
- 若出现滞后 → 减小q_v

最终标定表：
稳定恒速 → q_v = 0.01
一般泊车 → q_v = 0.02-0.03 ✓
复杂工况 → q_v = 0.04-0.05
```

### 3.5 代码调参示例

```c
// 基础配置
NoiseParameters_t noise = {
    .q_v = 0.02,  // 自动泊车推荐
    ...
};

// 自适应调整（根据实时条件）
if (velocity < 0.3) {
    // 极低速，轮滑多，容纳更大噪声
    noise.q_v = 0.03;
} else if (velocity < 2.0) {
    // 自动泊车速度，推荐值
    noise.q_v = 0.02;
} else if (velocity < 5.0) {
    // 低速行驶，轮滑少
    noise.q_v = 0.015;
} else {
    // 一般速度，精确模型
    noise.q_v = 0.01;
}

// 根据加速度调整
double accel = (velocity - last_velocity) / dt;
if (fabs(accel) > 0.5) {
    // 强加速，增加过程噪声
    noise.q_v = 0.03;
}
```

### 3.6 调参要点

- **q_v ↑（增大）**：容纳轮滑和加速度延迟，响应快
- **q_v ↓（减小）**：相信速度估计的平滑性，延迟大
- **自动泊车最优**：q_v = 0.02~0.03
- **极低速最优**：q_v = 0.03（因为轮滑多）

---

## 四、q_delta - 转向角过程噪声

### 4.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.q_delta` |
| **代码位置** | `localization_system.c` 第31行 |
| **功能** | 转向角预测的模型不确定性（转向指令到实际转向的延迟） |
| **默认值** | 0.02 rad² |
| **可调范围** | 0.01 ~ 0.05 |
| **推荐值** | 0.02（通用） |

### 4.2 调参指南

| 值 | 场景 | 表现 |
|----|------|------|
| **0.01** | 转向平稳无延迟 | 假设转向指令立即生效 |
| **0.02** | **一般场景** | 平衡 |
| **0.03-0.05** | 快速转向变化 | 容纳转向系统延迟 |

### 4.3 标定方法

```
实验步骤：
1. 记录转向盘输入角度 δ_input
2. 监测实际转向角响应 δ_actual
3. 计算响应延迟时间 τ（从输入到达50%响应）

判断标准：
- 若 τ < 0.1s → q_delta = 0.01（响应快）
- 若 τ = 0.1-0.2s → q_delta = 0.02（标准）
- 若 τ > 0.2s → q_delta = 0.03-0.05（延迟大）

现象判断：
- 转向滞后明显 → 增大q_delta
- 转向振荡 → 减小q_delta
```

### 4.4 代码调整

```c
noise.q_delta = 0.02;  // 基础配置

// 根据转向速率动态调整
double steering_rate = abs(delta - last_delta) / dt;
if (steering_rate > 0.5) {
    // 快速转向，增加过程噪声
    noise.q_delta = 0.03;
}
```

---

## 五、Q矩阵推荐配置

### 5.1 自动泊车场景（0.5-2.0 m/s，最常用）

```c
NoiseParameters_t noise = {
    .q_x = 0.005,       // 位置X轴
    .q_y = 0.005,       // 位置Y轴
    .q_theta = 0.01,    // 航向角
    .q_v = 0.02,        // 速度（推荐）
    .q_delta = 0.02,    // 转向角
    ...
};
```

**特点：**
- ✓ 平衡响应和平滑
- ✓ 容纳低速轮滑
- ✓ 容纳转向延迟
- ✓ 航向估计准确

### 5.2 极低速场景（< 0.3 m/s，泊车入库）

```c
NoiseParameters_t noise = {
    .q_x = 0.01,        // 位置允许更大变化
    .q_y = 0.01,
    .q_theta = 0.015,   // 航向更敏感
    .q_v = 0.03,        // 轮滑多，增大
    .q_delta = 0.03,    // 转向延迟明显
    ...
};
```

### 5.3 快速响应场景（转向频繁）

```c
NoiseParameters_t noise = {
    .q_x = 0.01,        // 更快响应位置
    .q_y = 0.01,
    .q_theta = 0.02,    // 快速跟踪航向
    .q_v = 0.03,
    .q_delta = 0.03,    // 快速响应转向
    ...
};
```

---

## 六、总结（Part 2）

### 核心原则

Q矩阵参数反映**对运动模型的信任程度**：

| 参数 | 含义 | 调整规则 |
|------|------|--------|
| **q_x, q_y** | 位置模型准确性 | 里程计误差大 → 增大 |
| **q_theta** | 航向模型准确性 | 漂移严重 → 增大 |
| **q_v** | 速度模型准确性 | 轮滑多 → 增大（低速关键）|
| **q_delta** | 转向模型准确性 | 延迟大 → 增大 |

### 快速调参流程

```
1. 从推荐值开始
2. 观察滤波结果
3. 判断问题类型：
   - 位置漂移 → 调整q_x, q_y
   - 航向漂移 → 调整q_theta
   - 速度延迟 → 调整q_v
   - 转向滞后 → 调整q_delta
4. 微调参数
5. 验证效果
```

### 自动泊车推荐值总结

```
q_x = 0.005    // 位置
q_y = 0.005
q_theta = 0.01 // 航向
q_v = 0.02     // 速度（低速关键）
q_delta = 0.02 // 转向
```

---

## Part 3：卡尔曼滤波测量噪声参数（R矩阵）

测量噪声反映了**传感器测量值的可靠性**。R值越大，滤波器越不相信该传感器；R值越小，滤波器越相信该传感器。

### ⭐ 重要提示

R矩阵的三个参数（r_wheel, r_imu, r_steering）**都必须通过实验标定**，不能仅靠猜测。这是系统精度的关键基础。

### 参数概述

| 参数 | 代码位置 | 默认值 | 范围 | 功能 | 标定难度 |
|------|--------|------|------|------|---------|
| **r_wheel** | localization_system.c 第33行 | 0.01 | 0.001~0.1 | 轮速计测量噪声 | 🟢 简单 |
| **r_imu** | localization_system.c 第34行 | 0.1 | 0.01~1.0 | IMU加速度测量噪声 | 🟡 中等 |
| **r_steering** | localization_system.c 第35行 | 0.05 | 0.01~0.2 | 转向角测量噪声 | 🟢 简单 |

---

## 一、r_wheel - 轮速计测量噪声 ⭐⭐⭐ 必须标定

### 1.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.r_wheel` |
| **代码位置** | `localization_system.c` 第33行 |
| **功能** | 轮速计速度测量的精度等级 |
| **默认值** | 0.01 m²/s² |
| **可调范围** | 0.001 ~ 0.1 m²/s² |
| **单位** | (m/s)² - 速度方差 |
| **标定优先级** | **⭐⭐⭐ 最高** |
| **影响范围** | 位置精度直接相关 |

### 1.2 轮速计分类与建议值

```
轮速计类型对应的r_wheel值：

类型             精度等级    r_wheel推荐值
─────────────────────────────────
高精度ABS传感器  ±1-2%      0.001 ~ 0.01
标准车速传感器   ±3-5%      0.01  ~ 0.03  ✓
低精度传感器     ±10%       0.05  ~ 0.1

判断依据：
- 查看传感器规格书上的精度指标
- 或根据标定实验结果计算
```

### 1.3 标定方法（最重要）

**方法A：静止采集法（快速，推荐）**

```
步骤1：环境准备
- 车辆完全停止
- 保持轮胎静止10秒
- 采集轮速传感器100个样本

步骤2：数据分析
velocity_readings = [v1, v2, v3, ..., v100]

// 理想情况：所有读数都应该是0
// 实际情况：有噪声，读数不为0

步骤3：计算标准差
mean_v = mean(velocity_readings)
variance = sum((vi - mean_v)²) / 100

步骤4：设置参数
r_wheel = variance

// 示例
// 读数范围：-0.05 ~ +0.05 m/s
// 标准差：σ = 0.025 m/s
// r_wheel = 0.025² = 0.000625 ≈ 0.001
```

**方法B：恒速采集法（精确，推荐）**

```
步骤1：测试设置
- 选择平坦直线路段
- 以恒定速度行驶（推荐1.0 m/s）
- 采集轮速读数200个样本（10秒）
- 同时记录RTK或高精度GPS的速度

步骤2：误差计算
for i = 1 to 200:
    error[i] = |wheel_velocity[i] - rtk_velocity[i]|

步骤3：统计
mean_error = mean(error[])
variance = sum((error[i] - mean_error)²) / 200

步骤4：设置参数
r_wheel = variance

// 示例
// 轮速与RTK对比：
// 误差范围 ±0.02 m/s
// 方差 = 0.0001
// r_wheel = 0.0001
```

**方法C：加减速法（综合，最准确）**

```
步骤1：采集数据
- 从0开始均匀加速到2 m/s（10秒）
- 然后匀速5秒
- 然后匀速减速到0（10秒）
- 全程采集轮速和参考速度（如RTK）

步骤2：分段分析
- 分为恒速段、加速段、减速段
- 对每段计算误差方差

步骤3：加权平均
r_wheel = 0.2×var_accel + 0.2×var_decel + 0.6×var_constant

步骤4：保守估计
// 取较大值以容纳动态误差
r_wheel = max_variance × 1.2  // 留20%余量
```

### 1.4 结果评估与调整

```
标定后的r_wheel评估：

r_wheel值      传感器质量评价        调整建议
─────────────────────────────────
< 0.001       极高（±0.5%）       可信，使用默认值
0.001-0.01    高（±1-2%）         可信，标定值 ✓
0.01-0.05     中等（±3-5%）       可接受，标定值
0.05-0.1      一般（±10%）        可用，需补偿
> 0.1         差（>±10%）         不推荐使用
```

### 1.5 代码实现

```c
// 标定结果应用
NoiseParameters_t noise;

// 方法1：直接设置标定值
noise.r_wheel = 0.001;  // 根据标定实验得出

// 方法2：在线计算（可选）
static double wheel_error_sum = 0;
static int wheel_count = 0;

// 在处理循环中
double error = wheel_velocity - reference_velocity;
wheel_error_sum += error * error;
wheel_count++;

if (wheel_count >= 100) {
    noise.r_wheel = wheel_error_sum / wheel_count;
    wheel_error_sum = 0;
    wheel_count = 0;
}

// 应用到滤波器
sys.kalman_filter.noise.r_wheel = noise.r_wheel;
```

### 1.6 故障诊断

```
症状                   可能原因              解决方案
─────────────────────────────────────
位置漂移明显      r_wheel过大           减小r_wheel
位置振荡频繁      r_wheel过小           增大r_wheel
位置滞后严重      r_wheel过大 + q_v过小  同时调整两者
轮滑时不适应      无法检测              需要增加轮滑检测算法
```

---

## 二、r_imu - IMU加速度测量噪声 ⭐⭐⭐ 必须标定

### 2.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.r_imu` |
| **代码位置** | `localization_system.c` 第34行 |
| **功能** | IMU加速度计的测量精度 |
| **默认值** | 0.1 m²/s⁴ |
| **可调范围** | 0.01 ~ 1.0 |
| **单位** | (m/s²)² - 加速度方差 |
| **标定优先级** | **⭐⭐⭐ 最高** |
| **应用** | 轮滑检测，加速度补正 |

### 2.2 IMU类型与建议值

```
IMU类型           精度等级    r_imu推荐值
─────────────────────────────────
高精度MEMS IMU   ±0.05g      0.001 ~ 0.01
标准MEMS IMU     ±0.1g       0.01  ~ 0.1  ✓
消费级IMU        ±0.3g       0.1   ~ 1.0
工业级IMU        ±0.02g      0.0001~ 0.001

g = 9.81 m/s²
```

### 2.3 标定方法

**方法A：静止采集法（基础噪声）**

```
步骤1：准备
- 车辆完全停止，轮胎不动
- 水平放置（避免倾斜）
- 采集加速度100个样本

步骤2：去掉重力加速度
// 假设Z轴向上，重力加速度≈9.81 m/s²
// X轴和Y轴应该没有加速度

accel_x_readings = [ax1, ax2, ..., ax100]
accel_y_readings = [ay1, ay2, ..., ay100]

// 计算X、Y轴的噪声（Z轴有重力，需要特殊处理）

步骤3：计算方差
variance_x = sum((axi - mean_ax)²) / 100
variance_y = sum((ayi - mean_ay)²) / 100

步骤4：设置参数
r_imu_xy = (variance_x + variance_y) / 2

// 示例
// X、Y轴读数范围 ±0.2 m/s²
// 标准差 σ = 0.1 m/s²
// r_imu = 0.1² = 0.01
```

**方法B：动态采集法（模型误差）**

```
步骤1：采集数据
- 缓慢加速从0到2 m/s（20秒）
- 同时采集IMU加速度和轮速加速度

步骤2：对比分析
for i = 1 to N:
    a_wheel[i] = (v_wheel[i+1] - v_wheel[i]) / dt  // 轮速推导
    a_imu[i] = measured_acceleration[i]             // IMU直接测
    error[i] = |a_imu[i] - a_wheel[i]|

步骤3：计算方差
variance = sum((error[i] - mean_error)²) / N

步骤4：设置参数
r_imu = variance × 1.2  // 保守估计

// 示例
// 误差范围 ±0.1 m/s²
// 方差 = 0.01
// r_imu = 0.012
```

**方法C：六位置标定法（最准确）**

```
步骤1：准备6个方向
将IMU传感器在以下6个方向上各放置10秒：
1. Z轴向上（正常）
2. Z轴向下（倒立）
3. X轴向上（横放）
4. X轴向下（横放反向）
5. Y轴向上（竖放）
6. Y轴向下（竖放反向）

步骤2：去掉重力加速度
在每个方向上：
- 主轴应该有±9.81 m/s²的重力
- 其他轴应该接近0（这部分就是噪声）

步骤3：计算噪声
对每个方向，计算非主轴的标准差
σ_1, σ_2, ..., σ_6

步骤4：加权平均
r_imu = (σ_1² + σ_2² + ... + σ_6²) / 6
```

### 2.4 代码实现

```c
// 标定结果应用
NoiseParameters_t noise;

// 根据IMU类型设置
noise.r_imu = 0.05;  // 标准MEMS IMU的推荐值

// 动态标定（可选）
static double imu_error_sum = 0;
static int imu_count = 0;

// 在处理循环中
double a_estimated = (velocity - last_velocity) / dt;
double a_measured = sqrt(imu.accel_x² + imu.accel_y²);  // 水平加速度

double error = a_measured - a_estimated;
imu_error_sum += error * error;
imu_count++;

if (imu_count >= 100) {
    noise.r_imu = imu_error_sum / imu_count;
    imu_error_sum = 0;
    imu_count = 0;
}
```

### 2.5 温度补偿（可选）

```
IMU在不同温度下的噪声变化：

温度      r_imu调整系数
─────────────────────
-10°C    ×1.3（低温零偏漂移）
0-25°C   ×1.0（标准）
25-40°C  ×1.2（高温噪声增加）
>40°C    ×1.5（极端高温）

// 代码示例
if (temperature < 0) {
    r_imu_adjusted = r_imu * 1.3;
} else if (temperature > 40) {
    r_imu_adjusted = r_imu * 1.5;
} else {
    r_imu_adjusted = r_imu * 1.0;
}
noise.r_imu = r_imu_adjusted;
```

---

## 三、r_steering - 转向角测量噪声 ⭐⭐⭐ 必须标定

### 3.1 基本概念

| 属性 | 说明 |
|------|------|
| **参数名** | `NoiseParameters_t.r_steering` |
| **代码位置** | `localization_system.c` 第35行 |
| **功能** | 转向角传感器的测量精度 |
| **默认值** | 0.05 rad² |
| **可调范围** | 0.01 ~ 0.2 rad² |
| **单位** | rad² - 角度方差 |
| **标定优先级** | **⭐⭐⭐ 最高** |
| **影响范围** | 航向估计精度直接相关 |

### 3.2 转向传感器分类与建议值

```
传感器类型        精度等级    r_steering推荐值
─────────────────────────────────
高精度LVDT       ±0.5°      0.001 ~ 0.01
标准电位计       ±1-2°      0.01  ~ 0.05  ✓
磁感应传感器     ±3-5°      0.05  ~ 0.1
低精度传感器     ±10°       0.1   ~ 0.2

角度与方差关系：
0.5° = 0.0087 rad → 方差 ≈ 0.000076 rad²
1°   = 0.0175 rad → 方差 ≈ 0.0003 rad²
2°   = 0.0349 rad → 方差 ≈ 0.0012 rad²
5°   = 0.0873 rad → 方差 ≈ 0.0076 rad²
```

### 3.3 标定方法

**方法A：静止标定法（快速）**

```
步骤1：准备
- 车辆停止，转向盘固定
- 在多个转向角位置（例如-20°, 0°, +20°）
- 每个位置采集20个样本

步骤2：采集数据
angles_at_0 = [δ1, δ2, ..., δ20]
angles_at_20_left = [δ1, δ2, ..., δ20]
angles_at_20_right = [δ1, δ2, ..., δ20]

步骤3：计算方差
variance_0 = var(angles_at_0)
variance_left = var(angles_at_20_left)
variance_right = var(angles_at_20_right)

r_steering = mean(variance_0, variance_left, variance_right)

// 示例
// 每个角度位置的标准差 ±1°（0.0175 rad）
// 方差 = 0.0175² ≈ 0.0003 rad²
// r_steering ≈ 0.0003
```

**方法B：动态采集法（推荐）**

```
步骤1：低速驾驶
- 以0.8 m/s的速度行驶
- 缓慢转向（从-15°到+15°）
- 采集200个样本（10秒）

步骤2：实际对比
// 理想情况：转向盘角度 = 实际转向角
// 实际情况：有传感器噪声和滞后

for i = 1 to 200:
    theoretical_angle = steering_input[i]
    measured_angle = sensor_output[i]
    error[i] = measured_angle - theoretical_angle

步骤3：计算方差
variance = sum((error[i] - mean_error)²) / 200
r_steering = variance

// 示例
// 误差范围 ±0.05 rad（±2.86°）
// 标准差 σ = 0.025 rad
// r_steering = 0.025² = 0.000625 ≈ 0.001
```

**方法C：RTK对比法（最准确）**

```
步骤1：长直线行驶
- 使用RTK测得的航向角作为参考真值
- 多次恒定转向角行驶
- 采集每种转向角下的100个样本

步骤2：计算理论航向变化
v = 1 m/s, δ = 0.1 rad (约5.7°)
L = 2.7 m (轴距)

理论航向变化率 = v × sin(δ) / L

步骤3：对比实际
RTK航向变化 = dθ_RTK / dt
预测航向变化 = δ × v / L

误差 = |RTK变化 - 预测变化|

步骤4：计算方差
r_steering = mean(error²)
```

### 3.4 代码实现

```c
// 标定结果应用
NoiseParameters_t noise;

// 根据传感器类型设置
// 标准电位计传感器（推荐）
noise.r_steering = 0.05;  // rad²

// 高精度LVDT传感器
noise.r_steering = 0.01;  // rad²

// 动态标定（可选）
static double steering_error_sum = 0;
static int steering_count = 0;

// 在处理循环中
double theoretical_steering = expected_steering_angle;
double measured_steering = sensor_reading;
double error = measured_steering - theoretical_steering;

steering_error_sum += error * error;
steering_count++;

if (steering_count >= 100) {
    noise.r_steering = steering_error_sum / steering_count;
    steering_error_sum = 0;
    steering_count = 0;
}
```

---

## 四、R矩阵推荐配置

### 4.1 自动泊车场景（标准传感器）

```c
NoiseParameters_t noise = {
    .r_wheel = 0.01,       // 标准车速传感器（±3-5%）
    .r_imu = 0.05,         // 标准MEMS IMU
    .r_steering = 0.05,    // 标准电位计（±1-2°）
    ...
};
```

**特点：**
- ✓ 基于标准商用传感器
- ✓ 平衡成本和精度
- ✓ 适合绝大多数应用

### 4.2 高精度场景（高精度传感器）

```c
NoiseParameters_t noise = {
    .r_wheel = 0.001,      // 高精度ABS传感器（±1-2%）
    .r_imu = 0.01,         // 高精度MEMS IMU
    .r_steering = 0.01,    // 高精度LVDT（±0.5°）
    ...
};
```

### 4.3 低成本场景（普通传感器）

```c
NoiseParameters_t noise = {
    .r_wheel = 0.05,       // 低精度传感器（±10%）
    .r_imu = 0.2,          // 消费级IMU
    .r_steering = 0.1,     // 低精度传感器
    ...
};
```

---

## 五、总结（Part 3）

### 核心原则

R矩阵反映**对传感器测量值的信任程度**：

| 参数 | 含义 | 原则 |
|------|------|------|
| **r_wheel** | 轮速精度 | 精度好→值小，精度差→值大 |
| **r_imu** | 加速度精度 | 精度好→值小，精度差→值大 |
| **r_steering** | 转向精度 | 精度好→值小，精度差→值大 |

### 标定优先级

```
优先级1（立即标定）：
├─ r_wheel 最简单，影响位置精度
├─ r_steering 最简单，影响航向精度
└─ r_imu 相对复杂，影响加速度精度

推荐顺序：r_wheel → r_steering → r_imu
预计时间：15分钟 + 20分钟 + 30分钟
```

### 快速标定流程

```
Step 1: 轮速计标定（10分钟）
├─ 静止采集100个样本
├─ 计算速度标准差
└─ r_wheel = σ²

Step 2: 转向角标定（15分钟）
├─ 固定转向角采集
├─ 计算角度标准差
└─ r_steering = σ²

Step 3: IMU标定（20分钟）
├─ 静止采集加速度
├─ 去掉重力后计算方差
└─ r_imu = σ²

总耗时：约45分钟
```

### 自动泊车推荐值总结

```
标准传感器配置（最常用）：

r_wheel = 0.01       // 轮速计
r_imu = 0.05         // IMU加速度
r_steering = 0.05    // 转向角
```

---

## Part 4：传感器校准参数

传感器校准参数用于**修正传感器的系统误差**，包括零偏、增益、尺度因子等。这些参数通常在传感器安装或首次上电时进行，并在定期维护时重新检查。

### 参数概述（12个参数）

| 分类 | 参数 | 默认值 | 功能 | 校准难度 |
|------|------|------|------|---------|
| **轮速计** | wheel_calibration_factor | 1.0 | 轮速计增益校准 | 🟢 简单 |
| **转向角** | steering_angle_offset | 0.0 | 转向零偏 | 🟢 简单 |
| | steering_angle_gain | 1.0 | 转向增益 | 🟡 中等 |
| **IMU加速度** | imu_accel_x_bias | 0.0 | X轴零偏 | 🟡 中等 |
| | imu_accel_y_bias | 0.0 | Y轴零偏 | 🟡 中等 |
| | imu_accel_z_bias | 0.0 | Z轴零偏 | 🟡 中等 |
| | imu_accel_x_scale | 1.0 | X轴尺度因子 | 🟡 中等 |
| | imu_accel_y_scale | 1.0 | Y轴尺度因子 | 🟡 中等 |
| | imu_accel_z_scale | 1.0 | Z轴尺度因子 | 🟡 中等 |
| **IMU陀螺仪** | imu_gyro_z_bias | 0.0 | Z轴角速度零偏 | 🟡 中等 |
| **IMU磁强计** | imu_mag_x_bias | 0.0 | X轴磁场零偏 | 🟠 困难 |
| | imu_mag_y_bias | 0.0 | Y轴磁场零偏 | 🟠 困难 |

---

## 一、轮速计校准参数（1个）

### 1.1 wheel_calibration_factor - 轮速计增益校准

| 属性 | 说明 |
|------|------|
| **参数名** | `CalibrationParameters_t.wheel_calibration_factor` |
| **代码位置** | `localization_system.c` 第42行 |
| **功能** | 修正轮速计的增益误差 |
| **默认值** | 1.0（无修正） |
| **可调范围** | 0.9 ~ 1.1 |
| **单位** | 无量纲（比例系数） |
| **校准优先级** | ⭐⭐⭐ 非常重要 |
| **影响范围** | 里程误差直接相关 |

### 1.2 产生原因与症状

```
常见原因：
1. 轮胎磨损 - 轮胎半径变化
2. 轮胎气压不同 - 轮胎变形不同
3. 传感器安装位置 - 齿圈尺寸差异
4. 电子标定偏差 - 出厂默认值

症状表现：
- 里程表读数偏大：校准因子 > 1.0
- 里程表读数偏小：校准因子 < 1.0
- 长距离行驶（100m+）能明显看出差异
- 定位漂移随时间线性增加
```

### 1.3 标定方法A：直线行驶法（推荐）

```
步骤1：准备
- 选择平坦宽敞的直线路段（至少200m）
- 使用高精度卷尺或RTK测量参考距离
- 标记起点和终点

步骤2：采集数据
- 缓慢行驶，从起点驶到终点
- 记录轮速计读数的累积距离

reference_distance = 200.0 m  // 使用卷尺测量
measured_distance = 198.5 m   // 轮速计读数

步骤3：计算校准因子
wheel_calibration_factor = reference_distance / measured_distance
wheel_calibration_factor = 200.0 / 198.5 = 1.0076

步骤4：应用
// 修正后的速度 = 原始速度 × 校准因子
corrected_velocity = raw_velocity × 1.0076
```

### 1.4 标定方法B：多段测试法（更准确）

```
步骤1：准备10段测试
- 每段50m，共500m
- 分别记录每段的轮速计读数

测试数据：
段号  参考距离(m)  轮速计读数(m)  误差
1     50.0         49.8          -0.4%
2     50.0         50.2          +0.4%
3     50.0         49.9          -0.2%
...
10    50.0         50.1          +0.2%

步骤2：统计分析
平均误差 = (-0.4% + 0.4% - 0.2% + ... + 0.2%) / 10
         = 0.05%

校准因子 = 1.0 / (1 + 平均误差%)
          = 1.0 / 1.0005
          = 0.9995
```

### 1.5 轮胎压力影响

```
轮胎气压与轮径变化的影响：

胎压(kPa)    相对轮径变化    wheel_calibration_factor
220         标准             1.0 ✓
200         -0.8%           1.008
240         +0.6%           0.994

建议：
- 使用标准胎压（通常印在驾驶室门框）
- 每次标定前检查和调整轮胎气压
- 如果经常改变胎压，需要重新标定
```

### 1.6 代码实现

```c
// 标定结果应用
CalibrationParameters_t calib;
calib.wheel_calibration_factor = 1.0076;  // 根据标定得出

// 在速度处理中应用
double calibrated_wheel_speed = 
    raw_wheel_speed * calib.wheel_calibration_factor;

// 长距离检验
double accumulated_distance = 0;
for (int i = 0; i < sample_count; i++) {
    double velocity = raw_wheel_speed[i] * calib.wheel_calibration_factor;
    accumulated_distance += velocity * dt;
}

// 检验：accumulated_distance应该等于实际行驶距离
if (fabs(accumulated_distance - reference_distance) > 0.5) {
    // 需要重新标定
    printf("Warning: 里程误差过大，建议重新标定\n");
}
```

---

## 二、转向角校准参数（2个）

### 2.1 steering_angle_offset - 转向角零偏

| 属性 | 说明 |
|------|------|
| **参数名** | `CalibrationParameters_t.steering_angle_offset` |
| **代码位置** | `localization_system.c` 第43行 |
| **功能** | 修正转向盘中心位置偏差 |
| **默认值** | 0.0 rad（0°） |
| **可调范围** | -0.3 ~ +0.3 rad（±17°） |
| **单位** | rad（弧度） |
| **校准优先级** | ⭐⭐⭐ 非常重要 |
| **影响范围** | 航向估计、转向中线 |

### 2.2 产生原因与症状

```
产生原因：
1. 转向盘物理位置 - 安装偏斜
2. 传感器零点偏差 - 出厂标定
3. 齿条磨损不均 - 机械不对称
4. 悬架改装 - 几何角度改变

典型症状：
- 直线行驶时，转向盘不在中心
- 自动驾驶时，车辆线性漂移到一侧
- 在反复转向后出现定向误差
```

### 2.3 标定方法：直线行驶法

```
步骤1：环境准备
- 在宽敞平直的空旷区域
- 没有风、不在斜坡
- 地面平坦且干燥

步骤2：直线行驶
- 放手（自动驾驶）或保持转向盘居中行驶100m
- 关闭所有横向控制
- 记录转向盘角度传感器的读数（应该在0°附近波动）

步骤3：数据分析
steering_readings = [δ1, δ2, δ3, ..., δN]

// 计算平均值
offset = mean(steering_readings)

// 如果offset != 0，说明有偏差
// 例如：offset = 0.05 rad (2.9°)

步骤4：应用校准
// 修正后的转向角 = 原始读数 - offset
corrected_steering = raw_steering - 0.05

// 代码中应该这样做
steering_angle_offset = 0.05;  // 存储偏差值
calibrated_angle = raw_angle - steering_angle_offset;
```

### 2.4 steering_angle_gain - 转向增益

| 属性 | 说明 |
|------|------|
| **参数名** | `CalibrationParameters_t.steering_angle_gain` |
| **代码位置** | `localization_system.c` 第44行 |
| **功能** | 修正转向角的线性增益误差 |
| **默认值** | 1.0（无修正） |
| **可调范围** | 0.9 ~ 1.1 |
| **单位** | 无量纲（比例系数） |
| **校准优先级** | ⭐⭐ 较重要 |

### 2.5 转向增益标定方法

```
步骤1：准备
- 使用转向盘角度传感器（精度±1°以上）
- 准备规范的转向输入：0° → 10° → 0° → -10° → 0°

步骤2：采集对比
// 理想情况：设定10°，传感器读10°
// 实际情况：设定10°，传感器读9.5°（增益不足）

true_angle = [0, 10, 0, -10, 0]  // 已知的转向输入
measured_angle = [0, 9.5, 0, -9.4, 0]  // 传感器读数

步骤3：计算增益
// 简单方法：对应正偏差点计算
gain_positive = true_angle_positive / measured_angle_positive
              = 10.0 / 9.5
              = 1.0526

// 精确方法：线性拟合
// 使用最小二乘法拟合：y = a*x + b
// 其中y=true_angle, x=measured_angle
// 斜率a就是gain

gain = 1.053  // 最终标定值
```

### 2.6 代码实现

```c
CalibrationParameters_t calib;
calib.steering_angle_offset = 0.05;   // rad, 约2.9°
calib.steering_angle_gain = 1.053;

// 转向角校准流程
double raw_steering = sensor_reading;
double calibrated_steering = 
    (raw_steering - calib.steering_angle_offset) * calib.steering_angle_gain;

// 应用到EKF
ekf.state.delta = calibrated_steering;  // 转向角状态

// 转向盘中线验证
if (fabs(sensor_reading - calib.steering_angle_offset) < 0.01) {
    printf("转向盘居中\n");
}
```

---

## 三、IMU加速度计校准参数（6个）

### 3.1 零偏参数（imu_accel_*_bias）

| 属性 | 说明 |
|------|------|
| **参数名** | `imu_accel_x_bias`, `imu_accel_y_bias`, `imu_accel_z_bias` |
| **代码位置** | `localization_system.c` 第45-47行 |
| **功能** | 修正加速度计的静息零偏 |
| **默认值** | 0.0 m/s² |
| **可调范围** | -0.5 ~ +0.5 m/s² |
| **单位** | m/s² |
| **校准优先级** | ⭐⭐⭐ 必须校准 |
| **影响范围** | 加速度估计、轮滑检测 |

### 3.2 零偏标定方法：六位置法

```
步骤1：静止测量（最准确）
在6个不同的方向上各放置IMU 10秒：

方向1：+Z向上（正常）
方向2：-Z向上（倒立）
方向3：+X向上（侧放）
方向4：-X向上（侧放反向）
方向5：+Y向上（竖放）
方向6：-Y向上（竖放反向）

步骤2：数据采集
for each direction:
    readings[direction] = [a1, a2, ..., a100]  // 100个样本
    mean_accel[direction] = mean(readings[direction])

步骤3：零偏计算
在每个方向上，有一个轴是重力加速度，两个轴是噪声+零偏

// 以+Z向上为例
// measured_z ≈ 9.81 + bias_z
// measured_x ≈ 0 + bias_x + noise
// measured_y ≈ 0 + bias_y + noise

// 使用所有6个方向的数据，通过最小二乘法求解

步骤4：完整公式
// 对于加速度计零偏的标准做法（Allan Variance方法）

bias_x = (mean_accel[+X] + mean_accel[-X]) / 2
bias_y = (mean_accel[+Y] + mean_accel[-Y]) / 2
bias_z = (mean_accel[+Z] + mean_accel[-Z] - 2*g) / 2

// 例子
// +Z方向读数：[9.82, 9.80, 9.81, ...]  均值 9.81
// -Z方向读数：[-9.78, -9.80, -9.79, ...]  均值 -9.79
// bias_z = (9.81 + (-(-9.79)) - 2*9.81) / 2 = 0.01

bias_x_final = bias_x  // 直接使用
bias_y_final = bias_y
bias_z_final = bias_z
```

### 3.3 尺度因子参数（imu_accel_*_scale）

| 属性 | 说明 |
|------|------|
| **参数名** | `imu_accel_x_scale`, `imu_accel_y_scale`, `imu_accel_z_scale` |
| **代码位置** | `localization_system.c` 第48-50行 |
| **功能** | 修正加速度计的灵敏度（增益） |
| **默认值** | 1.0（无修正） |
| **可调范围** | 0.95 ~ 1.05 |
| **单位** | 无量纲（比例系数） |
| **校准优先级** | ⭐⭐ 较重要 |

### 3.4 尺度因子标定方法

```
步骤1：已知加速度标定
使用转台或已知重力加速度的方向

最简单的方法：利用重力加速度
- Z轴向上时，应该测得 +9.81 m/s²
- Z轴向下时，应该测得 -9.81 m/s²

步骤2：计算尺度因子
measured_z_up = 9.50 m/s²  // 实际读数
true_z_up = 9.81 m/s²      // 理论值

scale_z = true_z_up / (measured_z_up - bias_z)
scale_z = 9.81 / 9.50 = 1.0326

// 保守一点，取到两位小数
scale_z = 1.03

步骤3：校验
corrected_z = (measured_z - bias_z) * scale_z
            = (9.50 - 0.01) * 1.03
            = 9.82 ≈ 9.81 ✓
```

### 3.5 代码实现

```c
// 加速度计校准参数
CalibrationParameters_t calib;
calib.imu_accel_x_bias = 0.01;
calib.imu_accel_y_bias = -0.02;
calib.imu_accel_z_bias = 0.05;
calib.imu_accel_x_scale = 1.02;
calib.imu_accel_y_scale = 1.01;
calib.imu_accel_z_scale = 1.03;

// 加速度校准流程
Vector3_t raw_accel = read_imu_accel();

Vector3_t calibrated_accel;
calibrated_accel.x = (raw_accel.x - calib.imu_accel_x_bias) * calib.imu_accel_x_scale;
calibrated_accel.y = (raw_accel.y - calib.imu_accel_y_bias) * calib.imu_accel_y_scale;
calibrated_accel.z = (raw_accel.z - calib.imu_accel_z_bias) * calib.imu_accel_z_scale;

// 验证：静止时应该接近[0, 0, 9.81]
double gravity_magnitude = sqrt(
    calibrated_accel.x * calibrated_accel.x +
    calibrated_accel.y * calibrated_accel.y +
    calibrated_accel.z * calibrated_accel.z
);
if (fabs(gravity_magnitude - 9.81) > 0.1) {
    printf("Warning: 加速度计校准可能有问题\n");
}
```

---

## 四、IMU陀螺仪校准参数（1个）

### 4.1 imu_gyro_z_bias - Z轴角速度零偏

| 属性 | 说明 |
|------|------|
| **参数名** | `imu_gyro_z_bias` |
| **代码位置** | `localization_system.c` 第51行 |
| **功能** | 修正陀螺仪的静息零偏 |
| **默认值** | 0.0 rad/s |
| **可调范围** | -0.1 ~ +0.1 rad/s |
| **单位** | rad/s（弧度/秒） |
| **校准优先级** | ⭐⭐⭐ 必须校准 |
| **影响范围** | 航向角估计、转向反应 |

### 4.2 陀螺仪零偏标定方法

```
步骤1：静止采集
- 车辆完全静止
- 采集陀螺仪Z轴读数200个样本（10秒）

gyro_z_readings = [ω1, ω2, ..., ω200]

步骤2：计算零偏
gyro_z_bias = mean(gyro_z_readings)

// 示例
// 读数范围：-0.005 ~ +0.005 rad/s
// 均值：+0.002 rad/s
// 零偏 = 0.002 rad/s

步骤3：验证（漂移测试）
- 用与陀螺仪零偏相反的角速度旋转
- 观察航向角漂移速度
- 漂移速度 = |零偏| × 时间

// 如果零偏 = 0.002 rad/s
// 在1小时内，航向角会漂移
// 0.002 * 3600 = 7.2 rad ≈ 413° 
// 这太大了！必须校准。
```

### 4.3 代码实现

```c
CalibrationParameters_t calib;
calib.imu_gyro_z_bias = 0.002;  // rad/s

// 陀螺仪校准应用
double raw_gyro_z = read_imu_gyro_z();
double calibrated_gyro_z = raw_gyro_z - calib.imu_gyro_z_bias;

// 在EKF中积分航向角
heading_rate = calibrated_gyro_z;  // 正确的角速度
heading_angle += heading_rate * dt;  // 积分更新

// 长时间验证（1小时测试）
// 静止时，航向角偏差应该 < 5°
double heading_drift_per_hour = calib.imu_gyro_z_bias * 3600;
if (fabs(heading_drift_per_hour) > 0.087) {  // 5° = 0.087 rad
    printf("Warning: 陀螺仪零偏过大，航向漂移 %.1f°/小时\n", 
           heading_drift_per_hour * 180 / 3.14159);
}
```

---

## 五、IMU磁强计校准参数（2个）

### 5.1 imu_mag_x_bias 和 imu_mag_y_bias - 磁场零偏

| 属性 | 说明 |
|------|------|
| **参数名** | `imu_mag_x_bias`, `imu_mag_y_bias` |
| **代码位置** | `localization_system.c` 第52-53行 |
| **功能** | 修正磁强计的硬铁效应 |
| **默认值** | 0.0 μT |
| **可调范围** | -50 ~ +50 μT |
| **单位** | μT（微特斯拉） |
| **校准优先级** | ⭐⭐ 如果使用磁罗盘需要 |
| **影响范围** | 磁罗盘航向估计 |

### 5.2 磁强计校准背景

```
为什么需要磁强计校准：

硬铁效应（Hard Iron）：
- 来自车辆金属结构的永久磁化
- 表现为测量值有固定偏移
- 与旋转无关

软铁效应（Soft Iron）：
- 来自车辆铁磁材料的感应磁化
- 表现为不同方向的灵敏度不同
- 需要用缩放矩阵修正

这里只涉及硬铁效应（零偏修正）
```

### 5.3 磁强计校准方法：旋转法

```
步骤1：准备
- 在室外或已知磁场的室内
- 远离大型金属物体和磁铁
- 地磁强度约46-51 μT（随地理位置变化）

步骤2：采集数据
- 缓慢旋转车辆360°
- 采集磁强计在水平面上的读数
- 采集至少100个完整的XY对

步骤3：数据分析
// 绘制X-Y散点图
// 理想情况：以原点为中心的圆
// 实际情况：以某个偏心点为中心的圆（硬铁效应）

// 使用最小二乘法拟合圆
// 圆心坐标就是零偏值

步骤4：计算零偏
// 简化方法：取最大最小值的平均
mag_x_min = -30 μT
mag_x_max = +40 μT
bias_x = (mag_x_min + mag_x_max) / 2 = 5 μT

mag_y_min = -25 μT
mag_y_max = +35 μT
bias_y = (mag_y_min + mag_y_max) / 2 = 5 μT
```

### 5.4 代码实现

```c
CalibrationParameters_t calib;
calib.imu_mag_x_bias = 5.0;   // μT
calib.imu_mag_y_bias = 5.0;   // μT

// 磁强计校准应用
Vector2_t raw_mag_xy = read_imu_mag_horizontal();

Vector2_t calibrated_mag;
calibrated_mag.x = raw_mag_xy.x - calib.imu_mag_x_bias;
calibrated_mag.y = raw_mag_xy.y - calib.imu_mag_y_bias;

// 计算磁罗盘航向（北向0°，东向90°）
double magnetic_heading = atan2(calibrated_mag.y, calibrated_mag.x);

// 验证：旋转360°，应该形成圆心为零的圆
// 可以计算旋转中心的偏移作为监控指标
```

---

## 六、校准参数推荐配置

### 6.1 标准车辆配置

```c
CalibrationParameters_t calib_standard = {
    // 轮速计
    .wheel_calibration_factor = 1.0,
    
    // 转向角
    .steering_angle_offset = 0.0,      // rad
    .steering_angle_gain = 1.0,
    
    // 加速度计
    .imu_accel_x_bias = 0.0,           // m/s²
    .imu_accel_y_bias = 0.0,
    .imu_accel_z_bias = 0.0,
    .imu_accel_x_scale = 1.0,
    .imu_accel_y_scale = 1.0,
    .imu_accel_z_scale = 1.0,
    
    // 陀螺仪
    .imu_gyro_z_bias = 0.0,            // rad/s
    
    // 磁强计（如果不用磁罗盘可以忽略）
    .imu_mag_x_bias = 0.0,             // μT
    .imu_mag_y_bias = 0.0,
};
```

### 6.2 校准优先级排序

```
优先级1（立即进行）- 影响核心精度：
├─ wheel_calibration_factor  → 里程误差直接影响
├─ steering_angle_offset      → 直线行驶漂移
├─ imu_accel_z_bias          → 需要正确的重力参考
└─ imu_gyro_z_bias           → 航向漂移

预计时间：30分钟

优先级2（应该进行）- 改善精度：
├─ steering_angle_gain
├─ imu_accel_x_bias, y_bias
├─ imu_accel_x_scale, y_scale, z_scale
└─ imu_mag_x_bias, y_bias

预计时间：45分钟

优先级3（可选）- 微调：
└─ Z轴以外的陀螺仪零偏（如果需要3DOF陀螺仪）
```

---

## 七、快速校准流程（30分钟）

```
步骤1：轮速计（5分钟）
├─ 准备200m直线测距段
├─ 测试行驶→记录轮速累积距离
└─ wheel_calibration_factor = 参考距离 / 轮速距离

步骤2：转向角零偏（5分钟）
├─ 直线缓慢行驶100m，记录转向盘角度波动
├─ steering_angle_offset = mean(角度读数)
└─ 应用校准：修正后 = 原始 - offset

步骤3：加速度计零偏（10分钟）
├─ 在Z轴向上和向下两个位置各采集100个样本
├─ bias_z = (mean_up - mean_down) / 2
├─ bias_x = bias_y = 0（假设车辆水平安装）
└─ 应用到程序

步骤4：陀螺仪零偏（5分钟）
├─ 静止10秒采集陀螺仪读数
├─ gyro_z_bias = mean(读数)
└─ 应用：修正角速度 = 原始 - bias

步骤5：验证（5分钟）
├─ 直线行驶100m，检查位置漂移
├─ 旋转360°，检查航向角完整性
└─ 评估：是否需要重新标定
```

---

## 八、总结（Part 4）

### 核心原则

校准参数用于**消除传感器的系统误差**：

| 类别 | 参数 | 用途 | 优先级 |
|------|------|------|-------|
| **轮速计** | wheel_calibration_factor | 修正里程误差 | ⭐⭐⭐ |
| **转向角** | steering_angle_offset | 修正零点偏差 | ⭐⭐⭐ |
| | steering_angle_gain | 修正增益误差 | ⭐⭐ |
| **加速度计** | bias (3个) | 修正零偏 | ⭐⭐⭐ |
| | scale (3个) | 修正灵敏度 | ⭐⭐ |
| **陀螺仪** | gyro_z_bias | 修正零偏 | ⭐⭐⭐ |
| **磁强计** | mag_bias (2个) | 修正硬铁效应 | ⭐⭐ |

### 标定耗时估计

```
快速标定（最少功能）：      30分钟
  └─ 轮速+转向+IMU基本参数

完整标定（推荐）：          90分钟
  └─ 所有12个参数逐一标定

精准标定（最优质量）：      2小时
  └─ 多次重复测试+统计分析
```

### 标定后检验清单

```
✓ 直线行驶100m，位置漂移 < 1m
✓ 旋转360°，航向角完整回到起点（偏差 < 5°）
✓ 静止1小时，头部漂移 < 10°
✓ 轮速与实际行驶距离对比 < 1%误差
✓ 转向盘回中后，车辆应倾向直线行驶
```

---

## Part 5：调参决策树 + 实际工作流程

本部分提供**完整的调参方法和决策框架**，帮助工程师快速诊断问题并进行有针对性的调参。

---

## 一、故障症状 → 参数诊断决策树

### 1.1 位置精度问题

```
症状：定位漂移（累积误差快速增长）
├─ 如果是直线行驶向一侧漂移
│  ├─ 检查：steering_angle_offset（转向零偏）
│  │  └─ 调整：校准转向盘中心位置
│  └─ 检查：filter_alpha（转向滤波系数）
│     └─ 调整：可能需要降低（减少滞后）
│
├─ 如果是里程持续偏小或偏大
│  ├─ 检查：wheel_calibration_factor（轮速校准）
│  │  └─ 调整：重新标定轮速计
│  ├─ 检查：r_wheel（轮速测量噪声）
│  │  └─ 调整：增大r_wheel（不相信轮速）
│  └─ 检查：轮胎气压
│     └─ 调整：恢复标准气压后重新标定
│
└─ 如果是位置在某个方向波动
   ├─ 检查：r_wheel 或 r_imu（测量噪声过小）
   │  └─ 调整：增大相应的R值
   └─ 检查：Q矩阵（过程噪声）
      └─ 调整：增大相应的Q值
```

### 1.2 航向精度问题

```
症状：航向角漂移（头部方向不稳定）
├─ 如果是持续朝一个方向转
│  ├─ 检查：imu_gyro_z_bias（陀螺仪零偏）
│  │  └─ 调整：重新标定陀螺仪
│  ├─ 检查：r_steering（转向测量噪声）
│  │  └─ 调整：增大r_steering（不相信转向）
│  └─ 检查：filter_alpha（转向滤波）
│     └─ 调整：可能需要提高（更相信传感器）
│
├─ 如果是快速波动（抖动）
│  ├─ 检查：filter_alpha（转向滤波系数太大）
│  │  └─ 调整：减小filter_alpha（增加平滑）
│  ├─ 检查：jitter_threshold（抖动阈值）
│  │  └─ 调整：增大阈值（忽略小抖动）
│  ├─ 检查：r_steering（过小，过度相信转向）
│  │  └─ 调整：增大r_steering
│  └─ 检查：q_delta（转向过程噪声）
│     └─ 调整：减小q_delta
│
└─ 如果是慢速漂移（几秒内明显偏离）
   ├─ 检查：q_theta（航向过程噪声）
   │  └─ 调整：增大q_theta（允许航向变化）
   └─ 检查：r_imu（IMU噪声）
      └─ 调整：增大r_imu（减少对加速度的依赖）
```

### 1.3 轮滑检测问题

```
症状：轮滑时定位发散
├─ 如果是检测不出轮滑（持续漂移）
│  ├─ 检查：r_imu（IMU测量噪声过大）
│  │  └─ 调整：减小r_imu（更相信IMU）
│  ├─ 检查：轮滑检测阈值
│  │  └─ 调整：降低阈值（更容易触发检测）
│  └─ 检查：q_v（速度过程噪声）
│     └─ 调整：减小q_v（对速度误差更敏感）
│
└─ 如果是过度检测轮滑（误检）
   ├─ 检查：r_imu（过小，过度相信IMU噪声）
   │  └─ 调整：增大r_imu
   └─ 检查：轮滑检测阈值
      └─ 调整：提高阈值（减少误检）
```

---

## 二、3阶段标定流程

### 阶段1：快速标定（30分钟）

目标：建立可用的基本参数集

```
1. 轮速计标定（5分钟）
   步骤：直线行驶200m → 对比轮速里程 → 得到wheel_calibration_factor
   验证：误差 < 2%

2. 转向零偏标定（5分钟）
   步骤：直线行驶100m，记录转向角波动 → 计算平均 → 得到steering_angle_offset
   验证：直线偏离 < 2m

3. 加速度计零偏（10分钟）
   步骤：Z轴上下两个方向各采集100样本 → 计算差值 → 得到imu_accel_*_bias
   验证：静止时加速度接近[0, 0, 9.81]

4. 陀螺仪零偏（5分钟）
   步骤：静止采集200样本 → 计算平均 → 得到imu_gyro_z_bias
   验证：1小时内头部漂移 < 10°

5. R矩阵初始化（5分钟）
   步骤：使用标准值（r_wheel=0.01, r_imu=0.05, r_steering=0.05）
   验证：位置和航向都有合理的估计
```

### 阶段2：精细标定（60分钟）

目标：优化噪声参数和增益参数

```
1. Q矩阵精细标定（20分钟）
   ├─ 位置噪声(q_x, q_y)：
   │  方法：直线测试，观察定位与实际位置偏差
   │  调整：偏差大→增大Q，波动→减小Q
   │
   ├─ 航向噪声(q_theta)：
   │  方法：圆形行驶，观察航向估计效果
   │  调整：跟踪滞后→增大Q，波动→减小Q
   │
   ├─ 速度噪声(q_v)：
   │  方法：加速减速测试，观察速度跟踪
   │  调整：反应慢→增大Q，波动→减小Q
   │
   └─ 转向噪声(q_delta)：
      方法：转向跟踪测试
      调整：同q_theta

2. R矩阵精细标定（20分钟）
   ├─ r_wheel：多段测试，计算轮速误差方差
   ├─ r_imu：加减速测试，计算IMU误差方差
   └─ r_steering：转向测试，计算转向误差方差

3. 转向滤波优化（10分钟）
   ├─ filter_alpha：调整平滑度（0.05-0.8）
   └─ jitter_threshold：调整抖动灵敏度（0.001-0.1）

4. 传感器增益校准（10分钟）
   ├─ steering_angle_gain：转向线性度
   ├─ imu_accel_*_scale：加速度灵敏度
   └─ wheel_calibration_factor：精细调整
```

### 阶段3：验证测试（30分钟）

目标：验证参数质量，进行最终调整

```
1. 直线行驶测试（5分钟）
   ├─ 距离：200m
   ├─ 检验项：
   │  ├─ 位置漂移 < 1m（0.5%）
   │  └─ 直线偏离 < 0.5m
   └─ 不通过→回到阶段2调整Q矩阵

2. 回转测试（5分钟）
   ├─ 方式：完整360°回转
   ├─ 检验项：
   │  ├─ 航向角回到原点（误差 < 5°）
   │  └─ 圆形轨迹完整
   └─ 不通过→调整q_theta或r_steering

3. 加减速测试（5分钟）
   ├─ 方式：从0加速到2m/s，再减速到0
   ├─ 检验项：
   │  ├─ 位置与IMU一致（误差 < 5%）
   │  └─ 轮滑检测正确
   └─ 不通过→调整q_v或r_imu

4. 长时间稳定性测试（10分钟）
   ├─ 方式：静止10分钟
   ├─ 检验项：
   │  ├─ 航向角漂移 < 10°
   │  └─ 位置不漂移
   └─ 不通过→调整陀螺仪或轮速零偏

5. 极端工况测试（5分钟）
   ├─ 快速转向
   ├─ 急加速/急减速
   ├─ 轮滑（如果支持）
   └─ 验证系统稳定性
```

---

## 三、22个参数完整快速参考表

### 3.1 转向过滤参数（2个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 调大效果 | 调小效果 | 标定优先级 |
|---|--------|--------|-------|------|------|---------|---------|-----------|
| 1 | filter_alpha | loc_sys.c:176 | 0.2 | 0.05~0.8 | - | 跟踪快但抖动 | 平滑但滞后 | ⭐⭐ |
| 2 | jitter_threshold | loc_sys.c:176 | 0.03 | 0.001~0.1 | rad² | 忽略更多抖动 | 检测更灵敏 | ⭐⭐ |

### 3.2 Q矩阵 - 过程噪声（5个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 调大效果 | 调小效果 | 标定优先级 |
|---|--------|--------|-------|------|------|---------|---------|-----------|
| 3 | q_x | loc_sys.c:27 | 0.005 | 0.001~0.05 | (m)² | 跟踪快 | 位置稳定 | ⭐⭐⭐ |
| 4 | q_y | loc_sys.c:28 | 0.005 | 0.001~0.05 | (m)² | 跟踪快 | 位置稳定 | ⭐⭐⭐ |
| 5 | q_theta | loc_sys.c:29 | 0.01 | 0.001~0.05 | (rad)² | 航向快速变化 | 航向稳定 | ⭐⭐⭐ |
| 6 | q_v | loc_sys.c:30 | 0.02 | 0.01~0.1 | (m/s)² | 速度快速变化 | 速度稳定 | ⭐⭐⭐ |
| 7 | q_delta | loc_sys.c:31 | 0.02 | 0.01~0.05 | (rad)² | 转向快速变化 | 转向稳定 | ⭐⭐ |

### 3.3 R矩阵 - 测量噪声（3个）⭐必须标定

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 调大效果 | 调小效果 | 标定优先级 |
|---|--------|--------|-------|------|------|---------|---------|-----------|
| 8 | r_wheel | loc_sys.c:33 | 0.01 | 0.001~0.1 | (m/s)² | 不信轮速 | 信任轮速 | ⭐⭐⭐ |
| 9 | r_imu | loc_sys.c:34 | 0.1 | 0.01~1.0 | (m/s²)² | 不信IMU | 信任IMU | ⭐⭐⭐ |
| 10 | r_steering | loc_sys.c:35 | 0.05 | 0.01~0.2 | (rad)² | 不信转向 | 信任转向 | ⭐⭐⭐ |

### 3.4 校准参数 - 轮速（1个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 功能 | 标定优先级 |
|---|--------|--------|-------|------|------|------|-----------|
| 11 | wheel_calibration_factor | loc_sys.c:42 | 1.0 | 0.9~1.1 | - | 轮速增益修正 | ⭐⭐⭐ |

### 3.5 校准参数 - 转向角（2个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 功能 | 标定优先级 |
|---|--------|--------|-------|------|------|------|-----------|
| 12 | steering_angle_offset | loc_sys.c:43 | 0.0 | ±0.3 | rad | 转向零偏修正 | ⭐⭐⭐ |
| 13 | steering_angle_gain | loc_sys.c:44 | 1.0 | 0.9~1.1 | - | 转向增益修正 | ⭐⭐ |

### 3.6 校准参数 - IMU加速度（6个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 功能 | 标定优先级 |
|---|--------|--------|-------|------|------|------|-----------|
| 14 | imu_accel_x_bias | loc_sys.c:45 | 0.0 | ±0.5 | m/s² | X轴零偏 | ⭐⭐⭐ |
| 15 | imu_accel_y_bias | loc_sys.c:46 | 0.0 | ±0.5 | m/s² | Y轴零偏 | ⭐⭐⭐ |
| 16 | imu_accel_z_bias | loc_sys.c:47 | 0.0 | ±0.5 | m/s² | Z轴零偏 | ⭐⭐⭐ |
| 17 | imu_accel_x_scale | loc_sys.c:48 | 1.0 | 0.95~1.05 | - | X轴灵敏度 | ⭐⭐ |
| 18 | imu_accel_y_scale | loc_sys.c:49 | 1.0 | 0.95~1.05 | - | Y轴灵敏度 | ⭐⭐ |
| 19 | imu_accel_z_scale | loc_sys.c:50 | 1.0 | 0.95~1.05 | - | Z轴灵敏度 | ⭐⭐ |

### 3.7 校准参数 - IMU陀螺仪（1个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 功能 | 标定优先级 |
|---|--------|--------|-------|------|------|------|-----------|
| 20 | imu_gyro_z_bias | loc_sys.c:51 | 0.0 | ±0.1 | rad/s | Z轴零偏 | ⭐⭐⭐ |

### 3.8 校准参数 - IMU磁强计（2个）

| # | 参数名 | 代码位置 | 默认值 | 范围 | 单位 | 功能 | 标定优先级 |
|---|--------|--------|-------|------|------|------|-----------|
| 21 | imu_mag_x_bias | loc_sys.c:52 | 0.0 | ±50 | μT | X轴硬铁偏差 | ⭐⭐ |
| 22 | imu_mag_y_bias | loc_sys.c:53 | 0.0 | ±50 | μT | Y轴硬铁偏差 | ⭐⭐ |

---

## 四、3种场景的推荐参数集合

### 场景1：自动泊车（0.5-2.0 m/s）✓ 标准推荐

```c
// 转向过滤（平衡精度和平滑）
.filter_alpha = 0.2,           // 标准平滑度
.jitter_threshold = 0.03,      // 标准抖动阈值

// Q矩阵（过程噪声）
.q_x = 0.005,                  // 位置信任中等
.q_y = 0.005,
.q_theta = 0.01,               // 航向信任中等
.q_v = 0.02,                   // 速度信任中等
.q_delta = 0.02,               // 转向信任中等

// R矩阵（测量噪声）
.r_wheel = 0.01,               // 轮速信任高
.r_imu = 0.05,                 // IMU信任中等
.r_steering = 0.05,            // 转向信任中等

// 校准参数（标准传感器）
.wheel_calibration_factor = 1.0,
.steering_angle_offset = 0.0,
.steering_angle_gain = 1.0,
.imu_accel_x_bias = 0.0,
.imu_accel_y_bias = 0.0,
.imu_accel_z_bias = 0.0,
.imu_accel_x_scale = 1.0,
.imu_accel_y_scale = 1.0,
.imu_accel_z_scale = 1.0,
.imu_gyro_z_bias = 0.0,
.imu_mag_x_bias = 0.0,
.imu_mag_y_bias = 0.0,
```

**特点**：
- ✓ 平衡精度和稳定性
- ✓ 适合大多数应用场景
- ✓ 推荐先用此配置

### 场景2：极低速（<0.3 m/s）

```c
// 转向过滤（更灵敏）
.filter_alpha = 0.1,           // 更多滤波，减少小幅抖动
.jitter_threshold = 0.05,      // 更不敏感，忽略小抖动

// Q矩阵（对速度变化更敏感）
.q_x = 0.001,                  // 低速时位置信息不可靠，降低Q
.q_y = 0.001,
.q_theta = 0.01,               // 保持不变
.q_v = 0.03,                   // 增大，更容易检测到速度变化
.q_delta = 0.01,               // 降低，转向更稳定

// R矩阵（更不信任轮速）
.r_wheel = 0.05,               // 增大，低速轮速误差大
.r_imu = 0.1,                  // 增大，依赖更多传感器融合
.r_steering = 0.03,            // 降低，转向更可靠

// 校准参数（同标准）
// ... (与场景1相同)
```

**特点**：
- ✓ 适合自动泊车最后停靠阶段
- ✓ 对轮滑更敏感
- ✓ 位置精度可能降低

### 场景3：快速行驶（>3.0 m/s）

```c
// 转向过滤（更快速）
.filter_alpha = 0.3,           // 更快的跟踪
.jitter_threshold = 0.01,      // 更敏感

// Q矩阵（对运动更敏感）
.q_x = 0.01,                   // 增大，快速变化更多
.q_y = 0.01,
.q_theta = 0.02,               // 增大，转向更快
.q_v = 0.05,                   // 增大，速度变化更快
.q_delta = 0.03,               // 增大，转向变化更快

// R矩阵（更信任轮速）
.r_wheel = 0.001,              // 减小，高速轮速准确
.r_imu = 0.01,                 // 减小，重点依赖轮速
.r_steering = 0.1,             // 增大，高速转向更动态

// 校准参数（同标准）
// ... (与场景1相同)
```

**特点**：
- ✓ 适合高速行驶
- ✓ 快速跟踪运动变化
- ✓ 对过程噪声更包容

---

## 五、调参快速流程图

```
开始
  │
  ├─ 导入标准参数集（场景1：自动泊车）
  │   
  ├─ 进行3阶段标定
  │   ├─ 阶段1：快速标定（30分钟）✓
  │   │   └─ 校准参数（wheel, steering, IMU零偏）
  │   │
  │   ├─ 阶段2：精细标定（60分钟）
  │   │   ├─ Q矩阵调整
  │   │   ├─ R矩阵精确测量
  │   │   └─ 转向滤波参数
  │   │
  │   └─ 阶段3：验证测试（30分钟）
  │       └─ 各种工况验证
  │
  ├─ 测试遇到问题？
  │   ├─ 位置漂移 → 查阅"位置精度问题"决策树
  │   ├─ 航向漂移 → 查阅"航向精度问题"决策树
  │   └─ 轮滑问题 → 查阅"轮滑检测问题"决策树
  │
  └─ 完成！保存参数配置
```

---

## 六、调参优先级速查表

### 最需要调参的指标排序

```
1. wheel_calibration_factor  - 不调不行，直接影响里程
2. steering_angle_offset      - 不调不行，导致侧向漂移
3. imu_gyro_z_bias            - 不调不行，导致航向漂移
4. q_x, q_y                   - 影响位置跟踪速度
5. q_theta                    - 影响航向跟踪速度
6. r_wheel, r_steering        - 影响噪声滤波效果
7. filter_alpha               - 影响转向平滑度
8. imu_accel_*_bias           - 影响加速度精度
9. r_imu                      - 影响加速度权重
10. q_v, q_delta              - 影响动态响应
11. steering_angle_gain       - 微调转向增益
12. imu_accel_*_scale         - 微调传感器灵敏度
13. jitter_threshold          - 微调抖动过滤
14. imu_mag_*_bias            - 仅在使用磁罗盘时需要
```

---

## 七、常见问题与快速解决

### Q: 参数太多了，我应该从哪里开始？
A: 按以下顺序：
1. wheel_calibration_factor（直线行驶200m对标，5分钟）
2. steering_angle_offset（直线行驶100m观察，5分钟）
3. imu_gyro_z_bias（静止采集，5分钟）
4. 使用标准Q、R矩阵值
5. 如果还有问题再精细调参

### Q: 参数什么时候需要重新标定？
A: 以下情况需要重新标定：
- 更换轮胎 → 重新标定 wheel_calibration_factor
- 更换或重装传感器 → 重新标定所有校准参数
- 长期使用后（>3个月） → 重新标定R矩阵
- 更换工作环境（温度变化大） → 重新标定IMU

### Q: 调参时应该一次改一个参数吗？
A: 是的。每次只改一个参数：
1. 记录当前参数值和当前效果
2. 改一个参数
3. 测试并评估效果
4. 记录结果
5. 恢复原始值，再改下一个参数

### Q: 参数改多少才合适？
A: 不超过20%：
- 第一次调整：改10%（测试灵敏度）
- 第二次调整：改5-10%（精细调整）
- 第三次调整：改1-5%（最后微调）

---

## 八、总结（Part 5 - 最终指南）

### 核心要点

| 阶段 | 重点 | 耗时 | 结果 |
|------|------|------|------|
| **快速** | 校准参数 | 30分钟 | 可用系统 |
| **精细** | 噪声参数 | 60分钟 | 性能优化 |
| **验证** | 工况测试 | 30分钟 | 质量保证 |

### 参数调参顺序（优先级）

```
第一批（立即调）：
├─ wheel_calibration_factor
├─ steering_angle_offset
├─ imu_gyro_z_bias
└─ IMU零偏（imu_accel_*_bias）

第二批（重点调）：
├─ q_x, q_y（位置跟踪）
├─ q_theta（航向跟踪）
├─ r_wheel, r_steering（信任度）
└─ filter_alpha（转向平滑）

第三批（微调）：
├─ q_v, q_delta
├─ r_imu
├─ steering_angle_gain
└─ imu_accel_*_scale
```

### 质量评估标准

```
✅ 基本可用：
  - 直线行驶100m误差 < 5m
  - 航向偏差 < 20°
  - 大多数工况下定位正确

✅ 良好精度：
  - 直线行驶200m误差 < 1m
  - 航向偏差 < 5°
  - 拐弯轨迹较为精确

✅ 优异精度：
  - 直线行驶200m误差 < 0.5m（0.25%）
  - 航向偏差 < 2°
  - 拐弯轨迹非常精确
  - 长时间稳定性好
```

### 文档使用建议

```
场景1：快速排查问题
→ 参考"故障症状→参数诊断决策树"

场景2：准备标定工作
→ 参考"3阶段标定流程"

场景3：查询参数信息
→ 参考"22个参数完整快速参考表"

场景4：参数初始化
→ 参考"3种场景的推荐参数集合"

场景5：调参思路不清
→ 参考"调参优先级速查表"和"常见问题"
```

---

## 附录：参数调试检验清单

### 每次调参后的验证步骤

```
□ 编译代码
  → 确保没有编译错误

□ 直线行驶测试（200m）
  → 位置精度检查
  → 应满足 < 1% 误差

□ 旋转测试（360°）
  → 航向完整性检查
  → 应回到起点（< 5°误差）

□ 加减速测试
  → 动态响应检查
  → 应平稳无大波动

□ 长时间稳定性测试（10分钟静止）
  → 漂移检查
  → 位置应不动，航向 < 10°漂移

□ 极端工况（快转向、急加减速）
  → 鲁棒性检查
  → 系统应保持稳定

□ 参数保存
  → 将最终参数保存到配置文件
  → 记录标定日期和环境条件
  → 备份旧参数以便对比
```

---

## 结束语

本文档覆盖了**车辆定位系统的全部22个参数**的：
- 📖 基本概念和参数分类
- 🔧 详细的标定和调参方法
- 🛠️ 代码实现示例
- 📊 完整的快速参考表
- 🎯 实际工作流程和决策树
- ⚡ 快速诊断和解决方案

**建议使用方法**：
1. 第一次使用：从Part 1开始，逐部分理解系统
2. 日常标定：直接跳到Part 5，按流程操作
3. 问题诊断：查询Part 5的决策树
4. 参数查询：使用Part 5的快速参考表

**预期效果**：
- ✓ 初学者可在2小时内完成系统标定
- ✓ 经验者可在30分钟内完成快速标定
- ✓ 优化者可在2小时内实现高精度调参

**版本信息**：
- 📅 文档创建时间：2025年12月16日
- 🎯 适用范围：EKF多传感器融合定位系统
- 📋 覆盖参数：22个（2个转向过滤 + 5个Q矩阵 + 3个R矩阵 + 12个校准参数）