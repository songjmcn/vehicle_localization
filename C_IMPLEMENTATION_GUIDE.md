# C语言实现的车辆定位系统 - 自动泊车版本

## 系统概述

这是一个专门为自动泊车场景（0.5 m/s ~ 2.0 m/s）设计的精确车辆定位系统。采用扩展卡尔曼滤波（EKF）融合多个传感器数据，并集成RTK标定功能。

## 核心特性

### 1. 多传感器融合
- **轮速计（Wheel Odometry）**：提供速度和距离信息
- **IMU（惯性测量单元）**：提供加速度和角速度
- **转向角传感器**：提供方向盘转角
- **RTK GNSS**：提供高精度位置标定

### 2. 运动模型
采用改进的单轨模型，特别优化了低速和前后退场景：

```
前进时：
  x = x + v * cos(θ + δ) * dt
  y = y + v * sin(θ + δ) * dt
  θ = θ + v * tan(δ) / L * dt

后退时：
  x = x - |v| * cos(θ - δ) * dt
  y = y - |v| * sin(θ - δ) * dt
  θ = θ - |v| * tan(δ) / L * dt
```

其中v为速度，δ为转向角，L为轴距。

### 3. 卡尔曼滤波
- **状态向量**：[x, y, θ, v, δ]（位置、航向角、速度、转向角）
- **动态模型**：基于运动模型的非线性预测
- **测量模型**：融合三个传感器的读数
- **自适应噪声**：根据轮滑动检测动态调整权重

### 4. RTK标定
- 收集定位过程中的RTK数据
- 计算轮速计校准因子和转向角零偏
- 支持在线和离线校准
- 自动应用校准参数

## 文件结构

```
vehicle_localization/
├── localization_types.h       # 数据类型定义
├── motion_model.h/c           # 运动模型实现
├── kalman_filter.h/c          # EKF实现
├── rtk_calibration.h/c        # RTK标定模块
├── localization_system.h/c    # 完整系统集成
├── example_usage.c            # 使用示例
├── Makefile                   # 编译配置
└── C_IMPLEMENTATION_GUIDE.md  # 本文档
```

## 编译和运行

### 编译

```bash
# 编译所有模块
make

# 仅清理构建文件
make clean

# 编译并运行示例
make run
```

### 编译选项

默认使用GCC编译器，C99标准。编译标志：
- `-Wall -Wextra`：启用所有警告
- `-O2`：二级优化
- `-std=c99`：使用C99标准
- `-lm`：链接数学库

## 系统接口

### 初始化系统

```c
LocalizationSystem_t localization_sys;
VehicleState_t initial_state = {.x=0, .y=0, .theta=0, .v=0, .delta=0};

localization_system_init(&localization_sys, &initial_state);
```

### 处理传感器数据

```c
SensorReadings_t sensor_data;
// 填充sensor_data...

VehicleState_t estimated_state;
localization_system_process(&localization_sys, &sensor_data, &estimated_state);

printf("Position: (%.2f, %.2f) m\n", estimated_state.x, estimated_state.y);
printf("Heading: %.2f rad\n", estimated_state.theta);
```

### RTK离线标定

```c
// 收集标定数据（多个点）
RTKData_t rtk_data;
// 填充rtk_data...
// 获取里程计估计状态
VehicleState_t odometry_state;
localization_system_get_state(&localization_sys, &odometry_state);
// 添加RTK标定点（离线模式：需要同时提供里程计和RTK状态）
localization_system_rtk_calibration_add_point(&localization_sys, &odometry_state, &rtk_data);

// 计算并应用校准参数
localization_system_apply_rtk_calibration(&localization_sys);

// 检查校准结果
double avg_error = rtk_calibration_get_avg_position_error(&localization_sys.rtk_calibration);
printf("平均位置误差: %.3f m\n", avg_error);
```

### 诊断信息

```c
DiagnosticsInfo_t diag = localization_system_get_diagnostics(&localization_sys);

if (diag.is_wheel_slipping) {
    printf("警告：轮滑动！加速度: %.2f m/s²\n", diag.wheel_slip_accel);
}

if (diag.is_stationary) {
    printf("车辆静止\n");
}

printf("更新计数: %u\n", diag.update_count);
```

## 低速泊车优化

### 1. 静止检测
```c
#define V_STATIC_THRESHOLD 0.05  /* 静止速度阈值 */
```
当速度低于此阈值时，系统：
- 停止更新位置
- 继续更新角度（基于陀螺仪）
- 降低过程噪声

### 2. 轮滑动检测
```c
#define WHEEL_SLIP_ACCEL_THRESHOLD 0.3  /* m/s² */
```
当检测到轮滑动时：
- 增加轮速计测量噪声权重（4倍）
- 降低轮速计对状态的影响
- 提高IMU的相对权重

### 3. 自适应参数
低速状态下的推荐噪声参数：
```c
NoiseParameters_t noise = {
    .q_x = 0.005,        /* 位置x噪声 */
    .q_y = 0.005,        /* 位置y噪声 */
    .q_theta = 0.01,     /* 角度噪声 */
    .q_v = 0.02,         /* 速度噪声（高于常规值） */
    .q_delta = 0.02,     /* 转向角噪声 */
    .r_wheel = 0.01,     /* 轮速计测量噪声 */
    .r_imu = 0.1,        /* IMU加速度测量噪声 */
    .r_steering = 0.05   /* 转向角测量噪声 */
};
```

### 4. 前后退支持
运动模型自动处理：
- 检测速度符号
- 在后退时反转转向角
- 保持一致的航向角更新

## 校准工作流程

### 步骤1：收集数据
运行车辆进行典型的泊车操作（直行、转向、后退），同时收集RTK数据：

```c
for (int i = 0; i < num_readings; i++) {
    // 获取传感器数据
    SensorReadings_t sensor_data = get_sensor_reading();
    
    // 运行定位
    VehicleState_t state;
    localization_system_process(&localization_sys, &sensor_data, &state);
    
    // 获取RTK数据（GPS频率通常为1-10 Hz）
    RTKData_t rtk_data = get_rtk_reading();
    if (rtk_data.quality >= 2) {  // 质量检查
        VehicleState_t odometry = localization_system_get_state(&localization_sys);
        localization_system_rtk_calibration_add_point(&localization_sys, &odometry, &rtk_data);
    }
}
```

### 步骤2：计算校准参数
```c
// 至少需要3个标定点
if (localization_system_apply_rtk_calibration(&localization_sys) == 0) {
    // 校准成功
    CalibrationParams_t cal = localization_sys.calibration;
    printf("轮速计校准因子: %.4f\n", cal.wheel_calibration_factor);
    printf("转向角零偏: %.4f rad\n", cal.steering_angle_offset);
}
```

### 步骤3：验证与应用
```c
double avg_pos_error = rtk_calibration_get_avg_position_error(&localization_sys.rtk_calibration);
double avg_heading_error = rtk_calibration_get_avg_heading_error(&localization_sys.rtk_calibration);

if (avg_pos_error < 0.1 && avg_heading_error < 0.1) {
    // 校准质量良好，应用参数
    // 校准参数已自动应用
    printf("✓ 校准成功！\n");
} else {
    // 收集更多数据重新校准
    printf("校准质量不足，请收集更多数据\n");
}
```

### 步骤4：在线微调（可选）
```c
// 在实际运行中继续优化
RTKData_t rtk = get_rtk_reading();
rtk_calibration_online_update(&localization_sys.rtk_calibration,
                              &current_state, &rtk, 0.05);  // alpha=0.05
```

## 性能指标

| 指标 | 目标值 | 说明 |
|------|-------|------|
| 位置精度（室外） | < 0.1 m | 1秒内累计误差 |
| 航向精度 | < 2° | 航向角误差 |
| 响应延迟 | < 100 ms | 转向响应时间 |
| 更新频率 | 20-50 Hz | 推荐频率 |
| 静止漂移 | < 0.05 m/min | 长期静止状态 |
| RTK校准后精度 | < 0.05 m | 使用RTK标定后 |

## 常见问题和解决方案

### 问题1：位置持续漂移
**症状**：静止时位置不断累加误差

**原因**：
- IMU零偏未校准
- 轮速计校准因子不准确

**解决方案**：
1. 收集更多RTK标定数据（至少10个点）
2. 在静止状态下多次校准
3. 检查传感器连接

### 问题2：转向滞后
**症状**：转向角变化后，航向角响应延迟

**原因**：
- 转向系统本身的延迟
- 卡尔曼滤波参数不匹配

**解决方案**：
1. 增加 `q_delta`（转向角过程噪声）
2. 减少 `r_steering`（转向角测量噪声）
3. 引入显式延迟补偿模型

### 问题3：倒车不精确
**症状**：倒车时位置和航向角误差大

**原因**：
- 运动模型在后退时不准确
- 轮滑动未正确检测

**解决方案**：
1. 降低后退速度（< 0.5 m/s）
2. 增加后退时的 `q_v`
3. 收集专门的后退RTK数据进行校准

## 内存使用

系统内存占用（典型配置）：
- RTK标定管理器（100个点）：约8 KB
- 卡尔曼滤波器状态：约200字节
- 总计：约8.2 KB（RAM）

可通过修改 `max_points` 参数调整：
```c
rtk_calibration_init(&manager, 200);  // 200个点，约16 KB
```

## 实时性考虑

### 计算复杂度
- 预测步骤：O(n²)，其中n=5（状态维数）
- 更新步骤：O(n³)（矩阵求逆）
- 单次更新：< 1 ms（典型ARM处理器）

### 实现建议
1. 使用固定大小的矩阵（避免动态分配）
2. 优先级中断处理传感器数据
3. 后台线程进行RTK处理

## 扩展和自定义

### 修改轴距
```c
#define WHEELBASE 2.7  /* 修改此值 */
```

### 调整噪声参数
```c
NoiseParameters_t custom_noise = {
    .q_x = 0.01,      /* 根据实际调整 */
    .q_y = 0.01,
    // ...
};
kalman_filter_init(&kf, &state, &custom_noise, &cal);
```

### 添加其他传感器
可以扩展测量模型和传感器数据结构：
```c
typedef struct {
    // 现有的传感器
    IMUData_t imu;
    // 添加新传感器
    double compass_heading;
    double gps_velocity;
} SensorReadings_t;
```

## 参考资料

### 相关论文
1. Thrun, S., Burgard, W., & Fox, D. (2005). "Probabilistic Robotics"
2. Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). "Estimation with Applications"

### 标准
- SAE J2945: Vehicle-to-Everything (V2X) Communications
- ISO 19141: Road vehicles — Test track for automated driving systems

## 许可证

本代码为示例实现，可自由使用和修改。

## 维护和更新

最后更新：2024年12月
版本：1.0
