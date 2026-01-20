# Python 车辆定位系统 - 快速开始指南

## 📦 模块安装和导入

```python
# 基础导入
from python.types import (
    VehicleState, VehicleGear, WheelDirection, VehicleDirection, FusionMode,
    SensorReadings, WheelOdometryData, SteeringData, IMUData,
    WheelConfig, SteeringConfig, NoiseParameters, CalibrationParams, FusionConfig,
    DiagnosticsInfo
)
from python.localization_system import LocalizationSystem

# 可选的低级导入
from python.wheel_odometry import WheelOdometryProcessor, derive_vehicle_direction
from python.steering_model import SteeringModel
from python.kalman_filter import KalmanFilter
from python.motion_model import predict, jacobian
```

## 🚀 基础使用示例

### 方式1：完整系统集成（推荐）

```python
# 创建定位系统实例
localization = LocalizationSystem(
    wheel_config=WheelConfig(),
    steering_config=SteeringConfig(),
    noise_params=NoiseParameters(),
    fusion_config=FusionConfig(mode=FusionMode.BASIC)
)

# 处理每一帧传感器数据
sensor_data = SensorReadings(
    timestamp=1000,
    wheel_odometry=WheelOdometryData(velocity=1.5, distance=0.0),
    steering=SteeringData(steering_angle=0.1),
    imu=IMUData(accel_x=0.1, accel_y=0.0, accel_z=9.8),
    gear=VehicleGear.DRIVE,
    rear_left_wheel_count=100,
    rear_right_wheel_count=100,
    rear_left_wheel_direction=WheelDirection.FORWARD,
    rear_right_wheel_direction=WheelDirection.FORWARD,
    has_wheel_pulse=True
)

# 执行融合，获取估计状态
state = localization.process(sensor_data)

print(f"位置: ({state.x:.2f}, {state.y:.2f}) m")
print(f"航向: {state.heading:.3f} rad")
print(f"速度: {state.v:.2f} m/s")
print(f"方向: {state.direction}")

# 获取诊断信息
diag = localization.get_diagnostics()
if diag.is_wheel_slipping:
    print("⚠️ 轮滑检测")
if diag.is_stationary:
    print("🛑 车辆静止")
```

### 方式2：向后兼容的字典输入

```python
# 使用简化的字典格式
sample = {
    'timestamp': 1000,
    'wheel_velocity': 1.5,
    'wheel_distance': 0.0,
    'steering_wheel_angle': 0.1,
    'imu_ax': 0.1,
    'imu_ay': 0.0,
    'imu_az': 9.8,
    'gear': VehicleGear.DRIVE,
    'left_pulse': 100,
    'right_pulse': 100,
    'rear_left_direction': WheelDirection.FORWARD,
    'rear_right_direction': WheelDirection.FORWARD,
    'has_wheel_pulse': True
}

state = localization.process_sample(sample)
```

## 🔧 常见配置

### 基础配置（默认）
```python
localization = LocalizationSystem()  # 使用所有默认参数
```

### 高性能配置（低延迟）
```python
localization = LocalizationSystem(
    wheel_config=WheelConfig(pulse_timeout_ms=200),
    noise_params=NoiseParameters(
        r_wheel=0.005,      # 降低测量噪声
        r_imu=0.05
    ),
    fusion_config=FusionConfig(
        mode=FusionMode.BASIC,  # 简化测量模型
        enable_adaptive_weighting=False
    )
)
```

### 精确配置（低速停车）
```python
localization = LocalizationSystem(
    wheel_config=WheelConfig(low_speed_threshold=0.01),
    noise_params=NoiseParameters(
        q_x=0.001,          # 增加位置信度
        q_y=0.001,
        q_v=0.05,           # 降低速度变化率
        r_imu_steering=0.03  # 关注转向信息
    ),
    fusion_config=FusionConfig(
        mode=FusionMode.IMU_STEERING,  # 4D观测
        imu_steering_vel_threshold=0.5,
        enable_adaptive_weighting=True
    )
)
```

## 📊 融合模式切换

### BASIC 模式（3D观测）
适用于：直线行驶、高速、停泊

```python
localization.switch_fusion_mode(FusionMode.BASIC)
# 观测: [车轮速度, IMU加速度, 转向角]
# 优点: 计算快速，稳定性好
# 缺点: 低速下缺少转向信息
```

### IMU_STEERING 模式（4D观测）
适用于：低速转向、停车操纵、城市驾驶

```python
localization.switch_fusion_mode(FusionMode.IMU_STEERING)
# 观测: [车轮速度, IMU加速度, 转向角, IMU横摆速率]
# 优点: 低速下转向准确
# 缺点: 对IMU漂移敏感
```

## 🔄 校准参数设置

```python
# 创建校准参数
calib = CalibrationParams(
    wheel_calibration_factor=1.02,      # 轮速计校准系数
    steering_angle_offset=0.05,         # 转向角偏移
    steering_angle_gain=1.01,           # 转向角增益
    imu_accel_bias_x=0.01,              # IMU加速度偏置
    imu_accel_scale_x=1.0,              # IMU加速度缩放
    ackermann_factor=0.95                # Ackermann模型系数
)

# 设置校准参数（信心度必须 >= 0.5）
success = localization.set_calibration(calib, confidence=0.95)

if success:
    print("✅ 校准参数已应用")
else:
    print("❌ 信心度不足，校准失败")
```

## 🐛 诊断和调试

```python
# 获取完整诊断信息
diag = localization.get_diagnostics()

print(f"轮滑状态: {diag.is_wheel_slipping}")
print(f"静止状态: {diag.is_stationary}")
print(f"轮滑加速度: {diag.wheel_slip_accel:.2f} m/s²")
print(f"残差规范: {diag.residual_norm:.4f}")
print(f"更新计数: {diag.update_count}")

# 分量诊断
state = localization.get_state()
print(f"\n当前状态：")
print(f"  位置: ({state.x:.3f}, {state.y:.3f}) m")
print(f"  航向: {state.heading:.4f} rad ({np.degrees(state.heading):.1f}°)")
print(f"  速度: {state.v:.3f} m/s")
print(f"  转向: {state.delta:.4f} rad")
print(f"  档位: {state.gear}")
print(f"  方向: {state.direction}")
```

## 🎯 典型使用流程

### 1. 初始化
```python
# 系统启动
localization = LocalizationSystem()

# 可选：设置初始位置
initial_state = VehicleState(
    x=0.0, y=0.0, heading=0.0,
    v=0.0, delta=0.0
)
localization.reset(initial_state)
```

### 2. 数据处理（主循环）
```python
for measurement in sensor_stream:
    # 构造传感器数据
    sensor_data = SensorReadings(
        timestamp=measurement['time'],
        wheel_odometry=...,
        steering=...,
        imu=...,
        gear=...,
        ...
    )
    
    # 执行融合
    estimated_state = localization.process(sensor_data)
    
    # 使用估计状态进行控制或记录
    log_state(estimated_state)
    
    # 可选：监控诊断
    if localization.get_diagnostics().is_wheel_slipping:
        adjust_control_strategy()
```

### 3. 离线处理
```python
import pandas as pd

# 加载数据
df = pd.read_csv('sensor_data.csv')

# 初始化系统
localization = LocalizationSystem()

# 处理所有样本
results = []
for idx, row in df.iterrows():
    state = localization.process_sample(row.to_dict())
    results.append({
        'timestamp': state.timestamp,
        'x': state.x,
        'y': state.y,
        'heading': state.heading,
        'v': state.v,
        'direction': state.direction
    })

# 保存结果
output_df = pd.DataFrame(results)
output_df.to_csv('localization_output.csv', index=False)
```

## 🎪 方向推导演示

```python
from python.wheel_odometry import derive_vehicle_direction

# 场景1：前进
direction = derive_vehicle_direction(
    velocity=1.0,
    gear=VehicleGear.DRIVE,
    rear_left_state=WheelDirection.FORWARD,
    rear_right_state=WheelDirection.FORWARD,
    v_static_threshold=0.02
)
print(f"前进: {direction}")  # VehicleDirection.FORWARD

# 场景2：倒车
direction = derive_vehicle_direction(
    velocity=0.5,
    gear=VehicleGear.REVERSE,
    rear_left_state=WheelDirection.REVERSE,
    rear_right_state=WheelDirection.REVERSE,
    v_static_threshold=0.02
)
print(f"倒车: {direction}")  # VehicleDirection.REVERSE

# 场景3：停泊
direction = derive_vehicle_direction(
    velocity=0.0,
    gear=VehicleGear.PARK,
    rear_left_state=WheelDirection.STATIONARY,
    rear_right_state=WheelDirection.STATIONARY,
    v_static_threshold=0.02
)
print(f"停泊: {direction}")  # VehicleDirection.STATIONARY
```

## ⚡ 性能优化建议

### 1. 向量化计算
```python
# 批量处理（而不是逐帧处理）
import numpy as np

# 准备批量数据
states = []
measurements = []

for i in range(100):
    state = localization.process(sensor_data[i])
    states.append(state)

# 转换为数组便于进一步处理
positions = np.array([(s.x, s.y) for s in states])
velocities = np.array([s.v for s in states])
```

### 2. 降低更新频率
```python
# 如果不需要每帧都更新
frame_count = 0
UPDATE_INTERVAL = 2  # 每2帧更新一次

for measurement in sensor_stream:
    frame_count += 1
    
    if frame_count % UPDATE_INTERVAL == 0:
        state = localization.process(sensor_data)
    else:
        state = localization.get_state()  # 直接获取当前状态
```

### 3. 预分配内存
```python
# 离线处理时预分配
num_samples = len(data)
x_values = np.zeros(num_samples)
y_values = np.zeros(num_samples)
v_values = np.zeros(num_samples)

for i, sensor_data in enumerate(data):
    state = localization.process(sensor_data)
    x_values[i] = state.x
    y_values[i] = state.y
    v_values[i] = state.v
```

## 🔍 常见问题解决

### Q: 为什么位置跳跃？
**A:** 检查：
- 车轮脉冲是否有异常（溢出、CAN错误）
- IMU校准是否正确
- 融合模式是否匹配场景

### Q: 转向角不准确？
**A:** 调整：
- `SteeringConfig.steering_angle_offset` 和 `.steering_angle_gain`
- `SteeringModel.apply_rtk_calibration()`
- 转向映射表（如果需要）

### Q: 低速时漂移？
**A:** 尝试：
- 切换到 `FusionMode.IMU_STEERING`
- 降低 `NoiseParameters.q_x` 和 `.q_y`
- 启用 `enable_adaptive_weighting`

### Q: CPU使用率过高？
**A:** 考虑：
- 降低传感器采样率
- 使用 `BASIC` 模式代替 `IMU_STEERING`
- 增加 `UPDATE_INTERVAL`

---

## 📚 相关文档
- [架构文档](ARCHITECTURE.md) - 系统设计和数据流
- [同步报告](SYNC_REPORT.md) - C-Python代码对齐详情
- [集成测试](test_integration.py) - 运行测试验证功能

## 🎓 进阶主题

### 自定义映射表
```python
# 创建自定义转向映射表
from python.types import SteeringMappingTable
import numpy as np

steering_wheel_angles = np.linspace(-8.79645943, 8.79645943, 155)
left_front_angles = ...  # 从标定数据
right_front_angles = ...

table = SteeringMappingTable(
    steering_wheel=steering_wheel_angles,
    left_wheel=left_front_angles,
    right_wheel=right_front_angles,
    num_points=155,
    valid=True
)

# 在转向模型中应用
localization.steering.mapping_table = table
```

### 自定义过程噪声
```python
# 基于车辆特性调整噪声
localization.kf.noise = NoiseParameters(
    q_x=0.005,      # 高精度定位
    q_y=0.005,
    q_theta=0.001,
    q_v=0.02,       # 低速度变化
    q_delta=0.005
)
```

---

**最后更新**: 2024年
**版本**: 1.0
**状态**: 生产就绪 ✅
