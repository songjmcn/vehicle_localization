% MATLAB 车辆定位系统 - 快速开始指南

本文件夹包含完整的MATLAB实现，对应C版本的所有功能。

## 📁 文件夹结构

```
matlab/
├── data_structures/     # 数据结构定义
│   ├── VehicleState.m   # 车辆状态类
│   ├── SensorData.m     # 传感器数据类
│   ├── NoiseParams.m    # 噪声参数类
│   ├── CalibrationParams.m # 校准参数类
│   └── FusionConfig.m   # 融合配置类
│
├── utils/               # 工具函数
│   └── angle_utils.m    # 角度处理工具
│
├── core/                # 核心算法
│   ├── MotionModel.m    # 运动学模型
│   ├── SteeringModel.m  # 转向模型
│   ├── KalmanFilter.m   # EKF滤波器
│   └── SteeringJitterFilter.m # 转向抖动过滤器
│
├── examples/            # 示例代码
│   └── basic_example.m  # 基础使用示例
│
├── tests/               # 单元测试
├── visualization/       # 可视化工具
│
└── constants.m          # 系统常数定义
```

## 🚀 快速开始

### 基础使用

```matlab
% 1. 添加路径
addpath('/Users/alex/workspace/code_review/vehicle_localization/matlab');
addpath('/Users/alex/workspace/code_review/vehicle_localization/matlab/data_structures');
addpath('/Users/alex/workspace/code_review/vehicle_localization/matlab/core');
addpath('/Users/alex/workspace/code_review/vehicle_localization/matlab/utils');

% 2. 初始化滤波器
kf = KalmanFilter();

% 3. 创建初始状态
initial_state = VehicleState(0, 0, 0, 0, 0, 0, constants.GEAR_NEUTRAL, constants.DIRECTION_STATIONARY);

% 4. 创建噪声和校准参数
noise = NoiseParams();
calibration = CalibrationParams();
fusion_config = FusionConfig();

% 5. 初始化滤波器
kf.init(initial_state, noise, calibration, fusion_config);

% 6. 主循环：预测和更新
for i = 1:100
    % 创建传感器数据
    sensor_data = SensorData(1.0, 0.01, 0.02, 0, 0.001, 0.1, i*0.05);
    
    % 预测步骤
    kf.predict(1.0, 0.1, i*0.05);
    
    % 更新步骤
    kf.update(sensor_data);
    
    % 获取估计状态
    state = kf.get_state();
    
    if mod(i, 20) == 0
        fprintf('第 %d 步: 位置 (%.3f, %.3f) m, 航向 %.2f deg\n', ...
                i, state.x, state.y, state.theta * constants.RAD_TO_DEG);
    end
end
```

## 📚 主要类与函数

### 数据结构
- **VehicleState** - 5维车辆状态 [x, y, θ, v, δ]
- **SensorData** - 多传感器融合数据
- **NoiseParams** - 卡尔曼滤波噪声参数
- **CalibrationParams** - 传感器校准参数
- **FusionConfig** - 融合算法配置

### 核心算法
- **MotionModel.predict()** - 状态预测
- **MotionModel.jacobian()** - 雅可比矩阵计算
- **KalmanFilter.predict()** - EKF预测步骤
- **KalmanFilter.update()** - EKF更新步骤 (自动选择融合模式)
- **SteeringModel** - 转向角处理与低通滤波

### 工具函数
- **angle_utils.normalize_angle()** - 角度规范化
- **angle_utils.angle_difference()** - 计算最小角度差
- **angle_utils.deg_to_rad()** / **rad_to_deg()** - 角度转换

## 🔧 配置选项

### 融合算法模式
```matlab
% 基础融合 (轮速 + 转向角)
fusion_config = FusionConfig('mode', constants.FUSION_MODE_BASIC);

% IMU+转向融合 (轮速 + IMU加速度 + 转向角)
fusion_config = FusionConfig('mode', constants.FUSION_MODE_IMU_STEERING);
```

### 噪声参数调整
```matlab
% 创建自定义噪声参数
noise = NoiseParams(...
    'q_x', 0.01, ...
    'q_y', 0.01, ...
    'q_theta', 0.001, ...
    'r_wheel', 0.1, ...
    'r_steering', 0.01 ...
);
```

## 📊 与C版本的对应关系

| C版本 | MATLAB版本 |
|------|----------|
| `motion_model_predict()` | `MotionModel.predict()` |
| `motion_model_jacobian()` | `MotionModel.jacobian()` |
| `kalman_filter_predict()` | `KalmanFilter.predict()` |
| `kalman_filter_update()` | `KalmanFilter.update()` |
| `kalman_filter_update_basic()` | `KalmanFilter.update_basic()` |
| `kalman_filter_update_imu_steering()` | `KalmanFilter.update_imu_steering()` |
| `steering_model_*()` | `SteeringModel.*()` |

## 💡 常见问题

### Q: 如何切换融合算法？
A: 通过 `fusion_config` 的 `setMode()` 方法：
```matlab
fusion_config.setMode(constants.FUSION_MODE_IMU_STEERING);
kf.fusion_config = fusion_config;
```

### Q: 如何重置滤波器？
A: 创建新的 KalmanFilter 对象或重新调用 init()

### Q: 精度不够怎么办？
A: 调整 `NoiseParams` 中的 Q 和 R 矩阵参数

## 📖 详细文档

- 参见 C 版本中的 `/docs` 文件夹获取详细的算法说明
- 具体参数标定指南见 `/docs/PARAMETER_CALIBRATION_GUID.MD`

## ✅ 验证

运行 `examples/basic_example.m` 验证安装：
```matlab
run examples/basic_example.m
```
