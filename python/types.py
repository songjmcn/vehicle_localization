from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Optional, List
import numpy as np

# ========== Enums ==========

class VehicleGear(IntEnum):
    """车辆档位"""
    PARK = 0
    REVERSE = 1
    NEUTRAL = 2
    DRIVE = 3
    LOW = 4
    INVALID = -1

class WheelDirection(IntEnum):
    """轮向状态"""
    INVALID = -1
    STATIONARY = 0
    FORWARD = 1
    REVERSE = 2
    STOP = 0  # alias for STATIONARY

class VehicleDirection(IntEnum):
    """车辆运动方向"""
    INVALID = -1
    STATIONARY = 0
    FORWARD = 1
    REVERSE = 2

class FusionMode(IntEnum):
    """融合模式枚举"""
    BASIC = 0
    IMU_STEERING = 1

# ========== Configuration Classes ==========

@dataclass
class WheelConfig:
    """轮速处理器配置"""
    pulses_per_wheel_rev: int = 96      # 编码器每圈脉冲数
    wheel_radius: float = 0.35          # 轮半径 (m)
    low_speed_threshold: float = 0.02   # 低速阈值 (m/s)
    pulse_timeout_ms: int = 500         # 脉冲超时 (ms)
    max_count: int = 4096               # 脉冲计数器最大值

@dataclass
class SteeringConfig:
    """转向配置"""
    steering_ratio: float = 15.3
    wheelbase: float = 2.76
    max_steering_wheel_angle: float = 8.79645943
    max_front_wheel_angle: float = 0.676454711
    steering_angle_offset: float = 0.0
    steering_angle_gain: float = 1.0
    ackermann_factor: float = 1.0
    enable_steering_filter: bool = True
    steering_filter_alpha: float = 0.1
    steering_deadzone: float = 0.02

@dataclass
class NoiseParameters:
    """Kalman滤波器噪声参数"""
    q_x: float = 0.02           # 位置x噪声
    q_y: float = 0.02           # 位置y噪声
    q_theta: float = 0.002      # 角度噪声
    q_v: float = 0.1            # 速度噪声
    q_delta: float = 0.01       # 转向角噪声
    r_wheel: float = 0.01       # 轮速计测量噪声
    r_imu: float = 0.1          # IMU加速度测量噪声
    r_steering: float = 0.05    # 转向角测量噪声
    r_imu_steering: float = 0.08  # IMU推导转向角噪声

@dataclass
class CalibrationParams:
    """校准参数"""
    wheel_calibration_factor: float = 1.0
    steering_angle_offset: float = 0.0
    steering_angle_gain: float = 1.0
    imu_accel_bias_x: float = 0.0
    imu_accel_bias_y: float = 0.0
    imu_gyro_bias_z: float = 0.0
    imu_accel_scale_x: float = 1.0
    imu_accel_scale_y: float = 1.0
    imu_lateral_accel_bias: float = 0.0
    imu_lateral_accel_scale: float = 1.0
    ackermann_factor: float = 1.0

@dataclass
class FusionConfig:
    """融合模式配置"""
    mode: FusionMode = FusionMode.BASIC
    imu_steering_vel_threshold: float = 0.5
    imu_steering_weight: float = 1.0
    enable_adaptive_weighting: bool = False

@dataclass
class DiagnosticsInfo:
    """诊断信息"""
    is_wheel_slipping: bool = False
    is_stationary: bool = False
    wheel_slip_accel: float = 0.0
    residual_norm: float = 0.0
    update_count: int = 0

# ========== Sensor & Odometry Data ==========

@dataclass
class IMUData:
    """IMU数据"""
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_z: float = 0.0

@dataclass
class SteeringData:
    """转向数据"""
    steering_angle: float = 0.0  # 前轮转向角 (rad)

@dataclass
class WheelOdometryData:
    """轮速计数据"""
    velocity: float = 0.0
    distance: float = 0.0
    timestamp: int = 0
    gear: VehicleGear = VehicleGear.NEUTRAL
    rear_left_direction: WheelDirection = WheelDirection.INVALID
    rear_right_direction: WheelDirection = WheelDirection.INVALID
    vehicle_direction: VehicleDirection = VehicleDirection.INVALID

@dataclass
class SensorReadings:
    """传感器数据容器（对应C端SensorReadings_t）"""
    timestamp: int = 0
    wheel_odometry: WheelOdometryData = field(default_factory=WheelOdometryData)
    steering: SteeringData = field(default_factory=SteeringData)
    imu: IMUData = field(default_factory=IMUData)
    gear: VehicleGear = VehicleGear.NEUTRAL
    rear_left_wheel_count: int = 0
    rear_right_wheel_count: int = 0
    rear_left_wheel_direction: WheelDirection = WheelDirection.INVALID
    rear_right_wheel_direction: WheelDirection = WheelDirection.INVALID
    has_wheel_pulse: bool = False

# ========== Vehicle State ==========

@dataclass
class VehicleState:
    """车辆状态（EKF状态向量）
    
    Units:
        - x, y: meters
        - heading: radians
        - v: m/s
        - delta: front wheel steering angle (radians)
    """
    x: float = 0.0          # 纵向位置 (m)
    y: float = 0.0          # 横向位置 (m)
    heading: float = 0.0    # 航向角 (rad)
    v: float = 0.0          # 纵向速度 (m/s)
    delta: float = 0.0      # 前轮转向角 (rad)
    timestamp: int = 0
    gear: VehicleGear = VehicleGear.NEUTRAL
    direction: VehicleDirection = VehicleDirection.INVALID
    steering_angle: float = 0.0

# ========== Steering Mapping Table ==========

@dataclass
class SteeringMappingTable:
    """转向角映射表（方向盘角 -> 前轮角）"""
    steering_wheel: Optional[np.ndarray] = None  # 方向盘转角数组
    left_wheel: Optional[np.ndarray] = None      # 左前轮转角数组
    right_wheel: Optional[np.ndarray] = None     # 右前轮转角数组
    num_points: int = 0
    valid: bool = False
