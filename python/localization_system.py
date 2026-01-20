"""定位系统集成模块"""
from typing import Optional
import numpy as np

from .wheel_odometry import WheelOdometryProcessor, derive_vehicle_direction
from .steering_model import SteeringModel, SteeringConfig
from .kalman_filter import KalmanFilter
from .types import (
    WheelConfig, SteeringConfig, VehicleState, VehicleGear, WheelDirection, 
    VehicleDirection, SensorReadings, FusionConfig, FusionMode, 
    NoiseParameters, CalibrationParams, DiagnosticsInfo
)

class LocalizationSystem:
    """车辆定位系统集成器
    
    集成轮速处理、转向模型和扩展卡尔曼滤波器的完整定位系统。
    """

    def __init__(self,
                 wheel_config: Optional[WheelConfig] = None,
                 steering_config: Optional[SteeringConfig] = None,
                 initial_state: Optional[VehicleState] = None,
                 noise_params: Optional[NoiseParameters] = None,
                 calibration_params: Optional[CalibrationParams] = None,
                 fusion_config: Optional[FusionConfig] = None):
        """初始化定位系统"""
        
        self.wheel_cfg = wheel_config if wheel_config is not None else WheelConfig()
        self.steering_cfg = steering_config if steering_config is not None else SteeringConfig()

        # 初始化各子模块
        self.wheel = WheelOdometryProcessor(self.wheel_cfg)
        self.steering = SteeringModel(self.steering_cfg)
        
        # Kalman滤波器
        if noise_params is None:
            noise_params = NoiseParameters()
        if calibration_params is None:
            calibration_params = CalibrationParams()
        if fusion_config is None:
            fusion_config = FusionConfig(mode=FusionMode.BASIC)
            
        self.kf = KalmanFilter(initial_state=initial_state, 
                              noise=noise_params,
                              calibration=calibration_params,
                              fusion_config=fusion_config)
        
        # 将转向模型设置到卡尔曼滤波器
        self.kf.set_steering_config(self.steering)

    def process(self, sensor_data: SensorReadings) -> VehicleState:
        """处理一个传感器样本并返回估计的车辆状态
        
        执行完整的融合流程：轮速处理 → 转向处理 → EKF预测 → EKF更新
        """
        timestamp_ms = sensor_data.timestamp

        # 步骤0：处理轮速数据
        if sensor_data.has_wheel_pulse:
            wheel_out = self.wheel.update(
                sensor_data.rear_left_wheel_count,
                sensor_data.rear_right_wheel_count,
                timestamp_ms,
                sensor_data.gear,
                sensor_data.rear_left_wheel_direction,
                sensor_data.rear_right_wheel_direction
            )
            velocity = wheel_out.velocity
            odometry_distance = wheel_out.distance
        else:
            velocity = sensor_data.wheel_odometry.velocity
            odometry_distance = sensor_data.wheel_odometry.distance

        # 步骤1：处理转向数据
        steering_wheel_angle = sensor_data.steering.steering_angle
        front_wheel_angle = self.steering.update(steering_wheel_angle, timestamp_ms)

        # 确保不超过前轮角的物理极限
        max_front = self.steering_cfg.max_front_wheel_angle
        if abs(front_wheel_angle) > max_front:
            front_wheel_angle = np.clip(front_wheel_angle, -max_front, max_front)

        # 步骤2：统一计算车辆运动方向（仅此一处）
        vehicle_direction = derive_vehicle_direction(
            velocity,
            sensor_data.gear,
            sensor_data.rear_left_wheel_direction,
            sensor_data.rear_right_wheel_direction,
            self.wheel_cfg.low_speed_threshold
        )

        # 步骤3：创建带方向信息的状态用于EKF
        current_state_with_direction = self.kf.get_state()
        current_state_with_direction.direction = vehicle_direction

        # 步骤4：EKF预测
        self.kf.predict_with_state(current_state_with_direction, timestamp_ms, self.steering)

        # 步骤5：准备传感器数据用于EKF更新
        sensor_data_for_update = SensorReadings(
            timestamp=timestamp_ms,
            wheel_odometry=sensor_data.wheel_odometry,
            steering=sensor_data.steering,
            imu=sensor_data.imu,
            gear=sensor_data.gear,
            rear_left_wheel_count=sensor_data.rear_left_wheel_count,
            rear_right_wheel_count=sensor_data.rear_right_wheel_count,
            rear_left_wheel_direction=sensor_data.rear_left_wheel_direction,
            rear_right_wheel_direction=sensor_data.rear_right_wheel_direction,
            has_wheel_pulse=sensor_data.has_wheel_pulse
        )
        
        # 使用处理后的轮速和前轮角覆盖原始值
        sensor_data_for_update.wheel_odometry.velocity = velocity
        sensor_data_for_update.steering.steering_angle = front_wheel_angle

        # 步骤6：EKF更新
        self.kf.update(sensor_data_for_update)

        # 步骤7：获取估计状态
        estimated_state = self.kf.get_state()
        estimated_state.gear = sensor_data.gear
        estimated_state.direction = vehicle_direction

        return estimated_state

    def process_sample(self, sample: dict) -> VehicleState:
        """处理字典样本（向后兼容）"""
        from .types import IMUData, SteeringData, WheelOdometryData
        
        # 构建传感器数据
        sensor_data = SensorReadings(
            timestamp=int(sample.get('timestamp', 0)),
            wheel_odometry=WheelOdometryData(
                velocity=float(sample.get('wheel_velocity', 0.0)),
                distance=float(sample.get('wheel_distance', 0.0))
            ),
            steering=SteeringData(
                steering_angle=float(sample.get('steering_wheel_angle', 0.0))
            ),
            imu=IMUData(
                accel_x=float(sample.get('imu_ax', 0.0)),
                accel_y=float(sample.get('imu_ay', 0.0)),
                accel_z=float(sample.get('imu_az', 9.8))
            ),
            gear=VehicleGear(int(sample.get('gear', VehicleGear.NEUTRAL))),
            rear_left_wheel_count=int(sample.get('left_pulse', 0)),
            rear_right_wheel_count=int(sample.get('right_pulse', 0)),
            rear_left_wheel_direction=WheelDirection(int(sample.get('rear_left_direction', 0))),
            rear_right_wheel_direction=WheelDirection(int(sample.get('rear_right_direction', 0))),
            has_wheel_pulse=bool(sample.get('has_wheel_pulse', False))
        )
        
        return self.process(sensor_data)

    def get_state(self) -> VehicleState:
        """获取当前状态估计"""
        return self.kf.get_state()

    def get_diagnostics(self) -> DiagnosticsInfo:
        """获取诊断信息"""
        return self.kf.get_diagnostics()

    def set_calibration(self, calibration: CalibrationParams, confidence: float = 1.0) -> bool:
        """手动设置校准参数"""
        if confidence < 0.5:
            return False
        
        # 更新卡尔曼滤波器的校准参数
        self.kf.calibration = calibration
        
        # 应用转向模型校准
        self.steering.apply_rtk_calibration(calibration)
        
        return True

    def reset(self, initial_state: Optional[VehicleState] = None):
        """重置系统"""
        self.kf.reset(initial_state)
        self.wheel.reset()
        self.steering.reset()

    def switch_fusion_mode(self, mode: FusionMode):
        """切换融合模式"""
        self.kf.switch_fusion_mode(mode)
