"""
低速车辆定位融合系统
支持轮速计、IMU和转向角的多传感器融合
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
from abc import ABC, abstractmethod


@dataclass
class VehicleState:
    """车辆状态"""
    x: float  # x位置 (m)
    y: float  # y位置 (m)
    theta: float  # 航向角 (rad)
    v: float  # 速度 (m/s)
    delta: float  # 转向角 (rad)


@dataclass
class SensorReading:
    """传感器读数"""
    wheel_velocity: float  # 轮速计速度 (m/s)
    imu_accel_x: float  # IMU x轴加速度 (m/s²)
    imu_accel_y: float  # IMU y轴加速度 (m/s²)
    imu_gyro_z: float  # IMU z轴角速度 (rad/s)
    steering_angle: float  # 转向角 (rad)
    timestamp: float  # 时间戳 (s)


class MotionModel(ABC):
    """运动模型基类"""
    
    @abstractmethod
    def predict(self, state: VehicleState, control: dict, dt: float) -> VehicleState:
        """预测下一状态"""
        pass


class BicycleModelLowSpeed(MotionModel):
    """
    低速改进的单轨模型
    - 支持前进和后退
    - 优化低速精度
    """
    
    def __init__(self, wheelbase: float = 2.7):
        """
        初始化
        
        Args:
            wheelbase: 轴距 (m)
        """
        self.wheelbase = wheelbase
        self.v_threshold = 0.05  # 静止速度阈值 (m/s)
        
    def predict(self, state: VehicleState, control: dict, dt: float) -> VehicleState:
        """
        预测下一时刻状态
        
        Args:
            state: 当前状态
            control: 控制输入 {'velocity': float, 'steering_angle': float}
            dt: 时间间隔 (s)
            
        Returns:
            下一时刻状态
        """
        v = control['velocity']
        delta = control['steering_angle']
        
        # 检测是否静止
        if abs(v) < self.v_threshold:
            # 完全静止，只更新角度（基于陀螺仪）
            new_state = VehicleState(
                x=state.x,
                y=state.y,
                theta=state.theta,
                v=0.0,
                delta=delta
            )
            return new_state
        
        # 运动学模型：考虑前进和后退
        sign_v = np.sign(v)
        abs_v = abs(v)
        
        # 转向角在后退时反向
        effective_delta = sign_v * delta
        
        # 位置更新
        dx = abs_v * np.cos(state.theta + effective_delta) * dt
        dy = abs_v * np.sin(state.theta + effective_delta) * dt
        
        # 航向角更新
        dtheta = (sign_v * abs_v * np.tan(effective_delta) / self.wheelbase) * dt
        
        new_state = VehicleState(
