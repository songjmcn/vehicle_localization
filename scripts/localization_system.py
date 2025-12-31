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
    x: float = 0.0      # x位置 (m)
    y: float = 0.0      # y位置 (m)
    theta: float = 0.0  # 航向角 (rad)
    v: float = 0.0      # 速度 (m/s)
    delta: float = 0.0  # 转向角 (rad)


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
        
        # ✅ 修复（2025年12月）：使用有符号速度直接计算
        # 关键原理：velocity本身是有符号的（v>0前进，v<0倒车）
        # 不应该在运动模型中再次反向处理转向角
        # 位置更新：v的负号自动处理倒车方向
        
        cos_theta = np.cos(state.theta)
        sin_theta = np.sin(state.theta)
        
        # 位置更新：使用初始heading（数值稳定性）
        # x_new = x + v·cos(θ)·Δt
        # y_new = y + v·sin(θ)·Δt
        # 当v<0时（倒车），位置自动减少 ✓
        new_x = state.x + v * cos_theta * dt
        new_y = state.y + v * sin_theta * dt
        
        # 航向角更新：yaw_rate = (v/L)·tan(δ)
        # 注意：v的符号自动处理了倒车时的方向
        try:
            tan_delta = np.tan(delta)
            yaw_rate = (v / self.wheelbase) * tan_delta
            dtheta = yaw_rate * dt
        except:
            # tan(δ)溢出时保持heading不变
            dtheta = 0.0
        
        new_theta = state.theta + dtheta
        
        # 规范化heading角到[-π, π]
        while new_theta > np.pi:
            new_theta -= 2 * np.pi
        while new_theta < -np.pi:
            new_theta += 2 * np.pi
        
        new_state = VehicleState(
            x=new_x,
            y=new_y,
            theta=new_theta,
            v=v,
            delta=delta
        )
        
        return new_state
    
    def jacobian(self, state: VehicleState, velocity: float, steering_angle: float, dt: float) -> np.ndarray:
        """
        ✅ 新增（2025年12月）：计算雅可比矩阵
        
        用于扩展卡尔曼滤波的协方差传播
        F[i][j] = ∂x_i_new / ∂x_j_old
        """
        F = np.eye(5)
        
        cos_theta = np.cos(state.theta)
        sin_theta = np.sin(state.theta)
        
        # 位置对heading的偏导
        F[0][2] = -velocity * sin_theta * dt  # ∂x/∂θ
        F[1][2] = velocity * cos_theta * dt   # ∂y/∂θ
        
        # ✅ 关键修复：位置对速度的偏导（不含sign_v）
        F[0][3] = cos_theta * dt              # ∂x/∂v
        F[1][3] = sin_theta * dt              # ∂y/∂v
        
        # 航向角对控制量的偏导
        try:
            tan_delta = np.tan(steering_angle)
            cos_delta = np.cos(steering_angle)
            
            F[2][3] = tan_delta / self.wheelbase * dt  # ∂θ/∂v
            
            if abs(cos_delta) > 1e-6:
                F[2][4] = velocity / (self.wheelbase * cos_delta * cos_delta) * dt  # ∂θ/∂δ
        except:
            pass
        
        return F


class ExtendedKalmanFilter:
    """
    ✅ 新增（2025年12月）：扩展卡尔曼滤波器
    
    与C代码同步的EKF实现
    """
    
    def __init__(self, initial_state: VehicleState, motion_model: MotionModel):
        """初始化卡尔曼滤波器"""
        self.motion_model = motion_model
        
        # 状态估计
        self.x_hat = np.array([
            initial_state.x,
            initial_state.y,
            initial_state.theta,
            initial_state.v,
            initial_state.delta
        ]).reshape(5, 1)
        
        # ✅ 修复：合理的初始协方差矩阵
        self.P = np.zeros((5, 5))
        self.P[0, 0] = 10.0   # x位置：10m不确定性
        self.P[1, 1] = 10.0   # y位置：10m不确定性
        self.P[2, 2] = 1.0    # 航向：1rad
        self.P[3, 3] = 0.5    # 速度：0.5 m/s
        self.P[4, 4] = 0.2    # 转向：0.2rad
        
        # 过程噪声
        self.Q = np.diag([0.001, 0.001, 0.0001, 0.01, 0.001])
        
        # 测量噪声
        self.R = np.diag([0.1, 0.5, 0.5, 0.05, 0.05])
        
        # 测量矩阵
        self.H = np.eye(5)
    
    def predict(self, velocity: float, steering_angle: float, dt: float) -> np.ndarray:
        """预测步骤"""
        state = VehicleState(
            x=self.x_hat[0, 0],
            y=self.x_hat[1, 0],
            theta=self.x_hat[2, 0],
            v=self.x_hat[3, 0],
            delta=self.x_hat[4, 0]
        )
        
        # 状态转移
        control = {'velocity': velocity, 'steering_angle': steering_angle}
        new_state = self.motion_model.predict(state, control, dt)
        self.x_hat = np.array([new_state.x, new_state.y, new_state.theta, 
                              new_state.v, new_state.delta]).reshape(5, 1)
        
        # 协方差传播
        F = self.motion_model.jacobian(state, velocity, steering_angle, dt)
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x_hat
    
    def update(self, measurement: np.ndarray) -> np.ndarray:
        """更新步骤"""
        y = measurement - self.H @ self.x_hat
        S = self.H @ self.P @ self.H.T + self.R
        
        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x_hat = self.x_hat + K @ y
            I = np.eye(5)
            self.P = (I - K @ self.H) @ self.P
        except:
            pass
        
        return self.x_hat


class LowSpeedLocalizationSystem:
    """低速车辆定位融合系统"""
    
    def __init__(self, wheelbase: float = 2.7, use_kalman: bool = False):
        """初始化系统"""
        self.wheelbase = wheelbase
        self.use_kalman = use_kalman
        self.motion_model = BicycleModelLowSpeed(wheelbase)
        self.state = VehicleState()
        
        if use_kalman:
            self.kalman = ExtendedKalmanFilter(self.state, self.motion_model)
        else:
            self.kalman = None
    
    def run_step(self, reading: SensorReading, dt: float = 0.01) -> VehicleState:
        """执行一步定位估计"""
        control = {
            'velocity': reading.wheel_velocity,
            'steering_angle': reading.steering_angle
        }
        
        if self.use_kalman:
            self.kalman.predict(reading.wheel_velocity, reading.steering_angle, dt)
            # 可选：添加测量更新
            measurement = np.array([
                reading.wheel_velocity,
                reading.imu_accel_x,
                reading.imu_accel_y,
                reading.imu_gyro_z,
                reading.steering_angle
            ]).reshape(5, 1)
            self.kalman.update(measurement)
            
            self.state = VehicleState(
                x=self.kalman.x_hat[0, 0],
                y=self.kalman.x_hat[1, 0],
                theta=self.kalman.x_hat[2, 0],
                v=self.kalman.x_hat[3, 0],
                delta=self.kalman.x_hat[4, 0]
            )
        else:
            self.state = self.motion_model.predict(self.state, control, dt)
        
        return self.state

