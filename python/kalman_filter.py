import numpy as np
from .types import (VehicleState, FusionMode, FusionConfig, NoiseParameters, 
                    CalibrationParams, DiagnosticsInfo, SensorReadings)
from .motion_model import predict as motion_predict, jacobian as motion_jacobian
from .utils import normalize_angle

# 常数定义
WHEEL_SLIP_ACCEL_THRESHOLD = 5.0
V_STATIC_THRESHOLD = 0.02
WHEELBASE = 2.76

class KalmanFilter:
    def __init__(self, initial_state: VehicleState = None, 
                 noise: NoiseParameters = None,
                 calibration: CalibrationParams = None,
                 fusion_config: FusionConfig = None):
        """初始化卡尔曼滤波器"""
        # state vector: [x, y, heading, v, delta]
        if initial_state is None:
            initial_state = VehicleState()
        self.x = np.array([initial_state.x,
                           initial_state.y,
                           initial_state.heading,
                           initial_state.v,
                           initial_state.delta], dtype=float)
        self.P = np.eye(5, dtype=float) * 0.1

        # 噪声参数
        if noise is None:
            noise = NoiseParameters()
        self.noise = noise
        
        # 校准参数
        if calibration is None:
            calibration = CalibrationParams()
        self.calibration = calibration
        
        # 融合配置
        if fusion_config is None:
            fusion_config = FusionConfig(mode=FusionMode.BASIC)
        self.fusion_config = fusion_config

        # 过程噪声Q (diag)
        self.Q = np.diag([noise.q_x, noise.q_y, noise.q_theta, 
                          noise.q_v, noise.q_delta]).astype(float)

        # 测量噪声（会在update时动态调整）
        self.r_wheel = noise.r_wheel
        self.r_imu = noise.r_imu
        self.r_steering = noise.r_steering
        self.r_imu_steering = noise.r_imu_steering

        self.last_timestamp = None
        self.steering_config = None  # 转向配置（由系统设置）
        
        # 诊断信息
        self.diagnostics = DiagnosticsInfo()

    def get_state(self) -> VehicleState:
        """获取当前状态估计"""
        return VehicleState(x=float(self.x[0]), y=float(self.x[1]),
                            heading=float(self.x[2]), v=float(self.x[3]), 
                            delta=float(self.x[4]))

    def get_diagnostics(self) -> DiagnosticsInfo:
        """获取诊断信息"""
        return self.diagnostics

    def reset(self, initial_state: VehicleState = None):
        """重置滤波器"""
        if initial_state is None:
            initial_state = VehicleState()
        self.x[0] = initial_state.x
        self.x[1] = initial_state.y
        self.x[2] = initial_state.heading
        self.x[3] = initial_state.v
        self.x[4] = initial_state.delta
        self.P = np.eye(5, dtype=float) * 0.1
        self.last_timestamp = None
        self.diagnostics = DiagnosticsInfo()

    def set_steering_config(self, steering_model):
        """设置转向配置（由系统调用）"""
        self.steering_config = steering_model

    def switch_fusion_mode(self, mode: FusionMode):
        """切换融合模式"""
        self.fusion_config.mode = mode

    def predict(self, velocity: float, steering_angle: float, timestamp_ms: int, steering_model=None):
        """EKF预测步骤"""
        if self.last_timestamp is None:
            self.last_timestamp = timestamp_ms
            self.x[3] = velocity
            self.x[4] = steering_angle
            return

        dt = (timestamp_ms - self.last_timestamp) / 1000.0
        if dt <= 0:
            return

        # 应用校准参数（与C代码同步）
        cal_velocity = velocity * self.calibration.wheel_calibration_factor
        cal_steering = steering_angle  # 已为前轮转向角
        
        state = VehicleState(x=self.x[0], y=self.x[1], heading=self.x[2], v=self.x[3], delta=self.x[4])
        new_state = motion_predict(state, cal_velocity, cal_steering, dt, steering_model)

        # 更新状态向量
        self.x[0] = new_state.x
        self.x[1] = new_state.y
        self.x[2] = new_state.heading
        self.x[3] = new_state.v
        self.x[4] = new_state.delta

        # 计算雅可比矩阵F
        F = motion_jacobian(state, cal_velocity, cal_steering, dt)

        # 协方差预测
        self.P = F.dot(self.P).dot(F.T) + self.Q

        self.last_timestamp = timestamp_ms

    def predict_with_state(self, state: VehicleState, timestamp_ms: int, steering_model=None):
        """使用外部提供的状态作为运动输入的预测步骤"""
        if self.last_timestamp is None:
            self.last_timestamp = timestamp_ms
            self.x[0] = state.x
            self.x[1] = state.y
            self.x[2] = state.heading
            self.x[3] = state.v
            self.x[4] = state.delta
            return

        dt = (timestamp_ms - self.last_timestamp) / 1000.0
        if dt <= 0:
            return

        cur_state = VehicleState(x=self.x[0], y=self.x[1], heading=self.x[2], v=self.x[3], delta=self.x[4])

        # 应用校准参数
        cal_velocity = state.v * self.calibration.wheel_calibration_factor
        cal_steering = state.delta

        # 使用运动模型传播
        new_state = motion_predict(cur_state, cal_velocity, cal_steering, dt, steering_model)

        # 更新状态
        self.x[0] = new_state.x
        self.x[1] = new_state.y
        self.x[2] = new_state.heading
        self.x[3] = new_state.v
        self.x[4] = new_state.delta

        # 计算雅可比矩阵
        F = motion_jacobian(cur_state, cal_velocity, cal_steering, dt)

        # 协方差预测
        self.P = F.dot(self.P).dot(F.T) + self.Q

        self.last_timestamp = timestamp_ms

    def update(self, sensor_data: SensorReadings):
        """根据融合模式路由到不同的update算法"""
        if self.fusion_config.mode == FusionMode.IMU_STEERING:
            self.update_imu_steering(sensor_data)
        else:
            self.update_basic(sensor_data)

    def update_basic(self, sensor_data: SensorReadings):
        """基础融合：3维观测(轮速+IMU加速度+转向角)"""
        # 检测轮滑动
        accel_mag = np.hypot(sensor_data.imu.accel_x, sensor_data.imu.accel_y)
        self.diagnostics.is_wheel_slipping = (accel_mag > WHEEL_SLIP_ACCEL_THRESHOLD)
        self.diagnostics.wheel_slip_accel = accel_mag
        self.diagnostics.is_stationary = (abs(sensor_data.wheel_odometry.velocity) < V_STATIC_THRESHOLD)

        # 测量向量 [wheel_velocity, imu_accel_magnitude, front_wheel_angle]
        z = np.array([
            sensor_data.wheel_odometry.velocity * self.calibration.wheel_calibration_factor,
            accel_mag,
            sensor_data.steering.steering_angle
        ], dtype=float)

        # 测量模型的雅可比矩阵 H (3x5)
        H = np.zeros((3, 5), dtype=float)
        H[0, 3] = 1.0  # 轮速计测量速度
        H[1, 3] = 1.0  # IMU加速度与速度相关
        H[2, 4] = 1.0  # 前轮转向角直接测量

        # 测量噪声（自适应）
        r_w = float(self.r_wheel)
        r_imu = float(self.r_imu)
        r_steer = float(self.r_steering)

        # 轮滑动时增加轮速计噪声权重
        if self.diagnostics.is_wheel_slipping:
            r_w *= 4.0

        # 低速时轮速计观测噪声增加
        if abs(sensor_data.wheel_odometry.velocity) < 0.1:
            r_w *= 10.0

        R = np.diag([r_w * r_w, r_imu * r_imu, r_steer * r_steer]).astype(float)

        # 预测测量值
        z_hat = np.array([self.x[3], 0.0, self.x[4]], dtype=float)

        # 残差
        y = z - z_hat
        y[2] = normalize_angle(y[2])

        self.diagnostics.residual_norm = np.linalg.norm(y)

        # 卡尔曼增益计算：K = P * H^T * (H * P * H^T + R)^-1
        S = H.dot(self.P).dot(H.T) + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # 矩阵奇异，跳过更新

        K = self.P.dot(H.T).dot(S_inv)

        # 状态更新
        self.x = self.x + K.dot(y)

        # 正规化角度
        self.x[2] = normalize_angle(self.x[2])
        self.x[4] = normalize_angle(self.x[4])

        # 协方差更新：P = (I - K * H) * P
        I = np.eye(5)
        self.P = (I - K.dot(H)).dot(self.P)

        self.diagnostics.update_count += 1

    def update_imu_steering(self, sensor_data: SensorReadings):
        """IMU转向融合算法：4维观测(轮速+IMU加速度+转向角+IMU推导转向角)"""
        v = sensor_data.wheel_odometry.velocity
        accel_y = sensor_data.imu.accel_y

        # 检测轮滑动
        accel_mag = np.hypot(sensor_data.imu.accel_x, sensor_data.imu.accel_y)
        self.diagnostics.is_wheel_slipping = (accel_mag > WHEEL_SLIP_ACCEL_THRESHOLD)
        self.diagnostics.wheel_slip_accel = accel_mag
        self.diagnostics.is_stationary = (abs(v) < V_STATIC_THRESHOLD)

        # 提取或计算IMU推导转向角
        delta_imu = 0.0
        has_valid_imu_steering = False

        if abs(v) > self.fusion_config.imu_steering_vel_threshold:
            # IMU横向加速度校准
            a_y_calibrated = (accel_y - self.calibration.imu_lateral_accel_bias) * \
                           self.calibration.imu_lateral_accel_scale
            
            # δ_imu = arctan(a_y * L / v²)
            delta_imu = np.arctan2(a_y_calibrated * WHEELBASE, v * v)
            has_valid_imu_steering = True

        # 根据速度自适应权重调整
        imu_weight = self.fusion_config.imu_steering_weight
        if self.fusion_config.enable_adaptive_weighting:
            if abs(v) < 2.0:
                imu_weight *= 0.5
            elif abs(v) > 10.0:
                imu_weight *= 1.5
            imu_weight = min(1.0, max(0.0, imu_weight))

        if not has_valid_imu_steering:
            # 速度过低，退回到基础算法
            self.update_basic(sensor_data)
            return

        # 测量向量 [wheel_velocity, imu_accel_magnitude, steering_angle, imu_steering]
        z = np.array([
            v * self.calibration.wheel_calibration_factor,
            accel_mag,
            sensor_data.steering.steering_angle,
            delta_imu
        ], dtype=float)

        # 测量模型的雅可比矩阵 H (4x5)
        H = np.zeros((4, 5), dtype=float)
        H[0, 3] = 1.0  # 轮速计测量速度
        H[1, 3] = 1.0  # IMU加速度与速度相关
        H[2, 4] = 1.0  # 转向角直接测量
        H[3, 4] = 1.0  # IMU推导转向角观测

        # 测量噪声（4x4矩阵）
        r_w = float(self.r_wheel)
        r_imu = float(self.r_imu)
        r_steer = float(self.r_steering)
        r_imu_steer = float(self.r_imu_steering) if self.r_imu_steering > 0 else 0.08

        # 轮滑动时增加轮速计噪声权重
        if self.diagnostics.is_wheel_slipping:
            r_w *= 4.0

        # 降低IMU权重
        r_imu_steer = r_imu_steer / (imu_weight * imu_weight + 0.01)

        R = np.diag([r_w * r_w, r_imu * r_imu, r_steer * r_steer, r_imu_steer * r_imu_steer]).astype(float)

        # 预测测量值
        z_hat = np.array([self.x[3], 0.0, self.x[4], self.x[4]], dtype=float)

        # 残差
        y = z - z_hat
        y[2] = normalize_angle(y[2])
        y[3] = normalize_angle(y[3])

        self.diagnostics.residual_norm = np.linalg.norm(y)

        # 卡尔曼增益计算
        S = H.dot(self.P).dot(H.T) + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # 如果4x4矩阵求逆失败，退回到基础算法
            self.update_basic(sensor_data)
            return

        K = self.P.dot(H.T).dot(S_inv)

        # 状态更新
        self.x = self.x + K.dot(y)

        # 正规化角度
        self.x[2] = normalize_angle(self.x[2])
        self.x[4] = normalize_angle(self.x[4])

        # 协方差更新
        I = np.eye(5)
        self.P = (I - K.dot(H)).dot(self.P)

        self.diagnostics.update_count += 1
