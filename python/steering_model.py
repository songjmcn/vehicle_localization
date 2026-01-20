from dataclasses import dataclass
from typing import Optional, Sequence
import numpy as np
from .utils import clamp

@dataclass
class SteeringConfig:
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

class MappingTable:
    """映射表：方向盘转角 -> (左前轮角, 右前轮角)"""
    def __init__(self, mapping_x: Sequence[float], mapping_l: Sequence[float], mapping_r: Sequence[float]):
        self.x = np.array(mapping_x, dtype=float)
        self.l = np.array(mapping_l, dtype=float)
        self.r = np.array(mapping_r, dtype=float)
        if not (self.x.size == self.l.size == self.r.size):
            raise ValueError('mapping arrays must have same length')

    def query(self, sw_angle: float):
        """线性插值查询映射表"""
        if sw_angle <= self.x[0]:
            return float(self.l[0]), float(self.r[0])
        if sw_angle >= self.x[-1]:
            return float(self.l[-1]), float(self.r[-1])
        idx = np.searchsorted(self.x, sw_angle) - 1
        x0, x1 = self.x[idx], self.x[idx+1]
        t = (sw_angle - x0) / (x1 - x0)
        l = (1-t)*self.l[idx] + t*self.l[idx+1]
        r = (1-t)*self.r[idx] + t*self.r[idx+1]
        return float(l), float(r)

class SteeringModel:
    def __init__(self, config: Optional[SteeringConfig] = None, mapping: Optional[MappingTable] = None):
        self.config = config if config is not None else SteeringConfig()
        self.mapping = mapping
        
        # 转向状态
        self.filtered_angle = 0.0
        self.prev_angle = 0.0
        self.prev_ts = None
        self.steering_rate = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.mean_wheel_angle = 0.0

    def wheel_to_front(self, steering_wheel_angle: float):
        """方向盘转角转前轮转向角"""
        if self.mapping is not None:
            left, right = self.mapping.query(steering_wheel_angle)
            mean_front = 0.5 * (left + right)
        else:
            mean_front = steering_wheel_angle / self.config.steering_ratio

        # 应用零偏和增益补偿
        calibrated = (mean_front - self.config.steering_angle_offset) * self.config.steering_angle_gain
        calibrated = clamp(calibrated, -self.config.max_front_wheel_angle, self.config.max_front_wheel_angle)
        return calibrated

    def update(self, steering_wheel_angle: float, timestamp_ms: Optional[int] = None) -> float:
        """处理新的方向盘转角样本，返回滤波后的前轮转向角"""
        # 步骤1：根据映射表查询前左轮和前右轮转角
        if self.mapping is not None:
            self.left_wheel_angle, self.right_wheel_angle = self.mapping.query(steering_wheel_angle)
        else:
            basic = steering_wheel_angle / self.config.steering_ratio
            self.left_wheel_angle = basic
            self.right_wheel_angle = basic

        # 步骤2：计算平均转向角
        self.mean_wheel_angle = (self.left_wheel_angle + self.right_wheel_angle) / 2.0

        # 步骤3：应用校准
        front = self.wheel_to_front(steering_wheel_angle)

        # 步骤4：计算转向角速率
        if timestamp_ms is not None and self.prev_ts is not None:
            dt = (timestamp_ms - self.prev_ts) / 1000.0
            if dt > 0:
                self.steering_rate = (front - self.prev_angle) / dt

        # 步骤5：应用低通滤波
        if self.config.enable_steering_filter:
            alpha = self.config.steering_filter_alpha
            self.filtered_angle = alpha * self.filtered_angle + (1.0 - alpha) * front
        else:
            self.filtered_angle = front

        # 步骤5.5：应用死区处理
        if abs(self.filtered_angle) < self.config.steering_deadzone:
            self.filtered_angle = 0.0

        # 步骤6：更新状态
        self.prev_angle = front
        if timestamp_ms is not None:
            self.prev_ts = timestamp_ms

        return float(self.filtered_angle)

    def get_filtered_angle(self) -> float:
        """获取滤波后的前轮转向角"""
        return float(self.filtered_angle)

    def get_steering_rate(self) -> float:
        """获取转向角速率"""
        return float(self.steering_rate)

    def get_wheel_angles(self):
        """获取前轮转角（左右分别）"""
        return float(self.left_wheel_angle), float(self.right_wheel_angle), float(self.mean_wheel_angle)

    def compute_yaw_rate(self, velocity: float, steering_front_angle: float) -> float:
        """计算航向角速率（使用Ackermann模型）"""
        if abs(velocity) < 0.01:
            return 0.0
        
        L = self.config.wheelbase
        
        # 使用Ackermann模型：考虑左右轮转角差异
        left_rad = self.left_wheel_angle if abs(self.left_wheel_angle) > 0 else steering_front_angle
        right_rad = self.right_wheel_angle if abs(self.right_wheel_angle) > 0 else steering_front_angle
        
        yaw_rate = velocity * (np.tan(left_rad) + np.tan(right_rad)) / (2.0 * L)
        return float(yaw_rate)

    def apply_rtk_calibration(self, calibration_params):
        """应用RTK标定结果"""
        if calibration_params is None:
            return

        # 应用转向角零偏
        self.config.steering_angle_offset = calibration_params.steering_angle_offset

        # 应用转向角增益（验证范围）
        if (calibration_params.steering_angle_gain < 0.5 or 
            calibration_params.steering_angle_gain > 1.5):
            self.config.steering_angle_gain = 1.0
            return False

        self.config.steering_angle_gain = calibration_params.steering_angle_gain
        self.config.ackermann_factor = calibration_params.ackermann_factor
        return True

    def reset(self):
        """重置转向模型状态"""
        self.filtered_angle = 0.0
        self.prev_angle = 0.0
        self.steering_rate = 0.0
        self.prev_ts = None
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.mean_wheel_angle = 0.0

    def clamp_angle(self, angle: float) -> float:
        """限制转向角在有效范围内"""
        return clamp(angle, -self.config.max_front_wheel_angle, self.config.max_front_wheel_angle)
