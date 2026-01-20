from typing import Optional
import numpy as np
from .types import (WheelConfig, WheelOdometryData, VehicleGear, WheelDirection, 
                    VehicleDirection)

# 常数
V_STATIC_THRESHOLD = 0.02

def _is_valid_gear(gear: VehicleGear) -> bool:
    """检查档位是否有效"""
    try:
        return gear in [VehicleGear.PARK, VehicleGear.REVERSE, VehicleGear.NEUTRAL, 
                       VehicleGear.DRIVE, VehicleGear.LOW]
    except Exception:
        return False

def _gear_theoretical_direction(gear: VehicleGear) -> VehicleDirection:
    """档位推导的理论方向"""
    if gear == VehicleGear.REVERSE:
        return VehicleDirection.REVERSE
    if gear in [VehicleGear.DRIVE, VehicleGear.LOW]:
        return VehicleDirection.FORWARD
    if gear in [VehicleGear.NEUTRAL, VehicleGear.PARK]:
        return VehicleDirection.STATIONARY
    return VehicleDirection.STATIONARY

def _wheel_states_consistent(left: WheelDirection, right: WheelDirection) -> bool:
    """检测左右轮状态是否一致且有效"""
    if left == WheelDirection.INVALID and right == WheelDirection.INVALID:
        return False
    if left == WheelDirection.INVALID or right == WheelDirection.INVALID:
        return True
    return left == right

def _get_wheel_state(left: WheelDirection, right: WheelDirection) -> WheelDirection:
    """获取一致的轮状态"""
    if left == WheelDirection.INVALID and right == WheelDirection.INVALID:
        return WheelDirection.INVALID
    if left == WheelDirection.INVALID:
        return right
    if right == WheelDirection.INVALID:
        return left
    if left == right:
        return left
    return WheelDirection.INVALID

def _wheel_direction_to_vehicle_direction(wheel_state: WheelDirection) -> VehicleDirection:
    """轮向状态转换为车辆运动方向"""
    if wheel_state == WheelDirection.INVALID:
        return VehicleDirection.INVALID
    if wheel_state == WheelDirection.STATIONARY or wheel_state == WheelDirection.STOP:
        return VehicleDirection.STATIONARY
    if wheel_state == WheelDirection.FORWARD:
        return VehicleDirection.FORWARD
    if wheel_state == WheelDirection.REVERSE:
        return VehicleDirection.REVERSE
    return VehicleDirection.INVALID

def derive_vehicle_direction(velocity: float, gear: VehicleGear,
                             rear_left_state: WheelDirection, rear_right_state: WheelDirection,
                             v_static_threshold: float = V_STATIC_THRESHOLD) -> VehicleDirection:
    """推导车辆运动方向（4层优先级）"""
    
    # 档位有效性检查
    if not _is_valid_gear(gear):
        return VehicleDirection.INVALID

    # P1: 低速判断 - 仅在P/N档时应用速度阈值
    if abs(velocity) < v_static_threshold:
        if gear in [VehicleGear.PARK, VehicleGear.NEUTRAL]:
            return VehicleDirection.STATIONARY
        # 对于D/R档，即使速度很低，也应该根据档位返回方向

    # 异常检测：左右轮状态都有效但相反
    if (rear_left_state != WheelDirection.INVALID and rear_right_state != WheelDirection.INVALID):
        if ((rear_left_state == WheelDirection.FORWARD and rear_right_state == WheelDirection.REVERSE) or
            (rear_left_state == WheelDirection.REVERSE and rear_right_state == WheelDirection.FORWARD)):
            return VehicleDirection.INVALID

    # P2: 轮状态判断 - 优先使用硬件轮状态
    if _wheel_states_consistent(rear_left_state, rear_right_state):
        wheel_state = _get_wheel_state(rear_left_state, rear_right_state)
        gear_direction = _gear_theoretical_direction(gear)

        # P档：轮应该停止
        if gear == VehicleGear.PARK:
            if wheel_state != WheelDirection.STATIONARY and wheel_state != WheelDirection.STOP:
                return VehicleDirection.INVALID
            return VehicleDirection.STATIONARY

        # 前进档
        if gear_direction == VehicleDirection.FORWARD:
            if wheel_state in [WheelDirection.FORWARD]:
                return VehicleDirection.FORWARD
            elif wheel_state in [WheelDirection.STATIONARY, WheelDirection.STOP]:
                return VehicleDirection.STATIONARY
            else:
                return VehicleDirection.INVALID

        # 后退档
        if gear_direction == VehicleDirection.REVERSE:
            if wheel_state in [WheelDirection.REVERSE]:
                return VehicleDirection.REVERSE
            elif wheel_state in [WheelDirection.STATIONARY, WheelDirection.STOP]:
                return VehicleDirection.STATIONARY
            else:
                # 档位优先：R档但轮显示Forward时，优先相信档位
                return VehicleDirection.REVERSE

        # N档：由轮状态决定
        if gear == VehicleGear.NEUTRAL:
            return _wheel_direction_to_vehicle_direction(wheel_state)

    # P3: 轮状态不一致或无效 - 使用档位判断
    gear_direction = _gear_theoretical_direction(gear)
    return gear_direction

class WheelOdometryProcessor:
    def __init__(self, config: Optional[WheelConfig] = None):
        self.config = config if config is not None else WheelConfig()
        self.is_initialized = False
        self.first_call = True
        self.last_left = 0
        self.last_right = 0
        self.last_timestamp = 0
        self.accum_distance = 0.0
        self.pulse_residual = 0.0
        self.last_velocity = 0.0
        self.is_timeout = False

    def init(self):
        """初始化处理器"""
        self.is_initialized = True
        self.first_call = True
        self.accum_distance = 0.0
        self.pulse_residual = 0.0
        self.last_velocity = 0.0

    def update(self, left_pulse: int, right_pulse: int, timestamp: int,
               gear: VehicleGear, rear_left_state: WheelDirection, rear_right_state: WheelDirection) -> WheelOdometryData:
        """处理脉冲数据"""
        if not self.is_initialized:
            self.init()

        # 首次调用：记录并返回零值
        if self.first_call:
            self.last_left = left_pulse
            self.last_right = right_pulse
            self.last_timestamp = timestamp
            self.first_call = False
            return WheelOdometryData(velocity=0.0, distance=0.0, timestamp=timestamp,
                                     gear=gear, rear_left_direction=rear_left_state,
                                     rear_right_direction=rear_right_state,
                                     vehicle_direction=VehicleDirection.INVALID)

        max_count = int(self.config.max_count)

        # 计算脉冲增量（处理溢出）
        def delta_with_overflow(curr, last):
            if max_count > 0:
                if curr < last and (last - curr) > (max_count // 2):
                    return (max_count - last) + curr
                else:
                    return curr - last
            else:
                return curr - last

        left_delta = int(delta_with_overflow(left_pulse, self.last_left))
        right_delta = int(delta_with_overflow(right_pulse, self.last_right))

        # 平均脉冲（处理小数部分累积）
        pulse_delta_float = (left_delta + right_delta) / 2.0 + self.pulse_residual
        pulse_delta = int(np.floor(pulse_delta_float))
        self.pulse_residual = pulse_delta_float - pulse_delta

        # 异常数据检测：两轮同时接近0但上一次不是溢出
        if self.config.max_count > 0:
            if left_pulse < 10 and right_pulse < 10:
                if self.last_left > 100 and self.last_right > 100:
                    if (self.last_left < (self.config.max_count - 100) and
                        self.last_right < (self.config.max_count - 100)):
                        # 异常：忽略此次更新
                        return WheelOdometryData(velocity=self.last_velocity,
                                               distance=self.accum_distance,
                                               timestamp=timestamp,
                                               gear=gear,
                                               rear_left_direction=WheelDirection.INVALID,
                                               rear_right_direction=WheelDirection.INVALID,
                                               vehicle_direction=VehicleDirection.INVALID)

        # 计算时间增量
        dt_ms = timestamp - self.last_timestamp
        if dt_ms <= 0:
            return WheelOdometryData(velocity=self.last_velocity, distance=self.accum_distance,
                                    timestamp=timestamp, gear=gear,
                                    rear_left_direction=rear_left_state,
                                    rear_right_direction=rear_right_state,
                                    vehicle_direction=VehicleDirection.INVALID)

        # 检查超时
        if dt_ms > self.config.pulse_timeout_ms and pulse_delta < 0.5:
            self.is_timeout = True
            return WheelOdometryData(velocity=0.0, distance=self.accum_distance,
                                    timestamp=timestamp, gear=gear,
                                    rear_left_direction=WheelDirection.STOP,
                                    rear_right_direction=WheelDirection.STOP,
                                    vehicle_direction=VehicleDirection.STATIONARY)

        self.is_timeout = False
        dt = dt_ms / 1000.0

        # 距离计算
        wheel_circumference = 2.0 * np.pi * self.config.wheel_radius
        distance_delta = (float(pulse_delta) / float(self.config.pulses_per_wheel_rev)) * wheel_circumference

        # 计算原始速度
        raw_velocity = distance_delta / dt

        # 推导车辆运动方向
        adjusted_velocity_for_direction = raw_velocity
        vehicle_dir = derive_vehicle_direction(adjusted_velocity_for_direction, gear,
                                              rear_left_state, rear_right_state,
                                              self.config.low_speed_threshold)

        # 应用方向到最终距离和速度
        final_distance_delta = abs(distance_delta)
        if vehicle_dir == VehicleDirection.REVERSE:
            final_distance_delta = -final_distance_delta
        elif vehicle_dir == VehicleDirection.STATIONARY:
            final_distance_delta = 0.0

        velocity = final_distance_delta / dt
        self.accum_distance += final_distance_delta

        # 更新最后样本
        self.last_left = left_pulse
        self.last_right = right_pulse
        self.last_timestamp = timestamp
        self.last_velocity = velocity

        return WheelOdometryData(velocity=velocity, distance=self.accum_distance,
                               timestamp=timestamp, gear=gear,
                               rear_left_direction=rear_left_state,
                               rear_right_direction=rear_right_state,
                               vehicle_direction=vehicle_dir)

    def set_velocity(self, velocity: float, timestamp: int) -> WheelOdometryData:
        """直接设置速度（不使用脉冲）"""
        if not self.is_initialized:
            self.init()

        # 检查超时
        if self.last_timestamp > 0:
            time_delta = timestamp - self.last_timestamp
            if time_delta > self.config.pulse_timeout_ms and abs(velocity) < 1e-6:
                self.is_timeout = True
                velocity = 0.0
            else:
                self.is_timeout = False

        self.last_timestamp = timestamp

        # 极低速补零
        if abs(velocity) < V_STATIC_THRESHOLD:
            velocity = 0.0

        # 保留速度符号用于积分
        signed_velocity = velocity

        # 积分距离
        if self.last_timestamp > 0 and timestamp > self.last_timestamp:
            dt = (timestamp - self.last_timestamp) / 1000.0
            if dt > 0:
                self.accum_distance += signed_velocity * dt

        self.last_velocity = signed_velocity

        return WheelOdometryData(velocity=signed_velocity, distance=self.accum_distance,
                               timestamp=timestamp)

    def reset(self):
        """重置处理器"""
        self.last_left = 0
        self.last_right = 0
        self.accum_distance = 0.0
        self.pulse_residual = 0.0
        self.last_velocity = 0.0
        self.last_timestamp = 0
        self.is_timeout = False
        self.first_call = True

    def is_timeout_check(self) -> bool:
        """检查是否超时"""
        return self.is_timeout
