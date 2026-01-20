from dataclasses import asdict
import numpy as np
from .types import VehicleState, VehicleDirection
from .utils import normalize_angle

# 常数
V_STATIC_THRESHOLD = 0.02

def get_sign_v_from_direction(direction: VehicleDirection) -> int:
    """从方向获取速度符号"""
    if direction == VehicleDirection.FORWARD:
        return 1
    elif direction == VehicleDirection.REVERSE:
        return -1
    else:
        return 0

def predict(state: VehicleState, velocity: float, steering_angle: float, dt: float, steering_model=None) -> VehicleState:
    """使用后轴中心单轨模型预测车辆状态
    
    遵循 ISO 8855 标准运动方程（后轴中心原点）
    """
    if dt <= 0:
        return VehicleState(x=state.x, y=state.y, heading=state.heading, v=velocity, delta=steering_angle)

    x = state.x
    y = state.y
    theta = state.heading

    v_signed = float(velocity)
    delta = float(steering_angle)

    # 获取车辆方向（由上层调用者已计算）
    direction = state.direction

    # 边界情况：如果方向未计算，基于速度判断
    if direction == VehicleDirection.INVALID:
        if abs(velocity) < V_STATIC_THRESHOLD:
            direction = VehicleDirection.STATIONARY
        else:
            direction = VehicleDirection.FORWARD if velocity > 0 else VehicleDirection.REVERSE

    # 检查是否完全静止（基于方向）
    if direction == VehicleDirection.STATIONARY:
        new_state = VehicleState(x=state.x, y=state.y, heading=state.heading, 
                                v=0.0, delta=steering_angle, gear=state.gear,
                                direction=VehicleDirection.STATIONARY)
        return new_state

    # 计算yaw rate
    L = 2.76  # 默认轴距
    if steering_model is not None:
        try:
            L = steering_model.config.wheelbase
        except Exception:
            pass

    tan_delta = np.tan(delta)
    yaw_rate = v_signed * tan_delta / L

    # 更新航向角（规范化到[-π, π]）
    new_heading = normalize_angle(theta + yaw_rate * dt)

    # 使用初始heading更新位置（离散积分的正确做法）
    cos_term = np.cos(theta)
    sin_term = np.sin(theta)

    new_x = x + v_signed * cos_term * dt
    new_y = y + v_signed * sin_term * dt

    new_state = VehicleState(x=new_x, y=new_y, heading=new_heading, v=v_signed, 
                            delta=steering_angle, gear=state.gear, direction=direction)
    return new_state

def jacobian(state: VehicleState, velocity: float, steering_angle: float, dt: float) -> np.ndarray:
    """计算运动模型的5x5雅可比矩阵
    
    状态向量顺序: [x, y, heading, v, delta]
    """
    F = np.eye(5, dtype=float)
    theta = state.heading
    v = float(velocity)
    delta = float(steering_angle)
    L = 2.76

    # 获取方向
    direction = state.direction
    if direction == VehicleDirection.INVALID:
        if abs(velocity) < V_STATIC_THRESHOLD:
            direction = VehicleDirection.STATIONARY
        else:
            direction = VehicleDirection.FORWARD if velocity > 0 else VehicleDirection.REVERSE

    # 位置关于航向角的偏导
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # ∂x/∂θ = -v·sin(θ)·dt
    F[0, 2] = -v * sin_theta * dt
    # ∂y/∂θ = v·cos(θ)·dt
    F[1, 2] = v * cos_theta * dt

    # 位置关于速度的偏导
    # ∂x/∂v = cos(θ)·dt
    F[0, 3] = cos_theta * dt
    # ∂y/∂v = sin(θ)·dt
    F[1, 3] = sin_theta * dt

    # 航向角关于控制量的偏导
    cos_delta = np.cos(delta)
    if abs(cos_delta) > 1e-6:
        tan_delta = np.tan(delta)
        
        # ∂θ/∂v = tan(δ)·dt/L
        F[2, 3] = tan_delta / L * dt

        # ∂θ/∂δ = v·dt/(L·cos²(δ))
        F[2, 4] = v / (L * cos_delta * cos_delta) * dt

    return F
