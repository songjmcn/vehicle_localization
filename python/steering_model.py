from dataclasses import dataclass
from typing import Optional, Sequence
import numpy as np
from .utils import clamp

@dataclass
class SteeringConfig:
    steering_ratio: float = 15.3
    wheelbase: float = 2.76
    max_steering_wheel_angle: float = 8.79645943  # rad
    max_front_wheel_angle: float = 0.676454711    # rad
    steering_angle_offset: float = 0.0
    steering_angle_gain: float = 1.0
    enable_steering_filter: bool = True
    steering_filter_alpha: float = 0.1
    steering_deadzone: float = 0.02

class MappingTable:
    """Optional mapping table from steering wheel angle -> (left_front, right_front).

    mapping_x: sequence of steering wheel angles (monotonic)
    mapping_l: left front wheel angles
    mapping_r: right front wheel angles
    """
    def __init__(self, mapping_x: Sequence[float], mapping_l: Sequence[float], mapping_r: Sequence[float]):
        self.x = np.array(mapping_x, dtype=float)
        self.l = np.array(mapping_l, dtype=float)
        self.r = np.array(mapping_r, dtype=float)
        if not (self.x.size == self.l.size == self.r.size):
            raise ValueError('mapping arrays must have same length')

    def query(self, sw_angle: float):
        # linear interpolation with boundary clamping
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
        self.filtered_angle = 0.0
        self.prev_angle = 0.0
        self.prev_ts = None
        self.steering_rate = 0.0

    def wheel_to_front(self, steering_wheel_angle: float):
        """Map steering wheel angle to front wheel angle (mean of left/right).

        Uses mapping table if provided; otherwise linear model: front = steering_wheel_angle / steering_ratio
        Then apply offset and gain and clamp to max_front_wheel_angle.
        """
        if self.mapping is not None:
            left, right = self.mapping.query(steering_wheel_angle)
            mean_front = 0.5 * (left + right)
        else:
            mean_front = steering_wheel_angle / self.config.steering_ratio

        calibrated = (mean_front - self.config.steering_angle_offset) * self.config.steering_angle_gain
        calibrated = clamp(calibrated, -self.config.max_front_wheel_angle, self.config.max_front_wheel_angle)
        return calibrated

    def update(self, steering_wheel_angle: float, timestamp_ms: Optional[int] = None) -> float:
        """Process a new steering wheel angle sample and return filtered front wheel angle."""
        front = self.wheel_to_front(steering_wheel_angle)
        if timestamp_ms is not None and self.prev_ts is not None:
            dt = (timestamp_ms - self.prev_ts) / 1000.0
            if dt > 0:
                self.steering_rate = (front - self.prev_angle) / dt
        # filtering
        if self.config.enable_steering_filter:
            alpha = self.config.steering_filter_alpha
            self.filtered_angle = alpha * self.filtered_angle + (1.0 - alpha) * front
        else:
            self.filtered_angle = front
        # deadzone
        if abs(self.filtered_angle) < self.config.steering_deadzone:
            self.filtered_angle = 0.0
        # update previous
        self.prev_angle = front
        if timestamp_ms is not None:
            self.prev_ts = timestamp_ms
        return float(self.filtered_angle)

    def compute_yaw_rate(self, velocity: float, steering_front_angle: float) -> float:
        """Compute yaw rate (rad/s) using small-angle or Ackermann approximation."""
        # avoid division by zero
        L = self.config.wheelbase
        return float(velocity * np.tan(steering_front_angle) / L)
