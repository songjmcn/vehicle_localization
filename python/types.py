from dataclasses import dataclass
from enum import Enum, IntEnum
import numpy as np

class VehicleGear(IntEnum):
    PARK = 0
    REVERSE = 1
    NEUTRAL = 2
    DRIVE = 3

class WheelDirection(IntEnum):
    INVALID = -1
    STATIONARY = 0
    FORWARD = 1
    REVERSE = -1

@dataclass
class WheelConfig:
    pulses_per_wheel_rev: int = 96
    wheel_radius: float = 0.35
    low_speed_threshold: float = 0.3
    pulse_timeout_ms: int = 500
    max_count: int = 4096

@dataclass
class WheelOdometryData:
    velocity: float = 0.0
    distance: float = 0.0
    timestamp: int = 0
    gear: VehicleGear = VehicleGear.NEUTRAL
    rear_left_direction: WheelDirection = WheelDirection.INVALID
    rear_right_direction: WheelDirection = WheelDirection.INVALID


@dataclass
class VehicleState:
        """Vehicle state used by motion model and Kalman filter.

        Units:
            - x, y: meters
            - heading: radians
            - v: m/s
            - delta: front wheel steering angle (radians)
        """
        x: float = 0.0
        y: float = 0.0
        heading: float = 0.0
        v: float = 0.0
        delta: float = 0.0
