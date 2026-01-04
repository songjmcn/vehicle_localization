from typing import Optional
import numpy as np
from .types import WheelConfig, WheelOdometryData, VehicleGear, WheelDirection

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

    def init(self):
        self.is_initialized = True
        self.first_call = True
        self.accum_distance = 0.0
        self.pulse_residual = 0.0
        self.last_velocity = 0.0

    def update(self, left_pulse: int, right_pulse: int, timestamp: int,
               gear: VehicleGear, rear_left_state: WheelDirection, rear_right_state: WheelDirection) -> WheelOdometryData:
        """Process pulses and return WheelOdometryData.

        Implements overflow handling and distance calculation consistent with C implementation.
        """
        if not self.is_initialized:
            self.init()

        # first call: record and return zeros
        if self.first_call:
            self.last_left = left_pulse
            self.last_right = right_pulse
            self.last_timestamp = timestamp
            self.first_call = False
            return WheelOdometryData(velocity=0.0, distance=0.0, timestamp=timestamp,
                                     gear=gear, rear_left_direction=rear_left_state,
                                     rear_right_direction=rear_right_state)

        max_count = int(self.config.max_count)
        # compute deltas with overflow handling
        def delta_with_overflow(curr, last):
            if max_count > 0:
                # detect wrap
                if curr < last and (last - curr) > (max_count // 2):
                    return (max_count - last) + curr
                else:
                    return curr - last
            else:
                return curr - last

        left_delta = int(delta_with_overflow(left_pulse, self.last_left))
        right_delta = int(delta_with_overflow(right_pulse, self.last_right))

        # average pulses
        pulse_delta_float = (left_delta + right_delta) / 2.0 + self.pulse_residual
        pulse_delta = int(np.floor(pulse_delta_float))
        self.pulse_residual = pulse_delta_float - pulse_delta

        # time delta
        dt_ms = timestamp - self.last_timestamp
        if dt_ms <= 0:
            # no time elapsed, return last known
            return WheelOdometryData(velocity=self.last_velocity, distance=self.accum_distance,
                                     timestamp=timestamp, gear=gear,
                                     rear_left_direction=rear_left_state,
                                     rear_right_direction=rear_right_state)
        dt = dt_ms / 1000.0

        # distance calculation
        wheel_circumference = 2.0 * np.pi * self.config.wheel_radius
        distance_delta = (float(pulse_delta) / float(self.config.pulses_per_wheel_rev)) * wheel_circumference

        # determine sign/direction: simplistic - based on gear
        if gear == VehicleGear.REVERSE:
            distance_signed = -abs(distance_delta)
        elif gear == VehicleGear.PARK or gear == VehicleGear.NEUTRAL:
            # if very small, consider stationary
            if abs(distance_delta) < (self.config.low_speed_threshold * dt):
                distance_signed = 0.0
            else:
                distance_signed = distance_delta
        else:
            distance_signed = abs(distance_delta)

        velocity = distance_signed / dt
        self.accum_distance += distance_signed

        # update last samples
        self.last_left = left_pulse
        self.last_right = right_pulse
        self.last_timestamp = timestamp
        self.last_velocity = velocity

        return WheelOdometryData(velocity=velocity, distance=self.accum_distance,
                                 timestamp=timestamp, gear=gear,
                                 rear_left_direction=rear_left_state,
                                 rear_right_direction=rear_right_state)
