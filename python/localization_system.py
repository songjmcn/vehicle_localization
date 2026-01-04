from .wheel_odometry import WheelOdometryProcessor
from .steering_model import SteeringModel, SteeringConfig
from .kalman_filter import KalmanFilter
from .types import WheelConfig, VehicleState, VehicleGear, WheelDirection
import numpy as np

class LocalizationSystem:
    """Top-level localization integrator that wires wheel odometry, steering and EKF.

    Simplified Python port of `src/localization_system.c` behavior for testing and analysis.
    """
    def __init__(self,
                 wheel_config: WheelConfig = None,
                 steering_config: SteeringConfig = None,
                 initial_state: VehicleState = None):
        self.wheel_cfg = wheel_config if wheel_config is not None else WheelConfig()
        self.steering_cfg = steering_config if steering_config is not None else SteeringConfig()

        self.wheel = WheelOdometryProcessor(self.wheel_cfg)
        self.steering = SteeringModel(self.steering_cfg)
        self.kf = KalmanFilter(initial_state=initial_state)

    def process_sample(self, sample: dict):
        """Process one sensor sample (dict) and return estimated VehicleState.

        Expected keys in sample dict:
          - left_pulse (int), right_pulse (int), timestamp (ms, int)
          - gear (VehicleGear or int)
          - steering_wheel_angle (float, rad)
          - imu_ax, imu_ay (float, m/s^2) optional
        """
        left = int(sample.get('left_pulse', 0))
        right = int(sample.get('right_pulse', 0))
        ts = int(sample.get('timestamp', 0))
        gear = sample.get('gear', VehicleGear.DRIVE)
        if isinstance(gear, int):
            gear = VehicleGear(gear)
        sw_angle = float(sample.get('steering_wheel_angle', 0.0))

        imu_ax = float(sample.get('imu_ax', 0.0))
        imu_ay = float(sample.get('imu_ay', 0.0))
        imu_accel_mag = np.hypot(imu_ax, imu_ay)

        # process wheel odometry
        wout = self.wheel.update(left, right, ts, gear, WheelDirection.FORWARD, WheelDirection.FORWARD)

        # process steering
        front = self.steering.update(sw_angle, ts)

        # EKF predict
        self.kf.predict(velocity=wout.velocity, steering_angle=front, timestamp_ms=ts, steering_model=self.steering)

        # EKF update (basic)
        self.kf.update_basic(wheel_velocity=wout.velocity, imu_accel_mag=imu_accel_mag, steering_meas=front)

        return self.kf.get_state()
