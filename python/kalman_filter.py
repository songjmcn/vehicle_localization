import numpy as np
from .types import VehicleState
from .motion_model import predict as motion_predict, jacobian as motion_jacobian
from .utils import normalize_angle

class KalmanFilter:
    def __init__(self, initial_state: VehicleState = None):
        # state vector: [x, y, heading, v, delta]
        if initial_state is None:
            initial_state = VehicleState()
        self.x = np.array([initial_state.x,
                           initial_state.y,
                           initial_state.heading,
                           initial_state.v,
                           initial_state.delta], dtype=float)
        self.P = np.eye(5, dtype=float) * 0.1

        # process noise Q (diag)
        q_x = 0.005
        q_y = 0.005
        q_theta = 0.01
        q_v = 0.02
        q_delta = 0.02
        self.Q = np.diag([q_x, q_y, q_theta, q_v, q_delta]).astype(float)

        # measurement noise defaults
        self.r_wheel = 0.01
        self.r_imu = 0.1
        self.r_steering = 0.05
        self.r_imu_steering = 0.08

        self.last_timestamp = None

    def get_state(self) -> VehicleState:
        return VehicleState(x=float(self.x[0]), y=float(self.x[1]),
                            heading=float(self.x[2]), v=float(self.x[3]), delta=float(self.x[4]))

    def predict(self, velocity: float, steering_angle: float, timestamp_ms: int, steering_model=None):
        """EKF predict step using motion_model.

        Args:
            velocity: measured/predicted longitudinal velocity (m/s)
            steering_angle: front wheel steering angle (rad)
            timestamp_ms: timestamp in ms
        """
        if self.last_timestamp is None:
            self.last_timestamp = timestamp_ms
            # initialize state velocity and delta
            self.x[3] = velocity
            self.x[4] = steering_angle
            return

        dt = (timestamp_ms - self.last_timestamp) / 1000.0
        if dt <= 0:
            return

        state = VehicleState(x=self.x[0], y=self.x[1], heading=self.x[2], v=self.x[3], delta=self.x[4])
        new_state = motion_predict(state, velocity, steering_angle, dt, steering_model)

        # update state vector
        self.x[0] = new_state.x
        self.x[1] = new_state.y
        self.x[2] = new_state.heading
        self.x[3] = new_state.v
        self.x[4] = new_state.delta

        # compute Jacobian F
        F = motion_jacobian(state, velocity, steering_angle, dt)

        # covariance predict
        self.P = F.dot(self.P).dot(F.T) + self.Q

        self.last_timestamp = timestamp_ms

    def update_basic(self, wheel_velocity: float, imu_accel_mag: float, steering_meas: float):
        """Basic measurement update using 3D observation vector:
           z = [wheel_velocity, imu_accel_mag, steering_meas]
           H: 3x5, H[0,3]=1, H[1,3]=1, H[2,4]=1
        """
        H = np.zeros((3,5), dtype=float)
        H[0,3] = 1.0
        H[1,3] = 1.0
        H[2,4] = 1.0

        z = np.array([wheel_velocity, imu_accel_mag, steering_meas], dtype=float)

        R = np.diag([self.r_wheel, self.r_imu, self.r_steering]).astype(float)

        # predicted measurement
        z_hat = np.array([self.x[3], 0.0, self.x[4]], dtype=float)

        y = z - z_hat
        # normalize angle residual for steering
        y[2] = normalize_angle(y[2])

        S = H.dot(self.P).dot(H.T) + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # singular innovation, skip update
            return
        K = self.P.dot(H.T).dot(S_inv)

        self.x = self.x + K.dot(y)

        # normalize angles
        self.x[2] = normalize_angle(self.x[2])
        self.x[4] = normalize_angle(self.x[4])

        I = np.eye(5)
        self.P = (I - K.dot(H)).dot(self.P)

    def update_imu_steering(self, wheel_velocity: float, imu_accel_mag: float, steering_meas: float, imu_derived_steering: float):
        """4D update including IMU-derived steering measurement."""
        H = np.zeros((4,5), dtype=float)
        H[0,3] = 1.0
        H[1,3] = 1.0
        H[2,4] = 1.0
        H[3,4] = 1.0

        z = np.array([wheel_velocity, imu_accel_mag, steering_meas, imu_derived_steering], dtype=float)
        R = np.diag([self.r_wheel, self.r_imu, self.r_steering, self.r_imu_steering]).astype(float)

        z_hat = np.array([self.x[3], 0.0, self.x[4], self.x[4]], dtype=float)
        y = z - z_hat
        y[2] = normalize_angle(y[2])
        y[3] = normalize_angle(y[3])

        S = H.dot(self.P).dot(H.T) + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P.dot(H.T).dot(S_inv)
        self.x = self.x + K.dot(y)
        self.x[2] = normalize_angle(self.x[2])
        self.x[4] = normalize_angle(self.x[4])
        I = np.eye(5)
        self.P = (I - K.dot(H)).dot(self.P)
