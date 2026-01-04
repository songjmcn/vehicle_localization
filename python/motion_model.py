from dataclasses import asdict
import numpy as np
from .types import VehicleState
from .utils import normalize_angle


def predict(state: VehicleState, velocity: float, steering_angle: float, dt: float, steering_model=None) -> VehicleState:
    """Predict next vehicle state using rear-axle-center single-track model.

    Args:
        state: VehicleState (x,y,heading,v,delta)
        velocity: longitudinal velocity (m/s), signed
        steering_angle: front wheel steering angle (rad)
        dt: timestep in seconds
        steering_model: optional SteeringModel for advanced yaw-rate
    Returns:
        VehicleState: new predicted state
    """
    if dt <= 0:
        return VehicleState(x=state.x, y=state.y, heading=state.heading, v=velocity, delta=steering_angle)

    x = state.x
    y = state.y
    theta = state.heading

    v_signed = float(velocity)
    delta = float(steering_angle)

    # compute yaw rate
    L = 2.76  # default wheelbase; steering_model may override
    if steering_model is not None:
        try:
            L = steering_model.config.wheelbase
        except Exception:
            pass

    yaw_rate = 0.0
    # use ackermann approx if available
    if steering_model is not None:
        yaw_rate = steering_model.compute_yaw_rate(v_signed, delta)
    else:
        # simple model
        yaw_rate = v_signed * np.tan(delta) / L

    new_theta = normalize_angle(theta + yaw_rate * dt)
    new_x = x + v_signed * np.cos(theta) * dt
    new_y = y + v_signed * np.sin(theta) * dt

    return VehicleState(x=new_x, y=new_y, heading=new_theta, v=v_signed, delta=delta)


def jacobian(state: VehicleState, velocity: float, steering_angle: float, dt: float) -> np.ndarray:
    """Return the 5x5 motion model Jacobian matrix F.

    State vector ordering: [x, y, heading, v, delta]
    """
    F = np.eye(5, dtype=float)
    theta = state.heading
    v = float(velocity)
    delta = float(steering_angle)
    L = 2.76

    # partials
    F[0,2] = -v * np.sin(theta) * dt
    F[1,2] = v * np.cos(theta) * dt

    F[0,3] = np.cos(theta) * dt
    F[1,3] = np.sin(theta) * dt

    # d(heading)/dv and d(heading)/ddelta
    # heading += (v * tan(delta) / L) * dt
    F[2,3] = (np.tan(delta) / L) * dt
    F[2,4] = (v / (L * (np.cos(delta)**2))) * dt

    return F
