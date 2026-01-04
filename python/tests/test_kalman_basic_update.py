from python.kalman_filter import KalmanFilter
from python.types import VehicleState


def test_kf_basic_update():
    init = VehicleState(x=0.0, y=0.0, heading=0.0, v=0.0, delta=0.0)
    kf = KalmanFilter(initial_state=init)
    # predict step (initialize timestamp)
    kf.predict(0.0, 0.0, 1000)
    # small dt predict
    kf.predict(0.0, 0.0, 1100)
    # measurement says wheel velocity = 1.0 m/s
    kf.update_basic(wheel_velocity=1.0, imu_accel_mag=0.0, steering_meas=0.0)
    # after update, state velocity should move towards 1.0
    v = kf.get_state().v
    assert v > 0.0


if __name__ == '__main__':
    test_kf_basic_update()
    print('kalman basic update test passed')
