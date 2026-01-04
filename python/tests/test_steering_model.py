from python.steering_model import SteeringModel, SteeringConfig, MappingTable


def test_wheel_to_front_linear():
    cfg = SteeringConfig(steering_ratio=10.0)
    sm = SteeringModel(cfg)
    # steering wheel angle = steering_ratio * front -> front expected = 0.1
    sw = 10.0 * 0.1
    front = sm.wheel_to_front(sw)
    assert abs(front - 0.1) < 1e-9


def test_compute_yaw_rate():
    cfg = SteeringConfig(wheelbase=2.0)
    sm = SteeringModel(cfg)
    v = 2.0
    front = 0.1
    yaw = sm.compute_yaw_rate(v, front)
    # yaw = v * tan(0.1) / L
    import math
    expected = v * math.tan(front) / cfg.wheelbase
    assert abs(yaw - expected) < 1e-12


if __name__ == '__main__':
    test_wheel_to_front_linear()
    test_compute_yaw_rate()
    print('steering tests passed')
