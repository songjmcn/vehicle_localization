from python.wheel_odometry import WheelOdometryProcessor
from python.types import WheelConfig, VehicleGear, WheelDirection


def test_overflow_behavior():
    cfg = WheelConfig(pulses_per_wheel_rev=96, wheel_radius=0.35, max_count=4096)
    proc = WheelOdometryProcessor(cfg)
    # first call
    out1 = proc.update(4090, 4090, 1000, VehicleGear.DRIVE, WheelDirection.FORWARD, WheelDirection.FORWARD)
    assert out1.velocity == 0.0
    # simulate wrap: last=4090 -> curr small
    out2 = proc.update(3, 4, 1100, VehicleGear.DRIVE, WheelDirection.FORWARD, WheelDirection.FORWARD)
    # left delta should be (4096-4090)+3 = 9
    # right delta = (4096-4090)+4 = 10 => avg ~9.5 -> floor 9 pulses
    # distance for 9 pulses: 9/96 * 2*pi*0.35
    expected_dist = (9.0/96.0) * (2.0 * 3.141592653589793 * 0.35)
    # velocity = distance / dt (0.1s)
    expected_vel = expected_dist / 0.1
    assert abs(out2.velocity - expected_vel) < 1e-6


if __name__ == '__main__':
    test_overflow_behavior()
    print('test passed')
