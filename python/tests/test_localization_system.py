from python.localization_system import LocalizationSystem
from python.types import WheelConfig, VehicleGear


def test_localization_basic_run():
    cfg = WheelConfig(pulses_per_wheel_rev=96, wheel_radius=0.35, max_count=4096)
    sys = LocalizationSystem(wheel_config=cfg)

    # simulate simple forward motion: pulses increasing by ~9 per 100ms -> ~1 m/s
    samples = []
    t0 = 1000000
    for i in range(5):
        samples.append({'left_pulse': 4090 + i*9, 'right_pulse': 4090 + i*9,
                        'timestamp': t0 + i*100,
                        'gear': VehicleGear.DRIVE,
                        'steering_wheel_angle': 0.0,
                        'imu_ax': 0.0, 'imu_ay': 0.0})

    states = []
    for s in samples:
        st = sys.process_sample(s)
        states.append(st)

    # assert monotonic x increase
    xs = [st.x for st in states]
    assert all(xs[i+1] >= xs[i] for i in range(len(xs)-1))


if __name__ == '__main__':
    test_localization_basic_run()
    print('localization system basic test passed')
