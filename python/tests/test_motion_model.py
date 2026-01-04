import numpy as np
from python.motion_model import predict, jacobian
from python.types import VehicleState


def test_predict_basic():
    s = VehicleState(x=0.0, y=0.0, heading=0.0, v=0.0, delta=0.0)
    v = 1.0
    delta = 0.0
    dt = 1.0
    ns = predict(s, v, delta, dt)
    assert abs(ns.x - 1.0) < 1e-12
    assert abs(ns.y - 0.0) < 1e-12
    assert abs(ns.heading - 0.0) < 1e-12


def test_jacobian_fd():
    s = VehicleState(x=0.0, y=0.0, heading=0.1, v=2.0, delta=0.05)
    v = 2.0
    delta = 0.05
    dt = 0.01
    F = jacobian(s, v, delta, dt)
    # finite difference check for column corresponding to heading perturbation
    eps = 1e-6
    s_pert = VehicleState(x=s.x, y=s.y, heading=s.heading + eps, v=v, delta=delta)
    p0 = predict(s, v, delta, dt)
    p1 = predict(s_pert, v, delta, dt)
    dx_dtheta = (p1.x - p0.x) / eps
    # compare to F[0,2]
    assert abs(dx_dtheta - F[0,2]) < 1e-6


if __name__ == '__main__':
    test_predict_basic()
    test_jacobian_fd()
    print('motion model tests passed')
