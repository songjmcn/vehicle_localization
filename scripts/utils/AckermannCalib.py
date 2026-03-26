import numpy as np
from SteerGainCalib import circle_fit_kappa
from scipy.optimize import minimize
def compute_delta_base(steer, offset, gain):
    return (steer - offset) * gain

def calibrate_cubic_ackermann(
        steer_left,
        steer_right,
        kappa_left,
        kappa_right,
        offset,
        gain,
        wheelbase):

    steer_all = np.concatenate([steer_left, steer_right])
    kappa_all = np.concatenate([kappa_left, kappa_right])

    delta_base = compute_delta_base(steer_all, offset, gain)

    def cost(k):

        delta = delta_base + k * delta_base**3
        kappa_model = np.tan(delta) / wheelbase

        return np.mean((kappa_model - kappa_all)**2)

    res = minimize(cost, x0=0.0)

    return res.x[0]

def calibrate_lr_gain(
        steer,
        kappa_rtk,
        offset,
        gain,
        ack_cubic,
        wheelbase):

    delta_base = compute_delta_base(steer, offset, gain)
    delta_nl = delta_base + ack_cubic * delta_base**3

    def cost(g):

        delta_real = g * delta_nl
        kappa_model = np.tan(delta_real) / wheelbase

        return np.mean((kappa_model - kappa_rtk)**2)

    res = minimize(cost, x0=1.0)

    return res.x[0]

def calibrate_ackermann_lr_model(
        steer_left,
        x_left,
        y_left,
        steer_right,
        x_right,
        y_right,
        steer_offset,
        steer_gain,
        wheelbase):

    # ===== RTK曲率 =====
    kappa_left = circle_fit_kappa(x_left, y_left)
    kappa_right = circle_fit_kappa(x_right, y_right)

    # 扩展成数组（每个样本统一目标）
    kappa_left_arr = np.full_like(steer_left, kappa_left)
    kappa_right_arr = np.full_like(steer_right, kappa_right)

    # ===== 标定 cubic =====
    ack_cubic = calibrate_cubic_ackermann(
        steer_left,
        steer_right,
        kappa_left_arr,
        kappa_right_arr,
        steer_offset,
        steer_gain,
        wheelbase)

    # ===== 标定左右 gain =====
    gain_left = calibrate_lr_gain(
        steer_left,
        kappa_left_arr,
        steer_offset,
        steer_gain,
        ack_cubic,
        wheelbase)

    gain_right = calibrate_lr_gain(
        steer_right,
        kappa_right_arr,
        steer_offset,
        steer_gain,
        ack_cubic,
        wheelbase)

    return ack_cubic, gain_left, gain_right