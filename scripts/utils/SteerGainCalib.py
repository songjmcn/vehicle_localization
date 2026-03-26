from scipy.optimize import minimize
import numpy as np

def steer_gain_cost(gain, steer, kappa_rtk, offset, L):
    delta = gain * (steer+ offset)
    kappa_model = np.tan(delta) / L
    return np.mean((kappa_model - kappa_rtk) ** 2 )

def calibrate_steer_gain_private(steer, kappa_rtk, offset,L):
    mask = np.abs(steer) > np.deg2rad(5)  # 只使用大于5度的转向数据进行标定

    steer_valid = steer[mask]
    kappa_valid = kappa_rtk[mask]

    res = minimize(
        steer,
        x0= 1.0,
        args= (steer_valid, kappa_valid, offset,L)
    )
    return res.x[0]

def circle_fit_kappa(x,y):
    xc = np.mean(x)
    yc = np.mean(y)

    r = np.mean(np.sqrt((x- xc) ** 2 + (y - yc) ** 2))
    return 1.0/r

def calibrate_steer_gain(steer, x_rtk, y_rtk, offset, L):
    kappa_rtk = circle_fit_kappa(x_rtk, y_rtk)
    print(f"RTK曲率: {kappa_rtk:.6f} 1/m")
    steer_gain = calibrate_steer_gain_private(steer, kappa_rtk, offset,L)
    print(f"标定得到的转向增益: {steer_gain:.4f}")
    return steer_gain