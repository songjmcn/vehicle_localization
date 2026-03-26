import numpy as np

def calibrate_imu_3_axis_bias_static(ax,ay,az):
    g = 9.80665

    bx = np.mean(ax)
    by = np.mean(ay)
    if az is not None:
        bz = np.mean(az) - g  # 假设Z轴方向为重力加速度方向
    else:
        bz = 0.0
    return bx, by, bz