import numpy as np
def compute_wheel_scale_one_segment(x_rtk,y_rtk,ds_wheel):
    # RTK 路径长度
    dx = np.diff(x_rtk)
    dy = np.diff(y_rtk)
    ds_rtk = np.hypot(dx, dy)

    s_rtk = np.sum(ds_rtk)
    s_wheel = np.sum(ds_wheel[:len(ds_rtk)])

    if s_wheel < 1.0:
        raise ValueError("wheel distance too small")

    return s_rtk / s_wheel
def calibrate_wheel_scale_private(segments, x_rtk, y_rtk, ds_wheel):
    scale_factors = []
    for (start, end) in segments:
        scale = compute_wheel_scale_one_segment(
            x_rtk[start:end],
            y_rtk[start:end],
            ds_wheel[start:end]
        )
        scale_factors.append(scale)

    if len(scale_factors) == 0:
        return None
    scales = np.array(scale_factors)
    print("wheel scale mean:", np.mean(scales))
    print("wheel scale std :", np.std(scales))
    wheel_scale = np.mean(scale_factors)
    return wheel_scale,scales

def compute_ds_from_loc(x,y,heading):
    x = np.asarray(x)
    y = np.asarray(y)
    psi = np.asarray(heading)

    dx = np.diff(x)
    dy = np.diff(y)
    psi_k = psi[:-1]

    ds =dx * np.cos(psi_k) + dy * np.sin(psi_k)
    return ds
def calibrate_wheel_scale(heading_rtk,x_rtk, y_rtk, heading_loc, x_loc, y_loc,steer_loc,timestamp):
    # 1. 计算定位路径增量 ds_loc
    ds_loc = compute_ds_from_loc(x_loc, y_loc, heading_loc)

    # 2. 检测直线段
    from utils.SteerOffsetCalib import detect_straight_segments

    straight_segments = detect_straight_segments(
        steer = steer_loc,
        heading = heading_rtk,
        x = x_rtk,
        y = y_rtk,
        timestamp=timestamp)
    print(f"Detected {len(straight_segments)} straight segments for wheel scale calibration")

    # 3. 计算 wheel scale
    wheel_scale,scales = calibrate_wheel_scale_private(
        straight_segments,
        x_rtk,
        y_rtk,
        ds_loc
    )
    return wheel_scale,scales