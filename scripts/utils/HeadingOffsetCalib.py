import numpy as np
def resample_by_distance(x, y, step=0.3):
    """
    沿轨迹每 step 米取一个点
    """
    x = np.asarray(x)
    y = np.asarray(y)

    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.hypot(dx, dy)
    s = np.concatenate([[0], np.cumsum(ds)])

    s_new = np.arange(0, s[-1], step)
    x_new = np.interp(s_new, s, x)
    y_new = np.interp(s_new, s, y)

    return x_new, y_new

def geometric_heading(x, y):
    """
    使用相邻位移反算 heading
    """
    dx = np.diff(x)
    dy = np.diff(y)

    heading = np.arctan2(dy, dx)
    return np.unwrap(heading)

def curvature(psi, step):
    """
    计算曲率
    """
    """
    κ ≈ dψ / ds
    """
    return np.abs(np.diff(psi) / step)

def detect_straight_segments(x, y,
                            step=0.3,
                            kappa_th=0.002,   # ≈ 500 m 转弯半径
                            min_len=5         # 最少 5 个点（≈1.5 m）
                        ):
    psi = geometric_heading(x, y)
    kappa = curvature(psi, step)
    print(f'psi={psi}')
    print(f'kappa={kappa}')
    straight = kappa < kappa_th

    segments = []
    start = None
    print(f'straight={straight}')
    for i, flag in enumerate(straight):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if i - start >= min_len:
                segments.append((start, i))
            start = None

    if start is not None and len(straight) - start >= min_len:
        segments.append((start, len(straight)))

    return segments, psi

def heading_bias_one_segment(x_rtk, y_rtk, psi_loc,seg):
    s, e = seg

    # RTK 几何 heading（整段）
    dx = x_rtk[e] - x_rtk[s]
    dy = y_rtk[e] - y_rtk[s]
    psi_rtk = np.arctan2(dy, dx)

    # 定位 heading（段内平均）
    psi_loc_seg = np.unwrap(psi_loc[s:e])
    psi_loc_mean = np.mean(psi_loc_seg)

    dpsi = np.unwrap([psi_loc_mean - psi_rtk])[0]
    return dpsi

def calibrate_heading_bias(x_rtk, y_rtk,psi_loc_raw,step=0.3):
    # 1. RTK 距离重采样
    x_r, y_r = resample_by_distance(x_rtk, y_rtk, step)

    # 2. 定位 heading 对齐到重采样长度
    psi_loc = psi_loc_raw[:len(x_r)]

    # 3. 直线段检测
    segments, psi_geo = detect_straight_segments(
        x_r, y_r, step=step
    )
    if len(segments) == 0:
        raise RuntimeError("No valid straight segments found")

    # 4. 每段计算 heading bias
    dpsis = []
    for seg in segments:
        dpsi = heading_bias_one_segment(
            x_r, y_r, psi_loc, seg
        )
        dpsis.append(dpsi)

    dpsis = np.unwrap(np.array(dpsis))
    print(f'dpsis for each segment: {dpsis}')
    # 5. 一致性校验（APA 工程经验值）
    if np.ptp(dpsis) > np.deg2rad(0.2):
        print(f"⚠️ Warning: heading bias variance large:{np.ptp(dpsis)} rad")

    # 6. 最终 heading bias
    heading_bias = np.mean(dpsis)

    return heading_bias, dpsis, segments
def line_fit_error(x,y):
    A = np.vstack([x, np.ones_like(x)]).T
    k,b = np.linalg.lstsq(A,y,rcond=None)[0]
    y_fit = k*x + b
    return np.std(y - y_fit )