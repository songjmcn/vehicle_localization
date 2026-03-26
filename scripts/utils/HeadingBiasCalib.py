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

    dpsi = np.unwrap([psi_rtk - psi_loc_mean])[0]
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

def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))

def geometric_heading_reverse(x,y,ds_min=0.05):
    """
    计算后退工况下的几何 heading
    """

    N = len(x)
    psi = np.zeros(N)
    last_valid = 0

    for i in range(1, N):

        dx = x[i] - x[last_valid]
        dy = y[i] - y[last_valid]
        ds = np.hypot(dx, dy)

        if ds > ds_min:
            psi_geo = np.arctan2(dy, dx)
            psi_vehicle = wrap_angle(psi_geo + np.pi)   # ⭐ 后退修正
            psi[i] = psi_vehicle
            last_valid = i
        else:
            psi[i] = psi[i-1]

    return psi

def detect_reverse_straight_segments(x,y,kappa_th=0.002,min_len=5.0):
    """
    返回多个直线段 (start, end)
    """

    N = len(x)
    segments = []
    start = None
    length = 0

    for i in range(2, N):

        dx1 = x[i-1] - x[i-2]
        dy1 = y[i-1] - y[i-2]

        dx2 = x[i] - x[i-1]
        dy2 = y[i] - y[i-1]

        ds = np.hypot(dx2, dy2)
        if ds < 1e-3:
            continue

        psi1 = np.arctan2(dy1, dx1)
        psi2 = np.arctan2(dy2, dx2)

        dpsi = wrap_angle(psi2 - psi1)
        kappa = abs(dpsi / ds)

        if kappa < kappa_th:

            if start is None:
                start = i-2
                length = 0

            length += ds

        else:
            if start is not None and length > min_len:
                segments.append((start, i))
            start = None
            length = 0

    if start is not None:
        segments.append((start, N-1))

    return segments

def heading_bias_reverse_one_segment(x_rtk,y_rtk,psi_loc,seg):

    s, e = seg

    dx = x_rtk[e] - x_rtk[s]
    dy = y_rtk[e] - y_rtk[s]

    # RTK 车辆 heading（后退）
    psi_geo = np.arctan2(dy, dx)
    psi_rtk = wrap_angle(psi_geo + np.pi)

    # 定位 heading 平均
    psi_loc_seg = np.unwrap(psi_loc[s:e])
    psi_loc_mean = np.mean(psi_loc_seg)

    dpsi = wrap_angle(psi_rtk - psi_loc_mean)

    return dpsi

def calibrate_heading_bias_reverse(
        x_rtk,
        y_rtk,
        psi_loc):
    x_r, y_r = resample_by_distance(x_rtk, y_rtk, step=1.0)
    segments = detect_reverse_straight_segments(x_rtk, y_rtk)

    if len(segments) == 0:
        raise RuntimeError("No valid reverse straight segment")

    dpsis = []

    for seg in segments:
        dpsi = heading_bias_reverse_one_segment(
            x_rtk, y_rtk, psi_loc, seg
        )
        dpsis.append(dpsi)

    dpsis = np.array(dpsis)

    # 一致性检查
    if np.ptp(dpsis) > np.deg2rad(0.5):
        print("WARNING: heading bias inconsistent across segments")

    return np.mean(dpsis), segments