import numpy as np
from scipy.signal import savgol_filter
def wrap_to_pi(angle):
    """ 将角度规范化到 [-pi, pi] """
    return (angle + np.pi) % (2 * np.pi) - np.pi



def calibrate_scale(x_est, y_est, x_rtk, y_rtk):
    # 计算 纵向尺度因子
    ds_est = np.sqrt(np.diff(x_est)**2 + np.diff(y_est)**2)
    ds_rtk = np.sqrt(np.diff(x_rtk)**2 + np.diff(y_rtk)**2)
    scale = np.sum(ds_rtk) / np.sum(ds_est)
    return scale



def calibrate_steer_offset_only_param(steer, heading_rtk, th=1e-4):
    # 计算方向盘零偏（仅在直线段）
    dpsi = np.abs(np.diff(heading_rtk))
    straight_mask = dpsi < th
    steer_offset = np.mean(steer[:-1][straight_mask])
    return steer_offset 

def detect_straight_segments(steer,heading,x,y,timestamp,steer_th=np.deg2rad(1.0),yaw_rate_th=np.deg2rad(0.5),curvature_th=0.003,min_duration=3.0):
    def compute_curvature(x,y,heading,win=15):
        """计算曲率"""
        kappa = np.zeros(len(x)-1)
        for i in range(win, len(x)-win):
            dx = x[i-win:i+win] - x[i]
            dy = y[i-win:i+win] - y[i]
            ds = np.sqrt(dx**2 + dy**2)
            dpsi = heading[i-win:i+win] - heading[i]
            kappa[i] = np.mean(dpsi/(ds+1e-3))
        return kappa
    x = savgol_filter(x, 11, 3)
    y = savgol_filter(y, 11, 3)
    heading = savgol_filter(heading, 21, 3)
    dt = np.diff(timestamp)
    yaw_rate = np.diff(heading) / dt
    curvature = compute_curvature(x, y, heading)
    print(f'curvature:{curvature}')
    mask = (
            (np.abs(steer[:-1])<steer_th) &
            (np.abs(yaw_rate) < yaw_rate_th) )
            #(np.abs(curvature) < curvature_th))
    # 连续时间筛选
    straight_segments = []
    start = None
    for i,m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            duration = timestamp[i] - timestamp[start]
            if duration >= min_duration:
                straight_segments.append((start, i))
            start = None
    if start is not None:
        duration = timestamp[len(mask)] - timestamp[start]
        if duration >= min_duration:
            straight_segments.append((start, len(mask)))       
    return straight_segments




def calibrate_steer_offset(steer_esi,heading_rtk,x_rtk,y_rtk,timestamp):
    # 检测直线段
    straight_segments = detect_straight_segments(steer_esi,heading_rtk,x_rtk,y_rtk,timestamp)
    print(f'straight_segments:{straight_segments}')
    steer_offsets = []
    for (start,end) in straight_segments:
        steer_offset = np.mean(steer_esi[start:end])
        steer_offsets.append(steer_offset)
    if len(steer_offsets) == 0:
        return None
    steer_offset_results = []
    weights = []
    for start,end in straight_segments:
        steer = steer_esi[start:end]
        heading = heading_rtk[start:end]
        calib_steer_offset = calibrate_steer_offset_only_param(steer,heading,th=1e-4)
        steer_offset_results.append(calib_steer_offset)
        std = np.std(steer)
        weights.append(1.0/(std+1e-6))
    # 加权平均
    if len(steer_offset_results) == 0:
        print('No valid steer offset results found.')
        return None
    steer_offset_results = np.array(steer_offset_results,dtype=float)
    weights = np.array(weights,dtype=float)
    weighted_average = np.average(steer_offset_results, weights=weights)
    return weighted_average