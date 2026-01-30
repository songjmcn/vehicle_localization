import pandas as pd
import numpy as np
import os
from scipy.interpolate import interp1d
def generate_rtk_ground_truth(
    rtk_df: pd.DataFrame,
    time_offset:float,
    ref_timestamps=None,
    heading_unit='rad',   # 'deg' or 'rad'
    sample_rate=100.0
):
    """
    rtk_df columns:
        ['timestamp', 'x', 'y', 'heading']
    """

    rtk = rtk_df.copy()

    # ---------- Step 1: 时间修正 ----------
    rtk['t_sync'] = rtk['timestamp'] + time_offset

    # ---------- Step 2: heading -> rad ----------
    if heading_unit == 'deg':
        yaw = np.deg2rad(rtk['heading'].values)
    else:
        yaw = rtk['heading'].values.copy()

    # ---------- Step 3: heading 连续化 ----------
    yaw = np.unwrap(yaw)

    # ---------- Step 4: 计算速度 ----------
    dt = np.diff(rtk['t_sync'].values)
    dx = np.diff(rtk['x'].values)
    dy = np.diff(rtk['y'].values)

    v = np.zeros(len(rtk))
    v[1:] = np.sqrt(dx**2 + dy**2) / dt

    # ---------- Step 5: yaw_rate ----------
    yaw_rate = np.zeros(len(rtk))
    yaw_rate[1:] = np.diff(yaw) / dt

    # ---------- Step 6: yaw 归一化 ----------
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

    # ---------- Step 7: 重采样（可选） ----------
    if ref_timestamps is not None:
        def interp(sig):
            f = interp1d(
                rtk['t_sync'],
                sig,
                kind='linear',
                fill_value='extrapolate'
            )
            return f(ref_timestamps)

        gt = pd.DataFrame({
            't': ref_timestamps,
            'x': interp(rtk['x'].values),
            'y': interp(rtk['y'].values),
            'yaw': interp(yaw),
            'v': interp(v),
            'yaw_rate': interp(yaw_rate),
        })
    else:
        gt = pd.DataFrame({
            't': rtk['t_sync'],
            'x': rtk['x'],
            'y': rtk['y'],
            'yaw': yaw,
            'v': v,
            'yaw_rate': yaw_rate,
        })

    return gt


if __name__ == "__main__":
    vehicle_localization_file = 'vehicle_localization.csv'
    rtk_trajectory_file = 'rtk_trajectory.csv'
    data_dir = '/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_20kph_001'
    vehicle_localization_file = os.path.join(data_dir, vehicle_localization_file)
    rtk_trajectory_file = os.path.join(data_dir, rtk_trajectory_file)
    vehicle_localization_data = pd.read_csv(vehicle_localization_file, sep=',')
    rtk_trajectory_data = pd.read_csv(rtk_trajectory_file, sep=',')
    rtk_trajectory_data = pd.DataFrame({
        'timestamp': rtk_trajectory_data['timestamp'].values,
        'x': rtk_trajectory_data['x'].values,
        'y': rtk_trajectory_data['y'].values,
        'heading': rtk_trajectory_data['heading_rad'].values
    })
    vehicle_localization_timestamp = vehicle_localization_data['timestamp'].values
    time_offset = -0.076
    rtk_ground_truth = generate_rtk_ground_truth(
        rtk_trajectory_data,
        time_offset=time_offset,
        ref_timestamps=vehicle_localization_timestamp,
        heading_unit='rad'
    )
    output_file = os.path.join(data_dir, 'rtk_ground_truth_time_sync.csv')
    rtk_ground_truth.to_csv(output_file, index=False,sep=',')
    print(f"Generated time-synchronized RTK ground truth saved to: {output_file}") 