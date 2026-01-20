import numpy as np
import pandas as pd
import os
def wrap_to_pi(angle):
    """ 将角度规范化到 [-pi, pi] """
    return (angle + np.pi) % (2 * np.pi) - np.pi
class LocalizationLineCalibrator:
    def __init__(self, dt=0.02):
        self.dt = dt  # 采样周期，例如 50Hz -> 0.02s
    @staticmethod    
    def calibrate_heading_bias(heading_est,heading_rtk):
        # 计算航向偏置
        return np.mean(wrap_to_pi(heading_rtk - heading_est))
    
    @staticmethod
    def calibrate_scale(x_est, y_est, x_rtk, y_rtk):
        # 计算 纵向尺度因子
        ds_est = np.sqrt(np.diff(x_est)**2 + np.diff(y_est)**2)
        ds_rtk = np.sqrt(np.diff(x_rtk)**2 + np.diff(y_rtk)**2)
        scale = np.sum(ds_rtk) / np.sum(ds_est)
        return scale
    @staticmethod
    def calibrate_acc_bias(ax,ay,static_mask):
        # 计算 IMU 零偏
        bax = np.mean(ax[static_mask])
        bay = np.mean(ay[static_mask])
        return bax, bay
    @staticmethod
    def calibrate_steer_offset(steer, heading_rtk, th=1e-4):
        # 计算方向盘零偏（仅在直线段）
        dpsi = np.abs(np.diff(heading_rtk))
        straight_mask = dpsi < th
        steer_offset = np.mean(steer[:-1][straight_mask])
        return steer_offset 

    def calibrate(self, rtk_data, algo_data, imu_data,vehicle_info):
        """
        rtk_data: {'x', 'y', 'yaw'} - RTK 真值
        algo_data: {'x', 'y', 'yaw'} - 定位算法输出值
        imu_data: {'ax','ay'} - IMU 纵向加速度
        vehicle_info: {'steer'} - 方向盘/前轮转角(用于筛选直线)
        """
        # 转为 numpy
        rtk_x, rtk_y, rtk_yaw = np.array(rtk_data['x']), np.array(rtk_data['y']), np.array(rtk_data['yaw'])
        est_x, est_y, est_yaw = np.array(algo_data['x']), np.array(algo_data['y']), np.array(algo_data['yaw'])
        imu_ax,imu_ay = np.array(imu_data['ax']),np.array(imu_data['ay'])
        steer = np.array(vehicle_info['steer'])

        # 1. 筛选直线段 (steer 接近 0 且位移大于一定阈值防止原地漂移影响)
        # straight_mask = np.abs(steer) < 0.02 # 弧度，约 1.1 度
        
        # # 2. 计算航向偏差 (Yaw Offset)
        # #yaw_diffs = wrap_to_pi(r_yaw - a_yaw)
        # #yaw_offset = np.mean(yaw_diffs[straight_mask])
        # dpsi = np.abs(np.diff(r_yaw))
        # steer_mask = dpsi < 1e-4
        # yaw_offset = np.mean(steer[:-1][steer_mask])  

        # # 3. 计算综合比例因子 (Scale Factor)
        # # 计算 RTK 步进距离
        # dr_x = np.diff(r_x)
        # dr_y = np.diff(r_y)
        # dist_rtk_steps = np.sqrt(dr_x**2 + dr_y**2)
        
        # # 计算 算法 步进��离
        # da_x = np.diff(a_x)
        # da_y = np.diff(a_y)
        # dist_algo_steps = np.sqrt(da_x**2 + da_y**2)

        # # 只取直线行驶部分的距离进行累加
        # mask_steps = straight_mask[:-1] # diff 之后长度减 1
        # total_dist_rtk = np.sum(dist_rtk_steps[mask_steps])
        # total_dist_algo = np.sum(dist_algo_steps[mask_steps])
        
        # system_scale_factor = total_dist_rtk / total_dist_algo if total_dist_algo > 0.1 else 1.0

        # # 4. 计算 IMU 加速度零偏 (ax Bias)
        # # 从 RTK 距离计算真值速度和加速度
        # v_rtk = dist_rtk_steps / self.dt
        # acc_rtk = np.diff(v_rtk) / self.dt
        
        # # 对齐 IMU 长度
        # imu_ax_sync = imu_ax[1:-1]
        # mask_acc = straight_mask[1:-1]
        
        # ax_bias = np.mean(imu_ax_sync[mask_acc] - acc_rtk[mask_acc])

        # return {
        #     "yaw_offset_rad": yaw_offset,
        #     "scale_factor": system_scale_factor,
        #     "imu_ax_bias": ax_bias
        # }


        # 1️⃣ 航向角偏置标定
        dpsi = LocalizationLineCalibrator.calibrate_heading_bias(est_yaw,rtk_yaw)
        # 2️⃣ 纵向尺度因子标定
        scale = LocalizationLineCalibrator.calibrate_scale(est_x,est_y,rtk_x,rtk_y)
        # 计算 heading 角变化率
        steer_mask = np.abs(steer) < 0.02
        # 3️⃣ IMU bias
        bax,bay = LocalizationLineCalibrator.calibrate_acc_bias(imu_ax,imu_ay,steer_mask)
        # 4️⃣ 方向盘零偏
        steer_offset = LocalizationLineCalibrator.calibrate_steer_offset(steer,rtk_yaw)
        # 5️⃣ 残差估计(不用于标定)
        heading_res = wrap_to_pi(rtk_yaw-(est_yaw+dpsi))
        pos_res = np.sqrt((est_x*scale - rtk_x)**2 + (est_y*scale - rtk_y)**2)
        return {
        "heading_bias_rad": dpsi,
        "scale_s": scale,
        "imu_bias_ax": bax,
        "imu_bias_ay": bay,
        "steer_offset": steer_offset,
        "res_heading_std": np.std(heading_res),
        "res_pos_rmse": np.sqrt(np.mean(pos_res**2))
    }
if __name__ == "__main__":
    # 测试用法
    vehicle_localizatin_file = 'vehicle_localization.csv'
    rtk_ground_truth_file = 'rtk_ground_truth_time_sync.csv'
    data_dir = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_5kph_002"
    vehicle_localizatin_file = os.path.join(data_dir, vehicle_localizatin_file)
    rtk_ground_truth_file = os.path.join(data_dir, rtk_ground_truth_file)
    vehicle_localization_data = pd.read_csv(vehicle_localizatin_file)
    rtk_data = pd.read_csv(rtk_ground_truth_file)
    localization_data = {'x': vehicle_localization_data['x'].values,
                                 'y': vehicle_localization_data['y'].values,
                                 'yaw': vehicle_localization_data['heading'].values}
    vehicle_data = {'steer': vehicle_localization_data['steering_angle'].values,}
    imu_data = {'ax':vehicle_localization_data['imu_acce_x'].values,'ay':vehicle_localization_data['imu_acce_y'].values}
    rtk_data = {'x':rtk_data['x'].values,
                        'y':rtk_data['y'].values,
                        'yaw':rtk_data['yaw'].values}
    
    calibrator = LocalizationLineCalibrator(dt=0.02)
                        
    res = calibrator.calibrate(rtk_data, localization_data, imu_data, vehicle_data)
    print(f"标定结果: {res}")