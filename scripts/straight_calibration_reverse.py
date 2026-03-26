import numpy as np
import json 
from utils.HeadingBiasCalib import calibrate_heading_bias_reverse
import pandas as pd
import os
def run_vehicle_localization(vehicle_data_file:str, vehicle_localization_file:str, config_file:str):
    """
    运行车辆定位脚本，生成车辆定位结果文件
    """
    os.system(f"build/bin/localization_example -i {vehicle_data_path} -o {vehicle_localization_file} -c {config_file}")
if __name__ == "__main__":
    straight_calib_json_file = 'calibration_straight_calib.json'
    with open(straight_calib_json_file, 'r') as f:
        calib_data = json.load(f)
    heading_bias_rad = calib_data['paramters']['heading_bias_rad']
    # run vehicle localization with calib file
    data_dir = "data/Lev01_R19_Stragight__backward_0_5kph_001"
    vehicle_data_file = 'vehicle_data.csv'
    vehicle_localization_file = 'vehicle_localization_for_heading_bias_reverse_calib.csv'
    rtk_ground_truth_file = 'rtk_ground_truth_time_sync.csv'
    vehicle_data_path = os.path.join(data_dir, vehicle_data_file)
    vehicle_localization_path = os.path.join(data_dir, vehicle_localization_file)
    rtk_ground_truth_file = os.path.join(data_dir, rtk_ground_truth_file)
    run_vehicle_localization(vehicle_data_path, vehicle_localization_path, straight_calib_json_file)
    print("Vehicle localization with straight calibration done.")
    # read vehicle localization data
    vehicle_localization_data = pd.read_csv(vehicle_localization_path)
    rtk_ground_truth = pd.read_csv(rtk_ground_truth_file)
    # calibrate heading bias reverse
    psi_loc = vehicle_localization_data['heading'].values
    x_rtk = rtk_ground_truth['x'].values
    y_rtk = rtk_ground_truth['y'].values
    psi_rtk = rtk_ground_truth['yaw'].values
    timestamp = vehicle_localization_data['timestamp'].values
    calib_heading_bias,segments = calibrate_heading_bias_reverse(x_rtk, y_rtk, psi_loc)
    print('Calibrated reverse heading bias (reverse): {:.6f} rad, straight calib value: {:.6f} rad'.format(calib_heading_bias, heading_bias_rad))

