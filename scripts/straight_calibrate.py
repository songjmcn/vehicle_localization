
import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from utils.SteerOffsetCalib import calibrate_steer_offset
from utils.HeadingBiasCalib import calibrate_heading_bias
from utils.WheelScaleCalib import calibrate_wheel_scale
import json

def run_vehicle_localization(vehicle_data_file:str, vehicle_localization_file:str, config_file:str):
    """
    运行车辆定位脚本，生成车辆定位结果文件
    """
    os.system(f"build/bin/localization_example -i {vehicle_data_path} -o {vehicle_localization_file} -c {config_file}")
if __name__ == "__main__":
    print("Straight Straight calibration tool")
    print("Start calibrating steer offset...")
    data_dir = "data/Lev01_R19_Stragight__forward_0_5kph_001"
    vehicle_data_file = 'vehicle_data.csv'
    vehicle_localization_file = 'vehicle_localization_for_steer_offset_calib.csv'
    rtk_data_file = 'rtk_data.csv'
    rtk_ground_truth_file = 'rtk_ground_truth_time_sync.csv'
    calibration_json_file = 'calibration.json'
    vehicle_data_path = os.path.join(data_dir, vehicle_data_file)
    vehicle_localization_path = os.path.join(data_dir, vehicle_localization_file)
    rtk_data_path = os.path.join(data_dir, rtk_data_file)
    rtk_ground_truth_path = os.path.join(data_dir, rtk_ground_truth_file)
    calibration_json_file = os.path.join(data_dir, calibration_json_file)
    time_offset = -0.076
    #if not os.path.exists(vehicle_localization_path):
    run_vehicle_localization(vehicle_data_path, vehicle_localization_path, calibration_json_file)
    vehicle_localization_data = pd.read_csv(vehicle_localization_path)    
    if not os.path.exists(rtk_ground_truth_path):
        from generate_rtk_trajectory_time_sync import generate_rtk_ground_truth
        rtk_data = pd.read_csv(rtk_data_path)
        rtk_trajectory_data = pd.DataFrame({
        'timestamp': rtk_data['timestamp'].values,
        'x': rtk_data['x'].values,
        'y': rtk_data['y'].values,
        'heading': rtk_data['heading_rad'].values})
        timestamp = vehicle_localization_data['timestamp'].values
        rtk_ground_truth = generate_rtk_ground_truth(rtk_df=rtk_trajectory_data, time_offset=time_offset, timestamp=timestamp)
        rtk_ground_truth.to_csv(rtk_ground_truth_path, index=False,sep=',')
    else:
        rtk_ground_truth = pd.read_csv(rtk_ground_truth_path)
    steer = vehicle_localization_data['steering_angle'].values
    heading_rtk = rtk_ground_truth['yaw'].values
    x_rtk = rtk_ground_truth['x'].values
    y_rtk = rtk_ground_truth['y'].values
    heading_loc = vehicle_localization_data['heading'].values
    timestamp = vehicle_localization_data['timestamp'].values
    calib_steer_offset = calibrate_steer_offset(steer, heading_rtk, x_rtk, y_rtk, timestamp)

    # calibrate heading bias
    print(f"Calibrated done, steer offset: {calib_steer_offset} rad, {np.rad2deg(calib_steer_offset)} deg")
    print('Start calibrating heading bias...')
    # create new calibration json file
    calibration_param = {
        'metadata':{
            'version':1,
            'timestamp': "2026-01-22T18:15:00Z",
            'tool': "line_calibrator_script"
        },
        'paramters':{
            'wheel_calibration_factor': 1.0,
            'steering_angle_offset_rad': calib_steer_offset,
            'steering_angle_offset_deg': np.rad2deg(calib_steer_offset),
            'heading_bias_rad':0.0,
            'steering_angle_gain': 1.0,
            'ackermann_factor': 0.0,
            'imu_accel_bias_x_mss': 0.0,
            'imu_accel_bias_y_mss': 0.0,
            'imu_gyro_bias_z_rads': 0.0,
            'imu_accel_scale_x': 1.0,
            'imu_accel_scale_y': 1.0
        }
    }
    calibration_json_file = 'calibration_steer_offset.json'
    calibration_json_file = os.path.join(data_dir, calibration_json_file)
    vehicle_localization_file = 'vehicle_localization_for_heading_bias_calib.csv'
    vehicle_localization_file = os.path.join(data_dir, vehicle_localization_file)
    with open(calibration_json_file,'w') as f:
        json.dump(calibration_param,f,indent=4) 
    # run localization exe with new calibration  file
    run_vehicle_localization(vehicle_data_path, vehicle_localization_file, calibration_json_file)
    vehicle_localization_data = pd.read_csv(vehicle_localization_file)
    yaw_rate_loc = vehicle_localization_data['yaw_rate'].values
    heading_bias, dpsis, segments= calibrate_heading_bias(x_rtk, y_rtk, yaw_rate_loc,step=1.0)
    print(f"Calibrated done, heading bias: {heading_bias} rad")
    print(f'calibrate heading bias, the dpsis:{dpsis}')

    # start calibrate wheel scale
    # create new calibration json file
    calibration_param = {
        'metadata':{
            'version':1,
            'timestamp': "2026-01-22T18:15:00Z",
            'tool': "line_calibrator_script"
        },
        'paramters':{
            'wheel_calibration_factor': 1.0,
            'steering_angle_offset_rad': calib_steer_offset,
            'steering_angle_offset_deg': np.rad2deg(calib_steer_offset),
            'heading_bias_rad':heading_bias,
            'steering_angle_gain': 1.0,
            'ackermann_factor': 0.0,
            'imu_accel_bias_x_mss': 0.0,
            'imu_accel_bias_y_mss': 0.0,
            'imu_gyro_bias_z_rads': 0.0,
            'imu_accel_scale_x': 1.0,
            'imu_accel_scale_y': 1.0
        }
    }
    calibration_json_file = 'calibration_heading_bias.json'
    calibration_json_file = os.path.join(data_dir, calibration_json_file)
    vehicle_localization_file = 'vehicle_localization_for_wheel_scale_calib.csv'
    vehicle_localization_file = os.path.join(data_dir, vehicle_localization_file)
    with open(calibration_json_file,'w') as f:
        json.dump(calibration_param,f,indent=4) 
    # run localization exe with new calibration  file
    run_vehicle_localization(vehicle_data_path, vehicle_localization_file, calibration_json_file)
    vehicle_localization_data = pd.read_csv(vehicle_localization_file)
    heading_rtk = rtk_ground_truth['yaw'].values
    x_rtk = rtk_ground_truth['x'].values
    y_rtk = rtk_ground_truth['y'].values
    heading_loc = vehicle_localization_data['heading'].values
    x_loc = vehicle_localization_data['x'].values
    y_loc = vehicle_localization_data['y'].values
    steer_loc = vehicle_localization_data['steering_angle'].values   
    timestamp = vehicle_localization_data['timestamp'].values
    wheel_scale,scales = calibrate_wheel_scale(
        heading_rtk,
        x_rtk,
        y_rtk,
        heading_loc,
        x_loc,
        y_loc,
        steer_loc,
        timestamp
    )
    print(f"Calibrated done, wheel scale: {wheel_scale} ")
    calibration_json_file = 'calibration_straight_calib.json'
    calibration_json_file = os.path.join(data_dir, calibration_json_file)
    with open(calibration_json_file,'w') as f:
        calibration_param['paramters']['wheel_calibration_factor'] = wheel_scale
        json.dump(calibration_param,f,indent=4) 
    print("All calibration done.")

    #run final localization with all calibration
    vehicle_localization_file = 'vehicle_localization_straight_calib.csv'
    vehicle_localization_file = os.path.join(data_dir, vehicle_localization_file)
    run_vehicle_localization(vehicle_data_path, vehicle_localization_file, calibration_json_file)
    print("Final localization with all calibration done.")