import pandas as pd
from datetime import datetime
from zoneinfo import ZoneInfo
from asammdf import MDF
import os
class LocalizationInfo:
    def __init__(self,x:float,y:float,heading:float,timestamp:float,steering_angle:float=None,steering_wheel_angle:float=None):
        '''
        初始化LocalizationInfo对象
        :param x: 8855坐标系的x坐标，为车辆行进方向
        :type x: float
        :param y: 8855坐标系的y坐标，垂直于车辆行进方向，左正右负
        :type y: float
        :param heading: 车辆航向角，单位为度，0度为正北方向， 逆时针方向，范围为[0,2π)
        :type heading: float
        :param timestamp: 定位时间戳，单位为秒
        :type timestamp: float
        :param steering_angle: 前轮转角，弧度，正值为左转，负值为右转
        :type steering_angle: float
        :param steering_wheel_angle: 方向盘转角，弧度，正值为左转，负值为右转
        :type steering_wheel_angle: float
        '''
        self.x = x
        self.y = y
        self.heading = heading
        self.timestamp = timestamp
        if steering_angle is not None:
            self.steering_angle = steering_angle
        if steering_wheel_angle is not None:
            self.steering_wheel_angle = steering_wheel_angle

class VehicleInfo:
    def __init__(self,rl_wheel_counter:int,rr_wheel_counter:int,rl_wheel_direction:str,rr_wheel_direction:str, velocity:float,steering_wheel_angle:float,gear:str,imu_x_acc:float,imu_y_acc:float,timestamp:float):
        '''
        初始化VehicleInfo对象
        :param rl_wheel_counter: 左后轮轮速计计数值
        :type rl_wheel_counter: int
        :param rr_wheel_counter: 右后轮轮速计计数值
        :type rr_wheel_counter: int
        :param rl_wheel_direction: 左后轮运动方向
        :type rl_wheel_direction: str
        :param rr_wheel_direction: 右后轮运动方向
        :type rr_wheel_direction: str
        :param velocity: 车速, 单位:m/s
        :type velocity: float
        :param steering_wheel_angle: 说明 方向盘转角, 单位角度
        :type steering_wheel_angle: float 
        :param gear: 档位
        :type gear: str
        :param imu_x_acc: IMU x轴加速度 单位:m/s²
        :type imu_x_acc: float
        :param imu_y_acc: IMU y轴加速度 单位:m/s²
        :type imu_y_acc: float
        :param timestamp: 时间戳 单位:秒
        :type timestamp: float
        '''
        self.velocity = velocity
        self.steering_wheel_angle = steering_wheel_angle
        self.imu_x_acc = imu_x_acc
        self.imu_y_acc = imu_y_acc
        self.rl_wheel_counter = rl_wheel_counter
        self.rr_wheel_counter = rr_wheel_counter
        self.rr_wheel_direction = rr_wheel_direction.decode('utf-8')
        self.rl_wheel_direction = rl_wheel_direction.decode('utf-8')
        self.gear = gear.decode('utf-8')
        self.timestamp = timestamp
def read_localization_data_from_csv(csv_file:str):
    if not csv_file:
        print("❌ 错误：未提供CSV文件路径")
        return None
    localization_data = []
    data = pd.read_csv(csv_file, sep=';')
    cols = [c.strip() for c in data.columns]
    data.columns = cols
    # 检测 steering 列名（支持 'steering_angle' 或 'steering angle'）
    if 'steering_angle' in data.columns:
        steering_col = 'steering_angle'
    elif 'steering angle' in data.columns:
        steering_col = 'steering angle'
    else:
        steering_col = None
    # 检查必须列
    for req in ['x', 'y', 'heading', 'timestamp']:
        if req not in data.columns:
            print(f"❌ 错误：CSV 缺少列 {req}")
            return None

    for index, row in data.iterrows():
        x = float(row['x'])
        y = float(row['y'])
        heading = float(row['heading'])
        timestamp = float(row['timestamp'])/1000.0  # 转换为秒
        steering_angle = float(row['steering_angle'])
        if steering_col is not None and pd.notna(row.get(steering_col)):
            try:
                steering_angle = float(row[steering_col])
            except Exception:
                steering_angle = None
        info = LocalizationInfo(x=x, y=y, heading=heading, timestamp=timestamp, steering_angle=steering_angle)
        localization_data.append(info)
    return localization_data
def read_mf4_vehicle_data(mf4_file:str,save_csv_file:str=None):
    """
    读取mf4文件中的vehicle_localization数据
    Args:
        mf4_file: mf4文件路径
    Returns:
        localization_data: LocalizationInfo对象列表
    """
    mdf = MDF(mf4_file)
    # vehicle_signal_mapping = {"ABS_RRWheelSpeedRC":"WheelCounter_RR",
    #                           "ABS_RLWheelSpeedRC":"WheelCounter_RL",
    #                           "ABS_RRWheelDirection":"WheelCounter_Dir_RR",
    #                           "ABS_RLWheelDirection":"WheelCounter_Dir_RL",
    #                           "SAS_SteerWheelAngle":"SteeringWheel_Angle",
    #                           "VCU_ModeGearSts  ":"GearMode",
    #                           "ESC_LongitudeACC":"IMU_y_acc",
    #                           "ESC_LateralACC":"IMU_x_acc",
    #                           "ABS_VehicleSpeed":"VehicleSpeed"}
    vehicle_signals = ['ABS_RRWheelSpeedRC','ABS_RLWheelSpeedRC','ABS_RRWheelDirection','ABS_RLWheelDirection','SAS_SteerWheelAngle',
                       'VCU_ModeGearSts','ESC_LongitudeACC','ESC_LateralACC','ABS_VehicleSpeed']
    df_vehicle = mdf.select(vehicle_signals)
    '''
    以ABS_RRWheelSpeedRC和ABS_RLWheelSpeedRC时间戳为基准
    '''
    index = vehicle_signals.index('ABS_RRWheelSpeedRC')
    ABS_RRWheelSpeedRC_signal  = df_vehicle[index]
    ABS_RRWheelSpeedRC_samples = ABS_RRWheelSpeedRC_signal.samples.tolist()
    ABS_RRWheelSpeedRC_timestamps = ABS_RRWheelSpeedRC_signal.timestamps.tolist()
    index = vehicle_signals.index('ABS_RLWheelSpeedRC')
    ABS_RLWHEELSpeedRC_signal  = df_vehicle[index]
    ABS_RLWHEELSpeedRC_samples = ABS_RLWHEELSpeedRC_signal.samples.tolist()    
    def get_nearest_signal_value(signal, target_ts,delta_time:float=0.015):
        """
        在timestamps中查找与target_ts时间差<0.01秒的信号，返回其索引，否则返回None
        """
        timestamps = signal.timestamps.tolist()
        for idx, ts in enumerate(timestamps):
            if abs(ts - target_ts) < delta_time:
                return idx
        return None
    vehicle_info_list = []
    ABS_RLWheelDirection_index =  vehicle_signals.index('ABS_RLWheelDirection')
    ABS_RRWheelDirection_index =  vehicle_signals.index('ABS_RRWheelDirection')
    SAS_SteerWheelAngle_index =  vehicle_signals.index('SAS_SteerWheelAngle')
    ABS_VehicleSpeed_index =  vehicle_signals.index('ABS_VehicleSpeed')
    ESC_LateralACC_index =  vehicle_signals.index('ESC_LateralACC')
    ESC_LongitudeACC_index =  vehicle_signals.index('ESC_LongitudeACC')
    VCU_ModeGearSts_index =  vehicle_signals.index('VCU_ModeGearSts')
    for i, ts in enumerate(ABS_RRWheelSpeedRC_timestamps):
        #print(f'processing timestamp:{ts}, index:{i}')
        rl_wheel_counter = ABS_RLWHEELSpeedRC_samples[i]
        rr_wheel_counter = ABS_RRWheelSpeedRC_samples[i]
        rl_wheel_direction = None
        rr_wheel_direction = None
        steering_wheel_angle = None
        vehicle_speed = None
        imu_x_acc = None
        imu_y_acc = None

        #print('ABS_RRWheelDirection_index:',ABS_RRWheelDirection_index)
        #print('ABS_RLWheelDirection_index:',ABS_RLWheelDirection_index)
        # 查找最近的信号值
        idx = get_nearest_signal_value(df_vehicle[ABS_RRWheelDirection_index], ts,delta_time = 0.015)
        if idx is not None:
            rl_wheel_direction = df_vehicle[ABS_RLWheelDirection_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[ABS_RRWheelDirection_index], ts,delta_time = 0.015)
        #print('idx for ABS_RRWheelDirection:',idx)
        if idx is not None:
            rr_wheel_direction = df_vehicle[ABS_RRWheelDirection_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[SAS_SteerWheelAngle_index], ts)
        if idx is not None:
            steering_wheel_angle = df_vehicle[SAS_SteerWheelAngle_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[ABS_VehicleSpeed_index], ts)
        if idx is not None:
            vehicle_speed = df_vehicle[ABS_VehicleSpeed_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[ESC_LateralACC_index], ts)
        if idx is not None:
            imu_y_acc = df_vehicle[ESC_LateralACC_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[ESC_LongitudeACC_index], ts)
        if idx is not None:
            imu_x_acc = df_vehicle[ESC_LongitudeACC_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[VCU_ModeGearSts_index], ts)
        if idx is not None:
            gear = df_vehicle[VCU_ModeGearSts_index].samples[idx]
        vehicle_info = VehicleInfo(rl_wheel_counter=rl_wheel_counter,
                                   rr_wheel_counter=rr_wheel_counter,
                                   rl_wheel_direction=rl_wheel_direction,
                                   rr_wheel_direction=rr_wheel_direction,
                                   velocity=vehicle_speed,
                                   steering_wheel_angle=steering_wheel_angle,
                                   gear = gear,
                                   imu_x_acc=imu_x_acc,
                                   imu_y_acc=imu_y_acc,
                                   timestamp=ts)
        #print(f"时间: {datetime.fromtimestamp(ts,tz=ZoneInfo('UTC'))}, 速度: {vehicle_info.velocity} m/s, 方向盘转角: {vehicle_info.steering_wheel_angle} rad")
        vehicle_info_list.append(vehicle_info)
    # save vehicle info to csv file
    if save_csv_file is None:
        save_csv_file = "vehicle_data.csv"
    paretn_dir = os.path.dirname(save_csv_file)
    if not os.path.exists(paretn_dir):
        os.makedirs(paretn_dir)
    with open(save_csv_file,'w') as f:
        # write header
        f.write("Speed,WheelCounter_RR,WheelCounter_RL,WheelCounter_Dir_RR,WheelCounter_Dir_RL,SteeringWheel_Angle,GearMode,IMU_x_acc,IMU_y_acc,IMU_z_acc,timestamp\r\n")
        for info in vehicle_info_list:
            f.write(f"{info.velocity},{info.rr_wheel_counter},{info.rl_wheel_counter},{info.rr_wheel_direction},{info.rl_wheel_direction},{info.steering_wheel_angle},{info.gear},{info.imu_x_acc},{info.imu_y_acc},0,{info.timestamp}\r\n")
if __name__=="__main__":
    #csv_file = "/Users/alex/workspace/code_review/vehicle_localization/vehicle_localization.csv"
    #localization_data = read_localization_data_from_csv(csv_file)
    #n = 5 if len(localization_data) > 5 else len(localization_data)
    #for info in localization_data:
    #    dt_object = datetime.fromtimestamp(info.timestamp,tz=ZoneInfo("UTC"))
    #    print(f"时间: {dt_object}, X: {info.x}, Y: {info.y}, Heading: {info.heading}, Steering Angle: {info.steering_angle}")
    
    '''
    test load vehicle data
    '''    
    read_mf4_vehicle_data("/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stype_backward_5kph_001.mf4",
                          "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stype_backward_5kph_001/vehicle_data.csv")