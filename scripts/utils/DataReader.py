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
def stand_still_str_format(v):
    if v == "VehSdslSts_0_not_standstill":
        return "Not Standstill"
    elif v == "VehSdslSts_1_standstill":
        return "Standstill"
    else:
        return "Unknown"
class VehicleInfo:
    def __init__(self,rl_wheel_counter:int,rr_wheel_counter:int,rl_wheel_direction:str,rr_wheel_direction:str, velocity:float,steering_wheel_angle:float,gear:str,imu_x_acc:float,imu_y_acc:float,imu_yaw_rate:float,stand_still:str,timestamp:float):
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
        self.stand_still = stand_still_str_format(stand_still)

        def _to_text(v):
            if isinstance(v, (bytes, bytearray)):
                return v.decode('utf-8', errors='ignore')
            if v is None:
                return "Unknown"
            return str(v)

        self.rr_wheel_direction = _to_text(rr_wheel_direction)
        self.rl_wheel_direction = _to_text(rl_wheel_direction)
        self.gear = _to_text(gear)
        self.imu_yaw_rate = imu_yaw_rate
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

    for _, row in data.iterrows():
        x = float(row['x'])
        y = float(row['y'])
        heading = float(row['heading'])
        timestamp = float(row['timestamp'])/1000.0  # 转换为秒
        steering_angle = None
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
    all_channels = set(mdf.channels_db.keys())

    required_signals = [
        'IRDrvnWhlRotlDistPlsCtr',
        'ILDrvnWhlRotlDistPlsCtr',
        'IRDrvnWhlRotlDircn',
        'ILDrvnWhlRotlDircn',
        'IStrgWhlAng',
        'IVehSpdAvg',
        'IVehDynYawRate_SDM',
        'IVSELatAcc_SDM',
        'IVSELongtAcc_SDM',
        'IVehSdslSts'
    ]
    optional_signals = []
    gear_candidates = ['ITrShftLvrPos', 'VCU_ModeGearSts', 'ISCUShiftrLvrPosn']

    missing_required = [s for s in required_signals if s not in all_channels]
    if missing_required:
        raise ValueError(f"MF4缺少必须信号: {missing_required}")

    gear_signal = next((s for s in gear_candidates if s in all_channels), None)
    if gear_signal is None:
        print("⚠ 未找到挡位信号(ITrShftLvrPos/VCU_ModeGearSts/ISCUShiftrLvrPosn)，将使用轮方向推断挡位")
    else:
        print(f"使用挡位信号: {gear_signal}")

    vehicle_signals = required_signals + [s for s in optional_signals if s in all_channels] + ([gear_signal] if gear_signal else [])
    df_vehicle = mdf.select(vehicle_signals)

    index = vehicle_signals.index('IRDrvnWhlRotlDistPlsCtr')
    IRDrvnWhlRotlDistPlsCtr_signal  = df_vehicle[index]
    IRDrvnWhlRotlDistPlsCtr_samples = IRDrvnWhlRotlDistPlsCtr_signal.samples.tolist()
    IRDrvnWhlRotlDistPlsCtr_timestamps = IRDrvnWhlRotlDistPlsCtr_signal.timestamps.tolist()
    index = vehicle_signals.index('ILDrvnWhlRotlDistPlsCtr')
    ILDrvnWhlRotlDistPlsCtr_signal  = df_vehicle[index]
    ILDrvnWhlRotlDistPlsCtr_samples = ILDrvnWhlRotlDistPlsCtr_signal.samples.tolist()

    def get_nearest_signal_value(signal, target_ts,delta_time:float=0.015):
        """
        在timestamps中查找与target_ts时间差<0.01秒的信号，返回其索引，否则返回None
        """
        timestamps = signal.timestamps.tolist()
        for idx, ts in enumerate(timestamps):
            if abs(ts - target_ts) < delta_time:
                return idx
        return None

    def to_text(v):
        if isinstance(v, (bytes, bytearray)):
            return v.decode('utf-8', errors='ignore')
        if v is None:
            return "Unknown"
        return str(v)

    def normalize_direction(v):
        text = to_text(v)
        if "Backward" in text or "_2_" in text:
            return "Backward"
        if "Forward" in text or "_1_" in text:
            return "Forward"
        return "Unknown"

    def normalize_gear(v):
        text = to_text(v)
        up = text.upper()
        if "PARK" in up or up in {"P", "1"}:
            return "P"
        if "REVERSE" in up or up in {"R", "2", "7"}:
            return "R"
        if "NEUTRAL" in up or up in {"N", "3", "6"}:
            return "N"
        if "DRIVE" in up or up in {"D", "4", "5"}:
            return "D"
        return text

    vehicle_info_list = []
    ILDrvnWhlRotlDircn_index =  vehicle_signals.index('ILDrvnWhlRotlDircn')
    IRDrvnWhlRotlDircn_index =  vehicle_signals.index('IRDrvnWhlRotlDircn')
    IStrgWhlAng_index =  vehicle_signals.index('IStrgWhlAng')
    IVehSpdAvg_index =  vehicle_signals.index('IVehSpdAvg')
    IVSELatAcc_SDM_index = vehicle_signals.index('IVSELatAcc_SDM') if 'IVSELatAcc_SDM' in vehicle_signals else None
    IVSELongtAcc_SDM_index = vehicle_signals.index('IVSELongtAcc_SDM') if 'IVSELongtAcc_SDM' in vehicle_signals else None
    IVehDynYawRate_SDM_index = vehicle_signals.index('IVehDynYawRate_SDM') if 'IVehDynYawRate_SDM' in vehicle_signals else None
    gear_signal_index = vehicle_signals.index(gear_signal) if gear_signal else None
    IVehSdslSts_index = vehicle_signals.index('IVehSdslSts') if 'IVehSdslSts' in vehicle_signals else None
    for i, ts in enumerate(IRDrvnWhlRotlDistPlsCtr_timestamps):
        #print(f'processing timestamp:{ts}, index:{i}')
        rl_wheel_counter = ILDrvnWhlRotlDistPlsCtr_samples[i]
        rr_wheel_counter = IRDrvnWhlRotlDistPlsCtr_samples[i]
        rl_wheel_direction = None
        rr_wheel_direction = None
        steering_wheel_angle = None
        vehicle_speed = None
        imu_x_acc = None
        imu_y_acc = None
        gear = None
        stand_still = None

        #print('ABS_RRWheelDirection_index:',ABS_RRWheelDirection_index)
        #print('ABS_RLWheelDirection_index:',ABS_RLWheelDirection_index)
        # 查找最近的信号值
        idx = get_nearest_signal_value(df_vehicle[ILDrvnWhlRotlDircn_index], ts,delta_time = 0.015)
        if idx is not None:
            rl_wheel_direction = df_vehicle[ILDrvnWhlRotlDircn_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[IRDrvnWhlRotlDircn_index], ts,delta_time = 0.015)
        #print('idx for ABS_RRWheelDirection:',idx)
        if idx is not None:
            rr_wheel_direction = df_vehicle[IRDrvnWhlRotlDircn_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[IStrgWhlAng_index], ts)
        if idx is not None:
            steering_wheel_angle = df_vehicle[IStrgWhlAng_index].samples[idx]
        idx = get_nearest_signal_value(df_vehicle[IVehSpdAvg_index], ts)
        if idx is not None:
            vehicle_speed = df_vehicle[IVehSpdAvg_index].samples[idx]
        if IVSELatAcc_SDM_index is not None:
            idx = get_nearest_signal_value(df_vehicle[IVSELatAcc_SDM_index], ts)
            if idx is not None:
                imu_y_acc = df_vehicle[IVSELatAcc_SDM_index].samples[idx]
        if IVSELongtAcc_SDM_index is not None:
            idx = get_nearest_signal_value(df_vehicle[IVSELongtAcc_SDM_index], ts)
            if idx is not None:
                imu_x_acc = df_vehicle[IVSELongtAcc_SDM_index].samples[idx]
        if gear_signal_index is not None:
            idx = get_nearest_signal_value(df_vehicle[gear_signal_index], ts)
            if idx is not None:
                gear = normalize_gear(df_vehicle[gear_signal_index].samples[idx])
        if IVehDynYawRate_SDM_index is not None:
            idx = get_nearest_signal_value(df_vehicle[IVehDynYawRate_SDM_index], ts)
            if idx is not None:
                imu_yaw_rate = df_vehicle[IVehDynYawRate_SDM_index].samples[idx]
            else:
                imu_yaw_rate = None
        if IVehSdslSts_index is not None:
            idx = get_nearest_signal_value(df_vehicle[IVehSdslSts_index], ts)
            if idx is not None:
                stand_still = to_text(df_vehicle[IVehSdslSts_index].samples[idx])

        rr_wheel_direction = normalize_direction(rr_wheel_direction)
        rl_wheel_direction = normalize_direction(rl_wheel_direction)

        if gear is None:
            if rr_wheel_direction == "Backward" or rl_wheel_direction == "Backward":
                gear = "R"
            else:
                gear = "D"

        vehicle_info = VehicleInfo(rl_wheel_counter=rl_wheel_counter,
                                   rr_wheel_counter=rr_wheel_counter,
                                   rl_wheel_direction=rl_wheel_direction,
                                   rr_wheel_direction=rr_wheel_direction,
                                   velocity=vehicle_speed,
                                   steering_wheel_angle=steering_wheel_angle,
                                   gear = gear,
                                   imu_x_acc=imu_x_acc,
                                   imu_y_acc=imu_y_acc,
                                   imu_yaw_rate=imu_yaw_rate,
                                   stand_still=stand_still,
                                   timestamp=ts)
        #print(f"时间: {datetime.fromtimestamp(ts,tz=ZoneInfo('UTC'))}, 速度: {vehicle_info.velocity} m/s, 方向盘转角: {vehicle_info.steering_wheel_angle} rad")
        vehicle_info_list.append(vehicle_info)
    # save vehicle info to csv file
    if save_csv_file is None:
        save_csv_file = "vehicle_data.csv"
    paretn_dir = os.path.dirname(save_csv_file)
    if paretn_dir and not os.path.exists(paretn_dir):
        os.makedirs(paretn_dir)
    with open(save_csv_file,'w', encoding='utf-8', newline='') as f:
        # write header
        f.write("Speed,WheelCounter_RR,WheelCounter_RL,WheelCounter_Dir_RR,WheelCounter_Dir_RL,SteeringWheel_Angle,GearMode,IMU_x_acc,IMU_y_acc,IMU_z_acc,IMU_yaw_rate,StandStill,timestamp\r\n")
        for info in vehicle_info_list:
            f.write(f"{info.velocity},{info.rr_wheel_counter},{info.rl_wheel_counter},{info.rr_wheel_direction},{info.rl_wheel_direction},{info.steering_wheel_angle},{info.gear},{info.imu_x_acc},{info.imu_y_acc},0,{info.imu_yaw_rate},{info.stand_still},{info.timestamp}\r\n")
    return vehicle_info_list
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
    read_mf4_vehicle_data("data/vehicle/001.mf4",
                          "data/vehicle/csv/forward_straight_data.csv")