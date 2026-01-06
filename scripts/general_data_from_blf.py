import can
import cantools
from plot_rtk import RTKTrajectoryPlotter
class Filter:
    def __init__(self,channel:int=0,filter_id:list=[]):
        self.channel = channel
        self.filter_id = filter_id
def load_can_data(blf_file:str,filters: list[Filter]=None):
    # Load the DBC file
    #db = cantools.database.load_file(dbc_file)
    blf_log = can.BLFReader(blf_file)
    filterd_can_msgs = []
    for msg in blf_log:
        print("Message ID: 0x{:X}, Channel:{}, Data: {}, Timestamp: {}".format(msg.arbitration_id, msg.channel, msg.data.hex(), msg.timestamp))
        if filters is not None and len(filters)>0:
            for f_id in filters:
                if msg.channel == f_id.channel and msg.arbitration_id in f_id.filter_id:
                    filterd_can_msgs.append(msg)
        else:
            filterd_can_msgs.append(msg)
    return filterd_can_msgs
def dbc_decode_message(dbc_path:str, messages:list[can.Message]):
    dbc = cantools.db.load_file(dbc_path)
    decoded_messages = {}
    for msg in messages:
            decoded_message = dbc.decode_message(msg.arbitration_id, msg.data)
            timestamp = msg.timestamp
            decoded_message['timestamp'] = timestamp
            # decoded_messages.append((msg.arbitration_id, decoded_message))
            if not msg.arbitration_id in decoded_messages:
                decoded_messages[msg.arbitration_id] = []
            decoded_messages[msg.arbitration_id].append(decoded_message)
    return decoded_messages
def get_signal_sync_time(signals:list, time:float):
    result =signals[0]
    for sig in signals:
        if sig['timestamp'] > time:
            break
        time_diff_current = time - sig['timestamp']
        time_diff_last = time- result['timestamp']
        if time_diff_current < time_diff_last:
            result = sig
    return result    
if __name__ == "__main__":
    blf_file = "/Users/alex/Downloads/LEV-RTK+Vehicle_Data/cercle.blf"
    vehicle_dbc = "dbc/LEV-01/LEV01_ADAS_V2.4_20240226.dbc"
    rtk_dbc = "dbc/RTK/CAN_Protocol 2.0_20210507-motorola.dbc"
    vehicle_signal_mapping = {"ABS_RRWheelSpeedRC":"WheelCounter_RR",
                              "ABS_RLWheelSpeedRC":"WheelCounter_RL",
                              "ABS_RRWheelDirection":"WheelCounter_Dir_RR",
                              "ABS_RLWheelDirection":"WheelCounter_Dir_RL",
                              "SAS_SteerWheelAngle":"SteeringWheel_Angle",
                              "VCU_ModeGearSts  ":"GearMode",
                              "ESC_LongitudeACC":"IMU_y_acc",
                              "ESC_LateralACC":"IMU_x_acc",}
    #filter_id = [0x123, 0x456]  # Example filter IDs
    filters = []
    filters.append(Filter(0,[0x123,0x124,0x17E,0x162,0x3E2,0x125]))
    filters.append(Filter(1,[0x32A,0x32D,0x32E]))
    can_data = load_can_data(blf_file, filters=filters)
    vehcle_can = [msg for msg in can_data if msg.channel == 0]
    print("vehicle can len:{}".format(len(vehcle_can)))
    rtk_can = [msg for msg in can_data if msg.channel == 1]
    vehicle_msgs = dbc_decode_message(vehicle_dbc, vehcle_can)
    csv_f = open("vehicle_data.csv","w")
    # Write CSV header
    wheel_counter_msg = vehicle_msgs[0x124]
    csv_f.write("Speed;WheelCounter_RR;WheelCounter_RL;WheelCounter_Dir_RR;WheelCounter_Dir_RL;SteeringWheel_Angle;GearMode;IMU_y_acc;IMU_x_acc;IMU_z_acc;timestamp\r\n")
    for msg in wheel_counter_msg:
        vehicle_wheel_direction_msg = vehicle_msgs[0x123]
        time = msg['timestamp']
        signal = get_signal_sync_time(vehicle_wheel_direction_msg, time)
        write_msg = {}
        print("siangal:{}".format(signal))
        write_msg["ABS_RRWheelDirection"] = signal["ABS_RRWheelDirection"]
        write_msg["ABS_RLWheelDirection"] = signal["ABS_RLWheelDirection"]
        write_msg["ABS_RRWheelSpeedRC"] = msg["ABS_RRWheelSpeedRC"]
        write_msg["ABS_RLWheelSpeedRC"] = msg["ABS_RLWheelSpeedRC"]
        steering_wheel_angle_msg = vehicle_msgs[0x17E]
        signal = get_signal_sync_time(steering_wheel_angle_msg, time)
        
        write_msg["SAS_SteerWheelAngle"] = signal["SAS_SteerWheelAngle"]
        gear_mode_msg = vehicle_msgs[0x162]
        signal = get_signal_sync_time(gear_mode_msg, time)
        write_msg["VCU_ModeGearSts"] = signal["VCU_ModeGearSts"]
        imu_long_acc_msg = vehicle_msgs[0x3E2]
        signal = get_signal_sync_time(imu_long_acc_msg, time)
        write_msg["ESC_LongitudeACC"] = signal["ESC_LongitudeACC"]
        write_msg["ESC_LateralACC"] = signal["ESC_LateralACC"]
        speed_msg = vehicle_msgs[0x125]
        signal = get_signal_sync_time(speed_msg, time) 
        write_msg["ABS_VehicleSpeed"] = signal["ABS_VehicleSpeed"]/3.6 # Convert km/h to m/s 
        csv_f.write("{};{};{};{};{};{};{};{};{};{};{}\r\n".format(
            write_msg["ABS_VehicleSpeed"],
            write_msg["ABS_RRWheelSpeedRC"],
            write_msg["ABS_RLWheelSpeedRC"],
            write_msg["ABS_RRWheelDirection"],
            write_msg["ABS_RLWheelDirection"],
            write_msg["SAS_SteerWheelAngle"],
            write_msg["VCU_ModeGearSts"],
            write_msg["ESC_LongitudeACC"],
            write_msg["ESC_LateralACC"],
            0,time))    
    csv_f.close()    
    RTK_signals_mapping = {"PosLon2":"RTK_y",
                           "PosLat2":"RTK_x",
                           "AngleHeading":"RTK_heading"}
    
    RTK_msgs = dbc_decode_message(rtk_dbc, rtk_can)
    csv_f = open("rtk_data.csv","w")
    csv_f.write("RTK_x;RTK_y;RTK_z;RTK_heading;RTK_qualit;timestamp\r\n")
    rtk_heading_msgs = RTK_msgs[0x32A]
    x_list = []
    y_list = []
    heading_list = []
    timestamp_list = []
    rtk_quality_list = []
    for heading_msg in rtk_heading_msgs:
        time = heading_msg['timestamp']
        rtk_y_msgs = RTK_msgs[0x32D]
        rtk_x_msgs = RTK_msgs[0x32E]
        rtk_y_signal = get_signal_sync_time(rtk_y_msgs, time)
        rtk_x_signal = get_signal_sync_time(rtk_x_msgs, time)
        x_list.append(rtk_x_signal["PosLat2"])
        y_list.append(rtk_y_signal["PosLon2"])
        heading_list.append(heading_msg["AngleHeading"])
        timestamp_list.append(time)
        rtk_quality_list.append(5)
    plotter = RTKTrajectoryPlotter()
    plotter.load_from_lists(x_list, y_list, heading_list, timestamp_list, rtk_quality_list,True)
    trajectory = plotter.convert_to_enu()
    traject_points = plotter.get_trajectory_latlon(False)
    points_num = traject_points['count']
    for ii in range(points_num):
        csv_f.write("{};{};{};{};{};{}\r\n".format(
            traject_points['lon_list'][ii],
            traject_points['lat_list'][ii],
            0,
            traject_points['heading_list'][ii],
            traject_points['quality_list'][ii],
            traject_points['timestamp_list'][ii]))

    #plotter.print_trajectory_statistics()
    plotter.plot_trajectory()
    csv_f.close()