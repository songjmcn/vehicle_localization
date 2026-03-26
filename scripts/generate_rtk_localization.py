import os
import pandas as pd
from utils.rtk import RTKData, RTKUtils
if __name__ == "__main__":
    # Example usage of RTKData and RTKUtils
    mf4_file = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__backward_0_5kph_001.mf4"
    save_dir = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__backward_0_5kph_001"
    rtk_csv_file = os.path.join(save_dir, "rtk_data.csv")
    rtk_trajectory_file = os.path.join(save_dir, "rtk_trajectory.csv")
    vehicle_localization_file = os.path.join(save_dir, "vehicle_localization.csv")
    vehicle_data = pd.read_csv(vehicle_localization_file)
    loc_heading = vehicle_data['heading'].values
    loc_timestamp = vehicle_data['timestamp'].values
    loc_velocity = vehicle_data['velocity'].values
    rtk_utils = RTKUtils(mf4_file)
    rtk_trajectory = rtk_utils.convert_to_body_with_localization(loc_timestamps=loc_timestamp,loc_velocities=loc_velocity,is_reverse=True)
    dframe = pd.DataFrame(rtk_trajectory)
    dframe.to_csv(rtk_trajectory_file, index=False)
    
    # rtk_utils.save_csv(rtk_csv_file,rtk_trajectory_file)
    best_time_offset,offsets,scores= rtk_utils.find_best_time_offset(loc_heading, loc_timestamp, is_show_plot=False)
    print(f"Best time offset: {best_time_offset} s")