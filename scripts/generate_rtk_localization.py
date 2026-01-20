import os
from utils.rtk import RTKData, RTKUtils
if __name__ == "__main__":
    # Example usage of RTKData and RTKUtils
    mf4_file = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_5kph_002.mf4"
    save_dir = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_5kph_002"
    rtk_csv_file = os.path.join(save_dir, "rtk_data.csv")
    rtk_trajectory_file = os.path.join(save_dir, "rtk_trajectory.csv")
    vehicle_localization_file = os.path.join(save_dir, "vehicle_localization.csv")
    rtk_utils = RTKUtils(mf4_file)
    rtk_utils.save_csv(rtk_csv_file,rtk_trajectory_file)
    best_time_offset,offsets,scores= rtk_utils.find_best_time_offset(vehicle_localization_file)
    print(f"Best time offset: {best_time_offset} s")