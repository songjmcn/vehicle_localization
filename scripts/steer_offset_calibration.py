from utils.DataReader import DataReader
from utils.rtk import RTKData, RTKUtils
from utils.LineCalibrator import LineCalibrator
import os

if __name__ == "__main__":
    # Paths
    mf4_file = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_20kph_001.mf4"
    save_dir = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_20kph_001"
    rtk_csv_file = os.path.join(save_dir, "rtk_data.csv")
    vehicle_localization_file = os.path.join(save_dir, "vehicle_localization.csv")

    # Read RTK data
    rtk_utils = RTKUtils(mf4_file)
    rtk_utils.save_csv(rtk_csv_file, None)
    rtk_data = RTKData(rtk_csv_file)

    # Read Vehicle Localization data
    data_reader = DataReader()
    vl_data = data_reader.read_vehicle_localization(vehicle_localization_file)

    # Calibrate steering offset
    calibrator = LineCalibrator(rtk_data, vl_data)
    best_offset, offsets, scores = calibrator.find_best_steering_offset()
    
    print(f"Best steering offset: {best_offset} degrees")