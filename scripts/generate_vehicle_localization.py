import os,sys
from utils.DataReader import read_mf4_vehicle_data
if __name__ == "__main__":
    mf4_file = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_20kph_001.mf4"
    vehicle_data_csv_file = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_20kph_001/vehicle_data.csv"
    vehicle_localization_file = "/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stragight__forward_0_20kph_001/vehicle_localization.csv"
    vehcile_localization_exe = "/Users/alex/workspace/code_review/vehicle_localization/build/bin/localization_example"
    read_mf4_vehicle_data(mf4_file, vehicle_data_csv_file)
    print(f"Vehicle data CSV generated at: {vehicle_data_csv_file}")
    print(f"Vehicle localization CSV will be generated at: {vehicle_localization_file}")
    os.system('{0} -i {1} -o {2}'.format(vehcile_localization_exe, vehicle_data_csv_file, vehicle_localization_file))