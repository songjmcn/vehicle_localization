/**
 * @file example_usage.c
 * @brief 定位系统使用示例
 */

#include "src/localization_system.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/**
 * @brief 模拟传感器数据的生成函数
 * 
 * 模拟一个J形停车场景：
 * 1. 直行100米
 * 2. 左转90度，后退30米
 */
void simulate_sensor_data(uint32_t step, SensorReadings_t *sensor_data) {
    static uint32_t start_time = 0;
    if (start_time == 0) {
        start_time = 1000;  /* 从1000ms开始 */
    }
    
    uint32_t dt = 50;  /* 50ms的时间步长 */
    uint32_t timestamp = start_time + step * dt;
    
    /* 阶段1：直行（0-100m）*/
    if (step < 2000) {  /* 100秒内直行 */
        sensor_data->wheel_odometry.velocity = 1.0;  /* 1 m/s */
        sensor_data->steering.steering_angle = 0.0;  /* 直行 */
    }
    /* 阶段2：转向并后退 */
    else if (step < 2600) {  /* 30秒转向 */
        sensor_data->wheel_odometry.velocity = 0.5;  /* 0.5 m/s */
        sensor_data->steering.steering_angle = 0.5;  /* 左转 */
    }
    /* 阶段3：停止 */
    else {
        sensor_data->wheel_odometry.velocity = 0.0;
        sensor_data->steering.steering_angle = 0.0;
    }
    
    /* 模拟IMU数据：低速状态下IMU数据较小 */
    double accel = 0.05;  /* 小的加速度噪声 */
    
    sensor_data->imu.accel_x = accel * (rand() % 100 - 50) / 50.0;
    sensor_data->imu.accel_y = accel * (rand() % 100 - 50) / 50.0;
    sensor_data->imu.accel_z = -9.8;  /* 重力加速度 */
    
    /* 陀螺仪数据：基于转向角变化估计 */
    double steering_rate = 0.01;  /* rad/s */
    sensor_data->imu.gyro_x = 0.0;
    sensor_data->imu.gyro_y = 0.0;
    sensor_data->imu.gyro_z = steering_rate * sin(sensor_data->steering.steering_angle);
    
    sensor_data->imu.timestamp = timestamp;
    
    /* 轮速计和转向角数据 */
    sensor_data->wheel_odometry.distance += sensor_data->wheel_odometry.velocity * dt / 1000.0;
    sensor_data->wheel_odometry.timestamp = timestamp;
    
    sensor_data->steering.steering_rate = 0.0;  /* 缓慢变化 */
    sensor_data->steering.timestamp = timestamp;
    
    sensor_data->timestamp = timestamp;
}
typedef struct VehicleData{
    uint32_t wheel_counter_rr;
    uint32_t wheel_counter_rl;
    WheelDirection_t wheel_direction_rr;
    WheelDirection_t wheel_direction_rl;
    VehicleGear_t gear;
    float vehicle_speed;          // m/s
    float steering_wheel_angle;   // rad
    float imu_accel_x;            // m/s
    float imu_accel_y;            // m/s
    float imu_accel_z;            // m/s
    double timestamp_s;           // ms  
}VehicleData;
typedef  struct VehicleDataArray{
    struct VehicleData *data_array;
    uint32_t data_count;
}VehicleDataArray;

VehicleGear_t get_gear(const char *gear_str){
    if(strcmp(gear_str,"P")==0){
        return GEAR_PARK;
    }else if(strcmp(gear_str,"R")==0){
        return GEAR_REVERSE;
    }else if(strcmp(gear_str,"N")==0){
        return GEAR_NEUTRAL;
    }else if(strcmp(gear_str,"D")==0){
        return GEAR_DRIVE;
    }else if(strcmp(gear_str,"L")==0){
        return GEAR_LOW;
    }else{
        return GEAR_INVALID;
    }
}   
WheelDirection_t get_wheel_direction(const char *dir_str){
    if(strcmp(dir_str,"Forward")==0){
        return WHEEL_DIRECTION_FORWARD;
    }else if(strcmp(dir_str,"Backward")==0){
        return WHEEL_DIRECTION_REVERSE;
    }else if(strcmp(dir_str,"Stop")==0){
        return WHEEL_DIRECTION_STOP;
    }
    else{
        return WHEEL_DIRECRTION_INVALID;
    }
}
bool load_csv_data(const char *filename,VehicleDataArray *vehicle_data_array){
    uint32_t line_num = 0;
    int ret =0;
    if(filename==NULL || vehicle_data_array==NULL){
        return false;
    }

    FILE *file= fopen(filename,"r");
    if(!file){
        return false;
    }
    char *line = malloc(8192);
    if(!line){
        fclose(file);
        return false;   
    }

    while(feof(file)==0){
        if(fgets(line,4096,file)==NULL){
            break;
        }
        line_num++;
    }
    fseek(file,0,SEEK_SET);
    if(vehicle_data_array->data_array){
        free(vehicle_data_array->data_array);
        vehicle_data_array->data_array = NULL;
        vehicle_data_array->data_count = 0;
    }
    vehicle_data_array->data_array = malloc(sizeof(VehicleData)*line_num);
    if(!vehicle_data_array->data_array){
        free(line);
        fclose(file);
        return false;
    }
    for(uint32_t i=0;i<line_num;i++){
        char gear[20]={0};
        char wheel_dir_rr[20]={0};
        char wheel_dir_rl[20]={0};
        VehicleData *pdata =NULL;
        if(fgets(line,4096,file)==NULL){
            break;
        }
        if(i==0){
            continue; //skip header
        }   

        pdata = vehicle_data_array->data_array+i-1;;
        double steering_angle = 0; {
            char *saveptr = NULL;
            char *fields[11] = {0};
            int idx = 0;
            char *token = strtok_r(line, ";\r\n", &saveptr);
            while (token != NULL && idx < 11) {
                fields[idx++] = token;
                token = strtok_r(NULL, ";\r\n", &saveptr);
            }
            ret = idx; /* parsed field count */
            if (ret >= 11) {
                pdata->vehicle_speed = (float) strtof(fields[0], NULL);
                pdata->wheel_counter_rr = (uint32_t) strtoul(fields[1], NULL, 10);
                pdata->wheel_counter_rl = (uint32_t) strtoul(fields[2], NULL, 10);
                strncpy(wheel_dir_rr, fields[3], sizeof(wheel_dir_rr) - 1);
                wheel_dir_rr[sizeof(wheel_dir_rr) - 1] = '\0';
                strncpy(wheel_dir_rl, fields[4], sizeof(wheel_dir_rl) - 1);
                wheel_dir_rl[sizeof(wheel_dir_rl) - 1] = '\0';
                steering_angle = strtod(fields[5], NULL);
                strncpy(gear, fields[6], sizeof(gear) - 1);
                gear[sizeof(gear) - 1] = '\0';
                pdata->imu_accel_x = (float) strtof(fields[7], NULL);
                pdata->imu_accel_y = (float) strtof(fields[8], NULL);
                pdata->imu_accel_z = (float) strtof(fields[9], NULL);
                pdata->timestamp_s = strtod(fields[10], NULL);
            }
            else {
                /* 解析失败，置零以避免未定义行为 */
                pdata->vehicle_speed = 0.0f;
                pdata->wheel_counter_rr = 0;
                pdata->wheel_counter_rl = 0;
                wheel_dir_rr[0] = wheel_dir_rl[0] = '\0';
                steering_angle = 0.0;
                gear[0] = '\0';
                pdata->imu_accel_x = pdata->imu_accel_y = pdata->imu_accel_z = 0.0f;
                pdata->timestamp_s = 0.0;
            }
        }
         pdata->gear = get_gear(gear);
         pdata->steering_wheel_angle = (float)(steering_angle * 3.14159265358979323846/180.0); // deg to rad
         pdata->wheel_direction_rr = get_wheel_direction(wheel_dir_rr);
         pdata->wheel_direction_rl = get_wheel_direction(wheel_dir_rl);
    }
    vehicle_data_array->data_count = line_num-1;
    return true;
}
/**
 * @brief 打印定位结果
 */
void print_localization_result(uint32_t step, const VehicleState_t *state, 
                              const DiagnosticsInfo_t *diag) {
    if (step % 200 == 0) {  /* 每10秒打印一次 */
        printf("[Step %u (%.1fs)] Position: (%.2f, %.2f) m, "
               "Heading: %.2f deg, Velocity: %.2f m/s\n",
               step, step * 0.05,
               state->x, state->y,
               RAD_TO_DEG(state->heading),
               state->v);
        
        if (diag->is_wheel_slipping) {
            printf("  [WARNING] Wheel slipping detected! accel=%.2f m/s²\n",
                   diag->wheel_slip_accel);
        }
        
        if (diag->is_stationary) {
            printf("  [INFO] Vehicle stationary\n");
        }
    }
}
const char* gear2str(VehicleGear_t gear){
    switch(gear){
        case GEAR_PARK:
            return "Parking";
        case GEAR_REVERSE:
            return "Reverse";
        case GEAR_NEUTRAL:
            return "Neutral";
        case GEAR_DRIVE:
            return "Drive";
        case GEAR_LOW:
            return "Low";
        default:
            return "Invalid";
    }
}
FILE* create_csv_file(const char* file_path) {
    FILE* f=fopen(file_path,"w");
    if (f==NULL) {
        return f;
    }
    fprintf(f,"x;y;heading;steering angle;gear;timestamp\r\n");
    return f;
}
bool save_localization_data(FILE *f,VehicleState_t *state){
    if(f==NULL || state==NULL){
        return false;
    }
    fprintf(f,"%.3f;%.3f;%.3f;%.3f;%s;%llu\r\n",
            state->x,
            state->y,
            state->heading,
            state->steering_angle,
            gear2str(state->gear),
            state->timestamp);
    return true;
}
LocalizationSystem_t localization_sys;
int main()
{

    /* 初始化定位系统 */
    //LocalizationSystem_t localization_sys;
    //VehicleDataArray vehicle_data_array={0};
    memset(&localization_sys, 0, sizeof(localization_sys));
    bool ret = false;
    VehicleState_t initial_state = {
        .x = 0.0,
        .y = 0.0,
        .heading = 0.0,
        .v = 0.0,
        .steering_angle = 0.0,
        .timestamp = 1000
    };
    
    if (localization_system_init(&localization_sys, &initial_state) != 0) {
        printf("ERROR: Failed to initialize localization system!\n");
        return -1;
    }
    CalibrationParams_t calibration = {
        .wheel_calibration_factor = 0.99240929,
        .steering_angle_offset = -0.00914977,
        .steering_angle_gain = 1.0,
        .imu_accel_bias_x = 0.0f,
        .imu_accel_bias_y = 0.0f,
        .imu_gyro_bias_z = 0.0f,
        .imu_accel_scale_x = 1.0,
        .imu_accel_scale_y = 1.0,
        .imu_lateral_accel_bias = 0.0f,
        .imu_lateral_accel_scale = 1.0f
    };
    localization_system_set_calibration(&localization_sys,&calibration);
    
    /* === ISO 8855坐标系配置确认 === */
    printf("═══════════════════════════════════════════════════════\n");
    printf("📍 ISO 8855 REAR AXLE CENTER (RAC) MODEL CONFIGURATION\n");
    printf("═══════════════════════════════════════════════════════\n");
    printf("✓ Coordinate System: ISO 8855 Body-Fixed Frame\n");
    printf("✓ Origin: Rear Axle Center (RAC)\n");
    printf("✓ X-axis: Longitudinal (forward positive)\n");
    printf("✓ Y-axis: Lateral (left positive)\n");
    printf("✓ Z-axis: Vertical (up positive)\n");
    printf("✓ Handedness: Right-hand coordinate system\n");
    printf("\n");
    printf("📊 SYSTEM CONFIGURATION:\n");
    printf("  • Velocity source: Rear wheel encoders (rear axle longitudinal speed)\n");
    printf("  • Steering angle: Front wheel steering δ (auto-reversed in reverse gear)\n");
    printf("  • IMU frame: Cartesian (body-aligned)\n");
    printf("  • Non-holonomic constraint: Rear wheels cannot slip sideways\n");
    printf("  • Wheelbase: %.2f m\n", 2.7f);
    printf("═══════════════════════════════════════════════════════\n\n");
    
    /* ===== 可选：配置轮速脉冲处理器 ===== */
    /* 
     * 如果硬件提供脉冲计数而非直接速度值，需要先配置脉冲处理器。
     * 此示例展示两种使用方式：
     * 方式1：传入脉冲计数，由处理器转换为速度
     * 方式2：直接传入速度值（使用默认配置）
     */
    // WheelConfig_t wheel_config = {
    //     .pulses_per_wheel_rev = 96,    /* 码盘：96脉冲/圈 */
    //     .wheel_radius = 0.35,           /* 轮半径：0.35米 */
    //     .low_speed_threshold = 0.05,     /* 低速平滑阈值 */
    //     .pulse_timeout_ms = 500         /* 脉冲超时检测 */
    // };
    //
    // if (localization_system_set_wheel_config(&localization_sys, &wheel_config) != 0) {
    //     printf("WARNING: Failed to configure wheel processor\n");
    // } else {
    //     printf("✓ Wheel odometry processor configured\n");
    //     printf("  - Pulses per revolution: %u\n", wheel_config.pulses_per_wheel_rev);
    //     printf("  - Wheel radius: %.2f m\n", wheel_config.wheel_radius);
    //     printf("  - Low speed threshold: %.2f m/s\n\n", wheel_config.low_speed_threshold);
    // }
    
    /* 运行定位循环（2600步 = 130秒）*/
    printf("Running localization process...\n");
    printf("───────────────────────────────────────────────────────\n");
    
    /* 脉冲计数模拟变量 */
    uint32_t accumulated_pulses = 0;
    (void)accumulated_pulses;  /* 此示例中未使用，保留供用户参考 */
    VehicleDataArray vehicle_data_array={0};
    ret = load_csv_data("vehicle_data.csv",&vehicle_data_array);
    if(ret==false){
        printf("ERROR: Failed to load vehicle data from vehicle_data.csv!\n");
        return -1;
    }
    FILE *csv_writer = create_csv_file("vehicle_localization.csv");
    for (uint32_t step = 0; step < vehicle_data_array.data_count; step++) {
        SensorReadings_t sensor_data;
        memset(&sensor_data, 0, sizeof(sensor_data));
        
        /* 填充IMU数据 */
        sensor_data.imu.accel_x = vehicle_data_array.data_array[step].imu_accel_x;
        sensor_data.imu.accel_y = vehicle_data_array.data_array[step].imu_accel_y;
        sensor_data.imu.accel_z = vehicle_data_array.data_array[step].imu_accel_z;
        sensor_data.imu.timestamp = (uint64_t)(vehicle_data_array.data_array[step].timestamp_s * 1000);
        
        /* 填充转向数据 */
        sensor_data.steering.steering_angle = vehicle_data_array.data_array[step].steering_wheel_angle;
        sensor_data.steering.timestamp = (uint64_t)(vehicle_data_array.data_array[step].timestamp_s * 1000);
        
        /* 填充档位数据 */
        sensor_data.gear = vehicle_data_array.data_array[step].gear;
        
        /* 填充时间戳 */
        sensor_data.timestamp = (uint64_t)(vehicle_data_array.data_array[step].timestamp_s * 1000);
        
        /* ===== 选项A：使用脉冲模式（推荐用于编码器） ===== */
        sensor_data.has_wheel_pulse = true;  /* 启用脉冲处理 */
        sensor_data.rear_right_wheel_count = vehicle_data_array.data_array[step].wheel_counter_rr;
        sensor_data.rear_right_wheel_direction = vehicle_data_array.data_array[step].wheel_direction_rr;
        sensor_data.rear_left_wheel_count = vehicle_data_array.data_array[step].wheel_counter_rl;
        sensor_data.rear_left_wheel_direction = vehicle_data_array.data_array[step].wheel_direction_rl;
        
        /* ===== 或选项B：使用直接速度模式（推荐用于速度传感器） ===== */
        // sensor_data.has_wheel_pulse = false;
        // sensor_data.wheel_odometry.velocity = vehicle_data_array.data_array[step].vehicle_speed;
        // sensor_data.rear_left_wheel_state = vehicle_data_array.data_array[step].wheel_direction_rl;
        // sensor_data.rear_right_wheel_state = vehicle_data_array.data_array[step].wheel_direction_rr;
        
        /* === 统一的定位处理调用（自动处理脉冲或速度）=== */
        VehicleState_t estimated_state;
        if (localization_system_process(&localization_sys, &sensor_data, &estimated_state) != 0) {
            printf("ERROR: Localization process failed at step %u\n", step);
            continue;
        }
        
        /* 获取诊断信息 */
        DiagnosticsInfo_t diag = localization_system_get_diagnostics(&localization_sys);
        
        /* 打印结果 */
        print_localization_result(step, &estimated_state, &diag);
        save_localization_data(csv_writer,&estimated_state);
    }
    fclose(csv_writer);
    printf("\n───────────────────────────────────────────────────────\n");
    printf("Note: RTK Online Update is disabled.\n");
    printf("Only offline RTK calibration for parameter estimation is available.\n");
    printf("See WHEEL_PULSE_INTEGRATION.md and INTEGRATION_GUIDE.md for calibration methods.\n");
    printf("───────────────────────────────────────────────────────\n");
    
    /* 最终状态 */
    printf("\nFinal localization result:\n");
    VehicleState_t final_state;
    localization_system_get_state(&localization_sys, &final_state);
    printf("  Position: (%.2f, %.2f) m\n", final_state.x, final_state.y);
    printf("  Heading: %.2f deg\n", RAD_TO_DEG(final_state.heading));
    printf("  Velocity: %.2f m/s\n", final_state.v);
    printf("  Steering angle: %.2f deg\n", RAD_TO_DEG(final_state.steering_angle));
    
    /* 释放资源 */
    localization_system_free(&localization_sys);
    printf("\n✓ System cleanup completed\n");
    
    return 0;
}
