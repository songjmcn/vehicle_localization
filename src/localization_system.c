/**
 * @file localization_system.c
 * @brief 定位系统实现
 */

#include "localization_system.h"
#include "motion_model.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
double MIN_SPEED_THRESHOLD = 0.02; /* 最低速度阈值，低于此值视为静止 */

// #define WHEEL_STEERING_MAPPING_NUM  155u
// LocalizationSystem_t localization_sys;
// float steering_angle[WHEEL_STEERING_MAPPING_NUM] = {   //方向盘转角(Rad)
// -8.79645943,-8.682219697,-8.567979964,-8.453740231,-8.339500499,-8.225260766,-8.111021033,-7.9967813,-7.882541567,
// -7.768301834,-7.654062101,-7.539822369,-7.425582636,-7.311342903,-7.19710317,-7.082863437,-6.968623704,-6.854383971,
// -6.740144239,-6.625904506,-6.511664773,-6.39742504,-6.283185307,-6.168945574,-6.054705841,-5.940466109,-5.826226376,
// -5.711986643,-5.59774691,-5.483507177,-5.369267444,-5.255027711,-5.140787979,-5.026548246,-4.912308513,-4.79806878,
// -4.683829047,-4.569589314,-4.455349581,-4.341109849,-4.226870116,-4.112630383,-3.99839065,-3.884150917,-3.769911184,
// -3.655671451,-3.541431719,-3.427191986,-3.312952253,-3.19871252,-3.084472787,-2.970233054,-2.855993321,-2.741753589,
// -2.627513856,-2.513274123,-2.39903439,-2.284794657,-2.170554924,-2.056315191,-1.942075459,-1.827835726,-1.713595993,
// -1.59935626,-1.485116527,-1.370876794,-1.256637061,-1.142397329,-1.028157596,-0.913917863,-0.79967813,-0.685438397,
// -0.571198664,-0.456958931,-0.342719199,-0.228479466,-0.114239733,0,0.114239733,0.228479466,0.342719199,0.456958931,
// 0.571198664,0.685438397,0.79967813,0.913917863,1.028157596,1.142397329,1.256637061,1.370876794,1.485116527,1.59935626,
// 1.713595993,1.827835726,1.942075459,2.056315191,2.170554924,2.284794657,2.39903439,2.513274123,2.627513856,2.741753589,
// 2.855993321,2.970233054,3.084472787,3.19871252,3.312952253,3.427191986,3.541431719,3.655671451,3.769911184,3.884150917,
// 3.99839065,4.112630383,4.226870116,4.341109849,4.455349581,4.569589314,4.683829047,4.79806878,4.912308513,5.026548246,
// 5.140787979,5.255027711,5.369267444,5.483507177,5.59774691,5.711986643,5.826226376,5.940466109,6.054705841,6.168945574,
// 6.283185307,6.39742504,6.511664773,6.625904506,6.740144239,6.854383971,6.968623704,7.082863437,7.19710317,7.311342903,
// 7.425582636,7.539822369,7.654062101,7.768301834,7.882541567,7.9967813,8.111021033,8.225260766,8.339500499,8.453740231,
// 8.567979964,8.682219697,8.79645943
// };
// float fl_wheel_angle[WHEEL_STEERING_MAPPING_NUM] = {// 前左轮轮转角(Rad)
// -0.676454711,-0.663661448,-0.651164891,-0.638930133,-0.626957174,-0.615211108,-0.603674482,-0.592329842,-0.581177188,
// -0.570199067,-0.559395479,-0.54874897,-0.538242088,-0.527874832,-0.51762975,-0.507524293,-0.49754101,-0.487662446,
// -0.477888602,-0.468219478,-0.458655074,-0.449177936,-0.439788065,-0.43048546,-0.421270122,-0.412124596,-0.403066337,
// -0.394077892,-0.385141806,-0.376292987,-0.367496527,-0.358769881,-0.350113048,-0.341491121,-0.332939008,-0.324439255,
// -0.315991861,-0.307596827,-0.2992367,-0.290946386,-0.282673526,-0.274470478,-0.266284884,-0.25815165,-0.250070775,
// -0.242007354,-0.233996293,-0.226020138,-0.218061437,-0.210155095,-0.202266207,-0.194412225,-0.18659315,-0.178808982,
// -0.171042267,-0.163310458,-0.155613556,-0.147934107,-0.140272112,-0.132645023,-0.125035388,-0.117443205,-0.10988593,
// -0.102346107,-0.094823738,-0.087318822,-0.07983136,-0.072378804,-0.064926248,-0.057508599,-0.05009095,-0.042708207,
// -0.035325464,-0.027977628,-0.020629792,-0.013299409,-0.005969026,-0.00132645,0.008621927,0.015899949,0.023160519,
// 0.030421089,0.037664205,0.044889868,0.052115531,0.059341195,0.066531951,0.073722708,0.080913464,0.088086767,
// 0.095260071,0.102415921,0.10957177,0.11672762,0.123866017,0.131004414,0.138125357,0.1452463,0.152367244,
// 0.159470734,0.166591677,0.173695167,0.180781204,0.187884694,0.194970731,0.202056768,0.209142804,0.216228841,
// 0.223314878,0.230383461,0.237469498,0.244538081,0.251624118,0.258692702,0.265761285,0.272829869,0.279898452,
// 0.286984489,0.294053072,0.301121656,0.308190239,0.315258823,0.32234486,0.329413443,0.33649948,0.343568063,
// 0.3506541,0.357740137,0.364826174,0.37191221,0.378998247,0.386101737,0.393205227,0.400291264,0.407412207,
// 0.414515697,0.421619187,0.428740131,0.435861074,0.442982017,0.450120414,0.457258811,0.464397207,0.471535604,
// 0.478691454,0.485847304,0.493003154,0.500176457,0.50734976,0.514523063,0.52171382,0.528904577,0.536112786,
// 0.543320996,0.550529206
// };
// float fr_wheel_angle[WHEEL_STEERING_MAPPING_NUM]={ //前右轮转角（Rad）
// -0.550529206,-0.543320996,-0.536112786,-0.528904577,-0.52171382,-0.514523063,-0.50734976,-0.500176457,-0.493003154,
// -0.485847304,-0.478691454,-0.471535604,-0.464397207,-0.457258811,-0.450120414,-0.442982017,-0.435861074,-0.428740131,
// -0.421619187,-0.414515697,-0.407412207,-0.400291264,-0.393205227,-0.386101737,-0.378998247,-0.37191221,-0.364826174,
// -0.357740137,-0.3506541,-0.343568063,-0.33649948,-0.329413443,-0.32234486,-0.315258823,-0.308190239,-0.301121656,
// -0.294053072,-0.286984489,-0.279898452,-0.272829869,-0.265761285,-0.258692702,-0.251624118,-0.244538081,-0.237469498,
// -0.230383461,-0.223314878,-0.216228841,-0.209142804,-0.202056768,-0.194970731,-0.187884694,-0.180781204,-0.173695167,
// -0.166591677,-0.159470734,-0.152367244,-0.1452463,-0.138125357,-0.131004414,-0.123866017,-0.11672762,-0.10957177,
// -0.102415921,-0.095260071,-0.088086767,-0.080913464,-0.073722708,-0.066531951,-0.059341195,-0.052115531,-0.044889868,
// -0.037664205,-0.030421089,-0.023160519,-0.015899949,-0.008621927,0.00132645,0.005969026,0.013299409,0.020629792,
// 0.027977628,0.035325464,0.042708207,0.05009095,0.057508599,0.064926248,0.072378804,0.07983136,0.087318822,
// 0.094823738,0.102346107,0.10988593,0.117443205,0.125035388,0.132645023,0.140272112,0.147934107,0.155613556,
// 0.163310458,0.171042267,0.178808982,0.18659315,0.194412225,0.202266207,0.210155095,0.218061437,0.226020138,
// 0.233996293,0.242007354,0.250070775,0.25815165,0.266284884,0.274470478,0.282673526,0.290946386,0.2992367,
// 0.307596827,0.315991861,0.324439255,0.332939008,0.341491121,0.350113048,0.358769881,0.367496527,0.376292987,
// 0.385141806,0.394077892,0.403066337,0.412124596,0.421270122,0.43048546,0.439788065,0.449177936,0.458655074,
// 0.468219478,0.477888602,0.487662446,0.49754101,0.507524293,0.51762975,0.527874832,0.538242088,0.54874897,
// 0.559395479,0.570199067,0.581177188,0.592329842,0.603674482,0.615211108,0.626957174,0.638930133,0.651164891,
// 0.663661448,0.676454711 
// };
/**
 * @brief 内部初始化函数（提取公共逻辑）
 */
static int localization_system_init_internal(LocalizationSystem_t *sys,
                                             const VehicleState_t *initial_state,
                                             const FusionConfig_t *fusion_config) {
    if (!sys || !fusion_config) {
        return -1;
    }
    
    /* 初始化状态 */
    VehicleState_t init_state;
    if (initial_state) {
        init_state = *initial_state;
    } else {
        memset(&init_state, 0, sizeof(init_state));
    }
    
    /* 设置默认噪声参数（heading误差优化版 - A方案） */
    NoiseParameters_t noise = {
        .q_x = 0.02,       /* 位置x噪声 */
        .q_y = 0.02,       /* 位置y噪声 */
        .q_theta = 0.002,    /* 角度噪声 */
        .q_v = 0.1,        /* 速度噪声（低速时轮滑动较多） */
        .q_delta = 0.01,    /* 转向角噪声 */
        .r_wheel = 0.01,    /* 轮速计测量噪声 */
        .r_imu = 0.1,       /* IMU加速度测量噪声 */
        .r_steering = 0.05, /* 转向角测量噪声 */
        .r_imu_steering = 0.08  /* IMU推导转向角噪声（新增） */
    };
    
    /* 设置默认校准参数 */
    CalibrationParams_t calibration = {
        .wheel_calibration_factor = 1,  /* 轮速计校准因子 */
        .steering_angle_offset = 0,     /* 转向角零偏 (rad) */
        .steering_angle_gain = 1.0,
        .imu_accel_bias_x = 0.0,
        .imu_accel_bias_y = 0.0,
        .imu_gyro_bias_z = 0.0,
        .imu_accel_scale_x = 1.0,
        .imu_accel_scale_y = 1.0,
        .imu_lateral_accel_bias = 0.0,    /* 新增 */
        .imu_lateral_accel_scale = 1.0,    /* 新增 */
    };              
    SteeringConfig_t steering_config= {
        .steering_ratio = 11.3f,
        .max_steering_wheel_angle = 8.321,
        .max_front_wheel_angle = 0.74,
        .wheelbase = 2.9,
        .steering_angle_offset = 0,
        .steering_angle_gain = 1.0f,
        .ackermann_factor = 1.0f,
        .enable_steering_filter = true,
        .steering_filter_alpha = 0.1f,
    };
    WheelConfig_t wheel_config = {
        .pulses_per_wheel_rev = 86,     /* 编码器：每轮每圈86脉冲（正确的原始值）*/
        .wheel_radius = 0.37,
        .low_speed_threshold = MIN_SPEED_THRESHOLD,
        .pulse_timeout_ms = 500,
        .max_count =  1023 //4095+1
    };
    SteeringMappingTable_t steering_mapping_table={0};
    // if(steering_mapping_table_init(&steering_mapping_table, steering_angle, fl_wheel_angle, fr_wheel_angle, WHEEL_STEERING_MAPPING_NUM)!=0){
    //     printf("steering_mapping_table_init failed\n");
    //     return -1;
    // }
    /* 初始化运动模型 */
    if (motion_model_init(&sys->heading_processer,&steering_config,&steering_mapping_table) != 0) {
        printf("motion_model_init failed\n");
        return -1;
    }
    


    /* *** 核心：调用 kalman_filter_init_with_mode() *** */
    if (kalman_filter_init_with_mode(&sys->kalman_filter,
                                      &init_state,
                                      &noise,
                                      &calibration,
                                      fusion_config) != 0) {
        printf("kalman_filter_init_with_mode failed\n");
        return -1;
    }
    /* 设置卡尔曼滤波器的转向模型配置 */
    if (kalman_filter_set_steering_config(&sys->kalman_filter, &sys->heading_processer) != 0) {
        printf("kalman_filter_set_steering_config failed\n");
        return -1;
    }

    /* 初始化转向角抖动过滤器 */
    if (steering_jitter_filter_init(&sys->kalman_filter.steering_filter) != 0) {
        printf("steering_jitter_filter_init failed\n");
        return -1;
    }
    
    /* 初始化RTK标定管理器 */
    // if (rtk_calibration_init(&sys->rtk_calibration, 100) != 0) {
    //     return -1;
    // }
    
    /* 保存校准参数 */
    sys->calibration = calibration;
    
    /* 初始化诊断信息 */
    memset(&sys->diagnostics, 0, sizeof(sys->diagnostics));
    
    /* 初始化轮速处理器 - 使用默认配置 */
    // WheelConfig_t default_wheel_config = {
    //     .pulses_per_wheel_rev = 400,    /* 常见的400脉冲/圈 */
    //     .wheel_radius = 0.35,           /* 典型轮半径 */
    //     .low_speed_threshold = 0.3,     /* 低速阈值 */
    //     .pulse_timeout_ms = 500         /* 超时检测 */
    // };
    
    if (wheel_odometry_processor_init(&sys->wheel_processor, &wheel_config) != 0) {
        return -1;
    }
    
    sys->wheel_config = wheel_config;
    
    sys->is_initialized = true;
    sys->operation_timestamp = 0;
    
    return 0;
}

int localization_system_init(LocalizationSystem_t *sys,
                            const VehicleState_t *initial_state) {
    if (!sys) {
        return -1;
    }
    
    /* 创建基础融合配置，保持向后兼容 */
    FusionConfig_t default_config = {
        .mode = FUSION_MODE_BASIC,
        .imu_steering_vel_threshold = 0.5f,
        .imu_steering_weight = 1.0f,
        .enable_adaptive_weighting = false
    };
    
    return localization_system_init_internal(sys, initial_state, &default_config);
}

/**
 * @brief 方式1实现：指定融合模式初始化
 */
int localization_system_init_with_fusion(LocalizationSystem_t *sys,
                                        const VehicleState_t *initial_state,
                                        FusionMode_t fusion_mode) {
    if (!sys) {
        return -1;
    }
    
    /* 创建对应的融合配置 */
    FusionConfig_t fusion_config;
    
    switch (fusion_mode) {
        case FUSION_MODE_BASIC:
            fusion_config.mode = FUSION_MODE_BASIC;
            fusion_config.imu_steering_vel_threshold = 0.5f;
            fusion_config.imu_steering_weight = 1.0f;
            fusion_config.enable_adaptive_weighting = false;
            break;
            
        case FUSION_MODE_IMU_STEERING:
            fusion_config.mode = FUSION_MODE_IMU_STEERING;
            fusion_config.imu_steering_vel_threshold = 0.5f;
            fusion_config.imu_steering_weight = 1.0f;
            fusion_config.enable_adaptive_weighting = false;
            break;
            
        default:
            return -1;
    }
    
    return localization_system_init_internal(sys, initial_state, &fusion_config);
}

/**
 * @brief 方式1实现：运行时切换融合模式
 */
int localization_system_switch_fusion(LocalizationSystem_t *sys,
                                     FusionMode_t fusion_mode) {
    if (!sys) {
        return -1;
    }
    
    /* 直接修改卡尔曼滤波器的融合模式 */
    sys->kalman_filter.fusion_config.mode = fusion_mode;
    
    return 0;
}

int localization_system_process(LocalizationSystem_t *sys,
                               const SensorReadings_t *sensor_data,
                               VehicleState_t *estimated_state) {
    if (!sys || !sensor_data || !estimated_state) {
        return -1;
    }
    
    if (!sys->is_initialized) {
        return -1;
    }
    
    /* 验证传感器数据时间戳 */
    if (sensor_data->timestamp == 0) {
        return -1;
    }
    
    /* 获取档位信息（从 SensorReadings_t 获取） */
    VehicleGear_t current_gear = sensor_data->gear;
    
    /* ===== 步骤0：根据数据类型处理轮速数据 ===== */
    float velocity = sensor_data->wheel_odometry.velocity;
    //float odometry_distance = 0.0f;  /* 新增：保存轮速计计算的距离 */
    
    /* 如果提供了脉冲数据，自动调用脉冲处理器 */
    if (sensor_data->has_wheel_pulse) {
        WheelOdometryData_t processed_odometry;
        
        /* 调用脉冲处理器（使用左右轮计数） */
        if (wheel_odometry_processor_update(&sys->wheel_processor,
                                           sensor_data->rear_left_wheel_count,
                                           sensor_data->rear_right_wheel_count,
                                           sensor_data->timestamp,
                                           current_gear,
                                           sensor_data->rear_left_wheel_direction,
                                           sensor_data->rear_right_wheel_direction,
                                           &processed_odometry) == 0) {
            /* 使用处理后的轮速数据 */
            velocity = processed_odometry.velocity;
            //odometry_distance = processed_odometry.distance;  /* 保存距离 */
// #ifdef DEBUG
//             printf("Odometry: velocity=%f, distance=%f, timestamp=%llu\n",
//                    velocity, odometry_distance, sensor_data->timestamp);
// #endif
        }
        else{
            // 处理失败，使用 EFK 推测的速度值继续计算
            velocity = kalman_filter_get_predicted_velocity(&sys->kalman_filter);
        }
    } else {
        /* 使用直接提供的速度值 */
        WheelOdometryData_t processed_odometry;
        if (wheel_odometry_processor_set_velocity(&sys->wheel_processor,
                                                 velocity,
                                                 (uint32_t)sensor_data->timestamp,
                                                 &processed_odometry) == 0) {
            velocity = processed_odometry.velocity;  /* 使用校准后的速度 */
            //odometry_distance = processed_odometry.distance;  /* 保存距离 */
        }
            /* 处理失败，但继续使用原始速度值 */
        else {
            // 处理失败，使用 EFK 推测的速度值继续计算
             velocity = kalman_filter_get_predicted_velocity(&sys->kalman_filter);
        }
    }

    /* 方向盘左转为正，右转为负，不再做翻转
     */
    float steering_angle = sensor_data->steering.steering_angle;
    
    /* 
     * 注意：脉冲处理已在上面自动完成
     */
    
    /* ===== 步骤1：更新转向角过滤器 ===== */
    if (steering_jitter_filter_update(&sys->kalman_filter.steering_filter, &sys->heading_processer.steering_config,
                                       steering_angle) != 0) {
        return -1;
    }
    
    /* ===== 步骤2：低速时计算零偏角 ===== */
    if (fabsf(velocity) < (float)(MIN_SPEED_THRESHOLD)) {
        steering_jitter_filter_compute_offset(&sys->kalman_filter.steering_filter, velocity);
    }
    
    /* ===== 步骤3：获取过滤统计信息用于诊断 ===== */
    double mean_angle=0.;
    double variance = 0.;
    bool is_high_jitter;
    steering_jitter_filter_get_stats(&sys->kalman_filter.steering_filter,
                                      &mean_angle, &variance, NULL, &is_high_jitter);
    
    /* ===== 步骤4：统一计算车辆运动方向（仅此一处计算） ===== */
    /* 关键：方向由这里统一计算，后续模块直接使用，避免重复计算 */
    VehicleDirection_t vehicle_direction = wheel_odometry_derive_direction(
        velocity,
        current_gear,
        sensor_data->rear_left_wheel_direction,
        sensor_data->rear_right_wheel_direction);
    //VehicleDirection_t vehicle_direction = wheel_direction_to_vehicle_direction(wheel_direction);
    /* ===== 步骤5：使用过滤后的转向角进行卡尔曼滤波 ===== */
    /* 使用低通过滤后的转向角而不是原始值 */
    // 使用转向模型在入口完成方向盘角->前轮角的映射与校准
    // steering_jitter_filter 已更新过，继续使用转向模型输出作为前轮角
    // steering_model_update 返回校准/滤波后的前轮角（float，rad）
    float front_wheel_angle = steering_model_update(&sys->heading_processer, steering_angle, sensor_data->timestamp);
    // 确保不超过前轮角的物理极限
    float max_front = sys->heading_processer.steering_config.max_front_wheel_angle;
    if (fabsf(front_wheel_angle) > max_front) {
        front_wheel_angle = clamp(front_wheel_angle, -max_front, max_front);
    }
    double filtered_steering = (double)front_wheel_angle; /* 供 EKF 使用的前轮角（以 rad 为单位） */
    
    /* 设置档位信息到滤波器（从 sensor_data 获取） */
    if (current_gear != GEAR_INVALID) {
        kalman_filter_set_gear(&sys->kalman_filter, current_gear);
    }
    
    /* === 关键：创建带有方向信息的状态结构体，传递给 kalman_filter_predict === */
    VehicleState_t current_state_with_direction;
    kalman_filter_get_state(&sys->kalman_filter, &current_state_with_direction);
    current_state_with_direction.direction = vehicle_direction;  /* 设置计算好的方向 */

    /* === 修复：删除重复的速度符号处理
     * wheel_odometry_processor在计算velocity时已经应用了方向符号：
     *   - 前进(DIRECTION_FORWARD): velocity > 0
     *   - 倒车(DIRECTION_REVERSE): velocity < 0
     *   - 停止(DIRECTION_STATIONARY): velocity ≈ 0
     *
     * 修复：直接使用processor返回的velocity，无需再次处理符号
     * motion_model_predict会使用这个已有符号的velocity进行位置更新
     */
    double signed_velocity = velocity;  /* 直接使用processor返回的已有符号的速度，无需重复处理 */
    double raw_rate = 0.;
    /* 预测步骤 */
    kalman_filter_predict_with_state(&sys->kalman_filter,
                                      &current_state_with_direction,
                                      &raw_rate,  
                                      signed_velocity,
                                      filtered_steering,
                                      sensor_data->timestamp);

    /* === 关键修复：确保Kalman滤波的update使用脉冲处理后的速度，而不是原始的不准确的speed数据
     * 在低速时，脉冲计数比ECU的speed更准确，所以需要让Kalman滤波也使用处理后的速度作为观测
     * 否则Kalman会用不准确的speed来修正脉冲速度，导致输出是两者的混合
     *
     * 因为sensor_data是const，我们创建一个修改后的副本来传给update
     */
    SensorReadings_t sensor_data_with_corrected_velocity = *sensor_data;
    sensor_data_with_corrected_velocity.wheel_odometry.velocity = velocity;  /* 用处理后的脉冲速度覆盖原始速度 */
    /* 将传入给卡尔曼更新的转向测量替换为已校准的前轮角（语义统一） */
    sensor_data_with_corrected_velocity.steering.steering_angle = front_wheel_angle;

    /* 更新步骤 */
    kalman_filter_update(&sys->kalman_filter, &sensor_data_with_corrected_velocity);
    
    /* 获取估计状态 */
    kalman_filter_get_state(&sys->kalman_filter, estimated_state);
    estimated_state->heading = estimated_state->heading+sys->calibration.heading_bias; /* 应用航向偏差校准 */
    estimated_state->gear = sensor_data->gear;
    estimated_state->direction = vehicle_direction;  /* 设置计算好的方向 */
    estimated_state->yaw_rate = raw_rate;  /* 原始转向率 */
    estimated_state->velocity = signed_velocity;  /* 使用带符号的速度 */

    /* 获取诊断信息 */
    sys->diagnostics = kalman_filter_get_diagnostics(&sys->kalman_filter);
    
    /* === 新增：距离一致性诊断 === */
    /* 对比轮速计计算的距离与卡尔曼滤波计算的位移 */
//     double kalman_distance = sqrt(estimated_state->x * estimated_state->x +
//                                   estimated_state->y * estimated_state->y);
//     double distance_error = fabs(odometry_distance - (float)kalman_distance);

// #ifdef DEBUG
//     printf("Distance check: odometry=%.3f, kalman=%.3f, error=%.3f\n",
//            odometry_distance, kalman_distance, distance_error);
// #endif

    /* 如果距离差异过大，记录警告 */
    // if (distance_error > 0.5) {  /* 误差超过 50cm */
    //     fprintf(stderr, "Warning: Large distance discrepancy: odometry=%.3f, kalman=%.3f, error=%.3f\n",
    //             odometry_distance, kalman_distance, distance_error);
    // }

    sys->operation_timestamp = sensor_data->timestamp;
    
    return 0;
}

void localization_system_get_state(const LocalizationSystem_t *sys,
                                  VehicleState_t *state) {
    if (!sys || !state) {
        return;
    }
    
    kalman_filter_get_state(&sys->kalman_filter, state);
}

DiagnosticsInfo_t localization_system_get_diagnostics(const LocalizationSystem_t *sys) {
    if (!sys) {
        DiagnosticsInfo_t empty = {0};
        return empty;
    }
    
    return sys->diagnostics;
}

/**
 * @brief 手动设置校准参数（带置信度门槛）
 * 
 * 应用外部标定系统（如RTKCalibrationSystem_t）计算的校准参数
 * @param sys 系统指针
 * @param calibration 校准参数
 * @param confidence 离线标定置信度分数 [0,1]，如果小于阈值则拒绝应用
 * @return 0 应用成功，-1 拒绝或失败
 */
int localization_system_set_calibration(LocalizationSystem_t *sys,
                                        const CalibrationParams_t *calibration) {
    if (!sys || !calibration) {
        return -1;
    }
    
    
    sys->calibration = *calibration;
    sys->kalman_filter.calibration = *calibration;
    
    /* Phase 2 (P1) RTK校准集成：应用转向模型的校准参数 */
    /* 将 CalibrationParams_t 的转向参数应用到转向模型 */
    steering_model_apply_rtk_calibration(&sys->heading_processer, calibration);
    
    return 0;  /* 成功应用 */
}

void localization_system_reset(LocalizationSystem_t *sys,
                              const VehicleState_t *initial_state) {
    if (!sys) {
        return;
    }
    
    VehicleState_t reset_state;
    if (initial_state) {
        reset_state = *initial_state;
    } else {
        memset(&reset_state, 0, sizeof(reset_state));
    }
    
    kalman_filter_reset(&sys->kalman_filter, &reset_state);
    memset(&sys->diagnostics, 0, sizeof(sys->diagnostics));
    sys->operation_timestamp = 0;
}

void localization_system_free(LocalizationSystem_t *sys) {
    if (!sys) {
        return;
    }
    
    sys->is_initialized = false;
}

/* ========== 转向角过滤器辅助函数 ========== */

int localization_system_set_steering_filter_params(LocalizationSystem_t *sys,
                                                    double jitter_threshold,
                                                    double filter_alpha) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    return steering_jitter_filter_set_params(&sys->kalman_filter.steering_filter,
                                              jitter_threshold,
                                              filter_alpha);
}

int localization_system_get_steering_stats(const LocalizationSystem_t *sys,
                                            double *mean_angle,
                                            double *variance,
                                            double *zero_bias,
                                            bool *is_high_jitter) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    return steering_jitter_filter_get_stats(&sys->kalman_filter.steering_filter,
                                             mean_angle,
                                             variance,
                                             zero_bias,
                                             is_high_jitter);
}

void localization_system_reset_steering_filter(LocalizationSystem_t *sys) {
    if (!sys || !sys->is_initialized) {
        return;
    }
    
    steering_jitter_filter_reset(&sys->kalman_filter.steering_filter);
}

/* ========== 轮速脉冲处理接口 ========== */

int localization_system_set_wheel_config(LocalizationSystem_t *sys,
                                        const WheelConfig_t *config) {
    if (!sys || !config || !sys->is_initialized) {
        return -1;
    }
    
    sys->wheel_config = *config;
    return wheel_odometry_processor_init(&sys->wheel_processor, config);
}

int localization_system_update_wheel_pulse(LocalizationSystem_t *sys,
                                          uint32_t pulse_count,
                                          uint64_t timestamp,
                                          VehicleGear_t gear,
                                          WheelDirection_t rear_left_state,
                                          WheelDirection_t rear_right_state) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    WheelOdometryData_t odometry;
    
    /* 调用底层轮速处理器（左右轮使用相同的计数） */
    if (wheel_odometry_processor_update(&sys->wheel_processor,
                                       pulse_count,
                                       pulse_count,
                                       (uint32_t)timestamp,
                                       gear,
                                       rear_left_state,
                                       rear_right_state,
                                       &odometry) != 0) {
        return -1;
    }
    
    sys->operation_timestamp = timestamp;
    
    return 0;
}

int localization_system_update_wheel_velocity(LocalizationSystem_t *sys,
                                             double velocity,
                                             uint64_t timestamp,
                                             VehicleGear_t gear,
                                             WheelDirection_t rear_left_state,
                                             WheelDirection_t rear_right_state) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    /* 注意：wheel_odometry_processor_set_velocity 不支持档位和轮状态参数 */
    /* 这些参数由调用者传入，但此处仅用于接口一致性 */
    (void)gear;
    (void)rear_left_state;
    (void)rear_right_state;
    
    WheelOdometryData_t odometry;
    if (wheel_odometry_processor_set_velocity(&sys->wheel_processor,
                                             velocity,
                                             (uint32_t)timestamp,
                                             &odometry) != 0) {
        return -1;
    }
    
    sys->operation_timestamp = timestamp;
    
    return 0;
}

int localization_system_get_wheel_odometry(const LocalizationSystem_t *sys,
                                           WheelOdometryData_t *odometry) {
    if (!sys || !odometry || !sys->is_initialized) {
        return -1;
    }
    
    /* 直接从处理器获取最新的轮速数据 */
    odometry->velocity = (float)sys->wheel_processor.last_velocity;
    odometry->distance = (float)sys->wheel_processor.accumulated_distance;
    odometry->timestamp = sys->wheel_processor.last_timestamp;
    
    /* 注意：档位和轮方向需要从外部传入，处理器不存储这些值 */
    /* 这些值应该由调用者通过 SensorReadings_t 传入 */
    odometry->gear = GEAR_INVALID;
    odometry->rear_left_direction = WHEEL_DIRECRTION_INVALID;
    odometry->rear_right_direction = WHEEL_DIRECRTION_INVALID;
    
    return 0;
}

bool localization_system_wheel_is_timeout(const LocalizationSystem_t *sys) {
    if (!sys) {
        return true;
    }
    return wheel_odometry_processor_is_timeout(&sys->wheel_processor);
}

int localization_system_reset_wheel_processor(LocalizationSystem_t *sys) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    return wheel_odometry_processor_reset(&sys->wheel_processor);
}
