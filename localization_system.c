/**
 * @file localization_system.c
 * @brief 定位系统实现
 */

#include "localization_system.h"
#include "motion_model.h"
#include <stdlib.h>
#include <string.h>

int localization_system_init(LocalizationSystem_t *sys,
                            const VehicleState_t *initial_state) {
    if (!sys) {
        return -1;
    }
    
    /* 初始化状态 */
    VehicleState_t init_state;
    if (initial_state) {
        init_state = *initial_state;
    } else {
        memset(&init_state, 0, sizeof(init_state));
    }
    
    /* 设置默认噪声参数（自动泊车场景优化） */
    NoiseParameters_t noise = {
        .q_x = 0.005,       /* 位置x噪声 */
        .q_y = 0.005,       /* 位置y噪声 */
        .q_theta = 0.01,    /* 角度噪声 */
        .q_v = 0.02,        /* 速度噪声（低速时轮滑动较多） */
        .q_delta = 0.02,    /* 转向角噪声 */
        .r_wheel = 0.01,    /* 轮速计测量噪声 */
        .r_imu = 0.1,       /* IMU加速度测量噪声 */
        .r_steering = 0.05  /* 转向角测量噪声 */
    };
    
    /* 设置默认校准参数 */
    CalibrationParams_t calibration = {
        .wheel_calibration_factor = 1.0,
        .steering_angle_offset = 0.0,
        .steering_angle_gain = 1.0,
        .imu_accel_bias_x = 0.0,
        .imu_accel_bias_y = 0.0,
        .imu_gyro_bias_z = 0.0,
        .imu_accel_scale_x = 1.0,
        .imu_accel_scale_y = 1.0
    };
    
    /* 初始化运动模型 */
    motion_model_init();
    
    /* 初始化卡尔曼滤波器 */
    if (kalman_filter_init(&sys->kalman_filter, &init_state, &noise, &calibration) != 0) {
        return -1;
    }
    
    /* 初始化转向角抖动过滤器 */
    if (steering_jitter_filter_init(&sys->kalman_filter.steering_filter) != 0) {
        return -1;
    }
    
    /* 初始化RTK标定管理器 */
    if (rtk_calibration_init(&sys->rtk_calibration, 100) != 0) {
        return -1;
    }
    
    /* 保存校准参数 */
    sys->calibration = calibration;
    
    /* 初始化诊断信息 */
    memset(&sys->diagnostics, 0, sizeof(sys->diagnostics));
    
    /* 初始化轮速处理器 - 使用默认配置 */
    WheelConfig_t default_wheel_config = {
        .pulses_per_wheel_rev = 400,    /* 常见的400脉冲/圈 */
        .wheel_radius = 0.35,           /* 典型轮半径 */
        .low_speed_threshold = 0.3,     /* 低速阈值 */
        .pulse_timeout_ms = 500         /* 超时检测 */
    };
    
    if (wheel_odometry_processor_init(&sys->wheel_processor, &default_wheel_config) != 0) {
        return -1;
    }
    
    sys->wheel_config = default_wheel_config;
    
    sys->is_initialized = true;
    sys->operation_timestamp = 0;
    
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
    
    /* 提取控制输入 */
    double velocity = sensor_data->wheel_odometry.velocity;
    double steering_angle = sensor_data->steering.steering_angle;
    
    /* 
     * 注意：如果用户通过 update_wheel_pulse() 或 update_wheel_velocity() 更新了
     * 轮速数据，建议在调用本函数前获取处理后的轮速数据并更新 sensor_data
     * 示例：
     *   localization_system_update_wheel_pulse(sys, pulse_count, timestamp);
     *   WheelOdometryData_t processed_odometry;
     *   localization_system_get_wheel_odometry(sys, &processed_odometry);
     *   sensor_data.wheel_odometry = processed_odometry;
     */
    
    /* ===== 步骤1：更新转向角过滤器 ===== */
    if (steering_jitter_filter_update(&sys->kalman_filter.steering_filter, 
                                       steering_angle) != 0) {
        return -1;
    }
    
    /* ===== 步骤2：低速时计算零偏角 ===== */
    if (fabs(velocity) < 0.2) {
        steering_jitter_filter_compute_offset(&sys->kalman_filter.steering_filter, velocity);
    }
    
    /* ===== 步骤3：获取过滤统计信息用于诊断 ===== */
    double mean_angle, variance;
    bool is_high_jitter;
    steering_jitter_filter_get_stats(&sys->kalman_filter.steering_filter,
                                      &mean_angle, &variance, NULL, &is_high_jitter);
    
    /* ===== 步骤4：使用过滤后的转向角进行卡尔曼滤波 ===== */
    /* 使用低通过滤后的转向角而不是原始值 */
    double filtered_steering = sys->kalman_filter.steering_filter.filtered_angle;
    if (fabs(filtered_steering) > STEERING_ANGLE_MAX) {
        filtered_steering = clamp(filtered_steering, -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX);
    }
    
    /* 预测步骤 */
    kalman_filter_predict(&sys->kalman_filter, 
                         velocity, 
                         filtered_steering, 
                         sensor_data->timestamp);
    
    /* 更新步骤 */
    kalman_filter_update(&sys->kalman_filter, sensor_data);
    
    /* 获取估计状态 */
    kalman_filter_get_state(&sys->kalman_filter, estimated_state);
    
    /* 获取诊断信息 */
    sys->diagnostics = kalman_filter_get_diagnostics(&sys->kalman_filter);
    
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
 * @brief RTK离线标定 - 添加一个标定点
 * 
 * 用于离线标定过程，收集多个点然后计算标定参数。
 * 不在实时定位过程中进行RTK融合。
 */
int localization_system_rtk_calibration_add_point(LocalizationSystem_t *sys,
                                                  const VehicleState_t *odometry_state,
                                                  const RTKData_t *rtk_state) {
    if (!sys || !odometry_state || !rtk_state) {
        return -1;
    }
    
    /* 添加标定点 */
    return rtk_calibration_add_point(&sys->rtk_calibration, 
                                     odometry_state, 
                                     rtk_state);
}

int localization_system_apply_rtk_calibration(LocalizationSystem_t *sys) {
    if (!sys) {
        return -1;
    }
    
    /* 计算校准参数 */
    CalibrationParams_t calibration;
    if (rtk_calibration_compute(&sys->rtk_calibration, &calibration) != 0) {
        return -1;
    }
    
    /* 保存校准参数 */
    sys->calibration = calibration;
    
    /* 更新卡尔曼滤波器中的校准参数 */
    sys->kalman_filter.calibration = calibration;
    
    return 0;
}

void localization_system_set_calibration(LocalizationSystem_t *sys,
                                        const CalibrationParams_t *calibration) {
    if (!sys || !calibration) {
        return;
    }
    
    sys->calibration = *calibration;
    sys->kalman_filter.calibration = *calibration;
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
    rtk_calibration_clear(&sys->rtk_calibration);
    memset(&sys->diagnostics, 0, sizeof(sys->diagnostics));
    sys->operation_timestamp = 0;
}

void localization_system_free(LocalizationSystem_t *sys) {
    if (!sys) {
        return;
    }
    
    rtk_calibration_free(&sys->rtk_calibration);
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
                                          uint32_t timestamp) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    WheelOdometryData_t odometry;
    if (wheel_odometry_processor_update(&sys->wheel_processor, 
                                       pulse_count, 
                                       timestamp, 
                                       &odometry) != 0) {
        return -1;
    }
    
    /* 将处理后的轮速数据保存到系统状态 */
    /* 注意：这里不直接修改 sensor_data，而是供 localization_system_process 使用 */
    sys->operation_timestamp = timestamp;
    
    return 0;
}

int localization_system_update_wheel_velocity(LocalizationSystem_t *sys,
                                             double velocity,
                                             uint32_t timestamp) {
    if (!sys || !sys->is_initialized) {
        return -1;
    }
    
    WheelOdometryData_t odometry;
    if (wheel_odometry_processor_set_velocity(&sys->wheel_processor,
                                             velocity,
                                             timestamp,
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
    odometry->velocity = sys->wheel_processor.last_velocity;
    odometry->distance = sys->wheel_processor.accumulated_distance;
    odometry->timestamp = sys->wheel_processor.last_timestamp;
    
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
