/**
 * @file fusion_integration_example.c
 * @brief 多融合算法架构集成示例
 * 
 * 本文件演示如何在实际项目中集成和使用多融合算法
 */

#include <stdio.h>
#include <math.h>
#include "kalman_filter.h"
#include "localization_types.h"

/* ========== 配置参数 ========== */

/**
 * @brief 创建基础参数配置
 */
static void create_basic_parameters(NoiseParameters_t *noise,
                                   CalibrationParams_t *calib) {
    if (!noise || !calib) return;
    
    /* 噪声参数 */
    noise->q_x = 0.01f;
    noise->q_y = 0.01f;
    noise->q_theta = 0.01f;
    noise->q_v = 0.1f;
    noise->q_delta = 0.05f;
    
    noise->r_wheel = 0.05f;
    noise->r_imu = 0.1f;
    noise->r_steering = 0.02f;
    noise->r_imu_steering = 0.05f;  /* 新增 */
    
    /* 校准参数 */
    calib->wheel_calibration_factor = 1.0f;
    calib->steering_angle_offset = 0.0f;
    calib->steering_angle_gain = 1.0f;
    
    calib->imu_accel_bias_x = 0.0f;
    calib->imu_accel_bias_y = 0.0f;
    calib->imu_gyro_bias_z = 0.0f;
    
    calib->imu_accel_scale_x = 1.0f;
    calib->imu_accel_scale_y = 1.0f;
    
    /* IMU转向融合参数(新增) */
    calib->imu_lateral_accel_bias = 0.05f;
    calib->imu_lateral_accel_scale = 1.0f;
}

/* ========== 方案1：使用基础算法 ========== */

/**
 * @brief 方案1示例 - 基础算法 (3D观测)
 * 
 * 适用场景：低速、泊车、对精度要求不高的应用
 */
int example_basic_fusion(void) {
    printf("=== 方案1: 基础融合算法 (3D观测) ===\n\n");
    
    KalmanFilter_t kf;
    VehicleState_t initial_state = {0, 0, 0, 0, 0, 0};
    NoiseParameters_t noise;
    CalibrationParams_t calib;
    
    create_basic_parameters(&noise, &calib);
    
    /* 初始化 - 自动默认为基础算法 */
    if (kalman_filter_init(&kf, &initial_state, &noise, &calib) != 0) {
        printf("ERROR: 初始化失败\n");
        return -1;
    }
    
    printf("✓ 使用基础算法初始化\n");
    printf("  - 融合模式: FUSION_MODE_BASIC\n");
    printf("  - 观测维数: 3D\n");
    printf("  - 应用场景: 低速、泊车\n\n");
    
    /* 模拟传感器数据 */
    SensorReadings_t sensor_data = {0};
    sensor_data.wheel_odometry.velocity = 0.5f;  /* 低速 */
    sensor_data.imu.accel_x = 0.0f;
    sensor_data.imu.accel_y = 0.1f;
    sensor_data.imu.accel_z = 9.8f;
    sensor_data.steering.steering_angle = 0.1f;
    
    /* 执行滤波步骤 */
    kalman_filter_predict(&kf, sensor_data.wheel_odometry.velocity,
                         sensor_data.steering.steering_angle, 0);
    kalman_filter_update(&kf, &sensor_data);
    
    VehicleState_t state;
    kalman_filter_get_state(&kf, &state);
    
    printf("状态估计结果:\n");
    printf("  x=%.4f, y=%.4f, theta=%.4f\n", state.x, state.y, state.theta);
    printf("  v=%.4f, delta=%.4f\n\n", state.v, state.delta);
    
    return 0;
}

/* ========== 方案2：使用IMU转向融合 ========== */

/**
 * @brief 方案2示例 - IMU转向融合 (4D观测)
 * 
 * 适用场景：正常行驶、高速、对精度要求高的应用
 */
int example_imu_steering_fusion(void) {
    printf("=== 方案2: IMU转向融合算法 (4D观测) ===\n\n");
    
    KalmanFilter_t kf;
    VehicleState_t initial_state = {0, 0, 0, 0, 0, 0};
    NoiseParameters_t noise;
    CalibrationParams_t calib;
    
    create_basic_parameters(&noise, &calib);
    
    /* 配置融合参数 */
    FusionConfig_t fusion_config = {
        .mode = FUSION_MODE_IMU_STEERING,       /* 选择IMU融合模式 */
        .imu_steering_vel_threshold = 0.5f,     /* 最小速度0.5m/s */
        .imu_steering_weight = 1.0f,            /* 融合权重1.0 */
        .enable_adaptive_weighting = true       /* 启用自适应权重 */
    };
    
    /* 以指定融合模式初始化 */
    if (kalman_filter_init_with_mode(&kf, &initial_state, 
                                     &noise, &calib, 
                                     &fusion_config) != 0) {
        printf("ERROR: 初始化失败\n");
        return -1;
    }
    
    printf("✓ 使用IMU转向融合算法初始化\n");
    printf("  - 融合模式: FUSION_MODE_IMU_STEERING\n");
    printf("  - 观测维数: 4D\n");
    printf("  - 应用场景: 正常行驶、高速\n");
    printf("  - 速度阈值: %.1f m/s\n", fusion_config.imu_steering_vel_threshold);
    printf("  - IMU权重: %.1f\n", fusion_config.imu_steering_weight);
    printf("  - 自适应权重: %s\n\n", fusion_config.enable_adaptive_weighting ? "启用" : "禁用");
    
    /* 模拟传感器数据 - 高速情况 */
    SensorReadings_t sensor_data = {0};
    sensor_data.wheel_odometry.velocity = 10.0f;  /* 高速 */
    sensor_data.imu.accel_x = 0.0f;
    sensor_data.imu.accel_y = 3.5f;               /* 明显的横向加速度 */
    sensor_data.imu.accel_z = 9.8f;
    sensor_data.steering.steering_angle = 0.2f;
    
    /* 执行滤波步骤 */
    kalman_filter_predict(&kf, sensor_data.wheel_odometry.velocity,
                         sensor_data.steering.steering_angle, 0);
    kalman_filter_update(&kf, &sensor_data);
    
    VehicleState_t state;
    kalman_filter_get_state(&kf, &state);
    
    printf("状态估计结果:\n");
    printf("  x=%.4f, y=%.4f, theta=%.4f\n", state.x, state.y, state.theta);
    printf("  v=%.4f, delta=%.4f\n", state.v, state.delta);
    
    DiagnosticsInfo_t diag = kalman_filter_get_diagnostics(&kf);
    printf("\n诊断信息:\n");
    printf("  残差范数: %.6f\n", diag.residual_norm);
    printf("  更新次数: %u\n", diag.update_count);
    printf("  轮滑动: %s\n", diag.is_wheel_slipping ? "是" : "否");
    printf("  静止状态: %s\n\n", diag.is_stationary ? "是" : "否");
    
    return 0;
}

/* ========== 方案3：动态融合模式切换 ========== */

/**
 * @brief 根据速度自适应切换融合模式
 */
void adaptive_fusion_mode_switch(KalmanFilter_t *kf, float velocity) {
    FusionMode_t current_mode = kf->fusion_config.mode;
    FusionMode_t new_mode = current_mode;
    
    if (fabs(velocity) < 1.0f) {
        /* 低速：使用基础算法 */
        new_mode = FUSION_MODE_BASIC;
    } else if (fabs(velocity) > 2.0f) {
        /* 高速：使用IMU融合算法 */
        new_mode = FUSION_MODE_IMU_STEERING;
    }
    /* 过渡区 (1-2 m/s)：保持不变 */
    
    if (new_mode != current_mode) {
        printf("融合模式切换: %s -> %s\n",
               current_mode == FUSION_MODE_BASIC ? "BASIC" : "IMU_STEERING",
               new_mode == FUSION_MODE_BASIC ? "BASIC" : "IMU_STEERING");
        kf->fusion_config.mode = new_mode;
    }
}

/**
 * @brief 方案3示例 - 动态融合模式切换
 * 
 * 适用场景：复杂场景需要动态切换算法模式
 */
int example_adaptive_fusion_switching(void) {
    printf("=== 方案3: 动态融合模式切换 ===\n\n");
    
    KalmanFilter_t kf;
    VehicleState_t initial_state = {0, 0, 0, 0, 0, 0};
    NoiseParameters_t noise;
    CalibrationParams_t calib;
    
    create_basic_parameters(&noise, &calib);
    
    /* 初始化为基础算法 */
    if (kalman_filter_init(&kf, &initial_state, &noise, &calib) != 0) {
        printf("ERROR: 初始化失败\n");
        return -1;
    }
    
    printf("✓ 初始化完成，使用动态切换模式\n\n");
    
    /* 模拟不同速度下的动态切换 */
    float velocities[] = {0.3f, 0.8f, 1.5f, 3.0f, 5.0f, 8.0f, 1.0f, 0.2f};
    int num_samples = sizeof(velocities) / sizeof(velocities[0]);
    
    printf("模拟过程中的动态模式切换:\n");
    printf("-" "-----" "-" "-----" "-" "----------" "-" "----------" "\n");
    printf("| 序号 | 速度(m/s) | 切换前模式 | 切换后模式 |\n");
    printf("-" "-----" "-" "-----" "-" "----------" "-" "----------" "\n");
    
    for (int i = 0; i < num_samples; i++) {
        float v = velocities[i];
        const char *mode_before = kf->fusion_config.mode == FUSION_MODE_BASIC 
                                 ? "BASIC" : "IMU_STEERING";
        
        adaptive_fusion_mode_switch(&kf, v);
        
        const char *mode_after = kf->fusion_config.mode == FUSION_MODE_BASIC 
                                ? "BASIC" : "IMU_STEERING";
        
        printf("| %4d | %8.1f | %10s | %10s |\n", 
               i, v, mode_before, mode_after);
    }
    printf("-" "-----" "-" "-----" "-" "----------" "-" "----------" "\n\n");
    
    return 0;
}

/* ========== 方案4：场景感知融合配置 ========== */

typedef enum {
    SCENARIO_PARKING,
    SCENARIO_NORMAL,
    SCENARIO_HIGHWAY
} DrivingScenario_t;

/**
 * @brief 根据行驶场景配置融合参数
 */
void configure_fusion_for_scenario(KalmanFilter_t *kf, 
                                   DrivingScenario_t scenario) {
    FusionConfig_t *config = &kf->fusion_config;
    
    switch (scenario) {
        case SCENARIO_PARKING:
            printf("场景: 泊车\n");
            config->mode = FUSION_MODE_BASIC;
            config->imu_steering_weight = 0.0f;
            printf("  - 模式: BASIC\n");
            printf("  - IMU权重: 0.0 (不使用IMU)\n");
            break;
            
        case SCENARIO_NORMAL:
            printf("场景: 正常行驶\n");
            config->mode = FUSION_MODE_IMU_STEERING;
            config->imu_steering_weight = 0.8f;
            config->enable_adaptive_weighting = true;
            printf("  - 模式: IMU_STEERING\n");
            printf("  - IMU权重: 0.8\n");
            printf("  - 自适应权重: 启用\n");
            break;
            
        case SCENARIO_HIGHWAY:
            printf("场景: 高速行驶\n");
            config->mode = FUSION_MODE_IMU_STEERING;
            config->imu_steering_weight = 1.0f;
            config->enable_adaptive_weighting = false;
            printf("  - 模式: IMU_STEERING\n");
            printf("  - IMU权重: 1.0 (最大权重)\n");
            printf("  - 自适应权重: 禁用\n");
            break;
    }
}

/**
 * @brief 方案4示例 - 场景感知融合配置
 */
int example_scenario_based_configuration(void) {
    printf("=== 方案4: 场景感知融合配置 ===\n\n");
    
    KalmanFilter_t kf;
    VehicleState_t initial_state = {0, 0, 0, 0, 0, 0};
    NoiseParameters_t noise;
    CalibrationParams_t calib;
    
    create_basic_parameters(&noise, &calib);
    
    if (kalman_filter_init(&kf, &initial_state, &noise, &calib) != 0) {
        printf("ERROR: 初始化失败\n");
        return -1;
    }
    
    printf("✓ 初始化完成\n\n");
    
    /* 演示不同场景的配置 */
    DrivingScenario_t scenarios[] = {
        SCENARIO_PARKING,
        SCENARIO_NORMAL,
        SCENARIO_HIGHWAY
    };
    
    for (int i = 0; i < 3; i++) {
        printf("%" "u) ", i + 1);
        configure_fusion_for_scenario(&kf, scenarios[i]);
        printf("\n");
    }
    
    return 0;
}

/* ========== 主函数 ========== */

/**
 * @brief 主函数 - 演示所有方案
 */
int main(void) {
    printf("╔═" "════" "════════════════════════════════════════════════════════════════╗\n");
    printf("║ " "多融合算法架构集成示例" " " "║\n");
    printf("║ " "演示基础算法 vs IMU转向融合 vs 动态切换 vs 场景感知" " " "║\n");
    printf("╚═" "════" "════════════════════════════════════════════════════════════════╝\n\n");
    
    /* 方案1：基础算法 */
    example_basic_fusion();
    printf("\n" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "\n\n");
    
    /* 方案2：IMU转向融合 */
    example_imu_steering_fusion();
    printf("\n" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "\n\n");
    
    /* 方案3：动态切换 */
    example_adaptive_fusion_switching();
    printf("\n" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "\n\n");
    
    /* 方案4：场景感知 */
    example_scenario_based_configuration();
    printf("\n" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "━" "\n\n");
    
    printf("✓ 所有示例执行完成\n\n");
    
    return 0;
}
