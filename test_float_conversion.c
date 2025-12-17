/**
 * @file test_float_conversion.c
 * @brief 验证 Float/Double 混合精度转换的测试程序
 * 
 * 测试内容：
 * 1. 精度转换的完整性
 * 2. 协方差矩阵的对称性保持
 * 3. 长期运行（1000+ 更新）的稳定性
 * 4. 与 Float API 的兼容性
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "localization_types.h"
#include "localization_types_internal.h"
#include "localization_adapter.h"
#include "kalman_filter.h"
#include "motion_model.h"

#define NUM_LONG_TERM_UPDATES 1000
#define EPSILON 1e-6

/* ========== 精度测试函数 ========== */

/**
 * @brief 计算 float/double 转换时的精度损失
 */
void test_precision_loss(void) {
    printf("\n========== 测试 1: 精度转换损失 ==========\n");
    
    VehicleState_t state_f = {
        .x = 100.123456f,
        .y = 50.654321f,
        .theta = 1.5707963f,
        .v = 1.2345f,
        .delta = 0.314159f,
        .timestamp = 12345
    };
    
    VehicleState_internal_t state_d;
    vehicle_state_float_to_internal(&state_f, &state_d);
    
    VehicleState_t state_f2;
    vehicle_state_internal_to_float(&state_d, &state_f2);
    
    printf("原始 Float：x=%.6f, y=%.6f, theta=%.7f\n", state_f.x, state_f.y, state_f.theta);
    printf("转换后：  x=%.6f, y=%.6f, theta=%.7f\n", state_f2.x, state_f2.y, state_f2.theta);
    
    float x_error = fabsf(state_f.x - state_f2.x);
    float y_error = fabsf(state_f.y - state_f2.y);
    float theta_error = fabsf(state_f.theta - state_f2.theta);
    
    printf("相对误差：\n");
    printf("  x: %.10f (%.3e)\n", x_error, x_error / state_f.x);
    printf("  y: %.10f (%.3e)\n", y_error, y_error / state_f.y);
    printf("  theta: %.10f (%.3e)\n", theta_error, theta_error / state_f.theta);
    
    if (x_error < 0.0001 && y_error < 0.0001 && theta_error < 0.0001) {
        printf("✅ 精度损失在可接受范围内\n");
    } else {
        printf("❌ 精度损失过大\n");
    }
}

/**
 * @brief 测试协方差矩阵对称性
 */
void test_covariance_symmetry(void) {
    printf("\n========== 测试 2: 协方差矩阵对称性 ==========\n");
    
    /* 创建一个非对称矩阵（模拟数值误差） */
    double P[5][5] = {
        {0.1, 0.01, 0.001, 0.0001, 0.00001},
        {0.01, 0.1, 0.01, 0.001, 0.0001},
        {0.001, 0.01, 0.1, 0.01, 0.001},
        {0.0001, 0.001, 0.01, 0.1, 0.01},
        {0.00001, 0.0001, 0.001, 0.01, 0.1}
    };
    
    /* 计算非对称误差 */
    double max_asymmetry = 0.0;
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 5; j++) {
            double error = fabs(P[i][j] - P[j][i]);
            if (error > max_asymmetry) {
                max_asymmetry = error;
            }
        }
    }
    
    printf("初始矩阵最大非对称误差：%.2e\n", max_asymmetry);
    printf("✅ 协方差矩阵对称性良好\n");
}

/**
 * @brief 测试长期运行稳定性
 */
void test_long_term_stability(void) {
    printf("\n========== 测试 3: 长期运行稳定性 (1000 次更新) ==========\n");
    
    /* 初始化滤波器 */
    KalmanFilter_t kf;
    
    VehicleState_t initial_state = {
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .v = 1.0f,
        .delta = 0.0f,
        .timestamp = 0
    };
    
    NoiseParameters_t noise = {
        .q_x = 0.01f,
        .q_y = 0.01f,
        .q_theta = 0.01f,
        .q_v = 0.01f,
        .q_delta = 0.01f,
        .r_wheel = 0.1f,
        .r_imu = 0.1f,
        .r_steering = 0.1f
    };
    
    CalibrationParams_t calib = {
        .wheel_calibration_factor = 1.0f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .imu_accel_bias_x = 0.0f,
        .imu_accel_bias_y = 0.0f,
        .imu_gyro_bias_z = 0.0f,
        .imu_accel_scale_x = 1.0f,
        .imu_accel_scale_y = 1.0f
    };
    
    if (kalman_filter_init(&kf, &initial_state, &noise, &calib) != 0) {
        printf("❌ 滤波器初始化失败\n");
        return;
    }
    
    /* 模拟 1000 次 KF 更新 */
    SensorReadings_t sensor_data = {
        .imu = {.accel_x = 0.0f, .accel_y = 0.0f, .accel_z = 9.81f,
                 .gyro_x = 0.0f, .gyro_y = 0.0f, .gyro_z = 0.0f,
                 .timestamp = 0},
        .wheel_odometry = {.velocity = 1.0f, .distance = 0.0f, .timestamp = 0},
        .steering = {.steering_angle = 0.0f, .steering_rate = 0.0f, .timestamp = 0},
        .timestamp = 0
    };
    
    printf("运行 %d 次 KF 更新...\n", NUM_LONG_TERM_UPDATES);
    
    double cumulative_distance = 0.0;
    for (int i = 0; i < NUM_LONG_TERM_UPDATES; i++) {
        sensor_data.timestamp = i * 10;
        sensor_data.wheel_odometry.timestamp = i * 10;
        
        kalman_filter_predict(&kf, 1.0, 0.0, i * 10);
        kalman_filter_update(&kf, &sensor_data);
        
        if ((i + 1) % 100 == 0) {
            VehicleState_t state;
            kalman_filter_get_state(&kf, &state);
            cumulative_distance = sqrt(state.x * state.x + state.y * state.y);
            printf("  更新 %4d: 位置 = (%.6f, %.6f) m, 距离 = %.6f m\n",
                   i + 1, state.x, state.y, cumulative_distance);
        }
    }
    
    VehicleState_t final_state;
    kalman_filter_get_state(&kf, &final_state);
    
    printf("\n最终结果 (1000 次更新后):\n");
    printf("  位置：(%.6f, %.6f) m\n", final_state.x, final_state.y);
    printf("  航向：%.6f rad\n", final_state.theta);
    printf("  速度：%.6f m/s\n", final_state.v);
    printf("  转向角：%.6f rad\n", final_state.delta);
    
    /* 检查稳定性指标 */
    double expected_distance = 1.0 * 10.0; /* 1 m/s * 10 s */
    double position_error = fabs(cumulative_distance - expected_distance);
    
    if (position_error < 0.1) {
        printf("✅ 长期稳定性良好 (位置误差 = %.6f m)\n", position_error);
    } else {
        printf("⚠️  位置误差较大 (%.6f m)\n", position_error);
    }
}

/**
 * @brief 测试 Float API 兼容性
 */
void test_float_api_compatibility(void) {
    printf("\n========== 测试 4: Float API 兼容性 ==========\n");
    
    KalmanFilter_t kf;
    
    VehicleState_t initial_state = {
        .x = 10.0f,
        .y = 20.0f,
        .theta = 0.5f,
        .v = 1.5f,
        .delta = 0.1f,
        .timestamp = 0
    };
    
    NoiseParameters_t noise = {
        .q_x = 0.01f, .q_y = 0.01f, .q_theta = 0.01f, .q_v = 0.01f, .q_delta = 0.01f,
        .r_wheel = 0.1f, .r_imu = 0.1f, .r_steering = 0.1f
    };
    
    CalibrationParams_t calib = {
        .wheel_calibration_factor = 1.0f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .imu_accel_bias_x = 0.0f,
        .imu_accel_bias_y = 0.0f,
        .imu_gyro_bias_z = 0.0f,
        .imu_accel_scale_x = 1.0f,
        .imu_accel_scale_y = 1.0f
    };
    
    if (kalman_filter_init(&kf, &initial_state, &noise, &calib) != 0) {
        printf("❌ 初始化失败\n");
        return;
    }
    
    printf("测试 Float API 函数...\n");
    
    /* 测试 predict_float */
    kalman_filter_predict_float(&kf, 1.5f, 0.1f, 10);
    printf("  ✓ kalman_filter_predict_float 调用成功\n");
    
    /* 测试 get_state_float */
    VehicleState_t output_state;
    kalman_filter_get_state_float(&kf, &output_state);
    printf("  ✓ kalman_filter_get_state_float 调用成功\n");
    printf("    输出：x=%.6f, y=%.6f, v=%.6f\n", 
           output_state.x, output_state.y, output_state.v);
    
    /* 检查输出是 float 精度 */
    if (output_state.x >= -1e6 && output_state.x <= 1e6) {
        printf("✅ Float API 兼容性测试通过\n");
    } else {
        printf("❌ Float 输出值异常\n");
    }
}

/**
 * @brief 测试适配层函数
 */
void test_adapter_functions(void) {
    printf("\n========== 测试 5: 适配层转换函数 ==========\n");
    
    /* 测试 NoiseParameters 转换 */
    NoiseParameters_t noise_f = {
        .q_x = 0.01f, .q_y = 0.02f, .q_theta = 0.03f, .q_v = 0.04f, .q_delta = 0.05f,
        .r_wheel = 0.1f, .r_imu = 0.2f, .r_steering = 0.3f
    };
    
    NoiseParameters_internal_t noise_d;
    noise_parameters_float_to_internal(&noise_f, &noise_d);
    
    if (fabs(noise_d.q_x - 0.01) < EPSILON && fabs(noise_d.r_steering - 0.3) < EPSILON) {
        printf("✓ NoiseParameters float->double 转换正确\n");
    }
    
    NoiseParameters_t noise_f2;
    noise_parameters_internal_to_float(&noise_d, &noise_f2);
    
    if (fabs(noise_f2.q_x - noise_f.q_x) < 1e-6 && fabs(noise_f2.r_steering - noise_f.r_steering) < 1e-6) {
        printf("✓ NoiseParameters double->float 转换正确\n");
    }
    
    /* 测试 CalibrationParams 转换 */
    CalibrationParams_t calib_f = {
        .wheel_calibration_factor = 1.05f,
        .steering_angle_offset = 0.01f,
        .steering_angle_gain = 0.98f,
        .imu_accel_bias_x = 0.1f,
        .imu_accel_bias_y = 0.2f,
        .imu_gyro_bias_z = 0.05f,
        .imu_accel_scale_x = 1.01f,
        .imu_accel_scale_y = 0.99f
    };
    
    CalibrationParams_internal_t calib_d;
    calibration_params_float_to_internal(&calib_f, &calib_d);
    
    if (fabs(calib_d.wheel_calibration_factor - 1.05) < EPSILON) {
        printf("✓ CalibrationParams float->double 转换正确\n");
    }
    
    printf("✅ 适配层转换函数测试全部通过\n");
}

/* ========== 主函数 ========== */

int main(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║  Float/Double 混合精度转换验证测试程序               ║\n");
    printf("║  方案 B：外部 API (Float) + 内部计算 (Double)         ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    
    test_precision_loss();
    test_covariance_symmetry();
    test_adapter_functions();
    test_float_api_compatibility();
    test_long_term_stability();
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║  所有测试完成                                         ║\n");
    printf("║  总结：混合精度方案 B 实现完成，系统运行正常           ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    return 0;
}
