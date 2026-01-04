/**
 * @file test_steering_rtk_integration.c
 * @brief Task 4: 转向模型 RTK 校准集成测试
 * 
 * 验证：
 * 1. 增益获取/设置正常
 * 2. 零偏获取/设置正常
 * 3. RTK校准参数应用到转向模型
 * 4. 定位系统集成转向模型校准
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "localization_types.h"
#include "steering_model.h"

/**
 * @brief 测试 1: 获取和设置转向增益
 */
int test_steering_gain_access(void) {
    printf("\n══════════════════════════════════════════════════\n");
    printf("Test 1: 转向增益获取和设置\n");
    printf("══════════════════════════════════════════════════\n");
    
    SteeringModel_t steering_model = {0};
    SteeringConfig_t config = {
        .steering_ratio = 15.3f,
        .max_steering_wheel_angle = 8.79645943f,
        .max_front_wheel_angle = 0.676454711f,
        .wheelbase = 2.76f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .enable_steering_filter = false,
        .steering_filter_alpha = 0.1f,
        .steering_deadzone = 0.02f,
    };
    
    if (steering_config_init(&steering_model, &config) != 0) {
        printf("❌ 初始化转向模型失败\n");
        return -1;
    }
    
    /* 测试初始增益 */
    float gain = steering_model_get_gain(&steering_model);
    printf("✓ 初始增益: %.4f\n", gain);
    if (fabsf(gain - 1.0f) > 1e-6f) {
        printf("❌ 初始增益错误！期望 1.0\n");
        return -1;
    }
    
    /* 测试设置增益 */
    int ret = steering_model_update_gain(&steering_model, 1.05f);
    if (ret != 0) {
        printf("❌ 设置增益失败\n");
        return -1;
    }
    
    gain = steering_model_get_gain(&steering_model);
    printf("✓ 更新后增益: %.4f\n", gain);
    if (fabsf(gain - 1.05f) > 1e-6f) {
        printf("❌ 更新增益错误！期望 1.05\n");
        return -1;
    }
    
    /* 测试无效增益（超出范围） */
    ret = steering_model_update_gain(&steering_model, 2.0f);  /* 超出范围 */
    if (ret == 0) {
        printf("❌ 应该拒绝超出范围的增益值！\n");
        return -1;
    }
    printf("✓ 正确拒绝了超出范围的增益值 (2.0)\n");
    
    printf("\n✅ Test 1 通过\n");
    return 0;
}

/**
 * @brief 测试 2: 获取和设置转向零偏
 */
int test_steering_offset_access(void) {
    printf("\n══════════════════════════════════════════════════\n");
    printf("Test 2: 转向零偏获取和设置\n");
    printf("══════════════════════════════════════════════════\n");
    
    SteeringModel_t steering_model = {0};
    SteeringConfig_t config = {
        .steering_ratio = 15.3f,
        .max_steering_wheel_angle = 8.79645943f,
        .max_front_wheel_angle = 0.676454711f,
        .wheelbase = 2.76f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .enable_steering_filter = false,
        .steering_filter_alpha = 0.1f,
        .steering_deadzone = 0.02f,
    };
    
    if (steering_config_init(&steering_model, &config) != 0) {
        printf("❌ 初始化转向模型失败\n");
        return -1;
    }
    
    /* 测试初始零偏 */
    float offset = steering_model_get_offset(&steering_model);
    printf("✓ 初始零偏: %.6f rad\n", offset);
    if (fabsf(offset) > 1e-6f) {
        printf("❌ 初始零偏错误！期望 0.0\n");
        return -1;
    }
    
    /* 直接修改零偏（通过配置结构体） */
    steering_model.steering_config.steering_angle_offset = 0.05f;
    
    offset = steering_model_get_offset(&steering_model);
    printf("✓ 更新后零偏: %.6f rad\n", offset);
    if (fabsf(offset - 0.05f) > 1e-6f) {
        printf("❌ 更新零偏错误！期望 0.05\n");
        return -1;
    }
    
    printf("\n✅ Test 2 通过\n");
    return 0;
}

/**
 * @brief 测试 3: 应用 RTK 校准参数
 */
int test_apply_rtk_calibration(void) {
    printf("\n══════════════════════════════════════════════════\n");
    printf("Test 3: 应用 RTK 校准参数\n");
    printf("══════════════════════════════════════════════════\n");
    
    SteeringModel_t steering_model = {0};
    SteeringConfig_t config = {
        .steering_ratio = 15.3f,
        .max_steering_wheel_angle = 8.79645943f,
        .max_front_wheel_angle = 0.676454711f,
        .wheelbase = 2.76f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .enable_steering_filter = false,
        .steering_filter_alpha = 0.1f,
        .steering_deadzone = 0.02f,
    };
    
    if (steering_config_init(&steering_model, &config) != 0) {
        printf("❌ 初始化转向模型失败\n");
        return -1;
    }
    
    /* 创建 RTK 校准参数 */
    CalibrationParams_t calibration = {
        .wheel_calibration_factor = 1.0f,
        .steering_angle_offset = 0.03f,      /* 3 度约 0.052 rad */
        .steering_angle_gain = 1.08f,        /* 增益 8% */
        .imu_accel_bias_x = 0.0f,
        .imu_accel_bias_y = 0.0f,
        .imu_gyro_bias_z = 0.0f,
        .imu_accel_scale_x = 1.0f,
        .imu_accel_scale_y = 1.0f,
        .imu_lateral_accel_bias = 0.0f,
        .imu_lateral_accel_scale = 1.0f,
    };
    
    printf("应用校准参数:\n");
    printf("  零偏: %.6f rad\n", calibration.steering_angle_offset);
    printf("  增益: %.4f\n", calibration.steering_angle_gain);
    
    /* 应用校准 */
    int ret = steering_model_apply_rtk_calibration(&steering_model, &calibration);
    if (ret != 0) {
        printf("❌ 应用 RTK 校准失败\n");
        return -1;
    }
    
    /* 验证参数已应用 */
    float offset = steering_model_get_offset(&steering_model);
    float gain = steering_model_get_gain(&steering_model);
    
    printf("✓ 应用后零偏: %.6f rad\n", offset);
    printf("✓ 应用后增益: %.4f\n", gain);
    
    if (fabsf(offset - 0.03f) > 1e-6f) {
        printf("❌ 零偏应用错误！期望 0.03\n");
        return -1;
    }
    
    if (fabsf(gain - 1.08f) > 1e-6f) {
        printf("❌ 增益应用错误！期望 1.08\n");
        return -1;
    }
    
    printf("\n✅ Test 3 通过\n");
    return 0;
}

/**
 * @brief 测试 4: 校准参数应用效果验证
 */
int test_calibration_effect(void) {
    printf("\n══════════════════════════════════════════════════\n");
    printf("Test 4: 校准参数应用效果\n");
    printf("══════════════════════════════════════════════════\n");
    
    SteeringModel_t steering_model = {0};
    SteeringConfig_t config = {
        .steering_ratio = 15.3f,
        .max_steering_wheel_angle = 8.79645943f,
        .max_front_wheel_angle = 0.676454711f,
        .wheelbase = 2.76f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .enable_steering_filter = false,
        .steering_filter_alpha = 0.1f,
        .steering_deadzone = 0.02f,
    };
    
    if (steering_config_init(&steering_model, &config) != 0) {
        printf("❌ 初始化转向模型失败\n");
        return -1;
    }
    
    /* 测试未校准状态 */
    printf("\n未校准状态:\n");
    float angle_no_calib = steering_wheel_to_front_wheel(&steering_model, 0.1f);
    printf("  方向盘转角 0.1 rad → 前轮转角 %.6f rad\n", angle_no_calib);
    
    /* 应用校准 */
    CalibrationParams_t calibration = {
        .wheel_calibration_factor = 1.0f,
        .steering_angle_offset = 0.02f,
        .steering_angle_gain = 1.1f,
        .imu_accel_bias_x = 0.0f,
        .imu_accel_bias_y = 0.0f,
        .imu_gyro_bias_z = 0.0f,
        .imu_accel_scale_x = 1.0f,
        .imu_accel_scale_y = 1.0f,
        .imu_lateral_accel_bias = 0.0f,
        .imu_lateral_accel_scale = 1.0f,
    };
    
    steering_model_apply_rtk_calibration(&steering_model, &calibration);
    
    /* 测试校准后状态 */
    printf("\n校准后状态 (增益 1.1, 零偏 0.02 rad):\n");
    float angle_with_calib = steering_wheel_to_front_wheel(&steering_model, 0.1f);
    printf("  方向盘转角 0.1 rad → 前轮转角 %.6f rad\n", angle_with_calib);
    
    /* 验证校准有效果 */
    if (fabsf(angle_no_calib - angle_with_calib) < 1e-6f) {
        printf("❌ 校准没有实际效果！\n");
        return -1;
    }
    
    printf("✓ 校准已正确应用（前轮转角差异: %.6f rad）\n", 
           fabs(angle_with_calib - angle_no_calib));
    
    printf("\n✅ Test 4 通过\n");
    return 0;
}

int main(void) {
    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║   Task 4: 转向模型 RTK 校准集成测试              ║\n");
    printf("║   测试转向增益和零偏的集成功能                    ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    
    int pass_count = 0;
    int fail_count = 0;
    
    if (test_steering_gain_access() == 0) {
        pass_count++;
    } else {
        fail_count++;
    }
    
    if (test_steering_offset_access() == 0) {
        pass_count++;
    } else {
        fail_count++;
    }
    
    if (test_apply_rtk_calibration() == 0) {
        pass_count++;
    } else {
        fail_count++;
    }
    
    if (test_calibration_effect() == 0) {
        pass_count++;
    } else {
        fail_count++;
    }
    
    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║                   测试总结                         ║\n");
    printf("╚════════════════════════════════════════════════════╝\n");
    printf("✅ 通过: %d/4\n", pass_count);
    printf("❌ 失败: %d/4\n", fail_count);
    
    if (fail_count == 0) {
        printf("\n🎉 所有测试通过！Task 4 集成成功。\n");
        return 0;
    } else {
        printf("\n⚠️  有 %d 个测试失败。\n", fail_count);
        return -1;
    }
}
