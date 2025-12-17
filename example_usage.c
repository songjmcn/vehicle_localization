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
               RAD_TO_DEG(state->theta),
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

int main() {
    printf("=== Vehicle Localization System Example ===\n");
    printf("Scenario: Automatic parking with J-shaped trajectory\n");
    printf("Stage 1: Go straight 100m at 1 m/s\n");
    printf("Stage 2: Turn left and reverse 30m at 0.5 m/s\n");
    printf("Stage 3: Stop\n\n");
    
    /* 初始化定位系统 */
    LocalizationSystem_t localization_sys;
    VehicleState_t initial_state = {
        .x = 0.0,
        .y = 0.0,
        .theta = 0.0,
        .v = 0.0,
        .delta = 0.0,
        .timestamp = 1000
    };
    
    if (localization_system_init(&localization_sys, &initial_state) != 0) {
        printf("ERROR: Failed to initialize localization system!\n");
        return -1;
    }
    
    printf("✓ Localization system initialized\n\n");
    
    /* ===== 可选：配置轮速脉冲处理器 ===== */
    /* 
     * 如果硬件提供脉冲计数而非直接速度值，需要先配置脉冲处理器。
     * 此示例展示两种使用方式：
     * 方式1：传入脉冲计数，由处理器转换为速度
     * 方式2：直接传入速度值（使用默认配置）
     */
    WheelConfig_t wheel_config = {
        .pulses_per_wheel_rev = 400,    /* 码盘：400脉冲/圈 */
        .wheel_radius = 0.35,           /* 轮半径：0.35米 */
        .low_speed_threshold = 0.3,     /* 低速平滑阈值 */
        .pulse_timeout_ms = 500         /* 脉冲超时检测 */
    };
    
    if (localization_system_set_wheel_config(&localization_sys, &wheel_config) != 0) {
        printf("WARNING: Failed to configure wheel processor\n");
    } else {
        printf("✓ Wheel odometry processor configured\n");
        printf("  - Pulses per revolution: %u\n", wheel_config.pulses_per_wheel_rev);
        printf("  - Wheel radius: %.2f m\n", wheel_config.wheel_radius);
        printf("  - Low speed threshold: %.2f m/s\n\n", wheel_config.low_speed_threshold);
    }
    
    /* 运行定位循环（2600步 = 130秒）*/
    printf("Running localization process...\n");
    printf("───────────────────────────────────────────────────────\n");
    
    /* 脉冲计数模拟变量 */
    uint32_t accumulated_pulses = 0;
    (void)accumulated_pulses;  /* 此示例中未使用，保留供用户参考 */
    double wheel_circumference = 2.0 * M_PI * wheel_config.wheel_radius;
    
    for (uint32_t step = 0; step < 2600; step++) {
        /* 生成传感器数据 */
        SensorReadings_t sensor_data;
        memset(&sensor_data, 0, sizeof(sensor_data));
        simulate_sensor_data(step, &sensor_data);
        
        /* ===== 演示：脉冲处理 ===== */
        /* 
         * 实际应用中，用户会从硬件读取脉冲计数，例如：
         *   uint32_t pulse_count = read_pulse_from_hardware();
         * 
         * 这里我们基于模拟的速度数据来反推脉冲计数用于演示
         */
        uint32_t dt_ms = 50;  /* 50ms时间间隔 */
        double distance_delta = sensor_data.wheel_odometry.velocity * (dt_ms / 1000.0);
        uint32_t pulse_delta = (uint32_t)(distance_delta / wheel_circumference * wheel_config.pulses_per_wheel_rev);
        accumulated_pulses += pulse_delta;
        
        uint32_t timestamp = 1000 + step * dt_ms;
        
        /* 方式1：使用脉冲处理（如果硬件提供脉冲计数）
         * localization_system_update_wheel_pulse(&localization_sys, accumulated_pulses, timestamp);
         */
        
        /* 方式2：直接使用速度值（如果硬件已转换为速度，或为简化示例）
         * 这样处理器仍会应用低速平滑和异常检测
         */
        localization_system_update_wheel_velocity(&localization_sys, 
                                                  sensor_data.wheel_odometry.velocity, 
                                                  timestamp);
        
        /* 从处理器获取处理后的轮速数据 */
        WheelOdometryData_t processed_odometry;
        if (localization_system_get_wheel_odometry(&localization_sys, &processed_odometry) == 0) {
            /* 使用处理后的数据更新sensor_data */
            sensor_data.wheel_odometry = processed_odometry;
        }
        
        /* 执行定位 */
        VehicleState_t estimated_state;
        if (localization_system_process(&localization_sys, &sensor_data, &estimated_state) != 0) {
            printf("ERROR: Localization process failed at step %u\n", step);
            continue;
        }
        
        /* 获取诊断信息 */
        DiagnosticsInfo_t diag = localization_system_get_diagnostics(&localization_sys);
        
        /* 打印结果 */
        print_localization_result(step, &estimated_state, &diag);
    }
    
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
    printf("  Heading: %.2f deg\n", RAD_TO_DEG(final_state.theta));
    printf("  Velocity: %.2f m/s\n", final_state.v);
    printf("  Steering angle: %.2f deg\n", RAD_TO_DEG(final_state.delta));
    
    /* 释放资源 */
    localization_system_free(&localization_sys);
    printf("\n✓ System cleanup completed\n");
    
    return 0;
}
