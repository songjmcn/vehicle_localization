/**
 * @file localization_adapter.h
 * @brief Float 和 Double 数据结构之间的转换适配层
 * @author Vehicle Localization System
 * 
 * 本文件提供公共 API (float) 和内部实现 (double) 之间的自动转换函数。
 * 支持方案 B 的混合精度实现：外部 float API + 内部 double 计算。
 */

#ifndef LOCALIZATION_ADAPTER_H
#define LOCALIZATION_ADAPTER_H

#include "localization_types.h"
#include "localization_types_internal.h"

/* ========== 数据转换函数 ========== */

/**
 * @brief 将外部 VehicleState_t (float) 转换为内部版本 (double)
 * @param[in] external 外部状态 (float)
 * @param[out] internal 内部状态 (double)
 */
static inline void vehicle_state_float_to_internal(
    const VehicleState_t *external,
    VehicleState_internal_t *internal)
{
    internal->x = (double)external->x;
    internal->y = (double)external->y;
    internal->theta = (double)external->theta;
    internal->v = (double)external->v;
    internal->delta = (double)external->delta;
    internal->timestamp = external->timestamp;
    internal->gear = external->gear;
    internal->direction = external->direction;
}

/**
 * @brief 将内部 VehicleState_internal_t (double) 转换为外部版本 (float)
 * @param[in] internal 内部状态 (double)
 * @param[out] external 外部状态 (float)
 */
static inline void vehicle_state_internal_to_float(
    const VehicleState_internal_t *internal,
    VehicleState_t *external)
{
    external->x = (float)internal->x;
    external->y = (float)internal->y;
    external->theta = (float)internal->theta;
    external->v = (float)internal->v;
    external->delta = (float)internal->delta;
    external->timestamp = internal->timestamp;
    external->gear = internal->gear;
    external->direction = internal->direction;
}

/**
 * @brief 将外部 IMUData_t (float) 转换为内部版本 (double)
 * @param[in] external 外部 IMU 数据 (float)
 * @param[out] internal 内部 IMU 数据 (double)
 */
static inline void imu_data_float_to_internal(
    const IMUData_t *external,
    IMUData_internal_t *internal)
{
    internal->accel_x = (double)external->accel_x;
    internal->accel_y = (double)external->accel_y;
    internal->accel_z = (double)external->accel_z;
    internal->gyro_x = (double)external->gyro_x;
    internal->gyro_y = (double)external->gyro_y;
    internal->gyro_z = (double)external->gyro_z;
    internal->timestamp = external->timestamp;
}

/**
 * @brief 将内部 IMUData_internal_t (double) 转换为外部版本 (float)
 * @param[in] internal 内部 IMU 数据 (double)
 * @param[out] external 外部 IMU 数据 (float)
 */
static inline void imu_data_internal_to_float(
    const IMUData_internal_t *internal,
    IMUData_t *external)
{
    external->accel_x = (float)internal->accel_x;
    external->accel_y = (float)internal->accel_y;
    external->accel_z = (float)internal->accel_z;
    external->gyro_x = (float)internal->gyro_x;
    external->gyro_y = (float)internal->gyro_y;
    external->gyro_z = (float)internal->gyro_z;
    external->timestamp = internal->timestamp;
}

/**
 * @brief 将外部 WheelOdometryData_t (float) 转换为内部版本 (double)
 * @param[in] external 外部轮速数据 (float)
 * @param[out] internal 内部轮速数据 (double)
 */
static inline void wheel_odometry_float_to_internal(
    const WheelOdometryData_t *external,
    WheelOdometryData_internal_t *internal)
{
    internal->velocity = (double)external->velocity;
    internal->distance = (double)external->distance;
    internal->timestamp = external->timestamp;
}

/**
 * @brief 将外部 SteeringData_t (float) 转换为内部版本 (double)
 * @param[in] external 外部转向数据 (float)
 * @param[out] internal 内部转向数据 (double)
 */
static inline void steering_data_float_to_internal(
    const SteeringData_t *external,
    SteeringData_internal_t *internal)
{
    internal->steering_angle = (double)external->steering_angle;
    internal->steering_rate = (double)external->steering_rate;
    internal->timestamp = external->timestamp;
}

/**
 * @brief 将外部 NoiseParameters_t (float) 转换为内部版本 (double)
 * @param[in] external 外部噪声参数 (float)
 * @param[out] internal 内部噪声参数 (double)
 */
static inline void noise_parameters_float_to_internal(
    const NoiseParameters_t *external,
    NoiseParameters_internal_t *internal)
{
    internal->q_x = (double)external->q_x;
    internal->q_y = (double)external->q_y;
    internal->q_theta = (double)external->q_theta;
    internal->q_v = (double)external->q_v;
    internal->q_delta = (double)external->q_delta;
    internal->r_wheel = (double)external->r_wheel;
    internal->r_imu = (double)external->r_imu;
    internal->r_steering = (double)external->r_steering;
}

/**
 * @brief 将内部 NoiseParameters_internal_t (double) 转换为外部版本 (float)
 * @param[in] internal 内部噪声参数 (double)
 * @param[out] external 外部噪声参数 (float)
 */
static inline void noise_parameters_internal_to_float(
    const NoiseParameters_internal_t *internal,
    NoiseParameters_t *external)
{
    external->q_x = (float)internal->q_x;
    external->q_y = (float)internal->q_y;
    external->q_theta = (float)internal->q_theta;
    external->q_v = (float)internal->q_v;
    external->q_delta = (float)internal->q_delta;
    external->r_wheel = (float)internal->r_wheel;
    external->r_imu = (float)internal->r_imu;
    external->r_steering = (float)internal->r_steering;
}

/**
 * @brief 将外部 CalibrationParams_t (float) 转换为内部版本 (double)
 * @param[in] external 外部校准参数 (float)
 * @param[out] internal 内部校准参数 (double)
 */
static inline void calibration_params_float_to_internal(
    const CalibrationParams_t *external,
    CalibrationParams_internal_t *internal)
{
    internal->wheel_calibration_factor = (double)external->wheel_calibration_factor;
    internal->steering_angle_offset = (double)external->steering_angle_offset;
    internal->steering_angle_gain = (double)external->steering_angle_gain;
    internal->imu_accel_bias_x = (double)external->imu_accel_bias_x;
    internal->imu_accel_bias_y = (double)external->imu_accel_bias_y;
    internal->imu_gyro_bias_z = (double)external->imu_gyro_bias_z;
    internal->imu_accel_scale_x = (double)external->imu_accel_scale_x;
    internal->imu_accel_scale_y = (double)external->imu_accel_scale_y;
}

/**
 * @brief 将内部 CalibrationParams_internal_t (double) 转换为外部版本 (float)
 * @param[in] internal 内部校准参数 (double)
 * @param[out] external 外部校准参数 (float)
 */
static inline void calibration_params_internal_to_float(
    const CalibrationParams_internal_t *internal,
    CalibrationParams_t *external)
{
    external->wheel_calibration_factor = (float)internal->wheel_calibration_factor;
    external->steering_angle_offset = (float)internal->steering_angle_offset;
    external->steering_angle_gain = (float)internal->steering_angle_gain;
    external->imu_accel_bias_x = (float)internal->imu_accel_bias_x;
    external->imu_accel_bias_y = (float)internal->imu_accel_bias_y;
    external->imu_gyro_bias_z = (float)internal->imu_gyro_bias_z;
    external->imu_accel_scale_x = (float)internal->imu_accel_scale_x;
    external->imu_accel_scale_y = (float)internal->imu_accel_scale_y;
}

/**
 * @brief 将外部完整传感器读数 (float) 转换为内部版本 (double)
 * @param[in] external 外部传感器读数 (float)
 * @param[out] internal 内部传感器读数 (double)
 */
static inline void sensor_readings_float_to_internal(
    const SensorReadings_t *external,
    SensorReadings_internal_t *internal)
{
    imu_data_float_to_internal(&external->imu, &internal->imu);
    wheel_odometry_float_to_internal(&external->wheel_odometry, &internal->wheel_odometry);
    steering_data_float_to_internal(&external->steering, &internal->steering);
    internal->timestamp = external->timestamp;
}

#endif /* LOCALIZATION_ADAPTER_H */
