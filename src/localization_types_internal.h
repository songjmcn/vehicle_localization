/**
 * @file localization_types_internal.h
 * @brief 车辆定位系统的内部数据类型定义 (Double 精度版本，用于KF核心计算)
 * @author Vehicle Localization System
 * 
 * 本文件定义内部使用的 double 精度数据结构，用于卡尔曼滤波器的
 * 矩阵运算和数值计算，以确保高精度和数值稳定性。
 * 
 * 与 localization_types.h 的对应关系：
 * - 公共 API：使用 float 版本（localization_types.h）
 * - 内部计算：使用 double 版本（本文件）
 */

#ifndef LOCALIZATION_TYPES_INTERNAL_H
#define LOCALIZATION_TYPES_INTERNAL_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "localization_types.h"  /* 包含公共枚举定义 */

/* ========== 常数定义 ========== */

#define PI 3.141592653589793
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)
#define WHEELBASE 2.7  /* 轴距 (m) */
#define MAX_HISTORY_SIZE 100

/* 速度阈值 */
#define V_STATIC_THRESHOLD 0.1  /* 静止速度阈值 (m/s), 优化为float精度推荐值 */
#define V_LOW_SPEED_THRESHOLD 1.5  /* 低速阈值 (m/s) */
#define V_PARKING_MAX 2.0  /* 自动泊车最大速度 (m/s) */

/* ========== 内部数据结构定义 (Double 精度) ========== */

/**
 * @brief 车辆状态结构体 (内部 Double 版本)
 * @note 外部 API 使用 VehicleState_t (float版本)
 */
typedef struct {
    double x;      /**< x位置 (m) */
    double y;      /**< y位置 (m) */
    double theta;  /**< 航向角 (rad) */
    double v;      /**< 速度 (m/s) */
    double delta;  /**< 转向角 (rad) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
    VehicleGear_t gear;  /**< 车辆档位 */
    VehicleDirection_t direction;  /**< 推导的运动方向 */
} VehicleState_internal_t;

/**
 * @brief IMU数据结构体 (内部 Double 版本)
 * @note 外部 API 使用 IMUData_t (float版本)
 */
typedef struct {
    double accel_x;   /**< x轴加速度 (m/s²) */
    double accel_y;   /**< y轴加速度 (m/s²) */
    double accel_z;   /**< z轴加速度 (m/s²) */
    double gyro_x;    /**< x轴角速度 (rad/s) */
    double gyro_y;    /**< y轴角速度 (rad/s) */
    double gyro_z;    /**< z轴角速度 (rad/s) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} IMUData_internal_t;

/**
 * @brief 轮速计数据结构体 (内部 Double 版本)
 * @note 外部 API 使用 WheelOdometryData_t (float版本)
 */
typedef struct {
    double velocity;   /**< 速度 (m/s) */
    double distance;   /**< 积分距离 (m) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} WheelOdometryData_internal_t;

/**
 * @brief 转向角数据结构体 (内部 Double 版本)
 * @note 外部 API 使用 SteeringData_t (float版本)
 */
typedef struct {
    double steering_angle;  /**< 转向角 (rad) */
    double steering_rate;   /**< 转向角速率 (rad/s) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} SteeringData_internal_t;

/**
 * @brief RTK定位数据结构体 (内部 Double 版本)
 * @note 外部 API 使用 RTKData_t (float版本)
 */
typedef struct {
    double x;      /**< RTK x位置 (m) */
    double y;      /**< RTK y位置 (m) */
    double z;      /**< 高度 (m) */
    double heading;  /**< RTK航向角 (rad) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
    uint8_t quality;  /**< 定位质量 (0-5, 5最优) */
} RTKData_internal_t;

/**
 * @brief 噪声参数结构体 (内部 Double 版本)
 * @note 外部 API 使用 NoiseParameters_t (float版本)
 */
typedef struct {
    double q_x;        /**< 位置x噪声 */
    double q_y;        /**< 位置y噪声 */
    double q_theta;    /**< 角度噪声 */
    double q_v;        /**< 速度噪声 */
    double q_delta;    /**< 转向角噪声 */
    double r_wheel;    /**< 轮速计测量噪声 */
    double r_imu;      /**< IMU加速度测量噪声 */
    double r_steering; /**< 转向角测量噪声 */
} NoiseParameters_internal_t;

/**
 * @brief 传感器校准参数结构体 (内部 Double 版本)
 * @note 外部 API 使用 CalibrationParams_t (float版本)
 */
typedef struct {
    double wheel_calibration_factor;  /**< 轮速计校准因子 */
    double steering_angle_offset;     /**< 转向角零偏 (rad) */
    double steering_angle_gain;       /**< 转向角增益 */
    double imu_accel_bias_x;          /**< IMU x轴加速度零偏 */
    double imu_accel_bias_y;          /**< IMU y轴加速度零偏 */
    double imu_gyro_bias_z;           /**< IMU z轴陀螺仪零偏 */
    double imu_accel_scale_x;         /**< IMU x轴加速度尺度因子 */
    double imu_accel_scale_y;         /**< IMU y轴加速度尺度因子 */
} CalibrationParams_internal_t;

/**
 * @brief 传感器读数融合结构体 (内部 Double 版本)
 */
typedef struct {
    IMUData_internal_t imu;
    WheelOdometryData_internal_t wheel_odometry;
    SteeringData_internal_t steering;
    uint32_t timestamp;  /**< 融合时间戳 */
} SensorReadings_internal_t;

/**
 * @brief 滤波器状态结构体 (内部存储，必须使用 Double)
 * 
 * @note **关键**：以下字段必须使用 double 精度：
 *   - x_hat[5]：状态估计向量，长期运行时累积误差需要高精度
 *   - P[5][5]：协方差矩阵，矩阵求逆需要高精度以保证数值稳定性
 *   - 所有矩阵操作：务必保留 double，以确保卡尔曼增益计算的可靠性
 */
typedef struct {
    double x_hat[5];        /**< 状态估计 [x, y, theta, v, delta] - 必须保留 Double */
    double P[5][5];         /**< 协方差矩阵 5x5 - **关键**：必须保留 Double */
    uint32_t previous_timestamp;  /**< 上一时间戳 */
} FilterState_internal_t;

/**
 * @brief 诊断信息结构体 (内部使用)
 */
typedef struct {
    bool is_wheel_slipping;   /**< 轮滑动标志 */
    bool is_stationary;       /**< 静止标志 */
    bool is_high_accel;       /**< 高加速度标志 */
    double wheel_slip_accel;  /**< 轮滑动加速度 */
    double residual_norm;     /**< 残差范数 */
    uint32_t update_count;    /**< 更新计数 */
} DiagnosticsInfo_internal_t;

/**
 * @brief 转向角抖动过滤器结构体 (内部使用 Double 精度)
 * 用于过滤转向角传感器的高频噪声，计算零偏角以改进航向估计
 */
typedef struct {
    /* 历史缓冲区 */
    double angle_history[10];      /**< 转向角历史值 */
    uint32_t history_index;        /**< 当前缓冲位置 */
    uint32_t history_count;        /**< 有效历史数据点数 */
    
    /* 统计量 */
    double mean_angle;             /**< 转向角均值 */
    double angle_variance;         /**< 转向角方差 */
    double zero_bias_angle;        /**< 零偏角估计 */
    
    /* 过滤参数 */
    double jitter_threshold;       /**< 抖动阈值 (rad) */
    double filter_alpha;           /**< 低通滤波系数 [0,1] */
    double filtered_angle;         /**< 低通过滤后的角度 */
    
    /* 诊断 */
    uint32_t update_count;         /**< 更新计数 */
    bool is_high_jitter;           /**< 高抖动标志 */
} SteeringJitterFilter_internal_t;

/**
 * @brief 角度规范化函数
 * @param angle 输入角度 (rad)
 * @return 规范化后的角度 [-PI, PI]
 */
static inline double normalize_angle_internal(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

/**
 * @brief 符号函数
 * @param x 输入值
 * @return -1.0, 0.0 或 1.0
 */
static inline double sign_func_internal(double x) {
    if (x > 1e-6) return 1.0;
    if (x < -1e-6) return -1.0;
    return 0.0;
}

/**
 * @brief 限制值在范围内
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限制后的值
 */
static inline double clamp_internal(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#endif /* LOCALIZATION_TYPES_INTERNAL_H */
