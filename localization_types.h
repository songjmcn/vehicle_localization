/**
 * @file localization_types.h
 * @brief 车辆定位系统的数据类型定义
 * @author Vehicle Localization System
 */

#ifndef LOCALIZATION_TYPES_H
#define LOCALIZATION_TYPES_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ========== 常数定义 ========== */

#define PI 3.141592653589793
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)
#define WHEELBASE 2.7  /* 轴距 (m) */
#define MAX_HISTORY_SIZE 100

/* 速度阈值 */
#define V_STATIC_THRESHOLD 0.05  /* 静止速度阈值 (m/s) */
#define V_LOW_SPEED_THRESHOLD 1.5  /* 低速阈值 (m/s) */
#define V_PARKING_MAX 2.0  /* 自动泊车最大速度 (m/s) */

/* 传感器阈值 */
#define WHEEL_SLIP_ACCEL_THRESHOLD 0.3  /* 轮滑动加速度阈值 (m/s²) */
#define STEERING_ANGLE_MAX 0.628  /* 最大转向角 (36°，约0.628 rad) */

/* ========== 数据结构定义 ========== */

/**
 * @brief 车辆状态结构体
 */
typedef struct {
    double x;      /**< x位置 (m) */
    double y;      /**< y位置 (m) */
    double theta;  /**< 航向角 (rad) */
    double v;      /**< 速度 (m/s) */
    double delta;  /**< 转向角 (rad) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} VehicleState_t;

/**
 * @brief IMU数据结构体
 */
typedef struct {
    double accel_x;   /**< x轴加速度 (m/s²) */
    double accel_y;   /**< y轴加速度 (m/s²) */
    double accel_z;   /**< z轴加速度 (m/s²) */
    double gyro_x;    /**< x轴角速度 (rad/s) */
    double gyro_y;    /**< y轴角速度 (rad/s) */
    double gyro_z;    /**< z轴角速度 (rad/s) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} IMUData_t;

/**
 * @brief 轮速计数据结构体
 */
typedef struct {
    double velocity;   /**< 速度 (m/s) */
    double distance;   /**< 积分距离 (m) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} WheelOdometryData_t;

/**
 * @brief 转向角数据结构体
 */
typedef struct {
    double steering_angle;  /**< 转向角 (rad) */
    double steering_rate;   /**< 转向角速率 (rad/s) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
} SteeringData_t;

/**
 * @brief RTK定位数据结构体（标定用）
 */
typedef struct {
    double x;      /**< RTK x位置 (m) */
    double y;      /**< RTK y位置 (m) */
    double z;      /**< 高度 (m) */
    double heading;  /**< RTK航向角 (rad) */
    uint32_t timestamp;  /**< 时间戳 (ms) */
    uint8_t quality;  /**< 定位质量 (0-5, 5最优) */
} RTKData_t;

/**
 * @brief 传感器读数融合结构体
 */
typedef struct {
    IMUData_t imu;
    WheelOdometryData_t wheel_odometry;
    SteeringData_t steering;
    uint32_t timestamp;  /**< 融合时间戳 */
} SensorReadings_t;

/**
 * @brief 噪声参数结构体
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
} NoiseParameters_t;

/**
 * @brief 传感器校准参数结构体
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
} CalibrationParams_t;

/**
 * @brief RTK标定结果结构体
 */
typedef struct {
    VehicleState_t odometry_state;  /**< 里程计状态 */
    RTKData_t rtk_state;             /**< RTK状态 */
    double position_error_x;         /**< x位置误差 */
    double position_error_y;         /**< y位置误差 */
    double heading_error;            /**< 航向角误差 */
    uint32_t calibration_count;      /**< 标定点数 */
} RTKCalibrationPoint_t;

/**
 * @brief 滤波器状态结构体
 */
typedef struct {
    double x_hat[5];        /**< 状态估计 [x, y, theta, v, delta] */
    double P[5][5];         /**< 协方差矩阵 5x5 */
    uint32_t previous_timestamp;  /**< 上一时间戳 */
} FilterState_t;

/**
 * @brief 诊断信息结构体
 */
typedef struct {
    bool is_wheel_slipping;   /**< 轮滑动标志 */
    bool is_stationary;       /**< 静止标志 */
    bool is_high_accel;       /**< 高加速度标志 */
    double wheel_slip_accel;  /**< 轮滑动加速度 */
    double residual_norm;     /**< 残差范数 */
    uint32_t update_count;    /**< 更新计数 */
} DiagnosticsInfo_t;

/**
 * @brief 转向角抖动过滤器结构体
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
} SteeringJitterFilter_t;

/**
 * @brief 角度规范化函数
 * @param angle 输入角度 (rad)
 * @return 规范化后的角度 [-PI, PI]
 */
static inline double normalize_angle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

/**
 * @brief 符号函数
 * @param x 输入值
 * @return -1.0, 0.0 或 1.0
 */
static inline double sign_func(double x) {
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
static inline double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#endif /* LOCALIZATION_TYPES_H */
