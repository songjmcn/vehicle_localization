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
#define V_STATIC_THRESHOLD 0.05 /* 静止速度阈值 (m/s), 优化为float精度推荐值 */
#define V_LOW_SPEED_THRESHOLD 1.5  /* 低速阈值 (m/s) */
#define V_PARKING_MAX 2.0  /* 自动泊车最大速度 (m/s) */

/* 传感器阈值 */
#define WHEEL_SLIP_ACCEL_THRESHOLD 0.3  /* 轮滑动加速度阈值 (m/s²) */

/* ========== 融合算法配置 ========== */

/**
 * @brief 融合算法模式枚举
 * 支持不同的多传感器融合策略
 */
typedef enum {
    FUSION_MODE_BASIC = 0,          /**< 基础算法：车轮转角+轮速+IMU加速度(3维观测) */
    FUSION_MODE_IMU_STEERING = 1    /**< 增强算法：融合IMU推导转向角(4维观测) */
} FusionMode_t;

/**
 * @brief 融合算法配置结构体
 * 用于选择和配置不同的融合策略
 */
typedef struct {
    FusionMode_t mode;                      /**< 当前融合模式 */
    float imu_steering_vel_threshold;       /**< IMU转向推导速度下限 (m/s)，低于此值禁用 */
    float imu_steering_weight;              /**< IMU转向融合权重 [0,1] */
    bool enable_adaptive_weighting;         /**< 是否启用自适应权重调整 */
} FusionConfig_t;

/* ========== 档位和方向枚举定义 ========== */

/**
 * @brief 车辆档位枚举
 * 用于准确判断车辆运动方向
 */
typedef enum {
    GEAR_PARK = 0,        /**< P档（停止） */
    GEAR_REVERSE = 1,     /**< R档（后退） */
    GEAR_NEUTRAL = 2,     /**< N档（空档） */
    GEAR_DRIVE = 3,       /**< D档（前进） */
    GEAR_LOW = 4,         /**< L档（低速档） */
    GEAR_INVALID = -1     /**< 无效档位 */
} VehicleGear_t;
typedef enum{
    WHEEL_DIRECRTION_INVALID = 0, /**< 无效方向 */
    WHEEL_DIRECTION_FORWARD = 1,  /**< 车轮前进 */
    WHEEL_DIRECTION_REVERSE = 2,   /**< 车轮后退 */
    WHEEL_DIRECTION_STOP = 3,
}WheelDirection_t;
/**
 * @brief 车辆运动方向枚举（派生值）
 * 从档位和速度综合计算而来
 */
typedef enum {
    DIRECTION_STATIONARY = 0,    /**< 静止（或速度近似为0） */
    DIRECTION_FORWARD = 1,       /**< 前进方向 */
    DIRECTION_REVERSE = -1,      /**< 后退方向 */
    DIRECTION_INVALID = 2        /**< 无效/错误方向（传感器故障或逻辑冲突） */
} VehicleDirection_t;

/* ========== 数据结构定义 ========== */

/**
 * @brief 车辆状态结构体 (外部 API 使用 float 版本)
 */
typedef struct {
    float x;              /**< 后轮轴中心X位置(m) */
    float y;              /**< 后轮轴中心Y位置(m) */
    float heading;        /**< 偏航角(rad) */
    float v;              /**< 后轮纵向速度(m/s) */
    float steering_angle; /**< 前轮转向角(rad)，后退时反向 */
    uint64_t timestamp;  /**< 时间戳 (ms) */
    VehicleGear_t gear;  /**< 车辆档位 */
    VehicleDirection_t direction;  /**< 推导的运动方向 */
} VehicleState_t;

/**
 * @brief IMU数据结构体 (外部 API 使用 float 版本)
 */
typedef struct {
    float accel_x;   /**< x轴加速度 (m/s²) */
    float accel_y;   /**< y轴加速度 (m/s²) */
    float accel_z;   /**< z轴加速度 (m/s²) */
    float gyro_x;    /**< x轴角速度 (rad/s) */
    float gyro_y;    /**< y轴角速度 (rad/s) */
    float gyro_z;    /**< z轴角速度 (rad/s) */
    uint64_t timestamp;  /**< 时间戳 (ms) */
} IMUData_t;

/**
 * @brief 轮速计码盘配置（脉冲处理用）
 */
typedef struct {
    uint32_t pulses_per_wheel_rev;  /**< 每圈脉冲数（如400） */
    double wheel_radius;             /**< 轮半径 (m)，如 0.35 */
    double low_speed_threshold;      /**< 低速阈值 (m/s)，默认 0.3 */
    uint32_t pulse_timeout_ms;       /**< 脉冲超时 (ms)，默认 500 */
    uint32_t max_count;              /**< 脉冲计数器最大值+1，用于处理溢出（如65535时设为65536）。0表示不处理溢出 */
} WheelConfig_t;

/**
 * @brief 原始脉冲数据（外部输入）
 */
typedef struct {
    uint32_t pulse_count;    /**< 脉冲计数（累计或增量） */
    uint64_t timestamp;      /**< 时间戳 (ms) */
} WheelPulseData_t;

/**
 * @brief 轮速计数据结构体 (外部 API 使用 float 版本)
 * 
 * 包含轮速处理后的速度、距离以及左右轮的运动方向状态
 */
typedef struct {
    float velocity;                        /**< 速度 (m/s) */
    float distance;                        /**< 积分距离 (m) */
    uint64_t timestamp;                    /**< 时间戳 (ms) */
    VehicleGear_t gear;                    /**< 档位（从传感器数据传入） */
    WheelDirection_t rear_left_direction;  /**< 左后轮运动方向 */
    WheelDirection_t rear_right_direction; /**< 右后轮运动方向 */
} WheelOdometryData_t;

/**
 * @brief 转向角数据结构体 (外部 API 使用 float 版本)
 */
typedef struct {
    float steering_angle;  /**< 方向盘转角 (rad) */
    float steering_rate;   /**< 方向盘转角速率 (rad/s) */
    uint64_t timestamp;  /**< 时间戳 (ms) */
} SteeringData_t;

/**
 * @brief RTK定位数据结构体（标定用 - 外部 API 使用 float 版本）
 */
typedef struct {
    float x;      /**< RTK x位置 (m) */
    float y;      /**< RTK y位置 (m) */
    float z;      /**< 高度 (m) */
    float heading;  /**< RTK航向角 (rad) */
    uint64_t timestamp;  /**< 时间戳 (ms) */
    uint8_t quality;  /**< 定位质量 (0-5, 5最优) */
} RTKData_t;

/**
 * @brief 传感器读数融合结构体
 * 
 * 集成多种传感器数据，支持脉冲计数模式和直接速度模式
 */
typedef struct {
    IMUData_t imu;
    WheelOdometryData_t wheel_odometry;
    SteeringData_t steering;
    VehicleGear_t gear;                      /**< 当前档位 */
    uint64_t timestamp;                      /**< 融合时间戳 */
    
    /* === 轮速脉冲处理相关字段 === */
    uint32_t rear_left_wheel_count;              /**< 脉冲计数（可选，脉冲模式使用） */
    uint32_t rear_right_wheel_count;              /**< 脉冲计数（可选，脉冲模式使用） */
    WheelDirection_t rear_left_wheel_direction;  /**< 左后轮运动状态 */
    WheelDirection_t rear_right_wheel_direction; /**< 右后轮运动状态 */
    bool has_wheel_pulse;                    /**< 是否包含脉冲数据（true=脉冲模式，false=速度模式） */
} SensorReadings_t;

/**
 * @brief 噪声参数结构体 (外部 API 使用 float 版本)
 */
typedef struct {
    float q_x;        /**< 位置x噪声 */
    float q_y;        /**< 位置y噪声 */
    float q_theta;    /**< 角度噪声 */
    float q_v;        /**< 速度噪声 */
    float q_delta;    /**< 转向角噪声 */
    float r_wheel;    /**< 轮速计测量噪声 */
    float r_imu;      /**< IMU加速度测量噪声 */
    float r_steering; /**< 转向角测量噪声 */
    float r_imu_steering;   /**< IMU推导转向角测量噪声(融合模式使用) */
} NoiseParameters_t;

/**
 * @brief 传感器校准参数结构体 (外部 API 使用 float 版本)
 */
typedef struct {
    float wheel_calibration_factor;  /**< 轮速计校准因子 */
    float steering_angle_offset;     /**< 转向角零偏 (rad) */
    float steering_angle_gain;       /**< 转向角增益 */
    float ackermann_factor;          /**< Ackermann转向几何因子 */
    float imu_accel_bias_x;          /**< IMU x轴加速度零偏 */
    float imu_accel_bias_y;          /**< IMU y轴加速度零偏 */
    float imu_gyro_bias_z;           /**< IMU z轴陀螺仪零偏 */
    float imu_accel_scale_x;         /**< IMU x轴加速度尺度因子 */
    float imu_accel_scale_y;         /**< IMU y轴加速度尺度因子 */
    float imu_lateral_accel_bias;    /**< IMU横向加速度零偏(融合模式使用) */
    float imu_lateral_accel_scale;   /**< IMU横向加速度尺度因子(融合模式使用) */
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
    uint64_t previous_timestamp;  /**< 上一时间戳 */
    VehicleGear_t previous_gear;  /**< 上一时刻档位 */
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
 * 
 * 范围说明：当前系统使用[-π, π]范围而非ISO 8855标准的[0, 2π]
 * 原因：[-π, π]对数值计算更友好，避免0° = 360°的歧义
 * 与atan2()输出一致，符合C/C++库标准
 */
static inline double normalize_angle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

/**
 * @brief 安全的角度规范化函数，检测>π的大差值
 * 
 * 用于标定和数据验证场景，可检测真实的大角度差
 * 不进行normalize，返回原始角度差以便检测异常
 * 
 * @param angle_diff 角度差 (rad)
 * @param out_is_large 输出参数，指示是否为大角度差(>π)
 * @return 原始角度差（未经normalize）
 */
static inline double heading_normalize_safe(double angle_diff, bool *out_is_large) {
    if (out_is_large) {
        *out_is_large = (fabs(angle_diff) > PI);
    }
    return angle_diff;
}

/**
 * @brief 安全的heading角度差计算
 * 
 * 计算两个heading角的差，检测是否超过±π范围
 * 用于标定数据质量检查
 * 
 * @param heading_measured 测量heading角 (rad, [-π, π])
 * @param heading_reference 参考heading角 (rad, [-π, π])
 * @param out_normalized 输出参数，normalize后的差
 * @param out_is_large 输出参数，指示是否为大角度差(>π)
 * @return 未normalize的原始差值
 * 
 * 例：heading_measured=2.5π, heading_reference=-0.5π
 *     raw_diff = 3π (>π，标记为large)
 *     normalized_diff = -π (normalize后的结果)
 */
static inline double heading_angle_diff_safe(double heading_measured, 
                                             double heading_reference,
                                             double *out_normalized,
                                             bool *out_is_large) {
    double raw_diff = heading_measured - heading_reference;
    double normalized_diff = normalize_angle(raw_diff);
    
    if (out_normalized) {
        *out_normalized = normalized_diff;
    }
    if (out_is_large) {
        *out_is_large = (fabs(raw_diff) > PI);
    }
    
    return raw_diff;
}

/**
 * @brief 从档位和速度推导车辆运动方向
 * 
 * 优先级：档位 > 速度信号
 * 
 * @param gear 当前档位
 * @param velocity 速度 (m/s)
 * @return 推导的运动方向
 */
static inline VehicleDirection_t derive_direction(VehicleGear_t gear, double velocity) {
    /* 档位优先判断 */
    switch (gear) {
        case GEAR_REVERSE:
            return DIRECTION_REVERSE;  /* R档明确为后退 */
        case GEAR_DRIVE:
        case GEAR_LOW:
            return DIRECTION_FORWARD;  /* D/L档明确为前进 */
        case GEAR_PARK:
        case GEAR_NEUTRAL:
            /* P/N档下判断静止 */
            if (fabs(velocity) < V_STATIC_THRESHOLD) {
                return DIRECTION_STATIONARY;
            }
            /* P/N下有速度则异常，返回INVALID */
            return DIRECTION_INVALID;
        case GEAR_INVALID:
        default:
            /* 无效档位，根据速度备选判断 */
            if (fabs(velocity) < V_STATIC_THRESHOLD) {
                return DIRECTION_STATIONARY;
            }
            /* 档位无效时不能确定方向 */
            return DIRECTION_INVALID;
    }
}

/**
 * @brief 从速度推导档位（用于向后兼容）
 * 
 * 当外部系统无法提供档位信息时调用此函数
 * 恢复原有的纯速度判断方式
 * 
 * @param velocity 速度 (m/s)
 * @return 推导的档位
 */
static inline VehicleGear_t derive_gear_from_velocity(double velocity) {
    if (fabs(velocity) < V_STATIC_THRESHOLD) {
        return GEAR_PARK;
    } else if (velocity > 0.0) {
        return GEAR_DRIVE;
    } else {
        return GEAR_REVERSE;
    }
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

/* ========== 统一方向推导系统（一处计算，全处使用）========== */

/**
 * @brief 获取速度符号（从方向枚举转换为符号值）
 * 用于后续模块直接使用，避免重复计算
 *
 * @param direction 运动方向（已计算的方向）
 * @return +1 (前进), -1 (后退), 0 (停止)
 */
static inline int get_sign_v_from_direction(VehicleDirection_t direction) {
    if (direction == DIRECTION_REVERSE) {
        return -1;
    } else if (direction == DIRECTION_STATIONARY) {
        return 0;
    } else {
        return 1;  /* DIRECTION_FORWARD or DIRECTION_INVALID */
    }
}

/**
 * @brief 获取有效转向角（考虑运动方向的转向角）
 * 用于后续模块直接使用，避免重复计算
 * 后退时转向角反向，前进时保持不变
 *
 * @param steering_angle 原始转向角 (rad)
 * @param direction 运动方向（已计算的方向）
 * @return 有效转向角（考虑方向的转向角）
 */
static inline double get_effective_steering_angle(double steering_angle, VehicleDirection_t direction) {
    int sign_v = get_sign_v_from_direction(direction);
    return (double)sign_v * steering_angle;
}

/**
 * @brief 检查方向是否有效（不是异常状态）
 * 用于后续模块判断是否能使用direction字段
 *
 * @param direction 运动方向
 * @return true 有效，false 无效或异常
 */
static inline bool is_direction_valid(VehicleDirection_t direction) {
    return (direction != DIRECTION_INVALID);
}

#endif /* LOCALIZATION_TYPES_H */
