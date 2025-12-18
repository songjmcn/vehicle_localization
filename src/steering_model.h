/**
 * @file steering_model.h
 * @brief 转向角转换模块 - 方向盘转角到车轮转向角
 * 
 * 本模块实现方向盘转角（steering wheel angle）到前轮转向角（front wheel steering angle）
 * 以及到车辆航向角变化率（yaw rate）的转换。
 * 
 * 转换链：
 * 1. 方向盘转角 (steering_wheel_angle) 
 *    ↓ [齿比转换]
 * 2. 前轮转向角 (front_wheel_angle/delta)
 *    ↓ [运动学模型]
 * 3. 车辆航向角速率 (yaw_rate = v*tan(delta)/L)
 *    ↓ [积分]
 * 4. 车辆航向角 (vehicle_heading/theta)
 */

#ifndef STEERING_MODEL_H
#define STEERING_MODEL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 转向系统配置参数
 */
typedef struct {
    float steering_ratio;           /**< 方向盘转向比（gear ratio），通常 14-20 */
    float max_steering_wheel_angle; /**< 最大方向盘转角 (rad)，通常 ±π (±180°) */
    float max_front_wheel_angle;    /**< 最大前轮转向角 (rad)，通常 ±0.628 rad (±36°) */
    float wheelbase;                /**< 轴距 (m) */
    float steering_angle_offset;    /**< 转向零偏补偿 (rad) */
    float steering_angle_gain;      /**< 转向角增益补偿 */
    bool enable_steering_filter;    /**< 是否启用转向低通滤波 */
    float steering_filter_alpha;    /**< 转向滤波系数 [0,1]，0=无滤波，1=完全滤波 */
} SteeringConfig_t;

/**
 * @brief 转向模型状态
 */
typedef struct {
    double filtered_steering_angle; /**< 滤波后的前轮转向角 (rad) */
    double prev_steering_angle;     /**< 前一时刻的转向角 (rad) */
    double steering_rate;           /**< 转向角速率 (rad/s) */
    uint32_t prev_timestamp;        /**< 前一时刻的时间戳 (ms) */
} SteeringModelState_t;

/**
 * @brief 初始化转向模型
 * 
 * @param config 转向系统配置参数
 * @return 0成功，-1失败
 */
int steering_model_init(const SteeringConfig_t *config);

/**
 * @brief 将方向盘转角转换为前轮转向角
 * 
 * 转换公式: front_wheel_angle = steering_wheel_angle / steering_ratio + offset
 * 
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 前轮转向角 (rad)
 */
double steering_wheel_to_front_wheel(double steering_wheel_angle);

/**
 * @brief 将前轮转向角转换为方向盘转角
 * 
 * 逆转换公式: steering_wheel_angle = (front_wheel_angle - offset) * steering_ratio
 * 
 * @param front_wheel_angle 前轮转向角 (rad)
 * @return 方向盘转角 (rad)
 */
double front_wheel_to_steering_wheel(double front_wheel_angle);

/**
 * @brief 计算车辆航向角速率（偏航角速率）
 * 
 * 基于二轮模型（bicycle model）：
 * yaw_rate = v * tan(delta) / L
 * 
 * @param velocity 车辆纵向速度 (m/s)
 * @param steering_angle 前轮转向角 (rad)
 * @return 航向角速率 (rad/s)
 */
double calculate_yaw_rate(double velocity, double steering_angle);

/**
 * @brief 更新转向角并应用低通滤波
 * 
 * @param steering_wheel_angle 方向盘转角输入 (rad)
 * @param timestamp 当前时间戳 (ms)
 * @return 滤波后的前轮转向角 (rad)
 */
double steering_model_update(double steering_wheel_angle, uint32_t timestamp);

/**
 * @brief 获取滤波后的前轮转向角
 * 
 * @return 前轮转向角 (rad)
 */
double steering_model_get_filtered_angle(void);

/**
 * @brief 获取转向角速率
 * 
 * @return 转向角速率 (rad/s)
 */
double steering_model_get_steering_rate(void);

/**
 * @brief 应用转向零偏和增益补偿
 * 
 * 补偿公式: compensated_angle = (angle - offset) * gain
 * 
 * @param angle 原始转向角 (rad)
 * @return 补偿后的转向角 (rad)
 */
double steering_model_apply_calibration(double angle);

/**
 * @brief 限制转向角在有效范围内
 * 
 * @param angle 输入转向角 (rad)
 * @return 限制后的转向角 (rad)
 */
double steering_model_clamp_angle(double angle);

/**
 * @brief 重置转向模型状态
 */
void steering_model_reset(void);

/**
 * @brief 获取当前转向系统配置
 * 
 * @return 配置参数指针
 */
const SteeringConfig_t* steering_model_get_config(void);

/**
 * @brief 更新转向系统配置
 * 
 * @param config 新的配置参数
 * @return 0成功，-1失败
 */
int steering_model_update_config(const SteeringConfig_t *config);

#endif /* STEERING_MODEL_H */
