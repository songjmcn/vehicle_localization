/**
 * @file kalman_filter.h
 * @brief 扩展卡尔曼滤波器（EKF）定义
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "localization_types.h"

/**
 * @brief 卡尔曼滤波器上下文结构体
 */
typedef struct {
    FilterState_t state;
    NoiseParameters_t noise;
    CalibrationParams_t calibration;
    DiagnosticsInfo_t diagnostics;
    SteeringJitterFilter_t steering_filter;  /**< 转向角抖动过滤器 */
} KalmanFilter_t;

/**
 * @brief 初始化卡尔曼滤波器
 * @param kf 滤波器指针
 * @param initial_state 初始状态
 * @param noise 噪声参数
 * @param calibration 校准参数
 * @return 0成功，-1失败
 */
int kalman_filter_init(KalmanFilter_t *kf,
                       const VehicleState_t *initial_state,
                       const NoiseParameters_t *noise,
                       const CalibrationParams_t *calibration);

/**
 * @brief 预测步骤
 * @param kf 滤波器指针
 * @param velocity 速度 (m/s)
 * @param steering_angle 转向角 (rad)
 * @param timestamp 时间戳 (ms)
 */
void kalman_filter_predict(KalmanFilter_t *kf,
                          double velocity,
                          double steering_angle,
                          uint32_t timestamp);

/**
 * @brief 更新步骤
 * @param kf 滤波器指针
 * @param sensor_data 传感器数据
 */
void kalman_filter_update(KalmanFilter_t *kf,
                         const SensorReadings_t *sensor_data);

/**
 * @brief 获取当前状态估计
 * @param kf 滤波器指针
 * @param state 输出状态指针
 */
void kalman_filter_get_state(const KalmanFilter_t *kf,
                            VehicleState_t *state);

/**
 * @brief 获取诊断信息
 * @param kf 滤波器指针
 * @return 诊断信息
 */
DiagnosticsInfo_t kalman_filter_get_diagnostics(const KalmanFilter_t *kf);

/**
 * @brief 重置滤波器
 * @param kf 滤波器指针
 * @param initial_state 初始状态
 */
void kalman_filter_reset(KalmanFilter_t *kf,
                        const VehicleState_t *initial_state);

/**
 * @brief 初始化转向角抖动过滤器
 * @param filter 过滤器指针
 * @return 0成功，-1失败
 */
int steering_jitter_filter_init(SteeringJitterFilter_t *filter);

/**
 * @brief 更新转向角抖动过滤器
 * @param filter 过滤器指针
 * @param steering_angle 原始转向角 (rad)
 * @return 0成功，-1失败
 */
int steering_jitter_filter_update(SteeringJitterFilter_t *filter,
                                   double steering_angle);

/**
 * @brief 计算转向角零偏
 * 仅在低速或静止时更新零偏，以提高准确性
 * @param filter 过滤器指针
 * @param velocity 当前速度 (m/s)
 * @return 0成功（更新了零偏），-1失败或条件不满足
 */
int steering_jitter_filter_compute_offset(SteeringJitterFilter_t *filter,
                                          double velocity);

/**
 * @brief 获取过滤器统计信息
 * @param filter 过滤器指针
 * @param mean_angle 输出：转向角均值
 * @param variance 输出：转向角方差
 * @param zero_bias 输出：零偏角
 * @param is_high_jitter 输出：是否高抖动
 * @return 0成功，-1失败
 */
int steering_jitter_filter_get_stats(const SteeringJitterFilter_t *filter,
                                     double *mean_angle,
                                     double *variance,
                                     double *zero_bias,
                                     bool *is_high_jitter);

/**
 * @brief 设置过滤器参数
 * @param filter 过滤器指针
 * @param jitter_threshold 抖动阈值 (rad)
 * @param filter_alpha 低通滤波系数 [0,1]
 * @return 0成功，-1失败
 */
int steering_jitter_filter_set_params(SteeringJitterFilter_t *filter,
                                       double jitter_threshold,
                                       double filter_alpha);

/**
 * @brief 重置过滤器状态
 * @param filter 过滤器指针
 */
void steering_jitter_filter_reset(SteeringJitterFilter_t *filter);

/* ========== Float API (for TC397 MCU compatibility) ========== */

/**
 * @brief 预测步骤 (Float版本)
 * @param kf 滤波器指针
 * @param velocity 速度 (m/s) - float
 * @param steering_angle 转向角 (rad) - float
 * @param timestamp 时间戳 (ms)
 */
void kalman_filter_predict_float(KalmanFilter_t *kf,
                                 float velocity,
                                 float steering_angle,
                                 uint32_t timestamp);

/**
 * @brief 更新步骤 (Float版本)
 * @param kf 滤波器指针
 * @param sensor_data 传感器数据
 */
void kalman_filter_update_float(KalmanFilter_t *kf,
                                const SensorReadings_t *sensor_data);

/**
 * @brief 获取当前状态估计 (Float版本)
 * @param kf 滤波器指针
 * @param state 输出状态指针 (内部保持double精度)
 */
void kalman_filter_get_state_float(const KalmanFilter_t *kf,
                                   VehicleState_t *state);

#endif /* KALMAN_FILTER_H */
