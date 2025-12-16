/**
 * @file localization_system.h
 * @brief 完整的车辆定位系统
 */

#ifndef LOCALIZATION_SYSTEM_H
#define LOCALIZATION_SYSTEM_H

#include "localization_types.h"
#include "kalman_filter.h"
#include "rtk_calibration.h"

/**
 * @brief 定位系统上下文
 */
typedef struct {
    KalmanFilter_t kalman_filter;
    RTKCalibrationManager_t rtk_calibration;
    CalibrationParams_t calibration;
    DiagnosticsInfo_t diagnostics;
    
    /* 系统状态 */
    bool is_initialized;
    uint32_t operation_timestamp;
} LocalizationSystem_t;

/**
 * @brief 初始化定位系统
 * @param sys 系统指针
 * @param initial_state 初始状态（NULL时为原点）
 * @return 0成功，-1失败
 */
int localization_system_init(LocalizationSystem_t *sys,
                            const VehicleState_t *initial_state);

/**
 * @brief 执行一个完整的定位步骤（预测+更新）
 * @param sys 系统指针
 * @param sensor_data 传感器数据
 * @param estimated_state 输出的估计状态
 * @return 0成功，-1失败
 */
int localization_system_process(LocalizationSystem_t *sys,
                               const SensorReadings_t *sensor_data,
                               VehicleState_t *estimated_state);

/**
 * @brief 获取当前状态估计
 * @param sys 系统指针
 * @param state 输出状态
 */
void localization_system_get_state(const LocalizationSystem_t *sys,
                                  VehicleState_t *state);

/**
 * @brief 获取诊断信息
 * @param sys 系统指针
 * @return 诊断信息
 */
DiagnosticsInfo_t localization_system_get_diagnostics(const LocalizationSystem_t *sys);

/**
 * @brief 添加RTK标定点
 * @param sys 系统指针
 * @param rtk_state RTK状态
 * @return 0成功，-1失败
 */
int localization_system_add_rtk_calibration(LocalizationSystem_t *sys,
                                           const RTKData_t *rtk_state);

/**
 * @brief 计算并应用RTK标定参数
 * @param sys 系统指针
 * @return 0成功，-1失败
 */
int localization_system_apply_rtk_calibration(LocalizationSystem_t *sys);

/**
 * @brief 手动设置校准参数
 * @param sys 系统指针
 * @param calibration 校准参数
 */
void localization_system_set_calibration(LocalizationSystem_t *sys,
                                        const CalibrationParams_t *calibration);

/**
 * @brief 重置定位系统
 * @param sys 系统指针
 * @param initial_state 初始状态（NULL时为原点）
 */
void localization_system_reset(LocalizationSystem_t *sys,
                              const VehicleState_t *initial_state);

/**
 * @brief 释放系统资源
 * @param sys 系统指针
 */
void localization_system_free(LocalizationSystem_t *sys);

/**
 * @brief 设置转向角过滤器参数
 * @param sys 系统指针
 * @param jitter_threshold 抖动阈值 (rad)
 * @param filter_alpha 低通滤波系数 [0,1]
 * @return 0成功，-1失败
 */
int localization_system_set_steering_filter_params(LocalizationSystem_t *sys,
                                                    double jitter_threshold,
                                                    double filter_alpha);

/**
 * @brief 获取转向角过滤器统计信息
 * @param sys 系统指针
 * @param mean_angle 输出：转向角均值
 * @param variance 输出：方差
 * @param zero_bias 输出：零偏角
 * @param is_high_jitter 输出：是否高抖动
 * @return 0成功，-1失败
 */
int localization_system_get_steering_stats(const LocalizationSystem_t *sys,
                                            double *mean_angle,
                                            double *variance,
                                            double *zero_bias,
                                            bool *is_high_jitter);

/**
 * @brief 重置转向角过滤器
 * @param sys 系统指针
 */
void localization_system_reset_steering_filter(LocalizationSystem_t *sys);

#endif /* LOCALIZATION_SYSTEM_H */
