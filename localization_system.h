/**
 * @file localization_system.h
 * @brief 完整的车辆定位系统
 */

#ifndef LOCALIZATION_SYSTEM_H
#define LOCALIZATION_SYSTEM_H

#include "localization_types.h"
#include "kalman_filter.h"
#include "rtk_calibration.h"
#include "wheel_odometry_processor.h"

/**
 * @brief 定位系统上下文
 */
typedef struct {
    KalmanFilter_t kalman_filter;
    RTKCalibrationManager_t rtk_calibration;
    CalibrationParams_t calibration;
    DiagnosticsInfo_t diagnostics;
    WheelOdometryProcessor_t wheel_processor;  /**< 轮速脉冲处理器 */
    WheelConfig_t wheel_config;                 /**< 轮速计配置 */
    
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
 * @brief 计算并应用RTK标定参数（离线标定，不在线融合）
 * 
 * 用于离线标定轮速计校准因子等参数，不是实时RTK融合定位。
 * 标定过程：
 *   1. 记录轮速里程计和RTK的位置
 *   2. 比较两者误差
 *   3. 计算校准参数
 * 
 * @param sys 系统指针
 * @param odometry_state 里程计估计的状态
 * @param rtk_state RTK测量的状态
 * @return 0成功，-1失败
 */
int localization_system_rtk_calibration_add_point(LocalizationSystem_t *sys,
                                                  const VehicleState_t *odometry_state,
                                                  const RTKData_t *rtk_state);

/**
 * @brief 离线标定完成，计算标定参数
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

/**
 * @brief 配置轮速计参数
 * @param sys 系统指针
 * @param config 轮速计配置
 * @return 0成功，-1失败
 */
int localization_system_set_wheel_config(LocalizationSystem_t *sys,
                                        const WheelConfig_t *config);

/**
 * @brief 更新轮速脉冲数据
 * 
 * 用于硬件提供脉冲计数时调用。系统会自动：
 *   1. 计算脉冲增量
 *   2. 转换为距离
 *   3. 计算速度
 *   4. 应用低速平滑
 *   5. 集成到轮速传感器数据
 * 
 * @param sys 系统指针
 * @param pulse_count 脉冲计数（累计值）
 * @param timestamp 时间戳 (ms)
 * @return 0成功，-1失败
 * 
 * @note 需要在 localization_system_set_wheel_config() 后调用
 * @note 推荐调用频率 50Hz 以上
 */
int localization_system_update_wheel_pulse(LocalizationSystem_t *sys,
                                          uint32_t pulse_count,
                                          uint32_t timestamp);

/**
 * @brief 更新轮速速度值
 * 
 * 用于硬件已提供速度值时调用。系统会：
 *   1. 应用低速平滑
 *   2. 积分距离
 *   3. 检测异常
 *   4. 集成到轮速传感器数据
 * 
 * @param sys 系统指针
 * @param velocity 速度 (m/s)
 * @param timestamp 时间戳 (ms)
 * @return 0成功，-1失败
 * 
 * @note 当硬件已转换脉冲→速度时使用此接口
 * @note 推荐调用频率 50Hz 以上
 */
int localization_system_update_wheel_velocity(LocalizationSystem_t *sys,
                                             double velocity,
                                             uint32_t timestamp);

/**
 * @brief 获取最新的轮速数据
 * @param sys 系统指针
 * @param odometry 输出的轮速数据
 * @return 0成功，-1失败
 */
int localization_system_get_wheel_odometry(const LocalizationSystem_t *sys,
                                           WheelOdometryData_t *odometry);

/**
 * @brief 检查轮速脉冲是否超时
 * @param sys 系统指针
 * @return true超时，false正常
 */
bool localization_system_wheel_is_timeout(const LocalizationSystem_t *sys);

/**
 * @brief 重置轮速处理器
 * @param sys 系统指针
 * @return 0成功，-1失败
 */
int localization_system_reset_wheel_processor(LocalizationSystem_t *sys);

#endif /* LOCALIZATION_SYSTEM_H */
