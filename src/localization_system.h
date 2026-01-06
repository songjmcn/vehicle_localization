/**
 * @file localization_system.h
 * @brief 完整的车辆定位系统
 */

#ifndef LOCALIZATION_SYSTEM_H
#define LOCALIZATION_SYSTEM_H

#include "localization_types.h"
#include "kalman_filter.h"
#include "wheel_odometry_processor.h"
#include "motion_model.h"

/**
 * @brief 定位系统上下文
 */
typedef struct {
    KalmanFilter_t kalman_filter;
    CalibrationParams_t calibration;
    DiagnosticsInfo_t diagnostics;
    WheelOdometryProcessor_t wheel_processor;  /**< 轮速脉冲处理器 */
    WheelConfig_t wheel_config;                 /**< 轮速计配置 */
    SteeringModel_t heading_processer;     /**< 转向模型配置 */
    /* 系统状态 */
    bool is_initialized;
    uint64_t operation_timestamp;
} LocalizationSystem_t;

/**
 * @brief 初始化定位系统
 * @param sys 系统指针
 * @param initial_state 初始状态（NULL时为原点）
 * @return 0成功，-1失败
 * 
 * @note 自动使用 FUSION_MODE_BASIC（向后兼容）
 */
int localization_system_init(LocalizationSystem_t *sys,
                            const VehicleState_t *initial_state);

/**
 * @brief 初始化定位系统并指定融合模式
 * @param sys 系统指针
 * @param initial_state 初始状态（NULL时为原点）
 * @param fusion_mode 融合模式 (FUSION_MODE_BASIC 或 FUSION_MODE_IMU_STEERING)
 * @return 0成功，-1失败
 * 
 * @example
 *   localization_system_init_with_fusion(&sys, NULL, FUSION_MODE_IMU_STEERING);
 */
int localization_system_init_with_fusion(LocalizationSystem_t *sys,
                                        const VehicleState_t *initial_state,
                                        FusionMode_t fusion_mode);

/**
 * @brief 运行时切换融合模式
 * @param sys 系统指针
 * @param fusion_mode 新的融合模式
 * @return 0成功，-1失败
 * 
 * @note 不重置卡尔曼滤波器状态，平滑切换
 */
int localization_system_switch_fusion(LocalizationSystem_t *sys,
                                     FusionMode_t fusion_mode);

/**
 * @brief 执行一个完整的定位步骤（预测+更新）
 * 
 * 此函数集成了轮速脉冲处理和定位计算。如果 sensor_data 包含脉冲数据，
 * 会自动调用脉冲处理器；否则直接使用 wheel_odometry 中的速度值。
 * 
 * @param sys 系统指针
 * @param sensor_data 传感器数据结构
 *        - 必须包含有效的 gear 字段
 *        - 可选包含 wheel_pulse_count 和轮状态（如果 has_wheel_pulse=true）
 *        - 或直接包含 wheel_odometry.velocity（速度模式）
 * @param estimated_state 输出的估计状态
 * @return 0成功，-1失败
 * 
 * @note sensor_data->gear 必须由调用者填充，用于方向推导
 * @note 脉冲处理已在本函数中自动集成，无需单独调用
 * 
 * @example
 *   // 脉冲模式
 *   sensor_data.has_wheel_pulse = true;
 *   sensor_data.wheel_pulse_count = pulse_count;
 *   sensor_data.rear_left_wheel_state = WHEEL_DIRECTION_FORWARD;
 *   sensor_data.rear_right_wheel_state = WHEEL_DIRECTION_FORWARD;
 *   
 *   // 速度模式
 *   sensor_data.has_wheel_pulse = false;
 *   sensor_data.wheel_odometry.velocity = speed_value;
 *   sensor_data.rear_left_wheel_state = WHEEL_DIRECTION_FORWARD;
 *   sensor_data.rear_right_wheel_state = WHEEL_DIRECTION_FORWARD;
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
 * @brief 手动设置校准参数
 * 
 * 用于应用外部标定系统计算的校准参数
 * （例如RTKCalibrationSystem_t计算的结果）
 * 
 * @param sys 系统指针
 * @param calibration 校准参数
 */
/**
 * @brief 手动设置校准参数（带置信度门槛）
 * @param sys 系统指针
 * @param calibration 校准参数
 * @param confidence 离线标定置信度分数 [0,1]，如果小于阈值则拒绝应用
 * @return 0 应用成功，-1 拒绝或失败
 */
int localization_system_set_calibration(LocalizationSystem_t *sys,
                                        const CalibrationParams_t *calibration,
                                        float confidence);

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
 * @brief 更新轮速脉冲数据（已整合到 localization_system_process）
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
 * @param gear 当前档位
 * @param rear_left_state 左后轮运动状态
 * @param rear_right_state 右后轮运动状态
 * @return 0成功，-1失败
 * 
 * @note 需要在 localization_system_set_wheel_config() 后调用
 * @note 推荐调用频率 50Hz 以上
 * @deprecated 建议通过 SensorReadings_t.has_wheel_pulse 在 localization_system_process 中使用
 */
int localization_system_update_wheel_pulse(LocalizationSystem_t *sys,
                                          uint32_t pulse_count,
                                          uint64_t timestamp,
                                          VehicleGear_t gear,
                                          WheelDirection_t rear_left_state,
                                          WheelDirection_t rear_right_state);

/**
 * @brief 更新轮速速度值（已整合到 localization_system_process）
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
 * @param gear 当前档位
 * @param rear_left_state 左后轮运动状态
 * @param rear_right_state 右后轮运动状态
 * @return 0成功，-1失败
 * 
 * @note 当硬件已转换脉冲→速度时使用此接口
 * @note 推荐调用频率 50Hz 以上
 * @deprecated 建议通过 SensorReadings_t.has_wheel_pulse 在 localization_system_process 中使用
 */
int localization_system_update_wheel_velocity(LocalizationSystem_t *sys,
                                             double velocity,
                                             uint64_t timestamp,
                                             VehicleGear_t gear,
                                             WheelDirection_t rear_left_state,
                                             WheelDirection_t rear_right_state);

/**
 * @brief 获取最新的轮速数据
 * @param sys 系统指针
 * @param odometry 输出的轮速数据（包括左右轮方向）
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
