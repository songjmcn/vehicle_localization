/**
 * @file wheel_odometry_processor.h
 * @brief 轮速计脉冲处理器 - 将脉冲计数转换为速度和距离
 * @details
 * 功能：
 *   1. 脉冲计数 → 距离转换（基于码盘参数）
 *   2. 距离 → 速度计算（时间微分）
 *   3. 低速平滑处理（中位数 + 滑动平均）
 *   4. 极低速和静止检测
 *   5. 脉冲超时检测
 */

#ifndef WHEEL_ODOMETRY_PROCESSOR_H
#define WHEEL_ODOMETRY_PROCESSOR_H

#include "localization_types.h"

/**
 * @brief 轮速处理器内部状态（对用户隐藏）
 */
typedef struct {
    /* 配置 */
    WheelConfig_t config;

    /* 脉冲处理 */
    uint32_t last_pulse_count;           /**< 上次合成脉冲计数 */
    uint32_t last_left_pulse_count;      /**< 上次左轮脉冲计数 */
    uint32_t last_right_pulse_count;     /**< 上次右轮脉冲计数 */
    double accumulated_distance;         /**< 累计距离 (m) */

    /* 速度处理 */
    double last_velocity;                /**< 上次计算的速度 */
    uint64_t last_timestamp;             /**< 上次时间戳 */

    /* 低速平滑缓冲（用于中位数和滑动平均） */
    double velocity_buffer[5];           /**< 速度缓冲（5点窗口） */
    uint32_t buffer_index;               /**< 当前缓冲位置 */
    uint32_t buffer_count;               /**< 有效缓冲数据点 */

    /* 异常检测 */
    uint32_t last_update_time;           /**< 上次更新的系统时间 */
    bool is_timeout;                     /**< 脉冲超时标志 */
    bool is_initialized;                 /**< 初始化标志 */
    bool first_call;                     /**< 首次调用标志 */

    float pulse_residual;                /**< 脉冲小数部分累积 */
} WheelOdometryProcessor_t;

/**
 * @brief 初始化轮速处理器
 * @param processor 处理器指针
 * @param config 轮速计配置
 * @return 0成功，-1失败
 */
int wheel_odometry_processor_init(WheelOdometryProcessor_t *processor,
                                  const WheelConfig_t *config);

/**
 * @brief 使用左右轮脉冲计数更新轮速数据，并推导车辆运动方向
 *
 * @param processor 处理器指针
 * @param left_pulse_count 左轮脉冲计数（累计值）
 * @param right_pulse_count 右轮脉冲计数（累计值）
 * @param timestamp 时间戳 (ms)
 * @param gear 当前档位（用于方向推导）
 * @param rear_left_state 左后轮运动状态（用于方向推导）
 * @param rear_right_state 右后轮运动状态（用于方向推导）
 * @param output 输出的轮速数据（包含推导的方向）
 * @return 0成功，-1失败
 *
 * @note 脉冲计数应为累计值，处理器会自动计算增量
 * @note 内部计算合成脉冲：pulse_count = (left_pulse_count + right_pulse_count) / 2
 * @note 需要至少2次调用才能计算出速度
 * @note 方向推导基于档位 + 轮状态 + 速度的多传感器融合
 */
int wheel_odometry_processor_update(WheelOdometryProcessor_t *processor,
                                    uint32_t left_pulse_count,
                                    uint32_t right_pulse_count,
                                    uint64_t timestamp,
                                    VehicleGear_t gear,
                                    WheelDirection_t rear_left_state,
                                    WheelDirection_t rear_right_state,
                                    WheelOdometryData_t *output);

/**
 * @brief 直接设置速度值（如果硬件已提供速度值）
 *
 * @param processor 处理器指针
 * @param velocity 速度 (m/s)
 * @param timestamp 时间戳 (ms)
 * @param output 输出的轮速数据
 * @return 0成功，-1失败
 *
 * @note 当外部硬件已完成脉冲-速度转换时使用此接口
 * @note 处理器仍会应用低速平滑和异常检测
 */
int wheel_odometry_processor_set_velocity(WheelOdometryProcessor_t *processor,
                                          double velocity,
                                          uint32_t timestamp,
                                          WheelOdometryData_t *output);

/**
 * @brief 重置处理器状态
 * @param processor 处理器指针
 * @return 0成功，-1失败
 */
int wheel_odometry_processor_reset(WheelOdometryProcessor_t *processor);

/**
 * @brief 获取处理器内部诊断信息
 * @param processor 处理器指针
 * @return 是否超时
 */
bool wheel_odometry_processor_is_timeout(const WheelOdometryProcessor_t *processor);

/* ========== 方向判断接口 ========== */

/**
 * @brief 推导车辆运动方向（综合速度、档位、左右后轮状态）
 *
 * @param velocity 综合车速 (m/s)
 * @param gear 当前档位 (GEAR_DRIVE, GEAR_REVERSE等)
 * @param rear_left_state 左后轮运动状态 (WHEEL_DIRECTION_FORWARD, WHEEL_DIRECTION_REVERSE, WHEEL_DIRECTION_STOP, WHEEL_DIRECRTION_INVALID)
 * @param rear_right_state 右后轮运动状态
 * @return VehicleDirection_t 车辆运动方向
 *         DIRECTION_FORWARD: 前进
 *         DIRECTION_REVERSE: 后退
 *         DIRECTION_STATIONARY: 停止
 *         DIRECTION_INVALID: 无效/错误
 *
 * @details
 * 判断优先级（从高到低）：
 *   P1: 静止判断 (v < 0.1 m/s) → STATIONARY
 *   P2: 轮状态判断 (左右轮状态都有效且相同)
 *   P3: 档位判断 (R→后退, D/L→前进, P/N→停止)
 *   P4: 异常检测 (左右轮状态不一致、档位矛盾)
 *
 * @note 采用多传感器融合，轮状态优先于档位
 * @note 当左右轮状态不一致时返回INVALID（可能出现异常或传感器故障）
 * @note 优化：直接返回VehicleDirection_t，避免不必要的类型转换
 */
VehicleDirection_t wheel_odometry_derive_direction(
    double velocity,
    VehicleGear_t gear,
    WheelDirection_t rear_left_state,
    WheelDirection_t rear_right_state);
VehicleDirection_t wheel_direction_to_vehicle_direction(WheelDirection_t direction);
#endif /* WHEEL_ODOMETRY_PROCESSOR_H */

