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
    uint32_t last_pulse_count;      /**< 上次脉冲计数 */
    double accumulated_distance;    /**< 累计距离 (m) */
    
    /* 速度处理 */
    double last_velocity;           /**< 上次计算的速度 */
    uint32_t last_timestamp;        /**< 上次时间戳 */
    
    /* 低速平滑缓冲（用于中位数和滑动平均） */
    double velocity_buffer[5];      /**< 速度缓冲（5点窗口） */
    uint32_t buffer_index;          /**< 当前缓冲位置 */
    uint32_t buffer_count;          /**< 有效缓冲数据点 */
    
    /* 异常检测 */
    uint32_t last_update_time;      /**< 上次更新的系统时间 */
    bool is_timeout;                /**< 脉冲超时标志 */
    bool is_initialized;            /**< 初始化标志 */
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
 * @brief 使用脉冲计数更新轮速数据
 * 
 * @param processor 处理器指针
 * @param pulse_count 脉冲计数（累计值）
 * @param timestamp 时间戳 (ms)
 * @param output 输出的轮速数据
 * @return 0成功，-1失败
 * 
 * @note 脉冲计数应为累计值，处理器会自动计算增量
 * @note 需要至少2次调用才能计算出速度
 */
int wheel_odometry_processor_update(WheelOdometryProcessor_t *processor,
                                    uint32_t pulse_count,
                                    uint32_t timestamp,
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

#endif /* WHEEL_ODOMETRY_PROCESSOR_H */
