/**
 * @file steering_model.h
 * @brief 转向角转换模块 - 方向盘转角到前轮转向角
 * 
 * 本模块实现方向盘转角（steering wheel angle）到前轮转向角（front wheel angle）
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
    float ackermann_factor;      /**< Ackermann转向几何因子，默认1.0 */
    bool enable_steering_filter;    /**< 是否启用转向低通滤波 */
    float steering_filter_alpha;    /**< 转向滤波系数 [0,1]，0=无滤波，1=完全滤波 */
    float steering_deadzone;        /**< 转向死区范围 (rad)，<此值认为无转向，默认 0.02 rad (±1.1°) */
} SteeringConfig_t;



/**
 * @brief 转向模型状态
 */
typedef struct {
    float filtered_steering_angle; /**< 滤波后的前轮转向角 (rad) */
    float prev_steering_angle;     /**< 前一时刻的前轮转向角 (rad) */
    float steering_rate;           /**< 前轮转向角速率 (rad/s) */
    uint64_t prev_timestamp;        /**< 前一时刻的时间戳 (ms) */
    float left_wheel_angle;        /**< 前左轮转向角 (rad) */
    float right_wheel_angle;       /**< 前右轮转向角 (rad) */
    float mean_wheel_angle;        /**< 平均前轮转向角 (rad) */
} SteeringModelState_t;

/**
 * @brief 转向角映射表结构
 * 用于存储方向盘转角到前左/右轮转角的非线性映射关系
 */
typedef struct {
    float *steering_wheel;         /**< 方向盘转角采样点数组 (rad) */
    float *left_wheel;             /**< 前左轮转角数组 (rad) */
    float *right_wheel;            /**< 前右轮转角数组 (rad) */
    int num_points;                 /**< 映射表采样点数 */
    bool valid;                     /**< 映射表是否已加载 */
} SteeringMappingTable_t;

/**
 * @brief Steering Model(转型模型) 综合配置，需要再init时传入此结构体
 * 
 */
typedef struct {
    SteeringConfig_t steering_config;         /* 转向系统配置参数 */
    SteeringMappingTable_t steering_mapping;  /* 转向映射表 */
    SteeringModelState_t   steering_state;      /* 转向模型状态 */

}SteeringModel_t;

/**
 * @brief 初始化转向模型
 * 
 * @param config 转向系统配置参数
 * @return 0成功，-1失败
 */
int steering_config_init(SteeringModel_t *mode,SteeringConfig_t* config);

/**
 * @brief 将方向盘转角转换为前轮转向角
 * 
 * 转换公式: front_wheel_angle = steering_wheel_angle / steering_ratio + offset
 * 如果映射表可用，使用映射表查询；否则使用线性模型
 * 
 * @param config 转向模型配置指针
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 前轮转向角 (rad)
 */
float steering_wheel_to_front_wheel(SteeringModel_t *config, float steering_wheel_angle);

/**
 * @brief 将前轮转向角转换为方向盘转角
 * 
 * 逆转换公式: steering_wheel_angle = (front_wheel_angle - offset) * steering_ratio
 * 
 * @param config 转向模型配置指针
 * @param front_wheel_angle 前轮转向角 (rad)
 * @return 方向盘转角 (rad)
 */
float front_wheel_to_steering_wheel(SteeringModel_t *config, float front_wheel_angle);

/**
 * @brief 计算车辆航向角速率（偏航角速率）
 * 
 * 基于二轮模型（bicycle model）：
 * yaw_rate = v * tan(delta) / L
 * 
 * @param config 转向模型配置指针
 * @param velocity 车辆纵向速度 (m/s)
 * @param steering_angle 前轮转向角 (rad)
 * @return 航向角速率 (rad/s)
 */
float calculate_yaw_rate(SteeringModel_t *config, float velocity, float steering_angle);

/**
 * @brief 更新转向角并应用低通滤波
 * 
 * @param config 转向模型配置指针，其中包含要更新的状态
 * @param steering_wheel_angle 方向盘转角输入 (rad)
 * @param timestamp 当前时间戳 (ms)
 * @return 滤波后的前轮转向角 (rad)
 */
float steering_model_update(SteeringModel_t *config, float steering_wheel_angle, uint64_t timestamp);

/**
 * @brief 获取滤波后的前轮转向角
 * 
 * @param config 转向模型配置指针
 * @return 前轮转向角 (rad)
 */
float steering_model_get_filtered_angle(SteeringModel_t *config);

/**
 * @brief 获取转向角速率
 * 
 * @param config 转向模型配置指针
 * @return 转向角速率 (rad/s)
 */
float steering_model_get_steering_rate(SteeringModel_t *config);

/**
 * @brief 应用转向零偏和增益补偿
 * 
 * 补偿公式: compensated_angle = (angle - offset) * gain
 * 
 * @param config 转向模型配置指针
 * @param angle 原始转向角 (rad)
 * @return 补偿后的转向角 (rad)
 */
float steering_model_apply_calibration(SteeringModel_t *config, float angle);

/**
 * @brief 限制转向角在有效范围内
 * 
 * @param config 转向模型配置指针
 * @param angle 输入转向角 (rad)
 * @return 限制后的转向角 (rad)
 */
float steering_model_clamp_angle(SteeringModel_t *config, float angle);

/**
 * @brief 重置转向模型状态
 * 
 * @param config 转向模型配置指针
 */
void steering_model_reset(SteeringModel_t *config);

/* ========== 映射表相关函数 ========== */

/**
 * @brief 初始化转向角映射表
 * 
 * @param config 转向模型配置指针
 * @param sw_samples 方向盘转角采样点数组 (deg)
 * @param left_samples 前左轮转角数组 (deg)
 * @param right_samples 前右轮转角数组 (deg)
 * @param num_points 数组元素个数
 * @return 0成功，-1失败
 */
int steering_mapping_init(SteeringModel_t *config,
                        const float *sw_samples,
                        const float *left_samples,
                        const float *right_samples,
                        int num_points);

/**
 * @brief  初始化转向角映射表
 * 
 * @param config[in,out] 转向模型配置指针 
 * @param mapping_table[in] 转向映射表结构体指针 
 * @return * int    0成功，-1失败
 */
int steering_mapping_init_ex(SteeringModel_t *config,
                        const SteeringMappingTable_t *mapping_table);  
/**
 * @brief   初始化转向角映射表 
 * 
 * @param mapping_table[in,out] 转向映射表结构体指针
 * @param sw_samples[in] 方向盘转角采样点数组 (rad)
 * @param left_samples[in] 前左轮转角数组 (rad)
 * @param right_samples[in] 前右轮转角数组 (rad)
 * @param num_points[in] 数组元素个数
 * @return 0成功，-1失败
 */
int steering_mapping_table_init(SteeringMappingTable_t *mapping_table,
                        const float *sw_samples,
                        const float *left_samples,
                        const float *right_samples,
                        int num_points);

/**
 * @brief 复制转向映射表
 * 
 * @param dest 目标映射表指针
 * @param src 源映射表指针
 * @return 0成功，-1失败
 */
int steering_mapping_table_copy(SteeringMappingTable_t *dest, const SteeringMappingTable_t *src);                    
/**
 * @brief 根据映射表查询前轮转角
 * 
 * @param config 转向模型配置指针
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @param left_wheel_angle 输出：前左轮转角指针 (rad)
 * @param right_wheel_angle 输出：前右轮转角指针 (rad)
 * @return 0成功，-1失败
 */
int steering_query_mapping(SteeringModel_t *config,
                         float steering_wheel_angle,
                         float *left_wheel_angle,
                         float *right_wheel_angle);

/**
 * @brief 计算平均转向角
 * 
 * @param config 转向模型配置指针
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 平均转向角 (rad)
 */
float steering_compute_mean_angle(SteeringModel_t *config, float steering_wheel_angle);

/**
 * @brief 计算Ackermann模型的航向角速率
 * 
 * @param config 转向模型配置指针
 * @param velocity 车辆纵向速度 (m/s)
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 航向角速率 (rad/s)
 */
float steering_calculate_yaw_rate_ackermann(SteeringModel_t *config,
                                            float velocity,
                                            float steering_wheel_angle);

/**
 * @brief 获取当前的前轮转角信息
 * 
 * @param config 转向模型配置指针
 * @param left_wheel 输出：前左轮转角指针
 * @param right_wheel 输出：前右轮转角指针
 * @param mean_angle 输出：平均转向角指针
 * @return 0成功，-1失败
 */
int steering_get_wheel_angles(SteeringModel_t *config,
                            float *left_wheel,
                            float *right_wheel,
                            float *mean_angle);

/**
 * @brief 清理映射表资源
 * 
 * @param config 转向模型配置指针
 */
void steering_mapping_cleanup(SteeringModel_t *config);

/* ========== Phase 2 (P1) RTK校准集成接口 ========== */

/**
 * @brief 获取当前转向角增益
 * 
 * 从转向模型配置中读取当前设置的增益值
 * 
 * @param config 转向模型配置指针
 * @return 当前转向角增益值 (1.0 = 无补偿)
 */
float steering_model_get_gain(SteeringModel_t *config);

/**
 * @brief 获取当前转向角零偏
 * 
 * 从转向模型配置中读取当前设置的零偏值
 * 
 * @param config 转向模型配置指针
 * @return 当前转向角零偏 (rad)
 */
float steering_model_get_offset(SteeringModel_t *config);

/**
 * @brief 更新转向角增益
 * 
 * 用于应用外部校准系统（RTKCalibrationSystem_t）计算的增益值
 * 
 * @param config 转向模型配置指针
 * @param new_gain 新的增益值 (1.0 = 无补偿，建议范围 [0.8, 1.2])
 * @return 0成功，-1失败（增益值超出合理范围）
 * 
 * @note 此函数不修改零偏，零偏需通过 steering_model_apply_rtk_calibration 更新
 */
int steering_model_update_gain(SteeringModel_t *config, float new_gain);

/**
 * @brief 应用 RTK 校准结果
 * 
 * 一次性应用来自 CalibrationParams_t 的所有校准参数
 * 包括转向角零偏和增益
 * 
 * @param config 转向模型配置指针
 * @param calibration RTK 校准参数指针
 * @return 0成功，-1失败（参数指针无效）
 * 
 * @note 这是主要的集成接口，用于从外部校准系统应用参数
 * @example
 *   CalibrationParams_t calibration;
 *   rtk_calibration_system_get_result(&calib_sys, &calibration);
 *   steering_model_apply_rtk_calibration(&steering_model, &calibration);
 */
int steering_model_apply_rtk_calibration(SteeringModel_t *config,
                                         const void *calibration);

#endif /* STEERING_MODEL_H */
