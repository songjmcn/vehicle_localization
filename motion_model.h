/**
 * @file motion_model.h
 * @brief 运动模型定义
 */

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include "localization_types.h"

/**
 * @brief 单轨运动模型初始化
 */
void motion_model_init(void);

/**
 * @brief 使用单轨模型预测车辆状态（支持前后退）
 * 
 * @param state 当前状态
 * @param velocity 速度 (m/s)
 * @param steering_angle 转向角 (rad)
 * @param dt 时间间隔 (s)
 * @return 预测的新状态
 */
VehicleState_t motion_model_predict(const VehicleState_t *state,
                                    double velocity,
                                    double steering_angle,
                                    double dt);

/**
 * @brief 计算运动学模型的状态转移雅可比矩阵
 * 
 * @param state 当前状态
 * @param velocity 速度 (m/s)
 * @param steering_angle 转向角 (rad)
 * @param dt 时间间隔 (s)
 * @param F 输出的雅可比矩阵 [5x5]
 */
void motion_model_jacobian(const VehicleState_t *state,
                          double velocity,
                          double steering_angle,
                          double dt,
                          double F[5][5]);

#endif /* MOTION_MODEL_H */
