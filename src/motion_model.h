/**
 * @file motion_model.h
 * @brief 运动模型定义
 */

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include "localization_types.h"
#include "steering_model.h"

/**
 * @brief 单轨运动模型初始化
 */
int motion_model_init(SteeringModel_t *mode, SteeringConfig_t *config,SteeringMappingTable_t *mapping_table);

/**
 * @brief 使用单轨模型预测车辆状态（支持前后退）
 * 
 * @param steering_config 转向模型配置指针（包含转向角转换映射）
 * @param state 当前状态
 * @param velocity 速度 (m/s)
 * @param steering_angle 转向角 (rad)
 * @param dt 时间间隔 (s)
 * @return 预测的新状态
 */
VehicleState_t motion_model_predict(SteeringModel_t *steering_config,
                                    const VehicleState_t *state,
                                    float velocity,
                                    float steering_angle,
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

/* ========== Float API (for TC397 MCU compatibility) ========== */

/**
 * @brief 使用单轨模型预测车辆状态 (Float版本)
 * 
 * 内部将float参数转换为double调用原有函数
 * 
 * @param steering_config 转向模型配置指针
 * @param state 当前状态
 * @param velocity 速度 (m/s) - float
 * @param steering_angle 转向角 (rad) - float
 * @param dt 时间间隔 (s) - float
 * @return 预测的新状态
 */
VehicleState_t motion_model_predict_float(SteeringModel_t *steering_config,
                                          const VehicleState_t *state,
                                          float velocity,
                                          float steering_angle,
                                          float dt);

/**
 * @brief 计算运动学模型的状态转移雅可比矩阵 (Float版本)
 * 
 * 内部将float参数转换为double调用原有函数
 *
 * @param state 当前状态
 * @param velocity 速度 (m/s) - float
 * @param steering_angle 转向角 (rad) - float
 * @param dt 时间间隔 (s) - float
 * @param F 输出的雅可比矩阵 [5x5] - double精度
 */
void motion_model_jacobian_float(const VehicleState_t *state,
                                 float velocity,
                                 float steering_angle,
                                 float dt,
                                 double F[5][5]);

#endif /* MOTION_MODEL_H */
