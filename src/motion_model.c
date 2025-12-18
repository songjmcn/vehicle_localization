/**
 * @file motion_model.c
 * @brief 运动模型实现
 * 
 * 本文件实现方案 B（混合精度）的运动模型：
 * - 公共 API 使用 float 版本
 * - 内部计算使用 double 精度
 * - 包含转向角转换模块的集成
 */

#include "motion_model.h"
#include "localization_adapter.h"
#include "localization_types_internal.h"
#include "steering_model.h"
#include <math.h>
#include <string.h>

void motion_model_init(void) {
    /* 初始化运动模型参数 */
    
    /* 初始化转向模型 */
    SteeringConfig_t steering_config = {
        .steering_ratio = 16.0f,              /* 方向盘齿比 16:1 */
        .max_steering_wheel_angle = 3.14159f, /* ±180° */
        .max_front_wheel_angle = 0.628f,      /* ±36° (0.628 rad) */
        .wheelbase = 2.7f,                    /* 轴距 2.7m */
        .steering_angle_offset = 0.0f,        /* 零偏补偿 */
        .steering_angle_gain = 1.0f,          /* 增益系数 */
        .enable_steering_filter = true,       /* 启用低通滤波 */
        .steering_filter_alpha = 0.15f        /* 滤波系数 */
    };
    
    steering_model_init(&steering_config);
}

/**
 * @brief 单轨模型的预测函数
 * 
 * 改进的模型支持前进和后退，使用档位+速度组合判断方向：
 * x_{k+1} = x_k + |v| * cos(theta_k + sign(direction)*delta) * dt
 * y_{k+1} = y_k + |v| * sin(theta_k + sign(direction)*delta) * dt
 * theta_{k+1} = theta_k + sign(direction)*|v|*tan(delta)/L * dt
 */
VehicleState_t motion_model_predict(const VehicleState_t *state,
                                    double velocity,
                                    double steering_angle,
                                    double dt) {
    VehicleState_t new_state = *state;
    
    /* 检查是否静止 */
    if (fabs(velocity) < V_STATIC_THRESHOLD) {
        /* 完全静止，只更新角度 */
        new_state.x = state->x;
        new_state.y = state->y;
        new_state.theta = state->theta;
        new_state.v = 0.0;
        new_state.delta = steering_angle;
        new_state.gear = state->gear;
        new_state.direction = DIRECTION_STATIONARY;
        return new_state;
    }
    
    /* === 改进：使用档位+速度组合判断方向 === */
    VehicleDirection_t direction;
    
    /* 如果有有效的档位信息，优先使用 */
    if (state->gear != GEAR_INVALID && state->gear != GEAR_NEUTRAL && state->gear != GEAR_PARK) {
        direction = derive_direction(state->gear, velocity);
    } else {
        /* 后备方案：仅基于速度信号（向后兼容） */
        direction = (velocity > 0.0) ? DIRECTION_FORWARD : DIRECTION_REVERSE;
    }
    
    /* 根据方向计算符号 */
    int sign_v = (direction == DIRECTION_REVERSE) ? -1 : 1;
    double abs_v = fabs(velocity);
    
    /* 转向角在后退时反向 */
    double effective_delta = sign_v * steering_angle;
    
    /* 位置更新 */
    double cos_term = cos(state->theta + effective_delta);
    double sin_term = sin(state->theta + effective_delta);
    
    new_state.x = state->x + abs_v * cos_term * dt;
    new_state.y = state->y + abs_v * sin_term * dt;
    
    /* 航向角更新 */
    double tan_delta = tan(effective_delta);
    new_state.theta = state->theta + (sign_v * abs_v * tan_delta / WHEELBASE) * dt;
    new_state.theta = normalize_angle(new_state.theta);
    
    new_state.v = velocity;
    new_state.delta = steering_angle;
    new_state.gear = state->gear;
    new_state.direction = direction;
    
    return new_state;
}

/**
 * @brief 计算雅可比矩阵
 */
void motion_model_jacobian(const VehicleState_t *state,
                          double velocity,
                          double steering_angle,
                          double dt,
                          double F[5][5]) {
    /* 初始化为单位矩阵 */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            F[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    /* 如果速度太小，不更新位置 */
    if (fabs(velocity) < V_STATIC_THRESHOLD) {
        return;
    }
    
    double theta = state->theta;
    
    /* === 改进：使用档位+速度组合判断方向 === */
    VehicleDirection_t direction;
    if (state->gear != GEAR_INVALID && state->gear != GEAR_NEUTRAL && state->gear != GEAR_PARK) {
        direction = derive_direction(state->gear, velocity);
    } else {
        direction = (velocity > 0.0) ? DIRECTION_FORWARD : DIRECTION_REVERSE;
    }
    
    int sign_v = (direction == DIRECTION_REVERSE) ? -1 : 1;
    double abs_v = fabs(velocity);
    double effective_delta = sign_v * steering_angle;
    
    double cos_term = cos(theta + effective_delta);
    double sin_term = sin(theta + effective_delta);
    
    /* ∂x/∂θ */
    F[0][2] = -abs_v * sin_term * dt;
    /* ∂y/∂θ */
    F[1][2] = abs_v * cos_term * dt;
    
    /* ∂x/∂v */
    F[0][3] = sign_v * cos_term * dt;
    /* ∂y/∂v */
    F[1][3] = sign_v * sin_term * dt;
    
    /* ∂θ/∂v 和 ∂θ/∂δ */
    double cos_delta = cos(effective_delta);
    if (fabs(cos_delta) > 1e-6) {
        double tan_delta = tan(effective_delta);
        
        /* ∂θ/∂v */
        F[2][3] = sign_v * tan_delta / WHEELBASE * dt;
        
        /* ∂θ/∂δ */
        F[2][4] = sign_v * abs_v / (WHEELBASE * cos_delta * cos_delta) * dt;
    }
}

/* ========== Float API Implementation (for TC397 MCU compatibility) ========== */

/**
 * @brief 使用单轨模型预测车辆状态 (Float版本)
 * 
 * 内部将float参数转换为double后调用原有的double版本函数
 */
VehicleState_t motion_model_predict_float(const VehicleState_t *state,
                                          float velocity,
                                          float steering_angle,
                                          float dt) {
    if (!state) {
        VehicleState_t empty = {0};
        return empty;
    }
    
    /* 转换float为double并调用原有函数 */
    return motion_model_predict(state,
                               (double)velocity,
                               (double)steering_angle,
                               (double)dt);
}

/**
 * @brief 计算运动学模型的状态转移雅可比矩阵 (Float版本)
 * 
 * 内部将float参数转换为double后调用原有的double版本函数
 */
void motion_model_jacobian_float(const VehicleState_t *state,
                                 float velocity,
                                 float steering_angle,
                                 float dt,
                                 double F[5][5]) {
    if (!state || !F) {
        return;
    }
    
    /* 转换float为double并调用原有函数 */
    motion_model_jacobian(state,
                         (double)velocity,
                         (double)steering_angle,
                         (double)dt,
                         F);
}
