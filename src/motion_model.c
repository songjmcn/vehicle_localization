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
 * theta_{k+1} = theta_k + sign(direction)*yaw_rate * dt
 * 
 * 航向角速率计算：
 * - 如果配置了转向映射表：使用Ackermann模型
 *   yaw_rate = v * (tan(delta_left) + tan(delta_right)) / (2*L)
 * - 否则：使用单轨模型
 *   yaw_rate = v * tan(delta) / L
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
    
    /* ========== 改进：查询转向映射表 ========== */
    double left_wheel_angle = steering_angle;
    double right_wheel_angle = steering_angle;
    
    if (steering_query_mapping(steering_angle, &left_wheel_angle, &right_wheel_angle) == 0) {
        /* 映射表查询成功，已获得前左右轮转角 */
        /* 注：实际转角已存储在steering_model的内部状态中 */
    } else {
        /* 映射表不可用，使用简单齿比转换 */
        left_wheel_angle = steering_angle;
        right_wheel_angle = steering_angle;
    }
    
    /* 转向角在后退时反向 */
    double effective_left_delta = sign_v * left_wheel_angle;
    double effective_right_delta = sign_v * right_wheel_angle;
    
    /* ========== 改进：使用Ackermann几何计算航向角速率 ========== */
    double yaw_rate;
    
    /* 检查是否使用Ackermann模型 */
    double left_delta_diff = fabs(left_wheel_angle - right_wheel_angle);
    if (left_delta_diff > 0.001) {
        /* Ackermann模型：考虑左右轮转角差异 */
        /* yaw_rate = v * (tan(delta_left) + tan(delta_right)) / (2*L) */
        double tan_left = tan(effective_left_delta);
        double tan_right = tan(effective_right_delta);
        yaw_rate = abs_v * (tan_left + tan_right) / (2.0 * WHEELBASE);
    } else {
        /* 单轨模型：左右轮转角相同 */
        /* yaw_rate = v * tan(delta) / L */
        double effective_delta = sign_v * steering_angle;
        double tan_delta = tan(effective_delta);
        yaw_rate = abs_v * tan_delta / WHEELBASE;
    }
    
    /* 位置更新 */
    double mean_delta = (effective_left_delta + effective_right_delta) / 2.0;
    double cos_term = cos(state->theta + mean_delta);
    double sin_term = sin(state->theta + mean_delta);
    
    new_state.x = state->x + abs_v * cos_term * dt;
    new_state.y = state->y + abs_v * sin_term * dt;
    
    /* 航向角更新 */
    new_state.theta = state->theta + yaw_rate * dt;
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
