/**
 * @file motion_model.c
 * @brief ISO 8855标准运动模型实现（后轴中心原点）
 *
 * @details
 * 本模块实现符合ISO 8855标准的车辆单轨运动模型：
 *
 * **坐标系定义：**
 * - 原点：后轮轴中心（RAC - Rear Axle Center）
 * - X轴：纵向（前方为正）
 * - Y轴：横向（左方为正）
 * - 参考帧：车体固定坐标系（body-fixed）
 *
 * **运动方程：**
 * ```
 * dx/dt = v·cos(ψ)
 * dy/dt = v·sin(ψ)
 * dψ/dt = (v/L)·tan(δ_eff)
 * ```
 * 其中：
 *   v = 后轮轴中心纵向速度 (m/s)，来自后轮编码器
 *   ψ = 车身偏航角 (rad)
 *   δ = 前轮转向角 (front wheel angle, rad)
 *   δ_eff = sign(v)·δ（后退时自动反向）
 *   L = 轴距 (m)
 *
 * **非完整约束：**
 * - 后轮中心的速度必须沿车身纵向轴
 * - 后轮不能侧滑（vy = 0 at rear axle）
 *
 * **实现特性：**
 * - 支持前进和后退运动
 * - 后退时自动反向转向角以维持直观的控制
 * - 支持Ackermann转向几何
 */

#include "motion_model.h"
#include "localization_adapter.h"
#include "localization_types_internal.h"
#include "steering_model.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

int motion_model_init(SteeringModel_t *mode, SteeringConfig_t *config,SteeringMappingTable_t *mapping_table) {
    if(mode == NULL || config == NULL || mapping_table == NULL){ 
        return -1;
    }   
    /* 初始化运动模型参数 */
    
    /* 初始化转向模型 */
    if(steering_config_init(mode,config) != 0){
        return -1;
    }
    if (mapping_table->valid && steering_mapping_init_ex(mode,mapping_table)!=0)
    {
        /* code */
        return -1;
    }
    return 0;
    
}

/**
 * @brief 单轨模型的预测函数
 * 
 * 倒车时转向角自动反向：
 * - 有效转向角：δ_eff = sign(v)·δ（后退时反向）
 * - 航向角速率：dψ/dt = (v/L)·tan(δ_eff)
 * - 位置更新：dx = v·cos(ψ), dy = v·sin(ψ)（使用有符号速度）
 * 
 * 具体计算流程：
 * 1. 判断运动方向（direction）
 * 2. 查询转向映射表获取左右轮转角
 * 3. 计算有效转向角：effective_delta = sign(direction)·steering_angle
 * 4. 计算航向角速率：yaw_rate = v_signed·tan(effective_delta)/L
 * 5. 更新航向角：heading = heading + yaw_rate·dt
 * 6. 使用有符号速度更新位置
 */
VehicleState_t motion_model_predict(SteeringModel_t *steering_config,
                                    const VehicleState_t *state,
                                    float velocity,
                                    float steering_angle,
                                    double dt) {
    VehicleState_t new_state = *state;
    
    /* === 关键修复：根据方向判断是否静止，而不仅基于速度 ===
     * 原因：车辆可能在很低的速度下缓慢前进（如0.03 m/s），
     * 此时不应该跳过积分，否则低速运动的距离会被丢失。
     *
     * 判断逻辑：
     * - 如果方向是DIRECTION_STATIONARY，则完全静止
     * - 否则，使用方向信息执行位置积分，即使速度很低
     */

    /* 获取已计算的方向（由上层localization_system计算） */
    VehicleDirection_t direction = state->direction;

    /* 边界情况：如果方向未计算，基于速度判断 */
    if (direction == DIRECTION_INVALID) {
        printf("direction invalid, derive from velocity\n");
        direction = (fabs(velocity) < V_STATIC_THRESHOLD) ?
                    DIRECTION_STATIONARY :
                    ((velocity > 0.0) ? DIRECTION_FORWARD : DIRECTION_REVERSE);
    }

    /* 检查是否完全静止（基于方向，而不仅基于速度） */
    if (direction == DIRECTION_STATIONARY) {
        /* 完全静止，只保持位置和角度 */
        new_state.x = state->x;
        new_state.y = state->y;
        new_state.heading = state->heading;
        new_state.v = 0.0;
        new_state.steering_angle = steering_angle;
        new_state.gear = state->gear;
        new_state.direction = DIRECTION_STATIONARY;
        //printf("--stationary state\n");
        return new_state;
    }
    
    /* === 关键修复：即使速度很小，只要有运动方向或转向角，就应该更新heading ===
     * 原因：Kalman滤波会基于观测到的转向角进行heading更新
     * 如果motion_model_predict完全跳过更新，heading会与转向角矛盾
     * 特别是在低速倒车或停车时，这非常重要
     * 
     * 即使velocity < V_STATIC_THRESHOLD，我们仍然应该更新heading
     * 例如：倒车时转向角改变，即使速度很小，heading也应该变化
     */
    /* 关键：不再在这里检查velocity大小，而是允许所有非静止方向的情况继续处理 */
    
    /* 根据已计算的方向获取符号（当前未在此函数中直接使用，但在雅可比矩阵中使用） */
    int sign_v = get_sign_v_from_direction(direction);
    (void)sign_v;  /* 标记为已知但未在此使用，避免编译警告 */

    /* 🔴 关键修复：使用有符号速度而不是绝对值
     * ISO 8855运动学方程是：
     *   dx = v·cos(ψ)  where v can be positive(forward) or negative(reverse)
     *   dy = v·sin(ψ)
     *
     * localization_system_process已确保：
     *   v > 0 when direction == DIRECTION_FORWARD
     *   v < 0 when direction == DIRECTION_REVERSE
     *
     * 这样可以自动处理后退时的方向反向：
     * - 后退时v < 0，导致dx和dy反向，x坐标减少（正确！）
     * - 转向角也在后退时反向(get_effective_steering_angle)
     * - 从而产生直观的后退运动
     */
    double v_signed = velocity;  /* 使用有符号速度 */

    /* ========== 改进：参数 steering_angle 语义已统一为前轮转向角 ========== */
    /* 现在 motion_model 接收的 steering_angle 已为前轮转向角（front_wheel_angle）
     * 因此直接使用该值计算 yaw_rate，不再从方向盘角做映射
     */
    // float left_wheel_angle = steering_angle;
    // float right_wheel_angle = steering_angle;
    double front_wheel_angle = steering_angle;
    front_wheel_angle = front_wheel_angle + steering_config->steering_config.ackermann_factor *front_wheel_angle *front_wheel_angle *front_wheel_angle;
    double tan_delta = tan(front_wheel_angle);
    double yaw_rate = v_signed * tan_delta / steering_config->steering_config.wheelbase;
    double heading_change = yaw_rate * dt;
    double new_heading = state->heading + heading_change;
    
    /* === 🔴 关键修复：使用初始heading而不是更新后的heading ===
     * 离散时间积分的正确做法：
     *   - 使用时间步开始时的heading来计算位置变化
     *   - 而不是步结束时的new_heading
     * 原因：
     *   1. 数值稳定性：避免heading变化累积
     *   2. 物理一致性：位移应使用初始方向而非最终方向
     *   3. 倒车时特别重要：错误的heading导致Y坐标错误累积
     */
    double cos_term = cos(state->heading);
    double sin_term = sin(state->heading);
    
    /* 使用有符号速度进行位置更新
     * 当v_signed为负(后退)时：
     * - new_x = x + (-|v|) * cos(heading) * dt  即 x减少
     * - new_y = y + (-|v|) * sin(heading) * dt  即y反向增长
     * 这与后退运动的直观概念一致
     */
    new_state.x = state->x + v_signed * cos_term * dt;
    new_state.y = state->y + v_signed * sin_term * dt;
    /* 航向角更新 
     * 
     * ===== ISO 8855 范围说明 =====
     * 本系统使用 [-π, π] 范围进行heading角处理
     * 标准范围: ISO 8855定义为[0, 2π]
     * 
     * 实际范围: [-π, π]（当前实现）
     * 原因:
     *   1. 数值计算友好：避免0° = 360°的歧义
     *   2. 与C库标准一致：atan2()输出范围为[-π, π]
     *   3. 简化卡尔曼滤波：残差计算避免π环绕
     *   4. 计算效率优先于ISO严格遵循
     * 
     * 约束条件:
     *   - new_heading总是被normalize到[-π, π]范围
     *   - 大尺度轨迹（>2π总转角）会出现累积误差
     *   - 标定中heading差>π的数据需特殊处理
     */
    //new_state.heading = state->heading + yaw_rate * dt;

    //DO NOT call normalize_angle here for better accuracy in kalman filter
    new_state.heading = new_heading;
    
    new_state.v = velocity;
    //new_state.steering_angle = steering_angle;
    new_state.steering_angle = steering_angle;  /* 使用前轮转向角 */
    new_state.gear = state->gear;
    new_state.direction = direction;
    
    return new_state;
}

/**
 * @brief 计算雅可比矩阵
 */
void motion_model_jacobian(const VehicleState_t *state,
                          SteeringModel_t *steering_config,
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
    
    /* === 关键修复：即使速度很小，也应该计算heading相关的偏导数 ===
     * 原因：当velocity接近0但steering_angle改变时，
     * yaw_rate = v*tan(delta)/L 仍然接近0，但heading应该略微变化
     * 更重要的是：F[2][4]（heading关于steering_angle的偏导）在任何速度下都应该计算
     * 
     * 如果完全跳过雅可比矩阵计算，Kalman滤波的协方差会被错误地保持为单位矩阵
     * 这会导致Kalman增益计算错误
     */
    /* 允许所有情况下计算雅可比矩阵，不再有velocity阈值检查 */

    double theta = state->heading;
    
    /* === 优化：直接使用已计算的方向，避免重复计算 === */
    /* 方向由上层 localization_system 在一处计算，这里直接使用 */
    VehicleDirection_t direction = state->direction;

    /* 如果方向未计算（边界情况），进行备选处理 */
    if (direction == DIRECTION_INVALID) {
        direction = (fabs(velocity) < V_STATIC_THRESHOLD) ?
                    DIRECTION_STATIONARY :
                    ((velocity > 0.0) ? DIRECTION_FORWARD : DIRECTION_REVERSE);
    }
    
    int sign_v = get_sign_v_from_direction(direction);
    
    /* === 位置关于航向角的偏导 ===
     * 根据ISO 8855运动学方程：
     *   x = x_0 + v·cos(ψ)·dt
     *   y = y_0 + v·sin(ψ)·dt
     * 
     * 偏导数（仅关于航向角θ，不涉及转向角δ）：
     *   ∂x/∂θ = -v·sin(θ)·dt
     *   ∂y/∂θ = v·cos(θ)·dt
     */
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    
    /* ∂x/∂θ = -v·sin(θ)·dt   （v可以是负数） */
    F[0][2] = -velocity * sin_theta * dt;
    /* ∂y/∂θ = v·cos(θ)·dt    （v可以是负数） */
    F[1][2] = velocity * cos_theta * dt;
    
    /* 位置关于速度的偏导 */
    /* ∂x/∂v = cos(θ)·dt （velocity已包含符号信息，倒车时v<0自动处理） */
    F[0][3] = cos_theta * dt;
    /* ∂y/∂v = sin(θ)·dt （velocity已包含符号信息，倒车时v<0自动处理） */
    F[1][3] = sin_theta * dt;
    
    /* === 航向角关于控制量的偏导 ===
     * 航向角速率方程：dψ/dt = (v/L)·tan(δ_eff)
     * 其中：δ_eff = sign(direction)·δ（倒车时反向）
     * 
     * ∂ψ/∂v = tan(δ)/L
     * ∂ψ/∂δ = v/(L·cos²(δ))
     */
    double cos_delta = cos(steering_angle);
    if (fabs(cos_delta) > 1e-6) {
        double tan_delta = tan(steering_angle);
        
        /* ∂θ/∂v = tan(δ)·dt/L   （velocity已包含符号） */
        double L = steering_config->steering_config.wheelbase;
        F[2][3] = tan_delta / L * dt;

        /* ∂θ/∂δ = v·dt/(L·cos²(δ))   （velocity已包含符号） */
        F[2][4] = velocity / (L * cos_delta * cos_delta) * dt;
    }
}

/* ========== Float API Implementation (for TC397 MCU compatibility) ========== */

/**
 * @brief 使用单轨模型预测车辆状态 (Float版本)
 * 
 * 内部将float参数转换为double后调用原有的double版本函数
 */
VehicleState_t motion_model_predict_float(SteeringModel_t *steering_config,
                                          const VehicleState_t *state,
                                          float velocity,
                                          float steering_angle,
                                          float dt) {
    if (!state) {
        VehicleState_t empty = {0};
        return empty;
    }
    
    /* 转换float为double并调用原有函数 */
    return motion_model_predict(steering_config,
                               state,
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
                                 SteeringModel_t *steering_config,
                                 float velocity,
                                 float steering_angle,
                                 float dt,
                                 double F[5][5]) {
    if (!state || !F) {
        return;
    }

    /* 转换float为double并调用原有函数 */
    motion_model_jacobian(state,
                         steering_config,
                         (double)velocity,
                         (double)steering_angle,
                         (double)dt,
                         F);
}
