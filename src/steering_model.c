/**
 * @file steering_model.c
 * @brief 转向角转换模块实现
 * 
 * 实现方向盘转角到前轮转向角的转换，以及航向角速率的计算
 * 
 * REFACTORED: 不再使用全局变量，所有函数通过 SteeringModel_t *config 参数
 * 传递配置和状态。这样支持多实例和线程安全操作。
 */

#include "steering_model.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define __MCU__  // Define MCU for this build
/**
 * @brief 线性插值辅助函数
 * @param x 输入值
 * @param x0, y0 第一个点
 * @param x1, y1 第二个点
 * @return 插值结果
 */
static float linear_interpolate(float x, float x0, float y0, float x1, float y1) {
    if (fabsf(x1 - x0) < 1e-6f) {
        return y0;
    }
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

/**
 * @brief 初始化转向模型
 * 
 * @param config 转向系统配置参数
 * @return 0成功，-1失败
 */
int steering_config_init(SteeringModel_t *mode, SteeringConfig_t* steering_config) {
    if (!mode || !steering_config) {
        return -1;
    }
    
    /* 验证参数有效性 */
    if (steering_config->steering_ratio <= 0.0f ||
        steering_config->wheelbase <= 0.0f ||
        steering_config->max_steering_wheel_angle <= 0.0f ||
        steering_config->max_front_wheel_angle <= 0.0f) {
        return -1;
    }
    
    if (steering_config->steering_filter_alpha < 0.0f || 
        steering_config->steering_filter_alpha > 1.0f) {
        return -1;
    }
    
    /* 验证死区参数有效性 */
    if (steering_config->steering_deadzone < 0.0f) {
        return -1;
    }
    
    /* 重置状态 */
    mode->steering_state.filtered_steering_angle = 0.0f;
    mode->steering_state.prev_steering_angle = 0.0f;
    mode->steering_state.steering_rate = 0.0f;
    mode->steering_state.prev_timestamp = 0;
    mode->steering_state.left_wheel_angle = 0.0f;
    mode->steering_state.right_wheel_angle = 0.0f;
    mode->steering_state.mean_wheel_angle = 0.0f;
    
    /* 复制配置参数 */
    mode->steering_config = *steering_config;
    
    return 0;
}

/**
 * @brief 方向盘转角转前轮转向角
 * 
 * 转换：前轮转角 = 方向盘转角 / 齿比 
 * 改进：如果映射表可用，使用映射表查询，否则使用线性模型
 */
float steering_wheel_to_front_wheel(SteeringModel_t *config, float steering_wheel_angle) {
    if (!config) {
        return 0.0f;
    }
    
    float left_wheel, right_wheel;
    
    if (config->steering_mapping.valid) {
        /* 使用映射表 */
        steering_query_mapping(config, steering_wheel_angle, &left_wheel, &right_wheel);
        /* 返回平均值 */
        float front_wheel_angle = (left_wheel + right_wheel) / 2.0f;
        return steering_model_clamp_angle(config, front_wheel_angle);
    } else {
        /* 使用线性模型（后备方案） */
        float front_wheel_angle = steering_wheel_angle / config->steering_config.steering_ratio;
        
        /* 应用零偏和增益补偿 */
        front_wheel_angle = steering_model_apply_calibration(config, front_wheel_angle);
        
        /* 限制在有效范围内 */
        return steering_model_clamp_angle(config, front_wheel_angle);
    }
}

/**
 * @brief 前轮转向角转方向盘转角
 */
float front_wheel_to_steering_wheel(SteeringModel_t *config, float front_wheel_angle) {
    if (!config) {
        return 0.0f;
    }
    
    /* 逆变换：方向盘转角 = 前轮转角 * 齿比 */
    float steering_wheel_angle = front_wheel_angle * config->steering_config.steering_ratio;
    
    /* 限制在有效范围内 */
    if (fabsf(steering_wheel_angle) > config->steering_config.max_steering_wheel_angle) {
        steering_wheel_angle = (steering_wheel_angle > 0) ?
            config->steering_config.max_steering_wheel_angle :
            -config->steering_config.max_steering_wheel_angle;
    }
    
    return steering_wheel_angle;
}

/**
 * @brief 计算航向角速率（偏航角速率）
 * 
 * 基于二轮运动学模型：
 * yaw_rate = v * tan(delta) / L
 * 
 * 其中：
 * v = 纵向速度 (m/s)
 * delta = 前轮转向角 (rad)
 * L = 轴距 (m)
 */
float calculate_yaw_rate(SteeringModel_t *config, float velocity, float steering_angle) {
    if (!config) {
        return 0.0f;
    }
    
    /* 避免低速噪声 */
    if (fabsf(velocity) < 0.01f) {
        return 0.0f;
    }
    
    float tan_delta = tanf(steering_angle);
    float yaw_rate = velocity * tan_delta / config->steering_config.wheelbase;
    
    return yaw_rate;
}

/**
 * @brief 更新转向角并应用低通滤波
 */
float steering_model_update(SteeringModel_t *config, float steering_wheel_angle, uint64_t timestamp) {
    if (!config) {
        return 0.0f;
    }
    
    SteeringModelState_t *state = &config->steering_state;
    
    /* 步骤1：根据映射表查询前左轮和前右轮转角 */
    if (config->steering_mapping.valid) {
        steering_query_mapping(config, steering_wheel_angle, 
                             &state->left_wheel_angle,
                             &state->right_wheel_angle);
    } else {
        /* 使用线性模型 */
        float basic = steering_wheel_angle / config->steering_config.steering_ratio;
        state->left_wheel_angle = basic;
        state->right_wheel_angle = basic;
    }
    
    /* 步骤2：计算平均转向角 */
    state->mean_wheel_angle = 
        (state->left_wheel_angle + state->right_wheel_angle) / 2.0f;
    
    /* 步骤3：应用校准 */
    float calibrated_angle = steering_model_apply_calibration(config, state->mean_wheel_angle);
    
    /* 步骤4：计算转向角速率 */
    if (state->prev_timestamp > 0) {
        float dt = (timestamp - state->prev_timestamp) / 1000.0f;
        if (dt > 0.0f && dt < 1.0f) {
            state->steering_rate = 
                (calibrated_angle - state->prev_steering_angle) / dt;
        }
    }
    
    /* 步骤5：应用低通滤波 */
    if (config->steering_config.enable_steering_filter) {
        float alpha = config->steering_config.steering_filter_alpha;
        state->filtered_steering_angle = 
            alpha * state->filtered_steering_angle + 
            (1.0f - alpha) * calibrated_angle;
    } else {
        state->filtered_steering_angle = calibrated_angle;
    }
    
    /* 步骤5.5：应用死区处理 */
    /* 当转向角小于死区范围时，设为零 */
    if (config->steering_config.steering_deadzone > 0.0f) {
        if (fabsf(state->filtered_steering_angle) < config->steering_config.steering_deadzone) {
            state->filtered_steering_angle = 0.0f;
        }
    }
    
    /* 步骤6：更新状态 */
    state->prev_steering_angle = calibrated_angle;
    state->prev_timestamp = timestamp;
    
    return state->filtered_steering_angle;
}

/**
 * @brief 获取滤波后的前轮转向角
 */
float steering_model_get_filtered_angle(SteeringModel_t *config) {
    if (!config) {
        return 0.0f;
    }
    return config->steering_state.filtered_steering_angle;
}

/**
 * @brief 获取转向角速率
 */
float steering_model_get_steering_rate(SteeringModel_t *config) {
    if (!config) {
        return 0.0f;
    }
    return config->steering_state.steering_rate;
}

/**
 * @brief 应用转向零偏和增益补偿
 * 
 * 补偿公式: compensated_angle = (angle - offset) * gain
 */
float steering_model_apply_calibration(SteeringModel_t *config, float angle) {
    if (!config) {
        return angle;
    }
    
    float compensated = (angle - config->steering_config.steering_angle_offset) * 
                         config->steering_config.steering_angle_gain;
    return compensated;
}

/**
 * @brief 限制转向角在有效范围内
 */
float steering_model_clamp_angle(SteeringModel_t *config, float angle) {
    if (!config) {
        return angle;
    }
    
    float max_angle = config->steering_config.max_front_wheel_angle;
    
    if (angle > max_angle) {
        return max_angle;
    } else if (angle < -max_angle) {
        return -max_angle;
    }
    
    return angle;
}

/**
 * @brief 重置转向模型状态
 */
void steering_model_reset(SteeringModel_t *config) {
    if (!config) {
        return;
    }
    
    config->steering_state.filtered_steering_angle = 0.0f;
    config->steering_state.prev_steering_angle = 0.0f;
    config->steering_state.steering_rate = 0.0f;
    config->steering_state.prev_timestamp = 0;
    config->steering_state.left_wheel_angle = 0.0f;
    config->steering_state.right_wheel_angle = 0.0f;
    config->steering_state.mean_wheel_angle = 0.0f;
}

/* ============================================================================
 * 映射表操作函数
 * ============================================================================ */

/**
 * @brief 初始化转向映射表
 * 
 * @param config 转向模型配置指针
 * @param sw_samples 方向盘转角数组 (弧度)
 * @param left_samples 前左轮转角数组 (弧度)
 * @param right_samples 前右轮转角数组 (弧度)
 * @param num_points 映射点数
 * @return 0 成功，-1 失败
 */
int steering_mapping_init(SteeringModel_t *config,
                         const float *sw_samples,
                         const float *left_samples,
                         const float *right_samples,
                         int num_points) {
    if (!config || !sw_samples || !left_samples || !right_samples || num_points < 2) {
        return -1;
    }
 #ifdef __SOC__   

    /* 清理旧的映射表 */
    steering_mapping_cleanup(config);
    
    /* 分配内存 */
    config->steering_mapping.steering_wheel = (float *)malloc(num_points * sizeof(float));
    config->steering_mapping.left_wheel = (float *)malloc(num_points * sizeof(float));
    config->steering_mapping.right_wheel = (float *)malloc(num_points * sizeof(float));
    
    if (!config->steering_mapping.steering_wheel || 
        !config->steering_mapping.left_wheel || 
        !config->steering_mapping.right_wheel) {
        steering_mapping_cleanup(config);
        return -1;
    }
    
    /* 复制数据 */
    memcpy(config->steering_mapping.steering_wheel, sw_samples, num_points * sizeof(float));
    memcpy(config->steering_mapping.left_wheel, left_samples, num_points * sizeof(float));
    memcpy(config->steering_mapping.right_wheel, right_samples, num_points * sizeof(float));
    
    config->steering_mapping.num_points = num_points;
    config->steering_mapping.valid = true;
    
    return 0;
#elif defined (__MCU__)
    config->steering_mapping.steering_wheel = (float *)sw_samples;
    config->steering_mapping.left_wheel = (float *)left_samples;
    config->steering_mapping.right_wheel = (float *)right_samples;
    config->steering_mapping.num_points = num_points;
    config->steering_mapping.valid = true;
    return 0;   
#else
#error "Must define __SOC__ or __MCU__"
#endif
}

int steering_mapping_init_ex(SteeringModel_t *config, const SteeringMappingTable_t *mapping_table)
{
    if(!config || !mapping_table || !mapping_table->valid){
        return -1;
    }
    return steering_mapping_table_copy(&config->steering_mapping, mapping_table);
}

int steering_mapping_table_init(SteeringMappingTable_t *mapping_table, const float *sw_samples, const float *left_samples, const float *right_samples, int num_points)
{
    if (!mapping_table || !sw_samples || !left_samples || !right_samples || num_points < 2) {
        return -1;
    }
 #ifdef __SOC__   

    /* 清理旧的映射表 */
    steering_mapping_cleanup(config);
    
    /* 分配内存 */
    mapping_table->steering_wheel = (float *)malloc(num_points * sizeof(float));
    mapping_table->left_wheel = (float *)malloc(num_points * sizeof(float));
    mapping_table->right_wheel = (float *)malloc(num_points * sizeof(float));
    
    if (!mapping_table->steering_wheel || 
        !mapping_table->left_wheel || 
        !mapping_table->right_wheel) {
        steering_mapping_cleanup(config);
        return -1;
    }
    
    /* 复制数据 */
    memcpy(mapping_table->steering_wheel, sw_samples, num_points * sizeof(float));
    memcpy(mapping_table->left_wheel, left_samples, num_points * sizeof(float));
    memcpy(mapping_table->right_wheel, right_samples, num_points * sizeof(float));
    
    mapping_table->num_points = num_points;
    mapping_table->valid = true;
    
    return 0;
#elif defined (__MCU__)
    mapping_table->steering_wheel = (float *)sw_samples;
    mapping_table->left_wheel = (float *)left_samples;
    mapping_table->right_wheel = (float *)right_samples;
    mapping_table->num_points = num_points;
    mapping_table->valid = true;
    return 0;   
#else
#error "Must define __SOC__ or __MCU__"
#endif
}

int steering_mapping_table_copy(SteeringMappingTable_t *dest, const SteeringMappingTable_t *src)
{
    if(!dest || !src || !src->valid){
        return -1;
    }   
#ifdef __SOC__
    /* 清理旧的映射表 */
    steering_mapping_table_cleanup(dest);   
    /* 分配内存 */
    dest->steering_wheel = (float *)malloc(src->num_points * sizeof(float));
    dest->left_wheel = (float *)malloc(src->num_points * sizeof(float));
    dest->right_wheel = (float *)malloc(src->num_points * sizeof(float));

    if (!dest->steering_wheel || 
        !dest->left_wheel || 
        !dest->right_wheel) {
        steering_mapping_table_cleanup(dest);
        return -1;
    }

    memcpy(dest->steering_wheel, src->steering_wheel, src->num_points * sizeof(float));
    memcpy(dest->left_wheel, src->left_wheel, src->num_points * sizeof(float));
    memcpy(dest->right_wheel, src->right_wheel, src->num_points * sizeof(float));

    dest->num_points = src->num_points;
    dest->valid = true; 
    return 0;
#elif defined (__MCU__)  
    dest->steering_wheel = src->steering_wheel;
    dest->left_wheel = src->left_wheel;
    dest->right_wheel = src->right_wheel;
    dest->num_points = src->num_points;
    dest->valid = true;
    return 0;
#else
#error "Must define __SOC__ or __MCU__"
#endif
    
}

/**
 * @brief 查询映射表获取前轮转角
 * 
 * @param config 转向模型配置指针
 * @param steering_wheel_angle 方向盘转角 (弧度)
 * @param left_wheel_angle 输出：前左轮转角 (弧度)
 * @param right_wheel_angle 输出：前右轮转角 (弧度)
 * @return 0 成功，-1 失败
 */
int steering_query_mapping(SteeringModel_t *config,
                         float steering_wheel_angle,
                         float *left_wheel_angle,
                         float *right_wheel_angle) {
    if (!config || !left_wheel_angle || !right_wheel_angle) {
        return -1;
    }
    
    if (!config->steering_mapping.valid) {
        return -1;
    }
    
    /* 查找插值区间 */
    int idx = -1;
    for (idx = 0; idx < config->steering_mapping.num_points - 1; idx++) {
        if (steering_wheel_angle >= config->steering_mapping.steering_wheel[idx] &&
            steering_wheel_angle <= config->steering_mapping.steering_wheel[idx + 1]) {
            break;
        }
    }
    
    /* 边界处理 */
    if (idx >= config->steering_mapping.num_points - 1) {
        idx = config->steering_mapping.num_points - 2;
    }
    if (idx < 0) {
        //printf("steering_query_mapping: steering_wheel_angle out of range\n");
        idx = 0;
    }
    
    /* 线性插值查询 */
    float x0 = config->steering_mapping.steering_wheel[idx];
    float x1 = config->steering_mapping.steering_wheel[idx + 1];
    
    *left_wheel_angle = linear_interpolate(
        steering_wheel_angle,
        x0, config->steering_mapping.left_wheel[idx],
        x1, config->steering_mapping.left_wheel[idx + 1]
    );
    
    *right_wheel_angle = linear_interpolate(
        steering_wheel_angle,
        x0, config->steering_mapping.right_wheel[idx],
        x1, config->steering_mapping.right_wheel[idx + 1]
    );
    
    return 0;
}

/**
 * @brief 计算平均前轮转角
 * 
 * @param config 转向模型配置指针
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 平均值 (弧度)
 */
float steering_compute_mean_angle(SteeringModel_t *config, float steering_wheel_angle) {
    if (!config) {
        return 0.0f;
    }
    
    /* 查询映射表获得前轮转角 */
    float left = 0.0f, right = 0.0f;
    
    if (steering_query_mapping(config, steering_wheel_angle, &left, &right) == 0) {
        return (left + right) / 2.0f;
    }
    
    /* 映射表无效，返回简单平均 */
    return (config->steering_state.left_wheel_angle + config->steering_state.right_wheel_angle) / 2.0f;
}

/**
 * @brief 使用Ackermann模型计算航向角速率
 * 
 * @param config 转向模型配置指针
 * @param velocity 车辆纵向速度 (m/s)
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 航向角速率 (rad/s)
 */
float steering_calculate_yaw_rate_ackermann(SteeringModel_t *config,
                                            float velocity,
                                            float steering_wheel_angle) {
    if (!config || fabsf(velocity) < 0.01f || config->steering_config.wheelbase < 0.01f) {
        return 0.0f;
    }
    
    /* 查询映射表获得前轮转角 */
    float left_rad = 0.0f, right_rad = 0.0f;
    
    if (steering_query_mapping(config, steering_wheel_angle, &left_rad, &right_rad) != 0) {
        /* 映射表无效，使用简单齿比模型 */
        left_rad = steering_wheel_angle / config->steering_config.steering_ratio;
        right_rad = steering_wheel_angle / config->steering_config.steering_ratio;
    }
    
    /* Ackermann模型：考虑左右轮转角差异 */
    float yaw_rate = velocity * (tanf(left_rad) + tanf(right_rad)) / 
                     (2.0f * config->steering_config.wheelbase);
    
    return yaw_rate;
}

/**
 * @brief 获取当前的前轮转角（左右分别）
 * 
 * @param config 转向模型配置指针
 * @param left_wheel 输出：前左轮转角
 * @param right_wheel 输出：前右轮转角
 * @param mean_angle 输出：平均转向角
 * @return 0 成功，-1 失败
 */
int steering_get_wheel_angles(SteeringModel_t *config,
                             float *left_wheel,
                             float *right_wheel,
                             float *mean_angle) {
    if (!config || !left_wheel || !right_wheel || !mean_angle) {
        return -1;
    }
    
    *left_wheel = config->steering_state.left_wheel_angle;
    *right_wheel = config->steering_state.right_wheel_angle;
    *mean_angle = (config->steering_state.left_wheel_angle + config->steering_state.right_wheel_angle) / 2.0f;
    
    return 0;
}

/**
 * @brief 清理映射表资源
 * 
 * @param config 转向模型配置指针
 */
void steering_mapping_cleanup(SteeringModel_t *config) {
    if (!config) {
        return;
    }
    
    if (config->steering_mapping.steering_wheel) {
        free(config->steering_mapping.steering_wheel);
        config->steering_mapping.steering_wheel = NULL;
    }
    
    if (config->steering_mapping.left_wheel) {
        free(config->steering_mapping.left_wheel);
        config->steering_mapping.left_wheel = NULL;
    }
    
    if (config->steering_mapping.right_wheel) {
        free(config->steering_mapping.right_wheel);
        config->steering_mapping.right_wheel = NULL;
    }
    
    config->steering_mapping.num_points = 0;
    config->steering_mapping.valid = false;
}
/* ========== Phase 2 (P1) RTK校准集成实现 ========== */

/**
 * @brief 获取当前转向角增益
 */
float steering_model_get_gain(SteeringModel_t *config) {
    if (!config) {
        return 1.0f;  /* 默认无补偿 */
    }
    
    return config->steering_config.steering_angle_gain;
}

/**
 * @brief 获取当前转向角零偏
 */
float steering_model_get_offset(SteeringModel_t *config) {
    if (!config) {
        return 0.0f;  /* 默认无偏置 */
    }
    
    return config->steering_config.steering_angle_offset;
}

/**
 * @brief 更新转向角增益
 */
int steering_model_update_gain(SteeringModel_t *config, float new_gain) {
    if (!config) {
        return -1;
    }
    
    /* 验证增益值合理性 */
    /* 允许范围 [0.5, 1.5]，推荐范围 [0.8, 1.2] */
    if (new_gain < 0.5f || new_gain > 1.5f) {
        return -1;  /* 增益超出合理范围 */
    }
    
    config->steering_config.steering_angle_gain = new_gain;
    return 0;
}

/**
 * @brief 应用 RTK 校准结果
 */
int steering_model_apply_rtk_calibration(SteeringModel_t *config,
                                         const void *calibration) {
    if (!config || !calibration) {
        return -1;
    }
    
    /* 由于 void* 指针，我们需要通过 struct 访问 CalibrationParams_t */
    /* 这里定义一个临时结构体指针来访问数据 */
    typedef struct {
        float wheel_calibration_factor;
        float steering_angle_offset;
        float steering_angle_gain;
        /* 其他字段不在此处列举 */
    } CalibrationParams_t;
    
    const CalibrationParams_t *calib_params = (const CalibrationParams_t *)calibration;
    
    /* 应用转向角零偏 */
    config->steering_config.steering_angle_offset = calib_params->steering_angle_offset;
    
    /* 应用转向角增益，并验证范围 */
    if (calib_params->steering_angle_gain < 0.5f || calib_params->steering_angle_gain > 1.5f) {
        /* 增益超出合理范围，使用默认值 */
        config->steering_config.steering_angle_gain = 1.0f;
        return -1;
    }
    
    config->steering_config.steering_angle_gain = calib_params->steering_angle_gain;
    
    return 0;
}