/**
 * @file steering_model.c
 * @brief 转向角转换模块实现
 * 
 * 实现方向盘转角到前轮转向角的转换，以及航向角速率的计算
 */

#include "steering_model.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 全局转向模型状态 */
static SteeringConfig_t g_steering_config = {
    .steering_ratio = 16.0f,           /* 默认齿比16:1 */
    .max_steering_wheel_angle = 3.14159f, /* ±180° */
    .max_front_wheel_angle = 0.628f,   /* ±36° */
    .wheelbase = 2.7f,                 /* 轴距2.7m */
    .steering_angle_offset = 0.0f,     /* 无零偏 */
    .steering_angle_gain = 1.0f,       /* 增益为1 */
    .enable_steering_filter = false,   /* 默认无滤波 */
    .steering_filter_alpha = 0.1f      /* 滤波系数10% */
};

static SteeringModelState_t g_steering_state = {
    .filtered_steering_angle = 0.0,
    .prev_steering_angle = 0.0,
    .steering_rate = 0.0,
    .prev_timestamp = 0
};

/**
 * @brief 初始化转向模型
 */
int steering_model_init(const SteeringConfig_t *config) {
    if (!config) {
        return -1;
    }
    
    /* 验证参数有效性 */
    if (config->steering_ratio <= 0.0f ||
        config->wheelbase <= 0.0f ||
        config->max_steering_wheel_angle <= 0.0f ||
        config->max_front_wheel_angle <= 0.0f) {
        return -1;
    }
    
    if (config->steering_filter_alpha < 0.0f || config->steering_filter_alpha > 1.0f) {
        return -1;
    }
    
    /* 保存配置 */
    g_steering_config = *config;
    
    /* 重置状态 */
    g_steering_state.filtered_steering_angle = 0.0;
    g_steering_state.prev_steering_angle = 0.0;
    g_steering_state.steering_rate = 0.0;
    g_steering_state.prev_timestamp = 0;
    
    return 0;
}

/**
 * @brief 方向盘转角转前轮转向角
 */
double steering_wheel_to_front_wheel(double steering_wheel_angle) {
    /* 转换：前轮转角 = 方向盘转角 / 齿比 
     * 可以根据需求自己增加映射表法来计算转向角
     */
    double front_wheel_angle = steering_wheel_angle / g_steering_config.steering_ratio;
    
    /* 应用零偏和增益补偿 */
    front_wheel_angle = steering_model_apply_calibration(front_wheel_angle);
    
    /* 限制在有效范围内 */
    return steering_model_clamp_angle(front_wheel_angle);
}

/**
 * @brief 前轮转向角转方向盘转角
 */
double front_wheel_to_steering_wheel(double front_wheel_angle) {
    /* 逆变换：方向盘转角 = 前轮转角 * 齿比 */
    double steering_wheel_angle = front_wheel_angle * g_steering_config.steering_ratio;
    
    /* 限制在有效范围内 */
    if (fabs(steering_wheel_angle) > g_steering_config.max_steering_wheel_angle) {
        steering_wheel_angle = (steering_wheel_angle > 0) ?
            g_steering_config.max_steering_wheel_angle :
            -g_steering_config.max_steering_wheel_angle;
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
double calculate_yaw_rate(double velocity, double steering_angle) {
    /* 避免低速噪声 */
    if (fabs(velocity) < 0.01) {
        return 0.0;
    }
    
    double tan_delta = tan(steering_angle);
    double yaw_rate = velocity * tan_delta / g_steering_config.wheelbase;
    
    return yaw_rate;
}

/**
 * @brief 更新转向角并应用低通滤波
 */
double steering_model_update(double steering_wheel_angle, uint32_t timestamp) {
    /* 步骤1：方向盘转角转前轮转向角 */
    double front_wheel_angle = steering_wheel_to_front_wheel(steering_wheel_angle);
    
    /* 步骤2：计算转向角速率 */
    if (g_steering_state.prev_timestamp > 0) {
        double dt = (timestamp - g_steering_state.prev_timestamp) / 1000.0;
        if (dt > 0 && dt < 1.0) {
            g_steering_state.steering_rate = 
                (front_wheel_angle - g_steering_state.prev_steering_angle) / dt;
        }
    }
    
    /* 步骤3：应用低通滤波 */
    if (g_steering_config.enable_steering_filter) {
        double alpha = g_steering_config.steering_filter_alpha;
        g_steering_state.filtered_steering_angle = 
            alpha * g_steering_state.filtered_steering_angle + 
            (1.0 - alpha) * front_wheel_angle;
    } else {
        g_steering_state.filtered_steering_angle = front_wheel_angle;
    }
    
    /* 步骤4：更新状态 */
    g_steering_state.prev_steering_angle = front_wheel_angle;
    g_steering_state.prev_timestamp = timestamp;
    
    return g_steering_state.filtered_steering_angle;
}

/**
 * @brief 获取滤波后的前轮转向角
 */
double steering_model_get_filtered_angle(void) {
    return g_steering_state.filtered_steering_angle;
}

/**
 * @brief 获取转向角速率
 */
double steering_model_get_steering_rate(void) {
    return g_steering_state.steering_rate;
}

/**
 * @brief 应用转向零偏和增益补偿
 * 
 * 补偿公式: compensated_angle = (angle - offset) * gain
 */
double steering_model_apply_calibration(double angle) {
    double compensated = (angle - g_steering_config.steering_angle_offset) * 
                         g_steering_config.steering_angle_gain;
    return compensated;
}

/**
 * @brief 限制转向角在有效范围内
 */
double steering_model_clamp_angle(double angle) {
    double max_angle = g_steering_config.max_front_wheel_angle;
    
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
void steering_model_reset(void) {
    g_steering_state.filtered_steering_angle = 0.0;
    g_steering_state.prev_steering_angle = 0.0;
    g_steering_state.steering_rate = 0.0;
    g_steering_state.prev_timestamp = 0;
}

/**
 * @brief 获取当前转向系统配置
 */
const SteeringConfig_t* steering_model_get_config(void) {
    return &g_steering_config;
}

/**
 * @brief 更新转向系统配置
 */
int steering_model_update_config(const SteeringConfig_t *config) {
    if (!config) {
        return -1;
    }
    
    /* 验证参数 */
    if (config->steering_ratio <= 0.0f ||
        config->wheelbase <= 0.0f ||
        config->max_steering_wheel_angle <= 0.0f ||
        config->max_front_wheel_angle <= 0.0f) {
        return -1;
    }
    
    if (config->steering_filter_alpha < 0.0f || config->steering_filter_alpha > 1.0f) {
        return -1;
    }
    
    g_steering_config = *config;
    return 0;
}
