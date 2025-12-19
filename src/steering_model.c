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
    .prev_timestamp = 0,
    .left_wheel_angle = 0.0,
    .right_wheel_angle = 0.0,
    .mean_wheel_angle = 0.0
};

/* 全局映射表 */
static SteeringMappingTable_t g_steering_mapping = {
    .steering_wheel = NULL,
    .left_wheel = NULL,
    .right_wheel = NULL,
    .num_points = 0,
    .valid = false
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
     * 改进：如果映射表可用，使用映射表查询，否则使用线性模型
     */
    double left_wheel, right_wheel;
    
    if (g_steering_mapping.valid) {
        /* 使用映射表 */
        steering_query_mapping(steering_wheel_angle, &left_wheel, &right_wheel);
        /* 返回平均值 */
        double front_wheel_angle = (left_wheel + right_wheel) / 2.0;
        return steering_model_clamp_angle(front_wheel_angle);
    } else {
        /* 使用线性模型（后备方案） */
        double front_wheel_angle = steering_wheel_angle / g_steering_config.steering_ratio;
        
        /* 应用零偏和增益补偿 */
        front_wheel_angle = steering_model_apply_calibration(front_wheel_angle);
        
        /* 限制在有效范围内 */
        return steering_model_clamp_angle(front_wheel_angle);
    }
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
 * 
 * 改进：如果映射表可用，使用Ackermann模型
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
    /* 步骤1：根据映射表查询前左轮和前右轮转角 */
    if (g_steering_mapping.valid) {
        steering_query_mapping(steering_wheel_angle, 
                             &g_steering_state.left_wheel_angle,
                             &g_steering_state.right_wheel_angle);
    } else {
        /* 使用线性模型 */
        double basic = steering_wheel_angle / g_steering_config.steering_ratio;
        g_steering_state.left_wheel_angle = basic;
        g_steering_state.right_wheel_angle = basic;
    }
    
    /* 步骤2：计算平均转向角 */
    g_steering_state.mean_wheel_angle = 
        (g_steering_state.left_wheel_angle + g_steering_state.right_wheel_angle) / 2.0;
    
    /* 步骤3：应用校准 */
    double calibrated_angle = steering_model_apply_calibration(g_steering_state.mean_wheel_angle);
    
    /* 步骤4：计算转向角速率 */
    if (g_steering_state.prev_timestamp > 0) {
        double dt = (timestamp - g_steering_state.prev_timestamp) / 1000.0;
        if (dt > 0 && dt < 1.0) {
            g_steering_state.steering_rate = 
                (calibrated_angle - g_steering_state.prev_steering_angle) / dt;
        }
    }
    
    /* 步骤5：应用低通滤波 */
    if (g_steering_config.enable_steering_filter) {
        double alpha = g_steering_config.steering_filter_alpha;
        g_steering_state.filtered_steering_angle = 
            alpha * g_steering_state.filtered_steering_angle + 
            (1.0 - alpha) * calibrated_angle;
    } else {
        g_steering_state.filtered_steering_angle = calibrated_angle;
    }
    
    /* 步骤6：更新状态 */
    g_steering_state.prev_steering_angle = calibrated_angle;
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

/* ============================================================================
 * 映射表操作函数
 * ============================================================================ */

/**
 * @brief 线性插值辅助函数
 * @param x 输入值
 * @param x0, y0 第一个点
 * @param x1, y1 第二个点
 * @return 插值结果
 */
static double linear_interpolate(double x, double x0, double y0, double x1, double y1) {
    if (fabs(x1 - x0) < 1e-9) {
        return y0;
    }
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

/**
 * @brief 初始化转向映射表
 * @param sw_samples 方向盘转角数组 (弧度)
 * @param left_samples 前左轮转角数组 (弧度)
 * @param right_samples 前右轮转角数组 (弧度)
 * @param num_points 映射点数
 * @return 0 成功，-1 失败
 */
int steering_mapping_init(const double *sw_samples, 
                         const double *left_samples,
                         const double *right_samples, 
                         int num_points) {
    if (!sw_samples || !left_samples || !right_samples || num_points < 2) {
        return -1;
    }
    
    /* 清理旧的映射表 */
    steering_mapping_cleanup();
    
    /* 分配内存 */
    g_steering_mapping.steering_wheel = (double *)malloc(num_points * sizeof(double));
    g_steering_mapping.left_wheel = (double *)malloc(num_points * sizeof(double));
    g_steering_mapping.right_wheel = (double *)malloc(num_points * sizeof(double));
    
    if (!g_steering_mapping.steering_wheel || 
        !g_steering_mapping.left_wheel || 
        !g_steering_mapping.right_wheel) {
        steering_mapping_cleanup();
        return -1;
    }
    
    /* 复制数据 */
    memcpy(g_steering_mapping.steering_wheel, sw_samples, num_points * sizeof(double));
    memcpy(g_steering_mapping.left_wheel, left_samples, num_points * sizeof(double));
    memcpy(g_steering_mapping.right_wheel, right_samples, num_points * sizeof(double));
    
    g_steering_mapping.num_points = num_points;
    g_steering_mapping.valid = true;
    
    return 0;
}

/**
 * @brief 查询映射表获取前轮转角
 * @param steering_wheel_angle 方向盘转角 (弧度)
 * @param left_wheel_angle 输出：前左轮转角 (弧度)
 * @param right_wheel_angle 输出：前右轮转角 (弧度)
 * @return 0 成功，-1 失败
 */
int steering_query_mapping(double steering_wheel_angle,
                          double *left_wheel_angle,
                          double *right_wheel_angle) {
    if (!g_steering_mapping.valid || !left_wheel_angle || !right_wheel_angle) {
        return -1;
    }
    
    /* 查找插值区间 */
    int idx = 0;
    for (idx = 0; idx < g_steering_mapping.num_points - 1; idx++) {
        if (steering_wheel_angle >= g_steering_mapping.steering_wheel[idx] &&
            steering_wheel_angle <= g_steering_mapping.steering_wheel[idx + 1]) {
            break;
        }
    }
    
    /* 边界处理 */
    if (idx >= g_steering_mapping.num_points - 1) {
        idx = g_steering_mapping.num_points - 2;
    }
    if (idx < 0) {
        idx = 0;
    }
    
    /* 线性插值查询 */
    double x0 = g_steering_mapping.steering_wheel[idx];
    double x1 = g_steering_mapping.steering_wheel[idx + 1];
    
    *left_wheel_angle = linear_interpolate(
        steering_wheel_angle,
        x0, g_steering_mapping.left_wheel[idx],
        x1, g_steering_mapping.left_wheel[idx + 1]
    );
    
    *right_wheel_angle = linear_interpolate(
        steering_wheel_angle,
        x0, g_steering_mapping.right_wheel[idx],
        x1, g_steering_mapping.right_wheel[idx + 1]
    );
    
    return 0;
}

/**
 * @brief 计算平均前轮转角
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @return 平均值 (弧度)
 */
double steering_compute_mean_angle(double steering_wheel_angle) {
    /* 查询映射表获得前轮转角 */
    double left = 0.0, right = 0.0;
    
    if (steering_query_mapping(steering_wheel_angle, &left, &right) == 0) {
        return (left + right) / 2.0;
    }
    
    /* 映射表无效，返回简单平均 */
    return (g_steering_state.left_wheel_angle + g_steering_state.right_wheel_angle) / 2.0;
}

/**
 * @brief 使用Ackermann模型计算航向角速率
 * @param velocity 车辆纵向速度 (m/s)
 * @param steering_wheel_angle 方向盘转角 (rad)
 * @param wheelbase 轴距 (m)
 * @return 航向角速率 (rad/s)
 */
double steering_calculate_yaw_rate_ackermann(double velocity,
                                            double steering_wheel_angle,
                                            double wheelbase) {
    if (fabs(velocity) < 0.01 || fabs(wheelbase) < 0.01) {
        return 0.0;
    }
    
    /* 查询映射表获得前轮转角 */
    double left_rad = 0.0, right_rad = 0.0;
    
    if (steering_query_mapping(steering_wheel_angle, &left_rad, &right_rad) != 0) {
        /* 映射表无效，使用简单齿比模型 */
        left_rad = steering_wheel_angle / g_steering_config.steering_ratio;
        right_rad = steering_wheel_angle / g_steering_config.steering_ratio;
    }
    
    /* Ackermann模型：考虑左右轮转角差异 */
    double yaw_rate = velocity * (tan(left_rad) + tan(right_rad)) / (2.0 * wheelbase);
    
    return yaw_rate;
}

/**
 * @brief 获取当前的前轮转角（左右分别）
 * @param left_wheel 输出：前左轮转角
 * @param right_wheel 输出：前右轮转角
 * @param mean_angle 输出：平均转向角
 * @return 0 成功，-1 失败
 */
int steering_get_wheel_angles(double *left_wheel,
                             double *right_wheel,
                             double *mean_angle) {
    if (!left_wheel || !right_wheel || !mean_angle) {
        return -1;
    }
    
    *left_wheel = g_steering_state.left_wheel_angle;
    *right_wheel = g_steering_state.right_wheel_angle;
    *mean_angle = (g_steering_state.left_wheel_angle + g_steering_state.right_wheel_angle) / 2.0;
    
    return 0;
}

/**
 * @brief 清理映射表资源
 */
void steering_mapping_cleanup(void) {
    if (g_steering_mapping.steering_wheel) {
        free(g_steering_mapping.steering_wheel);
        g_steering_mapping.steering_wheel = NULL;
    }
    
    if (g_steering_mapping.left_wheel) {
        free(g_steering_mapping.left_wheel);
        g_steering_mapping.left_wheel = NULL;
    }
    
    if (g_steering_mapping.right_wheel) {
        free(g_steering_mapping.right_wheel);
        g_steering_mapping.right_wheel = NULL;
    }
    
    g_steering_mapping.num_points = 0;
    g_steering_mapping.valid = false;
}
