/**
 * @file wheel_odometry_processor.c
 * @brief 轮速计脉冲处理器实现
 */

#include "wheel_odometry_processor.h"
#include <string.h>
#include <math.h>

/* ========== 内部辅助函数 ========== */

/**
 * @brief 获取三点数据的中位数
 */
static double median_3points(double a, double b, double c) {
    if (a > b) {
        if (b > c) return b;
        if (a > c) return c;
        return a;
    }
    if (a > c) return a;
    if (b > c) return c;
    return b;
}

/**
 * @brief 计算缓冲区数据的滑动平均（加权中位数）
 */
static double smooth_velocity(const double *buffer, uint32_t count) {
    if (count == 0) return 0.0;
    if (count == 1) return buffer[0];
    if (count == 2) return (buffer[0] + buffer[1]) / 2.0;
    
    /* 3个或以上：使用中位数 + 加权平均 */
    double sum = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        sum += buffer[i];
    }
    return sum / count;
}

/* ========== 公共接口实现 ========== */

int wheel_odometry_processor_init(WheelOdometryProcessor_t *processor,
                                  const WheelConfig_t *config) {
    if (!processor || !config) {
        return -1;
    }
    
    /* 参数验证 */
    if (config->pulses_per_wheel_rev == 0 || config->wheel_radius <= 0) {
        return -1;
    }
    
    /* 初始化 */
    memset(processor, 0, sizeof(WheelOdometryProcessor_t));
    processor->config = *config;
    processor->is_initialized = true;
    
    return 0;
}

int wheel_odometry_processor_update(WheelOdometryProcessor_t *processor,
                                    uint32_t pulse_count,
                                    uint32_t timestamp,
                                    WheelOdometryData_t *output) {
    if (!processor || !output) {
        return -1;
    }
    
    if (!processor->is_initialized) {
        return -1;
    }
    
    /* 初次调用时仅记录状态 */
    if (processor->buffer_count == 0) {
        processor->last_pulse_count = pulse_count;
        processor->last_timestamp = timestamp;
        processor->last_update_time = timestamp;
        
        output->velocity = 0.0;
        output->distance = 0.0;
        output->timestamp = timestamp;
        
        return 0;
    }
    
    /* 计算脉冲增量 */
    int32_t pulse_delta = (int32_t)(pulse_count - processor->last_pulse_count);
    uint32_t time_delta_ms = timestamp - processor->last_timestamp;
    
    /* 检查超时 */
    if (time_delta_ms > processor->config.pulse_timeout_ms && pulse_delta == 0) {
        processor->is_timeout = true;
        output->velocity = 0.0;
        output->distance = processor->accumulated_distance;
        output->timestamp = timestamp;
        processor->last_update_time = timestamp;
        return 0;
    }
    
    processor->is_timeout = false;
    processor->last_update_time = timestamp;
    
    /* 避免除零 */
    if (time_delta_ms == 0) {
        output->velocity = processor->last_velocity;
        output->distance = processor->accumulated_distance;
        output->timestamp = timestamp;
        return 0;
    }
    
    /* 脉冲 → 距离转换 */
    double wheel_circumference = 2.0 * M_PI * processor->config.wheel_radius;
    double distance_delta = (double)pulse_delta / processor->config.pulses_per_wheel_rev 
                           * wheel_circumference;
    
    processor->accumulated_distance += distance_delta;
    
    /* 计算速度 (m/s) */
    double dt_sec = time_delta_ms / 1000.0;
    double velocity = distance_delta / dt_sec;
    
    /* 极低速补零 */
    if (fabs(velocity) < 0.05) {
        velocity = 0.0;
    }
    
    /* 低速平滑处理 */
    if (fabs(velocity) < processor->config.low_speed_threshold) {
        /* 保存到缓冲区 */
        processor->velocity_buffer[processor->buffer_index] = velocity;
        processor->buffer_index = (processor->buffer_index + 1) % 5;
        if (processor->buffer_count < 5) {
            processor->buffer_count++;
        }
        
        /* 应用平滑滤波 */
        velocity = smooth_velocity(processor->velocity_buffer, processor->buffer_count);
    } else {
        /* 高速时清空缓冲，直接使用 */
        processor->buffer_count = 0;
        processor->buffer_index = 0;
    }
    
    /* 保存状态 */
    processor->last_pulse_count = pulse_count;
    processor->last_timestamp = timestamp;
    processor->last_velocity = velocity;
    
    /* 输出 */
    output->velocity = velocity;
    output->distance = processor->accumulated_distance;
    output->timestamp = timestamp;
    
    return 0;
}

int wheel_odometry_processor_set_velocity(WheelOdometryProcessor_t *processor,
                                          double velocity,
                                          uint32_t timestamp,
                                          WheelOdometryData_t *output) {
    if (!processor || !output) {
        return -1;
    }
    
    if (!processor->is_initialized) {
        return -1;
    }
    
    /* 检查超时 */
    if (processor->last_update_time > 0) {
        uint32_t time_delta = timestamp - processor->last_update_time;
        if (time_delta > processor->config.pulse_timeout_ms && fabs(velocity) < 1e-6) {
            processor->is_timeout = true;
            velocity = 0.0;
        } else {
            processor->is_timeout = false;
        }
    }
    
    processor->last_update_time = timestamp;
    
    /* 极低速补零 */
    if (fabs(velocity) < 0.05) {
        velocity = 0.0;
    }
    
    /* 低速平滑处理 */
    if (fabs(velocity) < processor->config.low_speed_threshold) {
        processor->velocity_buffer[processor->buffer_index] = velocity;
        processor->buffer_index = (processor->buffer_index + 1) % 5;
        if (processor->buffer_count < 5) {
            processor->buffer_count++;
        }
        velocity = smooth_velocity(processor->velocity_buffer, processor->buffer_count);
    } else {
        processor->buffer_count = 0;
        processor->buffer_index = 0;
    }
    
    /* 积分距离 */
    if (processor->last_timestamp > 0) {
        uint32_t dt_ms = timestamp - processor->last_timestamp;
        if (dt_ms > 0) {
            double dt_sec = dt_ms / 1000.0;
            processor->accumulated_distance += velocity * dt_sec;
        }
    }
    
    processor->last_timestamp = timestamp;
    processor->last_velocity = velocity;
    
    /* 输出 */
    output->velocity = velocity;
    output->distance = processor->accumulated_distance;
    output->timestamp = timestamp;
    
    return 0;
}

int wheel_odometry_processor_reset(WheelOdometryProcessor_t *processor) {
    if (!processor) {
        return -1;
    }
    
    processor->last_pulse_count = 0;
    processor->accumulated_distance = 0.0;
    processor->last_velocity = 0.0;
    processor->last_timestamp = 0;
    processor->buffer_index = 0;
    processor->buffer_count = 0;
    processor->is_timeout = false;
    memset(processor->velocity_buffer, 0, sizeof(processor->velocity_buffer));
    
    return 0;
}

bool wheel_odometry_processor_is_timeout(const WheelOdometryProcessor_t *processor) {
    if (!processor) {
        return false;
    }
    return processor->is_timeout;
}
