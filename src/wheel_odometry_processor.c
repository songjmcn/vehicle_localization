/**
 * @file wheel_odometry_processor.c
 * @brief 轮速计脉冲处理器实现（ISO 8855后轴中心模型）
 *
 * @details
 * 本模块处理后轮编码器数据：
 * - 输入：后轮左右脉冲计数（代表后轮轴中心纵向位移）
 * - 输出：后轮轴中心纵向速度(m/s)
 *
 * @note 输出的速度为后轮轴中心纵向速度（来源：后轮编码器）
 * @note 左右轮速度应该非常接近（<5%偏差正常，>10%表示打滑）
 * @note 约束：后轮不侧滑（非完整约束）
 */

#include "wheel_odometry_processor.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ========== 内部辅助函数 ========== */

/**
 * @brief 获取三点数据的中位数
 */
// static double median_3points(double a, double b, double c) {
//     if (a > b) {
//         if (b > c) return b;
//         if (a > c) return c;
//         return a;
//     }
//     if (a > c) return a;
//     if (b > c) return c;
//     return b;
// }

/**
 * @brief 计算缓冲区数据的滑动平均（加权中位数）
 */
static double smooth_velocity(const double *buffer, uint32_t count) {
    if (count == 0) return 0.0;
    if (count == 1) return buffer[0];
    if (count == 2) return (buffer[0] + buffer[1]) / 2.0;

    /* 3个或以上：计算带符号的平均值
     * 关键：保留速度的符号（倒车时为负）
     * 而不是简单地平均绝对值
     */
    double sum = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        sum += buffer[i];
    }
    double avg = sum / count;
    
    /* === 🔴 关键修复：确保输出保留倒车的负号 ===
     * 如果输入都是负数（倒车），输出应该也是负数
     * 而不是被平滑成0或正数
     */
    return avg;
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
    processor->first_call = true;   /* 标记首次调用 */

    return 0;
}

int wheel_odometry_processor_update(WheelOdometryProcessor_t *processor,
                                    uint32_t left_pulse_count,
                                    uint32_t right_pulse_count,
                                    uint64_t timestamp,
                                    VehicleGear_t gear,
                                    WheelDirection_t rear_left_state,
                                    WheelDirection_t rear_right_state,
                                    WheelOdometryData_t *output) {
    if (!processor || !output) {
        return -1;
    }

    if (!processor->is_initialized) {
        return -1;
    }

    /* 计算合成脉冲计数（取左右轮平均值） */
    uint32_t pulse_count = (left_pulse_count + right_pulse_count) / 2;

    /* 初次调用时仅记录状态，然后标记已初始化 */
    if (processor->first_call) {
        processor->first_call = false;  /* 标记已进行首次初始化 */
        processor->last_pulse_count = pulse_count;
        processor->last_timestamp = timestamp;
        processor->last_update_time = timestamp;
        processor->last_left_pulse_count = left_pulse_count;
        processor->last_right_pulse_count = right_pulse_count;

        output->velocity = 0.0f;
        output->distance = 0.0f;
        output->timestamp = timestamp;
        output->gear = gear;
        output->rear_left_direction = WHEEL_DIRECRTION_INVALID;
        output->rear_right_direction = WHEEL_DIRECRTION_INVALID;

        return 0;
    }


    /* === 关键修复：分别处理左右轮的溢出 === */
    int32_t left_delta, right_delta;

    if (processor->config.max_count > 0) {
        /* 使用动态阈值而不是硬编码的5 */
        uint32_t overflow_threshold = processor->config.max_count / 2;

        /* 检测和修正左轮溢出 */
        left_delta = (int32_t)(left_pulse_count - processor->last_left_pulse_count);
        if (left_pulse_count < processor->last_left_pulse_count) {
            uint32_t left_diff = processor->last_left_pulse_count - left_pulse_count;
            if (left_diff > overflow_threshold) {
                left_delta = (int32_t)((processor->config.max_count - processor->last_left_pulse_count) + left_pulse_count);
#ifdef DEBUG
                printf("Left overflow: last=%u, curr=%u, delta=%d\n",
                       processor->last_left_pulse_count, left_pulse_count, left_delta);
#endif
            }
        }

        /* 检测和修正右轮溢出 */
        right_delta = (int32_t)(right_pulse_count - processor->last_right_pulse_count);
        if (right_pulse_count < processor->last_right_pulse_count) {
            uint32_t right_diff = processor->last_right_pulse_count - right_pulse_count;
            if (right_diff > overflow_threshold) {
                right_delta = (int32_t)((processor->config.max_count - processor->last_right_pulse_count) + right_pulse_count);
#ifdef DEBUG
                printf("Right overflow: last=%u, curr=%u, delta=%d\n",
                       processor->last_right_pulse_count, right_pulse_count, right_delta);
#endif
            }
        }
    } else {
        left_delta = (int32_t)(left_pulse_count - processor->last_left_pulse_count);
        right_delta = (int32_t)(right_pulse_count - processor->last_right_pulse_count);
    }

    /* === 计算合成脉冲增量（脉冲累积缓冲） === */
    /* 问题：直接除法 (left_delta + right_delta) / 2 会导致脉冲增量为1时被截断为0 */
    /* 解决方案A：脉冲小数部分累积，保留到下次计算 */
    float pulse_delta_float = (float)(left_delta + right_delta) / 2.0f + processor->pulse_residual;
    int32_t pulse_delta = (int32_t)floorf(pulse_delta_float);
    processor->pulse_residual = pulse_delta_float - (float)pulse_delta;

#ifdef DEBUG
    printf("Pulse calculation: left_delta=%d, right_delta=%d, pulse_delta_float=%f, pulse_delta=%d, residual=%f\n",
           left_delta, right_delta, pulse_delta_float, pulse_delta, processor->pulse_residual);
#endif

    /* 保存当前的左右轮脉冲计数 */
    processor->last_left_pulse_count = left_pulse_count;
    processor->last_right_pulse_count = right_pulse_count;

    uint32_t time_delta_ms = timestamp - processor->last_timestamp;

    /* 检查超时 */
    if (time_delta_ms > processor->config.pulse_timeout_ms && pulse_delta < 0.5) {
        processor->is_timeout = true;
        output->velocity = 0.0f;
        output->distance = (float)processor->accumulated_distance;
        output->timestamp = timestamp;
        output->gear = gear;
        output->rear_left_direction = WHEEL_DIRECTION_STOP;
        output->rear_right_direction = WHEEL_DIRECTION_STOP;
        processor->last_update_time = timestamp;
        return 0;
    }

    processor->is_timeout = false;
    processor->last_update_time = timestamp;

    /* 避免除零 */
    if (time_delta_ms == 0) {
        output->velocity = (float)processor->last_velocity;
        output->distance = (float)processor->accumulated_distance;
        output->timestamp = timestamp;
        output->gear = gear;
        output->rear_left_direction = WHEEL_DIRECRTION_INVALID;
        output->rear_right_direction = WHEEL_DIRECRTION_INVALID;
        return 0;
    }

    /* 脉冲 → 距离转换 */
    double wheel_circumference = 2.0 * M_PI * processor->config.wheel_radius;
    double distance_delta = (double)pulse_delta / processor->config.pulses_per_wheel_rev
                           * wheel_circumference;
    if(pulse_delta>0){
        printf(
            "pulse_delta:%d, time_delta:%d, distance_delta:%lf\n",
            pulse_delta, time_delta_ms, distance_delta
        );
    }
    /* 计算原始速度（用于方向判断和平滑处理） */
    double dt_sec = time_delta_ms / 1000.0;
    double raw_velocity = distance_delta / dt_sec;

    /* 极低速补零 - 仅在关键时刻使用 */
    /* === 🔴 关键修复：不再简单地补零，而是保留符号 ===
     * 之前：低速时把velocity设为0做方向判断，导致后续计算也变成0
     * 现在：保留velocity的符号，用于方向判断和距离计算
     * 只有在完全静止（speed < V_STATIC_THRESHOLD且R/D档）时才返回STATIONARY
     */
    double adjusted_velocity_for_direction = raw_velocity;
    /* 不再设为0，这样倒车时的方向判断才能正确 */

    /* 低速平滑处理 - 基于原始速度 */
    double smoothed_velocity = raw_velocity;

    /* === 🔴 关键修复：禁用低速平滑滤波（会破坏倒车） ===
     * 低速平滑滤波的缺陷：
     *   1. 会把低速倒车的速度平滑成接近0
     *   2. 导致yaw_rate=0，heading不更新，Y坐标不累积
     *   3. 对于轮向数据错误的情况（R档但轮Forward）特别有害
     * 
     * 修复：完全禁用低速平滑，接受低速数据的天然噪声
     * （噪声会被Kalman滤波处理）
     */
    /* 禁用：原来的平滑滤波代码被注释掉
    if (fabs(raw_velocity) < processor->config.low_speed_threshold) {
        // ... 平滑滤波代码 ...
    } else {
        processor->buffer_count = 0;
        processor->buffer_index = 0;
    }
    */
    /* 现在直接使用raw_velocity，不进行任何平滑 */
    smoothed_velocity = raw_velocity;
    processor->buffer_count = 0;
    processor->buffer_index = 0;

    /* 计算平滑后的距离增量 */
    double smoothed_distance_delta = smoothed_velocity * dt_sec;

    /* === 推导运动方向（基于调整后的速度） === */
    VehicleDirection_t direction = wheel_odometry_derive_direction(
        adjusted_velocity_for_direction, gear, rear_left_state, rear_right_state);

    /* === 关键修复：根据方向调整距离增量 === */
    /* 倒车时，distance_delta应该变成负数
     * 但由于low_speed_threshold可能把distance平滑成0，需要恢复幅度
     */
    double final_distance_delta = fabs(smoothed_distance_delta);  /* 获取幅度 */

    if (direction == DIRECTION_REVERSE) {
        final_distance_delta = -final_distance_delta;  /* 后退：距离增量为负 */
    } else if (direction == DIRECTION_STATIONARY) {
        final_distance_delta = 0.0;                       /* 停止：距离不增加 */
    }
    /* 前进时保持原值（final_distance_delta为正） */

    /* 计算最终的有符号速度（基于调整后的距离增量）*/
    double signed_velocity = final_distance_delta / dt_sec;

    /* === 累加距离 === */
    processor->accumulated_distance += final_distance_delta;

#ifdef DEBUG
    printf("wheel direction:%d, pulse_delta:%d, raw_velocity:%f, smoothed_distance:%lf->%lf, velocity:%f, accumulated:%lf\n",
           direction, pulse_delta, raw_velocity, smoothed_distance_delta, final_distance_delta, signed_velocity, processor->accumulated_distance);
#endif

    /* 保存状态 */
    processor->last_pulse_count = pulse_count;
    processor->last_timestamp = timestamp;
    processor->last_velocity = signed_velocity;  /* 保存有符号的速度 */

    /* === 输出 === */
    output->velocity = (float)signed_velocity;   /* 现在包含方向信息：后退为负 */
    output->distance = (float)processor->accumulated_distance;
    output->timestamp = timestamp;
    output->gear = gear;  /* 保存档位信息 */

    /* === 输出左右轮状态 === */
    output->rear_left_direction = rear_left_state;
    output->rear_right_direction = rear_right_state;

    /* 🔴 调试：输出wheel processor结果 */
    /*
    if (direction == DIRECTION_REVERSE) {
        static FILE *wheel_debug_log = NULL;
        static int wheel_call_count = 0;
        if (!wheel_debug_log) {
            wheel_debug_log = fopen("/tmp/wheel_debug.log", "w");
        }
        if (wheel_debug_log && wheel_call_count++ < 5000) {
            fprintf(wheel_debug_log, "direction=REVERSE, final_dist=%.6f, signed_v=%.6f | raw_v=%.6f, smooth_v=%.6f, pulse_delta=%d\n",
                    final_distance_delta, signed_velocity, raw_velocity, smoothed_velocity, pulse_delta);
            fflush(wheel_debug_log);
        }
    }
    */

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
    if (fabs(velocity) < V_STATIC_THRESHOLD) {
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

    /* === 关键修改：距离积分时考虑速度符号（运动方向） === */
    /* 注意：此函数缺少档位和轮状态参数，无法进行完整的方向推导 */
    /* 所以保留速度的符号进行积分：后退时速度会减少，前进时速度会增加 */
    double signed_velocity = velocity;  /* 保留原速度符号 */

    /* 积分距离时使用有符号的速度 */
    if (processor->last_timestamp > 0) {
        uint32_t dt_ms = timestamp - processor->last_timestamp;
        if (dt_ms > 0) {
            double dt_sec = dt_ms / 1000.0;
            /* === 关键修改：使用有符号的速度进行积分 === */
            /* 这样后退时距离会减少，前进时距离会增加 */
            processor->accumulated_distance += signed_velocity * dt_sec;
        }
    }

    processor->last_timestamp = timestamp;
    processor->last_velocity = signed_velocity;  /* 保存有符号的速度 */

    /* 输出 */
    output->velocity = (float)signed_velocity;   /* 保留方向符号 */
    output->distance = (float)processor->accumulated_distance;
    output->timestamp = timestamp;

    return 0;
}

int wheel_odometry_processor_reset(WheelOdometryProcessor_t *processor) {
    if (!processor) {
        return -1;
    }

    processor->last_pulse_count = 0;
    processor->last_left_pulse_count = 0;
    processor->last_right_pulse_count = 0;
    processor->accumulated_distance = 0.0;
    processor->pulse_residual = 0.0f;   /* 重置脉冲小数部分 */
    processor->last_velocity = 0.0;
    processor->last_timestamp = 0;
    processor->buffer_index = 0;
    processor->buffer_count = 0;
    processor->is_timeout = false;
    processor->first_call = true;   /* 重置首次调用标志 */
    memset(processor->velocity_buffer, 0, sizeof(processor->velocity_buffer));

    return 0;
}

bool wheel_odometry_processor_is_timeout(const WheelOdometryProcessor_t *processor) {
    if (!processor) {
        return false;
    }
    return processor->is_timeout;
}

/* ========== 车辆方向推导模块 ========== */

/**
 * @brief 内部函数：判断档位是否有效
 */
static bool is_valid_gear(VehicleGear_t gear) {
    return (gear == GEAR_DRIVE || gear == GEAR_REVERSE ||
            gear == GEAR_NEUTRAL || gear == GEAR_LOW ||
            gear == GEAR_PARK);
}

/**
 * @brief 内部函数：档位推导的理论方向
 */
static VehicleDirection_t gear_theoretical_direction(VehicleGear_t gear) {
    switch (gear) {
        case GEAR_REVERSE:
            return DIRECTION_REVERSE;
        case GEAR_DRIVE:
        case GEAR_LOW:
            return DIRECTION_FORWARD;
        case GEAR_NEUTRAL:
        case GEAR_PARK:
            return DIRECTION_STATIONARY;
        default:
            return DIRECTION_STATIONARY;
    }
}

/**
 * @brief 内部函数：检测左右轮状态是否一致且有效
 * @return true 一致且有效，false 不一致或无效
 */
static bool wheel_states_consistent(WheelDirection_t left_state, WheelDirection_t right_state) {
    /* 都无效 */
    if (left_state == WHEEL_DIRECRTION_INVALID && right_state == WHEEL_DIRECRTION_INVALID) {
        return false;
    }

    /* 都有效且相同 */
    if (left_state != WHEEL_DIRECRTION_INVALID && right_state != WHEEL_DIRECRTION_INVALID) {
        return left_state == right_state;
    }

    /* 一个有效一个无效 */
    return false;
}

/**
 * @brief 内部函数：获取一致的轮状态
 * @pre 需要先调用 wheel_states_consistent() 确认状态一致
 */
static WheelDirection_t get_wheel_state(WheelDirection_t left_state, WheelDirection_t right_state) {
    if (left_state != WHEEL_DIRECRTION_INVALID) {
        return left_state;
    }
    return right_state;
}

/**
 * @brief 推导车辆运动方向
 *
 * 采用4层优先级融合策略：
 * - 优先使用左右后轮运动状态
 * - 其次使用档位信息
 * - 最后使用速度符号
 *
 * @return VehicleDirection_t 车辆运动方向（DIRECTION_FORWARD/REVERSE/STATIONARY/INVALID）
 */
VehicleDirection_t wheel_odometry_derive_direction(
    double velocity,
    VehicleGear_t gear,
    WheelDirection_t rear_left_state,
    WheelDirection_t rear_right_state) {

    /* === 🔴 关键修复：档位优先于速度阈值 ===
     * 之前的顺序导致低速倒车被误判为STATIONARY
     * 原因：低速平滑滤波会使velocity接近0，直接触发V_STATIC_THRESHOLD判断
     * 修复方案：先检查档位，再根据档位决定是否应用速度阈值
     */

    /* 档位有效性检查 */
    if (!is_valid_gear(gear)) {
        return DIRECTION_INVALID;
    }

    /* P1: 静止判断 - 仅在P档或N档时应用速度阈值 */
    if (fabs(velocity) < V_STATIC_THRESHOLD) {
        /* 只有在档位为P或N时才能判定为停止，否则继续检查档位意图 */
        if (gear == GEAR_PARK || gear == GEAR_NEUTRAL) {
            return DIRECTION_STATIONARY;
        }
        /* 对于D/R档，即使速度很低，也应该根据档位返回方向，而不是STATIONARY
         * 这在启动和低速倒车时特别重要 */
    }

    /* 异常检测：左右轮状态都有效但相反 */
    if (rear_left_state != WHEEL_DIRECRTION_INVALID && rear_right_state != WHEEL_DIRECRTION_INVALID) {
        if ((rear_left_state == WHEEL_DIRECTION_FORWARD && rear_right_state == WHEEL_DIRECTION_REVERSE) ||
            (rear_left_state == WHEEL_DIRECTION_REVERSE && rear_right_state == WHEEL_DIRECTION_FORWARD)) {
            return DIRECTION_INVALID;  /* 一前一后，异常（轮滑或传感器故障） */
        }
    }

    /* P2: 轮状态判断 - 优先使用硬件轮状态 */
    if (wheel_states_consistent(rear_left_state, rear_right_state)) {
        WheelDirection_t wheel_state = get_wheel_state(rear_left_state, rear_right_state);

        /* 验证轮状态与档位是否一致 */
        VehicleDirection_t gear_direction = gear_theoretical_direction(gear);

        /* 档位指示停止（P档） */
        if (gear == GEAR_PARK) {
            /* 轮状态应该也是停止 */
            if (wheel_state != WHEEL_DIRECTION_STOP) {
                return DIRECTION_INVALID;  /* P档但轮在动 - 异常 */
            }
            return DIRECTION_STATIONARY;
        }

        /* 档位指示前进（D档或L档） */
        if (gear_direction == DIRECTION_FORWARD) {
            if (wheel_state == WHEEL_DIRECTION_FORWARD) {
                return DIRECTION_FORWARD;  /* 正常前进 */
            } else if (wheel_state == WHEEL_DIRECTION_STOP) {
                return DIRECTION_STATIONARY;  /* 前进档但轮停止 - 可能是初始状态 */
            } else if (wheel_state == WHEEL_DIRECTION_REVERSE) {
                return DIRECTION_INVALID;  /* 前进档但轮反向 - 异常 */
            }
        }

        /* 档位指示后退（R档） */
        if (gear_direction == DIRECTION_REVERSE) {
            if (wheel_state == WHEEL_DIRECTION_REVERSE) {
                return DIRECTION_REVERSE;  /* 正常后退 */
            } else if (wheel_state == WHEEL_DIRECTION_STOP) {
                return DIRECTION_STATIONARY;  /* 后退档但轮停止 - 可能是初始状态 */
            } else if (wheel_state == WHEEL_DIRECTION_FORWARD) {
                /* === 🔴 关键修复：档位优先判断 ===
                 * 当R档但轮显示Forward时，通常表示：
                 * 1. 脉冲计数器溢出（计数从0-4095重置）
                 * 2. 轮向传感器故障
                 * 3. 数据包乱序导致的临时不一致
                 * 解决方案：优先相信档位，返回DIRECTION_REVERSE而不是INVALID
                 */
                return DIRECTION_REVERSE;  /* 档位优先：R档 → 倒车 */
            }
        }

        /* 空档（N档）- 由轮状态决定 */
        if (gear == GEAR_NEUTRAL) {
            return wheel_direction_to_vehicle_direction(wheel_state);
        }
    }

    /* P3: 轮状态不一致或无效 - 使用档位判断 */
    VehicleDirection_t gear_direction = gear_theoretical_direction(gear);

    if (gear_direction == DIRECTION_FORWARD) {
        return DIRECTION_FORWARD;
    } else if (gear_direction == DIRECTION_REVERSE) {
        return DIRECTION_REVERSE;
    } else if (gear_direction == DIRECTION_STATIONARY) {
        return DIRECTION_STATIONARY;
    }

    return DIRECTION_INVALID;
}

VehicleDirection_t wheel_direction_to_vehicle_direction(WheelDirection_t direction) {
    switch (direction) {
        case WHEEL_DIRECRTION_INVALID:
            return DIRECTION_INVALID;
        case WHEEL_DIRECTION_FORWARD:
            return DIRECTION_FORWARD;
        case WHEEL_DIRECTION_REVERSE:
            return DIRECTION_REVERSE;
        case WHEEL_DIRECTION_STOP:
            return DIRECTION_STATIONARY;
        default:
            return DIRECTION_INVALID;
    }
}
