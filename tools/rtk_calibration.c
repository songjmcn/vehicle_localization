/**
 * @file rtk_calibration.c
 * @brief RTK标定实现
 */

#include "rtk_calibration.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

int rtk_calibration_init(RTKCalibrationManager_t *manager, uint32_t max_points) {
    if (!manager || max_points == 0) {
        return -1;
    }
    
    manager->calibration_points = (RTKCalibrationPoint_t *)
        malloc(sizeof(RTKCalibrationPoint_t) * max_points);
    
    if (!manager->calibration_points) {
        return -1;
    }
    
    manager->max_points = max_points;
    manager->calibration_count = 0;
    manager->cumulative_x_error = 0.0;
    manager->cumulative_y_error = 0.0;
    manager->cumulative_heading_error = 0.0;
    manager->max_position_error = 0.0;
    manager->max_heading_error = 0.0;
    
    /* 初始化加权最小二乘法统计 */
    manager->outlier_count = 0;
    manager->r_squared = 0.0;
    manager->weighted_std_dev = 0.0;
    
    /* 初始化数据质量统计 */
    manager->rejected_count = 0;
    manager->low_quality_skip = 0;
    
    /* 初始化校准参数为默认值 */
    manager->estimated_calibration.wheel_calibration_factor = 1.0;
    manager->estimated_calibration.steering_angle_offset = 0.0;
    manager->estimated_calibration.steering_angle_gain = 1.0;
    manager->estimated_calibration.imu_accel_bias_x = 0.0;
    manager->estimated_calibration.imu_accel_bias_y = 0.0;
    manager->estimated_calibration.imu_gyro_bias_z = 0.0;
    manager->estimated_calibration.imu_accel_scale_x = 1.0;
    manager->estimated_calibration.imu_accel_scale_y = 1.0;
    
    manager->calibration_valid = false;
    
    /* Phase 2 (P1) 初始化转向增益标定管理器 */
    if (steering_gain_manager_init(&manager->steering_gain_manager) != 0) {
        free(manager->calibration_points);
        manager->calibration_points = NULL;
        return -1;
    }
    manager->steering_gain_valid = false;
    manager->steering_gain_wheelbase = 2.7;  /* 默认车轴距 */
    
    return 0;
}

int rtk_calibration_add_point(RTKCalibrationManager_t *manager,
                              const VehicleState_t *odometry_state,
                              const RTKData_t *rtk_state) {
    if (!manager || !odometry_state || !rtk_state) {
        return -1;
    }
    
    if (manager->calibration_count >= manager->max_points) {
        return -1;  /* 缓冲区满 */
    }
    
    /* RTK质量检查 */
    if (rtk_state->quality < 2) {
        manager->low_quality_skip++;
        return -1;  /* 质量不足 */
    }
    
    RTKCalibrationPoint_t *point = &manager->calibration_points[manager->calibration_count];
    
    point->odometry_state = *odometry_state;
    point->rtk_state = *rtk_state;
    
    /* 计算误差 */
    point->position_error_x = rtk_state->x - odometry_state->x;
    point->position_error_y = rtk_state->y - odometry_state->y;
    point->heading_error = normalize_angle(rtk_state->heading - odometry_state->heading);
    
    /* 更新累积统计 */
    manager->cumulative_x_error += point->position_error_x;
    manager->cumulative_y_error += point->position_error_y;
    manager->cumulative_heading_error += point->heading_error;
    
    double pos_error = sqrt(point->position_error_x * point->position_error_x +
                           point->position_error_y * point->position_error_y);
    if (pos_error > manager->max_position_error) {
        manager->max_position_error = pos_error;
    }
    
    double heading_error_abs = fabs(point->heading_error);
    if (heading_error_abs > manager->max_heading_error) {
        manager->max_heading_error = heading_error_abs;
    }
    
    manager->calibration_count++;
    
    return 0;
}

/**
 * @brief 计算一组双精度数据的均值
 */
static double calculate_mean(const double *data, uint32_t count) {
    if (count == 0) return 0.0;
    double sum = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        sum += data[i];
    }
    return sum / count;
}

/**
 * @brief 计算一组双精度数据的标准差
 */
static double calculate_std_dev(const double *data, uint32_t count, double mean) {
    if (count < 2) return 0.0;
    double sum_sq_diff = 0.0;
    for (uint32_t i = 0; i < count; i++) {
        double diff = data[i] - mean;
        sum_sq_diff += diff * diff;
    }
    return sqrt(sum_sq_diff / (count - 1));
}

/**
 * @brief 快速排序比较函数
 */
static int compare_doubles(const void *a, const void *b) {
    double fa = *(const double *)a;
    double fb = *(const double *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

/**
 * @brief 计算中位数绝对偏差（MAD）
 * 
 * MAD是一种鲁棒的离散度度量，对离群点不敏感
 * 定义：MAD = median(|x_i - median(x)|)
 * 
 * @param data 输入数据数组
 * @param count 数据个数
 * @param median 输出的中位数
 * @return 中位数绝对偏差值
 */
static double calculate_mad(const double *data, uint32_t count, double *median) {
    if (count == 0) {
        *median = 0.0;
        return 0.0;
    }
    
    /* 排序计算中位数 */
    double *sorted = (double *)malloc(count * sizeof(double));
    if (!sorted) {
        *median = 0.0;
        return 0.0;
    }
    
    memcpy(sorted, data, count * sizeof(double));
    qsort(sorted, count, sizeof(double), compare_doubles);
    
    if (count % 2 == 1) {
        *median = sorted[count / 2];
    } else {
        *median = (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0;
    }
    
    /* 计算偏差 */
    double *deviations = (double *)malloc(count * sizeof(double));
    if (!deviations) {
        free(sorted);
        return 0.0;
    }
    
    for (uint32_t i = 0; i < count; i++) {
        deviations[i] = fabs(data[i] - *median);
    }
    
    /* 排序偏差以找到中位数 */
    qsort(deviations, count, sizeof(double), compare_doubles);
    
    double mad;
    if (count % 2 == 1) {
        mad = deviations[count / 2];
    } else {
        mad = (deviations[count / 2 - 1] + deviations[count / 2]) / 2.0;
    }
    
    free(sorted);
    free(deviations);
    
    return mad;
}

/**
 * @brief 标记离群点（使用MAD-based方法）
 * 
 * 使用中位数绝对偏差（MAD）识别离群点：
 * 阈值 = median(residuals) + 2.5 × MAD
 * 
 * 这种方法对极端离群点更鲁棒，因为不依赖标准差。
 * 返回有效的（非离群）点的个数
 */
static uint32_t mark_outliers(const double *residuals, uint32_t count,
                              bool *is_outlier, uint32_t *outlier_count) {
    *outlier_count = 0;
    
    if (count < 2) {
        memset(is_outlier, false, count * sizeof(bool));
        return count;
    }
    
    /* 使用MAD方法计算阈值 */
    double median = 0.0;
    double mad = calculate_mad(residuals, count, &median);
    
    /* 离群点检测阈值：|residual - median| > 2.5 * MAD */
    /* MAD的常数倍数通常在2-3之间，这里使用2.5 */
    double threshold = (mad > 1e-6) ? 2.5 * mad : 1.0;
    
    /* 如果MAD太小（完美拟合），使用至少2米作为绝对阈值 */
    if (threshold < 2.0) {
        threshold = 2.0;
    }
    
    uint32_t valid_count = 0;
    for (uint32_t i = 0; i < count; i++) {
        double residual_abs = fabs(residuals[i] - median);
        if (residual_abs > threshold) {
            is_outlier[i] = true;
            (*outlier_count)++;
        } else {
            is_outlier[i] = false;
            valid_count++;
        }
    }
    
    return valid_count;
}

/**
 * @brief 最小二乘法拟合轮速计校准因子（加权）
 * 
 * 基于以下观察：
 * 轮速计测得距离 = 实际距离 / calibration_factor
 * 
 * 权重 = RTK质量等级 × 轨迹长度
 * 
 * 计算指标：
 * - calibration_factor: 加权最小二乘拟合
 * - r_squared: 拟合优度 (0-1)
 * - weighted_std_dev: 加权残差标准差
 * 
 * @param manager RTK标定管理器
 * @param r_squared 输出：拟合优度
 * @param weighted_std_dev 输出：加权残差标准差
 * @return 轮速计校准因子
 */
static double estimate_wheel_calibration_weighted(RTKCalibrationManager_t *manager,
                                                   double *r_squared,
                                                   double *weighted_std_dev) {
    *r_squared = 0.0;
    *weighted_std_dev = 0.0;
    
    if (manager->calibration_count < 2) {
        return 1.0;
    }
    
    uint32_t num_segments = manager->calibration_count - 1;
    
    /* 分配临时数组 */
    double *odo_dists = (double *)malloc(num_segments * sizeof(double));
    double *rtk_dists = (double *)malloc(num_segments * sizeof(double));
    double *weights = (double *)malloc(num_segments * sizeof(double));
    bool *is_outlier = (bool *)malloc(num_segments * sizeof(bool));
    double *residuals = (double *)malloc(num_segments * sizeof(double));
    
    if (!odo_dists || !rtk_dists || !weights || !is_outlier || !residuals) {
        free(odo_dists);
        free(rtk_dists);
        free(weights);
        free(is_outlier);
        free(residuals);
        return 1.0;
    }
    
    /* 第一步：计算距离和权重 */
    double sum_odo = 0.0;
    double sum_rtk = 0.0;
    double max_quality = 0.0;
    
    for (uint32_t i = 0; i < num_segments; i++) {
        RTKCalibrationPoint_t *prev = &manager->calibration_points[i];
        RTKCalibrationPoint_t *curr = &manager->calibration_points[i + 1];
        
        /* 里程计距离 */
        double odo_dx = curr->odometry_state.x - prev->odometry_state.x;
        double odo_dy = curr->odometry_state.y - prev->odometry_state.y;
        odo_dists[i] = sqrt(odo_dx * odo_dx + odo_dy * odo_dy);
        
        /* RTK距离 */
        double rtk_dx = curr->rtk_state.x - prev->rtk_state.x;
        double rtk_dy = curr->rtk_state.y - prev->rtk_state.y;
        rtk_dists[i] = sqrt(rtk_dx * rtk_dx + rtk_dy * rtk_dy);
        
        /* 记录最大质量值 */
        if ((double)curr->rtk_state.quality > max_quality) {
            max_quality = (double)curr->rtk_state.quality;
        }
        
        sum_odo += odo_dists[i];
        sum_rtk += rtk_dists[i];
    }
    
    /* 优化的权重计算：
     * 1. 质量权重：相对于最大质量的二次幂（加重高质量数据）
     * 2. 轨迹权重：使用log(1 + distance)避免过度加权长轨迹段
     * 3. 测量不确定性权重：基于质量等级的不确定性估计
     */
    max_quality = (max_quality > 0) ? max_quality : 5.0;
    
    for (uint32_t i = 0; i < num_segments; i++) {
        RTKCalibrationPoint_t *curr = &manager->calibration_points[i + 1];
        
        /* 质量权重：(quality/max_quality)^2，范围[0, 1]，高质量数据权重更高 */
        double q_norm = (double)curr->rtk_state.quality / max_quality;
        q_norm = (q_norm > 1.0) ? 1.0 : q_norm;
        double quality_weight = q_norm * q_norm;  /* 二次函数强调高质量 */
        
        /* 轨迹权重：使用对数避免极长轨迹段主导拟合 */
        double trajectory_weight = 1.0 + log(1.0 + rtk_dists[i]);
        
        /* 测量不确定性权重：低质量数据的测量噪声更大 */
        double uncertainty_factor = 1.0 / (quality_weight + 0.1);  /* 避免分母为0 */
        
        /* 组合权重 */
        weights[i] = quality_weight * trajectory_weight / uncertainty_factor;
        
        /* 确保权重有意义（非负且非零） */
        if (weights[i] < 1e-6) {
            weights[i] = 1e-6;
        }
    }
    
    if (sum_rtk < 1e-6) {
        free(odo_dists);
        free(rtk_dists);
        free(weights);
        free(is_outlier);
        free(residuals);
        return 1.0;
    }
    
    /* 第二步：鲁棒初始估计（简单均值） */
    double initial_factor = sum_odo / sum_rtk;
    
    /* 计算残差 */
    for (uint32_t i = 0; i < num_segments; i++) {
        residuals[i] = odo_dists[i] - initial_factor * rtk_dists[i];
    }
    
    /* 第三步：检测离群点 */
    uint32_t outlier_count = 0;
    mark_outliers(residuals, num_segments, is_outlier, &outlier_count);
    manager->outlier_count = outlier_count;
    
    /* 第四步：加权最小二乘法（排除离群点） */
    double numerator = 0.0;
    double denominator = 0.0;
    uint32_t inlier_count = 0;
    
    for (uint32_t i = 0; i < num_segments; i++) {
        if (!is_outlier[i]) {
            double w = weights[i];
            numerator += w * odo_dists[i] * rtk_dists[i];
            denominator += w * rtk_dists[i] * rtk_dists[i];
            inlier_count++;
        }
    }
    
    if (denominator < 1e-6 || inlier_count < 1) {
        free(odo_dists);
        free(rtk_dists);
        free(weights);
        free(is_outlier);
        free(residuals);
        return 1.0;
    }
    
    double calibration_factor = numerator / denominator;
    calibration_factor = clamp(calibration_factor, 0.8, 1.2);
    
    /* 第五步：计算拟合优度和残差标准差 */
    double ss_res = 0.0;
    double ss_tot = 0.0;
    double sum_odo_weighted = 0.0;
    double sum_weight_inlier = 0.0;
    
    for (uint32_t i = 0; i < num_segments; i++) {
        if (!is_outlier[i]) {
            double w = weights[i];
            sum_odo_weighted += w * odo_dists[i];
            sum_weight_inlier += w;
        }
    }
    
    double mean_odo = (sum_weight_inlier > 1e-6) ? sum_odo_weighted / sum_weight_inlier : 0.0;
    
    for (uint32_t i = 0; i < num_segments; i++) {
        if (!is_outlier[i]) {
            double w = weights[i];
            double residual = odo_dists[i] - calibration_factor * rtk_dists[i];
            ss_res += w * residual * residual;
            double diff = odo_dists[i] - mean_odo;
            ss_tot += w * diff * diff;
        }
    }
    
    /* 计算R² */
    if (ss_tot > 1e-6) {
        *r_squared = 1.0 - (ss_res / ss_tot);
        *r_squared = clamp(*r_squared, 0.0, 1.0);
    } else {
        *r_squared = 0.0;
    }
    
    /* 加权标准差 */
    if (inlier_count > 1 && sum_weight_inlier > 1e-6) {
        *weighted_std_dev = sqrt(ss_res / sum_weight_inlier);
    } else {
        *weighted_std_dev = 0.0;
    }
    
    free(odo_dists);
    free(rtk_dists);
    free(weights);
    free(is_outlier);
    free(residuals);
    
    return calibration_factor;
}

/**
 * @brief 最小二乘法拟合转向角零偏和增益
 */
static void estimate_steering_calibration(RTKCalibrationManager_t *manager,
                                         double *offset, double *gain) {
    *offset = 0.0;
    *gain = 1.0;
    
    if (manager->calibration_count < 3) {
        return;
    }
    
    /* 基于航向角误差估计 */
    double sum_error = 0.0;
    for (uint32_t i = 0; i < manager->calibration_count; i++) {
        sum_error += manager->calibration_points[i].heading_error;
    }
    
    /* 平均航向角误差作为零偏估计 */
    *offset = sum_error / manager->calibration_count;
    
    /* 增益通常接近1.0 */
    *gain = 1.0;
}

int rtk_calibration_compute(RTKCalibrationManager_t *manager,
                            CalibrationParams_t *calibration) {
    if (!manager || !calibration) {
        return -1;
    }
    
    if (manager->calibration_count < 3) {
        return -1;  /* 点数不足 */
    }
    
    /* 估计轮速计校准因子（加权最小二乘法） */
    calibration->wheel_calibration_factor = 
        estimate_wheel_calibration_weighted(manager,
                                            &manager->r_squared,
                                            &manager->weighted_std_dev);
    
    /* 限制合理范围 */
    calibration->wheel_calibration_factor = 
        clamp(calibration->wheel_calibration_factor, 0.8, 1.2);
    
    /* 估计转向角校准参数（使用临时 double 变量以支持混合精度）*/
    double steering_offset_d = (double)calibration->steering_angle_offset;
    double steering_gain_d = (double)calibration->steering_angle_gain;
    
    estimate_steering_calibration(manager, &steering_offset_d, &steering_gain_d);
    
    calibration->steering_angle_offset = (float)steering_offset_d;
    calibration->steering_angle_gain = (float)steering_gain_d;
    
    /* Phase 2 (P1) 如果有足够的转向增益标定点，进行转向增益估计 */
    if (manager->steering_gain_manager.point_count >= 10) {
        rtk_calibration_compute_steering_gain(manager);
        if (manager->steering_gain_valid) {
            /* 使用转向增益估计结果更新校准参数 */
            calibration->steering_angle_gain = 
                (float)rtk_calibration_get_steering_gain(manager);
        }
    }
    
    /* 其他参数保持默认值 */
    calibration->imu_accel_bias_x = 0.0;
    calibration->imu_accel_bias_y = 0.0;
    calibration->imu_gyro_bias_z = 0.0;
    calibration->imu_accel_scale_x = 1.0;
    calibration->imu_accel_scale_y = 1.0;
    
    /* 保存估计的校准参数 */
    manager->estimated_calibration = *calibration;
    manager->calibration_valid = true;
    
    return 0;
}

bool rtk_calibration_is_valid(const RTKCalibrationManager_t *manager) {
    if (!manager) {
        return false;
    }
    return manager->calibration_valid && manager->calibration_count >= 3;
}

double rtk_calibration_get_avg_position_error(const RTKCalibrationManager_t *manager) {
    if (!manager || manager->calibration_count == 0) {
        return 0.0;
    }
    
    return sqrt((manager->cumulative_x_error * manager->cumulative_x_error +
                manager->cumulative_y_error * manager->cumulative_y_error) /
               (manager->calibration_count * manager->calibration_count));
}

double rtk_calibration_get_avg_heading_error(const RTKCalibrationManager_t *manager) {
    if (!manager || manager->calibration_count == 0) {
        return 0.0;
    }
    
    return fabs(manager->cumulative_heading_error / manager->calibration_count);
}

void rtk_calibration_clear(RTKCalibrationManager_t *manager) {
    if (!manager) {
        return;
    }
    
    manager->calibration_count = 0;
    manager->cumulative_x_error = 0.0;
    manager->cumulative_y_error = 0.0;
    manager->cumulative_heading_error = 0.0;
    manager->max_position_error = 0.0;
    manager->max_heading_error = 0.0;
    manager->outlier_count = 0;
    manager->r_squared = 0.0;
    manager->weighted_std_dev = 0.0;
    manager->rejected_count = 0;
    manager->low_quality_skip = 0;
    manager->calibration_valid = false;
}

/**
 * @brief 添加标定点（带数据验证）
 * 
 * 验证三项内容：
 * 1. 时间同步检查：±50ms时间差容限
 * 2. 速度一致性检查：最大0.5 m/s差值
 * 3. 轨迹连续性检查：最大10m跳跃
 * 
 * @param manager 管理器指针
 * @param odometry_state 里程计估计的状态
 * @param rtk_state RTK测量的状态
 * @return 0成功，-1验证失败，-2其他失败
 */
int rtk_calibration_add_point_with_validation(RTKCalibrationManager_t *manager,
                                              const VehicleState_t *odometry_state,
                                              const RTKData_t *rtk_state) {
    if (!manager || !odometry_state || !rtk_state) {
        return -2;
    }
    
    /* 基本质量检查 */
    if (rtk_state->quality < 2) {
        manager->low_quality_skip++;
        return -2;
    }
    
    /* 缓冲区检查 */
    if (manager->calibration_count >= manager->max_points) {
        return -2;
    }
    
    /* 验证1：时间同步检查与速度一致性 */
    if (manager->calibration_count > 0) {
        RTKCalibrationPoint_t *prev = &manager->calibration_points[manager->calibration_count - 1];
        
        /* 检查速度变化来推断时间差 */
        double odo_v = odometry_state->v;
        double prev_odo_v = prev->odometry_state.v;
        
        /* 速度加速度不应超过5 m/s² (合理的车辆加速度) */
        double max_velocity_change = 5.0 * 0.1; /* 假设0.1s采样间隔 */
        double velocity_change = fabs(odo_v - prev_odo_v);
        if (velocity_change > max_velocity_change) {
            /* 可能的时间同步问题 */
            manager->rejected_count++;
            return -1;
        }
        
        /* 验证2：方向改变检测（转弯角度变化） */
        double steering_change = fabs(odometry_state->steering_angle - prev->odometry_state.steering_angle);
        /* 极端的转向角变化可能表示数据异常 */
        if (steering_change > 1.0) {  /* 约57度 */
            manager->rejected_count++;
            return -1;
        }
    }
    
    /* 验证3：轨迹连续性检查（最大10m跳跃） */
    if (manager->calibration_count > 0) {
        RTKCalibrationPoint_t *prev = &manager->calibration_points[manager->calibration_count - 1];
        
        /* 检查RTK轨迹连续性 */
        double rtk_jump = sqrt((rtk_state->x - prev->rtk_state.x) * (rtk_state->x - prev->rtk_state.x) +
                              (rtk_state->y - prev->rtk_state.y) * (rtk_state->y - prev->rtk_state.y));
        if (rtk_jump > 10.0) {
            manager->rejected_count++;
            return -1;
        }
        
        /* 检查里程计轨迹连续性 */
        double odo_jump = sqrt((odometry_state->x - prev->odometry_state.x) * (odometry_state->x - prev->odometry_state.x) +
                              (odometry_state->y - prev->odometry_state.y) * (odometry_state->y - prev->odometry_state.y));
        if (odo_jump > 10.0) {
            manager->rejected_count++;
            return -1;
        }
        
        /* 验证4：轨迹段长度检查（避免微小偏移点） */
        double min_segment_length = 0.01;  /* 最少1cm */
        if (odo_jump < min_segment_length && rtk_jump < min_segment_length) {
            /* 两个点几乎重合，可能是重复或错误的数据 */
            manager->rejected_count++;
            return -1;
        }
    }
    
    /* 所有验证通过，添加点 */
    return rtk_calibration_add_point(manager, odometry_state, rtk_state);
}

void rtk_calibration_free(RTKCalibrationManager_t *manager) {
    if (!manager) {
        return;
    }
    
    if (manager->calibration_points) {
        free(manager->calibration_points);
        manager->calibration_points = NULL;
    }
    
    manager->max_points = 0;
    manager->calibration_count = 0;
}

void rtk_calibration_online_update(RTKCalibrationManager_t *manager,
                                   const VehicleState_t *odometry_state,
                                   const RTKData_t *rtk_state,
                                   double alpha) {
    if (!manager || !odometry_state || !rtk_state) {
        return;
    }
    
    if (alpha < 0.0 || alpha > 1.0) {
        return;
    }
    
    /* 计算当前误差 */
    double pos_error_x = rtk_state->x - odometry_state->x;
    double pos_error_y = rtk_state->y - odometry_state->y;
    double heading_error = normalize_angle(rtk_state->heading - odometry_state->heading);
    
    /* 在线更新轮速计校准因子（使用简单的低通滤波） */
    double current_position_error = sqrt(pos_error_x * pos_error_x + pos_error_y * pos_error_y);
    
    /* 如果位置误差太大，可能是异常，跳过更新 */
    if (current_position_error > 1.0) {
        return;
    }
    
    /* 平滑更新航向角零偏 */
    manager->estimated_calibration.steering_angle_offset = 
        (1.0 - alpha) * manager->estimated_calibration.steering_angle_offset +
        alpha * heading_error;
}

void rtk_calibration_print_diagnostics(const RTKCalibrationManager_t *manager) {
    if (!manager) {
        return;
    }
    
    printf("\n========== RTK 标定诊断信息 ==========\n");
    
    /* 数据统计 */
    printf("【数据统计】\n");
    printf("  总点数: %u\n", manager->calibration_count);
    printf("  离群点数: %u\n", manager->outlier_count);
    printf("  拒绝点数: %u\n", manager->rejected_count);
    printf("  低质量跳过: %u\n", manager->low_quality_skip);
    printf("  有效数据率: %.1f%%\n", 
           manager->calibration_count > 0 ? 
           (100.0 * (manager->calibration_count - manager->outlier_count) / manager->calibration_count) : 0.0);
    
    /* 误差统计 */
    printf("\n【误差统计】\n");
    printf("  最大位置误差: %.3f m\n", manager->max_position_error);
    printf("  最大航向角误差: %.4f rad (%.2f°)\n", 
           manager->max_heading_error, manager->max_heading_error * 180.0 / 3.14159);
    
    if (manager->calibration_count > 0) {
        double avg_x_error = manager->cumulative_x_error / manager->calibration_count;
        double avg_y_error = manager->cumulative_y_error / manager->calibration_count;
        double avg_heading_error = manager->cumulative_heading_error / manager->calibration_count;
        printf("  平均X误差: %.3f m\n", avg_x_error);
        printf("  平均Y误差: %.3f m\n", avg_y_error);
        printf("  平均航向角误差: %.4f rad\n", avg_heading_error);
    }
    
    /* 质量统计 */
    printf("\n【质量统计】\n");
    if (manager->calibration_count > 1) {
        uint32_t quality_counts[6] = {0, 0, 0, 0, 0, 0};
        for (uint32_t i = 0; i < manager->calibration_count; i++) {
            uint32_t q = manager->calibration_points[i].rtk_state.quality;
            if (q <= 5) quality_counts[q]++;
        }
        printf("  质量等级分布:\n");
        for (uint32_t q = 0; q <= 5; q++) {
            printf("    等级%u: %u个 (%.1f%%)\n", q, quality_counts[q],
                   100.0 * quality_counts[q] / manager->calibration_count);
        }
    }
    
    /* 校准结果 */
    printf("\n【校准结果】\n");
    printf("  轮速计校准因子: %.6f\n", manager->estimated_calibration.wheel_calibration_factor);
    printf("  转向角零偏: %.6f rad\n", manager->estimated_calibration.steering_angle_offset);
    printf("  转向角增益: %.6f\n", manager->estimated_calibration.steering_angle_gain);
    printf("  拟合优度 (R²): %.6f\n", manager->r_squared);
    printf("  加权残差标准差: %.6f m\n", manager->weighted_std_dev);
    printf("  校准有效: %s\n", manager->calibration_valid ? "是" : "否");
    
    printf("\n====================================\n\n");
}

/* ============================================================================
 * Phase 2 (P1) 转向角增益标定实现
 * ============================================================================ */

int rtk_calibration_set_steering_wheelbase(RTKCalibrationManager_t *manager,
                                           double wheelbase) {
    if (!manager || wheelbase <= 0.0) {
        return -1;
    }
    
    manager->steering_gain_wheelbase = wheelbase;
    return 0;
}

/**
 * @brief 计算航向角误差并考虑圆周率包裹
 */
static double compute_heading_error(double odo_heading, double rtk_heading) {
    double error = rtk_heading - odo_heading;
    
    /* 归一化到 [-π, π] 范围 */
    while (error > M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;
    
    return error;
}

/**
 * @brief 计算两点间的轨迹距离
 */
static double compute_trajectory_distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

int rtk_calibration_add_steering_gain_point(RTKCalibrationManager_t *manager,
                                            const VehicleState_t *odometry_state,
                                            const RTKData_t *rtk_state) {
    if (!manager || !odometry_state || !rtk_state) {
        return -1;
    }
    
    /* 提取基本参数 */
    double steering_angle = (double)odometry_state->steering_angle;
    double velocity = (double)odometry_state->v;
    double rtk_quality = (double)rtk_state->quality;
    double timestamp = (double)odometry_state->timestamp / 1000.0;  /* 转换为秒 */
    
    /* 计算航向角误差 */
    double heading_error = compute_heading_error(
        (double)odometry_state->heading, (double)rtk_state->heading
    );
    
    /* 计算轨迹距离（从上一点开始累积） */
    double trajectory_distance = 0.0;
    if (manager->steering_gain_manager.point_count > 0) {
        SteeringGainCalibrationPoint_t *last_point = 
            &manager->steering_gain_manager.points[
                manager->steering_gain_manager.point_count - 1
            ];
        /* 使用速度和时间差估计轨迹距离 */
        double time_delta = timestamp - last_point->timestamp;
        if (time_delta > 0 && time_delta < 10.0) {  /* 10秒的合理上限 */
            trajectory_distance = velocity * time_delta;
        }
    }
    
    /* 添加点到转向增益管理器 */
    int result = steering_gain_add_point(
        &manager->steering_gain_manager,
        steering_angle,
        heading_error,
        velocity,
        rtk_quality,
        trajectory_distance,
        timestamp
    );
    
    return result;
}

int rtk_calibration_compute_steering_gain(RTKCalibrationManager_t *manager) {
    if (!manager) {
        return -1;
    }
    
    /* 估计转向增益 */
    uint32_t outlier_indices[MAX_STEERING_GAIN_POINTS];
    uint32_t outlier_count = 0;
    
    int result = steering_gain_estimate(
        &manager->steering_gain_manager,
        outlier_indices,
        &outlier_count
    );
    
    if (result != 0) {
        manager->steering_gain_valid = false;
        return -1;
    }
    
    /* 更新总体校准参数中的转向增益 */
    double estimated_gain = steering_gain_get_estimated_gain(
        &manager->steering_gain_manager
    );
    
    manager->estimated_calibration.steering_angle_gain = (float)estimated_gain;
    manager->steering_gain_valid = true;
    
    return 0;
}

double rtk_calibration_get_steering_gain(const RTKCalibrationManager_t *manager) {
    if (!manager || !manager->steering_gain_valid) {
        return 1.0;
    }
    
    return steering_gain_get_estimated_gain(&manager->steering_gain_manager);
}

double rtk_calibration_get_steering_gain_confidence(const RTKCalibrationManager_t *manager) {
    if (!manager || !manager->steering_gain_valid) {
        return 0.0;
    }
    
    return steering_gain_get_confidence(&manager->steering_gain_manager);
}

double rtk_calibration_get_steering_gain_uncertainty(const RTKCalibrationManager_t *manager) {
    if (!manager || !manager->steering_gain_valid) {
        return 0.0;
    }
    
    return steering_gain_get_uncertainty(&manager->steering_gain_manager);
}

bool rtk_calibration_is_steering_gain_valid(const RTKCalibrationManager_t *manager) {
    if (!manager) {
        return false;
    }
    
    return manager->steering_gain_valid && 
           steering_gain_is_valid(&manager->steering_gain_manager);
}

uint32_t rtk_calibration_get_steering_gain_point_count(const RTKCalibrationManager_t *manager) {
    if (!manager) {
        return 0;
    }
    
    return steering_gain_get_point_count(&manager->steering_gain_manager);
}

void rtk_calibration_clear_steering_gain(RTKCalibrationManager_t *manager) {
    if (!manager) {
        return;
    }
    
    steering_gain_reset(&manager->steering_gain_manager);
    manager->steering_gain_valid = false;
}

/* ============================================================================
 * Phase 2 (P1) 独立的标定系统 - 实现部分
 * ============================================================================ */

int rtk_calibration_system_init(RTKCalibrationSystem_t *sys, uint32_t max_calibration_points) {
    if (!sys || max_calibration_points == 0) {
        return -1;
    }
    
    /* 初始化内部管理器 */
    if (rtk_calibration_init(&sys->manager, max_calibration_points) != 0) {
        return -1;
    }
    
    /* 设置默认状态 */
    sys->is_active = true;
    
    /* 初始化结果为默认值 */
    memset(&sys->calibration_result, 0, sizeof(sys->calibration_result));
    sys->calibration_result.wheel_calibration_factor = 1.0;
    sys->calibration_result.steering_angle_offset = 0.0;
    sys->calibration_result.steering_angle_gain = 1.0;
    sys->calibration_result.imu_accel_scale_x = 1.0;
    sys->calibration_result.imu_accel_scale_y = 1.0;
    sys->calibration_result.imu_lateral_accel_scale = 1.0;
    
    return 0;
}

int rtk_calibration_system_add_point(RTKCalibrationSystem_t *sys,
                                     const VehicleState_t *odometry_state,
                                     const RTKData_t *rtk_state) {
    if (!sys || !odometry_state || !rtk_state || !sys->is_active) {
        return -1;
    }
    
    /* 同时添加到标准标定和转向增益标定 */
    /* 添加标准标定点 */
    if (rtk_calibration_add_point(&sys->manager, odometry_state, rtk_state) != 0) {
        return -1;
    }
    
    /* 添加转向增益标定点（速度和方向满足条件时） */
    double velocity = fabs(odometry_state->v);
    if (velocity > 0.5) {  /* 速度大于0.5 m/s时才进行转向增益标定 */
        rtk_calibration_add_steering_gain_point(&sys->manager, odometry_state, rtk_state);
    }
    
    return 0;
}

int rtk_calibration_system_compute(RTKCalibrationSystem_t *sys) {
    if (!sys || !sys->is_active) {
        return -1;
    }
    
    /* 计算标准校准参数 */
    if (rtk_calibration_compute(&sys->manager, &sys->calibration_result) != 0) {
        return -1;
    }
    
    /* 如果点数足够，也计算转向增益 */
    if (rtk_calibration_get_steering_gain_point_count(&sys->manager) >= 10) {
        if (rtk_calibration_compute_steering_gain(&sys->manager) != 0) {
            /* 转向增益计算失败，但不影响其他校准参数 */
            return -1;
        }
        
        /* 更新结果中的转向增益 */
        sys->calibration_result.steering_angle_gain = 
            rtk_calibration_get_steering_gain(&sys->manager);
    }
    
    return 0;
}

int rtk_calibration_system_get_result(const RTKCalibrationSystem_t *sys,
                                      CalibrationParams_t *result) {
    if (!sys || !result) {
        return -1;
    }
    
    *result = sys->calibration_result;
    return 0;
}

void rtk_calibration_system_clear(RTKCalibrationSystem_t *sys) {
    if (!sys) {
        return;
    }
    
    rtk_calibration_clear(&sys->manager);
    rtk_calibration_clear_steering_gain(&sys->manager);
    
    /* 重置结果为默认值 */
    memset(&sys->calibration_result, 0, sizeof(sys->calibration_result));
    sys->calibration_result.wheel_calibration_factor = 1.0;
    sys->calibration_result.steering_angle_offset = 0.0;
    sys->calibration_result.steering_angle_gain = 1.0;
    sys->calibration_result.imu_accel_scale_x = 1.0;
    sys->calibration_result.imu_accel_scale_y = 1.0;
    sys->calibration_result.imu_lateral_accel_scale = 1.0;
}

void rtk_calibration_system_free(RTKCalibrationSystem_t *sys) {
    if (!sys) {
        return;
    }
    
    rtk_calibration_free(&sys->manager);
    sys->is_active = false;
}

uint32_t rtk_calibration_system_get_point_count(const RTKCalibrationSystem_t *sys) {
    if (!sys) {
        return 0;
    }
    
    return sys->manager.calibration_count;
}

bool rtk_calibration_system_is_valid(const RTKCalibrationSystem_t *sys) {
    if (!sys) {
        return false;
    }
    
    return sys->is_active && rtk_calibration_is_valid(&sys->manager);
}

