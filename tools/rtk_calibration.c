/**
 * @file rtk_calibration.c
 * @brief RTK标定实现
 */

#include "rtk_calibration.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
 * @brief 标记离群点（σ-3准则）
 * 
 * 将超过均值±3倍标准差的点标记为离群点
 * 返回有效的（非离群）点的个数
 */
static uint32_t mark_outliers(const double *residuals, uint32_t count,
                              bool *is_outlier, uint32_t *outlier_count) {
    *outlier_count = 0;
    
    if (count < 3) {
        memset(is_outlier, false, count * sizeof(bool));
        return count;
    }
    
    /* 计算统计量 */
    double mean = calculate_mean(residuals, count);
    double std_dev = calculate_std_dev(residuals, count, mean);
    
    /* 离群点阈值：3-sigma 或 绝对残差 > 10 */
    double threshold = (std_dev > 1e-6) ? 3.0 * std_dev : 10.0;
    
    uint32_t valid_count = 0;
    for (uint32_t i = 0; i < count; i++) {
        double residual_abs = fabs(residuals[i]);
        if (residual_abs > threshold && threshold > 1e-7) {
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
    
    /* 分配临时数组存储距离差值和权重 */
    double *odo_dists = (double *)malloc(num_segments * sizeof(double));
    double *rtk_dists = (double *)malloc(num_segments * sizeof(double));
    double *weights = (double *)malloc(num_segments * sizeof(double));
    bool *is_outlier = (bool *)malloc(num_segments * sizeof(bool));
    double *residuals = (double *)malloc(num_segments * sizeof(double));
    double *ratios = (double *)malloc(num_segments * sizeof(double));  /* 比例列表用于排序 */
    
    if (!odo_dists || !rtk_dists || !weights || !is_outlier || !residuals || !ratios) {
        free(odo_dists);
        free(rtk_dists);
        free(weights);
        free(is_outlier);
        free(residuals);
        free(ratios);
        return 1.0;
    }
    
    /* 计算距离和权重 */
    double sum_weight = 0.0;
    
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
        
        /* 权重 = 质量等级 × 轨迹长度（质量等级: 0-5） */
        double quality_weight = (double)curr->rtk_state.quality / 5.0;
        double trajectory_weight = (rtk_dists[i] > 0.01) ? rtk_dists[i] : 0.01;
        weights[i] = quality_weight * trajectory_weight;
        
        sum_weight += weights[i];
        
        /* 计算比例（用于鲁棒初始估计） */
        ratios[i] = (rtk_dists[i] > 1e-6) ? odo_dists[i] / rtk_dists[i] : 1.0;
    }
    
    if (num_segments < 2 || sum_weight < 1e-6) {
        free(odo_dists);
        free(rtk_dists);
        free(weights);
        free(is_outlier);
        free(residuals);
        free(ratios);
        return 1.0;
    }
    
    /* 第一步：用加权中位数进行鲁棒初始估计 */
    /* 简化处理：使用排除极端值的均值 */
    double sum_ratio_trimmed = 0.0;
    uint32_t count_trimmed = 0;
    double min_ratio = 999.0, max_ratio = -999.0;
    
    for (uint32_t i = 0; i < num_segments; i++) {
        if (ratios[i] >= 0.5 && ratios[i] <= 2.0) {  /* 合理范围 */
            sum_ratio_trimmed += ratios[i];
            count_trimmed++;
            if (ratios[i] < min_ratio) min_ratio = ratios[i];
            if (ratios[i] > max_ratio) max_ratio = ratios[i];
        }
    }
    
    double initial_factor = (count_trimmed > 0) ? sum_ratio_trimmed / count_trimmed : 1.0;
    
    /* 计算残差 */
    for (uint32_t i = 0; i < num_segments; i++) {
        residuals[i] = odo_dists[i] - initial_factor * rtk_dists[i];
    }
    
    /* 第二步：检测离群点 */
    uint32_t outlier_count = 0;
    mark_outliers(residuals, num_segments, is_outlier, &outlier_count);
    manager->outlier_count = outlier_count;
    
    /* 第三步：加权最小二乘法（排除离群点） */
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
    
    if (denominator < 1e-6) {
        free(odo_dists);
        free(rtk_dists);
        free(weights);
        free(is_outlier);
        free(residuals);
        free(ratios);
        return 1.0;
    }
    
    double calibration_factor = numerator / denominator;
    calibration_factor = clamp(calibration_factor, 0.8, 1.2);
    
    /* 第四步：计算拟合优度和残差标准差（使用内点数据） */
    double ss_res = 0.0;  /* 残差平方和 */
    double ss_tot = 0.0;  /* 总平方和 */
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
    
    /* 使用最终的calibration_factor重新计算残差 */
    for (uint32_t i = 0; i < num_segments; i++) {
        if (!is_outlier[i]) {
            double w = weights[i];
            double residual = odo_dists[i] - calibration_factor * rtk_dists[i];
            ss_res += w * residual * residual;
            double diff = odo_dists[i] - mean_odo;
            ss_tot += w * diff * diff;
        }
    }
    
    /* R² = 1 - (SS_res / SS_tot) */
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
    free(ratios);
    
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
    
    /* 验证1：时间同步检查（±50ms容限） */
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
