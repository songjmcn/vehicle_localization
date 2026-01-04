/**
 * @file steering_gain_calibration.c
 * @brief 转向角增益标定核心实现 (Phase 2 - P1)
 * @details 基于航向角误差使用加权最小二乘法估计转向角增益
 * @version 1.0
 * @date 2026-01-04
 */

#include "steering_gain_calibration.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

/* ============================================================================
 * 辅助函数
 * ============================================================================ */

/**
 * @brief 比较两个double值用于qsort
 */
static int compare_doubles(const void *a, const void *b) {
    double diff = *(double *)a - *(double *)b;
    if (diff < -1e-9) return -1;
    if (diff > 1e-9) return 1;
    return 0;
}

/**
 * @brief 计算中位数
 */
static double compute_median(double *values, uint32_t count) {
    if (count == 0) return 0.0;
    
    qsort(values, count, sizeof(double), compare_doubles);
    
    if (count % 2 == 1) {
        return values[count / 2];
    } else {
        return (values[count / 2 - 1] + values[count / 2]) / 2.0;
    }
}

/**
 * @brief 计算中位数绝对偏差 (MAD)
 */
static double compute_mad(double *values, uint32_t count) {
    if (count == 0) return 0.0;
    
    double median = compute_median(values, count);
    double *deviations = (double *)malloc(count * sizeof(double));
    if (!deviations) return -1.0;
    
    for (uint32_t i = 0; i < count; i++) {
        deviations[i] = fabs(values[i] - median);
    }
    
    double mad = compute_median(deviations, count);
    free(deviations);
    
    return mad;
}

/**
 * @brief 计算加权平均值
 */
static double compute_weighted_average(double *values, double *weights, uint32_t count) {
    if (count == 0) return 0.0;
    
    double sum = 0.0;
    double weight_sum = 0.0;
    
    for (uint32_t i = 0; i < count; i++) {
        sum += values[i] * weights[i];
        weight_sum += weights[i];
    }
    
    return weight_sum > 1e-9 ? sum / weight_sum : 0.0;
}

/**
 * @brief 获取速度权重因子
 * @details 根据车速分配权重
 */
static double get_velocity_weight(double velocity) {
    if (velocity < 0.5) {
        return 0.3;  /* 低速，噪声主导，权重降低 */
    } else if (velocity < 5.0) {
        return 1.0;  /* 正常范围 */
    } else {
        return 0.8;  /* 高速，非线性增强 */
    }
}

/**
 * @brief 计算轨迹连续性权重
 * @details w = log(1 + distance)
 */
static double get_continuity_weight(double distance) {
    if (distance < 0.01) {
        return 0.1;  /* 距离太小 */
    }
    return log(1.0 + distance);
}

/* ============================================================================
 * 公开函数实现
 * ============================================================================ */

int steering_gain_manager_init(SteeringGainManager_t *manager) {
    if (!manager) return -1;
    
    memset(manager, 0, sizeof(SteeringGainManager_t));
    
    manager->estimated_gain = 1.0;
    manager->gain_bias = 0.0;
    manager->gain_uncertainty = -1.0;
    manager->r_squared = 0.0;
    manager->confidence = 0.0;
    manager->is_valid = 0;
    
    return 0;
}

int steering_gain_add_point(
    SteeringGainManager_t *manager,
    double steering_angle,
    double heading_error,
    double velocity,
    double rtk_quality,
    double trajectory_distance,
    double timestamp
) {
    if (!manager) return -1;
    if (manager->point_count >= MAX_STEERING_GAIN_POINTS) return -1;
    
    /* 基本有效性检查 */
    if (velocity < MIN_VALID_VELOCITY) return -1;
    if (velocity > MAX_VALID_VELOCITY) return -1;
    if (fabs(steering_angle) > MAX_STEERING_ANGLE) return -1;
    if (rtk_quality < 0.0 || rtk_quality > 5.0) return -1;
    if (!isfinite(heading_error) || !isfinite(steering_angle)) return -1;
    
    uint32_t idx = manager->point_count;
    SteeringGainCalibrationPoint_t *point = &manager->points[idx];
    
    point->timestamp = timestamp;
    point->steering_angle = steering_angle;
    point->heading_error = heading_error;
    point->velocity = velocity;
    point->rtk_quality = rtk_quality;
    point->trajectory_distance = trajectory_distance;
    point->is_outlier = 0;
    point->residual = 0.0;
    
    /* 初始权重计算 */
    point->computed_weight = steering_gain_compute_weight(
        steering_angle, velocity, rtk_quality, trajectory_distance
    );
    
    manager->point_count++;
    
    /* 更新统计范围 */
    if (idx == 0) {
        manager->min_velocity = velocity;
        manager->max_velocity = velocity;
        manager->min_steering_angle = steering_angle;
        manager->max_steering_angle = steering_angle;
    } else {
        if (velocity < manager->min_velocity) manager->min_velocity = velocity;
        if (velocity > manager->max_velocity) manager->max_velocity = velocity;
        if (steering_angle < manager->min_steering_angle) 
            manager->min_steering_angle = steering_angle;
        if (steering_angle > manager->max_steering_angle)
            manager->max_steering_angle = steering_angle;
    }
    
    return 0;
}

double steering_gain_compute_weight(
    double steering_angle,
    double velocity,
    double rtk_quality,
    double trajectory_distance
) {
    if (velocity < MIN_VALID_VELOCITY) return 0.0;
    if (fabs(steering_angle) < 1e-6) return 0.0;  /* 直行时转向增益无法估计 */
    
    /* 质量权重: (q/5)² */
    double quality_weight = (rtk_quality / 5.0) * (rtk_quality / 5.0);
    
    /* 速度权重 */
    double velocity_weight = get_velocity_weight(velocity);
    
    /* 连续性权重 */
    double continuity_weight = get_continuity_weight(trajectory_distance);
    
    /* 不确定性修正: 1/(q + 0.1) */
    double uncertainty_correction = 1.0 / (rtk_quality + 0.1);
    
    /* 综合权重 */
    double weight = quality_weight * velocity_weight * continuity_weight * uncertainty_correction;
    
    /* 限制在 [0, 1] 范围 */
    if (weight < 0.0) weight = 0.0;
    if (weight > 1.0) weight = 1.0;
    
    return weight;
}

int steering_gain_detect_outliers(
    SteeringGainManager_t *manager,
    uint8_t *outlier_flags,
    uint32_t *outlier_count
) {
    if (!manager || manager->point_count < 3) return -1;
    
    /* 临时数组存储残差 */
    double *residuals = (double *)malloc(manager->point_count * sizeof(double));
    if (!residuals) return -1;
    
    /* 计算所有点的残差（使用加权平均作为初步估计） */
    double heading_mean = 0.0;
    double weight_sum = 0.0;
    
    for (uint32_t i = 0; i < manager->point_count; i++) {
        double w = manager->points[i].computed_weight;
        heading_mean += manager->points[i].heading_error * w;
        weight_sum += w;
    }
    
    heading_mean = weight_sum > 1e-9 ? heading_mean / weight_sum : 0.0;
    
    /* 计算初步的增益估计（简单线性回归） */
    double sum_xy = 0.0;
    double sum_x2 = 0.0;
    
    for (uint32_t i = 0; i < manager->point_count; i++) {
        double w = manager->points[i].computed_weight;
        double x = manager->points[i].steering_angle * manager->points[i].velocity;
        double y = manager->points[i].heading_error;
        
        sum_xy += w * x * y;
        sum_x2 += w * x * x;
    }
    
    double slope = (sum_x2 > 1e-9) ? sum_xy / sum_x2 : 0.0;
    
    /* 计算残差 */
    for (uint32_t i = 0; i < manager->point_count; i++) {
        double x = manager->points[i].steering_angle * manager->points[i].velocity;
        double y = manager->points[i].heading_error;
        double predicted = slope * x;
        residuals[i] = y - predicted;
    }
    
    /* 计算MAD */
    double mad = compute_mad(residuals, manager->point_count);
    double median_residual = compute_median(residuals, manager->point_count);
    
    /* 标记离群点 */
    uint32_t n_outliers = 0;
    for (uint32_t i = 0; i < manager->point_count; i++) {
        double deviation = fabs(residuals[i] - median_residual);
        
        if (deviation > MAD_SEVERE_OUTLIER_THRESHOLD * mad) {
            /* 严重离群 */
            outlier_flags[i] = 1;
            manager->points[i].is_outlier = 1;
            n_outliers++;
        } else if (deviation > MAD_OUTLIER_THRESHOLD * mad) {
            /* 轻度离群 */
            outlier_flags[i] = 0;  /* 保留但权重会降低 */
            manager->points[i].is_outlier = 0;
        } else {
            /* 正常点 */
            outlier_flags[i] = 0;
            manager->points[i].is_outlier = 0;
        }
    }
    
    free(residuals);
    *outlier_count = n_outliers;
    
    return 0;
}

int steering_gain_estimate(
    SteeringGainManager_t *manager,
    uint32_t *outlier_indices,
    uint32_t *outlier_count
) {
    if (!manager || manager->point_count < 3) return -1;
    
    /* 离群检测 */
    uint8_t *outlier_flags = (uint8_t *)malloc(manager->point_count * sizeof(uint8_t));
    if (!outlier_flags) return -1;
    
    uint32_t n_outliers = 0;
    if (steering_gain_detect_outliers(manager, outlier_flags, &n_outliers) < 0) {
        free(outlier_flags);
        return -1;
    }
    
    /* 收集有效点 */
    double *x_values = (double *)malloc(manager->point_count * sizeof(double));
    double *y_values = (double *)malloc(manager->point_count * sizeof(double));
    double *weights = (double *)malloc(manager->point_count * sizeof(double));
    
    if (!x_values || !y_values || !weights) {
        free(x_values);
        free(y_values);
        free(weights);
        free(outlier_flags);
        return -1;
    }
    
    uint32_t valid_count = 0;
    double total_weight = 0.0;
    
    for (uint32_t i = 0; i < manager->point_count; i++) {
        if (outlier_flags[i]) continue;  /* 跳过离群点 */
        
        /* x = steering_angle * velocity */
        double x = manager->points[i].steering_angle * manager->points[i].velocity;
        if (fabs(x) < 1e-6) continue;  /* 跳过直行情况 */
        
        x_values[valid_count] = x;
        y_values[valid_count] = manager->points[i].heading_error;
        weights[valid_count] = manager->points[i].computed_weight;
        total_weight += weights[valid_count];
        
        valid_count++;
    }
    
    /* 权重正规化 */
    if (valid_count < 3 || total_weight < 1e-9) {
        manager->is_valid = 0;
        manager->confidence = 0.0;
        goto cleanup;
    }
    
    for (uint32_t i = 0; i < valid_count; i++) {
        weights[i] /= total_weight;
    }
    
    /* 加权最小二乘法求解 */
    /* 目标: minimize sum(w_i * (y_i - a - b*x_i)^2) */
    /* 约束假设: a ≈ 0 (直接求b) */
    
    double weighted_xy = 0.0;
    double weighted_x2 = 0.0;
    
    for (uint32_t i = 0; i < valid_count; i++) {
        double w = weights[i];
        weighted_xy += w * x_values[i] * y_values[i];
        weighted_x2 += w * x_values[i] * x_values[i];
    }
    
    double slope = (weighted_x2 > 1e-9) ? weighted_xy / weighted_x2 : 0.0;
    
    /* 从斜率反算增益 */
    /* heading_error ≈ k * (gain - 1) * steering * velocity */
    /* 其中 k ≈ 1/L (L为轴距，通常2.5~3.0) */
    /* slope ≈ k * (gain - 1) */
    double k_estimate = 1.0 / DEFAULT_WHEELBASE;  /* 参考轴距 */
    
    manager->gain_bias = slope / k_estimate;
    manager->estimated_gain = 1.0 + manager->gain_bias;
    
    /* 计算统计指标 */
    double sum_sq_residual = 0.0;
    double ss_tot = 0.0;
    double weighted_mean = compute_weighted_average(y_values, weights, valid_count);
    
    for (uint32_t i = 0; i < valid_count; i++) {
        double predicted = slope * x_values[i];
        double residual = y_values[i] - predicted;
        double ss_weight = weights[i];
        
        sum_sq_residual += ss_weight * residual * residual;
        ss_tot += ss_weight * (y_values[i] - weighted_mean) * (y_values[i] - weighted_mean);
        
        manager->points[i].residual = residual;
    }
    
    /* R² 计算 */
    manager->r_squared = (ss_tot > 1e-9) ? 1.0 - (sum_sq_residual / ss_tot) : 0.0;
    if (manager->r_squared < 0.0) manager->r_squared = 0.0;
    if (manager->r_squared > 1.0) manager->r_squared = 1.0;
    
    /* 加权标准差 */
    manager->weighted_std_dev = sqrt(sum_sq_residual / valid_count);
    
    /* 增益不确定度（粗估）*/
    manager->gain_uncertainty = manager->weighted_std_dev / k_estimate;
    
    /* 统计信息 */
    manager->valid_point_count = valid_count;
    manager->outlier_count = n_outliers;
    manager->average_weight = total_weight / valid_count;
    manager->total_weight = total_weight;
    
    /* 计算置信度 */
    double conf_r2 = (manager->r_squared > 0.85) ? 1.0 : manager->r_squared / 0.85;
    double conf_points = (double)valid_count / (double)fmax(10, manager->point_count);
    double conf_outlier = 1.0 - (double)n_outliers / (double)manager->point_count;
    
    manager->confidence = conf_r2 * conf_points * conf_outlier;
    if (manager->confidence < 0.0) manager->confidence = 0.0;
    if (manager->confidence > 1.0) manager->confidence = 1.0;
    
    /* 有效性检查 */
    manager->is_valid = (manager->r_squared > 0.7 && manager->confidence > 0.5) ? 1 : 0;
    
    /* 复制离群点索引 */
    if (outlier_indices) {
        uint32_t idx = 0;
        for (uint32_t i = 0; i < manager->point_count && idx < n_outliers; i++) {
            if (outlier_flags[i]) {
                outlier_indices[idx++] = i;
            }
        }
    }
    if (outlier_count) {
        *outlier_count = n_outliers;
    }
    
cleanup:
    free(x_values);
    free(y_values);
    free(weights);
    free(outlier_flags);
    
    return 0;
}

double steering_gain_get_estimated_gain(const SteeringGainManager_t *manager) {
    if (!manager || !manager->is_valid) return -1.0;
    return manager->estimated_gain;
}

double steering_gain_get_uncertainty(const SteeringGainManager_t *manager) {
    if (!manager || !manager->is_valid) return -1.0;
    return manager->gain_uncertainty;
}

double steering_gain_get_confidence(const SteeringGainManager_t *manager) {
    if (!manager) return 0.0;
    return manager->confidence;
}

int steering_gain_is_valid(const SteeringGainManager_t *manager) {
    if (!manager) return 0;
    return manager->is_valid;
}

void steering_gain_print_diagnostics(const SteeringGainManager_t *manager) {
    if (!manager) return;
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║  Steering Gain Calibration Diagnostics (Phase 2 - P1)  ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    
    /* 数据统计 */
    printf("\n📊 数据统计\n");
    printf("  总点数: %u\n", manager->point_count);
    printf("  有效点数: %u\n", manager->valid_point_count);
    printf("  离群点数: %u\n", manager->outlier_count);
    printf("  有效率: %.1f%%\n", manager->point_count > 0 ? 
           100.0 * manager->valid_point_count / manager->point_count : 0.0);
    
    /* 速度范围 */
    printf("\n🚗 车速范围\n");
    printf("  最小: %.2f m/s\n", manager->min_velocity);
    printf("  最大: %.2f m/s\n", manager->max_velocity);
    
    /* 转向范围 */
    printf("\n🎛️  转向角范围\n");
    printf("  最小: %.4f rad (%.2f度)\n", manager->min_steering_angle,
           manager->min_steering_angle * 180.0 / M_PI);
    printf("  最大: %.4f rad (%.2f度)\n", manager->max_steering_angle,
           manager->max_steering_angle * 180.0 / M_PI);
    
    /* 权重信息 */
    printf("\n权重信息\n");
    printf("  平均权重: %.4f\n", manager->average_weight);
    printf("  权重总和: %.2f\n", manager->total_weight);
    
    /* 拟合质量 */
    printf("\n拟合质量\n");
    printf("  R: %.4f\n", manager->r_squared);
    printf("  加权标准差: %.6f rad\n", manager->weighted_std_dev);
    printf("  置信度: %.2f (%.1f%%)\n", manager->confidence, manager->confidence * 100.0);
    
    /* 增益估计 */
    printf("\n转向角增益估计\n");
    if (manager->is_valid) {
        printf("  状态: 有效\n");
        printf("  估计增益: %.6f\n", manager->estimated_gain);
        printf("  增益偏差: %+.6f\n", manager->gain_bias);
        printf("  不确定度: ±%.6f\n", manager->gain_uncertainty);
    } else {
        printf("  状态: 无效\n");
        printf("  原因: 数据不足或拟合质量差\n");
    }
    
    printf("\n");
}

void steering_gain_reset(SteeringGainManager_t *manager) {
    if (manager) {
        steering_gain_manager_init(manager);
    }
}

uint32_t steering_gain_get_point_count(const SteeringGainManager_t *manager) {
    return manager ? manager->point_count : 0;
}

int steering_gain_get_point(const SteeringGainManager_t *manager, uint32_t index,
                           SteeringGainCalibrationPoint_t *point) {
    if (!manager || !point || index >= manager->point_count) return -1;
    
    *point = manager->points[index];
    return 0;
}

int steering_gain_load_csv(SteeringGainManager_t *manager, const char *filename) {
    if (!manager || !filename) return -1;
    
    FILE *file = fopen(filename, "r");
    if (!file) return -1;
    
    steering_gain_reset(manager);
    
    char buffer[512];
    int loaded = 0;
    
    /* 跳过CSV头 */
    if (fgets(buffer, sizeof(buffer), file) == NULL) {
        fclose(file);
        return -1;
    }
    
    /* 读取数据行 */
    while (fgets(buffer, sizeof(buffer), file)) {
        double timestamp, steering_angle, heading_error, velocity, rtk_quality, trajectory_distance;
        
        if (sscanf(buffer, "%lf,%lf,%lf,%lf,%lf,%lf",
                   &timestamp, &steering_angle, &heading_error,
                   &velocity, &rtk_quality, &trajectory_distance) == 6) {
            
            if (steering_gain_add_point(manager, steering_angle, heading_error,
                                       velocity, rtk_quality, trajectory_distance,
                                       timestamp) == 0) {
                loaded++;
            }
        }
    }
    
    fclose(file);
    return loaded;
}

int steering_gain_save_csv(const SteeringGainManager_t *manager, const char *filename) {
    if (!manager || !filename) return -1;
    
    FILE *file = fopen(filename, "w");
    if (!file) return -1;
    
    /* 写入CSV头 */
    fprintf(file, "timestamp,steering_angle,heading_error,velocity,rtk_quality,trajectory_distance\n");
    
    /* 写入数据 */
    int saved = 0;
    for (uint32_t i = 0; i < manager->point_count; i++) {
        const SteeringGainCalibrationPoint_t *p = &manager->points[i];
        
        if (fprintf(file, "%.6f,%.8f,%.8f,%.4f,%.1f,%.4f\n",
                   p->timestamp, p->steering_angle, p->heading_error,
                   p->velocity, p->rtk_quality, p->trajectory_distance) > 0) {
            saved++;
        }
    }
    
    fclose(file);
    return saved;
}

int steering_gain_save_diagnostics_csv(const SteeringGainManager_t *manager,
                                     const char *filename) {
    if (!manager || !filename) return -1;
    
    FILE *file = fopen(filename, "w");
    if (!file) return -1;
    
    /* 写入CSV头 */
    fprintf(file, "index,steering_angle,heading_error,velocity,rtk_quality,weight,is_outlier,residual\n");
    
    /* 写入数据 */
    int saved = 0;
    for (uint32_t i = 0; i < manager->point_count; i++) {
        const SteeringGainCalibrationPoint_t *p = &manager->points[i];
        
        if (fprintf(file, "%u,%.8f,%.8f,%.4f,%.1f,%.6f,%d,%.8f\n",
                   i, p->steering_angle, p->heading_error,
                   p->velocity, p->rtk_quality, p->computed_weight,
                   p->is_outlier, p->residual) > 0) {
            saved++;
        }
    }
    
    fclose(file);
    return saved;
}
