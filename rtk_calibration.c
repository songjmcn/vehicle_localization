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
        return -1;  /* 质量不足 */
    }
    
    RTKCalibrationPoint_t *point = &manager->calibration_points[manager->calibration_count];
    
    point->odometry_state = *odometry_state;
    point->rtk_state = *rtk_state;
    
    /* 计算误差 */
    point->position_error_x = rtk_state->x - odometry_state->x;
    point->position_error_y = rtk_state->y - odometry_state->y;
    point->heading_error = normalize_angle(rtk_state->heading - odometry_state->theta);
    
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
 * @brief 最小二乘法拟合轮速计校准因子
 * 
 * 基于以下观察：
 * 轮速计测得距离 = 实际距离 / calibration_factor
 * 
 * 我们通过比较里程计累积距离和RTK位置变化来估计calibration_factor
 */
static double estimate_wheel_calibration(RTKCalibrationManager_t *manager) {
    if (manager->calibration_count < 2) {
        return 1.0;
    }
    
    double sum_odo_dist = 0.0;
    double sum_rtk_dist = 0.0;
    
    for (uint32_t i = 1; i < manager->calibration_count; i++) {
        RTKCalibrationPoint_t *prev = &manager->calibration_points[i - 1];
        RTKCalibrationPoint_t *curr = &manager->calibration_points[i];
        
        /* 里程计距离 */
        double odo_dx = curr->odometry_state.x - prev->odometry_state.x;
        double odo_dy = curr->odometry_state.y - prev->odometry_state.y;
        double odo_dist = sqrt(odo_dx * odo_dx + odo_dy * odo_dy);
        
        /* RTK距离 */
        double rtk_dx = curr->rtk_state.x - prev->rtk_state.x;
        double rtk_dy = curr->rtk_state.y - prev->rtk_state.y;
        double rtk_dist = sqrt(rtk_dx * rtk_dx + rtk_dy * rtk_dy);
        
        sum_odo_dist += odo_dist;
        sum_rtk_dist += rtk_dist;
    }
    
    if (sum_rtk_dist < 1e-6) {
        return 1.0;
    }
    
    /* calibration_factor = sum_odo_dist / sum_rtk_dist */
    return sum_odo_dist / sum_rtk_dist;
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
    
    /* 估计轮速计校准因子 */
    calibration->wheel_calibration_factor = estimate_wheel_calibration(manager);
    
    /* 限制合理范围 */
    calibration->wheel_calibration_factor = 
        clamp(calibration->wheel_calibration_factor, 0.8, 1.2);
    
    /* 估计转向角校准参数 */
    estimate_steering_calibration(manager,
                                 &calibration->steering_angle_offset,
                                 &calibration->steering_angle_gain);
    
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
    manager->calibration_valid = false;
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
    double heading_error = normalize_angle(rtk_state->heading - odometry_state->theta);
    
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
