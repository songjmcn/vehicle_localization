/**
 * @file rtk_calibration.h
 * @brief RTK标定模块
 */

#ifndef RTK_CALIBRATION_H
#define RTK_CALIBRATION_H

#include "localization_types.h"

/**
 * @brief RTK标定管理器
 */
typedef struct {
    RTKCalibrationPoint_t *calibration_points;
    uint32_t calibration_count;
    uint32_t max_points;
    
    /* 累积误差统计 */
    double cumulative_x_error;
    double cumulative_y_error;
    double cumulative_heading_error;
    double max_position_error;
    double max_heading_error;
    
    /* 校准参数 */
    CalibrationParams_t estimated_calibration;
    bool calibration_valid;
} RTKCalibrationManager_t;

/**
 * @brief 初始化RTK标定管理器
 * @param manager 管理器指针
 * @param max_points 最大标定点数
 * @return 0成功，-1失败
 */
int rtk_calibration_init(RTKCalibrationManager_t *manager, uint32_t max_points);

/**
 * @brief 添加标定点
 * @param manager 管理器指针
 * @param odometry_state 里程计估计的状态
 * @param rtk_state RTK测量的状态
 * @return 0成功，-1失败
 */
int rtk_calibration_add_point(RTKCalibrationManager_t *manager,
                              const VehicleState_t *odometry_state,
                              const RTKData_t *rtk_state);

/**
 * @brief 计算校准参数（基于累积的标定点）
 * @param manager 管理器指针
 * @param calibration 输出的校准参数
 * @return 0成功，-1失败
 */
int rtk_calibration_compute(RTKCalibrationManager_t *manager,
                            CalibrationParams_t *calibration);

/**
 * @brief 获取标定统计信息
 * @param manager 管理器指针
 * @return 标定是否有效
 */
bool rtk_calibration_is_valid(const RTKCalibrationManager_t *manager);

/**
 * @brief 获取平均位置误差
 * @param manager 管理器指针
 * @return 平均误差 (m)
 */
double rtk_calibration_get_avg_position_error(const RTKCalibrationManager_t *manager);

/**
 * @brief 获取平均航向角误差
 * @param manager 管理器指针
 * @return 平均误差 (rad)
 */
double rtk_calibration_get_avg_heading_error(const RTKCalibrationManager_t *manager);

/**
 * @brief 清空标定数据
 * @param manager 管理器指针
 */
void rtk_calibration_clear(RTKCalibrationManager_t *manager);

/**
 * @brief 释放RTK标定管理器
 * @param manager 管理器指针
 */
void rtk_calibration_free(RTKCalibrationManager_t *manager);

/**
 * @brief 在线校准：使用单个RTK点调整校准参数
 * @param manager 管理器指针
 * @param odometry_state 里程计状态
 * @param rtk_state RTK状态
 * @param alpha 自适应增益 (0.0 - 1.0)
 */
void rtk_calibration_online_update(RTKCalibrationManager_t *manager,
                                   const VehicleState_t *odometry_state,
                                   const RTKData_t *rtk_state,
                                   double alpha);

#endif /* RTK_CALIBRATION_H */
