/**
 * @file rtk_calibration.h
 * @brief RTK标定模块
 * 
 * Phase 2 (P1) 扩展: 包含转向角增益标定功能
 */

#ifndef RTK_CALIBRATION_H
#define RTK_CALIBRATION_H

#include "localization_types.h"
#include "steering_gain_calibration.h"

/**
 * @brief RTK标定管理器
 * 
 * Phase 2 (P1) 扩展: 集成转向角增益标定功能
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
    
    /* 加权最小二乘法统计 */
    uint32_t outlier_count;
    double r_squared;  /* 拟合优度 R² */
    double weighted_std_dev;  /* 加权残差标准差 */
    
    /* 数据质量统计 */
    uint32_t rejected_count;  /* 被拒绝的点数（验证失败） */
    uint32_t low_quality_skip;  /* 因质量低而跳过的点数 */
    
    /* 校准参数 */
    CalibrationParams_t estimated_calibration;
    bool calibration_valid;
    
    /* Phase 2 (P1) 转向增益标定管理器 */
    SteeringGainManager_t steering_gain_manager;
    bool steering_gain_valid;
    double steering_gain_wheelbase;  /* 车轴距(m)，用于增益反演 */
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
 * @brief 添加标定点（带数据验证）
 * 
 * 验证项：
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

/**
 * @brief 打印诊断信息（用于调试和验证算法）
 * 
 * 输出包含：
 * - 数据统计：点数、离群点数、拒绝点数
 * - 权重分析：权重分布、质量等级统计
 * - 拟合结果：校准因子、R²、残差标准差
 * - 迭代过程：初始估计、优化步骤
 * 
 * @param manager 管理器指针
 */
void rtk_calibration_print_diagnostics(const RTKCalibrationManager_t *manager);

/* ============================================================================
 * Phase 2 (P1) 转向角增益标定 API
 * ============================================================================ */

/**
 * @brief 设置转向增益标定的车轴距参数
 * 
 * 默认值: 2.7m (可根据实际车型调整)
 * 车轴距用于从航向角误差反演增益: gain = 1 + slope/(k*wheelbase)
 * 
 * @param manager 管理器指针
 * @param wheelbase 车轴距 (m)
 * @return 0成功，-1失败
 */
int rtk_calibration_set_steering_wheelbase(RTKCalibrationManager_t *manager,
                                           double wheelbase);

/**
 * @brief 添加转向增益标定点
 * 
 * 该函数集成里程计和RTK数据，计算航向角误差和权重，
 * 为转向增益估计提供数据点。
 * 
 * @param manager 管理器指针
 * @param odometry_state 里程计估计的状态
 * @param rtk_state RTK测量的状态
 * @return 0成功，-1失败
 */
int rtk_calibration_add_steering_gain_point(RTKCalibrationManager_t *manager,
                                            const VehicleState_t *odometry_state,
                                            const RTKData_t *rtk_state);

/**
 * @brief 计算转向增益标定参数
 * 
 * 基于累积的转向增益标定点，使用加权最小二乘法估计转向角增益。
 * 该函数也会更新 manager->estimated_calibration 中的 steering_angle_gain 字段。
 * 
 * @param manager 管理器指针
 * @return 0成功，-1失败
 */
int rtk_calibration_compute_steering_gain(RTKCalibrationManager_t *manager);

/**
 * @brief 获取转向增益估计值
 * @param manager 管理器指针
 * @return 估计的增益值 (或 1.0 如果无效)
 */
double rtk_calibration_get_steering_gain(const RTKCalibrationManager_t *manager);

/**
 * @brief 获取转向增益置信度
 * @param manager 管理器指针
 * @return 置信度 [0.0, 1.0]
 */
double rtk_calibration_get_steering_gain_confidence(const RTKCalibrationManager_t *manager);

/**
 * @brief 获取转向增益不确定性（±范围）
 * @param manager 管理器指针
 * @return 不确定性范围
 */
double rtk_calibration_get_steering_gain_uncertainty(const RTKCalibrationManager_t *manager);

/**
 * @brief 检查转向增益标定是否有效
 * @param manager 管理器指针
 * @return true 有效，false 无效或点数不足
 */
bool rtk_calibration_is_steering_gain_valid(const RTKCalibrationManager_t *manager);

/**
 * @brief 获取转向增益标定点数
 * @param manager 管理器指针
 * @return 标定点数
 */
uint32_t rtk_calibration_get_steering_gain_point_count(const RTKCalibrationManager_t *manager);

/**
 * @brief 清空转向增益标定数据
 * @param manager 管理器指针
 */
void rtk_calibration_clear_steering_gain(RTKCalibrationManager_t *manager);

/* ============================================================================
 * Phase 2 (P1) 独立的标定系统 - 完全与定位代码分离
 * ============================================================================ */

/**
 * @brief RTK标定系统（独立运行）
 * 
 * 这是一个独立的校准系统，不依赖于定位系统。
 * 它完全管理标定过程，最后输出 CalibrationParams_t 结果。
 * 定位系统通过外部接口获取校准参数。
 */
typedef struct {
    RTKCalibrationManager_t manager;          /* 内部管理器 */
    bool is_active;                           /* 是否处于活跃标定状态 */
    CalibrationParams_t calibration_result;   /* 最后一次标定结果 */
} RTKCalibrationSystem_t;

/**
 * @brief 初始化标定系统（独立）
 * @param sys 标定系统指针
 * @param max_calibration_points 最大标定点数
 * @return 0成功，-1失败
 */
int rtk_calibration_system_init(RTKCalibrationSystem_t *sys, uint32_t max_calibration_points);

/**
 * @brief 添加标定点到独立系统
 * @param sys 标定系统指针
 * @param odometry_state 里程计状态
 * @param rtk_state RTK状态
 * @return 0成功，-1失败
 */
int rtk_calibration_system_add_point(RTKCalibrationSystem_t *sys,
                                     const VehicleState_t *odometry_state,
                                     const RTKData_t *rtk_state);

/**
 * @brief 计算标定参数（独立）
 * @param sys 标定系统指针
 * @return 0成功，-1失败
 */
int rtk_calibration_system_compute(RTKCalibrationSystem_t *sys);

/**
 * @brief 获取标定结果
 * @param sys 标定系统指针
 * @param result 输出的标定参数
 * @return 0成功，-1失败
 */
int rtk_calibration_system_get_result(const RTKCalibrationSystem_t *sys,
                                      CalibrationParams_t *result);

/**
 * @brief 清空标定数据
 * @param sys 标定系统指针
 */
void rtk_calibration_system_clear(RTKCalibrationSystem_t *sys);

/**
 * @brief 释放标定系统资源
 * @param sys 标定系统指针
 */
void rtk_calibration_system_free(RTKCalibrationSystem_t *sys);

/**
 * @brief 获取标定点数
 * @param sys 标定系统指针
 * @return 点数
 */
uint32_t rtk_calibration_system_get_point_count(const RTKCalibrationSystem_t *sys);

/**
 * @brief 检查标定是否有效
 * @param sys 标定系统指针
 * @return true 有效，false 无效
 */
bool rtk_calibration_system_is_valid(const RTKCalibrationSystem_t *sys);

#endif /* RTK_CALIBRATION_H */
