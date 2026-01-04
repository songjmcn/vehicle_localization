#ifndef OFFLINE_CALIBRATION_CSV_H
#define OFFLINE_CALIBRATION_CSV_H

#include <stdint.h>
#include <time.h>

/**
 * @file offline_calibration_csv.h
 * @brief 离线标定程序头文件
 * 
 * 使用 vehicle_localization.csv 和 rtk_converted.csv 进行离线标定，
 * 生成 JSON 格式的标定结果。
 */

/* ==================== 数据结构定义 ==================== */

/**
 * @brief 里程计测量数据（来自 vehicle_localization.csv）
 */
typedef struct {
    double x;                  // 后轮轴中心 X 位置 (m)
    double y;                  // 后轮轴中心 Y 位置 (m)
    double heading;            // 偏航角 (rad)
    double steering_angle;     // 前轮转向角 (rad)
    char gear[16];             // 档位 (Drive/Reverse/Park)
    uint64_t timestamp_ms;     // 时间戳 (ms)
} OdometryMeasurement_t;

/**
 * @brief RTK 测量数据（来自 rtk_converted.csv）
 */
typedef struct {
    double x;                  // RTK x 位置 (m)
    double y;                  // RTK y 位置 (m)
    double heading_deg;        // 航向角 (度)
    double heading_rad;        // 航向角 (rad)
    double z;                  // 高度 (m)
    uint8_t quality;           // 定位质量 (0-5, 5最优)
    double timestamp_sec;      // 时间戳 (秒, 浮点数)
    uint64_t timestamp_ms;     // 转换后的时间戳 (ms)
} RTKMeasurement_t;

/**
 * @brief 对齐的标定点
 */
typedef struct {
    OdometryMeasurement_t odo;
    RTKMeasurement_t rtk;
    double position_error_m;   // 位置误差 (m)
    double heading_error_rad;  // 航向角误差 (rad)
    uint8_t valid;             // 是否有效 (1=有效, 0=无效)
} CalibrationPoint_t;

/**
 * @brief 标定统计信息
 */
typedef struct {
    uint32_t total_odo_points;
    uint32_t total_rtk_points;
    uint32_t matched_points;
    uint32_t valid_points;
    uint32_t rejected_points;
    
    // 位置误差统计
    double position_error_mean;
    double position_error_std;
    double position_error_min;
    double position_error_max;
    
    // 航向角误差统计
    double heading_error_mean;
    double heading_error_std;
    double heading_error_max;
    
    // 数据质量
    uint32_t rtk_quality_distribution[6];  // [0]=quality 0, [5]=quality 5
    uint32_t low_speed_count;   // < 0.1 m/s
    uint32_t medium_speed_count;// 0.1-5.0 m/s
    uint32_t high_speed_count;  // > 5.0 m/s
} CalibrationStats_t;

/**
 * @brief 离线标定结果
 */
typedef struct {
    float wheel_calibration_factor;
    float wheel_factor_confidence;
    float wheel_factor_uncertainty;
    
    float steering_angle_offset_rad;
    float steering_angle_offset_deg;
    float steering_angle_gain;
    float steering_gain_confidence;
    float steering_gain_uncertainty;
    
    float imu_accel_bias_x;
    float imu_accel_bias_y;
    float imu_gyro_bias_z;
    float imu_accel_scale_x;
    float imu_accel_scale_y;
    
    // 质量指标
    float r_squared;
    float avg_position_error;
    float max_position_error;
    float avg_heading_error;
    float confidence_score;
    
    // 时间戳
    time_t calibration_time;
    char calibration_time_str[64];
} CalibrationResult_t;

/* ==================== 函数接口 ==================== */

/**
 * @brief 从 CSV 文件读取里程计数据
 */
int load_odometry_csv(const char *filename, OdometryMeasurement_t *measurements, int max_count);

/**
 * @brief 从 CSV 文件读取 RTK 数据
 */
int load_rtk_csv(const char *filename, RTKMeasurement_t *measurements, int max_count);

/**
 * @brief 对齐里程计和 RTK 数据
 */
int align_measurements(const OdometryMeasurement_t *odo_data, int odo_count,
                       const RTKMeasurement_t *rtk_data, int rtk_count,
                       CalibrationPoint_t *calib_points, int max_points);

/**
 * @brief 计算标定统计信息
 */
int compute_calibration_stats(const CalibrationPoint_t *points, int count, CalibrationStats_t *stats);

/**
 * @brief 执行离线标定计算
 */
int perform_calibration(const CalibrationPoint_t *points, int count, CalibrationResult_t *result);

/**
 * @brief 将结果输出为 JSON 文件
 */
int output_json_result(const char *filename, const CalibrationResult_t *result,
                       const CalibrationStats_t *stats, const CalibrationPoint_t *points,
                       int point_count);

/**
 * @brief 打印校准摘要信息
 */
void print_calibration_summary(const CalibrationResult_t *result, const CalibrationStats_t *stats);

#endif // OFFLINE_CALIBRATION_CSV_H
