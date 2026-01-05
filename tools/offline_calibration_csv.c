#include "offline_calibration_csv.h"
#include "rtk_calibration.h"
#include "localization_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ==================== Constants ==================== */

#define MAX_CSV_LINE_LENGTH 1024
#define TIME_SYNC_TOLERANCE_MS 50
#define MAX_VELOCITY_DIFF 0.5
#define MAX_POSITION_JUMP 10.0
#define MIN_RTK_QUALITY 2

/* ==================== Helper Functions ==================== */

/**
 * @brief Normalize angle to (-PI, PI]
 */
static double normalize_angle_internal(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}

/**
 * @brief Split CSV line into fields
 */
static int split_csv_line(char *line, char **fields, int max_fields) {
    int count = 0;
    char *ptr = line;
    char *start = line;
    
    while (*ptr && count < max_fields) {
        if (*ptr == ';') {
            *ptr = '\0';
            while (*start == ' ') start++;
            fields[count++] = start;
            start = ptr + 1;
        }
        ptr++;
    }
    
    if (count < max_fields && start < ptr) {
        while (*start == ' ') start++;
        fields[count++] = start;
    }
    
    return count;
}

/**
 * @brief Trim newline and whitespace from end of line
 */
static void trim_line(char *line) {
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r' || line[len-1] == ' ')) {
        line[--len] = '\0';
    }
}

/* ==================== CSV Loading Functions ==================== */

int load_odometry_csv(const char *filename, OdometryMeasurement_t *measurements, int max_count) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "ERROR: Cannot open odometry file: %s\n", filename);
        return -1;
    }
    
    char line[MAX_CSV_LINE_LENGTH];
    int line_num = 0;
    int meas_count = 0;
    
    // Skip header line
    if (fgets(line, sizeof(line), fp) == NULL) {
        fprintf(stderr, "ERROR: Empty odometry CSV file\n");
        fclose(fp);
        return -1;
    }
    line_num++;
    
    while (fgets(line, sizeof(line), fp) && meas_count < max_count) {
        line_num++;
        trim_line(line);
        
        if (strlen(line) == 0) continue;
        
        char *fields[10];
        int field_count = split_csv_line(line, fields, 10);
        
        if (field_count < 6) {
            fprintf(stderr, "WARNING: Line %d has insufficient fields (need 6, got %d)\n", line_num, field_count);
            continue;
        }
        
        OdometryMeasurement_t *m = &measurements[meas_count];
        m->x = atof(fields[0]);
        m->y = atof(fields[1]);
        m->heading = atof(fields[2]);
        m->steering_angle = atof(fields[3]);
        
        // Copy gear string
        strncpy(m->gear, fields[4], sizeof(m->gear) - 1);
        m->gear[sizeof(m->gear) - 1] = '\0';
        
        m->timestamp_ms = strtoull(fields[5], NULL, 10);
        meas_count++;
    }
    
    fclose(fp);
    
    printf("✓ Loaded %d odometry measurements from %s\n", meas_count, filename);
    return meas_count;
}

int load_rtk_csv(const char *filename, RTKMeasurement_t *measurements, int max_count) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "ERROR: Cannot open RTK file: %s\n", filename);
        return -1;
    }
    
    char line[MAX_CSV_LINE_LENGTH];
    int line_num = 0;
    int meas_count = 0;
    
    // Skip header line
    if (fgets(line, sizeof(line), fp) == NULL) {
        fprintf(stderr, "ERROR: Empty RTK CSV file\n");
        fclose(fp);
        return -1;
    }
    line_num++;
    
    while (fgets(line, sizeof(line), fp) && meas_count < max_count) {
        line_num++;
        trim_line(line);
        
        if (strlen(line) == 0) continue;
        
        char *fields[10];
        int field_count = split_csv_line(line, fields, 10);
        
        if (field_count < 7) {
            fprintf(stderr, "WARNING: Line %d has insufficient fields (need 7, got %d)\n", line_num, field_count);
            continue;
        }
        
        RTKMeasurement_t *m = &measurements[meas_count];
        m->x = atof(fields[0]);
        m->y = atof(fields[1]);
        m->heading_deg = atof(fields[2]);
        m->heading_rad = atof(fields[3]);
        m->z = atof(fields[4]);
        m->quality = (uint8_t)atoi(fields[5]);
        
        // Convert timestamp from seconds (float) to milliseconds (uint64)
        m->timestamp_sec = atof(fields[6]);
        m->timestamp_ms = (uint64_t)(m->timestamp_sec * 1000.0 + 0.5);
        
        if (m->quality >= MIN_RTK_QUALITY) {
            meas_count++;
        }
    }
    
    fclose(fp);
    
    printf("✓ Loaded %d RTK measurements (quality >= %d) from %s\n", meas_count, MIN_RTK_QUALITY, filename);
    return meas_count;
}

/* ==================== Data Alignment ==================== */

int align_measurements(
    const OdometryMeasurement_t *odo_data, int odo_count,
    const RTKMeasurement_t *rtk_data, int rtk_count,
    CalibrationPoint_t *calib_points, int max_points) {
    
    int count = 0;
    
    printf("\nAligning measurements (tolerance: ±%d ms)...\n", TIME_SYNC_TOLERANCE_MS);
    
    // For each RTK measurement, find closest odometry measurement
    for (int i = 0; i < rtk_count && count < max_points; i++) {
        uint64_t rtk_time = rtk_data[i].timestamp_ms;
        
        // Find closest odometry match
        int best_j = -1;
        int64_t best_diff = TIME_SYNC_TOLERANCE_MS + 1;
        
        for (int j = 0; j < odo_count; j++) {
            int64_t diff = llabs((long long)odo_data[j].timestamp_ms - (long long)rtk_time);
            if (diff < best_diff) {
                best_diff = diff;
                best_j = j;
            }
        }
        
        if (best_j >= 0 && best_diff <= TIME_SYNC_TOLERANCE_MS) {
            CalibrationPoint_t *p = &calib_points[count];
            
            // Copy measurements
            p->odo = odo_data[best_j];
            p->rtk = rtk_data[i];
            
            // Compute errors
            double dx = rtk_data[i].x - odo_data[best_j].x;
            double dy = rtk_data[i].y - odo_data[best_j].y;
            p->position_error_m = sqrt(dx * dx + dy * dy);
            
            /* === Heading角范围验证 (ISO 8855一致性) === */
            double heading_diff = rtk_data[i].heading_rad - odo_data[best_j].heading;
            
            /* 保存原始差值(未normalize) */
            p->heading_error_raw = heading_diff;
            
            /* 检测是否为大角度差(>π) */
            p->heading_error_large = (fabs(heading_diff) > M_PI) ? 1 : 0;
            
            /* normalize后的结果用于后续标定 */
            p->heading_error_rad = normalize_angle_internal(heading_diff);
            
            /* 如果heading误差异常大，发出警告 */
            if (p->heading_error_large) {
                fprintf(stderr, "⚠ Warning: Large heading error at RTK point %u: "
                        "raw=%.4f rad (%.2f°), normalized=%.4f rad (%.2f°)\n",
                        i, heading_diff, heading_diff * 180 / M_PI,
                        p->heading_error_rad, p->heading_error_rad * 180 / M_PI);
            }
            
            // Mark as valid if error within reasonable bounds
            p->valid = (p->position_error_m < MAX_POSITION_JUMP) ? 1 : 0;
            
            count++;
        }
    }
    
    printf("✓ Aligned %d point pairs\n", count);
    return count;
}

/* ==================== Statistics Computation ==================== */

int compute_calibration_stats(const CalibrationPoint_t *points, int count, CalibrationStats_t *stats) {
    if (count == 0) {
        fprintf(stderr, "ERROR: No calibration points to analyze\n");
        return -1;
    }
    
    // Initialize
    memset(stats, 0, sizeof(*stats));
    stats->total_rtk_points = count;
    
    // Position error statistics
    double pos_sum = 0, pos_sum_sq = 0;
    double head_sum = 0, head_sum_sq = 0;
    stats->position_error_min = 1e9;
    stats->position_error_max = -1e9;
    stats->heading_error_max = -1e9;
    
    /* ===== Heading大误差统计 ===== */
    uint32_t heading_large_count = 0;
    
    int valid_count = 0;
    
    for (int i = 0; i < count; i++) {
        if (!points[i].valid) continue;
        
        valid_count++;
        
        // Position error
        pos_sum += points[i].position_error_m;
        pos_sum_sq += points[i].position_error_m * points[i].position_error_m;
        
        if (points[i].position_error_m < stats->position_error_min)
            stats->position_error_min = points[i].position_error_m;
        if (points[i].position_error_m > stats->position_error_max)
            stats->position_error_max = points[i].position_error_m;
        
        // Heading error
        double abs_head_err = fabs(points[i].heading_error_rad);
        head_sum += abs_head_err;
        head_sum_sq += points[i].heading_error_rad * points[i].heading_error_rad;
        
        if (abs_head_err > stats->heading_error_max)
            stats->heading_error_max = abs_head_err;
        
        /* 计数大heading误差点 */
        if (points[i].heading_error_large) {
            heading_large_count++;
        }
        
        // Track RTK quality distribution
        if (points[i].rtk.quality < 6) {
            stats->rtk_quality_distribution[points[i].rtk.quality]++;
        }
    }
    
    if (valid_count == 0) {
        fprintf(stderr, "ERROR: No valid calibration points\n");
        return -1;
    }
    
    // Calculate statistics
    stats->valid_points = valid_count;
    stats->position_error_mean = pos_sum / valid_count;
    stats->heading_error_mean = head_sum / valid_count;
    
    /* ===== 更新heading大误差统计 ===== */
    stats->heading_error_large_count = heading_large_count;
    stats->heading_error_large_ratio = (valid_count > 0) 
                                      ? (double)heading_large_count / valid_count 
                                      : 0;
    
    /* 如果大误差占比过高，发出警告 */
    if (stats->heading_error_large_ratio > 0.1) {
        fprintf(stderr, "⚠ WARNING: High proportion of large heading errors (>π): %.1f%% (%d/%d)\n",
                stats->heading_error_large_ratio * 100.0,
                heading_large_count, valid_count);
        fprintf(stderr, "  This suggests potential RTK coordinate transformation issues\n");
        fprintf(stderr, "  (ISO 8855 [-π, π] range assumption may be violated)\n");
    }
    
    double pos_var = (pos_sum_sq / valid_count) - (stats->position_error_mean * stats->position_error_mean);
    stats->position_error_std = (pos_var > 0) ? sqrt(pos_var) : 0;
    
    double head_var = (head_sum_sq / valid_count) - 
                     (stats->heading_error_mean * stats->heading_error_mean);
    stats->heading_error_std = (head_var > 0) ? sqrt(head_var) : 0;
    
    printf("✓ Computed statistics for %d valid points\n", valid_count);
    return 0;
}

/* ==================== Calibration ==================== */

int perform_calibration(const CalibrationPoint_t *points, int count, CalibrationResult_t *result) {
    if (count < 10) {
        fprintf(stderr, "ERROR: Insufficient calibration points (need >= 10, got %d)\n", count);
        return -1;
    }
    
    RTKCalibrationManager_t calib_manager;
    rtk_calibration_init(&calib_manager, 5000);
    
    printf("\nPerforming calibration with %d points...\n", count);
    
    int added = 0;
    for (int i = 0; i < count; i++) {
        if (!points[i].valid) continue;
        
        // Create VehicleState_t from OdometryMeasurement_t
        VehicleState_t odo_state = {0};
        odo_state.x = points[i].odo.x;
        odo_state.y = points[i].odo.y;
        odo_state.heading = points[i].odo.heading;
        odo_state.v = 0.0;  // Velocity not available from CSV
        odo_state.steering_angle = points[i].odo.steering_angle;
        
        // Create RTKData_t from RTKMeasurement_t
        RTKData_t rtk_state = {0};
        rtk_state.x = points[i].rtk.x;
        rtk_state.y = points[i].rtk.y;
        rtk_state.heading = points[i].rtk.heading_rad;
        rtk_state.quality = points[i].rtk.quality;
        
        int ret = rtk_calibration_add_point(&calib_manager, &odo_state, &rtk_state);
        if (ret == 0) {
            added++;
        }
    }
    
    printf("✓ Added %d valid points to calibration system\n", added);
    
    CalibrationParams_t calib_params = {0};
    int ret = rtk_calibration_compute(&calib_manager, &calib_params);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Calibration computation failed\n");
        return -1;
    }

    /* 计算并收集统计信息以写入输出（确保 stats 已计算） */
    CalibrationStats_t stats;
    if (compute_calibration_stats(points, count, &stats) != 0) {
        fprintf(stderr, "ERROR: Failed to compute calibration statistics\n");
        return -1;
    }

    /* Populate result structure */
    result->wheel_calibration_factor = calib_params.wheel_calibration_factor;
    result->wheel_factor_confidence = (added > 20) ? 0.95f : 0.75f;
    result->wheel_factor_uncertainty = 0.01f;
    
    /* 符号处理说明：
     * 离线标定计算：heading_error = RTK_heading - VL_heading
     * 存储含义：offset 代表 VL 相对 RTK 的转向误差
     * 应用公式：cal_steering = raw_steering - offset
     * 
     * 为确保参数语义清晰，这里将计算的偏差反转符号后保存：
     * - 如果 VL 转向过度（+heading_error），offset 应为正（减小转向）
     * - 如果 VL 转向不足（-heading_error），offset 应为负（增加转向）
     */
    result->steering_angle_offset_rad = -1.0f * calib_params.steering_angle_offset;
    result->steering_angle_offset_deg = -1.0f * (calib_params.steering_angle_offset * 180.0f / (float)M_PI);
    result->steering_angle_gain = calib_params.steering_angle_gain;
    result->steering_gain_confidence = (added > 20) ? 0.92f : 0.70f;
    result->steering_gain_uncertainty = 0.02f;
    
    result->imu_accel_bias_x = calib_params.imu_accel_bias_x;
    result->imu_accel_bias_y = calib_params.imu_accel_bias_y;
    result->imu_gyro_bias_z = calib_params.imu_gyro_bias_z;
    result->imu_accel_scale_x = calib_params.imu_accel_scale_x;
    result->imu_accel_scale_y = calib_params.imu_accel_scale_y;
    
    /* 使用实际计算结果填充统计字段 */
    result->r_squared = (float)calib_manager.r_squared;
    result->avg_position_error = (float)stats.position_error_mean;
    result->max_position_error = (float)stats.position_error_max;
    result->avg_heading_error = (float)stats.heading_error_mean;
    /* confidence 基于 r_squared 与有效点比率综合计算 */
    float point_ratio = (count > 0) ? (float)added / (float)count : 0.0f;
    result->confidence_score = (float)(0.5 * (double)result->r_squared + 0.5 * point_ratio);
    if (result->confidence_score > 1.0f) result->confidence_score = 1.0f;
    
    result->calibration_time = time(NULL);
    struct tm *tm_info = localtime(&result->calibration_time);
    if (tm_info) {
        strftime(result->calibration_time_str, sizeof(result->calibration_time_str),
                 "%Y-%m-%dT%H:%M:%SZ", tm_info);
    }
    
    printf("✓ Calibration completed successfully\n");
    return 0;
}

/* ==================== JSON Output ==================== */

int output_json_result(const char *filename, const CalibrationResult_t *result,
                       const CalibrationStats_t *stats, const CalibrationPoint_t *points,
                       int point_count) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr, "ERROR: Cannot create output file: %s\n", filename);
        return -1;
    }
    
    fprintf(fp, "{\n");
    fprintf(fp, "  \"metadata\": {\n");
    fprintf(fp, "    \"format_version\": 1,\n");
    fprintf(fp, "    \"timestamp\": \"%s\",\n", result->calibration_time_str);
    fprintf(fp, "    \"tool\": \"offline_calibration_from_csv\",\n");
    fprintf(fp, "    \"calibration_points_count\": %d\n", stats->valid_points);
    fprintf(fp, "  },\n");
    
    fprintf(fp, "  \"parameters\": {\n");
    fprintf(fp, "    \"wheel_calibration_factor\": %.8f,\n", result->wheel_calibration_factor);
    fprintf(fp, "    \"wheel_factor_confidence\": %.4f,\n", result->wheel_factor_confidence);
    fprintf(fp, "    \"wheel_factor_uncertainty\": %.6f,\n", result->wheel_factor_uncertainty);
    fprintf(fp, "    \"steering_angle_offset_rad\": %.8f,\n", result->steering_angle_offset_rad);
    fprintf(fp, "    \"steering_angle_offset_deg\": %.4f,\n", result->steering_angle_offset_deg);
    fprintf(fp, "    \"steering_angle_gain\": %.8f,\n", result->steering_angle_gain);
    fprintf(fp, "    \"steering_gain_confidence\": %.4f,\n", result->steering_gain_confidence);
    fprintf(fp, "    \"steering_gain_uncertainty\": %.6f,\n", result->steering_gain_uncertainty);
    fprintf(fp, "    \"imu_accel_bias_x_mss\": %.6f,\n", result->imu_accel_bias_x);
    fprintf(fp, "    \"imu_accel_bias_y_mss\": %.6f,\n", result->imu_accel_bias_y);
    fprintf(fp, "    \"imu_gyro_bias_z_rads\": %.8f,\n", result->imu_gyro_bias_z);
    fprintf(fp, "    \"imu_accel_scale_x\": %.8f,\n", result->imu_accel_scale_x);
    fprintf(fp, "    \"imu_accel_scale_y\": %.8f\n", result->imu_accel_scale_y);
    fprintf(fp, "  },\n");
    
    fprintf(fp, "  \"statistics\": {\n");
    fprintf(fp, "    \"mean_position_error_m\": %.6f,\n", stats->position_error_mean);
    fprintf(fp, "    \"std_position_error_m\": %.6f,\n", stats->position_error_std);
    fprintf(fp, "    \"min_position_error_m\": %.6f,\n", stats->position_error_min);
    fprintf(fp, "    \"max_position_error_m\": %.6f,\n", stats->position_error_max);
    fprintf(fp, "    \"mean_heading_error_rad\": %.8f,\n", stats->heading_error_mean);
    fprintf(fp, "    \"std_heading_error_rad\": %.8f,\n", stats->heading_error_std);
    fprintf(fp, "    \"max_heading_error_rad\": %.8f\n", stats->heading_error_max);
    fprintf(fp, "  },\n");
    
    /* ===== ISO 8855坐标系验证统计 ===== */
    fprintf(fp, "  \"iso8855_heading_validation\": {\n");
    fprintf(fp, "    \"description\": \"ISO 8855 heading range validation ([-pi, pi])\",\n");
    fprintf(fp, "    \"large_error_count\": %u,\n", stats->heading_error_large_count);
    fprintf(fp, "    \"large_error_ratio\": %.4f,\n", stats->heading_error_large_ratio);
    fprintf(fp, "    \"large_error_threshold_rad\": %.4f,\n", (float)M_PI);
    fprintf(fp, "    \"status\": \"%s\"\n", 
            (stats->heading_error_large_ratio < 0.05) ? "PASS" :
            (stats->heading_error_large_ratio < 0.10) ? "WARNING" : "FAIL");
    fprintf(fp, "  },\n");
    
    fprintf(fp, "  \"rtk_quality_analysis\": {\n");
    fprintf(fp, "    \"quality_0_count\": %d,\n", stats->rtk_quality_distribution[0]);
    fprintf(fp, "    \"quality_1_count\": %d,\n", stats->rtk_quality_distribution[1]);
    fprintf(fp, "    \"quality_2_count\": %d,\n", stats->rtk_quality_distribution[2]);
    fprintf(fp, "    \"quality_3_count\": %d,\n", stats->rtk_quality_distribution[3]);
    fprintf(fp, "    \"quality_4_count\": %d,\n", stats->rtk_quality_distribution[4]);
    fprintf(fp, "    \"quality_5_count\": %d\n", stats->rtk_quality_distribution[5]);
    fprintf(fp, "  },\n");
    
    fprintf(fp, "  \"quality_assessment\": {\n");
    fprintf(fp, "    \"overall_confidence\": %.4f,\n", result->confidence_score);
    fprintf(fp, "    \"status\": \"%s\"\n", (result->confidence_score > 0.8) ? "Good" : 
                                         (result->confidence_score > 0.6) ? "Fair" : "Poor");
    fprintf(fp, "  }\n");
    fprintf(fp, "}\n");
    
    fclose(fp);
    printf("✓ Results written to: %s\n", filename);
    return 0;
}

/* ==================== Console Output ==================== */

void print_calibration_summary(const CalibrationResult_t *result, const CalibrationStats_t *stats) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║           OFFLINE CALIBRATION RESULTS SUMMARY              ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("CALIBRATION PARAMETERS:\n");
    printf("  Wheel Calibration Factor:  %.8f (conf: %.1f%%, unc: %.6f)\n",
           result->wheel_calibration_factor, result->wheel_factor_confidence * 100,
           result->wheel_factor_uncertainty);
    printf("  Steering Angle Offset:     %.4f° / %.8f rad\n",
           result->steering_angle_offset_deg, result->steering_angle_offset_rad);
    printf("  Steering Angle Gain:       %.8f (conf: %.1f%%, unc: %.6f)\n",
           result->steering_angle_gain, result->steering_gain_confidence * 100,
           result->steering_gain_uncertainty);
    printf("  IMU Accel Bias (X,Y):      %.6f, %.6f m/s²\n",
           result->imu_accel_bias_x, result->imu_accel_bias_y);
    printf("  IMU Gyro Bias (Z):         %.8f rad/s\n", result->imu_gyro_bias_z);
    printf("\n");
    
    printf("ERROR STATISTICS (from %d valid points):\n", stats->valid_points);
    printf("  Position Error:  Mean=%.4f m, Std=%.4f m, Min=%.4f m, Max=%.4f m\n",
           stats->position_error_mean, stats->position_error_std,
           stats->position_error_min, stats->position_error_max);
    printf("  Heading Error:   Mean=%.6f rad, Std=%.6f rad, Max=%.6f rad\n",
           stats->heading_error_mean, stats->heading_error_std,
           stats->heading_error_max);
    printf("\n");
    
    /* ===== ISO 8855 Heading角范围验证 ===== */
    printf("ISO 8855 HEADING VALIDATION ([-π, π] range check):\n");
    printf("  Large Error Count:  %u points (>π)\n", stats->heading_error_large_count);
    printf("  Large Error Ratio:  %.2f%%\n", stats->heading_error_large_ratio * 100);
    printf("  Status:             %s\n",
           (stats->heading_error_large_ratio < 0.05) ? "✓ PASS" :
           (stats->heading_error_large_ratio < 0.10) ? "⚠ WARNING (5-10%)" : 
           "✗ FAIL (>10%)");
    if (stats->heading_error_large_ratio >= 0.05) {
        printf("  NOTE: High proportion of large heading errors detected.\n");
        printf("        This may indicate RTK coordinate transformation issues.\n");
        printf("        Check rtk_coordinate_converter.py for correctness.\n");
    }
    printf("\n");
    
    printf("RTK QUALITY DISTRIBUTION:\n");
    printf("  Quality 0 (Fix): %d points\n", stats->rtk_quality_distribution[0]);
    printf("  Quality 1-5:     %d points\n", 
           stats->rtk_quality_distribution[1] + stats->rtk_quality_distribution[2] +
           stats->rtk_quality_distribution[3] + stats->rtk_quality_distribution[4] +
           stats->rtk_quality_distribution[5]);
    printf("\n");
    
    printf("OVERALL CONFIDENCE: %.1f%% - %s\n", result->confidence_score * 100,
           (result->confidence_score > 0.8) ? "✓ Good" :
           (result->confidence_score > 0.6) ? "⚠ Fair" : "✗ Poor");
    printf("\n");
}

