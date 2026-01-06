/**
 * @file main_offline_calibration.c
 * @brief 离线标定主程序
 * 
 * 使用 vehicle_localization.csv 和 rtk_converted.csv 进行离线标定
 * 输出 JSON 格式的标定结果
 */

#include "offline_calibration_csv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_POINTS 10000

int main(int argc, char *argv[]) {
    if (argc < 4 || argc > 5) {
        fprintf(stderr, "Usage: %s <odometry_csv> <rtk_csv> <output_json> [--auto-shift]\n", argv[0]);
        fprintf(stderr, "\nExample:\n");
        fprintf(stderr, "  %s vehicle_localization.csv rtk_converted.csv calibration_result.json --auto-shift\n\n", argv[0]);
        return 1;
    }

    const char *odo_file = argv[1];
    const char *rtk_file = argv[2];
    const char *json_file = argv[3];
    int do_auto_shift = 0;
    if (argc == 5 && strcmp(argv[4], "--auto-shift") == 0) {
        do_auto_shift = 1;
    }
    
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║     离线标定工具 (Offline Calibration Tool) v1.0               ║\n");
    printf("║     Vehicle Localization System - Phase 2                     ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
    
    // 分配内存
    OdometryMeasurement_t *odo_data = malloc(sizeof(OdometryMeasurement_t) * MAX_POINTS);
    RTKMeasurement_t *rtk_data = malloc(sizeof(RTKMeasurement_t) * MAX_POINTS);
    CalibrationPoint_t *calib_points = malloc(sizeof(CalibrationPoint_t) * MAX_POINTS);
    
    if (!odo_data || !rtk_data || !calib_points) {
        fprintf(stderr, "ERROR: Memory allocation failed\n");
        return 1;
    }
    
    printf("📂 输入文件 (Input Files):\n");
    printf("  ├─ 里程计数据: %s\n", odo_file);
    printf("  └─ RTK 数据:   %s\n\n", rtk_file);
    
    // 第1步：加载 CSV 文件
    printf("📖 第1步：读取数据文件 (Step 1: Loading CSV files)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    int odo_count = load_odometry_csv(odo_file, odo_data, MAX_POINTS);
    if (odo_count < 0) {
        fprintf(stderr, "ERROR: Failed to load odometry data\n");
        free(odo_data);
        free(rtk_data);
        free(calib_points);
        return 1;
    }
    
    int rtk_count = load_rtk_csv(rtk_file, rtk_data, MAX_POINTS);
    if (rtk_count < 0) {
        fprintf(stderr, "ERROR: Failed to load RTK data\n");
        free(odo_data);
        free(rtk_data);
        free(calib_points);
        return 1;
    }
    
    // 第2步：数据对齐
    printf("\n📍 第2步：数据对齐与同步 (Step 2: Data alignment)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    /* 自动搜索时间偏移（可选）: 粗搜索 -> 细搜索 */
    if (do_auto_shift) {
        printf("🔎 自动时间偏移搜索已启用 (Auto time-shift search)\n");
        double best_shift = 0.0;
        double best_score = INFINITY;

        /* 1) 粗略搜索：-2.0s 到 +2.0s，步长 0.1s */
        for (double s = -2.0; s <= 2.0; s += 0.1) {
            /* 创建临时里程计副本并应用偏移（秒->ms） */
            OdometryMeasurement_t *tmp_odo = malloc(sizeof(OdometryMeasurement_t) * odo_count);
            if (!tmp_odo) break;
            for (int i = 0; i < odo_count; ++i) {
                tmp_odo[i] = odo_data[i];
                int64_t shift_ms = (int64_t)llround(s * 1000.0);
                tmp_odo[i].timestamp_ms = (uint64_t)((int64_t)tmp_odo[i].timestamp_ms + shift_ms);
            }

            CalibrationPoint_t *tmp_points = malloc(sizeof(CalibrationPoint_t) * MAX_POINTS);
            if (!tmp_points) { free(tmp_odo); break; }
            int matched = align_measurements(tmp_odo, odo_count, rtk_data, rtk_count, tmp_points, MAX_POINTS);
            double score = INFINITY;
            if (matched >= 10) {
                CalibrationStats_t tmp_stats;
                if (compute_calibration_stats(tmp_points, matched, &tmp_stats) == 0) {
                    score = tmp_stats.position_error_mean;
                }
            }
            if (score < best_score) {
                best_score = score;
                best_shift = s;
            }

            free(tmp_odo);
            free(tmp_points);
        }

        /* 2) 细化搜索：在 best_shift +/-0.2s 内以 0.01s 步长搜索 */
        double fine_start = best_shift - 0.2;
        double fine_end = best_shift + 0.2;
        for (double s = fine_start; s <= fine_end; s += 0.01) {
            OdometryMeasurement_t *tmp_odo = malloc(sizeof(OdometryMeasurement_t) * odo_count);
            if (!tmp_odo) break;
            for (int i = 0; i < odo_count; ++i) {
                tmp_odo[i] = odo_data[i];
                int64_t shift_ms = (int64_t)llround(s * 1000.0);
                tmp_odo[i].timestamp_ms = (uint64_t)((int64_t)tmp_odo[i].timestamp_ms + shift_ms);
            }

            CalibrationPoint_t *tmp_points = malloc(sizeof(CalibrationPoint_t) * MAX_POINTS);
            if (!tmp_points) { free(tmp_odo); break; }
            int matched = align_measurements(tmp_odo, odo_count, rtk_data, rtk_count, tmp_points, MAX_POINTS);
            double score = INFINITY;
            if (matched >= 10) {
                CalibrationStats_t tmp_stats;
                if (compute_calibration_stats(tmp_points, matched, &tmp_stats) == 0) {
                    score = tmp_stats.position_error_mean;
                }
            }
            if (score < best_score) {
                best_score = score;
                best_shift = s;
            }

            free(tmp_odo);
            free(tmp_points);
        }

        printf("最佳时间偏移: %.3f s, 平均位置误差: %.6f m\n", best_shift, best_score == INFINITY ? -1.0 : best_score);
        /* 将最优偏移应用到原始里程计数据（就地修改） */
        int64_t best_shift_ms = (int64_t)llround(best_shift * 1000.0);
        for (int i = 0; i < odo_count; ++i) {
            odo_data[i].timestamp_ms = (uint64_t)((int64_t)odo_data[i].timestamp_ms + best_shift_ms);
        }
    }

    int aligned_count = align_measurements(odo_data, odo_count, rtk_data, rtk_count,
                                           calib_points, MAX_POINTS);
    if (aligned_count < 10) {
        fprintf(stderr, "ERROR: Insufficient aligned points for calibration\n");
        free(odo_data);
        free(rtk_data);
        free(calib_points);
        return 1;
    }
    
    // 第3步：计算统计信息
    printf("\n📊 第3步：统计分析 (Step 3: Statistical analysis)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    CalibrationStats_t stats;
    if (compute_calibration_stats(calib_points, aligned_count, &stats) != 0) {
        fprintf(stderr, "ERROR: Failed to compute statistics\n");
        free(odo_data);
        free(rtk_data);
        free(calib_points);
        return 1;
    }
    
    printf("✓ Position error statistics computed\n");
    printf("✓ Heading error statistics computed\n");
    
    // 第4步：执行标定
    printf("\n🔧 第4步：执行标定计算 (Step 4: Performing calibration)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    CalibrationResult_t result;
    if (perform_calibration(calib_points, aligned_count, &result) != 0) {
        fprintf(stderr, "ERROR: Calibration failed\n");
        free(odo_data);
        free(rtk_data);
        free(calib_points);
        return 1;
    }
    
    // 第5步：输出结果
    printf("\n💾 第5步：生成输出文件 (Step 5: Generating output)\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    if (output_json_result(json_file, &result, &stats, calib_points, aligned_count) != 0) {
        fprintf(stderr, "ERROR: Failed to write JSON output\n");
        free(odo_data);
        free(rtk_data);
        free(calib_points);
        return 1;
    }
    
    // 打印摘要
    print_calibration_summary(&result, &stats);
    
    printf("📁 输出文件 (Output File):\n");
    printf("  └─ %s\n\n", json_file);
    
    printf("✅ 离线标定完成！(Offline calibration completed successfully!)\n\n");
    
    // 清理
    free(odo_data);
    free(rtk_data);
    free(calib_points);
    
    return 0;
}
