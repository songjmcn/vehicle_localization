/**
 * @file offline_calibration.c
 * @brief 超简化的离线参数标定工具 - 直接处理CSV数据流
 * 
 * 功能：
 * - 读取CSV文件中的里程计和RTK数据
 * - 计算误差统计和标定参数
 * - 输出结果文件
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PI 3.141592653589793
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)

#define MIN_RTK_QUALITY 2
#define MAX_BUFFER 1024

typedef struct {
    uint32_t count;
    double sum_pos_error;
    double max_pos_error;
    double min_pos_error;
    double sum_heading_error;
    double max_heading_error;
    double sum_velocity;
    double sum_steering;
    double pos_bias_x;
    double pos_bias_y;
    double heading_bias;
} CalibrationStats_t;

/**
 * @brief 标准化角度到 [-pi, pi]
 */
static double normalize_angle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

/**
 * @brief 处理单行CSV数据
 */
static int process_csv_line(const char *line, CalibrationStats_t *stats) {
    uint32_t timestamp;
    double odo_x, odo_y, odo_theta, odo_v, odo_delta;
    double rtk_x, rtk_y, rtk_heading;
    uint8_t rtk_quality;
    
    int ret = sscanf(line, "%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%hhu",
                     &timestamp,
                     &odo_x, &odo_y, &odo_theta, &odo_v, &odo_delta,
                     &rtk_x, &rtk_y, &rtk_heading, &rtk_quality);
    
    if (ret != 10) {
        fprintf(stderr, "行格式错误\n");
        return -1;
    }
    
    /* 检查RTK质量 */
    if (rtk_quality < MIN_RTK_QUALITY) {
        return 0;  /* 跳过但不算错误 */
    }
    
    /* 计算误差 */
    double dx = rtk_x - odo_x;
    double dy = rtk_y - odo_y;
    double pos_error = sqrt(dx * dx + dy * dy);
    
    double heading_error = normalize_angle(rtk_heading - odo_theta);
    
    /* 更新统计 */
    stats->count++;
    stats->sum_pos_error += pos_error;
    stats->max_pos_error = fmax(stats->max_pos_error, pos_error);
    stats->min_pos_error = fmin(stats->min_pos_error, pos_error);
    
    stats->sum_heading_error += fabs(heading_error);
    stats->max_heading_error = fmax(stats->max_heading_error, fabs(heading_error));
    
    stats->sum_velocity += odo_v;
    stats->sum_steering += fabs(odo_delta);
    
    stats->pos_bias_x += dx;
    stats->pos_bias_y += dy;
    stats->heading_bias += heading_error;
    
    return 0;
}

/**
 * @brief 读取CSV并统计
 */
static int process_csv_file(const char *filename, CalibrationStats_t *stats,
                            uint32_t *total_rows, uint32_t *skipped_rows) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "无法打开文件: %s\n", filename);
        return -1;
    }
    
    printf("正在处理CSV文件: %s\n", filename);
    
    char line[MAX_BUFFER];
    uint32_t line_num = 0;
    uint32_t data_rows = 0;
    
    /* 跳过标题行 */
    if (fgets(line, sizeof(line), file) == NULL) {
        fprintf(stderr, "文件为空\n");
        fclose(file);
        return -1;
    }
    line_num++;
    
    /* 处理数据行 */
    while (fgets(line, sizeof(line), file) != NULL) {
        line_num++;
        
        if (process_csv_line(line, stats) == 0) {
            if (stats->count > data_rows) {
                data_rows = stats->count;
            }
        } else {
            (*skipped_rows)++;
        }
    }
    
    fclose(file);
    
    *total_rows = line_num - 1;
    *skipped_rows = (line_num - 1) - stats->count;
    
    printf("✓ 处理完成: %u有效行, %u低质量行\n", stats->count, *skipped_rows);
    
    return (stats->count >= 5) ? 0 : -1;
}

/**
 * @brief 计算平均值和最终参数
 */
static void compute_final_params(CalibrationStats_t *stats) {
    if (stats->count == 0) return;
    
    stats->pos_bias_x /= stats->count;
    stats->pos_bias_y /= stats->count;
    stats->heading_bias /= stats->count;
    
    double avg_pos_error = stats->sum_pos_error / stats->count;
    double avg_heading_error = stats->sum_heading_error / stats->count;
    
    printf("\n═══════════════════════════════════════════════════════\n");
    printf("             📊 标定结果统计\n");
    printf("═══════════════════════════════════════════════════════\n\n");
    
    printf("有效标定点:          %u\n", stats->count);
    printf("平均位置误差:        %.4f m\n", avg_pos_error);
    printf("最大位置误差:        %.4f m\n", stats->max_pos_error);
    printf("最小位置误差:        %.4f m\n", stats->min_pos_error);
    printf("平均航向角误差:      %.4f rad (%.2f°)\n",
           avg_heading_error, RAD_TO_DEG(avg_heading_error));
    printf("最大航向角误差:      %.4f rad (%.2f°)\n",
           stats->max_heading_error, RAD_TO_DEG(stats->max_heading_error));
    printf("平均速度:            %.2f m/s\n", stats->sum_velocity / stats->count);
    printf("平均转向角:          %.4f rad (%.2f°)\n",
           stats->sum_steering / stats->count,
           RAD_TO_DEG(stats->sum_steering / stats->count));
    
    printf("\n标定参数:\n");
    printf("  位置X偏差:         %.6f m\n", stats->pos_bias_x);
    printf("  位置Y偏差:         %.6f m\n", stats->pos_bias_y);
    printf("  航向角偏差:        %.6f rad (%.2f°)\n",
           stats->heading_bias, RAD_TO_DEG(stats->heading_bias));
    
    printf("═══════════════════════════════════════════════════════\n");
}

/**
 * @brief 保存结果
 */
static int save_results(const char *prefix, CalibrationStats_t *stats,
                        uint32_t total_rows, uint32_t skipped_rows) {
    char filename[512];
    
    /* 保存参数文件 */
    snprintf(filename, sizeof(filename), "%s_params.txt", prefix);
    FILE *file = fopen(filename, "w");
    if (file) {
        fprintf(file, "# RTK离线标定参数\n");
        fprintf(file, "# 数据点数: %u\n", stats->count);
        fprintf(file, "# 跳过行数: %u\n\n", skipped_rows);
        fprintf(file, "position_bias_x = %.8f\n", stats->pos_bias_x);
        fprintf(file, "position_bias_y = %.8f\n", stats->pos_bias_y);
        fprintf(file, "heading_bias = %.8f\n", stats->heading_bias);
        fprintf(file, "wheel_calibration_factor = 1.0\n");
        fprintf(file, "steering_angle_offset = 0.0\n");
        fprintf(file, "steering_angle_gain = 1.0\n");
        fclose(file);
        printf("\n✓ 参数已保存到: %s\n", filename);
    }
    
    /* 保存报告文件 */
    snprintf(filename, sizeof(filename), "%s_report.txt", prefix);
    file = fopen(filename, "w");
    if (file) {
        fprintf(file, "# RTK离线标定报告\n\n");
        fprintf(file, "## 数据统计\n");
        fprintf(file, "- 总行数: %u\n", total_rows);
        fprintf(file, "- 有效点数: %u\n", stats->count);
        fprintf(file, "- 跳过行数: %u\n\n", skipped_rows);
        
        fprintf(file, "## 位置误差\n");
        fprintf(file, "- 平均: %.4f m\n", stats->sum_pos_error / stats->count);
        fprintf(file, "- 最大: %.4f m\n", stats->max_pos_error);
        fprintf(file, "- 最小: %.4f m\n\n", stats->min_pos_error);
        
        fprintf(file, "## 航向角误差 (rad)\n");
        fprintf(file, "- 平均: %.6f (%.2f°)\n",
                stats->sum_heading_error / stats->count,
                RAD_TO_DEG(stats->sum_heading_error / stats->count));
        fprintf(file, "- 最大: %.6f (%.2f°)\n\n",
                stats->max_heading_error,
                RAD_TO_DEG(stats->max_heading_error));
        
        fprintf(file, "## 标定参数\n");
        fprintf(file, "- 位置X偏差: %.8f m\n", stats->pos_bias_x);
        fprintf(file, "- 位置Y偏差: %.8f m\n", stats->pos_bias_y);
        fprintf(file, "- 航向角偏差: %.8f rad\n", stats->heading_bias);
        
        fclose(file);
        printf("✓ 报告已保存到: %s\n", filename);
    }
    
    return 0;
}

/* ============ 主程序 ============ */

int main(int argc, char *argv[]) {
    printf("\n╔══════════════════════════════════════════════════════╗\n");
    printf("║     离线RTK参数标定工具 - CSV数据处理版本            ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");
    
    if (argc < 2) {
        printf("用法: %s <input.csv> [output_prefix]\n", argv[0]);
        printf("例: %s calibration_data.csv result\n", argv[0]);
        return 1;
    }
    
    const char *input_file = argv[1];
    const char *output_prefix = (argc >= 3) ? argv[2] : "calibration_result";
    
    /* 初始化统计 */
    CalibrationStats_t stats;
    memset(&stats, 0, sizeof(stats));
    stats.min_pos_error = 1e9;
    
    uint32_t total_rows = 0, skipped_rows = 0;
    
    /* 处理CSV文件 */
    if (process_csv_file(input_file, &stats, &total_rows, &skipped_rows) != 0) {
        fprintf(stderr, "✗ 处理失败\n");
        return 1;
    }
    
    /* 计算最终参数 */
    compute_final_params(&stats);
    
    /* 保存结果 */
    if (save_results(output_prefix, &stats, total_rows, skipped_rows) != 0) {
        fprintf(stderr, "✗ 保存失败\n");
        return 1;
    }
    
    printf("\n✓ 标定完成！\n\n");
    return 0;
}
