/**
 * @file generate_calibration_csv.c
 * @brief 生成离线标定用的示例CSV数据
 * 
 * 用于创建包含里程计和RTK数据的CSV文件，模拟实际的标定场景
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define PI 3.141592653589793
#define DEG_TO_RAD(x) ((x) * PI / 180.0)

/**
 * @brief 生成J型停车轨迹的标定数据
 * 
 * 场景：
 * 1. 直行100米 (0-100s)
 * 2. 转向并后退30米 (100-130s)  
 * 3. 停止 (130-150s)
 */
int main(int argc, char *argv[]) {
    const char *output_file = (argc > 1) ? argv[1] : "calibration_example.csv";
    
    printf("生成离线标定示例数据: %s\n", output_file);
    
    FILE *file = fopen(output_file, "w");
    if (!file) {
        fprintf(stderr, "无法打开文件: %s\n", output_file);
        return 1;
    }
    
    /* 写入CSV头 */
    fprintf(file, "timestamp,odo_x,odo_y,odo_theta,odo_v,odo_delta,"
                  "rtk_x,rtk_y,rtk_heading,rtk_quality\n");
    
    /* 参数 */
    uint32_t start_time = 1000;  /* 开始时间 */
    uint32_t dt = 50;            /* 时间步长 (ms) */
    uint32_t total_steps = 3000; /* 总步数 */
    
    /* 标定参数（模拟的） */
    double wheel_cal = 0.98;     /* 轮速计校准因子 */
    double steering_offset = 0.02; /* 转向角零偏 */
    
    double x_odo = 0.0;
    double y_odo = 0.0;
    double theta_odo = 0.0;
    
    printf("生成 %u 步的数据...\n", total_steps);
    
    for (uint32_t step = 0; step < total_steps; step++) {
        uint32_t timestamp = start_time + step * dt;
        
        double v_cmd = 0.0;      /* 命令速度 */
        double steering_cmd = 0.0; /* 命令转向角 */
        
        /* 阶段1：直行100米 (0-2000步，100秒) */
        if (step < 2000) {
            v_cmd = 1.0;
            steering_cmd = 0.0;
        }
        /* 阶段2：转向并后退 (2000-2600步) */
        else if (step < 2600) {
            v_cmd = 0.5;
            steering_cmd = 0.5;  /* 左转 */
        }
        /* 阶段3：停止 */
        else {
            v_cmd = 0.0;
            steering_cmd = 0.0;
        }
        
        /* 里程计（带随机噪声） */
        double v_odo = v_cmd * wheel_cal; /* 轮速计测量（有校准因子误差） */
        double steering_odo = steering_cmd + steering_offset; /* 转向角（有零偏） */
        
        /* 更新里程计状态 */
        double dt_sec = dt / 1000.0;
        double dx = v_odo * cos(theta_odo) * dt_sec;
        double dy = v_odo * sin(theta_odo) * dt_sec;
        
        x_odo += dx;
        y_odo += dy;
        
        if (fabs(v_odo) > 0.01) {
            double dtheta = v_odo * tan(steering_odo) / 2.7 * dt_sec; /* 轴距2.7m */
            theta_odo += dtheta;
        }
        
        /* RTK真值（更精确） */
        double x_rtk = 0.0;
        double y_rtk = 0.0;
        double theta_rtk = 0.0;
        
        /* RTK生成真实轨迹（与命令一致，不含校准误差） */
        if (step < 2000) {
            x_rtk = step * 1.0 * dt_sec; /* 精确的速度 */
            y_rtk = 0.0;
            theta_rtk = 0.0;
        } else if (step < 2600) {
            double progress = (step - 2000) / 600.0;
            x_rtk = 100.0 - progress * 30.0 * sin(progress * PI/2);
            y_rtk = progress * 30.0 * (1 - cos(progress * PI/2));
            theta_rtk = progress * PI/2;
        } else {
            x_rtk = 100.0 - 21.2;
            y_rtk = 30.0;
            theta_rtk = PI/2;
        }
        
        /* 添加小的RTK随机噪声 */
        double rtk_noise_pos = (rand() % 100) / 50000.0;  /* ±1mm */
        double rtk_noise_heading = (rand() % 100) / 10000.0; /* ±0.01rad */
        
        x_rtk += rtk_noise_pos;
        y_rtk += rtk_noise_pos;
        theta_rtk += rtk_noise_heading;
        
        /* RTK质量：根据位置随机设置 */
        uint8_t rtk_quality = 4;  /* 大多数时间高质量 */
        if (step % 500 < 50) {
            rtk_quality = 2;  /* 偶尔质量较低 */
        }
        
        /* 写入CSV行 */
        fprintf(file, "%u,%.6f,%.6f,%.6f,%.4f,%.6f,"
                     "%.6f,%.6f,%.6f,%u\n",
                timestamp,
                x_odo, y_odo, theta_odo, v_odo, steering_odo,
                x_rtk, y_rtk, theta_rtk, rtk_quality);
    }
    
    fclose(file);
    
    printf("✓ 成功生成 %u 行数据\n", total_steps);
    printf("✓ 文件位置: %s\n", output_file);
    printf("\n生成参数配置:\n");
    printf("  轮速计校准因子: %.4f (实际值应为1.0)\n", wheel_cal);
    printf("  转向角零偏: %.4f rad (%.2f°)\n", steering_offset, steering_offset * 180 / PI);
    printf("  预期标定精度: <0.3m 位置误差\n");
    
    return 0;
}
