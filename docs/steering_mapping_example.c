/**
 * @file steering_mapping_example.c
 * @brief 转向映射表的使用示例
 * 
 * 演示如何：
 * 1. 从CSV文件加载映射表
 * 2. 初始化转向模型
 * 3. 查询映射表获取前轮转角
 * 4. 计算Ackermann航向角速率
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 假设这些头文件可用 */
#include "steering_model.h"

#define MAX_MAP_POINTS 51
#define CSV_BUFFER 1024

/**
 * @brief 从CSV文件加载映射表
 * 
 * CSV格式：
 * SteeringWheelAngle(rad),SteeringWheelAngle(deg),LeftWheelAngle(rad),LeftWheelAngle(deg),RightWheelAngle(rad),RightWheelAngle(deg)
 * 0.0,0.0,0.0,0.0,0.0,0.0
 * 0.05,2.87,0.00312,0.18,0.00312,0.18
 * ...
 * 
 * @param filename CSV文件路径
 * @param steering_wheel 输出：方向盘转角数组
 * @param left_wheel 输出：前左轮转角数组
 * @param right_wheel 输出：前右轮转角数组
 * @param max_points 最大点数
 * @return 实际读取的点数，-1表示失败
 */
static int load_mapping_from_csv(const char *filename,
                                 double *steering_wheel,
                                 double *left_wheel,
                                 double *right_wheel,
                                 int max_points) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("Error: Cannot open file %s\n", filename);
        return -1;
    }
    
    char buffer[CSV_BUFFER];
    int point_count = 0;
    
    /* 跳过CSV头 */
    if (fgets(buffer, sizeof(buffer), fp) == NULL) {
        printf("Error: CSV file is empty\n");
        fclose(fp);
        return -1;
    }
    
    /* 读取数据行 */
    while (fgets(buffer, sizeof(buffer), fp) && point_count < max_points) {
        double sw, sw_deg, lw, lw_deg, rw, rw_deg;
        
        /* 解析CSV行 */
        if (sscanf(buffer, "%lf,%lf,%lf,%lf,%lf,%lf",
                   &sw, &sw_deg, &lw, &lw_deg, &rw, &rw_deg) == 6) {
            steering_wheel[point_count] = sw;
            left_wheel[point_count] = lw;
            right_wheel[point_count] = rw;
            point_count++;
        }
    }
    
    fclose(fp);
    
    if (point_count < 2) {
        printf("Error: Need at least 2 points in CSV file\n");
        return -1;
    }
    
    printf("Loaded %d mapping points from %s\n", point_count, filename);
    return point_count;
}

/**
 * @brief 演示转向映射表的使用
 */
int main(int argc, char *argv[]) {
    const char *csv_file = "steering_mapping.csv";
    
    /* 如果提供了参数，使用第一个参数作为CSV文件 */
    if (argc > 1) {
        csv_file = argv[1];
    }
    
    printf("=== Steering Mapping Table Example ===\n\n");
    
    /* 第1步：加载映射表 */
    printf("Step 1: Loading steering mapping table from CSV\n");
    
    double steering_wheel[MAX_MAP_POINTS];
    double left_wheel[MAX_MAP_POINTS];
    double right_wheel[MAX_MAP_POINTS];
    
    int num_points = load_mapping_from_csv(csv_file, 
                                          steering_wheel,
                                          left_wheel,
                                          right_wheel,
                                          MAX_MAP_POINTS);
    if (num_points <= 0) {
        printf("Error: Failed to load mapping table\n");
        return -1;
    }
    
    /* 第2步：初始化转向模型 */
    printf("\nStep 2: Initializing steering model\n");
    
    /* 初始化转向配置 */
    SteeringConfig_t steering_config = {
        .steering_ratio = 16.0f,
        .max_steering_wheel_angle = 3.14159f,
        .max_front_wheel_angle = 0.628f,
        .wheelbase = 2.7f,
        .steering_angle_offset = 0.0f,
        .steering_angle_gain = 1.0f,
        .enable_steering_filter = true,
        .steering_filter_alpha = 0.15f
    };
    
    if (steering_config_init(&steering_config) != 0) {
        printf("Error: Failed to initialize steering model\n");
        return -1;
    }
    
    printf("Steering model initialized with wheelbase: %.2f m\n", steering_config.wheelbase);
    
    /* 第3步：初始化映射表 */
    printf("\nStep 3: Setting up steering mapping table\n");
    
    if (steering_mapping_init(steering_wheel, left_wheel, right_wheel, num_points) != 0) {
        printf("Error: Failed to initialize mapping table\n");
        return -1;
    }
    
    printf("Mapping table initialized with %d points\n", num_points);
    
    /* 第4步：测试映射表查询 */
    printf("\nStep 4: Testing mapping table queries\n");
    printf("%-15s %-15s %-15s %-15s %-15s\n",
           "SteeringWheel", "LeftWheel", "RightWheel", "Mean", "Difference");
    printf("-%.14s-%.14s-%.14s-%.14s-%.14s\n",
           "-------------", "-------------", "-------------", "-------------", "-------------");
    
    double test_angles[] = {-0.5, -0.2, 0.0, 0.2, 0.5};
    for (int i = 0; i < 5; i++) {
        double sw_angle = test_angles[i];
        double left_angle = 0.0, right_angle = 0.0;
        
        /* 查询映射表 */
        if (steering_query_mapping(sw_angle, &left_angle, &right_angle) == 0) {
            double mean = (left_angle + right_angle) / 2.0;
            double diff = fabs(left_angle - right_angle);
            
            printf("%-15.4f %-15.4f %-15.4f %-15.4f %-15.4f\n",
                   sw_angle, left_angle, right_angle, mean, diff);
        } else {
            printf("%-15.4f (query failed)\n", sw_angle);
        }
    }
    
    /* 第5步：计算Ackermann航向角速率 */
    printf("\nStep 5: Computing Ackermann yaw rates\n");
    
    double vehicle_speed = 10.0;  /* m/s */
    printf("Vehicle speed: %.1f m/s\n", vehicle_speed);
    printf("%-15s %-20s %-20s\n", "SteeringWheel", "SimpleModel", "AckermannModel");
    printf("-%.14s-%.19s-%.19s\n",
           "-------------", "-------------------", "-------------------");
    
    for (int i = 0; i < 5; i++) {
        double sw_angle = test_angles[i];
        double left_angle = 0.0, right_angle = 0.0;
        
        /* 查询映射表 */
        steering_query_mapping(sw_angle, &left_angle, &right_angle);
        
        /* 简单模型：yaw_rate = v * tan(delta) / L */
        double mean_angle = (left_angle + right_angle) / 2.0;
        double simple_yaw_rate = vehicle_speed * tan(mean_angle) / steering_config.wheelbase;
        
        /* Ackermann模型：yaw_rate = v * (tan(delta_left) + tan(delta_right)) / (2*L) */
        double ackermann_yaw_rate = vehicle_speed * (tan(left_angle) + tan(right_angle)) / 
                                    (2.0 * steering_config.wheelbase);
        
        printf("%-15.4f %-20.6f %-20.6f\n",
               sw_angle, simple_yaw_rate, ackermann_yaw_rate);
    }
    
    /* 第6步：演示动态更新 */
    printf("\nStep 6: Dynamic steering angle updates\n");
    
    double steering_angles[] = {0.0, 0.1, 0.2, 0.15, 0.0};
    uint32_t timestamps[] = {0, 100, 200, 300, 400};
    
    printf("%-10s %-15s %-15s %-15s %-15s\n",
           "Time(ms)", "SteeringWheel", "Filtered", "LeftWheel", "RightWheel");
    printf("-%.9s-%.14s-%.14s-%.14s-%.14s\n",
           "---------", "-------------", "-------------", "-------------", "-------------");
    
    for (int i = 0; i < 5; i++) {
        double filtered_angle = steering_model_update(steering_angles[i], timestamps[i]);
        double left_w = 0.0, right_w = 0.0;
        
        steering_query_mapping(steering_angles[i], &left_w, &right_w);
        steering_get_wheel_angles(&left_w, &right_w);
        
        printf("%-10u %-15.4f %-15.4f %-15.4f %-15.4f\n",
               timestamps[i], steering_angles[i], filtered_angle, left_w, right_w);
    }
    
    /* 清理 */
    printf("\nStep 7: Cleaning up\n");
    steering_mapping_cleanup();
    printf("Mapping table cleaned up\n");
    
    printf("\n=== Example completed successfully ===\n");
    return 0;
}
