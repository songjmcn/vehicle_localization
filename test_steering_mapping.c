/**
 * @file test_steering_mapping.c
 * @brief 转向角映射表完整功能测试
 */

#include "src/steering_model.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define TEST_POINTS 11
#define CSV_FILE "steering_mapping.csv"

/* 定义PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 从CSV文件加载映射表
 */
int load_mapping_from_csv(const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("❌ 无法打开文件: %s\n", filename);
        return -1;
    }
    
    /* 分配内存：初始申请21个点 */
    int max_points = 100;
    double *steering_wheel = (double *)malloc(max_points * sizeof(double));
    double *left_wheel = (double *)malloc(max_points * sizeof(double));
    double *right_wheel = (double *)malloc(max_points * sizeof(double));
    
    if (!steering_wheel || !left_wheel || !right_wheel) {
        printf("❌ 内存分配失败\n");
        fclose(fp);
        return -1;
    }
    
    /* 跳过头行 */
    char line[256];
    if (fgets(line, sizeof(line), fp) == NULL) {
        printf("❌ 读取CSV头部失败\n");
        fclose(fp);
        return -1;
    }
    
    /* 读取数据行 */
    int num_points = 0;
    while (fgets(line, sizeof(line), fp) && num_points < max_points) {
        double sw_rad, sw_deg, left_rad, left_deg, right_rad, right_deg;
        
        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf",
                  &sw_rad, &sw_deg, &left_rad, &left_deg, &right_rad, &right_deg) == 6) {
            steering_wheel[num_points] = sw_rad;
            left_wheel[num_points] = left_rad;
            right_wheel[num_points] = right_rad;
            num_points++;
        }
    }
    
    fclose(fp);
    
    if (num_points == 0) {
        printf("❌ CSV文件中没有数据\n");
        return -1;
    }
    
    /* 初始化映射表 */
    int result = steering_mapping_init(steering_wheel, left_wheel, right_wheel, num_points);
    
    free(steering_wheel);
    free(left_wheel);
    free(right_wheel);
    
    return result;
}

/**
 * @brief 测试映射表查询
 */
void test_mapping_query() {
    printf("\n=== 测试1: 映射表查询 ===\n");
    
    double test_angles[] = {-M_PI, -M_PI/2, -M_PI/4, 0, M_PI/4, M_PI/2, M_PI};
    
    printf("方向盘(rad)  方向盘(deg)  左轮(rad)   左轮(deg)   右轮(rad)   右轮(deg)   差异(rad)\n");
    printf("%-12.4f %-12.2f %-11.4f %-11.2f %-11.4f %-11.2f %-11.4f\n",
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    for (int i = 0; i < 7; i++) {
        double sw = test_angles[i];
        double left, right, mean;
        
        /* 更新转向角 */
        steering_model_update(sw, 0);
        
        /* 获取前轮转角 */
        int ret = steering_get_wheel_angles(&left, &right, &mean);
        if (ret != 0) {
            printf("❌ 查询失败: steering_wheel = %.4f\n", sw);
            continue;
        }
        
        double diff = fabs(left - right);
        printf("%-12.4f %-12.2f %-11.4f %-11.2f %-11.4f %-11.2f %-11.4f\n",
               sw, sw * 180.0 / M_PI, left, left * 180.0 / M_PI, 
               right, right * 180.0 / M_PI, diff);
    }
    
    printf("✅ 映射表查询测试完成\n");
}

/**
 * @brief 测试平均转向角计算
 */
void test_mean_steering_angle() {
    printf("\n=== 测试2: 平均转向角计算 ===\n");
    
    double test_angles[] = {-M_PI/2, -M_PI/4, 0, M_PI/4, M_PI/2};
    
    printf("方向盘(rad)  平均转角(rad) 平均转角(deg)\n");
    
    for (int i = 0; i < 5; i++) {
        double sw = test_angles[i];
        
        /* 更新转向角 */
        steering_model_update(sw, 0);
        
        /* 获取平均转向角 */
        double left, right, mean_angle;
        steering_get_wheel_angles(&left, &right, &mean_angle);
        
        printf("%-12.4f %-14.4f %-14.2f\n",
               sw, mean_angle, mean_angle * 180.0 / M_PI);
    }
    
    printf("✅ 平均转向角计算测试完成\n");
}

/**
 * @brief 测试Ackermann航向角速率计算
 */
void test_yaw_rate() {
    printf("\n=== 测试3: Ackermann航向角速率计算 ===\n");
    
    double velocities[] = {1.0, 5.0, 10.0, 20.0};
    double test_angles[] = {-M_PI/4, 0, M_PI/4};
    double wheelbase = 2.7;
    
    for (int v_idx = 0; v_idx < 4; v_idx++) {
        double velocity = velocities[v_idx];
        printf("\n速度: %.2f m/s\n", velocity);
        printf("方向盘(deg)  航向角速率(rad/s)  航向角速率(deg/s)\n");
        
        for (int a_idx = 0; a_idx < 3; a_idx++) {
            double sw = test_angles[a_idx];
            double yaw_rate = steering_calculate_yaw_rate_ackermann(velocity, sw, wheelbase);
            
            printf("%-12.2f %-18.6f %-18.2f\n",
                   sw * 180.0 / M_PI, yaw_rate, yaw_rate * 180.0 / M_PI);
        }
    }
    
    printf("✅ 航向角速率计算测试完成\n");
}

/**
 * @brief 测试与简单模型的对比
 */
void test_comparison() {
    printf("\n=== 测试4: 与简单线性模型的对比 ===\n");
    
    double velocity = 10.0;
    double wheelbase = 2.7;
    double steering_ratio = 16.0;
    
    printf("对比简单模型 vs Ackermann模型的航向角速率差异\n\n");
    printf("方向盘(deg)  简单模型(rad/s)  Ackermann(rad/s)  误差比例(%%)\n");
    
    for (double sw_deg = -30; sw_deg <= 30; sw_deg += 10) {
        double sw_rad = sw_deg * M_PI / 180.0;
        
        /* 简单模型：使用齿比转换 */
        double simple_delta = sw_rad / steering_ratio;
        double simple_yaw = velocity * tan(simple_delta) / wheelbase;
        
        /* Ackermann模型 */
        double ackermann_yaw = steering_calculate_yaw_rate_ackermann(velocity, sw_rad, wheelbase);
        
        double diff_percent = 0;
        if (fabs(simple_yaw) > 1e-6) {
            diff_percent = 100.0 * fabs(ackermann_yaw - simple_yaw) / fabs(simple_yaw);
        }
        
        printf("%-12.1f %-16.6f %-17.6f %-15.2f\n",
               sw_deg, simple_yaw, ackermann_yaw, diff_percent);
    }
    
    printf("✅ 模型对比测试完成\n");
}

/**
 * @brief 主测试函数
 */
int main(void) {
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║      转向角映射表完整功能测试                             ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    
    /* 检查映射表文件 */
    FILE *fp = fopen(CSV_FILE, "r");
    if (!fp) {
        printf("\n❌ 错误: 找不到映射表文件 '%s'\n", CSV_FILE);
        printf("   请先运行: ./build/bin/generate_steering_mapping\n\n");
        return 1;
    }
    fclose(fp);
    
    printf("\n📂 映射表文件: %s\n", CSV_FILE);
    
    /* 加载映射表 */
    printf("\n🔄 加载映射表...\n");
    if (load_mapping_from_csv(CSV_FILE) != 0) {
        printf("❌ 映射表加载失败\n");
        return 1;
    }
    printf("✅ 映射表加载成功\n");
    
    /* 执行各项测试 */
    test_mapping_query();
    test_mean_steering_angle();
    test_yaw_rate();
    test_comparison();
    
    /* 清理资源 */
    steering_mapping_cleanup();
    
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║              所有测试完成 ✅                             ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    return 0;
}
