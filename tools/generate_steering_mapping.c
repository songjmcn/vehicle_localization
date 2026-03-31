/**
 * @file generate_steering_mapping.c
 * @brief 生成转向映射表的工具程序
 * 
 * 生成方向盘转角到前左/右轮转角的映射表CSV文件
 * 支持多种转向特性曲线模式
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../src/macros.h"

#define MAX_MAPPING_POINTS 51
#define CSV_FILENAME "steering_mapping.csv"

/**
 * @brief 转向特性模式
 */
typedef enum {
    STEERING_LINEAR,      /* 线性特性 */
    STEERING_PROGRESSIVE, /* 渐进式特性（非线性） */
    STEERING_QUICK_RATIO, /* 快速齿比（转向灵敏） */
    STEERING_SLOW_RATIO   /* 缓慢齿比（转向稳定） */
} SteeringCharacteristic_t;

/**
 * @brief 计算前轮转角（基于渐进式特性）
 * 
 * 渐进式特性模拟实际车辆的转向机构：
 * - 小转角：齿比较大（转向不灵敏）
 * - 中等转角：齿比适中
 * - 大转角：齿比较小（转向灵敏）
 * 
 * @param steering_wheel 方向盘转角 (弧度)
 * @param characteristic 转向特性
 * @param base_ratio 基础齿比
 * @return 前轮转角 (弧度)
 */
static double compute_wheel_angle(double steering_wheel, 
                                  SteeringCharacteristic_t characteristic,
                                  double base_ratio) {
    double wheel_angle = 0.0;
    
    switch (characteristic) {
        case STEERING_LINEAR:
            /* 线性：wheel = steering / ratio */
            wheel_angle = steering_wheel / base_ratio;
            break;
            
        case STEERING_PROGRESSIVE: {
            /* 渐进式：非线性齿比
             * 小角度用大齿比（18:1），大角度用小齿比（12:1）
             * 使用二次多项式拟合
             */
            double normalized = steering_wheel / 3.14159265;  /* 归一化到±1 */
            double ratio = base_ratio - 4.0 * fabs(normalized);  /* 齿比范围 [12, 20] */
            wheel_angle = steering_wheel / ratio;
            break;
        }
            
        case STEERING_QUICK_RATIO:
            /* 快速齿比：反应灵敏 */
            wheel_angle = steering_wheel / (base_ratio - 3.0);
            break;
            
        case STEERING_SLOW_RATIO:
            /* 缓慢齿比：反应稳定 */
            wheel_angle = steering_wheel / (base_ratio + 2.0);
            break;
            
        default:
            wheel_angle = steering_wheel / base_ratio;
            break;
    }
    
    /* 限幅：防止前轮转角过大 */
    double max_wheel_angle = 0.628318530718;  /* ±36° */
    if (wheel_angle > max_wheel_angle) {
        wheel_angle = max_wheel_angle;
    } else if (wheel_angle < -max_wheel_angle) {
        wheel_angle = -max_wheel_angle;
    }
    
    return wheel_angle;
}

/**
 * @brief 生成映射表
 * 
 * @param num_points 采样点数
 * @param characteristic 转向特性
 * @param base_ratio 基础齿比
 * @param filename 输出文件名
 * @return 0 成功，-1 失败
 */
static int generate_mapping_table(int num_points,
                                  SteeringCharacteristic_t characteristic,
                                  double base_ratio,
                                  const char *filename) {
    if (num_points < 2 || num_points > MAX_MAPPING_POINTS) {
        printf("Error: num_points must be between 2 and %d\n", MAX_MAPPING_POINTS);
        return -1;
    }
    
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        printf("Error: Cannot open file %s for writing\n", filename);
        return -1;
    }
    
    /* 写入CSV头 */
    fprintf(fp, "SteeringWheelAngle(rad),SteeringWheelAngle(deg),LeftWheelAngle(rad),LeftWheelAngle(deg),");
    fprintf(fp, "RightWheelAngle(rad),RightWheelAngle(deg)\n");
    
    /* 最大方向盘转角：±3 rad (±172°) */
    double max_steering_wheel = 3.0;
    
    for (int i = 0; i < num_points; i++) {
        /* 计算当前采样点的方向盘转角 */
        double t = (i == 0) ? 0.0 : ((double)(i - num_points/2) / (num_points/2 - 1));
        double steering_wheel = t * max_steering_wheel;
        
        /* 计算前左轮转角（假设对称） */
        double left_wheel = compute_wheel_angle(steering_wheel, characteristic, base_ratio);
        
        /* 计算前右轮转角
         * 对于简单的Ackermann几何：
         * - 直线行驶时，左右轮转角相同
         * - 转向时，由于车轴几何关系，可能有微小差异
         * 这里假设差异很小（<1%）
         */
        double right_wheel = left_wheel * (1.0 - 0.005 * fabs(steering_wheel));
        
        /* 转换为度数 */
        double steering_wheel_deg = steering_wheel * 180.0 / PI;
        double left_wheel_deg = left_wheel * 180.0 / PI;
        double right_wheel_deg = right_wheel * 180.0 / PI;
        
        /* 写入CSV行 */
        fprintf(fp, "%.6f,%.2f,%.6f,%.2f,%.6f,%.2f\n",
                steering_wheel, steering_wheel_deg,
                left_wheel, left_wheel_deg,
                right_wheel, right_wheel_deg);
    }
    
    fclose(fp);
    printf("Mapping table generated successfully: %s\n", filename);
    printf("  - Points: %d\n", num_points);
    printf("  - Characteristic: %s\n", 
           characteristic == STEERING_LINEAR ? "Linear" :
           characteristic == STEERING_PROGRESSIVE ? "Progressive" :
           characteristic == STEERING_QUICK_RATIO ? "QuickRatio" :
           "SlowRatio");
    printf("  - Base ratio: %.2f\n", base_ratio);
    
    return 0;
}

/**
 * @brief 打印使用方法
 */
static void print_usage(const char *prog_name) {
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -p <num>      Number of mapping points (2-51, default 21)\n");
    printf("  -c <mode>     Steering characteristic: linear/progressive/quickratio/slowratio (default progressive)\n");
    printf("  -r <ratio>    Base steering ratio (default 16.0)\n");
    printf("  -o <file>     Output CSV filename (default steering_mapping.csv)\n");
    printf("  -h            Show this help\n");
    printf("\nExamples:\n");
    printf("  %s                              # Generate with defaults\n", prog_name);
    printf("  %s -p 31 -c progressive         # Generate with 31 points, progressive characteristic\n", prog_name);
    printf("  %s -r 14.5 -o mymap.csv         # Generate with ratio 14.5, save to mymap.csv\n", prog_name);
}

int main(int argc, char *argv[]) {
    int num_points = 21;
    SteeringCharacteristic_t characteristic = STEERING_PROGRESSIVE;
    double base_ratio = 16.0;
    const char *output_file = CSV_FILENAME;
    
    /* 解析命令行参数 */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            num_points = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            const char *mode = argv[++i];
            if (strcmp(mode, "linear") == 0) {
                characteristic = STEERING_LINEAR;
            } else if (strcmp(mode, "progressive") == 0) {
                characteristic = STEERING_PROGRESSIVE;
            } else if (strcmp(mode, "quickratio") == 0) {
                characteristic = STEERING_QUICK_RATIO;
            } else if (strcmp(mode, "slowratio") == 0) {
                characteristic = STEERING_SLOW_RATIO;
            }
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            base_ratio = atof(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }
    
    printf("Generating steering mapping table...\n");
    printf("  Parameters: %d points, ratio %.1f, output %s\n", 
           num_points, base_ratio, output_file);
    
    return generate_mapping_table(num_points, characteristic, base_ratio, output_file);
}
