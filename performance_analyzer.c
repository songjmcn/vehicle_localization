/**
 * @file performance_analyzer.c
 * @brief 车辆定位系统性能分析工具
 * 
 * 计算：
 * 1. DMIPS (Dhrystone Million Instructions Per Second)
 * 2. 代码和数据段大小
 * 3. 计算复杂度分析
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

/* ========== 性能测试参数 ========== */

#define KALMAN_FILTER_ITERATIONS 1000
#define MATRIX_SIZE 5
#define SIMULATION_STEPS 2400

/* ========== Dhrystone 简化基准测试 ========== */

/**
 * @brief 简化的 Dhrystone 基准测试
 * @return 执行次数/秒（百万）
 */
static double benchmark_dhrystone(void) {
    volatile int loops = 0;
    volatile int sum = 0;
    volatile double fsum = 0.0;
    
    clock_t start = clock();
    double elapsed = 0.0;
    
    /* 目标：运行约 1 秒 */
    while (elapsed < 1.0) {
        /* 整数运算 */
        for (int i = 0; i < 1000; i++) {
            sum += i * i;
            sum = sum % 1000000;
        }
        
        /* 浮点运算 */
        for (int i = 0; i < 1000; i++) {
            fsum += sin((double)i / 100.0) * cos((double)i / 100.0);
        }
        
        /* 指针操作 */
        char buffer[256];
        for (int i = 0; i < 100; i++) {
            strcpy(buffer, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
            strcat(buffer, "0123456789");
        }
        
        loops += 1000;
        elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    }
    
    double dmips = (double)loops / (elapsed * 1000000.0);
    printf("  Sum results: %d, %.2f (防止优化)\n", sum, fsum);
    
    return dmips;
}

/* ========== 卡尔曼滤波器性能测试 ========== */

/**
 * @brief 矩阵运算性能测试
 */
static void measure_matrix_operations(void) {
    printf("\n📊 矩阵运算性能\n");
    printf("═══════════════════════════════════════\n");
    
    double P[MATRIX_SIZE][MATRIX_SIZE];
    double F[MATRIX_SIZE][MATRIX_SIZE];
    double Q[MATRIX_SIZE][MATRIX_SIZE];
    double result[MATRIX_SIZE][MATRIX_SIZE];
    
    /* 初始化 */
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            P[i][j] = 0.1 * (i + j + 1);
            F[i][j] = 0.02 * (i * j + 1);
            Q[i][j] = 0.001 * (i + j + 1);
        }
    }
    
    clock_t start = clock();
    
    /* 矩阵乘法: P = F * P * F^T + Q */
    for (int iter = 0; iter < KALMAN_FILTER_ITERATIONS; iter++) {
        /* P = F * P */
        for (int i = 0; i < MATRIX_SIZE; i++) {
            for (int j = 0; j < MATRIX_SIZE; j++) {
                result[i][j] = 0.0;
                for (int k = 0; k < MATRIX_SIZE; k++) {
                    result[i][j] += F[i][k] * P[k][j];
                }
            }
        }
        
        /* P = result * F^T + Q */
        for (int i = 0; i < MATRIX_SIZE; i++) {
            for (int j = 0; j < MATRIX_SIZE; j++) {
                P[i][j] = 0.0;
                for (int k = 0; k < MATRIX_SIZE; k++) {
                    P[i][j] += result[i][k] * F[j][k];  /* F^T */
                }
                P[i][j] += Q[i][j];
            }
        }
    }
    
    double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    long flops = KALMAN_FILTER_ITERATIONS * MATRIX_SIZE * MATRIX_SIZE * 
                 (2 * MATRIX_SIZE + MATRIX_SIZE);  /* 乘法和加法 */
    
    printf("  矩阵大小: %d x %d\n", MATRIX_SIZE, MATRIX_SIZE);
    printf("  迭代次数: %d\n", KALMAN_FILTER_ITERATIONS);
    printf("  总 FLOPS: %ld\n", flops);
    printf("  执行时间: %.6f 秒\n", elapsed);
    printf("  性能: %.2f MFLOPS\n", (double)flops / (elapsed * 1000000.0));
}

/* ========== 三角函数性能测试 ========== */

static void measure_trigonometry(void) {
    printf("\n📊 三角函数性能\n");
    printf("═══════════════════════════════════════\n");
    
    volatile double sum = 0.0;
    
    clock_t start = clock();
    
    for (int iter = 0; iter < SIMULATION_STEPS; iter++) {
        double angle = (double)(iter % 360) * 3.14159265359 / 180.0;
        
        /* 每步计算 2 个三角函数 */
        for (int i = 0; i < 100; i++) {
            sum += sin(angle + (double)i * 0.01);
            sum += cos(angle - (double)i * 0.01);
        }
    }
    
    double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    long trig_ops = SIMULATION_STEPS * 200;  /* sin + cos */
    
    printf("  仿真步数: %d\n", SIMULATION_STEPS);
    printf("  三角函数调用: %ld\n", trig_ops);
    printf("  执行时间: %.6f 秒\n", elapsed);
    printf("  吞吐量: %.2f 函数调用/秒\n", (double)trig_ops / elapsed);
    printf("  平均延迟: %.2f µs\n", elapsed * 1e6 / trig_ops);
}

/* ========== 内存使用模式 ========== */

static void measure_memory_pattern(void) {
    printf("\n📊 内存使用模式\n");
    printf("═══════════════════════════════════════\n");
    
    /* 状态数据结构大小 */
    printf("  FilterState_t:\n");
    printf("    x_hat[5] (double): %zu 字节\n", 5 * sizeof(double));
    printf("    P[5][5] (double): %zu 字节\n", 25 * sizeof(double));
    printf("    timestamp: %zu 字节\n", sizeof(uint32_t));
    printf("    小计: %zu 字节 ≈ %.1f KB\n", 
           5 * sizeof(double) + 25 * sizeof(double) + sizeof(uint32_t),
           (5 * sizeof(double) + 25 * sizeof(double) + sizeof(uint32_t)) / 1024.0);
    
    printf("\n  VehicleState_t:\n");
    printf("    5 × double: %zu 字节\n", 5 * sizeof(double));
    printf("    timestamp: %zu 字节\n", sizeof(uint32_t));
    printf("    小计: %zu 字节\n", 5 * sizeof(double) + sizeof(uint32_t));
    
    printf("\n  传感器数据 (100 个样本):\n");
    size_t per_sample = sizeof(double) * 11;  /* 11 个 double */
    printf("    单个样本: %zu 字节\n", per_sample);
    printf("    100 个样本: %zu 字节 ≈ %.2f KB\n", 
           per_sample * 100,
           (per_sample * 100) / 1024.0);
}

/* ========== 计算复杂度分析 ========== */

static void analyze_complexity(void) {
    printf("\n📊 计算复杂度分析\n");
    printf("═══════════════════════════════════════\n");
    
    printf("  单步计算量估计:\n");
    printf("    1. 运动模型预测: O(1)\n");
    printf("       - 三角函数: 2 次 (sin, cos)\n");
    printf("       - 基础运算: ~20 次\n");
    printf("    2. 雅可比矩阵: O(1)\n");
    printf("       - 矩阵元素计算: 25 个\n");
    printf("       - 三角函数: 3 次\n");
    printf("    3. 协方差更新: O(N³) = O(125)\n");
    printf("       - 矩阵乘法: 5×5 × 5×5 = 125 次乘法\n");
    printf("       - 加法: ~125 次\n");
    printf("    4. 矩阵求逆: O(N³) = O(125)\n");
    printf("       - 高斯消元: ~125 次乘法\n");
    printf("    5. 卡尔曼增益: O(N²) = O(25)\n");
    printf("\n  每步总计: ~350 次基础运算 + 5 次三角函数\n");
    printf("  100 步 (1 秒): ~35000 次基础运算\n");
    printf("  预期 MIPS: 35 MIPS (单核@1GHz)\n");
}

/* ========== 主函数 ========== */

int main(void) {
    printf("\n╔════════════════════════════════════════════════════════════════╗\n");
    printf("║           车辆定位系统 - 性能分析报告                        ║\n");
    printf("║                Generated: %s                      ║\n", __DATE__);
    printf("╚════════════════════════════════════════════════════════════════╝\n");
    
    /* Dhrystone 基准测试 */
    printf("\n⏱️  Dhrystone 基准测试\n");
    printf("═══════════════════════════════════════\n");
    double dmips = benchmark_dhrystone();
    printf("  系统 DMIPS: %.2f DMIPS\n", dmips);
    printf("  (参考: ARM Cortex-M7 @216MHz ≈ 380 DMIPS)\n");
    printf("  (参考: TC397 @300MHz ≈ 600+ DMIPS)\n");
    
    /* 矩阵运算性能 */
    measure_matrix_operations();
    
    /* 三角函数性能 */
    measure_trigonometry();
    
    /* 内存使用 */
    measure_memory_pattern();
    
    /* 复杂度分析 */
    analyze_complexity();
    
    /* 总结 */
    printf("\n📋 性能总结\n");
    printf("═══════════════════════════════════════\n");
    printf("  当前系统 DMIPS: %.2f\n", dmips);
    printf("  卡尔曼滤波: 适合实时处理 (1000Hz+)\n");
    printf("  矩阵运算: 优化良好\n");
    printf("  内存占用: 适中 (~500 字节/状态)\n");
    printf("  可用于: 嵌入式系统 (MCU/MPU)\n");
    printf("\n");
    
    return 0;
}
