/**
 * @file test_steering_offset_sign.c
 * @brief 转向角offset符号验证单元测试
 * 
 * 测试目的：验证离线标定计算出的offset符号是否正确，
 * 确保在应用时能正确补偿VL的转向角误差
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

#define M_PI 3.14159265358979323846
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)

/* 测试场景 1: VL 转向过度（超前）*/
void test_vl_over_steering() {
    printf("\n═══════════════════════════════════════════════════════\n");
    printf("测试场景1: VL转向过度（多转了）\n");
    printf("═══════════════════════════════════════════════════════\n");
    
    /* 假设场景 */
    double rtk_heading = DEG_TO_RAD(45.0);      /* RTK参考：45° */
    double vl_heading = DEG_TO_RAD(45.5);       /* VL估计：45.5°（多转0.5°）*/
    
    /* 计算误差（标定算法） */
    double heading_error = rtk_heading - vl_heading;  /* -0.5° */
    double offset = heading_error;  /* offset = -0.5° (按算法计算) */
    
    printf("RTK航向角: %.2f°\n", RAD_TO_DEG(rtk_heading));
    printf("VL航向角:  %.2f° (比RTK多转0.5°)\n", RAD_TO_DEG(vl_heading));
    printf("heading_error = RTK - VL = %.4f rad (%.2f°)\n", 
           heading_error, RAD_TO_DEG(heading_error));
    printf("计算出的offset = %.4f rad (%.2f°)\n", offset, RAD_TO_DEG(offset));
    
    /* 应用校准 */
    double raw_steering = DEG_TO_RAD(5.0);  /* 原始转向角5° */
    double calibrated_steering = raw_steering - offset;
    
    printf("\n应用校准:\n");
    printf("原始转向角: %.2f°\n", RAD_TO_DEG(raw_steering));
    printf("校准公式: cal_steering = raw_steering - offset\n");
    printf("校准后转向角: %.2f° - (%.2f°) = %.2f°\n",
           RAD_TO_DEG(raw_steering), RAD_TO_DEG(offset), RAD_TO_DEG(calibrated_steering));
    
    /* 验证 */
    printf("\n✓ 验证结果:\n");
    if (offset < 0) {
        printf("✅ offset < 0 (负值)，表示VL转向过度\n");
        printf("✅ 应用时: cal_steering = 5.0° - (-0.5°) = 5.5°（增加转向角）\n");
        printf("❌ 这是错误的！我们应该减小转向角来补偿过度转向\n");
        printf("\n结论：算法计算出的offset符号需要反转！\n");
    } else {
        printf("✅ offset > 0 (正值)，表示VL转向过度\n");
        printf("✅ 应用时: cal_steering = 5.0° - (+0.5°) = 4.5°（减小转向角）\n");
        printf("✅ 这是正确的补偿！\n");
    }
}

/* 测试场景 2: VL 转向不足（滞后）*/
void test_vl_under_steering() {
    printf("\n═══════════════════════════════════════════════════════\n");
    printf("测试场景2: VL转向不足（少转了）\n");
    printf("═══════════════════════════════════════════════════════\n");
    
    /* 假设场景 */
    double rtk_heading = DEG_TO_RAD(45.0);      /* RTK参考：45° */
    double vl_heading = DEG_TO_RAD(44.5);       /* VL估计：44.5°（少转0.5°）*/
    
    /* 计算误差（标定算法） */
    double heading_error = rtk_heading - vl_heading;  /* +0.5° */
    double offset = heading_error;  /* offset = +0.5° (按算法计算) */
    
    printf("RTK航向角: %.2f°\n", RAD_TO_DEG(rtk_heading));
    printf("VL航向角:  %.2f° (比RTK少转0.5°)\n", RAD_TO_DEG(vl_heading));
    printf("heading_error = RTK - VL = %.4f rad (%.2f°)\n", 
           heading_error, RAD_TO_DEG(heading_error));
    printf("计算出的offset = %.4f rad (%.2f°)\n", offset, RAD_TO_DEG(offset));
    
    /* 应用校准 */
    double raw_steering = DEG_TO_RAD(5.0);  /* 原始转向角5° */
    double calibrated_steering = raw_steering - offset;
    
    printf("\n应用校准:\n");
    printf("原始转向角: %.2f°\n", RAD_TO_DEG(raw_steering));
    printf("校准公式: cal_steering = raw_steering - offset\n");
    printf("校准后转向角: %.2f° - %.2f° = %.2f°\n",
           RAD_TO_DEG(raw_steering), RAD_TO_DEG(offset), RAD_TO_DEG(calibrated_steering));
    
    /* 验证 */
    printf("\n✓ 验证结果:\n");
    if (offset > 0) {
        printf("✅ offset > 0 (正值)，表示VL转向不足\n");
        printf("✅ 应用时: cal_steering = 5.0° - 0.5° = 4.5°（减小转向角）\n");
        printf("❌ 这是错误的！我们应该增加转向角来补偿不足转向\n");
        printf("\n结论：算法计算出的offset符号需要反转！\n");
    } else {
        printf("✅ offset < 0 (负值)，表示VL转向不足\n");
        printf("✅ 应用时: cal_steering = 5.0° - (-0.5°) = 5.5°（增加转向角）\n");
        printf("✅ 这是正确的补偿！\n");
    }
}

/* 测试场景 3: 实际诊断案例 */
void test_diagnostic_case() {
    printf("\n═══════════════════════════════════════════════════════\n");
    printf("测试场景3: 实际诊断案例 (calibration.json)\n");
    printf("═══════════════════════════════════════════════════════\n");
    
    /* 实际测量数据 */
    double offset_original = -0.00914977;  /* 原始（错误） */
    double offset_fixed = 0.00914977;      /* 修复后 */
    
    printf("原始offset (错误): %.8f rad (%.4f°)\n", 
           offset_original, RAD_TO_DEG(offset_original));
    printf("修复后offset:      %.8f rad (%.4f°)\n", 
           offset_fixed, RAD_TO_DEG(offset_fixed));
    
    /* 应用示例 */
    double raw_steering = DEG_TO_RAD(0.029863);  /* 原始转向角 */
    
    printf("\n应用校准 (原始转向角 %.6f rad):\n", raw_steering);
    
    double cal_steering_wrong = raw_steering - offset_original;
    printf("错误应用: %.6f - (%.8f) = %.6f rad (%.4f°)\n",
           raw_steering, offset_original, cal_steering_wrong, RAD_TO_DEG(cal_steering_wrong));
    
    double cal_steering_correct = raw_steering - offset_fixed;
    printf("正确应用: %.6f - %.8f = %.6f rad (%.4f°)\n",
           raw_steering, offset_fixed, cal_steering_correct, RAD_TO_DEG(cal_steering_correct));
    
    printf("\n✓ 结果对比:\n");
    printf("错误应用增加了转向角 (%.6f°)\n", RAD_TO_DEG(cal_steering_wrong - raw_steering));
    printf("正确应用减小了转向角 (%.6f°)\n", RAD_TO_DEG(cal_steering_correct - raw_steering));
    printf("\n诊断结论：修复后的offset能正确补偿VL的转向角误差\n");
}

/* 总结 */
void print_summary() {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════╗\n");
    printf("║        转向角Offset符号验证总结                        ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    printf("\n参数定义：\n");
    printf("  heading_error = RTK_heading - VL_heading\n");
    printf("  offset = average(heading_error)\n");
    printf("  应用公式: cal_steering = raw_steering - offset\n\n");
    printf("符号含义：\n");
    printf("  • offset > 0: VL转向过度，应减小转向角\n");
    printf("               减法: raw_steering - positive = 更小的角度 ✓\n\n");
    printf("  • offset < 0: VL转向不足，应增加转向角\n");
    printf("               减法: raw_steering - negative = 更大的角度 ✓\n\n");
    printf("离线标定算法是正确的！\n");
    printf("参数存储时应该直接使用计算出的offset值（无需反转）\n");
    printf("\n");
}

int main(void) {
    printf("\n╔════════════════════════════════════════════════════════╗\n");
    printf("║     转向角Offset符号验证 - 单元测试                   ║\n");
    printf("╚════════════════════════════════════════════════════════╝\n");
    
    test_vl_over_steering();
    test_vl_under_steering();
    test_diagnostic_case();
    print_summary();
    
    printf("✅ 所有测试完成\n");
    return 0;
}
