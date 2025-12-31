/**
 * @file example_fusion_mode_usage.c
 * @brief 演示融合模式初始化的三种方式
 * 
 * 本文件展示如何使用方式1（修改原有init保持兼容）来初始化定位系统：
 * - 方式1a：基础初始化（向后兼容）
 * - 方式1b：指定融合模式初始化
 * - 方式1c：运行时切换融合模式
 */

#include "localization_system.h"
#include <stdio.h>
#include <unistd.h>

/* ============================================================================
   示例 1: 向后兼容的基础初始化（原有方式，无需修改）
   ============================================================================ */

void example_1_basic_init(void) {
    printf("\n=== 示例 1: 基础初始化（向后兼容）===\n");
    
    LocalizationSystem_t sys;
    VehicleState_t init_state = {
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .v = 0.0f,
        .delta = 0.0f,
        .gear = GEAR_PARK,
        .timestamp = 0
    };
    
    /* 方式1a：原有的初始化方法，自动使用 FUSION_MODE_BASIC */
    if (localization_system_init(&sys, &init_state) != 0) {
        printf("❌ 初始化失败\n");
        return;
    }
    
    printf("✅ 初始化成功\n");
    printf("   - 融合模式: FUSION_MODE_BASIC\n");
    printf("   - 系统状态: 已初始化\n");
}

/* ============================================================================
   示例 2: 指定融合模式初始化
   ============================================================================ */

void example_2_fusion_mode_init(void) {
    printf("\n=== 示例 2: 指定融合模式初始化 ===\n");
    
    LocalizationSystem_t sys;
    VehicleState_t init_state = {.x=0, .y=0, .theta=0, .v=0, .delta=0, .gear=GEAR_PARK};
    
    /* 方式1b：指定使用高级融合模式 */
    if (localization_system_init_with_fusion(&sys, &init_state, FUSION_MODE_IMU_STEERING) != 0) {
        printf("❌ 初始化失败\n");
        return;
    }
    
    printf("✅ 初始化成功\n");
    printf("   - 融合模式: FUSION_MODE_IMU_STEERING\n");
    printf("   - IMU转向权重: 1.0\n");
    printf("   - IMU转向速度阈值: 0.5 m/s\n");
}

/* ============================================================================
   示例 3: 运行时切换融合模式
   ============================================================================ */

void example_3_runtime_switch(void) {
    printf("\n=== 示例 3: 运行时切换融合模式 ===\n");
    
    LocalizationSystem_t sys;
    VehicleState_t init_state = {.x=0, .y=0, .theta=0, .v=0, .delta=0, .gear=GEAR_PARK};
    
    /* 先用基础融合初始化 */
    if (localization_system_init(&sys, &init_state) != 0) {
        return;
    }
    printf("✅ 初始化成功，当前模式: FUSION_MODE_BASIC\n");
    
    /* 模拟运行... */
    printf("   处理数据...\n");
    sleep(1);
    
    /* 方式1c：运行时切换到高级融合模式 */
    printf("🔄 检测到IMU质量良好，切换到高级融合模式...\n");
    if (localization_system_switch_fusion(&sys, FUSION_MODE_IMU_STEERING) != 0) {
        printf("❌ 切换失败\n");
        return;
    }
    
    printf("✅ 成功切换到: FUSION_MODE_IMU_STEERING\n");
    printf("   - 状态保持连续（无重置）\n");
    
    /* 继续处理 */
    printf("   继续处理数据...\n");
}

/* ============================================================================
   示例 4: 条件选择融合模式
   ============================================================================ */

void example_4_conditional_selection(void) {
    printf("\n=== 示例 4: 根据环境条件选择融合模式 ===\n");
    
    LocalizationSystem_t sys;
    VehicleState_t init_state = {.x=0, .y=0, .theta=0, .v=0, .delta=0, .gear=GEAR_PARK};
    
    /* 检测IMU传感器质量 */
    bool has_high_quality_imu = 1;  /* 假设检测到高质量IMU */
    
    if (has_high_quality_imu) {
        printf("✓ 检测到高质量IMU传感器\n");
        printf("  初始化使用: FUSION_MODE_IMU_STEERING\n");
        
        if (localization_system_init_with_fusion(&sys, &init_state, 
                                                 FUSION_MODE_IMU_STEERING) != 0) {
            return;
        }
        printf("✅ 初始化成功\n");
    } else {
        printf("✗ IMU质量较低\n");
        printf("  初始化使用: FUSION_MODE_BASIC\n");
        
        if (localization_system_init(&sys, &init_state) != 0) {
            return;
        }
        printf("✅ 初始化成功\n");
    }
}

/* ============================================================================
   示例 5: 多系统实例支持
   ============================================================================ */

void example_5_multiple_instances(void) {
    printf("\n=== 示例 5: 多系统实例支持 ===\n");
    
    LocalizationSystem_t sys_front, sys_rear;
    VehicleState_t init_state = {.x=0, .y=0, .theta=0, .v=0, .delta=0, .gear=GEAR_PARK};
    
    /* 前部定位系统：使用高级融合 */
    printf("初始化前部定位系统...\n");
    if (localization_system_init_with_fusion(&sys_front, &init_state, 
                                             FUSION_MODE_IMU_STEERING) != 0) {
        return;
    }
    printf("✅ 前部系统初始化成功\n");
    
    /* 后部定位系统：使用基础融合 */
    printf("初始化后部定位系统...\n");
    if (localization_system_init(&sys_rear, &init_state) != 0) {
        return;
    }
    printf("✅ 后部系统初始化成功\n");
    
    printf("\n🎯 多系统实例已独立初始化，可独立运行\n");
    printf("   - 前部系统: FUSION_MODE_IMU_STEERING\n");
    printf("   - 后部系统: FUSION_MODE_BASIC\n");
}

/* ============================================================================
   主函数
   ============================================================================ */

int main(void) {
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║   融合模式初始化 - 方式1演示（修改原有init保持兼容）    ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    
    /* 运行所有示例 */
    example_1_basic_init();
    example_2_fusion_mode_init();
    example_3_runtime_switch();
    example_4_conditional_selection();
    example_5_multiple_instances();
    
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║                  所有示例完成！                         ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n💡 关键特性总结：\n");
    printf("   ✓ 向后兼容 - 原有代码无需修改\n");
    printf("   ✓ 融合模式选择 - 初始化时指定\n");
    printf("   ✓ 运行时切换 - 平滑更改算法\n");
    printf("   ✓ 多实例支持 - 独立的配置\n");
    
    return 0;
}
