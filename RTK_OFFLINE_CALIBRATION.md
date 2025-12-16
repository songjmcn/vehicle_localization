# RTK离线标定工作流程指南

## 概述

本文档详细说明如何使用RTK数据进行**离线参数标定**。系统现在采用分离架构：
- **实时定位**：仅使用轮式里程计 + IMU + EKF（简单、确定性）
- **离线标定**：使用RTK测量数据与里程计数据的对比来优化参数

## 为什么使用离线RTK标定？

### 优势
1. **简化实时系统**：避免在线RTK融合的复杂性和延迟
2. **灵活的标定**：可以在任何时间进行离线参数优化
3. **数据重用**：同一套传感器数据可用于多种标定目的
4. **批处理能力**：收集数据后可离线计算最优参数
5. **提高可靠性**：不受RTK延迟或中断的影响

## 工作流程

### 阶段1：数据收集
在实际运行中收集轮式里程计和RTK数据对：

```c
#include "localization_system.h"
#include <stdio.h>

int main() {
    // 初始化定位系统
    LocalizationSystem_t loc_sys;
    localization_system_init(&loc_sys);
    
    // 数据收集阶段
    printf("=== RTK离线标定 - 数据收集阶段 ===\n");
    
    for (int i = 0; i < 5000; i++) {  // 收集250秒的数据
        // 获取传感器读数
        SensorReadings_t readings = get_sensor_readings();
        
        // 运行定位（使用初始标定参数）
        VehicleState_t localization_state;
        localization_system_process(&loc_sys, &readings, &localization_state);
        
        // 每5秒收集一个RTK标定点（100步 × 50ms = 5秒）
        if (i % 100 == 0) {
            RTKData_t rtk_meas = get_rtk_reading();
            
            // 检查RTK质量
            if (rtk_meas.quality >= 2) {
                // 关键：获取此时刻的里程计估计状态
                VehicleState_t odometry_state;
                localization_system_get_state(&loc_sys, &odometry_state);
                
                // 添加标定点（需要同时提供里程计和RTK状态）
                int ret = localization_system_rtk_calibration_add_point(
                    &loc_sys, 
                    &odometry_state,  // 里程计估计
                    &rtk_meas         // RTK真值
                );
                
                if (ret == 0) {
                    printf("[收集点%d] ODO: (%.2f, %.2f) RTK: (%.2f, %.2f)\n",
                           i/100, 
                           odometry_state.x, odometry_state.y,
                           rtk_meas.x, rtk_meas.y);
                }
            }
        }
        
        if (i % 500 == 0) {
            printf("进度：%.1f%%\n", 100.0 * i / 5000);
        }
    }
    
    printf("✓ 数据收集完成，共%d个标定点\n", 
           localization_sys.rtk_calibration.calibration_points);
    
    // 阶段2：离线计算
    printf("\n=== 阶段2：计算标定参数 ===\n");
    
    int ret = localization_system_apply_rtk_calibration(&loc_sys);
    if (ret == 0) {
        printf("✓ 标定成功！\n");
        
        // 查看标定结果
        double avg_error = rtk_calibration_get_avg_position_error(
            &loc_sys.rtk_calibration);
        printf("平均位置误差: %.3f m\n", avg_error);
        
        // 获取新的标定参数
        double wheel_cal = localization_system_get_wheel_calibration(&loc_sys);
        printf("轮半径标定因子: %.6f\n", wheel_cal);
    } else {
        printf("✗ 标定失败\n");
        return -1;
    }
    
    // 清理
    localization_system_free(&loc_sys);
    return 0;
}
```

### 阶段2：计算标定参数

```c
// 在离线处理工具中执行
int compute_calibration(LocalizationSystem_t *sys) {
    // 检查数据充分性
    if (sys->rtk_calibration.calibration_points < 10) {
        printf("标定点不足，需要至少10个点\n");
        return -1;
    }
    
    // 计算参数（内部使用最小二乘法）
    int ret = localization_system_apply_rtk_calibration(sys);
    if (ret != 0) {
        printf("参数计算失败\n");
        return -1;
    }
    
    // 验证结果
    double error = rtk_calibration_get_avg_position_error(&sys->rtk_calibration);
    if (error > 1.0) {  // 1米误差阈值
        printf("警告：标定精度较低，平均误差%.3f m\n", error);
    }
    
    return 0;
}
```

### 阶段3：应用新标定参数

标定参数自动应用到系统中，之后的定位将使用更新的参数：

```c
// 标定后的定位运行
printf("\n=== 阶段3：使用标定参数进行定位 ===\n");

for (int i = 0; i < 1000; i++) {
    SensorReadings_t readings = get_sensor_readings();
    VehicleState_t state;
    
    // 此时使用的是标定后的参数
    localization_system_process(&loc_sys, &readings, &state);
    
    if (i % 200 == 0) {
        printf("[%.1fs] 位置: (%.2f, %.2f) m, 航向: %.2f°\n",
               i * 0.05, state.x, state.y, RAD_TO_DEG(state.theta));
    }
}
```

## 关键代码详解

### 1. 获取里程计状态

```c
VehicleState_t odometry_state;
localization_system_get_state(&loc_sys, &odometry_state);
```

这获取的是**纯里程计积分**的状态，用作标定的基准。

### 2. 添加标定点

```c
int ret = localization_system_rtk_calibration_add_point(
    &loc_sys,
    &odometry_state,  // 里程计估计（基准）
    &rtk_meas         // RTK真值
);
```

- **返回值**：0=成功，-1=失败
- **目的**：计算里程计与RTK之间的误差向量

### 3. 计算参数

```c
int ret = localization_system_apply_rtk_calibration(&loc_sys);
```

- 执行最小二乘拟合
- 计算轮半径标定因子
- 计算系统误差
- 自动应用到系统中

## 应用场景

### 场景1：新车参数标定
```
1. 装配新车后进行标定测试（2-5分钟）
2. 收集至少20个RTK数据点
3. 离线计算最优轮半径标定因子
4. 应用到生产系统
```

### 场景2：定期维护后重标定
```
1. 轮胎更换、底盘调整后
2. 重新收集10-20分钟的数据
3. 执行重标定流程
4. 验证标定精度（应 < 0.5m）
```

### 场景3：不同工作条件下的标定
```
1. 不同地面硬度：混凝土、沥青、砾石等
2. 不同负载：空载、半载、满载
3. 不同速度范围
4. 为每个条件创建独立的标定参数集
```

## 最佳实践

### 数据收集建议
- **最小标定点数**：10-20个点
- **推荐点数**：50-100个点
- **时间覆盖**：至少覆盖所有工作速度范围
- **空间覆盖**：不同方向的运动（直行、转向、后退）

### 质量检查
```c
// 只接受高质量的RTK数据
if (rtk_data.quality >= 2) {  // 0=失败, 1=单点, 2=浮点, 4=固定
    localization_system_rtk_calibration_add_point(...);
}
```

### 结果验证
```c
// 验证标定效果
double error = rtk_calibration_get_avg_position_error(&sys->rtk_calibration);

if (error < 0.2) {
    printf("优秀：精度 < 20cm\n");
} else if (error < 0.5) {
    printf("良好：精度 < 50cm\n");
} else if (error < 1.0) {
    printf("可接受：精度 < 1m\n");
} else {
    printf("需要重新标定\n");
}
```

## 故障排除

### 问题1：标定点数不足
```
原因：RTK信号弱，数据收集时间太短
解决：延长收集时间，确保在RTK信号良好的区域测试
```

### 问题2：标定精度差（误差 > 1m）
```
原因：
- 轮胎打滑或偏磨
- RTK数据质量低（质量值 < 2）
- 传感器安装不正确
- 收集数据时速度变化过快

解决：
1. 检查轮胎状态
2. 增加高质量RTK点的数量
3. 重新检查传感器对齐
4. 使用恒定速度进行标定
```

### 问题3：标定后定位精度反而下降
```
原因：可能存在异常数据点或离群值

解决：
1. 检查RTK质量值
2. 移除明显的离群数据点
3. 使用更健壮的最小二乘算法
4. 验证里程计读数的连续性
```

## 系统架构

```
┌─────────────────────────────────────────┐
│     实时定位系统（EKF）                  │
│  输入：轮脉冲 + IMU → 里程计 + EKF      │
│  使用：标定后的轮半径参数                │
└─────────────────────────────────────────┘
                    ↓
        RTK离线标定模块（批处理）
        收集：里程计 vs RTK数据对
        计算：优化轮半径标定因子
        反馈：应用新参数到系统
```

## 集成步骤

### 步骤1：在项目中启用RTK标定
```c
// 在初始化时自动创建标定模块
LocalizationSystem_t sys;
localization_system_init(&sys);  // rtk_calibration已初始化
```

### 步骤2：数据收集模式
```c
// 标定模式
#define CALIBRATION_MODE 1

#if CALIBRATION_MODE
    // 收集标定数据
    localization_system_rtk_calibration_add_point(&sys, &odo, &rtk);
#endif
```

### 步骤3：离线处理
```bash
# 运行离线标定工具
$ ./calibration_tool --input collected_data.bin --output calibration_params.bin

# 验证结果
$ ./verify_calibration --params calibration_params.bin
```

### 步骤4：应用参数
```c
// 保存标定参数
localization_system_save_calibration(&sys, "calibration.dat");

// 在新启动时加载
localization_system_load_calibration(&sys, "calibration.dat");
```

## 参考文献

- [轮式里程计标定](PARAMETER_CALIBRATION_GUID.MD)
- [集成指南](INTEGRATION_GUIDE.md)
- [低速定位](low_speed_localization_guide.md)

---

**版本**: 2.0  
**更新日期**: 2024年  
**状态**: RTK在线融合已移除，仅保留离线标定功能
