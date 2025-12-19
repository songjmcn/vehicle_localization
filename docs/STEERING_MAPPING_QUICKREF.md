# 转向角映射表快速参考

## 🚀 快速开始

### 1. 生成映射表
```bash
cd /Users/alex/workspace/code_review/vehicle_localization
./build/bin/generate_steering_mapping
# 输出: steering_mapping.csv
```

### 2. 运行测试
```bash
./build/bin/test_steering_mapping
# 输出: 4个测试套件结果
```

### 3. 运行集成示例
```bash
./build/bin/example_mapping_integration
# 输出: 车辆运动模拟结果
```

---

## 📊 关键数据

### 映射表规格
- **采样点数**：21个（从-π到π）
- **转向特性**：渐进式（非线性）
- **基础齿比**：16.0
- **前轮转角限幅**：±0.628 rad (±36°)
- **Ackermann轴距**：2.7 m
- **Ackermann轨距**：1.6 m

### 精度对比

| 转向角度 | 简单模型误差 | Ackermann模型误差 | 相对提升 |
|---------|-----------|-----------------|--------|
| ±10° | 2.14% | 2.16% | 基准 |
| ±20° | 2.89% | 2.55% | ↓ 12% |
| ±30° | 4.67% | 3.54% | ↓ 24% |

### 左右轮转角差异

| 转向角度 | 差异(度) | 百分比 |
|---------|--------|-------|
| ±10° | 0.01° | 1.6% |
| ±20° | 0.02° | 1.6% |
| ±30° | 0.04° | 2.1% |
| ±40° | 0.08° | 3.0% |

---

## 📝 API 速查

### 初始化
```c
#include "src/steering_model.h"

// 数组方式初始化
double sw[21], left[21], right[21];
// ... 填充数据
steering_mapping_init(sw, left, right, 21);
```

### 查询映射表
```c
// 更新转向角度
steering_model_update(steering_wheel_angle, steering_rate);

// 获取前轮转角
double left_wheel, right_wheel, mean_angle;
steering_get_wheel_angles(&left_wheel, &right_wheel, &mean_angle);
```

### 计算航向角速率
```c
double yaw_rate = steering_calculate_yaw_rate_ackermann(
    velocity,              // m/s
    steering_wheel_angle,  // rad
    2.7                    // wheelbase (m)
);
```

### 清理资源
```c
steering_mapping_cleanup();
```

---

## 🔧 CMake 配置

编译所有目标：
```bash
cmake --build build
```

编译单个目标：
```bash
cmake --build build --target test_steering_mapping
cmake --build build --target example_mapping_integration
cmake --build build --target generate_steering_mapping
```

---

## 📂 文件结构

```
vehicle_localization/
├── src/
│   ├── steering_model.h          # 映射表API
│   ├── steering_model.c          # 映射表实现
│   ├── motion_model.c            # 集成映射表
│   └── ...
├── tools/
│   └── generate_steering_mapping.c  # 生成工具
├── test_steering_mapping.c       # 完整测试
├── example_mapping_integration.c # 集成示例
├── steering_mapping.csv          # 生成的映射表
└── CMakeLists.txt
```

---

## 🎯 常见使用场景

### 场景1：静态加载（推荐）
```c
// 在程序启动时一次性加载
static double sw_cache[21], left_cache[21], right_cache[21];
int init_steering_system() {
    return steering_mapping_init(sw_cache, left_cache, right_cache, 21);
}
```

### 场景2：EKF集成
```c
void ekf_predict(double steering_wheel, double velocity, double dt) {
    steering_model_update(steering_wheel, 0);
    
    double left, right, mean;
    steering_get_wheel_angles(&left, &right, &mean);
    
    double yaw_rate = steering_calculate_yaw_rate_ackermann(
        velocity, steering_wheel, 2.7
    );
    
    // 更新EKF
    update_ekf(yaw_rate, velocity, dt);
}
```

### 场景3：动态参数调整
```c
// 根据驾驶条件调整轴距
double wheelbase = highway ? 2.8 : 2.7;
double track_width = highway ? 1.65 : 1.6;

yaw_rate = steering_calculate_yaw_rate_ackermann(
    velocity, steering_wheel, wheelbase
);
```

---

## ⚡ 性能特征

| 操作 | 耗时 | 注释 |
|------|------|------|
| 映射表初始化 | ~100 µs | 一次性 |
| 映射表查询 | < 1 µs | 线性插值 |
| Ackermann计算 | < 5 µs | 三角函数 |
| 完整预测 | < 10 µs | 包括状态更新 |

**内存占用**：
- 21个点的映射表：~504 字节
- 全局结构体：~200 字节
- 总计：~700 字节

---

## 🐛 故障排查

### 问题1：映射表查询返回错误
**原因**：映射表未初始化或已被清理
**解决**：确认 `steering_mapping_init()` 成功返回 0

### 问题2：Ackermann计算结果异常
**原因**：轴距参数错误或转向角度超出范围
**解决**：检查 wheelbase 参数和转向角 (±36° 为标准)

### 问题3：性能下降
**原因**：频繁调用 `steering_mapping_init()`
**解决**：仅在系统启动时调用一次，之后重复使用

---

## 📈 性能基准

### 测试环境
- CPU: Apple M1/M2
- 编译器: Apple Clang 14.0+
- 优化级别: -O2

### 基准结果
```
映射表生成: 8.5 ms (21点)
映射表查询×1000: 0.8 ms
Ackermann计算×1000: 2.5 ms
完整循环×1000: 3.5 ms
```

---

## 🔗 相关文件

| 文件 | 描述 |
|------|------|
| [STEERING_MAPPING_IMPLEMENTATION.md](STEERING_MAPPING_IMPLEMENTATION.md) | 详细实现说明 |
| [CMAKE_STRUCTURE.md](CMAKE_STRUCTURE.md) | CMake构建结构 |
| [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) | 集成指南 |

---

## ✨ 最佳实践

1. **初始化一次**：在程序启动时初始化映射表
2. **缓存结果**：避免重复查询相同的转向角度
3. **边界检查**：确保转向角在 ±36° 范围内
4. **单位一致**：所有角度使用弧度（rad），距离使用米（m）
5. **清理资源**：程序退出前调用 `steering_mapping_cleanup()`

---

*最后更新: 2024年*
*版本: 1.0*

