# 卡尔曼滤波器调试与诊断指南

## 📋 概述

本指南提供了扩展卡尔曼滤波（EKF）在车辆定位中的诊断方法、常见问题及解决方案。

---

## 🔍 状态向量与矩阵

### 1. 状态向量结构

```c
typedef struct {
    double x;       // 全局X位置 (m)
    double y;       // 全局Y位置 (m)
    double heading; // 航向角 (rad)
    double velocity;// 纵向速度 (m/s)
    double steering;// 转向角 (rad)
} State;
```

### 2. 协方差矩阵P（5×5）

```
P = [P_xx  P_xy  P_xh  P_xv  P_xs]
    [P_yx  P_yy  P_yh  P_yv  P_ys]
    [P_hx  P_hy  P_hh  P_hv  P_hs]
    [P_vx  P_vy  P_vh  P_vv  P_vs]
    [P_sx  P_sy  P_sh  P_sv  P_ss]

P[i][i] = 状态i的方差（不确定性的平方）
P[i][j] = 状态i和j之间的协方差（相关性）
```

对角元素的物理含义：
| i | 符号 | 单位 | 含义 | 标准差σ |
|---|------|------|------|--------|
| 0 | P_xx | m² | X位置方差 | √P_xx |
| 1 | P_yy | m² | Y位置方差 | √P_yy |
| 2 | P_hh | rad² | 航向方差 | √P_hh |
| 3 | P_vv | (m/s)² | 速度方差 | √P_vv |
| 4 | P_ss | rad² | 转向方差 | √P_ss |

---

## ⚠️ 常见问题及症状

### 问题1：协方差矩阵奇异（Singular Matrix）

#### 症状
```
[ERROR] Matrix is singular
[ERROR] Cannot compute inverse for Kalman gain
运行崩溃或输出NaN
```

#### 可能原因

1. **初始协方差太小**
   ```c
   P[i][i] = 0.0  // ❌ 完全确定性，无信息增益
   ```

2. **测量噪声太大**
   ```c
   R[i][i] = 1e10  // ❌ 认为测量完全不可信
   ```

3. **状态转移矩阵有问题**
   ```c
   F[i][j] = NaN或Inf  // ❌ 无效的雅可比值
   ```

#### 诊断方法

```python
# Python诊断脚本
import numpy as np

def check_singular_matrix(P):
    """检查矩阵是否接近奇异"""
    
    # 计算行列式
    det = np.linalg.det(P)
    print(f"行列式: {det}")
    if abs(det) < 1e-10:
        print("⚠️ 矩阵接近奇异！")
    
    # 计算条件数
    cond = np.linalg.cond(P)
    print(f"条件数: {cond}")
    if cond > 1e10:
        print("⚠️ 矩阵数值不稳定！")
    
    # 计算特征值
    evals = np.linalg.eigvals(P)
    print(f"特征值: {evals}")
    if any(abs(e) < 1e-10 for e in evals):
        print("⚠️ 存在接近零的特征值！")
    
    return det, cond, evals
```

#### 解决方案

```c
// ✅ 修复1：设置合理的初始协方差
kf->state.P[0][0] = 10.0;    // 不要设为0
kf->state.P[1][1] = 10.0;
kf->state.P[2][2] = 1.0;
kf->state.P[3][3] = 0.5;
kf->state.P[4][4] = 0.2;

// ✅ 修复2：检查测量噪声合理性
// R应该反映真实的传感器精度
R[0][0] = 0.1;  // 轮速计精度：±0.1 m/s
R[1][1] = 0.5;  // IMU精度：±0.5 m/s²
R[2][2] = 0.05; // 转向角传感器：±0.05 rad

// ✅ 修复3：验证雅可比矩阵
// 检查F矩阵中是否有NaN或Inf
for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
        if (isnan(F[i][j]) || isinf(F[i][j])) {
            fprintf(stderr, "F[%d][%d] = %f (无效!)\n", i, j, F[i][j]);
        }
    }
}
```

---

### 问题2：Y坐标始终不变

#### 症状
```
初始状态：x=0, y=0
转向输入：30°
倒车速度：-1.0 m/s
预期：y应该随转向累积
实际：y保持0 ❌
```

#### 根本原因链

```
可能原因1：heading角计算错误
  ↓
heading = 0°（无论转向如何）
  ↓
sin(heading) ≈ 0
  ↓
Δy = v * sin(heading) * dt ≈ 0
  
可能原因2：雅可比矩阵F错误
  ↓
F[1][3] = 0（应该是sin_theta）
  ↓
协方差P[1][1]无法更新
  ↓
卡尔曼增益K[1] = 0
  ↓
测量无法更新Y

可能原因3：测量矩阵H错误
  ↓
H不包含Y相关的观测
  ↓
无法约束Y的更新
```

#### 诊断步骤

```c
// 步骤1：打印heading变化
printf("t=%d, heading=%f rad, v=%f, steering=%f\n",
       t, state.heading, state.velocity, state.steering);
// 预期：heading应该随steering输入而变化

// 步骤2：验证运动方程
double dy_expected = state.velocity * sin(state.heading) * dt;
printf("预期Δy = %f\n", dy_expected);
// 预期：非零值（除非heading=0且这是预期的）

// 步骤3：检查雅可比矩阵
printf("F[1][3] = %f (应该 ≈ sin(heading)*dt = %f)\n",
       F[1][3], sin(state.heading) * dt);

// 步骤4：检查协方差演化
printf("P[1][1] = %f (应该增加，不应该为常数)\n",
       state.P[1][1]);

// 步骤5：检查卡尔曼增益
printf("K[1] = %f (应该非零)\n", K[1]);
```

#### 解决方案

```c
// ✅ 解决方案1：修复heading计算
// 确保不调用get_effective_steering_angle()，直接使用velocity的符号

// ✅ 解决方案2：验证雅可比矩阵
// F[1][3]应该是sin(heading)*dt，不应包含sign_v

// ✅ 解决方案3：检查测量模型
// 确保H矩阵正确映射状态到观测
// 例如，如果测量IMU加速度，应该包含heading的贡献
```

---

### 问题3：协方差矩阵不收敛

#### 症状

```
初始P[0][0] = 10.0
运行1秒后P[0][0] = 9.8（略微减少）
运行10秒后P[0][0] = 9.5（非常缓慢）
100秒后仍未收敛到合理范围
```

#### 可能原因

1. **Q矩阵（过程噪声）太大**
   ```c
   Q[0][0] = 100.0  // ❌ 告诉滤波器不信任运动模型
   // 结果：P[0][0]无法减小
   ```

2. **R矩阵（测量噪声）太大**
   ```c
   R[0][0] = 1000.0  // ❌ 告诉滤波器不信任传感器
   // 结果：卡尔曼增益太小，测量无法修正
   ```

3. **H矩阵（测量矩阵）没有观测该状态**
   ```c
   H[0][0] = 0  // ❌ 无法观测X位置
   // 结果：协方差无法从测量获得约束
   ```

4. **没有闭环反馈**
   ```c
   // 如果只有前向递推，没有测量更新，P只能增加
   // P_{predict} = F·P·F^T + Q  (P增加)
   // P_{update} = P - K·S·K^T   (P应该减少，但如果K=0就不会)
   ```

#### 诊断方法

```python
# 绘制协方差演化曲线
import matplotlib.pyplot as plt

# 从日志读取P矩阵对角元素
P_xx_history = [...]  # 每个时刻的P[0][0]
P_yy_history = [...]
P_hh_history = [...]
P_vv_history = [...]
P_ss_history = [...]

time = range(len(P_xx_history))

fig, axes = plt.subplots(2, 3, figsize=(15, 8))

axes[0, 0].plot(time, P_xx_history)
axes[0, 0].set_title('P[0][0] (X位置方差)')
axes[0, 0].set_ylabel('方差 (m²)')

axes[0, 1].plot(time, P_yy_history)
axes[0, 1].set_title('P[1][1] (Y位置方差)')

axes[0, 2].plot(time, P_hh_history)
axes[0, 2].set_title('P[2][2] (航向方差)')

axes[1, 0].plot(time, P_vv_history)
axes[1, 0].set_title('P[3][3] (速度方差)')

axes[1, 1].plot(time, P_ss_history)
axes[1, 1].set_title('P[4][4] (转向方差)')

plt.tight_layout()
plt.savefig('covariance_evolution.png')
plt.show()

# 检查特征：
# - 应该单调递减（或振荡递减）
# - 最终应该收敛到某个值
# - 收敛速度与Q和R的比例相关
```

#### 解决方案

```c
// ✅ 调整Q和R的比例
// Q：过程噪声（运动模型的不确定性）
Q[0][0] = 0.01;    // 位置：运动模型很准确
Q[2][2] = 0.001;   // 航向：运动模型很准确
Q[3][3] = 0.1;     // 速度：可能有加速度变化

// R：测量噪声（传感器的不确定性）
R[0][0] = 0.1;     // 轮速计：精度较好
R[1][1] = 0.5;     // IMU加速度：精度一般
R[2][2] = 0.05;    // 转向角传感器：精度好

// ✅ 确保H矩阵正确
// 每个观测应该与某些状态相关

// ✅ 手动调整初始P值
// 让滤波器有空间改进估计
kf->state.P[0][0] = 10.0;  // 足够大，允许收敛
```

---

### 问题4：卡尔曼增益为零

#### 症状

```
K[0] = 0.0 (应该非零)
K[1] = 0.0
K[2] = 0.0
...
即使有测量，也无法更新状态估计
```

#### 可能原因

```
情况1：P矩阵过小
  ⟹ 分子 P·H^T 太小
  ⟹ K = P·H^T / S ≈ 0

情况2：H矩阵的行不相关
  ⟹ H矩阵秩不足
  ⟹ 无法观测所有状态
  ⟹ 对应的K行为0

情况3：R矩阵（测量噪声）太大
  ⟹ S = H·P·H^T + R ≈ R（很大）
  ⟹ K = P·H^T / S ≈ 0
```

#### 诊断

```c
// 打印K矩阵
printf("Kalman Gain Matrix:\n");
for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
        printf("K[%d][%d] = %.6f\n", i, j, K[i][j]);
    }
}

// 检查是否所有元素都为0
int all_zero = 1;
for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
        if (fabs(K[i][j]) > 1e-6) {
            all_zero = 0;
        }
    }
}
if (all_zero) {
    printf("警告：卡尔曼增益矩阵全为0！\n");
}
```

#### 解决方案

```c
// ✅ 增加初始协方差
P[i][i] = 10.0;  // 而不是0.1

// ✅ 减小测量噪声
R[i][i] = 0.1;   // 需要合理值，不要太大

// ✅ 验证H矩阵
// H应该有足够的秩以观测所有状态
// 对于完整观测：rank(H) = 5
```

---

### 问题5：数值发散（Divergence）

#### 症状

```
初始时刻：状态估计合理
运行1秒后：Y坐标 = 100m（不合理）
运行2秒后：heading = 50 rad（超出范围）
运行3秒后：输出NaN
```

#### 可能原因

1. **时间步长太大**
   ```c
   dt = 1.0;  // ❌ 太大，离散化误差大
   ```

2. **F矩阵在某个工作点奇异**
   ```c
   // 当接近奇点时，tan(steering)→∞
   // 导致heading_rate→∞
   ```

3. **协方差矩阵变成非正定**
   ```c
   // P应该始终是正定矩阵
   // 如果P = F·P·F^T出现错误，可能变得非正定
   ```

4. **测量更新计算错误**
   ```c
   // P = P - K·S·K^T
   // 如果K或S计算有误，P可能变负定
   ```

#### 诊断

```c
#include <math.h>

bool check_divergence(const KalmanFilter* kf) {
    // 检查1：状态范围
    if (fabs(kf->state.x) > 1000.0 ||
        fabs(kf->state.y) > 1000.0) {
        printf("错误：位置发散\n");
        return true;
    }
    
    if (fabs(kf->state.heading) > 2*M_PI + 1.0) {
        printf("错误：航向发散\n");
        return true;
    }
    
    if (fabs(kf->state.velocity) > 50.0) {
        printf("错误：速度发散\n");
        return true;
    }
    
    // 检查2：协方差矩阵
    for (int i = 0; i < 5; i++) {
        if (kf->state.P[i][i] < 0 || 
            isnan(kf->state.P[i][i]) ||
            isinf(kf->state.P[i][i])) {
            printf("错误：协方差P[%d][%d]非法\n", i, i);
            return true;
        }
        
        if (kf->state.P[i][i] > 1e6) {
            printf("警告：P[%d][%d]过大(%f)\n", 
                   i, i, kf->state.P[i][i]);
        }
    }
    
    return false;
}
```

#### 解决方案

```c
// ✅ 使用更小的时间步长
dt = 0.001;  // 1ms（而不是1s）

// ✅ 限制steering角范围
#define MAX_STEERING (M_PI / 4)  // ±45°
steering = clamp(steering, -MAX_STEERING, MAX_STEERING);

// ✅ 定期检查矩阵有效性
if (check_divergence(kf)) {
    // 重新初始化
    kalman_filter_init(kf);
}
```

---

## 📊 卡尔曼滤波工作流程图

```
输入：control(steering, gear)
   ↓
[预测阶段]
  ↓ 计算运动模型
  ↓ x_pred = motion_model(x, u)
  ↓ F = jacobian(motion_model)
  ↓ P_pred = F·P·F^T + Q
   ↓
   检查P是否有效
   ↓
[测量更新]
  ↓ 获取传感器数据
  ↓ z = [wheel_speed, accel, steering_angle]
  ↓ H = 测量矩阵（映射状态到观测）
  ↓ y = z - H·x_pred
  ↓ S = H·P_pred·H^T + R
  ↓ K = P_pred·H^T·S^{-1}
  ↓ x_update = x_pred + K·y
  ↓ P_update = P_pred - K·S·K^T
   ↓
   输出：updated state
```

---

## 🔧 实用调试技巧

### 1. 生成协方差追踪日志

```c
void log_covariance(FILE* file, const KalmanFilter* kf, int timestamp) {
    fprintf(file, "%d,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            timestamp,
            kf->state.P[0][0],  // P_xx
            kf->state.P[1][1],  // P_yy
            kf->state.P[2][2],  // P_hh
            kf->state.P[3][3],  // P_vv
            kf->state.P[4][4]   // P_ss
    );
}
```

### 2. 可视化协方差椭圆

```python
import numpy as np
import matplotlib.pyplot as plt

def plot_covariance_ellipse(P_xx, P_yy, P_xy, x, y, ax, **kwargs):
    """绘制2D协方差椭圆"""
    
    # 构造2x2协方差矩阵
    cov = np.array([[P_xx, P_xy], [P_xy, P_yy]])
    
    # 特征值和特征向量
    evals, evecs = np.linalg.eigh(cov)
    order = evals.argsort()[::-1]
    evals, evecs = evals[order], evecs[:, order]
    
    # 椭圆参数
    angle = np.degrees(np.arctan2(*evecs[:, 0][::-1]))
    width, height = 4 * np.sqrt(evals)  # 2σ
    
    # 绘制
    ellipse = plt.matplotlib.patches.Ellipse(
        (x, y), width, height, angle=angle, **kwargs
    )
    ax.add_patch(ellipse)
```

### 3. 数值微分验证雅可比

```c
void verify_jacobian(const State* state, double epsilon) {
    // 数值雅可比：J_ij ≈ (f_i(x + e_j) - f_i(x - e_j)) / (2*epsilon)
    
    State state_pert = *state;
    
    for (int j = 0; j < 5; j++) {
        // 计算F[:,j]
        double* x_ptr[] = {&state_pert.x, &state_pert.y, 
                          &state_pert.heading, &state_pert.velocity,
                          &state_pert.steering};
        
        // x + epsilon
        *x_ptr[j] += epsilon;
        State x_plus = state_pert;
        *x_ptr[j] -= epsilon;
        
        // x - epsilon
        *x_ptr[j] -= epsilon;
        State x_minus = state_pert;
        *x_ptr[j] += epsilon;
        
        // 数值导数
        printf("数值J[:,,%d]: ", j);
        printf("[%.6f, %.6f, %.6f, %.6f, %.6f]\n",
               (x_plus.x - x_minus.x) / (2*epsilon),
               (x_plus.y - x_minus.y) / (2*epsilon),
               (x_plus.heading - x_minus.heading) / (2*epsilon),
               (x_plus.velocity - x_minus.velocity) / (2*epsilon),
               (x_plus.steering - x_minus.steering) / (2*epsilon)
        );
    }
}
```

---

## ✅ 检查清单

部署前的完整检查：

- [ ] 协方差矩阵P是正定的
  ```bash
  $ ./check_covariance_positive_definite.py
  ```

- [ ] 雅可比矩阵F在工作范围内有效
  ```bash
  $ ./verify_jacobian_range.py
  ```

- [ ] 测量矩阵H正确映射状态
  ```bash
  $ ./verify_measurement_matrix.py
  ```

- [ ] Q和R的比例合理
  ```bash
  $ 检查Q_norm / R_norm ∈ [0.01, 100]
  ```

- [ ] 初始协方差足够大（允许改进）
  ```bash
  $ P[i][i] ∈ [0.1, 1000]（针对不同状态调整）
  ```

- [ ] 时间步长足够小
  ```bash
  $ dt ≤ 0.01s（100Hz或以上）
  ```

---

**最后更新：2025年12月31日**  
**版本：1.0**  
**状态：可用于生产调试**
