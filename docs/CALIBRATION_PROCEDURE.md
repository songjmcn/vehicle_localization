# 车辆定位系统参数标定流程与理论

## 目录
1. [标定理论基础](#标定理论基础)
2. [参数体系](#参数体系)
3. [标定流程](#标定流程)
4. [理论推导](#理论推导)
5. [实践案例](#实践案例)
6. [标定验证](#标定验证)

---

## 标定理论基础

### 1.1 定位系统模型

车辆定位系统基于**轮速计里程计+IMU融合**的原理：

$$\begin{align}
x_{k+1} &= x_k + v_k \cos(\theta_k) \cdot \Delta t \\
y_{k+1} &= y_k + v_k \sin(\theta_k) \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega_k \cdot \Delta t
\end{align}$$

其中：
- $(x_k, y_k)$：车辆位置
- $\theta_k$：车辆航向角
- $v_k$：纵向速度（来自轮速计）
- $\omega_k$：角速度（来自IMU）
- $\Delta t$：时间间隔

### 1.2 轮速计工作原理

轮速计（编码器）原理：

$$d = \frac{N}{pulses\_per\_rev} \times C_{wheel}$$

其中：
- $N$：脉冲计数增量
- $pulses\_per\_rev$：每圈脉冲数（编码器分辨率）
- $C_{wheel} = 2\pi r$：轮周长
- $r$：轮子半径

**关键问题**：
- 编码器脉冲易受**计数器溢出**影响（0-4095循环）
- 轮胎**打滑/空转**导致脉冲增长但位移不足
- 左右轮差异导致**转向时距离高估**

### 1.3 标定的目标

$$d_{actual} = d_{pulse} \times K_{calib}$$

标定目标是找到校准系数 $K_{calib}$，使算法输出距离最接近**真实位移**。

**验证方式**：
$$error = \frac{|d_{actual} - d_{RTK}|}{d_{RTK}} \times 100\% < 2\%$$

---

## 参数体系

### 2.1 主要参数

| 参数名 | 含义 | 单位 | 范围 | 说明 |
|--------|------|------|------|------|
| `pulses_per_wheel_rev` | 编码器分辨率 | 脉冲/圈 | 48-192 | **硬件参数**，需实测 |
| `wheel_radius` | 轮子半径 | m | 0.3-0.5 | **硬件参数**，需精确测量 |
| `max_count` | 计数器最大值 | - | 4096 | **固件参数**，范围0-4095 |
| `low_speed_threshold` | 低速滤波阈值 | m/s | 0.5-2.0 | 低速平滑处理临界值 |
| `pulse_timeout_ms` | 脉冲超时 | ms | 500-1000 | 静止判定超时 |

### 2.2 参数间的关系

$$\begin{align}
v &= \frac{d}{\Delta t} = \frac{N}{pulses\_per\_rev \times \Delta t} \times 2\pi r \\
\\
d_{total} &= \sum_{i=1}^{k} \frac{N_i}{pulses\_per\_rev} \times 2\pi r \\
\\
accuracy &= 1 - \frac{|d_{calc} - d_{RTK}|}{d_{RTK}}
\end{align}$$

---

## 标定流程

### 3.1 前期准备

#### 步骤1：硬件信息收集

1. **获取编码器规格**
   - 查阅芯片手册：编码器型号及脉冲数/圈
   - 例：`AB Encoder Model: PPR=96`（每圈96脉冲）
   
2. **测量轮子参数**
   - 轮胎半径：直接测量 **静止轮胎外径 ÷ 2**
   - 建议方法：测量至少5个位置的轮胎厚度，取平均值
   - 记录胎压，重复测量（胎压影响半径约1-3mm）
   
3. **检查固件配置**
   - 确认计数器容量：`max_count` 通常为4096（0-4095）
   - 检查计数器溢出处理是否正确

#### 步骤2：数据采集准备

1. **测试场景选择**
   - **直线场景**：笔直道路，长度>200m
   - **标准场景**：混合速度（低速+高速）
   - **转向场景**：弯道和转向（可选）
   
2. **测试设备**
   - RTK-GPS（参考真值）
   - CAN数据记录（vehicle_data.csv）
   - 轮速计原始脉冲数据

### 3.2 数据采集

#### 步骤3：原始数据采集

**采集要求**：
```
采集时间：≥60秒（足够多的样本点）
采样率：≥10Hz（CAN报文频率）
GPS精度：<0.1m（RTK定位精度）
```

**记录项**：
- 时间戳：$t_i$
- 左轮脉冲计数：$RL_i$
- 右轮脉冲计数：$RR_i$
- RTK位置：$(x_{RTK}, y_{RTK})$
- 档位：Gear（D/R）
- 轮向标志：Direction

#### 步骤4：异常数据清理

**关键问题检测**：

1. **脉冲计数器溢出**
   ```python
   # 检测溢出的标志
   if current < previous and (previous - current) > max_count/2:
       delta = (max_count - previous) + current  # 溢出修正
   ```

2. **CAN帧错误（双轮同时异常归零）**
   ```python
   # 如果两轮同时接近0，但上一帧不是接近溢出值
   if curr_RL < 10 and curr_RR < 10:
       if prev_RL > 100 and prev_RR > 100:
           if prev_RL < (max_count-100) and prev_RR < (max_count-100):
               skip_this_frame()  # 标记为异常，跳过
   ```

3. **速度异常**
   ```python
   # 脉冲距离变化与速度不符
   pulse_distance = (curr_count - prev_count) / pulses_per_rev * wheel_circumference
   time_delta = t_curr - t_prev
   implied_speed = pulse_distance / time_delta
   
   if abs(implied_speed - can_speed) > 5.0:  # 大于5 m/s的异常
       mark_as_suspicious()
   ```

### 3.3 参数反演

#### 步骤5：基础参数确定

**已知参数**（硬件手册）：
- $pulses\_per\_rev$：编码器规格（假设96）
- $r$：轮子半径测量值（假设0.35m）
- $max\_count$：计数器容量（通常4096）

**计算轮周长**：
$$C_{wheel} = 2\pi r = 2\pi \times 0.35 = 2.199 \text{ m}$$

#### 步骤6：校准系数求解

**方法1：最小二乘法**

对采集的N个数据点，计算脉冲距离与RTK距离的关系：

$$\min \sum_{i=1}^{N} (d_{pulse,i} - K \cdot d_{RTK,i})^2$$

求解：
$$K = \frac{\sum_{i=1}^{N} d_{pulse,i} \cdot d_{RTK,i}}{\sum_{i=1}^{N} d_{RTK,i}^2}$$

**方法2：简单比值法**

$$K = \frac{\sum_{all} d_{pulse}}{\sum_{RTK} d_{RTK}}$$

其中求和遍历整个采样轨迹。

#### 步骤7：参数调整

**根据标定结果调整**：

| 情况 | 原因 | 调整参数 | 说明 |
|------|------|---------|------|
| 脉冲距离 > RTK距离 | 打滑/转向高估 | $K < 1.0$ | 添加校准系数 |
| 脉冲距离 < RTK距离 | 轮子参数误差 | 增大 $r$ 或 $pulses\_per\_rev$ | 检查硬件参数 |
| 误差不稳定 | 异常帧未清理 | 重复步骤4 | 确保数据质量 |

---

## 理论推导

### 4.1 脉冲距离计算

**基本公式**：
$$d = \frac{N_{pulse}}{pulses\_per\_rev} \times C_{wheel}$$

**差分累计**（逐帧计算）：
$$\Delta d_i = \frac{\Delta N_i}{pulses\_per\_rev} \times 2\pi r$$

**总距离**：
$$d_{total} = \sum_{i=1}^{k} \Delta d_i = \frac{1}{pulses\_per\_rev} \times 2\pi r \times \sum_{i=1}^{k} \Delta N_i$$

### 4.2 左右轮平均处理

**双轮脉冲平均**：
$$N_{avg} = \frac{N_L + N_R}{2}$$

**原因**：
- 消除左右轮转速不一致（转向、轮胎差异）
- 减少单轮打滑的影响
- 提高直线行驶精度

### 4.3 溢出处理

**计数器循环**：
$$N \in [0, max\_count-1]$$

**溢出检测**：
当 $N_{curr} < N_{prev}$ 且 $(N_{prev} - N_{curr}) > \frac{max\_count}{2}$，判定为溢出

**修正**：
$$\Delta N = (max\_count - N_{prev}) + N_{curr}$$

**推导**：
```
例：max_count=4096, 范围0-4095
从 3241 溢出到 0：
  简单计算：0 - 3241 = -3241（错误）
  正确计算：(4096 - 3241) + 0 = 855（正确）
  
验证：3241 + 855 = 4096 ✓
```

### 4.4 校准系数的物理含义

$$K = \frac{d_{actual}}{d_{pulse}}$$

- $K < 1.0$：脉冲高估（轮胎打滑、转向）
- $K = 1.0$：完全标定（理想情况）
- $K > 1.0$：脉冲低估（编码器参数误）

**典型值**：
- 直线行驶：$K \approx 0.95-1.05$
- 含转向：$K \approx 0.90-0.98$
- 低速行驶：$K \approx 0.85-0.95$（打滑多）

---

## 实践案例

### 5.1 本项目的标定历程

#### 问题描述
- **初始输出**：188.15m（距离1.79x过大）
- **RTK参考**：104.84m
- **用户期望**：103.85m
- **编码器规格**：96 pulses/rev
- **轮子半径**：0.35m

#### 标定步骤

**步骤1：识别问题**
```
理论脉冲距离计算：
  总脉冲数（初始算法）：8633
  理论距离：8633 / 96 × 2.199 = 197.76m
  
对比RTK距离：197.76m / 104.84m = 1.89x（问题严重）
```

**步骤2：异常数据检测**
```
发现第1589行异常：两轮同时归零(RL=0, RR=0)
  - 但上一帧：RL=3241, RR=3254
  - 下一帧：RL=3243, RR=3255
  
判定：CAN帧错误，不是真溢出
处理：跳过此帧，继续累加

结果：清理异常后脉冲数 = 4537
```

**步骤3：参数反演**
```
清理后计算：
  脉冲距离 = 4537 / 96 × 2.199 = 103.93m
  RTK距离 = 104.84m
  误差 = 0.87%
  
校准系数：K = 103.85 / 197.76 = 0.525（原始值）
```

**步骤4：代码修复**
```c
// 修复前：双重除2
distance_delta = ((double)pulse_delta / 2.0) / 96 * circumference;  // 减半

// 修复后：正确的平均处理
double pulse_delta = (left_delta + right_delta) / 2.0;  // 平均（Line 189）
distance_delta = (double)pulse_delta / 96 * circumference;  // 直接使用

// 异常检测：忽略双轮同时异常归零的帧
if (both_wheels_near_zero && prev_not_near_overflow) {
    skip_frame();
}
```

**最终结果**：
```
✅ Localization距离：103.98m
✅ RTK距离：104.84m
✅ 误差：0.82%（完美！）
```

### 5.2 标定参数表

```
编码器参数：pulses_per_wheel_rev = 96
轮子参数：wheel_radius = 0.35m
轮周长：C = 2.199m
计数器：max_count = 4096(0-4095)

标定前数据：
  脉冲总数（含异常）：8633
  理论距离：197.76m
  实际距离（RTK）：104.84m
  比例：1.89x ❌

标定后数据：
  脉冲总数（清理异常）：4537
  理论距离：103.93m
  实际距离（RTK）：104.84m
  比例：0.992x ✅
  
校准系数：K = 1.0 ✅（无需额外补偿）
```

---

## 标定验证

### 6.1 验证方法

#### 验证1：直线距离精度
```python
# 比较脉冲距离与RTK轨迹长度
pulse_distance = calculate_from_encoder()
rtk_distance = calculate_trajectory_length(rtk_data)

error = abs(pulse_distance - rtk_distance) / rtk_distance
assert error < 0.02, f"误差过大: {error*100:.2f}%"
```

#### 验证2：转向精度
```python
# 比较航向角
theta_pulse = atan2(y_final - y_init, x_final - x_init)
theta_rtk = atan2(y_rtk_final - y_rtk_init, x_rtk_final - x_rtk_init)

angle_error = abs(theta_pulse - theta_rtk)
assert angle_error < np.radians(5), f"角度误差: {np.degrees(angle_error):.1f}°"
```

#### 验证3：速度精度
```python
# 比较瞬时速度（脉冲计算 vs 微分RTK）
v_pulse = distance_delta / time_delta
v_rtk = np.linalg.norm(rtk_velocity)

velocity_rmse = np.sqrt(np.mean((v_pulse - v_rtk)**2))
assert velocity_rmse < 0.5, f"速度RMSE: {velocity_rmse:.3f} m/s"
```

### 6.2 标定精度指标

| 指标 | 优秀 | 良好 | 可接受 | 不合格 |
|------|------|------|--------|--------|
| 距离误差 | <0.5% | <1% | <2% | >2% |
| 角度误差 | <1° | <2° | <5° | >5° |
| 速度RMSE | <0.2 m/s | <0.5 m/s | <1.0 m/s | >1.0 m/s |
| 数据异常率 | <0.1% | <1% | <5% | >5% |

### 6.3 重新标定条件

需要重新执行标定的条件：
1. **硬件变更**：更换编码器或轮胎
2. **胎压变化**：轮胎胎压变化 >0.5 bar
3. **精度下降**：误差从<1%增长到>3%
4. **新场景**：完全不同的行驶环境（高温/低温）

---

## 标定工具脚本

### 7.1 Python标定脚本示例

```python
#!/usr/bin/env python3
import csv
import math
import numpy as np

def calibrate_wheel_odometry():
    """完整的轮速计标定流程"""
    
    # 参数配置
    PULSES_PER_REV = 96
    WHEEL_RADIUS = 0.35
    WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
    MAX_COUNT = 4096
    OVERFLOW_THRESHOLD = MAX_COUNT // 2
    
    # 读取原始数据
    vehicle_data = read_csv('vehicle_data.csv')
    rtk_data = read_csv('rtk_converted.csv')
    
    # 步骤1：清理异常数据
    print("步骤1：清理异常数据...")
    cleaned_data = remove_anomalies(vehicle_data)
    anomaly_count = len(vehicle_data) - len(cleaned_data)
    print(f"  移除异常帧: {anomaly_count}行")
    
    # 步骤2：计算脉冲距离
    print("步骤2：计算脉冲距离...")
    pulse_distances = []
    total_pulse = 0
    
    for i in range(1, len(cleaned_data)):
        rl_curr = int(cleaned_data[i]['WheelCounter_RL'])
        rl_prev = int(cleaned_data[i-1]['WheelCounter_RL'])
        
        # 处理溢出
        rl_delta = rl_curr - rl_prev
        if rl_curr < rl_prev and (rl_prev - rl_curr) > OVERFLOW_THRESHOLD:
            rl_delta = (MAX_COUNT - rl_prev) + rl_curr
        
        total_pulse += rl_delta
        distance = total_pulse / PULSES_PER_REV * WHEEL_CIRCUMFERENCE
        pulse_distances.append(distance)
    
    final_pulse_distance = pulse_distances[-1]
    print(f"  总脉冲数: {total_pulse}")
    print(f"  脉冲距离: {final_pulse_distance:.2f}m")
    
    # 步骤3：计算RTK距离
    print("步骤3：计算RTK参考距离...")
    rtk_distance = calculate_trajectory_length(rtk_data)
    print(f"  RTK距离: {rtk_distance:.2f}m")
    
    # 步骤4：计算校准系数
    print("步骤4：计算校准系数...")
    K = final_pulse_distance / rtk_distance if rtk_distance > 0 else 1.0
    print(f"  原始系数: K = {K:.4f}")
    
    # 步骤5：验证精度
    print("步骤5：精度验证...")
    error = abs(final_pulse_distance - rtk_distance) / rtk_distance * 100
    print(f"  距离误差: {error:.2f}%")
    
    if error < 2:
        print("  ✅ 标定成功！误差<2%")
        return {'K': K, 'error': error, 'status': 'OK'}
    else:
        print("  ❌ 标定失败，需要调整参数")
        return {'K': K, 'error': error, 'status': 'FAILED'}

def remove_anomalies(data):
    """移除异常数据帧"""
    cleaned = []
    for i in range(len(data)):
        rl = int(data[i]['WheelCounter_RL'])
        rr = int(data[i]['WheelCounter_RR'])
        
        if i == 0:
            cleaned.append(data[i])
            continue
        
        prev_rl = int(data[i-1]['WheelCounter_RL'])
        prev_rr = int(data[i-1]['WheelCounter_RR'])
        
        # 检测异常：两轮同时近零，但上一帧不是接近溢出
        if rl < 10 and rr < 10 and prev_rl > 100 and prev_rr > 100:
            if prev_rl < 4000 and prev_rr < 4000:
                print(f"  发现异常帧 #{i}: RL {prev_rl}->{rl}, RR {prev_rr}->{rr}")
                continue  # 跳过异常帧
        
        cleaned.append(data[i])
    
    return cleaned

def calculate_trajectory_length(data):
    """计算轨迹总长度"""
    total = 0
    for i in range(1, len(data)):
        x_curr = float(data[i]['x'])
        y_curr = float(data[i]['y'])
        x_prev = float(data[i-1]['x'])
        y_prev = float(data[i-1]['y'])
        
        dist = math.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)
        total += dist
    
    return total

# 运行标定
if __name__ == '__main__':
    result = calibrate_wheel_odometry()
    print(f"\n最终结果: {result}")
```

---

## 总结与建议

### 关键要点

1. **硬件参数是基础**
   - 准确的编码器规格和轮子参数至关重要
   - 建议每次测试前重新测量轮子半径（考虑胎压影响）

2. **数据质量决定标定质量**
   - 异常数据（CAN帧错误）必须清理
   - 溢出处理必须正确实现

3. **不同场景需要不同标定**
   - 直线行驶：脉冲距离准确
   - 转向行驶：脉冲距离高估（内外轮速度差）
   - 低速行驶：打滑影响较大

4. **定期验证**
   - 每月验证一次精度
   - 环境变化时（温度、路面）重新标定

### 后续改进方向

- [ ] 实现**环境自适应**：根据速度和转向自动调整K值
- [ ] 添加**轮胎打滑检测**：识别高打滑工况
- [ ] 支持**温度补偿**：考虑胎压随温度变化
- [ ] 多工况标定：分别对低速、高速、转向场景标定

