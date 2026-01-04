# 转向角增益标定理论基础

## 1. 基本概念

### 1.1 转向角增益定义

转向角增益（Steering Angle Gain）是车辆实际响应的转向角与期望转向角之间的比例关系：

$$\text{gain} = \frac{\delta_{\text{actual}}}{\delta_{\text{command}}}$$

理想情况下，增益 = 1.0。但在实际系统中，由于：
- 转向系统的机械特性不匹配
- 控制器的标定不准确
- 传感器的零偏和缩放因子误差
- 时变特性和温度影响

实际增益可能与1.0偏离，造成航向控制误差。

### 1.2 增益误差的影响

设理想的转向角为 $\delta$，实际转向角为 $\delta' = \text{gain} \cdot \delta$

在自行车模型中，横摆角速度为：
$$\omega = \frac{v}{L} \tan(\delta')$$

当增益偏离1.0时：
$$\Delta\omega = \frac{v}{L} [\tan(\text{gain} \cdot \delta) - \tan(\delta)]$$

这导致航向角积分误差：
$$\Delta\psi = \int \Delta\omega \, dt$$

## 2. 增益估计理论

### 2.1 基于航向角误差的线性关系

航向角误差与转向角增益的关系可近似为线性：

$$\Delta\psi = k \cdot (\text{gain} - 1.0) \cdot \delta \cdot v + \text{noise}$$

其中：
- $k$ 是与车型相关的常数（通常 $k \approx 1/L$，L为轴距）
- $\delta$ 是转向角命令
- $v$ 是车速
- noise 是传感器噪声和模型不确定性

### 2.2 反演估计

如果已知航向角误差 $\Delta\psi$ 和相关的转向角、车速，可反演估计增益：

$$\text{gain} \approx 1.0 + \frac{\Delta\psi}{k \cdot \delta \cdot v}$$

但由于存在噪声和多种误差源，需要使用加权最小二乘法综合所有数据。

### 2.3 最小二乘法框架

定义观测方程：
$$\Delta\psi_i = a + b \cdot \delta_i \cdot v_i + \epsilon_i$$

其中：
- $a$ 是截距项（应接近0）
- $b$ 与增益偏差相关：$b = k \cdot (\text{gain} - 1.0)$
- $\epsilon_i$ 是观测噪声

加权最小二乘目标函数：
$$J = \sum_{i=1}^{n} w_i \cdot (\Delta\psi_i - a - b \cdot \delta_i \cdot v_i)^2$$

最小化此函数可得 $a$ 和 $b$ 的最优估计，进而得到增益估计。

## 3. 权重因子设计

### 3.1 权重的多维考虑

为了提升标定精度，权重应综合考虑多个因素：

$$w_i = w_{\text{quality}} \times w_{\text{velocity}} \times w_{\text{continuity}}$$

#### 3.1.1 质量权重（Quality Weight）

基于RTK位置和速度的测量质量：
$$w_{\text{quality}} = \left(\frac{q_i}{q_{\max}}\right)^2$$

其中 $q_i$ 是RTK质量等级（0-5），$q_{\max} = 5$。

**理由**：高质量测量的可靠性显著高于低质量测量，故使用平方权重。

#### 3.1.2 速度权重（Velocity Weight）

基于车速的有效性：
$$w_{\text{velocity}} = \begin{cases}
0.3 & \text{if } v < 0.5 \text{ m/s} \quad \text{(低速，噪声主导)} \\
1.0 & \text{if } 0.5 \leq v < 5.0 \text{ m/s} \quad \text{(正常范围)} \\
0.8 & \text{if } v \geq 5.0 \text{ m/s} \quad \text{(高速，非线性增强)}
\end{cases}$$

#### 3.1.3 轨迹连续性权重（Continuity Weight）

基于相邻点之间的距离：
$$w_{\text{continuity}} = \log(1 + d_i)$$

其中 $d_i$ 是相邻RTK点的距离（米）。

**理由**：轨迹越连续，位置估计的可靠性越高。

#### 3.1.4 不确定性修正（Uncertainty Correction）

引入反向不确定性权重：
$$w_{\text{uncertainty}} = \frac{1}{q_i + 0.1}$$

综合权重最终表达式：
$$w_i = \left(\frac{q_i}{5}\right)^2 \times v_{\text{weight}} \times \log(1 + d_i) \times \frac{1}{q_i + 0.1}$$

### 3.2 权重的正规化

为避免权重过度膨胀，应进行正规化：
$$\tilde{w}_i = \frac{w_i}{\sum_{j=1}^{n} w_j}$$

## 4. 离群检测

### 4.1 使用MAD算法

对于转向-航向关系的离群检测，使用中位数绝对偏差（MAD）：

1. 计算所有 $\delta_i \cdot v_i$ 值的拟合残差：
   $$r_i = \Delta\psi_i - (\text{median}(\Delta\psi) + \text{slope} \cdot \delta_i \cdot v_i)$$

2. 计算残差的MAD：
   $$\text{MAD} = \text{median}(|r_i - \text{median}(r)|)$$

3. 标记离群点：
   $$|r_i - \text{median}(r)| > 2.5 \times \text{MAD}$$

### 4.2 离群点处理

- **轻度离群**（$2.5 < z < 3.5 \times \text{MAD}$）：权重衰减至0.5
- **重度离群**（$z > 3.5 \times \text{MAD}$）：完全排除（权重=0）

## 5. 特殊场景处理

### 5.1 低速场景（v < 0.5 m/s）

在低速时：
- 航向误差主要来自传感器噪声而非增益偏差
- 减少此场景的权重：$w_{\text{velocity}} = 0.3$
- 仍然纳入计算，但影响较小

### 5.2 高速场景（v > 5.0 m/s）

在高速时：
- 转向角增益的影响被放大
- 增加权重但考虑非线性：$w_{\text{velocity}} = 0.8$
- 后续可扩展为多项式增益模型

### 5.3 静止或极低速

- 当 $v < 0.1$ m/s 时，完全忽略该数据点
- 防止被零除错误和传感器漂移

## 6. 估计精度指标

### 6.1 拟合优度（R²）

$$R^2 = 1 - \frac{\sum w_i(y_i - \hat{y}_i)^2}{\sum w_i(y_i - \bar{y})^2}$$

其中 $\hat{y}_i$ 是预测值，$\bar{y}$ 是加权平均。

### 6.2 加权标准差（Weighted Standard Deviation）

$$\sigma_w = \sqrt{\frac{\sum w_i(y_i - \hat{y}_i)^2}{(n-1) \sum w_i / n}}$$

### 6.3 置信度指标

综合指标：
$$\text{confidence} = \min\left(\frac{R^2}{\text{threshold}}, 1.0\right) \times \frac{n_{\text{valid}}}{n}$$

其中：
- $n_{\text{valid}}$ 是有效（非离群）数据点数
- $n$ 是总数据点数
- threshold 通常设为 0.85

## 7. 数据结构定义

### 7.1 单点结构

```c
typedef struct {
    double timestamp;           // 时间戳 (秒)
    double steering_angle;      // 转向角命令 (弧度)
    double heading_error;       // 航向角误差 (弧度)
    double velocity;            // 车速 (m/s)
    double rtk_quality;         // RTK质量等级 (0-5)
    double trajectory_distance; // 轨迹距离 (米)
    uint8_t is_outlier;         // 是否为离群点 (0/1)
    double weight;              // 计算的权重
} SteeringGainCalibrationPoint_t;
```

### 7.2 管理结构

```c
typedef struct {
    SteeringGainCalibrationPoint_t points[MAX_CALIBRATION_POINTS];
    uint32_t count;             // 当前点数
    
    // 估计结果
    double estimated_gain;      // 估计的增益值
    double gain_bias;           // 增益偏差 (gain - 1.0)
    double gain_uncertainty;    // 增益不确定度 (±σ)
    
    // 质量指标
    double r_squared;           // R² 拟合优度
    double weighted_std_dev;    // 加权标准差
    double confidence;          // 置信度 (0-1)
    
    // 统计信息
    uint32_t valid_points;      // 有效数据点数
    uint32_t outlier_count;     // 离群点数
    double avg_weight;          // 平均权重
} SteeringGainManager_t;
```

## 8. 算法流程

### 8.1 标定流程

```
1. 数据收集
   └─ 记录 (转向角, 航向误差, 速度, RTK质量, 轨迹)
   
2. 数据预处理
   ├─ 检查速度有效性 (v > 0.1 m/s)
   ├─ 检查转向角范围 (-π/4 ~ π/4)
   └─ 检查轨迹连续性
   
3. 权重计算
   ├─ 质量权重: (q/5)²
   ├─ 速度权重: 基于场景
   ├─ 连续性权重: log(1+d)
   └─ 正规化加总
   
4. 离群检测 (MAD算法)
   ├─ 初步拟合
   ├─ 计算MAD
   └─ 标记离群点
   
5. 最小二乘估计
   ├─ 加权WLS求解
   ├─ 计算 R²
   └─ 计算置信度
   
6. 结果输出
   └─ 增益值, 不确定度, 质量指标
```

## 9. 与Phase 1的集成

### 9.1 数据来源

Phase 1 (P0+) 提供的校准数据：
- **轮速计校准因子** → 改进的里程计精度
- **转向零偏** → 准确的转向角基准
- **质量权重框架** → 沿用权重计算方法
- **MAD离群检测** → 复用离群点识别

### 9.2 集成点

```
RTK标定系统 (P0+)
    ├─ 轮速计校准 ✓
    ├─ 转向零偏 ✓
    └─ [新增] 转向增益估计 (P1)
        ↓
    生成综合 CalibrationParameters_t
        ↓
    应用到轨迹估计和转向模型
```

## 10. 后续扩展

### 10.1 非线性增益模型

当前为线性模型。可扩展为多项式：
$$\text{gain}(\delta) = a_0 + a_1 \delta + a_2 \delta^2$$

### 10.2 时变增益模型

若环境温度、轮胎气压等时变因素影响显著，可引入：
$$\text{gain}(t) = g_0 + k_T \cdot T(t) + k_P \cdot P(t)$$

### 10.3 自适应在线更新

Phase 3+: 利用卡尔曼滤波实时更新增益参数。

---

## 参考资源

- 自行车模型: Kiencke & Nielsen (2005)
- 鲁棒统计: Huber & Ronchetti (2009)
- 加权最小二乘: Wikipedia - Least Squares

**文档版本**: 1.0  
**最后更新**: 2026年1月4日
