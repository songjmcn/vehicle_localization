# 坐标系与轨迹可视化指南

## 📋 概述

本文档解释了轨迹可视化中的坐标系重新定义，以及如何正确理解和调试定位系统的输出。

---

## 🎯 坐标系定义

### 1. 全局参考坐标系（ISO 8855）

车辆定位系统采用**ISO 8855国际标准自行车模型**，定义如下：

#### 坐标原点
**后轮轴中心（RAC）**，即车辆后轴两轮的接触点中间。

#### 坐标轴方向（全局）
```
         North
           ↑  Y轴
           |
     ← ←---O---→ →  X轴
     West  |    East
           |
         South
```

| 轴 | 正方向 | 车辆理解 | 含义 |
|----|--------|---------|------|
| X | 向东 | 向前 | 前进时X增加 |
| Y | 向北 | 向左 | 左转时Y增加 |
| ψ | 逆时针 | 绕Z轴 | 航向角，0°=东，90°=北 |

#### 数学表示

状态向量：
$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \psi \\ v \\ \delta \end{bmatrix}$$

其中：
- $(x, y)$ - 全局位置（后轮轴中心）
- $\psi$ - 航向角（从东方向逆时针）
- $v$ - 纵向速度（可正可负）
- $\delta$ - 转向角

### 2. 实际测试场景中的坐标系

```
        起点方向
            ↑
            | 
  RTK轨迹  |  定位轨迹
    X轴   |
←--------o--------→ 
        Y轴
```

**关键：** RTK原始数据的坐标系与定位系统不对齐。

---

## 🔄 坐标轴重新定义（可视化坐标系）

### 修改原因

定位算法（motion_model.c）的运动方程：
```c
x_new = x + v * cos(heading) * dt;
y_new = y + v * sin(heading) * dt;
```

这遵循严格的**数学标准直角坐标系**：
- X轴：向右（东）
- Y轴：向上（北）
- 原点在0,0

但在实验场景中，RTK设备的起始位置不在原点，导致可视化时轨迹偏离。

### 新的可视化坐标系

为了使轨迹图更易理解，我们进行坐标变换：

#### 变换公式

```python
# 将ISO标准坐标 (x, y) 变换为可视化坐标
x_visual = -y_standard      # Y轴反向映射到水平轴
y_visual =  x_standard      # X轴映射到竖直轴
```

#### 几何意义

```
ISO 8855标准坐标：          可视化坐标：
        North               (看向北)
         ↑ Y                    ↑ X
         |                      |
    ←----o----→ X          ← ---o--- → Y
         |                      |
       South                  (南侧)
       
原点：(0, 0)                原点：(0, 0)
X增加：向东                  X增加：向上（北）
Y增加：向北                  Y增加：向西（左侧负数）
```

### 变换表格

| 标准坐标点 | 可视化坐标点 | 说明 |
|-----------|-----------|------|
| (0, 0) | (0, 0) | 原点不变 |
| (1, 0) | (0, 1) | 向东1m → 向北（向上）1m |
| (0, 1) | (-1, 0) | 向北1m → 向西（向左）1m |
| (1, 1) | (-1, 1) | 东北方向 → 向北向左对角 |
| (-1, 0) | (0, -1) | 向西1m → 向南（向下）1m |
| (0, -1) | (1, 0) | 向南1m → 向东（向右）1m |

---

## 📊 可视化坐标轴含义

### 新坐标系的物理含义

#### Y轴（水平）
```
← 向西（左正）Y ← 向东（右负）→ 
```

- **左侧（负数）：** 东方（前进时向右）
- **右侧（正数）：** 西方（前进时向左）
- **名称：** "Y坐标 (m) [左正右负]"

#### X轴（竖直）
```
           ↑ 向北（北正）X
           | 
           |
  →→→→ 向东（东正）
           |
           ↓ 向南（南负）X
```

- **向上（正数）：** 北方（左转时向上）
- **向下（负数）：** 南方（右转时向下）
- **名称：** "X坐标 (m) [纵轴，向上为北]"

### 为什么这样改？

```
原始轨迹（ISO 8855）：
    100
     |
    50 |  RTK ××
       |        ×
      0 +------+-----
        -50  0  50  100  X

问题：轨迹散落在象限各处，难以比较
     RTK起点: (-2.72, 3.23)
     定位起点: (0, 0)

变换后（可视化）：
        100 ↑ X（北）
         |  ×RTK
       50|   × 定位轨迹
         |×××   
        0+--------→ -Y（西）
         |
      -50|

优点：轨迹紧凑，易于叠加和比较
     都从左上角开始，方便观察误差
```

---

## 🔧 代码实现

### Python脚本修改（compare_rtk_localizaiton.py）

#### 修改1：plot_trajectory_overlay()

**修复前：**
```python
def plot_trajectory_overlay(self):
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(self.rtk_df['x'], self.rtk_df['y'], 'o-', label='RTK')
    ax.plot(self.loc_df['x'], self.loc_df['y'], 's-', label='Localization')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
```

**修复后：**
```python
def plot_trajectory_overlay(self):
    fig, ax = plt.subplots(figsize=(12, 8))
    # 坐标变换：(x, y) → (-y, x)
    ax.plot(-self.rtk_df['y'], self.rtk_df['x'], 'o-', label='RTK')
    ax.plot(-self.loc_df['y'], self.loc_df['x'], 's-', label='Localization')
    ax.set_xlabel('Y 坐标 (m) [左正右负]')
    ax.set_ylabel('X 坐标 (m) [纵轴]')
```

#### 修改2：plot_trajectory_overlay_with_raw_rtk()

```python
# RTK轨迹
ax.plot(-self.raw_rtk_df['y'], self.raw_rtk_df['x'], 
        'o', markersize=3, alpha=0.5, label='Raw RTK')
# 定位轨迹
ax.plot(-self.loc_df['y'], self.loc_df['x'], 
        's-', markersize=4, label='Localization')
```

#### 修改3：plot_error_map()

```python
# 散点图显示误差
for i in range(len(df)):
    x_std = -df['y'].iloc[i]      # Y坐标反向映射
    y_std = df['x'].iloc[i]        # X坐标映射
    dx = -df['delta_y'].iloc[i]    # 误差也变换
    dy = df['delta_x'].iloc[i]
    ax.scatter(x_std, y_std, ...)
    ax.quiver(x_std, y_std, dx, dy, ...)
```

---

## 📐 坐标变换的矩阵表示

### 旋转矩阵

将坐标 $(x, y)$ 变换为可视化坐标 $(x_{vis}, y_{vis})$：

$$\begin{bmatrix} x_{vis} \\ y_{vis} \end{bmatrix} = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} -y \\ x \end{bmatrix}$$

这是一个**逆时针旋转90°**的操作。

### 向量变换

速度向量 $(v_x, v_y)$ 的变换：

$$\begin{bmatrix} v_{x,vis} \\ v_{y,vis} \end{bmatrix} = \begin{bmatrix} -v_y \\ v_x \end{bmatrix}$$

### 航向角变换

原航向角 $\psi$ 变换为可视化航向角 $\psi_{vis}$：

$$\psi_{vis} = \psi + 90° = \psi + \frac{\pi}{2}$$

这是因为坐标系旋转了90°。

---

## ✅ 验证与测试

### 测试点验证

| 运动 | 标准坐标 | 可视化坐标 | 视觉检查 |
|-----|--------|----------|---------|
| 向前100m | (100, 0) | (0, 100) | 向上 ✓ |
| 向左50m | (0, 50) | (-50, 0) | 向左 ✓ |
| 向后100m | (-100, 0) | (0, -100) | 向下 ✓ |
| 向右50m | (0, -50) | (50, 0) | 向右 ✓ |

### 航向角验证

| 初始航向 | 运动 | 标准坐标变化 | 可视化坐标变化 | 一致性 |
|---------|-----|-----------|-------------|--------|
| 0° | 直行 | Δx>0, Δy≈0 | Δx_vis>0, Δy_vis≈0 | ✓ |
| 90° | 直行 | Δx≈0, Δy>0 | Δx_vis<0, Δy_vis≈0 | ✓ |
| 180° | 直行 | Δx<0, Δy≈0 | Δx_vis<0, Δy_vis≈0 | ✓ |

---

## 🔍 调试指南

### 如果轨迹方向反了

检查变换公式：
```python
# ❌ 错误
x_vis = y_std
y_vis = -x_std

# ✅ 正确
x_vis = -y_std
y_vis = x_std
```

### 如果误差箭头指向相反

检查误差向量的变换：
```python
# ❌ 错误
dx_vis = delta_x
dy_vis = delta_y

# ✅ 正确（与坐标变换一致）
dx_vis = -delta_y
dy_vis = delta_x
```

### 如果轨迹重合度很差

可能原因：
1. **坐标原点不对齐** - RTK起点不是(0,0)
2. **初始航向角偏差** - 需要校准
3. **算法本身的误差** - 检查雅可比矩阵、协方差等

### 调试步骤

```bash
# 1. 运行定位
./build/bin/localization_example > log.txt

# 2. 生成可视化
python3 scripts/compare_rtk_localizaiton.py

# 3. 检查结果
# 查看：
#   - 两条轨迹是否在同一象限
#   - 相对位置是否合理
#   - 误差向量方向是否正确

# 4. 如果不合理，检查：
#   - 变换公式 (x_vis, y_vis) = (-y, x) 是否应用
#   - 坐标原点对齐
#   - RTK数据的坐标系定义
```

---

## 📚 相关文件

| 文件 | 用途 |
|------|------|
| `scripts/compare_rtk_localizaiton.py` | 主可视化脚本 |
| `src/motion_model.c` | 运动方程（使用ISO 8855坐标） |
| `matlab/visualization/` | MATLAB版本的可视化 |
| `docs/COORDINATE_SYSTEM_SPECIFICATION.md` | ISO 8855详细规范 |

---

## 🎓 学习资源

### ISO 8855标准自行车模型
- 原点：后轮轴中心
- 坐标系：东北天（X东，Y北）
- 航向角：从东方向逆时针

### 常见坐标系混淆

| 系统 | X轴 | Y轴 | 原点 |
|-----|-----|-----|------|
| 数学 | 向右 | 向上 | 左下 |
| 图像 | 向右 | 向下 | 左上 |
| 车辆(ISO 8855) | 向东 | 向北 | 车后轮中心 |
| 可视化(本项目) | 向北(竖直) | 向西(水平) | 起点 |

---

**最后更新：2025年12月31日**  
**版本：1.0**  
**状态：已验证**
