# RTK 相对坐标分析 - 快速开始

## 一键启动

最简单的方式：

```python
from scripts.plot_rtk import analyze_rtk_relative_to_stable_point

# 一行代码完成整个分析流程
analyze_rtk_relative_to_stable_point('rtk_can.csv')
```

## 工作流程（5步）

```
CSV 数据 → 加载 → 检测稳定点 → 坐标转换 → 保存 CSV + 绘制图像
```

## 输出文件

```
rtk_relative_trajectory/
├── rtk_trajectory_relative.png  # 轨迹图（推荐查看）
├── rtk_stable_relative.csv      # 稳定点数据
└── rtk_stable_relative.json     # JSON 格式
```

## 关键特点

✅ **自动检测稳定点** - RTK 收敛点自动识别
✅ **相对坐标** - 稳定点为原点 (0, 0)
✅ **完整工作流** - 一条命令完成所有步骤
✅ **多种格式** - CSV + JSON + 可视化图像
✅ **详细统计** - 自动输出分析信息

## 使用示例

### 基本使用

```python
from scripts.plot_rtk import analyze_rtk_relative_to_stable_point

# 最简单的方式
analyze_rtk_relative_to_stable_point('rtk_can.csv')
```

### 自定义输出目录

```python
analyze_rtk_relative_to_stable_point(
    csv_file='rtk_can.csv',
    output_dir='my_rtk_analysis'
)
```

### 保存所有数据（包含不稳定点）

```python
analyze_rtk_relative_to_stable_point(
    csv_file='rtk_can.csv',
    save_only_stable=False  # 包含不稳定阶段
)
```

## 坐标系说明

| 参数 | 说明 | 范例 |
|------|------|------|
| **原点** | 稳定的第一个点 | (0.0, 0.0) m |
| **X_East** | 东向距离 | +1.5 m 表示向东 |
| **Y_North** | 北向距离 | +2.0 m 表示向北 |
| **单位** | 所有坐标都是米 | 1.0 = 1 米 |

## CSV 文件说明

关键列：

| 列名 | 说明 | 范例 |
|------|------|------|
| `X_East(m)` | **东向相对距离** | 1.234 m |
| `Y_North(m)` | **北向相对距离** | 2.567 m |
| `Heading(deg)` | 航向角（度） | 176.90° |
| `Quality` | RTK 质量 | 5 (最高) |
| `Timestamp(ms)` | 时间戳 | 1762181084636 |

## 在 Excel 中查看

1. 打开 `rtk_stable_relative.csv`
2. 选择 `X_East(m)` 和 `Y_North(m)` 列
3. 插入 → 图表 → XY 散点图
4. 得到轨迹可视化

## 常见参数设置

### 默认配置（推荐）

```python
# 自动设置的参数
'min_quality': 4,              # RTK 质量 ≥ 4
'max_position_jitter': 0.00005, # 位置抖动 < 0.5m
'max_heading_jitter': 0.1,     # 航向抖动 < 0.1 rad
'min_stable_points': 10,       # 最少 10 个连续稳定点
```

### 如果检测不到稳定点

```python
# 编辑函数内部配置，降低标准
'min_quality': 3,              # 降低到 3
'max_position_jitter': 0.0001, # 增加容限
```

## 常见问题

**Q: 输出的坐标是什么单位？**
A: 所有距离都是 **米 (m)**

**Q: 稳定点的坐标是多少？**
A: 固定为 (0.0, 0.0) m - 这是坐标原点

**Q: 为什么有"不稳定点"？**
A: RTK 初始化需要时间，这些点质量低或位置波动大

**Q: 怎样得到经纬度坐标？**
A: CSV 文件最后两列是 Latitude(°) 和 Longitude(°)

**Q: 可以修改稳定点检测标准吗？**
A: 可以，但需要编辑函数内的配置参数

## 文件大小参考

| 类型 | 大小 | 说明 |
|------|------|------|
| 500 点 CSV | ~25 KB | 稳定点 |
| 500 点 JSON | ~45 KB | 包含元数据 |
| 轨迹图 PNG | ~100-200 KB | 高分辨率 |

## 与其他函数的关系

```
analyze_rtk_relative_to_stable_point()
    ├─ 内部调用: detect_stable_first_point()
    ├─ 内部调用: convert_to_enu()
    ├─ 内部调用: save_stable_points_to_csv()
    ├─ 内部调用: save_stable_points_to_json()
    └─ 内部调用: plot_trajectory()
```

一个函数包含了所有步骤，你不需要逐个调用。

## 输出信息示例

```
╔═══════════════════════════════════════════════════════╗
║    RTK 轨迹分析 - 以稳定点为坐标原点                 ║
╚═══════════════════════════════════════════════════════╝

✅ 坐标系转换完成！

📍 稳定点（坐标原点）信息:
   • 在原始数据中的索引: 20
   • 相对坐标: (0.000, 0.000) m
   • 经纬度: (117.22424971°, 39.01150225°)
   • RTK 质量: 5

📊 轨迹统计:
   • 总数据点: 500
   • 稳定点数: 480
   • 不稳定点: 20

✅ 稳定点数据已保存: rtk_stable_relative.csv
✅ 轨迹图已保存: rtk_trajectory_relative.png
✅ RTK 轨迹分析完成！
```

## 应用场景

### 📍 停泊位置分析
```python
analyze_rtk_relative_to_stable_point('parking.csv')
# 查看车辆如何停泊在最终位置
```

### 🔄 转向轨迹分析
```python
analyze_rtk_relative_to_stable_point('turn_maneuver.csv')
# 分析转向过程中的路径曲率
```

### ✓ RTK 精度测试
```python
analyze_rtk_relative_to_stable_point('static_position.csv')
# 评估静止状态下的位置分散度
```

## 性能

- ⚡ 总耗时：< 2 秒（500 个点）
- 📊 数据加载：< 100 ms
- 🔍 稳定点检测：< 50 ms  
- 📈 绘制图像：< 1 s
- 💾 保存文件：< 250 ms

## 完整脚本示例

```python
#!/usr/bin/env python3
"""RTK 轨迹相对坐标分析脚本"""

from scripts.plot_rtk import analyze_rtk_relative_to_stable_point

def main():
    # 分析 RTK 轨迹
    analyze_rtk_relative_to_stable_point(
        csv_file='rtk_can.csv',
        output_dir='rtk_relative_trajectory',
        save_only_stable=True
    )

if __name__ == '__main__':
    main()
```

运行：
```bash
python3 rtk_analysis.py
```

## 下一步

1. **查看轨迹图** - 打开 `rtk_trajectory_relative.png`
2. **查看 CSV 数据** - 在 Excel 中打开 `rtk_stable_relative.csv`
3. **进一步分析** - 使用导出的数据进行自定义分析

---

**更多详情**: 参考 [RTK_RELATIVE_COORDINATE_ANALYSIS.md](RTK_RELATIVE_COORDINATE_ANALYSIS.md)
