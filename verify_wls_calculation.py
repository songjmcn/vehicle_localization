"""
验证加权最小二乘法的数值计算
"""
import numpy as np

# 测试数据：前8段factor=1，第9段异常
odo_dists = np.array([1, 1, 1, 1, 1, 1, 1, 1, 92.0])
rtk_dists = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1.0])

# 权重（假设质量都是4/5=0.8，轨迹长度为rtk_dists）
weights = 0.8 * rtk_dists

print("=== 不做任何离群点处理 ===")
# WLS: factor = sum(w * odo * rtk) / sum(w * rtk²)
numerator = np.sum(weights * odo_dists * rtk_dists)
denominator = np.sum(weights * rtk_dists ** 2)
factor_all = numerator / denominator
print(f"Factor (all): {factor_all:.4f}")
print(f"  numerator: {numerator:.2f}")
print(f"  denominator: {denominator:.2f}")

print("\n=== 排除第9段（离群点） ===")
# 只用前8段
mask = np.array([True] * 8 + [False])
weights_inlier = weights[mask]
odo_inlier = odo_dists[mask]
rtk_inlier = rtk_dists[mask]

numerator = np.sum(weights_inlier * odo_inlier * rtk_inlier)
denominator = np.sum(weights_inlier * rtk_inlier ** 2)
factor_inlier = numerator / denominator
print(f"Factor (no outlier): {factor_inlier:.4f}")

print("\n=== 残差分析 ===")
residuals = odo_dists - factor_all * rtk_dists
print(f"Residuals with factor={factor_all:.4f}:")
for i, r in enumerate(residuals):
    print(f"  段{i}: {r:8.2f}")

print(f"\nMean residual: {np.mean(residuals):.4f}")
print(f"Std dev: {np.std(residuals, ddof=1):.4f}")
print(f"Max |residual|: {np.max(np.abs(residuals)):.4f}")

# 鲁棒初始估计
ratios = odo_dists / rtk_dists
print(f"\n=== 比例分析 ===")
print(f"Ratios: {ratios}")
ratios_trimmed = ratios[(ratios >= 0.5) & (ratios <= 2.0)]
factor_trimmed = np.mean(ratios_trimmed)
print(f"Trimmed mean (0.5-2.0): {factor_trimmed:.4f}")

# 用trimmed factor计算残差
residuals_trimmed = odo_dists - factor_trimmed * rtk_dists
print(f"\nResiduals with factor={factor_trimmed:.4f}:")
for i, r in enumerate(residuals_trimmed):
    marker = "**OUTLIER**" if abs(r) > 5.0 else ""
    print(f"  段{i}: {r:8.2f}  {marker}")
