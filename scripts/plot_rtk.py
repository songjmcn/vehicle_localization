import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import numpy as np
import csv
import os
import platform
from typing import List, Tuple, Dict, Optional

# 检测系统并设置中文字体
system = platform.system()
if system == 'Darwin':  # macOS
    font_list = [
        'Arial Unicode MS',
        'Songti SC',
        'Heiti SC',
        'PingFang SC',
        'SimHei',
        'DejaVu Sans'
    ]
elif system == 'Windows':
    font_list = [
        'SimHei',
        'Microsoft YaHei',
        'SimSun',
        'Arial',
        'DejaVu Sans'
    ]
else:  # Linux
    font_list = [
        'DejaVu Sans',
        'WenQuanYi Micro Hei',
        'SimHei',
        'Arial'
    ]

# 设置字体
plt.rcParams['font.sans-serif'] = font_list
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['font.size'] = 10

# 尝试刷新字体缓存
try:
    import matplotlib.font_manager
    if hasattr(matplotlib.font_manager, '_rebuild'):
        matplotlib.font_manager._rebuild()
    elif hasattr(matplotlib.font_manager, 'rebuild'):
        matplotlib.font_manager.rebuild()
except Exception:
    pass

class RTKTrajectoryPlotter:
    """
    RTK轨迹可视化模块（支持List数据格式 + 稳定点检测）
    
    功能：
    1. 接收x, y, heading的List数据
    2. 检测稳定的第一个点（RTK收敛点）
    3. 自动转换为相对坐标（以稳定点为原点）
    4. 绘制轨迹、方向、统计信息
    """
    
    def __init__(self):
        """初始化RTK轨迹绘制器"""
        self.raw_x = []          # 原始x列表（经度或东向）
        self.raw_y = []          # 原始y列表（纬度或北向）
        self.raw_heading = []    # 原始heading列表（弧度）
        self.timestamps = []     # 时间戳列表
        self.quality = []        # RTK质量列表
        
        self.trajectory = []     # 转换后的轨迹
        self.ref_x = None        # 参考点x
        self.ref_y = None        # 参考点y
        self.ref_index = 0       # 参考点索引
        self.is_latlon = False   # 是否为经纬度坐标
        
        # 稳定点检测参数
        self.stability_config = {
            'min_quality': 4,                    # 最小质量等级
            'max_position_jitter': 0.5,         # 最大位置抖动（米）
            'max_heading_jitter': 0.1,          # 最大航向抖动（弧度）
            'min_stable_points': 10,            # 最少连续稳定点数
            'check_window_size': 20             # 检测窗口大小
        }
    
    def load_from_lists(self, 
                       x_list: List[float],
                       y_list: List[float],
                       heading_list: List[float],
                       timestamps_list: Optional[List[int]] = None,
                       quality_list: Optional[List[int]] = None,
                       is_latlon: bool = False) -> None:
        """
        从列表加载RTK数据
        
        Args:
            x_list: x坐标列表
                   - 若is_latlon=True: 经度 (°)
                   - 若is_latlon=False: 东向距离 (m)
            y_list: y坐标列表
                   - 若is_latlon=True: 纬度 (°)
                   - 若is_latlon=False: 北向距离 (m)
            heading_list: 航向角列表 (rad)
            timestamps_list: 时间戳列表 (ms)，可选
            quality_list: RTK质量列表 (0-5)，可选
            is_latlon: 是否为经纬度坐标 (默认False=ENU)
        
        Returns:
            None
        """
        
        if not x_list or not y_list or not heading_list:
            print("ERROR: Empty input lists!")
            return
        
        if not (len(x_list) == len(y_list) == len(heading_list)):
            print(f"ERROR: List length mismatch! x:{len(x_list)}, y:{len(y_list)}, heading:{len(heading_list)}")
            return
        
        self.raw_x = list(x_list)
        self.raw_y = list(y_list)
        self.raw_heading = list(heading_list)
        self.is_latlon = is_latlon
        
        # 处理时间戳（如果不提供，自动生成）
        if timestamps_list:
            self.timestamps = list(timestamps_list)
        else:
            # 自动生成时间戳（50ms间隔）
            self.timestamps = [i * 50 for i in range(len(x_list))]
        
        # 处理质量（如果不提供，默认为5）
        if quality_list:
            self.quality = list(quality_list)
        else:
            self.quality = [5] * len(x_list)
        
        print(f"✓ Loaded {len(x_list)} RTK points")
        print(f"  Coordinate system: {'WGS84经纬度' if is_latlon else 'ENU笛卡尔坐标'}")
        print(f"  X range: [{min(x_list):.4f}, {max(x_list):.4f}]")
        print(f"  Y range: [{min(y_list):.4f}, {max(y_list):.4f}]")
    
    def set_stability_config(self, config: Dict) -> None:
        """
        设置稳定点检测参数
        
        Args:
            config: 配置字典，支持的键：
                - min_quality: 最小质量等级 (0-5)
                - max_position_jitter: 最大位置抖动 (m)
                - max_heading_jitter: 最大航向抖动 (rad)
                - min_stable_points: 最少连续稳定点数
                - check_window_size: 检测窗口大小
        """
        for key, value in config.items():
            if key in self.stability_config:
                self.stability_config[key] = value
                print(f"✓ Set {key} = {value}")
    
    def load_from_csv(self, csv_file: str, is_latlon: bool = True) -> None:
        """
        从CSV文件加载RTK数据（支持 rtk_can.csv 格式）
        
        rtk_can.csv 格式:
            RTK_x;RTK_y;RTK_z;RTK_heading;RTK_qualit;timestamp
            39.01150225;117.22424971;0;177.37;5;1762180984.6362047
        
        其中：
            RTK_x: 纬度 (°)
            RTK_y: 经度 (°)
            RTK_z: 高度 (m)
            RTK_heading: 航向角 (°)
            RTK_qualit: RTK质量等级 (0-5)
            timestamp: 时间戳 (秒)
        
        Args:
            csv_file: CSV文件路径
            is_latlon: 是否为经纬度坐标 (默认True，因为rtk_can使用经纬度)
        
        Returns:
            None
        """
        
        if not os.path.exists(csv_file):
            print(f"ERROR: CSV file not found: {csv_file}")
            return
        
        print(f"📂 加载RTK数据文件: {csv_file}")
        
        x_list = []
        y_list = []
        heading_list = []
        timestamps_list = []
        quality_list = []
        
        try:
            with open(csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f, delimiter=',')
                
                for row_idx, row in enumerate(reader):
                    try:
                        # 提取数据 (处理可能的空格)
                        lat = float(row['RTK_x'].strip())           # 纬度
                        lon = float(row['RTK_y'].strip())           # 经度
                        heading_deg = float(row['RTK_heading'].strip())  # 航向角（度）
                        quality = int(float(row['RTK_qualit'].strip()))  # 质量
                        timestamp = float(row['timestamp'].strip())      # 时间戳（秒）
                        
                        # 转换航向角从度到弧度
                        heading_rad = np.radians(heading_deg)
                        
                        # 时间戳从秒转换为毫秒
                        timestamp_ms = int(timestamp * 1000)
                        
                        x_list.append(lon)  # X = 经度
                        y_list.append(lat)  # Y = 纬度
                        heading_list.append(heading_rad)
                        timestamps_list.append(timestamp_ms)
                        quality_list.append(quality)
                        
                    except (ValueError, KeyError) as e:
                        print(f"WARNING: 第 {row_idx + 2} 行数据格式错误: {e}")
                        continue
            
            if not x_list:
                print(f"ERROR: No valid data found in CSV file!")
                return
            
            # 加载数据
            self.load_from_lists(
                x_list=x_list,
                y_list=y_list,
                heading_list=heading_list,
                timestamps_list=timestamps_list,
                quality_list=quality_list,
                is_latlon=is_latlon
            )
            
            print(f"✓ CSV 数据加载完成，共 {len(x_list)} 个有效数据点")
            
        except Exception as e:
            print(f"ERROR: Failed to read CSV file: {e}")
            import traceback
            traceback.print_exc()
    
    def load_from_rtk_converted_csv(self, csv_file: str) -> None:
        """
        从 rtk_converted.csv 文件加载RTK数据（优化后的轨迹）
        
        文件格式:
            x;y;heading;heading_rad;z;quality;timestamp
            0.0;0.0;-0.0;-0.0;0.0;5;1761920619.139165
        
        坐标系统：ISO 8855 车体坐标系
            - X 轴：纵轴（前向），单位米
            - Y 轴：横轴（左向为正），单位米
            - heading_rad：航向角（弧度），[-π, π]
            - 质量等级：0-5
            - 时间戳：秒
        
        Args:
            csv_file: CSV文件路径
        
        Returns:
            None
        """
        
        if not os.path.exists(csv_file):
            print(f"ERROR: CSV file not found: {csv_file}")
            return
        
        print(f"📂 加载RTK转换数据文件: {csv_file}")
        print(f"   坐标系统: ISO 8855车体坐标系")
        print(f"   X轴: 纵轴(前向), Y轴: 横轴(左向为正)")
        
        x_list = []
        y_list = []
        heading_list = []
        timestamps_list = []
        quality_list = []
        
        try:
            with open(csv_file, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f, delimiter=';')
                
                for row_idx, row in enumerate(reader):
                    try:
                        # 提取数据 (处理可能的空格)
                        # 注意：rtk_converted.csv 已经在车体坐标系中
                        x = float(row['x'].strip())                    # X轴（纵轴，前向）
                        y = float(row['y'].strip())                    # Y轴（横轴，左向为正）
                        heading_rad = float(row['heading_rad'].strip())  # 航向角（弧度）
                        quality = int(float(row['quality'].strip()))    # 质量
                        timestamp = float(row['timestamp'].strip())     # 时间戳（秒）
                        
                        # 时间戳从秒转换为毫秒
                        timestamp_ms = int(timestamp * 1000)
                        
                        x_list.append(x)
                        y_list.append(y)
                        heading_list.append(heading_rad)
                        timestamps_list.append(timestamp_ms)
                        quality_list.append(quality)
                        
                    except (ValueError, KeyError) as e:
                        print(f"WARNING: 第 {row_idx + 2} 行数据格式错误: {e}")
                        continue
            
            if not x_list:
                print(f"ERROR: No valid data found in CSV file!")
                return
            
            # 加载数据（is_latlon=False 表示使用笛卡尔坐标，不需要进行经纬度转换）
            self.load_from_lists(
                x_list=x_list,
                y_list=y_list,
                heading_list=heading_list,
                timestamps_list=timestamps_list,
                quality_list=quality_list,
                is_latlon=False  # rtk_converted.csv 已经是笛卡尔坐标
            )
            
            print(f"✓ RTK转换数据加载完成，共 {len(x_list)} 个有效数据点")
            
        except Exception as e:
            print(f"ERROR: Failed to read CSV file: {e}")
            import traceback
            traceback.print_exc()
    
    def detect_stable_first_point(self) -> Tuple[int, Dict]:
        """
        检测稳定的第一个点（RTK收敛点）
        
        算法：
        1. 扫描前N个点，检查质量
        2. 计算相邻点之间的距离和航向变化
        3. 找到第一个稳定区间（连续M个点都满足稳定条件）
        4. 该区间的起点作为参考点
        
        Returns:
            (stable_index, info_dict): 
                - stable_index: 稳定点的索引
                - info_dict: 稳定点的详细信息
        """
        
        print("\n╔════════════════════════════════════════════════════╗")
        print("║        RTK稳定点检测（初始化点）                     ║")
        print("╚════════════════════════════════════════════════════╝\n")
        
        if len(self.raw_x) < self.stability_config['check_window_size']:
            print("WARNING: Data too short for stability detection, using first point")
            return 0, self._get_point_info(0)
        
        x_coords = np.array(self.raw_x)
        y_coords = np.array(self.raw_y)
        headings = np.array(self.raw_heading)
        quality = np.array(self.quality)
        
        min_quality = self.stability_config['min_quality']
        max_pos_jitter = self.stability_config['max_position_jitter']
        max_head_jitter = self.stability_config['max_heading_jitter']
        min_stable = self.stability_config['min_stable_points']
        window_size = self.stability_config['check_window_size']
        
        print(f"参数配置:")
        print(f"  最小质量等级:    {min_quality}")
        print(f"  最大位置抖动:    {max_pos_jitter} m")
        print(f"  最大航向抖动:    {max_head_jitter} rad")
        print(f"  最少连续稳定点:  {min_stable}")
        print(f"  检测窗口:        {window_size} points\n")
        
        # 计算每个点的稳定性评分
        stability_scores = []
        
        for i in range(len(self.raw_x)):
            score = self._calculate_stability_score(i, x_coords, y_coords, 
                                                   headings, quality,
                                                   max_pos_jitter, 
                                                   max_head_jitter, min_quality)
            stability_scores.append(score)
        
        # 找到第一个稳定区间
        stable_index = None
        
        for i in range(len(self.raw_x) - min_stable):
            # 检查从i开始的min_stable个点是否都稳定
            if all(stability_scores[i:i+min_stable]):
                stable_index = i
                break
        
        if stable_index is None:
            print("WARNING: No stable region found in data!")
            print("Using first point with highest quality instead.\n")
            
            # 降级方案：使用质量最高的点
            best_quality_idx = np.argmax(quality[:window_size])
            stable_index = best_quality_idx
        
        # 获取稳定点信息
        point_info = self._get_point_info(stable_index)
        
        # 打印分析结果
        self._print_stability_analysis(stable_index, stability_scores, 
                                      x_coords, y_coords, quality)
        
        return stable_index, point_info
    
    def _calculate_stability_score(self, 
                                  idx: int,
                                  x_coords: np.ndarray,
                                  y_coords: np.ndarray,
                                  headings: np.ndarray,
                                  quality: np.ndarray,
                                  max_pos_jitter: float,
                                  max_head_jitter: float,
                                  min_quality: int) -> bool:
        """
        计算单个点的稳定性评分
        
        稳定性判据：
        1. 质量等级 >= min_quality
        2. 与前5个点的平均距离 < max_pos_jitter
        3. 与前5个点的平均航向差 < max_head_jitter
        
        Args:
            idx: 当前点索引
            x_coords, y_coords: 坐标数组
            headings: 航向角数组
            quality: 质量数组
            max_pos_jitter: 最大位置抖动
            max_head_jitter: 最大航向抖动
            min_quality: 最小质量等级
        
        Returns:
            bool: 该点是否稳定
        """
        
        # 条件1：质量检查
        if quality[idx] < min_quality:
            return False
        
        # 条件2和3：仅在有足够历史数据时检查
        if idx < 5:
            return True
        
        # 计算与前5个点的平均距离
        prev_5_x = x_coords[idx-5:idx]
        prev_5_y = y_coords[idx-5:idx]
        
        avg_x = np.mean(prev_5_x)
        avg_y = np.mean(prev_5_y)
        
        distance_variance = np.sqrt((x_coords[idx] - avg_x)**2 + 
                                   (y_coords[idx] - avg_y)**2)
        
        # 条件2：位置稳定性
        if distance_variance > max_pos_jitter:
            return False
        
        # 条件3：航向稳定性
        prev_5_heading = headings[idx-5:idx]
        heading_variance = np.std(prev_5_heading)
        
        if heading_variance > max_head_jitter:
            return False
        
        return True
    
    def _get_point_info(self, idx: int) -> Dict:
        """获取点的详细信息"""
        return {
            'index': idx,
            'timestamp': self.timestamps[idx],
            'x': self.raw_x[idx],
            'y': self.raw_y[idx],
            'heading': self.raw_heading[idx],
            'quality': self.quality[idx]
        }
    
    def _print_stability_analysis(self,
                                 stable_idx: int,
                                 stability_scores: List[bool],
                                 x_coords: np.ndarray,
                                 y_coords: np.ndarray,
                                 quality: np.ndarray) -> None:
        """打印稳定性分析结果"""
        
        unstable_count = len([s for s in stability_scores[:stable_idx] if not s])
        
        print(f"检测结果:")
        print(f"  稳定第一个点的索引:  {stable_idx}")
        print(f"  跳过的不稳定点:      {unstable_count}")
        print(f"  起始时间戳:         {self.timestamps[stable_idx]} ms")
        
        point = self._get_point_info(stable_idx)
        print(f"\n稳定点信息:")
        print(f"  坐标 (X, Y):       ({point['x']:.6f}, {point['y']:.6f})")
        print(f"  航向角:            {point['heading']:.6f} rad ({np.degrees(point['heading']):.2f}°)")
        print(f"  RTK质量:           {point['quality']}")
        
        # 分析稳定点前的不稳定数据
        if stable_idx > 0:
            print(f"\n不稳定数据分析 (前{stable_idx}个点):")
            
            # 计算前段的统计
            early_quality = quality[:stable_idx]
            early_x = x_coords[:stable_idx]
            early_y = y_coords[:stable_idx]
            
            print(f"  质量分布:")
            for level in range(6):
                count = np.sum(early_quality == level)
                if count > 0:
                    print(f"    Level {level}: {count} points")
            
            # 计算位置变化
            if len(early_x) > 1:
                pos_change = np.sqrt((early_x[-1] - early_x[0])**2 + 
                                    (early_y[-1] - early_y[0])**2)
                print(f"  起止点位置变化:    {pos_change:.6f}")
        
        print()
    
    def convert_to_enu(self, use_stable_first_point: bool = True) -> List[Dict]:
        """
        将坐标转换为相对坐标
        
        Args:
            use_stable_first_point: 是否检测稳定的第一个点 (默认True)
        
        Returns:
            list: 转换后的轨迹列表
        """
        
        if not self.raw_x:
            print("ERROR: No data loaded!")
            return []
        
        # 检测稳定的第一个点（如果需要）
        if use_stable_first_point:
            self.ref_index, ref_point_info = self.detect_stable_first_point()
        else:
            self.ref_index = 0
            ref_point_info = self._get_point_info(0)
        
        self.ref_x = ref_point_info['x']
        self.ref_y = ref_point_info['y']
        
        print(f"\n╔═══════════════════════════════════════════════════╗")
        print(f"║       坐标转换为相对坐标（原点平移）                 ║")
        print(f"╚═══════════════════════════════════════════════════╝\n")
        
        if self.is_latlon:
            print(f"转换模式: WGS84经纬度 → ECEF → ENU\n")
            print(f"参考点（原点，第{self.ref_index}个点）:")
            print(f"  纬度: {self.ref_y:.6f}°")
            print(f"  经度: {self.ref_x:.6f}°")
            self._convert_latlon_to_enu()
        else:
            print(f"转换模式: 直接原点平移\n")
            print(f"参考点（原点，第{self.ref_index}个点）:")
            print(f"  X: {self.ref_x:.4f} m")
            print(f"  Y: {self.ref_y:.4f} m")
            self._convert_direct_offset()
        
        print(f"\n✓ 转换完成，共{len(self.trajectory)}个点")
        print(f"✓ 有效数据点: {len(self.trajectory) - self.ref_index} 个\n")
        
        return self.trajectory
    
    def _convert_direct_offset(self) -> None:
        """直接平移（原始数据已为米制）"""
        self.trajectory = []
        
        for i, (x, y, heading) in enumerate(zip(self.raw_x, self.raw_y, self.raw_heading)):
            point = {
                'index': i,
                'timestamp': self.timestamps[i],
                'x': x - self.ref_x,
                'y': y - self.ref_y,
                'z': 0.0,
                'heading': heading,
                'quality': self.quality[i],
                'lat': None,
                'lon': None,
                'is_before_stable': i < self.ref_index  # 标记是否在稳定点之前
            }
            self.trajectory.append(point)
    
    def _convert_latlon_to_enu(self) -> None:
        """经纬度转ENU坐标"""
        self.trajectory = []
        ref_height = 0.0
        
        for i, (lon, lat, heading) in enumerate(zip(self.raw_x, self.raw_y, self.raw_heading)):
            east, north, up = self._latlon_to_enu(
                self.ref_y, self.ref_x, ref_height,
                lat, lon, 0.0
            )
            
            point = {
                'index': i,
                'timestamp': self.timestamps[i],
                'x': east,
                'y': north,
                'z': up,
                'heading': heading,
                'quality': self.quality[i],
                'lat': lat,
                'lon': lon,
                'is_before_stable': i < self.ref_index
            }
            self.trajectory.append(point)
    
    @staticmethod
    def _latlon_to_enu(ref_lat, ref_lon, ref_h, target_lat, target_lon, target_h) -> Tuple[float, float, float]:
        """
        经纬度转ENU坐标（静态方法）
        
        Args:
            ref_lat, ref_lon, ref_h: 参考点的纬度、经度、高度
            target_lat, target_lon, target_h: 目标点的纬度、经度、高度
        
        Returns:
            (east, north, up): ENU坐标（米）
        """
        
        # WGS84参数
        WGS84_A = 6378137.0           # 长半轴（米）
        WGS84_E2 = 0.00669437999014132  # 第一偏心率平方
        
        # 转换为弧度
        ref_lat_rad = np.radians(ref_lat)
        ref_lon_rad = np.radians(ref_lon)
        target_lat_rad = np.radians(target_lat)
        target_lon_rad = np.radians(target_lon)
        
        # 计算曲率半径N
        N_ref = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(ref_lat_rad)**2)
        N_target = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(target_lat_rad)**2)
        
        # 计算ECEF坐标
        ref_x = (N_ref + ref_h) * np.cos(ref_lat_rad) * np.cos(ref_lon_rad)
        ref_y = (N_ref + ref_h) * np.cos(ref_lat_rad) * np.sin(ref_lon_rad)
        ref_z = (N_ref * (1.0 - WGS84_E2) + ref_h) * np.sin(ref_lat_rad)
        
        target_x = (N_target + target_h) * np.cos(target_lat_rad) * np.cos(target_lon_rad)
        target_y = (N_target + target_h) * np.cos(target_lat_rad) * np.sin(target_lon_rad)
        target_z = (N_target * (1.0 - WGS84_E2) + target_h) * np.sin(target_lat_rad)
        
        # 相对向量
        delta_x = target_x - ref_x
        delta_y = target_y - ref_y
        delta_z = target_z - ref_z
        
        # 旋转矩阵：ECEF → ENU
        sin_lat = np.sin(ref_lat_rad)
        cos_lat = np.cos(ref_lat_rad)
        sin_lon = np.sin(ref_lon_rad)
        cos_lon = np.cos(ref_lon_rad)
        
        east = -sin_lon * delta_x + cos_lon * delta_y
        north = -sin_lat * cos_lon * delta_x - sin_lat * sin_lon * delta_y + cos_lat * delta_z
        up = cos_lat * cos_lon * delta_x + cos_lat * sin_lon * delta_y + sin_lat * delta_z
        
        return east, north, up
    
    def plot_trajectory(self, save_path: Optional[str] = None, figsize: Tuple[int, int] = (16, 12)) -> None:
        """
        绘制RTK轨迹（包括不稳定初始阶段）
        """
        
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return
        
        x_coords = np.array([p['x'] for p in self.trajectory])
        y_coords = np.array([p['y'] for p in self.trajectory])
        headings = np.array([p['heading'] for p in self.trajectory])
        quality = np.array([p['quality'] for p in self.trajectory])
        is_before = np.array([p['is_before_stable'] for p in self.trajectory])
        
        # 创建图表
        fig = plt.figure(figsize=figsize)
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        ax1 = fig.add_subplot(gs[0, :])  # 轨迹图占上面两列
        ax2 = fig.add_subplot(gs[1, 0])
        ax3 = fig.add_subplot(gs[1, 1])
        ax4 = fig.add_subplot(gs[2, 0])
        ax5 = fig.add_subplot(gs[2, 1])
        
        fig.suptitle('RTK Trajectory Analysis with Stable Point Detection', 
                    fontsize=16, fontweight='bold')
        
        # ===== 子图1：轨迹平面图 =====
        # 分两部分绘制：不稳定部分和稳定部分
        # 注意：Y为横轴（左正右负），X为纵轴（上前下后）
        unstable_mask = is_before
        stable_mask = ~is_before
        
        if np.any(unstable_mask):
            ax1.plot(y_coords[unstable_mask], x_coords[unstable_mask], 
                    'r--', linewidth=2, alpha=0.5, label='Unstable Data')
            ax1.scatter(y_coords[unstable_mask], x_coords[unstable_mask], 
                       c=quality[unstable_mask], cmap='Reds', s=30, alpha=0.4)
        
        if np.any(stable_mask):
            ax1.plot(y_coords[stable_mask], x_coords[stable_mask], 
                    'b-', linewidth=2.5, alpha=0.8, label='Stable Data')
            ax1.scatter(y_coords[stable_mask], x_coords[stable_mask], 
                       c=quality[stable_mask], cmap='RdYlGn', s=40, alpha=0.7, vmin=0, vmax=5)
        
        # 标记稳定的第一个点
        ax1.plot(0, 0, 'g*', markersize=20, label='Stable Origin', zorder=10)
        ax1.plot(y_coords[-1], x_coords[-1], 'r^', markersize=12, label='End Point', zorder=5)
        
        # 绘制方向箭头（按距离间隔20cm显示一次）
        arrow_length = max(0.5, (y_coords.max() - y_coords.min()) / 30)
        distance_interval = 0.2  # 20cm间隔显示一次航向
        
        cumulative_distance = 0.0
        last_arrow_distance = 0.0
        
        for i in range(self.ref_index, len(self.trajectory)):
            point = self.trajectory[i]
            
            # 计算从上一个点到当前点的距离
            if i > self.ref_index:
                prev_point = self.trajectory[i - 1]
                dx = point['x'] - prev_point['x']
                dy = point['y'] - prev_point['y']
                segment_distance = np.sqrt(dx**2 + dy**2)
                cumulative_distance += segment_distance
            
            # 每隔distance_interval(20cm)显示一次航向
            if cumulative_distance - last_arrow_distance >= distance_interval:
                heading = point['heading']
                dx = arrow_length * np.cos(heading)
                dy = arrow_length * np.sin(heading)
                
                arrow = FancyArrowPatch(
                    (point['y'], point['x']),
                    (point['y'] + dy, point['x'] + dx),
                    arrowstyle='->', mutation_scale=15, color='darkblue', 
                    linewidth=1.5, alpha=0.6
                )
                ax1.add_patch(arrow)
                last_arrow_distance = cumulative_distance
        
        ax1.set_xlabel('Y 横向坐标 (m) - 左正(+) 右负(-) - ISO 8855', fontsize=11, fontweight='bold')
        ax1.set_ylabel('X 纵向坐标 (m) - 上前(+) 下后(-) - ISO 8855', fontsize=11, fontweight='bold')
        ax1.set_title('RTK轨迹 (俯视图，ISO 8855坐标系) - 稳定第一个点检测', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best', fontsize=10)
        ax1.axis('equal')
        
        # ===== 子图2：X方向变化 =====
        timestamps = np.array(self.timestamps)
        timestamps_rel = (timestamps - timestamps[self.ref_index]) / 1000.0
        
        ax2.axhline(y=0, color='g', linestyle='--', linewidth=2, alpha=0.5, label='Stable Point')
        ax2.fill_between(timestamps_rel[unstable_mask], x_coords[unstable_mask], 
                        alpha=0.3, color='red', label='Unstable')
        ax2.plot(timestamps_rel, x_coords, 'b-', linewidth=2, marker='o', markersize=3)
        
        ax2.set_xlabel('Time from Stable Point (s)', fontsize=11)
        ax2.set_ylabel('X / East Position (m)', fontsize=11)
        ax2.set_title('X Position vs Time', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=9)
        
        # ===== 子图3：Y方向变化 =====
        ax3.axhline(y=0, color='g', linestyle='--', linewidth=2, alpha=0.5)
        ax3.fill_between(timestamps_rel[unstable_mask], y_coords[unstable_mask], 
                        alpha=0.3, color='red')
        ax3.plot(timestamps_rel, y_coords, 'g-', linewidth=2, marker='s', markersize=3)
        
        ax3.set_xlabel('Time from Stable Point (s)', fontsize=11)
        ax3.set_ylabel('Y / North Position (m)', fontsize=11)
        ax3.set_title('Y Position vs Time', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # ===== 子图4：质量演化 =====
        ax4.bar(range(len(self.trajectory)), quality, width=1, 
               color=['red' if b else 'green' for b in is_before], alpha=0.7)
        ax4.axvline(x=self.ref_index, color='blue', linestyle='--', linewidth=2, label='Stable Point')
        ax4.set_xlabel('Point Index', fontsize=11)
        ax4.set_ylabel('RTK Quality', fontsize=11)
        ax4.set_title('RTK Quality Evolution', fontsize=12, fontweight='bold')
        ax4.grid(True, alpha=0.3, axis='y')
        ax4.legend(fontsize=9)
        ax4.set_ylim([0, 5.5])
        
        # ===== 子图5：航向角变化 =====
        headings_deg = np.degrees(headings)
        ax5.fill_between(range(len(headings)), headings_deg, alpha=0.3, color='red')
        ax5.plot(range(len(headings)), headings_deg, 'r-', linewidth=2, marker='^', markersize=3)
        ax5.axvline(x=self.ref_index, color='blue', linestyle='--', linewidth=2, label='Stable Point')
        
        ax5.set_xlabel('Point Index', fontsize=11)
        ax5.set_ylabel('Heading (°)', fontsize=11)
        ax5.set_title('Heading Angle Evolution', fontsize=12, fontweight='bold')
        ax5.grid(True, alpha=0.3)
        ax5.legend(fontsize=9)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"\n✓ Figure saved to: {save_path}")
        
        plt.show()
    
    def print_statistics(self) -> None:
        """打印轨迹统计信息"""
        
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return
        
        # 分离稳定和不稳定数据
        stable_traj = [p for p in self.trajectory if not p['is_before_stable']]
        unstable_traj = [p for p in self.trajectory if p['is_before_stable']]
        
        if not stable_traj:
            print("ERROR: No stable data!")
            return
        
        x_coords = np.array([p['x'] for p in stable_traj])
        y_coords = np.array([p['y'] for p in stable_traj])
        headings = np.array([p['heading'] for p in stable_traj])
        quality = np.array([p['quality'] for p in stable_traj])
        
        # 计算距离
        dx = np.diff(x_coords)
        dy = np.diff(y_coords)
        distances = np.sqrt(dx**2 + dy**2)
        total_distance = np.sum(distances)
        
        headings_deg = np.degrees(headings)
        
        print("\n" + "="*75)
        print("RTK TRAJECTORY STATISTICS (AFTER STABLE POINT)".center(75))
        print("="*75)
        
        print("\n【数据分布】")
        print(f"  总数据点:          {len(self.trajectory)}")
        print(f"  不稳定点:          {len(unstable_traj)} (前期初始化阶段)")
        print(f"  稳定点:            {len(stable_traj)} (有效定位数据)")
        print(f"  稳定点占比:        {100*len(stable_traj)/len(self.trajectory):.1f}%")
        
        if unstable_traj:
            unstable_time = (unstable_traj[-1]['timestamp'] - unstable_traj[0]['timestamp']) / 1000.0
            print(f"  不稳定期时长:      {unstable_time:.2f} s")
        
        print("\n【坐标范围】(稳定数据)")
        print(f"  X (East):   [{x_coords.min():10.4f}, {x_coords.max():10.4f}] m")
        print(f"  Y (North):  [{y_coords.min():10.4f}, {y_coords.max():10.4f}] m")
        
        total_range = np.sqrt((x_coords.max()-x_coords.min())**2 + 
                             (y_coords.max()-y_coords.min())**2)
        print(f"  覆盖范围:   {total_range:10.4f} m")
        
        print("\n【运动统计】(稳定数据)")
        print(f"  总距离:            {total_distance:10.4f} m")
        if len(distances) > 0:
            print(f"  平均步长:          {np.mean(distances):10.6f} m")
            print(f"  最大步长:          {np.max(distances):10.6f} m")
            print(f"  最小步长:          {np.min(distances):10.6f} m")
        
        print("\n【航向统计】(稳定数据)")
        print(f"  航向范围:          [{headings_deg.min():7.2f}°, {headings_deg.max():7.2f}°]")
        print(f"  平均航向:          {np.mean(headings_deg):7.2f}°")
        print(f"  航向标准差:        {np.std(headings_deg):7.2f}°")
        
        print("\n【质量统计】(稳定数据)")
        quality_counts = np.bincount(quality.astype(int), minlength=6)
        for level in range(6):
            count = quality_counts[level]
            if count > 0:
                percentage = 100 * count / len(stable_traj)
                bar = "█" * int(percentage / 2)
                print(f"  Level {level}: {count:5d} points ({percentage:5.1f}%) {bar}")
        
        print("\n【时间统计】(稳定数据)")
        timestamps_stable = np.array([p['timestamp'] for p in stable_traj])
        time_span = (timestamps_stable[-1] - timestamps_stable[0]) / 1000.0
        print(f"  时间跨度:          {time_span:10.2f} s")
        if time_span > 0:
            print(f"  平均速度:          {total_distance / time_span:10.4f} m/s")
            print(f"  采样率:            {len(stable_traj) / max(time_span, 0.001):10.2f} Hz")
        
        print("\n" + "="*75 + "\n")
    
    def plot_trajectory_with_heading_markers(self, save_path: Optional[str] = None, 
                                            figsize: Tuple[int, int] = (14, 10),
                                            distance_interval: float = 0.2) -> None:
        """
        绘制轨迹，并在每隔指定距离处显示航向箭头
        
        Args:
            save_path: 图表保存路径（可选）
            figsize: 图表大小（宽，高）
            distance_interval: 航向箭头显示间隔（米，默认0.2m=20cm）
        """
        
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return
        
        x_coords = np.array([p['x'] for p in self.trajectory])
        y_coords = np.array([p['y'] for p in self.trajectory])
        headings = np.array([p['heading'] for p in self.trajectory])
        quality = np.array([p['quality'] for p in self.trajectory])
        is_before = np.array([p['is_before_stable'] for p in self.trajectory])
        
        # 创建图表
        fig, ax = plt.subplots(figsize=figsize)
        
        # 分两部分绘制：不稳定部分和稳定部分
        # 注意：Y为横轴（左正右负），X为纵轴（上前下后）
        unstable_mask = is_before
        stable_mask = ~is_before
        
        if np.any(unstable_mask):
            ax.plot(y_coords[unstable_mask], x_coords[unstable_mask], 
                    'r--', linewidth=2, alpha=0.5, label='Unstable Data')
            ax.scatter(y_coords[unstable_mask], x_coords[unstable_mask], 
                       c='red', s=30, alpha=0.3)
        
        if np.any(stable_mask):
            ax.plot(y_coords[stable_mask], x_coords[stable_mask], 
                    'b-', linewidth=2, alpha=0.7, label='Trajectory')
            ax.scatter(y_coords[stable_mask], x_coords[stable_mask], 
                       c=quality[stable_mask], cmap='viridis', s=50, 
                       alpha=0.6, vmin=0, vmax=5)
        
        # 标记关键点
        ax.plot(0, 0, 'g*', markersize=25, label='Start (Stable)', zorder=10)
        ax.plot(y_coords[-1], x_coords[-1], 'r^', markersize=15, label='End', zorder=5)
        
        # 绘制航向箭头（按距离间隔显示）
        arrow_length = max(0.3, (y_coords.max() - y_coords.min()) / 25)
        
        cumulative_distance = 0.0
        last_arrow_distance = 0.0
        
        # 统计信息
        arrow_count = 0
        arrow_positions = []
        
        for i in range(self.ref_index, len(self.trajectory)):
            point = self.trajectory[i]
            
            # 计算从上一个点到当前点的距离
            if i > self.ref_index:
                prev_point = self.trajectory[i - 1]
                dx = point['x'] - prev_point['x']
                dy = point['y'] - prev_point['y']
                segment_distance = np.sqrt(dx**2 + dy**2)
                cumulative_distance += segment_distance
            
            # 每隔distance_interval显示一次航向箭头
            if cumulative_distance - last_arrow_distance >= distance_interval:
                heading = point['heading']
                dx = arrow_length * np.cos(heading)
                dy = arrow_length * np.sin(heading)
                
                # 创建箭头 (注意：绘图坐标为(y,x)，所以箭头方向也要转换)
                arrow = FancyArrowPatch(
                    (point['y'], point['x']),
                    (point['y'] + dy, point['x'] + dx),
                    arrowstyle='->', mutation_scale=18, color='darkgreen', 
                    linewidth=1.2, alpha=0.7, zorder=3
                )
                ax.add_patch(arrow)
                
                arrow_count += 1
                arrow_positions.append((point['y'], point['x'], 
                                      np.degrees(heading), cumulative_distance))
                last_arrow_distance = cumulative_distance
        
        # 颜色条表示质量
        sm = plt.cm.ScalarMappable(cmap='viridis', 
                                   norm=plt.Normalize(vmin=0, vmax=5))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, label='RTK Quality Level')
        
        # 图表标题和标签（ISO 8855坐标系）
        ax.set_xlabel('Y 横向坐标 (m) - 左正(+) 右负(-) - ISO 8855', fontsize=12, fontweight='bold')
        ax.set_ylabel('X 纵向坐标 (m) - 上前(+) 下后(-) - ISO 8855', fontsize=12, fontweight='bold')
        ax.set_title(f'RTK轨迹与车辆航向 (ISO 8855坐标系)\n(每{distance_interval*100:.0f}cm显示一个航向箭头)', 
                    fontsize=14, fontweight='bold')
        ax.invert_xaxis()  # 反转横轴，使左正右负
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.legend(loc='best', fontsize=11)
        ax.axis('equal')
        
        # 添加文字信息
        total_distance = sum(np.sqrt((x_coords[i] - x_coords[i-1])**2 + 
                                     (y_coords[i] - y_coords[i-1])**2) 
                            for i in range(1, len(x_coords)))
        
        info_text = f'Total Distance: {total_distance:.2f}m\n'
        info_text += f'Heading Arrows: {arrow_count} (every {distance_interval*100:.0f}cm)\n'
        info_text += f'Heading Range: {np.degrees(headings.min()):.1f}° ~ {np.degrees(headings.max()):.1f}°'
        
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"\n✓ Figure saved to: {save_path}")
        
        # 打印箭头信息
        print(f"\n📊 航向箭头显示统计:")
        print(f"  显示间隔：{distance_interval*100:.0f} cm")
        print(f"  箭头数量：{arrow_count} 个")
        print(f"  轨迹总长：{total_distance:.2f} m")
        
        plt.show()
    
    def save_to_csv(self, csv_path: str, include_unstable: bool = False) -> None:
        """
        保存轨迹数据为CSV文件
        
        Args:
            csv_path: CSV文件保存路径
            include_unstable: 是否包含不稳定数据
        """
        
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return
        
        print(f"Saving trajectory to CSV: {csv_path}")
        
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f, delimiter=';')
            
            # 写入头部
            headers = [
                'Index', 'Timestamp(ms)', 'Time_from_Stable(s)',
                'X_East(m)', 'Y_North(m)', 'Z_Up(m)',
                'Heading(rad)', 'Heading(deg)', 'Quality', 'Data_Status'
            ]
            
            if self.trajectory[0]['lat'] is not None:
                headers.extend(['Latitude(°)', 'Longitude(°)'])
            
            writer.writerow(headers)
            
            # 写入数据
            start_time = self.timestamps[self.ref_index]
            
            for point in self.trajectory:
                # 可选：跳过不稳定数据
                if point['is_before_stable'] and not include_unstable:
                    continue
                
                time_s = (point['timestamp'] - start_time) / 1000.0
                heading_deg = np.degrees(point['heading'])
                status = "UNSTABLE" if point['is_before_stable'] else "STABLE"
                
                row = [
                    point['index'],
                    f"{point['timestamp']:.0f}",
                    f"{time_s:.3f}",
                    f"{point['x']:.6f}",
                    f"{point['y']:.6f}",
                    f"{point['z']:.6f}",
                    f"{point['heading']:.6f}",
                    f"{heading_deg:.2f}",
                    point['quality'],
                    status
                ]
                
                if point['lat'] is not None:
                    row.extend([
                        f"{point['lat']:.8f}",
                        f"{point['lon']:.8f}"
                    ])
                
                writer.writerow(row)
    
    def _draw_coordinate_system(self, ax, origin=(0, 0), scale=1.0, arrow_width=2):
        """
        在图表上绘制坐标系箭头（ISO 8855标准）
        
        参数:
            ax: matplotlib axes对象
            origin: 原点位置 (y, x) - 注意：已转换为(y_min, x_min)
            scale: 箭头长度缩放因子
            arrow_width: 箭头宽度
            
        注意：
            - Y轴箭头在水平方向（红色）- 左正右负
            - X轴箭头在垂直方向（绿色）- 上前下后
        """
        # 获取轴的当前范围
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]
        
        # 箭头长度（轴长度的10%）
        arrow_len = max(x_range, y_range) * 0.1 * scale
        
        # Y轴箭头在水平方向（红色）- 因为水平轴表示Y坐标
        # Y轴：左正(+) 右负(-)，所以箭头指向左侧（正方向）
        arrow_y = FancyArrowPatch(origin, (origin[0] + arrow_len, origin[1]),
                                arrowstyle='->', mutation_scale=20, linewidth=arrow_width,
                                color='red', zorder=10)
        ax.add_patch(arrow_y)
        ax.text(origin[0] - arrow_len*1.2, origin[1] - 0.5, 'Y+ (左)', fontsize=10, color='red', fontweight='bold', ha='right')
        
        # X轴箭头在垂直方向（绿色）- 因为垂直轴表示X坐标
        # X轴：上前(+) 下后(-)，所以箭头指向上方（正方向）
        arrow_x = FancyArrowPatch(origin, (origin[0], origin[1] + arrow_len),
                                arrowstyle='->', mutation_scale=20, linewidth=arrow_width,
                                color='green', zorder=10)
        ax.add_patch(arrow_x)
        ax.text(origin[0] - 0.5, origin[1] + arrow_len*1.1, 'X+ (前)', fontsize=10, color='green', fontweight='bold', ha='right')
        
        # 原点标记
        ax.plot(origin[0], origin[1], 'k+', markersize=15, markeredgewidth=2, zorder=10)
        
        count = len([p for p in self.trajectory if not p['is_before_stable'] or include_unstable])
        print(f"✓ CSV file saved successfully with {count} points!")
    
    def get_trajectory_latlon(self, include_unstable: bool = False) -> Dict:
        """
        获取轨迹的经纬度坐标列表
        
        Returns:
            dict: 包含以下键值：
                - 'lon_list': 经度列表 (°)
                - 'lat_list': 纬度列表 (°)
                - 'heading_list': 航向角列表 (rad)
                - 'timestamp_list': 时间戳列表 (ms)
                - 'quality_list': 质量列表
                - 'count': 总点数
                - 'stable_count': 稳定点数
        """
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return None
        
        lon_list, lat_list, heading_list, timestamp_list, quality_list = [], [], [], [], []
        
        for point in self.trajectory:
            if point['is_before_stable'] and not include_unstable:
                continue
            if point['lat'] is not None and point['lon'] is not None:
                lon_list.append(point['lon'])
                lat_list.append(point['lat'])
            else:
                lon_list.append(point['x'])
                lat_list.append(point['y'])
            heading_list.append(point['heading'])
            timestamp_list.append(point['timestamp'])
            quality_list.append(point['quality'])
        
        stable_count = len([p for p in self.trajectory if not p['is_before_stable']])
        return {
            'lon_list': lon_list,
            'lat_list': lat_list,
            'heading_list': heading_list,
            'timestamp_list': timestamp_list,
            'quality_list': quality_list,
            'count': len(lon_list),
            'stable_count': stable_count,
            'include_unstable': include_unstable
        }
    
    def extract_rtk_lists(self, include_unstable: bool = False) -> Tuple[List[float], List[float], List[float], List[int], List[int]]:
        """
        快速提取RTK轨迹数据（适合与general_data_from_blf.py集成）
        
        Returns:
            tuple: (lon_list, lat_list, heading_list, timestamp_list, quality_list)
        """
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return [], [], [], [], []
        
        lon_list, lat_list, heading_list, timestamp_list, quality_list = [], [], [], [], []
        
        for point in self.trajectory:
            if point['is_before_stable'] and not include_unstable:
                continue
            if point['lat'] is not None and point['lon'] is not None:
                lon_list.append(point['lon'])
                lat_list.append(point['lat'])
            else:
                lon_list.append(point['x'])
                lat_list.append(point['y'])
            heading_list.append(point['heading'])
            timestamp_list.append(point['timestamp'])
            quality_list.append(point['quality'])
        
        return lon_list, lat_list, heading_list, timestamp_list, quality_list
    
    def get_stable_trajectory_points(self) -> List[Dict]:
        """
        获取所有稳定点的完整信息（仅稳定数据）
        
        Returns:
            list: 稳定点列表，每个点包含所有信息：
                {
                    'index': 原始索引,
                    'timestamp': 时间戳(ms),
                    'lon': 经度(°),
                    'lat': 纬度(°),
                    'heading': 航向角(rad),
                    'quality': 质量等级,
                    'x': ENU东向距离(m),
                    'y': ENU北向距离(m),
                    'z': ENU竖向距离(m)
                }
        """
        if not self.trajectory:
            print("ERROR: No trajectory data!")
            return []
        
        stable_points = []
        for point in self.trajectory:
            if not point['is_before_stable']:
                stable_points.append({
                    'index': point['index'],
                    'timestamp': point['timestamp'],
                    'lon': point['lon'] if point['lon'] is not None else point['x'],
                    'lat': point['lat'] if point['lat'] is not None else point['y'],
                    'heading': point['heading'],
                    'quality': point['quality'],
                    'x': point['x'],
                    'y': point['y'],
                    'z': point['z']
                })
        
        return stable_points


# ===== 使用示例 =====

def example_parking_maneuver():
    """
    示例：停车轨迹（包含不稳定初始化阶段）
    """
    print("\n╔═══════════════════════════════════════════════════════════╗")
    print("║      RTK轨迹可视化 - 稳定点检测示例                        ║")
    print("╚═══════════════════════════════════════════════════════════╝\n")
    
    # 模拟数据：包含不稳定初始化阶段
    
    # 第一阶段：初始化，不稳定（15个点，质量逐渐提升）
    phase1_x = [116.407 + np.random.normal(0, 0.00002) for _ in range(15)]
    phase1_y = [39.904 + np.random.normal(0, 0.00002) for _ in range(15)]
    phase1_heading = [0 + np.random.normal(0, 0.1) for _ in range(15)]
    phase1_quality = [0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5]
    
    # 第二阶段：稳定，直行100米（100个点）
    phase2_x = [116.407 + i * 0.00001 for i in range(100)]
    phase2_y = [39.904 + 0.00001 + i * 0.00002 for i in range(100)]
    phase2_heading = [0] * 100
    phase2_quality = [5] * 100
    
    # 第三阶段：稳定，转向并后退（50个点）
    phase3_x = [116.407 + 0.001 - i * 0.00001 for i in range(50)]
    phase3_y = [39.904 + 0.00301 + i * 0.00001 for i in range(50)]
    phase3_heading = [-(i / 50) * np.pi/2 for i in range(50)]
    phase3_quality = [5] * 50
    
    # 合并
    x_list = phase1_x + phase2_x + phase3_x
    y_list = phase1_y + phase2_y + phase3_y
    heading_list = phase1_heading + phase2_heading + phase3_heading
    quality_list = phase1_quality + phase2_quality + phase3_quality
    timestamps = [i * 50 for i in range(len(x_list))]
    
    # 创建绘制器
    plotter = RTKTrajectoryPlotter()
    
    # 设置稳定点检测参数
    plotter.set_stability_config({
        'min_quality': 4,
        'max_position_jitter': 0.00001,    # 米（经纬度空间）
        'max_heading_jitter': 0.05,
        'min_stable_points': 5,
        'check_window_size': 30
    })
    
    # 加载数据
    plotter.load_from_lists(
        x_list=x_list,
        y_list=y_list,
        heading_list=heading_list,
        timestamps_list=timestamps,
        quality_list=quality_list,
        is_latlon=True
    )
    
    # 转换为相对坐标（自动检测稳定第一个点）
    trajectory = plotter.convert_to_enu(use_stable_first_point=True)
    
    # 打印统计
    plotter.print_statistics()
    
    # 绘制
    plotter.plot_trajectory(save_path="rtk_stable_detection.png", figsize=(16, 12))
    
    # 保存CSV（包含所有数据）
    plotter.save_to_csv("rtk_trajectory_stable.csv", include_unstable=True)
    
    print("\n✓ RTK轨迹可视化完成！")


def plot_rtk_from_csv(csv_file: str = "rtk_can.csv", output_dir: str = None) -> None:
    """
    从 CSV 文件读取 RTK 数据并进行可视化
    
    Args:
        csv_file: RTK CSV 文件路径 (默认: rtk_can.csv)
        output_dir: 输出目录 (默认: None，使用当前目录)
    """
    
    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║         RTK轨迹可视化 - 从CSV文件读取数据             ║")
    print("╚═══════════════════════════════════════════════════════╝\n")
    
    # 检查文件是否存在
    if not os.path.exists(csv_file):
        print(f"ERROR: CSV file not found: {csv_file}")
        print(f"\n💡 提示: 请确保文件存在于以下位置之一:")
        print(f"  1. 当前目录: {os.path.abspath(csv_file)}")
        print(f"  2. 项目根目录: {os.path.join('/Users/alex/workspace/code_review/vehicle_localization', csv_file)}")
        return
    
    # 创建绘制器
    plotter = RTKTrajectoryPlotter()
    
    # 设置稳定点检测参数
    plotter.set_stability_config({
        'min_quality': 4,
        'max_position_jitter': 0.00005,    # 米（经纬度空间）
        'max_heading_jitter': 0.1,
        'min_stable_points': 10,
        'check_window_size': 30
    })
    
    # 从 CSV 文件加载数据
    plotter.load_from_csv(csv_file, is_latlon=True)
    
    # 检查数据是否加载成功
    if not plotter.raw_x:
        print("ERROR: No data loaded from CSV file!")
        return
    
    # 转换为相对坐标（自动检测稳定第一个点）
    trajectory = plotter.convert_to_enu(use_stable_first_point=True)
    
    # 打印统计信息
    plotter.print_statistics()
    
    # 确定输出路径
    if output_dir is None:
        output_dir = "rtk_trajectory_plots"
    os.makedirs(output_dir, exist_ok=True)
    
    # 输出文件路径
    plot_path = os.path.join(output_dir, "rtk_trajectory_from_csv.png")
    csv_path = os.path.join(output_dir, "rtk_trajectory_processed.csv")
    
    # 绘制轨迹
    print(f"\n📊 绘制轨迹...")
    plotter.plot_trajectory(save_path=plot_path, figsize=(14, 10))
    print(f"✓ 轨迹图表已保存: {plot_path}")
    
    # 保存处理后的数据
    print(f"\n💾 保存处理后的数据...")
    plotter.save_to_csv(csv_path, include_unstable=True)
    print(f"✓ 处理后的数据已保存: {csv_path}")
    
    print("\n✓ RTK轨迹可视化完成！")
    print(f"\n📁 输出目录: {os.path.abspath(output_dir)}")


if __name__ == "__main__":
    import sys
    
    # 检查命令行参数
    if len(sys.argv) > 1:
        # 如果提供了 CSV 文件路径
        csv_file = sys.argv[1]
        plot_rtk_from_csv(csv_file)
    else:
        # 尝试在当前目录或项目目录中查找 rtk_can.csv
        possible_paths = [
            
        ]
        
        csv_file = None
        for path in possible_paths:
            if os.path.exists(path):
                csv_file = path
                break
        
        if csv_file:
            print(f"🔍 找到 RTK 数据文件: {os.path.abspath(csv_file)}")
            plot_rtk_from_csv(csv_file)
        else:
            print("⚠️  未找到 rtk_can.csv 文件")
            print("\n用法:")
            print("  python3 plot_rtk.py")
            print("  python3 plot_rtk.py <rtk_can.csv路径>")
            print("\n运行示例:")
            print("  python3 example_parking_maneuver()")
