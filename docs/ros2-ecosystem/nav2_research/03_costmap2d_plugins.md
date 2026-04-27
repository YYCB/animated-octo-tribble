# Costmap2D 插件体系 — 双层架构与五类插件详解

---

## 一、双层 Costmap 架构

Nav2 维护两个独立的代价地图实例，通过 `nav2_costmap_2d::Costmap2DROS` 管理：

```
┌──────────────────────────────────────────────────────────────────────────┐
│  Global Costmap（全局代价地图）                                           │
│  节点名：/global_costmap/global_costmap                                  │
│  更新频率：~1 Hz（慢，用于全局路径规划）                                   │
│  空间范围：整张地图（无限大滚动窗口可选）                                   │
│  输入：/map（静态地图）+ 激光雷达（可选，标记动态障碍）                      │
│  → 供 planner_server 使用                                               │
├──────────────────────────────────────────────────────────────────────────┤
│  Local Costmap（局部代价地图）                                            │
│  节点名：/local_costmap/local_costmap                                    │
│  更新频率：10-20 Hz（快，用于实时避障）                                    │
│  空间范围：机器人周围固定窗口（如 3m × 3m 或 5m × 5m）                    │
│  输入：激光雷达 / 深度相机 / 超声波                                         │
│  → 供 controller_server 使用                                            │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 二、Costmap2D 分层架构（Layer 栈）

每个代价地图实例由多个**插件层（Layer）**叠加组成，按顺序更新：

```
┌─────────────────────────────────────┐
│  Master Costmap（最终代价地图）       │
│  = 各层 updateCosts() 叠加结果        │
├─────────────────────────────────────┤
│  Layer N: InflationLayer（最后）     │ ← 对障碍膨胀，产生代价梯度
├─────────────────────────────────────┤
│  Layer 3: ObstacleLayer / VoxelLayer │ ← 传感器数据（激光/深度）
├─────────────────────────────────────┤
│  Layer 2: StaticLayer（最先）        │ ← /map 静态障碍
├─────────────────────────────────────┤
│  （可选）SocialLayer / CustomLayer   │ ← 自定义代价
└─────────────────────────────────────┘
```

**关键接口**：

```cpp
// nav2_costmap_2d::Layer 抽象基类
class Layer {
  virtual void onInitialize() = 0;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double * min_x, double * min_y,
                            double * max_x, double * max_y) = 0;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                           int min_i, int min_j, int max_i, int max_j) = 0;
  virtual bool isClearable() = 0;  // 是否支持 clear_costmap 服务清除
};
```

---

## 三、五类核心插件详解

### 3.1 StaticLayer

**功能**：将 `/map`（`nav_msgs/OccupancyGrid`）的占用网格复制到代价地图中。

```yaml
StaticLayer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true  # QoS: TRANSIENT_LOCAL，确保获取最新地图
  enabled: true
  subscribe_to_updates: true           # 监听地图更新（SLAM 实时地图）
  map_topic: /map
```

**代价映射**：
- OccupancyGrid 100（占用）→ Costmap 254（`LETHAL_OBSTACLE`）
- OccupancyGrid 0（自由）→ Costmap 0（FREE_SPACE）
- OccupancyGrid -1（未知）→ Costmap 255（`NO_INFORMATION`）

### 3.2 InflationLayer

**功能**：在障碍物周围生成代价梯度，机器人避开障碍时保持安全距离。这是 Nav2 中**最重要的层**。

```yaml
InflationLayer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0     # 代价衰减指数（越大衰减越快，机器人越靠近障碍）
  inflation_radius: 0.55       # m，膨胀半径（应 ≥ 机器人内切圆半径）
  inflate_unknown: false       # 是否对未知区域也膨胀
  inflate_around_unknown: true # 是否在未知区域边缘膨胀
```

**代价函数**：
```
cost(d) = 253 * exp(-cost_scaling_factor * (d - robot_radius))  for d > robot_radius
cost(d) = LETHAL_OBSTACLE (254)                                  for d ≤ robot_radius
```

**调优建议**：
- `inflation_radius` 应略大于机器人半径，让规划器保持安全间距
- `cost_scaling_factor` 越小 → 代价衰减越慢 → 机器人更倾向走宽阔路径

### 3.3 ObstacleLayer（2D 激光雷达）

**功能**：处理 2D 激光雷达数据，标记/清除障碍物。

```yaml
ObstacleLayer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan        # 传感器源列表（可配置多个）
  scan:
    topic: /scan                   # sensor_msgs/LaserScan
    max_obstacle_height: 2.0       # m，只考虑此高度以下的障碍
    clearing: true                 # 是否用光束清除旧障碍（动态障碍消失后清除）
    marking: true                  # 是否标记新障碍
    data_type: "LaserScan"
    raytrace_max_range: 3.0        # m，光线追踪最大距离
    raytrace_min_range: 0.0
    obstacle_max_range: 2.5        # m，标记障碍最大距离
    obstacle_min_range: 0.0
```

**ObstacleLayer vs VoxelLayer**：
- ObstacleLayer：2D 网格，处理 `LaserScan`，不处理高度信息
- VoxelLayer：3D 体素，处理 `PointCloud2`（深度相机/3D 激光），能区分地面和障碍

### 3.4 VoxelLayer（3D 点云）

**功能**：处理 3D 点云，进行体素化后投影到 2D 代价地图。

```yaml
VoxelLayer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  enabled: true
  publish_voxel_map: true           # 可视化 3D 体素（RViz）
  origin_z: 0.0
  z_resolution: 0.05                # m，体素 Z 分辨率
  z_voxels: 16                      # Z 方向体素数（0 ~ 0.8m）
  max_obstacle_height: 2.0
  mark_threshold: 0                 # 多少个体素标记后才算障碍
  observation_sources: pointcloud_sensor
  pointcloud_sensor:
    topic: /camera/depth/points      # sensor_msgs/PointCloud2
    data_type: "PointCloud2"
    max_obstacle_height: 2.0
    min_obstacle_height: 0.0
    clearing: true
    marking: true
    obstacle_max_range: 2.0
    raytrace_max_range: 3.0
```

### 3.5 SpatioTemporalVoxelLayer（STVL，动态障碍预测）

**功能**：在 VoxelLayer 基础上增加**时间维度**，障碍物随时间自动衰减（动态障碍消失后自动清除）。

```yaml
SpatioTemporalVoxelLayer:
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
  enabled: true
  voxel_decay: 15.0             # s，体素存活时间（超时自动清除）
  decay_model: 0                # 0=线性衰减，1=指数衰减
  voxel_size: 0.05
  track_unknown_space: true
  observation_persistence: 0.0 # s，0=每帧刷新
  max_obstacle_height: 2.0
  mark_threshold: 0
  update_footprint_enabled: true
  combination_method: 1         # 1=最大值叠加
```

---

## 四、Costmap 代价值语义

```
值    名称                含义
0     FREE_SPACE          机器人可以移动
1-252 (代价梯度)          靠近障碍，越高越危险
253   INSCRIBED_INFLATED  机器人内切圆触及障碍（强烈不建议进入）
254   LETHAL_OBSTACLE     致命障碍（机器人中心在此 = 碰撞）
255   NO_INFORMATION      未知（未探索）
```

---

## 五、Costmap 调试与可视化

```bash
# RViz 可视化：
# Add → Map → Topic: /global_costmap/costmap
# Add → Map → Topic: /local_costmap/costmap

# 手动清除代价地图（卡住时有用）：
ros2 service call /global_costmap/clear_entirely_global_costmap \
    nav2_msgs/srv/ClearEntireCostmap

# 动态查看代价地图分辨率/尺寸：
ros2 topic echo /global_costmap/costmap_updates

# 体素地图可视化（VoxelLayer）：
ros2 topic echo /voxel_grid  # sensor_msgs/PointCloud2，在 RViz 中显示
```

---

## 六、Footprint（机器人轮廓）配置

Costmap 需要知道机器人的形状，用于膨胀计算：

```yaml
# 圆形机器人（最简单，适合差速驱动圆形机器人）
global_costmap:
  ros__parameters:
    robot_radius: 0.22            # m（用 robot_radius 代替 footprint）

# 多边形机器人（矩形机器人）
local_costmap:
  ros__parameters:
    footprint: "[ [0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25] ]"
    footprint_padding: 0.01       # m，额外安全边距
```
