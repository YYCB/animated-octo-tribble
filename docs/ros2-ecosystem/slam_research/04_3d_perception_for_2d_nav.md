# 局部 3D 感知接 Nav2 costmap — RGB-D 深度的 CPU-only 路径

> 目标：把 RGB-D 相机的 depth 数据**全 CPU**地喂进 Nav2 local_costmap，让机器人看见**桌椅腿、悬空障碍、矮挡板**等 2D LiDAR 看不到的东西。
>
> **硬约束**：S100 主控 → 无 CUDA → **不能用 nvblox**。本文给出 CPU 替代路径。

---

## 一、为什么 2D LiDAR + Costmap 不够

典型 2D LiDAR 安装高度 15–25 cm，看不到：

| 障碍 | 高度位置 | 危险性 |
|------|---------|--------|
| 桌椅腿（跨度小） | 与雷达同高，但点稀 | 高（卡轮）|
| 悬空挡板 / 横梁 | 在雷达上方 | 高（撞机器人上半身）|
| 货架/桌面边缘 | 雷达扫不到下方 | 中 |
| 楼梯下行 | "看到地"以下 | **致命**（跌落）|
| 矮于雷达的玩具/线缆 | 在雷达下方 | 中 |

RGB-D 相机的视锥能覆盖这些区域。需要的工程链路：

```
/depth (Image) ─→ depth → PointCloud2 ─→ 降采样 + 地面分割
                                              │
                                              ▼
                            spatio_temporal_voxel_layer (STVL)
                                              │
                                              ▼
                            Nav2 local_costmap （Voxel/STVL Layer）
```

---

## 二、为什么不用 nvblox（必读）

`docs/ros2-ecosystem/perception_research/` 里讨论过 nvblox。在 S100 上**完全不可用**，原因：

| 维度 | nvblox | S100 上的现实 |
|------|--------|--------------|
| TSDF 集成 | CUDA kernel | 无 CUDA |
| ESDF 计算 | CUDA + 并行 BFS | 无 CUDA |
| 点云投影 | GPU memory + cuBLAS | 无 |
| 替代品 | — | **STVL 或 nav2_costmap_2d::VoxelLayer** |

> 在 Orin AGX 上 nvblox 是 perception_research 的合理选择；切到 S100 后**一律换成本节方案**。

---

## 三、深度→点云链路

### 3.1 节点选型

| 包 | 节点 | 作用 |
|----|------|------|
| `depth_image_proc` | `point_cloud_xyz` | depth + camera_info → PointCloud2 (XYZ) |
| `depth_image_proc` | `point_cloud_xyzrgb` | depth + color + camera_info → PointCloud2 (XYZ+RGB)，**costmap 不需要 RGB** |
| `pcl_ros` | `voxel_grid` | 体素降采样 |
| `pcl_ros` | `passthrough` | 按高度切片（去地面 / 去天花板） |

> 推荐：**只用 XYZ**（无 RGB），节省一半带宽和 CPU。

### 3.2 launch 骨架

```python
ComposableNodeContainer(
    name='depth_to_cloud_container',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        # 1) Depth → PointCloud2 （Image transport，可选 zero-copy 单进程）
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            name='point_cloud_xyz',
            remappings=[
                ('image_rect', '/depth/image_rect_raw'),
                ('camera_info', '/depth/camera_info'),
                ('points', '/depth/points'),
            ],
        ),
        # 2) Voxel downsample
        ComposableNode(
            package='pcl_ros',
            plugin='pcl_ros::VoxelGrid',
            name='voxel_grid',
            parameters=[{'leaf_size': 0.05,
                         'input_frame': 'depth_optical_frame',
                         'output_frame': 'depth_optical_frame'}],
            remappings=[('input', '/depth/points'),
                        ('output', '/depth/points_downsampled')],
        ),
        # 3) Passthrough：保留 z ∈ [0.05, 1.8] 的点（去地面 + 去天花板）
        ComposableNode(
            package='pcl_ros',
            plugin='pcl_ros::PassThrough',
            name='ground_filter',
            parameters=[{'filter_field_name': 'z',
                         'filter_limit_min': 0.05,
                         'filter_limit_max': 1.8,
                         'input_frame': 'base_link',
                         'output_frame': 'base_link'}],
            remappings=[('input', '/depth/points_downsampled'),
                        ('output', '/depth/points_filtered')],
        ),
    ],
)
```

> **同进程组合**（`ComposableNodeContainer`）很关键——避免 PointCloud2 在三个节点之间反复序列化/拷贝；详见仓库 `ros2-core/ros2_topic_communication_research/`。

### 3.3 Voxel/Passthrough 参数选择

| 参数 | 推荐值 | 作用 |
|------|-------|------|
| `leaf_size`（VoxelGrid） | **0.05 m** | 5cm 体素够 costmap 用；越小越准但 CPU↑ |
| `filter_limit_min`（z, base_link）| 0.05 m | 留 5cm 容差，否则地面噪点充斥 |
| `filter_limit_max`（z, base_link）| **机器人最高点 + 0.1m** | 滤掉天花板/灯具 |

> ⚠️ Passthrough 的 `input_frame`/`output_frame` 要写成 `base_link`，让节点先把点云从 `depth_optical_frame` 变换到 `base_link`，再按高度过滤。

### 3.4 地面分割（可选增强）

简单 passthrough 在地面不平时会漏（楼层接缝、坡道）。更鲁棒的方案：

| 方法 | 复杂度 | CPU |
|------|-------|-----|
| Passthrough 高度切片 | 极低 | 极低 |
| RANSAC 平面拟合（`pcl_ros::SACSegmentation` ）| 中 | 中（可在低分辨率点云上做）|
| 法向量过滤（`pcl::NormalEstimation` ） | 高 | 高，**ARM 上不推荐** |

> 对 S100，**Passthrough 已经够 costmap 用**。RANSAC 留作进阶选项。

---

## 四、Costmap Layer 选择：STVL vs VoxelLayer

Nav2 内置两个能消费 PointCloud2 的 layer：

### 4.1 `nav2_costmap_2d::VoxelLayer`

- 内置在 `nav2_costmap_2d`
- 维护一个 3D 栅格，按"被占据 / 未知 / 自由"三态
- **没有时间衰减**——障碍消失后清除依赖 raycast
- **在动态环境下幽灵障碍多**

### 4.2 `spatio_temporal_voxel_layer (STVL)`

> 上游：[`SteveMacenski/spatio_temporal_voxel_layer`](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)（Steve Macenski，与 SLAM Toolbox 同作者，与 Nav2 紧密配合）

- 第三方包，apt 安装：`ros-humble-spatio-temporal-voxel-layer`
- 用 **OpenVDB 稀疏体素**做 3D 表示（CPU 高效）
- **基于时间衰减**清除：每个体素有"上次观测时间"，超时自动消除——动态环境下大幅减少幽灵障碍
- 支持多输入源（雷达 + RGB-D + 超声波同时喂）

### 4.3 对比表

| 维度 | VoxelLayer | STVL |
|------|-----------|------|
| 安装 | 自带 Nav2 | 第三方（apt） |
| 数据结构 | 稠密 3D bool | 稀疏 OpenVDB |
| 内存占用 | 中（与体素总数线性） | 低（与实际占据数线性）|
| 清除机制 | raycast 反向 | 时间衰减（更鲁棒）|
| 动态环境表现 | 一般 | 优 |
| ARM CPU 占用 | 中 | **更低**（OpenVDB 优化好）|
| 上手难度 | 低 | 低 |
| 推荐度 | 备选 | **★ 首选** |

---

## 五、STVL 完整配置（推荐）

> 配置写在 Nav2 的 `local_costmap` 段：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      track_unknown_space: true
      transform_tolerance: 0.3

      plugins: ["voxel_layer", "stvl_layer", "inflation_layer"]
      # ↑ voxel_layer = 2D LiDAR；stvl_layer = RGB-D 深度

      # ---------- 2D LiDAR 走 ObstacleLayer / VoxelLayer 不变 ----------
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: false
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0

      # ---------- STVL 用于 RGB-D ----------
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 15.0          # s，体素自动消失时间
        decay_model: 0             # 0=Linear, 1=Exponential, -1=Persistent
        voxel_size: 0.05           # m
        track_unknown_space: true
        unknown_threshold: 15
        mark_threshold: 0
        update_footprint_enabled: true
        combination_method: 1       # 0=Overwrite, 1=Maximum
        origin_z: 0.0
        publish_voxel_map: false
        transform_tolerance: 0.2
        mapping_mode: false
        map_save_duration: 60.0
        observation_sources: rgbd_cloud
        rgbd_cloud:
          data_type: PointCloud2
          topic: /depth/points_filtered
          marking: true
          clearing: true
          obstacle_range: 4.0       # m，超出忽略
          min_obstacle_height: 0.05 # m，base_link 系
          max_obstacle_height: 1.8
          expected_update_rate: 0.0 # 0=不强制
          observation_persistence: 0.0
          inf_is_valid: false
          filter: voxel
          voxel_min_points: 0
          clear_after_reading: true
          max_z: 7.0
          min_z: 0.1
          vertical_fov_angle: 0.8727 # rad ≈ 50°（深度相机 FOV）
          horizontal_fov_angle: 1.05 # rad ≈ 60°
          decay_acceleration: 5.0
          model_type: 0              # 0=深度相机；1=3D LiDAR

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

### 5.1 调参矩阵

| 现象 | 调整 |
|------|------|
| 障碍残留太久（人走过仍标记）| ↓ `voxel_decay` 到 5–8s |
| 障碍消失太快（高速时被擦除）| ↑ `voxel_decay` 到 20–30s |
| 远处幽灵障碍 | ↓ `obstacle_range` 到 3.0；或调小 `vertical_fov_angle` 防止天花板入帧 |
| 桌脚检测不到 | 1) 检查深度相机有效距离下限（多数 RealSense 0.3m）；2) ↓ `min_obstacle_height` 到 0.02 |
| CPU 占用高 | ↑ `voxel_size` 到 0.08；↑ depth 相机 voxel_grid `leaf_size` 到 0.08 |

---

## 六、S100 上的 CPU 预算

| 组件 | 估算单核占用（720p depth @ 30Hz）|
|------|--------------------------------|
| `depth_image_proc::PointCloudXyzNode` | 10%–20% |
| `pcl_ros::VoxelGrid` (leaf 0.05) | 5%–10% |
| `pcl_ros::PassThrough` | < 5% |
| STVL 集成 + costmap 更新 | 5%–15% |
| **总计** | **25%–50%** |

降级开关：
1. depth 分辨率从 720p 降到 480p（`/depth/image_rect_raw` remap 前在驱动配置）
2. depth 帧率从 30Hz 降到 15Hz
3. VoxelGrid `leaf_size` 0.05 → 0.08
4. STVL `voxel_size` 0.05 → 0.08

> 经验法则：**只为 costmap 服务时，depth 480p @ 15Hz 完全够**。30Hz 是给视觉感知（VLA / FoundationPose 等）准备的。

---

## 七、楼梯/坠落检测（CPU 方案）

STVL 默认只标记"占据"，不主动检测"地面消失"。简单可行的工程方案：

```
专门的 stair_detector_node：
  ├── 订阅 /depth/points_filtered
  ├── 在 base_link 系下取一个"前方矩形 ROI"（如 x∈[0.3, 0.8], y∈[-0.2, 0.2]）
  ├── 统计 ROI 内 z 平均值
  ├── 若 z_mean < -0.05（地面下沉）→ 发布"虚拟激光点"或直接通过 collision_monitor stop_zone
  └── 简单、CPU 极低、覆盖楼梯下行场景
```

或者：把"地面缺失"映射成 `costmap_2d::FREE_SPACE` 反向被高 cost 区域包围——但这需要自定义 plugin。**首选简单方案 + collision_monitor 兜底**。

---

## 八、何时不用本方案

| 场景 | 替代 |
|------|------|
| 只关心 2D 平面避障（货架、AGV）| 直接用 `nav2_costmap_2d::ObstacleLayer + LaserScan`，本节方案多余 |
| 想要持久 3D 地图（不只是 costmap）| OctoMap CPU 版 / RTAB-Map 内置 OctoMap |
| 极度算力紧张（S100 已满载）| 关闭 STVL，仅留 LaserScan 层；机器人速度限制到 < 0.5m/s |

---

## 九、与已有调研的关系

- `nav2_research/03_costmap2d_plugins.md` — 本节是其 STVL 部分的"完整工程化操作手册"
- `realsense-ros-4.57.7-analysis` — RealSense 在 ROS 2 的话题命名 / camera_info 结构 / TF (`color_optical_frame` 等)
- `orbbec_sdk_ros2_analysis` — Orbbec 类似
- `perception_research/` — nvblox 章节在 S100 上**禁用**；本节是其 CPU 替代
- `realtime_research/` — STVL 主线程同样不要绑控制核

---

下一步：[05_dynamic_and_lifelong.md](./05_dynamic_and_lifelong.md) — Lifelong 模式、动态障碍、地图持久化与版本管理。
