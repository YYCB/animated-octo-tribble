# Nvblox 3D 重建

> 覆盖 Nvblox GPU 加速体素重建架构、关键参数调优、与 Nav2 costmap 集成、人体感知动态障碍物分离，以及 Orin AGX 性能数据。

---

## 一、Nvblox 架构

### 1.1 三层体素结构

Nvblox（NVIDIA，2022）是 GPU 加速的增量式 3D 重建系统，内部维护三个互补的体素层：

```
Nvblox 体素层架构：

输入：RGB-D 帧序列 + 里程计/SLAM 位姿
    │
    ▼
┌──────────────────────────────────────────────────────────────────┐
│  TsdfLayer（截断有符号距离场）                                      │
│  ├── 每个 voxel 存储：tsdf_value（有符号距离）+ weight             │
│  ├── GPU 并行更新（每帧 ~1–5ms）                                   │
│  └── 表面零交叉 → Mesh 提取（Marching Cubes）                      │
├──────────────────────────────────────────────────────────────────┤
│  EsdfLayer（欧氏有符号距离场）                                      │
│  ├── 从 TSDF 导出：每个 voxel 到最近障碍物的欧氏距离               │
│  ├── GPU BFS 传播（~2ms/更新）                                     │
│  └── 直接作为 Nav2 costmap 输入（ESDF 切片 → 2D occupancy）       │
├──────────────────────────────────────────────────────────────────┤
│  ColorLayer（颜色体素）                                            │
│  ├── 每个 voxel 存储 RGB 颜色（语义可选）                           │
│  ├── 用于 Mesh 可视化（Foxglove 3D 面板）                          │
│  └── 可存储语义标签（扩展至 SemanticLayer）                        │
└──────────────────────────────────────────────────────────────────┘
    │
    ▼
输出：
  /nvblox/mesh       (nvblox_msgs/Mesh)    → 可视化
  /nvblox/esdf_slice (sensor_msgs/Image)   → Nav2 costmap
  /nvblox/map_slice  (nav_msgs/OccupancyGrid) → 导航
```

### 1.2 GPU 加速原理

Nvblox 将体素更新操作设计为 CUDA kernel：

- **TSDF 更新**：每个输入点独立更新对应体素及其邻域体素，完全并行
- **ESDF 传播**：GPU BFS 从已知障碍物向外传播距离值（层次化位图加速）
- **Mesh 提取**：Marching Cubes 每个 voxel 独立执行，高度并行

### 1.3 `NvbloxNode` ROS 2 接口

| 输入话题 | 消息类型 | 说明 |
|---------|---------|------|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图（16UC1 或 32FC1） |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图（颜色层可选） |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/odom` 或 `/visual_odometry/pose` | `nav_msgs/Odometry` / `geometry_msgs/PoseStamped` | 位姿估计 |

| 输出话题 | 消息类型 | 说明 |
|---------|---------|------|
| `/nvblox/mesh` | `nvblox_msgs/Mesh` | 彩色 Mesh（可视化） |
| `/nvblox/esdf_slice` | `sensor_msgs/Image` | ESDF 水平切片（障碍物距离图） |
| `/nvblox/map_slice` | `nav_msgs/OccupancyGrid` | 2D 占用栅格（Nav2 直接使用） |
| `/nvblox/depth_processed` | `sensor_msgs/Image` | 滤波后深度图 |

---

## 二、关键参数调优

### 2.1 核心参数说明

```yaml
# nvblox_node.launch.py 参数配置
nvblox_node:
  ros__parameters:
    # ── 体素分辨率 ────────────────────────────────────────────────
    voxel_size: 0.05          # 5cm（默认）；精度 vs 内存权衡
                               # 0.02m → 高精度（操控）；0.10m → 导航

    # ── TSDF 更新频率 ─────────────────────────────────────────────
    max_tsdf_update_hz: 30.0  # 与相机帧率对齐；低配置可降为 10Hz

    # ── ESDF 参数 ─────────────────────────────────────────────────
    esdf_slice_height: 0.5    # ESDF 切片高度（米，相对于机器人底部）
    esdf_2d: true             # 输出 2D 占用图（Nav2 使用）
    esdf_max_distance_m: 2.0  # 最大障碍物感知距离

    # ── 深度滤波 ──────────────────────────────────────────────────
    max_depth_m: 5.0          # 超过此深度的点云丢弃
    min_depth_m: 0.1          # 近距离点云丢弃（防止机器人自遮挡）

    # ── Mesh 输出 ─────────────────────────────────────────────────
    mesh_update_hz: 5.0       # Mesh 更新频率（可视化用，不影响地图）

    # ── 内存管理 ──────────────────────────────────────────────────
    max_voxels_per_layer: 50000000  # GPU 体素内存上限
    use_non_equal_lidar: false       # LiDAR 非等角扫描支持
```

### 2.2 voxel_size 选择建议

| 场景 | voxel_size | GPU 内存（5m×5m×2m 空间） | 备注 |
|------|------------|------------------------|------|
| 精细操控（手部） | 0.02m | ~800MB | 操控任务需精细表面重建 |
| 标准操控 | 0.04m | ~100MB | 推荐 Orin 操控场景 |
| **导航（推荐）** | 0.05m | ~50MB | Orin AGX 导航推荐值 |
| 大场景导航 | 0.10m | ~7MB | 低内存/低精度需求 |

---

## 三、与 Nav2 costmap 集成

### 3.1 NvbloxCostmapLayer 配置

```yaml
# nav2_params.yaml — nvblox costmap layer 集成
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ['static_layer', 'nvblox_layer', 'inflation_layer']
      nvblox_layer:
        plugin: 'nvblox::nav2::NvbloxCostmapLayer'
        enabled: true
        nvblox_map_slice_topic: '/nvblox/map_slice'
        max_obstacle_distance: 1.0    # 超过此距离的体素不加入 costmap
        lethal_obstacle_distance: 0.3 # 致命障碍物距离

local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ['voxel_layer', 'nvblox_layer', 'inflation_layer']
      nvblox_layer:
        plugin: 'nvblox::nav2::NvbloxCostmapLayer'
        nvblox_map_slice_topic: '/nvblox/map_slice'
        rolling_window: true
        width: 5.0
        height: 5.0
```

### 3.2 ESDF 切片 → 2D costmap 转换原理

```
ESDF voxel grid (3D)
    │
    │ 水平切片（esdf_slice_height = 0.5m）
    ▼
2D ESDF slice (W × H float32，米单位距离场)
    │
    │ 阈值映射
    │  distance < 0.3m → LETHAL_OBSTACLE (254)
    │  0.3m < d < 1.0m → 线性衰减代价
    │  distance > 1.0m → FREE_SPACE (0)
    ▼
nav_msgs/OccupancyGrid
    │
    ▼
Nav2 LocalCostmap / GlobalCostmap → MPPI / SmacPlanner
```

---

## 四、`nvblox_msgs/Mesh` 可视化

### 4.1 Foxglove 3D 面板配置

```json
{
  "panel": "3D",
  "layers": [
    {
      "type": "Mesh",
      "topic": "/nvblox/mesh",
      "visible": true,
      "color": "auto"
    },
    {
      "type": "PointCloud",
      "topic": "/camera/depth/points",
      "visible": true,
      "colorField": "z"
    }
  ]
}
```

### 4.2 `nvblox_msgs/Mesh` 消息结构

```python
# nvblox_msgs/Mesh 结构
header:  Header
# 顶点数组（xyz）
vertices: geometry_msgs/Vector3[]
# 三角形索引
triangles: geometry_msgs/Point[]
# 顶点颜色（RGB）
vertex_colors: std_msgs/ColorRGBA[]
# block_indices: 更新了哪些 block（差分更新）
block_indices: nvblox_msgs/Index3D[]
```

---

## 五、人体感知模式

### 5.1 nvblox_human_detection 架构

```
RGB 图像
    │
    ▼
nvblox_human_segmentation（人体实例分割）
  ├── PeopleSemSegNet（NVIDIA TRT 推理）
  └── 输出：人体 mask（semantic_segmentation/image）
    │                          │
    ▼                          ▼
深度图 + 人体 mask         正常深度图（非人体区域）
    │                          │
    ▼                          ▼
人体专用点云              场景静态 TSDF 更新
（动态障碍物追踪）         （忽略人体区域）
    │
    ▼
/nvblox/human_detections (Detection3DArray)
→ Nav2 局部 costmap 动态障碍物层
```

### 5.2 配置示例

```yaml
# nvblox_node with human detection
nvblox_node:
  ros__parameters:
    use_human_segmentation: true
    human_segmentation_model_path: '/models/people_semseg_net.engine'
    human_voxel_size: 0.10           # 人体体素较粗（速度优先）
    people_decay_time_s: 2.0         # 人体离开后多久清除体素
```

---

## 六、性能数据（Orin AGX）

### 6.1 典型工作负载性能

| 配置 | 更新延迟（per 帧） | GPU 内存 | CPU 占用 | 备注 |
|------|----------------|---------|---------|------|
| `voxel_size=0.05`, 深度 640×480, 30Hz | ~8ms | 400MB | 15% | Orin AGX 推荐配置 |
| `voxel_size=0.05`, 深度 848×480, 30Hz | ~11ms | 450MB | 18% | RealSense D435 分辨率 |
| `voxel_size=0.03`, 深度 848×480, 30Hz | ~22ms | 1.2GB | 25% | 高精度操控 |
| `voxel_size=0.05` + 人体分割, 30Hz | ~18ms | 700MB | 28% | 含 PeopleSemSegNet |
| `voxel_size=0.10`, 深度 640×480, 10Hz | ~3ms | 50MB | 5% | 超低功耗模式 |

### 6.2 内存占用估算

```python
# voxel 内存估算
def estimate_memory(voxel_size_m: float,
                    scene_size_m: tuple = (10, 10, 3)) -> float:
    """估算 TSDF + ESDF + Color 层总 GPU 内存（MB）"""
    voxels_per_axis = [s / voxel_size_m for s in scene_size_m]
    total_voxels = voxels_per_axis[0] * voxels_per_axis[1] * voxels_per_axis[2]

    # 每个 voxel 占用（字节）：TSDF(8) + ESDF(8) + Color(4) + 权重(4) = 24B
    bytes_per_voxel = 24
    total_mb = total_voxels * bytes_per_voxel / 1024 / 1024

    return total_mb

# 示例
print(estimate_memory(0.05, (10, 10, 3)))  # ≈ 576MB（稀疏存储后实际更少）
```

> **稀疏存储优化**：Nvblox 使用 Block-based 稀疏存储（每个 Block = 8×8×8 voxel），仅分配被观测到的 Block，实际内存比上述估算低 50–80%（取决于场景稀疏程度）。
