# 定位方案 — AMCL / SLAM Toolbox / nav2_collision_monitor

---

## 一、Nav2 定位体系概述

Nav2 的定位负责维护 `map → odom → base_link` 的 TF 变换链：

```
map ──(AMCL / SLAM Toolbox)──→ odom ──(里程计)──→ base_link
  │                                                    │
  │              TF 链说明                             │
  │  map → odom：由定位节点发布（修正累积误差）          │
  │  odom → base_link：由底盘里程计发布（高频，连续）    │
  └────────────────────────────────────────────────────┘

chassis_protocol 的职责：
  发布 /odom (nav_msgs/Odometry)
  发布 TF：odom → base_link（通过 odometry publisher）
```

---

## 二、AMCL（Adaptive Monte Carlo Localization）

### 2.1 算法原理

AMCL 是**粒子滤波**定位算法，适合在**已知地图**上定位：

```
输入：
  - /scan（激光雷达 LaserScan）
  - /map（静态地图）
  - /odom（里程计，用于粒子运动预测）

输出：
  - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
  - TF: map → odom

算法流程：
  1. 初始化：从 /initialpose 生成 N 个粒子（位置假设）
  2. 预测步：用里程计运动模型移动粒子
  3. 更新步：用激光雷达与地图的匹配程度对粒子重新加权
  4. 重采样：高权重粒子存活，低权重粒子消失（Adaptive = 自适应粒子数）
  5. 估计：粒子集的加权均值 = 当前位置估计
```

### 2.2 关键参数

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2      # 旋转导致旋转的噪声（里程计模型参数）
    alpha2: 0.2      # 平移导致旋转的噪声
    alpha3: 0.2      # 平移导致平移的噪声
    alpha4: 0.2      # 旋转导致平移的噪声
    alpha5: 0.2      # （只在 omni 模型使用）
    base_frame_id: base_footprint  # 机器人底盘坐标系
    beam_skip_distance: 0.5        # m，跳过超过此距离的激光束（遮挡容忍）
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: map
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0  # m，激光与地图匹配最大距离
    laser_max_range: 100.0          # m，使用的激光雷达最大测距
    laser_min_range: -1.0
    laser_model_type: likelihood_field  # beam 或 likelihood_field（更常用）
    max_beams: 60                   # 每次更新使用的激光束数
    max_particles: 2000             # 最大粒子数
    min_particles: 500              # 最小粒子数
    odom_frame_id: odom
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1             # 每隔几步重采样一次
    robot_model_type: nav2_amcl::DifferentialMotionModel  # 或 OmniMotionModel
    save_pose_rate: 0.5              # Hz，保存位姿到参数服务器（重启恢复）
    sigma_hit: 0.2
    tf_broadcast: true               # 是否发布 map → odom TF
    transform_tolerance: 1.0        # s，TF 超时容忍
    update_min_a: 0.2               # rad，触发更新的最小旋转量
    update_min_d: 0.25              # m，触发更新的最小平移量
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
    map_topic: map
    set_initial_pose: false          # 是否从参数自动设置初始位姿
    initial_pose:                    # 如果 set_initial_pose: true
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

### 2.3 AMCL 的局限性

- **需要预先建好的静态地图**（不支持动态地图）
- **绑架问题**（Kidnapped Robot Problem）：机器人被搬移后难以重新定位
  - 解决：`global_localization` 服务（全局重定位，粒子均匀撒布）
- 开放/无特征环境中精度差（走廊末端相似）

---

## 三、SLAM Toolbox（在线建图 + 定位）

### 3.1 工作模式

SLAM Toolbox 支持三种模式：

| 模式 | 功能 | 适用场景 |
|------|------|---------|
| **Online Async Mapping** | 实时建图（生成 /map），同时定位 | 首次探索未知环境 |
| **Lifelong Mapping** | 在已有地图上更新（动态地图）| 长期运行，环境缓慢变化 |
| **Localization Only** | 仅定位（加载已有 .posegraph）| 已建图完成，仅定位 |

### 3.2 配置（Localization 模式，替代 AMCL）

```yaml
slam_toolbox:
  ros__parameters:
    # 模式：localization（仅定位，不更新地图）
    mode: localization
    map_file_name: /maps/my_map  # .posegraph 文件路径（无扩展名）
    map_start_at_dock: true      # 从 dock 位置开始定位
    
    # 传感器
    scan_topic: /scan
    use_scan_matching: true
    use_scan_barycenter: true
    
    # 里程计
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    
    # 性能
    resolution: 0.05             # m，地图分辨率
    max_laser_range: 20.0        # m
    minimum_time_interval: 0.5  # s，最小地图更新间隔
    
    # 回环检测（localization 模式下不建图但仍做回环）
    loop_search_maximum_distance: 3.0  # m
    do_loop_closing: true
```

### 3.3 AMCL vs SLAM Toolbox 对比

| 特性 | AMCL | SLAM Toolbox（Localization）|
|------|:----:|:---:|
| 建图能力 | ❌（仅定位）| ✅（可在线更新）|
| 计算开销 | 低 | 中 |
| 精度（环境丰富）| 高 | 高 |
| 精度（走廊/对称）| 中 | 高（回环检测）|
| 绑架恢复 | 差 | 较好（回环检测）|
| 动态地图支持 | ❌ | ✅（Lifelong）|
| ROS 2 成熟度 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |

**建议**：
- 静态室内环境，计算资源有限 → **AMCL**
- 动态/大型环境，需要持续建图 → **SLAM Toolbox**
- 首次建图阶段 → SLAM Toolbox（Online Async）
- 部署运行阶段 → AMCL 或 SLAM Toolbox（Localization）

---

## 四、nav2_collision_monitor

### 4.1 角色

`nav2_collision_monitor` 是一个**独立于导航栈的安全监控节点**，不依赖 costmap，直接处理传感器数据，在检测到危险时**减速或停止**机器人。

```
用途：
  - 作为最后一道软件安全保障（独立于 controller_server）
  - 处理 controller_server 来不及反应的近距离障碍
  - 可以在导航栈其他部分失效时继续工作
```

### 4.2 工作原理

```
┌────────────────────────────────────────────────────────────────┐
│  nav2_collision_monitor                                        │
│                                                                │
│  传感器输入：                                                   │
│    /scan (LaserScan) → 检测多边形区域内的点                     │
│    /pointcloud (PointCloud2)                                   │
│    /range (Range，超声波/红外）                                 │
│                                                                │
│  监控区域（Polygon）：                                          │
│    stop_zone（停止区）：前方 20cm 内有障碍 → 速度置零            │
│    slowdown_zone（减速区）：前方 50cm 内有障碍 → 速度乘以 0.5   │
│    approach_zone（接近区）：前方 1m 内有障碍 → 速度按距离递减   │
│                                                                │
│  输出：                                                        │
│    /cmd_vel_smoothed → 经过监控/修改的速度命令                  │
│    （原始 /cmd_vel 来自 controller_server，输出发给底盘）        │
└────────────────────────────────────────────────────────────────┘
```

### 4.3 配置示例

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: cmd_vel    # 接收 controller_server 的输出
    cmd_vel_out_topic: cmd_vel_smoothed  # 发给底盘的修改后速度
    state_topic: collision_monitor_state
    transform_tolerance: 0.2
    source_timeout: 1.0          # s，传感器数据超时
    
    # 监控区域定义
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: polygon
      points: "[ [0.3, 0.3], [0.3, -0.3], [-0.05, -0.3], [-0.05, 0.3] ]"
      action_type: approach      # stop / slowdown / approach / limit
      max_points: 20             # 区域内超过此点数才触发
      slowdown_ratio: 0.3        # 减速到原速度的 30%（approach 模式）
      time_before_collision: 2.0 # s，预计碰撞时间阈值
      simulation_time_step: 0.1
      visualize: true
      polygon_pub_topic: approach_polygon

    # 停止区（近距离紧急停止）
    polygons: ["StopZone", "FootprintApproach"]
    StopZone:
      type: polygon
      points: "[ [0.2, 0.2], [0.2, -0.2], [-0.1, -0.2], [-0.1, 0.2] ]"
      action_type: stop
      max_points: 3

    # 传感器源
    observation_sources: ["scan"]
    scan:
      type: scan
      topic: /scan
```

### 4.4 collision_monitor 在本仓库的集成位置

```
controller_server → /cmd_vel
                          ↓
               nav2_collision_monitor（Safety Layer）
                          ↓
               /cmd_vel_smoothed
                          ↓
               chassis_protocol（DiffDriveController）
```

**注意**：需要更新 chassis_protocol 的 `cmd_vel` 订阅 topic 为 `/cmd_vel_smoothed`（或通过 `topic_remapping` 实现）。

---

## 五、nav2_map_server

### 5.1 功能

`nav2_map_server` 提供三个服务：

| 服务/Topic | 功能 |
|-----------|------|
| `/map` (topic) | 发布静态地图（OccupancyGrid）|
| `/load_map` (service) | 动态加载新地图文件 |
| `/save_map` (service) | 保存当前地图到文件 |

### 5.2 地图文件格式

```yaml
# my_map.yaml（地图元信息）
image: my_map.pgm         # PGM 格式的占用网格图片
resolution: 0.05          # m/pixel
origin: [-10.0, -10.0, 0.0]  # 地图左下角坐标（x, y, yaw）
negate: 0                 # 是否反转像素值
occupied_thresh: 0.65     # 像素 > 此阈值 = 占用
free_thresh: 0.196        # 像素 < 此阈值 = 自由
```
