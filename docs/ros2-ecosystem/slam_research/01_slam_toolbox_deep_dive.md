# SLAM Toolbox 深度调研

> 上游：[`SteveMacenski/slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox)
> 关联：本专题 [00_index.md](./00_index.md) §三、`nav2_research/06_localization.md` §三

`slam_toolbox` 是 ROS 2 上 2D LiDAR SLAM 的事实标准，其核心是 **Karto 风格的 pose-graph SLAM** + Ceres 后端 + 多模式（建图 / 定位 / 终生学习）切换。本文从内部数据结构、状态机、参数全表、ARM CPU 性能预算四个维度展开。

---

## 一、包结构与节点类型

```
slam_toolbox/
├── lib/karto_sdk/        ← Karto SLAM 核心算法（C++，无 ROS 依赖）
│   ├── Mapper.cpp        ← scan-matcher + pose graph 构造
│   ├── ScanMatcher.cpp   ← 相关性扫描匹配（CSM）
│   └── Karto.cpp         ← 核心数据结构（LocalizedRangeScan、Vertex/Edge）
├── solvers/
│   ├── ceres_solver.cpp  ← 默认后端（Levenberg-Marquardt + Ceres）
│   ├── g2o_solver.cpp    ← 备选后端
│   └── spa_solver.cpp    ← 旧版 SPA 后端
├── src/
│   ├── slam_toolbox_async.cpp      ← node: AsyncSlamToolboxNode（推荐用于建图）
│   ├── slam_toolbox_sync.cpp       ← node: SyncSlamToolboxNode（确定性，慢）
│   ├── slam_toolbox_localization.cpp  ← node: LocalizationSlamToolboxNode
│   ├── slam_toolbox_lifelong.cpp   ← node: LifelongSlamToolboxNode
│   └── slam_toolbox_map_and_localization.cpp ← 同时建图 + 定位
└── config/                ← 参数样例
```

可执行节点对应关系：

| 可执行 | 模式 | 适用阶段 | S100 上的建议 |
|--------|------|---------|--------------|
| `async_slam_toolbox_node` | Online Async Mapping | 首次建图 | ✅ **推荐**：CPU 调度更柔，不阻塞 |
| `sync_slam_toolbox_node` | Online Sync Mapping | 严格确定性建图（很少需要） | ⚠️ 在 ARM 上可能跟不上 |
| `localization_slam_toolbox_node` | Localization Only | 部署运行 | ✅ **推荐**：替代 AMCL |
| `lifelong_slam_toolbox_node` | Lifelong Mapping | 长期演化 | ⚠️ 资源占用高于 localization，需评估 |

---

## 二、内部数据结构（pose-graph SLAM）

### 2.1 三个核心对象

```
LocalizedRangeScan
  ├── 一帧激光（被 Karto 接收后立即拷贝去畸变 + 坐标系归一）
  ├── 关键属性：corrected_pose（修正后位姿）、odom_pose（原始里程计）、scan ranges
  └── 是 pose graph 中"顶点"的载体

Vertex（顶点）
  ├── 持有一个 LocalizedRangeScan
  ├── unique_id：单调递增整数
  └── 是图优化的优化变量

Edge（边）
  ├── 类型 A：连续帧约束（相邻 scan_match 得到的相对位姿）
  ├── 类型 B：回环约束（loop closure 找到的相对位姿）
  └── 每条边带 covariance（协方差），决定优化时的"权重"
```

### 2.2 处理流程（伪代码）

```cpp
// karto_sdk/Mapper.cpp::process()
bool Mapper::Process(LocalizedRangeScan* pScan) {
    // 1) 距离/角度阈值过滤（minimum_travel_distance / heading）
    if (!HasMovedEnough(pScan, prev_scan_)) return false;

    // 2) Scan matching：与"running scans"（最近 N 帧）做相关性匹配
    Pose2 best_pose = scan_matcher_->MatchScan(
        pScan, running_scans_, /*coarse + fine*/);
    pScan->SetCorrectedPose(best_pose);

    // 3) 加入 pose graph：新建 Vertex + Edge(连续帧)
    auto vertex = graph_->AddVertex(pScan);
    graph_->AddEdge(prev_vertex_, vertex, relative_pose, covariance);

    // 4) 回环检测：在 graph 中搜索"近邻 Vertex 但 unique_id 远的"
    auto loop_candidates = graph_->FindNearLinkedScans(pScan, loop_search_radius);
    for (auto* candidate : loop_candidates) {
        Pose2 lc_pose;
        if (scan_matcher_->MatchScan_Loop(pScan, candidate, &lc_pose, &cov)) {
            graph_->AddEdge(candidate->Vertex(), vertex, lc_pose, cov);
        }
    }

    // 5) 触发图优化（Ceres）
    solver_->Compute();

    // 6) 更新 OccupancyGrid（异步，每 minimum_time_interval 一次）
    return true;
}
```

### 2.3 `.posegraph` / `.data` 文件结构

序列化通过 boost::serialization（XML/Binary 二选一）。一个完整地图包含三个文件：

| 文件 | 内容 | 大小量级（中型办公室）|
|------|------|---------------------|
| `my_map.posegraph` | Karto::Mapper 整个对象（vertices + edges + graph） | 10–80 MB |
| `my_map.data` | LocalizedRangeScan 原始数据（每帧 ~5–8 KB × N 帧）| 与扫描帧数线性相关 |
| `my_map.yaml` + `my_map.pgm` | OccupancyGrid 渲染产物（导出为 nav2_map_server 可用格式） | < 5 MB |

> ⚠️ **lifelong 模式下 `.data` 会持续增长**。如果 24h 不重启，建议加 `transform_publish_period: 0.02` 之外的 map decay 配置（详见 [05](./05_dynamic_and_lifelong.md)）。

---

## 三、Scan Matcher（CPU 热点 #1）

### 3.1 算法：相关性扫描匹配（Correlative Scan Matching）

```
两阶段 grid search：
  Coarse stage:
    搜索范围 ±correlation_search_space_dimension（默认 0.3 m × 0.3 m × ±π/4）
    分辨率 correlation_search_space_resolution（默认 0.01 m）
    × angle_resolution（默认 ~0.0349 rad ≈ 2°）
    打分函数：把候选位姿的激光点投到栅格化的"参考扫描占据图"，求重合分数
  Fine stage:
    在 coarse 最优解附近做更细的网格（fine_correlation_search_space_*）
```

**算法复杂度** ≈ `O((dx/res_x) × (dy/res_y) × (dθ/res_θ) × N_points)`，对 ARM 单核非常敏感。

### 3.2 三个独立的 Matcher 实例

```
Mapper 内部维护 3 个 ScanMatcher：
  ├── m_pSequentialScanMatcher  ← 用于连续帧匹配（频繁调用）
  ├── m_pGraphScanMatcher       ← 用于回环检测（偶尔，搜索更大）
  └── m_pLoopScanMatcher        ← 优化后回放（不常用）
```

每个 matcher 都对应一组独立的 grid search 参数，参数表中以前缀 `loop_match_` / `link_match_` / `correlation_search_space_` 区分。

### 3.3 ARM CPU 占用经验

> 以 RPLIDAR A2（10 Hz、360 点 / 帧）+ Cortex-A55 类 ARM 核为参考量级。S100 主频/核心代差具体数值需实测，下表给出**相对趋势**而非绝对值。

| 阶段 | 主要负载 | 单核占用估计 |
|------|---------|------------|
| 连续帧 ScanMatcher | coarse(0.3m,0.0349rad) + fine | 30%–50% |
| 回环 ScanMatcher（每秒触发一次以下）| 范围更大 | 阵发性 60%–90% |
| Ceres optimize | 顶点 < 200 时基本在毫秒级；顶点 > 1000 后明显 | 10%–40%（阵发） |
| OccupancyGrid 渲染 | 每 minimum_time_interval 一次 | 5%–20%（阵发） |

**S100 调度建议**（参考 `realtime_research/`）：把 SLAM Toolbox 主线程绑核到非控制环 CPU，避免抢占 chassis_protocol 的实时控制循环。

---

## 四、参数全表（建图 / 定位双视图）

### 4.1 通用参数（所有模式共用）

```yaml
slam_toolbox:
  ros__parameters:
    # ---------- 帧/话题 ----------
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint   # 与 chassis_protocol 一致
    scan_topic: /scan
    use_map_saver: true          # 提供 /slam_toolbox/save_map 服务

    # ---------- 求解器 ----------
    solver_plugin: solver_plugins::CeresSolver  # 默认；可选 G2oSolver / SpaSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY # ARM 上推荐 SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG       # 仅在 trust_strategy=DOGLEG 时生效
    ceres_loss_function: None                   # 可选 HuberLoss（对 outlier 更鲁棒）

    # ---------- 频率/时序 ----------
    transform_publish_period: 0.02      # s，发布 map→odom TF 周期，50 Hz
    map_update_interval: 5.0            # s，重新发布 /map 的最大间隔
    minimum_time_interval: 0.5          # s，两帧 scan 之间最短间隔（节流）
    transform_timeout: 0.2              # s，TF 等待超时

    # ---------- LiDAR 几何 ----------
    resolution: 0.05                    # m，OccupancyGrid 分辨率
    max_laser_range: 12.0               # m，超出则丢弃；2D LiDAR 室内 8–15 推荐
    minimum_travel_distance: 0.5        # m，触发新关键帧的最小平移
    minimum_travel_heading: 0.5         # rad ≈ 28.6°
    scan_buffer_size: 10                # 滑窗：参与连续帧匹配的最近帧数
    scan_buffer_maximum_scan_distance: 10.0  # m，滑窗内最大累积位移
```

### 4.2 ScanMatcher 参数（CPU 调优重点）

```yaml
    # ---------- 连续帧匹配 ----------
    use_scan_matching: true
    use_scan_barycenter: true           # 以激光重心而非原点对齐，鲁棒性 ↑
    correlation_search_space_dimension: 0.3      # m，搜索半径
    correlation_search_space_resolution: 0.01    # m，搜索步长
    correlation_search_space_smear_deviation: 0.03 # m，模糊化标准差
    angle_variance_penalty: 1.0
    distance_variance_penalty: 0.5
    fine_search_angle_offset: 0.00349    # rad ≈ 0.2°
    coarse_search_angle_offset: 0.349    # rad ≈ 20°
    coarse_angle_resolution: 0.0349      # rad ≈ 2°
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true         # 增大搜索范围直到匹配成功

    # ---------- 回环 ----------
    do_loop_closing: true
    loop_search_maximum_distance: 3.0    # m，回环候选最远距离
    loop_match_minimum_chain_size: 10    # 回环链最小顶点数
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45  # 越大越严格（少假阳性，多漏检）
```

**S100 / ARM 弱核降级建议**：

| 参数 | 默认 | ARM 紧张时 | 影响 |
|------|------|-----------|------|
| `correlation_search_space_resolution` | 0.01 | **0.025** | CPU↓ ~6×；精度 ↓ |
| `coarse_angle_resolution` | 0.0349 | **0.0698** | CPU↓ 2× |
| `correlation_search_space_dimension` | 0.3 | **0.2** | CPU↓ ~2.25×；适合慢速机器人 |
| `minimum_time_interval` | 0.5 | **0.8** | 关键帧密度 ↓ |
| `loop_search_maximum_distance` | 3.0 | **2.0** | 回环搜索范围小，假阳性少 |

### 4.3 模式专属参数

#### Mapping (`async_slam_toolbox_node`)

```yaml
    mode: mapping
    enable_interactive_mode: true   # 通过 RViz Plugin 手动加约束/删顶点
    debug_logging: false
```

#### Localization (`localization_slam_toolbox_node`)

```yaml
    mode: localization
    map_file_name: /maps/my_map     # 不带扩展名
    map_start_at_dock: true         # true: 用 dock 位姿初始化；false: 等 /initialpose
    # 或显式：
    # map_start_pose: [0.0, 0.0, 0.0]   # x, y, yaw

    # 定位模式下有些参数无效（如 map_update_interval），但保留即可
```

#### Lifelong (`lifelong_slam_toolbox_node`)

```yaml
    mode: lifelong
    map_file_name: /maps/my_map
    lifelong_search_use_tree: true    # 用 KD-tree 加速 vertex 搜索
    lifelong_minimum_score: 0.1       # 顶点 utility score 低于此值会被裁剪
    lifelong_node_removal_score: 0.04
    lifelong_overlap_score_scale: 0.06
    lifelong_constraint_multiplier: 0.08
    lifelong_nearby_penalty: 0.001
    lifelong_candidates_scale: 0.03
```

详见 [05_dynamic_and_lifelong.md](./05_dynamic_and_lifelong.md) §三。

---

## 五、TF 与话题接口

### 5.1 输入

| Topic | Type | 来源 | QoS |
|-------|------|------|-----|
| `/scan` | `sensor_msgs/LaserScan` | 雷达驱动 / `hobot_sensors`（S100） | SensorDataQoS（BEST_EFFORT, depth=5）|
| `/initialpose`（可选） | `geometry_msgs/PoseWithCovarianceStamped` | RViz | RELIABLE |
| TF: `odom → base_link` | tf2 | **chassis_protocol + ekf_node**（见 [02](./02_odom_fusion_with_robot_localization.md)） | — |
| TF: `base_link → laser_frame` | tf2（静态）| URDF + `robot_state_publisher` | — |

### 5.2 输出

| Topic / TF | 含义 |
|-----------|------|
| `/map` (`OccupancyGrid`, latched / TRANSIENT_LOCAL) | 当前栅格地图 |
| `/map_metadata` | 与上述同步 |
| TF: `map → odom` | 由 slam_toolbox 直接广播；publish_period 由 `transform_publish_period` 控制 |
| `/slam_toolbox/graph_visualization` (`MarkerArray`) | RViz 可视化 pose graph |
| `/slam_toolbox/scan_visualization` | 当前帧匹配可视化 |

### 5.3 服务

| Service | 用途 |
|---------|------|
| `/slam_toolbox/save_map` (`SaveMap`) | 导出 `.pgm + .yaml`（OccupancyGrid 形式）|
| `/slam_toolbox/serialize_map` (`SerializePoseGraph`) | 导出 `.posegraph + .data`（**部署必备**） |
| `/slam_toolbox/deserialize_map` (`DeserializePoseGraph`) | 加载已建好的图，可继续编辑/定位 |
| `/slam_toolbox/clear_changes` | 清空交互模式中的未提交编辑 |
| `/slam_toolbox/manual_loop_closure` | 手动添加回环约束（环境对称导致漏检时使用）|
| `/slam_toolbox/toggle_interactive_mode` | 启用 / 关闭 RViz 交互编辑 |

---

## 六、典型故障与排查

| 症状 | 可能原因 | 排查/解决 |
|------|---------|----------|
| 地图"漂"，长走廊重影 | scan matcher 在对称环境信任激光过多 | ↑ `minimum_travel_distance`、检查 IMU/odom 是否进入 EKF（[02](./02_odom_fusion_with_robot_localization.md)）|
| `map → odom` TF 跳变剧烈 | 回环优化把 graph 一次性拉回；odom 漂移过大 | 改进 odom（融合 IMU）；调高 `transform_publish_period` 让跳变可视 |
| 回环不触发 | `loop_search_maximum_distance` < 实际累积漂移 | ↑ 该值；或用 `/slam_toolbox/manual_loop_closure` 兜底 |
| 回环误触发，地图扭曲 | `loop_match_minimum_response_fine` 太松 | 0.45 → 0.55；启用 `ceres_loss_function: HuberLoss` |
| 启动后无 `/map` 输出 | 第一帧 scan 在 odom→base_link TF 不可用前到达 | 调大 `transform_timeout`；检查 `robot_state_publisher` 是否先于 SLAM 启动 |
| `lifelong` 模式 CPU/内存不断增长 | utility score 阈值过松，老顶点未被裁剪 | ↓ `lifelong_minimum_score` 阈值（更激进裁剪）|
| 定位模式启动失败：`Failed to load map` | `.posegraph` 路径错误 / 不带扩展名却传了扩展名 | 路径必须**不含扩展名**，且 `.posegraph` 与 `.data` 同目录 |
| Localization 模式偶发"丢失" | 与建图时雷达高度/角度有差异；FOV 不一致 | 重新检查 `base_link → laser_frame` TF；必要时重建图 |

---

## 七、S100 主控下的 CPU 预算与排布建议

> 假设：S100 上分配 **2 个 ARM 核** 给 SLAM 栈；其它核用于 chassis_protocol 实时控制环、Nav2、感知。

```
Core 0  ────  chassis_protocol（实时控制环 + odom 发布）  ← 隔离 / SCHED_FIFO
Core 1  ────  Nav2 controller_server / planner_server
Core 2  ────  slam_toolbox 主线程（ScanMatcher + Mapper.Process）
Core 3  ────  ekf_node + STVL + depth_image_proc + Ceres 后端线程

不严格隔离时，至少保证 chassis_protocol 在独占核上跑（参考 realtime_research/）
```

### 推荐起步参数（S100 + RPLIDAR / Slamtec 类）

```yaml
slam_toolbox:
  ros__parameters:
    resolution: 0.05
    max_laser_range: 10.0
    minimum_time_interval: 0.5
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    transform_publish_period: 0.05         # 20 Hz 已足够，省 CPU
    correlation_search_space_resolution: 0.02   # 默认 0.01 减半
    coarse_angle_resolution: 0.0698        # 默认 0.0349 翻倍
    use_scan_barycenter: true
    do_loop_closing: true
    loop_search_maximum_distance: 2.5
    loop_match_minimum_response_fine: 0.5  # 收紧
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_loss_function: HuberLoss
```

实测后再按 §四.2 的降级矩阵微调。

---

## 八、与 Nav2 / chassis_protocol 的衔接要点

- TF 链：`map`(slam_toolbox) → `odom`(ekf_node) → `base_link`(ekf_node) → `laser_frame`(URDF)
  - **不要让 chassis_protocol 直接发 odom→base_link** —— 这条链交给 ekf_node。chassis_protocol 配置 `enable_odom_tf: false`。
- 部署阶段切换到 `localization_slam_toolbox_node` 后，地图来自 `.posegraph` 反序列化，但 Nav2 仍然需要一个 `/map` topic 让 costmap StaticLayer 订阅 → slam_toolbox 在 localization 模式下会**继续发布 `/map`**（不会更新内容），无需再启动 `nav2_map_server`。
- 详细 QoS / 启动顺序见 [07_chassis_and_s100_integration.md](./07_chassis_and_s100_integration.md)。

---

## 九、源码引用速查

| 主题 | 路径（上游 `slam_toolbox`，commit ROS 2 Humble HEAD 附近）|
|------|----------------------------------------------------|
| Karto 主流程 | `lib/karto_sdk/src/Mapper.cpp::Process` |
| ScanMatcher | `lib/karto_sdk/src/Karto.cpp::ScanMatcher` |
| Ceres solver | `solvers/ceres_solver.cpp` |
| Async node | `src/slam_toolbox_async.cpp` |
| Localization node | `src/slam_toolbox_localization.cpp` |
| Lifelong node | `src/experimental/slam_toolbox_lifelong.cpp` |
| 序列化 | `src/serialization.cpp`（boost::serialization）|

---

下一步：[02_odom_fusion_with_robot_localization.md](./02_odom_fusion_with_robot_localization.md) — 让 SLAM Toolbox 真正稳定的关键前提。
