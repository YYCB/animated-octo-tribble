# Lifelong 建图与动态环境处理

> 关联：[01_slam_toolbox_deep_dive.md](./01_slam_toolbox_deep_dive.md) §四.3（Lifelong 参数表）、[04_3d_perception_for_2d_nav.md](./04_3d_perception_for_2d_nav.md)（STVL 时间衰减）

> 本文回答三个工程问题：
> 1. 长期运行下，**已建好的地图怎么自动跟随环境变化**？
> 2. 动态物体（人、推车、临时陈列）怎么不被永久写进地图？
> 3. 地图怎么版本化、回滚、灾备？

---

## 一、三种"地图随时间演化"的策略

| 策略 | 何时用 | 实现方式 |
|------|-------|---------|
| **冻结 + 重建** | 环境变化大、可计划停机重测 | Localization 模式跑日常，每月手动重新跑 mapping |
| **Lifelong** | 环境缓慢渐变（家具搬动、季节摆设） | `lifelong_slam_toolbox_node` 持续更新 pose graph + 老顶点裁剪 |
| **静态地图 + 时间衰减 costmap** | 动态主要靠局部感知应对 | Localization + STVL 时间衰减（[04](./04_3d_perception_for_2d_nav.md)）|

> **本专题主推：Lifelong + STVL 组合**。Lifelong 处理"几小时到几天"尺度的变化，STVL 处理"秒到分钟"尺度的瞬时动态。

---

## 二、Lifelong Mapping 工作机制

### 2.1 与 Mapping / Localization 的差异

```
Mapping（async）：
  ├── 每帧都加 vertex
  ├── pose graph 持续增长，无裁剪
  └── 适合首次建图，跑 30 分钟就该停

Localization：
  ├── 加载已有 .posegraph，不再增加 vertex
  ├── 仅做 scan→graph 的"重定位"
  └── 资源占用最低

Lifelong：
  ├── 加载已有 .posegraph，**继续增加新 vertex**
  ├── 同时根据 utility score **裁剪老 vertex**（保持图大小有界）
  ├── 触发回环、约束更新、graph optimize
  └── 资源占用 > Localization，< Mapping
```

### 2.2 Utility Score（裁剪算法）

每个 vertex 持有一个动态 score，越低越"无用、可删":

```
utility(v) = α₁ × overlap(v, neighbors)
           + α₂ × constraint_strength(v)
           - α₃ × proximity_penalty(v)
           + α₄ × candidates_count(v)
```

对应参数（来自 [01](./01_slam_toolbox_deep_dive.md) §四.3）：

| 参数 | 含义 |
|------|------|
| `lifelong_minimum_score: 0.1` | 低于该值 → 标记为"待删" |
| `lifelong_node_removal_score: 0.04` | 低于该值 → 立即删除（不再保留连接边）|
| `lifelong_overlap_score_scale: 0.06` | 与邻居重叠权重 α₁ |
| `lifelong_constraint_multiplier: 0.08` | 约束强度权重 α₂ |
| `lifelong_nearby_penalty: 0.001` | 近邻惩罚 α₃（避免冗余）|
| `lifelong_candidates_scale: 0.03` | 候选数权重 α₄ |
| `lifelong_search_use_tree: true` | 用 KD-tree 加速候选搜索（必开）|

### 2.3 Lifelong 的代价

| 代价 | 实测趋势 |
|------|---------|
| CPU | 比 Localization 高 30%–60%，主要在 graph optimize 阵发 |
| 内存 | 顶点数稳态约束在 ~2k–5k；不裁剪时无界增长 |
| `.data` 文件持续增大 | 每个 vertex 配套一帧 LocalizedRangeScan，~5–8KB |
| 地图"漂移" | 偶发：错误回环把已稳定地图扭曲——需要监控 |

### 2.4 何时**不**该用 Lifelong

- 环境频繁大变（如布展间隙）：用 Mapping → 重做
- 极致 CPU 紧张（S100 + Nav2 + STVL + 控制环已满载）：用 Localization
- 没有持久化磁盘（容器化 ephemeral 部署）：Lifelong 没意义

---

## 三、动态物体的处理（人、推车、椅子）

### 3.1 三层防御

```
┌─────────────────────────────────────────────────┐
│  Layer 1：感知阶段过滤（最难，可选）              │
│   ├── DynaSLAM / DS-SLAM 思路：分割人/车并跳过该区域│
│   └── 工程降级：实测在 ARM CPU 上跑不动；放弃       │
├─────────────────────────────────────────────────┤
│  Layer 2：建图阶段过滤（务实）                   │
│   ├── 提高 minimum_travel_distance：减少抓拍频率   │
│   ├── Lifelong utility score 自动裁剪短期 vertex   │
│   └── 必要时手动用 RViz 交互模式删除问题 vertex     │
├─────────────────────────────────────────────────┤
│  Layer 3：运行阶段过滤（必做）                   │
│   ├── STVL 时间衰减（voxel_decay = 5–15s）        │
│   ├── nav2_costmap_2d 的 ObstacleLayer raycast 清除│
│   └── 全部基于 CPU，S100 完全胜任                  │
└─────────────────────────────────────────────────┘
```

### 3.2 实战配方

> **目标：地图（OccupancyGrid）记录"长期不变的结构"；costmap 记录"瞬时占据"**

```
SLAM Toolbox（lifelong）参数收紧：
  minimum_travel_distance: 0.8     ← 默认 0.5 上调，少抓人移动
  minimum_time_interval: 1.0       ← 默认 0.5 上调
  loop_match_minimum_response_fine: 0.55  ← 收紧回环避免误锁人体反射

STVL 参数加快衰减：
  voxel_decay: 8.0                 ← 默认 15.0 减半，人走过 8 秒清掉
  decay_model: 0 (Linear)
  voxel_size: 0.05
```

### 3.3 ObstacleLayer 的 raycast 清除（自动）

`nav2_costmap_2d::ObstacleLayer` / `VoxelLayer` 在每次新 scan 到达时，沿激光束反向 raycast，自动清除该路径上的旧障碍。**这是 LaserScan 通道下"瞬时动态"的主要清除机制**——不需要额外配置，只要 `clearing: true`。

STVL 不依赖 raycast，靠 `voxel_decay` 时间衰减——更适合 RGB-D 这种"raycast 不准确"的传感器。

---

## 四、地图持久化与版本管理

### 4.1 文件构成

```
my_map/
├── my_map.posegraph    # boost 序列化的 Karto Mapper 全图
├── my_map.data         # 每个 vertex 的原始 LocalizedRangeScan
├── my_map.yaml         # OccupancyGrid 元数据
└── my_map.pgm          # OccupancyGrid 图像
```

| 文件 | 大小（中型办公室） |
|------|------------------|
| `.posegraph` | 10–80 MB |
| `.data` | 与扫描帧数线性，可达数百 MB（lifelong 长期运行后）|
| `.yaml` + `.pgm` | < 5 MB |

### 4.2 推荐版本化方案

不要把地图文件直接放进 git（二进制大）。推荐：

```
maps/
├── current/         ← 软链接，指向当前生效版本
├── 2026-04-15/      ← 时间戳目录
│   ├── my_map.posegraph
│   ├── my_map.data
│   ├── my_map.yaml
│   ├── my_map.pgm
│   └── meta.json    ← 自维护：建图时长、覆盖区域、operator、git sha 等
├── 2026-04-22/
└── 2026-04-29/
```

发布脚本：
```bash
# 建图完毕后
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /maps/2026-04-29/my_map}"
ros2 run nav2_map_server map_saver_cli -f /maps/2026-04-29/my_map
ln -sfn /maps/2026-04-29 /maps/current

# 部署节点统一指向 /maps/current/my_map
```

### 4.3 Lifelong 模式的"自动落盘"

`lifelong_slam_toolbox_node` 不会自动写盘——你必须在以下时机手动调用 `/slam_toolbox/serialize_map`：

| 时机 | 触发方式 |
|------|---------|
| 重启前 | systemd `ExecStop` 调用 ros2 service |
| 周期性快照 | cron / ROS Timer 节点每 1h 触发一次 |
| 关键事件 | "进入新房间"、"完成回环"等业务事件触发 |

**注意**：`serialize_map` 调用期间 SLAM Toolbox 主线程会短暂阻塞（中型图~几百 ms），别在导航关键时刻调。

### 4.4 OccupancyGrid 的"重导出"

部署时 Nav2 StaticLayer 可以从 `.posegraph` 自动渲染——但启动会慢。**最佳实践**：除了 `.posegraph` 还落一份 `.yaml + .pgm`，让 StaticLayer 直接读图像，启动快（< 1s）；slam_toolbox 只负责 `map → odom`。

---

## 五、灾备与回滚

### 5.1 触发回滚的事件

- Lifelong 错误回环导致地图扭曲，被运维或自检发现
- 地图文件损坏（断电写盘到一半）
- 部署版本切换后定位精度劣化

### 5.2 回滚机制

```bash
# 假设 2026-04-29 出问题，回滚到 2026-04-22
ln -sfn /maps/2026-04-22 /maps/current
systemctl restart slam_localization
```

systemd unit 设计要点：
- `WorkingDirectory=/maps/current`
- `Restart=on-failure`
- 启动时 sanity check：geomarkers 数 < 100 / posegraph 文件 < 1KB → 拒绝启动

---

## 六、监控指标（Lifelong 必做）

| 指标 | 来源 | 报警阈值 |
|------|------|---------|
| pose graph vertex 数 | `/slam_toolbox/graph_visualization` MarkerArray 计数 | 持续增长不收敛 → 裁剪失效 |
| `.data` 文件大小 | 文件监控 | 增长率 > 100 MB / 天 |
| `map → odom` TF 跳变幅度 | tf_echo + 自定义节点 | 单次跳变 > 0.5m → 错误回环 |
| 优化耗时 | `/slam_toolbox/optimization_time`（自打点）| > 1s 持续 → 图过大 |
| 系统 CPU | `top` / `node_exporter` | > 70% 持续 → 降级到 Localization |

可观测性详见 `nav2_research/`（如未来出 `10_observability_and_recovery.md`）；本专题不重复实现。

---

## 七、运行模式切换决策树

```
新部署 / 季度大改造 →
  ┌── async_slam_toolbox_node（Mapping）
  │     ↓ 30 min – 2 h，覆盖整个区域
  │   /serialize_map  →  my_map.posegraph
  │     ↓
  └── 切换到 →

日常运行（环境稳定）→
  ┌── localization_slam_toolbox_node
  │     ↓ 每周抽查地图老化情况
  └── 老化明显（家具迁移、新增隔断）→

中等老化（不重测）→
  ┌── lifelong_slam_toolbox_node
  │     ↓ 持续监控 §六 指标
  │     ↓ 周期性 /serialize_map
  └── CPU/内存吃紧 → 回退 localization

定位严重劣化 / 误差 > 30cm →
  停机重测 Mapping
```

---

## 八、常见误区

| 误区 | 真相 |
|------|------|
| "Lifelong 模式是 fire-and-forget 的" | 它需要监控 + 周期性落盘 + 偶尔人工介入修图 |
| "动态物体会被 SLAM Toolbox 自动忽略" | 不会——只有当一帧的多次匹配响应低时才会被回环拒绝；移动慢的物体（推车）会被建图 |
| "STVL 的时间衰减能解决一切动态" | 仅解决 costmap 层，不影响已建好的 OccupancyGrid 静态地图 |
| "把 .posegraph 文件复制到另一台机器即可用" | 需要 base_link → laser_frame 静态 TF 一致；URDF 不同则定位会偏 |
| "Lifelong 跑久了会越来越准" | 在足够约束下会，但错误回环也会累积；监控不可省 |

---

## 九、launch 切换骨架

`bringup/` 同时维护两份 launch，部署时选其一：

```python
# launch/slam_lifelong.launch.py
Node(package='slam_toolbox', executable='lifelong_slam_toolbox_node',
     parameters=['config/slam_toolbox_lifelong.yaml',
                 {'map_file_name': '/maps/current/my_map'}])

# launch/slam_localization.launch.py
Node(package='slam_toolbox', executable='localization_slam_toolbox_node',
     parameters=['config/slam_toolbox_localization.yaml',
                 {'map_file_name': '/maps/current/my_map'}])
```

> 注意 `map_file_name` **不带扩展名**，slam_toolbox 内部会自己加 `.posegraph`。

---

## 十、与已有调研的关系

- [01](./01_slam_toolbox_deep_dive.md) §四.3 — Lifelong 参数定义；本节是其运维语义化
- [04](./04_3d_perception_for_2d_nav.md) §五 — STVL 时间衰减；本节明确"两层时间尺度"分工
- `realtime_research/` — Lifelong 优化线程的 CPU 隔离同样适用
- `chassis_protocol/` — 与建图模式无关，对接接口不变

---

下一步：[06_evaluation_and_benchmarking.md](./06_evaluation_and_benchmarking.md) — 怎么客观判断"建图/定位变好了"，而不是凭感觉。
