# SLAM 栈调研 — 总索引

> **调研范围**：面向 **2D LiDAR + IMU + RGB-D 室内移动机器人** 的 SLAM / 定位 / 局部感知完整栈。
> **目标平台**：**地瓜机器人 RDK S100（D-Robotics）作为主控**——ARM CPU + BPU/NPU + 自研 SoC，**无 CUDA、无 NVIDIA GPU**。所有选型均必须是 **CUDA-free / CPU-only / ARM 友好**。
> **关联文档**：
> - `docs/ros2-ecosystem/nav2_research/06_localization.md`（AMCL vs SLAM Toolbox 入门级对比，已存在）
> - `docs/ros2-ecosystem/nav2_research/03_costmap2d_plugins.md`（costmap 层）
> - `docs/ros2-ecosystem/perception_research/`（3D 感知，但其中 **nvblox 部分在 S100 上不可用**）
> - `chassis_protocol/`（提供 `/odom` 和 `odom → base_link` TF）

---

## 一、为什么需要这个专题

`docs/ros2-ecosystem/nav2_research/06_localization.md` 已经覆盖了 **AMCL / SLAM Toolbox 的"入门级"差异 + collision_monitor**。但本专题要回答的是更工程化、更具体的一组问题：

1. **2D LiDAR + IMU + RGB-D** 这个传感器组合下，建图到底应该走什么路径？SLAM Toolbox 一把梭还是要拼接 VIO？
2. **wheel odom 抖 + IMU 漂** 时，里程计该怎么融合才能让 SLAM Toolbox 真正稳？`robot_localization` EKF 还是 UKF？协方差怎么填？
3. **走廊、玻璃幕墙、空旷大堂**——2D LiDAR 退化时怎么办？是否要引入 VIO / VSLAM？
4. **RGB-D 深度数据**——能让 2D 导航看见桌椅腿、悬空障碍吗？怎么 **不依赖 CUDA** 把它喂进 Nav2 costmap？
5. **S100 主控（无 CUDA、ARM CPU 较弱）** 跑得动哪些？哪些必须降级？
6. **lifelong / 动态环境**——商场/办公室长期运行，地图怎么持续更新？

这些问题在已有文档中只有零散提及，这里做系统化的源码级深化。

---

## 二、整体架构（S100 主控版本）

```
┌──────────────────────────────────────────────────────────────────────────┐
│  传感器层（物理设备 + 驱动）                                                │
│                                                                          │
│  2D LiDAR ──┐    IMU ──┐    RGB-D ──┐    Wheel encoder ──┐               │
│   /scan     │   /imu/data│   /color   │    （chassis_protocol）            │
│   (LaserScan)│  (Imu)    │   /depth   │    /joint_states                  │
│             │            │   /info    │                                   │
│             │            │   (Image,  │                                   │
│             │            │    CameraInfo)                                 │
└─────────────┼────────────┼────────────┼─────────────────────┬─────────────┘
              │            │            │                     │
              ▼            ▼            ▼                     ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  里程计融合层（CPU）                                                       │
│                                                                          │
│   chassis_protocol → /odom (wheel only, 50–100 Hz)                       │
│              │                                                            │
│              ▼                                                            │
│   robot_localization::ekf_node                                           │
│   ├── 输入：/odom (wheel) + /imu/data                                     │
│   ├── 输出：/odometry/filtered                                            │
│   └── TF:   odom → base_link（覆盖 chassis_protocol 的 TF）                │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │ /odometry/filtered + TF (odom→base_link)
                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  SLAM 主链路（CPU）                                                       │
│                                                                          │
│   ┌─ Mapping 阶段 ─────────────────────────────────────────────┐          │
│   │  slam_toolbox (online_async)                              │          │
│   │  ├── 输入：/scan + TF(odom→base_link)                      │          │
│   │  ├── 内部：scan_matcher → posegraph → loop_closure         │          │
│   │  ├── 输出：/map (OccupancyGrid) + TF(map→odom)             │          │
│   │  └── 落盘：.posegraph + .data + .yaml                       │          │
│   └────────────────────────────────────────────────────────────┘          │
│                                                                          │
│   ┌─ Deployment 阶段 ──────────────────────────────────────────┐          │
│   │  slam_toolbox (localization) ← 推荐                        │          │
│   │  或 nav2_amcl ← 资源极限时降级                              │          │
│   └────────────────────────────────────────────────────────────┘          │
│                                                                          │
│   ┌─ 2D LiDAR 退化兜底（可选）─────────────────────────────────┐          │
│   │  VINS-Fusion / OpenVINS / ORB-SLAM3 (mono-inertial)        │          │
│   │  ├── 输入：/color + /imu/data                              │          │
│   │  └── 输出：/vio/odom → 喂给 robot_localization EKF 第二个轮次│          │
│   └────────────────────────────────────────────────────────────┘          │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │ /map + TF(map→odom→base_link)
                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  局部 3D 感知（CPU，喂 Nav2 costmap）                                      │
│                                                                          │
│   depth_image_proc / pcl_ros：depth → /pointcloud                        │
│   pcl::VoxelGrid 降采样 + 地面分割（PassThrough / RANSAC）                │
│              │                                                            │
│              ▼                                                            │
│   spatio_temporal_voxel_layer (STVL) ← 推荐                              │
│   或 nav2_costmap_2d::VoxelLayer ← 极轻量降级                             │
│              │                                                            │
│              ▼                                                            │
│   Nav2 local_costmap                                                     │
└──────────────────────────────────────────────────────────────────────────┘
```

> **关键不变量**：所有方框内的算法都是 **CPU 实现**，无 CUDA 依赖。这是 S100 主控带来的硬约束；同时也意味着该栈在普通 x86 / ARM 工控机 / Orin AGX 上**无差别可移植**。

---

## 三、文档目录

| 文件 | 内容 |
|------|------|
| [01_slam_toolbox_deep_dive.md](./01_slam_toolbox_deep_dive.md) | SLAM Toolbox 内部数据结构（pose graph, Karto scan matcher, Ceres solver）、三种模式下的状态机、`.posegraph` 格式、所有关键参数及调参矩阵、ARM CPU 性能预算 |
| [02_odom_fusion_with_robot_localization.md](./02_odom_fusion_with_robot_localization.md) | `robot_localization` EKF/UKF 选型、`_config` 矩阵语义、wheel + IMU 双源配置、协方差填法、与 chassis_protocol `/odom` 对接、常见失败模式排查 |
| [03_visual_slam_cpu_backends.md](./03_visual_slam_cpu_backends.md) | OpenVINS / VINS-Fusion / ORB-SLAM3 三家 CPU VIO/VSLAM 横向对比、ARM 性能基线、作为 2D LiDAR 退化兜底接入 EKF 的方式、与 SLAM Toolbox 的边界 |
| [04_3d_perception_for_2d_nav.md](./04_3d_perception_for_2d_nav.md) | RGB-D 深度 → PointCloud2 → STVL/VoxelLayer 集成路径，**全 CPU**；点云降采样、地面分割、悬空障碍检测；为何不选 nvblox |
| [05_dynamic_and_lifelong.md](./05_dynamic_and_lifelong.md) | Lifelong mapping 模式、map decay、动态物体过滤（DynaSLAM 思路 vs 实际工程降级）、地图版本与持久化 |
| [06_evaluation_and_benchmarking.md](./06_evaluation_and_benchmarking.md) | 建图/定位的客观评估方法（APE / RPE）、`evo` 工具链、TUM RGB-D / KITTI 数据集、S100 性能基线方法学 |
| [07_chassis_and_s100_integration.md](./07_chassis_and_s100_integration.md) | 与 `chassis_protocol` 的 QoS/TF/odom 对接清单、S100 主控时的整机拓扑、TogetheROS / `hobot_sensors` 衔接、launch 文件骨架 |

---

## 四、S100 主控带来的硬约束（必读）

| 约束 | 含义 | 对本专题的影响 |
|------|------|---------------|
| **无 CUDA** | NVIDIA cuVSLAM / nvblox / Isaac ROS vSLAM 全部不可用 | SLAM 算法必须是纯 CPU；`perception_research/` 中 nvblox 章节在 S100 阶段视为"不可选" |
| **BPU/NPU ≠ GPGPU** | BPU 只跑量化好的 NN 推理，**不能跑通用并行计算（BA、scan matching）** | 即使 S100 算力总值不低，**SLAM 主链路不能从 BPU 取得加速**；调度上要保证 SLAM 的 CPU 核独占 |
| **ARM Cortex-A 核** | 单核性能弱于 Orin AGX | scan_matcher、Ceres BA、ORB-SLAM3 ORB 提取等热点函数需要预算 + 降级开关 |
| **TogetheROS** | 基于 ROS 2 Humble 的发行版，自带 `hobot_sensors`/`hobot_codec`/`hobot_dnn` | SLAM 节点本身完全兼容上游 ROS 2；驱动/编解码层走 hobot 节点（详见 07）|
| **rmw / DDS** | 默认 Cyclone 或 Fast-DDS，与上游一致 | 与 chassis_protocol 的 QoS 兼容性已在 nav2_research/07 中验证，本专题继承 |

---

## 五、三阶段演进路线（建议）

### Phase A：建图阶段（首次部署 / 环境变化时手动重跑）

```
chassis_protocol → /odom ──┐
IMU ──────────────────────┤── ekf_node ── /odometry/filtered ── TF(odom→base_link)
                          │                                              │
                          │                                              ▼
                          └─────────── slam_toolbox(online_async) ── /map + TF(map→odom)
                                              │
                                              ▼
                                       手动 /save_map 服务
                                       生成 my_map.{posegraph,data,yaml}
```

**输出物**：`my_map.posegraph` + `my_map.data` + `my_map.yaml`（OccupancyGrid 描述）

### Phase B：日常运行（部署阶段）

```
chassis_protocol → /odom ──┐
IMU ──────────────────────┤── ekf_node ── /odometry/filtered ── TF(odom→base_link)
                          │                                              │
                          │                                              ▼
                          └─────────── slam_toolbox(localization) ── TF(map→odom)
                                       加载 my_map.posegraph
                                              │
RGB-D depth ─→ pointcloud ─→ STVL ────────────┴──── Nav2 local_costmap
2D LiDAR ─────────────────────────────────────────── Nav2 + slam_toolbox 共用 /scan
```

### Phase C：长期运行（lifelong，可选）

```
slam_toolbox(lifelong) 替换 (localization)
  ├── 周期性回环 + 局部地图更新
  ├── 旧 submap 衰减（map decay）
  └── 异步导出 .posegraph 持久化
```

详见 [05_dynamic_and_lifelong.md](./05_dynamic_and_lifelong.md)。

---

## 六、与已有调研的关系

| 已有专题 | 本专题如何衔接 |
|---------|---------------|
| `nav2_research/06_localization.md` | 本专题 01 是其 SLAM Toolbox 部分的**深化扩展**：参数全表、posegraph 内部结构、ARM CPU 占用、调参决策树 |
| `nav2_research/03_costmap2d_plugins.md` | 本专题 04 提供 STVL 的**完整 RGB-D 接入流程**（含点云预处理），补足 costmap 层入口 |
| `perception_research/` | 本专题 04 明确：在 S100 上 **不使用 nvblox**，给出 CPU 替代方案的迁移指引 |
| `realsense-ros-4.57.7-analysis` / `orbbec_sdk_ros2_analysis` | 提供深度相机驱动；本专题 04/07 衔接其输出的 `/depth` 与 `/color` 话题 |
| `chassis_protocol/` | 本专题 02/07 复用其 `/odom` 与 `odom→base_link` TF；**SLAM 栈不重新实现里程计** |
| `realtime_research/` | 本专题在调参时引用其 CPU 隔离/SCHED_FIFO 建议（SLAM Toolbox 后端线程不抢占控制环）|
| `microros_research/` | 暂无直接关联（IMU 若走 micro-ROS，可参考其传输配置）|

---

## 七、本专题不覆盖的内容（明确边界）

- **3D LiDAR SLAM**（LIO-SAM / FAST-LIO / Cartographer 3D）：当前传感器是 2D LiDAR，不在范围内。后续如果硬件变更可单开专题
- **多机协同 SLAM**（如 CCM-SLAM、DOOR-SLAM）：单机器人场景
- **GPU 加速建图**（nvblox、cuVSLAM、SuperPoint+SuperGlue）：S100 无 CUDA，不可选
- **室外定位**（RTK-GNSS、IMU pre-integration with GPS）：室内场景
- **机械臂感知**（FoundationPose 等）：见 `perception_research/`

---

## 八、阅读顺序建议

- **要快速建一个能跑的栈** → 02 → 01 → 07 → 04（按部署顺序）
- **要深入理解算法** → 01 → 02 → 03 → 05
- **要做基线对比和验证** → 06 → 03（VIO 备选）
- **要规划 S100 硬件资源** → 01 §7（性能预算）→ 03 §5 → 04 §6 → 07 §5
