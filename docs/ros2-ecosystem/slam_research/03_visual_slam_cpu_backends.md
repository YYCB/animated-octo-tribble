# 视觉 SLAM / VIO（CPU 后端） — 2D LiDAR 退化场景兜底

> **使用前提**：你已经按 [02](./02_odom_fusion_with_robot_localization.md) 把 wheel + IMU 融合到 `ekf_node`，并且 [01](./01_slam_toolbox_deep_dive.md) 的 SLAM Toolbox 在常规环境下能跑稳。本文讨论"在长走廊 / 玻璃幕墙 / 大堂等 2D LiDAR 退化场景下"如何用**纯 CPU** 的视觉惯性里程计（VIO）做兜底。
>
> **硬约束（再次强调）**：S100 主控无 CUDA，`Isaac ROS cuVSLAM`、`ORB-SLAM3 + CUDA fork`、`SuperPoint+SuperGlue` 等不可用。本文只比较**完全在 CPU 上跑**的方案。

---

## 一、为什么需要 VIO 兜底

### 1.1 2D LiDAR 退化的典型场景

| 场景 | 表现 | 根因 |
|------|------|------|
| 长走廊 | scan_matcher 在前向无法约束，`map→odom` 沿走廊方向缓慢漂 | 激光横向特征丰富，纵向只有两面墙——欠约束 |
| 玻璃幕墙 / 落地玻璃 | 雷达穿透，地图出现"破洞" + 误匹配 | 激光物理失效 |
| 空旷大堂（>10m） | 超过 `max_laser_range` 的方向无回波 | 信息密度过低 |
| 镜面/抛光地面 | 反射造成幻像点 | 多径 |
| 满墙的对称结构（机柜/书架）| ScanMatcher 找到错误的回环 | 特征自相似 |

VIO 在这些场景下**仍然能工作**——只要相机有纹理（即使墙面单调，门、灯、装饰物都能提供足够 ORB 特征）+ IMU 提供短期约束。

### 1.2 集成方式（非替换）

**不要**用 VIO 替换 SLAM Toolbox。本节方案是：

```
                      正常场景：信任 SLAM Toolbox
   ┌──────────────────────────────────────────────────┐
   │  /scan ─→ slam_toolbox ─→ map→odom  TF           │
   └──────────────────────────────────────────────────┘

                      退化兜底：信任 VIO 的相对位姿
   ┌──────────────────────────────────────────────────┐
   │  /color + /imu/data ─→ VIO ─→ /vio/odom          │
   │         │                                         │
   │         └─→ ekf_node 第二个输入源（odom1）         │
   │                                                   │
   │  优势：当 wheel/scan 出问题时，VIO 提供独立的位姿  │
   │       约束，EKF 自动加权                          │
   └──────────────────────────────────────────────────┘
```

VIO 只产生 `nav_msgs/Odometry`（不发布 TF），由 `ekf_node` 与 wheel/IMU 一起融合。SLAM Toolbox 仍然负责 `map→odom`。

---

## 二、候选方案矩阵

| 方案 | 类型 | 传感器需求 | License | ROS 2 支持 | ARM CPU | 成熟度 |
|------|------|------------|---------|-----------|--------|--------|
| **VINS-Fusion** | optimization-based VIO | mono/stereo + IMU | GPLv3 | 社区移植（HKUST 上游 ROS 1，需用 ros1_bridge 或社区 ROS 2 fork）| 中（双线程优化）| ⭐⭐⭐⭐ |
| **OpenVINS** | filter-based VIO（MSCKF）| mono/stereo + IMU | GPLv3 | ROS 2 官方分支（mins 项目下） | 低（最轻）| ⭐⭐⭐⭐ |
| **ORB-SLAM3** | feature-based VSLAM | mono/stereo/RGB-D + IMU | GPLv3 | 社区移植（多个 fork）| 中–高（ORB 提取重）| ⭐⭐⭐⭐⭐ |
| **RTAB-Map** | RGB-D / Stereo SLAM + 回环 | RGB-D / stereo + (IMU 可选)| BSD | ✅ 官方 ROS 2 包（`rtabmap_ros`）| 中–高 | ⭐⭐⭐⭐⭐ |
| ~~Isaac ROS vSLAM (cuVSLAM)~~ | filter-based VIO | RGB-D + IMU | NVIDIA 专有 | ✅ | **不可用（CUDA）** | — |
| ~~MASLAM / SuperPoint+SuperGlue~~ | learning-based | mono | 学术 | 实验 | **不可用（GPU 推理）** | — |

> **结论**：本场景三选一即可——
> - **OpenVINS** ：CPU 占用最低，作为"轻量级里程计源"接入 EKF
> - **ORB-SLAM3 (mono-inertial)** ：精度更高，但 ORB 提取在 ARM 上吃 CPU
> - **RTAB-Map** ：RGB-D 场景下"全栈"方案，自带回环、地图，**但与 SLAM Toolbox 角色重叠**——只在不用 SLAM Toolbox 时考虑

> **License 提醒**：上述三个 VIO 方案均为 **GPLv3**。商业产品集成时请评估 dynamic linking 与 service 部署场景的合规义务，本文档只做技术调研。

---

## 三、OpenVINS（首推，最轻）

> 上游：[`rpng/open_vins`](https://github.com/rpng/open_vins)，ROS 2 端在 `mins` 项目（多传感器 MSCKF）

### 3.1 算法核心（MSCKF / Multi-State Constraint Kalman Filter）

- 状态：当前 IMU 状态 + 滑动窗口内 N 个历史相机位姿（约 11 帧）
- 特征：FAST 角点 + KLT 跟踪
- 测量：把同一特征在多帧中的观测当作"约束"，对状态做 EKF 更新
- **不**做大规模 BA，**不**保留全局地图——只输出**漂移有界的局部里程计**，正好契合"喂 EKF 第二输入源"的角色

### 3.2 ARM CPU 占用经验

| 子模块 | 占用 |
|--------|------|
| FAST 角点提取（30Hz @ 640×480） | 单核 15%–25% |
| KLT 跟踪 | 单核 10%–20% |
| MSCKF EKF 更新 | < 5% |
| 整体（双核分配） | ~30%–50% 单核当量 |

> 320×240 灰度输入 + 30Hz 时占用可压到 ~15%，足以在 S100 上常开。

### 3.3 关键参数（节选 `mins/config/.../estimator_config.yaml`）

```yaml
estimator:
  use_stereo: false
  max_clones: 11               # 滑窗大小
  max_slam: 50                 # SLAM 特征上限（短期保留）
  max_msckf_in_update: 40
  num_pts: 200                 # 每帧追踪特征数（CPU 紧张可降到 100–150）
  fast_threshold: 15           # FAST 提取阈值
  grid_x: 5                    # 特征均匀化网格
  grid_y: 4
  min_px_dist: 10
  histogram_method: "HISTOGRAM"

  # IMU 噪声（必须按你的 IMU 实测填，否则 EKF 必发散）
  imu_acc_noise: 0.01
  imu_acc_walk: 0.0002
  imu_gyro_noise: 0.001
  imu_gyro_walk: 1.0e-5

cam0:
  T_imu_cam: [...]             # IMU→相机外参（4×4），必须用 Kalibr 标定
  intrinsics: [fx, fy, cx, cy] # 内参
  distortion_coeffs: [k1, k2, p1, p2]
  distortion_model: radtan
  resolution: [640, 480]
```

### 3.4 输出与对接

- `/ov_msckf/odomimu` (`nav_msgs/Odometry`) — 把它接到 EKF：

```yaml
# 追加到 02_odom_fusion 的 ekf.yaml
odom1: /ov_msckf/odomimu
odom1_config: [true,  true,  false,    # x, y, z       ← 信 VIO 的 x,y
               false, false, true,     # roll,pitch,yaw← 信 VIO 的 yaw
               false, false, false,    # vx, vy, vz
               false, false, false,    # vroll, vpitch, vyaw（已交给 IMU 直接通道）
               false, false, false]    # ax, ay, az
odom1_differential: true                # ★ 用相对增量，避免与 wheel 绝对位姿冲突
odom1_relative: false
odom1_pose_rejection_threshold: 2.0
```

> `differential: true` 是关键——它把 VIO 的绝对位姿转成"自上一帧以来的相对位移"喂 EKF，避免和 wheel odom 的"另一份"绝对位姿打架。

---

## 四、ORB-SLAM3（精度优先，CPU 占用更高）

> 上游：[`UZ-SLAMLab/ORB_SLAM3`](https://github.com/UZ-SLAMLab/ORB_SLAM3)；ROS 2 端可参考 `zang09/ORB_SLAM3_ROS2` 等社区 fork

### 4.1 与 OpenVINS 的本质差异

| 维度 | OpenVINS | ORB-SLAM3 |
|------|----------|-----------|
| 输出 | 局部里程计（漂移） | 全局一致位姿 + 关键帧地图 + 回环 |
| 后端 | MSCKF（filter）| Local BA + Loop BA + Atlas（map merging）|
| 特征 | FAST + KLT 跟踪 | ORB（含 BRIEF 描述子，回环用）|
| CPU | 低 | 中–高（ORB 提取） |
| 漂移 | 有界但持续累积 | 回环触发后大幅修正 |
| 输出延迟 | 帧到帧 ms 级 | Loop BA 触发时短暂阻塞 |

### 4.2 在本架构中的角色（重要边界）

**不要**用 ORB-SLAM3 替代 SLAM Toolbox 的 `map→odom`。原因：

1. ORB-SLAM3 的"全局地图"是稀疏关键帧 + 特征点，不是 OccupancyGrid，**Nav2 costmap 用不了**
2. 它的回环修正会让位姿大幅跳变，反过来污染 wheel/IMU 融合
3. 我们已经有 SLAM Toolbox 提供 OccupancyGrid 和回环——重复劳动

**正确用法**：只取 ORB-SLAM3 的"前端 VIO 估计"作为 EKF 的第二输入。可以通过启动 `Mono-Inertial` 模式 + 关闭 LoopClosingThread 来节省 CPU。

```cpp
// orb_slam3_ros2/src/mono_inertial_node.cpp 的关键改造：
// 在 System::TrackMonocular 之后只取 mTcw（当前位姿），按 nav_msgs/Odometry 发布
// 不订阅 /loop_closure 等下游 topic，让回环线程做但不影响外部
```

### 4.3 ARM 性能调优要点

```yaml
# ORB-SLAM3 配置（YAML，节选）
ORBextractor.nFeatures: 1000      # 默认 1200，ARM 上降到 800–1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8           # 金字塔层数；6 也够用
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# 输入分辨率：建议 640×480 或更低；1080p 在 ARM 上必卡顿
Camera.width: 640
Camera.height: 480
Camera.fps: 30                    # 可降到 15
```

---

## 五、RTAB-Map（仅在不用 SLAM Toolbox 时考虑）

> 上游：[`introlab/rtabmap`](https://github.com/introlab/rtabmap) + ROS 2 包 `rtabmap_ros`（apt 可装）

### 5.1 定位

RTAB-Map 是**全栈** SLAM——它本身做：
- RGB-D 视觉里程计（也可以接外部 odom）
- 特征匹配 + Bag-of-Words 回环
- 持久化 + 多 session 拼接
- **可以输出 OccupancyGrid**（从 depth 投影），所以可以独立替代 SLAM Toolbox

### 5.2 何时选 RTAB-Map 而非 SLAM Toolbox

| 维度 | 选 SLAM Toolbox | 选 RTAB-Map |
|------|----------------|-------------|
| 主传感器 | 2D LiDAR | RGB-D / stereo |
| 室内场景 LiDAR 退化频繁 | 需要 VIO 兜底 | RTAB-Map 直接搞定 |
| ARM CPU 紧张 | ✅（更轻）| ⚠️（视觉 + BoW 回环重）|
| 已有大量 LiDAR 部署 | ✅（一致性）| ❌ |
| 空间 ≥ 楼层级、跨 session | 可（lifelong）| ✅（更专业）|

> **本专题主线方案是"SLAM Toolbox + 可选 OpenVINS 兜底"**。RTAB-Map 在此处只作"备选 B 计划"列出，不展开参数细节；如确定切换到 RTAB-Map 路线再单开调研。

---

## 六、外参/时间同步（CPU 方案精度的天花板）

无论选 OpenVINS 还是 ORB-SLAM3，**精度上限取决于外参标定 + 时间同步**，与算法本身关系不大。

### 6.1 IMU↔Camera 外参（必做）

工具：[`ethz-asl/kalibr`](https://github.com/ethz-asl/kalibr)（标准）

流程：
1. 录制一段 5 分钟"激励充分"的 rosbag（充分激发三轴旋转 + 平移）
2. 棋盘格 / AprilTag 标定板
3. `kalibr_calibrate_imu_camera` 输出 `T_imu_cam` 4×4 外参
4. 把外参填到 OpenVINS / ORB-SLAM3 配置

**误差规模**：1° 角度外参误差 → VIO 漂移每米 +1.7cm。

### 6.2 时间同步

| 同步方式 | 实施难度 | 残留误差 |
|---------|---------|---------|
| 硬件触发（IMU 中断打时戳给相机） | 高 | < 0.1ms |
| PTP（IEEE 1588）| 中 | < 1ms |
| Software-only：相机 SDK 时间戳 | 低 | 5–30ms（IMU 通常稳，相机抖）|
| 跨主机 NTP | 低 | 10–100ms（不可用！）|

> 软同步下，VIO 标定中可以让 Kalibr 估算一个 `time_offset`（IMU 相对相机的偏移），作为静态补偿。S100 + USB 相机的"软同步 + 估算 time_offset"通常够 OpenVINS 用。

---

## 七、决策树（如何选）

```
┌─ 当前 SLAM Toolbox 在你的场景里稳定吗？─────────┐
│                                                │
├─ 是 ──→ 不需要 VIO，省 CPU 留给 Nav2 / 感知       │
│                                                │
└─ 否 ──→ 退化是哪类？                              │
        ├─ 长走廊 / 玻璃 / 大堂                     │
        │     ↓                                   │
        │  CPU 是否紧张？                           │
        │     ├─ 紧张 ──→ OpenVINS（最轻）          │
        │     └─ 宽裕 ──→ ORB-SLAM3 mono-inertial   │
        │                                         │
        ├─ 满墙对称 / scan_matcher 误回环 ──→        │
        │     先调 SLAM Toolbox 阈值（[01] §六）     │
        │     仍不行 → ORB-SLAM3（描述子回环更可靠）│
        │                                         │
        └─ 频繁打滑 / 抖动 ──→                      │
              先在 chassis_protocol 做协方差自适应  │
              （[02] §四.1）                        │
              仍不行 → OpenVINS                    │
```

---

## 八、launch 集成示例（OpenVINS 兜底）

```python
# 在 02 的 localization_stack.launch.py 基础上追加：
Node(package='ov_msckf', executable='run_subscribe_msckf',
     parameters=['config/openvins_estimator.yaml'],
     remappings=[('/cam0/image_raw', '/color/image_raw'),
                 ('/imu0', '/imu/data_raw')]),  # OpenVINS 用原始 IMU
# ekf.yaml 里加上 odom1: /ov_msckf/odomimu（见 §3.4）
```

> ⚠️ OpenVINS 需要的是**原始** IMU（未经 Madgwick 融合），与 EKF 用的 `/imu/data` 不同。两条 IMU 链可以并存（同一硬件话题分发到 `/imu/data_raw` 与 `/imu/data`）。

---

## 九、典型故障

| 症状 | 原因 / 排查 |
|------|------------|
| OpenVINS 启动几秒后位姿狂飘 | IMU 噪声参数错误（用了别人的 yaml）→ 需用 Allan Variance 实测 |
| ORB 特征数 < 50 | 场景纹理太差 / FAST 阈值太高 → 调低 `iniThFAST` 到 12 |
| EKF 接入 VIO 后反而更不稳 | 1) `differential: false` 误用绝对位姿；2) VIO 协方差填零（默认）→ 强制覆盖 `pose.covariance` |
| ORB-SLAM3 启动几秒就 "Loss" | 单目初始化失败 → 需要前 50 帧充分平移；起步时直接静止会 init 失败 |
| OpenVINS CPU 占用突然飙到 80% | `num_pts` 配置太高；或图像分辨率没降 |
| 场景切换后 VIO 不收敛 | mono-inertial 在尺度初始化失败时无救——加一颗第二目升级到 stereo-inertial |

---

## 十、与已有专题的衔接

- [02_odom_fusion_with_robot_localization.md](./02_odom_fusion_with_robot_localization.md) §三：本节的 `odom1` 配置直接追加到那里
- `realsense-ros-4.57.7-analysis` / `orbbec_sdk_ros2_analysis`：相机驱动话题命名遵循它们既有 namespace
- `realtime_research/`：VIO 主线程不要绑到 chassis_protocol 的实时核
- `perception_research/`：本节的 ORB-SLAM3 不与该专题的 FoundationPose / RT-DETR 等"感知"任务竞争 BPU——它在 CPU 上跑

---

下一步：[04_3d_perception_for_2d_nav.md](./04_3d_perception_for_2d_nav.md) — 把 RGB-D 深度数据喂给 Nav2 costmap，看见桌椅腿和悬空障碍。
