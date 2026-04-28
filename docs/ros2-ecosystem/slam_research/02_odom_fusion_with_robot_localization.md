# 里程计融合 — robot_localization (EKF/UKF) + IMU + Wheel

> 上游：[`cra-ros-pkg/robot_localization`](https://github.com/cra-ros-pkg/robot_localization)（Tom Moore 维护）
> 关联：本专题 [00_index.md](./00_index.md) §二、[01_slam_toolbox_deep_dive.md](./01_slam_toolbox_deep_dive.md) §五、`chassis_protocol/` 提供的 `/odom`

> **核心论点**：SLAM Toolbox 不直接消费 IMU。让 SLAM Toolbox"稳"的唯一通用做法是在它**前面**加 `robot_localization::ekf_node`，把 wheel odom 与 IMU 融合后再喂进去。本专题专门解决这一环。

---

## 一、为什么必须做 odom 融合

### 1.1 SLAM Toolbox 不读 IMU

`slam_toolbox` 的输入只有 `/scan` 和 TF `odom → base_link`。它内部假设 odom 已经是"尽可能好的位姿增量"。如果直接用 chassis_protocol 的纯 wheel odom：

| 场景 | 纯 wheel odom 的问题 | 后果 |
|------|---------------------|------|
| 原地旋转（差速底盘）| 转角靠左右轮差，标定不完美时累积旋转误差极大 | 地图开始扭曲，回环兜不回来 |
| 打滑（地毯/油污）| 轮子转了但没动，里程虚增 | 局部地图穿墙 |
| 颠簸（门槛）| 短期内轮速噪声暴涨 | scan_matcher 输入劣化，匹配失败 |
| 悬空（被搬起）| 编码器在转，机器人不动 | 完全错误的姿态注入 |

IMU 提供高频角速度（陀螺）+ 加速度，在短期内对**旋转**和**抖动**有强约束。融合后效果：

- 短时旋转精度：几乎完全由 IMU 主导（陀螺零偏小时漂移 ~1°/min）
- 长时平移精度：仍由 wheel encoder 提供尺度
- 突发打滑：协方差自适应可以让 EKF 放弃 wheel 的不可信测量

### 1.2 融合方案的边界

- ✅ 本文方案：**`robot_localization::ekf_node`（双源 EKF）** —— 简单、稳定、上游标杆
- ❌ 不在范围内：自研 IMU pre-integration（属于 VIO 后端，见 [03](./03_visual_slam_cpu_backends.md)）
- ❌ 不在范围内：在 `slam_toolbox` 内部直接打 IMU 补丁（社区有 PR 但未合入主线）

---

## 二、`robot_localization` 包结构

```
robot_localization/
├── src/
│   ├── ros_filter.cpp          ← 模板基类，定义 Update/Predict 调度
│   ├── ekf.cpp                 ← Extended Kalman Filter 实现
│   ├── ukf.cpp                 ← Unscented Kalman Filter 实现
│   ├── ekf_node.cpp            ← 节点 entry：rclcpp::spin(EkfNode)
│   ├── ukf_node.cpp            ← 节点 entry
│   ├── navsat_transform.cpp    ← GPS→UTM→map 转换（室外，本专题不用）
│   └── ros_filter_utilities.cpp
├── include/robot_localization/
│   ├── filter_base.hpp         ← 状态/协方差/度量更新抽象
│   ├── filter_common.hpp       ← 状态向量索引（StateMemberX = 0, ...StateMemberAz = 14）
│   └── ...
└── params/
    ├── ekf.yaml                ← 完整参数样例（推荐起步）
    └── ukf.yaml
```

### 2.1 状态向量（15 维 + Quaternion 内部表示）

```
state[0..2]   = [x, y, z]              （世界系位置）
state[3..5]   = [roll, pitch, yaw]     （世界系姿态，欧拉角；内部用四元数）
state[6..8]   = [vx, vy, vz]           （base_link 系线速度）
state[9..11]  = [vroll, vpitch, vyaw]  （base_link 系角速度）
state[12..14] = [ax, ay, az]           （base_link 系线加速度）
```

> 即使是 2D 平面机器人，状态向量仍然是 15 维。我们通过 `_config` 矩阵**屏蔽**掉 z / roll / pitch / vz / vroll / vpitch / az 等通道。

### 2.2 EKF vs UKF 选型

| 维度 | EKF | UKF | 推荐 |
|------|-----|-----|------|
| 计算量 | 低 | 中（~3× EKF）| **EKF** 用于 S100 |
| 强非线性时精度 | 中（一阶泰勒）| 高（sigma points）| 本场景 2D 几乎线性，差异不显著 |
| 上手难度 | 低 | 中 | **EKF** |
| 室内 2D + IMU | 完全够用 | 杀鸡用牛刀 | **EKF** |

> **结论**：本专题全程用 `ekf_node`。下文除非特别说明都指 EKF。

---

## 三、`_config` 矩阵：决定融合什么

每个传感器输入都需要一个 15 元素的 `bool` 向量，告诉 EKF "**这条消息的哪些字段我可以信任并用于更新**"。

### 3.1 字段顺序（务必记牢）

```
[ x,    y,    z,
  roll, pitch, yaw,
  vx,   vy,   vz,
  vroll,vpitch,vyaw,
  ax,   ay,   az ]
```

### 3.2 双源典型配置

> **设计原则**：
> - **wheel odom** 提供 `[vx, vyaw]`（差速底盘没有 vy；如果是 mecanum/omni 则 `[vx, vy, vyaw]`）
> - **IMU** 提供 `[vyaw]`（陀螺）+ `[ax, ay]`（加速度）
> - **不要让两者都说同一件事**：例如不要让 wheel odom 提供 `vyaw` 同时让 IMU 也提供 `vyaw`，否则方差小者主导，可能导致难以排查的偏置注入。**让 IMU 主导旋转**，让 wheel 主导平移。

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0                   # Hz，EKF predict 频率
    sensor_timeout: 0.1               # s，输入超时容忍
    two_d_mode: true                  # ★ 平面机器人必开
    publish_tf: true                  # ★ 发布 odom→base_link TF
    publish_acceleration: false
    transform_time_offset: 0.0
    transform_timeout: 0.0
    print_diagnostics: true
    debug: false

    # ---------- 帧 ----------
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link        # 与 chassis_protocol 一致
    world_frame: odom                 # ★ EKF 工作在 odom 坐标系（不参与 map→odom，留给 SLAM）

    # ---------- 输入 0：wheel odom（来自 chassis_protocol） ----------
    odom0: /odom
    odom0_config: [false, false, false,    # x, y, z
                   false, false, false,    # roll, pitch, yaw
                   true,  false, false,    # vx, vy, vz       ← 差速：只信 vx
                   false, false, false,    # vroll, vpitch, vyaw  ← 旋转交给 IMU
                   false, false, false]    # ax, ay, az
    odom0_queue_size: 10
    odom0_differential: false           # 用绝对量；false=直接用 vx，true=对位姿差分
    odom0_relative: false
    odom0_pose_rejection_threshold: 5.0
    odom0_twist_rejection_threshold: 1.0
    odom0_nodelay: true

    # ---------- 输入 1：IMU ----------
    imu0: /imu/data                     # 必须经过 imu_filter_madgwick 等做姿态融合
    imu0_config: [false, false, false,   # x, y, z
                  false, false, false,   # roll, pitch, yaw  ← 不直接用 IMU 的绝对 yaw（漂移）
                  false, false, false,   # vx, vy, vz
                  false, false, true,    # vroll, vpitch, vyaw ← ★ 信 vyaw（陀螺）
                  true,  true,  false]   # ax, ay, az
    imu0_queue_size: 10
    imu0_differential: false
    imu0_relative: true                 # ★ 起始时 IMU yaw 不必与世界对齐
    imu0_remove_gravitational_acceleration: true
    imu0_pose_rejection_threshold: 0.8
    imu0_twist_rejection_threshold: 0.8
    imu0_linear_acceleration_rejection_threshold: 0.8
    imu0_nodelay: true

    # ---------- 过程噪声协方差（15×15, 行优先）----------
    # 默认对角线即可；以下是典型 2D 室内：
    process_noise_covariance: [
      0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0.025,0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0.025,0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.015]

    initial_estimate_covariance: [
      1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-9]
```

### 3.3 输出

| Topic / TF | 含义 | QoS |
|-----------|------|-----|
| `/odometry/filtered` (`nav_msgs/Odometry`) | 融合后的 odom，**SLAM Toolbox 不直接订阅**，但 Nav2 部分组件会用 | RELIABLE |
| TF: `odom → base_link` | EKF 发布；**取代 chassis_protocol 的同名 TF** | — |

> 关键操作：把 chassis_protocol `ChassisNode` 配置成 `publish_tf: false`，让 EKF 独占 odom→base_link 的发布权。SLAM Toolbox 的 `odom_frame: odom` 与 EKF 的 `odom_frame: odom` 必须完全一致。

---

## 四、协方差矩阵填法（最易错的一环）

### 4.1 输入消息自身协方差

`/odom` 与 `/imu/data` 自带 6×6 / 3×3 协方差矩阵。EKF 在 update 步用的是这些**输入侧**协方差，不是 `process_noise_covariance`。

`chassis_protocol` 默认填的协方差很可能是占位（如全 0 或 0.001 对角）。**这是最常见的失败根因**——协方差为 0 等于"完全可信"，EKF 会把所有权重给它，IMU 形同虚设。

#### 经验值（差速底盘 + RPLIDAR + 6 轴 MEMS IMU）

```
nav_msgs/Odometry.twist.covariance（6×6, 行优先 [vx, vy, vz, vroll, vpitch, vyaw]）：
  对角：[0.01,    1e6,     1e6,     1e6,     1e6,     0.05]
  非对角：0
  备注：vy/vz/vroll/vpitch 给极大值 = 该通道完全不可信，不会被 EKF 误用

sensor_msgs/Imu.angular_velocity_covariance（3×3, [vroll, vpitch, vyaw]）：
  对角：[0.001, 0.001, 0.005]   ← 6 轴消费级 MEMS 经验值

sensor_msgs/Imu.linear_acceleration_covariance（3×3, [ax, ay, az]）：
  对角：[0.04, 0.04, 0.04]      ← 静止 Allan 方差实测后再调
```

> 把 1e6 用于"屏蔽不可信通道"是社区惯例，比把 `*_config` 设成 false 更鲁棒（即使将来误把 config 改回 true，1e6 仍然让该通道在权重上被压制）。

### 4.2 `process_noise_covariance` 经验值

代表"模型不准导致的状态不确定性增长率"。常见调法：

| 状态 | 默认 | 调小（更信预测）| 调大（更信测量）|
|------|------|----------------|----------------|
| `vx` | 0.025 | 0.005 | 0.1 |
| `vyaw` | 0.02 | 0.005 | 0.1 |
| `ax/ay` | 0.01 | 0.001 | 0.05 |

**实战流程**：
1. 静止时 `/odometry/filtered` 应该不漂；如果漂 → process noise 太大
2. 推车快速直行 + 急停时 EKF 收敛速度太慢 → process noise 太小
3. 急转弯 yaw 不跟随 → IMU vyaw 协方差太大

---

## 五、IMU 预处理（关键前置步骤）

`robot_localization` **不做** IMU 滤波/姿态解算。`/imu/data` 应该已经是"姿态融合后的 IMU"——即包含**有效的 orientation 字段**（即使我们的 `imu0_config` 不让它进入更新）。

### 5.1 推荐链路

```
原始 IMU 驱动（/imu/data_raw, 含 angular_velocity + linear_acceleration） ──┐
                                                                          │
                                                          imu_filter_madgwick
                                                                          │
                                                          /imu/data（含 orientation 四元数）
                                                                          │
                                                                  ekf_filter_node
```

### 5.2 `imu_filter_madgwick` 配置示例

```yaml
imu_filter:
  ros__parameters:
    use_mag: false                # 室内磁场不可靠，关闭
    publish_tf: false             # IMU 不发 TF
    world_frame: enu              # ROS 标准
    gain: 0.1                     # Madgwick gain，0.1 起步
    zeta: 0.0
    fixed_frame: base_link        # 与 EKF base_link 一致
    orientation_stddev: 0.05
```

### 5.3 IMU 静态零偏标定

EKF 不做零偏在线估计（`ros2_imu_tools` 等其他包做）。最简便：开机静止 5 秒，工具自动测算并加进消息中。MEMS IMU 不标定时陀螺零偏可达 1°/s，融合后会让 yaw 持续旋转——SLAM 地图必然漂。

---

## 六、TF 拓扑（最终版）

```
                                  ┌─────────────────────────┐
                                  │   slam_toolbox          │
                                  │   发布: map → odom      │
                                  └────────────┬────────────┘
                                               │
                                               ▼
                                   map ─────────── odom
                                                    │
                                  ┌─────────────────┴─────┐
                                  │   ekf_filter_node     │
                                  │   发布: odom→base_link│  ←──── /odom (chassis_protocol)
                                  └─────────────────┬─────┘  ←──── /imu/data (Madgwick 之后)
                                                    │
                                                    ▼
                                           base_link
                                            │   │   │
                                ┌───────────┘   │   └────────────┐
                                ▼               ▼                ▼
                           laser_frame      imu_link        camera_link
                          （URDF 静态TF）   （URDF 静态）   （URDF 静态）
```

**禁止重复发布同一对 frames 之间的 TF**：
- chassis_protocol：`publish_tf: false`（让出 odom→base_link）
- IMU 驱动：`publish_tf: false`（IMU 数据走话题，不发 TF）
- ekf_node：`publish_tf: true` 唯一发 odom→base_link
- slam_toolbox：唯一发 map→odom

---

## 七、典型故障与排查

| 症状 | 检查清单 |
|------|---------|
| `/odometry/filtered` 完全没有输出 | 1) `frequency` 是否合理；2) 输入话题是否到达（`ros2 topic hz`）；3) `sensor_timeout` 是否过短 |
| 输出位姿在静止时缓慢漂移 | 1) IMU 零偏标定；2) IMU 协方差是否过小；3) `process_noise_covariance` 是否过大 |
| 输出位姿对急转响应迟钝 | 1) IMU `imu0_config[11]=true` 是否打开；2) `imu0_twist_rejection_threshold` 不要太低 |
| 推车打滑后 EKF 严重发散 | 1) wheel odom 协方差未在打滑时上升 → 在 chassis_protocol 中实现"打滑检测"，把协方差临时拉到 1e3 |
| TF 报 "Could not find transform odom→base_link" | 多源同时发 odom→base_link → 关闭 chassis_protocol 的 TF 发布 |
| EKF 频繁打印 "Failed to meet sufficient overlap..." | 输入时间戳带未来时间或乱序 → 检查 IMU 时间戳是否用 `now()` 而非硬件采集时间 |
| `/imu/data` orientation 全 0 / 全 NaN | `imu_filter_madgwick` 没启动 / `use_mag: true` 但无磁力计 |

---

## 八、与 chassis_protocol 的对接清单

| 项 | chassis_protocol 该做什么 | EKF 该做什么 |
|---|--------------------------|-------------|
| `/odom` 话题 | 发布 `nav_msgs/Odometry`，`twist.covariance` 填**真实**协方差（非零、非 1e-9）| 订阅，`odom0_config = [...vx=true...]` |
| `odom→base_link` TF | **不发**（`publish_tf: false`）| 唯一发布者 |
| `/odom` 时间戳 | 必须用 `clock->now()`，不允许用编码器采样时刻（除非 PTP 同步）| sensor_timeout 默认 0.1s 即可 |
| 打滑检测 | 在 chassis_protocol 检测到电流/编码器异常时，把 `twist.covariance[0,0]` 临时上调 | 自适应权重生效 |
| `joint_states` | 仍然发布（Nav2/MoveIt 用）| 不订阅 |

> chassis_protocol 当前是否已暴露"协方差自适应"接口需要在源码层确认；如未实现，作为待落地项记入 [07](./07_chassis_and_s100_integration.md) §对接 TODO。

---

## 九、launch 骨架

```python
# bringup/launch/localization_stack.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) IMU 姿态融合
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             parameters=['config/imu_filter.yaml'],
             remappings=[('imu/data_raw', '/imu/data_raw'),
                         ('imu/data', '/imu/data')]),
        # 2) EKF
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node',
             parameters=['config/ekf.yaml']),
        # 3) SLAM Toolbox（建图或定位二选一）
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             parameters=['config/slam_toolbox_mapping.yaml']),
    ])
```

---

## 十、源码引用速查

| 主题 | 路径 |
|------|------|
| EKF predict/update 主循环 | `robot_localization/src/ros_filter.cpp::periodicUpdate` |
| 状态向量索引 | `robot_localization/include/robot_localization/filter_common.hpp::StateMember*` |
| 协方差解析 | `robot_localization/src/ros_filter.cpp::loadParams` |
| 异步话题缓冲 | `robot_localization/src/ros_filter.cpp::enqueueMeasurement` |
| TF 发布 | `robot_localization/src/ros_filter.cpp::publishOutput` |

---

下一步：[03_visual_slam_cpu_backends.md](./03_visual_slam_cpu_backends.md) — 当 2D LiDAR 退化时（长走廊 / 玻璃墙），如何加 VIO 兜底。
