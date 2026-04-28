# 与 chassis_protocol / S100 主控的整机集成

> 把 [00](./00_index.md)–[06](./06_evaluation_and_benchmarking.md) 的所有结论落到 **S100 主控的实机**上。本文是部署侧的"操作手册 + 验证清单"。
>
> 关联：`chassis_protocol/` 源码、`docs/ros2-ecosystem/nav2_research/07_chassis_protocol_integration.md`（已存在的 Nav2 对接清单）

---

## 一、整机拓扑（S100 主控版）

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  地瓜机器人 RDK S100（主控）                                                   │
│  TogetheROS（基于 ROS 2 Humble）+ rmw_cyclonedds_cpp（推荐）                   │
│                                                                              │
│  ┌─────────────────────┐   ┌──────────────────────┐   ┌────────────────────┐ │
│  │ 传感器驱动层         │   │ 里程计 / SLAM 层      │   │ 导航 / 控制层      │ │
│  │                     │   │                      │   │                    │ │
│  │ hobot_sensors:      │   │ imu_filter_madgwick  │   │ Nav2 (full stack)  │ │
│  │  └ /scan            │──→│  /imu/data           │──→│  ├ planner_server  │ │
│  │ hobot_codec:        │   │ ekf_filter_node      │   │  ├ controller_server│ │
│  │  └ /color, /depth   │   │  /odometry/filtered  │   │  └ collision_monitor│ │
│  │ chassis_protocol:   │   │  TF: odom→base_link  │   │ velocity_smoother  │ │
│  │  └ /odom (no TF)    │   │ slam_toolbox         │   │                    │ │
│  │ imu_driver:         │   │  TF: map→odom        │   │                    │ │
│  │  └ /imu/data_raw    │   │  /map                │   │                    │ │
│  └─────────────────────┘   └──────────────────────┘   └────────────────────┘ │
│                                                                              │
│                            chassis_protocol（也运行在 S100 上）              │
│                                  ↕  TCP / UDP / RS485                        │
└──────────────────────────────────────────────────────────────────────────────┘
                                    ↕
                            ┌────────────────────┐
                            │  底盘 MCU（独立板） │
                            │  电机 / 编码器      │
                            └────────────────────┘
```

> **简化原则**：所有 ROS 2 节点运行在 S100 上；底盘单独 MCU 通过 chassis_protocol HAL 对接（已有架构）。SLAM 栈不引入额外计算节点。

---

## 二、节点 / 话题 / TF 总账

### 2.1 节点清单

| 节点 | 包 | 角色 | 是否 lifecycle |
|------|---|-----|--------------|
| `chassis_node` | `chassis_protocol` | 底盘 HAL 适配 | 否（普通 Node）|
| `imu_filter_madgwick_node` | `imu_filter_madgwick` | IMU 姿态融合 | 否 |
| `ekf_filter_node` | `robot_localization` | wheel + IMU 融合 | 否 |
| `async_slam_toolbox_node` / `localization_slam_toolbox_node` | `slam_toolbox` | SLAM | ✅ |
| `point_cloud_xyz_node` | `depth_image_proc` | depth → cloud | 否（component）|
| `voxel_grid_node` | `pcl_ros` | 降采样 | 否（component）|
| Nav2 八件套 | `nav2_*` | 导航 | ✅ |
| 雷达驱动 | `hobot_sensors` 或上游 sllidar/rplidar | LiDAR | 否 |
| 相机驱动 | `hobot_codec` 或 `realsense2_camera` 等 | RGB-D | 否 |

### 2.2 话题清单

| Topic | Type | 发 | 收 | QoS |
|-------|------|---|---|-----|
| `/cmd_vel` | `Twist` | Nav2 controller_server | chassis_node | RELIABLE / VOLATILE / depth=1 |
| `/odom` | `Odometry` | chassis_node | ekf_filter_node | RELIABLE / VOLATILE |
| `/imu/data_raw` | `Imu` | imu_driver | imu_filter_madgwick_node | SensorDataQoS |
| `/imu/data` | `Imu` | imu_filter_madgwick | ekf_filter_node + (OpenVINS) | RELIABLE 推荐 |
| `/odometry/filtered` | `Odometry` | ekf_filter_node | Nav2 部分组件（TF 主用）| RELIABLE |
| `/scan` | `LaserScan` | LiDAR 驱动 | slam_toolbox + Nav2 costmap | SensorDataQoS |
| `/map` | `OccupancyGrid` | slam_toolbox | Nav2 costmap_2d StaticLayer | RELIABLE / TRANSIENT_LOCAL |
| `/depth/image_rect_raw` | `Image` | 相机驱动 | depth_image_proc | SensorDataQoS |
| `/depth/camera_info` | `CameraInfo` | 相机驱动 | depth_image_proc | RELIABLE / TRANSIENT_LOCAL |
| `/depth/points_filtered` | `PointCloud2` | passthrough | STVL | SensorDataQoS |
| `/diagnostics` | `DiagnosticArray` | 各节点 | aggregator | RELIABLE |

### 2.3 TF 总账（**唯一发布者原则**）

| Edge | 发布者 | 频率 |
|------|-------|-----|
| `map → odom` | slam_toolbox（async / localization / lifelong）| `transform_publish_period`（默认 0.05s）|
| `odom → base_link` | **ekf_filter_node 唯一** | `frequency`（默认 50Hz）|
| `base_link → laser_frame` | `robot_state_publisher`（URDF 静态）| 一次性 |
| `base_link → imu_link` | `robot_state_publisher` | 一次性 |
| `base_link → camera_link` | `robot_state_publisher` | 一次性 |
| `camera_link → depth_optical_frame` | 相机驱动（动态发布相机内部 TF）| 启动一次 |

**绝对不要**让以下几个组件中的任何一个发布 `odom→base_link`：
- chassis_protocol：`publish_tf` 概念在当前源码中不存在（核对 `chassis_node.cpp:24` —— 只发 `/odom` topic，**没有** `tf2_ros::TransformBroadcaster`），✅ 默认行为已经正确
- `nav2_amcl`：如果切到 AMCL 会冲突——记得改 `tf_broadcast: false`（但本方案不用 AMCL）
- 任何旧版 `differential_drive_controller` mock

> **chassis_protocol 现状核查**：`chassis_protocol/ros2_adapter/chassis_node.cpp` 当前只创建 `odom_pub_` 等 publisher，未包含 TF broadcaster，**默认行为符合本方案**。仅需保证未来不要"补一个 TF 发布者"。如果需要未启用 EKF 时的 fallback，建议增加显式开关 `enable_odom_tf`（默认 `false`），交由 EKF 接管。

---

## 三、chassis_protocol 必须做对的几件事

### 3.1 `/odom` 协方差必须真实

定位 [02_odom_fusion](./02_odom_fusion_with_robot_localization.md) §四.1 的核心结论：**协方差错填会让 EKF 完全失效**。

#### 当前代码状态

```cpp
// chassis_protocol/ros2_adapter/chassis_node.cpp:149-152（核对）
// Copy covariance (36 elements each)
for (size_t i = 0; i < 36; ++i) {
    msg.pose.covariance[i]  = odom.cov_pose[i];
    msg.twist.covariance[i] = odom.cov_twist[i];
}
```

✅ chassis_node 把 HAL 层的 `cov_pose` / `cov_twist` 透传到了 ROS 消息——**这是正确的**。

#### 需要核查 / 落实

- HAL 层（`DifferentialController` / `MecanumController`）填的协方差是什么？
  - 如果是全零 → EKF 把它当"完美测量"，IMU 形同虚设，必坏
  - 如果是占位的 1e-9 → 同上
  - **正确做法**：按 [02 §四.1](./02_odom_fusion_with_robot_localization.md) 经验值（vx 对角 0.01、vyaw 对角 0.05、其它通道 1e6）填进 HAL 数据结构

#### TODO（建议在 chassis_protocol 跟进）

| 项 | 描述 |
|---|---|
| HAL 协方差默认值 | 在 `DifferentialController::publishOdom()` 等位置设合理默认（差速 vy/vz/vroll/vpitch = 1e6）|
| 打滑检测 | 当电流异常 / 编码器突变时，把 `cov_twist[0]` (vx) 临时调到 1e3 |
| 速度方差自适应 | 高速时方差略升，低速降——可选优化 |

### 3.2 `/odom` 时间戳

EKF 默认 `sensor_timeout: 0.1s`。如果 chassis_node 用了"消息到达 ROS 节点的时刻"而不是"采样时刻"，在串口/网络抖动下时间戳会不连续，EKF 拒绝更新。

**正确做法**：
- 消息携带 MCU 端的硬件时间戳（如有 PTP 同步则直接用）
- 否则用 `node_->now()`（即 ROS 时钟），但必须在 HAL 收到帧的瞬间立即取，不要在 callback 队列尾部才取

### 3.3 `frame_id` / `child_frame_id`

```cpp
// chassis_node.cpp:76-77
node_->declare_parameter("frame_id",       std::string("odom"));
node_->declare_parameter("child_frame_id", std::string("base_link"));
```

✅ 默认值 `odom` / `base_link` 与本专题 EKF / SLAM Toolbox 配置一致。

> 如果项目用 `base_footprint`，记得**统一改全栈**（chassis_node `child_frame_id`、EKF `base_link_frame`、SLAM Toolbox `base_frame`、URDF）。

### 3.4 `/cmd_vel` QoS 对接

`nav2_research/07_chassis_protocol_integration.md` §二已展开。要点：Nav2 controller_server 默认 RELIABLE 发布，chassis_node 当前订阅默认 QoS（rclcpp 默认 = 10 depth, RELIABLE/VOLATILE）—— 一致 ✅。

---

## 四、TogetheROS / hobot_* 衔接

S100 自带的 hobot 系列节点与本方案的兼容点：

| Hobot 节点 | 输出话题 | 与本方案对接 |
|-----------|---------|-------------|
| `hobot_sensors` LiDAR | `/scan`（标准 LaserScan）| ✅ 直接喂 slam_toolbox / Nav2 |
| `hobot_codec` 相机 | `/color`、`/depth`、`/info`（标准 Image / CameraInfo） | ✅ 直接喂 depth_image_proc → STVL |
| `hobot_dnn` | 推理结果话题（vision_msgs 兼容）| 与本专题无关（属于感知，不是 SLAM）|
| `hobot_imu`（如有）| `/imu/data_raw` 标准 sensor_msgs/Imu | ✅ 喂 Madgwick |

**不兼容/需自查**：

- TogetheROS 的 ROS 2 版本若不是 Humble（如 Foxy），`slam_toolbox` 与 `robot_localization` 的对应分支需匹配
- hobot 节点的 QoS 默认值需用 `ros2 topic info -v` 验证；与本方案要求不一致时优先**改本方案订阅 QoS**而不是改 hobot 节点
- rmw 实现：S100 默认是哪一个需确认；本方案以 cyclonedds 测试为基线

> **建议第一次启动顺序**（核对兼容性）：
> 1. 单独跑 `hobot_sensors` LiDAR → `ros2 topic echo /scan` 验证频率/帧 ID
> 2. 加上 `chassis_node` → `ros2 topic echo /odom` 验证协方差 ≠ 0
> 3. 加上 `imu_filter_madgwick + ekf_filter_node` → `tf2_echo odom base_link`
> 4. 加上 `slam_toolbox` → RViz 看 `/map` 出现
> 5. 加上 RGB-D pipeline → `/depth/points_filtered` 频率
> 6. 最后启动 Nav2

---

## 五、S100 资源排布建议

### 5.1 CPU 核分配（假设 S100 有 ≥ 4 ARM 核可用）

| Core | 任务 | 隔离/调度 |
|------|------|----------|
| 0 | chassis_node + IMU 驱动 + 雷达驱动 | SCHED_FIFO（参考 `realtime_research/`）|
| 1 | ekf_filter_node + Madgwick + slam_toolbox 主线程 | SCHED_OTHER nice=-5 |
| 2 | Nav2 controller_server / planner_server / collision_monitor | SCHED_OTHER nice=0 |
| 3 | depth_image_proc / pcl_ros / STVL / 其它感知 | SCHED_OTHER nice=5 |

> 实际 S100 核数 / 大小核架构需要在硬件文档中确认；本表是**编排原则**，不是 affinity 文件。

### 5.2 内存预算（粗估）

| 组件 | 内存 |
|------|------|
| chassis_node | < 50 MB |
| ekf_filter_node | < 30 MB |
| slam_toolbox（建图 1h，中型办公室）| 200–400 MB |
| Nav2 全栈 | 250–500 MB |
| RGB-D pipeline + STVL | 150–300 MB |
| **合计** | **~700 MB – 1.3 GB** |

S100 主流配置 4–8GB DDR——在 1GB 工作集上应能轻松跑动。

### 5.3 存储

- 地图持久化：`/maps/current/`（见 [05](./05_dynamic_and_lifelong.md) §四）
- 日志 / rosbag：建议挂载独立分区 `/data`，避免地图 IO 与日志冲突
- TogetheROS 镜像本身放 `/`

---

## 六、典型部署 launch（一键起栈）

```python
# bringup/launch/full_stack.launch.py（伪代码骨架）
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # 1) 静态描述
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=['urdf/my_robot.urdf']),

        # 2) 传感器驱动（按平台选择）
        # —— S100：用 hobot_sensors / hobot_codec
        Node(package='hobot_sensors', executable='lidar_driver',
             parameters=['config/lidar.yaml']),
        Node(package='hobot_codec', executable='rgbd_node',
             parameters=['config/rgbd.yaml']),

        # 3) 底盘
        Node(package='chassis_protocol', executable='chassis_node',
             parameters=['config/chassis.yaml']),

        # 4) IMU 滤波
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             parameters=['config/imu_filter.yaml']),

        # 5) odom 融合
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node',
             parameters=['config/ekf.yaml']),

        # 6) SLAM
        Node(package='slam_toolbox', executable='localization_slam_toolbox_node',
             parameters=['config/slam_localization.yaml',
                         {'map_file_name': '/maps/current/my_map'}]),

        # 7) RGB-D pipeline（同进程组合）
        ComposableNodeContainer(
            name='rgbd_container', package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ...见 [04] §三.2
            ]),

        # 8) Nav2
        IncludeLaunchDescription('config/nav2_bringup.launch.py'),
    ])
```

---

## 七、首次实机调试 checklist

按下面顺序逐项过，每项不通过不要进下一项：

```
□ [硬件] S100 / 雷达 / 相机 / IMU / 底盘电源全部就绪
□ [URDF] base_link → laser_frame / imu_link / camera_link 静态 TF 与实物一致
□ [底盘] chassis_node 启动，ros2 topic hz /odom 在合理范围
□ [底盘] /odom 的 twist.covariance 不全为 0；vx 项 0.001–0.1 量级
□ [雷达] /scan 频率与雷达额定一致（10/15/20 Hz）；frame_id 正确
□ [IMU] /imu/data_raw 频率（典型 100/200 Hz）；accelerometer 静止时 z≈9.8
□ [IMU] /imu/data 输出 orientation 不为 NaN；gyro 静止时 ≈ 0
□ [TF] tf2_echo odom base_link 输出连续，由 ekf_filter_node 发布
□ [TF] 关闭 IMU 一段时间后 ekf 输出明显劣化（验证 IMU 真的进入了融合）
□ [SLAM] slam_toolbox 启动，/map 在 RViz 可见
□ [SLAM] map → odom TF 由 slam_toolbox 发布
□ [SLAM] 推车走一圈，地图无明显穿墙、回环触发
□ [RGB-D] /depth/image_rect_raw 频率符合预期
□ [RGB-D] /depth/points_filtered 在 RViz 可见，桌椅腿清晰
□ [Nav2] STVL 层在 RViz 显示障碍；动态物体衰减可见
□ [Nav2] 给一个目标点，机器人能避开桌椅腿
□ [整体] CPU 占用 < 70%、内存 < 1.5GB、TF 无丢失警告
□ [基线] 跑 [06] §五的回归测试套件，记录 baseline 指标
```

---

## 八、选 ros1 vs 上游 ROS 2 包的小心事项

**必须**全部用 ROS 2 包：
- `slam_toolbox`（ROS 2 已成熟）
- `robot_localization`（ROS 2 已成熟）
- `nav2`（ROS 1 没有同版本）
- `imu_filter_madgwick`（ROS 2 已有）
- `depth_image_proc` / `pcl_ros`（ROS 2 已有）
- `spatio_temporal_voxel_layer`（apt 有 ROS 2 humble 版）

**潜在 ROS 1 only / 需注意**：
- VINS-Fusion 上游是 ROS 1；用 ROS 2 fork 或 ros1_bridge
- ORB-SLAM3 ROS 2 端是社区 fork，需评估稳定性

**TogetheROS 的版本**需 ros2_control / Nav2 / slam_toolbox 同 ROS 2 distro（建议 Humble）。

---

## 九、与已有调研文档的引用关系

| 已有文档 | 本节引用方式 |
|---------|-------------|
| `nav2_research/07_chassis_protocol_integration.md` | QoS 表格、`/cmd_vel` 对接细节直接复用 |
| `nav2_research/06_localization.md` | 入门级对比；本节是工程化升级 |
| `realsense-ros-4.57.7-analysis` / `orbbec_sdk_ros2_analysis` | 替代 hobot_codec 的相机驱动方案（如果用上游 RealSense / Orbbec 而不是 hobot）|
| `realtime_research/` | CPU 隔离 / SCHED_FIFO / RT 内核（本节 §五.1 所依据）|
| `chassis_protocol/CMakeLists.txt`、`chassis_node.cpp` | 现状核查、TODO 列表的事实依据 |

---

## 十、后续工作建议

短期（1–2 周）：
1. 在 chassis_protocol HAL 层补全协方差默认值（差速 vy/vz/vroll/vpitch = 1e6）
2. 跑通 [04](./04_3d_perception_for_2d_nav.md) 的 STVL pipeline 并 [06](./06_evaluation_and_benchmarking.md) 录第一份基线
3. 在 RDK S100 实机上验证 hobot_sensors → /scan 与本栈兼容性

中期（1 个月）：
4. 把 [03](./03_visual_slam_cpu_backends.md) 的 OpenVINS 兜底跑起来（在退化场景测试集上）
5. lifelong 模式连续 24h 稳定性测试

长期（按需）：
6. 评估是否需要把 SLAM 主线程改造为 LifecycleNode（与 Nav2 lifecycle_manager 统一编排）
7. RGB-D 替换为带 IMU 的相机（如 RealSense D435i + IMU）做硬同步

---

> 至此 `slam_research/` 调研专题完成。后续如果引入 3D LiDAR、多机协同、户外 RTK 等需求，建议另起专题。
