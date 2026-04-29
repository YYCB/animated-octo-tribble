# 与 `chassis_protocol` 的完整对接清单

> 本文档是 Nav2 与本仓库 `chassis_protocol` HAL 层对接的操作手册。  
> chassis_protocol 架构：proto → frame_codec → ITransport → ChassisHal → IChassisController → ChassisNode

---

## 一、对接全景图

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Nav2 栈                                                                │
│  controller_server (20 Hz) → /cmd_vel                                   │
│      → velocity_smoother (50 Hz) → /cmd_vel_smoothed                    │
│      → collision_monitor → /cmd_vel_chassis                             │
│  planner_server ← /map + global_costmap                                │
│  bt_navigator ← TF: map → odom → base_link                             │
│  amcl/slam_toolbox ← /scan (LaserScan)                                 │
└─────────────────────────────────────────────────────────────────────────┘
        ↕ /cmd_vel_chassis（发）  ↕ /odom（收）        ↕ TF（收）
┌─────────────────────────────────────────────────────────────────────────┐
│  chassis_protocol → ros2_control 接口                                   │
│  DiffDriveController                                                    │
│  ├── 订阅：/cmd_vel → 驱动底盘                                           │
│  └── 发布：/odom (nav_msgs/Odometry)                                    │
│            TF: odom → base_link（通过 odom publisher）                  │
│            /joint_states（轮子 pos/vel）                                 │
└─────────────────────────────────────────────────────────────────────────┘
        ↕ 串口/UDP/RS485
┌─────────────────────────────────────────────────────────────────────────┐
│  底盘硬件（电机驱动器）                                                   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 二、QoS 对接清单

Nav2 内部对各 Topic 的 QoS 有**严格约定**，chassis_protocol 的发布/订阅 QoS 必须匹配。

### 2.1 /cmd_vel（Nav2 发布，chassis_protocol 订阅）

```yaml
# Nav2 controller_server 发布 /cmd_vel 的 QoS：
QoS:
  reliability: RELIABLE
  durability: VOLATILE
  history: KEEP_LAST
  depth: 1

# chassis_protocol DiffDriveController 订阅：
# ros2_control DiffDriveController 默认订阅 QoS：
QoS:
  reliability: BEST_EFFORT  # ← 注意！默认是 BEST_EFFORT
  # 需要改为 RELIABLE 以匹配 Nav2 发布方
```

**解决方案**：在 DiffDriveController 参数中设置 QoS 兼容：

```yaml
diff_drive_controller:
  ros__parameters:
    cmd_vel_topic: cmd_vel
    # DiffDriveController Humble+ 支持配置 QoS
    cmd_vel_qos:
      reliability: reliable   # 确保与 Nav2 匹配
```

或在 Nav2 侧发布 BEST_EFFORT：

```yaml
controller_server:
  ros__parameters:
    cmd_vel_topic: cmd_vel
    # Nav2 Humble+ 支持配置输出 QoS
    use_realtime_priority: false
```

**最简方案**：使用 `topic_tools::relay` 节点在两者之间做 QoS 桥接（不推荐，增加延迟）。

### 2.2 /odom（chassis_protocol 发布，Nav2 消费）

```yaml
# Nav2 bt_navigator / amcl 订阅 /odom 的 QoS：
QoS:
  reliability: RELIABLE
  durability: VOLATILE
  history: KEEP_LAST
  depth: 1

# ros2_control DiffDriveController 发布 /odom 的默认 QoS：
QoS:
  reliability: RELIABLE  # ✅ 已匹配
  history: KEEP_LAST
  depth: 100             # depth 差异无关紧要（订阅方不缓冲）
```

### 2.3 /scan（激光雷达，AMCL/Costmap 消费）

```yaml
# AMCL 和 ObstacleLayer 期望的 QoS：
QoS:
  reliability: BEST_EFFORT  # 激光雷达通常用 BEST_EFFORT（传感器数据）
  history: KEEP_LAST
  depth: 5

# 激光雷达驱动默认通常已经是 BEST_EFFORT，无需修改
```

---

## 三、TF 树对接清单

Nav2 要求完整的 TF 树：`map → odom → base_link → [传感器坐标系]`

### 3.1 TF 发布职责划分

| TF 变换 | 发布者 | 频率 | 备注 |
|---------|--------|------|------|
| `map → odom` | AMCL / SLAM Toolbox | 变化时（~10 Hz）| 定位节点修正累积误差 |
| `odom → base_link` | chassis_protocol（DiffDriveController）| 50-100 Hz | 里程计积分 |
| `base_link → base_footprint` | 静态 TF（launch 中）| 静态 | 2D 导航用 base_footprint |
| `base_link → laser_frame` | 激光雷达驱动 / 静态 TF | 静态 | URDF 中定义 |
| `base_link → camera_frame` | 相机驱动 / 静态 TF | 静态 | 深度相机 |

### 3.2 DiffDriveController 的 TF 发布配置

```yaml
diff_drive_controller:
  ros__parameters:
    # TF 配置
    base_frame_id: base_footprint   # 与 Nav2 的 robot_base_frame 一致
    odom_frame_id: odom
    publish_odom: true              # 发布 /odom topic
    publish_odom_tf: true           # 发布 odom → base_footprint TF（✅必须开启）
    
    # 里程计配置
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    
    # 底盘参数（必须与实际硬件一致）
    wheel_separation: 0.287         # m，两轮中心距
    wheel_radius: 0.033             # m，轮子半径
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
```

### 3.3 静态 TF（launch 文件中）

```python
# my_robot_bringup/launch/bringup.launch.py
def generate_launch_description():
    # base_footprint → base_link（地面投影点 → 机器人中心）
    base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.0', '0', '0', '0',
                   'base_footprint', 'base_link'])
    
    # base_link → laser_frame（激光雷达安装位置）
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.18', '0', '0', '0',  # 18cm 高
                   'base_link', 'laser_frame'])
    
    return LaunchDescription([
        base_to_footprint,
        laser_tf,
        # URDF 中有完整坐标系时，用 robot_state_publisher 代替静态 TF
    ])
```

---

## 四、Odom 来源选择

chassis_protocol 可以提供两种里程计数据：

### 4.1 方案 A：编码器里程计（推荐）

```
底盘电机编码器 → frame_codec 解析 → ChassisHal → ChassisNode
→ DiffDriveController（通过 ros2_control StateInterface 读取轮速）
→ 积分计算 → /odom + TF

优点：高频（与控制周期同频，50-100 Hz）
缺点：累积漂移（长时间运行后定位误差大）
需要 AMCL/SLAM Toolbox 修正 map → odom
```

### 4.2 方案 B：融合里程计（可选增强）

```
编码器里程计 + IMU（chassis_protocol 中的 IMU 读取）
→ robot_localization EKF 融合
→ 精度更高的 /odom + TF

适用：地面不平、轮子打滑场景
参考：robot_localization EKF 节点（nav2 官方推荐伴侣）
```

### 4.3 robot_localization EKF 配置（可选）

```yaml
# ekf_node（robot_localization 包）
ekf_filter_node:
  ros__parameters:
    frequency: 50.0          # Hz，EKF 输出频率
    sensor_timeout: 0.1      # s
    two_d_mode: true         # 2D 导航（忽略 z/roll/pitch）
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
    
    # 传感器：编码器里程计
    odom0: /diff_drive_controller/odom  # DiffDriveController 的里程计
    odom0_config: [true, true, false,
                   false, false, true,
                   true, true, false,
                   false, false, true,
                   false, false, false]  # x, y, yaw + 速度
    odom0_differential: false
    
    # 传感器：IMU（若 chassis_protocol 提供）
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true,    # roll, pitch, yaw
                  false, false, false,
                  true, true, true,    # angular_vel
                  true, false, false]  # linear_acc_x
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
```

---

## 五、cmd_vel 拓扑（完整命令链路）

```
上层来源（任选其一激活）：
  Nav2 bt_navigator → controller_server → /cmd_vel        （20 Hz，受 controller_frequency 约束）
  遥控器 → teleop_twist_joy → /teleop/cmd_vel
  LLM Agent（rai）→ navigate_to Action → Nav2 → /cmd_vel
  手动调试 → ros2 topic pub /cmd_vel ...

  ↓ twist_mux（命令优先级仲裁，按优先级合并多源 cmd_vel）
     配置：teleop（最高）> e_stop（停止）> nav2 > default（最低）
  ↓
  /cmd_vel（仲裁后唯一命令，仍是 20 Hz）
  ↓
  nav2_velocity_smoother（频率解耦 + 加速度限幅，输入 20 Hz / 输出 50 Hz）
  ↓
  /cmd_vel_smoothed（50 Hz）
  ↓
  nav2_collision_monitor（最后一道安全检查，可以降速/停止）
  ↓
  /cmd_vel_chassis（送往底盘的最终命令）
  ↓
  DiffDriveController（topic_remapping 到 /velocity_controller/cmd_vel）
  ↓
  ros2_control StateInterface/CommandInterface（update_rate 100 Hz）
  ↓
  chassis_protocol SystemInterface（write/read）
  ↓
  底盘驱动器（RS485/UDP）
```

> **关键约束**：在 S100（D-Robotics RDK，ARM + BPU，CPU-only）平台上，DWB/MPPI 单次评估代价较重，
> `controller_frequency` 不应超过 20 Hz，否则会出现 `control loop missed its desired rate` 警告。
> 因此**规划/控制频率（20 Hz）与下发频率（50 Hz）必须解耦**——这正是 §5.2 引入 `velocity_smoother` 的原因。

### 5.1 twist_mux 配置

```yaml
# twist_mux_params.yaml
twist_mux:
  topics:
    - name: teleop          # 遥控器（最高优先级，安全接管）
      topic: /teleop/cmd_vel
      timeout: 0.5
      priority: 100
    - name: nav             # Nav2 导航
      topic: /cmd_vel
      timeout: 0.5
      priority: 10
  locks:
    - name: e_stop          # E-Stop 锁（触发后停止所有运动）
      topic: /e_stop
      timeout: 0.0          # 永久有效直到解锁
      priority: 255
```

### 5.2 nav2_velocity_smoother 配置（频率解耦：20 Hz → 50 Hz）

#### 5.2.1 为什么需要

Nav2 `controller_server` 按 `controller_frequency` 周期性调用控制器插件（DWB / MPPI / RPP）算一次速度
并发布 `/cmd_vel`，**默认 20 Hz**。下游（chassis_protocol → DiffDriveController → 底盘）若需要 50 Hz
的命令流（topic 层面或底盘协议帧率要求），不应通过简单提高 `controller_frequency` 来实现：

- S100 ARM + BPU（CPU-only）单次 DWB/MPPI 评估代价较重，50 Hz 大概率跑不满，会出现
  `control loop missed its desired rate` 警告，反而引发抖动；
- 控制器频率提高，local_costmap 更新、TF 查询都要跟着提速，整体算力吃紧；
- 用 Python relay 节点把 20 Hz 复制重发成 50 Hz 没有加速度限幅，遇到突变指令会冲击底盘；
- 在 `chassis_protocol` 帧编解码层做插值违反 HAL 分层（参见 `chassis_protocol/docs/design/01_architecture.md`），
  HAL 层应保持无状态写入。

#### 5.2.2 推荐做法：nav2_velocity_smoother

`nav2_velocity_smoother` 以自己的 `smoothing_frequency` 重新发布 cmd_vel，并在两次 Nav2 输出之间
按加速度/加加速度（jerk）限幅做插值，**输入 20 Hz、输出 50 Hz**，且更平滑。

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 50.0       # ← 输出 50 Hz（与底盘期望频率对齐）
    feedback: "OPEN_LOOP"           # 或 CLOSED_LOOP（用 odom 反馈实际速度）
    odom_topic: /odom
    odom_duration: 0.1              # s，odom 数据有效窗口
    # 速度/加速度限幅（与 ros2_control DiffDriveController 限幅保持一致）
    max_velocity:      [0.5, 0.0, 1.5]    # [vx, vy, wz]，差速底盘 vy=0
    min_velocity:     [-0.5, 0.0, -1.5]
    max_accel:         [1.0, 0.0, 2.0]
    max_decel:        [-1.5, 0.0, -2.5]
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0           # s，超过此时间无新 cmd_vel 则归零
    # Topic 重映射：输入接 twist_mux 输出，输出接 collision_monitor 输入
    # 一般在 launch 中通过 remappings 完成：
    #   ('cmd_vel',          'cmd_vel'         )  ← 来自 twist_mux
    #   ('cmd_vel_smoothed', 'cmd_vel_smoothed')  ← 送往 collision_monitor
```

#### 5.2.3 与 collision_monitor 的串联

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: cmd_vel_smoothed   # ← 接 velocity_smoother 输出
    cmd_vel_out_topic: cmd_vel_chassis   # ← 送给 DiffDriveController
    # ... polygons / sources 配置略
```

#### 5.2.4 决策建议（方案选型）

| 真实约束 | 推荐方案 |
|---------|---------|
| ROS topic 层面要求 50 Hz（下游订阅者按 topic 频率工作）| **方案 A**：加 `nav2_velocity_smoother`，`smoothing_frequency: 50.0`（本节） |
| 仅关心电机速度环刷新 50/100 Hz | **方案 B**：调高 `ros2_control` `update_rate`，`cmd_vel` 留 20 Hz；`DiffDriveController` 内部以 `update_rate` 持续把最近一次 cmd_vel 写到底盘，直到 `cmd_vel_timeout` |
| 物理底盘协议要求 ≥ 50 Hz 心跳/速度帧（否则触发看门狗）| 在 `chassis_protocol` 的 `IChassisController` 层做"保持最后值并以固定频率周期性发送"，与 `ros2_control` `update_rate` 对齐（HAL 的合理职责）|

> 绝大多数情况选 **方案 A**：规划 20 Hz 不动，下发 50 Hz，平滑性也更好。
> 方案 B 与方案 A 不互斥，可同时使用（见 §5.2.5）。

#### 5.2.5 方案 B 的 ros2_control 端配置参考

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100              # ros2_control 主循环 100 Hz

diff_drive_controller:
  ros__parameters:
    publish_rate: 50              # /odom 与 odom→base_link TF 发布频率
    cmd_vel_timeout: 0.5          # 超过此时间无新 cmd_vel 则归零
    # 限幅应与 velocity_smoother 一致，避免双重限幅互相打架
    linear.x.max_velocity: 0.5
    linear.x.max_acceleration: 1.0
    angular.z.max_velocity: 1.5
    angular.z.max_acceleration: 2.0
```

---

## 六、chassis_protocol 改造对接检查表

| 项目 | 状态 | 说明 |
|------|:----:|------|
| ChassisNode 实现 DiffDriveController 所需的 `wheel_left_joint` / `wheel_right_joint` StateInterface | 需验证 | 参见 Phase 1 对接评估 |
| `/odom` 发布（nav_msgs/Odometry）| 需验证 | DiffDriveController 默认发布 |
| `odom → base_footprint` TF 发布 | 需验证 | `publish_odom_tf: true` |
| `wheel_separation` / `wheel_radius` 参数正确配置 | 需验证 | 与实际硬件一致 |
| `/cmd_vel` 订阅 QoS 与 Nav2 兼容 | 需修复 | DiffDriveController BEST_EFFORT → RELIABLE |
| 激光雷达发布 `/scan` 并有正确 `frame_id` | 需验证 | `laser_frame` 需在 TF 树中 |
| 静态 TF：`base_link → laser_frame` 已配置 | 需添加 | 在 bringup launch 中添加 |
| `nav2_velocity_smoother` 配置（频率解耦 20 → 50 Hz）| 需添加 | `smoothing_frequency: 50.0`，参见 §5.2 |
| `nav2_collision_monitor` cmd_vel 链路配置 | 需添加 | `cmd_vel_in_topic: cmd_vel_smoothed`，`cmd_vel_out_topic: cmd_vel_chassis` |
| E-Stop 与 ros2_control lifecycle `deactivate` 联动 | 需实现 | 参见 Phase 1 / Phase 3 安全层 |

---

## 七、Nav2 + chassis_protocol 最小化 launch 文件

```python
# my_robot_nav2/launch/nav2_chassis_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    my_robot = get_package_share_directory('my_robot_bringup')

    return LaunchDescription([
        # 1. 静态 TF
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_frame']),

        # 2. chassis_protocol（ros2_control + DiffDriveController）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [my_robot, '/launch/chassis.launch.py'])),

        # 3. 激光雷达驱动（如 rplidar）
        Node(package='rplidar_ros', executable='rplidar_composition',
             parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser_frame'}]),

        # 4. Nav2（完整栈）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup, '/launch/bringup_launch.py']),
            launch_arguments={
                'map': [my_robot, '/maps/my_map.yaml'],
                'params_file': [my_robot, '/config/nav2_params.yaml'],
                'use_sim_time': 'false',
            }.items()),
    ])
```
