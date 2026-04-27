# 与 `chassis_protocol` 的完整对接清单

> 本文档是 Nav2 与本仓库 `chassis_protocol` HAL 层对接的操作手册。  
> chassis_protocol 架构：proto → frame_codec → ITransport → ChassisHal → IChassisController → ChassisNode

---

## 一、对接全景图

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Nav2 栈                                                                │
│  controller_server → /cmd_vel_smoothed (collision_monitor 之后)         │
│  planner_server ← /map + global_costmap                                │
│  bt_navigator ← TF: map → odom → base_link                             │
│  amcl/slam_toolbox ← /scan (LaserScan)                                 │
└─────────────────────────────────────────────────────────────────────────┘
        ↕ /cmd_vel（发）        ↕ /odom（收）        ↕ TF（收）
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
  Nav2 bt_navigator → controller_server → /cmd_vel
  遥控器 → teleop_twist_joy → /teleop/cmd_vel
  LLM Agent（rai）→ navigate_to Action → Nav2 → /cmd_vel
  手动调试 → ros2 topic pub /cmd_vel ...

  ↓ twist_mux（命令优先级仲裁，按优先级合并多源 cmd_vel）
     配置：teleop（最高）> e_stop（停止）> nav2 > default（最低）
  ↓
  /cmd_vel（仲裁后唯一命令）
  ↓
  nav2_collision_monitor（最后一道安全检查，可以降速/停止）
  ↓
  /cmd_vel_smoothed
  ↓
  DiffDriveController（/velocity_controller/cmd_vel，topic_remapping）
  ↓
  ros2_control StateInterface/CommandInterface
  ↓
  chassis_protocol SystemInterface（write/read）
  ↓
  底盘驱动器（RS485/UDP）
```

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
| `nav2_collision_monitor` cmd_vel 链路配置 | 需添加 | `cmd_vel_out` → `cmd_vel_smoothed` |
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
