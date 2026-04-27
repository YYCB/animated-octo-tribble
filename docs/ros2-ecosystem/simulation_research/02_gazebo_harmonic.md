# Gazebo Harmonic（gz-sim 8.x）深度调研

> **版本基准**：Gazebo Harmonic 8.x（LTS，支持至 2028 年）/ ROS 2 Jazzy 官方配对  
> **关联文档**：
> - `docs/ros2-ecosystem/ros2_control_research/`（Phase 1，Hardware Interface）
> - `docs/ros2-ecosystem/rosbag2_research/`（Phase 6，仿真录制）
> - `docs/ros2-ecosystem/realtime_research/07_integration.md`（Phase 7，进程隔离）

---

## 一、架构：gz-sim ECS 设计

Gazebo Harmonic（正式名称 gz-sim，非"Gazebo Classic"）采用 Entity-Component-System (ECS) 架构，与 Classic Gazebo 有根本性差异：

### 1.1 ECS 核心概念

```
gz-sim ECS 架构
┌──────────────────────────────────────────────────────────────────┐
│  World（场景图）                                                   │
│  ├─ Entity 0（World）                                             │
│  │    └─ Components: WorldComponent, PhysicsComponent            │
│  ├─ Entity 1（Model: robot）                                      │
│  │    └─ Components: ModelComponent, PoseComponent, NameComponent│
│  │         └─ Entity 2（Link: base_link）                         │
│  │              └─ Components: LinkComponent, InertialComponent   │
│  │                   └─ Entity 3（Joint: left_wheel_joint）       │
│  │                        └─ Components: JointComponent,          │
│  │                                       JointVelocityCmd         │
│  └─ Entity N（Sensor: lidar）                                     │
│       └─ Components: SensorComponent, RenderEnginePlugin          │
├──────────────────────────────────────────────────────────────────┤
│  Systems（插件，在每个仿真步骤按优先级执行）                          │
│  ├─ Physics（gz::sim::systems::Physics）— 积分器 + 碰撞检测        │
│  ├─ Sensors（gz::sim::systems::Sensors）— 传感器数据更新           │
│  ├─ SceneBroadcaster — 可视化状态广播                              │
│  ├─ UserCommands — 外部命令注入                                    │
│  └─ 自定义 System 插件（gz::sim::System 接口）                     │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 Server / Client 分离架构

```
┌─────────────────┐   gz-transport (zenoh)   ┌──────────────────┐
│  gz-sim Server  │◄────────────────────────►│  gz-sim Client   │
│  (仿真进程)      │   服务 / 话题 / 可视化    │  (GUI 进程)       │
│  - 物理积分      │                          │  - 场景渲染       │
│  - 传感器更新    │                          │  - 用户交互       │
│  - 系统插件      │                          │  - Inspector UI  │
└─────────────────┘                          └──────────────────┘
         ▲
         │ gz-transport
         ▼
┌─────────────────┐
│  ros_gz_bridge  │   ←→  ROS 2 DDS 域
│  (桥接进程)      │
└─────────────────┘
```

**分离架构优势**：Server 无 GUI 时（`--headless-rendering`）可在 CI / 训练服务器运行，Client 可连接远程 Server 进行调试。

---

## 二、与 Classic Gazebo 的核心区别

| 特性 | Classic Gazebo（11） | Gazebo Harmonic（8.x） |
|------|---------------------|----------------------|
| **架构** | 整体 Monolithic | ECS（Entity-Component-System） |
| **传输层** | ROS 1 style msgs / Gazebo Transport | gz-transport（基于 zenoh 路由） |
| **插件 API** | `gazebo::ModelPlugin`（C++ 虚基类） | `gz::sim::System`（接口集合） |
| **世界格式** | SDF 1.6-1.9 | SDF 1.10-1.11（更丰富物理属性） |
| **物理引擎** | ODE（默认）/ Bullet | DART（默认）/ Bullet / TPE |
| **渲染** | Ogre 1.x | Ogre 2.x（`gz-rendering 8`） |
| **Python 支持** | 有限 | ✅ 完整 Python bindings（8.x） |
| **ARM64** | ⚠️ 非官方 | ✅ 官方支持 |
| **包名** | `gazebo_*` | `gz_*` / `ros_gz_*` |
| **ROS 2 版本** | 通过 `gazebo_ros_pkgs`（Humble 仍支持） | `ros_gz`（Jazzy 默认） |

### 2.1 自定义系统插件 API

```cpp
// gz-sim 8.x 自定义 System 插件示例
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

class MyChassisPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,  // 初始化时调用
      public gz::sim::ISystemPreUpdate,  // 物理步前调用
      public gz::sim::ISystemUpdate,     // 物理步调用（积分后）
      public gz::sim::ISystemPostUpdate  // 物理步后（只读）
{
public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override
    {
        // 读取 SDF 参数，注册感兴趣的 Components
    }
    
    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override
    {
        // 注入控制命令（关节速度 / 力矩）
        auto velCmd = ecm.Component<gz::sim::components::JointVelocityCmd>(joint_entity_);
        *velCmd = gz::sim::components::JointVelocityCmd({target_vel_});
    }
};

GZ_ADD_PLUGIN(MyChassisPlugin, gz::sim::System,
              MyChassisPlugin::ISystemConfigure,
              MyChassisPlugin::ISystemPreUpdate)
```

---

## 三、`ros_gz_bridge`：ROS 2 ↔ gz-transport 桥接

### 3.1 架构与原理

`ros_gz_bridge` 实现 ROS 2 DDS 与 gz-transport（zenoh）之间的消息转换：

```
ROS 2 Publisher → DDS → ros_gz_bridge → gz-transport → gz-sim
gz-sim Sensor  → gz-transport → ros_gz_bridge → DDS → ROS 2 Subscriber
```

### 3.2 `parameter_bridge` 节点配置

```yaml
# ros_gz_bridge.yaml — 完整桥接配置示例
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name: "/model/robot/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states"
  gz_topic_name: "/model/robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

### 3.3 Launch 文件集成

```python
# ros_gz_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ros2pkg import get_package_share_directory

def generate_launch_description():
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py',
        ]),
        launch_arguments={
            'gz_args': '-r -s --headless-rendering world.sdf',
            # -r: run immediately; -s: server only (no GUI)
        }.items(),
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                get_package_share_directory('my_robot'),
                'config', 'ros_gz_bridge.yaml'
            ]),
            'use_sim_time': True,
        }],
        output='screen',
    )
    
    return LaunchDescription([gz_sim, bridge])
```

### 3.4 支持的消息类型对照表

| ROS 2 类型 | gz.msgs 类型 | 方向 |
|-----------|-------------|------|
| `rosgraph_msgs/msg/Clock` | `gz.msgs.Clock` | GZ→ROS |
| `geometry_msgs/msg/Twist` | `gz.msgs.Twist` | ROS→GZ |
| `geometry_msgs/msg/Wrench` | `gz.msgs.Wrench` | GZ→ROS |
| `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | GZ→ROS |
| `sensor_msgs/msg/Image` | `gz.msgs.Image` | GZ→ROS |
| `sensor_msgs/msg/CameraInfo` | `gz.msgs.CameraInfo` | GZ→ROS |
| `sensor_msgs/msg/PointCloud2` | `gz.msgs.PointCloudPacked` | GZ→ROS |
| `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` | GZ→ROS |
| `sensor_msgs/msg/Imu` | `gz.msgs.IMU` | GZ→ROS |
| `sensor_msgs/msg/JointState` | `gz.msgs.Model` | GZ→ROS |
| `tf2_msgs/msg/TFMessage` | `gz.msgs.Pose_V` | 双向 |
| `std_msgs/msg/Bool` | `gz.msgs.Boolean` | 双向 |
| `std_msgs/msg/Float64` | `gz.msgs.Double` | 双向 |

---

## 四、`gz_ros2_control`：ros2_control 仿真集成

### 4.1 GazeboSystem 原理

`gz_ros2_control` 实现了 `ros2_control` 的 `SystemInterface` 抽象，使同一套控制器代码无需修改即可在仿真中运行：

```
gz-sim 物理循环
    ↓  PreUpdate()
gz_ros2_control::GazeboSystem::read()
    ↓  从 ECM 读取关节位置/速度/力矩
controller_manager::update()
    ↓  运行所有注册控制器
gz_ros2_control::GazeboSystem::write()
    ↓  将控制命令写入 ECM Components
gz-sim 物理积分
```

### 4.2 URDF/xacro 配置

```xml
<!-- robot.urdf.xacro — gz_ros2_control 插件配置 -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="chassis">
  
  <!-- ros2_control 硬件接口声明 -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <!-- 仿真模式：使用 GazeboSystem 插件 -->
      <plugin>gz_ros2_control/GazeboSystem</plugin>
    </hardware>
    
    <joint name="left_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-10.0</param>
        <param name="max">10.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  
  <!-- SDF 模型层：gz_ros2_control 系统插件 -->
  <gazebo>
    <plugin filename="gz_ros2_control-system" 
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <parameters>$(find my_robot)/config/ros2_controllers.yaml</parameters>
      <controller_manager_prefix_node_name>controller_manager</controller_manager_prefix_node_name>
    </plugin>
  </gazebo>
  
</robot>
```

### 4.3 迁移：`ign_ros2_control` → `gz_ros2_control`

Humble 之前使用 Ignition（ign）命名，Jazzy 起统一使用 gz 命名：

| 旧名（Humble/Ionic） | 新名（Harmonic/Jazzy） |
|---------------------|----------------------|
| `ign_ros2_control` | `gz_ros2_control` |
| `ign_ros2_control::IgnitionSystem` | `gz_ros2_control::GazeboSystem` |
| `ignition_ros2_control` | `gz_ros2_control` |
| SDF 插件名 `ign_ros2_control` | `gz_ros2_control` |
| `GZ_VERSION=fortress` | `GZ_VERSION=harmonic` |

---

## 五、传感器插件：SDF 配置详解

### 5.1 相机传感器

```xml
<!-- SDF 相机传感器配置 -->
<sensor name="rgb_camera" type="camera">
  <always_on>1</always_on>
  <update_rate>30.0</update_rate>
  <visualize>true</visualize>
  <topic>/camera/image</topic>
  <camera name="rgb_camera">
    <horizontal_fov>1.2566</horizontal_fov>  <!-- 72 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>RGB_INT8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- 像素值归一化噪声 -->
    </noise>
    <distortion>
      <k1>-0.25</k1>
      <k2>0.12</k2>
      <k3>0.0</k3>
      <p1>-0.00028</p1>
      <p2>-0.00005</p2>
      <center>0.5 0.5</center>
    </distortion>
  </camera>
</sensor>
```

### 5.2 深度相机（RGBD）

```xml
<sensor name="depth_camera" type="depth_camera">
  <update_rate>30.0</update_rate>
  <topic>/depth_camera</topic>
  <camera name="depth_camera">
    <horizontal_fov>1.2566</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>  <!-- 32位浮点深度 -->
    </image>
    <clip>
      <near>0.2</near>
      <far>8.0</far>    <!-- RealSense D435 最大测量距离 -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1 cm 深度噪声 -->
    </noise>
  </camera>
</sensor>
```

### 5.3 GPU LiDAR

```xml
<sensor name="gpu_lidar" type="gpu_lidar">
  <update_rate>10.0</update_rate>
  <topic>/lidar/points</topic>
  <lidar>
    <scan>
      <horizontal>
        <samples>1800</samples>      <!-- 水平分辨率：0.2° -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>         <!-- 16 线 LiDAR -->
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15° -->
        <max_angle>0.261799</max_angle>   <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>130.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1 cm 测距噪声 -->
    </noise>
  </lidar>
</sensor>
```

### 5.4 IMU 传感器

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>200.0</update_rate>
  <topic>/imu/data</topic>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>         <!-- rad/s 白噪声 -->
          <bias_mean>7.5e-6</bias_mean>
          <bias_stddev>8e-7</bias_stddev>
          <dynamic_bias_stddev>4.3e-6</dynamic_bias_stddev>  <!-- 随机游走 -->
          <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y, z 同上 -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>       <!-- m/s^2 白噪声 -->
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
          <dynamic_bias_stddev>4.3e-4</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
        </noise>
      </x>
    </linear_acceleration>
  </imu>
</sensor>
```

### 5.5 力矩传感器（ForceTorque）

```xml
<sensor name="ft_sensor" type="force_torque">
  <update_rate>100.0</update_rate>
  <topic>/ft_sensor/wrench</topic>
  <force_torque>
    <frame>parent</frame>            <!-- child / parent / sensor -->
    <measure_direction>parent_to_child</measure_direction>
    <force>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>  <!-- 0.1 N 噪声 -->
        </noise>
      </x>
    </force>
    <torque>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.005</stddev>  <!-- 0.005 N·m 噪声 -->
        </noise>
      </x>
    </torque>
  </force_torque>
</sensor>
```

---

## 六、物理引擎对比

Gazebo Harmonic 支持多种物理引擎插件，通过 SDF `<physics>` 标签选择：

```xml
<world name="default">
  <!-- DART 物理引擎（默认推荐） -->
  <physics name="dart_physics" type="dart">
    <max_step_size>0.001</max_step_size>   <!-- 1 ms 步长 -->
    <real_time_factor>1.0</real_time_factor>
    <dart>
      <solver>
        <solver_type>dantzig</solver_type>
        <num_threads>4</num_threads>
      </solver>
    </dart>
  </physics>
  
  <!-- 或使用 Bullet（软体仿真） -->
  <!-- <physics name="bullet_physics" type="bullet">...</physics> -->
</world>
```

| 物理引擎 | 精度 | 接触稳定性 | 软体支持 | 性能 | 适用场景 |
|---------|------|----------|---------|------|---------|
| **DART** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ❌ | 中等 | 机械臂 / 移动机器人（默认推荐） |
| **Bullet** | ⭐⭐⭐ | ⭐⭐⭐ | ✅（软体） | 快 | 软体交互 / 布料仿真 |
| **TPE（Trivial Physics Engine）** | ⭐ | ⭐⭐ | ❌ | 极快 | 纯运动学 / 导航测试 |
| **DART + Bullet 接触** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 部分 | 中等 | 精确接触研究 |

---

## 七、Harmonic（8.x）新特性

### 7.1 gz-rendering 8 与 gz-sensors 8

```
gz-rendering 8 新特性：
├─ Ogre 2.3 后端（Metal / Vulkan / OpenGL 3.3+）
├─ 改进的 PBR 材质（GGX BRDF）
├─ 动态天空盒（大气散射模型）
├─ 更好的阴影质量（CSM：Cascaded Shadow Maps）
└─ GPU 实例化渲染（大规模场景优化）

gz-sensors 8 新特性：
├─ 统一传感器噪声配置（dynamic_bias 支持）
├─ 深度相机结构光噪声模型
├─ GPU 批量 LiDAR 射线投射优化
└─ 热成像相机传感器（thermal_camera）
```

### 7.2 Python Bindings（gz-sim 8 新增）

```python
# gz-sim Python 绑定（Harmonic 8.x）
import gz.sim8 as gz_sim
import gz.math7 as gz_math
import gz.msgs10 as gz_msgs

# 创建仿真服务器
server_config = gz_sim.ServerConfig()
server_config.SetSdfFile("world.sdf")
server = gz_sim.Server(server_config)

# 运行指定步数
server.Run(blocking=False, iterations=1000, paused=False)

# 使用 ECM API 查询实体
# （主要通过 System 插件的 ECM 引用访问）
```

### 7.3 ARM64 官方支持

Harmonic 8.x 提供官方 ARM64 二进制包：

```bash
# Ubuntu 22.04 (Jammy) ARM64 安装
sudo apt install -y lsb-release wget gnupg

# 添加 OSRF apt 源
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
     -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
     signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
     http://packages.osrfoundation.org/gazebo/ubuntu-stable \
     $(lsb_release -cs) main" | \
     sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update && sudo apt install -y gz-harmonic
```

---

## 八、仿真确定性与 `rosbag2` 集成

### 8.1 `gz::sim::Server` 步进 API

```python
# Python 控制仿真步进（确定性测试）
import gz.sim8 as gz_sim
import threading

server = gz_sim.Server(config)
server.Run(blocking=False, iterations=0, paused=True)  # 启动但暂停

def step_simulation(n_steps: int):
    """精确步进 n 步"""
    for _ in range(n_steps):
        server.Run(blocking=True, iterations=1, paused=False)

# 在测试中：步进 → 断言状态 → 步进
step_simulation(100)   # 仿真 100 ms（@10ms步长）
assert robot_pose_x > 0.5  # 断言机器人位置
```

### 8.2 与 `rosbag2` 录制配合

```bash
# 1. 启动无 GUI 仿真（headless + 步进控制）
gz sim --headless-rendering -r world.sdf &

# 2. 启动桥接
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=bridge.yaml

# 3. 录制关键 topic
ros2 bag record \
    /clock /tf /tf_static \
    /joint_states /odom \
    /camera/image_raw /camera/depth/image_raw \
    /scan /imu/data \
    --output sim_demo \
    --storage mcap \
    --max-bag-duration 60

# 注意：必须设置 use_sim_time=true
ros2 param set /record use_sim_time true
```

### 8.3 `--headless-rendering` 模式

```bash
# headless 模式（无显示器 / CI 环境）
gz sim --headless-rendering -r world.sdf

# 或通过 xvfb（虚拟帧缓冲）
xvfb-run -a gz sim -r world.sdf
```

**关键注意事项**：
- `--headless-rendering` 需要 EGL 或 OSMesa 支持
- GPU 传感器（Camera / LiDAR）仍需 GPU（无 GUI 但需 GPU 渲染）
- CPU-only 传感器（IMU / FT / Contact）无此限制
- 容器环境需挂载 `/dev/dri`：`docker run --device /dev/dri`
