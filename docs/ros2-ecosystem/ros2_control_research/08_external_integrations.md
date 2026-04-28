# ros2_control 外部集成：MoveIt 2 / gz_ros2_control / isaac_ros2_control

---

## 一、与 MoveIt 2 的集成

### 1.1 集成拓扑

```
MoveIt 2 MoveGroup Node
  ├── PlanningScene（碰撞检测）
  │     ← /joint_states（来自 JointStateBroadcaster）
  │     ← /tf（来自 robot_state_publisher）
  ├── Planner（OMPL / Pilz / STOMP）
  │     → JointTrajectory（规划结果）
  └── TrajectoryExecutionManager
        ↓ FollowJointTrajectory Action
        ↓ /joint_trajectory_controller/follow_joint_trajectory
  ros2_control JointTrajectoryController
        ↓ command_interfaces（关节位置/速度/力矩）
  hardware_interface（实际驱动器）
```

### 1.2 MoveIt 2 所需的 ros2_control 接口

必须运行的控制器（MoveIt 2 启动前提）：

| 控制器 | 作用 |
|--------|------|
| `joint_state_broadcaster` | 发布 `/joint_states`，MoveIt 2 用于更新规划场景 |
| `joint_trajectory_controller` | 接收 `FollowJointTrajectory` Action |

### 1.3 MoveIt 2 配置（`moveit_controllers.yaml`）

```yaml
# config/moveit_controllers.yaml
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller

joint_trajectory_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory  # → /joint_trajectory_controller/follow_joint_trajectory
  default: true
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
```

### 1.4 MoveIt Servo（实时增量控制）

MoveIt Servo 可以绕过规划器，直接接受增量速度命令（用于遥操作、VLA 实时控制）：

```
外部输入（手柄/VLA 输出）
  → /servo_server/delta_twist_cmds（geometry_msgs/TwistStamped）
      或 /servo_server/delta_joint_cmds（control_msgs/JointJog）
  → MoveIt Servo Server
      → 实时 IK + 奇异点规避 + 关节限位检查
      → /joint_trajectory_controller/joint_trajectory（Topic 模式）
  → JTC（低延迟 topic 接收模式，非 Action）
```

Servo 的 ros2_control 后端配置：
```yaml
# servo 使用 JTC 的 topic 接口
command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /joint_trajectory_controller/joint_trajectory
```

---

## 二、与 Gazebo Harmonic 的集成（`gz_ros2_control`）

### 2.1 架构

```
Gazebo Harmonic（物理引擎）
  ├── 关节物理仿真（位置/速度/力矩）
  └── GazeboSimSystem（hardware_interface 插件）
        ↓ 实现 SystemInterface::read()/write()
  ros2_control ResourceManager
  ↕ 同真实硬件完全相同的 state/command interfaces
  ControllerManager + 控制器
```

`gz_ros2_control` 是一个 **Gazebo 系统插件**，把 Gazebo 的关节仿真器包装成 `SystemInterface`：

### 2.2 URDF 配置

```xml
<!-- 真实硬件 -->
<hardware>
  <plugin>my_robot_hardware/MyRobotSystem</plugin>
</hardware>

<!-- 切换为 Gazebo 仿真 -->
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  <!-- 无需其他参数，自动从 URDF 几何/惯量信息初始化 -->
</hardware>
```

或用 Xacro 参数化切换：
```xml
<xacro:if value="${use_sim}">
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</xacro:if>
<xacro:unless value="${use_sim}">
  <plugin>my_robot_hardware/MyRobotSystem</plugin>
</xacro:unless>
```

### 2.3 Gazebo Launch 配置

```python
# launch/sim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_ros2_control_demos_path, '/gz_sim.launch.py']),
            launch_arguments={'gz_args': 'empty.sdf -r'}.items()
        ),
        # 生成机器人到 Gazebo
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'my_robot', '-topic', 'robot_description']
        ),
        # bridge：Gazebo 时钟 → ROS 2
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        ),
        # controller_manager（与真实硬件相同）
        Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[robot_description, controller_params]
        ),
    ])
```

### 2.4 `GazeboSimSystem::read()` / `write()` 实现原理

```cpp
// gz_ros2_control/src/gz_ros2_control_plugin.cpp
return_type GazeboSimSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    // 从 Gazebo 关节仿真器读取当前状态
    position_state_[i] = sim_joints_[i].Position(0);  // 弧度
    velocity_state_[i] = sim_joints_[i].Velocity(0);  // rad/s
    effort_state_[i]   = sim_joints_[i].Force(0);     // N·m
  }
  return return_type::OK;
}

return_type GazeboSimSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    // 根据 command interface 类型（position/velocity/effort）施加到 Gazebo 关节
    if (joint_control_method_[i] == POSITION) {
      sim_joints_[i].SetPosition(0, position_command_[i], true);
    } else if (joint_control_method_[i] == VELOCITY) {
      sim_joints_[i].SetVelocity(0, velocity_command_[i]);
    } else if (joint_control_method_[i] == EFFORT) {
      sim_joints_[i].SetForce(0, effort_command_[i]);
    }
  }
  return return_type::OK;
}
```

---

## 三、与 Isaac Sim 的集成（`isaac_ros2_control`）

### 3.1 架构

```
NVIDIA Isaac Sim（OmniVerse 物理引擎）
  ├── ArticulationView（USD 关节驱动）
  └── IsaacSystem（hardware_interface 插件）
        ↓ 通过 Isaac Sim Python Bindings 访问 ArticulationView
  ros2_control（与真实/Gazebo 完全相同）
```

### 3.2 URDF 配置

```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <!-- 或 isaac_ros2_control/IsaacSystem（官方 Isaac ROS 组件） -->
  <param name="joint_commands_topic">/isaac/joint_commands</param>
  <param name="joint_states_topic">/isaac/joint_states</param>
</hardware>
```

Isaac Sim 最常见的桥接方式是 **topic-based**：Isaac 发布关节状态到 topic，订阅控制命令 topic，`TopicBasedSystem` 作为 hardware_interface 的桥接层。

### 3.3 `TopicBasedSystem`（通用仿真桥接）

`topic_based_ros2_control` 是一个通用方案，不绑定特定仿真器：

```yaml
# URDF param
<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
<param name="joint_commands_topic">/my_sim/joint_commands</param>
<param name="joint_states_topic">/my_sim/joint_states</param>
<param name="trigger_joint_command_threshold">0.001</param>
```

工作原理：
- `read()`：订阅 `sensor_msgs/JointState` topic，更新 state_interfaces
- `write()`：发布 `sensor_msgs/JointState`（作为 command），仿真器订阅执行

---

## 四、`mock_components/GenericSystem`（无硬件开发）

### 4.1 用途

开发和测试阶段，不接真实硬件时使用：

```xml
<hardware>
  <plugin>mock_components/GenericSystem</plugin>
  <param name="fake_sensor_commands">false</param>
  <param name="state_following_offset">0.0</param>
  <!-- 
    fake_sensor_commands=true：允许外部通过 topic 注入传感器值（测试用）
    state_following_offset：在命令值基础上加偏差（模拟摩擦等）
  -->
</hardware>
```

### 4.2 行为

```cpp
// GenericSystem::read()
for (size_t i = 0; i < state_interfaces_.size(); ++i) {
  // 直接把对应的 command_interface 值复制给 state_interface
  // 模拟"完美驱动器"（命令立即到达）
  state_interfaces_[i].set_value(
    command_interfaces_[i].get_value() + state_following_offset_);
}
```

---

## 五、Sim2Real 切换工作流

这是具身智能开发的**标准流程**，完全通过 Xacro 参数化实现：

```
阶段 1：控制器逻辑开发
  → use_mock_hardware=true（GenericSystem）
  → 本地桌面运行，无需任何硬件

阶段 2：仿真验证
  → use_sim=true（GazeboSimSystem 或 IsaacSystem）
  → 物理真实性更高，可做 domain randomization

阶段 3：真机部署
  → use_mock_hardware=false, use_sim=false（真实 hardware_interface）
  → 控制器代码和配置文件**完全不变**
```

切换命令：
```bash
# mock 模式
ros2 launch my_robot robot.launch.py use_mock_hardware:=true

# Gazebo 模式
ros2 launch my_robot sim.launch.py use_sim:=true

# 真机模式
ros2 launch my_robot robot.launch.py use_mock_hardware:=false
```
