# 仿真中的 ros2_control

## Gazebo Harmonic + ros2_control

### URDF/xacro 条件切换

```xml
<!-- robot.urdf.xacro -->
<xacro:arg name="sim_mode" default="false"/>

<ros2_control name="ChassisSystem" type="system">
  <xacro:if value="$(arg sim_mode)">
    <hardware>
      <plugin>gz_ros2_control/GazeboSystem</plugin>
    </hardware>
  </xacro:if>
  <xacro:unless value="$(arg sim_mode)">
    <hardware>
      <plugin>chassis_protocol_ros2_control/ChassisHalInterface</plugin>
      <param name="port">/dev/ttyUSB0</param>
      <param name="baudrate">921600</param>
    </hardware>
  </xacro:unless>

  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### SDF 世界中加载 gz_ros2_control

```xml
<!-- world.sdf -->
<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(find my_robot_bringup)/config/controllers.yaml</parameters>
  <ros>
    <remapping>~/robot_description:=/robot_description</remapping>
  </ros>
</plugin>
```

### controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # 仿真中100Hz（实机1kHz）

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.5
    wheel_radius: 0.1
    use_stamped_vel: false

joint_state_broadcaster:
  ros__parameters:
    joints: ["left_wheel_joint", "right_wheel_joint"]
```

## Isaac Sim + ros2_control

```python
# Isaac Sim Python 扩展中配置 ros2_control
from omni.isaac.ros2_bridge import ROS2RobotController

controller = ROS2RobotController(
    robot_prim_path="/World/Robot",
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    control_mode="position",  # position / velocity / effort
    publish_rate=100.0
)
```

## 仿真时钟配置

```yaml
# launch 参数
use_sim_time: true

# node 配置
controller_manager:
  ros__parameters:
    use_sim_time: true

# /clock topic 由 gz-sim 或 Isaac Sim 发布
# max_step_size 需与 controller update_rate 对齐
# gz-sim: <max_step_size>0.001</max_step_size> → 1kHz 仿真步进
```

## 完整仿真 Launch 文件结构

```python
# sim_bringup.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_mode", default_value="true"),

        # 1. 启动 Gazebo Harmonic
        IncludeLaunchDescription("gz_sim.launch.py",
            launch_arguments={"world": "my_world.sdf"}.items()),

        # 2. robot_state_publisher（仿真模式 xacro）
        Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             parameters=[{"robot_description": robot_desc_sim,
                          "use_sim_time": True}]),

        # 3. ros2_control_node（使用仿真时钟）
        Node(package="controller_manager",
             executable="ros2_control_node",
             parameters=[controllers_yaml, {"use_sim_time": True}]),

        # 4. 生成控制器
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "diff_drive_controller"]),

        # 5. MoveIt 2（可选）
        IncludeLaunchDescription("move_group.launch.py",
            launch_arguments={"use_sim_time": "true"}.items()),
    ])
```
