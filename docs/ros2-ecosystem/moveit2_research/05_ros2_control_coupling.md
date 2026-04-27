# MoveIt 2 与 ros2_control 耦合点 — FollowJointTrajectory + 控制器切换

---

## 一、耦合关系概览

MoveIt 2 与 ros2_control 的耦合通过**两个层面**实现：

```
层面 1：轨迹执行（规划模式）
  MoveIt Trajectory Execution Manager
  → FollowJointTrajectory Action
  → JointTrajectoryController（ros2_control）
  → Hardware Interface

层面 2：实时速度控制（Servo 模式）
  MoveIt Servo
  → /joint_group_velocity_controller/commands（Float64MultiArray）
  → JointGroupVelocityController（ros2_control）
  → Hardware Interface
```

---

## 二、FollowJointTrajectory Action 详解

### 2.1 消息结构

```
action: control_msgs/action/FollowJointTrajectory

Goal：
  trajectory_msgs/JointTrajectory trajectory
    ├── string[] joint_names             # 关节名列表
    └── JointTrajectoryPoint[] points    # 轨迹点序列
        ├── float64[] positions          # 关节角度（rad）
        ├── float64[] velocities         # 关节速度（rad/s）
        ├── float64[] accelerations      # 关节加速度
        └── builtin_interfaces/Duration time_from_start

  goal_time_tolerance                    # 到达目标时间容忍
  path_tolerance []                      # 路径跟踪容忍度（per 关节）
  goal_tolerance []                      # 目标容忍度（per 关节）

Feedback：
  float64 desired / actual / error      # per 关节

Result：
  int32 error_code
  SUCCESSFUL = 0
  INVALID_GOAL = -1
  INVALID_JOINTS = -2
  PATH_TOLERANCE_VIOLATED = -4
  GOAL_TOLERANCE_VIOLATED = -5
```

### 2.2 MoveIt 侧配置（moveit_controllers.yaml）

```yaml
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller
    - gripper_controller        # 可选：夹爪独立控制器

  joint_trajectory_controller:
    type: FollowJointTrajectory   # 或 GripperCommand（夹爪）
    action_ns: follow_joint_trajectory
    default: true                 # 默认使用此控制器
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

  gripper_controller:
    type: GripperCommand
    action_ns: gripper_action
    default: true
    joints:
      - finger_joint
```

### 2.3 ros2_control 侧配置（ros2_controllers.yaml）

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position              # 位置控制
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false    # 不允许部分关节目标
    open_loop_control: false            # 闭环（需要 state_interfaces）
    allow_integration_in_goal_trajectories: false  # 不允许纯速度轨迹积分
    # 插补配置
    interpolation_method: splines       # none / splines
    # 容忍度（覆盖 MoveIt 发送的值）
    stopped_velocity_tolerance: 0.01   # rad/s
    goal_time: 0.0                     # s（0=使用轨迹中的时间戳）
```

---

## 三、轨迹执行管理器（Trajectory Execution Manager）

MoveIt 2 的 `TrajectoryExecutionManager` 负责：
1. 选择正确的控制器（根据关节组）
2. 分割轨迹（多关节组时分别发送）
3. 监控执行进度和错误
4. 处理执行取消/抢占

### 3.1 执行监控配置

```yaml
# move_group 参数
trajectory_execution:
  allowed_execution_duration_scaling: 1.2   # 允许执行时间超出规划时间 20%
  allowed_goal_duration_margin: 0.5         # s，目标到达时间裕量
  allowed_start_tolerance: 0.01            # rad，执行前检查起始状态容忍度
  execution_duration_monitoring: true       # 开启执行时间监控
  wait_for_trajectory_completion: true      # 等待轨迹完成后再规划下一个
```

### 3.2 执行错误处理

```cpp
// C++ 执行结果处理
auto result = move_group.execute(trajectory);
switch (result.val) {
  case moveit::core::MoveItErrorCode::SUCCESS:
    RCLCPP_INFO(logger, "Execution succeeded");
    break;
  case moveit::core::MoveItErrorCode::CONTROL_FAILED:
    // JointTrajectoryController 报告跟踪误差超限
    RCLCPP_ERROR(logger, "Control failed - check path/goal tolerances");
    break;
  case moveit::core::MoveItErrorCode::TIMED_OUT:
    RCLCPP_ERROR(logger, "Execution timed out");
    break;
  case moveit::core::MoveItErrorCode::PREEMPTED:
    RCLCPP_WARN(logger, "Execution preempted by new goal");
    break;
}
```

---

## 四、控制器切换（规划模式 ↔ Servo 模式）

### 4.1 为什么需要切换

JointTrajectoryController 和 JointGroupVelocityController **不能同时激活**，因为它们都向同一组关节写入命令（CommandInterface 只允许一个写者）。

### 4.2 自动切换（推荐）

MoveIt Servo 支持在启动/停止时自动切换控制器：

```yaml
# moveit_servo_params.yaml
switch_controller: true                    # 启用自动切换
control_output_type: velocity              # 速度模式
command_out_topic: /joint_group_velocity_controller/commands

# 当 start_servo service 被调用时：
# stop:  ["joint_trajectory_controller"]
# start: ["joint_group_velocity_controller"]

# 当 stop_servo service 被调用时：
# stop:  ["joint_group_velocity_controller"]
# start: ["joint_trajectory_controller"]
```

### 4.3 手动切换（ROS 2 Service）

```bash
# 停止轨迹控制器，启动速度控制器
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['joint_group_velocity_controller'],
    deactivate_controllers: ['joint_trajectory_controller'],
    strictness: 2}"

# 切换回轨迹控制器
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['joint_trajectory_controller'],
    deactivate_controllers: ['joint_group_velocity_controller'],
    strictness: 2}"
```

### 4.4 切换时序安全

```
正确的切换时序：
  1. 发送停止命令给当前控制器（让速度归零）
  2. 等待机器人完全静止（vel < threshold）
  3. 调用 switch_controller
  4. 启动新控制器

危险操作：
  ❌ 在机器人高速运动中切换控制器（可能导致跳变）
  ❌ 两个控制器同时激活（ros2_control 会报错）
```

---

## 五、时间参数化（轨迹速度/加速度）

MoveIt 规划出的轨迹**只有位置序列**，没有时间戳。Trajectory Execution 前必须进行时间参数化：

### 5.1 内置方法

```python
# moveit_py
arm = moveit.get_planning_component("arm")
plan_result = arm.plan()

# 方法1：TimeOptimalTrajectoryGeneration（TOTG，推荐）
plan_result.trajectory.apply_totg_time_parameterization(
    velocity_scaling_factor=1.0,      # 1.0 = 使用 joint_limits.yaml 中的最大速度
    acceleration_scaling_factor=1.0
)

# 方法2：Ruckig（Jerk-limited，最平滑）
plan_result.trajectory.apply_ruckig_smoothing(
    velocity_scaling_factor=0.8,
    acceleration_scaling_factor=0.8
)
```

### 5.2 通过 MoveGroupInterface（C++）

```cpp
move_group.setMaxVelocityScalingFactor(0.8);      # 80% 最大速度
move_group.setMaxAccelerationScalingFactor(0.5);  # 50% 最大加速度
move_group.move();  # 自动应用时间参数化后执行
```

---

## 六、chassis_protocol 集成检查表

| 项目 | 说明 | 状态 |
|------|------|:----:|
| 关节名（URDF）与 ros2_controllers.yaml 中的名称一致 | 必须完全匹配 | 需验证 |
| JointTrajectoryController 声明 position CommandInterface | MoveIt 默认用位置控制 | 需验证 |
| JointTrajectoryController 声明 position + velocity StateInterface | 闭环控制必须 | 需验证 |
| FollowJointTrajectory action server 正常响应 | `ros2 action list` 可见 | 需测试 |
| 轨迹跟踪容忍度匹配硬件能力 | 过严格会频繁超限报错 | 需调参 |
| 控制器频率（update_rate）≥ MoveIt 期望频率（250 Hz）| 低频率会导致轨迹抖动 | 需验证 |
