# 常用内置控制器源码导读

> 源码仓库：`ros2_controllers`（https://github.com/ros-controls/ros2_controllers）  
> 版本：`humble`  
> 本节重点剖析：JTC / DiffDrive / Mecanum / ForwardCommand，解析其内部实现逻辑以指导具身智能应用选型

---

## 一、控制器全景速查表

| 控制器 | 用途 | Command Interface | State Interface |
|--------|------|-------------------|----------------|
| `JointTrajectoryController` | 多关节轨迹追踪（操作臂必备） | position/velocity/effort | position/velocity |
| `ForwardCommandController` | 直通命令（透传） | any | — |
| `JointGroupPositionController` | 多关节位置直控 | position | — |
| `JointGroupVelocityController` | 多关节速度直控 | velocity | — |
| `DiffDriveController` | 差速底盘 | velocity（左右轮） | velocity（里程计） |
| `MecanumDriveController` | 全向底盘（麦克纳姆轮） | velocity（4 轮） | velocity |
| `AckermannSteeringController` | 阿克曼转向底盘 | velocity + steering | position + velocity |
| `PidController` | 单回路 PID（ChainableController） | — | — |
| `AdmittanceController` | 导纳控制（ChainableController） | ft_sensor state → position cmd | force/torque |
| `GripperActionController` | 夹爪 Action 接口 | position/effort | position/effort |
| `JointStateBroadcaster` | 发布 JointState topic（必装） | — | all state interfaces |
| `IMUSensorBroadcaster` | 发布 IMU topic | — | imu state interfaces |
| `ForceTorqueSensorBroadcaster` | 发布 Wrench topic | — | ft state interfaces |
| `RangeSensorBroadcaster` | 发布超声/测距 topic | — | distance state interfaces |

---

## 二、`JointTrajectoryController`（JTC）

### 2.1 定位

JTC 是具身智能**操作臂**的核心控制器，接收 `FollowJointTrajectory` Action，执行多关节时间参数化轨迹。

### 2.2 关键架构

```
FollowJointTrajectory Action Server
  → new trajectory 请求
  → 存入 RealtimeBuffer<trajectory_msgs::msg::JointTrajectory>

update() [实时]
  → 从 RealtimeBuffer 取当前轨迹
  → 时间插值：sample_trajectory(current_time)
      → cubic/quintic 样条 或 linear
  → PID 控制：
      error = goal_pos - current_pos
      command = kp*error + kd*(goal_vel - current_vel) + ki*∫error
  → 写入 command_interfaces_（position/velocity/effort 之一）
  → 检查是否超时/到达目标 → 发布 Action feedback/result
```

### 2.3 关键参数

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]   # 或 [velocity] / [effort] / [position, velocity]
    state_interfaces: [position, velocity]
    
    # 轨迹到达判断阈值
    constraints:
      stopped_velocity_tolerance: 0.01     # rad/s
      goal_time: 0.0                       # 0 = 不检查时间
      joint1:
        goal: 0.01                         # rad
        trajectory: 0.1
    
    # 插值方式
    interpolation_method: splines          # splines / none
    
    # 是否允许部分关节
    allow_partial_joints_goal: false
    
    # 安全：轨迹超时后的行为
    open_loop_control: false               # false = 闭环（读反馈）
    allow_integration_in_goal_trajectories: true
```

### 2.4 与 MoveIt 2 的接口

MoveIt 2 规划器输出 `JointTrajectory`，通过 `FollowJointTrajectory` Action 发给 JTC：

```
MoveIt 2 MoveGroup Node
  → 规划：OMPL/Pilz/STOMP → JointTrajectory（含时间戳 + 位置 + 速度 + 加速度）
  → 执行：ActionClient → /joint_trajectory_controller/follow_joint_trajectory
      → JTC 实时执行轨迹
```

---

## 三、`DiffDriveController`

### 3.1 定位

差速底盘控制器：接收 `geometry_msgs/Twist`（cmd_vel），控制左右轮速度，发布里程计。是**Nav2 与底盘驱动**的标准桥梁。

### 3.2 内部逻辑

```cpp
// 简化
void DiffDriveController::update(time, period) {
  // 读取 cmd_vel（通过 RealtimeBuffer，由 topic callback 写入）
  auto velocity_command = velocity_command_unstamped_ptr_->readFromRT();
  double linear  = velocity_command->linear.x;
  double angular = velocity_command->angular.z;

  // 正运动学逆解：Twist → 左右轮速度
  double vel_left  = (linear - angular * wheel_separation_ / 2.0) / wheel_radius_;
  double vel_right = (linear + angular * wheel_separation_ / 2.0) / wheel_radius_;

  // 限速
  vel_left  = clamp(vel_left,  -max_wheel_angular_velocity_, max_wheel_angular_velocity_);
  vel_right = clamp(vel_right, -max_wheel_angular_velocity_, max_wheel_angular_velocity_);

  // 写入 command interfaces（左右轮速度）
  registered_left_wheel_handles_[i].velocity.get().set_value(vel_left);
  registered_right_wheel_handles_[i].velocity.get().set_value(vel_right);

  // 里程计：前向运动学（读左右轮速度 state）
  double left_feedback  = registered_left_wheel_handles_[i].feedback.get().get_value();
  double right_feedback = registered_right_wheel_handles_[i].feedback.get().get_value();
  odometry_.update(left_feedback, right_feedback, period.seconds());

  // 发布 odom + TF（通过 RealtimePublisher）
  if (realtime_odom_pub_->trylock()) {
    auto & odom_msg = realtime_odom_pub_->msg_;
    odom_msg.pose.pose.position.x = odometry_.getX();
    odom_msg.pose.pose.position.y = odometry_.getY();
    // ...
    realtime_odom_pub_->unlockAndPublish();
  }
}
```

### 3.3 关键参数

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.4          # 左右轮间距（m）
    wheel_radius: 0.1              # 轮半径（m）
    
    # 速度限制
    linear.x.max_velocity: 1.5    # m/s
    linear.x.min_velocity: -1.5
    angular.z.max_velocity: 1.0   # rad/s
    
    # 里程计发布
    odom_frame_id: odom
    base_frame_id: base_link
    publish_rate: 50.0             # Hz
    
    # 速度指令超时（超时后停车）
    cmd_vel_timeout: 0.5           # s
```

### 3.4 与 chassis_protocol 的对比

| 特性 | `chassis_protocol::ChassisNode` | `DiffDriveController` |
|------|--------------------------------|----------------------|
| cmd_vel 接收 | `geometry_msgs/Twist` subscriber | `geometry_msgs/Twist` subscriber |
| 里程计发布 | 自定义 | `nav_msgs/Odometry` 标准 |
| TF 广播 | 手动 | 内置（odom → base_link） |
| 轮速限制 | 自定义 | 参数化配置 |
| 与 Nav2 兼容 | 需适配 | 开箱即用 |

---

## 四、`MecanumDriveController`

### 4.1 定位

全向底盘（麦克纳姆轮）控制器，与 DiffDriveController 类似，但支持 `Twist.linear.y`（横移）。

### 4.2 运动学

```
4 轮位置（前左 FL / 前右 FR / 后左 RL / 后右 RR）：
  vel_FL = (vx - vy - (lx + ly) * ωz) / wheel_radius
  vel_FR = (vx + vy + (lx + ly) * ωz) / wheel_radius
  vel_RL = (vx + vy - (lx + ly) * ωz) / wheel_radius
  vel_RR = (vx - vy + (lx + ly) * ωz) / wheel_radius

其中：lx = wheelbase/2，ly = track_width/2
```

配置示例：
```yaml
mecanum_drive_controller:
  ros__parameters:
    front_left_wheel_names: ["front_left_wheel_joint"]
    front_right_wheel_names: ["front_right_wheel_joint"]
    rear_left_wheel_names: ["rear_left_wheel_joint"]
    rear_right_wheel_names: ["rear_right_wheel_joint"]
    wheel_separation_x: 0.3     # wheelbase（前后）
    wheel_separation_y: 0.4     # track width（左右）
    wheel_radius: 0.1
```

---

## 五、`ForwardCommandController`

### 5.1 定位

最简单的控制器：**直接透传 Float64MultiArray topic 到 command interfaces**，无任何控制律。

### 5.2 用途

- 开发调试（手动发命令测试硬件）
- VLA / 外部算法直接输出关节 command（绕过 JTC）
- 用于 ChainableController 的末端（接受上游控制器的 reference interfaces）

```yaml
forward_command_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3]
    interface_name: position
# 订阅 /forward_command_controller/commands (std_msgs/Float64MultiArray)
# 直接写入 joint1/position, joint2/position, joint3/position
```

---

## 六、`JointStateBroadcaster`（必装）

这是一个**不发 command 的特殊控制器**，只读取所有 state interfaces 并发布 `sensor_msgs/JointState`：

```yaml
joint_state_broadcaster:
  ros__parameters:
    # 默认发布所有关节
    joints: []        # 为空时自动发布所有关节
    interfaces: []    # 为空时发布 position/velocity/effort
    publish_rate: 50.0
```

发布到 `/joint_states` topic（`robot_state_publisher` 和 MoveIt 2 的必要输入）。

**在具身智能整机中，`JointStateBroadcaster` 是必须运行的后台控制器之一。**

---

## 七、`AdmittanceController`（ChainableController，力控核心）

### 7.1 原理

导纳控制（Admittance Control）：
```
外力 F_ext（来自力矩传感器）
  → 虚拟弹簧阻尼模型：M*ẍ + D*ẋ + K*(x - x_eq) = F_ext
  → 输出：修正后的末端位置增量 Δx
  → 发布到 reference_interfaces_（供 JTC 使用）
```

适用场景：
- 接触操作（拧螺丝、打磨、装配）
- 人机协作（安全性）
- 柔顺运动（与环境交互不碰撞）

### 7.2 配置

```yaml
admittance_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    ft_sensor:
      name: ft_sensor
      frame:
        id: tool0
    
    # 导纳参数（惯性/阻尼/刚度）
    admittance:
      mass: [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]      # [Fx Fy Fz Tx Ty Tz]
      damping: [20.0, 20.0, 20.0, 1.0, 1.0, 1.0]
      stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 0 = 纯阻尼
```
