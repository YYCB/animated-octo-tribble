# MoveIt Servo — 实时增量控制

---

## 一、MoveIt Servo 的角色

MoveIt Servo 是 MoveIt 2 的**实时增量控制子系统**，允许以高频（默认 100-200 Hz）连续发送速度/增量命令给机械臂，**绕过离散的运动规划**：

```
传统 MoveIt 规划执行流程（离散，延迟高）：
  目标 pose → OMPL 规划（100ms-5s）→ 时间参数化 → FollowJointTrajectory → 硬件

MoveIt Servo 流程（实时，延迟低）：
  速度/增量命令 → Servo（1-10ms）→ JointGroupVelocityController → 硬件
```

**核心用途**：
- VLA 输出的末端速度流（EE Pose Delta at 50-100 Hz）
- 手柄 Jog（操作员实时微调）
- 视觉伺服（闭环目标跟踪）
- 遥操作

---

## 二、Servo 架构与输入类型

```
输入接口（任选其一）：
  /servo/delta_twist_cmds (geometry_msgs/TwistStamped)
    → 末端执行器笛卡尔速度增量（6D：vx,vy,vz + ωx,ωy,ωz）

  /servo/joint_jog (control_msgs/JointJog)
    → 各关节速度直接指定

内部处理管线（每个控制周期）：
  1. 接收输入命令
  2. 碰撞检测（可选，基于 planning scene）
  3. 奇异点检测与速度缩减
  4. 关节限位检测（接近限位时降速）
  5. IK 速度求解（Jacobian 伪逆）
  6. 输出 → ros2_control

输出接口（二选一）：
  → trajectory_msgs/JointTrajectory（发给 JointTrajectoryController）
  → std_msgs/Float64MultiArray（发给 JointGroupVelocityController，推荐）
```

---

## 三、Servo 配置详解

```yaml
# moveit_servo_params.yaml
###############################################
# 话题配置
###############################################
move_group_name: arm                        # 控制的关节组
ee_frame_name: tool0                        # 末端执行器 frame（TF 中必须存在）
robot_link_command_frame: base_link         # 命令参考坐标系

# 输入话题
cartesian_command_in_topic: ~/delta_twist_cmds   # TwistStamped 输入
joint_command_in_topic: ~/joint_jog              # JointJog 输入

# 输出控制器话题
command_out_topic: /joint_group_velocity_controller/commands
command_out_type: std_msgs/msg/Float64MultiArray  # 速度模式

###############################################
# 控制频率与延迟
###############################################
publish_period: 0.034        # s，控制周期（~30 Hz，建议与控制器频率匹配）
incoming_command_timeout: 0.1  # s，命令超时（超时后停止运动）

###############################################
# 安全限制
###############################################
# 关节限位减速（接近软限位时降速）
joint_limit_margin: 0.1      # rad，提前减速的裕量

# 碰撞检测
check_collisions: true
collision_check_rate: 10.0   # Hz（低于 publish_period，避免阻塞）
self_collision_proximity_threshold: 0.01  # m，自碰撞警告距离
scene_collision_proximity_threshold: 0.02 # m，环境碰撞警告距离
collision_distance_safety_factor: 1000.0  # 碰撞减速因子

# 奇异点处理（见下一节）
lower_singularity_threshold: 17.0    # 开始减速的奇异值阈值
hard_stop_singularity_threshold: 5.0 # 强制停止的奇异值阈值
leaving_singularity_threshold_multiplier: 2.0  # 离开奇异点时的宽松因子

###############################################
# 速度缩放
###############################################
scale:
  cartesian: 0.1    # 笛卡尔速度缩放（0.1 = 最大速度的 10%）
  joint: 0.1        # 关节速度缩放

###############################################
# 状态监控
###############################################
status_topic: ~/status        # ServoStatus 消息
use_smoothing: true           # 启用速度平滑（减少抖动）
smoothing_filter_plugin_name: online_signal_smoothing::ButterworthFilterPlugin
```

---

## 四、奇异点处理机制

奇异点是机械臂失去自由度的构型（Jacobian 矩阵降秩），Servo 有专门的处理逻辑：

### 4.1 奇异值检测

```
通过 Jacobian 矩阵的最小奇异值（SVD 分解）衡量距奇异点的距离：

σ_min > lower_singularity_threshold（如 17）：正常运动
σ_min 在 [hard_stop, lower]（如 [5, 17]）：速度线性衰减
σ_min < hard_stop_singularity_threshold（如 5）：完全停止
```

### 4.2 典型奇异构型

- **Shoulder Singularity**（肩部）：关节 2 伸直（θ2 = 0）
- **Elbow Singularity**（肘部）：关节 4 伸直（θ4 = 0 或 π）
- **Wrist Singularity**（腕部）：关节 4 和 6 共线（θ5 = 0）

### 4.3 奇异点处理策略（代码层）

```cpp
// MoveIt Servo 的奇异点处理（源码参考）
if (servo_status == StatusCode::DECELERATE_FOR_SINGULARITY) {
    // 速度缩减：离奇异点越近，速度越慢
    velocity_scale = std::min(1.0,
        (smallest_sv - hard_stop_threshold) /
        (lower_threshold - hard_stop_threshold));
    delta_theta *= velocity_scale;
} else if (servo_status == StatusCode::HALT_FOR_SINGULARITY) {
    // 强制停止
    delta_theta.setZero();
}
```

---

## 五、Servo 使能 / 停止管理（Service）

```bash
# 使能 Servo（开始接收命令）
ros2 service call /servo/start_servo std_srvs/srv/Trigger

# 停止 Servo（停止运动，切回规划模式）
ros2 service call /servo/stop_servo std_srvs/srv/Trigger

# 暂停（不停止控制器，只暂停命令处理）
ros2 service call /servo/pause_servo std_srvs/srv/Trigger
```

**注意**：Servo 和 MoveGroup 规划模式**不能同时激活**。需要先 stop_servo 再调用 MoveGroup。

---

## 六、ros2_control 控制器配置（配合 Servo）

MoveIt Servo 输出关节速度，需要对应的 ros2_control 控制器：

```yaml
# ros2_controllers.yaml（Servo 专用）
joint_group_velocity_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

# moveit_controllers.yaml（MoveGroup 规划执行）
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller    # 规划执行用
  joint_trajectory_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
```

**控制器切换**（Servo ↔ 规划模式）：

```python
# 激活 Servo：切换到速度控制器
switcher = ControllerSwitcher()
switcher.switch(
    stop_controllers=["joint_trajectory_controller"],
    start_controllers=["joint_group_velocity_controller"],
    strictness=SwitchController.Request.BEST_EFFORT
)
ros2_service_call('/servo/start_servo', Trigger)

# 回到规划模式：切换回轨迹控制器
ros2_service_call('/servo/stop_servo', Trigger)
switcher.switch(
    stop_controllers=["joint_group_velocity_controller"],
    start_controllers=["joint_trajectory_controller"]
)
```

---

## 七、Servo 与 VLA 对接（快速参考）

VLA 输出末端速度流 → MoveIt Servo 实时执行：

```python
# VlaServoNode（简化版）
import rclpy
from geometry_msgs.msg import TwistStamped

class VlaServoNode(Node):
    def __init__(self):
        super().__init__('vla_servo_node')
        self.servo_pub = self.create_publisher(
            TwistStamped, '/servo/delta_twist_cmds', 10)
        # VLA 推理结果订阅
        self.vla_sub = self.create_subscription(
            Float32MultiArray, '/vla/action_output',
            self.on_vla_action, 10)

    def on_vla_action(self, msg):
        # VLA 输出 [dx, dy, dz, droll, dpitch, dyaw, gripper]（7D）
        action = msg.data
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'tool0'    # EE 坐标系
        twist.twist.linear.x = action[0]
        twist.twist.linear.y = action[1]
        twist.twist.linear.z = action[2]
        twist.twist.angular.x = action[3]
        twist.twist.angular.y = action[4]
        twist.twist.angular.z = action[5]
        self.servo_pub.publish(twist)
```

详细对接设计见 `06_vla_integration.md`。
