# VLA 输出对接 MoveIt 2 — pose_goal / joint_goal / cartesian_path / Servo

---

## 一、VLA 输出格式回顾（Phase 3 成果）

Phase 3 分析了主流 VLA 的动作输出格式：

| VLA 模型 | 输出格式 | 维度 | 频率 |
|---------|---------|------|------|
| RT-2 / OpenVLA | EE Pose Delta（笛卡尔增量）| 7D（Δx,Δy,Δz,Δroll,Δpitch,Δyaw,gripper）| 50-100 Hz |
| π0 / RDT | Joint Delta（关节增量）or Absolute | 7D（每关节）| 50 Hz |
| ACT / Diffusion Policy | Joint Absolute（关节绝对）| 7D | 50 Hz |

---

## 二、四种对接模式

### 模式 A：Servo（实时增量，首选）

**适用**：EE Pose Delta 输出（RT-2/OpenVLA），要求低延迟连续控制

```
VLA → EE Pose Delta (7D at 50Hz)
       ↓ VlaServoBridge Node
       /servo/delta_twist_cmds (TwistStamped, ~50Hz)
       ↓ MoveIt Servo
       /joint_group_velocity_controller/commands
       ↓ JointGroupVelocityController
       → 硬件（控制周期 ~5ms）
```

```python
# vla_servo_bridge.py（关键逻辑）
from geometry_msgs.msg import TwistStamped
import numpy as np

class VlaServoBridge(Node):
    EE_DELTA_SCALE = 0.05   # 归一化增量 → m 或 rad 的缩放因子

    def on_vla_action(self, msg):
        action = np.array(msg.data[:6])      # [dx,dy,dz,droll,dpitch,dyaw]
        gripper = msg.data[6]

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'tool0'      # EE 坐标系（Servo 会做 TF 转换）
        twist.twist.linear.x  = action[0] * self.EE_DELTA_SCALE
        twist.twist.linear.y  = action[1] * self.EE_DELTA_SCALE
        twist.twist.linear.z  = action[2] * self.EE_DELTA_SCALE
        twist.twist.angular.x = action[3] * self.EE_DELTA_SCALE
        twist.twist.angular.y = action[4] * self.EE_DELTA_SCALE
        twist.twist.angular.z = action[5] * self.EE_DELTA_SCALE
        self.servo_pub.publish(twist)
        # 夹爪单独处理
        self.publish_gripper_command(gripper)
```

**优点**：延迟最低（< 20ms 端到端）、最适合 VLA 实时推理流  
**缺点**：无碰撞规划（仅运动学检查），需要 Servo 安全限制保护

### 模式 B：Joint Delta → JointJog（关节增量）

**适用**：Joint Delta 输出（π0/RDT），直接控制各关节速度

```python
from control_msgs.msg import JointJog

class VlaJointJogBridge(Node):
    JOINT_DELTA_SCALE = 0.1  # rad 缩放

    def on_vla_action(self, msg):
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        # π0 输出关节增量 → 转为速度（除以时间步长）
        dt = 0.02   # 50Hz → 20ms
        jog.velocities = [d * self.JOINT_DELTA_SCALE / dt
                          for d in msg.data[:6]]
        self.joint_jog_pub.publish(jog)
```

### 模式 C：pose_goal → MoveGroupInterface（离散目标）

**适用**：LLM Agent 发送高级"移到某位置"指令，对延迟不敏感

```python
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped

moveit = MoveItPy(node_name="vla_moveit_node")
arm = moveit.get_planning_component("arm")

def navigate_to_pose(target_pose: PoseStamped):
    """LLM 工具调用入口"""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=target_pose,
                       pose_link="tool0")
    plan_result = arm.plan()
    if plan_result:
        moveit.execute(plan_result.trajectory, blocking=True)
        return True
    return False
```

**适用场景**：
- LLM Agent 的"拾取"/"放置"工具调用
- VLA 推理每隔几秒才更新一次目标（非实时流）

### 模式 D：cartesian_path（笛卡尔直线路径）

**适用**：VLA 输出一系列末端位姿序列（非增量），需要走直线

```python
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose

def execute_cartesian_path(waypoints: list[Pose], eef_step=0.01):
    """
    沿笛卡尔直线通过一系列位姿点（用 Pilz LIN 或 compute_cartesian_path）
    eef_step: m，路径点间距（越小越精确但计算越慢）
    """
    arm.set_start_state_to_current_state()
    
    # 方法1：Pilz LIN（推荐，严格直线）
    for wp in waypoints:
        pose_target = PoseStamped()
        pose_target.header.frame_id = "world"
        pose_target.pose = wp
        arm.set_goal_state(pose_stamped_msg=pose_target)
        arm.set_planner_id("LIN")    # Pilz 直线运动
        result = arm.plan()
        if result:
            moveit.execute(result.trajectory, blocking=True)

    # 方法2：compute_cartesian_path（兼容性更好）
    fraction, trajectory = arm.compute_cartesian_path(
        waypoints=waypoints,
        eef_step=eef_step,
        jump_threshold=0.0
    )
    if fraction > 0.95:  # 至少 95% 路径可达
        moveit.execute(trajectory, blocking=True)
```

---

## 三、完整对接架构图

```
┌──────────────────────────────────────────────────────────────────────────┐
│  VLA 推理节点（50Hz）                                                     │
│  OpenVLA / π0 / RDT → action_output[7D]                                 │
└───────────────────────────┬──────────────────────────────────────────────┘
                            │ /vla/action_output (Float32MultiArray)
                            │
             ┌──────────────▼──────────────────┐
             │  VlaActionRouter（路由决策）       │
             │  根据当前任务阶段选择接口：         │
             │  ├─ 精细操作（Servo 模式）          │
             │  ├─ 关节控制（JointJog 模式）       │
             │  ├─ 粗略移动（MoveGroup 规划）      │
             │  └─ 抓取/放置（Pilz LIN 序列）      │
             └──────┬────────────┬───────────────┘
                    │            │
         ┌──────────▼──┐   ┌────▼────────────────┐
         │  MoveIt Servo│   │  MoveGroupInterface  │
         │  TwistStamped│   │  plan() + execute()  │
         └──────┬───────┘   └────────┬─────────────┘
                │                    │
         ┌──────▼────────────────────▼─────────────┐
         │  ros2_control                            │
         │  JointGroupVelocityController /          │
         │  JointTrajectoryController               │
         └─────────────────────────────────────────┘
```

---

## 四、模式选型矩阵

| 场景 | VLA 输出 | 推荐模式 | 延迟 | 安全性 |
|------|---------|---------|------|-------|
| VLA 连续精细抓取 | EE Delta 50Hz | **Servo（模式A）**| < 20ms | 依赖 Servo 安全限制 |
| VLA 关节控制 | Joint Delta 50Hz | **JointJog（模式B）**| < 20ms | 依赖关节限位 |
| LLM 工具调用（"去拾取"）| 目标 Pose | **pose_goal（模式C）**| 0.5-5s | 完整碰撞规划 |
| VLA 输出末端轨迹序列 | Pose 序列 | **cartesian_path（模式D）**| < 1s | Pilz 直线碰撞检查 |
| 组合场景（移近+精细）| 混合 | **C→A 切换** | N/A | 移近用规划，精细用Servo |

---

## 五、E-Stop 与安全层集成

```python
class VlaActionRouter(Node):
    def __init__(self):
        # ...
        # 订阅 E-Stop 信号
        self.estop_sub = self.create_subscription(
            Bool, '/e_stop', self.on_estop, qos_profile_sensor_data)
        self.is_stopped = False

    def on_estop(self, msg):
        if msg.data and not self.is_stopped:
            self.is_stopped = True
            # 立即停止 Servo
            self.get_client('/servo/stop_servo').call_async(Trigger.Request())
            # 发送零速度（双保险）
            self.servo_pub.publish(TwistStamped())
            RCLCPP_WARN(self.get_logger(), "E-Stop: VLA action halted")

    def on_vla_action(self, msg):
        if self.is_stopped:
            return  # 丢弃所有 VLA 命令
        # 正常路由...
```
