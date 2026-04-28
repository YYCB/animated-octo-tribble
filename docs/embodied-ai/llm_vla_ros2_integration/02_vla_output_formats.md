# VLA 输出格式对比与 ROS 2 转换层设计

> 覆盖模型：OpenVLA、RT-2、π0（pi-zero）、RDT-1B  
> 关联：`isaac_ros_nitros_research/05_managed_pub_sub.md`（NitrosTensorList，VLA 输入/输出 tensor）

---

## 一、主流 VLA 模型输出格式对比

### 1.1 汇总表

| 模型 | 来源 | 推理频率 | 动作空间 | 输出维度 | 坐标系 |
|------|------|----------|---------|---------|--------|
| **RT-2** | Google DeepMind 2023 | ~1-2 Hz | 末端位姿增量 + 夹爪 | 8D（ΔPos 3 + ΔRot 3 + gripper 1 + terminate 1）| 工具坐标系 |
| **OpenVLA** | Stanford / Berkeley 2024 | ~6 Hz | 末端位姿增量 + 夹爪 | 7D（同 RT-2，无 terminate）| 工具坐标系 |
| **π0 (pi-zero)** | Physical Intelligence 2024 | 10-50 Hz | 关节空间增量 | 7D（6DOF 臂 + 夹爪）| 关节空间 |
| **RDT-1B** | 清华 2024 | 25 Hz | 关节空间绝对值 | 128D（多模态，可配置）| 关节空间 |
| **Octo** | Berkeley 2024 | ~5-10 Hz | 末端位姿增量 | 7D（+ chunk 预测，多步）| 工具坐标系 |
| **OpenPI (LeRobot)** | Hugging Face 2024 | 25-50 Hz | 关节空间 | 视机器人配置 | 关节空间 |

### 1.2 动作空间语义

#### 末端位姿增量（EE Pose Delta）— RT-2 / OpenVLA 风格

```
输出：[Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper_open]
单位：通常已归一化到 [-1, 1] 或 [-0.05m, 0.05m] per step
需要：IK 求解（MoveIt kinematics）将 ΔEE Pose → ΔJoint

优点：模型更容易泛化（与机器人构型无关）
缺点：IK 有奇异点问题；需要正确的工具坐标系定义
```

#### 关节空间增量（Joint Delta）— π0 风格

```
输出：[Δq₁, Δq₂, ..., Δq₇]  单位：rad per step
需要：直接累加到当前关节角
      q_target = q_current + Δq * dt_scale

优点：无需 IK；关节限位检查简单
缺点：与机器人构型强绑定；泛化性差
```

#### 关节空间绝对值（Joint Absolute）— RDT 风格

```
输出：[q₁, q₂, ..., q_n]  单位：rad（绝对值）
需要：作为 JointTrajectory 的目标点

优点：不累积误差
缺点：首步可能有大位移（需要平滑插值）
```

---

## 二、动作 Token 的量化与反量化

OpenVLA / RT-2 等基于语言模型的 VLA 将连续动作离散化为"动作 token"：

```python
# OpenVLA 动作 token 的处理（简化）
# 训练时：连续动作 → token
action_bins = 256  # 动作离散化精度
action_min, action_max = -1.0, 1.0  # 归一化范围

def action_to_token(action: float) -> int:
    """连续动作 → 离散 token（0-255）"""
    normalized = (action - action_min) / (action_max - action_min)
    return int(normalized * (action_bins - 1))

def token_to_action(token: int) -> float:
    """离散 token → 连续动作"""
    normalized = token / (action_bins - 1)
    return normalized * (action_max - action_min) + action_min

# 推理输出：7 个 token（vocab 中的特殊 action token）
# 例如：[152, 89, 201, 143, 167, 98, 220]
# 反量化：[0.19m/s, -0.30m/s, 0.58m/s, 0.12rad/s, ...]
```

---

## 三、ROS 2 动作桥接层（Action Bridge）设计

### 3.1 核心职责

动作桥接节点（`VlaActionBridgeNode`）负责：
1. 接收 VLA 推理输出（`NitrosTensorList` 或 `Float32MultiArray`）
2. 反量化 / 解析动作维度
3. 坐标系变换（若需要）
4. **转换为目标控制器格式**（见 3.2）
5. 安全检查（关节限位、速度限制）
6. 发布到 ros2_control

### 3.2 输出格式适配矩阵

| VLA 输出格式 | 目标控制器 | 转换方法 |
|------------|---------|---------|
| ΔEE Pose | JTC（FollowJointTrajectory）| KDL/TracIK IK → 关节轨迹 |
| ΔEE Pose | MoveIt Servo（CartesianJog）| 直接发 TwistStamped |
| ΔJoint | JTC | 累加 + 插值 → JointTrajectory |
| ΔJoint | JointGroupVelocity | 换算为速度命令 |
| Joint Absolute | JTC | 单点轨迹（需平滑） |

### 3.3 `VlaActionBridgeNode` 参考实现

```cpp
// vla_action_bridge/vla_action_bridge_node.hpp
class VlaActionBridgeNode : public rclcpp::Node {
public:
  VlaActionBridgeNode(const rclcpp::NodeOptions & options)
  : Node("vla_action_bridge", options)
  {
    // 参数
    action_space_ = declare_parameter<std::string>("action_space", "ee_pose_delta");
    action_dim_   = declare_parameter<int>("action_dim", 7);
    control_freq_ = declare_parameter<double>("control_frequency", 10.0);
    max_joint_vel_= declare_parameter<std::vector<double>>("max_joint_velocities", {});

    // 订阅 VLA 输出（NITROS 路径：NitrosTensorList；CPU 路径：Float32MultiArray）
    vla_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "vla/action_output", rclcpp::SensorDataQoS(),
      std::bind(&VlaActionBridgeNode::onVlaOutput, this, _1));

    // 当前关节状态（用于增量动作）
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&VlaActionBridgeNode::onJointState, this, _1));

    // 输出：发给 MoveIt Servo（CartesianJog）
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

    // 控制频率保证
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_freq_),
      std::bind(&VlaActionBridgeNode::controlLoop, this));
  }

private:
  void onVlaOutput(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(action_mutex_);
    latest_action_ = msg->data;
    action_received_ = true;
  }

  void controlLoop() {
    std::lock_guard<std::mutex> lock(action_mutex_);
    if (!action_received_) return;  // 无新动作，保持静止

    if (action_space_ == "ee_pose_delta") {
      publishEEDelta(latest_action_);
    } else if (action_space_ == "joint_delta") {
      publishJointDelta(latest_action_);
    }
    action_received_ = false;  // 消费一次
  }

  void publishEEDelta(const std::vector<float> & action) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = now();
    twist.header.frame_id = "tool0";
    twist.twist.linear.x  = applyScale(action[0]);  // Δx → vx
    twist.twist.linear.y  = applyScale(action[1]);  // Δy → vy
    twist.twist.linear.z  = applyScale(action[2]);  // Δz → vz
    twist.twist.angular.x = applyScale(action[3]);
    twist.twist.angular.y = applyScale(action[4]);
    twist.twist.angular.z = applyScale(action[5]);
    // action[6] = gripper → 另一个 publisher
    twist_pub_->publish(twist);
  }

  // 成员变量
  std::string action_space_;
  int action_dim_;
  double control_freq_;
  std::vector<float> latest_action_;
  bool action_received_{false};
  std::mutex action_mutex_;
  sensor_msgs::msg::JointState current_joint_state_;
  // publishers / subscribers / timer...
};
```

### 3.4 MoveIt Servo 作为 VLA 末端执行器（推荐方案）

MoveIt Servo 专为**实时增量笛卡尔控制**设计，天然适合 VLA EE Pose Delta 输出：

```
VLA 输出 ΔEE Pose（7D）
  ↓ VlaActionBridgeNode
  ↓ geometry_msgs::TwistStamped（工具坐标系速度）
  ↓ /servo_node/delta_twist_cmds topic
MoveIt Servo（125 Hz 内部循环）
  ↓ 实时 IK 求解（KDL / TracIK）
  ↓ 奇异点检测 + 关节限位滤波
  ↓ JointTrajectory（单步，短时间戳）
JTC（ros2_control）
  ↓ CommandInterface
机械臂
```

**MoveIt Servo 的关键优势**：
- 内置奇异点回避（减小靠近奇异点时的速度）
- 内置关节限位保护
- 支持 Cartesian 和 Joint 两种输入
- 可以在不停止的情况下切换输入来源（VLA → 人工操控）

---

## 四、夹爪控制的 ROS 2 接口

VLA 输出的最后一维通常是夹爪开合度（0=关闭，1=打开）：

```cpp
// 夹爪控制（发给 GripperActionController）
auto gripper_goal = control_msgs::action::GripperCommand::Goal{};
gripper_goal.command.position = action[6] * gripper_max_open_;  // 归一化 → 实际开合度（m）
gripper_goal.command.max_effort = 10.0;  // N
gripper_action_client_->async_send_goal(gripper_goal);
```

---

## 五、π0 / RDT 的高频关节空间控制

π0 和 RDT 以更高频率（25-50 Hz）输出关节增量，适合精细操作：

```python
# π0 风格的高频控制循环（Python 节点）
class PiZeroBridgeNode(Node):
    def __init__(self):
        # 订阅 π0 推理输出（25Hz）
        self.action_sub = self.create_subscription(
            Float32MultiArray, '/pi_zero/action', self.on_action, 10)
        # 直接发给 JointGroupVelocityController
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_names = ['joint_1', ..., 'joint_7']
        self.q_current = np.zeros(7)

    def on_action(self, msg):
        delta_q = np.array(msg.data[:7])  # rad，单步增量
        # 转换为速度（假设 25Hz → dt=0.04s）
        velocities = delta_q / 0.04
        # 安全限速
        velocities = np.clip(velocities, -self.max_vel, self.max_vel)
        cmd = Float64MultiArray(data=velocities.tolist())
        self.vel_pub.publish(cmd)
```

---

## 六、训练与推理时坐标系一致性

**常见坑**：训练时用的是相机坐标系或世界坐标系，推理时 ROS 2 的 TF 树可能不匹配。

建议：
1. 在 VlaActionBridgeNode 中明确指定 `frame_id`（与训练数据一致）
2. 使用 `tf2_ros::TransformListener` 在线查询坐标变换
3. 将坐标系信息作为元数据写入 rosbag2（见 `07_data_flywheel.md`）
