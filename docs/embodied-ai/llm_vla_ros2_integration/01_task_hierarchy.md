# 四层任务分层详解 — 架构边界、契约与异步通信模式

---

## 一、分层设计原则

### 1.1 单向依赖（上层依赖下层，下层不感知上层）

```
Layer 4（认知）→ Layer 3（技能）→ Layer 2（协调）→ Layer 1（控制）→ Layer 0（安全）
```

每一层只通过**明确定义的接口**与下一层交互，绝不跨层调用。这保证：
- Layer 1 实时控制环不受 Layer 4 LLM 推理延迟影响
- 可以独立替换任意层的实现（换模型不影响控制器）

### 1.2 时间边界（每层有最大允许延迟）

| 层 | 最大允许响应延迟 | 超时行为 |
|----|----------------|---------|
| Layer 4 LLM | 30 s | 报告超时，保持 Layer 3 当前行为 |
| Layer 3 VLA | 1 s | 降级到安全姿态 / 暂停 |
| Layer 2 BT | 500 ms | 执行 Recovery 行为（Nav2 语义）|
| Layer 1 Control | 10 ms（1 个控制周期）| 保持上一个命令 / 零速度 |
| Layer 0 Safety | 1 ms | 硬件 E-Stop |

### 1.3 状态机语义（不只是发消息）

各层之间不能只发"one-shot 命令"，而要支持：
- **取消**（任务执行到一半时上层改变主意）
- **反馈**（下层进度实时上报）
- **抢占**（高优先级任务打断低优先级）

ROS 2 `action` 接口天然支持这三点，是跨层通信的**首选机制**。

---

## 二、Layer 4 — 认知层（LLM 任务规划）

### 2.1 角色

接收**人类自然语言指令**（或自主感知触发），将其分解为可由 Layer 3/2 执行的**技能序列**。

### 2.2 输入

| 来源 | 格式 | ROS 2 接口 |
|------|------|-----------|
| 人类指令 | `string` | `std_msgs/String` topic 或 Service |
| 场景描述（VLM） | `string`（JSON/自然语言）| 同上 |
| 场景图（结构化）| JSON object list | 自定义 msg |
| 任务完成/失败反馈 | `bool + reason` | Action Feedback / Status Topic |

### 2.3 输出（工具调用模式）

LLM 的输出应被**结构化为工具调用**，而不是直接发 ROS 消息：

```json
// LLM 输出示例（OpenAI Function Calling 格式）
{
  "tool": "pick_object",
  "args": {
    "object_name": "red_cup",
    "destination": "shelf_A"
  }
}
```

`rai` / `ros2_llm` 等框架负责将这个 JSON 翻译成对应的 ROS 2 Action 调用。

### 2.4 LLM 工具注册（ROS 2 服务/Action → LLM 工具）

```python
# rai 框架风格（简化）
@ros2_tool(action_type=PickAction, description="Pick up an object by name")
def pick_object(object_name: str, destination: str) -> PickResult:
    """Send pick action to manipulation layer and wait for result."""
    goal = PickAction.Goal(object_name=object_name, destination=destination)
    return pick_action_client.send_goal_and_wait(goal, timeout=10.0)

# 注册后，LLM 可以通过函数调用触发 ROS 2 动作
```

---

## 三、Layer 3 — 技能层（VLA 动作推理）

### 3.1 角色

接收**语言条件（task description）+ 视觉观测（图像）**，输出**低层动作命令**（关节增量、末端位姿、速度）。

### 3.2 VLA 的两种运行模式

#### 模式 A：单步推理（Reactive，高频）

```
频率：10-30 Hz（50-100ms 每步）
输入：最新 RGB 图像 + 当前指令 string
输出：ΔJoint（6-7 维 float）或 ΔEE Pose（7 维）
特点：连续、小步长、类似 MPC
代表模型：π0、RDT-1B
```

#### 模式 B：序列推理（Planning，低频）

```
频率：0.1-2 Hz（500ms-5s 每次）
输入：关键帧图像（多视角）+ 任务描述
输出：完整 JointTrajectory（多步）
特点：一次性规划较长的动作序列
代表模型：RT-2、OpenVLA（goal-conditioned）
```

### 3.3 与 Layer 2 的接口

```
模式 A（连续控制）：
  → 直接发 JointState 增量到 ros2_control：
    topic: /vla/joint_commands (sensor_msgs/JointState)
    控制器：自定义 VLAFollowerController（继承 ros2_control Controller）

模式 B（轨迹规划）：
  → 调用 MoveIt 2 FollowJointTrajectory Action：
    action: /follow_joint_trajectory
    轨迹：VLA 输出的关节序列 + 时间戳
```

### 3.4 安全包装层（Safety Wrapper）

在 VLA 输出和实际控制器之间**必须**有安全检查节点：

```cpp
// VLASafetyWrapper 节点
class VLASafetyWrapper : public rclcpp::Node {
  void onVlaCommand(const sensor_msgs::msg::JointState & cmd) {
    // 1. 关节限位检查（URDF limits）
    if (!checkJointLimits(cmd)) { triggerSafetyStop(); return; }
    // 2. 速度/加速度限制
    auto filtered = rateLimit(cmd, max_velocity_, max_acceleration_);
    // 3. 碰撞预检（可选：FCL/Bullet 快速碰撞查询）
    if (collision_check_enabled_ && checkCollision(filtered)) {
      publishEmergencyHold(); return;
    }
    // 4. 通过：下发控制命令
    safe_cmd_pub_->publish(filtered);
  }
};
```

---

## 四、Layer 2 — 协调层（行为树 / 状态机）

### 4.1 角色

将 Layer 4 分解出的"技能名称"映射到**具体的 ROS 2 Action 调用序列**，处理重试、错误恢复、条件分支。

### 4.2 行为树（BT）作为协调语言

```xml
<!-- 示例：pick_and_place 行为树 -->
<BehaviorTree ID="PickAndPlace">
  <Sequence name="pick_and_place_sequence">
    <!-- 导航到抓取位置 -->
    <NavigateToPose name="nav_to_pick" goal="{pick_approach_pose}"/>
    <!-- VLA 精细抓取 -->
    <RosAction name="vla_grasp"
               action_type="GraspAction"
               server_name="/vla/grasp"
               goal="{object_description}"/>
    <!-- 导航到放置位置 -->
    <NavigateToPose name="nav_to_place" goal="{place_pose}"/>
    <!-- VLA 精细放置 -->
    <RosAction name="vla_place"
               action_type="PlaceAction"
               server_name="/vla/place"
               goal="{object_description}"/>
  </Sequence>
</BehaviorTree>
```

### 4.3 BT 节点的 ROS 2 集成（BehaviorTree.CPP v4）

```cpp
// 自定义 Action 节点（将 BT tick 映射到 ROS 2 Action）
class VLAGraspAction : public BT::StatefulActionNode,
                       public nav2_behavior_tree::BtActionNode<GraspAction> {
  BT::NodeStatus onRunning() override {
    if (action_client_->isResultAvailable()) {
      auto result = action_client_->getResult();
      if (result.success) return BT::NodeStatus::SUCCESS;
      else return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;  // 异步等待
  }
};
```

### 4.4 LLM → BT 动态生成（前沿实践）

```
LLM 输出 BT XML（或 JSON）→ BT Factory 动态加载 → 执行

优点：LLM 可以根据任务动态生成执行计划
缺点：动态生成的 BT 可能包含未注册的节点（需要验证）
实践：限制 LLM 只能使用预定义的 BT 节点库（Few-shot prompting）
```

---

## 五、Layer 1 — 控制层（ros2_control）

### 5.1 接口契约

Layer 1 对上层**暴露标准接口**，不感知上层是 VLA、BT 还是人工操控：

```
机械臂：
  接收：FollowJointTrajectory Action / JointGroupVelocityController topic
  状态：/joint_states (sensor_msgs/JointState)

移动底盘：
  接收：/cmd_vel (geometry_msgs/TwistStamped)
  状态：/odom (nav_msgs/Odometry)

复合平台（移动 + 操作）：
  上述二者同时存在
  + /tf（base_link → tool0 等坐标系变换）
```

### 5.2 VLA 接入 ros2_control 的两种方案

```
方案 A：经由 MoveIt 2（推荐，安全性高）
  VLA → JointTrajectory → MoveIt FollowJointTrajectory → JTC → ros2_control

方案 B：直接写 CommandInterface（低延迟，需自行保证安全）
  VLA → 自定义 VLAController（继承 ControllerInterface）
       → write() 中直接写 CommandInterface
       → HardwareInterface → 硬件
```

---

## 六、Layer 0 — 安全层

安全层不是 ROS 2 软件栈的一部分，而是**硬件 + 固件**层：

```
安全监控机制（并行于 ROS 2 软件栈）：
  ├── 关节力矩传感器：超限立即断电（微控制器，< 1ms 响应）
  ├── 碰撞检测：碰撞传感器 / 电流突变检测（HAL 层处理）
  ├── 软件 E-Stop：ros2_control lifecycle deactivate（~10ms）
  └── 硬件 E-Stop：急停按钮直接切断驱动器电源（< 1ms）
```

**ros2_control 的 lifecycle 管理（deactivate）是软件 E-Stop 的实现点**，参见 `ros2_control_research/05_controller_lifecycle.md`。
