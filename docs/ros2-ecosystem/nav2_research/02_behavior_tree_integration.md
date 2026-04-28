# BehaviorTree.CPP v4 与 nav2_behavior_tree 集成详解

---

## 一、BehaviorTree.CPP v4 在 Nav2 中的角色

Nav2 使用 **BehaviorTree.CPP（BT.CPP）v4** 作为行为树引擎。bt_navigator 将整个导航逻辑（规划→跟踪→恢复）编码为 XML 格式的 BT，由 BT.CPP 执行。

### 1.1 BT 节点类型速查

| 类型 | 返回值 | 说明 |
|------|--------|------|
| `SequenceNode` | 全 SUCCESS → SUCCESS；任一 FAILURE → FAILURE | 顺序执行（类似 AND）|
| `FallbackNode` | 任一 SUCCESS → SUCCESS；全 FAILURE → FAILURE | 选择执行（类似 OR）|
| `ParallelNode` | 配置 M/N 成功条件 | 并行执行 |
| `ReactiveSequence` | 每次 tick 重新从第一子节点开始 | 响应式序列 |
| `ReactiveFallback` | 同上 | 响应式选择 |
| `ActionNode`（叶节点）| RUNNING / SUCCESS / FAILURE | 实际工作（调用 ROS 2 Action 等）|
| `ConditionNode`（叶节点）| SUCCESS / FAILURE（即时返回）| 状态检查 |
| `DecoratorNode` | 修改子节点返回值 | Inverter / Repeat / Retry 等 |

---

## 二、`nav2_behavior_tree` 内置节点全表

### 2.1 Action 节点（调用 ROS 2 Action 服务器）

| BT 节点名 | 对应 Action 服务器 | 功能 |
|----------|-----------------|------|
| `ComputePathToPose` | `/compute_path_to_pose` | 规划到目标 pose 的路径 |
| `ComputePathThroughPoses` | `/compute_path_through_poses` | 规划经过多个 poses 的路径 |
| `FollowPath` | `/follow_path` | 跟踪给定路径 |
| `SmoothPath` | `/smooth_path` | 平滑路径 |
| `Spin` | `/spin` | 原地旋转恢复 |
| `BackUp` | `/back_up` | 后退恢复 |
| `DriveOnHeading` | `/drive_on_heading` | 沿朝向直行 |
| `Wait` | `/wait` | 等待指定时间 |
| `ClearEntireCostmap` | `/global_costmap/clear_entirely_global_costmap` 等 | 清除代价地图（Service）|
| `NavigateToPose` | `/navigate_to_pose` | 子导航（嵌套导航）|

### 2.2 Condition 节点（即时条件检查）

| BT 节点名 | 功能 |
|----------|------|
| `GoalReached` | 当前 pose 是否到达目标（xy + yaw tolerance）|
| `GoalUpdated` | 是否收到新的导航 Goal（用于响应式中断）|
| `IsPathValid` | 当前路径是否仍然可行（无碰撞）|
| `IsBatteryLow` | 电池电量是否低于阈值（`/battery_state` topic）|
| `IsStuck` | 机器人是否陷入局部极小值 |
| `InitialPoseReceived` | 是否已收到初始位置估计 |

### 2.3 Decorator 节点

| BT 节点名 | 功能 |
|----------|------|
| `RateController` | 限制子树的 tick 频率（降频，避免过频繁规划）|
| `DistanceController` | 机器人移动超过距离阈值才重新 tick 子树 |
| `TimeExpiredCondition` | 超时后返回 SUCCESS（强制继续）|
| `GoalUpdatedController` | 仅当 Goal 更新时 tick 子树 |
| `SpeedController` | 根据当前速度调整 tick 频率 |

---

## 三、内置 BT 配置文件解析

### 3.1 `navigate_to_pose_w_recovery.xml`（Nav2 默认）

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="NavigateToPose">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <!-- 主序列：规划 + 跟踪 -->
      <PipelineSequence name="NavigateWithReplanning">
        <!-- 距离控制器：移动 1m 后重新规划 -->
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathRecovery">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <!-- 规划失败时：清除代价地图后重试 -->
            <ClearEntireCostmap server_name="/global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>

        <!-- 路径跟踪（含跟踪失败恢复）-->
        <RecoveryNode number_of_retries="1" name="FollowPathRecovery">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap server_name="/local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>

      <!-- 导航失败时的全局恢复 -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>          <!-- 如果 Goal 更新，直接重试主序列 -->
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap server_name="/local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap server_name="/global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>           <!-- 旋转 90° -->
          <Wait wait_duration="5.0"/>        <!-- 等待 5s -->
          <BackUp backup_dist="0.30"/>       <!-- 后退 30cm -->
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

**关键机制**：
- `RecoveryNode`：先执行第一子节点；若失败，执行第二子节点（恢复），然后重试第一子节点，最多 `number_of_retries` 次
- `PipelineSequence`：类似 `Sequence`，但已经 RUNNING 的子节点不重新从头 tick（避免重复规划）
- `RoundRobin`：轮流尝试恢复动作（旋转 → 等待 → 后退 → 再旋转 → ...）

---

## 四、自定义 BT 节点开发

### 4.1 自定义 Action 节点（调用 ROS 2 Action）

```cpp
// custom_bt_nodes/navigate_to_charging_station.hpp
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "my_robot_msgs/action/navigate_to_charging_station.hpp"

class NavigateToChargingStation
    : public nav2_behavior_tree::BtActionNode<
        my_robot_msgs::action::NavigateToChargingStation>
{
public:
  NavigateToChargingStation(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
  : BtActionNode<my_robot_msgs::action::NavigateToChargingStation>(
      xml_tag_name, action_name, conf)
  {}

  // 填写 Goal
  void on_tick() override {
    // 从黑板或参数读取充电桩位置
    std::string station_id;
    getInput<std::string>("station_id", station_id);
    goal_.station_id = station_id;
  }

  // 处理 Feedback（可选）
  BT::NodeStatus on_feedback(
      const std::shared_ptr<const Feedback> feedback) override
  {
    RCLCPP_INFO(node_->get_logger(),
        "Distance to station: %.2f m", feedback->distance_remaining);
    return BT::NodeStatus::RUNNING;
  }

  // 声明端口（BT 黑板输入/输出）
  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("station_id", "Charging station ID")
    });
  }
};
```

### 4.2 自定义 Condition 节点

```cpp
// custom_bt_nodes/is_battery_low.hpp
class IsBatteryLow : public BT::ConditionNode {
public:
  IsBatteryLow(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf), node_(conf.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
            battery_level_ = msg->percentage;
        });
  }

  BT::NodeStatus tick() override {
    double threshold;
    getInput<double>("min_battery", threshold);
    return (battery_level_ < threshold)
        ? BT::NodeStatus::SUCCESS   // 电量低 → 触发恢复
        : BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("min_battery", 0.1, "Low battery threshold (0-1)")};
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  double battery_level_{1.0};
};
```

### 4.3 注册自定义节点（Plugin 方式）

```cpp
// custom_nav2_bt_nodes_plugin.cpp
#include "behaviortree_ros2/plugins.hpp"

BT_REGISTER_NODES(factory) {
  // 注册 Action 节点
  nav2_behavior_tree::RegisterRosAction<NavigateToChargingStation>(
      factory, "NavigateToChargingStation", node, server_timeout);
  // 注册 Condition 节点
  factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
}
```

```yaml
# bt_navigator 参数：加载自定义插件
bt_navigator:
  ros__parameters:
    plugin_libs:
      - custom_nav2_bt_nodes_plugin   # 自定义插件 so 文件名
```

---

## 五、BT 黑板（Blackboard）关键键值

Nav2 在 BT 黑板上预定义了以下键值，供 BT 节点读写：

| 键 | 类型 | 说明 |
|----|------|------|
| `goal` | `geometry_msgs/PoseStamped` | 当前导航目标（NavigateToPose）|
| `goals` | `vector<PoseStamped>` | 多航点目标（NavigateThroughPoses）|
| `path` | `nav_msgs/Path` | 规划器输出的全局路径 |
| `goal_pose` | `PoseStamped` | 与 goal 等同，某些节点使用此名 |
| `error_code_id` | `int` | 错误码（规划失败 / 跟踪失败等）|
| `number_recoveries` | `int` | 当前恢复次数计数 |
| `node` | `rclcpp::Node::SharedPtr` | 共享 ROS 2 节点实例 |

---

## 六、BT 调试工具

- **Groot2**：官方 BT 可视化调试工具，连接 BT.CPP 的 ZMQ publisher 端口实时查看 BT 状态
  ```bash
  # bt_navigator 参数：开启 Groot2 监控
  bt_navigator:
    ros__parameters:
      enable_groot_monitoring: true
      groot_zmq_publisher_port: 1666
      groot_zmq_server_port: 1667
  ```
- **ros2 topic echo /bt_navigator/transition_event**：监控 lifecycle 状态变化
- **rosbag2 录制 /bt_navigator/log**：记录 BT 执行日志（nav2 默认发布）
