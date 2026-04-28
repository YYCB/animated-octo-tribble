# Nav2 高阶配置深化

> 深化研究 Phase 5 中留下的开放问题：MPPI 参数系统调优方法、SmacPlannerLattice 运动学约束文件生成、nav2_collision_monitor 配置，以及自定义 BT 节点开发。

---

## 一、MPPI Controller 深化调优

### 1.1 MPPI 工作原理回顾

Model Predictive Path Integral（MPPI）通过 Monte Carlo 采样多条扰动轨迹并以加权平均确定控制输出：

```
算法流程：
  ① 生成 K 条扰动序列  δu_k ~ N(0, Σ)
  ② 对每条轨迹仿真 H 步（Horizon）
  ③ 计算每条轨迹代价 S_k
  ④ 权重：w_k = exp(-S_k / λ)
  ⑤ 更新控制序列：u ← u + Σ_k w_k δu_k（归一化）
  ⑥ 执行 u[0]，滚动前进
```

**关键超参数**：`K`（样本数）、`H`（Horizon）、`λ`（temperature）、`Σ`（探索噪声）。

### 1.2 参数系统调优指南

```yaml
# nav2_params.yaml — MPPI 精调配置
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      
      # 基础参数
      time_steps: 56          # Horizon H：越大越平滑，但计算量线性增加
      model_dt: 0.05          # 仿真步长（秒）：0.05×56 = 2.8s 预测窗口
      batch_size: 2000        # 样本数 K：GPU 上可增到 4096+
      
      # 探索参数
      vx_std: 0.2             # x 方向速度扰动标准差
      vy_std: 0.2             # y 方向（差分底盘设 0.0）
      wz_std: 0.4             # 角速度扰动（转弯场景增大）
      
      # 代价温度（λ）
      temperature: 0.3        # 越小：选择代价最小轨迹；越大：轨迹更多样
      gamma: 0.015            # 折扣因子（远端代价权重）
      
      # 约束
      vx_max: 0.5
      vx_min: -0.35           # 允许少量倒退
      vy_max: 0.0
      wz_max: 1.9
      
      # 代价权重调优（这是调参的核心）
      critics: ["ConstraintCritic", "GoalCritic",
                "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic",
                "PreferForwardCritic"]
      
      ConstraintCritic.enabled: true
      ConstraintCritic.cost_power: 1
      ConstraintCritic.cost_weight: 4.0
      
      GoalCritic.enabled: true
      GoalCritic.cost_power: 1
      GoalCritic.cost_weight: 5.0
      GoalCritic.threshold_to_consider: 1.4
      
      PathAlignCritic.enabled: true
      PathAlignCritic.cost_weight: 14.0    # 路径跟随场景调高
      PathAlignCritic.max_path_occupancy_ratio: 0.05
      
      PreferForwardCritic.enabled: true
      PreferForwardCritic.cost_weight: 5.0  # 室内机器人调高，避免倒退
```

### 1.3 调优工作流

```
MPPI 调优流程（推荐顺序）：

1. 先调 PreferForwardCritic
   └── 目标：消除不必要的倒退行为
   └── 诊断：观察 /local_plan 是否经常出现倒退段

2. 再调 PathAlignCritic + PathFollowCritic
   └── 目标：轨迹贴近全局路径
   └── 诊断：/local_plan 与 /plan 的横向偏差

3. 调 GoalCritic + GoalAngleCritic
   └── 目标：精确停止位姿（位置 < 5cm，角度 < 5°）
   └── 诊断：goal reached 后 /odom 的残差

4. 最后调 temperature (λ) 和 vx_std/wz_std
   └── 目标：在障碍物密集场景下不卡死
   └── 诊断：rviz2 显示采样轨迹的多样性
```

### 1.4 GPU 加速配置

在 Jetson Orin AGX 上，MPPI 可利用 CUDA 加速样本评估：

```yaml
# 需要安装 nav2_mppi_controller 的 CUDA 版本
FollowPath:
  use_cuda: true           # 开启 GPU 采样
  batch_size: 4096         # GPU 下可大幅增加样本数
  # Orin AGX 测试：batch_size=4096，56步，约 2.1ms/次更新
```

---

## 二、SmacPlannerLattice 运动学约束配置

### 2.1 运动学原语文件生成

SmacPlannerLattice 需要预计算的运动学原语文件（`.mprim`），描述机器人在每个离散状态下能执行的运动动作。

```bash
# 使用 nav2_smac_planner 提供的生成脚本
ros2 run nav2_smac_planner precompute_primitives.py \
  --config primitives_config.yaml \
  --output mecanum_primitives.mprim
```

```yaml
# primitives_config.yaml — 麦轮底盘运动学原语参数
minimum_turning_radius: 0.0    # 全向底盘为 0.0（差分底盘设为实际最小转弯半径）
angular_resolution_of_heading: 8   # 离散朝向数（8 = 45°间隔，16 = 22.5°间隔）
angle_quantization_bins: 72        # 高精度规划时使用 72
allow_reverse_expansion: false     # 全向底盘可以设 true
grid_resolution: 0.05              # 地图分辨率一致
turning_radius_penalty: 2.0
# 速度约束
max_linear_vel: 0.5
max_angular_vel: 1.9
```

### 2.2 Lattice Planner 集成

```yaml
# nav2_params.yaml — SmacPlannerLattice
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerLattice"
      allow_unknown: false
      tolerance: 0.25
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      analytic_expansion_ratio: 3.5
      minimum_turning_radius: 0.0    # 麦轮底盘
      reverse_penalty: 2.0
      change_penalty: 0.05
      non_straight_penalty: 1.05
      cost_penalty: 2.0
      rotation_penalty: 5.0
      retrospective_penalty: 0.015
      lattice_filepath: "/opt/robot/config/mecanum_primitives.mprim"
      lookup_table_size: 20.0
      cache_obstacle_heuristic: true
      smooth_path: true
```

---

## 三、nav2_collision_monitor

### 3.1 架构

`nav2_collision_monitor` 是 Nav2 Iron/Humble-backport 引入的独立安全层，独立于 local planner 运行，直接修改速度指令或触发停机。

```
数据流：

/cmd_vel（来自 Controller Server）
    │
    ▼
nav2_collision_monitor
  ├── 读取传感器：LaserScan / PointCloud2 / Range
  ├── 检测 zones：Polygon（前方 1m × 0.8m），Circle（全向 0.3m）
  └── 动作：
       STOP（0 速）/ SLOWDOWN（降速比例）/ LIMIT（硬限速）/ APPROACH
    │
    ▼
/cmd_vel_smoothed（发送给底盘）
```

### 3.2 配置示例

```yaml
# collision_monitor_params.yaml
collision_monitor:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel"
    cmd_vel_out_topic: "cmd_vel_smoothed"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    stop_pub_timeout: 2.0
    
    polygons: ["PolygonFront", "CircleSlow"]
    
    PolygonFront:
      type: "Polygon"
      points: "[[0.5, 0.4], [0.5, -0.4], [0.0, -0.4], [0.0, 0.4]]"
      action_type: "stop"          # 前方停止区
      min_points: 4
      visualize: true
      polygon_pub_topic: "polygon_front"
    
    CircleSlow:
      type: "Circle"
      radius: 0.8                  # 降速区（0.8m 半径）
      action_type: "slowdown"
      slowdown_ratio: 0.3          # 降到 30% 速度
      min_points: 2
      visualize: true
    
    observation_sources: ["scan", "pointcloud"]
    
    scan:
      type: "Scan"
      topic: "/scan"
    
    pointcloud:
      type: "PointCloud"
      topic: "/point_cloud_downsampled"
      min_height: 0.1
      max_height: 1.0
```

---

## 四、自定义 BT 节点开发

### 4.1 BehaviorTree.CPP 基础类型

Nav2 使用 BehaviorTree.CPP v4，自定义节点继承以下类之一：

| 类型 | 应用场景 | 返回值 |
|------|---------|-------|
| `BT::SyncActionNode` | 一次性同步动作（如打印日志） | SUCCESS / FAILURE |
| `BT::StatefulActionNode` | 多帧异步动作（如等待感知结果） | RUNNING / SUCCESS / FAILURE |
| `BT::ConditionNode` | 检查条件（如 battery_ok） | SUCCESS / FAILURE |
| `BT::DecoratorNode` | 修改子节点行为（如重试） | 取决于子节点 |

### 4.2 完整自定义 Action 节点示例

```cpp
// custom_bt_nodes/check_gripper_ready.hpp
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

class CheckGripperReady : public BT::StatefulActionNode
{
public:
  CheckGripperReady(const std::string& name,
                    const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("gripper_topic", "/gripper_state",
                                 "Gripper state topic"),
      BT::OutputPort<bool>("is_ready", "Whether gripper is ready"),
    };
  }

  BT::NodeStatus onStart() override
  {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string topic;
    getInput("gripper_topic", topic);

    subscription_ = node->create_subscription<std_msgs::msg::Bool>(
      topic, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        gripper_ready_ = msg->data;
        received_ = true;
      });

    deadline_ = node->now() + rclcpp::Duration(5, 0);  // 5s timeout
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (received_) {
      setOutput("is_ready", gripper_ready_);
      return gripper_ready_ ? BT::NodeStatus::SUCCESS
                            : BT::NodeStatus::FAILURE;
    }
    if (node->now() > deadline_) {
      RCLCPP_WARN(node->get_logger(), "CheckGripperReady: timeout");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { subscription_.reset(); }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  bool received_ = false;
  bool gripper_ready_ = false;
  rclcpp::Time deadline_;
};
```

### 4.3 注册到 Nav2 BT 插件

```cpp
// custom_bt_plugin.cpp
#include <behaviortree_cpp_v3/bt_factory.h>
#include "check_gripper_ready.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckGripperReady>("CheckGripperReady");
}
```

```cmake
# CMakeLists.txt
add_library(custom_bt_plugin SHARED custom_bt_plugin.cpp)
target_link_libraries(custom_bt_plugin behaviortree_cpp_v3)
pluginlib_export_plugin_description_file(nav2_core bt_plugin.xml)
```

### 4.4 BT XML 中使用自定义节点

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- 标准 Nav2 节点 -->
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      
      <!-- 自定义节点 -->
      <CheckGripperReady
        gripper_topic="/gripper_state"
        is_ready="{gripper_ready}"/>
      
      <!-- 条件分支 -->
      <Fallback>
        <Condition ID="IsGripperReady" gripper_ready="{gripper_ready}"/>
        <AlwaysFailure/>
      </Fallback>
      
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 4.5 在 Nav2 params 中加载自定义 BT 插件

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - custom_bt_plugin  # 自定义插件
    default_nav_to_pose_bt_xml: "/opt/robot/config/custom_navigation.xml"
```
