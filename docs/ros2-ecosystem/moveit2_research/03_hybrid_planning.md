# MoveIt Hybrid Planning — Global + Local Planner 协同

---

## 一、为什么需要 Hybrid Planning

传统 MoveIt 规划流程的问题：

```
问题 1：规划时间长（OMPL 需要 0.5-5s）→ 动态障碍出现时轨迹已过时
问题 2：整段轨迹一次性计算 → 执行过程中无法重新规划
问题 3：局部障碍变化需要完整重新规划 → 系统响应慢
```

Hybrid Planning 的解决方案：

```
全局规划器（Global Planner）：离线/慢速计算整体路径（OMPL，0.5-5s）
局部规划器（Local Planner）：在线/快速处理局部障碍和轨迹偏差（50-100 Hz）

类比：Nav2 中 global planner + local controller（DWB/MPPI）的操作臂版本
```

---

## 二、Hybrid Planning 架构

```
┌───────────────────────────────────────────────────────────────────────┐
│  HybridPlanningManager（协调者）                                       │
│  ├── 触发全局规划 → GlobalPlannerAction                                │
│  ├── 监控局部规划状态                                                   │
│  └── 处理重规划请求（局部失败 → 触发全局重规划）                          │
└───────────────────┬──────────────────────────────────────────────────┘
                    │
          ┌─────────┴──────────┐
          ▼                    ▼
┌──────────────────┐  ┌──────────────────────────────────────────────────┐
│  Global Planner  │  │  Local Planner                                   │
│  (单次运行)       │  │  (连续循环，50-100 Hz)                            │
│  ├─ OMPL 规划    │  │  ├─ 从全局轨迹缓冲区取当前段                       │
│  ├─ 生成全局      │  │  ├─ 根据当前机器人状态调整                          │
│  │   RobotTrajectory│  ├─ 碰撞检测（局部场景）                           │
│  └─ 推送到轨迹    │  └─ 发布 JointTrajectory → ros2_control             │
│      缓冲区       │                                                     │
└──────────────────┘  └──────────────────────────────────────────────────┘
          │                    ▲
          │ TrajectoryBuffer    │ forward lookahead
          └────────────────────┘
```

---

## 三、Hybrid Planning 组件详解

### 3.1 HybridPlanningManager

协调全局和局部规划器，处理以下事件：

```
事件                              处理
────────────────────────────────────────────────────────
收到新目标                        → 触发全局规划
全局规划完成                      → 启动局部规划
局部规划器报告碰撞/卡住            → 停止局部，触发全局重规划
全局规划失败                      → 返回错误给客户端
目标到达                          → 完成
执行过程中收到新目标（抢占）         → 中止当前局部，触发新全局规划
```

### 3.2 全局规划器插件接口

```cpp
// moveit/moveit_ros/hybrid_planning/global_planner/global_planner_plugin.hpp
class GlobalPlannerInterface {
  virtual moveit_msgs::action::GlobalPlanner::Result
  plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
           moveit_msgs::action::GlobalPlanner>> goal_handle) = 0;
};

// 内置实现：MoveItPlanningPipelinePlugin
// → 直接使用 MoveGroup 的规划管线（OMPL/Pilz/STOMP）
```

### 3.3 局部规划器插件接口

```cpp
// moveit/moveit_ros/hybrid_planning/local_planner/local_planner_plugin.hpp
class LocalPlannerInterface {
  virtual moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory & local_trajectory,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            moveit_msgs::action::LocalPlanner>> goal_handle) = 0;
};

// 内置实现：ForwardTrajectory（直接转发全局轨迹段）
// 自定义：实现避障、速度调整等局部优化
```

---

## 四、配置示例

```yaml
# hybrid_planning_manager.yaml
hybrid_planning_manager:
  ros__parameters:
    # 全局规划器
    global_planner_name: MoveItPlanningPipelinePlugin
    global_planner_action_name: /global_planner/run_hybrid_planning
    
    # 局部规划器
    local_planner_name: ForwardTrajectory
    local_planner_action_name: /local_planner/run_local_planning
    
    # 重规划策略
    replanning_threshold: 0.1       # m，路径偏差超过此值触发重规划
    
# local_planner.yaml
local_planner:
  ros__parameters:
    group_name: arm
    local_solution_topic: /local_trajectory_controller/joint_trajectory
    local_solution_type: trajectory_msgs/msg/JointTrajectory
    publish_local_solution_enabled: true
    
    # ForwardTrajectory 参数
    forward_trajectory:
      wait_for_trajectory_completion: false
      trajectory_lookup_mode: USE_NEAREST_ROBOT_POINT
      joint_group_name: arm
```

---

## 五、自定义局部规划器（避障示例）

```cpp
// custom_local_planner.hpp
#include "moveit/local_planner/local_planner_plugin_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"

class CollisionAwareLocalPlanner : public LocalPlannerInterface {
public:
  bool initialize(const rclcpp::Node::SharedPtr & node,
                  const moveit::core::RobotModelConstPtr & robot_model,
                  const std::string & group_name) override
  {
    node_ = node;
    robot_model_ = robot_model;
    group_name_ = group_name;
    // 订阅局部 planning scene（来自 planning_scene_monitor）
    scene_sub_ = node_->create_subscription<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10,
        [this](const moveit_msgs::msg::PlanningScene::SharedPtr msg) {
            // 更新局部碰撞场景
        });
    return true;
  }

  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory & global_trajectory,
        const std::shared_ptr<GoalHandle> goal_handle) override
  {
    moveit_msgs::action::LocalPlanner::Feedback feedback;
    
    // 1. 找到全局轨迹中最近的点
    size_t current_waypoint = findNearestWaypoint(global_trajectory);
    
    // 2. 前视 N 个轨迹点，检查碰撞
    for (size_t i = current_waypoint; i < current_waypoint + lookahead_; i++) {
        if (checkCollision(global_trajectory.getWayPoint(i))) {
            // 3. 发现碰撞：请求全局重规划
            feedback.feedback = "collision_ahead";
            return feedback;
        }
    }
    
    // 4. 无碰撞：发布局部轨迹段
    publishLocalTrajectory(global_trajectory, current_waypoint);
    feedback.feedback = "executing";
    return feedback;
  }
};
```

---

## 六、Hybrid Planning vs 传统 MoveGroup 对比

| 特性 | 传统 MoveGroup | Hybrid Planning |
|------|:------------:|:---------------:|
| 规划延迟 | 0.5-5s | 全局 0.5-5s，局部 < 10ms |
| 动态障碍适应 | ❌（需重新规划）| ✅（局部实时避障）|
| 执行中断恢复 | 慢（全局重规划）| 快（局部规划器先处理，必要时请求全局）|
| 实现复杂度 | 低 | 高（需要实现局部规划器）|
| 成熟度 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐（仍在快速发展）|
| 适用场景 | 静态/慢速动态环境 | 动态环境，人机协作，抓取 |

---

## 七、与本仓库的关联

- **移动操作（Mobile Manipulation）**：chassis_protocol 底盘移动时，操作臂同时运动，Hybrid Planning 可以让臂在底盘运动过程中持续更新轨迹
- **LLM 驱动**：LLM Agent 发送新任务目标时，Hybrid Planning Manager 可以平滑地中断当前执行并重规划，无需等待当前轨迹完成
