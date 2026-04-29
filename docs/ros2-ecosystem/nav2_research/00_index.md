# Nav2 架构调研 — 总索引

> **调研范围**：Nav2（Navigation2）完整架构——六大服务器、BT 导航、Costmap2D 插件体系、规划/控制器对比、定位方案、与 `chassis_protocol` 的对接清单  
> **关联文档**：  
> - 本仓库 `docs/ros2-ecosystem/ros2_control_research/`（Phase 1，cmd_vel 接口端）  
> - 本仓库 `docs/embodied-ai/llm_vla_ros2_integration/01_task_hierarchy.md`（Nav2 是 Layer 2 协调层的导航实现）  
> - Nav2 官方文档：https://docs.nav2.org

---

## 一、Nav2 整体服务器拓扑图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                       导航客户端（上层应用）                               │
│  LLM Agent / rai 工具调用 / 自定义任务节点                                │
│  → NavigateToPose Action Goal / NavigateThroughPoses Action Goal         │
└─────────────────────────┬────────────────────────────────────────────────┘
                          │ Action (NavigateToPose / NavigateThroughPoses)
┌─────────────────────────▼────────────────────────────────────────────────┐
│  bt_navigator（行为树导航器）                                             │
│  ├── 加载 navigate_to_pose.xml / navigate_through_poses.xml              │
│  ├── BT Tick Loop（默认 100ms）                                           │
│  └── 调用下列服务器（通过 BT 节点 → Action / Service）                    │
└──┬──────────────┬───────────────┬──────────────┬───────────────┬─────────┘
   │              │               │              │               │
   ▼              ▼               ▼              ▼               ▼
planner_server  controller_server  behavior_server  smoother_server  waypoint_follower
（路径规划）    （跟踪控制）        （恢复行为）    （轨迹平滑）    （多航点）
   │              │               │              │
   ▼              ▼               ▼              ▼
 map → path    path → cmd_vel   BT 恢复子树   平滑后 path
                    │
              /cmd_vel (Twist)
                    │
              ┌─────▼──────┐
              │  底盘驱动   │
              │ chassis_   │
              │ protocol   │
              └────────────┘

横向依赖：
  所有服务器 ← global_costmap（全局地图，用于规划）
  controller_server ← local_costmap（局部地图，用于避障）
  bt_navigator ← /tf（odom → base_link → map 变换链）
  planner_server ← /map (OccupancyGrid) 或 SLAM Toolbox 实时地图
```

---

## 二、核心消息流（NavigateToPose 完整路径）

```
1. 客户端发送 Goal：
   NavigateToPose.Goal{pose: PoseStamped("map")}

2. bt_navigator 接收，加载 navigate_to_pose.xml，开始 tick BT：
   BT: ComputePathToPose → FollowPath → (失败时) RecoveryNode

3. ComputePathToPose BT 节点 → planner_server Action：
   ComputePathToPose.Goal{start, goal, planner_id: "GridBased"}
   ← Result{path: nav_msgs/Path}

4. FollowPath BT 节点 → controller_server Action：
   FollowPath.Goal{path, controller_id: "FollowPath"}
   ← 持续 Feedback{current_pose, speed, distance_remaining}
   → 内部循环发布 /cmd_vel (geometry_msgs/TwistStamped)

5. chassis_protocol 消费 /cmd_vel：
   DiffDriveController → 速度命令 → 底盘硬件

6. odom 反馈：
   chassis_protocol odom publisher → /odom → tf（odom → base_link）

7. 到达目标或超时/失败 → bt_navigator 返回 Result 给客户端
```

---

## 三、文档目录

| 文件 | 内容 |
|------|------|
| [01_architecture_overview.md](./01_architecture_overview.md) | 六大服务器 lifecycle 管理、节点图、消息流、参数配置框架 |
| [02_behavior_tree_integration.md](./02_behavior_tree_integration.md) | BT.CPP v4 集成、nav2_behavior_tree 内置节点全表、自定义 BT 节点开发 |
| [03_costmap2d_plugins.md](./03_costmap2d_plugins.md) | Costmap2D 双层架构（global/local）、5 类插件详解、参数调优 |
| [04_planner_plugins.md](./04_planner_plugins.md) | NavFn / SmacPlanner（A*/Hybrid-A*/State-Lattice）/ ThetaStar 对比 |
| [05_controller_plugins.md](./05_controller_plugins.md) | DWB / MPPI / RPP 原理与参数对比、高速 / 精准 / 受限空间场景选型 |
| [06_localization.md](./06_localization.md) | AMCL vs SLAM Toolbox、nav2_map_server、nav2_collision_monitor |
| [07_chassis_protocol_integration.md](./07_chassis_protocol_integration.md) | 与 chassis_protocol 的完整对接清单（QoS / TF / odom / cmd_vel 拓扑 / **velocity_smoother 频率解耦 20→50 Hz**）|

---

## 四、关键参数文件组织（Nav2 推荐结构）

```
my_robot_bringup/
├── config/
│   ├── nav2_params.yaml          ← 主参数文件（所有服务器）
│   ├── bt/
│   │   ├── navigate_to_pose.xml
│   │   └── navigate_through_poses.xml
│   └── costmap/
│       ├── global_costmap.yaml   （可选单独文件）
│       └── local_costmap.yaml
└── launch/
    └── nav2_bringup.launch.py
```

---

## 五、与本仓库其他模块的交叉引用

| 已有工作 | 与 Nav2 的关联 |
|---------|--------------|
| `chassis_protocol` HAL（本仓库）| Nav2 的 `/cmd_vel` 消费者；`/odom` 提供者；TF `odom→base_link` 发布者 |
| `ros2_control_research/`（Phase 1）| `DiffDriveController` = controller_server 和底盘之间的中间件；ros2_control 的 `cmd_vel` QoS 需与 Nav2 保持一致 |
| `llm_vla_ros2_integration/01_task_hierarchy.md` | Nav2 的 `NavigateToPose` 是 Layer 2 的导航接口；LLM Agent（Layer 4）通过 `rai` 工具调用它 |
| `ros2_lifecycle_node_research/` | Nav2 所有服务器均为 LifecycleNode；lifecycle manager 统一管理其状态机 |
