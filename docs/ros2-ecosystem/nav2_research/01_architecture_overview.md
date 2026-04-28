# Nav2 整体架构概览 — 六大服务器、Lifecycle 管理与参数体系

---

## 一、六大服务器一览

Nav2 将"导航"解耦为独立的 **LifecycleNode** 服务器，每个服务器专注一个职责：

| 服务器 | 节点名 | 核心职责 | 插件接口 |
|--------|--------|---------|---------|
| **bt_navigator** | `bt_navigator` | 加载并执行导航行为树；将 Action Goal 路由到子服务器 | `NavigatorBase`（可替换整棵 BT）|
| **planner_server** | `planner_server` | 全局路径规划（map → path）| `nav2_core::GlobalPlanner` |
| **controller_server** | `controller_server` | 路径跟踪控制（path → cmd_vel）| `nav2_core::Controller` |
| **behavior_server** | `behavior_server` | 恢复行为执行（旋转逃脱、备份等）| `nav2_core::Behavior` |
| **smoother_server** | `smoother_server` | 路径平滑（减少折线）| `nav2_core::Smoother` |
| **waypoint_follower** | `waypoint_follower` | 多航点顺序导航 | `nav2_core::WaypointTaskExecutor` |

另有辅助节点（不是"服务器"但同等重要）：

| 节点 | 职责 |
|------|------|
| `nav2_lifecycle_manager` | 统一管理所有服务器的 configure/activate/deactivate/cleanup |
| `nav2_map_server` | 提供静态地图（`/map` topic + `load_map` service）|
| `nav2_amcl` | 粒子滤波定位（`/amcl_pose`，`map → odom` TF）|
| `nav2_collision_monitor` | 独立的碰撞监控节点（不依赖其他服务器）|

---

## 二、LifecycleNode 状态机与 nav2_lifecycle_manager

### 2.1 为什么 Nav2 用 Lifecycle

Nav2 所有服务器都是 `nav2_util::LifecycleNode`（封装了 `rclcpp_lifecycle::LifecycleNode`）。这使得：
1. **有序启动**：lifecycle manager 按依赖顺序 `configure → activate`，避免地图未加载就开始规划
2. **安全停止**：`deactivate → cleanup` 确保资源（costmap 内存、插件线程）被正确释放
3. **热重启**：不重启进程即可重新加载参数和地图

### 2.2 lifecycle_manager 启动顺序

```yaml
# nav2_lifecycle_manager 参数
lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names:
      - map_server        # 1. 先加载地图
      - amcl              # 2. 定位（需要地图）
      - bt_navigator      # 3. 导航器
      - planner_server    # 4. 规划器（需要 costmap）
      - controller_server # 5. 控制器（需要 costmap）
      - behavior_server   # 6. 恢复行为
      - smoother_server   # 7. 平滑器
      - waypoint_follower # 8. 航点跟随器（可选）
```

### 2.3 lifecycle_manager 的邦德（Bond）机制

lifecycle_manager 和每个服务器之间维持一个 **bond（心跳连接）**：
- 服务器每秒发送一次心跳
- 若超时（默认 4s）未收到心跳，lifecycle_manager 认为服务器崩溃
- 自动触发整个导航栈的 `deactivate → activate`（热重启）

```cpp
// nav2_util 的 bond 使用（服务器端）
bond_ = std::make_unique<bond::Bond>(
    "bond_topic", get_name(), shared_from_this());
bond_->setHeartbeatPeriod(0.10);  // 100ms 心跳
bond_->setHeartbeatTimeout(4.0);  // 4s 超时
bond_->start();
```

---

## 三、bt_navigator 详解

### 3.1 导航器插件（Navigator）

bt_navigator 支持多种 Navigator 插件，每个插件对应一种 Action 接口：

| Navigator 插件 | Action 接口 | BT 文件 |
|--------------|------------|---------|
| `NavigateToPoseNavigator` | `NavigateToPose` | navigate_to_pose.xml |
| `NavigateThroughPosesNavigator` | `NavigateThroughPoses` | navigate_through_poses.xml |
| 自定义（如 `FollowPath`）| 自定义 Action | 自定义 XML |

### 3.2 BT Tick 循环

```
bt_navigator 的主循环（每次 NavigateToPose Goal 到来）：

1. 将 Goal 写入 BT 黑板（goal_pose / goals / path 等）
2. 创建 BT 实例（从 XML 加载）
3. 循环 tick：
   while (tree.tickOnce() == RUNNING) {
       sleep(tick_period);  // 默认 100ms（bt_loop_duration）
       if (goal_updated) {
           // 新 Goal 抢占：更新黑板 goal_pose，BT 继续运行
       }
   }
4. 返回 SUCCEEDED / FAILED → Action Result
```

### 3.3 关键参数

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map          # 全局坐标系
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10       # ms，BT tick 间隔
    default_server_timeout: 20 # s，等待服务器响应超时
    # 导航器插件列表
    navigators:
      - nav2_bt_navigator/NavigateToPoseNavigator
      - nav2_bt_navigator/NavigateThroughPosesNavigator
    # 各导航器的 BT 文件
    NavigateToPose:
      default_bt_xml_filename: "navigate_to_pose_w_recovery.xml"
```

---

## 四、planner_server 详解

### 4.1 角色

接收 `ComputePathToPose` / `ComputePathThroughPoses` Action，返回 `nav_msgs/Path`。

### 4.2 全局 Costmap

planner_server 内部维护一个 **global_costmap**（`nav2_costmap_2d::Costmap2DROS`）：
- 输入：静态地图（`/map`）+ 传感器数据（激光/点云）
- 输出：代价地图（255=障碍，0=自由，1-253=代价梯度）
- 更新频率：通常 0.5-2 Hz（全局地图不需要实时更新）

### 4.3 规划器插件加载

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]     # 可配置多个规划器
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # 或 SmacPlanner 等
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

---

## 五、controller_server 详解

### 5.1 角色

接收 `FollowPath` Action（含路径），执行**路径跟踪控制**，以固定频率（默认 20 Hz）发布 `/cmd_vel`。

### 5.2 局部 Costmap

controller_server 内部维护 **local_costmap**：
- 范围：以机器人为中心的小区域（通常 3-5m × 3-5m）
- 更新频率：10-20 Hz（需要实时避障）
- 传感器：激光雷达 / 深度相机 / 超声波

### 5.3 Goal Checker 与 Progress Checker

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0         # Hz
    # 目标到达检测
    goal_checker_plugins: ["general_goal_checker"]
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25          # m
      yaw_goal_tolerance: 0.25         # rad
    # 进度监测（卡住检测）
    progress_checker_plugin: "progress_checker"
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5   # m，每 10s 至少移动 0.5m
      movement_time_allowance: 10.0   # s
```

---

## 六、behavior_server 详解

behavior_server 提供**恢复行为**，当导航卡住时由 BT 的 RecoveryNode 触发：

| 内置行为 | 插件 | 说明 |
|---------|------|------|
| `spin` | `nav2_behaviors/Spin` | 原地旋转，尝试重新定位 |
| `back_up` | `nav2_behaviors/BackUp` | 后退，逃离局部极小值 |
| `drive_on_heading` | `nav2_behaviors/DriveOnHeading` | 沿当前朝向直线运动 |
| `wait` | `nav2_behaviors/Wait` | 等待指定时间（等待动态障碍离开）|
| `assisted_teleop` | `nav2_behaviors/AssistedTeleop` | 接管手柄输入（安全辅助）|

---

## 七、smoother_server 详解

smoothe_server 接收规划器输出的"折线"路径，平滑为曲线：

| 插件 | 算法 | 特点 |
|------|------|------|
| `SimpleSmoother` | 迭代滑动平均 | 最快，轻微平滑 |
| `SavitzkyGolaySmoother` | Savitzky-Golay 滤波 | 保持曲率连续性 |
| `ConstrainedSmoother` | 约束优化（梯度下降）| 最高质量，适合高速场景 |

---

## 八、Nav2 全局参数（共享配置）

```yaml
# nav2_params.yaml 顶层结构
bt_navigator:
  ros__parameters: {...}

planner_server:
  ros__parameters:
    planner_plugins: [...]
    global_costmap:
      global_costmap:
        ros__parameters: {...}   # costmap 在服务器参数下配置

controller_server:
  ros__parameters:
    controller_plugins: [...]
    local_costmap:
      local_costmap:
        ros__parameters: {...}

# 所有节点共享 use_sim_time（通过 launch 统一设置）
```
