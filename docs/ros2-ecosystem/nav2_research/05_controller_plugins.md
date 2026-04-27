# 局部控制器插件对比 — DWB / MPPI / RPP

---

## 一、局部控制器概览

局部控制器（Local Controller）运行在 **controller_server** 中，以固定频率（默认 20 Hz）：
1. 读取当前机器人 pose（来自 TF / odom）
2. 读取全局路径（来自 planner_server）
3. 读取局部代价地图（实时障碍信息）
4. 输出 `/cmd_vel`（`geometry_msgs/TwistStamped`）

接口：`nav2_core::Controller`

```cpp
class Controller {
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * goal_checker) = 0;
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
};
```

---

## 二、DWB（Dynamic Window Approach B）

### 2.1 算法原理

DWB 是 ROS 1 `dwa_local_planner` 的 ROS 2 重构版本，基于**动态窗口法**：

```
1. 采样速度空间（在 [v_min, v_max] × [ω_min, ω_max] 内均匀采样）
2. 对每个采样速度，模拟机器人在 sim_time（如 1.7s）内的轨迹
3. 用多个 Critic 对每条轨迹打分（障碍距离、目标对齐、路径偏离等）
4. 选择得分最高的速度命令发布
```

### 2.2 Critic（评分插件）体系

DWB 的评分由一组 **Critic 插件**加权叠加：

| Critic | 功能 | 关键参数 |
|--------|------|---------|
| `GoalAlign` | 鼓励机器人朝向目标 | `scale: 24.0` |
| `PathAlign` | 鼓励机器人沿全局路径行进 | `scale: 32.0` |
| `GoalDist` | 最小化到目标的距离 | `scale: 24.0` |
| `PathDist` | 最小化到全局路径的偏离 | `scale: 32.0` |
| `ObstacleFootprint` | 检查轨迹是否碰撞（footprint 级别）| `scale: 0.01` |
| `Oscillation` | 惩罚来回振荡运动 | `scale: 1.0` |
| `BaseObstacle` | 轨迹到障碍物距离代价 | `scale: 0.02` |
| `TwirlingAction` | 惩罚原地旋转（目标附近）| `scale: 1.0` |

### 2.3 配置示例

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x: 0.0
      min_vel_y: 0.0                # 差速底盘 vy = 0
      max_vel_x: 0.26               # m/s，最大前进速度
      max_vel_y: 0.0
      max_vel_theta: 1.0            # rad/s，最大旋转速度
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5                # m/s²，加速度限制
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20                # 线速度采样数
      vy_samples: 5
      vtheta_samples: 20            # 角速度采样数
      sim_time: 1.7                 # s，轨迹模拟时间
      linear_granularity: 0.05      # m，轨迹点间距
      angular_granularity: 0.025    # rad
      transform_tolerance: 0.2      # s，TF 时间容忍
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: true
      critics:
        - RotateToGoal
        - Oscillation
        - BaseObstacle
        - GoalAlign
        - PathAlign
        - PathDist
        - GoalDist
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
```

### 2.4 DWB 适用场景

- 差速驱动机器人，室内中速导航（< 0.5 m/s）
- 需要灵活的评分策略定制（Critic 权重调整）
- 成熟稳定，大量生产部署案例

**缺点**：
- 采样效率低（O(n×m) 轨迹模拟，高速时性能差）
- 避障反应慢（sim_time 内预测，对突发障碍响应不及时）

---

## 三、MPPI（Model Predictive Path Integral）

### 3.1 算法原理

MPPI 是 Nav2 新一代控制器（Humble 引入），基于**随机轨迹优化**：

```
1. 在当前速度附近随机采样 K 条噪声轨迹（K 通常 1000-2000）
2. 每条轨迹模拟 T 步（如 56 步 × 0.05s = 2.8s 预测区间）
3. 对每条轨迹计算总代价（障碍 + 路径偏离 + 速度限制）
4. 用信息论加权平均（MPPI 公式）计算最优控制序列
5. 执行第一步，下一帧重新计算（Receding Horizon MPC）

GPU 加速：
  Nav2 MPPI 支持 GPU 并行化轨迹模拟（CUDA，Jetson 上显著加速）
```

### 3.2 Critic 体系（MPPI 的代价函数）

| Critic | 功能 |
|--------|------|
| `ObstaclesCritic` | 障碍物代价（Costmap 值直接作为代价）|
| `GoalCritic` | 到达目标的代价（仅最终状态）|
| `GoalAngleCritic` | 目标朝向对齐代价 |
| `PathAlignCritic` | 路径对齐代价 |
| `PathFollowCritic` | 路径跟踪（路径点距离）|
| `PreferForwardCritic` | 惩罚倒车运动 |
| `TwirlingCritic` | 惩罚旋转（提高直线稳定性）|
| `VelocityCritic` | 速度平滑（惩罚速度突变）|

### 3.3 配置示例

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  time_steps: 56              # 预测步数
  model_dt: 0.05              # s，每步时间（预测区间 = 56 × 0.05 = 2.8s）
  batch_size: 2000            # 随机轨迹数量（越多越好但更慢）
  vx_std: 0.2                 # 线速度噪声标准差
  vy_std: 0.0                 # 差速底盘 vy 噪声 = 0
  wz_std: 0.4                 # 角速度噪声标准差
  vx_max: 0.5                 # m/s，最大速度
  vx_min: -0.35               # m/s，允许倒车
  vy_max: 0.0
  wz_max: 1.9
  iteration_count: 1          # MPPI 迭代次数（>1 收敛更好但更慢）
  prune_distance: 1.7         # m，路径修剪距离（跳过已通过的路径点）
  transform_tolerance: 0.1
  temperature: 0.3            # MPPI 温度参数（越低越deterministic）
  gamma: 0.015                # 折扣因子
  motion_model: "DiffDrive"   # DiffDrive / Omni / Ackermann
  visualize: false            # 是否可视化轨迹（调试用）
  critics:
    ["ConstraintCritic", "GoalCritic", "GoalAngleCritic",
     "ObstaclesCritic", "PathAlignCritic", "PathFollowCritic",
     "PreferForwardCritic", "TwirlingCritic"]
  ObstaclesCritic:
    critical_weight: 20.0
    repulsion_weight: 1.5
    collision_cost: 10000.0   # 碰撞代价（极大值）
  PathAlignCritic:
    use_path_orientations: true
```

### 3.4 MPPI 适用场景

- **高速场景**（0.5-2 m/s），需要长预测区间避障
- 狭窄环境中的精细避障
- 有 GPU（Jetson AGX Orin）时性能显著提升
- 动态障碍场景（MPPI 的随机采样对动态障碍更鲁棒）

---

## 四、RPP（Regulated Pure Pursuit）

### 4.1 算法原理

RPP 是传统 Pure Pursuit 的改进版：

```
纯追踪（Pure Pursuit）：
  - 在路径上选一个"前视点"（look-ahead point，距离 = lookahead_distance）
  - 计算曲率 κ，转换为转角/角速度命令
  
RPP 改进：
  1. 速度调节：靠近障碍时降速（Regulated = 调节）
  2. 碰撞检测：检查前视路径是否碰撞，若碰撞则降速或停止
  3. 目标对齐：到达目标点后旋转对齐朝向
```

### 4.2 配置示例

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.5          # m/s，期望速度
  lookahead_dist: 0.6              # m，前视距离（初始）
  min_lookahead_dist: 0.3          # m，最小前视距离
  max_lookahead_dist: 0.9          # m，最大前视距离
  lookahead_time: 1.5              # s，动态前视距离 = v × lookahead_time
  rotate_to_heading_angular_vel: 1.8  # rad/s，对齐目标朝向的旋转速度
  transform_tolerance: 0.1
  use_velocity_scaled_lookahead_dist: false  # 是否用速度缩放前视距离
  # 速度调节（靠近障碍降速）
  use_regulated_linear_velocity_scaling: true
  use_cost_regulated_linear_velocity_scaling: false
  regulated_linear_scaling_min_radius: 0.9  # m，转弯半径<此值时降速
  regulated_linear_scaling_min_speed: 0.25  # m/s，降速下限
  # 碰撞检测
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 1.0  # s，碰撞时间阈值
```

### 4.3 RPP 适用场景

- 计算资源受限（Raspberry Pi 等低功耗平台）
- 路径跟踪精度要求高（如沿固定路线行驶）
- 速度较慢、障碍较少的室内环境
- 调参简单（参数语义清晰）

---

## 五、三大控制器横向对比

| 特性 | DWB | MPPI | RPP |
|------|:---:|:---:|:---:|
| 计算复杂度 | 中（CPU）| 高（GPU 加速）| 低 |
| 避障性能 | 中（局部最优）| 强（全局随机采样）| 中（碰撞检测降速）|
| 高速场景 | ❌（采样效率低）| ✅ | ⭕（简单但稳定）|
| 动态障碍 | ⭕ | ✅（随机采样更鲁棒）| ❌（反应慢）|
| 路径跟踪精度 | 中 | 高 | 高 |
| 调参难度 | 高（Critic 权重多）| 中（Critic 语义清晰）| 低（参数少）|
| Jetson 优化 | ❌ | ✅（CUDA 并行）| ❌ |
| 成熟度 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |

---

## 六、选型建议（本仓库 chassis_protocol）

| 场景 | 推荐控制器 | 理由 |
|------|-----------|------|
| 室内低速（< 0.3 m/s），传统差速机器人 | **DWB** | 成熟稳定，大量室内案例 |
| 室内/室外中高速（0.3-1 m/s），Jetson AGX Orin | **MPPI** | GPU 加速，避障鲁棒 |
| 固定路线巡逻，资源受限 | **RPP** | 低计算开销，精准跟踪 |

对于本仓库的移动操作平台（chassis_protocol 差速底盘 + 操作臂）：
- **首选 MPPI**（Jetson AGX Orin 有 GPU，动态障碍多，速度较高）
- **调试期用 DWB**（参数语义直观，出问题容易诊断）
