# move_group 插件体系 — Planner / IK / Collision / Sensor

---

## 一、move_group 插件架构概览

move_group 是 MoveIt 2 的**核心协调节点**，其所有关键功能均通过 `pluginlib` 插件加载，可在不修改核心代码的情况下替换：

```
move_group
├── PlannerPlugin（规划算法）         ← ompl_interface / pilz / stomp
├── KinematicsPlugin（IK 求解）       ← kdl / trac_ik / bio_ik（per 关节组）
├── CollisionPlugin（碰撞检测）        ← fcl（默认） / bullet
├── ConstraintSamplerPlugin（约束采样）
└── PlanningRequestAdapterPlugin（轨迹后处理链）
    ├── AddTimeParameterization
    ├── FixStartStateBounds
    ├── FixWorkspaceBounds
    └── ResolveConstraintFrames
```

---

## 二、规划器插件（Planner Plugin）

### 2.1 OMPL（Open Motion Planning Library）— 默认规划器

**特点**：采样式运动规划，不需要梯度信息，适合高维关节空间。

#### 常用算法对比

| 算法 | 类型 | 特点 | 适用场景 |
|------|------|------|---------|
| `RRTConnect` | 双向 RRT | 最快，默认选项 | 通用，快速原型 |
| `RRT*` | 渐近最优 RRT | 路径质量最优，慢 | 对路径长度有要求 |
| `PRM` / `PRMstar` | 概率路图 | 离线预计算，多查询高效 | 固定环境多次查询 |
| `BKPIECE` / `LBKPIECE` | 基于投影的 KPIECE | 对关节空间不均匀采样效率高 | 复杂关节约束 |
| `EST` | 扩展空间树 | 对分散空间有优势 | 狭窄通道问题 |
| `SBL` | 单向 BL | 轻量 | 简单场景 |

#### OMPL 配置文件

```yaml
# ompl_planning.yaml
arm:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - RRT
    - RRTstar
    - TRRT
    - EST
    - LBKPIECE
    - BKPIECE
    - KPIECE
    - BiTRRT
    - PDST
    - STRIDE
    - BiEST
    - ProjEST
    - LazyPRM
    - LazyPRMstar
    - SPARS
    - SPARStwo
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0          # 最大扩展步长（0=自动）
  RRTstar:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05     # 直接采样目标的概率
    delay_collision_checking: 1
  # 关节组级别参数
  longest_valid_segment_fraction: 0.005  # 碰撞检测精度（越小越精确但越慢）
  projection_evaluator: joints(joint1,joint2)  # 投影用于 KPIECE 类算法

# 全局规划参数（所有规划器）
planning_plugin: ompl_interface/OMPLPlanner
request_adapters:
  - default_planner_request_adapters/AddTimeOptimalParameterization
  - default_planner_request_adapters/ResolveConstraintFrames
  - default_planner_request_adapters/FixWorkspaceBounds
  - default_planner_request_adapters/FixStartStateBounds
  - default_planner_request_adapters/FixStartStateCollision
```

### 2.2 Pilz Industrial Motion Planner — 工业轨迹

**特点**：生成**确定性**、工业标准的轨迹（不是采样式），速度快，支持直线/圆弧。

| 运动类型 | Pilz 指令 | 描述 |
|---------|----------|------|
| `PTP` (Point-to-Point) | `pilz_industrial_motion_planner/PTP` | 关节空间点到点，梯形速度曲线 |
| `LIN` (Linear) | `pilz_industrial_motion_planner/LIN` | 笛卡尔空间直线运动（末端走直线）|
| `CIRC` (Circular) | `pilz_industrial_motion_planner/CIRC` | 笛卡尔空间圆弧运动 |
| `Sequence` | `pilz_industrial_motion_planner/sequence_service` | 多段运动无缝拼接（工业流程）|

```yaml
# pilz_industrial_motion_planner_planning.yaml
planning_plugin: pilz_industrial_motion_planner/CommandPlanner

# SRDF 中配置关节限制（Pilz 严格遵守）
# joint_limits.yaml 中的速度/加速度/jerk 限制必须正确
```

**Pilz 的优势场景**：
- 焊接、搬运等需要确定性轨迹的工业应用
- VLA 输出的末端直线运动（LIN 指令）
- 需要多段轨迹无缝连接（Sequence）

### 2.3 STOMP（Stochastic Trajectory Optimization）

**特点**：随机轨迹优化，可以优化**光滑性**和**代价函数**（如靠近障碍物的代价）。

```yaml
stomp_planner:
  num_timesteps: 60
  num_iterations: 40
  num_rollouts: 30              # 每次迭代的随机轨迹数
  max_rollouts: 60
  initialization_method: 1      # 1=LINEAR_INTERPOLATION
  control_cost_weight: 0.0
  task:
    noise_generator:
      class: stomp_moveit/NormalDistributionSampling
      stddev: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]  # 每个关节噪声
    cost_functions:
      - class: stomp_moveit/CollisionCostFunction
        collision_penalty: 1.0
        cost_weight: 1.0
        kernel_window_percentage: 0.2
        longest_valid_segment_fraction: 0.05
    noisy_filters:
      - class: stomp_moveit/JointLimitsFilter
      - class: stomp_moveit/MultiParentAveragingFilter
    update_filters:
      - class: stomp_moveit/PolynomialSmoothingFilter
```

---

## 三、IK 求解器插件（Kinematics Plugin）

IK 插件为每个**关节组**独立配置，在 `kinematics.yaml` 中设定：

### 3.1 KDL（默认，基于数值迭代）

```yaml
# kinematics.yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005  # rad，关节搜索步长
  kinematics_solver_timeout: 0.005            # s，单次 IK 超时
  kinematics_solver_attempts: 3               # 最大重试次数
```

- **算法**：Jacobian 伪逆迭代（数值 IK）
- **优点**：通用，支持任意 URDF 串联链
- **缺点**：速度慢（~5ms/求解），靠近奇异点时发散

### 3.2 TracIK（推荐替换 KDL）

```yaml
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
  solve_type: Speed        # Speed / Distance / Manip1 / Manip2
```

- **算法**：组合 SQP 优化 + 随机重启，显著优于 KDL
- **解类型**：
  - `Speed`：找到第一个解（最快）
  - `Distance`：最接近当前关节值的解（最自然）
  - `Manip1` / `Manip2`：最大化可操作度（远离奇异点）
- **性能**：比 KDL 快 2-3 倍，成功率高 ~25%

### 3.3 bio_ik（基于遗传算法/粒子群）

```yaml
arm:
  kinematics_solver: bio_ik/BioIKKinematicsPlugin
  kinematics_solver_timeout: 0.05     # 更长时间换取更好的解
  mode: bio2_memetic               # bio1 / bio2 / bio2_memetic（最优）
  position_scale: 1.0
  rotation_scale: 0.5
```

- **算法**：粒子群优化（PSO）+ 记忆进化（Memetic）
- **优点**：
  - 支持**冗余自由度优化**（7+轴机械臂）
  - 可自定义目标函数（如避开障碍、最大化可操作度）
  - 对复杂约束（多末端执行器）有独特优势
- **缺点**：比 TracIK 慢（推荐 timeout >= 0.02s）

#### IK 插件选型建议

| 场景 | 推荐 |
|------|------|
| 6 轴工业臂，快速在线 IK | **TracIK**（Speed 模式）|
| 7 轴冗余臂，姿态优化 | **bio_ik**（Manip 目标）|
| 快速原型/调试 | KDL |
| 需要最接近当前关节值的解 | **TracIK**（Distance 模式）|

---

## 四、碰撞检测插件（Collision Plugin）

### 4.1 FCL（Flexible Collision Library）— 默认

```
碰撞对象表示：
  - 机器人链接：URDF 中的 collision mesh（凸包 or 精确网格）
  - 场景对象：Octomap（来自点云）+ 几何体（Box/Cylinder/Sphere/Mesh）
  - 碰撞矩阵（SRDF 中禁用永久不碰撞的链接对，加速检测）

FCL 性能指标：
  - 简单机器人（6轴，粗略 mesh）：~0.1ms/次
  - 复杂机器人（7轴，精细 mesh）：~1-5ms/次
  - Octomap 查询：~0.5-2ms（取决于分辨率）
```

### 4.2 碰撞矩阵配置（SRDF）

```xml
<!-- my_robot.srdf：禁用永久不碰撞的链接对 -->
<disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
<disable_collisions link1="base_link" link2="upper_arm_link" reason="Never"/>
```

**MoveIt Setup Assistant** 会自动生成碰撞矩阵（通过随机采样检测哪些对永远不碰撞）。

---

## 五、传感器集成（3D 感知）

MoveIt 可将深度相机点云集成到 planning scene（作为 Octomap）：

```yaml
# sensors_3d.yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /camera/depth/color/points
    max_range: 5.0              # m，点云最大使用距离
    padding_offset: 0.1         # m，机器人自身点云过滤（避免自碰撞误判）
    padding_scale: 1.0
    point_subsample: 1          # 点云降采样（1=全部使用）
    filtered_cloud_topic: /filtered_cloud  # 过滤后发布（可视化调试）
    # Octomap 分辨率（越小越精确但更慢）
    octomap_resolution: 0.05    # m
```

```yaml
# move_group 参数
octomap_resolution: 0.05
octomap_frame: world            # Octomap 坐标系（通常=map或world）
max_range: 5.0
```
