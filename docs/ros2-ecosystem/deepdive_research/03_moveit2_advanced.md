# MoveIt 2 高阶特性深化

> 深化研究 Phase 4 中留下的高阶问题：bio_ik 求解器、STOMP 随机优化、碰撞矩阵优化、Grasp Pose Detection 集成与力控装配。

---

## 一、bio_ik 求解器（PSO 遗传算法）

### 1.1 bio_ik 原理

`bio_ik` 是基于生物启发优化（Particle Swarm + 遗传算法）的逆运动学求解器，通过最小化代价函数（CartesianGoal + 关节限制 + 自定义目标）求解 IK，支持超冗余自由度（6+ DoF）和多末端执行器。

**相比 KDL 的优势**：

| 指标 | KDL | bio_ik |
|------|-----|--------|
| 求解方式 | 迭代 Jacobian 伪逆 | PSO + 遗传算法 |
| 冗余 DoF 处理 | 差 | 可配置关节优先级 |
| 多目标（位置 + 朝向 + 肘偏好） | 不支持 | 支持 |
| 无解时行为 | 发散 | 返回最优近似解 |
| 速度（7-DoF，Humble） | ~0.1ms | ~2ms（可 trade-off） |

### 1.2 配置 bio_ik

```bash
# 安装
sudo apt install ros-humble-bio-ik
```

```yaml
# kinematics.yaml（MoveIt 2 配置）
arm:
  kinematics_solver: bio_ik/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.02      # 20ms budget
  kinematics_solver_attempts: 3

  # bio_ik 特有参数
  bio_ik:
    mode: gd_full                      # gradient_descent_full（更快）或 pso（更鲁棒）
    memetic_iterations: 4              # PSO × 遗传的迭代轮数
    initial_guess_mode: CURRENT_STATE  # 从当前状态开始，避免大跳跃
```

### 1.3 自定义 IK 目标（多目标优化）

```cpp
#include <bio_ik/bio_ik.h>

// 在 MoveIt 2 Planning Scene 中使用 bio_ik 多目标
bio_ik::BioIKKinematicsQueryOptions ik_options;
ik_options.replace = true;
ik_options.return_approximate_solution = true;  // 无精确解时返回最优近似

// 主要目标：末端执行器位姿
ik_options.goals.emplace_back(
  std::make_unique<bio_ik::PoseGoal>(
    "end_effector",
    target_position,
    target_orientation));

// 次要目标：偏好手肘向上（避免奇异构型）
ik_options.goals.emplace_back(
  std::make_unique<bio_ik::MinimalDisplacementGoal>(0.1));  // 最小化关节运动

// 约束：特定关节保持接近 0
ik_options.goals.emplace_back(
  std::make_unique<bio_ik::JointVariableGoal>(
    "wrist_roll_joint", 0.0, 0.5));  // 目标值, 权重

// 调用 IK
move_group.setJointValueTarget(target_pose, "end_effector", ik_options);
```

---

## 二、STOMP 随机优化规划器

### 2.1 何时使用 STOMP

**STOMP（Stochastic Trajectory Optimization for Motion Planning）**：通过对参考轨迹加高斯噪声、评估代价、更新参数的方式优化轨迹。

**使用场景**：
- 轨迹需要满足高阶代价（如末端执行器尽量保持水平、关节力矩最小化）
- OMPL 路径虽然无碰撞但轨迹不平滑
- 需要 post-process 优化已有路径

```
STOMP 工作流：

初始轨迹（来自 OMPL 或直线插值）
    │
    ▼
for N iterations:
  ① 生成 K 条扰动轨迹：θ_k = θ + ε_k，ε_k ~ N(0, R^-1)
  ② 评估每条轨迹代价：S_k（碰撞 + 平滑度 + 自定义）
  ③ 计算改进量：Δθ = R^-1 Σ_k P_k ε_k（P_k ∝ exp(-S_k/λ)）
  ④ 更新：θ ← θ + α Δθ
    │
    ▼
平滑、低代价的最优轨迹
```

### 2.2 STOMP 配置

```yaml
# ompl_planning.yaml — 同时配置 STOMP 作为 Post-processor
planning_pipelines:
  pipeline_names: ["ompl", "stomp"]

stomp:
  planning_plugins: ['stomp_moveit/StompPlanner']
  request_adapters:
    - default_planning_request_adapters/ResolveConstraintFrames
    - default_planning_request_adapters/ValidateWorkspaceBounds
  
  StompPlanner:
    num_timesteps: 60
    num_iterations: 50               # 优化迭代次数
    num_iterations_after_valid: 0
    num_rollouts: 30                 # 每次迭代的扰动轨迹数
    max_rollouts: 30
    exponentiated_cost_sensitivity: 0.5
    
    cost_functions:
      - class: stomp_moveit/CollisionCostFunction
        collision_penalty: 1.0
        cost_weight: 0.1
      - class: stomp_moveit/SmoothnessCostFunction
        cost_weight: 1.0
```

### 2.3 代码中选择规划器

```cpp
// 对于通常规划，使用 OMPL（快速）
move_group.setPlanningPipelineId("ompl");
move_group.setPlannerId("RRTConnect");

// 对于需要高质量轨迹（如精密装配路径），使用 STOMP
move_group.setPlanningPipelineId("stomp");
```

---

## 三、碰撞矩阵优化（ACM）

### 3.1 问题

MoveIt 2 的 `AllowedCollisionMatrix`（ACM）决定哪些 link pair 需要检查碰撞。对于复杂机器人（10+ link），默认 ACM 可能：
- 漏禁用永不碰撞的 pair → 不必要的碰撞检查开销
- 误禁用实际可能碰撞的 pair → 安全隐患

### 3.2 SRDF 中配置 ACM

```xml
<!-- robot.srdf -->
<robot name="embodied_robot">
  <!-- 永不碰撞的 link pair（连接关节的相邻 link） -->
  <disable_collisions link1="base_link" link2="chassis_link" reason="Adjacent"/>
  <disable_collisions link1="arm_link1" link2="arm_link2" reason="Adjacent"/>
  
  <!-- 几何上永不接触 -->
  <disable_collisions link1="base_link" link2="arm_link5" reason="Never"/>
  <disable_collisions link1="left_wheel" link2="arm_link1" reason="Never"/>
  
  <!-- 工具坐标系间（夹爪自身 link 之间） -->
  <disable_collisions link1="gripper_left_finger" link2="gripper_right_finger"
                      reason="Default"/>
</robot>
```

### 3.3 动态 ACM（避免传送带/工件碰撞误报）

```cpp
// 在抓取前临时允许末端执行器与目标物体碰撞
collision_detection::AllowedCollisionMatrix acm =
  planning_scene_->getAllowedCollisionMatrix();
acm.setEntry("gripper_left_finger", "target_object", true);
acm.setEntry("gripper_right_finger", "target_object", true);
planning_scene_->setAllowedCollisionMatrix(acm);

// 规划抓取轨迹
// ... plan and execute ...

// 恢复（抓取完成后）
acm.removeEntry("gripper_left_finger", "target_object");
planning_scene_->setAllowedCollisionMatrix(acm);
```

---

## 四、Grasp Pose Detection

### 4.1 GraspNet 集成

`moveit2_grasps` + 深度学习检测器（GraspNet-1Billion / AnyGrasp）构建端到端抓取流程：

```
流程：

RGB-D 图像
    │
    ▼
FoundationPose / AnyGrasp（GPU节点）
    │ 输出：GraspArray（位姿 + 质量分数）
    ▼
MoveIt2 GraspFilter
  ├── 运动学可达性过滤
  ├── 碰撞检查
  └── 排序（IK 成功率 × 抓取质量分数）
    │
    ▼
GraspExecutor
  ├── 生成预抓取轨迹（pre-grasp → grasp → post-grasp）
  └── 执行（包含力控）
```

### 4.2 GraspCandidate 数据结构

```cpp
// moveit2_grasps 核心数据类型
moveit_grasps::GraspCandidatePtr candidate =
  std::make_shared<moveit_grasps::GraspCandidate>(
    grasp_msg,     // geometry_msgs::msg::PoseStamped — 抓取姿态
    grasp_data_,   // 夹爪参数（指尖尺寸、抓取深度）
    approach_direction  // 接近方向
  );

// 过滤：检查 IK 可达性
std::vector<moveit_grasps::GraspCandidatePtr> valid_grasps;
grasp_filter_->filterGrasps(
  candidates,
  valid_grasps,
  planning_scene_monitor_,
  arm_jmg_,
  current_state);

// 执行最优抓取
grasp_executor_->pick(
  valid_grasps[0],         // 最优抓取候选
  move_group_arm_,
  move_group_gripper_);
```

---

## 五、力控装配（Impedance Control）

### 5.1 场景与需求

力控装配（Force-Controlled Assembly）用于：
- 轴孔装配（Peg-in-Hole）：容许 < 1mm 位置误差
- 连接器插拔（USB / RJ45）
- 螺钉拧紧（力矩控制终止）

**核心挑战**：如何在接触过程中将位置偏差转换为柔顺力（阻抗控制）。

### 5.2 Cartesian 阻抗控制（基于 ros2_control）

```cpp
// impedance_controller.hpp
class ImpedanceController : public controller_interface::ControllerInterface
{
  // 阻抗控制律：F_ext → Δx 修正
  Eigen::VectorXd computeImpedanceCorrection(
    const Eigen::VectorXd& f_ext,    // 外力/力矩（来自 FT 传感器）
    const Eigen::VectorXd& x_desired // 目标位姿
  ) {
    // 阻抗模型：M·x'' + D·x' + K·Δx = F_ext
    // 稳态（低频）近似：Δx = K^-1 · F_ext
    Eigen::VectorXd delta_x = K_inv_ * f_ext;
    return x_desired + delta_x;
  }

  controller_interface::return_type
  update(const rclcpp::Time&, const rclcpp::Duration& dt) override
  {
    // 读取 FT 传感器
    Eigen::VectorXd f_ext = ft_sensor_->get_wrench();

    // 接触检测：外力超阈值时切换到力控模式
    if (f_ext.norm() > force_threshold_) {
      auto x_corrected = computeImpedanceCorrection(f_ext, x_goal_);
      sendCartesianCommand(x_corrected);
    } else {
      // 纯位置控制
      sendCartesianCommand(x_goal_);
    }
    return controller_interface::return_type::OK;
  }

private:
  Eigen::Matrix6d K_inv_;            // 刚度矩阵的逆
  double force_threshold_ = 5.0;    // N，接触检测阈值
  std::unique_ptr<semantic_components::ForceTorqueSensor> ft_sensor_;
};
```

### 5.3 MoveIt 2 Force-Torque Sensor 集成

```yaml
# moveit_config/sensors_3d.yaml
sensors:
  - sensor_plugin: moveit_ros_perception/PointCloudOctomapUpdater
    point_cloud_topic: /point_cloud

# FT 传感器通过 ros2_control Semantic Component 读取
# MoveIt 2 规划使用力控约束

planning_scene_monitor_options:
  publish_robot_description_semantic: true

# 在 move_group 中允许力控规划
move_group:
  capabilities:
    - move_group/ApplyPlanningSceneService
    - move_group/CartesianPathService
```

### 5.4 轴孔装配 BT 序列

```xml
<!-- peg_in_hole_bt.xml -->
<BehaviorTree ID="PegInHole">
  <Sequence>
    <!-- 1. 移动到孔上方 5cm -->
    <MoveToPose goal="{pre_insert_pose}"/>
    
    <!-- 2. 慢速下压直到接触（力超阈值） -->
    <ForceControlledApproach
      direction="z_down"
      contact_force_threshold="3.0"
      max_distance="0.1"/>
    
    <!-- 3. 螺旋搜索（补偿位置误差） -->
    <SpiralSearch
      radius="0.003"
      force_threshold="10.0"
      success_force="2.0"/>
    
    <!-- 4. 垂直插入（阻抗控制） -->
    <ImpedanceControlledInsert
      target_depth="0.02"
      insertion_force="8.0"
      stiffness_z="500"/>
    
    <!-- 5. 验证装配成功（力矩稳定 + 位置到达） -->
    <VerifyAssembly force_threshold="2.0" position_tolerance="0.001"/>
  </Sequence>
</BehaviorTree>
```
