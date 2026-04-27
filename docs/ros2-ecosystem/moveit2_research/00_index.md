# MoveIt 2 架构调研 — 总索引

> **调研范围**：MoveIt 2 完整架构——move_group 插件体系、MoveIt Servo 实时控制、Hybrid Planning、planning_scene_monitor 实时性、ros2_control 耦合点、VLA 输出对接、moveit_py API  
> **关联文档**：  
> - `docs/ros2-ecosystem/ros2_control_research/`（Phase 1，FollowJointTrajectory 接口端）  
> - `docs/embodied-ai/llm_vla_ros2_integration/`（Phase 3，VLA 输出 → MoveIt 桥接）  
> - MoveIt 2 官方文档：https://moveit.picknik.ai

---

## 一、MoveIt 2 整体架构图

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         用户 / 上层应用                                       │
│  moveit_py / MoveGroupInterface(C++) / RViz Motion Planning Panel            │
│  LLM Agent(rai) / VLA Bridge Node                                            │
└────────────────────────────┬─────────────────────────────────────────────────┘
                             │ Action / Service / Topic
┌────────────────────────────▼─────────────────────────────────────────────────┐
│  move_group 节点（核心协调者）                                                 │
│                                                                              │
│  ┌──────────────────┐  ┌─────────────────────┐  ┌──────────────────────┐   │
│  │  Planning Pipeline│  │ Planning Scene Monitor│  │  Trajectory Execution│  │
│  │  ├─ OMPL          │  │  ├─ /robot_description│  │  Manager             │  │
│  │  ├─ Pilz          │  │  ├─ /joint_states     │  │  ├─ FollowJointTraj  │  │
│  │  └─ STOMP         │  │  ├─ /tf              │  │  └─ GripperCommand   │  │
│  └──────────────────┘  │  └─ perception topic  │  └──────────────────────┘  │
│                         └─────────────────────┘                             │
│  ┌──────────────────┐  ┌─────────────────────┐                              │
│  │  IK Solver Plugin│  │  Collision Checking  │                              │
│  │  ├─ KDL           │  │  ├─ FCL（凸包）       │                              │
│  │  ├─ TracIK        │  │  └─ Bullet（可选）    │                              │
│  │  └─ bio_ik        │  └─────────────────────┘                             │
│  └──────────────────┘                                                       │
└─────────────────────┬────────────────────────────────────────────────────────┘
                      │ FollowJointTrajectory Action
┌─────────────────────▼────────────────────────────────────────────────────────┐
│  ros2_control（控制层）                                                       │
│  JointTrajectoryController ← FollowJointTrajectory                          │
│  → Hardware Interface（SystemInterface）                                    │
└──────────────────────────────────────────────────────────────────────────────┘

实时增量控制通道（绕过规划）：
┌──────────────────────────────┐
│  MoveIt Servo                │
│  /servo/delta_twist_cmds     │ ← VLA 输出 / joystick / 视觉伺服
│  /servo/joint_jog            │
│  → JointGroupVelocityController (ros2_control)
└──────────────────────────────┘
```

---

## 二、核心消息流（MoveGroup 规划 + 执行）

```
1. 客户端发送目标：
   MoveGroupInterface::setPoseTarget(target_pose) 或
   MoveGroupInterface::setJointValueTarget(joint_values)

2. move_group 构造 MotionPlanRequest：
   MotionPlanRequest{
     group_name: "arm",
     goal_constraints: [position_constraint, orientation_constraint],
     num_planning_attempts: 10,
     allowed_planning_time: 5.0,
     planner_id: "RRTConnect"
   }

3. Planning Pipeline（OMPL/Pilz/STOMP）求解：
   MotionPlanRequest → MotionPlanResponse{trajectory: RobotTrajectory}

4. 轨迹后处理（可选）：
   → TimeParameterization（速度/加速度限制）
   → 碰撞检查（FCL）

5. Trajectory Execution Manager 执行：
   → FollowJointTrajectory Action Goal → ros2_control JointTrajectoryController
   → 关节运动 → 硬件

6. 反馈：
   /joint_states → planning_scene_monitor → 实时更新碰撞场景
```

---

## 三、文档目录

| 文件 | 内容 |
|------|------|
| [01_move_group_plugins.md](./01_move_group_plugins.md) | Planner(OMPL/Pilz/STOMP) + IK(KDL/TracIK/bio_ik) + Collision(FCL) + 传感器集成 |
| [02_moveit_servo.md](./02_moveit_servo.md) | 实时增量控制架构、奇异点处理、Jog 模式、VLA 对接 |
| [03_hybrid_planning.md](./03_hybrid_planning.md) | Global + Local Planner 协同框架，适用于动态障碍场景 |
| [04_planning_scene_monitor.md](./04_planning_scene_monitor.md) | 实时碰撞场景维护、性能瓶颈与优化 |
| [05_ros2_control_coupling.md](./05_ros2_control_coupling.md) | FollowJointTrajectory 对接详解、控制器切换、QoS |
| [06_vla_integration.md](./06_vla_integration.md) | VLA(Phase 3) 输出 → MoveIt 对接：pose_goal / joint_goal / cartesian_path / Servo |
| [07_moveit_py.md](./07_moveit_py.md) | moveit_py Python API：规划、执行、场景管理 |

---

## 四、MoveIt 2 关键配置文件组织（MoveIt Setup Assistant 生成）

```
my_robot_moveit_config/
├── config/
│   ├── my_robot.srdf             ← 关节组、末端执行器、碰撞矩阵
│   ├── kinematics.yaml           ← IK 插件配置（per 关节组）
│   ├── ompl_planning.yaml        ← OMPL 规划器参数
│   ├── pilz_industrial_motion_planner_planning.yaml
│   ├── joint_limits.yaml         ← 速度/加速度限制
│   ├── moveit_controllers.yaml   ← 与 ros2_control 控制器的映射
│   └── sensors_3d.yaml           ← 深度相机点云集成
├── launch/
│   ├── move_group.launch.py      ← 启动 move_group 节点
│   ├── moveit_rviz.launch.py     ← RViz + Motion Planning Panel
│   └── demo.launch.py            ← 完整演示（含 ros2_control fake）
└── CMakeLists.txt
```

---

## 五、与本仓库其他模块的交叉引用

| 已有工作 | 与 MoveIt 2 的关联 |
|---------|-----------------|
| `chassis_protocol` HAL（本仓库）| 移动底盘 + 操作臂的统一 ros2_control 栈；mobile manipulation 场景中 chassis 提供底盘 odom，MoveIt 规划臂的运动 |
| `ros2_control_research/`（Phase 1）| `JointTrajectoryController` = MoveIt 和硬件之间的执行层；`JointGroupVelocityController` = MoveIt Servo 的目标控制器 |
| `llm_vla_ros2_integration/`（Phase 3）| VLA 输出（EE Pose Delta 7D / Joint Delta）→ MoveIt Servo（实时）或 MoveGroupInterface（离散）|
| `nav2_research/`（Phase 5）| mobile manipulation：Nav2 负责底盘导航到操作位，MoveIt 负责臂的规划执行 |
