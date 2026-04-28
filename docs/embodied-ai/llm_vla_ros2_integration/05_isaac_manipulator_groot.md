# NVIDIA Isaac Manipulator 与 GROOT 工作流编排

> 参考：https://developer.nvidia.com/isaac/manipulator  
> GROOT：https://developer.nvidia.com/isaac/groot

---

## 一、Isaac Manipulator 概述

**Isaac Manipulator** 是 NVIDIA 推出的端到端机械臂操作工作流，整合了：
- cuMotion（GPU 加速运动规划）
- FoundationPose（6-DoF 物体位姿估计）
- ESdf（来自 Nvblox 的环境感知地图）
- Isaac ROS NITROS（零拷贝感知管线）

### 1.1 核心组件架构

```
┌──────────────────────────────────────────────────────────────────┐
│                Isaac Manipulator 工作流                           │
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐   │
│  │ FoundationPose│    │   Nvblox     │    │  cuMotion        │   │
│  │（物体位姿估计）│    │（ESDF 地图）  │    │（GPU 运动规划）   │   │
│  └──────┬───────┘    └──────┬───────┘    └────────┬─────────┘   │
│         │                   │                      │             │
│         └───────────────────┴──────────────────────┘             │
│                             │                                    │
│                    MoveIt 2（规划请求）                           │
│                             │                                    │
│                    ros2_control（执行）                           │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 cuMotion — GPU 加速运动规划

cuMotion 是 Isaac Manipulator 的核心规划器，作为 **MoveIt 2 规划器插件**接入：

```
传统 OMPL 规划（CPU）：
  → 随机采样 + 碰撞检测（FCL，串行）
  → 典型规划时间：100ms - 2s（复杂场景）

cuMotion（GPU）：
  → GPU 并行化碰撞检测（数千个采样点同时检测）
  → 基于 ESDF 地图（Nvblox 提供）的梯度规划
  → 典型规划时间：10-50ms（包含 ESDF 更新）
```

**MoveIt 2 集成**：

```yaml
# moveit_config/config/planning_pipeline.yaml
planning_plugins:
  - name: cumotion
    type: nvidia::isaac::cumotion::CumotionPlannerManager
    
# cuMotion 参数
cumotion:
  robot:
    urdf: /workspaces/isaac_ros-dev/src/my_robot/urdf/robot.urdf
  world:
    # Nvblox ESDF 地图作为碰撞世界
    esdf_service: /nvblox_node/get_esdf_and_gradient
  planning:
    time_limit: 0.05      # 50ms 规划预算
    interpolation_dt: 0.1 # 轨迹插值步长
```

---

## 二、FoundationPose — 零样本 6-DoF 位姿估计

FoundationPose 是 NVIDIA 的通用物体位姿估计模型，**无需特定物体训练**：

### 2.1 工作原理

```
输入：
  - RGB 图像（NitrosImage）
  - 深度图（NitrosDepthImage）
  - 物体 3D 模型（.obj 或 .ply 网格文件）

输出：
  - 物体位姿（PoseWithCovarianceStamped）
  - 追踪置信度

模式：
  1. 初始化：从 RGBD + 网格文件估计初始位姿（较慢，~500ms）
  2. 追踪：从上一帧位姿出发的精细化（快，~15ms @ Jetson AGX Orin）
```

### 2.2 ROS 2 接口

```
订阅：
  /rgb/image_rect_color (NitrosImage)
  /depth/image_rect_raw (NitrosImage)
  /camera_info (NitrosCameraInfo)
  /foundation_pose/mesh_file_path (std_msgs/String, 触发重初始化)

发布：
  /foundation_pose/pose_estimation/output (PoseArray)
  /foundation_pose/pose_estimation/debug_image (NitrosImage)  ← 可视化
```

### 2.3 与 Isaac Manipulator 的集成

```python
# launch：Isaac Manipulator 感知 + 规划 + 执行完整流程
launch_args = [
    # 物体模型文件路径
    DeclareLaunchArgument('mesh_file_path', default_value='/models/cup.obj'),
    # 目标物体类别（用于 pick planning）
    DeclareLaunchArgument('target_object', default_value='cup'),
]

manipulator_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('isaac_ros_foundationpose'), '/launch/foundationpose.launch.py']),
    launch_arguments={
        'mesh_file_path': LaunchConfiguration('mesh_file_path'),
    }.items()
)

# Nvblox（环境 3D 重建）
nvblox_bringup = IncludeLaunchDescription(...)

# MoveIt 2 + cuMotion
moveit_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare('my_robot_moveit'), '/launch/move_group.launch.py']),
    launch_arguments={
        'planning_pipeline': 'cumotion',
    }.items()
)
```

---

## 三、GROOT — 技能工作流编排

**GROOT**（General Robot Operations and Operations Tool）是 NVIDIA 的机器人技能编排工具，定位是：

```
GROOT = 可视化 + 技能库管理 + 工作流编排 + 模仿学习数据采集
```

### 3.1 GROOT 的核心概念

| 概念 | 说明 |
|------|------|
| **技能（Skill）** | 一个可执行的原子操作（如 grasp, place, nav_to），有输入参数和成功/失败状态 |
| **工作流（Workflow）** | 技能的有序组合（类似行为树的上一层抽象）|
| **人机协作（Teleop）** | 通过 GROOT UI 进行遥操作，数据自动采集为训练集 |
| **技能库（Skill Library）** | 在 GROOT 中注册的所有可用技能（通过 ROS 2 Action 注册）|

### 3.2 GROOT 工作流 YAML 格式

```yaml
# groot_workflow.yaml
name: pick_and_place_cup
version: "1.0"

steps:
  - id: detect_cup
    skill: FoundationPoseDetect
    params:
      mesh_file: /models/cup.obj
    outputs:
      - name: cup_pose
        type: geometry_msgs/PoseStamped

  - id: plan_grasp
    skill: CuMotionPick
    params:
      target_object: "{detect_cup.cup_pose}"
      grasp_offset: [0, 0, 0.05]  # 5cm above object
    depends_on: [detect_cup]

  - id: execute_pick
    skill: ExecuteTrajectory
    params:
      trajectory: "{plan_grasp.trajectory}"
    depends_on: [plan_grasp]

  - id: navigate_to_place
    skill: Nav2Navigate
    params:
      goal: {x: 1.5, y: 0.0, z: 0.0, frame: "map"}
    depends_on: [execute_pick]

  - id: execute_place
    skill: CuMotionPlace
    params:
      place_pose: {x: 0.0, y: 0.0, z: 0.8, frame: "base_link"}
    depends_on: [navigate_to_place]
```

### 3.3 GROOT + LLM 的结合

GROOT 的技能库可以作为 LLM 的工具调用后端：

```python
# 将 GROOT 技能注册为 rai 工具
from rai.tools import GrootSkillTool

# GROOT 通过 ROS 2 Action 暴露技能执行接口
pick_skill = GrootSkillTool(
    name="pick_object",
    description="Use Isaac Manipulator to pick an object using FoundationPose + cuMotion",
    groot_skill_name="FoundationPosePick",
    groot_action_server="/groot/execute_skill",
    timeout=60.0
)

navigate_skill = GrootSkillTool(
    name="navigate_to",
    description="Navigate the mobile base to a named location or pose",
    groot_skill_name="Nav2Navigate",
    groot_action_server="/groot/execute_skill",
    timeout=120.0
)

# LLM 可以通过调用这些工具来触发 GROOT 技能
agent = RaiAgent(llm_client=..., tools=[pick_skill, navigate_skill, ...])
```

---

## 四、Isaac Manipulator 性能数据（Jetson AGX Orin）

| 功能 | 耗时 | 说明 |
|------|------|------|
| FoundationPose 初始化 | ~500ms | 首次检测，慢 |
| FoundationPose 追踪 | ~15ms | 从上一帧精化 |
| Nvblox ESDF 更新 | ~10ms/帧 | 增量更新 |
| cuMotion 规划 | 10-50ms | 含碰撞检测 |
| 完整 pick（感知→规划→执行） | ~2-5s | 含机械臂运动时间 |

---

## 五、与 `chassis_protocol` + 本仓库的对接

Isaac Manipulator 通常部署在**机械臂**场景，但对于本仓库的**移动机械臂（移动底盘 + 操作臂）**场景：

```
chassis_protocol（本仓库 HAL）
  ↕ ros2_control SystemInterface
底盘 DiffDriveController（/cmd_vel）
  ← Nav2 导航（Layer 2）
  ← GROOT navigate_to skill（Layer 2）
  ← LLM navigate_to 工具调用（Layer 4）

机械臂（7-DOF）
  ← FollowJointTrajectory（cuMotion 规划结果）
  ← VLA 直接控制（高频模式，VlaActionBridgeNode）
```

关键接口：`chassis_protocol` 暴露的 `cmd_vel` 和 `joint_states` 必须与 Nav2 和 MoveIt 2 的 QoS 兼容（BEST_EFFORT + KEEP_LAST=1 for cmd_vel，RELIABLE for trajectory execution）。
