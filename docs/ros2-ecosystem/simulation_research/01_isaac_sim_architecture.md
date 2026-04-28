# Isaac Sim 架构深度调研

> **版本基准**：Isaac Sim 4.2 / Isaac Lab 1.2 / Isaac ROS 2.x（2024）  
> **关联文档**：
> - `docs/ros2-ecosystem/isaac_ros_nitros_research/`（Phase 2，NITROS GPU 零拷贝）
> - `docs/ros2-ecosystem/ros2_control_research/`（Phase 1，Hardware Interface）

---

## 一、基础架构：Omniverse 技术栈

Isaac Sim 构建在 NVIDIA Omniverse 平台之上，形成以下分层架构：

```
┌─────────────────────────────────────────────────────────────────┐
│                    Isaac Sim 应用层                               │
│  ├─ Isaac Lab（RL 训练框架）                                      │
│  ├─ Isaac ROS（ROS 2 集成扩展）                                   │
│  └─ 自定义 Python Extension                                      │
├─────────────────────────────────────────────────────────────────┤
│                    Omniverse Kit 框架                             │
│  ├─ Extension Manager（插件热加载）                               │
│  ├─ OmniGraph（可视化节点编程 / Action Graph）                    │
│  ├─ Python Script Engine（Python 3.10）                          │
│  └─ USD Composer（场景编辑器）                                    │
├─────────────────────────────────────────────────────────────────┤
│                    核心运行时层                                    │
│  ├─ USD（Universal Scene Description）— 场景图 / 资产描述         │
│  ├─ NVIDIA PhysX 5（GPU 加速刚体 / 关节 / 软体物理）              │
│  ├─ RTX Renderer（光线追踪 + DLSS + Denoising）                  │
│  └─ Carbonite SDK（底层平台抽象）                                  │
├─────────────────────────────────────────────────────────────────┤
│                    硬件 / 驱动层                                   │
│  ├─ NVIDIA RTX GPU（最低 RTX 3080，推荐 RTX 4090）               │
│  ├─ CUDA 12.x                                                    │
│  └─ OptiX 8.x（光线追踪后端）                                     │
└─────────────────────────────────────────────────────────────────┘
```

### 1.1 USD（Universal Scene Description）

USD 是 Isaac Sim 的核心场景描述格式，由 Pixar 开发、NVIDIA 扩展：

```python
# 典型 USD 场景操作示例
from pxr import Usd, UsdPhysics, UsdGeom

# 打开或创建场景
stage = Usd.Stage.Open("robot_scene.usd")

# 获取关节 Prim
joint_prim = stage.GetPrimAtPath("/World/Robot/base_link/joint_0")

# 配置物理属性
joint_api = UsdPhysics.RevoluteJoint(joint_prim)
joint_api.GetLowerLimitAttr().Set(-180.0)  # degrees
joint_api.GetUpperLimitAttr().Set(180.0)
```

**USD 关键概念**：
- **Prim**（Primitive）：场景图的基本节点（Mesh / Light / Camera / Joint 等）
- **Schema**：属性集合的类型定义（`UsdPhysics.RigidBodyAPI` / `UsdPhysics.RevoluteJoint`）
- **Layer**：USD 支持多层叠加（base layer + override layer），实现 Domain Randomization 非破坏性修改
- **References**：从外部 .usd 文件引用资产，实现资产库管理

### 1.2 PhysX 5 物理引擎

PhysX 5 相较 PhysX 4 的核心改进：

| 特性 | PhysX 4（旧版） | PhysX 5（Isaac Sim 4.x） |
|------|---------------|------------------------|
| **关节求解器** | TGS（Temporal Gauss-Seidel） | TGS + Articulation GPU Solver |
| **软体仿真** | 有限 | FEM 软体（衣物 / 可形变物体） |
| **GPU 并行** | 部分 | 完整 GPU 刚体 + 关节并行 |
| **接触稳定性** | 中等 | 改进摩擦锥 + 接触缓存 |
| **最大刚体数** | ~64K GPU | ~1M+ GPU |

### 1.3 RTX Renderer

Isaac Sim 支持三种渲染模式，影响传感器仿真质量：

| 渲染模式 | 说明 | 传感器质量 | 性能 |
|---------|------|----------|------|
| **RTX - Interactive** | 完整光线追踪，实时 DLSS | ⭐⭐⭐⭐⭐ | 低帧率（≈10-30 fps） |
| **RTX - Accurate** | Path Tracing，最高精度 | ⭐⭐⭐⭐⭐ | 极低帧率（离线用） |
| **RaytracedLighting** | 混合光栅+光追 | ⭐⭐⭐⭐ | 中等（≈30-60 fps） |

---

## 二、Isaac Sim ROS 2 集成

### 2.1 集成架构概览

```
Isaac Sim 进程                    ROS 2 DDS 域
┌─────────────────────────────┐   ┌──────────────────────────────┐
│  OmniGraph Action Graph     │   │                              │
│  ┌─────────────────────┐   │   │  /camera/image_raw           │
│  │ OgnROS2CameraHelper │──►├───►│  /joint_states              │
│  │ OgnROS2Publisher    │   │   │  /clock                      │
│  │ OgnROS2Subscriber   │◄──├───◄│  /cmd_vel                   │
│  └─────────────────────┘   │   │  /tf, /tf_static             │
│                             │   │                              │
│  Isaac ROS2 Bridge Extension│   │  controller_manager          │
│  (omni.isaac.ros2_bridge)   │   │  Nav2 / MoveIt 2             │
└─────────────────────────────┘   └──────────────────────────────┘
```

### 2.2 ROS 2 Bridge Extension

`omni.isaac.ros2_bridge` 是 Isaac Sim 官方 ROS 2 集成扩展：

```python
# 在 Isaac Sim Python 环境中启用 ROS 2 Bridge
import omni.isaac.ros2_bridge
from omni.isaac.ros2_bridge import ROS2Camera, ROS2Lidar

# 方式一：Python API 直接创建 ROS 2 接口
from omni.isaac.sensor import Camera
from omni.isaac.ros2_bridge import ROS2CameraHelper

camera = Camera(
    prim_path="/World/Camera",
    position=np.array([0.0, 0.0, 1.0]),
    resolution=(640, 480),
)

ros2_camera = ROS2CameraHelper(
    viewport=viewport,
    rgb=True,
    depth=True,
    camera_info=True,
    namespace="/robot",
    topic="camera",
    frame_id="camera_link",
)
```

### 2.3 OmniGraph 节点（OGN）

OmniGraph 是 Isaac Sim 的可视化 / 脚本化数据流图系统，核心 ROS 2 OGN 节点：

| OGN 节点 | 功能 | 对应 ROS 2 概念 |
|---------|------|---------------|
| `OgnROS2Publisher` | 将 OmniGraph 数据发布为 ROS 2 topic | `rclcpp::Publisher` |
| `OgnROS2Subscriber` | 订阅 ROS 2 topic 驱动 OmniGraph 数据 | `rclcpp::Subscription` |
| `OgnROS2CameraHelper` | 相机图像 → ROS 2 `sensor_msgs/Image` | Camera Publisher |
| `OgnROS2LidarHelper` | RTX LiDAR 点云 → `sensor_msgs/PointCloud2` | LiDAR Publisher |
| `OgnROS2IMUHelper` | IMU 数据 → `sensor_msgs/Imu` | IMU Publisher |
| `OgnROS2OdometryHelper` | 里程计 → `nav_msgs/Odometry` | Odometry Publisher |
| `OgnROS2TfHelper` | 关节 TF → `/tf` | TF Broadcaster |
| `OgnROS2JointStateHelper` | 关节状态 → `sensor_msgs/JointState` | Joint State Publisher |
| `OgnROS2ClockHelper` | 仿真时间 → `/clock` | Clock Publisher |

**OmniGraph 节点配置 YAML 示例**：
```yaml
# action_graph.yaml（概念示意）
nodes:
  - node_type: OgnROS2ClockHelper
    inputs:
      enabled: true
      frameId: ""
      topicName: "/clock"
  
  - node_type: OgnROS2CameraHelper
    inputs:
      topicName: "image_raw"
      frameId: "camera_link"
      width: 640
      height: 480
      renderProductPath: "/Render/RenderProduct_Camera"
      type: "rgb"  # rgb / depth / instance_segmentation / semantic_segmentation
```

### 2.4 `isaac_ros_nvblox` 与 `isaac_ros_jetson`

| 包 | 功能 | 与仿真关联 |
|----|------|----------|
| `isaac_ros_nvblox` | 实时 3D 重建（voxel map）+ 碰撞场 | 从仿真深度相机重建场景，验证重建质量 |
| `isaac_ros_visual_slam` | 视觉 SLAM（CUDA 加速） | 在仿真中验证 SLAM 精度与 Sim-to-Real |
| `isaac_ros_image_pipeline` | GPU 加速图像处理流水线 | 仿真 → NITROS 直通处理 |
| `isaac_ros_jetson` | Jetson 平台特化配置 | 仿真→Jetson 部署的配置一致性 |

---

## 三、传感器仿真

### 3.1 RGB 相机

Isaac Sim 使用 RTX 渲染器进行相机仿真，支持：

```python
from omni.isaac.sensor import Camera
import numpy as np

camera = Camera(
    prim_path="/World/Robot/camera_link/camera",
    position=np.array([0.0, 0.0, 0.0]),
    frequency=30,       # Hz
    resolution=(1920, 1080),
    orientation=rot_utils.euler_angles_to_quat([0, 0, 0]),
)
camera.initialize()

# 相机内参配置
camera.set_focal_length(24.0)      # mm，对应 fx/fy
camera.set_focus_distance(400.0)   # cm
camera.set_horizontal_aperture(20.955)  # mm，影响 FOV

# 获取图像数据
camera.get_rgba()       # (H, W, 4) uint8
camera.get_depth()      # (H, W) float32，单位 m
camera.get_normals()    # (H, W, 4) float32
camera.get_semantic_segmentation()  # (H, W) uint32 (label ID)
camera.get_instance_id_segmentation()
camera.get_bounding_boxes_2d_tight()   # 2D 检测框
camera.get_bounding_boxes_3d()         # 3D 检测框
```

**噪声模型配置**：
```python
# 镜头畸变（径向 + 切向）
from omni.isaac.sensor import CameraDistortion
distortion = CameraDistortion(
    camera_prim_path=camera.prim_path,
    k1=-0.02, k2=0.003, k3=0.0,   # 径向畸变
    p1=0.0002, p2=0.0001,           # 切向畸变
)
```

### 3.2 深度相机（Structured Light / ToF）

```python
# Isaac Sim 深度相机：结构光噪声模型
depth_camera = Camera(
    prim_path="/World/Robot/depth_camera",
    resolution=(640, 480),
)

# 深度噪声模型（仿 RealSense D435）
# 在 USD 层通过 Physics Scene 的 Noise 属性配置
# 典型配置：sigma_depth = 0.002 * depth^2（m，随距离二次增长）
```

### 3.3 RTX LiDAR

Isaac Sim 支持两种 LiDAR 仿真模式：

```python
from omni.isaac.sensor import LidarSensor

lidar = LidarSensor(
    prim_path="/World/Robot/lidar",
    name="lidar",
    # 光线追踪模式（RTX）：最高精度，支持光线折射 / 反射材质
    lidar_type="Rotating",
    high_lod=True,         # 使用 RTX 精确渲染
    yaw_offset=0.0,
    enable_semantics=False,
)

# 或使用预定义传感器配置（仿 Velodyne VLP-16）
lidar = LidarSensor(
    prim_path="/World/Robot/lidar",
    config_file_path="$ISAAC_ROOT/data/sensors/lidar/Velodyne_VLP16.json",
)
```

**LiDAR 配置文件（JSON）关键字段**：
```json
{
  "class": "sensor",
  "type": "rotating",
  "driveWorksId": "LIDAR_CUSTOM",
  "channels": 16,
  "rotationRate": 10.0,
  "numberOfEmitters": 16,
  "emitters": {
    "elevationDeg": [-15.0, -13.0, ..., 15.0],
    "azimuthZeroDeg": 0.0,
    "azimuthDeg": 360.0
  },
  "intensityProcessing": "linear",
  "minRangeM": 0.1,
  "maxRangeM": 100.0,
  "rangeResolutionM": 0.002,
  "noiseMean": 0.0,
  "noiseStd": 0.01
}
```

### 3.4 IMU 传感器

```python
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(
    prim_path="/World/Robot/imu_link/imu",
    name="imu",
    frequency=200,  # Hz
    translation=np.array([0, 0, 0]),
    orientation=np.array([1, 0, 0, 0]),  # xyzw quaternion
    linear_acceleration_filter_size=1,
    angular_velocity_filter_size=1,
)

# IMU 噪声配置（USD 层）
# accel_noise_density: 0.003924 m/s^2/sqrt(Hz) — 仿 ICM-42688
# gyro_noise_density:  0.000261 rad/s/sqrt(Hz)
# accel_random_walk:   0.000098 m/s^3/sqrt(Hz)
# gyro_random_walk:    2.91e-6  rad/s^2/sqrt(Hz)
```

### 3.5 力矩传感器（Contact Reporter）

```python
# 配置接触传感器（用于末端执行器力反馈）
from omni.isaac.sensor import ContactSensor

contact_sensor = ContactSensor(
    prim_path="/World/Robot/end_effector/contact_sensor",
    name="contact_sensor",
    min_threshold=0,
    max_threshold=10000000,
    radius=-1,     # -1 = 使用 Prim 碰撞体积
)
# 读取接触力（法向力 / 切向力 / 法向冲量）
contact_data = contact_sensor.get_current_frame()
# contact_data: {"in_contact": bool, "force": np.array([fx, fy, fz])}
```

---

## 四、关节驱动仿真

### 4.1 PhysX Articulation

PhysX Articulation 是 Isaac Sim 关节机器人仿真的核心数据结构：

```
Articulation（根 Prim）
  └─ Link 0（base_link，固定或浮动根）
       └─ Joint 0（Revolute / Prismatic / Fixed）
            └─ Link 1
                 └─ Joint 1
                      └─ Link 2
                           └─ ...
```

**Articulation 求解器参数**：
```python
# 在 USD 中设置 PhysX Articulation 属性
from pxr import PhysxSchema

articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
articulation_api.GetSolverPositionIterationCountAttr().Set(32)  # 位置迭代次数
articulation_api.GetSolverVelocityIterationCountAttr().Set(1)   # 速度迭代次数
articulation_api.GetSleepThresholdAttr().Set(0.005)
articulation_api.GetStabilizationThresholdAttr().Set(0.001)
```

### 4.2 ArticulationController Python API

```python
from omni.isaac.core.articulations import ArticulationView
import torch

# 创建 Articulation 视图（支持批量操作）
robot = ArticulationView(
    prim_paths_expr="/World/envs/env_.*/Robot",  # 正则匹配多个环境
    name="robot_view",
)
robot.initialize()

# ── 位置控制 ──────────────────────────────────────────
robot.set_joint_position_targets(
    positions=torch.zeros(num_envs, num_dof),
    joint_indices=None,    # None = 所有关节
)

# ── 速度控制 ──────────────────────────────────────────
robot.set_joint_velocity_targets(
    velocities=torch.zeros(num_envs, num_dof),
)

# ── 力矩控制（直接力矩） ───────────────────────────────
robot.set_joint_efforts(
    efforts=torch.zeros(num_envs, num_dof),
)

# ── 读取状态 ──────────────────────────────────────────
positions = robot.get_joint_positions()     # (num_envs, num_dof)
velocities = robot.get_joint_velocities()   # (num_envs, num_dof)
efforts = robot.get_measured_joint_efforts()  # (num_envs, num_dof)
```

### 4.3 控制模式对比

| 控制模式 | PhysX 驱动方式 | 仿真特性 | 适用场景 |
|---------|-------------|---------|---------|
| **Position Control** | PD 驱动器（`stiffness` + `damping`） | 稳定，易调 | 位置轨迹跟踪 |
| **Velocity Control** | 速度驱动器（`damping` 主导） | 中等稳定性 | 速度跟踪、差速底盘 |
| **Effort / Torque** | 直接施力矩 | 最接近真实，需精确模型 | 力控、RL 训练 |
| **Impedance** | 位置 + 力矩混合（需上层计算） | 柔顺控制 | 接触任务 |

---

## 五、材质与摩擦：对强化学习的影响

### 5.1 PhysX Materials 配置

```python
from pxr import UsdShade, PhysxSchema, UsdPhysics

# 创建物理材质
material_prim = stage.DefinePrim("/World/Materials/FloorMaterial", "Material")
physics_material = UsdPhysics.MaterialAPI.Apply(material_prim)
physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)

# 基础摩擦系数
physics_material.GetStaticFrictionAttr().Set(0.8)    # 静摩擦
physics_material.GetDynamicFrictionAttr().Set(0.6)   # 动摩擦
physics_material.GetRestitutionAttr().Set(0.1)        # 恢复系数（弹性）

# PhysX 高级属性
physx_material.GetImprovePatchFrictionAttr().Set(True)   # 改进面片摩擦
physx_material.GetFrictionCombineModeAttr().Set("multiply")  # average/min/max/multiply
physx_material.GetRestitutionCombineModeAttr().Set("max")
```

### 5.2 摩擦参数对 RL 训练的影响

| 参数误差类型 | 对策略的影响 | Domain Randomization 范围建议 |
|------------|------------|------------------------------|
| **地面静摩擦 ±30%** | 步态稳定性降低，滑动接触失效 | μ_static ∈ [0.5, 1.2] |
| **关节摩擦 ±50%** | 速度控制偏差，力矩估计误差 | viscous ∈ [0.0, 0.01] |
| **恢复系数 ±50%** | 弹跳行为异常（不接触任务影响小） | e ∈ [0.0, 0.2] |
| **物体质量 ±20%** | 抓取力矩规划失败 | m ∈ [0.8m, 1.2m] |

---

## 六、Domain Randomization：`omni.replicator` API

### 6.1 Replicator 架构

`omni.replicator` 是 Isaac Sim 4.x 的统一数据生成 / Domain Randomization 框架：

```python
import omni.replicator.core as rep

# ── 基础 DR 写入器 ──────────────────────────────────────
with rep.new_layer():
    # 随机化物体位置
    objects = rep.get.prim(path_pattern="/World/Objects/.*")
    with rep.trigger.on_frame(num_frames=1000):
        with objects:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.5, -0.5, 0.8), (0.5, 0.5, 1.2)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
            )
        
        # 随机化光照
        lights = rep.get.light(light_type="DomeLight")
        with lights:
            rep.modify.attribute(
                "inputs:intensity",
                rep.distribution.normal(1000, 200),
            )
        
        # 随机化纹理
        with objects:
            rep.randomizer.texture(
                textures=rep.utils.get_usd_files("/path/to/textures/"),
                project_uvw=True,
            )

# ── 启动数据生成 ──────────────────────────────────────
rep.orchestrator.run()
```

### 6.2 可随机化参数完整列表

| 类别 | 参数 | Replicator API |
|------|------|---------------|
| **物体位姿** | 位置 / 旋转 / 缩放 | `rep.modify.pose()` |
| **光照** | 强度 / 颜色 / 方向 / 类型 | `rep.modify.attribute()` |
| **纹理** | 反照率 / 法线 / 粗糙度贴图 | `rep.randomizer.texture()` |
| **材质** | 颜色 / 金属度 / 粗糙度 | `rep.randomizer.material()` |
| **相机** | 位置 / FOV / 焦距 / 噪声 | `rep.modify.pose()` + `rep.modify.attribute()` |
| **物理属性** | 质量 / 摩擦 / 惯量 | USD Attribute 直接修改 |
| **背景** | 天空盒 / HDRI 环境贴图 | `rep.randomizer.materials()` |
| **物体数量** | 散落物体数量 | `rep.create.from_usd()` 循环 |

---

## 七、Isaac Lab（前 OmniIsaacGymEnvs 继承者）

### 7.1 架构与设计理念

Isaac Lab 是 Isaac Sim 上的 RL 训练框架，核心特性：

```
Isaac Lab 架构
┌─────────────────────────────────────────────────────────┐
│  Env Registry（gymnasium 兼容接口）                       │
│  ├─ isaaclab.envs.ManagerBasedRLEnv                     │
│  │    ├─ ObservationManager（obs 计算 + 噪声）            │
│  │    ├─ ActionManager（动作空间 + 处理器）                │
│  │    ├─ RewardManager（奖励函数注册）                    │
│  │    ├─ TerminationManager（终止条件）                   │
│  │    └─ CurriculumManager（课程学习）                    │
│  └─ isaaclab.envs.DirectRLEnv（轻量级直接 RL 接口）       │
├─────────────────────────────────────────────────────────┤
│  Scene & Asset 管理                                      │
│  ├─ ArticulationCfg / RigidObjectCfg / DeformableObjectCfg│
│  ├─ SceneEntityCfg（传感器 / 地形 / 物体组合）             │
│  └─ TerrainGenerator（程序化地形生成）                    │
├─────────────────────────────────────────────────────────┤
│  GPU 并行仿真（PhysX GPU + CUDA）                         │
│  ├─ 1024–4096 并行环境（单 RTX 4090）                    │
│  ├─ 全 Tensor 接口（torch.Tensor → GPU 直通）            │
│  └─ 零 CPU-GPU 拷贝（关键路径全 GPU）                     │
├─────────────────────────────────────────────────────────┤
│  RL 算法集成                                              │
│  ├─ RSL-RL（ETH 行走机器人常用）                          │
│  ├─ RL-Games（PPO / SAC）                                │
│  ├─ skrl（PyTorch native）                               │
│  └─ stable-baselines3（gymnasium 标准接口）               │
└─────────────────────────────────────────────────────────┘
```

### 7.2 环境注册与配置

```python
# 自定义 Isaac Lab 环境配置（以差速底盘为例）
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.sensors import CameraCfg, RayCasterCfg
import isaaclab.sim as sim_utils

@configclass
class ChassisEnvCfg(ManagerBasedRLEnvCfg):
    # 场景配置
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=1024,          # 并行环境数
        env_spacing=4.0,        # 环境间距（m）
        replicate_physics=True, # 复用物理场景
    )
    
    # 机器人资产
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path="chassis.usd"),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.1),
            joint_pos={".*": 0.0},
        ),
        actuators={
            "left_wheel": ImplicitActuatorCfg(
                joint_names_expr=["left_wheel_joint"],
                velocity_limit=10.0,   # rad/s
                effort_limit=50.0,     # N·m
                stiffness=0.0,         # 速度控制：stiffness=0
                damping=1000.0,
            ),
        },
    )
    
    # 仿真步进配置
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=0.005,         # 5 ms 物理步长
        render_interval=4,  # 每 4 物理步渲染一次
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=0.8,
            dynamic_friction=0.6,
        ),
    )
    
    # 训练配置
    episode_length_s: float = 20.0   # 每 episode 20 秒
    decimation: int = 4              # 控制频率 = 1/(dt*decimation) = 50 Hz
```

### 7.3 GPU 并行仿真性能估算（RTX 4090）

| 并行环境数 | 物理步长 | 控制频率 | 仿真 FPS（物理） | 实际训练吞吐 |
|----------|---------|---------|--------------|------------|
| 256 | 5 ms | 50 Hz | ~30,000 | ~1.5M steps/s |
| 1024 | 5 ms | 50 Hz | ~12,000 | ~600K steps/s |
| 4096 | 5 ms | 50 Hz | ~4,000 | ~200K steps/s |
| 1024（带 RTX 渲染） | 5 ms | 50 Hz | ~3,000 | ~150K steps/s |

> **注**：启用 RTX 渲染（传感器数据生成）会显著降低训练吞吐量，纯物理 RL 训练建议关闭渲染器。

---

## 八、Isaac Sim 与 ROS 2 集成的关键配置

### 8.1 仿真时间与 `/clock`

```python
# 启用 /clock 发布（Isaac Sim Python 层）
from omni.isaac.ros2_bridge import ROS2Clock

clock_graph = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ROS2Clock.inputs:execIn"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2Clock.inputs:topicName", "/clock"),
        ],
    }
)
```

### 8.2 完整 Launch 文件示例

```python
# isaac_sim_ros2_launch.py（概念伪代码）
import subprocess
import sys
from pathlib import Path

ISAAC_PYTHON = "/isaac-sim/python.sh"
SCRIPT_PATH = Path(__file__).parent / "isaac_sim_init.py"

def launch_isaac_sim():
    """启动 Isaac Sim 并加载 ROS 2 Bridge"""
    cmd = [
        ISAAC_PYTHON,
        str(SCRIPT_PATH),
        "--/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge",
        "--headless",     # 无 GUI 模式（CI / 训练用）
        "--allow-root",   # Docker 容器内
    ]
    return subprocess.Popen(cmd, env={
        "ROS_DOMAIN_ID": "0",
        "FASTRTPS_DEFAULT_PROFILES_FILE": "/ros2_fastdds.xml",
    })
```
