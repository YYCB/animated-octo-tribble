# 具身智能方向 ROS 2 调研路线图（plan.md）

> 面向角色：具身智能行业的**架构师 / 软件开发工程师 / 系统工程师**
> 起点：本仓库已完成的 ROS 2 核心 12 模块源码调研、`ros2_custom_fork_roadmap` 性能路线图、相机 SDK / HAL 对比、`chassis_protocol` 自研 HAL 落地。
> 目标：在已有"经典 ROS 2 内核"基础上，沿"经典控制栈 → 新一代数据通路 → AI 大脑 → 工程基础设施"四条主线迭代，形成完整的具身智能架构知识地图。
>
> **使用方式**：每个阶段单独立项，按本文件顺序串行迭代；每阶段完成后回填本文件的"产出索引"链接，并在 `docs/ROS2_RESEARCH_INDEX.md` 增加目录项。

---

## 总进度看板

- [x] **Phase 1 — `ros2_control` 源码深度调研**（已完成）
- [ ] **Phase 2 — Isaac ROS / NITROS 调研**（GPU 零拷贝 + Type Negotiation，与 custom-fork 路线图呼应）
- [ ] **Phase 3 — LLM / VLA 与 ROS 2 集成模式**（行业风口，架构判断必备）
- [ ] **Phase 4 — MoveIt 2 架构调研**（操作栈事实标准）
- [ ] **Phase 5 — Nav2 架构调研**（移动栈事实标准，对接 chassis_protocol）
- [ ] **Phase 6 — `rosbag2` 与模仿学习数据管线**
- [ ] **Phase 7 — 实时性 / `PREEMPT_RT` / `iceoryx2`**
- [ ] **Phase 8 — micro-ROS 与分布式部署**
- [ ] **Phase 9 — 仿真栈：Isaac Sim / Gazebo Harmonic ↔ ROS 2 桥**
- [ ] **Phase 10 — 多模态感知栈深化**
- [ ] **Phase 11 — 安全（sros2 / DDS-Security）与 OTA**
- [ ] **Phase 12 — 已有调研的"二期深化"专题集合**

---

## 通用方法论（每个 Phase 都遵循）

每个调研专题统一按以下结构产出，目录形如 `docs/<分类>/<主题>_research/`：

1. `00_index.md` — 索引、整体架构图、关键调用链总览
2. `01_architecture.md` — 模块分层、核心数据结构、生命周期
3. `02_xxx.md ~ 0N_xxx.md` — 按子模块逐个剖析（源码级 + 行号引用）
4. `0N+1_data_flows.md` — 端到端调用链 / 时序图
5. `0N+2_integration.md` — 与本仓库已有组件（`chassis_protocol` / 相机 SDK / 已调研 ROS 2 模块）的对接点和决策矩阵
6. （可选）`benchmarks/` — 实测脚本与数据

**统一交付标准**：
- 文档语言：中文为主、关键术语保留英文原词
- 源码引用：`仓库相对路径:行号` 或上游 `repo@tag:path:line`
- 每个 Phase 末尾必须更新 `docs/ROS2_RESEARCH_INDEX.md` 表格 + 本文件总进度看板

---

## Phase 1 — `ros2_control` 源码深度调研

**目录建议**：`docs/ros2-ecosystem/ros2_control_research/`
**优先级**：⭐⭐⭐⭐⭐
**依赖**：已完成的 `ros2_lifecycle_node_research`、`ros2_rclcpp_core_research`、`chassis_protocol`

### 调研目标
- 用源码级理解回答："何时用 `ros2_control` / 何时像 `chassis_protocol` 那样自研 HAL？"
- 给出具身智能整机（底盘 + 机械臂 + 夹爪 + 头部）在 `ros2_control` 框架下的统一拓扑建议

### 子主题清单
- [x] `controller_manager` 实时调度循环（`update()` 时序、`read()` / `write()` 顺序、`update_rate`）
- [x] `hardware_interface`：`SystemInterface` / `ActuatorInterface` / `SensorInterface` 三种抽象的差异与选型
- [x] `resource_manager` 与 `LoanedStateInterface` / `LoanedCommandInterface` 的内存模型
- [x] 控制器生命周期（与 `lifecycle_node` 的关系）、`ChainableController` 级联（impedance / admittance 控制必备）
- [x] 常用控制器：`joint_trajectory_controller` / `diff_drive_controller` / `mecanum_drive_controller` / `forward_command_controller`
- [x] URDF `<ros2_control>` tag 解析、插件加载机制（`pluginlib`）
- [x] 与 `MoveIt 2` 的接口（`FollowJointTrajectory` action）、与 Gazebo / Isaac Sim 的硬件插件
- [x] **决策矩阵**：`ros2_control` vs 自研 HAL（以本仓库 `chassis_protocol` 为反例对照）

### 验收
- [x] 产出 `00_index.md ~ 10_integration.md`（共 11 个文档）
- [x] 产出一份 `chassis_protocol` 改造为 `hardware_interface::SystemInterface` 的可行性评估（见 `10_integration.md`）
- [x] 更新 `ROS2_RESEARCH_INDEX.md`、勾选本看板

### 产出索引
- `docs/ros2-ecosystem/ros2_control_research/00_index.md` — 总索引 + 架构图 + 调用链速查
- `docs/ros2-ecosystem/ros2_control_research/01_architecture.md` — 包结构 + 核心数据结构 + 生命周期
- `docs/ros2-ecosystem/ros2_control_research/02_controller_manager.md` — 实时循环 + Service 接口 + 原子切换
- `docs/ros2-ecosystem/ros2_control_research/03_hardware_interface.md` — System/Actuator/Sensor 对比 + 选型决策树
- `docs/ros2-ecosystem/ros2_control_research/04_resource_manager.md` — 内存布局 + LoanedInterface + 接口所有权
- `docs/ros2-ecosystem/ros2_control_research/05_controller_lifecycle.md` — 生命周期回调 + ChainableController + realtime_tools
- `docs/ros2-ecosystem/ros2_control_research/06_builtin_controllers.md` — JTC / DiffDrive / Mecanum / FCC / Admittance
- `docs/ros2-ecosystem/ros2_control_research/07_urdf_plugin_loading.md` — URDF 解析 + pluginlib + Xacro 最佳实践
- `docs/ros2-ecosystem/ros2_control_research/08_external_integrations.md` — MoveIt 2 / Gazebo / Isaac Sim / mock_components
- `docs/ros2-ecosystem/ros2_control_research/09_data_flows.md` — 启动 + 稳态 + 规划 + 切换 + 关机 + 错误时序
- `docs/ros2-ecosystem/ros2_control_research/10_integration.md` — 决策矩阵 + chassis_protocol 改造评估 + 整机拓扑建议

---

## Phase 2 — Isaac ROS / NITROS 调研（已完成 ✅）

**目录**：`docs/ros2-ecosystem/isaac_ros_nitros_research/`

### 子主题清单
- [x] **Type Adaptation**（REP-2007）：`rclcpp::TypeAdapter` 源码、与 ROS 消息的双向映射
- [x] **Type Negotiation**（REP-2009）：发布/订阅双方协商最优内存格式（CPU / CUDA / NvSci / DMA-BUF）
- [x] NITROS 桥接节点（`isaac_ros_nitros`）的实现：如何在保持 ROS 2 topic 语义的前提下传 GPU buffer
- [x] Managed NITROS Publisher/Subscriber、`NitrosImage` / `NitrosPointCloud` 类型
- [x] Isaac ROS 主要包族：DNN inference、visual SLAM、AprilTag、Stereo、Nvblox
- [x] 与 `image_transport` / `point_cloud_transport` 的关系
- [x] **对照本仓库 custom-fork 路线图**：哪些优化项 NITROS 已实现（D4/D5/D6 ✅）、哪些仍是开放空间（D1/D2/D7/D10/D11）

### 产出索引
- `00_index.md` — 总览 + 整体架构图 + 与已有工作关联
- `01_architecture.md` — NITROS 分层架构 + NitrosNode/GXF 包结构 + Jetson 特化
- `02_type_adaptation.md` — REP-2007 + TypeAdapter 源码 + NITROS NitrosImage TypeAdapter 实现
- `03_type_negotiation.md` — REP-2009 + 协商 5 步协议 + NvSci / DMA-BUF 跨进程 GPU 共享
- `04_nitros_bridge.md` — GXF ↔ ROS 2 桥接机制 + NitrosPublisher/Subscriber 内部流程
- `05_managed_pub_sub.md` — NitrosType 类型层次 + NitrosImage/PointCloud/TensorList 访问示例
- `06_package_ecosystem.md` — DNN/vSLAM/Nvblox/AprilTag 功能矩阵 + 具身智能适用性评估
- `07_transport_plugins.md` — 与 image_transport/point_cloud_transport 关系 + 降级链路处理
- `08_data_flows.md` — 三个场景完整时序（全 GPU / 插入 CPU 节点 / 跨进程 NvSci）
- `09_integration.md` — D1~D12 逐项对照矩阵 + 组合优化路线图 + 选型建议

---

## Phase 3 — LLM / VLA 与 ROS 2 集成模式（已完成 ✅）

**目录**：`docs/embodied-ai/llm_vla_ros2_integration/`

### 子主题清单
- [x] **任务分层模型**：高层 task planner（LLM）→ 中层 skill/behavior（BT/状态机）→ 低层控制器（ros2_control）的边界、异步通信契约、安全层设计
- [x] **OpenVLA / RT-2 / π0 / RDT** VLA 输出格式对比（EE Delta vs Joint Delta vs Joint Absolute）+ 转换层（VlaActionBridgeNode）+ MoveIt Servo 接入
- [x] **`rai` (Robec)** 项目：LLM Agent + ROS 2 工具调用模式、Safety Governor、ReAct 循环
- [x] **`ros2_llm` / Foxglove + LLM** 可观测性：ToolCallLog msg、Foxglove Panel、PlotJuggler、ros2_tracing 端到端延迟
- [x] **NVIDIA Isaac Manipulator / GROOT**：cuMotion GPU 规划、FoundationPose、GROOT 技能编排 YAML
- [x] 推理服务部署：本机 TensorRT / 远端 gRPC / Triton Server + 选型决策矩阵
- [x] **数据飞轮**：rosbag2 录制配置 + 清洗流水线 + LeRobot 格式转换 + sim2real 验证

### 产出索引
- `00_index.md` — 核心挑战时间尺度表 + 分层架构总图 + 关键集成接口速查
- `01_task_hierarchy.md` — 四层架构详解（L0~L4）、异步模式、安全包装层、VLA 接入 ros2_control 两种方案
- `02_vla_output_formats.md` — 6 个 VLA 模型对比表 + 动作 token 量化 + VlaActionBridgeNode + MoveIt Servo
- `03_rai_llm_agent.md` — rai 工具定义（Service/Action/TopicReader）+ ReAct Agent 循环 + Safety Governor
- `04_ros2_llm_observability.md` — ToolCallLog msg + Foxglove Panel + PlotJuggler + ros2_tracing + rosbag2 录制
- `05_isaac_manipulator_groot.md` — cuMotion MoveIt 插件 + FoundationPose + GROOT workflow YAML + 性能数据
- `06_inference_deployment.md` — 本机 TRT / 远端 gRPC / Triton 三种模式 + 网络延迟分析 + 选型矩阵
- `07_data_flywheel.md` — rosbag2 录制配置 + 清洗流水线 + LeRobot 格式 + align_observations + sim2real
- `08_integration.md` — 5 方案横向对比 + 分层架构参考实现图 + 选型决策矩阵 + 关键风险与缓解

---

## Phase 4 — MoveIt 2 架构调研

**目录建议**：`docs/ros2-ecosystem/moveit2_research/`
**优先级**：⭐⭐⭐⭐
**依赖**：Phase 1（`ros2_control` 接口）

### 子主题清单
- [ ] `move_group` 节点的插件体系：Planner（OMPL / Pilz / STOMP）、IK（KDL / TracIK / bio_ik）、Collision（FCL）、Sensor
- [ ] `moveit_py` Python API（取代 `moveit_commander`）
- [ ] **MoveIt Servo**：实时增量控制、奇异点处理
- [ ] **Hybrid Planning**（global + local planner 协同）
- [ ] `planning_scene_monitor` 的实时性瓶颈与优化
- [ ] 与 `ros2_control` 的耦合点（`FollowJointTrajectory` action）
- [ ] 与 VLA（Phase 3）输出的对接：`pose_goal` / `joint_goal` / `cartesian_path`

---

## Phase 5 — Nav2 架构调研（已提前完成 ✅）

**目录**：`docs/ros2-ecosystem/nav2_research/`

### 子主题清单
- [x] 整体架构：`bt_navigator` / `planner_server` / `controller_server` / `behavior_server` / `smoother_server` + lifecycle_manager + bond 机制
- [x] BehaviorTree 集成：BT.CPP v4、`nav2_behavior_tree` 内置节点全表（Action/Condition/Decorator）、自定义 BT 节点开发、Groot2 调试
- [x] `Costmap2D` 插件体系（static / inflation / obstacle / voxel / SpatioTemporalVoxelLayer）
- [x] 主流 planner 对比（NavFn / SmacPlanner2D / SmacPlannerHybrid / SmacPlannerLattice / ThetaStar）
- [x] 主流 controller 对比（DWB / MPPI / RPP）
- [x] AMCL / SLAM Toolbox / nav2_collision_monitor / nav2_map_server / robot_localization EKF
- [x] **与 `chassis_protocol` 的对接清单**：QoS 匹配、TF 树职责划分、odom 来源、cmd_vel 拓扑（twist_mux + collision_monitor）、最小化 launch 文件

### 产出索引
- `00_index.md` — 整体服务器拓扑图 + NavigateToPose 完整消息流
- `01_architecture_overview.md` — 六大服务器详解 + lifecycle_manager + bond 机制 + 参数体系
- `02_behavior_tree_integration.md` — BT.CPP v4 节点类型 + 内置节点全表 + 默认 XML 解析 + 自定义节点开发 + Groot2
- `03_costmap2d_plugins.md` — 双层 Costmap + Layer 栈 + 五类插件（Static/Inflation/Obstacle/Voxel/STVL）+ 调试命令
- `04_planner_plugins.md` — NavFn/SmacPlanner2D/Hybrid/Lattice/ThetaStar 对比表 + 差速底盘选型建议
- `05_controller_plugins.md` — DWB/MPPI/RPP 原理+配置+对比表 + Jetson 选型建议
- `06_localization.md` — AMCL 粒子滤波 + SLAM Toolbox 三种模式 + collision_monitor 安全链路 + map_server 格式
- `07_chassis_protocol_integration.md` — QoS 对接清单 + TF 树对接 + odom 来源方案 + cmd_vel 拓扑（twist_mux）+ 改造检查表 + 最小化 launch

---

## Phase 6 — `rosbag2` 与模仿学习数据管线

**目录建议**：`docs/ros2-ecosystem/rosbag2_research/`
**优先级**：⭐⭐⭐⭐

### 子主题清单
- [ ] `rosbag2_cpp` 架构、`SequentialWriter` / `SequentialReader`
- [ ] Storage plugin：`mcap` vs `sqlite3`，压缩、分片、索引
- [ ] `rosbag2_transport`：录制/回放的时钟模型（与 `ros2_clock_time_research` 衔接）
- [ ] 大规模采集时的零拷贝、IPC、磁盘吞吐瓶颈
- [ ] 面向模仿学习的数据 schema 设计：观测同步、动作对齐、episode 切分
- [ ] Foxglove / PlotJuggler / Webviz 的回放生态

---

## Phase 7 — 实时性 / `PREEMPT_RT` / `iceoryx2`

**目录建议**：`docs/ros2-core/ros2_realtime_research/`
**优先级**：⭐⭐⭐⭐
**依赖**：`ros2_topic_communication_research` / `ros2_rclcpp_core_research`

### 子主题清单
- [ ] `PREEMPT_RT` 内核要点、ROS 2 节点的 CPU pinning / SCHED_FIFO 设置
- [ ] Executor 实时性对比：`SingleThreadedExecutor` / `MultiThreadedExecutor` / `StaticSingleThreadedExecutor` / `EventsExecutor`
- [ ] **`iceoryx` / `iceoryx2`** 共享内存中间件，`rmw_iceoryx`
- [ ] CycloneDDS 低延迟配置、Zenoh-RMW 候选
- [ ] DDS-SHM vs iceoryx vs LoanedMessage 三者深度对比（深化 `ros2_topic_communication_research` 的 SHM 章节）
- [ ] 实测：1kHz 控制环抖动测量方法

---

## Phase 8 — micro-ROS 与分布式部署

**目录建议**：`docs/ros2-ecosystem/micro_ros_research/`
**优先级**：⭐⭐⭐

### 子主题清单
- [ ] micro-ROS 在 STM32 / ESP32 / Zephyr 上的运行模型
- [ ] XRCE-DDS Agent / Client 协议
- [ ] `ROS_DOMAIN_ID` / DDS Discovery Server 在多机/多容器/跨网段部署中的实践
- [ ] **具身智能整机部署拓扑参考**：主控 SoC（Jetson/X86）+ 多 MCU 关节驱动 + 远端云端

---

## Phase 9 — 仿真栈：Isaac Sim / Gazebo Harmonic ↔ ROS 2 桥

**目录建议**：`docs/ros2-ecosystem/sim_bridges_research/`
**优先级**：⭐⭐⭐⭐

### 子主题清单
- [ ] Gazebo Harmonic 架构、`ros_gz_bridge` / `ros_gz_sim` 消息映射
- [ ] Isaac Sim `omni.isaac.ros2_bridge`、Action Graph、OmniGraph
- [ ] 仿真时钟（`/clock` topic）与 `use_sim_time`（衔接 `ros2_clock_time_research`）
- [ ] Sim2Real：传感器噪声、domain randomization、`ros2_control` 的 `mock_components` / `gz_ros2_control` / `isaac_ros2_control`
- [ ] 数据生成管线：仿真 → rosbag2 → VLA 训练（与 Phase 3 / Phase 6 闭环）

---

## Phase 10 — 多模态感知栈深化

**目录建议**：`docs/ros2-ecosystem/perception_stack_research/`
**优先级**：⭐⭐⭐
**依赖**：已完成的 `realsense-ros-4.57.7-analysis` / `orbbec_sdk_ros2_analysis`

### 子主题清单
- [ ] `image_transport` / `point_cloud_transport` 的插件体系（compressed / theora / zstd / draco）
- [ ] 多传感器时间同步：`message_filters::TimeSynchronizer` 源码、硬件 PTP / IEEE 1588
- [ ] GPU 推理节点接入：`isaac_ros_dnn_inference` / `tensorrt_yolov8` / `ros2_torch`
- [ ] 触觉、IMU、力矩、关节扭矩等"非视觉"传感器在 ROS 2 中的标准消息与采集模式
- [ ] 与 NITROS（Phase 2）的衔接：何时升级到 GPU 零拷贝管线

---

## Phase 11 — 安全（sros2 / DDS-Security）与 OTA

**目录建议**：`docs/ros2-ecosystem/security_ota_research/`
**优先级**：⭐⭐

### 子主题清单
- [ ] `sros2` 工具、DDS-Security 五大插件（Auth / AccessControl / Cryptographic / Logging / DataTagging）
- [ ] keystore 管理、企业 CA 集成
- [ ] 容器化部署：`ros:humble` 镜像、`docker-compose` 多节点拓扑
- [ ] OTA 升级架构（Mender / RAUC / 自研）、A/B 分区与 ROS 2 节点滚动重启

---

## Phase 12 — 已有调研的"二期深化"专题集合

按需挑选，不强制顺序。每项独立成小专题。

- [ ] **`rmw` 完整抽象层对比**：`rmw_fastrtps_cpp` vs `rmw_cyclonedds_cpp` vs `rmw_zenoh` 并排剖析
- [ ] **QoS 调优手册**：相机 / 激光 / 控制环三类典型 topic 的 QoS profile + 失败案例库
- [ ] **整机故障管理**：`lifecycle_node` + `diagnostic_aggregator` + BehaviorTree 的协同
- [ ] **custom-fork 路线图执行篇**：挑 1-2 个最高 ROI 优化（如 IPC 零拷贝 + Executor）做实测 benchmark
- [ ] **`arm_protocol` / `gripper_protocol`**：复用 `chassis_protocol` 的分层模式，沉淀"具身智能 HAL 框架"

---

## 迭代节奏建议

- 每个 Phase 视为一个独立 PR（或一组 PR），完成后回填本文件 + `ROS2_RESEARCH_INDEX.md`
- Phase 1 → 2 → 3 是建议的"主线三连"，完成后即覆盖"经典控制 + 新一代数据通路 + AI 大脑"三大支柱
- Phase 4-12 可根据当时业务优先级动态重排
- 每个 Phase 启动前，先在该 Phase 章节列出"本次具体要回答的 3-5 个核心问题"，避免无边界发散

---

## 维护说明

- 本文件是**活文档**：每个 Phase 完成后必须：
  1. 勾选"总进度看板"对应项
  2. 勾选该 Phase 的"子主题清单"完成项（未完成可标注原因）
  3. 在该 Phase 末尾追加"产出索引"小节，列出生成的文档路径
- 新增 Phase 请追加到末尾，不要打乱已编号的阶段
