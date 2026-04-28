# 具身智能方向 ROS 2 调研路线图（plan.md）

> 面向角色：具身智能行业的**架构师 / 软件开发工程师 / 系统工程师**
> 起点：本仓库已完成的 ROS 2 核心 12 模块源码调研、`ros2_custom_fork_roadmap` 性能路线图、相机 SDK / HAL 对比、`chassis_protocol` 自研 HAL 落地。
> 目标：在已有"经典 ROS 2 内核"基础上，沿"经典控制栈 → 新一代数据通路 → AI 大脑 → 工程基础设施"四条主线迭代，形成完整的具身智能架构知识地图。
>
> **使用方式**：每个阶段单独立项，按本文件顺序串行迭代；每阶段完成后回填本文件的"产出索引"链接，并在 `docs/ROS2_RESEARCH_INDEX.md` 增加目录项。

---

## 总进度看板

- [x] **Phase 1 — `ros2_control` 源码深度调研**（已完成）
- [x] **Phase 2 — Isaac ROS / NITROS 调研**（已完成）
- [x] **Phase 3 — LLM / VLA 与 ROS 2 集成模式**（已完成）
- [x] **Phase 4 — MoveIt 2 架构调研**（已完成）
- [x] **Phase 5 — Nav2 架构调研**（已完成，提前执行）
- [x] **Phase 6 — `rosbag2` 与模仿学习数据管线**（已完成）
- [x] **Phase 7 — 实时性 / `PREEMPT_RT` / `iceoryx2`**（已完成）
- [x] **Phase 8 — micro-ROS 与分布式部署**（已完成）
- [x] **Phase 9 — 仿真栈：Isaac Sim / Gazebo Harmonic ↔ ROS 2 桥**（已完成）
- [x] **Phase 10 — 多模态感知栈深化**（已完成）
- [x] **Phase 11 — 安全（sros2 / DDS-Security）与 OTA**（已完成）
- [x] **Phase 12 — 已有调研的"二期深化"专题集合**（已完成）
- [x] **Phase 13 — SLAM 栈调研（S100 主控 / CUDA-free）**（已完成）

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

## Phase 4 — MoveIt 2 架构调研（已完成 ✅）

**目录**：`docs/ros2-ecosystem/moveit2_research/`

### 子主题清单
- [x] `move_group` 节点的插件体系：Planner（OMPL 算法对比 / Pilz PTP+LIN+CIRC / STOMP 随机优化）、IK（KDL / TracIK solve_type / bio_ik PSO）、Collision（FCL 性能指标 + 碰撞矩阵优化）、Sensor 传感器集成（Octomap）
- [x] `moveit_py` Python API：pose_goal / joint_goal / named_pose / 规划器切换 / planning scene 管理 / 抓取放置 / 约束规划
- [x] **MoveIt Servo**：实时增量控制架构（TwistStamped/JointJog）、奇异点检测与速度缩减、Servo 使能/停止 Service、ros2_control 控制器切换
- [x] **Hybrid Planning**：Global + Local Planner 协同框架、自定义局部规划器开发模式、动态重规划触发机制
- [x] `planning_scene_monitor` 的实时性瓶颈（锁争用 + Octomap 更新）与优化策略（碰撞矩阵 / 降采样 / 批量更新 / 冻结传感器）
- [x] 与 `ros2_control` 的耦合点：FollowJointTrajectory 消息结构、moveit_controllers.yaml 配置、轨迹执行管理器、时间参数化（TOTG + Ruckig）
- [x] 与 VLA（Phase 3）输出的对接：四种模式（Servo / JointJog / pose_goal / cartesian_path）对比矩阵 + VlaActionRouter 路由设计 + E-Stop 集成

### 产出索引
- `00_index.md` — 整体架构图 + 完整规划执行消息流 + 配置文件组织 + 模块交叉引用
- `01_move_group_plugins.md` — Planner（OMPL/Pilz/STOMP）+ IK（KDL/TracIK/bio_ik）+ Collision（FCL）+ 传感器集成
- `02_moveit_servo.md` — 实时增量控制、奇异点处理、Servo↔规划模式切换、VLA 对接速览
- `03_hybrid_planning.md` — Global+Local 协同架构、自定义局部规划器、与传统 MoveGroup 对比
- `04_planning_scene_monitor.md` — 实时碰撞场景维护、性能瓶颈分析、优化策略
- `05_ros2_control_coupling.md` — FollowJointTrajectory 详解、控制器切换时序、时间参数化、对接检查表
- `06_vla_integration.md` — 四种 VLA→MoveIt 对接模式 + VlaActionRouter 完整设计 + E-Stop 安全层
- `07_moveit_py.md` — moveit_py 完整 API：规划/执行/场景管理/抓取放置/约束规划

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
- [x] `rosbag2_cpp` 架构、`SequentialWriter` / `SequentialReader`
- [x] Storage plugin：`mcap` vs `sqlite3`，压缩、分片、索引
- [x] `rosbag2_transport`：录制/回放的时钟模型（与 `ros2_clock_time_research` 衔接）
- [x] 大规模采集时的零拷贝、IPC、磁盘吞吐瓶颈
- [x] 面向模仿学习的数据 schema 设计：观测同步、动作对齐、episode 切分
- [x] Foxglove / PlotJuggler / Webviz 的回放生态

### 产出索引
- `docs/ros2-ecosystem/rosbag2_research/00_index.md` — 总索引 + 数据飞轮架构图 + 录制/回放消息流时序 + 与本仓库关联速查
- `docs/ros2-ecosystem/rosbag2_research/01_architecture.md` — rosbag2_cpp 核心架构：包结构、SequentialWriter/Reader 内部流程、BagMetadata 数据结构、StorageOptions/RecordOptions/PlayOptions 全览、Converter plugin、Python 绑定
- `docs/ros2-ecosystem/rosbag2_research/02_storage_plugins.md` — MCAP vs SQLite3 深度对比：格式内部结构、时间范围查询原理、压缩插件（zstd/lz4）、分片策略、具身智能采集建议
- `docs/ros2-ecosystem/rosbag2_research/03_transport_recording.md` — rosbag2_transport 层：RecordOptions/PlayOptions 完整参数、时钟模型（wall/sim）、QoS 兼容性处理、CLI 参数说明、预加载缓存机制
- `docs/ros2-ecosystem/rosbag2_research/04_zero_copy_performance.md` — 大规模采集性能：零拷贝路径、磁盘吞吐瓶颈分析、多线程录制、丢帧检测、具身智能场景带宽估算与调优建议
- `docs/ros2-ecosystem/rosbag2_research/05_imitation_learning_schema.md` — 面向模仿学习的数据 schema：观测同步、标准 topic 清单、动作对齐、Episode 切分、LeRobot HDF5 转换脚本设计、align_observations 实现、数据质量检查
- `docs/ros2-ecosystem/rosbag2_research/06_playback_ecosystem.md` — 回放可视化生态：Foxglove Studio（WebSocket bridge + Layout JSON）、PlotJuggler、rviz2、mcap CLI 工具、具身智能 VLA rollout 调试场景
- `docs/ros2-ecosystem/rosbag2_research/07_integration.md` — 与本仓库组件对接：chassis_protocol 集成、Phase 3 数据飞轮闭环、Phase 7 实时性对接、Phase 9 仿真数据生成、整体数据管线架构图、选型决策矩阵

---

## Phase 7 — 实时性 / `PREEMPT_RT` / `iceoryx2`

**目录**：`docs/ros2-ecosystem/realtime_research/`
**优先级**：⭐⭐⭐⭐
**依赖**：`ros2_topic_communication_research` / `ros2_rclcpp_core_research`

### 子主题清单
- [x] `PREEMPT_RT` 内核要点、ROS 2 节点的 CPU pinning / SCHED_FIFO 设置
- [x] Executor 实时性对比：`SingleThreadedExecutor` / `MultiThreadedExecutor` / `StaticSingleThreadedExecutor` / `EventsExecutor`
- [x] **`iceoryx` / `iceoryx2`** 共享内存中间件，`rmw_iceoryx`
- [x] CycloneDDS 低延迟配置、Zenoh-RMW 候选
- [x] DDS-SHM vs iceoryx vs LoanedMessage 三者深度对比
- [x] 实测：1kHz 控制环抖动测量方法（cyclictest / ros2_tracing）
- [x] `ros2_control` 实时控制环深化（4kHz 力控配置、ChainableController）
- [x] `chassis_protocol` RT 改造路线图（无锁缓冲区 + epoll 替换）

### 产出索引
- `docs/ros2-ecosystem/realtime_research/00_index.md` — 总索引：实时性需求矩阵、完整实时栈架构图、延迟预算分配表、与本仓库关联速查
- `docs/ros2-ecosystem/realtime_research/01_preempt_rt_kernel.md` — PREEMPT_RT 补丁原理、Ubuntu 22.04/24.04 安装（ubuntu-pro vs 手动编译）、BIOS 调优、cyclictest/hwlatdetect 测试、Jetson Orin RT 内核
- `docs/ros2-ecosystem/realtime_research/02_ros2_realtime.md` — 内存锁定（mlockall）、CPU 隔离（isolcpus/cpuset/taskset）、线程优先级（SCHED_FIFO/RR）、Executor 实时对比、realtime_tools（RealtimeBuffer/Publisher/Box）、Loaned Messages、完整 RT 节点模板
- `docs/ros2-ecosystem/realtime_research/03_iceoryx2.md` — iceoryx2 vs iceoryx1 对比、零拷贝 IPC 架构、RouDi 守护进程、rmw_iceoryx RMW 层、性能数据表、WaitSet 事件驱动、与 ros2_control 集成、局限性
- `docs/ros2-ecosystem/realtime_research/04_dds_tuning.md` — RMW 四维对比表、CycloneDDS RT 配置（XML 详解）、FastDDS RT 配置（historyMemoryPolicy）、QoS 实时参数（DEADLINE/LIVELINESS/LATENCY_BUDGET）、网络栈调优、选型决策矩阵
- `docs/ros2-ecosystem/realtime_research/05_ros2_control_rt.md` — controller_manager update() 时序图、hardware_interface RT 约束（禁止操作清单）、ChainableController 延迟叠加、4kHz 力控配置、EtherCAT/RS485 RT 实现要点、chassis_protocol 对接要点
- `docs/ros2-ecosystem/realtime_research/06_latency_profiling.md` — ros2_tracing+LTTng（tracepoint 列表）、tracetools_analysis 脚本、cyclictest 完整参数与直方图解读、ros2_performance_test、perf+ftrace、端到端延迟测量方法、典型问题诊断决策树
- `docs/ros2-ecosystem/realtime_research/07_integration.md` — chassis_protocol RT 改造路线图（SPSCRingBuffer + epoll 替换 select）、Phase 6 录制进程隔离（CPU 核分配）、整体 RT 部署图（8核分配）、Jetson Orin 特化配置（nvpmodel+jetson_clocks）、容器化 RT（Docker 参数）、选型决策矩阵

---

## Phase 8 — micro-ROS 与分布式部署

**目录**：`docs/ros2-ecosystem/microros_research/`
**优先级**：⭐⭐⭐
**状态**：✅ 已完成

### 子主题清单
- [x] micro-ROS 在 STM32 / ESP32 / Zephyr 上的运行模型
- [x] XRCE-DDS Agent / Client 协议
- [x] `ROS_DOMAIN_ID` / DDS Discovery Server 在多机/多容器/跨网段部署中的实践
- [x] **具身智能整机部署拓扑参考**：主控 SoC（Jetson/X86）+ 多 MCU 关节驱动 + 远端云端

### 产出索引

| 文件 | 内容 |
|------|------|
| [00_index.md](./ros2-ecosystem/microros_research/00_index.md) | 总览：分布式架构图、延迟资源预算表、chassis_protocol 关联速查、文档导航 |
| [01_architecture.md](./ros2-ecosystem/microros_research/01_architecture.md) | micro-ROS 软件栈、rclc vs rclcpp API 对比、XRCE-DDS 协议、Agent 架构、RTOS/硬件对比 |
| [02_transport_layer.md](./ros2-ecosystem/microros_research/02_transport_layer.md) | Serial/UDP/USB CDC/Custom Transport、分片处理、Discovery vs Static、可靠性模式 |
| [03_rclc_programming.md](./ros2-ecosystem/microros_research/03_rclc_programming.md) | rclc Executor、Timer、Service、参数服务器、Lifecycle、内存策略、完整关节控制器示例 |
| [04_ros2_control_integration.md](./ros2-ecosystem/microros_research/04_ros2_control_integration.md) | 分布式硬件接口架构、MicroRosSystemInterface 实现、延迟分析、同步策略、EtherCAT 混合 |
| [05_distributed_deployment.md](./ros2-ecosystem/microros_research/05_distributed_deployment.md) | DDS Domain 隔离、Agent 容器化、多机器人 namespace、边缘-云协同、时间同步 |
| [06_firmware_build.md](./ros2-ecosystem/microros_research/06_firmware_build.md) | micro_ros_setup 构建、自定义消息集成、OTA 固件更新、GitHub Actions CI、版本矩阵 |
| [07_integration.md](./ros2-ecosystem/microros_research/07_integration.md) | chassis_protocol 迁移评估矩阵（方案A/B对比）、Phase 7 RT 对接、Phase 6 录制对接、整体架构图、选型决策矩阵 |

---

## Phase 9 — 仿真栈：Isaac Sim / Gazebo Harmonic ↔ ROS 2 桥 ✅

**实际目录**：`docs/ros2-ecosystem/simulation_research/`
**优先级**：⭐⭐⭐⭐

### 产出索引

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./ros2-ecosystem/simulation_research/00_index.md) | 整体架构、仿真器对比矩阵、文档导航 |
| [01_gazebo_harmonic.md](./ros2-ecosystem/simulation_research/01_gazebo_harmonic.md) | Gazebo Harmonic 架构、ros_gz_bridge 消息映射 |
| [02_isaac_sim.md](./ros2-ecosystem/simulation_research/02_isaac_sim.md) | Isaac Sim omni.isaac.ros2_bridge、OmniGraph、Action Graph |
| [03_sim_time.md](./ros2-ecosystem/simulation_research/03_sim_time.md) | /clock topic、use_sim_time、时钟模型 |
| [04_sim2real.md](./ros2-ecosystem/simulation_research/04_sim2real.md) | 传感器噪声、Domain Randomization、mock_components |
| [05_sensor_simulation.md](./ros2-ecosystem/simulation_research/05_sensor_simulation.md) | RGB-D/LiDAR/IMU/FT 传感器仿真、Allan Variance、精度评估 |
| [06_training_data_generation.md](./ros2-ecosystem/simulation_research/06_training_data_generation.md) | omni.replicator 管线、rosbag2 录制、Isaac Lab 并行、数据质量检查 |
| [07_integration.md](./ros2-ecosystem/simulation_research/07_integration.md) | GazeboSystem plugin、ChassisHalMock、VLA 数据飞轮、决策矩阵 |

---

## Phase 10 — 多模态感知栈深化 ✅

**实际目录**：`docs/ros2-ecosystem/perception_research/`
**优先级**：⭐⭐⭐

### 产出索引

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./ros2-ecosystem/perception_research/00_index.md) | 感知栈需求矩阵、架构图、NITROS 关系、版本矩阵 |
| [01_object_detection.md](./ros2-ecosystem/perception_research/01_object_detection.md) | RT-DETR TRT 部署、YOLOv8、PointPillars、MoveIt 2 碰撞对象集成 |
| [02_pose_estimation.md](./ros2-ecosystem/perception_research/02_pose_estimation.md) | FoundationPose 零样本 6D、DOPE 训练、AprilTag、手眼标定 |
| [03_nvblox_3d_reconstruction.md](./ros2-ecosystem/perception_research/03_nvblox_3d_reconstruction.md) | TSDF/ESDF/Color 层架构、Nav2 costmap 层、Orin AGX 性能 |
| [04_visual_language.md](./ros2-ecosystem/perception_research/04_visual_language.md) | CLIP/SigLIP、Grounding DINO、SAM 2、LLM→感知→操作链 |
| [05_integration.md](./ros2-ecosystem/perception_research/05_integration.md) | 35ms 端到端延迟分析、NITROS 零拷贝节点表、RT CPU 分配 |

---

## Phase 11 — 安全（sros2 / DDS-Security）与 OTA ✅

**实际目录**：`docs/ros2-ecosystem/security_research/`
**优先级**：⭐⭐

### 产出索引

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./ros2-ecosystem/security_research/00_index.md) | 威胁模型表、安全栈架构图、版本矩阵 |
| [01_sros2.md](./ros2-ecosystem/security_research/01_sros2.md) | DDS-Security 五大插件、完整 sros2 CLI 工作流、AES-256 性能基准 |
| [02_ota_update.md](./ros2-ecosystem/security_research/02_ota_update.md) | Docker 蓝绿部署、OSTree A/B、SWUpdate、ESP32 OTA、TUF 框架 |
| [03_integration.md](./ros2-ecosystem/security_research/03_integration.md) | ChassisNode sros2 配置、分层安全架构、GitHub Actions OTA、决策矩阵 |

---

## Phase 12 — 已有调研的"二期深化"专题集合 ✅

**实际目录**：`docs/ros2-ecosystem/deepdive_research/`
**优先级**：按需

### 产出索引

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./ros2-ecosystem/deepdive_research/00_index.md) | 深化背景、开放问题汇总、导航表 |
| [01_ros2_control_advanced.md](./ros2-ecosystem/deepdive_research/01_ros2_control_advanced.md) | ChainableController 三级链式、Ruckig 轨迹、GPIO Interface、Semantic Components、热重启 |
| [02_nav2_advanced.md](./ros2-ecosystem/deepdive_research/02_nav2_advanced.md) | MPPI 系统调优、SmacPlannerLattice 运动学文件、collision_monitor、BT 自定义节点 |
| [03_moveit2_advanced.md](./ros2-ecosystem/deepdive_research/03_moveit2_advanced.md) | bio_ik PSO、STOMP 随机优化、动态 ACM、Grasp Pose Detection、力控装配 |
| [04_data_flywheel.md](./ros2-ecosystem/deepdive_research/04_data_flywheel.md) | 完整飞轮架构、在线质量筛选、EWC 持续学习、DVC+MLflow 版本管理 |
| [05_summary.md](./ros2-ecosystem/deepdive_research/05_summary.md) | 12 Phase 核心发现汇总表、实施路线图（P0–P3）、10 个开放问题、参考索引 |

---

## Phase 13 — SLAM 栈调研（S100 主控 / CUDA-free）✅

**实际目录**：`docs/ros2-ecosystem/slam_research/`
**优先级**：⭐⭐⭐⭐⭐（与硬件平台直接绑定）
**触发**：硬件平台从 Orin AGX 切换到地瓜机器人 RDK S100 作为主控；S100 无 CUDA / 无 NVIDIA GPU，原 `perception_research/` 中 nvblox 等 GPU 路径不可用，需要为 2D LiDAR + IMU + RGB-D 室内移动机器人搭建一套**纯 CPU / ARM 友好**的 SLAM 栈。

### 调研目标
- 在 S100 主控（无 CUDA）约束下，给出 SLAM / 定位 / 局部 3D 感知的完整选型
- 解决"SLAM Toolbox 何时漂、为何漂、如何调"的工程问题
- 把 wheel + IMU 双源融合、2D LiDAR 退化兜底、RGB-D 接 costmap 三件事讲透
- 与已有 `chassis_protocol` HAL 无缝衔接（`/odom` + 协方差 + TF 唯一发布者）

### 子主题清单
- [x] SLAM Toolbox 内部数据结构（pose graph、ScanMatcher、Ceres 后端）+ 三种模式切换 + 参数全表 + ARM 性能预算
- [x] `robot_localization` EKF/UKF 选型 + `_config` 矩阵语义 + 协方差填法（含 1e6 屏蔽惯例）+ Madgwick 前置链
- [x] CPU 视觉 SLAM/VIO 横向对比（OpenVINS / VINS-Fusion / ORB-SLAM3 / RTAB-Map）+ 作为 EKF 第二输入的接入方式
- [x] RGB-D depth → STVL 的全 CPU 路径（同进程 ComposableNode + VoxelGrid + Passthrough）+ 为何不用 nvblox
- [x] Lifelong / 动态环境 / 地图持久化 / 版本管理 / 灾备
- [x] APE / RPE / `evo` 工具链 + 回归测试套件 + S100 vs Orin 性能基线方法学
- [x] 与 `chassis_protocol` 对接清单（QoS / TF / 协方差 / TODO）+ S100 主控整机拓扑 + TogetheROS hobot_* 衔接 + launch 骨架 + 实机调试 checklist

### 产出索引

| 文件 | 内容摘要 |
|------|---------|
| [00_index.md](./ros2-ecosystem/slam_research/00_index.md) | 总索引、S100 主控硬约束（无 CUDA / BPU≠GPGPU / ARM）、整体架构图、三阶段演进路线、与已有调研的衔接、明确边界 |
| [01_slam_toolbox_deep_dive.md](./ros2-ecosystem/slam_research/01_slam_toolbox_deep_dive.md) | Karto pose-graph 流程、3 个 ScanMatcher 实例、`.posegraph + .data` 文件结构、4 种节点对应模式、参数全表、ARM 降级矩阵、推荐起步参数、典型故障 8 类 |
| [02_odom_fusion_with_robot_localization.md](./ros2-ecosystem/slam_research/02_odom_fusion_with_robot_localization.md) | EKF vs UKF 选型、15 维状态向量、`_config` 矩阵双源典型配置、协方差填法（1e6 屏蔽惯例）、`imu_filter_madgwick` 前置链、TF 唯一发布者原则、与 chassis_protocol 对接清单 |
| [03_visual_slam_cpu_backends.md](./ros2-ecosystem/slam_research/03_visual_slam_cpu_backends.md) | 4 种 CPU VIO/VSLAM 横向对比表、OpenVINS（首推，最轻）参数、ORB-SLAM3 角色边界、RTAB-Map 备选定位、外参/时间同步天花板、决策树、launch 接入 EKF 的 `differential: true` 关键 |
| [04_3d_perception_for_2d_nav.md](./ros2-ecosystem/slam_research/04_3d_perception_for_2d_nav.md) | RGB-D depth → cloud → 降采样 → passthrough → STVL 全 CPU 链路、ComposableNode 同进程组合、STVL vs VoxelLayer 对比、完整配置 yaml、调参矩阵、楼梯/坠落检测、为何禁用 nvblox |
| [05_dynamic_and_lifelong.md](./ros2-ecosystem/slam_research/05_dynamic_and_lifelong.md) | 三种地图演化策略、Lifelong utility score 算法、动态物体三层防御、地图版本化目录布局、灾备回滚、运行时监控指标 |
| [06_evaluation_and_benchmarking.md](./ros2-ecosystem/slam_research/06_evaluation_and_benchmarking.md) | APE/RPE 定义、`evo` 工具链命令、Gazebo Harmonic 做 GT、回归测试套件目录、S100 vs Orin AGX 基线对照表、参数误调对比示例、签字交付 checklist |
| [07_chassis_and_s100_integration.md](./ros2-ecosystem/slam_research/07_chassis_and_s100_integration.md) | 整机拓扑图、节点/话题/TF 总账、chassis_protocol 现状核查 + 协方差 TODO、TogetheROS / hobot_* 衔接、CPU 核分配表、内存预算、一键 launch 骨架、首次实机调试 17 项 checklist |

### 验收
- [x] 所有产出文档完成且互相交叉引用
- [x] 全栈选型均为 **CPU-only / CUDA-free**，与 S100 硬件约束一致
- [x] chassis_protocol 现状已核查（`chassis_node.cpp:24-30, 76-77, 149-152`），不发 TF + 透传协方差 = 与本方案兼容；明确 HAL 协方差默认值 TODO
- [x] 更新 `ROS2_RESEARCH_INDEX.md` 与本文件 ✅

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
