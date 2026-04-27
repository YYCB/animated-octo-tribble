# 12 Phase 研究总结与实施路线图

> 本文档是 12 个研究阶段的综合总结，包含核心发现汇总表、优先级实施路线图、10 个开放问题清单，以及完整参考资料索引。

---

## 一、12 Phase 核心发现汇总表

| Phase | 主题 | 核心发现 | 本仓库状态 | 优先级 |
|-------|------|---------|-----------|-------|
| **1** | ros2_control 基础 | 1kHz 控制环路可行（Humble + PREEMPT_RT）；ChainableController 消除链式节点间 ~200μs 延迟 | chassis_protocol 用自定义 HAL，**未接入 ros2_control SystemInterface** | P1 迁移 |
| **2** | NITROS 零拷贝 | NITROS 零拷贝仅在同一 ComponentContainer 内有效；跨节点仍需序列化；感知端到端延迟可降 ~40% | 未使用 | P2 感知部署时 |
| **3** | lerobot/VLA 策略 | LeRobot HuggingFace 生态最成熟；ACT 适合精操作；Diffusion 轨迹最平滑；需 GPU OrinAGX | 未集成 | P3 策略训练 |
| **4** | MoveIt 2 | bio_ik 对冗余臂优于 KDL；STOMP 对复杂代价函数优于 OMPL；阻抗控制装配需 FT 传感器 | 未集成 | P2 机械臂部署 |
| **5** | Nav2 | MPPI 在动态障碍优于 DWB；collision_monitor 是独立安全层（强烈推荐）；BT 高度可定制 | 未集成 | P2 自主导航 |
| **6** | Isaac Sim/Lab | Isaac Lab 1024 并行环境 RTX4090 可行；omni.replicator 合成数据管线已成熟 | 未搭建 | P3 数据采集 |
| **7** | micro-ROS | ESP32 micro-ROS 适合 < 10Hz 传感器；Zenoh XRCE-DDS 桥延迟 ~1ms | chassis ESP32 可集成 | P3 MCU 集成 |
| **8** | 实时性优化 | PREEMPT_RT + CPU 隔离（isolcpus=4-7）是基线；cyclictest < 50μs 为合格 | 未配置 RT 内核 | P1 RT 优化 |
| **9** | 仿真桥接 | GazeboSystem plugin 双向桥接；xacro sim_mode 切换；ChassisHalMock 实现 SI 接口 | 07_integration 有完整方案 | P2 仿真环境 |
| **10** | 感知栈 | RT-DETR TRT 部署 < 10ms；nvblox 3D 重建实时 @ Orin；FoundationPose 零样本 6D 抓取 | 未集成 | P2 感知部署 |
| **11** | 安全/OTA | sros2 控制话题加密必须；cosign 镜像签名成本极低；ESP32 OTA 签名可行 | 未配置 | P1 安全基线 |
| **12** | 深化专题 | Ruckig 实时轨迹适合精操作；MPPI GPU 加速可用；数据飞轮完整架构设计完成 | 本 Phase 研究产出 | 参考实施 |

---

## 二、优先级实施路线图

### P0（立即，< 1 周）

```
✅ 基础安全：
   └── 启用 cosign 镜像签名（GitHub Actions CI 集成，约 1 人天）
   └── sros2 /cmd_vel + /joint_commands 加密配置

✅ 代码质量：
   └── chassis_protocol 接入 ros2_control SystemInterface（替换自定义 HAL 注册）
       — 解锁 Controller Manager、Chainable Controller、热切换等生态特性
```

### P1（近期，1–4 周）

```
🔧 实时性基线：
   ├── 部署 PREEMPT_RT 内核（Jetson Orin 或 x86 开发机）
   ├── 配置 CPU 隔离（isolcpus=4-7）
   └── cyclictest 验证 < 50μs

🔧 安全强化：
   ├── sros2 全话题 ACL 配置
   └── permissions.xml 维护脚本

🔧 仿真环境：
   ├── Gazebo Harmonic 安装 + GazeboSystem plugin
   └── xacro sim_mode 切换（实机/仿真 URDF）
```

### P2（中期，1–3 月）

```
🚀 感知栈（Phase 10）：
   ├── RT-DETR TRT 部署（目标检测）
   ├── nvblox 3D 场景重建
   └── FoundationPose 6D 抓取位姿估计

🚀 导航（Phase 5）：
   ├── Nav2 MPPI Controller 调优
   ├── nav2_collision_monitor 独立安全层
   └── SmacPlannerLattice（麦轮全向规划）

🚀 机械臂（Phase 4）：
   ├── MoveIt 2 bio_ik 集成
   └── 力控装配（FT 传感器 + 阻抗控制器）
```

### P3（长期，3–12 月）

```
🌐 数据飞轮：
   ├── LeRobot 数据管线 + DVC 版本管理
   ├── Isaac Lab 大规模并行仿真
   ├── 在线数据质量筛选器
   └── EWC 持续学习防遗忘

🌐 VLA 策略：
   ├── ACT / Diffusion Policy 训练
   ├── OpenVLA / pi0 推理部署
   └── 策略评估 → OTA 更新闭环

🌐 OTA 平台：
   ├── Mender.io 或 OSTree A/B 更新
   └── MCU 固件 OTA（ED25519 签名）
```

---

## 三、10 个开放问题

> 以下是经过 12 Phase 研究后仍需进一步探索的关键问题：

| # | 问题 | 相关 Phase | 研究方向 |
|---|------|-----------|---------|
| 1 | **chassis_protocol → ros2_control 迁移的完整工程量？** | Phase 1, 9 | 评估 SystemInterface 实现难度；HAL 层 API 对齐 |
| 2 | **Jetson Orin AGX PREEMPT_RT 内核稳定性？** | Phase 8 | 测试 5.15-rt 内核在 AGX 上的 USB/PCIe 稳定性 |
| 3 | **nvblox + MPPI 联合使用时 CPU/GPU 资源冲突？** | Phase 5, 10 | Profile Orin AGX 上的 CUDA 流竞争情况 |
| 4 | **FoundationPose 对透明/反光物体的失败模式？** | Phase 10 | 添加偏振相机或结构光补充深度 |
| 5 | **EWC λ 值对不同任务域的最优范围？** | Phase 12 | 在 lerobot 基准上系统搜索 λ（100~1000） |
| 6 | **sros2 在 10+ 节点系统的证书管理开销？** | Phase 11 | 自动化证书轮换；评估 Vault + 动态证书 |
| 7 | **STOMP 在狭窄空间（< 5cm 间隙）的收敛率？** | Phase 12 | 与 CHOMP 对比；调整 cost function 权重 |
| 8 | **micro-ROS ESP32 ↔ ROS 2 在高频（> 100Hz）下的稳定性？** | Phase 7 | 测试 Zenoh XRCE-DDS 桥在 200Hz 下的丢包率 |
| 9 | **LeRobot ACT 在真实抓取任务的 Zero-Shot 泛化范围？** | Phase 3 | 评估未见过的物体类别 + 光照变化 |
| 10 | **数据飞轮的最小可行 episode 数（启动成本）？** | Phase 12 | 实验：50/100/500 episodes 对策略性能的影响 |

---

## 四、关键工具版本矩阵（汇总）

| 工具/框架 | 推荐版本 | 说明 |
|---------|---------|------|
| ROS 2 | Humble（LTS 到 2027） | 生产环境首选 |
| Gazebo | Harmonic（ROS 2 Humble 对应） | 替代已弃用的 Gazebo Classic |
| Isaac Sim | 4.2+ | 需要 RTX GPU |
| Isaac Lab | 2.0+ | 基于 Isaac Sim 4.x |
| MoveIt 2 | Humble 对应版本（2.5+） | |
| Nav2 | Humble 对应（1.2+） | |
| nvblox | 0.0.5+（Isaac ROS） | |
| lerobot | 0.3+ | HuggingFace |
| OpenVLA | 7B（Llama 3.1 backbone） | |
| SROS2 | Humble 内置 | |
| cosign | 2.x | keyless 签名 |
| DVC | 3.x | 数据版本管理 |
| MLflow | 2.x | 实验追踪 |

---

## 五、完整文档参考索引

### Phase 1–4（基础架构）

| Phase | 目录 | 文档数 | 核心文件 |
|-------|------|-------|---------|
| 1 | realtime_research/ | 8 | 07_integration.md（chassis_protocol 对接） |
| 2 | nitros_research/ | 5 | 04_integration.md |
| 3 | lerobot_research/ | 6 | 05_integration.md |
| 4 | moveit_research/ | 6 | 05_integration.md |

### Phase 5–8（中间层）

| Phase | 目录 | 文档数 | 核心文件 |
|-------|------|-------|---------|
| 5 | nav2_research/ | 6 | 05_integration.md |
| 6 | isaac_research/ | 6 | 05_integration.md |
| 7 | microros_research/ | 5 | 04_integration.md |
| 8 | （含于 realtime_research） | — | 07_integration.md |

### Phase 9–12（高阶专题）

| Phase | 目录 | 文档数 | 核心文件 |
|-------|------|-------|---------|
| 9 | simulation_research/ | 8 | 07_integration.md（数字孪生 + 决策矩阵） |
| 10 | perception_research/ | 6 | 05_integration.md（35ms 端到端延迟分析） |
| 11 | security_research/ | 4 | 03_integration.md（分层安全架构） |
| 12 | deepdive_research/ | 6 | 05_summary.md（本文档） |

---

## 六、快速参考卡片

```
具身智能 ROS 2 架构快速参考（Phase 12 总结版）：

硬件层：
  Jetson Orin AGX（主算力）
  + ESP32（低功耗 MCU → micro-ROS）
  + FT 传感器、LiDAR、RGB-D、IMU

实时控制（≤ 1ms 响应）：
  PREEMPT_RT + CPU 隔离
  → ros2_control（1kHz）
  → chassis_protocol HAL
  → 电机驱动

感知（≤ 35ms 端到端）：
  Isaac ROS NITROS 零拷贝
  → RT-DETR TRT（检测 <10ms）
  → nvblox（3D 重建，实时）
  → FoundationPose（6D 抓取位姿）

导航（MPPI + BT）：
  Nav2 + SmacPlannerLattice
  + collision_monitor（独立安全层）

操作（MoveIt 2）：
  bio_ik + STOMP
  + 阻抗控制装配

VLA 策略（端到端）：
  OpenVLA / pi0 → ACT / Diffusion Policy
  → 数据飞轮（EWC + DVC + MLflow）

安全（纵深防御）：
  sros2（DDS-Security）
  + cosign（镜像签名）
  + OSTree A/B OTA
  + collision_monitor（物理安全）
```
