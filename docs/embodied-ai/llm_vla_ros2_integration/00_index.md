# LLM / VLA 与 ROS 2 集成模式 — 总索引

> **调研范围**：将"大模型推理"接入"具身智能控制环"的系统工程方法  
> **关联文档**：  
> - 本仓库 `docs/ros2-ecosystem/ros2_control_research/`（Phase 1，控制环边界）  
> - 本仓库 `docs/ros2-ecosystem/isaac_ros_nitros_research/`（Phase 2，GPU 感知管线）  
> - REP-2000（ROS 2 版本政策）、OpenVLA / RT-2 / π0 原始论文

---

## 一、核心挑战：时间尺度的鸿沟

具身智能系统同时跑着"思考速度"截然不同的两类组件：

```
┌─────────────────────────────────────────────────────────────────────┐
│  组件                 │ 典型周期    │ 延迟要求   │ 失败代价         │
├─────────────────────────────────────────────────────────────────────┤
│  LLM 任务规划         │ 0.5-10 s   │ 宽松        │ 重试             │
│  VLA 动作推理         │ 50-500 ms  │ 中等        │ 运动停顿         │
│  行为树 / 状态机       │ 10-100 ms  │ 软实时      │ 任务失败         │
│  MoveIt 2 路径规划    │ 50-500 ms  │ 软实时      │ 碰撞             │
│  ros2_control 控制环  │ 1-10 ms    │ 硬实时      │ 机械损坏/倾倒    │
│  安全监控（E-Stop）    │ < 1 ms     │ 硬实时      │ 人身安全         │
└─────────────────────────────────────────────────────────────────────┘
```

**核心设计原则**：**上层慢、下层快；上层可重试、下层不可中断**。任何集成方案都必须在不同时间尺度的层之间设置清晰的异步接口与安全降级路径。

---

## 二、分层架构总图

```
┌───────────────────────────────────────────────────────────────────────┐
│                     Layer 4：认知层（0.5-10 s）                        │
│                                                                       │
│  LLM 任务规划（GPT-4o / Llama3 / Qwen2.5）                            │
│    输入：场景描述（来自 VLM）+ 用户指令                                 │
│    输出：自然语言任务序列 / 工具调用序列 / 代码片段                      │
│    接口：ROS 2 Action Server（长时 Goal）/ REST API                    │
├───────────────────────────────────────────────────────────────────────┤
│                     Layer 3：技能层（50-500 ms）                       │
│                                                                       │
│  VLA 动作推理（OpenVLA / π0 / RDT / RT-2）                            │
│    输入：图像（NitrosImage）+ 语言条件                                 │
│    输出：动作 token → ΔJoint / ΔEE Pose / JointTrajectory             │
│    接口：ROS 2 Topic（NitrosTensorList）/ Action                      │
├───────────────────────────────────────────────────────────────────────┤
│                     Layer 2：协调层（10-100 ms）                       │
│                                                                       │
│  行为树（Nav2 BT / BehaviorTree.CPP v4）                              │
│  技能抽象（Pick / Place / Navigate / Grasp）                          │
│    接口：MoveIt 2 Action / Nav2 Action / ros2_control Action          │
├───────────────────────────────────────────────────────────────────────┤
│                     Layer 1：控制层（1-10 ms）                         │
│                                                                       │
│  ros2_control（JTC / DiffDriveController / 自定义）                   │
│  chassis_protocol HAL（串口 / RS485 / UDP）                           │
│    接口：StateInterface（位置/速度/力矩）+ CommandInterface            │
├───────────────────────────────────────────────────────────────────────┤
│                     Layer 0：安全层（< 1 ms）                          │
│                                                                       │
│  E-Stop Monitor / 力矩限制 / 碰撞停止（硬件或 FPGA）                   │
│  与 ros2_control 生命周期（deactivate）联动                            │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 三、关键集成接口速查

| 接口 | 从哪层 | 到哪层 | ROS 2 机制 | 主要数据 |
|------|--------|--------|-----------|---------|
| 任务指令 | Layer 4 LLM | Layer 3 VLA / Layer 2 BT | `action` (Goal/Result/Feedback) | 自然语言 string / task struct |
| 视觉感知 | NITROS 感知管线 | Layer 4 LLM（VLM） | `topic` (NitrosImage) | 图像 tensor |
| 动作 token | Layer 3 VLA | Layer 2 BT | `topic` / `action` | float[] / JointTrajectory |
| 关节轨迹 | Layer 2 MoveIt | Layer 1 ros2_control | `action` (FollowJointTrajectory) | JointTrajectory |
| cmd_vel | Layer 2 Nav2 | Layer 1 chassis_protocol | `topic` (Twist) | Twist |
| 状态反馈 | Layer 1 ros2_control | Layer 2/3 | `topic` (JointState) | pos/vel/eff |
| E-Stop | 任意层 | Layer 0 | `topic` + 硬件 GPIO | bool |

---

## 四、文档目录

| 文件 | 内容 |
|------|------|
| [01_task_hierarchy.md](./01_task_hierarchy.md) | 四层任务分层详解、每层边界契约、异步通信模式 |
| [02_vla_output_formats.md](./02_vla_output_formats.md) | OpenVLA / RT-2 / π0 / RDT 输出格式对比，转换层（Action Bridge）设计 |
| [03_rai_llm_agent.md](./03_rai_llm_agent.md) | `rai`（Robec）框架：LLM Agent + ROS 2 工具调用模式 |
| [04_ros2_llm_observability.md](./04_ros2_llm_observability.md) | `ros2_llm` 包、Foxglove + LLM 可观测性集成 |
| [05_isaac_manipulator_groot.md](./05_isaac_manipulator_groot.md) | NVIDIA Isaac Manipulator + GROOT 工作流编排 |
| [06_inference_deployment.md](./06_inference_deployment.md) | 本机 GPU / 远端 gRPC / Triton Inference Server 部署方式对比 |
| [07_data_flywheel.md](./07_data_flywheel.md) | rosbag2 采集 → 清洗 → VLA 训练 → 回放验证数据闭环 |
| [08_integration.md](./08_integration.md) | 5 个开源方案横向对比表 + 选型决策矩阵 |

---

## 五、与本仓库已有工作的交叉引用

| 已有工作 | 与 Phase 3 的关联 |
|---------|-----------------|
| `ros2_control_research/02_controller_manager.md` | ros2_control 实时循环是 Layer 1 的实现基础；VLA 动作 token 最终要落地为 CommandInterface |
| `ros2_control_research/10_integration.md` | chassis_protocol → SystemInterface 迁移后，底盘控制接口统一为 cmd_vel/JointTrajectory |
| `isaac_ros_nitros_research/06_package_ecosystem.md` | TensorRT 推理节点 = VLA Layer 3 的核心执行引擎 |
| `isaac_ros_nitros_research/09_integration.md` | NITROS D4/D5/D6 零拷贝：VLA 的视觉输入路径无需 CPU 拷贝 |
| `ros2_custom_fork_roadmap/04_phase2_rclcpp_executor.md` | EventsExecutor：Layer 2 行为树节点的调度延迟优化 |
