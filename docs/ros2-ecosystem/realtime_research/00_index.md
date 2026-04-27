# 实时性 / PREEMPT_RT / iceoryx2 — 总索引

> **调研范围**：具身智能机器人实时控制栈的完整技术图谱——从 PREEMPT_RT 内核补丁到 ROS 2 实时执行器、iceoryx2 零拷贝 IPC、DDS 调优，再到 ros2_control 实时控制环的具体实现  
> **关联文档**：
> - `docs/ros2-ecosystem/ros2_control_research/`（Phase 1，controller_manager 实时循环）
> - `docs/ros2-ecosystem/rosbag2_research/07_integration.md`（Phase 6，进程隔离建议）
> - `docs/ros2-core/ros2_topic_communication_research/`（SHM/Loaned Message 基础）

---

## 一、具身智能实时性需求矩阵

具身智能系统横跨多个频率域，不同子系统对延迟和抖动有截然不同的要求：

| 控制层 | 典型频率 | 最大允许周期抖动 | 最坏情况延迟 | 故障后果 |
|--------|---------|----------------|------------|--------|
| **安全停止（E-Stop）** | 事件驱动 | — | **< 1 ms** | 人员/设备伤害 |
| **力 / 力矩控制（Force Control）** | 4 kHz | ± 25 μs | < 250 μs | 接触不稳定、振荡 |
| **关节位置 / 速度控制** | 1 kHz | ± 100 μs | < 1 ms | 轨迹跟踪误差 |
| **阻抗 / 导纳控制** | 500 Hz – 1 kHz | ± 200 μs | < 2 ms | 柔顺性丧失 |
| **视觉伺服（高速）** | 100 Hz | ± 1 ms | < 10 ms | 抓取失败 |
| **视觉伺服（标准）** | 30 Hz | ± 5 ms | < 33 ms | 末端偏移 |
| **VLA 推理（策略输出）** | 10–30 Hz | ± 10 ms | < 100 ms | 任务失败 |
| **导航规划** | 5–10 Hz | ± 20 ms | < 200 ms | 路径偏差 |
| **遥测 / rosbag2 录制** | 异步 | 宽松 | 宽松 | 数据丢失 |

> **关键结论**：安全停止和力控是硬实时需求，必须使用 PREEMPT_RT 内核 + SCHED_FIFO。关节控制（1 kHz）在 PREEMPT_RT 下可满足，视觉伺服及以上层次在普通 Linux 下也可满足。

---

## 二、完整实时栈架构图

### 2.1 软件栈纵向视图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        具身智能实时控制栈                                      │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  应用层（非 RT）                                                       │   │
│  │  VLA 推理 / 导航规划 / Foxglove 监控 / rosbag2 录制                   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│            ↕  DDS（CycloneDDS BEST_EFFORT / RELIABLE）                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  ROS 2 中间层（部分 RT）                                               │   │
│  │  ├─ rclcpp::StaticSingleThreadedExecutor（SCHED_FIFO prio 80）      │   │
│  │  ├─ realtime_tools::RealtimePublisher                               │   │
│  │  ├─ tf2_ros::TransformBroadcaster（非 RT 核）                        │   │
│  │  └─ DDS threads → 非 RT 核（cpuset 隔离）                            │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│            ↕  iceoryx2 共享内存（sub-microsecond IPC）                       │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  ros2_control 实时层（硬实时）                                         │   │
│  │  ├─ controller_manager::update()  @ 1–4 kHz（SCHED_FIFO prio 90）  │   │
│  │  │    read()  → update()  → write()                                 │   │
│  │  ├─ hardware_interface::SystemInterface                              │   │
│  │  │    ChassisHal::read() / write()（无锁、无 malloc）                │   │
│  │  └─ ChainableController（impedance / admittance）                   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│            ↕  RS485 / EtherCAT DMA（确定性传输）                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  操作系统层                                                           │   │
│  │  ├─ PREEMPT_RT 内核（linux-realtime 或手动打补丁）                   │   │
│  │  ├─ CPU isolation（isolcpus / cpuset cgroup）                       │   │
│  │  ├─ mlockall(MCL_CURRENT | MCL_FUTURE)（防止 page fault）           │   │
│  │  ├─ hrtimer（高精度定时器，CONFIG_HZ_1000）                          │   │
│  │  └─ IRQ affinity → 非 RT 核                                         │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│            ↕  硬件抽象                                                       │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  硬件层                                                               │   │
│  │  ├─ x86 工控机（推荐 Intel i7/i9，关闭 SMI / C-states）              │   │
│  │  ├─ Jetson Orin（ARM Cortex-A78AE，NVIDIA RT 内核包）               │   │
│  │  └─ EtherCAT 主站 / RS485 驱动器 / USB-Serial                       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 水平进程隔离视图（CPU 核心分配）

```
┌─────────────────────────────────────────────────────────────────────────┐
│  物理 CPU（8核示例：Core 0-7）                                             │
│                                                                         │
│  Core 0    ┌────────────────────┐   系统服务 / IRQ 默认亲和               │
│            │  OS + IRQ + DDS   │   (非隔离核)                            │
│  Core 1    └────────────────────┘                                       │
│                                                                         │
│  Core 2    ┌────────────────────┐   SCHED_FIFO prio 90                  │
│            │ controller_manager │   isolcpus=2,3                        │
│  Core 3    │ hardware_interface │   nohz_full=2,3                       │
│            └────────────────────┘                                       │
│                                                                         │
│  Core 4    ┌────────────────────┐   SCHED_FIFO prio 70                  │
│            │  DDS 发布/订阅线程  │   CycloneDDS recv/send                 │
│  Core 5    └────────────────────┘                                       │
│                                                                         │
│  Core 6    ┌────────────────────┐   SCHED_OTHER（普通优先级）             │
│            │ rosbag2 录制进程   │   不影响 RT 核                          │
│            └────────────────────┘                                       │
│                                                                         │
│  Core 7    ┌────────────────────┐   SCHED_OTHER                         │
│            │ VLA 推理 / 监控    │   GPU 异步工作                          │
│            └────────────────────┘                                       │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 三、延迟预算分配表

以 1 kHz 关节控制环（总预算 1 ms = 1,000,000 ns）为例：

| 延迟来源 | 典型值 | PREEMPT_RT 目标 | 说明 |
|---------|-------|----------------|------|
| OS 调度抖动（jitter） | 普通: 100–500 μs | **< 50 μs** | cyclictest 验证；关键靠 isolcpus + nohz_full |
| SMI 硬件中断 | 0–300 μs | **< 10 μs**（关闭 C-states 后） | hwlatdetect 验证 |
| hrtimer 精度误差 | 1–10 μs | **< 5 μs** | CONFIG_HZ_1000 + hrtimer |
| DDS 发布（本机 SHM） | 5–50 μs | **< 20 μs**（iceoryx2: < 1 μs） | rmw_iceoryx 可极大降低 |
| controller_manager dispatch | 2–5 μs | **< 5 μs** | StaticSingleThreadedExecutor |
| controller update() 计算 | 10–100 μs | **< 200 μs** | 与算法复杂度相关 |
| hardware_interface read/write | 20–500 μs | **< 200 μs** | 取决于总线类型（EtherCAT < 100μs；RS485 变长） |
| **合计（最坏情况）** | | **< 500 μs** | 1 ms 预算的 50%，留足余量 |

对于 4 kHz 力控（总预算 250 μs）：OS 抖动必须 < 20 μs，hardware_interface 必须 < 50 μs（通常需要 EtherCAT 或专用高速总线）。

---

## 四、与本仓库关联速查

| 本仓库组件 | 实时性关联点 | 相关文档 |
|-----------|------------|---------|
| `chassis_protocol/chassis/chassis_hal.cpp` | `ChassisHal::read()` / `write()` 中存在 mutex 和动态分配，需 RT 化 | `07_integration.md §1` |
| `chassis_protocol/transport/rs485_transport.cpp` | RS485 阻塞 `select()` 超时策略影响确定性 | `07_integration.md §1` |
| `chassis_protocol/chassis/version_negotiator.cpp` | 版本协商逻辑应移出 RT 线程，在 `on_activate()` 中一次性执行 | `07_integration.md §1` |
| Phase 6 `rosbag2_research/07_integration.md` | 录制进程 CPU 隔离建议；与 controller_manager 优先级隔离 | `07_integration.md §2` |
| `ros2_control_research/02_controller_manager.md` | CM update() 时序、RT 线程绑定方法 | `05_ros2_control_rt.md` |

---

## 五、文档导航表

| 文件 | 内容摘要 |
|------|---------|
| [01_preempt_rt_kernel.md](./01_preempt_rt_kernel.md) | PREEMPT_RT 补丁原理、Ubuntu 22.04/24.04 安装、内核配置项、BIOS 调优、cyclictest/hwlatdetect 测试方法、Jetson Orin RT 内核 |
| [02_ros2_realtime.md](./02_ros2_realtime.md) | 内存锁定、CPU 隔离、线程优先级、Executor 实时对比、realtime_tools 用法、零拷贝 IPC、完整 RT 节点模板 |
| [03_iceoryx2.md](./03_iceoryx2.md) | iceoryx2 零拷贝 IPC 架构、RouDi 守护进程、rmw_iceoryx RMW 层、性能数据、WaitSet、与 ros2_control 集成 |
| [04_dds_tuning.md](./04_dds_tuning.md) | RMW 四维对比、CycloneDDS/FastDDS RT 配置、QoS 实时参数、网络栈调优、选型决策矩阵 |
| [05_ros2_control_rt.md](./05_ros2_control_rt.md) | controller_manager update() 时序、硬件接口 RT 约束、ChainableController 延迟叠加、力控 4kHz 配置、EtherCAT/RS485 实现要点 |
| [06_latency_profiling.md](./06_latency_profiling.md) | ros2_tracing+LTTng、tracetools_analysis、cyclictest、perf+ftrace、端到端延迟测量、典型问题诊断 |
| [07_integration.md](./07_integration.md) | chassis_protocol RT 改造路线图、Phase 6 录制进程隔离、整体部署图、Jetson Orin 特化、容器化 RT、选型决策矩阵 |

---

## 六、实时栈技术选型速览

```
                   延迟要求
                     ↓
          ┌──────────────────────┐
          │  < 100 μs（力控）？   │
          └──────────┬───────────┘
                     │ 是                    否（1ms 关节控制）
                     ↓                         ↓
          ┌──────────────────────┐   ┌──────────────────────┐
          │  PREEMPT_RT 必须     │   │  PREEMPT_RT 推荐      │
          │  EtherCAT 总线       │   │  RS485/USB 可接受     │
          │  iceoryx2 IPC        │   │  CycloneDDS SHM 可用  │
          │  SCHED_FIFO prio 90  │   │  SCHED_FIFO prio 80   │
          └──────────────────────┘   └──────────────────────┘
                     │                         │
                     ↓                         ↓
          ┌──────────────────────────────────────────────────┐
          │  两者共同基础：                                     │
          │  - mlockall(MCL_CURRENT | MCL_FUTURE)            │
          │  - isolcpus + nohz_full                          │
          │  - StaticSingleThreadedExecutor                  │
          │  - hardware_interface 中禁止 malloc/mutex/printf  │
          └──────────────────────────────────────────────────┘
```
