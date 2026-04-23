# 独立维护 ROS 2 定制版本 —— 性能优化路线图

> **读者画像**：具身智能软件工程师 / 中间件工程师 / 系统架构师
> **目标**：明确"Fork 一套 ROS 2 并长期维护，以获得更高性能"的**全景图**——能做什么、代价是什么、分阶段怎么做。
> **前置知识**：熟悉 rclcpp / DDS 基础概念；若不熟，请先阅读 [ros2_topic_communication_research](../ros2_topic_communication_research/) 与 [ros2_rclcpp_core_research](../ros2_rclcpp_core_research/)。

---

## 为什么要这个专题

官方 ROS 2（Humble / Iron / Jazzy）是通用机器人中间件，其默认参数**在"功能覆盖"与"开箱即用"之间做了折中**，并非为具身智能场景的极限性能优化：

- 一台双足人形 / 四足 / 灵巧手的控制闭环通常要求 **≥ 500 Hz**，端到端延迟 **P99 < 2 ms**，抖动 σ < 200 μs；
- 视觉-触觉-本体感融合的消息尺寸从几 KB 到几 MB 不等，单机节点数常超 40+，DDS 发现风暴、默认 Executor 的 O(N) 扫描、rmw 的多次拷贝都会成为**系统性瓶颈**；
- 通用配置下，Ubuntu 22.04 + 默认 FastDDS，**单机 1 KB 消息 P99 延迟 0.5–1.5 ms**、**10 MB 消息 3–15 ms**（端到端含序列化、传输、Executor 调度）；
- 与此同时，**硬实时（PREEMPT_RT + SCHED_FIFO + CPU 隔离 + IRQ 亲和 + 大页 + 零拷贝）** 的可达下界通常是 **P99 < 200 μs、抖动 < 50 μs**——**存在一个数量级的优化空间**。

Fork 的收益来自于**放弃"通用性折中"**；代价则来自于**长期维护一套偏离上游的代码库**——每一项优化都在"性能收益"和"维护成本"两条轴上同时前进。本专题的价值在于：**把每一个取舍都量化、分阶段呈现，避免盲目优化或盲目追随上游**。

---

## 文档结构

| 文档 | 内容 | 面向决策 |
|------|------|---------|
| [01 动机与基线](./01_motivation_and_baseline.md) | 为什么 fork；官方实现的典型瓶颈；性能基线测量 | 是否值得 fork |
| [02 可优化维度](./02_optimization_dimensions.md) | 12 个维度的性能杠杆与量化指标 | 优化方向选择 |
| [03 阶段 1：配置级优化](./03_phase1_config_tuning.md) | 零代码改动：QoS / 编译选项 / rmw 实现切换 | 起步收益 |
| [04 阶段 2：rclcpp 改造](./04_phase2_rclcpp_executor.md) | Executor 重写、内存池、回调组重构、固定分配 | 中期收益 |
| [05 阶段 3：rmw 零拷贝](./05_phase3_rmw_zero_copy.md) | Loaned Message 深化、rmw 层去冗余拷贝 | 大消息收益 |
| [06 阶段 4：SHM / Iceoryx](./06_phase4_dds_shm_iceoryx.md) | FastDDS SHM / Iceoryx / 单机零拷贝总线 | 同机多节点收益 |
| [07 阶段 5：替换 DDS](./07_phase5_replace_dds.md) | Zenoh / 自研协议 / 去 DDS 化 | 远景收益 |
| [08 阶段 6：实时化](./08_phase6_rt_and_kernel.md) | PREEMPT_RT、CPU 隔离、IRQ 亲和、内存锁定 | 抖动收益 |
| [09 维护策略](./09_maintenance_strategy.md) | 上游追赶、patch 管理、回归测试、CI | 长期可持续性 |
| [10 基准方法论](./10_benchmark_methodology.md) | performance_test / iRobot bench / 自建看板 | 衡量改进 |
| [11 决策矩阵](./11_decision_matrix.md) | ROI 对比、阶段准入/退出条件、风险矩阵 | 投入产出评估 |

---

## 阅读顺序建议

- **系统架构师**（评估是否要做 Fork）：01 → 02 → 11 → 09
- **中间件工程师**（准备动手改代码）：01 → 03 → 04 → 05 → 06 → 10
- **实时系统工程师**：08 → 10 → 04
- **项目经理 / TL**：01 → 02 → 09 → 11

---

## 核心判断（TL;DR）

1. **不要轻易 fork 整个 ROS 2**。80% 的收益来自**阶段 1–2**（配置 + rclcpp 局部改造），且这部分可以以"downstream 包"的形式叠加在官方发行版上，不需要 fork。
2. **真正值得 fork 的只有 3 个子项目**：`rclcpp`、`rmw_*`（或你选的 DDS）、以及极少数 `rcl_*` 修补——且最好以 **独立 rclcpp fork + pinned rmw + patch-set 形式**管理，而不是 fork 整个 colcon workspace。
3. **每引入一项偏离上游的修改，至少同时建立**：(a) 独立的性能基准用例、(b) 与上游等价性的回归测试、(c) 每 6 个月一次的 rebase 成本评估。
4. **实时化（阶段 6）的收益远大于算法级优化**——在未做 PREEMPT_RT + CPU 隔离的系统上做 rclcpp 优化，P99 抖动瓶颈不会被消除。
5. **终极方案不是"更快的 DDS"，而是"合适的传输"**：控制环 → 共享内存 / 总线直通；感知流 → DDS / Zenoh；跨机协调 → DDS / Zenoh；云边协同 → Zenoh / gRPC。

---

## 相关专题

- [ros2_topic_communication_research](../ros2_topic_communication_research/) — 通信基础（发布/订阅/序列化）
- [ros2_rclcpp_core_research](../ros2_rclcpp_core_research/) — Executor / Node 设计
- [ros2_qos_research](../ros2_qos_research/) — QoS 三层映射
- [ros2_service_discovery_research](../ros2_service_discovery_research/) — SPDP / SEDP 发现协议
