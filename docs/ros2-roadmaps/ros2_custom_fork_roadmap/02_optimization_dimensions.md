# 02 可优化维度 —— 12 条性能杠杆

> **读法**：每个维度给出"**杠杆原理 / 上限收益 / 代价 / 与阶段的对应**"。不是所有杠杆都要拉满——有些杠杆之间互斥或边际收益递减。

## 杠杆总览

```
                                延迟↓           吞吐↑    抖动↓    代价
─────────────────────────────────────────────────────────────────────
D1  Executor 模型               ★★★            ★★       ★★★      中
D2  回调组 & 线程模型            ★★             ★★★      ★★       低
D3  内存分配策略                 ★★             ★        ★★★      中
D4  消息拷贝次数（零拷贝）        ★★★            ★★★      ★        高
D5  序列化格式                   ★★             ★★       ★        高
D6  传输层（UDP/SHM/Iceoryx）    ★★★            ★★★      ★★       中
D7  DDS 选择 & 调参              ★★             ★★       ★        低
D8  QoS 策略                    ★★             ★        ★★       低
D9  发现协议                    ★              ★        ★★       中
D10 实时内核 & 调度              ★              ★        ★★★      中
D11 CPU 亲和 & IRQ 亲和          ★              ★        ★★★      低
D12 编译器 & LTO & PGO          ★              ★        ★        低
```

---

## D1. Executor 模型

**原理**：默认 `SingleThreadedExecutor` 在 `wait_for_work` 返回后**每次都扫描所有 entity**（O(N) 查找可执行项）。`MultiThreadedExecutor` 虽然并发但存在**全局锁**（wait_mutex_）。真正能降延迟的是事件驱动的 `EventsExecutor`（rclcpp 在 Iron 之后引入）与**应用特化 Executor**（静态注册、静态调度表）。

**可达收益**：
- 从默认 `SingleThreadedExecutor` → `EventsExecutor`：单次调度延迟 5–15 μs → 1–3 μs；
- 静态 Executor（硬编码回调顺序）：< 500 ns；
- 40+ entity 场景下，扫描开销从 20 μs 降到 < 1 μs。

**代价**：
- `EventsExecutor` 的 callback 必须线程安全；
- 静态 Executor 放弃动态增删 entity 的能力——具身智能控制环通常可接受；
- 维护成本中等：rclcpp 内部 API 不稳定，每次上游升级需要重新适配。

**对应阶段**：[04 阶段 2](./04_phase2_rclcpp_executor.md)

---

## D2. 回调组与线程模型

**原理**：`MutuallyExclusive` 回调组内的回调串行，`Reentrant` 内可并行。**但即使是 Reentrant，同一 Subscription 实例的回调仍串行**（rcl 层有 `take` 锁）。合理的线程模型是：
- 控制环独占一个高优先级线程（SCHED_FIFO）；
- 感知流共享一个低优先级线程池；
- 慢任务（参数服务 / tf 查询）走独立线程池。

**可达收益**：
- 控制环延迟 **不再受感知回调抖动影响**，P99 抖动降 50–80%；
- 吞吐提升 2–4×（如果之前只有 1 个线程）。

**代价**：
- 线程间共享数据需自行加锁 / lock-free；
- 回调组划分错误会导致**看似并行实则串行**或**死锁**（A 等 B 发布，B 被锁在 A 的回调组里）。

**对应阶段**：[03 阶段 1](./03_phase1_config_tuning.md) 配置即可获得一半收益；[04 阶段 2](./04_phase2_rclcpp_executor.md) 进一步定制。

---

## D3. 内存分配策略

**原理**：热路径上的 `malloc/new` 是**非实时**的——glibc 的 `ptmalloc` 有线程缓存但仍可能触发系统调用；rclcpp 的 `Allocator` 模板参数允许注入自定义分配器。

**方案梯度**：
1. **TLSF**（Two-Level Segregated Fit）：O(1) 确定性分配；
2. **tcmalloc / jemalloc**：更快的通用分配器，非确定性但吞吐高；
3. **Pool allocator**：为每种消息类型预分配固定大小的池；
4. **完全 alloc-free 热路径**：启动时 warm-up，运行时 0 分配。

**可达收益**：
- 高频回调的 P99 抖动降 3–10×（从几百 μs 到几十 μs）；
- 对于 `make_shared<Msg>` 类操作，使用 `make_shared_for_overwrite` + pool 可降 80% 开销。

**代价**：
- 需要审计整个调用栈确保没有隐式分配（`std::string`、`std::vector::reserve` 默认倍增、`shared_ptr` 控制块等）；
- Pool 的生命周期管理复杂，释放乱序会导致碎片；
- 兼容性：第三方库（如 ros2_eigen）可能不走自定义 allocator。

**对应阶段**：[04 阶段 2](./04_phase2_rclcpp_executor.md)。

---

## D4. 消息拷贝次数（零拷贝）

**原理**：参见 [01 章 §2.2](./01_motivation_and_baseline.md#22-rclcpp--rcl--rmw--dds-的消息拷贝次数)。降到 0 拷贝需要：
- IntraProcess + `unique_ptr` 传递（同进程 0 拷贝）；
- `borrow_loaned_message` + SHM 传输（跨进程 0 拷贝）；
- 整条链路的 QoS / 内存模型兼容。

**可达收益**：
- 1 MB 消息端到端延迟从 3–15 ms 降到 50–200 μs（**10–100×**）；
- 吞吐从 ~800 MB/s 升到 ~10 GB/s（受内存带宽限制）。

**代价**：
- **仅 PoD 类型**（或 bitwise copyable）才能 Loaned；`std::vector` / `std::string` 不行，需改用固定容量的 `rosidl_runtime_cpp::BoundedSequence`；
- 消息类型需要 rosidl 生成时声明 `@verbatim ros2idl`；
- 所有链路都要支持：rclcpp、rmw、DDS、目标机器的 SHM 实现；
- 接口不稳定，Humble 与 Iron 的 API 有差异。

**对应阶段**：[05 阶段 3](./05_phase3_rmw_zero_copy.md) + [06 阶段 4](./06_phase4_dds_shm_iceoryx.md)。

---

## D5. 序列化格式

**原理**：默认 CDR v1 是**为 WAN 设计**的中立格式，但在同机 / 同架构下，直接二进制 memcpy 更快。

**方案**：
- **CDR v2**（DDS-XTypes 2.0）：默认 little-endian、无对齐填充，比 v1 快 10–20%；
- **CDR_LE**（固定小端）：跳过字节序协商，再快 5%；
- **自定义 flatbuffers / capnproto**：结构体即序列化，0 拷贝访问；
- **裸结构体**（only 同机 + 同版本）：彻底 0 序列化。

**可达收益**：
- 小消息（< 256 B）：序列化占比 30–40% → 0，约等于延迟减半；
- 大消息（> 1 MB）：主要受内存带宽限制，收益 10–20%；
- SIMD 优化的 CDR 实现（如 rmw_zenoh 的 flatbuffers 路径）对点云 2–3×。

**代价**：
- 放弃跨版本 / 跨架构 / 跨语言兼容；
- 需要新的 IDL 代码生成器或消息接口；
- 工具链（rosbag / rviz）要适配新格式。

**对应阶段**：[05 阶段 3](./05_phase3_rmw_zero_copy.md)（保留 CDR）或 [07 阶段 5](./07_phase5_replace_dds.md)（换格式）。

---

## D6. 传输层

**原理**：单机不同进程间通信，走内核 UDP → loopback → 内核 socket buffer，即使是 localhost 也有 5–15 μs 的内核调用与 2 次拷贝。替代方案：
- **SHM（共享内存）**：mmap + ringbuffer，0 拷贝，延迟 < 2 μs；
- **Iceoryx**：专为 ROS 2 设计的 SHM 中间件，2–5 μs P99；
- **Linux AF_UNIX**：比 UDP 少一次协议栈；
- **跨 NUMA / 跨机**：仍需 UDP/TCP，但可用 kernel bypass（DPDK / XDP）。

**可达收益**：
- 同机延迟 300 μs → 2 μs（**150×**）；
- 跨机延迟取决于网卡 / 交换机，10GbE 下 P99 可达 20–50 μs（AF_XDP / solarflare）。

**代价**：
- SHM 需要**所有通信方运行在同一内核 namespace 且共享 /dev/shm**，容器化场景要配置 `--ipc=host`；
- SHM 没有天然的崩溃恢复——死掉的 publisher 会遗留 shared ringbuffer 引用；
- Iceoryx 引入 RouDi daemon 作为单点依赖。

**对应阶段**：[06 阶段 4](./06_phase4_dds_shm_iceoryx.md)。

---

## D7. DDS 实现选择与调参

**原理**：FastDDS / CycloneDDS / Connext 有不同的设计取舍：
- **FastDDS**：多线程、特性全、有 SHM；默认配置偏保守；
- **CycloneDDS**：单线程 reactor、延迟抖动最小；社区首选 RT 场景；
- **Connext**：商业，性能和工具链最成熟；许可证成本。

**调参杠杆**：
- Reliability 历史缓冲大小；
- ACK/NACK 周期；
- Fragmentation 阈值（大消息分片）；
- Heartbeat 周期。

**可达收益**：
- 从默认 FastDDS 切换到 CycloneDDS + 调优，控制环 P99 抖动可降 30–50%；
- 大消息吞吐对比：Connext > CycloneDDS ≈ FastDDS（需启用 async publish mode）。

**代价**：
- DDS 切换涉及整套 QoS 默认值、URI、profile XML 差异；
- 某些 ROS 2 功能（actions / services）在不同 DDS 上的行为有微妙差别；
- 调参需要对 RTPS 协议有深入理解。

**对应阶段**：[03 阶段 1](./03_phase1_config_tuning.md)。

---

## D8. QoS 策略

**原理**：QoS 不仅是"功能开关"也是**性能参数**。见 [ros2_qos_research](../../ros2-core/ros2_qos_research/)。

**常见误用**：
- 控制环用 RELIABLE + history=10 → 每条消息都要 ACK + 缓存 10 条；
- 感知流用 RELIABLE → 单次丢包导致发布端积压；
- 未设 Deadline → 无法早期发现性能退化。

**可达收益**：
- 控制环改 BEST_EFFORT + KEEP_LAST=1 → 延迟降 20–40%；
- 感知流改 BEST_EFFORT → 吞吐升 2×。

**代价**：
- 语义改变：丢消息可能影响算法；
- 不同订阅者兼容性（Reliability 策略不匹配会导致无法通信）。

**对应阶段**：[03 阶段 1](./03_phase1_config_tuning.md)。

---

## D9. 发现协议

**原理**：SPDP/SEDP 的 O(N²) 开销 + 固定 3 s 周期在大规模部署下是瓶颈。替代方案：
- **Discovery Server**（FastDDS）：中心化发现，O(N)；
- **静态发现**（CycloneDDS 的 peer list）：完全省掉动态发现；
- **Zenoh routers**：分层发现，支持 WAN。

**可达收益**：
- 40 节点冷启动从 30–60 s 降到 2–5 s；
- 稳态 meta-traffic 从 5 Mbps 降到 < 500 Kbps。

**代价**：
- Discovery Server 是单点故障，需做 HA；
- 静态发现失去热插拔能力；
- 调试体验变差（`ros2 node list` 行为改变）。

**对应阶段**：[03 阶段 1](./03_phase1_config_tuning.md)。

---

## D10. 实时内核与调度

**原理**：标准 Linux 的抢占粒度为 10 ms（HZ=100）或 1 ms（HZ=1000）；PREEMPT_RT 将内核大部分不可抢占段变为可抢占，抢占粒度 < 100 μs。

**方案层级**：
1. `CONFIG_PREEMPT` → `CONFIG_PREEMPT_RT`；
2. `SCHED_FIFO` / `SCHED_DEADLINE` 进程优先级；
3. CPU 隔离（`isolcpus=` 内核参数）；
4. IRQ 亲和（将 DMA / 网卡 IRQ 绑定到非隔离核）。

**可达收益**：
- P99 抖动 σ：200 μs → 30 μs；
- 最差情况延迟（P99.99）：毫秒级 → 百 μs 级。

**代价**：
- PREEMPT_RT 内核吞吐比标准内核低 5–15%；
- 驱动兼容性（NVIDIA GPU 驱动在 RT 内核上历来有问题，虽然在逐步改善）；
- 需要定期同步上游 RT patch。

**对应阶段**：[08 阶段 6](./08_phase6_rt_and_kernel.md)。

---

## D11. CPU 亲和与 IRQ 亲和

**原理**：
- 将控制线程钉到**专用核**，该核通过 `isolcpus` 或 cgroup 不被调度器分配其他任务；
- 将网卡 IRQ 从控制核搬走；
- 配合 `nohz_full` 降低时钟中断；
- 大页（`transparent_hugepage=never` + 手动 hugepages）避免 TLB miss。

**可达收益**：
- 抖动 σ 再降 30–50%；
- 缓存命中率明显提升（特别是对 L2/L3 敏感的控制算法）。

**代价**：
- 占用 CPU 核（隔离 4 个核意味着丢失 4 核的通用计算能力）；
- 配置复杂、依赖硬件布局（NUMA 节点、核号映射）；
- 不会被容器编排器（k8s）自动继承，部署脚本复杂。

**对应阶段**：[08 阶段 6](./08_phase6_rt_and_kernel.md)。

---

## D12. 编译器、LTO、PGO

**原理**：
- `-O3` + `-march=native` + LTO 可提升 5–15%；
- PGO（Profile-Guided Optimization）再提升 5–10%（对分支密集的调度器路径尤其显著）；
- `-fno-exceptions`（若业务允许）可减少二进制体积与少量运行时开销；
- 链接时选 `mold` 比 `ld.bfd` 快 10×（编译体验，不影响运行时）。

**可达收益**：
- 整体 CPU 占用降 5–20%；
- 对 rclcpp 的回调分派路径尤其敏感（大量模板展开）。

**代价**：
- 构建时间显著增加（LTO 20–50%，PGO 需要两轮构建）；
- 调试体验变差（内联后堆栈丢失）；
- 二进制不能跨 CPU 微架构部署。

**对应阶段**：[03 阶段 1](./03_phase1_config_tuning.md)（开编译选项）；[09 维护策略](./09_maintenance_strategy.md)（做 PGO CI）。

---

## 杠杆间的依赖与互斥

```
D10 实时内核 ─┬─> 几乎是所有其他优化的"放大器"
              └─> 不做 D10，D2/D3/D11 的收益被抖动淹没

D4 零拷贝  ─ 依赖 → D6 SHM/Iceoryx 与兼容的 D5 序列化
D1 EventsExecutor ─ 与某些第三方 executor（如 irobot_events_executor）互斥
D7 DDS 切换 ─ 与 D8/D9 的具体配置强耦合
D11 CPU 亲和  ← 必须在 D10 基础上才能发挥
```

## 一句话建议

> **不要同时拉 12 根杠杆**。按 03–08 的阶段顺序逐个引入，每引入一个杠杆都回归跑基线（10 章），量化收益、量化成本，再决定是否保留。

下一章：[03 阶段 1：配置级优化](./03_phase1_config_tuning.md)。
