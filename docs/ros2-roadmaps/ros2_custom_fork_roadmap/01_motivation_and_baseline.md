# 01 动机与基线 —— 为什么 Fork，以及从哪里测起

## 1. Fork 的合理动机（与不合理动机）

| 动机 | 是否合理 | 评注 |
|------|---------|------|
| "控制环延迟要 < 200 μs，官方实现做不到" | ✅ 合理 | 真实需求，但需确认瓶颈在 ROS 2 而非内核 / DDS 配置 |
| "我们要支持非 DDS 传输（EtherCAT / CAN-FD 直通）" | ✅ 合理 | 官方 rmw 抽象无法零开销穿透到 L2 总线 |
| "生产系统要求可审计 / 可裁剪 / 安全合规" | ✅ 合理 | 官方代码量 120 万+ 行，完整审计成本极高，裁剪后可降 10×+ |
| "我们要定制 QoS 策略 / 新增 QoS 维度" | ⚠️ 边缘 | 多数情况可通过 rclcpp 应用层包装实现，不必改内核 |
| "上游迭代慢 / bug 修复慢" | ⚠️ 边缘 | 先尝试提 PR，fork 的维护成本往往超过等上游的成本 |
| "我们喜欢自己的代码风格" | ❌ 不合理 | 这是用 1 人·年换 0 个功能 |
| "我们想取代 DDS 节省内存" | ⚠️ 边缘 | DDS 本身内存开销不大（< 50 MB/节点），瓶颈常在应用层 |

**核心判定：Fork 的合理性 = 量化收益 / 年维护成本 > 3**。年维护成本至少 2–4 人·年（见 [09 维护策略](./09_maintenance_strategy.md)）。

## 2. 官方实现的典型瓶颈（按出现频率排序）

### 2.1 Executor 调度开销

`rclcpp::SingleThreadedExecutor::spin()` 的热路径大致是：

1. `wait_for_work()` → `rcl_wait()` → 底层 `epoll_wait` / DDS listener；
2. `get_next_ready_executable()` —— **O(N) 扫描所有 callback group / entity**；
3. `execute_any_executable()` —— 调用回调前后做多次虚函数跳转与 `shared_ptr` 原子操作；
4. 回调执行；
5. 回到 1。

**瓶颈点**：
- 第 2 步的 O(N) 扫描，当 `N >= 50` 个 entity 时单次扫描就有 5–20 μs；
- 第 3 步的 `shared_ptr` 在 MultiThreadedExecutor 下有争用（原子 RMW 序列）；
- `AnyExecutable` 对象的动态分配（每次调度都 `new`，虽然 `EventsExecutor` 改善了此点）；
- **事件驱动与轮询驱动并存**：即使收到单次事件，仍会扫描所有 entity 以实现 FIFO 公平性——这在高频控制回调下是纯浪费。

### 2.2 rclcpp → rcl → rmw → DDS 的消息拷贝次数

一条 1 KB 消息从用户 `publisher->publish(msg)` 到订阅端回调，默认配置下的拷贝次数：

| 步骤 | 是否拷贝 | 说明 |
|------|---------|------|
| `rclcpp::Publisher::publish(msg)` | (0) | 接受 const ref / unique_ptr / shared_ptr |
| 类型支持 `cdr_serialize` | **+1 拷贝** | 用户结构体 → CDR 字节流（rmw_serialized_message_t） |
| rmw → DDS `write()` | 0 或 +1 | FastDDS 允许接管序列化缓冲；默认情况拷贝到 DDS history cache |
| DDS 传输层（UDP / SHM） | **+1 拷贝** | UDP：内核 socket buffer；SHM：ringbuffer 写入 |
| 订阅端 DDS 接收 | **+1 拷贝** | 从 socket / SHM 到 DataReader cache |
| rmw → rcl `take()` | 0 | 句柄传递 |
| 反序列化 CDR → 用户结构体 | **+1 拷贝** | 通常是就地构造 |
| IntraProcess 快速路径 | **0 拷贝** | 仅同进程、`unique_ptr`、兼容 QoS 时启用 |

**默认 4–5 次拷贝**；每 MB 消息的额外开销约 **200–500 μs**（取决于 L3 缓存热度）。

### 2.3 发现风暴（Discovery Storm）

SPDP（Simple Participant Discovery Protocol）默认每 **3 秒**广播一次所有本参与者的元数据。一个 40 节点系统中：

- 每节点 1–3 个 Participant，共 40–120 个 Participant；
- 每 Participant 有 20+ endpoint（pub/sub/service/action 合计）；
- SEDP（Endpoint Discovery）成对交换所有 endpoint 元数据，O(Participant²) × O(Endpoint) 的消息；
- 启动时前 30 秒常见 **每秒 500+ 条发现包**，CPU 占用 30–60%；
- DDS meta-traffic 即使在稳态也持续消耗 1–5% CPU 和 5–20 Mbps 带宽。

### 2.4 QoS 默认值的性能陷阱

| QoS | 默认值 | 问题 |
|-----|--------|------|
| `reliability` | RELIABLE | 控制环用 RELIABLE 会引入 ACK/NACK 往返，比 BEST_EFFORT 多 10–50 μs |
| `history.depth` | 10 | 大消息下每 history 条目都占内存；10 × 1 MB = 10 MB/订阅 |
| `durability` | VOLATILE | 合理 |
| `deadline` | 无 | 未设置 → 无法检测 late publisher，问题被掩盖 |
| `liveliness` | AUTOMATIC | 未显式设置 lease duration，发现期拉长 |

### 2.5 内存分配路径

- 每次 `publish()` 默认会触发 **至少 2 次 `operator new`**（序列化缓冲 + DDS history 副本）；
- 回调路径中的 `make_shared<MsgT>` 又是一次；
- 动态分配在非实时系统上是 10–50 μs，在 glibc malloc 争用下可达数百 μs；
- PREEMPT_RT 下的 **page fault** 会引发调度器切换，延迟抖动放大。

### 2.6 序列化开销

CDR 序列化的成本在于：
- **小消息**（< 256 B）：函数调用开销 > 拷贝开销，占端到端延迟 30–40%；
- **中消息**（1 KB）：序列化 ≈ 传输；
- **大消息**（> 1 MB）：序列化 < 传输，但对于**数组/点云**，默认实现不使用 SIMD，可再快 2–3×。

## 3. 基线测量方法

**在做任何优化前，先建立可重复的基线**。推荐工具链：

1. **performance_test**（Apex.AI 开源）：`ros-humble-performance-test`，标准化端到端延迟 / 吞吐测量。
2. **iRobot reference-system**（[osrf/reference-system](https://github.com/ros-realtime/reference-system)）：27 节点的参考系统，覆盖真实传感器-控制管线的调度模式。
3. **自建 micro-benchmark**：围绕你的**真实控制环**（如 500 Hz IMU → 控制器 → 执行器）。

### 3.1 基线测量矩阵

| 维度 | 测量指标 | 工具 |
|------|---------|------|
| 端到端延迟 | P50 / P99 / P99.9 | performance_test |
| 吞吐 | msg/s @ 给定延迟约束 | performance_test |
| 单跳延迟（rclcpp → rmw） | trace | `ros2_tracing` + `babeltrace2` |
| Executor 调度延迟 | 回调就绪 → 回调开始 | `ros2_tracing` |
| 拷贝次数 | 实际拷贝 / 应有拷贝 | 代码审查 + eBPF uprobe |
| 内存分配 | alloc/s in hot path | `perf`、`heaptrack`、`jemalloc --prof` |
| 抖动 | σ(latency) | cyclictest（裸机）+ performance_test |
| 发现耗时 | 从 `node start` 到 `peer_count` 稳定 | 自建 node + 监听 `/rosout` |

### 3.2 推荐基线场景（最小集）

```
场景 A：单进程内 10 个 Node，1 KB 消息，100 Hz，RELIABLE
场景 B：同机多进程 10 个 Node，1 MB 消息，30 Hz，BEST_EFFORT
场景 C：500 Hz 控制回路，timer → callback → publish，32 B 消息
场景 D：发现收敛时间，40 Node 冷启动
场景 E：突发负载，1 MB × 100 条消息突发后恢复时间
```

每个场景采集至少 60 秒、10⁶ 条样本，去除前 5 秒预热期。

## 4. 典型基线数据（参考值，非承诺）

以 Intel i7-12700 / Ubuntu 22.04 / ROS 2 Humble / 默认 FastDDS 为例：

| 场景 | 指标 | 默认值 | 说明 |
|------|------|--------|------|
| A-IPC | P99 端到端 | 40–80 μs | 启用 IntraProcess 后 |
| A-DDS | P99 端到端 | 300–800 μs | 默认未启 IPC |
| B | P99 端到端 | 3–15 ms | 1 MB 大消息 |
| C | P99 抖动 σ | 100–300 μs | 非 RT 内核 |
| C-RT | P99 抖动 σ | 30–80 μs | PREEMPT_RT + SCHED_FIFO |
| D | 发现稳定时间 | 20–60 s | SPDP 默认 3 s 周期 |

**读懂基线的意义**：任何"X 技术能提升 10 倍性能"的宣称，必须对应到上表中具体哪一行——否则就是销售话术。

## 5. 本章小结

- Fork 的前提是**量化的、可复现的、有业务意义的**性能差距；
- ROS 2 官方实现的 6 大典型瓶颈：Executor 扫描、多次拷贝、发现风暴、QoS 默认值、动态分配、序列化；
- 任何优化开始之前，先用 performance_test / ros2_tracing 建立**基线数据集**；
- 不要相信"提升 X 倍"的泛泛数字，只相信**在你的场景下**测出来的数据。

下一章：[02 可优化维度](./02_optimization_dimensions.md)。
