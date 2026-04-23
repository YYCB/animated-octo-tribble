# 07 阶段 5：替换 DDS —— Zenoh、自研协议、去 DDS 化

> **目标**：当 DDS 本身（而非其配置/插件）成为瓶颈或维护负担时，替换整个中间件层。
>
> **警告**：这是**所有阶段中 ROI 最低**的一步。除非有非常明确的业务需求（WAN 部署、非 IP 网络、百节点以上集群、严格的确定性要求），否则**不推荐**进入本阶段。

---

## 7.1 为什么 DDS 会成为问题

1. **发现协议的本质 O(N²)**：即使 Discovery Server 缓解，底层仍是"每个 endpoint 要知道所有 matching endpoint"；节点数过千时发现延迟仍几十秒；
2. **WAN 不友好**：DDS 多播在公网上不工作，穿越 NAT 需要特殊的 tunnel；
3. **资源占用**：即使空闲，每个 participant 消耗 20–40 MB RSS 和 1–3% CPU；
4. **可观测性差**：DDS internal 的调试 hook 很少，问题难定位；
5. **QoS 矩阵复杂**：QoS 冲突时静默失败，难以维护。

---

## 7.2 替代方案

### 7.2.1 Zenoh（`rmw_zenoh_cpp`）

**设计哲学**：
- 从 pub/sub 扩展到 **pub/sub/get/put/query**（兼具 DDS + kv store 特性）；
- 路由器架构（zenohd）：客户端 - 路由器 - 路由器；
- 原生支持 TCP / QUIC / shared memory / Bluetooth 等多种传输；
- 发现是**惰性**的——订阅时才 announce，不做 3 秒心跳广播。

**状态（2024）**：`rmw_zenoh_cpp` 已是 ROS 2 Jazzy 官方支持的 RMW 实现，Iron 上仍实验。

**优势对比 DDS**：
- 发现风暴大幅缓解（惰性 + 路由器聚合）；
- WAN 场景天然支持（通过 zenohd 路由）；
- 单机 SHM 性能接近 Iceoryx（且不需要 RouDi daemon）；
- 消息可 persistent（内建 storage backend）。

**劣势**：
- 生态相对年轻，某些 ROS 2 功能（Actions / Lifecycle）的语义实现仍在完善；
- 运维体系（zenohd 集群、HA、监控）需要重新学习；
- 性能调优的最佳实践尚未形成社区共识。

**何时选 Zenoh**：
- 多机 / 跨地域部署；
- 需要把 ROS 2 数据直接送入云端；
- 希望消息具有"历史 + 最新值"的存储查询语义；
- 团队愿意承担"非主流 RMW"的调试成本。

### 7.2.2 eCAL (Continental)

汽车行业的替代品，专注**同机多进程 + 跨机 TCP**。无 DDS 血缘。
- 性能对标 Iceoryx，延迟 μs 级；
- 有独立的 rec/play/monitor 工具链；
- **没有 rmw_ecal 的官方实现**——需要自己封装 rmw 层；
- **不推荐**：ROS 2 生态整合工作量巨大。

### 7.2.3 自研消息总线

**典型取舍**：
- 完全 drop DDS，写一个"类 Kafka topic + 类 RPC"的组合；
- 传输层：SHM（同机）+ uTP/自定义 UDP（跨机）；
- 发现层：etcd / Consul / 自研 Raft；
- 序列化：flatbuffers / capnp / 自定义；
- **彻底失去 ROS 2 生态**（rviz / rosbag / rqt / MoveIt / Nav2 全部要重写）；
- 工作量：**30–100 人·年**级别；
- **仅在企业级强业务需求下合理**（如特斯拉的 "ROS 没用，我们自己写"）。

### 7.2.4 混合架构（推荐路径）

**不要一刀切替换**。现实的做法：
- **保留 DDS 作为默认 RMW**，控制大部分 topic；
- **关键控制环走自研 SHM 总线**（不过 rclcpp），几个专用 C++ 类直接读写 ringbuffer；
- **WAN 部分走 Zenoh 或 gRPC**（桥接节点）；
- **不 fork rmw，只在应用层多一套 API**。

这保持了 80% 的 ROS 2 兼容性，又能在**关键路径**获得极限性能。

---

## 7.3 替换 DDS 的工作拆解

假设决定换 DDS（不管换成 Zenoh 还是自研），工作量包含：

### 7.3.1 RMW 层
- 实现 `rmw_create_node`、`rmw_publish`、`rmw_take`、`rmw_service_server_is_available` 等 100+ 函数；
- 实现 WaitSet 与 GuardCondition（rcl 的事件多路复用依赖此）；
- 实现 Discovery（节点 / endpoint / service / action）；
- 实现 QoS 语义（reliability, history, durability, deadline, ...）；
- **参考**：`rmw_zenoh_cpp` 约 1.5 万行 C++；`rmw_fastrtps_cpp` 约 3 万行。

### 7.3.2 Type Support
- 每种消息类型需要 serialize / deserialize / get_type_name / equal 等函数；
- 如果换序列化格式，需要写新的 rosidl_generator_xxx（约 3–5 千行 Python + C++ 模板）。

### 7.3.3 工具链适配
- `ros2 topic echo / list / info` 依赖 rmw 能查询拓扑；
- `rosbag2` 依赖 rmw serialize；
- `rviz` 依赖 rmw 订阅/反序列化；
- **若新传输与 CDR 不兼容**，所有这些工具都要适配或保留 bridge 模式。

### 7.3.4 测试
- ROS 2 的测试套（`test_rmw_implementation` 等）要通过；
- 互操作性：与标准 DDS 对端的桥接是否工作；
- 性能：跑 performance_test 对比。

**总工作量**：
- **Zenoh（用 rmw_zenoh_cpp）**：1 人·季度（主要是适配 + 踩坑）；
- **自研 RMW**：10 人·年起；
- **连 rosidl 也重写**：20 人·年起。

---

## 7.4 决策矩阵

| 场景 | 建议 |
|------|------|
| 单机机器人（桌面 / 人形） | 保留 DDS + SHM，不要替换 |
| 多机集群（仓储 AGV、机群） | **考虑 Zenoh**（仅 rmw 层替换） |
| 需要云边协同（数据上云） | **Zenoh router** 或保留 DDS + 桥接 |
| 汽车 / 工业（硬实时、证书要求） | 保留商业 DDS（Connext Pro）或走 SOME/IP |
| 大规模社区部署（> 1000 节点） | Zenoh 分层路由 |
| 非 IP 链路（CAN / EtherCAT） | 不走 rmw——直接在驱动层读写总线 |

---

## 7.5 不被普遍认可的"性能神话"

- **"换 DDS 能提升 10 倍性能"** —— 只有在**当前 DDS 未优化且不走 SHM** 的情况下才有此数字。优化后的 CycloneDDS + Iceoryx 与 Zenoh SHM 在微基准上**几乎一致**（1–3 μs P99）。
- **"Zenoh 没有发现开销"** —— 惰性发现仍有开销，只是时序分布不同；路由器本身的路由表维护是新增开销。
- **"自研总线更快"** —— 初版一定快，但一旦要加 RELIABLE / 历史 / QoS / 监控，很快就会长成"一个新的 DDS"。

---

## 7.6 本阶段结束准则

- **实施完**：跑全量基准 + 业务压测 1 个月稳定运行；
- **撤退条件**：若任何关键业务指标（延迟、吞吐、可用性）显著变差且 1 个月内无法修复——**回滚到 DDS**。
- 失败不是失败——很多团队在此阶段停步或回退，这是**理性决策**。

---

## 7.7 维护成本

- 改动面：**rmw 层重写 + 可能的 rosidl + 上下游工具链适配**；
- 与上游兼容性：**差**——你在某种意义上"离开了 ROS 2 主流"；
- 每次 ROS 2 版本升级：**4–8 周**；
- 典型团队规模：**4–8 人**（含测试 / 文档 / 工具）；
- 年成本：**500 万 – 1000 万 RMB**（含人力 + CI 基础设施）。

除非你的业务能明确算出"每年节省 > 2000 万"的价值，否则**本阶段应放入长期规划而非当下执行**。

下一章：[08 阶段 6：实时化](./08_phase6_rt_and_kernel.md)。
