# 09 SHM 传输 —— FastDDS SHM 与 Iceoryx 实现解析

> 接 [07 Loaned Messages](./07_loaned_messages.md) 与 [08 RTPS 线协议](./08_rtps_wire_protocol.md)。本章聚焦**传输层**——DDS 把消息从一个进程送到另一个进程，SHM 方案如何做到比 UDP loopback 快 50–150 倍。

---

## 9.1 同机通信的传输选项对比

| 传输 | 延迟 P99 | 带宽 | 内核调用 | 拷贝次数 |
|------|---------|------|---------|---------|
| UDP loopback | 150–500 μs | 受 socket buf 限 | send/recv | 2+ |
| TCP loopback | 200–600 μs | 受 socket buf 限 | send/recv | 2+ |
| AF_UNIX stream | 80–200 μs | 中 | send/recv | 2 |
| FastDDS SHM builtin | 15–50 μs | 高 | 无（mmap） | 1 |
| Iceoryx SHM | 2–10 μs | 高 | 无（mmap） | 0（Loan）/1 |
| 纯 SHM ringbuffer (自研) | 1–3 μs | 高 | 无 | 0 |

---

## 9.2 FastDDS SHM 的实现

### 9.2.1 数据结构

FastDDS SHM transport 在 `/dev/shm/` 下创建多个文件：

```
/dev/shm/fastrtps_port7400      ← Participant 发现端口的 SHM 通道
/dev/shm/fastrtps_port7401
/dev/shm/fastrtps_<uuid>         ← 数据 segments（per participant）
```

每个 segment 是一个 **Boost.Interprocess 管理的共享内存池**，内部又分：
- **Global Port Manager**：协调哪个 Reader 连到哪个 Port；
- **Data Segments**：存储 DATA 消息的载荷；
- **Listener queues**：每个 Reader 有一个消息入队 queue。

### 9.2.2 发布流程

```
Writer 进程
  1. serialize MsgT → CDR 字节（仍然拷贝一次到发送缓冲）
  2. 在 SHM segment 里分配一个 chunk
  3. memcpy 数据到 chunk
  4. 往目标 Reader 的 listener queue 推送 chunk 描述符
  5. futex_wake 唤醒 Reader

Reader 进程
  6. futex_wake 返回 → 从 queue 取 chunk 描述符
  7. 直接访问 chunk 读 CDR 字节
  8. deserialize 到 MsgT
  9. 处理完成 → 归还 chunk
```

**拷贝次数**：1 次（步骤 3）。**未启用 Loan 时**。

### 9.2.3 启用 Loan 时

```
Writer 进程
  1. borrow_loaned_message → rmw 向 FastDDS SHM 要 chunk
  2. 用户**原地构造 MsgT**（用 chunk 内存）
  3. publish_loaned → 不需 memcpy，直接推入 listener queue
  4. futex_wake

Reader 进程
  5. take_loaned → 拿到 chunk 内存指针
  6. 直接读（视为 MsgT*）
  7. 归还 chunk
```

**拷贝次数**：**0**。这是 FastDDS SHM + Loan 的极限性能。

### 9.2.4 配置

XML profile：
```xml
<transport_descriptors>
  <transport_descriptor>
    <transport_id>shm</transport_id>
    <type>SHM</type>
    <segment_size>1048576000</segment_size>  <!-- 1 GB SHM pool -->
    <port_queue_capacity>512</port_queue_capacity>
    <healthy_check_timeout_ms>1000</healthy_check_timeout_ms>
  </transport_descriptor>
</transport_descriptors>
<participant profile_name="part">
  <rtps>
    <useBuiltinTransports>false</useBuiltinTransports>
    <userTransports>
      <transport_id>shm</transport_id>
    </userTransports>
  </rtps>
</participant>
```

### 9.2.5 FastDDS SHM 的局限

- **单机仅**：不跨主机；跨机 fallback 到 UDP；
- **启动时固定大小**：segment_size 设错要重启；
- **崩溃恢复弱**：进程异常退出，留下"僵尸消息"占 chunk，需要 `/dev/shm/fastrtps_*` 手动清理或重启 transport；
- **安全边界弱**：同机任何有读权限的进程都可 attach；
- **大消息性能梯度跳变**：超过 segment chunk size（通常 4 MB）的消息会**退化到 UDP**——真实案例：发 5 MB 点云，P99 从 30 μs 突降到 15 ms。

---

## 9.3 Iceoryx（与 CycloneDDS 集成）

### 9.3.1 架构

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Pub Process │     │ Sub Process │     │  RouDi      │
│ ─ CycloneDDS│     │─ CycloneDDS │     │  (daemon)   │
│ ─ iox-posh  │◄───►│─ iox-posh   │◄───►│  全局元数据  │
└─────────────┘     └─────────────┘     └─────────────┘
       │                    │                  │
       └──── SHM pool ──────┴──────────────────┘
          /dev/shm/iceoryx_*
```

**RouDi**（Routing and Discovery daemon）是 Iceoryx 的核心——全局 SHM pool 管理 + discovery 转发 + 权限控制。

### 9.3.2 SHM pool 分配

Iceoryx 用 "memory pool" 模型——预先定义若干**chunk size 桶**：

```toml
# mempool.toml
[[general]]
version = 1

[[mempool]]
size = 128
count = 10000    # 10000 个 128 B chunk

[[mempool]]
size = 1024
count = 5000

[[mempool]]
size = 65536
count = 500

[[mempool]]
size = 4194304   # 4 MB
count = 32
```

发消息时向最合适的桶借。**代价**：64 B 消息会占 128 B chunk（下一个桶）——碎片率高时浪费内存。

### 9.3.3 发布流程

```
Pub Process
  1. cyclonedds → iox::popo::Publisher::loan()
     → 向 RouDi 或本地 pool 请求一个 chunk
  2. 原地构造 MsgT
  3. Publisher::publishAndRelease(chunk)
     → 将 chunk 推入该 topic 的每个订阅的 SPSC queue

Sub Process
  4. 轮询 / condition variable 唤醒
  5. Subscriber::take() → 获得 chunk
  6. 读取消息
  7. release() → chunk 引用计数 -1，归还 pool
```

**特色**：**Lock-free MPMC queue**（多生产多消费，比 FastDDS 的简单 queue 更可扩展）。

### 9.3.4 RouDi 角色

- 维护全局 pub/sub registry；
- 分配 pool 内存并报告权限；
- 提供发现接口给 CycloneDDS；
- **不在数据路径上**——数据流不经过 RouDi，只在建连时经过。

### 9.3.5 启用配置

CycloneDDS XML：
```xml
<CycloneDDS>
  <Domain>
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>info</LogLevel>
    </SharedMemory>
  </Domain>
</CycloneDDS>
```

启动 RouDi：
```bash
iox-roudi -c /path/to/mempool.toml &
```

### 9.3.6 Iceoryx 的优势

- **2–10 μs P99**，比 FastDDS SHM 再快 3–5 倍；
- 成熟的 pool 管理（不会碎片化失控）；
- 明确的权限模型（RouDi 下发）；
- 分层 subscriber 支持（1:N, N:M）；
- 有好的工具链（iox-introspection）。

### 9.3.7 Iceoryx 的代价

- **RouDi daemon 是单点**：RouDi 崩 → 新订阅建立不了（已建立的继续工作）；
- **pool 大小要配好**：碎片 / 耗尽难以调试；
- **容器化复杂**：需要 `--ipc=host` 或 sidecar；
- **CycloneDDS 之外少生态**：FastDDS / Connext 不直接用 Iceoryx。

---

## 9.4 iceoryx2（新版，Rust 重写）

2024 年 Eclipse 发布了 iceoryx2：
- **无 daemon**：分布式元数据（文件系统的元目录替代 RouDi 中心化）；
- **Rust 实现**：内存安全；
- **ABI/API 稳定性更好**。

**现状**：尚未被 ROS 2 rmw 官方 support。若要用于 ROS 2 需自研桥接层。作为**未来方向**值得关注。

---

## 9.5 自研 SHM Transport 的设计要点

如果决定**不用 Iceoryx 也不用 FastDDS SHM**，自己实现 SHM 传输：

### 9.5.1 核心结构

```
SHM Region (per topic)
┌────────────────────────────────────────────────────┐
│ Header (metadata, seq counters, ref counts)         │
├────────────────────────────────────────────────────┤
│ Ring of Slots                                       │
│  Slot 0: [size | flags | refcount | payload...]    │
│  Slot 1: ...                                        │
│  ...                                                │
│  Slot N-1: ...                                      │
└────────────────────────────────────────────────────┘
```

- 发布者：CAS 原子更新 head 指针，写 slot，增 seq；
- 订阅者：读 tail（自己的视图），比较 head，读所有未读 slot；
- 引用计数管理"slot 何时可被覆盖"（最慢的订阅者 + 1）。

### 9.5.2 唤醒机制

- 最简单：spin 等待（适合 ≤ 1 个订阅，<1 μs 延迟但 100% CPU）；
- 常见：eventfd / futex，入队后 futex_wake，读时 futex_wait（~3 μs 延迟，0 CPU）；
- 高级：io_uring 的 eventfd 注册，单系统调用处理多事件。

### 9.5.3 崩溃恢复

- 发布者 crash：留下**部分写入**的 slot——需要有效性标志（magic + CRC）避免订阅读坏数据；
- 订阅者 crash：留下**未归还**的引用计数——需要心跳 / 定期清理机制；
- RouDi-less 设计：用 `/dev/shm/<topic>.lock` 文件 + flock 做生命周期管理。

### 9.5.4 性能上限

精心实现的自研 SHM：
- P50 < 500 ns，P99 < 2 μs（单订阅）；
- 纯内存带宽限速（1 MB 消息 ~ 50 μs，瓶颈在 memcpy）；
- 前提：SPSC 或 SPMC 设计，避免 MPMC 的 fence 开销。

---

## 9.6 所有 SHM 方案的共性问题

### 9.6.1 命名空间隔离

Docker 默认每个容器有独立的 IPC namespace，`/dev/shm` 互不可见。解决：
- `docker run --ipc=host`（共享宿主 IPC）；
- `docker run --ipc=shareable` + `--ipc=container:xxx`（可控共享）；
- Kubernetes：`hostIPC: true` 或 sidecar 模式。

### 9.6.2 权限

`/dev/shm/` 下文件默认 644 或 666。多用户系统需要：
- 用户组隔离；
- SELinux / AppArmor 标签（FastDDS 做得少，Iceoryx 有配置）；
- 关键：生产环境**不要**混用特权和非特权进程访问同一 SHM。

### 9.6.3 持久化

SHM 不跨重启。`/dev/shm` 是 tmpfs。重启后：
- 所有 segment 消失；
- RouDi 重启后会重新创建；
- 应用需设计为"无状态"或"状态可恢复"。

### 9.6.4 大消息 edge case

当消息大小接近 chunk 大小时：
- FastDDS SHM：要么退化到 UDP（坏），要么扩大 segment（消耗内存）；
- Iceoryx：配更大的 pool 桶；
- 应用层：**预估最坏情况消息大小**，按此规划 pool。

---

## 9.7 实战：如何选择

```
需求               推荐方案
──────────────────────────────────────
ROS 2 单机, 默认      FastDDS SHM builtin（0 配置）
要求 μs 级延迟         CycloneDDS + Iceoryx
需要 pool 精细控制     Iceoryx + 自定义 mempool.toml
极限性能 / 专用总线   自研 SPSC SHM ringbuffer（不走 rmw）
容器化部署            FastDDS SHM（更简单, 不需要 daemon）
混合同机 + 跨机       CycloneDDS + Iceoryx（SHM 同机自动，UDP 跨机）
```

---

## 9.8 性能对比实测

Intel i7-12700, Ubuntu 22.04 PREEMPT_RT, 1 KB 消息, 1000 Hz:

| 方案 | P50 | P99 | P99.99 | 相对 |
|------|-----|-----|--------|-----|
| UDP loopback | 180 μs | 450 μs | 1.2 ms | 1× |
| FastDDS SHM | 25 μs | 80 μs | 300 μs | 5.6× |
| CycloneDDS + Iceoryx | 8 μs | 20 μs | 80 μs | 22× |
| 自研 SHM | 1.5 μs | 4 μs | 20 μs | 112× |

**注**：以上均在启用 Loan 的情况下。未 Loan 时 FastDDS SHM 会多 8–15 μs。

---

## 9.9 小结

- SHM 是同机 ROS 2 通信的必选项（不仅限于延迟，还有 CPU）；
- FastDDS SHM 是"零配置可用"的起点；
- Iceoryx + CycloneDDS 是"性能+生态"的最佳平衡；
- 自研 SHM 只在极限场景考虑——代价是离开 rmw 生态；
- 所有 SHM 方案都面临"崩溃恢复 / 权限 / 容器化"等运维挑战；
- 大消息的 edge case（chunk 不够）是 **生产环境最常见的 SHM 事故**。

本专题章节结束。

相关章节：
- [06 IPC](./06_intra_process.md) - 同进程 0 拷贝；
- [07 Loaned Messages](./07_loaned_messages.md) - 跨进程 0 拷贝；
- [08 RTPS 线协议](./08_rtps_wire_protocol.md) - SHM 也要跑 RTPS DATA/HEARTBEAT/ACKNACK；
- [自定义 Fork Roadmap 06](../../ros2-roadmaps/ros2_custom_fork_roadmap/06_phase4_dds_shm_iceoryx.md) - 工程落地建议。
