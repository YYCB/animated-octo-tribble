# 06 阶段 4：DDS SHM / Iceoryx —— 同机零拷贝总线

> **目标**：同机多进程场景下，把 DDS 的传输层替换为共享内存，使跨进程延迟接近 IntraProcess（μs 级）。
>
> **前置**：阶段 3 的 Loaned Message 已就绪——若消息类型不支持 Loan，SHM 传输不能发挥 0 拷贝优势。

---

## 6.1 为什么需要 SHM 传输

### 6.1.1 Localhost UDP 的开销

通过 `lo` 接口发送 UDP：
```
publisher 进程
  ├ DDS serialize           ~2 μs  (1 KB)
  ├ UDP send syscall        ~3 μs  (上下文切换)
  ├ 内核 UDP 栈处理         ~5 μs
  ├ 拷贝到 loopback buffer  ~3 μs
  ├ 内核投递                ~3 μs
  ├ epoll wakeup            ~2 μs
  └ subscriber recv syscall ~3 μs
 总计：~20 μs + message_size / memcpy_bandwidth
```

1 MB 消息：20 μs + 2 × (1 MB / 20 GB/s) = 20 μs + 100 μs = **120 μs**（不含序列化）。

### 6.1.2 SHM ringbuffer 的开销

```
publisher 进程
  ├ 获取 SHM chunk          ~50 ns   (atomic CAS on free-list)
  ├ 就地构造消息             ~0       (用户代码)
  ├ enqueue 指针            ~50 ns   (atomic store release)
  └ futex_wake (如有等待者)  ~500 ns

subscriber 进程
  ├ atomic load acquire     ~50 ns
  └ 处理消息（直接读 SHM）   ~0
 总计：~1–2 μs，与消息大小无关
```

**1 MB 消息从 120 μs 降到 2 μs，60×**。

---

## 6.2 方案对比

| 方案 | 接入方式 | 延迟 P99 | 成熟度 | 生态 |
|------|---------|---------|--------|------|
| **FastDDS SHM builtin** | RMW_IMPLEMENTATION + XML | 8–15 μs | 成熟 | 官方默认 |
| **CycloneDDS SHM (Iceoryx)** | Cyclone + Iceoryx + RouDi | 3–6 μs | 成熟 | 官方支持 |
| **Eclipse iceoryx2** | 独立中间件，Rust 重写 | 2–4 μs | 较新 | 独立生态 |
| **自研 SHM transport** | 自己写 rmw 或 DDS plugin | 1–2 μs | 自承 | 完全自主 |

### 6.2.1 FastDDS SHM

**配置方式**：XML profile 里启用：
```xml
<participant profile_name="participant_shm">
  <rtps>
    <userTransports>
      <transport_id>shm</transport_id>
    </userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps>
</participant>

<transport_descriptor>
  <transport_id>shm</transport_id>
  <type>SHM</type>
  <segment_size>262144000</segment_size>  <!-- 256 MB -->
</transport_descriptor>
```

**优点**：零依赖、只改 XML；
**缺点**：FastDDS 的 SHM 实现有历史包袱——早期版本仍会在 history cache 做一次拷贝，必须验证版本 ≥ 2.10。

### 6.2.2 Iceoryx + CycloneDDS

**架构**：
```
┌────────────┐   ┌────────────┐   ┌────────────┐
│ process A  │   │ process B  │   │   RouDi    │
│ cyclonedds │   │ cyclonedds │   │   daemon   │
│ ↕          │   │ ↕          │   │            │
│ iceoryx_   │───│ iceoryx_   │───│ SHM pool   │
│ posh       │   │ posh       │   │ manager    │
└────────────┘   └────────────┘   └────────────┘
```

**工作方式**：
- RouDi（Routing and Discovery）daemon 管理全局 SHM pool；
- 各进程通过 Iceoryx client 连接 RouDi，申请/归还 chunk；
- CycloneDDS 检测到对端在同机 + iceoryx 可用时，自动走 SHM 路径；
- 跨机自动 fallback 到 UDP。

**启用**（CycloneDDS XML）：
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

加上 RouDi daemon：
```bash
iox-roudi &
```

### 6.2.3 iceoryx2

Iceoryx 1.x 的 Rust 重写版本，设计目标：
- 不依赖 RouDi daemon（分布式元数据）；
- 更小的 footprint；
- 更严格的生命周期管理。

**现状（2024）**：**尚未被 ROS 2 rmw 层官方支持**，需自研桥接。但作为"未来的 Iceoryx"值得关注。

### 6.2.4 自研 SHM Transport

**动机**：如果已经走到阶段 7（替换 DDS），可以直接写一个专用 SHM 总线：
- MPSC（多生产 / 单消费）或 SPMC（单生产 / 多消费）ringbuffer；
- 无需全局 daemon（元数据存 `/dev/shm` 文件系统的元目录）；
- 专为已知的 topic schema 优化；

**收益**：P99 < 2 μs，0 系统调用 fast path；
**代价**：失去所有 DDS 生态（rviz / rqt / rosbag / 跨语言）。

---

## 6.3 容器化部署的坑

SHM 依赖宿主机 `/dev/shm` 和 POSIX shared memory。Docker 下需要：
```
docker run --ipc=host ...
```
或
```
docker run --ipc=shareable --shm-size=2g ...
```
且**所有通信方（包括 RouDi）用同一个 IPC 命名空间**。Kubernetes 下通过 `hostIPC: true` 或专用的 "sidecar + hostPath volume" 方案。

---

## 6.4 SHM 的副作用

### 6.4.1 崩溃恢复

一个 publisher 崩溃留下的 SHM chunk 可能：
- 被消费端引用着，RouDi 能回收但延迟恢复；
- 若 consumer 也崩溃，出现 "孤儿 chunk"——需要 RouDi 重启或手动 `rm /dev/shm/iox_*`。

### 6.4.2 内存占用

SHM pool 的大小在启动时固定，一次性分配（通常几百 MB 到几 GB）。如果设置过小 → 消息发送失败；过大 → 浪费物理内存。

### 6.4.3 消息大小分档

Iceoryx 的 SHM pool 分"chunk 桶"（32 B, 128 B, ..., 4 MB）。发送 64 KB 消息会占用 128 KB 桶——**不匹配的消息尺寸导致内存浪费**。需要配置合理的 `mempool.toml`：
```toml
[[general]]
version = 1
[[mempool]]
size = 128
count = 10000
[[mempool]]
size = 1024
count = 5000
[[mempool]]
size = 1048576
count = 32
```

### 6.4.4 安全性

SHM 是 kernel-global 的，同宿主上所有进程都可能 attach 到同一个 segment。生产环境需要：
- 用 Linux namespaces / 隔离用户（SHM 文件权限）；
- 或用 Iceoryx 的 access control 功能（RouDi 下发权限）。

---

## 6.5 推荐架构（具身智能项目的典型配置）

```
Host (PREEMPT_RT Linux)
 ├── iox-roudi (nice -10)
 ├── Component Container A（核 4-5，SCHED_FIFO 90）
 │    ├─ Imu Driver
 │    ├─ Motor Driver
 │    └─ Controller (500 Hz)
 │       └─ 内部：IntraProcess 零拷贝
 ├── Component Container B（核 2-3，SCHED_OTHER）
 │    ├─ Camera Driver
 │    └─ Vision Node
 ├── Component Container C（核 6-7，nice 0）
 │    ├─ Planner
 │    └─ Navigation Stack
 ├── UI/Monitoring Container（核 0-1，nice +5）
 │    ├─ rosbag2 recorder
 │    ├─ diagnostic_aggregator
 │    └─ rosbridge
 └── 容器间通信：CycloneDDS + Iceoryx SHM
     └── 容器内通信：IntraProcess（0 拷贝 unique_ptr）
```

**要点**：
- 控制链路内部用 IPC，延迟最小；
- 容器间（如控制 → 规划反馈）用 SHM；
- 跨机（如日志上传到网关）自动 fallback UDP；
- RouDi 跑在专用低延迟优先级，不抢控制核。

---

## 6.6 本阶段结束准则

预期指标：

| 场景 | Phase3 后 | Phase4 后 | 收益 |
|------|----------|----------|------|
| 1 MB 跨进程（同机） | 300 μs | 15 μs | 20× |
| 5 MB 点云 | 1.2 ms | 30 μs | 40× |
| CPU 占用（10 MB/s 流） | 基准 | 降 60% | — |
| 内存带宽占用 | 基准 | 降 80% | — |

**结束判定**：如果到此延迟 / 吞吐已经满足业务需求 → **强烈建议停止**，不要进入阶段 5（替换 DDS）。阶段 5 的收益边际很小，但维护成本翻倍。

---

## 6.7 维护成本

- 改动面：**XML 配置 + 可能的 rmw 层 patch + RouDi 部署脚本 + 消息 pool 配置**；
- 与上游兼容性：**良好**——Iceoryx/SHM 都是 CycloneDDS/FastDDS 官方支持特性；
- 升级周期：RouDi / Iceoryx 的 ABI 会变，约每 1 年需要一次适配（约 1 周工作量）；
- 典型团队规模：**0.5 人**兼职维护（配置级 + 监控）；
- 这是**性价比第二高的阶段**（仅次于阶段 1），推荐做。

下一章（可选方向）：[07 阶段 5：替换 DDS](./07_phase5_replace_dds.md)。
