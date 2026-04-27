# iceoryx2 零拷贝 IPC 架构

> iceoryx2 是由 Eclipse 基金会维护的高性能进程间通信（IPC）库，专为嵌入式系统和机器人实时控制场景设计。本文档覆盖其架构原理、ROS 2 集成方法（`rmw_iceoryx`）、性能数据及与 ros2_control 的结合要点。

---

## 一、iceoryx2 vs iceoryx1：主要变化

| 对比维度 | iceoryx (v1) | iceoryx2 |
|---------|------------|---------|
| **实现语言** | C++ | **Rust**（核心），C 和 C++ 绑定 |
| **API 风格** | 命令式，较冗长 | 类型安全，Builder 模式，更符合人体工程学 |
| **Node 概念** | 无 | **引入 Node**（类似 ROS 2 Node，管理资源生命周期） |
| **服务发现** | RouDi 守护进程 | RouDi 守护进程（改进版，支持多实例） |
| **内存安全** | 手动管理 | Rust 所有权系统，编译期保证无内存泄漏 |
| **延迟** | ~1 μs | **< 100 ns**（优化后） |
| **支持平台** | Linux, QNX, macOS | Linux, Windows, macOS（扩展中） |
| **ROS 2 集成** | `rmw_iceoryx` | `rmw_iceoryx2`（开发中） |
| **WaitSet** | 有限支持 | **完整 WaitSet API**（事件驱动） |

> **当前状态（2024）**：`rmw_iceoryx`（v1 绑定）已可用于 ROS 2 Humble/Iron；`rmw_iceoryx2` 正在积极开发中，可关注 [eclipse-iceoryx/iceoryx2](https://github.com/eclipse-iceoryx/iceoryx2) 进展。

---

## 二、核心概念

### 2.1 Service — 类型化通信端点

iceoryx2 的 `Service` 类似于 ROS 2 的 Topic，但强调类型安全和内存模型：

```rust
// Rust API 示例（iceoryx2 原生）
use iceoryx2::prelude::*;

// 定义消息类型（需要 Copy 或 Pod）
#[repr(C)]
struct JointState {
    position: [f64; 8],
    velocity: [f64; 8],
    timestamp_ns: u64,
}

// 创建 publish-subscribe Service
let service = zero_copy::Service::new(&"JointState".try_into()?)
    .publish_subscribe()
    .open_or_create::<JointState>()?;
```

### 2.2 Node — 资源生命周期管理

```rust
// Node 管理 Service 的生命周期
let node = NodeBuilder::new()
    .name(&"rt_controller".try_into()?)
    .create::<zero_copy::Service>()?;

// Publisher 附属于 Node
let publisher = node
    .service_builder(&"JointState".try_into()?)
    .publish_subscribe()
    .open_or_create::<JointState>()?
    .publisher_builder()
    .create()?;
```

### 2.3 iceoryx2 与 ROS 2 Node 的对应关系

| iceoryx2 概念 | ROS 2 对应 | 说明 |
|-------------|----------|------|
| `Node` | `rclcpp::Node` | 资源所有者 |
| `Service`（publish-subscribe） | `Topic` | 类型化通信通道 |
| `Service`（request-response） | `Service` | 暂未完全支持 |
| `Publisher` | `rclcpp::Publisher` | 发布端 |
| `Subscriber` | `rclcpp::Subscription` | 订阅端 |
| `RouDi` | `rmw` + DDS Daemon | 服务发现与内存管理 |
| `WaitSet` | `rcl_wait_set_t` | 事件多路复用 |

---

## 三、共享内存模型

iceoryx2 的核心设计是**真正的零拷贝**：发布者和订阅者共享同一块物理内存，没有任何数据复制。

### 3.1 POSIX 共享内存布局

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     iceoryx2 共享内存段                                    │
│                                                                         │
│  /dev/shm/iceoryx2_<service_name>  (POSIX shm_open)                    │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │ Service Metadata  │ 服务配置（类型、容量、QoS 等）               │    │
│  ├────────────────────────────────────────────────────────────────┤    │
│  │ Sample Pool       │ 预分配的消息样本池（固定大小）               │    │
│  │  ┌──────────────┐ │                                             │    │
│  │  │ Sample #0    │ │  ← Publisher loan() 返回此内存地址          │    │
│  │  │ Sample #1    │ │  ← 订阅者 receive() 得到同一地址（无拷贝）   │    │
│  │  │ ...          │ │                                             │    │
│  │  └──────────────┘ │                                             │    │
│  ├────────────────────────────────────────────────────────────────┤    │
│  │ SPSC Queue       │ 无锁单生产者单消费者队列（存储指针/索引）       │    │
│  └────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 零拷贝数据路径

```
发布流程：
  publisher.loan()          ← 从 Sample Pool 借一块内存（O(1)，无 malloc）
    ↓
  *sample = JointState{...}  ← 直接写入共享内存（零拷贝）
    ↓
  publisher.send(sample)     ← 将内存块索引写入无锁 SPSC 队列（~100ns）
    ↓
  RouDi 通知（可选，用于 WaitSet 唤醒）

接收流程：
  subscriber.receive()       ← 从 SPSC 队列读取索引，返回共享内存指针
    ↓
  process(*sample)           ← 直接读取共享内存（订阅者和发布者访问同一物理页）
    ↓
  drop(sample)               ← 引用计数减一，归还 Sample Pool
```

### 3.3 内存安全保证

- **所有权**：`loan()` 返回的 `SampleMut<T>` 在 `send()` 后归还给系统，不可再访问（Rust 类型系统强制）
- **引用计数**：多个订阅者可同时持有同一内存块的只读引用，最后一个 `drop` 时归还
- **无 ABA 问题**：iceoryx2 使用代（generation）计数器防止 ABA 问题

---

## 四、RouDi 守护进程

`RouDi`（Routing and Discovery Daemon）是 iceoryx 的中心化服务，负责：

1. **共享内存管理**：为每个 Service 分配和管理共享内存段
2. **服务发现**：当 Publisher/Subscriber 创建时，注册到 RouDi
3. **进程监控**：检测进程崩溃，回收其占用的内存资源

```bash
# 启动 RouDi（通常作为系统服务运行）
# iceoryx1
iox-roudi

# iceoryx2
iox2-roudi

# 以指定配置启动（内存大小等）
iox2-roudi --config /etc/iceoryx2/roudi_config.toml
```

**RouDi 配置示例（TOML）：**

```toml
# /etc/iceoryx2/roudi_config.toml
[global]
# 共享内存段大小（字节）
shm_size = 104857600  # 100 MB

[monitoring]
# 进程心跳超时（毫秒）
heartbeat_timeout_ms = 5000

[[segment]]
# 为实时控制预留的高优先级内存段
reader = "rt_reader"
writer = "rt_writer"
size = 33554432  # 32 MB
```

---

## 五、rmw_iceoryx RMW 层

`rmw_iceoryx` 是 iceoryx 的 ROS 2 RMW 实现，允许在同机器 ROS 2 节点间使用 iceoryx 零拷贝 IPC，同时保留 DDS 用于跨机器通信。

### 5.1 安装与配置

```bash
# 安装（ROS 2 Humble）
sudo apt install ros-humble-rmw-iceoryx-cpp ros-humble-iceoryx-binding-c

# 启动 RouDi（必须在 ROS 2 节点前启动）
iox-roudi &

# 配置所有节点使用 iceoryx RMW
export RMW_IMPLEMENTATION=rmw_iceoryx_cpp

# 运行节点
ros2 run my_package my_rt_node
```

### 5.2 与 CycloneDDS 的混合使用

iceoryx 仅支持本机通信，跨机器仍需 DDS。常见架构是在同一机器内使用 iceoryx，跨机器使用 CycloneDDS：

```
┌──────────────────────────────────────────────────────────────┐
│  机器 A（机器人主机）                                           │
│                                                              │
│  controller_manager ─── iceoryx ──→ hardware_interface      │
│       ↑ iceoryx                                              │
│  VLA 节点 ─── iceoryx ──→ trajectory_controller             │
│                                                              │
│  ros2_bridge（DDS ↔ iceoryx 桥）                             │
│       ↕ CycloneDDS（UDP）                                    │
└──────────────────────────────────────────────────────────────┘
         ↕ 网络
┌──────────────────────────────────────────────────────────────┐
│  机器 B（操作控制台）                                           │
│  Foxglove Bridge ─── CycloneDDS ───→ 可视化                  │
└──────────────────────────────────────────────────────────────┘
```

**配置方式（CycloneDDS + iceoryx 混合）：**

```xml
<!-- cyclonedds.xml：启用 iceoryx 共享内存传输 -->
<CycloneDDS>
  <Domain>
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>info</LogLevel>
    </SharedMemory>
  </Domain>
</CycloneDDS>
```

```bash
# 同机器通信走 iceoryx（SHM），跨机器走 UDP
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
```

---

## 六、性能数据

### 6.1 延迟对比（端到端，单条消息，同机器）

| IPC 机制 | 消息大小 | P50 延迟 | P99 延迟 | P99.9 延迟 | 备注 |
|---------|---------|---------|---------|-----------|------|
| **iceoryx2** | 8 字节 | **< 100 ns** | **< 300 ns** | **< 1 μs** | 无 RouDi 通知模式 |
| **iceoryx2（WaitSet）** | 8 字节 | ~500 ns | ~2 μs | ~5 μs | 含事件通知 |
| **iceoryx1** | 8 字节 | ~1 μs | ~5 μs | ~20 μs | — |
| **CycloneDDS SHM** | 8 字节 | ~2 μs | ~10 μs | ~50 μs | 含序列化 |
| **CycloneDDS UDP（loopback）** | 8 字节 | ~15 μs | ~50 μs | ~200 μs | 含序列化 + 网络栈 |
| **FastDDS SHM** | 8 字节 | ~3 μs | ~15 μs | ~80 μs | — |
| **ROS 2 IPC（IntraProcess）** | 8 字节 | ~1 μs | ~5 μs | ~20 μs | 仅同进程 |

> 数据来源：iceoryx2 官方 benchmark（`cargo bench --bench publish_subscribe`），在 x86_64 Linux 5.15 PREEMPT_RT 上测量。

### 6.2 吞吐量

| IPC 机制 | 消息大小 | 最大吞吐量 | CPU 占用 |
|---------|---------|---------|--------|
| iceoryx2（WaitSet 轮询） | 4 KB | ~10 GB/s | 低（共享内存） |
| iceoryx2（零拷贝） | 1 MB | ~20 GB/s | 极低 |
| CycloneDDS SHM | 4 KB | ~2 GB/s | 中 |
| CycloneDDS UDP | 4 KB | ~500 MB/s | 较高 |

---

## 七、WaitSet — 事件驱动 vs 轮询

### 7.1 WaitSet API

`WaitSet` 允许 RT 线程等待多个事件（新消息到达、定时器触发），避免忙等待：

```rust
// Rust API 示例
let waitset = WaitSetBuilder::new().create::<zero_copy::Service>()?;

// 注册订阅者到 WaitSet
let _guard = waitset.attach_notification(&subscriber)?;

// 等待事件（阻塞，直到有消息或超时）
let result = waitset.wait_and_process_once(|attachment_id| {
    if attachment_id.corresponds_to(&subscriber) {
        if let Ok(Some(sample)) = subscriber.receive() {
            // 处理 sample（零拷贝）
        }
    }
    CallbackProgression::Continue
})?;
```

### 7.2 RT 场景下的选择

| 模式 | 适用场景 | 延迟 | CPU 占用 |
|------|---------|------|--------|
| **轮询（busy-wait）** | 4kHz 力控，最低延迟需求 | 最低（< 1μs） | 100%（独占一个核） |
| **WaitSet（事件驱动）** | 1kHz 控制，与 hrtimer 结合 | 低（~1-5μs） | 低（只在有事件时唤醒） |
| **WaitSet + 超时** | 防止丢包的 watchdog | 有界 | 极低 |

**推荐配置**：对于 1kHz 关节控制，推荐 WaitSet 模式（配合 PREEMPT_RT hrtimer 定时唤醒），节省 CPU 资源供 VLA 推理使用。对于 4kHz 力控，可考虑轮询（需要独占 CPU 核心）。

---

## 八、与 ros2_control 集成

### 8.1 ros2_control 中的 Loaned Messages + iceoryx2

ros2_control 的 `ResourceManager` 已经使用 `LoanedStateInterface` / `LoanedCommandInterface` 作为接口数据的引用（无拷贝传递）。结合 iceoryx2 可进一步实现跨进程零拷贝：

```cpp
// hardware_interface 中使用 iceoryx2 C 绑定
// （rmw_iceoryx 自动处理，用户代码无需直接调用 iceoryx2 API）

// read() 函数：从 iceoryx2 借用的内存中读取传感器数据
hardware_interface::return_type
ChassisHAL::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // 通过 hardware_interface 的接口值（LoanedStateInterface 内部已是共享内存）
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
        hw_positions_[i] = read_encoder(i);  // RT 安全的 HAL 读取
    }
    return hardware_interface::return_type::OK;
}
```

### 8.2 进程间通信架构（controller_manager + hardware_interface）

```
┌─────────────────────────────────────────────┐
│  ros2_control 进程                           │
│  ├─ controller_manager（SCHED_FIFO 90）      │
│  │    ↓ LoanedCommandInterface（本进程内）    │
│  ├─ ChainableController A                   │
│  └─ ChainableController B                   │
└─────────────────────────────────────────────┘
         ↕ iceoryx2 共享内存（零拷贝 IPC）
┌─────────────────────────────────────────────┐
│  hardware_interface 进程（可独立部署）         │
│  ├─ ChassisHAL::read()（SCHED_FIFO 85）      │
│  └─ ChassisHAL::write()                     │
│         ↕ RS485/EtherCAT                    │
│  物理执行器                                   │
└─────────────────────────────────────────────┘
```

---

## 九、局限性与适用边界

| 限制项 | 说明 | 变通方案 |
|-------|------|--------|
| **仅限单机** | 共享内存只在同一物理机上工作 | 跨机器使用 CycloneDDS/FastDDS |
| **不支持跨网络** | 无 UDP 传输层 | 与 DDS 混合使用（本机 iceoryx2，跨机 DDS） |
| **服务发现依赖 RouDi** | RouDi 进程崩溃会导致所有 iceoryx2 通信中断 | RouDi 运行为系统服务，自动重启 |
| **消息大小需提前配置** | SamplePool 中每个 Sample 大小固定 | 大消息（图像）需要单独 Service |
| **Rust 原生 API** | C++/C 绑定功能滞后于 Rust API | 使用 `iceoryx2-cxx`（C++ 绑定） |
| **rmw_iceoryx2 尚未稳定** | ROS 2 集成仍在开发中 | 生产环境使用 `rmw_iceoryx`（iceoryx1 绑定） |

---

## 十、快速上手配置

```bash
# ── 安装 iceoryx（Ubuntu 22.04）──────────────────────────────────────
sudo apt install ros-humble-iceoryx-binding-c \
                 ros-humble-rmw-iceoryx-cpp

# ── 安装 iceoryx2（从源码）──────────────────────────────────────────
cargo install iox2-cli
# 或通过 git 编译（需要 Rust 1.75+）

# ── 启动 RouDi 服务 ─────────────────────────────────────────────────
# 创建 systemd 服务文件
cat > /etc/systemd/system/iceoryx-roudi.service << 'EOF'
[Unit]
Description=iceoryx RouDi Service Discovery Daemon
After=network.target

[Service]
Type=forking
ExecStart=/usr/bin/iox-roudi
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable --now iceoryx-roudi

# ── ROS 2 节点使用 iceoryx RMW ──────────────────────────────────────
export RMW_IMPLEMENTATION=rmw_iceoryx_cpp
ros2 launch my_bringup rt_control.launch.py
```
