# 与本仓库已有组件的对接

> 本文档将 Phase 7 实时性技术栈与本仓库的 `chassis_protocol`（Phase 0 自研 HAL）和 Phase 6 rosbag2 录制方案具体对接，给出可操作的改造路线图和部署架构。

---

## 一、chassis_protocol RT 改造路线图

### 1.1 现状分析

通过分析 `chassis_protocol/CMakeLists.txt` 和相关源码，当前架构如下：

```
chassis_protocol
├── chassis/
│   ├── chassis_hal.cpp           ← 主要 HAL 实现
│   ├── version_negotiator.cpp   ← 版本协商（含动态分配和 mutex）
│   ├── differential/            ← 差速底盘运动学
│   └── mecanum/                 ← 麦克纳姆运动学
├── transport/
│   ├── rs485_transport.cpp      ← RS485 阻塞 I/O
│   ├── tcp_transport.cpp
│   └── udp_transport.cpp
└── ros2_adapter/
    └── chassis_node.cpp         ← ROS 2 适配层
```

**RT 不友好点清单：**

| 组件 | 问题 | 严重度 | 影响 |
|------|------|-------|------|
| `rs485_transport::read()` | 内部使用 `select()` 阻塞等待 | 🔴 高 | 若 RS485 无数据，read() 阻塞直到超时 |
| `version_negotiator::negotiate()` | 含多次动态分配和 mutex 锁 | 🔴 高 | 若在 RT 线程调用会引入不确定延迟 |
| `chassis_hal::read()` | 可能直接调用 transport read（阻塞） | 🔴 高 | RT 控制循环卡死 |
| `chassis_hal::write()` | 同步等待 RS485 ACK | 🟡 中 | 增加控制环延迟 |
| `differential_controller` | 未检查是否含动态分配 | 🟡 中 | 运动学计算通常安全 |

### 1.2 改造目标架构

```
改造后架构：

┌─────────────────────────────────────────────────────────┐
│  RT 线程（SCHED_FIFO 85，Core 2）                        │
│                                                         │
│  hardware_interface::SystemInterface                    │
│  ├─ read()   → rx_ring_buffer_.try_pop()  ← 无锁，O(1) │
│  └─ write()  → tx_ring_buffer_.push()     ← 无锁，O(1) │
└─────────────────────────────────────────────────────────┘
         ↑ 无锁环形缓冲区（Lock-free SPSC Ring Buffer）
┌─────────────────────────────────────────────────────────┐
│  RS485 驱动线程（SCHED_OTHER，Core 0）                   │
│                                                         │
│  RS485Driver::driver_thread()                           │
│  ├─ epoll_wait(rs485_fd, timeout=5ms)                   │
│  ├─ 收到数据 → decode_frame() → rx_ring_buffer_.push()  │
│  └─ tx_ring_buffer_.try_pop() → encode_frame() → write() │
└─────────────────────────────────────────────────────────┘
         ↕ RS485 串口
┌─────────────────────────────────────────────────────────┐
│  底盘驱动器（MCU 固件）                                   │
└─────────────────────────────────────────────────────────┘
```

### 1.3 read() / write() 改造：无锁、无动态分配

**Step 1：定义无锁环形缓冲区（SPSC Ring Buffer）**

```cpp
// chassis/rt/spsc_ring_buffer.hpp
// 单生产者单消费者无锁环形缓冲区
// 适用场景：RS485 驱动线程写入，RT 控制线程读取

#pragma once
#include <atomic>
#include <array>
#include <cstddef>

template<typename T, size_t Capacity>
class SPSCRingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0,
        "Capacity must be power of 2");
public:
    // 生产者调用（RS485 驱动线程）
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) & (Capacity - 1);
        if (next == tail_.load(std::memory_order_acquire))
            return false;  // 满了，丢弃（驱动线程处理）
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // 消费者调用（RT 控制线程）
    bool try_pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire))
            return false;  // 空
        item = buffer_[tail];
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    // 消费者调用：只查看最新数据（跳过旧数据）
    bool peek_latest(T& item) {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (head == tail) return false;
        // 读取 head-1 位置（最新的）
        size_t latest = (head - 1) & (Capacity - 1);
        item = buffer_[latest];
        tail_.store(head, std::memory_order_release);  // 丢弃中间帧
        return true;
    }

private:
    alignas(64) std::array<T, Capacity> buffer_;
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};
```

**Step 2：改造 ChassisHAL 使用无锁缓冲区**

```cpp
// chassis/chassis_hal_rt.hpp — RT 改造版本

#pragma once
#include <hardware_interface/system_interface.hpp>
#include "rt/spsc_ring_buffer.hpp"
#include "transport/rs485_transport.hpp"

struct SensorFrame {
    float left_position;    // 编码器计数
    float right_position;
    float left_velocity;
    float right_velocity;
    int64_t timestamp_ns;   // 传感器时间戳
};

struct CommandFrame {
    float left_velocity_cmd;
    float right_velocity_cmd;
    uint8_t checksum;
};

class ChassisHALRT : public hardware_interface::SystemInterface {
public:
    // ── 生命周期（非 RT 线程）────────────────────────────────────────────
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) override
    {
        // ✅ 允许：动态分配（配置阶段）
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        // ✅ 允许：版本协商（一次性，移出 RT 路径）
        transport_ = std::make_unique<RS485Transport>(
            declare_parameter<std::string>("port", "/dev/ttyUSB0"),
            declare_parameter<int>("baudrate", 115200));

        if (!transport_->open()) {
            RCLCPP_ERROR(get_logger(), "RS485 打开失败");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 版本协商（非 RT，允许阻塞）
        if (!do_version_negotiation()) {
            RCLCPP_ERROR(get_logger(), "版本协商失败");
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State&) override
    {
        // 启动 RS485 驱动线程（非 RT）
        driver_running_ = true;
        driver_thread_ = std::thread([this]() {
            this->rs485_driver_thread();
        });

        // 锁定内存（为 RT 做准备）
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_WARN(get_logger(), "mlockall 失败");
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State&) override
    {
        driver_running_ = false;
        if (driver_thread_.joinable()) driver_thread_.join();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ── RT 控制循环接口（严格 RT 安全）─────────────────────────────────
    hardware_interface::return_type read(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/) override
    {
        // ✅ 无锁，O(1)，确定性
        SensorFrame frame;
        if (rx_buffer_.peek_latest(frame)) {
            hw_positions_[0] = frame.left_position;
            hw_positions_[1] = frame.right_position;
            hw_velocities_[0] = frame.left_velocity;
            hw_velocities_[1] = frame.right_velocity;
        }
        // 若无新数据，保持上一次值（安全的退化行为）
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/) override
    {
        // ✅ 无锁，O(1)，确定性
        CommandFrame cmd;
        cmd.left_velocity_cmd  = static_cast<float>(hw_commands_[0]);
        cmd.right_velocity_cmd = static_cast<float>(hw_commands_[1]);
        // 简单校验和（栈计算，无动态分配）
        cmd.checksum = static_cast<uint8_t>(
            cmd.left_velocity_cmd + cmd.right_velocity_cmd);

        tx_buffer_.push(cmd);  // 无锁推入，驱动线程异步发送
        return hardware_interface::return_type::OK;
    }

private:
    // RS485 驱动线程（非 RT）
    void rs485_driver_thread() {
        // 使用 epoll_wait 替代阻塞 read（确定性超时）
        int epoll_fd = epoll_create1(0);
        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.fd = transport_->fd();
        epoll_ctl(epoll_fd, EPOLL_CTL_ADD, transport_->fd(), &ev);

        while (driver_running_.load(std::memory_order_relaxed)) {
            struct epoll_event events[1];
            int nfds = epoll_wait(epoll_fd, events, 1, 5);  // 5ms 超时

            if (nfds > 0) {
                SensorFrame frame;
                if (transport_->read_frame(frame)) {
                    rx_buffer_.push(frame);
                }
            }

            // 发送待发命令
            CommandFrame cmd;
            while (tx_buffer_.try_pop(cmd)) {
                transport_->write_frame(cmd);
            }
        }
        close(epoll_fd);
    }

    bool do_version_negotiation() {
        // 发送版本请求帧，等待响应（允许阻塞，非 RT）
        // ... 原 version_negotiator 逻辑 ...
        return true;
    }

    // 无锁缓冲区（16 帧容量，适合 1kHz 控制 + 200Hz RS485）
    SPSCRingBuffer<SensorFrame, 16>  rx_buffer_;
    SPSCRingBuffer<CommandFrame, 16> tx_buffer_;

    std::unique_ptr<RS485Transport> transport_;
    std::thread driver_thread_;
    std::atomic<bool> driver_running_{false};

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
};
```

### 1.4 RS485 Transport 的确定性读写

`select()` 超时策略 → `epoll_wait` 替换：

```cpp
// transport/rs485_transport_rt.cpp — 改造版本

// ❌ 旧代码（阻塞 select，不确定延迟）
bool RS485Transport::read_blocking(SensorFrame& frame, int timeout_ms) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    struct timeval tv = {0, timeout_ms * 1000};
    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    // ...
}

// ✅ 新代码（epoll，驱动线程专用，不在 RT 线程调用）
class RS485TransportRT {
public:
    bool open_epoll() {
        epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
        struct epoll_event ev;
        ev.events = EPOLLIN | EPOLLET;  // 边沿触发
        ev.data.fd = serial_fd_;
        return epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, serial_fd_, &ev) == 0;
    }

    // 在驱动线程中调用（非 RT）
    int wait_readable(int timeout_ms) {
        struct epoll_event events[1];
        return epoll_wait(epoll_fd_, events, 1, timeout_ms);
    }

private:
    int epoll_fd_{-1};
    int serial_fd_{-1};
};
```

---

## 二、Phase 6 录制进程隔离

### 2.1 录制进程 CPU 隔离策略

rosbag2 录制对控制系统的主要影响：
1. 占用 CPU 时间（序列化、磁盘 I/O）
2. 触发内核 page cache 操作（影响 RT 进程的内存性能）
3. 磁盘 I/O 可能触发中断（若磁盘和 RT 核共享 IRQ）

**隔离方案：**

```bash
# rosbag2 录制进程绑定到非 RT 核（Core 6,7）
taskset -c 6,7 ros2 bag record \
    /joint_states /odom /cmd_vel /imu/data \
    --storage mcap \
    -o /data/bags/episode_001

# 或通过 launch 文件
# ros2_bag_recorder.launch.py
```

```python
# ros2_bag_recorder.launch.py — 带 CPU 隔离的录制 launch 文件
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rosbag2_recorder = Node(
        package='rosbag2_transport',
        executable='record',
        name='rosbag2_recorder',
        # 绑定到非 RT 核心，避免与控制器竞争
        prefix=['taskset', '-c', '6,7'],
        parameters=[{
            'storage.storage_id': 'mcap',
            'storage.uri': '/data/bags/episode',
            'record.all_topics': False,
            'record.topics': [
                '/joint_states',
                '/odom',
                '/cmd_vel',
                '/imu/data',
                '/tf',
                '/tf_static',
            ],
        }]
    )
    return LaunchDescription([rosbag2_recorder])
```

### 2.2 与 controller_manager 的优先级隔离

```
进程优先级矩阵：

进程                           调度策略        优先级   CPU 核
───────────────────────────────────────────────────────────────
controller_manager update()   SCHED_FIFO     90      Core 2,3
hardware_interface threads    SCHED_FIFO     85      Core 2,3
RT Executor（ROS 2 节点）      SCHED_FIFO     80      Core 2,3
─────────────── 优先级边界 ────────────────────────────────────
DDS EventThread               SCHED_FIFO     40      Core 4,5
ksoftirqd                     SCHED_FIFO     50      Core 0,1
───────────────────────────────────────────────────────────────
rosbag2 录制进程               SCHED_OTHER    0       Core 6,7
VLA 推理进程                   SCHED_OTHER    0       Core 7
Foxglove Bridge                SCHED_OTHER    0       Core 6,7
```

---

## 三、整体 RT 架构部署图

### 3.1 CPU 核心分配（8 核 Jetson Orin AGX 示例）

```
Jetson Orin AGX（12核：4× Cortex-A78AE 大核 + 8× Cortex-A78AE 小核）

大核（高性能，推荐 RT）：
┌─────────────────────────────────────────────────────────────────┐
│ Core 0-1（大核）：OS 系统服务 / IRQ / DDS 线程                    │
│   taskset: 不隔离                                                │
│   调度：SCHED_OTHER + SCHED_FIFO（DDS/IRQ）                     │
├─────────────────────────────────────────────────────────────────┤
│ Core 2-3（大核，isolcpus=2,3）：实时控制                          │
│   controller_manager + hardware_interface                        │
│   调度：SCHED_FIFO 85-90                                         │
│   内存：mlockall，nohz_full                                      │
├─────────────────────────────────────────────────────────────────┤
│ Core 4-5（大核）：VLA GPU 推理调度 + ROS 2 非 RT 节点             │
│   Nav2 / tf2 / 传感器处理                                        │
│   调度：SCHED_OTHER                                              │
│   GPU：CUDA 流异步（不阻塞 CPU RT 核）                            │
├─────────────────────────────────────────────────────────────────┤
│ Core 6-7（大核）：rosbag2 录制 / Foxglove Bridge / 遥测           │
│   调度：SCHED_OTHER（I/O 密集型）                                │
│   优先级：nice +10（主动降低优先级）                               │
└─────────────────────────────────────────────────────────────────┘

小核（低功耗，适合后台任务）：
┌─────────────────────────────────────────────────────────────────┐
│ Core 8-11（小核）：后台服务 / 日志 / 系统监控                      │
│   journald / rsyslog / SSH / prometheus                          │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 ROS 2 通信拓扑

```
┌─────────────────────────────────────────────────────────────────────────┐
│  具身智能完整 ROS 2 RT 通信拓扑                                            │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │  实时控制域（Core 2,3）                                            │  │
│  │                                                                  │  │
│  │  controller_manager                                              │  │
│  │    ├─ diff_drive_controller                                      │  │
│  │    │    ← /cmd_vel（iceoryx2，< 1μs）                            │  │
│  │    │    → /odom（iceoryx2，同进程）                               │  │
│  │    ├─ joint_state_broadcaster                                    │  │
│  │    │    → /joint_states（iceoryx2 → 降频 CycloneDDS）            │  │
│  │    └─ ChassisHAL（hardware_interface）                           │  │
│  │         ← SPSCRingBuffer ← RS485 驱动线程                        │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│            ↕ CycloneDDS SHM（同机，< 5μs）                              │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │  中间层（Core 4,5）                                               │  │
│  │                                                                  │  │
│  │  cmd_vel_mux（twist_mux）                                        │  │
│  │    ← /cmd_vel/joystick（优先级 100）                             │  │
│  │    ← /cmd_vel/nav2（优先级 50）                                  │  │
│  │    ← /cmd_vel/vla（优先级 10）                                   │  │
│  │    → /cmd_vel（最高优先级有效命令）                                │  │
│  │                                                                  │  │
│  │  nav2_stack                                                      │  │
│  │  vla_action_client                                               │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│            ↕ CycloneDDS UDP（跨机器）                                   │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │  远端（操作控制台）                                                 │  │
│  │  foxglove_bridge → Foxglove Studio（Web）                        │  │
│  │  rosbag2_player（回放）                                           │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 四、Jetson Orin 特化配置

### 4.1 CUDA 流 vs RT 控制线程的分离

Jetson Orin 的 GPU（Ampere 架构）共享系统内存（统一内存架构），CUDA 操作可能影响 CPU 缓存和内存带宽：

```bash
# Jetson Orin 特化配置脚本
# orin_rt_setup.sh

#!/bin/bash

# 1. 设置最大性能模式（避免动态调频影响 RT）
sudo nvpmodel -m 0    # MAXN 模式
sudo jetson_clocks    # 锁定所有时钟

# 2. 固定 CPU 频率
for cpu in /sys/devices/system/cpu/cpu{0..3}; do
    echo performance | sudo tee ${cpu}/cpufreq/scaling_governor
done

# 3. 设置 GPU 优先级（降低 GPU 任务对 CPU 的干扰）
# 在 CUDA 应用中设置低优先级流
# cudaStreamCreateWithPriority(&stream, cudaStreamNonBlocking, -1);

# 4. NUMA 配置（Orin 是统一内存架构，NUMA 不适用，但需关闭 NUMA balancing）
sudo sysctl kernel.numa_balancing=0

# 5. 内存配置
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/enabled
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/defrag

echo "Jetson Orin RT 配置完成"
```

### 4.2 tegrastats 监控

```bash
# 实时监控 CPU/GPU/内存使用（每秒采样）
tegrastats --interval 1000 | tee /var/log/tegrastats.log

# 关键指标（控制过程中监控）：
# CPU[0-3]: RT 核心使用率（应 < 80% 以保留余量）
# CPU[4-7]: 其他核心使用率
# GPU: VLA 推理 GPU 使用率
# RAM: 内存使用（RT 系统应使用 mlockall，减少 page fault）
# SWAP: 应为 0（RT 系统不允许 swap）

# 禁用 swap（RT 必须项）
sudo swapoff -a
# 持久化（/etc/fstab 中注释掉 swap 行）
```

---

## 五、容器化 RT

### 5.1 Docker RT 容器配置

在 Docker 中运行 RT 控制系统有以下关键约束：

```bash
# RT 容器启动命令
docker run \
    --privileged \
    --ulimit memlock=-1:-1 \
    --ulimit rtprio=99:99 \
    --cpuset-cpus="2,3" \
    --network=host \
    --pid=host \
    --ipc=host \
    -v /dev:/dev \
    -v /sys:/sys:ro \
    -v /proc:/proc:ro \
    my_ros2_rt_image:humble \
    ros2 launch chassis_bringup rt_control.launch.py
```

**关键选项说明：**

| 选项 | 说明 | RT 必要性 |
|------|------|---------|
| `--privileged` | 允许容器内使用所有 Linux capabilities | 必须（mlockall、SCHED_FIFO 需要 CAP_SYS_NICE、CAP_IPC_LOCK） |
| `--ulimit memlock=-1:-1` | 取消内存锁定大小限制 | 必须（mlockall） |
| `--ulimit rtprio=99:99` | 允许设置 RT 优先级 | 必须（SCHED_FIFO） |
| `--cpuset-cpus="2,3"` | 绑定容器到指定 CPU 核 | 强烈推荐 |
| `--network=host` | 使用宿主机网络（避免网络 namespace 隔离引入延迟） | 推荐 |
| `--ipc=host` | 共享宿主机 IPC namespace（iceoryx 共享内存跨容器） | iceoryx 必须 |

### 5.2 裸机 vs 容器 RT 延迟对比

| 部署方式 | 额外延迟开销 | cyclictest 最大延迟 | 推荐场景 |
|---------|-----------|-----------------|--------|
| **裸机（直接运行）** | 无 | < 50μs | 生产力控、最严格 RT |
| **Docker（--privileged + cpuset）** | ~5-15μs（namespace 开销） | < 100μs | 开发/测试，接受轻微延迟 |
| **Docker（无特殊配置）** | ~100-500μs（调度隔离） | 数毫秒 | 非 RT 应用 |
| **Kubernetes（RT 节点）** | ~50-200μs（cgroup 开销） | < 500μs | 大规模部署但 RT 要求宽松 |

**结论**：对于 1kHz 关节控制，Docker `--privileged + cpuset` 方案可接受（额外 15μs 在 1ms 预算中占 1.5%）；对于 4kHz 力控，推荐裸机部署。

---

## 六、选型决策矩阵

### 6.1 PREEMPT_RT 必选/可选/不需要场景

| 场景 | 控制频率 | 最大允许延迟 | PREEMPT_RT | 说明 |
|------|---------|-----------|-----------|------|
| 力矩/力控（精密抓取） | 4 kHz | < 250 μs | **必选** | 没有 RT 内核无法达标 |
| 关节速度控制（工业机械臂） | 1 kHz | < 1 ms | **必选** | 建议 RT，不 RT 有风险 |
| 差速底盘速度控制 | 100–500 Hz | < 10 ms | **可选** | 普通内核通常能满足 |
| 视觉伺服（30-100Hz） | 30–100 Hz | < 33 ms | **不需要** | CFS 调度即可 |
| VLA 策略推理 | 10–30 Hz | < 100 ms | **不需要** | GPU 推理为主要瓶颈 |
| 导航规划（Nav2） | 5–10 Hz | < 200 ms | **不需要** | 算法延迟为主 |

### 6.2 iceoryx2 vs DDS 通信选型

| 通信场景 | 延迟要求 | 是否跨机器 | 推荐方案 | 理由 |
|---------|---------|---------|--------|------|
| controller_manager ↔ hardware_interface（同进程） | < 1μs | 否 | **ros2_control 内部接口**（已是内存引用） | 无需额外 IPC |
| 控制器 ↔ 传感器节点（同机器） | < 5μs | 否 | **iceoryx2 / CycloneDDS SHM** | 零拷贝，避免 DDS 序列化 |
| 机器人主控 ↔ 操作控制台 | < 100ms | **是** | **CycloneDDS UDP BEST_EFFORT** | 唯一支持跨网络的方案 |
| 机器人 ↔ rosbag2 录制 | 宽松 | 否 | **CycloneDDS（匹配 QoS）** | 不影响 RT，单机 SHM 更好 |
| 机器人 ↔ VLA 模型服务 | < 100ms | **是**（远端 GPU 服务器） | **gRPC / HTTP**（非 ROS 2） | 跨协议，低频 |

### 6.3 chassis_protocol 集成决策

| 问题 | 建议决策 | 理由 |
|------|---------|------|
| 是否将 chassis_protocol 改造为 `hardware_interface::SystemInterface`？ | **是，长期目标** | 统一到 ros2_control 框架，获得 RT 控制循环、ChainableController 等能力（见 Phase 1 评估） |
| 改造后 RS485 Transport 是否保留？ | **保留，但加无锁缓冲层** | RS485 传输本身合理，只需解决 RT 阻塞问题 |
| version_negotiator 是否保留？ | **保留，但移出 RT 路径** | 在 `on_configure()`/`on_activate()` 中执行，RT 线程不调用 |
| 是否引入 iceoryx2 替换 DDS？ | **短期：CycloneDDS SHM；长期：iceoryx2** | CycloneDDS SHM 可用性更好，iceoryx2 待 rmw_iceoryx2 成熟后迁移 |

---

## 七、完整 RT 系统启动脚本

```bash
#!/bin/bash
# start_rt_system.sh — 具身智能 RT 控制系统启动脚本

set -e

echo "=== 具身智能 RT 控制系统启动 ==="

# ── 1. 验证 RT 内核 ─────────────────────────────────────────────────
if ! grep -q "PREEMPT_RT" /proc/version; then
    echo "❌ 需要 PREEMPT_RT 内核"
    exit 1
fi

# ── 2. 系统优化 ──────────────────────────────────────────────────────
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/enabled
sudo swapoff -a
sudo systemctl stop irqbalance

# 固定 CPU 频率（避免动态调频）
for cpu in /sys/devices/system/cpu/cpu{0..3}/cpufreq; do
    echo performance | sudo tee ${cpu}/scaling_governor 2>/dev/null || true
done

# ── 3. 启动 iceoryx RouDi ────────────────────────────────────────────
if ! pgrep iox-roudi > /dev/null; then
    sudo iox-roudi &
    sleep 1
    echo "✅ RouDi 已启动"
fi

# ── 4. 配置 RMW ──────────────────────────────────────────────────────
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds_shm.xml
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# ── 5. 启动 RT 控制节点（Core 2,3，SCHED_FIFO 90）────────────────────
sudo taskset -c 2,3 chrt -f 90 \
    ros2 launch chassis_bringup rt_control_system.launch.py &
RT_PID=$!

echo "✅ RT 控制节点已启动 (PID: $RT_PID)"

# ── 6. 启动录制节点（Core 6,7，低优先级）────────────────────────────
taskset -c 6,7 \
    ros2 bag record -o /data/bags/$(date +%Y%m%d_%H%M%S) \
    /joint_states /odom /cmd_vel /imu/data /tf /tf_static &
BAG_PID=$!
echo "✅ rosbag2 录制已启动 (PID: $BAG_PID)"

# ── 7. 启动 Foxglove Bridge（Core 6,7）──────────────────────────────
taskset -c 6,7 \
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

echo "=== 启动完成 ==="
echo "RT 控制:  Core 2,3，SCHED_FIFO 90"
echo "录制:     Core 6,7，SCHED_OTHER"
echo "Foxglove: http://localhost:8765"

wait $RT_PID
```
