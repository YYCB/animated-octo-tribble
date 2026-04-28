# NITROS 与 custom-fork 路线图对照 — 已覆盖 / 未覆盖分析

> 本文对照本仓库 `docs/ros2-roadmaps/ros2_custom_fork_roadmap/02_optimization_dimensions.md` 中的 12 条性能杠杆（D1~D12），逐项分析 Isaac ROS / NITROS 的覆盖情况与互补关系。

---

## 一、对照总表

| 维度 | 描述 | NITROS 覆盖程度 | 说明 |
|------|------|:--------------:|------|
| **D1** Executor 模型 | EventsExecutor / 静态 Executor 降调度延迟 | ⭕ 部分覆盖 | GXF 调度器替代了 rclcpp Executor（在 NITROS 节点内部）；边界 Topic 仍走标准 Executor |
| **D2** 回调组 & 线程模型 | 控制环独占高优先级线程 | ❌ 未覆盖 | NITROS 不处理 rclcpp 线程模型；仍需自行配置 SCHED_FIFO |
| **D3** 内存分配策略 | TLSF / Pool allocator 消除热路径 malloc | ⭕ 部分覆盖 | GXF 内部使用固定池分配 GPU buffer；CPU 侧 rclcpp 消息分配未改变 |
| **D4** 消息拷贝次数（零拷贝） | IntraProcess + LoanedMessage + SHM | ✅ **超越覆盖** | NITROS TypeAdapter 实现了 GPU 侧完整零拷贝，比 LoanedMessage 更彻底（跨进程也零拷贝） |
| **D5** 序列化格式 | CDR v2 / FlatBuffers / 裸结构体 | ✅ **有效绕过** | NITROS 对图像/张量完全不序列化，传递的是 NvSci 句柄（20 字节）；序列化问题在 GPU 路径上不存在 |
| **D6** 传输层（SHM/Iceoryx） | 同机进程间 SHM 替代 UDP | ✅ **超越覆盖** | NvSci 是专为 GPU 设计的跨进程共享内存，比 Iceoryx 更适合 GPU buffer 传输 |
| **D7** DDS 选择 & 调参 | FastDDS → CycloneDDS，调优参数 | ❌ 未覆盖 | NITROS 不替换 DDS；仍需单独优化 DDS 配置 |
| **D8** QoS 策略 | BEST_EFFORT + KEEP_LAST=1 降延迟 | ⭕ 部分覆盖 | NITROS Publisher 默认使用 `SensorDataQoS()`（BEST_EFFORT + KEEP_LAST=10）；可自定义 |
| **D9** 发现协议 | Discovery Server 降冷启动时间 | ❌ 未覆盖 | NITROS 节点仍走标准 DDS 发现；大规模部署仍需配置 Discovery Server |
| **D10** 实时内核 & 调度 | PREEMPT_RT + SCHED_FIFO | ❌ 未覆盖 | NITROS 本身不提供实时保证；GXF 调度器不是硬实时调度器 |
| **D11** CPU 亲和 & IRQ 亲和 | isolcpus + 线程绑核 | ❌ 未覆盖 | 需单独配置（与 NITROS 无直接关联） |
| **D12** 编译器 & LTO & PGO | -O3 / LTO / PGO 编译优化 | ⭕ 部分覆盖 | NVIDIA 已对 TensorRT / cuVSLAM 做过高度优化；rclcpp 层的 LTO/PGO 仍需自行配置 |

图例：✅ = 有效覆盖或超越；⭕ = 部分覆盖；❌ = 未覆盖（仍需 custom-fork 处理）

---

## 二、NITROS 已有效覆盖的优化项（D4 / D5 / D6）

### D4：消息零拷贝（GPU 路径超越 LoanedMessage）

custom-fork 路线图中，D4 的最佳实现是 LoanedMessage + SHM，对 CPU 侧大消息实现零拷贝。

NITROS 在 GPU 路径上实现了**更彻底的零拷贝**：

```
custom-fork LoanedMessage（CPU 路径）：
  发布端：原地填充 DDS SHM 内存（1 次写）
  订阅端：直接引用 DDS SHM（0 拷贝读）
  跨进程：需 DDS SHM 配置，仍经过 ringbuffer 同步
  适用范围：任意 ROS 消息类型（含 CPU 大消息）

NITROS TypeAdapter + NvSci（GPU 路径）：
  发布端：GPU buffer 直接由相机/传感器填充（0 写）
  订阅端：通过 NvSci 句柄映射（0 拷贝读）
  跨进程：NvSci 跨进程句柄，物理内存不移动
  适用范围：图像 / 点云 / Tensor（GPU 加速算法）
```

**结论**：两者互补，不冲突。LoanedMessage 处理 CPU 控制消息（JointState、TwistStamped 等），NITROS 处理 GPU 感知消息（图像、点云、Tensor）。

### D5：序列化（GPU 路径完全绕过）

NITROS 对 GPU buffer 的跨节点传输：
- **同进程**：零序列化（TypeAdapter + IntraProcess）
- **跨进程（NvSci）**：传递 20 字节句柄，无 CDR 序列化
- **降级路径**：才触发 CDR 序列化（GPU→CPU 拷贝 + convert_to_ros_message）

因此 custom-fork 中关于"替换序列化格式"的优化项对于图像/张量流完全不适用（NITROS 已绕过了序列化）。该优化项的价值主要体现在**控制消息**（小消息）的场景。

### D6：传输层（NvSci > Iceoryx for GPU）

| 比较项 | Iceoryx | NvSci |
|--------|---------|-------|
| 设计目标 | CPU SHM，通用 ROS 消息 | GPU/VIC/DLA 跨进程共享 |
| 延迟 | 2-5µs（CPU 侧） | ~50µs（首次 export/import） |
| 吞吐（GPU 路径） | 需要 GPU→CPU→SHM→CPU→GPU | 0 字节移动 |
| Jetson 集成 | 一般（不感知 GPU 拓扑） | 深度集成（感知 iGPU/DLA/VIC） |

**结论**：在 Jetson 平台上，NITROS NvSci 是比 Iceoryx 更好的 GPU buffer 传输方案；在 x86+dGPU 上，Iceoryx 仍是 CPU 大消息的最佳选择。

---

## 三、NITROS 未覆盖、仍需 custom-fork 处理的优化项

### D1：Executor 模型（控制环）

NITROS 的 GXF Scheduler 优化了 GPU 感知算子的调度，但 ROS 2 控制环（ros2_control 的 ControllerManager）仍使用标准 rclcpp Executor：

**仍需做**：
```
ControllerManager（1000 Hz 实时循环）
  → 仍使用 rclcpp::TimerBase + Executor
  → 需要 EventsExecutor 或静态 Executor
  → 需要 SCHED_FIFO 线程优先级
```

→ custom-fork 的 Phase 2（Executor 定制）对 **控制环** 仍然必要。

### D2：线程模型（控制 vs 感知分离）

具身智能整机的线程模型建议：
```
核 0-1（SCHED_FIFO，实时）：ControllerManager 循环
核 2-3（SCHED_FIFO，次优先）：ros2_control 读写 HW
核 4-7（SCHED_OTHER）：NITROS 推理管线（GXF 内部线程）
核 8+（SCHED_OTHER）：ROS 2 通用节点、rviz、诊断
```

这个线程模型需要 custom-fork 配合 PREEMPT_RT 内核配置，NITROS 本身不提供。

### D7：DDS 配置

NITROS 节点仍通过 DDS 传递非 GPU 消息（检测结果、状态信息等）和协商元数据。DDS 的调优（CycloneDDS / FastDDS 选择、QoS 参数、multicast 配置）仍需单独进行。

### D10/D11：实时内核（具身智能关键需求）

NITROS 的感知管线（图像→DNN→检测）不是硬实时系统，但底层控制环（ros2_control）需要：
- PREEMPT_RT 内核（保证 ControllerManager 的周期稳定性）
- 控制线程 CPU 隔离（避免感知管线的 GPU 中断/DMA 引发抖动）

---

## 四、完整优化路线图（NITROS + custom-fork 组合）

```
┌──────────────────────────────────────────────────────────────────────┐
│           具身智能整机优化路线（NITROS + custom-fork 组合）           │
├───────────────────────────────────────────┬──────────────────────────┤
│          感知层（图像 / 点云 / Tensor）     │    控制层（关节 / 底盘）  │
├───────────────────────────────────────────┼──────────────────────────┤
│ Isaac ROS NITROS                          │ custom-fork（必要）       │
│  ✅ D4 GPU 零拷贝（TypeAdapter + NvSci）   │  ✅ D1 EventsExecutor     │
│  ✅ D5 绕过序列化（GPU buffer 直传）       │  ✅ D2 控制线程 SCHED_FIFO│
│  ✅ D6 NvSci 跨进程 GPU 内存共享          │  ✅ D10 PREEMPT_RT 内核    │
│  ⭕ D3 GXF 内部 GPU buffer 池分配         │  ✅ D11 CPU 隔离 / 亲和    │
│  ⭕ D1 GXF 调度器（仅 NITROS 内部）       │  ✅ D3 实时 allocator       │
│                                           │  ✅ D7 CycloneDDS 调优     │
│ 仍需 custom-fork：                        │                           │
│  ❌ D7 DDS 配置（非 GPU 消息路径）         │                           │
│  ❌ D9 Discovery Server（大规模节点）      │                           │
│  ❌ D12 rclcpp 层 LTO/PGO               │                           │
└───────────────────────────────────────────┴──────────────────────────┘
```

---

## 五、给 custom-fork 路线图追加的对照小节

> 建议在 `docs/ros2-roadmaps/ros2_custom_fork_roadmap/05_phase3_rmw_zero_copy.md` 末尾追加：

### NITROS 对 D4/D5/D6 的影响

若项目使用 Isaac ROS NITROS（目标硬件为 Jetson 或 x86+NVIDIA dGPU），则：

1. **图像 / 点云 / Tensor 类消息**：
   - NITROS TypeAdapter + NvSci 已提供比 LoanedMessage 更彻底的 GPU 零拷贝
   - Phase 3（rmw 零拷贝）和 Phase 4（Iceoryx）对这类消息**价值大幅降低**
   - **建议**：直接使用 NITROS，不在感知消息路径上投入 custom-fork 优化工作

2. **控制消息（JointState, TwistStamped, Odometry）**：
   - 这类消息小（< 1KB）且时间敏感
   - NITROS 未处理这类消息
   - **建议**：对控制消息仍做 LoanedMessage + IntraProcess 优化（如 ControllerManager 与控制器在同进程）

3. **跨机通信**：
   - NvSci 仅限单机（Jetson 或单 GPU 机器）
   - 跨机仍需 DDS UDP，D5/D6 的优化对跨机场景仍然有价值

---

## 六、选型建议（总结）

| 场景 | 推荐方案 |
|------|---------|
| Jetson 平台，全感知推理管线 | Isaac ROS NITROS（覆盖 D4/D5/D6） |
| x86 + dGPU，GPU 推理 | Isaac ROS NITROS（CUDA IPC 替代 NvSci）|
| 控制环（任何平台） | custom-fork D1/D2/D10/D11 |
| 非 GPU 业务消息（服务、参数等） | custom-fork D7/D8/D9 |
| 纯 CPU 大消息（无 GPU）| custom-fork D4 LoanedMessage + D6 Iceoryx |
| 多机分布式（跨网络） | custom-fork D6 DPDK/XDP + D9 Discovery Server |
