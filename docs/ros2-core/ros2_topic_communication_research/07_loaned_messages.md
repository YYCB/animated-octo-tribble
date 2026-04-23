# 07 Loaned Messages —— 跨进程零拷贝机制

> 本章接 [06 IntraProcess](./06_intra_process.md)。IPC 解决同进程，Loaned Message 解决**跨进程同机**（以及跨机 SHM 场景）的零拷贝问题。

---

## 7.1 为什么需要 Loaned Message

IPC 路径下零拷贝是"进程内指针传递"。跨进程时即使走 SHM，**默认也仍有拷贝**：

```
用户构造 MsgT          (栈或堆)
  ↓ memcpy
CDR 序列化缓冲         (rmw 层 new)
  ↓ memcpy
DDS history cache     (DDS 内部 new)
  ↓ memcpy
SHM ringbuffer segment (DDS 分配)
```

共 3 次拷贝。即使 SHM 传输不做第三次拷贝（可能原地写），前两次仍然存在。

**Loaned Message 的思路**：
- **颠倒构造顺序**——不是"用户先造消息再发布"，而是"向 DDS 借一块 SHM 里的内存，用户原地构造"；
- 一次都不拷贝：从用户写入 → 对端读取，是同一块物理内存；
- 订阅端也可用 `take_loaned_message` 直接拿这块内存，零拷贝。

---

## 7.2 数据流对比

### 传统路径

```
[用户 MsgT on stack]
       │ copy
       ▼
[CDR buffer on heap]  ← rmw_serialize
       │ copy
       ▼
[DDS history cache]   ← DDS 内部
       │ copy
       ▼
[SHM segment]         ← 传输
       │ (pointer)
       ▼
[订阅端 CDR buffer]   ← rmw_take
       │ deserialize
       ▼
[订阅端 MsgT on heap]
```

### Loaned 路径

```
[SHM segment slot]   ← borrow_loaned_message 分配
   ▲
   │ 用户原地构造（reinterpret_cast<MsgT*>）
   │ 填充字段
   │
publish_loaned()
   │
   │ DDS 只更新 ringbuffer 索引（无拷贝）
   ▼
[订阅端 take_loaned]
   │
   ▼
[订阅端直接读 SHM segment]  ← 与发布端同一块物理内存
```

**极限可达**：1 MB 消息传输 < 10 μs，纯粹受内存带宽和 cache line 同步限制。

---

## 7.3 API 与使用方式

### 7.3.1 发布端

```cpp
auto pub = node->create_publisher<my_msgs::FixedMsg>("topic", qos);

if (!pub->can_loan_messages()) {
  // Fallback 到传统路径
  auto msg = std::make_unique<my_msgs::FixedMsg>();
  fill(*msg);
  pub->publish(std::move(msg));
  return;
}

auto loaned = pub->borrow_loaned_message();   // 向 rmw 借
loaned.get().field_a = 1.23;                   // 原地填
loaned.get().field_b[0] = 4.56;
// ...
pub->publish(std::move(loaned));               // 发送 + 释放所有权
```

### 7.3.2 订阅端（较少直接使用）

多数订阅者仍用回调：
```cpp
auto sub = node->create_subscription<my_msgs::FixedMsg>(
    "topic", qos,
    [](std::shared_ptr<const my_msgs::FixedMsg> msg) {
      // 此处 msg 背后可能就是 SHM 地址，rclcpp 透明处理
    });
```

rclcpp 的回调路径会**自动走 loaned take**（若可行），然后包装为 `shared_ptr<const MsgT>`——用户不需要写特殊代码。

底层显式 API：
```cpp
// rcl 层
const MsgT * loaned_ptr = nullptr;
rcl_take_loaned_message(sub_handle, (void**)&loaned_ptr, nullptr, nullptr);
// 处理
rcl_return_loaned_message_from_subscription(sub_handle, (void*)loaned_ptr);
```

---

## 7.4 `can_loan_messages()` 的判定

返回 true 的**充要条件**：

1. **rmw 实现支持 loan**：
   - `rmw_fastrtps_cpp` >= 一定版本 + SHM transport 启用；
   - `rmw_cyclonedds_cpp` + Iceoryx 启用；
   - `rmw_zenoh_cpp` 部分支持。
2. **消息类型是 "fixed memory"**：
   - rosidl 生成代码时被识别为 POD / bitwise copyable；
   - **关键判据**：消息 IDL 不含 `string`（非 bounded）、不含 dynamic `sequence`、不含嵌套的变长成员。
3. **Publisher QoS 不要求转储到 history cache（由 DDS 实现决定）**；
4. **消息大小不超过 SHM chunk 上限**（Iceoryx 默认 4 MB，可配）。

**三条里任一不满足就退化为传统路径**——`can_loan_messages()` 返回 false。

---

## 7.5 消息类型设计：支持 Loan 的 IDL 约束

**可以 Loan**：
```idl
# fixed.msg
float64[7] joint_positions
float64[7] joint_velocities
int64 timestamp_ns
int32 sequence_id
uint8[16] header_uuid
```
无变长字段 → POD → 可以 Loan。

**不能 Loan**（包含 `std::string`）：
```idl
# bad.msg
string frame_id              # ❌
float64 x
```

**改造方案 A：使用 bounded**：
```idl
# good.msg
string<=16 frame_id          # ✅ BoundedString
float64 x
```
rosidl 会生成固定容量 `char[17]`（16 chars + null）而非 `std::string`，满足 POD 要求。

**改造方案 B：用固定容量数组**：
```idl
float64[] positions          # ❌ 动态 sequence
# → 改为
float64[32] positions        # ✅ 固定数组，max 32 元素
```
代价：总是占 32 × 8 = 256 B，即使只用 7 个（内存换性能）。

**改造方案 C：BoundedSequence**：
```idl
sequence<float64, 32> positions  # ✅ 最多 32 个，但 serialize 只传实际长度
```
需要 rosidl / 类型支持同时实现 BoundedSequence 的 POD 表示——并非所有 rmw 都支持。

---

## 7.6 标准消息的适配

**问题**：`std_msgs/Header`、`sensor_msgs/Imu`、`geometry_msgs/PoseStamped` 等标准消息全都含 `string frame_id` —— **都不能 Loan**。

**实践选择**：
- **选项 A：边界转换**——内部用自定义 Loan 友好消息，与标准工具（rviz / rosbag / 第三方包）交互时通过 bridge node 转成标准消息；
- **选项 B：提 PR**——把 `string` 改为 `string<=32`，但会破坏下游兼容性，上游接受度低；
- **选项 C：并行定义**——标准消息原样，自定义 `my_msgs/ImuFast` 用 `uint32 frame_id`（查表替代字符串）。

**具身智能项目通常走 C**：内部高频数据用自定义 Loan 消息，边界（用户交互）用标准消息。

---

## 7.7 Loaned Message 的内存管理

### 7.7.1 生命周期

```
borrow_loaned_message
   │
   │ rmw 向 DDS 申请一个 SHM chunk
   │ 返回 unique_ptr-like 句柄 (LoanedMessage<T>)
   ▼
[用户写数据]
   │
   ▼
publish(std::move(loaned))
   │
   │ DDS 接管句柄，标记为"已发布"
   │ chunk 进入 SHM ringbuffer，供订阅读取
   ▼
[订阅消费完后]
   │
   │ 订阅端 take_loaned 得到指针
   │ 使用后调用 return_loaned 归还
   ▼
[chunk 引用计数 = 0 → 回 pool]
```

**陷阱**：
- 借了但未发布（业务逻辑分支直接 `return`）：句柄析构时会自动 **return**，SHM chunk 归还；
- 订阅端忘记 `return_loaned`：SHM pool 泄漏，长时间运行后 `borrow` 返回失败；
- 订阅端保存指针到全局变量长期使用：发布端的 chunk 不能重用，造成 pool 耗尽——**SHM 下的"僵尸消息"问题**。

### 7.7.2 Pool 耗尽

```
[Error] Failed to borrow loaned message: no chunks available
```
**原因**：SHM pool 不够大，或消费端消费太慢（背压）。
**对策**：
- 增大 pool（`mempool.toml`）；
- 减小 `history.depth`（订阅保留更少历史）；
- 应用层加速消费 / 降采样。

---

## 7.8 Loan 与 IPC 的关系

**IPC** 和 **Loaned** 是两条正交的零拷贝路径：

| 场景 | 路径 | 典型延迟 |
|------|------|---------|
| 同进程、unique_ptr、相容 QoS | IPC | 1–5 μs |
| 跨进程、SHM 启用、Loan 友好消息 | Loaned over SHM | 3–10 μs |
| 跨进程、SHM 启用、非 Loan 消息 | SHM 传输但有拷贝 | 20–100 μs |
| 跨进程、仅 UDP | UDP 传输 + 拷贝 | 100–500 μs |

**最佳实践**：同进程用 IPC，跨进程用 Loaned+SHM，自动路由。

---

## 7.9 多订阅者下的 Loan

一个 publisher 发一条 Loan 消息给 3 个订阅：
- FastDDS + SHM：在 SHM ringbuffer 中维护引用计数，所有订阅消费完后 chunk 才归还；
- 如果 3 个订阅有跨进程有同进程——**IPC 订阅直接 move/share，SHM 订阅走 Loan**——这是 rmw + rclcpp 协作的透明分发；
- **所有订阅都必须有"能处理 Loaned"的能力**——即使上层是普通 `shared_ptr<const MsgT>` 回调，底层 take 路径知道如何从 SHM 读。

---

## 7.10 性能数据

单机 Intel i7-12700，FastDDS + SHM，1 MB Loan 友好消息：

| 路径 | P50 | P99 | CPU per msg |
|------|-----|-----|-------------|
| IPC（同进程） | 2 μs | 5 μs | < 1 μs |
| Loaned + SHM（跨进程） | 8 μs | 20 μs | ~3 μs |
| SHM 无 Loan | 80 μs | 200 μs | ~40 μs |
| UDP | 600 μs | 2 ms | ~100 μs |

小消息（64 B）差距较小（序列化不是瓶颈），但 Loaned 仍胜：

| 路径 | P50 |
|------|-----|
| IPC | 1 μs |
| Loaned + SHM | 3 μs |
| UDP | 80 μs |

---

## 7.11 调试技巧

- 开启 rclcpp 日志 DEBUG：会输出 "publishing via loaned" / "loaned fallback to copy"；
- `fastdds discovery` 查看 SHM transport 是否启用；
- `/dev/shm` 下有对应的 segment 文件（`fastrtps_*` 或 `iceoryx_*`）；
- 使用 `ros2 topic bw` 观察带宽——非 Loan 路径 CPU 占用显著高于 Loan 路径；
- `perf stat -e task-clock,cache-misses` 对比；Loan 的 cache-misses 显著低（因为跨进程但同 cache line）。

---

## 7.12 源码锚点

- `rclcpp::LoanedMessage<T>`（`rclcpp/include/rclcpp/loaned_message.hpp`）——RAII 包装；
- `rclcpp::Publisher<T>::borrow_loaned_message` / `publish(LoanedMessage)`；
- `rcl_borrow_loaned_message` / `rcl_publish_loaned_message`（`rcl/rcl/publisher.h`）；
- `rmw_borrow_loaned_message` / `rmw_publish_loaned_message`（`rmw/rmw/rmw.h`）；
- FastDDS：`DataWriter::loan_sample` / `write`；
- CycloneDDS：`dds_loan_sample` / `dds_write`。

---

## 7.13 小结

- Loaned Message 是跨进程零拷贝的**唯一通用方案**；
- 依赖三个条件同时满足：rmw 支持 + 消息类型 POD + SHM 启用；
- 消息类型设计是最大的前置工程——必须系统性改造 IDL；
- Loan 与 IPC 正交，rclcpp 内部透明地选择最优路径；
- 对大消息收益巨大（> 10×），小消息收益中等（2–3×）。

下一章：[08 RTPS 线协议](./08_rtps_wire_protocol.md)。
