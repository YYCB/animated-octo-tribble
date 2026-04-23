# 06 IntraProcess Communication（IPC）深度剖析

> 本章补充 [02 发布路径](./02_publish_path.md) / [03 接收路径](./03_receive_path.md) 未覆盖的 IPC 快速路径。理解 IPC 是理解"为什么 ROS 2 同进程通信能做到 μs 级"的关键。

---

## 6.1 IPC 的存在动机

当发布者和订阅者在**同一进程**时，走"序列化 → DDS write → 内核 loopback → DDS read → 反序列化"是**纯浪费**：
- 序列化/反序列化：每 KB 消息 ~500 ns；
- DDS history cache：一次内存拷贝；
- 即使走 SHM 传输：仍有 ringbuffer 指针同步的跨 CPU 开销；
- 对比：直接把 `unique_ptr<MsgT>` 从 publisher 的栈移交给 subscription 的回调——**接近零开销**。

因此 ROS 2 设计了 `IntraProcessManager`：一个在 **Context 级别**（每进程一个）维护的"进程内广播总线"，绕过所有 rmw 层调用。

---

## 6.2 关键类与职责

```
rclcpp::experimental::IntraProcessManager
 │   单例 (per Context)
 │
 ├── publishers_   : unordered_map<uint64_t id, PublisherIntraProcessBase*>
 ├── subscriptions_: unordered_map<uint64_t id, SubscriptionIntraProcessBase*>
 │
 ├── add_publisher(pub, pub_options)
 ├── add_subscription(sub)
 ├── do_intra_process_publish(pub_id, MessageT*, allocator)
 ├── do_intra_process_publish_and_return_shared(pub_id, MessageT*, alloc)
 └── matches (pub_id → [sub_id, ...])  // 按 topic + QoS 兼容性匹配
```

**PublisherIntraProcessBase**：每个 publisher 对应一个（非多态的），在创建时向 Manager 注册。

**SubscriptionIntraProcessBase**：每个 subscription 的 IPC 端点；内含一个**有界队列 / 或环形缓冲**存储待消费的消息。

---

## 6.3 发布路径（IPC 启用时）

**代码路径**（简化）：

```cpp
// rclcpp::Publisher<MessageT>::publish
void publish(std::unique_ptr<MessageT> msg) {
  if (intra_process_is_enabled_) {
    // 分支 A: 有 IPC 订阅
    do_intra_process_publish(std::move(msg));
    // 同时也可能需要发布到 DDS（若有 inter-process 订阅）
  } else {
    // 分支 B: 只有 DDS 路径
    this->do_inter_process_publish(*msg);
  }
}
```

关键是 `do_intra_process_publish` 的实现——它通过 `IntraProcessManager::do_intra_process_publish`：

1. **查表**：根据 publisher id 找到所有匹配的 subscription；
2. **按订阅者类型分发**：
   - 若所有订阅都是 `const MsgT&` / `const SharedPtr&`：构造一个 `shared_ptr<const MsgT>`，分发给所有订阅（每个订阅端的队列收到同一个 shared_ptr）；
   - 若只有一个订阅且是 `unique_ptr<MsgT>`：直接 move 过去；
   - **混合场景**（既有 shared 又有 unique）：最后一个 unique 的订阅收到 move，其他收 shared；若有多个 unique 订阅——**必须 clone**（拷贝）。

3. **入队**：每个匹配 subscription 的队列 push 一条；
4. **触发 GuardCondition**：rcl 层的 `rcl_guard_condition_t`，让等待中的 Executor 的 `rcl_wait` 返回。

### 6.3.1 "最后一个 unique" 的优化

这是 IntraProcessManager 的精巧设计。假设有：
- Sub A: `unique_ptr<MsgT>` 回调
- Sub B: `const shared_ptr<const MsgT>&` 回调
- Sub C: `const shared_ptr<const MsgT>&` 回调

Manager 的分发策略：
- 先构造 `shared_ptr<const MsgT>` 给 B 和 C（同一个 shared_ptr，内容只读）；
- 给 A 的处理：因为 A 要独占，Manager 会 **clone** MsgT 一份（因为原消息被 shared 给 B/C，不能独占）。

**优化**：如果**只有一个 unique 订阅，没有 shared 订阅**，则直接 move，零拷贝。这也是为什么 "IPC 零拷贝" 的最佳实践是：**所有订阅都用 `const shared_ptr<const MsgT>&`**（shared 语义一致），除非明确只有一个 unique 订阅。

### 6.3.2 队列实现

每个 `SubscriptionIntraProcessBase` 持有一个固定容量（`qos.history.depth`）的队列：
- 默认实现：`std::deque`（动态分配）；
- 可定制：用户可传入 `SubscriptionOptions::template_intra_process_buffer` 选择 `RingBuffer` 实现；
- **RingBuffer 是性能友好的选择**——固定容量 + 无动态分配。

---

## 6.4 接收路径（IPC）

Executor 在 `wait()` 返回后，会遍历 `WaitSet`，检查哪些 subscription 的 IPC GuardCondition 被触发：

```cpp
// 简化
for (auto & sub : subscriptions_) {
  if (sub->is_intra_process_ready()) {
    auto msg = sub->take_intra_process();  // 从内部队列出队
    sub->invoke_callback(std::move(msg));
  }
}
```

**零拷贝点**：
- `take_intra_process` 直接从队列 `pop` 出 `unique_ptr` 或 `shared_ptr`；
- 回调接收该 `unique_ptr`/`shared_ptr`——**对象从未被拷贝**，只是所有权转移。

---

## 6.5 IPC 的六大前置条件

**工程上反复踩坑**——每条都有真实事故案例：

### 6.5.1 NodeOptions.use_intra_process_comms=true

默认是 `false`。每个 Node 独立设置。**跨 Node 要一致**——否则一方不开 IPC，整个链路降级。

### 6.5.2 发布端用 `unique_ptr`

`publish(std::unique_ptr<MsgT>)` 才能利用"last unique takes move"优化。
`publish(const MsgT&)` / `publish(shared_ptr<MsgT>)` 都会导致拷贝。

### 6.5.3 订阅端使用正确的回调签名

推荐签名（IPC 友好）：
- `void cb(std::unique_ptr<MsgT>)` —— 独占所有权；
- `void cb(std::shared_ptr<const MsgT>)` —— 只读共享；
- `void cb(const MsgT&)` —— 常引用（内部仍是 shared_ptr 存活期内）。

**不推荐**：
- `void cb(std::shared_ptr<MsgT>)` —— 非 const shared，多个订阅者拿到**同一个可变对象**时数据竞争；
- `void cb(MsgT)` —— 值传递，每次拷贝。

### 6.5.4 QoS 兼容

IPC 需要 QoS `history=KEEP_LAST`、`durability=VOLATILE`（或 TRANSIENT_LOCAL 两端一致）、reliability 两端兼容（publisher ≥ subscriber）。

**不兼容时**：`IntraProcessManager::can_communicate` 返回 false，该 pub-sub 对**退化为 DDS 路径**——日志级 INFO 会输出 "intra_process_comm disabled due to incompatible QoS"，**不开日志就不知道**。

### 6.5.5 Publisher/Subscription 属于同一进程

跨 Component Container 不行——典型陷阱是"我用了 component_container_mt，但每个 component 都在自己的进程"——那就还是 DDS。必须用**同一个** container 并 `ros2 component load` 到同一个进程。

### 6.5.6 消息类型一致

严格对应。父类/子类不行（C++ 没有运行时多态类型识别）。

---

## 6.6 IPC 与 DDS 的并存

当一个 publisher 同时有：
- IPC 订阅：进程内 3 个 subscription；
- inter-process 订阅：远程节点 5 个 subscription；

`publish()` 会做什么？
- 先走 IPC 路径（move/shared 给 3 个本地）；
- **同时** 走 DDS 路径（序列化 + rmw_publish，广播到 5 个远程）；
- 本地 3 个订阅**不会再次**从 DDS 收到（rmw 层会过滤自己的 participant）。

性能影响：**本地零拷贝 + 远程 DDS 拷贝**——两条路径并行，发布端总开销 ≈ max(IPC, DDS) ≈ DDS 开销。这意味着**启用 IPC 不会减慢混合场景**，但也不会使 DDS 部分变快——若要 DDS 部分也零拷贝，需进入阶段 3/4（SHM + Loaned）。

---

## 6.7 IPC 的局限与陷阱

### 6.7.1 多订阅者下的 const 传染

推荐所有订阅都用 `shared_ptr<const MsgT>` —— const 的设计是为了防止一个订阅修改消息影响其他订阅。**但如果业务逻辑要修改消息**（如填一个计算结果），必须 `const_cast` 或 clone——const_cast 未定义行为风险，clone 则放弃零拷贝。

### 6.7.2 消息生命周期

IPC 下一条消息的生命周期由**最后一个持有 shared_ptr** 的订阅决定。如果某个订阅把 shared_ptr 存入自己的队列长期不消费——**内存就不释放**。这对大消息（1 MB+）是**隐形内存泄漏**。

**诊断**：`heaptrack` 或 `valgrind --tool=massif` 观察长驻 allocation。

### 6.7.3 回调内重新发布

```cpp
void cb(std::unique_ptr<MsgT> msg) {
  msg->data = compute(*msg);
  other_pub->publish(std::move(msg));  // 重新发布
}
```

理论上零拷贝串联。但如果 `other_pub` 的订阅含 `shared_ptr` —— move 会失败，降级拷贝。

### 6.7.4 Executor 的重复唤醒

当 IPC publisher 和同进程 subscription 在不同 Executor / 不同线程下运行——IPC 的 GuardCondition 触发会 wakeup 两次（publisher 自己的 Executor + subscription 的 Executor）。这在超高频场景（> 1 kHz）下会引入额外的 CPU 占用，可通过合理的 Executor 布局避免。

---

## 6.8 源码引用点

核心位置（rclcpp 仓库，版本随发行漂移）：
- `rclcpp/include/rclcpp/experimental/intra_process_manager.hpp` —— Manager 的接口；
- `rclcpp/src/rclcpp/experimental/intra_process_manager.cpp` —— 分发逻辑、matcher；
- `rclcpp/include/rclcpp/experimental/subscription_intra_process.hpp` —— subscription IPC 端；
- `rclcpp/include/rclcpp/publisher.hpp` 的 `do_intra_process_publish` 方法；
- `rclcpp/include/rclcpp/experimental/buffers/` —— 队列实现（RingBuffer / Deque）。

**阅读建议**：从 `Publisher::publish(unique_ptr)` 开始跟——能把 IPC 分发的全路径走一遍。

---

## 6.9 本章小结

- IPC 是 ROS 2 同进程性能的核心机制，绕过 rmw/DDS；
- 零拷贝的前提是 6 个条件同时满足——任何一个不满足都悄悄降级；
- "shared_ptr 多订阅 + unique_ptr 单订阅"是最佳实践；
- IPC 与 DDS 并存不冲突，但不能互相替代；
- 长驻 shared_ptr 订阅是大消息场景的隐性内存坑。

相关章节：
- [05 phase3_rmw_zero_copy](../ros2_custom_fork_roadmap/05_phase3_rmw_zero_copy.md) - 跨进程零拷贝的 Loaned 方案；
- [07 loaned_messages.md](./07_loaned_messages.md) - Loaned Message 对比 IPC。
