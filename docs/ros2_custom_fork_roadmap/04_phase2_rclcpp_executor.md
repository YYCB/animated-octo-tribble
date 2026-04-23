# 04 阶段 2：rclcpp 改造 —— Executor / 内存 / 回调组

> **目标**：在阶段 1 穷尽之后，进入 rclcpp 代码修改。本阶段可以**不 fork 整个 ros2.repos**——通过 "downstream patch package" 或 "shadow rclcpp" 的方式叠加。
>
> **决策门槛**：本阶段需要 **≥1 位熟悉 rclcpp 内部结构的工程师**（对 `Executor` / `Waitable` / `CallbackGroup` / `memory_strategy` / `AnyExecutable` 等概念熟悉）。

---

## 4.1 改造策略：Patch vs Fork vs Shadow

三种做法：

### A. **Downstream Patch Package**（推荐起点）
- 你的仓库里新建 `my_ros2_perf/` 包，**继承或组合**官方 rclcpp 类，覆盖热点方法；
- 不 patch 官方源码，只在应用层订阅时用你的派生类；
- 例：`MyEventsExecutor : public rclcpp::Executor` 重写 `spin()`；
- **维护成本最低**，但 rclcpp 私有 API 访问受限（许多关键点在 `detail::` 或 pimpl 里）。

### B. **Shadow rclcpp**（中度成本）
- 把官方 rclcpp 源码 vendor 一份进自己仓库（不建 git submodule，直接拷贝并打 tag）；
- 修改后编译输出 `librclcpp.so`，用 `LD_LIBRARY_PATH` 或 `CMAKE_PREFIX_PATH` 覆盖；
- **维护成本中**：每次上游更新要 cherry-pick / rebase；
- 优点：**保留原包名**，上层应用代码一行不改。

### C. **Full Fork**（高成本）
- Fork `ros2/rclcpp` 到自己 org；
- 改 `package.xml`、`CMakeLists.txt` 里的 package name（`my_rclcpp`）；
- 上层应用要改 `find_package(my_rclcpp)`；
- **彻底自主**，但每个升级周期都是大战役。

**建议路径**：A → B → C（只有在 B 维护成本变得不可接受时才走到 C）。

---

## 4.2 Executor 重写

### 4.2.1 官方 Executor 的热路径瓶颈（再次量化）

参见 [01 章 §2.1](./01_motivation_and_baseline.md#21-executor-调度开销)。关键结论：
- `wait_set_` 每次 `wait()` 返回都需要**重建**（rebuild_wait_set）；
- `get_next_ready_executable` 在 40 entity 下 5–20 μs；
- `AnyExecutable` 是值类型，每次 `std::move` 仍涉及多个 `shared_ptr` 原子 inc/dec。

### 4.2.2 EventsExecutor 原理

EventsExecutor 的突破是：**把"rcl 的就绪通知"从轮询改为回调**——rmw 底层有消息到达时，直接 push 一个事件到 Executor 的 lock-free 队列。Executor 只消费队列，不再扫描 entity 集合。

**关键接口**（概念）：
- `rmw_subscription_set_on_new_message_callback(sub, cb, user_data)` —— rmw 层新增的**回调注册 API**；
- Executor 为每个 subscription 注册这个回调，回调体就是 "把事件入队"；
- `spin()` 循环只做：从事件队列取一个 → 执行对应 entity 的回调。

**收益**：O(N) 扫描 → O(1) 出队；2000 Hz 小消息场景下 CPU 占用降 40%。

**工程陷阱**：
- 回调在 rmw **发现线程** / **DDS listener 线程** 里被调用——**不能做任何重活**，否则会阻塞 DDS；
- 事件队列满了怎么办？官方实现是动态扩容（`std::deque`），**会分配**；若要 alloc-free 必须改成 fixed-size ringbuffer；
- 单次 rmw 事件不代表消息数，需要在回调里取 `message_count` 增量，正确处理**丢事件**（listener 回调合并）。

### 4.2.3 应用特化 Executor

如果你的节点回调是**静态已知**的（控制环常见），可以写一个 **StaticScheduledExecutor**：
- 构造时枚举所有 entity，建一个固定顺序表；
- `spin_once()` 每次按表轮询 `rcl_trigger_guard_condition`-like 就绪位；
- 无任何动态分配、无虚函数派发、无 `shared_ptr` 原子操作；
- 延迟 < 500 ns，抖动 σ < 100 ns。

**代价**：只服务于你的特定节点，不是通用 Executor——但这正是具身智能控制节点所需要的。

### 4.2.4 推荐实施顺序

1. **先切到 EventsExecutor**（Iron 起已官方 available）→ 改一行 `main()`；
2. **若仍不足**，实现应用特化 Executor 给控制环节点；
3. **感知 / 规划等复杂节点仍用 EventsExecutor**，不一刀切。

---

## 4.3 内存策略（MemoryStrategy & Allocator）

### 4.3.1 rclcpp 的 Allocator 注入点

rclcpp 的关键类大都有 `Allocator` 模板参数：
- `Publisher<MsgT, Allocator>`
- `Subscription<MsgT, Allocator>`
- `Executor::MemoryStrategy`（分配 `AnyExecutable` / wait_set 存储）

**实践建议**：
- 用 **TLSF 分配器**（https://github.com/ros-realtime/tlsf_cpp）替换默认 `std::allocator`；
- 为每种高频消息类型创建专用 pool：
  ```cpp
  using MsgAlloc = tlsf_heap_allocator<ImuMsg>;
  auto pub = node->create_publisher<ImuMsg, MsgAlloc>(
    "imu", qos, pub_options);
  ```
- 回调中避免使用 `shared_ptr<const MsgT>`（走 shared 会多一次 control block alloc），改用 `unique_ptr<MsgT>`。

### 4.3.2 消息池（Loaned 之外的方案）

Loaned Message 需要 DDS 支持（阶段 3）。在那之前可以做**用户态消息池**：
- 启动时 `pool.reserve(1024)` 预分配；
- Publisher 从 pool 获取 `unique_ptr<MsgT>`（自定义 deleter 归还池）；
- 实现时注意 **ABA 问题**与 **多生产者/单消费者**或 **SPSC** 模型。

### 4.3.3 std::string / std::vector 陷阱

ROS IDL 生成的消息里的 `string` / `dynamic array` 默认是 `std::string` / `std::vector<T>`。每次 `operator=` 或 `emplace_back` 都可能重新分配。

**规避**：
- 消息定义里用 `string<=128`（BoundedString，编译为 `std::string` 但带容量提示）；
- 用 `sequence<T, 64>`（BoundedSequence，容量上界 64）；
- 在 `rosidl_generator_cpp` 中开启 "固定容量 POD" 生成（需要 rosidl 选项或自定义 generator）。

**彻底方案**：自研 rosidl generator 产出 C 结构体（无 std::string/vector），配合 rmw Loaned Message。工作量大，属于**阶段 3 / 5**的内容。

---

## 4.4 回调组与线程模型再设计

### 4.4.1 识别"脏"回调

"脏"回调 = 执行时间 > 控制周期 1/10 的回调。常见：
- tf2 查询（`lookupTransform` 带 timeout）
- 参数服务回调
- 服务处理（特别是返回大消息的）
- 日志 I/O（rosout 的 sync 写磁盘）

### 4.4.2 回调组拆分原则

```
CallbackGroup A (MutuallyExclusive, 高优先级线程)
  └── 500 Hz control timer callback
  └── joint_state subscription callback

CallbackGroup B (Reentrant, 中优先级线程池)
  └── camera subscription
  └── lidar subscription

CallbackGroup C (MutuallyExclusive, 低优先级线程)
  └── parameter service
  └── diagnostics publisher
  └── tf lookup service

CallbackGroup D (MutuallyExclusive, 异步线程)
  └── 慢服务（如 planning 调用）
```

### 4.4.3 MultiThreadedExecutor 的陷阱

`MultiThreadedExecutor` **所有回调组共享同一个 Executor**——意味着高优先级回调可能排在低优先级之后被 wait_mutex_ 阻塞。

**替代**：
- **多 Executor + 多线程**：每个优先级一个 `SingleThreadedExecutor`，分别 spin 在不同 RT 优先级的线程；
- **CallbackGroupExecutor**（社区方案）：每个回调组一个独立 Executor。

### 4.4.4 代码样例概念

```cpp
// 控制环 Executor（SCHED_FIFO 90）
auto exec_ctrl = std::make_shared<rclcpp::executors::EventsExecutor>();
exec_ctrl->add_callback_group(cb_group_A, node->get_node_base_interface());
std::thread t_ctrl([&]{
  set_thread_rt(90);
  pin_to_cpu(4);
  exec_ctrl->spin();
});

// 感知 Executor（SCHED_OTHER，nice -5）
auto exec_perc = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
exec_perc->add_callback_group(cb_group_B, node->get_node_base_interface());
std::thread t_perc([&]{
  set_thread_nice(-5);
  exec_perc->spin();
});

// ...
```

---

## 4.5 Publisher / Subscription 热点

### 4.5.1 `publish()` 的两个路径

源码上 `Publisher::publish()` 根据 IPC 启用与否走两条路：
- **IPC 路径**：`intra_process_manager_->do_intra_process_publish(...)`，直接插入 subscription 的队列，0 序列化；
- **DDS 路径**：`rcl_publish(msg_buf)` → `rmw_publish` → 序列化 + DDS write。

**优化点**：
1. `publish(std::unique_ptr<MsgT>)` 重载，move 进去——Manager 直接转移所有权；
2. 避免 `publish(const MsgT&)`——退化为拷贝；
3. 启用 IPC 后，发布不兼容 QoS 的订阅会**自动退化到 DDS 路径**（设计上的安全但性能上的陷阱），通过日志 `intra_process=false` 可观察。

### 4.5.2 Take 路径

订阅端从 rmw 取消息通常：
```
rcl_take → rmw_take_with_info → DDS DataReader.take() → 反序列化 → callback
```

**优化**：
- 使用 `take_loaned_message`（阶段 3）；
- 订阅回调用 `MsgT::SharedPtr` 会触发 ROS 层 shared_ptr 构造；改用 `MsgT::ConstSharedPtr` 无帮助（都一样）；**真正省拷贝**的是 `MsgT::UniquePtr`——配合 IPC 零拷贝。

---

## 4.6 Timer 精度

rclcpp Timer 实际调用 `rcl_timer_t`，依赖 Clock 类型：
- `RCL_SYSTEM_TIME` / `RCL_STEADY_TIME`：由内核时钟驱动，精度受 timer_slack 影响，通常 50–200 μs jitter；
- `RCL_ROS_TIME`：走 `/clock` 订阅，抖动更大；
- 自研方案：改用 `timerfd` + CLOCK_MONOTONIC，手动 `clock_nanosleep(TIMER_ABSTIME)`——μs 级精度。

**控制环的正确做法**：
```cpp
while (running) {
  next += period;
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
  do_control_step();
}
```
**不要用 rclcpp::Timer 驱动 500 Hz 控制环** —— Timer 精度不够，且要走 Executor 分派。控制环应该是一个 `std::thread` + `clock_nanosleep` + 直接调用 `publisher->publish()`。

---

## 4.7 节流 / 背压

高频发布 + 慢订阅者场景下：
- RELIABLE：发布端被阻塞在 `publish()`（可能死锁控制环！）；
- BEST_EFFORT：订阅端 DDS queue 满 → 丢消息；
- 正确方案：**显式降采样**（throttle）+ **显式背压**（ack 信号）。

rclcpp 没有内建 throttle。自行实现：
```cpp
if (now - last_pub < min_interval) return;
```

---

## 4.8 本阶段结束准则

预期收益（叠加阶段 1 之上）：

| 场景 | Phase1 后 | Phase2 后 | 收益 |
|------|----------|----------|------|
| A-IPC | 65 μs | 30 μs | 2.1× |
| C（500 Hz 控制） | σ=120 μs | σ=50 μs | 2.4× |
| Executor CPU 占用 | 基准 | 降 40% | — |
| 40 entity 节点调度 | 15 μs/loop | 2 μs/loop | 7.5× |

**下一阶段入门门槛**：本阶段做完后，如果还需要进一步降**大消息延迟**或**跨进程延迟**，进入 [05 阶段 3：rmw 零拷贝](./05_phase3_rmw_zero_copy.md)；如果主要问题是**同机多进程开销**，可以跳到 [06 阶段 4：SHM / Iceoryx](./06_phase4_dds_shm_iceoryx.md)。

---

## 4.9 本阶段维护成本

- 改动面：**自建 rclcpp patch 包 + 自定义 Executor 实现**；
- 与上游兼容性：**中**——rclcpp 私有 API 每个 ROS 2 发行版都可能变；
- 每次 ROS 2 版本升级：**1–2 周**（rebase + 重新验证）；
- 典型团队规模：**1.5–2 人**（1 人主维护 + 0.5 人 CI + 0.5 人 review）；
- 失控风险：若用 **Full Fork 方式**，半年内积累的差异可能让 rebase 成本指数增长——**务必坚持 Patch/Shadow 方式**，把 diff 控制在 ≤ 3000 行。
