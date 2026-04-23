# rcl（四）：Wait Set、rcl_wait() 与事件循环

## 一、Wait Set 的设计哲学

`rcl_wait_set_t` 是 ROS2 事件驱动架构的核心。它聚合所有需要监听的实体，通过一次 `rcl_wait()` 调用等待任意实体就绪，从而避免多线程轮询。

```
等待集合（rcl_wait_set）内包含：
  ┌─────────────────────────────────────────────────────┐
  │  subscriptions[N]   ← 订阅者（等待新消息）           │
  │  guard_conditions[M]← 守护条件（手动触发）           │
  │  timers[K]          ← 定时器（等待超时）             │
  │  clients[J]         ← 客户端（等待服务响应）         │
  │  services[L]        ← 服务端（等待请求）             │
  │  events[P]          ← QoS 事件（不兼容等）          │
  └─────────────────────────────────────────────────────┘
         │
         ▼ rcl_wait(wait_set, timeout)
  底层 select() / epoll() / WaitForMultipleObjects()
         │
         ▼ 返回后
  就绪的实体：wait_set.subscriptions[i] != NULL 表示第 i 个就绪
```

---

## 二、rcl_wait() 源码解析

```c
// rcl/src/rcl/wait.c

rcl_ret_t
rcl_wait(rcl_wait_set_t * wait_set, int64_t timeout)
{
  // ── Step 1: 验证 wait_set ────────────────────────────
  if (!rcl_wait_set_is_valid(wait_set)) {
    return RCL_RET_WAIT_SET_INVALID;
  }
  
  // ── Step 2: 计算定时器最小剩余时间 ───────────────────
  // 将 timeout 与所有定时器的剩余时间取 min
  // 确保 rcl_wait() 不会睡眠超过最近的定时器触发时间
  int64_t min_timeout = timeout;
  for (size_t i = 0; i < wait_set->size_of_timers; ++i) {
    const rcl_timer_t * timer = wait_set->impl->timers[i];
    if (!timer) continue;
    
    int64_t timer_timeout;
    rcl_timer_get_time_until_next_call(timer, &timer_timeout);
    
    if (timer_timeout <= 0) {
      // 定时器已超时，立即返回
      min_timeout = 0;
      break;
    }
    if (min_timeout < 0 || timer_timeout < min_timeout) {
      min_timeout = timer_timeout;
    }
  }
  
  // ── Step 3: ★ 调用 rmw_wait()（底层 OS 阻塞等待）────
  // 将 wait_set 的所有实体句柄传给 DDS 层
  // 超时后或有就绪实体时返回
  rmw_time_t wait_timeout = {
    (uint64_t)(min_timeout / 1000000000ULL),   // 秒
    (uint64_t)(min_timeout % 1000000000ULL)    // 纳秒
  };
  
  rmw_ret_t rmw_ret = rmw_wait(
    wait_set->impl->rmw_subscriptions,      // DDS DataReader 句柄列表
    wait_set->impl->rmw_guard_conditions,   // Guard Condition 句柄列表
    wait_set->impl->rmw_services,           // DDS Service 句柄列表
    wait_set->impl->rmw_clients,            // DDS Client 句柄列表
    wait_set->impl->rmw_events,             // QoS Event 句柄列表
    wait_set->impl->rmw_wait_set,           // 底层 rmw wait set
    min_timeout < 0 ? nullptr : &wait_timeout
  );
  
  // ── Step 4: 检查定时器就绪状态 ───────────────────────
  // DDS 层不知道 ROS2 定时器，需要在 rcl 层手动检查
  for (size_t i = 0; i < wait_set->size_of_timers; ++i) {
    const rcl_timer_t * timer = wait_set->impl->timers[i];
    if (!timer) continue;
    
    bool is_ready = false;
    rcl_timer_is_ready(timer, &is_ready);
    if (!is_ready) {
      // 置 NULL 表示此定时器未就绪
      wait_set->timers[i] = NULL;
    }
  }
  
  // ── Step 5: 处理 rmw_wait() 返回值 ───────────────────
  if (RMW_RET_TIMEOUT == rmw_ret) {
    // 超时（所有实体的指针已被 rmw_wait 置 NULL）
    // 定时器可能就绪（Step 4 已处理）
    return RCL_RET_TIMEOUT;
  }
  
  return rcl_convert_rmw_ret_to_rcl_ret(rmw_ret);
}
```

### rmw_wait() 的工作原理（FastDDS 实现）

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_wait.cpp

rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  CustomWaitSet * custom_wait_set = static_cast<CustomWaitSet *>(wait_set->data);
  
  // 阻塞等待（使用条件变量 + mutex）
  std::unique_lock<std::mutex> lock(custom_wait_set->condition_mutex);
  
  auto predicate = [&]() {
    // 检查是否有任何就绪实体
    return has_ready_entities(subscriptions, guard_conditions, services, clients, events);
  };
  
  if (wait_timeout) {
    auto timeout_ns = std::chrono::nanoseconds(
      wait_timeout->sec * 1000000000ULL + wait_timeout->nsec);
    custom_wait_set->condition.wait_for(lock, timeout_ns, predicate);
  } else {
    custom_wait_set->condition.wait(lock, predicate);
  }
  
  // 将未就绪实体的指针置 NULL（调用者通过检查 NULL 判断就绪状态）
  clean_up_not_ready_entities(subscriptions, guard_conditions, ...);
  
  return RMW_RET_OK;
}
```

---

## 三、Guard Condition：手动触发的等待实体

Guard Condition 是一种可以从任意线程手动触发的等待实体，用于实现：
- SIGINT 处理（shutdown 时唤醒所有 wait set）
- 图变化通知（新节点/topic 出现时唤醒）
- 进程内通信（IntraProcess 消息到达时唤醒）

```c
// rcl/include/rcl/guard_condition.h

typedef struct rcl_guard_condition_s {
  struct rcl_guard_condition_impl_s * impl;
} rcl_guard_condition_t;

// 触发 guard condition（任何线程均可调用）
rcl_ret_t
rcl_trigger_guard_condition(rcl_guard_condition_t * guard_condition)
{
  return rmw_trigger_guard_condition(guard_condition->impl->rmw_handle);
}
```

### Guard Condition 内部（FastDDS）

```cpp
// 基于 DDS WaitSet + GuardCondition 实现

// 触发（生产者线程）：
rmw_trigger_guard_condition(guard_condition)
  → condition->set_trigger_value(true)   // 原子操作
  → condition->notify_all()              // 唤醒所有 wait_for() 的线程

// 等待（消费者线程）：
rmw_wait(..., guard_conditions, ...)
  → WaitSet::wait(timeout)               // 阻塞
  → 被唤醒 → 检查 condition->get_trigger_value()
  → 置回 false，将 condition 指针保留（表示就绪）
```

---

## 四、完整的 Executor 等待循环实现（rcl 层视角）

```c
// 等效于 rclcpp::SingleThreadedExecutor::spin() 的 C 语言版本

rcl_context_t context;
rcl_init(argc, argv, &init_options, &context);

rcl_node_t node;
rcl_node_init(&node, "my_node", "", &context, &node_options);

rcl_subscription_t sub;
rcl_subscription_init(&sub, &node, ts, "chatter", &sub_opts);

rcl_timer_t timer;
rcl_timer_init2(&timer, &clock, &context, RCL_S_TO_NS(1), timer_callback, allocator, true);

// 等待集合初始化
rcl_wait_set_t ws = rcl_get_zero_initialized_wait_set();
rcl_wait_set_init(&ws,
  1,  // subscriptions
  1,  // guard_conditions（interrupt guard condition）
  1,  // timers
  0, 0, 0,
  &context, allocator);

// 添加 interrupt guard condition（shutdown 用）
rcl_wait_set_add_guard_condition(&ws, 
  rcl_context_get_interrupt_guard_condition(&context), NULL);

// 主循环
while (rcl_context_is_valid(&context)) {
  // 重置 wait_set（清除上次就绪状态）
  rcl_wait_set_clear(&ws);
  
  // 重新添加订阅者和定时器
  rcl_wait_set_add_subscription(&ws, &sub, NULL);
  rcl_wait_set_add_timer(&ws, &timer, NULL);
  rcl_wait_set_add_guard_condition(&ws,
    rcl_context_get_interrupt_guard_condition(&context), NULL);
  
  // ★ 阻塞等待（-1 表示永久等待）
  rcl_ret_t ret = rcl_wait(&ws, -1);
  
  if (ret == RCL_RET_TIMEOUT) continue;
  
  // 检查各实体是否就绪（非 NULL = 就绪）
  for (size_t i = 0; i < ws.size_of_subscriptions; ++i) {
    if (ws.subscriptions[i]) {
      // 接收消息
      MyMsg msg;
      rmw_message_info_t info;
      if (RCL_RET_OK == rcl_take(&sub, &msg, &info, NULL)) {
        subscription_callback(&msg);
      }
    }
  }
  
  for (size_t i = 0; i < ws.size_of_timers; ++i) {
    if (ws.timers[i]) {
      // 触发定时器
      rcl_timer_call(&timer);
    }
  }
  
  // 检查 interrupt guard condition（shutdown 信号）
  for (size_t i = 0; i < ws.size_of_guard_conditions; ++i) {
    if (ws.guard_conditions[i]) {
      // shutdown 信号触发
      break;
    }
  }
}

// 清理
rcl_wait_set_fini(&ws);
rcl_timer_fini(&timer);
rcl_subscription_fini(&sub, &node);
rcl_node_fini(&node);
rcl_shutdown(&context);
```

---

## 五、rcl_wait_set_t 内存管理

### 5.1 初始化与调整大小

```c
// 初始化（预分配固定容量）
rcl_wait_set_init(
  &wait_set,
  num_subscriptions,     // 固定容量，不支持动态扩展
  num_guard_conditions,
  num_timers,
  num_clients,
  num_services,
  num_events,
  context,
  allocator);

// 每次循环前必须清空（将所有指针置 NULL，重置就绪状态）
rcl_wait_set_clear(&wait_set);

// 重新添加实体（每次循环都需要重新添加！）
rcl_wait_set_add_subscription(&wait_set, &sub, NULL);
```

### 5.2 内部 rmw_wait_set 结构

```c
// rcl_wait_set_impl_t 内部保存 rmw 层的等待集合
typedef struct rcl_wait_set_impl_s
{
  // rmw 层句柄（与具体 DDS wait set 对应）
  rmw_wait_set_t * rmw_wait_set;
  
  // rmw 层实体句柄数组（与 rcl 层实体一一对应）
  rmw_subscriptions_t  rmw_subscriptions;
  rmw_guard_conditions_t rmw_guard_conditions;
  rmw_services_t       rmw_services;
  rmw_clients_t        rmw_clients;
  rmw_events_t         rmw_events;
  
  rcl_allocator_t allocator;
} rcl_wait_set_impl_t;
```

---

## 六、静态 Executor 优化（Iron+ 新特性）

`StaticSingleThreadedExecutor` 避免了每次循环都重建 wait_set 的开销：

```c
// 普通 Executor（每次循环重建）：
// rcl_wait_set_clear() → rcl_wait_set_add_*() × N → rcl_wait()
// 时间复杂度：O(N) per loop iteration（N = 实体数量）

// 静态 Executor（只在拓扑变化时重建）：
// 初始化时：构建一次 wait_set
// 运行时：直接调用 rcl_wait()，不重建
// 时间复杂度：O(1) per loop iteration（无拓扑变化时）

// 相关源码：
// rclcpp/src/rclcpp/executors/static_single_threaded_executor.cpp
// → StaticExecutorEntitiesCollector：跟踪拓扑变化，按需重建 wait_set
```

---

## 七、rcl_clock_t 与仿真时间

```c
// rcl/include/rcl/time.h

// 时钟类型
typedef enum rcl_clock_type_e {
  RCL_CLOCK_UNINITIALIZED = 0,
  RCL_ROS_TIME,      // 使用 /clock topic（仿真时间，来自 gazebo 等）
  RCL_SYSTEM_TIME,   // 系统时钟（gettimeofday/clock_gettime）
  RCL_STEADY_TIME,   // 单调时钟（不受 NTP 调整影响）
} rcl_clock_type_t;

// 获取当前时间
rcl_ret_t
rcl_clock_get_now(rcl_clock_t * clock, rcl_time_point_value_t * time_point_value)
{
  switch (clock->type) {
    case RCL_SYSTEM_TIME:
      return rcl_get_system_time(time_point_value);
    case RCL_STEADY_TIME:
      return rcl_get_steady_time(time_point_value);
    case RCL_ROS_TIME:
      // 先检查 /clock topic 是否有效（ROS_TIME_IS_ACTIVE）
      if (clock->impl->ros_time_is_active) {
        *time_point_value = clock->impl->current_time;
        return RCL_RET_OK;
      }
      // 降级到系统时钟
      return rcl_get_system_time(time_point_value);
  }
}
```

### 仿真时间与定时器的交互

```c
// 当 use_sim_time=true 时：
// 1. rclcpp::Node 创建 /clock 订阅者
// 2. 收到 /clock 消息时更新 rcl_clock_t 内部时间
// 3. 触发时钟跳转回调（rcl_clock_jump_callback）
// 4. 定时器检查：当前仿真时间 >= last_call_time + period → 触发

// 关键：仿真时间可以暂停（不发布 /clock）或加速（快速发布 /clock）
// rcl_wait() 中的 timeout 仍基于实际时间
// 但定时器的"就绪"判断基于仿真时间
```

---

## 八、rcl 层关键函数速查表

| 函数 | 作用 | 关键参数 |
|------|------|---------|
| `rcl_init()` | 初始化 ROS2 运行时 | argc, argv, options, context |
| `rcl_shutdown()` | 关闭 ROS2 运行时 | context |
| `rcl_ok()` | 检查 context 是否有效 | context |
| `rcl_node_init()` | 创建节点 | node, name, ns, context |
| `rcl_publisher_init()` | 创建发布者 | publisher, node, type_support, topic, options |
| `rcl_publish()` | 发布消息（C 结构体） | publisher, ros_message, allocation |
| `rcl_subscription_init()` | 创建订阅者 | subscription, node, type_support, topic, options |
| `rcl_take()` | 接收消息 | subscription, ros_message, message_info |
| `rcl_service_init()` | 创建服务端 | service, node, type_support, service_name |
| `rcl_client_init()` | 创建客户端 | client, node, type_support, service_name |
| `rcl_timer_init2()` | 创建定时器 | timer, clock, context, period, callback |
| `rcl_wait_set_init()` | 初始化等待集合 | wait_set, 各类型容量, context |
| `rcl_wait()` | 阻塞等待就绪实体 | wait_set, timeout（纳秒，-1=永久） |
| `rcl_timer_call()` | 执行定时器回调 | timer |
| `rcl_trigger_guard_condition()` | 手动触发 guard condition | guard_condition |
