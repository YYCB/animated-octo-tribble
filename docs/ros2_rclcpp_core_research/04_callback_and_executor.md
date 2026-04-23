# rclcpp（四）：回调系统与 Executor 完整实现

## 一、Executor 基类架构

```cpp
// rclcpp/include/rclcpp/executor.hpp（精简）

class Executor
{
public:
  // ── 节点管理 ──────────────────────────────────────────
  // 将节点（或其 NodeBase 接口）添加到 executor
  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);
  void add_node(rclcpp::Node::SharedPtr node);
  
  // ── 执行控制 ──────────────────────────────────────────
  // 纯虚：子类实现具体的 spin 策略
  virtual void spin() = 0;
  
  // spin_once：执行一个就绪回调，timeout 内没有就绪则返回
  void spin_once(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  
  // spin_until_future_complete：等待 future 完成
  template<typename FutureT, typename TimeRepT, typename TimeT>
  FutureReturnCode spin_until_future_complete(
    FutureT & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1));
  
  // spin_some：执行所有当前就绪回调（不阻塞等待新事件）
  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0));

protected:
  // ── 核心方法 ──────────────────────────────────────────
  bool get_next_executable(
    AnyExecutable & any_executable,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));
  
  bool get_next_ready_executable(AnyExecutable & any_executable);
  
  void execute_any_executable(AnyExecutable & any_exec);
  void execute_subscription(rclcpp::SubscriptionBase::SharedPtr sub);
  void execute_timer(rclcpp::TimerBase::SharedPtr timer);
  void execute_service(rclcpp::ServiceBase::SharedPtr service);
  void execute_client(rclcpp::ClientBase::SharedPtr client);
  void execute_waitable(rclcpp::Waitable::SharedPtr waitable, ...);

  // wait_set 管理
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();
  
  // 所有节点（弱引用，不阻止节点销毁）
  std::vector<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;
  
  // 原子标志：当前是否在 spin 中
  std::atomic_bool spinning{false};
  
  // context（用于检查 rclcpp::ok()）
  rclcpp::Context::SharedPtr context_;
};
```

---

## 二、get_next_executable() 详细流程

```cpp
// rclcpp/src/rclcpp/executor.cpp

bool Executor::get_next_executable(
  AnyExecutable & any_executable,
  std::chrono::nanoseconds timeout)
{
  // Step 1: 尝试立即获取就绪实体（非阻塞）
  bool success = get_next_ready_executable(any_executable);
  if (success) return true;
  
  // Step 2: 没有就绪实体，需要重建 wait_set 并等待
  
  // 收集所有节点的实体（订阅者、定时器、服务等）
  collect_entities();
  
  // 重置 wait_set
  rcl_wait_set_clear(&wait_set_);
  
  // 将所有实体添加到 wait_set
  for (auto & subscription : current_collection_.subscriptions) {
    rcl_wait_set_add_subscription(&wait_set_, subscription->get_subscription_handle().get(), nullptr);
  }
  for (auto & timer : current_collection_.timers) {
    rcl_wait_set_add_timer(&wait_set_, timer->get_timer_handle().get(), nullptr);
  }
  for (auto & service : current_collection_.services) {
    rcl_wait_set_add_service(&wait_set_, service->get_service_handle().get(), nullptr);
  }
  // ...（client, guard_condition, event, waitable 类似）
  
  // Step 3: ★ 阻塞等待
  rcl_ret_t status = rcl_wait(&wait_set_, timeout.count());
  
  // Step 4: 再次尝试获取就绪实体
  return get_next_ready_executable(any_executable);
}
```

---

## 三、get_next_ready_executable() 就绪实体选取

```cpp
bool Executor::get_next_ready_executable(AnyExecutable & any_executable)
{
  // 优先级顺序：Subscription > Timer > Service > Client > Waitable
  // （注意：在同一优先级内按 round-robin 方式选取，避免饥饿）
  
  // 检查 Timer
  for (auto & [key, timer_ref] : current_collection_.timers) {
    auto timer = timer_ref.lock();
    if (!timer) continue;
    
    // wait_set 中对应位置非 NULL 表示就绪
    if (wait_set_.timers[timer_index] != nullptr) {
      // 检查 CallbackGroup 是否允许（并发控制）
      auto group = timer->get_callback_group();
      if (group->can_be_taken_from().exchange(false)) {
        any_executable.timer = timer;
        any_executable.callback_group = group;
        return true;
      }
    }
  }
  
  // 检查 Subscription
  for (auto & [key, sub_ref] : current_collection_.subscriptions) {
    auto subscription = sub_ref.lock();
    if (!subscription) continue;
    
    if (wait_set_.subscriptions[sub_index] != nullptr) {
      auto group = subscription->get_callback_group();
      if (group->can_be_taken_from().exchange(false)) {
        any_executable.subscription = subscription;
        any_executable.callback_group = group;
        return true;
      }
    }
  }
  
  // 检查 Service、Client、Waitable...（类似）
  return false;
}
```

---

## 四、CallbackGroup 并发控制实现

```cpp
// rclcpp/include/rclcpp/callback_group.hpp

class CallbackGroup
{
public:
  // 类型：互斥 or 可重入
  CallbackGroupType type_;
  
  // ★ 核心：原子标志，控制是否可以从此组取出回调执行
  // - MutuallyExclusive：同一时刻只有一个回调在执行（取出后设为 false，执行完后设回 true）
  // - Reentrant：始终为 true（可以并发执行多个回调）
  std::atomic<bool> can_be_taken_from_{true};
  
  // 所属此组的实体（Subscription/Timer/Service/Client）
  std::vector<std::weak_ptr<rclcpp::SubscriptionBase>> subscription_ptrs_;
  std::vector<std::weak_ptr<rclcpp::TimerBase>> timer_ptrs_;
  std::vector<std::weak_ptr<rclcpp::ServiceBase>> service_ptrs_;
  std::vector<std::weak_ptr<rclcpp::ClientBase>> client_ptrs_;
  std::vector<std::weak_ptr<rclcpp::Waitable>> waitable_ptrs_;
};
```

### 互斥 vs 可重入的区别

```
MutuallyExclusive（互斥）：
  - can_be_taken_from 初始为 true
  - 取出一个回调执行时：可以取出（true）→ 执行中（false）→ 执行完（true）
  - 同一时刻最多 1 个回调在执行
  - 适合：有共享状态、需要串行保护的回调
  
Reentrant（可重入）：
  - can_be_taken_from 始终为 true
  - 多个线程可以同时从此组取出回调执行
  - 适合：无状态的计算回调（每次调用独立）

// 执行完后恢复 can_be_taken_from（由 Executor 在执行完回调后恢复）
auto group = any_executable.callback_group;
execute_any_executable(any_executable);  // 执行回调
group->trigger_notify_guard_condition(); // 通知可以再次取出
if (group->type() == CallbackGroupType::MutuallyExclusive) {
  group->can_be_taken_from().store(true);
}
```

---

## 五、MultiThreadedExecutor 并发模型

```cpp
// rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp

void MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false));
  
  // 创建 number_of_threads_ - 1 个额外工作线程（主线程算 1 个）
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  for (; thread_id < number_of_threads_ - 1; ++thread_id) {
    auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
    threads.emplace_back(func);
  }
  
  // 主线程也参与工作
  run(thread_id);
  
  for (auto & thread : threads) {
    thread.join();
  }
}

void MultiThreadedExecutor::run(size_t thread_id)
{
  (void)thread_id;
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      // ★ 获取锁（只有一个线程在 wait 中，其他线程等待）
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      if (!get_next_executable(any_exec, std::chrono::nanoseconds(0))) {
        // 没有立即就绪的任务，等待
        std::unique_lock<std::mutex> lk(wait_mutex_);
        // 释放锁，等待下一个 get_next_executable() 成功
        wait_condition_.wait(lk, [this]() { return !spinning.load() || ...; });
        continue;
      }
    }
    // 释放 wait_mutex 后再执行（允许其他线程进入等待）
    execute_any_executable(any_exec);
  }
}
```

### 多线程数据竞争分析

```
问题：多个线程同时调用 get_next_ready_executable()，
      可能取到同一个就绪的 subscription

解决：wait_mutex_ 保护整个 get_next_executable() 调用
     并且 CallbackGroup::can_be_taken_from_.exchange(false) 是原子操作
     → 同一 group 的回调不会被两个线程同时取出执行
```

---

## 六、AnyExecutable 数据结构

```cpp
// rclcpp/include/rclcpp/any_executable.hpp

struct AnyExecutable
{
  // 以下字段互斥，同一时刻只有一个非空
  rclcpp::SubscriptionBase::SharedPtr subscription;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::ServiceBase::SharedPtr service;
  rclcpp::ClientBase::SharedPtr client;
  rclcpp::Waitable::SharedPtr waitable;
  std::shared_ptr<void> data;  // waitable 的额外数据
  
  // 关联信息
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  rclcpp::CallbackGroup::SharedPtr callback_group;
};
```

---

## 七、execute_any_executable() 分发

```cpp
// rclcpp/src/rclcpp/executor.cpp

void Executor::execute_any_executable(AnyExecutable & any_exec)
{
  if (!any_exec.subscription && !any_exec.timer && ...) return;
  
  if (any_exec.timer) {
    TRACEPOINT(rclcpp_executor_execute, static_cast<const void *>(any_exec.timer.get()));
    execute_timer(any_exec.timer);
  }
  if (any_exec.subscription) {
    TRACEPOINT(rclcpp_executor_execute, ...);
    execute_subscription(any_exec.subscription);
  }
  if (any_exec.service) {
    execute_service(any_exec.service);
  }
  if (any_exec.client) {
    execute_client(any_exec.client);
  }
  if (any_exec.waitable) {
    any_exec.waitable->execute(any_exec.data);
  }
  
  // ★ 执行完后，通知 CallbackGroup 可以再次取出
  // 对于 MutuallyExclusive：can_be_taken_from = true
  if (any_exec.callback_group) {
    any_exec.callback_group->trigger_notify_guard_condition();
    any_exec.callback_group->can_be_taken_from().store(true);
  }
}
```

---

## 八、spin_until_future_complete() 实现

```cpp
// 最常用的异步等待模式
auto future = client->async_send_request(request);
executor.spin_until_future_complete(future);

// 内部实现：
template<typename FutureT>
FutureReturnCode
Executor::spin_until_future_complete(FutureT & future, ...)
{
  // 循环：继续 spin，直到 future 完成或超时
  while (rclcpp::ok(context_) && spinning.load()) {
    // 检查 future 是否完成（非阻塞）
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      return FutureReturnCode::SUCCESS;
    }
    
    // 检查超时
    if (timeout > 0 && elapsed >= timeout) {
      return FutureReturnCode::TIMEOUT;
    }
    
    // 执行一次事件循环（处理响应回调，触发 future 完成）
    spin_once_impl(remaining_time);
  }
  
  return FutureReturnCode::INTERRUPTED;
}
```

---

## 九、Waitable 接口（自定义可等待实体）

`Waitable` 允许自定义实体参与 Executor 的等待-执行循环，Action Server/Client 就是通过 Waitable 实现的。

```cpp
// rclcpp/include/rclcpp/waitable.hpp

class Waitable
{
public:
  // 向 wait_set 添加自身（可能添加多个实体）
  virtual void add_to_wait_set(rcl_wait_set_t * wait_set) = 0;
  
  // 检查是否就绪
  virtual bool is_ready(rcl_wait_set_t * wait_set) = 0;
  
  // 获取执行数据（在持有 wait_set 锁时调用，此时可以 take 消息）
  virtual std::shared_ptr<void> take_data() = 0;
  
  // 执行回调（在锁外调用，可以安全执行耗时操作）
  virtual void execute(std::shared_ptr<void> & data) = 0;
};
```

### Action 如何使用 Waitable

```cpp
// rclcpp_action/src/server.cpp

class ServerGoalHandleImpl : public rclcpp::Waitable
{
  // 内部包含 3 个 Service + 2 个 Publisher
  // add_to_wait_set：将 3 个 service 添加到 wait_set
  // is_ready：检查任意 service 是否收到请求
  // take_data：从就绪的 service 取出请求
  // execute：调用用户的 GoalCallback/CancelCallback/ResultCallback
};
```

---

## 十、Executor 性能关键指标

```
SingleThreadedExecutor 瓶颈：
  - 所有回调串行执行
  - 回调执行时，新消息在 DDS 队列中积压
  - 适用：回调执行时间短（< 1ms），节点数少

MultiThreadedExecutor 注意事项：
  - 默认线程数 = CPU 核数（std::thread::hardware_concurrency()）
  - 每个线程持有 wait_mutex_，竞争可能成为瓶颈
  - 共享状态必须通过 MutuallyExclusive CallbackGroup 保护

StaticSingleThreadedExecutor（Iron+ 推荐）：
  - 首次 spin 时构建 wait_set，之后不重建（除非拓扑变化）
  - 适合：固定拓扑的生产系统，节点/发布者不动态添加/删除
  - 性能比 SingleThreadedExecutor 提升约 10-30%（基准测试）

EventsExecutor（实验性）：
  - 基于事件驱动，无需轮询 wait_set
  - 使用 epoll/kqueue 直接监听 DDS 套接字
  - 延迟可降低 50%+（早期基准测试）
```
