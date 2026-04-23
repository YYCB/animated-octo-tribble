# ROS2 Component（三）：Executor、CallbackGroup 与调度模型

## 一、Executor 类层次

```
rclcpp::Executor  (抽象基类)
├── SingleThreadedExecutor       # 单线程，顺序执行
├── MultiThreadedExecutor        # 多线程，并发执行
├── StaticSingleThreadedExecutor # 静态拓扑优化（Iron+）
└── EventsExecutor               # 基于事件驱动（Rolling，实验性）
```

### 源码位置

```
rclcpp/include/rclcpp/executors/
├── single_threaded_executor.hpp
├── multi_threaded_executor.hpp
└── static_single_threaded_executor.hpp

rclcpp/src/rclcpp/executors/
├── single_threaded_executor.cpp
├── multi_threaded_executor.cpp
└── static_single_threaded_executor.cpp
```

---

## 二、SingleThreadedExecutor 核心 spin 逻辑

```cpp
// rclcpp/src/rclcpp/executors/single_threaded_executor.cpp

void SingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false));

  while (rclcpp::ok(this->context_) && spinning.load()) {
    // 等待并取出一个可执行的任务（timer/subscription/service/client）
    rclcpp::AnyExecutable any_exec;
    if (get_next_executable(any_exec, std::chrono::nanoseconds(-1))) {
      // 执行回调
      execute_any_executable(any_exec);
    }
  }
}
```

### get_next_executable() 内部逻辑

```cpp
bool Executor::get_next_executable(
  AnyExecutable & any_executable,
  std::chrono::nanoseconds timeout)
{
  // 1. 收集所有已就绪的实体（使用 rcl_wait_set）
  collect_entities();
  
  // 2. 调用 rcl_wait() 等待任意实体就绪（底层 select/epoll/WaitForMultipleObjects）
  auto rcl_wait_ret = rcl_wait(&wait_set_, rcl_ns_to_steady_time(timeout.count()));
  
  // 3. 从就绪集合中取出一个（优先级：timer > subscription > service > client > waitable）
  return get_next_ready_executable(any_executable);
}
```

---

## 三、MultiThreadedExecutor 并发模型

```cpp
// rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp

void MultiThreadedExecutor::spin()
{
  // 创建 number_of_threads_ 个工作线程
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  
  {
    std::lock_guard<std::mutex> wait_lock(wait_mutex_);
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
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
  while (rclcpp::ok(context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      if (!get_next_executable(any_exec, std::chrono::nanoseconds(0))) {
        // 没有就绪任务，等待
        std::unique_lock<std::mutex> lock(wait_mutex_);
        wait_condition_.wait(lock);
        continue;
      }
    }
    execute_any_executable(any_exec);
  }
}
```

---

## 四、CallbackGroup 并发控制

CallbackGroup 是 ROS2 中控制回调并发安全的机制：

```cpp
// 两种类型
enum class CallbackGroupType {
  MutuallyExclusive,   // 互斥：同一组的回调不会并发执行
  Reentrant,           // 可重入：同一组的回调可以并发执行
};
```

### 在 Component 中使用 CallbackGroup

```cpp
class MyComponent : public rclcpp::Node
{
public:
  explicit MyComponent(const rclcpp::NodeOptions & options)
  : Node("my_component", options)
  {
    // 创建一个互斥回调组（用于需要串行保护的回调）
    cb_group_mutex_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // 创建一个可重入回调组（用于无状态的计算回调）
    cb_group_reentrant_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    
    // 将订阅绑定到特定回调组
    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = cb_group_mutex_;
    
    sub_ = create_subscription<std_msgs::msg::String>(
      "input", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { process(msg); },
      sub_opts);
    
    // 服务绑定到可重入组（无状态查询）
    auto srv_opts = rclcpp::ServicesQoS();
    service_ = create_service<std_srvs::srv::Trigger>(
      "query",
      [this](auto req, auto res) { handle_query(req, res); },
      srv_opts,
      cb_group_reentrant_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_mutex_;
  rclcpp::CallbackGroup::SharedPtr cb_group_reentrant_;
};
```

---

## 五、Executor 与 ComponentManager 的集成流程

```
┌─────────────────────────────────────────────────────────┐
│  component_container (main)                              │
│                                                          │
│  auto exec = make_shared<SingleThreadedExecutor>();      │
│  auto mgr  = make_shared<ComponentManager>(exec);        │
│                                                          │
│  exec->add_node(mgr);   ← mgr 本身作为节点加入 executor  │
│  exec->spin();           ← 开始事件循环                   │
└──────────────────────┬──────────────────────────────────┘
                       │ ros2 component load ...
                       ↓ LoadNode 服务被触发
┌─────────────────────────────────────────────────────────┐
│  ComponentManager::on_load_node()                        │
│                                                          │
│  auto factory = create_component_factory(resource);      │
│  auto wrapper = factory->create_node_instance(options);  │
│                                                          │
│  exec->add_node(wrapper.get_node_base_interface());      │
│  ← 新组件节点加入同一 executor！                         │
└─────────────────────────────────────────────────────────┘
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
  
  // 所属节点和回调组（用于并发控制决策）
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  rclcpp::CallbackGroup::SharedPtr callback_group;
};
```

---

## 七、Component 调试与监控

### 7.1 查看已加载组件

```bash
# 列出容器内所有组件
ros2 component list

# 或通过服务
ros2 service call /ComponentManager/list_nodes \
  composition_interfaces/srv/ListNodes "{}"
```

### 7.2 动态加载与卸载

```bash
# 加载（指定命名空间和重映射）
ros2 component load /ComponentManager \
  demo_nodes_cpp demo_nodes_cpp::Talker \
  --node-name my_talker \
  --node-namespace /my_ns \
  --remap /chatter:=/my_chatter

# 卸载（需要唯一 ID，从 list 命令获取）
ros2 component unload /ComponentManager 1
```

### 7.3 进程内通信验证（源码级）

```cpp
// rclcpp/src/rclcpp/publisher_base.cpp
// 发布时的路径选择：

void PublisherBase::publish_intra_process(...)
{
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    // 进程内通信管理器不可用（未启用），走 DDS 路径
    return;
  }
  // 调用进程内路径
  ipm->do_intra_process_publish(...);
}
```

---

## 八、版本演进对比

| 版本 | 特性变化 |
|------|---------|
| Dashing (2019) | Component 机制引入，基础 ComponentManager |
| Foxy (2020) | 进程内通信优化，支持 NodeOptions |
| Galactic (2021) | StaticSingleThreadedExecutor 优化大型系统性能 |
| Humble (2022, LTS) | ComponentManagerIsolated 可选隔离模式 |
| Iron (2023) | EventsExecutor 实验性引入 |
| Jazzy (2024, LTS) | 进一步稳定 Events-based Executor |
| Rolling | 持续迭代中 |

---

## 九、性能数据参考

根据 ROS2 设计文档和基准测试：

- **进程内通信 vs DDS 通信**：延迟降低约 10x（取决于消息大小）
- **零拷贝阈值**：消息 > 64KB 时，DDS 零拷贝（iceoryx/shared memory）性能优于进程内 `unique_ptr` 传递
- **多组件 single-threaded executor**：所有组件共享一个线程，回调串行化，总延迟增加但无锁竞争
- **multi-threaded executor**：线程数建议 = CPU 核数（默认），但须通过 CallbackGroup 保护共享状态

---

## 十、源码阅读推荐路径（进阶）

```
1. Executor 基类：
   rclcpp/src/rclcpp/executor.cpp
   → get_next_executable()
   → execute_any_executable()

2. 进程内通信完整流程：
   rclcpp/src/rclcpp/intra_process_manager.cpp
   → do_intra_process_publish()
   → do_intra_process_publish_and_return_shared()

3. wait_set 与 rcl_wait 联动：
   rclcpp/src/rclcpp/wait_set_policies/
   rcl/src/rcl/wait.c   （底层 C 实现）

4. 回调组并发决策：
   rclcpp/src/rclcpp/executor_entities_collection.cpp（Iron+）
   rclcpp/src/rclcpp/executor.cpp → can_be_executed_now()
```
