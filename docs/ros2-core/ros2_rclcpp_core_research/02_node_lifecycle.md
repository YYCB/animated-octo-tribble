# rclcpp（二）：Node 构造与生命周期

## 一、Node 构造函数全流程

```cpp
// rclcpp/src/rclcpp/node.cpp

Node::Node(
  const std::string & node_name,
  const NodeOptions & options)
: Node(node_name, "", options)  // 委托给带命名空间的构造函数
{}

Node::Node(
  const std::string & node_name,
  const std::string & namespace_,
  const NodeOptions & options)
{
  // ── Step 1: 创建 NodeBase ─────────────────────────────
  // NodeBase 封装 rcl_node_t，是所有接口的基础
  node_base_ = std::make_shared<rclcpp::node_interfaces::NodeBase>(
    &node_name,
    &namespace_,
    options.context(),          // rclcpp::Context（封装 rcl_context）
    *(options.get_rcl_node_options()),  // rcl_node_options_t
    options.use_intra_process_comms(),
    options.enable_topic_statistics());
  // 内部调用：rcl_node_init()
  
  // ── Step 2: 创建 NodeGraph ────────────────────────────
  // 提供图查询接口（get_node_names, get_topic_names_and_types 等）
  node_graph_ = std::make_shared<rclcpp::node_interfaces::NodeGraph>(
    node_base_.get());
  
  // ── Step 3: 创建 NodeLogging ──────────────────────────
  // 提供 get_logger() 方法
  node_logging_ = std::make_shared<rclcpp::node_interfaces::NodeLogging>(
    node_base_.get());
  
  // ── Step 4: 创建 NodeTimers ───────────────────────────
  node_timers_ = std::make_shared<rclcpp::node_interfaces::NodeTimers>(
    node_base_.get());
  
  // ── Step 5: 创建 NodeTopics ───────────────────────────
  // 提供 create_publisher/create_subscription 的底层支持
  node_topics_ = std::make_shared<rclcpp::node_interfaces::NodeTopics>(
    node_base_.get(), node_timers_.get());
  
  // ── Step 6: 创建 NodeServices ─────────────────────────
  node_services_ = std::make_shared<rclcpp::node_interfaces::NodeServices>(
    node_base_.get());
  
  // ── Step 7: 创建 NodeWaitables ────────────────────────
  node_waitables_ = std::make_shared<rclcpp::node_interfaces::NodeWaitables>(
    node_base_.get());
  
  // ── Step 8: 创建 NodeClock ────────────────────────────
  // 初始化时钟（默认 RCL_ROS_TIME，支持 sim time）
  node_clock_ = std::make_shared<rclcpp::node_interfaces::NodeClock>(
    node_base_, node_topics_, node_graph_, node_services_, node_logging_);
  
  // ── Step 9: 创建 NodeParameters ───────────────────────
  // 初始化参数系统（创建参数服务、注册 declare_parameter 等）
  node_parameters_ = std::make_shared<rclcpp::node_interfaces::NodeParameters>(
    node_base_,
    node_logging_,
    node_topics_,
    node_services_,
    node_clock_,
    options.parameter_overrides(),
    options.start_parameter_services(),
    options.start_parameter_event_publisher(),
    options.parameter_event_qos(),
    options.parameter_event_publisher_options(),
    options.allow_undeclared_parameters(),
    options.automatically_declare_parameters_from_overrides());
  // 内部创建：
  //   /node_name/get_parameters service
  //   /node_name/set_parameters service
  //   /node_name/list_parameters service
  //   /node_name/describe_parameters service
  //   /node_name/parameter_events publisher
  
  // ── Step 10: 创建 NodeTypeDescriptions（Iron+）─────────
  // node_type_descriptions_ = ...
}
```

---

## 二、NodeBase 的构造（最重要的一步）

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_base.cpp

NodeBase::NodeBase(
  const std::string * node_name,
  const std::string * namespace_,
  rclcpp::Context::SharedPtr context,
  const rcl_node_options_t & rcl_node_options,
  bool use_intra_process_comms,
  bool enable_topic_statistics)
{
  // ── 1. 分配 rcl_node_t ───────────────────────────────
  node_handle_ = std::shared_ptr<rcl_node_t>(
    new rcl_node_t,
    // 自定义 deleter（确保 fini 被调用）
    [](rcl_node_t * node) {
      if (rcl_node_fini(node) != RCL_RET_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
          "Error in destructor while finalizing node: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete node;
    }
  );
  *node_handle_ = rcl_get_zero_initialized_node();
  
  // ── 2. 调用 rcl_node_init() ──────────────────────────
  rcl_ret_t ret = rcl_node_init(
    node_handle_.get(),
    node_name->c_str(),
    namespace_->c_str(),
    context->get_rcl_context().get(),
    &rcl_node_options
  );
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("Could not initialize node: " +
      std::string(rcl_get_error_string().str));
  }
  
  // ── 3. 获取图 guard condition ────────────────────────
  // 包装 rcl 层的 graph_guard_condition
  const rcl_guard_condition_t * gc =
    rcl_node_get_graph_guard_condition(node_handle_.get());
  graph_notify_condition_ = std::make_shared<rclcpp::GuardCondition>(
    context, rclcpp::GuardCondition::IntraProcessSetting::Skip);
  graph_notify_condition_->set_on_trigger_callback(
    [this](size_t) { notify_guard_condition_is_set_ = true; });
  
  // ── 4. 创建默认 CallbackGroup ────────────────────────
  default_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  
  // ── 5. 如果启用进程内通信，注册到 IntraProcessManager ─
  if (use_intra_process_comms) {
    context->get_sub_context<rclcpp::IntraProcessManager>();
    // IntraProcessManager 是 context 的子上下文，所有节点共享
  }
}
```

---

## 三、Node 析构流程

```cpp
// Node 析构时（按构造的逆序）：

~Node()
{
  // 析构各接口（shared_ptr 的引用计数降为 0 时触发）
  // 顺序：node_parameters_ → node_clock_ → node_waitables_
  //       → node_services_ → node_topics_ → node_timers_
  //       → node_logging_ → node_graph_ → node_base_
  
  // node_base_ 析构时：
  //   rcl_node_fini() → 通知 DDS 该节点离线
}
```

### shared_ptr 引用计数分析

```
Node 持有 node_base_ (shared_ptr, rc=1)
  │
  └── Publisher<T> 持有 node_base_ weak_ptr（不增加引用计数）
  └── Subscription<T> 持有 node_base_ weak_ptr

当 Node 被销毁：
  1. node_base_ 引用计数 → 0
  2. NodeBase 析构：rcl_node_fini()
  3. 任何仍存活的 Publisher（用户持有 shared_ptr）尝试访问 node_base_ 时：
     weak_ptr.lock() 返回 nullptr → 安全失败
```

---

## 四、create_publisher() 模板方法

```cpp
// rclcpp/include/rclcpp/node.hpp（模板方法，在头文件中实现）

template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
std::shared_ptr<PublisherT>
Node::create_publisher(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const PublisherOptionsWithAllocator<AllocatorT> & options =
    create_default_publisher_options<AllocatorT>())
{
  // 委托给 rclcpp::create_publisher<>() 自由函数（node_impl.hpp）
  return rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
    *this,          // 传入 Node 接口
    topic_name,
    qos,
    options);
}
```

### rclcpp::create_publisher 实现

```cpp
// rclcpp/include/rclcpp/create_publisher.hpp

template<typename MessageT, typename AllocatorT, typename PublisherT>
std::shared_ptr<PublisherT>
create_publisher(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const PublisherOptionsWithAllocator<AllocatorT> & options)
{
  // Step 1: 获取消息类型支持（编译期静态信息）
  auto message_type_support_handle =
    rclcpp::get_message_type_support_handle<MessageT>();
  
  // Step 2: 构建 rcl publisher 选项（含 QoS）
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  publisher_options.qos = rclcpp::adapt_qos_for_publishers(qos.get_rmw_qos_profile());
  
  // Step 3: 实例化 Publisher<T>（调用构造函数）
  auto pub = PublisherT::make_shared(
    node_topics->get_node_base_interface(),
    message_type_support_handle,
    topic_name,
    publisher_options,
    options.event_callbacks,
    options.use_default_callbacks,
    options.use_intra_process_comm
  );
  
  // Step 4: 注册到 NodeTopics（加入 CallbackGroup）
  node_topics->add_publisher(pub, options.callback_group);
  
  return pub;
}
```

---

## 五、Publisher<T> 构造函数

```cpp
// rclcpp/include/rclcpp/publisher.hpp

template<typename MessageT, typename AllocatorT>
Publisher<MessageT, AllocatorT>::Publisher(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const rosidl_message_type_support_t * type_support_handle,
  const std::string & topic_name,
  const rcl_publisher_options_t & publisher_options,
  ...)
: PublisherBase(...)  // 调用非模板基类构造
{
  // PublisherBase 构造：
  //   rcl_publisher_init()
  //   注册 QoS 事件回调
  //   如果 use_intra_process，向 IntraProcessManager 注册
  
  // 存储类型特定信息（序列化函数）
  message_allocator_ = std::make_shared<MessageAllocator>(...);
}
```

### PublisherBase 构造

```cpp
// rclcpp/src/rclcpp/publisher_base.cpp

PublisherBase::PublisherBase(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const std::string & topic,
  const rosidl_message_type_support_t & type_support_handle,
  const rcl_publisher_options_t & publisher_options,
  ...)
{
  // ★ 核心：调用 rcl_publisher_init()
  rcl_ret_t ret = rcl_publisher_init(
    publisher_handle_.get(),          // rcl_publisher_t
    node_base->get_rcl_node_handle(), // rcl_node_t
    &type_support_handle,
    topic.c_str(),
    &publisher_options
  );
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("could not create publisher: " +
      std::string(rcl_get_error_string().str));
  }
  
  // 注册 QoS 事件处理（如 deadline_missed, liveliness_lost 等）
  bind_event_callbacks(event_callbacks, use_default_callbacks);
}
```

---

## 六、create_subscription() 模板方法

```cpp
// 订阅者创建（类似 Publisher）
template<typename MessageT, typename CallbackT, ...>
typename rclcpp::Subscription<MessageT, AllocatorT>::SharedPtr
Node::create_subscription(
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const SubscriptionOptionsWithAllocator<AllocatorT> & options = ...)
{
  return rclcpp::create_subscription<MessageT>(
    *this, topic_name, qos, std::forward<CallbackT>(callback), options);
}
```

### 回调类型推导

```cpp
// rclcpp 在编译期推导回调签名：

// 用户写：
node->create_subscription<std_msgs::msg::String>("topic", 10,
  [](const std_msgs::msg::String & msg) { /* ... */ });

// 模板推导：
// CallbackT = lambda type
// 通过 rclcpp::function_traits<CallbackT>::argument_type<0>
// 判断用户想要 const_ref / shared_ptr / unique_ptr 等

// AnySubscriptionCallback 存储适配后的回调
// 在执行时（execute_subscription）调用正确的版本
```

---

## 七、Node::declare_parameter() 源码

```cpp
// rclcpp/include/rclcpp/node.hpp

template<typename ParameterT>
auto Node::declare_parameter(
  const std::string & name,
  const ParameterT & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor = {},
  bool ignore_override = false)
{
  return this->node_parameters_->declare_parameter(
    name,
    rclcpp::ParameterValue(default_value),  // 转换为 ParameterValue
    parameter_descriptor,
    ignore_override
  );
}
```

```cpp
// NodeParameters::declare_parameter() 内部：

// 1. 检查参数是否已声明（重复声明抛出 rclcpp::exceptions::ParameterAlreadyDeclaredException）
// 2. 检查是否有命令行覆盖值（--param name:=value）
// 3. 如果有覆盖值，用覆盖值代替默认值
// 4. 存储到 parameters_ 映射（std::map<std::string, ParameterValue>）
// 5. 发布到 /parameter_events topic（通知监听者参数变化）
```

---

## 八、Node 的线程安全性

```
rclcpp::Node 本身不是线程安全的！

规则：
  - 同一个 Node 的方法，不要从多个线程并发调用
  - 回调函数的并发由 Executor + CallbackGroup 控制
  - 可以在不同线程使用不同 Node（各自独立）

例外（线程安全的操作）：
  - rcl_trigger_guard_condition()（用于 shutdown、进程内通信）
  - rcl_clock_get_now()（加锁）
  - IntraProcessManager（内部使用 mutex）

安全的多线程模式：
  1. 每个线程一个 Executor + 一组 Node（常见模式）
  2. 单个 MultiThreadedExecutor 管理多个 Node（通过 CallbackGroup 保护共享状态）
```

---

## 九、Node 的移动与复制

```cpp
// Node 不可复制（=delete）
Node(const Node &) = delete;
Node & operator=(const Node &) = delete;

// Node 不可移动（资源绑定到 rcl_node_t 指针）
Node(Node &&) = delete;
Node & operator=(Node &&) = delete;

// 正确的传递方式：shared_ptr
auto node = std::make_shared<rclcpp::Node>("my_node");
auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
executor->add_node(node);  // 传入 shared_ptr（增加引用计数）
executor->spin();
```
