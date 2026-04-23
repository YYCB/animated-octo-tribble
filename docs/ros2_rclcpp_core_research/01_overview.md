# rclcpp（一）：架构总览与类层次

## 一、rclcpp 在 ROS2 中的位置

```
                用户代码
                   │
                   ▼
┌──────────────────────────────────────────────────┐
│  rclcpp（本层）                                   │
│                                                  │
│  Node  Publisher<T>  Subscription<T>             │
│  Timer  Service<T>  Client<T>                    │
│  Executor  CallbackGroup  IntraProcessManager    │
│  LifecycleNode（rclcpp_lifecycle）               │
└──────────────────────────┬───────────────────────┘
                           │ 调用
┌──────────────────────────▼───────────────────────┐
│  rcl（C 语言客户端库）                            │
│  rcl_node_t  rcl_publisher_t  rcl_wait_set_t     │
└──────────────────────────┬───────────────────────┘
                           │ 调用
                          rmw → DDS
```

---

## 二、rclcpp 源码目录结构

```
ros2/rclcpp/
├── rclcpp/
│   ├── include/rclcpp/
│   │   ├── rclcpp.hpp                         # 顶层 include（汇总所有头文件）
│   │   ├── node.hpp                           # Node 类（最重要）
│   │   ├── node_impl.hpp                      # Node 模板方法实现
│   │   │
│   │   ├── node_interfaces/                   # ★ 接口分离设计
│   │   │   ├── node_base_interface.hpp        # 基础接口（名称、GUID、context）
│   │   │   ├── node_topics_interface.hpp      # Topic 相关接口
│   │   │   ├── node_services_interface.hpp    # Service 相关接口
│   │   │   ├── node_parameters_interface.hpp  # 参数相关接口
│   │   │   ├── node_timers_interface.hpp      # 定时器接口
│   │   │   ├── node_waitables_interface.hpp   # Waitable 接口
│   │   │   ├── node_clock_interface.hpp       # 时钟接口
│   │   │   ├── node_graph_interface.hpp       # 图查询接口
│   │   │   ├── node_logging_interface.hpp     # 日志接口
│   │   │   └── node_type_descriptions_interface.hpp # 类型描述接口（Iron+）
│   │   │
│   │   ├── publisher.hpp                      # Publisher<T> 模板类
│   │   ├── publisher_base.hpp                 # Publisher 非模板基类
│   │   ├── subscription.hpp                   # Subscription<T> 模板类
│   │   ├── subscription_base.hpp              # Subscription 非模板基类
│   │   ├── timer.hpp                          # TimerBase / WallTimer / GenericTimer
│   │   ├── service.hpp                        # Service<T> 模板类
│   │   ├── client.hpp                         # Client<T> 模板类
│   │   │
│   │   ├── executor.hpp                       # Executor 基类
│   │   ├── executors/
│   │   │   ├── single_threaded_executor.hpp
│   │   │   ├── multi_threaded_executor.hpp
│   │   │   └── static_single_threaded_executor.hpp
│   │   │
│   │   ├── callback_group.hpp                 # CallbackGroup
│   │   ├── any_executable.hpp                 # AnyExecutable
│   │   ├── any_subscription_callback.hpp      # 订阅回调类型封装
│   │   │
│   │   ├── intra_process_manager.hpp          # 进程内通信管理
│   │   ├── qos.hpp                            # QoS 包装类
│   │   ├── qos_event.hpp                      # QoS 事件
│   │   ├── context.hpp                        # rclcpp::Context（封装 rcl_context）
│   │   ├── node_options.hpp                   # NodeOptions
│   │   └── type_support_decl.hpp              # 类型支持
│   │
│   └── src/rclcpp/
│       ├── node.cpp                           # ★★★★★
│       ├── publisher_base.cpp                 # ★★★★
│       ├── subscription_base.cpp              # ★★★★
│       ├── executor.cpp                       # ★★★★★
│       ├── intra_process_manager.cpp          # ★★★★★
│       └── ...
│
├── rclcpp_components/                         # Component 支持
├── rclcpp_action/                             # Action 客户端/服务端
└── rclcpp_lifecycle/                          # Lifecycle Node
```

---

## 三、Node 的接口分离设计

### 3.1 为什么要接口分离？

ROS2 的 `Node` 类非常庞大（100+ 个方法），如果全部放在一个类中会造成：
- 编译依赖爆炸（修改任何部分都需重编译所有依赖）
- 难以测试（必须创建完整 Node 才能测试单个功能）
- 难以扩展（如 LifecycleNode 需要重写很多内容）

**解决方案**：将 Node 的功能切分为多个接口，通过组合实现。

### 3.2 Node 类的组合结构

```cpp
// rclcpp/include/rclcpp/node.hpp（精简）

class Node : public NodeBaseInterface,
             public NodeClockInterface,
             public NodeGraphInterface,
             public NodeLoggingInterface,
             public NodeParametersInterface,
             public NodeServicesInterface,
             public NodeTimersInterface,
             public NodeTopicsInterface,
             public NodeWaitablesInterface,
             public NodeTypeDescriptionsInterface  // Iron+
{
public:
  explicit Node(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions());

  // ── 发布者 ──────────────────────────────────────
  template<typename MessageT, typename AllocatorT = std::allocator<void>>
  typename rclcpp::Publisher<MessageT, AllocatorT>::SharedPtr
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const PublisherOptionsWithAllocator<AllocatorT> & options = ...);

  // ── 订阅者 ──────────────────────────────────────
  template<typename MessageT, typename CallbackT, ...>
  typename rclcpp::Subscription<MessageT, AllocatorT>::SharedPtr
  create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const SubscriptionOptionsWithAllocator<AllocatorT> & options = ...);

  // ── 定时器 ──────────────────────────────────────
  template<typename DurationRepT, typename DurationT, typename CallbackT>
  typename rclcpp::WallTimer<CallbackT>::SharedPtr
  create_wall_timer(
    std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  // ── 服务 ────────────────────────────────────────
  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    CallbackT && callback,
    const rclcpp::QoS & qos = ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  // ── 客户端 ──────────────────────────────────────
  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    const rclcpp::QoS & qos = ServicesQoS(),
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

private:
  // 各接口的具体实现（通过 Pimpl 模式）
  std::shared_ptr<rclcpp::node_interfaces::NodeBase> node_base_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTopics> node_topics_;
  std::shared_ptr<rclcpp::node_interfaces::NodeParameters> node_parameters_;
  std::shared_ptr<rclcpp::node_interfaces::NodeServices> node_services_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTimers> node_timers_;
  std::shared_ptr<rclcpp::node_interfaces::NodeWaitables> node_waitables_;
  std::shared_ptr<rclcpp::node_interfaces::NodeClock> node_clock_;
  std::shared_ptr<rclcpp::node_interfaces::NodeGraph> node_graph_;
  std::shared_ptr<rclcpp::node_interfaces::NodeLogging> node_logging_;
};
```

### 3.3 NodeBase 的内容

```cpp
// rclcpp/include/rclcpp/node_interfaces/node_base_interface.hpp

class NodeBaseInterface
{
public:
  // 节点名（最终名，已应用重映射）
  virtual const char * get_name() const = 0;
  virtual const char * get_namespace() const = 0;
  virtual const char * get_fully_qualified_name() const = 0;
  
  // rcl 句柄
  virtual rcl_node_t * get_rcl_node_handle() = 0;
  
  // Context
  virtual rclcpp::Context::SharedPtr get_context() = 0;
  
  // 默认 CallbackGroup（用于未指定 group 的实体）
  virtual rclcpp::CallbackGroup::SharedPtr get_default_callback_group() = 0;
  
  // 所有 CallbackGroup 迭代（Executor 用于构建 wait_set）
  virtual void for_each_callback_group(
    const CallbackGroupFunction & func) = 0;
  
  // 全局唯一 ID（DDS GUID 对应）
  virtual const rcl_guard_condition_t *
  get_notify_guard_condition() = 0;
  
  // 触发图变化通知（当节点创建/销毁发布者等时调用）
  virtual void trigger_notify_guard_condition() = 0;
};
```

---

## 四、NodeOptions：节点创建选项

```cpp
// rclcpp/include/rclcpp/node_options.hpp

class NodeOptions
{
public:
  // ── 上下文 ──────────────────────────────────────
  NodeOptions & context(rclcpp::Context::SharedPtr context);
  
  // ── 参数 ────────────────────────────────────────
  // 初始参数值（程序化方式，不依赖命令行）
  NodeOptions & parameter_overrides(
    const std::vector<rclcpp::Parameter> & parameter_overrides);
  
  // ── 进程内通信 ───────────────────────────────────
  // 是否启用零拷贝进程内通信
  NodeOptions & use_intra_process_comms(bool use_intra_process_comms);
  
  // ── 参数服务 ─────────────────────────────────────
  // 是否启动参数服务（可禁用以减少 topic 数量）
  NodeOptions & start_parameter_services(bool start_parameter_services);
  NodeOptions & start_parameter_event_publisher(bool value);
  
  // ── 日志 ────────────────────────────────────────
  NodeOptions & enable_rosout(bool enable_rosout);
  
  // ── 底层 rcl 选项 ────────────────────────────────
  const rcl_node_options_t * get_rcl_node_options() const;
  
  // ── 命令行参数 ───────────────────────────────────
  // 允许在 NodeOptions 中传入命令行参数（覆盖全局参数）
  NodeOptions & arguments(const std::vector<std::string> & arguments);
};
```

---

## 五、rclcpp 关键设计模式

### 5.1 类型擦除模式（TypeErased）

rclcpp 使用两层结构解决模板与虚函数不兼容的问题：

```
Publisher<T>（模板类，类型安全）
  ├── 继承自 PublisherBase（非模板，虚函数接口）
  └── 存储 TypeSupport<T>（序列化/反序列化实现）

Executor 持有 PublisherBase::SharedPtr（类型擦除）
在需要发布时通过虚函数调用，然后向下转型使用 TypeSupport

这样 Executor 无需知道具体消息类型
```

### 5.2 RAII + shared_ptr 所有权模型

```cpp
// Node 持有所有实体的 weak_ptr（防止循环引用）
// 实体（Publisher/Subscription 等）持有 shared_ptr

// 节点存储 CallbackGroup 的 weak_ptr
std::vector<rclcpp::CallbackGroup::WeakPtr> callback_groups_;

// 实体存储 CallbackGroup 的 shared_ptr
rclcpp::CallbackGroup::SharedPtr callback_group_;

// 当用户丢弃 Publisher 的 shared_ptr 后：
// 1. Publisher 析构，调用 rcl_publisher_fini()
// 2. 从 Node 的 topic 列表中移除
// 3. rcl 层通知 rmw 层，DDS DataWriter 被销毁
// 4. SEDP 广播：该 DataWriter 不再可用
```

### 5.3 AnySubscriptionCallback：回调类型封装

```cpp
// rclcpp/include/rclcpp/any_subscription_callback.hpp

// 订阅回调支持多种签名：
using SubscriptionCallbackTypeCode = ...;

// 签名 1：只接受消息（最常用）
void callback(const MessageT & msg);
void callback(MessageT msg);

// 签名 2：接受消息 + 元数据
void callback(const MessageT & msg, const rclcpp::MessageInfo & info);

// 签名 3：shared_ptr（避免拷贝）
void callback(std::shared_ptr<const MessageT> msg);
void callback(std::shared_ptr<MessageT> msg);  // 可修改

// 签名 4：序列化消息（绕过反序列化）
void callback(std::shared_ptr<const rclcpp::SerializedMessage> msg);

// AnySubscriptionCallback 通过 std::variant 存储所有可能的回调类型
template<typename MessageT, typename AllocatorT>
class AnySubscriptionCallback {
  using SharedPtrCallback = std::function<void (std::shared_ptr<const MessageT>)>;
  using ConstRefCallback  = std::function<void (const MessageT &)>;
  using MoveCallback      = std::function<void (MessageT)>;
  // ... 其他类型
  
  std::variant<
    ConstRefCallback,
    SharedPtrCallback,
    MoveCallback,
    // ...
  > callback_;
  
  // dispatch：根据存储的类型调用正确的回调
  void dispatch(std::shared_ptr<MessageT> message, const rclcpp::MessageInfo & info);
};
```

---

## 六、rclcpp 与 C++17/20 特性使用

| 特性 | rclcpp 用途 |
|------|------------|
| `std::variant` | `AnySubscriptionCallback`、`AnyExecutable` |
| `std::optional` | 参数值、可选回调参数 |
| `if constexpr` | 编译期消息类型分支（序列化 vs 零拷贝） |
| `std::shared_ptr` | 几乎所有 ROS 对象的生命周期管理 |
| `std::weak_ptr` | 防止循环引用（Node → CallbackGroup） |
| `std::unique_ptr` | 进程内通信零拷贝消息传递 |
| `std::function` | 回调类型擦除 |
| `Concepts`（C++20） | `MessageT` 约束（Iron+ 部分引入） |
| `std::atomic` | `spinning` 标志、instance ID |

---

## 七、rclcpp 版本演进关键节点

| 版本 | 关键变化 |
|------|---------|
| Dashing（2019） | 基础 API，Node、Publisher、Subscription |
| Foxy（2020） | `create_wall_timer` 改为 duration 参数；QoS 全面重构 |
| Galactic（2021） | `StaticSingleThreadedExecutor`；`TypeAdapter` 引入 |
| Humble（2022 LTS） | `SubscriptionTopicStatistics`；`NodeLoansInterface` |
| Iron（2023） | `NodeTypeDescriptionsInterface`；EventsExecutor 实验性 |
| Jazzy（2024 LTS） | Executor 重构；`GenericPublisher`/`GenericSubscription` 稳定 |
| Rolling | 持续迭代 |

---

## 八、源码阅读关键路径

```
rclcpp 核心（按重要性）：
  1. rclcpp/src/rclcpp/node.cpp              # Node 构造函数（理解节点初始化）
  2. rclcpp/src/rclcpp/executor.cpp          # Executor::spin() 核心
  3. rclcpp/src/rclcpp/intra_process_manager.cpp  # 进程内通信
  4. rclcpp/include/rclcpp/publisher.hpp     # Publisher<T> 实现
  5. rclcpp/include/rclcpp/subscription.hpp  # Subscription<T> 实现
  
GitHub 链接：
  https://github.com/ros2/rclcpp/tree/rolling/rclcpp/src/rclcpp
  https://github.com/ros2/rclcpp/tree/rolling/rclcpp/include/rclcpp
```
