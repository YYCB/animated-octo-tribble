# rcl（一）：架构总览与核心数据结构

## 一、rcl 在 ROS2 中的定位

```
┌────────────────────────────────────────────────────────────────┐
│                    用户代码                                      │
├────────────────────────────────────────────────────────────────┤
│  rclcpp（C++）  │  rclpy（Python）  │  rclnodejs（Node.js）    │
│                   语言客户端库                                   │
├────────────────────────────────────────────────────────────────┤
│                 rcl（本层）                                      │
│   C 语言，语言无关的 ROS 客户端库核心逻辑                         │
│   提供：节点、发布者、订阅者、服务、客户端、Timer、Wait           │
├────────────────────────────────────────────────────────────────┤
│                 rcl_yaml_param_parser                           │
│                 rcl_action（动作库）                             │
│                 rcl_lifecycle（生命周期）                         │
├────────────────────────────────────────────────────────────────┤
│                 rmw（中间件抽象接口）                             │
│   纯虚接口，隐藏 DDS 实现细节                                    │
├────────────────────────────────────────────────────────────────┤
│  rmw_fastrtps_cpp  │  rmw_cyclonedds_cpp  │  rmw_connextdds     │
│                   具体 DDS 实现                                  │
└────────────────────────────────────────────────────────────────┘
```

### 设计原则

1. **语言中立**：纯 C 实现（C99），所有语言绑定共享同一底层逻辑
2. **内存显式管理**：调用者管理内存，通过 `rcl_allocator_t` 注入分配器
3. **错误码返回**：所有函数返回 `rcl_ret_t`，无异常（C 不支持异常）
4. **零隐藏状态**：所有状态通过传入的句柄（handle）显式管理

---

## 二、rcl 源码目录结构

```
ros2/rcl/
├── rcl/
│   ├── include/rcl/
│   │   ├── rcl.h                  # 顶层头文件（汇总所有公共接口）
│   │   ├── context.h              # rcl_context_t：全局 ROS 上下文
│   │   ├── node.h                 # rcl_node_t：节点
│   │   ├── publisher.h            # rcl_publisher_t：发布者
│   │   ├── subscription.h         # rcl_subscription_t：订阅者
│   │   ├── service.h              # rcl_service_t：服务端
│   │   ├── client.h               # rcl_client_t：客户端
│   │   ├── timer.h                # rcl_timer_t：定时器
│   │   ├── wait.h                 # rcl_wait_set_t：等待集合
│   │   ├── guard_condition.h      # rcl_guard_condition_t：守护条件
│   │   ├── event.h                # rcl_event_t：QoS 事件
│   │   ├── graph.h                # 图查询（节点、topic、service 列表）
│   │   ├── time.h                 # rcl_time_t、时钟源
│   │   ├── arguments.h            # 命令行参数解析
│   │   ├── init_options.h         # rcl_init_options_t
│   │   └── error_handling.h       # 错误字符串、错误码
│   └── src/rcl/
│       ├── context.c              # ★★★★★
│       ├── node.c                 # ★★★★★
│       ├── publisher.c            # ★★★★★
│       ├── subscription.c         # ★★★★
│       ├── service.c              # ★★★
│       ├── client.c               # ★★★
│       ├── timer.c                # ★★★★
│       ├── wait.c                 # ★★★★★
│       ├── graph.c                # ★★★
│       └── init.c                 # ★★★★★
├── rcl_action/                    # Action 协议（基于 rcl）
│   ├── include/rcl_action/
│   │   ├── action_client.h
│   │   ├── action_server.h
│   │   └── goal_handle.h
│   └── src/rcl_action/
│       ├── action_client.c
│       └── action_server.c
└── rcl_yaml_param_parser/         # YAML 参数文件解析
    └── src/
        └── parser.c
```

---

## 三、核心数据结构详解

### 3.1 rcl_context_t（全局上下文）

```c
// rcl/include/rcl/context.h

typedef struct rcl_context_s
{
  // 全局参数（--ros-args 解析结果）
  rcl_arguments_t global_arguments;
  
  // 实现私有数据（指向 rcl_context_impl_t）
  struct rcl_context_impl_s * impl;
  
  // 实例 ID（每次 rcl_init() 生成唯一 ID，用于检测上下文重用）
  // 原子操作，线程安全
  rcl_context_instance_id_t instance_id_storage;
  
} rcl_context_t;
```

```c
// 内部实现（rcl/src/rcl/context.c）
typedef struct rcl_context_impl_s
{
  rcl_init_options_t init_options;           // 初始化选项（含 rmw 选项）
  
  // rmw 上下文（与具体 DDS 实现对接）
  rmw_context_t rmw_context;
  
  // 所有使用此 context 的节点（用于 shutdown 时清理）
  rcl_node_t ** nodes;
  size_t nodes_size;
  
  // 全局守护条件（用于 Ctrl+C 信号处理）
  rcl_guard_condition_t * interrupt_guard_condition;
  
  // 进程内通信管理（跨节点共享）
  rcl_intraprocess_manager_t * intraprocess_manager;
} rcl_context_impl_t;
```

### 3.2 rcl_node_t（节点句柄）

```c
// rcl/include/rcl/node.h

typedef struct rcl_node_s
{
  // 实现私有数据
  struct rcl_node_impl_s * impl;
} rcl_node_t;
```

```c
// 内部实现（rcl/src/rcl/node.c）
typedef struct rcl_node_impl_s
{
  rcl_node_options_t options;            // 节点选项（含参数覆盖、命名空间等）
  
  // rmw 节点句柄（与 DDS 参与者绑定）
  rmw_node_t * rmw_node_handle;
  
  // 图守护条件（图变化时触发，用于 wait_for_service 等）
  rcl_guard_condition_t * graph_guard_condition;
  
  // 解析后的完整名称（含命名空间）
  char * fq_name;               // fully qualified name: /namespace/name
  char * logger_name;           // 日志分类名
  
  // rosout 发布者（/rosout topic，用于日志收集）
  rcl_publisher_t * rosout_publisher;
  
  // 时钟（与节点关联，支持 sim time）
  rcl_clock_t clock;
} rcl_node_impl_t;
```

### 3.3 rcl_publisher_t（发布者句柄）

```c
// rcl/include/rcl/publisher.h

typedef struct rcl_publisher_s
{
  const rcl_node_t * impl_node;           // 所属节点（弱引用）
  struct rcl_publisher_impl_s * impl;     // 实现数据
} rcl_publisher_t;
```

```c
// 内部实现
typedef struct rcl_publisher_impl_s
{
  rcl_publisher_options_t options;        // QoS、分配器等选项
  
  // rmw 发布者（与 DDS DataWriter 对应）
  rmw_publisher_t * rmw_handle;
  
  // 消息类型支持（序列化/反序列化接口）
  const rosidl_message_type_support_t * type_support;
  
  // Topic 名称（解析后，含完整命名空间）
  char * expanded_topic_name;
  
  // 统计信息（可选，需要 RCL_PUBLISHER_STATISTICS_ENABLED）
  rcl_publisher_event_callbacks_t event_callbacks;
  
} rcl_publisher_impl_t;
```

### 3.4 rcl_subscription_t（订阅者句柄）

```c
typedef struct rcl_subscription_impl_s
{
  rcl_subscription_options_t options;
  rmw_subscription_t * rmw_handle;        // DDS DataReader 对应
  const rosidl_message_type_support_t * type_support;
  char * expanded_topic_name;
  
  // 内容过滤（Content Filtered Topic，DDS 扩展）
  rcl_subscription_content_filter_options_t content_filter_options;
} rcl_subscription_impl_t;
```

### 3.5 rcl_wait_set_t（等待集合）

```c
// rcl/include/rcl/wait.h

typedef struct rcl_wait_set_s
{
  // 各类型实体数组（等待就绪）
  const rcl_subscription_t ** subscriptions;
  const rcl_guard_condition_t ** guard_conditions;
  const rcl_timer_t ** timers;
  const rcl_client_t ** clients;
  const rcl_service_t ** services;
  const rcl_event_t ** events;
  
  // 各类型已分配数量
  size_t size_of_subscriptions;
  size_t size_of_guard_conditions;
  size_t size_of_timers;
  size_t size_of_clients;
  size_t size_of_services;
  size_t size_of_events;
  
  // 实现私有（rmw_wait_set 句柄、分配器等）
  struct rcl_wait_set_impl_s * impl;
} rcl_wait_set_t;
```

---

## 四、rcl_ret_t 错误码体系

```c
// rcl/include/rcl/types.h

typedef rmw_ret_t rcl_ret_t;

// 通用错误码（与 rmw 共享部分）
#define RCL_RET_OK                          0
#define RCL_RET_ERROR                       1
#define RCL_RET_TIMEOUT                     2
#define RCL_RET_BAD_ALLOC                   10
#define RCL_RET_INVALID_ARGUMENT            11
#define RCL_RET_UNSUPPORTED                 3

// rcl 专有错误码（从 100 开始）
#define RCL_RET_ALREADY_INIT                100
#define RCL_RET_NOT_INIT                    101
#define RCL_RET_MISMATCHED_RMW_ID          102
#define RCL_RET_NODE_INVALID                200
#define RCL_RET_NODE_INVALID_NAME          201
#define RCL_RET_NODE_INVALID_NAMESPACE     202
#define RCL_RET_NODE_NAME_NON_EXISTENT     203
#define RCL_RET_PUBLISHER_INVALID          300
#define RCL_RET_SUBSCRIPTION_INVALID       400
#define RCL_RET_SUBSCRIPTION_TAKE_FAILED   401
#define RCL_RET_CLIENT_INVALID             500
#define RCL_RET_CLIENT_TAKE_FAILED         501
#define RCL_RET_SERVICE_INVALID            600
#define RCL_RET_SERVICE_TAKE_FAILED        601
#define RCL_RET_TIMER_INVALID              800
#define RCL_RET_TIMER_CANCELED             801
#define RCL_RET_WAIT_SET_INVALID           900
#define RCL_RET_WAIT_SET_EMPTY             901
#define RCL_RET_WAIT_SET_FULL              902
#define RCL_RET_TOPIC_INVALID             1000

// 使用示例
rcl_ret_t ret = rcl_publisher_publish(&pub, msg, NULL);
if (RCL_RET_OK != ret) {
  RCUTILS_LOG_ERROR_NAMED("rcl", "Failed to publish: %s",
    rcl_get_error_string().str);
  rcl_reset_error();
}
```

---

## 五、rcl 内存管理模型

### 5.1 rcl_allocator_t

```c
// rcutils/include/rcutils/allocator.h

typedef struct rcutils_allocator_s
{
  // 分配函数（默认 malloc）
  void * (*allocate)(size_t size, void * state);
  
  // 释放函数（默认 free）
  void (* deallocate)(void * pointer, void * state);
  
  // 重分配函数（默认 realloc）
  void * (*reallocate)(void * pointer, size_t size, void * state);
  
  // 零初始化分配（默认 calloc）
  void * (*zero_allocate)(size_t number_of_elements, size_t size_of_element, void * state);
  
  // 分配器状态（自定义内存池时使用）
  void * state;
} rcutils_allocator_t;
```

### 5.2 内存管理规则

- 所有 `rcl_*_init()` 函数：**调用者提供已分配的句柄结构，rcl 填充内部数据**
- 所有 `rcl_*_fini()` 函数：**rcl 释放内部数据，调用者负责释放句柄本身**
- `rcl_get_zero_initialized_*()` 宏：提供安全的零初始化句柄

```c
// 正确的内存管理模式
rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();  // 栈分配
rcl_publisher_init(&publisher, &node, type_support, topic_name, &options);
// ... 使用 publisher ...
rcl_publisher_fini(&publisher, &node);  // 释放内部数据
// publisher 本身在栈上自动释放
```

---

## 六、关键宏定义

```c
// rcl/include/rcl/macros.h

// 检查并返回错误
#define RCL_CHECK_ARGUMENT_FOR_NULL(argument, error_return_type) \
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(argument, error_return_type)

// 检查节点是否有效
#define RCL_CHECK_FOR_NULL_WITH_MSG(value, msg, error_statement) \
  if (NULL == (value)) { \
    RCL_SET_ERROR_MSG(msg); \
    error_statement; \
  }

// 检查 rcl_ret_t 并传播错误
#define RCL_RET_FROM_RMW_RET(rmw_ret) \
  (rmw_ret == RMW_RET_OK ? RCL_RET_OK : \
   rmw_ret == RMW_RET_TIMEOUT ? RCL_RET_TIMEOUT : \
   RCL_RET_ERROR)
```

---

## 七、rcl 与 rclcpp 的关系

```
rclcpp::Publisher<T>::publish(msg)
       │
       ├─ 序列化消息（rosidl_typesupport）
       │
       └─► rcl_publish(&rcl_publisher_, serialized_msg, nullptr)
               │
               └─► rmw_publish(rmw_publisher_handle, msg, nullptr)
                       │
                       └─► FastDDS DataWriter::write(data)
```

rclcpp 是 rcl 的 **C++ 包装层**，主要职责：
1. RAII 资源管理（智能指针，确保 fini 被调用）
2. 模板化（泛型消息类型）
3. 回调注册（lambda、std::function）
4. Executor 调度集成
5. 进程内通信 IntraProcessManager 集成

---

## 八、参考源码位置

```
GitHub: ros2/rcl (rolling 分支)

关键文件：
  rcl/src/rcl/init.c           ← rcl_init() 实现
  rcl/src/rcl/context.c        ← 上下文管理
  rcl/src/rcl/node.c           ← 节点初始化
  rcl/src/rcl/publisher.c      ← 发布者实现
  rcl/src/rcl/subscription.c   ← 订阅者实现
  rcl/src/rcl/wait.c           ← 等待集合（核心循环基础）
  rcl/src/rcl/timer.c          ← 定时器实现
  rcl/src/rcl/graph.c          ← 图查询接口
```
