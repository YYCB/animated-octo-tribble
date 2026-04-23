# rcl（三）：Node、Publisher、Subscription 源码详解

## 一、rcl_node_init() 完整流程

```c
// rcl/src/rcl/node.c

rcl_ret_t
rcl_node_init(
  rcl_node_t * node,
  const char * name,
  const char * namespace_,
  rcl_context_t * context,
  const rcl_node_options_t * options)
{
  // ── Step 1: 参数检查 ──────────────────────────────────
  RCL_CHECK_ARGUMENT_FOR_NULL(node, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(name, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(namespace_, RCL_RET_INVALID_ARGUMENT);
  if (!rcl_context_is_valid(context)) {
    return RCL_RET_NOT_INIT;
  }
  
  // ── Step 2: 验证节点名合法性 ─────────────────────────
  // 规则：只允许字母、数字、下划线；不能以数字开头；不能为空
  if (!rcl_node_name_is_valid(name)) {
    RCL_SET_ERROR_MSG("node name is invalid");
    return RCL_RET_NODE_INVALID_NAME;
  }
  
  // ── Step 3: 解析名称和命名空间 ───────────────────────
  // 应用重映射规则（--remap __node:=custom_name）
  char * remapped_node_name = NULL;
  ret = rcl_remap_node_name(
    &options->arguments,       // 节点本地参数
    &context->global_arguments, // 全局参数
    name,                       // 原始名称
    allocator,
    &remapped_node_name);       // 输出重映射后的名称
  
  char * remapped_namespace = NULL;
  ret = rcl_remap_node_namespace(
    &options->arguments,
    &context->global_arguments,
    namespace_,
    allocator,
    &remapped_namespace);
  
  // ── Step 4: 组合完整限定名 ───────────────────────────
  // fq_name = /namespace/name（或 /name 若 namespace 为空）
  char * fq_name = NULL;
  ret = rcl_node_get_fully_qualified_name_helper(
    remapped_node_name, remapped_namespace, &fq_name, allocator);
  
  // ── Step 5: 分配并填充 impl ──────────────────────────
  node->impl = allocator.allocate(sizeof(rcl_node_impl_t), allocator.state);
  memset(node->impl, 0, sizeof(rcl_node_impl_t));
  
  // ── Step 6: ★ 通过 rmw 创建节点（最终创建/注册到 DDS）─
  node->impl->rmw_node_handle = rmw_create_node(
    &context->impl->rmw_context,
    remapped_node_name,
    remapped_namespace
  );
  if (!node->impl->rmw_node_handle) {
    RCL_SET_ERROR_MSG("failed to create rmw node");
    goto fail;
  }
  
  // ── Step 7: 获取图 guard condition ───────────────────
  // 当网络图发生变化时（节点加入/离开，topic 出现/消失），此条件被触发
  node->impl->graph_guard_condition = allocator.allocate(
    sizeof(rcl_guard_condition_t), allocator.state);
  
  const rmw_guard_condition_t * rmw_graph_gc =
    rmw_node_get_graph_guard_condition(node->impl->rmw_node_handle);
  ret = rcl_guard_condition_init_from_rmw(
    node->impl->graph_guard_condition,
    rmw_graph_gc,
    context, options->allocator);
  
  // ── Step 8: 初始化 rosout 发布者 ─────────────────────
  if (options->enable_rosout) {
    ret = rcl_logging_rosout_init_publisher_for_node(node);
  }
  
  // ── Step 9: 初始化时钟 ───────────────────────────────
  ret = rcl_clock_init(RCL_SYSTEM_TIME, &node->impl->clock, &allocator);
  
  return RCL_RET_OK;
fail:
  rcl_node_fini(node);
  return fail_ret;
}
```

---

## 二、rcl_publisher_init() 完整流程

```c
// rcl/src/rcl/publisher.c

rcl_ret_t
rcl_publisher_init(
  rcl_publisher_t * publisher,
  const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rcl_publisher_options_t * options)
{
  // ── Step 1: 参数验证 ──────────────────────────────────
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_PUBLISHER_INVALID);
  
  // ── Step 2: 扩展 topic 名称 ───────────────────────────
  // 相对名 "chatter" → 绝对名 "/ns/chatter"
  char * expanded_topic_name = NULL;
  ret = rcl_expand_topic_name(
    topic_name,
    rcl_node_get_name(node),       // 节点名（用于 ~/xxx 私有名解析）
    rcl_node_get_namespace(node),  // 命名空间（用于相对名解析）
    &expanded_topic_name,
    &allocator);
  
  // ── Step 3: 应用重映射规则 ────────────────────────────
  char * remapped_topic_name = NULL;
  ret = rcl_remap_topic_name(
    &node->impl->options.arguments,
    &node->impl->context->global_arguments,
    expanded_topic_name,
    rcl_node_get_name(node),
    rcl_node_get_namespace(node),
    &allocator,
    &remapped_topic_name);
  
  // ── Step 4: 验证 topic 名称合法性 ────────────────────
  ret = rmw_validate_full_topic_name(remapped_topic_name, &validation_result, &invalid_idx);
  
  // ── Step 5: 分配 impl 结构体 ──────────────────────────
  publisher->impl = allocator.allocate(sizeof(rcl_publisher_impl_t), allocator.state);
  
  // ── Step 6: 构建 rmw QoS（将 rcl QoS 转换为 rmw QoS）──
  rmw_qos_profile_t rmw_qos;
  ret = rcl_qos_profile_to_rmw_qos(&options->qos, &rmw_qos);
  
  // ── Step 7: ★ 通过 rmw 创建发布者（最终创建 DDS DataWriter）
  publisher->impl->rmw_handle = rmw_create_publisher(
    node->impl->rmw_node_handle,
    type_support,
    remapped_topic_name,   // ROS2 名称，rmw 内部会加 "rt/" 前缀
    &rmw_qos,
    &publisher_options
  );
  
  // ── Step 8: 保存元数据 ────────────────────────────────
  publisher->impl->options = *options;
  publisher->impl->expanded_topic_name = remapped_topic_name;
  publisher->impl->type_support = type_support;
  
  return RCL_RET_OK;
}
```

---

## 三、rcl_publish() 核心实现

```c
// rcl/src/rcl/publisher.c

rcl_ret_t
rcl_publish(
  const rcl_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  // Step 1: 检查发布者有效性
  if (!rcl_publisher_is_valid(publisher)) {
    return RCL_RET_PUBLISHER_INVALID;
  }
  
  // Step 2: ★ 直接委托给 rmw（无额外逻辑）
  // 注意：rcl 层不做序列化！序列化在 rclcpp 层完成
  // 这里的 ros_message 是指向 C 结构体的指针（已是序列化格式或原始类型）
  rmw_ret_t rmw_ret = rmw_publish(
    publisher->impl->rmw_handle,
    ros_message,
    allocation           // 可选：预分配内存（减少运行时分配）
  );
  
  return rcl_convert_rmw_ret_to_rcl_ret(rmw_ret);
}

// 序列化版本（发送已序列化的字节流）
rcl_ret_t
rcl_publish_serialized_message(
  const rcl_publisher_t * publisher,
  const rcl_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  rmw_ret_t ret = rmw_publish_serialized_message(
    publisher->impl->rmw_handle,
    serialized_message,
    allocation);
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}
```

### 发布路径对比（三种方式）

```
方式1（rclcpp 标准路径）：
  rclcpp::Publisher<T>::publish(const T & msg)
    → 使用 rosidl_typesupport 序列化 msg → rcl_serialized_message_t
    → rcl_publish_serialized_message() → rmw_publish_serialized_message()

方式2（rclcpp 零拷贝路径）：
  rclcpp::Publisher<T>::publish(std::unique_ptr<T> msg)
    → 进程内通信时：IntraProcessManager::do_intra_process_publish()（不经过 rcl）
    → 进程外通信时：序列化后走方式1

方式3（直接使用 rcl，C 语言）：
  rcl_publish(&publisher, &msg, NULL)
    → 要求 msg 已是 C 语言结构体（non-idiomatic，通常配合 typesupport 手动序列化）
```

---

## 四、rcl_subscription_init() 与消息接收

### 4.1 初始化（与 publisher 对称）

```c
// rcl/src/rcl/subscription.c

rcl_ret_t
rcl_subscription_init(
  rcl_subscription_t * subscription,
  const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rcl_subscription_options_t * options)
{
  // 扩展 topic 名 → 重映射 → 验证（与 publisher 相同流程）
  // ...
  
  // ★ 创建 rmw subscription（DDS DataReader）
  subscription->impl->rmw_handle = rmw_create_subscription(
    node->impl->rmw_node_handle,
    type_support,
    remapped_topic_name,
    &rmw_qos,
    &subscription_options
  );
  
  return RCL_RET_OK;
}
```

### 4.2 rcl_take()：从 DDS 接收消息

```c
// rcl/src/rcl/subscription.c

rcl_ret_t
rcl_take(
  const rcl_subscription_t * subscription,
  void * ros_message,           // 输出：接收到的消息（C 结构体）
  rmw_message_info_t * message_info,  // 输出：消息元数据
  rmw_subscription_allocation_t * allocation)
{
  // 验证订阅者有效性
  if (!rcl_subscription_is_valid(subscription)) {
    return RCL_RET_SUBSCRIPTION_INVALID;
  }
  
  // ★ 委托给 rmw take（从 DDS DataReader 读取一条消息）
  bool taken = false;
  rmw_ret_t rmw_ret = rmw_take_with_info(
    subscription->impl->rmw_handle,
    ros_message,
    &taken,
    message_info,
    allocation
  );
  
  if (RMW_RET_OK != rmw_ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return RCL_RET_ERROR;
  }
  
  if (!taken) {
    // 没有可读消息（可能是 spurious wakeup）
    return RCL_RET_SUBSCRIPTION_TAKE_FAILED;
  }
  
  return RCL_RET_OK;
}
```

### 4.3 rcl_take_serialized_message()

```c
// 接收原始序列化字节（不反序列化）
rcl_ret_t
rcl_take_serialized_message(
  const rcl_subscription_t * subscription,
  rcl_serialized_message_t * serialized_message,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  // 分配或复用 serialized_message 的 buffer
  bool taken = false;
  rmw_ret_t rmw_ret = rmw_take_serialized_message_with_info(
    subscription->impl->rmw_handle,
    serialized_message,
    &taken,
    message_info,
    allocation
  );
  return rcl_convert_rmw_ret_to_rcl_ret(rmw_ret);
}
```

### 4.4 rmw_message_info_t（消息元数据）

```c
// rmw/include/rmw/types.h

typedef struct rmw_message_info_s
{
  // 消息发布时间戳（纳秒，由 DDS 添加）
  rmw_time_point_value_t source_timestamp;
  
  // 消息接收时间戳
  rmw_time_point_value_t received_timestamp;
  
  // 发布者 GUID（可用于过滤特定发布者的消息）
  rmw_gid_t publisher_gid;
  
  // 是否来自进程内通信
  bool from_intra_process;
  
} rmw_message_info_t;
```

---

## 五、rcl_service_init() 与 rcl_client_init()

### 5.1 Service（服务端）

```c
// rcl/src/rcl/service.c

rcl_ret_t
rcl_service_init(
  rcl_service_t * service,
  const rcl_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rcl_service_options_t * options)
{
  // 扩展服务名 → 重映射
  // service_name: "add_two_ints" → "/ns/add_two_ints"
  // DDS topic 名：
  //   Request:  rq/ns/add_two_ints/AddTwoIntsRequest_
  //   Response: rr/ns/add_two_ints/AddTwoIntsReply_
  
  // ★ rmw 创建服务（实际创建两个 DDS topic）
  service->impl->rmw_handle = rmw_create_service(
    node->impl->rmw_node_handle,
    type_support,
    expanded_service_name,
    &rmw_qos
  );
  
  return RCL_RET_OK;
}

// 接收请求
rcl_ret_t
rcl_take_request(
  const rcl_service_t * service,
  rmw_request_id_t * request_header,  // 包含：sequence_number、client_gid
  void * ros_request)
{
  bool taken = false;
  rmw_ret_t ret = rmw_take_request(
    service->impl->rmw_handle,
    request_header,
    ros_request,
    &taken);
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}

// 发送响应
rcl_ret_t
rcl_send_response(
  const rcl_service_t * service,
  rmw_request_id_t * request_header,  // ★ 必须传回原始 request_header（含 sequence_number）
  void * ros_response)
{
  rmw_ret_t ret = rmw_send_response(
    service->impl->rmw_handle,
    request_header,
    ros_response);
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}
```

### 5.2 Client（客户端）

```c
// rcl/src/rcl/client.c

// 发送请求
rcl_ret_t
rcl_send_request(
  const rcl_client_t * client,
  const void * ros_request,
  int64_t * sequence_number)  // 输出：请求序列号（用于匹配响应）
{
  rmw_ret_t ret = rmw_send_request(
    client->impl->rmw_handle,
    ros_request,
    sequence_number    // DDS 层分配唯一序列号
  );
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}

// 接收响应
rcl_ret_t
rcl_take_response(
  const rcl_client_t * client,
  rmw_request_id_t * request_header,  // 含 sequence_number，用于匹配
  void * ros_response)
{
  bool taken = false;
  rmw_ret_t ret = rmw_take_response(
    client->impl->rmw_handle,
    request_header,
    ros_response,
    &taken);
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}
```

### 5.3 请求-响应匹配机制

```
Client                              Service
  │                                    │
  ├── rcl_send_request()               │
  │   sequence_number = 42             │
  │   ──────────── rq/.../Request ──►  │
  │                                    ├── rcl_take_request(header={seq=42})
  │                                    ├── 处理请求
  │                                    └── rcl_send_response(header={seq=42})
  │   ◄─────────── rr/.../Reply ───────┤
  ├── rcl_take_response(header={seq=42})
  │   匹配 sequence_number == 42
  └── 回调触发
```

---

## 六、rcl_timer_t 实现

```c
// rcl/src/rcl/timer.c

rcl_ret_t
rcl_timer_init2(
  rcl_timer_t * timer,
  rcl_clock_t * clock,
  rcl_context_t * context,
  int64_t period,            // 触发周期（纳秒）
  const rcl_timer_callback_t callback,
  rcl_allocator_t allocator,
  bool autostart)
{
  timer->impl->period = period;
  timer->impl->callback = callback;
  timer->impl->clock = clock;
  
  // ★ 通过 rcl_clock 注册定时器（支持仿真时间）
  // 真实时间：不注册，依靠 rcl_wait_set 的 timeout 参数
  // 仿真时间：注册为时钟跳转回调（确保仿真时间推进时触发）
  if (clock->type == RCL_ROS_TIME) {
    rcl_clock_add_jump_callback(clock, jump_threshold, timer_clock_jump_callback, timer);
  }
  
  // 记录创建时间（下次触发 = 创建时间 + period）
  rcl_ret_t ret = rcl_clock_get_now(clock, &timer->impl->last_call_time);
  
  return RCL_RET_OK;
}

// 检查定时器是否就绪（在 rcl_wait() 后调用）
rcl_ret_t
rcl_timer_is_ready(const rcl_timer_t * timer, bool * is_ready)
{
  int64_t time_until_call;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &time_until_call);
  
  // 剩余时间 <= 0 表示已就绪
  *is_ready = (time_until_call <= 0);
  return ret;
}

// 执行定时器回调
rcl_ret_t
rcl_timer_call(rcl_timer_t * timer)
{
  // 更新上次调用时间（补偿漂移）
  int64_t now;
  rcl_clock_get_now(timer->impl->clock, &now);
  
  // ★ 防止漂移积累：
  // 下次触发时间 = last_call_time + period（不是 now + period）
  // 这样即使本次调用晚了，下次还是按原计划时间触发
  timer->impl->last_call_time =
    timer->impl->last_call_time + timer->impl->period;
  
  // 执行回调
  rcl_timer_callback_t cb = timer->impl->callback;
  if (cb) {
    cb(timer, time_since_last_call);
  }
  
  return RCL_RET_OK;
}
```

---

## 七、topic 名称解析规则速查

| 输入名称 | 类型 | 示例 | 解析后（节点 ns=/ns, name=my_node）|
|---------|------|------|----------------------------------|
| `foo` | 相对 | `foo` | `/ns/foo` |
| `/foo` | 绝对 | `/foo` | `/foo` |
| `~/foo` | 私有 | `~/foo` | `/ns/my_node/foo` |
| `~foo` | 私有（旧） | `~foo` | `/ns/my_node/foo` |

| DDS topic 前缀 | 用途 |
|---------------|------|
| `rt/` | ROS Topic |
| `rq/` | Service Request |
| `rr/` | Service Response |
| `rp/` | Parameter |
| `ra/` | Action Goal/Cancel/Result |
| `rs/` | Action Status |

---

## 八、完整发布-接收调用链（C 语言视角）

```c
// ── 发布端 ──────────────────────────────────────────────
// 1. 初始化
rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
rcl_publisher_init(&pub, &node, ts, "chatter", &pub_opts);

// 2. 发布（C 结构体直接发）
std_msgs__msg__String msg;
msg.data.data = "hello";
msg.data.size = 5;
rcl_publish(&pub, &msg, NULL);

// ── 接收端 ──────────────────────────────────────────────
// 1. 初始化
rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
rcl_subscription_init(&sub, &node, ts, "chatter", &sub_opts);

// 2. 等待+接收（事件循环）
rcl_wait_set_t ws = rcl_get_zero_initialized_wait_set();
rcl_wait_set_init(&ws, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
rcl_wait_set_add_subscription(&ws, &sub, NULL);

while (rcl_ok(&context)) {
  rcl_wait(&ws, RCL_S_TO_NS(1));  // 最多等 1 秒
  
  if (ws.subscriptions[0]) {  // 非 NULL 表示有数据
    std_msgs__msg__String received;
    rmw_message_info_t info;
    rcl_take(&sub, &received, &info, NULL);
    // 处理 received.data.data
  }
}
```
