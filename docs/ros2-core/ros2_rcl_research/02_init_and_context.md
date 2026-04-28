# rcl（二）：init/shutdown 与 Context 生命周期

## 一、rcl_init() 完整源码解析

`rcl_init()` 是整个 ROS2 程序的入口，它初始化全局 Context，建立与 DDS 中间件的连接。

```c
// rcl/src/rcl/init.c

rcl_ret_t
rcl_init(
  int argc,
  char const * const * argv,
  const rcl_init_options_t * options,
  rcl_context_t * context)
{
  // Step 1: 验证参数
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t fail_ret = RCL_RET_ERROR;
  
  // Step 2: 检查 context 未被初始化（防止重复调用）
  if (rcl_context_is_valid(context)) {
    RCL_SET_ERROR_MSG("context already initialized");
    return RCL_RET_ALREADY_INIT;
  }
  
  // Step 3: 分配 context 内部实现
  context->impl = options->impl->allocator.allocate(
    sizeof(rcl_context_impl_t),
    options->impl->allocator.state);
  memset(context->impl, 0, sizeof(rcl_context_impl_t));
  
  // Step 4: 解析命令行参数（--ros-args 格式）
  // 包括：节点名称覆盖、命名空间、remap 规则、参数覆盖等
  ret = rcl_parse_arguments(
    argc, argv, allocator, &context->global_arguments);
  if (RCL_RET_OK != ret) goto fail;
  
  // Step 5: 处理 --log-level 等全局日志参数
  ret = rcl_logging_configure_with_output_handler(
    &context->global_arguments, &allocator, rcl_logging_multiple_output_handler);
  if (RCL_RET_OK != ret) goto fail;
  
  // Step 6: ★ 调用 rmw 初始化（最终调用 DDS 初始化）
  ret = rmw_init(&options->impl->rmw_init_options, &context->impl->rmw_context);
  if (RCL_RET_OK != ret) goto fail;
  
  // Step 7: 注册信号处理器（SIGINT, SIGTERM → 触发 shutdown）
  ret = rcl_sigint_handler_setup(context);
  if (RCL_RET_OK != ret) goto fail;
  
  // Step 8: 生成实例 ID（原子递增，全局唯一）
  rcutils_atomic_store(
    &context->instance_id_storage,
    __rcl_next_unique_id++);  // 全局原子计数器
  
  return RCL_RET_OK;

fail:
  // 清理已分配资源
  if (context->impl) {
    rcl_shutdown(context);
    context->impl->allocator.deallocate(context->impl, ...);
  }
  return fail_ret;
}
```

### 关键步骤分解

```
rcl_init(argc, argv, options, context)
   │
   ├─ rcl_parse_arguments()          # 解析 --ros-args 参数
   │   ├─ --remap /foo:=/bar         # 名称重映射规则
   │   ├─ --param foo:=bar           # 参数赋值
   │   ├─ --node-name my_node        # 节点名覆盖
   │   └─ --log-level debug          # 日志级别
   │
   ├─ rcl_logging_configure()        # 配置日志输出（stdout/文件/rosout）
   │
   ├─ rmw_init()                     # ★ DDS 初始化（创建 DomainParticipant）
   │   └─ FastDDS：eprosima::fastdds::dds::DomainParticipantFactory::create_participant()
   │
   └─ 生成 instance_id（原子自增，用于检测 context 是否 shutdown）
```

---

## 二、rcl_init_options_t 详解

```c
// rcl/include/rcl/init_options.h

typedef struct rcl_init_options_impl_s
{
  rcl_allocator_t allocator;
  
  // rmw 初始化选项（传递给具体 DDS 实现）
  rmw_init_options_t rmw_init_options;
  
  // 域 ID（覆盖 ROS_DOMAIN_ID 环境变量）
  size_t domain_id;
  
} rcl_init_options_impl_t;
```

### 构建初始化选项

```c
// 标准用法
rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);

// 获取 rmw 选项（用于设置 DDS 专有配置）
rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

// 设置域 ID（覆盖环境变量）
rcl_init_options_set_domain_id(&init_options, 42);

// rclcpp::init() 内部的实现：
// rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
// rcl_init_options_init(&init_options, rcl_get_default_allocator());
// rcl_init(argc, argv, &init_options, &context);
```

---

## 三、rcl_shutdown() 详解

```c
// rcl/src/rcl/init.c

rcl_ret_t
rcl_shutdown(rcl_context_t * context)
{
  // Step 1: 将 instance_id 设置为 0（标记已 shutdown）
  // 所有检查 rcl_context_is_valid() 的代码会立即返回 false
  rcutils_atomic_store(&context->instance_id_storage, 0);
  
  // Step 2: 触发所有在 rcl_wait() 中阻塞的线程
  // 通过 interrupt_guard_condition 唤醒所有 wait set
  rcl_ret_t ret = rcl_trigger_guard_condition(
    context->impl->interrupt_guard_condition);
  
  // Step 3: rmw shutdown（通知 DDS 层关闭）
  // DDS 会广播 SPDP NOT_ALIVE_DISPOSED（其他节点感知离线）
  ret = rmw_shutdown(&context->impl->rmw_context);
  
  return ret;
}
```

### Shutdown 后的状态检查机制

```c
// 任何 rcl 操作都会先检查 context 有效性
bool rcl_context_is_valid(const rcl_context_t * context)
{
  return context &&
         context->impl &&
         // instance_id != 0 表示 context 有效
         0 != rcutils_atomic_load(&context->instance_id_storage);
}

// 在 rcl_publisher_publish() 中的检查：
if (!rcl_context_is_valid(publisher->impl->context)) {
  RCL_SET_ERROR_MSG("context is not valid");
  return RCL_RET_NOT_INIT;
}
```

---

## 四、命令行参数解析（rcl_arguments_t）

### 4.1 支持的参数格式

```bash
ros2 run my_pkg my_node \
  --ros-args \
  --remap /old_topic:=/new_topic \       # topic 重映射
  --remap __node:=custom_name \          # 节点名覆盖
  --remap __ns:=/my_namespace \          # 命名空间
  --param my_param:=42 \                 # 参数赋值
  --params-file /path/to/params.yaml \   # 参数文件
  --log-level debug \                    # 全局日志级别
  --log-level my_node:=info \            # 特定节点日志级别
  --enclave /my_enclave \                # 安全 enclave
  -- \                                   # 分隔符，之后是非 ROS 参数
  non_ros_arg_1 non_ros_arg_2
```

### 4.2 rcl_arguments_t 结构

```c
// rcl/include/rcl/arguments.h

typedef struct rcl_arguments_impl_s
{
  // 重映射规则列表
  rcl_remap_t * remap_rules;
  int num_remap_rules;
  
  // 参数覆盖列表
  rcl_param_rule_t * parameter_overrides;
  int num_parameter_overrides;
  
  // 参数文件路径列表
  char ** parameter_files;
  int num_parameter_files;
  
  // 日志级别设置
  rcl_log_level_t * log_levels;
  int num_log_levels;
  
  // 未解析的非 ROS 参数（索引到原始 argv）
  int * nonros_argc;
  const char ** nonros_argv;
  
  // 封装（SROS2）
  char * enclave;
  
} rcl_arguments_impl_t;
```

### 4.3 名称解析流程

```c
// rcl/src/rcl/node.c → rcl_node_init()

// 1. 获取全局重映射规则（来自 rcl_init 的参数）
const rcl_arguments_t * global_args = &context->global_arguments;

// 2. 与节点本地重映射规则合并（优先级：节点本地 > 全局）
// 最终解析节点名、命名空间、topic 名

// 示例：解析 topic 名
// 输入: "chatter"
// 节点命名空间: "/my_ns"
// 全局 remap: /my_ns/chatter → /remapped/chatter
// 输出: "/remapped/chatter"（DDS topic: rt/remapped/chatter）
char * expanded_topic_name = NULL;
rcl_expand_topic_name(
  "chatter",           // 相对名称
  node_name,           // 节点名（用于私有名 ~/xxx 解析）
  node_namespace,      // 命名空间（用于相对名 xxx 解析）
  &expanded_topic_name,
  &allocator);

// 应用重映射
rcl_remap_topic_name(
  &node->impl->options.arguments,
  context_args,
  expanded_topic_name,
  node_name,
  node_namespace,
  &allocator,
  &remapped_topic_name);
```

---

## 五、信号处理（SIGINT）

```c
// rcl/src/rcl/init.c

// 注册信号处理
static rcl_ret_t
rcl_sigint_handler_setup(rcl_context_t * context)
{
  // 创建全局 guard condition（用于从 rcl_wait() 中唤醒）
  context->impl->interrupt_guard_condition =
    allocator.allocate(sizeof(rcl_guard_condition_t), allocator.state);
  
  rcl_guard_condition_init(
    context->impl->interrupt_guard_condition,
    context, options);
  
  // 将此 guard condition 注册到 rmw 层（DDS 信号通知）
  rmw_guard_condition_t * gc =
    context->impl->interrupt_guard_condition->impl->rmw_handle;
  
  // 注册 SIGINT 处理函数
  signal(SIGINT, rcl_signal_handler);
  signal(SIGTERM, rcl_signal_handler);
  
  return RCL_RET_OK;
}

// SIGINT 处理函数
static void
rcl_signal_handler(int signal_value)
{
  // 遍历所有活跃的 context，执行 shutdown
  for (size_t i = 0; i < g_contexts_count; ++i) {
    rcl_shutdown(g_contexts[i]);
  }
}
```

### rclcpp::ok() 的实现

```cpp
// rclcpp/src/rclcpp/utilities.cpp

bool rclcpp::ok(rclcpp::Context::SharedPtr context)
{
  return context->is_valid();  // 最终调用 rcl_context_is_valid()
}

// 典型的 spin 循环
while (rclcpp::ok()) {
  executor.spin_once();
}
// 当 SIGINT 触发 → rcl_shutdown() → instance_id=0 → rcl_context_is_valid()=false
// → rclcpp::ok() 返回 false → while 循环退出
```

---

## 六、多 Context 支持（高级用法）

ROS2 支持在同一进程中运行多个独立的 Context：

```cpp
// 创建独立 context
auto ctx1 = std::make_shared<rclcpp::Context>();
auto ctx2 = std::make_shared<rclcpp::Context>();
ctx1->init(argc, argv, rclcpp::InitOptions());
ctx2->init(argc, argv, rclcpp::InitOptions());

// 在不同 context 中创建节点（互相隔离）
auto node1 = std::make_shared<rclcpp::Node>("node1", 
  rclcpp::NodeOptions().context(ctx1));
auto node2 = std::make_shared<rclcpp::Node>("node2",
  rclcpp::NodeOptions().context(ctx2));

// 独立 shutdown（不影响另一个 context 的节点）
ctx1->shutdown("reason");

// 使用场景：测试框架、隔离的子系统
```

---

## 七、rcl_context_get_domain_id()

```c
// rcl/src/rcl/context.c

rcl_ret_t
rcl_context_get_domain_id(
  rcl_context_t * context,
  size_t * domain_id)
{
  // 优先使用 init_options 中设置的 domain_id
  // 若未设置（RCL_DEFAULT_DOMAIN_ID），从环境变量读取
  if (context->impl->init_options.impl->domain_id != RCL_DEFAULT_DOMAIN_ID) {
    *domain_id = context->impl->init_options.impl->domain_id;
  } else {
    // 读取 ROS_DOMAIN_ID 环境变量
    char * domain_id_str = getenv("ROS_DOMAIN_ID");
    if (domain_id_str) {
      *domain_id = (size_t)atoi(domain_id_str);
    } else {
      *domain_id = 0;  // 默认 Domain 0
    }
  }
  return RCL_RET_OK;
}
```

---

## 八、日志系统初始化

```c
// rcl 初始化时配置日志
ret = rcl_logging_configure_with_output_handler(
  &context->global_arguments,
  &allocator,
  rcl_logging_multiple_output_handler
);

// 日志输出处理器（同时写到多个 sink）
void rcl_logging_multiple_output_handler(
  const rcl_logger_severity_map_t * severity_map,
  const char * name,
  rcl_log_severity_t severity,
  const char * format,
  va_list args)
{
  // 1. 输出到 stdout（默认启用）
  if (g_stdout_handler) g_stdout_handler(...);
  
  // 2. 输出到文件（--log-file 参数指定）
  if (g_file_handler) g_file_handler(...);
  
  // 3. 输出到 rosout（/rosout topic，其他节点可订阅）
  // 在节点创建后通过 rcl_logging_rosout 发布
}
```

### 日志宏展开

```c
// RCLCPP_INFO(logger, "msg %s", arg) 展开为：
RCUTILS_LOG_INFO_NAMED(logger.get_name(), "msg %s", arg)
// 最终调用：
rcutils_logging_log_message(
  "my_node",            // logger name
  RCUTILS_LOG_SEVERITY_INFO,
  "msg %s",             // format
  arg                   // va_args
)
// → 检查日志级别 → 格式化 → 调用所有注册的 handler
```
