# rcl（五）：速查手册与底层内存模型

## 一、rcl 初始化/销毁标准模式

```c
// ── 完整的 C 语言 ROS2 程序骨架 ──────────────────────────────

#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

int main(int argc, char * argv[])
{
  // 1. 初始化 rcl
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, rcl_get_default_allocator());
  rcl_init(argc, argv, &init_options, &context);
  rcl_init_options_fini(&init_options);

  // 2. 创建节点
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_opts = rcl_node_get_default_options();
  rcl_node_init(&node, "my_node", "", &context, &node_opts);

  // 3. 获取类型支持
  const rosidl_message_type_support_t * ts =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

  // 4. 创建发布者
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_publisher_init(&pub, &node, ts, "chatter", &pub_opts);

  // 5. 创建订阅者
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  rcl_subscription_init(&sub, &node, ts, "chatter", &sub_opts);

  // 6. 等待集合
  rcl_wait_set_t ws = rcl_get_zero_initialized_wait_set();
  rcl_wait_set_init(&ws, 1, 1, 0, 0, 0, 0, &context, rcl_get_default_allocator());

  // 7. 主循环
  while (rcl_context_is_valid(&context)) {
    rcl_wait_set_clear(&ws);
    rcl_wait_set_add_subscription(&ws, &sub, NULL);
    rcl_wait(&ws, RCL_S_TO_NS(1));

    if (ws.subscriptions[0]) {
      std_msgs__msg__String msg;
      std_msgs__msg__String__init(&msg);
      rmw_message_info_t info;
      if (RCL_RET_OK == rcl_take(&sub, &msg, &info, NULL)) {
        printf("Received: %s\n", msg.data.data);
      }
      std_msgs__msg__String__fini(&msg);
    }
  }

  // 8. 清理（逆序）
  rcl_wait_set_fini(&ws);
  rcl_subscription_fini(&sub, &node);
  rcl_publisher_fini(&pub, &node);
  rcl_node_fini(&node);
  rcl_shutdown(&context);
  rcl_context_fini(&context);
  return 0;
}
```

---

## 二、rcl 内存模型总结

### 2.1 内存所有权规则

```
所有权规则（重要！避免内存泄漏）：

rcl_*_init() 函数：
  - 内部调用 allocator.allocate() 分配 impl 结构体
  - 调用者提供句柄（rcl_publisher_t 等），在栈或堆上均可

rcl_*_fini() 函数：
  - 释放 init 期间分配的所有内部资源
  - 不释放句柄本身（调用者负责）

消息内存（rcl_take 接收的消息）：
  - 调用者负责分配（调用前构造）
  - 调用者负责释放（调用后析构）
  - 不可复用同一缓冲区（除非调用 rosidl 的 init() 重置）

序列化消息（rcl_serialized_message_t）：
  - rcl_take_serialized_message() 会自动 realloc 缓冲区
  - 调用者必须调用 rmw_serialized_message_fini() 释放
```

### 2.2 allocator 传播路径

```
rcl_init_options_init(options, my_allocator)
    │
    └─► options.allocator = my_allocator
                │
                └─► rcl_init(options, context)
                        │
                        └─► context.impl = my_allocator.allocate(sizeof(impl))
                                │
                                └─► rmw_init(rmw_options, rmw_context)
                                        （rmw 层有自己的内存管理）
```

---

## 三、错误处理规范

```c
// rcl 错误字符串是线程本地的（thread-local storage）
// 因此不同线程的错误互不影响

// 获取错误字符串
const char * error_str = rcl_get_error_string().str;

// 重置错误（必须在使用完错误字符串后调用，否则下次错误会追加）
rcl_reset_error();

// 标准错误处理模式
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    return 1; \
  } \
}

RCCHECK(rcl_publisher_init(&pub, &node, ts, "chatter", &opts));
```

---

## 四、rcl_action 简介（Action 协议底层）

```c
// rcl_action/include/rcl_action/action_server.h

// Action Server 是对 rcl service + publisher 的组合封装
typedef struct rcl_action_server_s {
  struct rcl_action_server_impl_s * impl;
} rcl_action_server_t;

// 内部包含：
// - 3 个 Service（goal/cancel/result）
// - 2 个 Publisher（feedback/status）
// - Goal Handle 状态机
```

```c
// Action Server 初始化
rcl_action_server_t action_server = rcl_action_get_zero_initialized_server();
rcl_action_server_init(
  &action_server,
  &node,
  &clock,
  action_ts,          // rosidl_action_type_support_t
  "navigate_to_pose",
  &server_options
);

// 等待集合中添加 action server（需要 5 个 entity 槽位）
// 3 services + 2 guard_conditions（进程内通信）
size_t num_subscriptions = 0, num_guard_conditions = 0, ...;
rcl_action_wait_set_get_num_entities(&action_server,
  &num_subscriptions, &num_guard_conditions, ...);
```

---

## 五、rcl_lifecycle 简介

```c
// rcl_lifecycle 基于 rcl_service 实现状态机管理
// 每个 LifecycleNode 自动创建：
//   - /node_name/change_state service
//   - /node_name/get_state service
//   - /node_name/get_available_states service
//   - /node_name/get_available_transitions service

rcl_lifecycle_state_machine_t state_machine =
  rcl_lifecycle_get_zero_initialized_state_machine();

rcl_lifecycle_state_machine_init(
  &state_machine,
  &node,
  &ts_srv_change_state,
  &ts_srv_get_state,
  &ts_srv_get_available_states,
  &ts_srv_get_available_transitions,
  &ts_pub_transition_event,
  allocate_memory,    // 是否分配默认状态/转换
  &allocator
);
```

---

## 六、时间常量宏

```c
// rcl/include/rcl/time.h

// 时间单位转换
#define RCL_S_TO_NS(seconds)         ((int64_t)(seconds) * 1000000000LL)
#define RCL_MS_TO_NS(milliseconds)   ((int64_t)(milliseconds) * 1000000LL)
#define RCL_US_TO_NS(microseconds)   ((int64_t)(microseconds) * 1000LL)
#define RCL_NS_TO_S(nanoseconds)     ((nanoseconds) / 1000000000LL)
#define RCL_NS_TO_MS(nanoseconds)    ((nanoseconds) / 1000000LL)
#define RCL_NS_TO_US(nanoseconds)    ((nanoseconds) / 1000LL)

// 特殊值
#define RCL_DURATION_INFINITE   (-1LL)   // rcl_wait() 永久等待
#define RCL_TIME_MAX            INT64_MAX

// 使用示例
rcl_wait(&ws, RCL_S_TO_NS(1));      // 等待最多 1 秒
rcl_timer_init2(&timer, &clock, &ctx, RCL_MS_TO_NS(100), cb, alloc, true);  // 100ms 周期
```

---

## 七、源码阅读路径（推荐顺序）

```
第一轮（理解整体）：
  1. rcl/include/rcl/rcl.h          # 所有公共接口汇总
  2. rcl/src/rcl/init.c             # 初始化入口
  3. rcl/src/rcl/node.c             # 节点实现

第二轮（理解通信）：
  4. rcl/src/rcl/publisher.c        # 发布者实现
  5. rcl/src/rcl/subscription.c     # 订阅者实现
  6. rcl/src/rcl/wait.c             # ★ 最重要：等待机制

第三轮（理解时间）：
  7. rcl/src/rcl/timer.c            # 定时器实现
  8. rcl/src/rcl/time.c             # 时钟实现

第四轮（理解名称解析）：
  9. rcl/src/rcl/expand_topic_name.c # topic 名称扩展
  10. rcl/src/rcl/remap.c            # 名称重映射

GitHub 链接：
  https://github.com/ros2/rcl/tree/rolling/rcl/src/rcl
```

---

## 八、与 rclcpp 对应关系

| rcl 函数 | rclcpp 等效 |
|---------|------------|
| `rcl_init()` | `rclcpp::init()` |
| `rcl_shutdown()` | `rclcpp::shutdown()` |
| `rcl_ok()` | `rclcpp::ok()` |
| `rcl_node_init()` | `rclcpp::Node::Node()` |
| `rcl_publisher_init()` | `Node::create_publisher<T>()` |
| `rcl_publish()` | `Publisher<T>::publish()` |
| `rcl_subscription_init()` | `Node::create_subscription<T>()` |
| `rcl_take()` | 在 Executor 内部调用 |
| `rcl_timer_init2()` | `Node::create_wall_timer()` |
| `rcl_wait_set_init()` | 由 Executor 内部管理 |
| `rcl_wait()` | `Executor::spin()` 内部调用 |
| `rcl_service_init()` | `Node::create_service<T>()` |
| `rcl_client_init()` | `Node::create_client<T>()` |
| `rcl_guard_condition_init()` | `rclcpp::GuardCondition` |
