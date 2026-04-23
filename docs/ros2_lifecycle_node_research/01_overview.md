# Lifecycle Node（一）：状态机架构

## 一、为什么需要 Lifecycle Node

```
普通 Node 的问题：
  1. 节点一启动就立即工作（发布/订阅）
  2. 无法在不重启的情况下重新配置
  3. 依赖节点的启动顺序难以保证
  4. 无法优雅地在错误状态恢复

Lifecycle Node 解决方案：
  1. 明确的状态机：控制节点何时开始工作
  2. 运行时可重新配置（ACTIVE → deactivate → INACTIVE → cleanup → UNCONFIGURED → configure → INACTIVE → activate → ACTIVE）
  3. 外部管理器（LifecycleManager）可协调多节点启动顺序
  4. 错误状态可恢复（ERRORPROCESSING → cleanup/error）
```

---

## 二、完整状态列表（Primary States + Transition States）

```
Primary States（稳定状态）：
  UNKNOWN      (0) - 初始化时的瞬时状态，立即转为 UNCONFIGURED
  UNCONFIGURED (1) - 已创建，未配置（等待 configure）
  INACTIVE     (2) - 已配置，未激活（等待 activate）
  ACTIVE       (3) - 正在工作（可发布/订阅）
  FINALIZED    (4) - 已关闭（终态，不可恢复）

Transition States（过渡状态，回调执行期间）：
  CONFIGURING          (10) - configure() 回调执行中
  CLEANINGUP           (11) - cleanup() 回调执行中
  SHUTTINGDOWN         (12) - shutdown() 回调执行中
  ACTIVATING           (13) - activate() 回调执行中
  DEACTIVATING         (14) - deactivate() 回调执行中
  ERRORPROCESSING      (15) - 错误处理中
```

---

## 三、完整状态转换表

```
from_state        | transition    | callback           | success_state | failure_state
───────────────────────────────────────────────────────────────────────────────────
UNCONFIGURED      | configure     | on_configure()     | INACTIVE      | UNCONFIGURED
INACTIVE          | activate      | on_activate()      | ACTIVE        | INACTIVE
ACTIVE            | deactivate    | on_deactivate()    | INACTIVE      | ACTIVE
INACTIVE          | cleanup       | on_cleanup()       | UNCONFIGURED  | INACTIVE
ACTIVE            | shutdown      | on_shutdown()      | FINALIZED     | FINALIZED
INACTIVE          | shutdown      | on_shutdown()      | FINALIZED     | FINALIZED
UNCONFIGURED      | shutdown      | on_shutdown()      | FINALIZED     | FINALIZED
ERRORPROCESSING   | cleanup       | on_error()→cleanup | UNCONFIGURED  | FINALIZED
                  |               | (若cleanup成功)    |               |
```

---

## 四、rcl_lifecycle 状态机数据结构

```c
// rcl_lifecycle/include/rcl_lifecycle/rcl_lifecycle.h

typedef struct rcl_lifecycle_state_machine_s {
  // ★ 当前状态
  const rcl_lifecycle_state_t * current_state;
  
  // ★ 完整状态转换图（邻接表）
  const rcl_lifecycle_transition_map_t * transition_map;
  
  // ★ 管理 Service（5个）
  rcl_lifecycle_com_interface_t com_interface;
  
} rcl_lifecycle_state_machine_t;

typedef struct rcl_lifecycle_state_s {
  const char * label;    // "unconfigured", "inactive", etc.
  uint8_t id;            // 数字 ID
  
  // 可用的转换
  const rcl_lifecycle_transition_t ** valid_transitions;
  unsigned int valid_transition_size;
} rcl_lifecycle_state_t;

typedef struct rcl_lifecycle_transition_s {
  const char * label;    // "configure", "activate", etc.
  unsigned int id;
  const rcl_lifecycle_state_t * start;
  const rcl_lifecycle_state_t * goal;
} rcl_lifecycle_transition_t;
```

---

## 五、rcl_lifecycle_state_machine_init() 源码

```c
// rcl_lifecycle/src/rcl_lifecycle.c

rcl_ret_t
rcl_lifecycle_state_machine_init(
  rcl_lifecycle_state_machine_t * state_machine,
  rcl_node_t * node_handle,
  const rosidl_message_type_support_t * ts_pub_notify,
  const rosidl_service_type_support_t * ts_srv_change_state,
  const rosidl_service_type_support_t * ts_srv_get_state,
  const rosidl_service_type_support_t * ts_srv_get_available_states,
  const rosidl_service_type_support_t * ts_srv_get_available_transitions,
  const rosidl_service_type_support_t * ts_srv_get_transition_graph,
  bool default_states,
  rcl_allocator_t * allocator)
{
  // ★ Step 1: 初始化状态图（如果使用默认状态）
  if (default_states) {
    rcl_ret_t ret = rcl_lifecycle_init_default_state_machine(state_machine, allocator);
    // 创建 5 个 Primary States + 6 个 Transition States
    // 填充 transition_map（15 条转换边）
  }
  
  // ★ Step 2: 创建管理 Services
  // change_state Service
  rcl_service_init(
    &state_machine->com_interface.srv_change_state,
    node_handle,
    ts_srv_change_state,
    "~/change_state",
    &service_options
  );
  
  // get_state Service
  rcl_service_init(
    &state_machine->com_interface.srv_get_state,
    node_handle,
    ts_srv_get_state,
    "~/get_state",
    &service_options
  );
  
  // 其他 3 个 Service...
  
  // ★ Step 3: 创建 lifecycle_state Topic Publisher（通知状态变化）
  rcl_publisher_init(
    &state_machine->com_interface.pub_transition_event,
    node_handle,
    ts_pub_notify,
    "~/transition_event",   // lifecycle_msgs/msg/TransitionEvent
    &publisher_options
  );
  
  return RCL_RET_OK;
}
```

---

## 六、rcl_lifecycle_trigger_transition() — 状态转换核心

```c
// rcl_lifecycle/src/rcl_lifecycle.c

rcl_lifecycle_ret_t
rcl_lifecycle_trigger_transition_by_id(
  rcl_lifecycle_state_machine_t * state_machine,
  uint8_t id,
  bool publish_notification)
{
  // ★ Step 1: 在转换图中查找合法转换
  const rcl_lifecycle_transition_t * transition =
    rcl_lifecycle_get_transition_by_id(state_machine, id);
  
  if (!transition) {
    // 从当前状态无法执行此转换
    return RCL_LIFECYCLE_RET_ERROR;
  }
  
  // ★ Step 2: 确认起始状态匹配
  if (state_machine->current_state->id != transition->start->id) {
    return RCL_LIFECYCLE_RET_ERROR;
  }
  
  // ★ Step 3: 转换到目标状态
  state_machine->current_state = transition->goal;
  
  // ★ Step 4: 发布状态变化通知（~/transition_event）
  if (publish_notification) {
    lifecycle_msgs__msg__TransitionEvent msg;
    msg.timestamp = get_current_time();
    msg.transition.id = transition->id;
    msg.transition.label = transition->label;
    msg.start_state = *transition->start;
    msg.goal_state = *transition->goal;
    
    rcl_publish(&state_machine->com_interface.pub_transition_event, &msg, NULL);
  }
  
  return RCL_LIFECYCLE_RET_OK;
}
```

---

## 七、change_state Service 响应流程

```
外部调用（ros2 lifecycle set /my_node configure）：
  1. 发送 ChangeState 请求：{transition: {id: CONFIGURE_ID}}
  2. rcl_lifecycle 的 change_state Service Handler（在 rclcpp::LifecycleNode::execute_change_state）：
     a. 转换到过渡状态（UNCONFIGURED → CONFIGURING）
     b. 调用用户的 on_configure() 回调
     c. 根据返回值 CallbackReturn：
        SUCCESS → 转换到 INACTIVE
        FAILURE → 转换回 UNCONFIGURED
        ERROR   → 转换到 ERRORPROCESSING
     d. 发送 ChangeState 响应（success: true/false）
  3. 发布 ~/transition_event（通知所有监听者）
```
