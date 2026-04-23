# Service/Action（三）：Action 架构与状态机

## 一、Action 的本质：5个 DDS Topics 的组合

```
Action 名：/navigate_to_pose

DDS Topics（自动创建）：
  ┌── 服务类（Service）：
  │   rq/navigate_to_pose/_action/send_goalRequest_
  │   rr/navigate_to_pose/_action/send_goalReply_
  │   rq/navigate_to_pose/_action/cancel_goalRequest_
  │   rr/navigate_to_pose/_action/cancel_goalReply_
  │   rq/navigate_to_pose/_action/get_resultRequest_
  │   rr/navigate_to_pose/_action/get_resultReply_
  │
  └── 话题类（Topic）：
      rt/navigate_to_pose/_action/feedback
      rt/navigate_to_pose/_action/status
```

### 消息类型 IDL 文件关系

```
action 文件：nav2_msgs/action/NavigateToPose.action
  ───────────────────────────────
  # Goal
  geometry_msgs/PoseStamped pose
  string behavior_tree
  ---
  # Result
  std_msgs/Empty result
  ---
  # Feedback
  geometry_msgs/PoseStamped current_pose
  float32 distance_remaining

编译生成（rosidl_default_generators）：
  NavigateToPose_SendGoal_Request  (Goal content)
  NavigateToPose_SendGoal_Response (accepted:bool, stamp:builtin_interfaces/Time)
  NavigateToPose_GetResult_Request (goal_id:UUID)
  NavigateToPose_GetResult_Response (status:int8, result)
  NavigateToPose_CancelGoal_Request  (→ action_msgs/srv/CancelGoal_Request)
  NavigateToPose_CancelGoal_Response (→ action_msgs/srv/CancelGoal_Response)
  NavigateToPose_FeedbackMessage (goal_id:UUID + Feedback content)
  NavigateToPose_GoalStatusArray  (→ action_msgs/msg/GoalStatusArray)
```

---

## 二、Goal 生命周期状态机

```
                     send_goal()
                         │
                         ▼
                   ┌─────────────┐
                   │  ACCEPTING  │ ← Server 的 handle_goal() 正在运行
                   └──────┬──────┘
              accepted     │      rejected
            ┌──────────────┤
            ▼              ▼
      ┌──────────┐   ┌──────────┐
      │ ACCEPTED │   │ REJECTED │ (终态)
      └────┬─────┘   └──────────┘
           │ execute()（Server 主动调用）
           ▼
      ┌──────────┐
      │ EXECUTING│ ← Server 执行中，可发布 Feedback
      └────┬─────┘
           │
   ┌───────┼───────────────┐
   │       │               │
   ▼       ▼               ▼
┌──────┐ ┌──────────────┐ ┌──────────────┐
│CANCEL│ │  SUCCEEDED   │ │   ABORTED    │
│ ING  │ │  (终态)      │ │  (终态)      │
└──┬───┘ └──────────────┘ └──────────────┘
   │
   ▼
┌──────────┐
│ CANCELED │ (终态)
└──────────┘

状态码（action_msgs/msg/GoalStatus）：
  STATUS_UNKNOWN   = 0
  STATUS_ACCEPTED  = 1
  STATUS_EXECUTING = 2
  STATUS_CANCELING = 3
  STATUS_SUCCEEDED = 4
  STATUS_CANCELED  = 5
  STATUS_ABORTED   = 6
```

---

## 三、rcl_action 层数据结构

```c
// rcl_action/include/rcl_action/action_server.h

typedef struct rcl_action_server_impl_s {
  rcl_service_t goal_service;      // send_goal 请求处理
  rcl_service_t cancel_service;    // cancel_goal 请求处理
  rcl_service_t result_service;    // get_result 请求处理
  rcl_publisher_t feedback_publisher;   // Feedback 发布
  rcl_publisher_t status_publisher;     // GoalStatusArray 发布
  
  rcl_timer_t expire_timer;        // Goal 超期清理定时器
  
  // 目标管理
  rcl_action_goal_handle_t ** goal_handles;
  size_t num_goal_handles;
  
  rcl_action_server_options_t options;
} rcl_action_server_impl_t;

// Goal Handle 状态
typedef struct rcl_action_goal_info_s {
  action_msgs__msg__GoalInfo info;   // goal_id(UUID) + stamp
} rcl_action_goal_info_t;

typedef struct rcl_action_goal_handle_impl_s {
  rcl_action_goal_info_t info;
  rcl_action_goal_state_t state;    // 枚举：ACCEPTED/EXECUTING/...
  rcl_clock_t * clock;
} rcl_action_goal_handle_impl_t;
```

---

## 四、rcl_action_accept_new_goal() 状态转换

```c
// rcl_action/src/rcl_action/action_server.c

rcl_action_goal_handle_t *
rcl_action_accept_new_goal(
  rcl_action_server_t * action_server,
  const rcl_action_goal_info_t * goal_info)
{
  // 1. 分配新 GoalHandle
  rcl_action_goal_handle_t * goal_handle = new_goal_handle(goal_info);
  
  // 2. ★ 初始状态：ACCEPTED
  goal_handle->impl->state = GOAL_STATE_ACCEPTED;
  
  // 3. 将 handle 加入 goal_handles 数组
  goal_server_impl->goal_handles[goal_server_impl->num_goal_handles++] = goal_handle;
  
  // 4. ★ 立即发布 GoalStatusArray（通知所有 Client 有新 Goal 被接受）
  rcl_action_publish_status(action_server);
  
  return goal_handle;
}

// 状态转换函数（有限状态机实现）：
rcl_ret_t
rcl_action_update_goal_state(
  rcl_action_goal_handle_t * goal_handle,
  rcl_action_goal_event_t goal_event)  // EXECUTE/CANCEL_GOAL/SUCCEED/ABORT/CANCELED
{
  // 查表：goal_state_transition_table[current_state][event] = next_state
  rcl_action_goal_state_t new_state = 
    goal_state_transition_table[goal_handle->impl->state][goal_event];
  
  if (new_state == GOAL_STATE_UNKNOWN) {
    return RCL_RET_ACTION_GOAL_EVENT_INVALID;  // 非法状态转换
  }
  
  goal_handle->impl->state = new_state;
  return RCL_RET_OK;
}
```

### 状态转换表

```
事件（goal_event）：
  EXECUTE        → ACCEPTED    → EXECUTING
  CANCEL_GOAL    → EXECUTING   → CANCELING
  SUCCEED        → EXECUTING   → SUCCEEDED
  ABORT          → EXECUTING   → ABORTED
  CANCELED       → CANCELING   → CANCELED
  SUCCEED        → CANCELING   → SUCCEEDED (可以在取消过程中完成)
```

---

## 五、rclcpp_action::Server<T> 架构

```cpp
// rclcpp_action/include/rclcpp_action/server.hpp

template<typename ActionT>
class Server : public ServerBase
{
public:
  // 用户需要实现的 3 个回调：
  
  // 1. handle_goal: 决定接受/拒绝 Goal
  using GoalCallback = std::function<
    GoalResponse(
      const GoalUUID &,
      std::shared_ptr<const typename ActionT::Goal>)>;
  
  // 2. handle_cancel: 决定是否允许取消
  using CancelCallback = std::function<
    CancelResponse(
      std::shared_ptr<ServerGoalHandle<ActionT>>)>;
  
  // 3. handle_accepted: Goal 被接受后的执行逻辑
  using AcceptedCallback = std::function<
    void(std::shared_ptr<ServerGoalHandle<ActionT>>)>;

  // 创建：
  static std::shared_ptr<Server<ActionT>>
  make_shared(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & name,
    const rcl_action_server_options_t & options,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    AcceptedCallback handle_accepted)
  {
    // 内部调用 rcl_action_server_init() 初始化 5 个 DDS 实体
    ...
  }
};
```

---

## 六、send_goal 请求到 handle_accepted 完整流程

```cpp
// rclcpp_action/src/server.cpp

// ① Executor 检测到 goal_service 有数据 → execute_goal_request_received()
void ServerBase::execute_goal_request_received(
  std::shared_ptr<rcl_action_server_t> action_server)
{
  // 1. 从 rcl 取出 SendGoal 请求
  rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
  rcl_ret_t ret = rcl_action_take_goal_request(
    action_server.get(),
    &request_header,
    &goal_request   // 包含 UUID + Goal 内容
  );
  
  // 2. 提取 UUID（16字节随机 ID）
  GoalUUID uuid;
  convert_goal_id_to_ros(&goal_info.goal_id, &uuid);
  
  // 3. ★ 调用用户的 handle_goal 回调
  GoalResponse response = call_handle_goal_callback(uuid, goal_request);
  
  // 4. 构建 SendGoal 响应
  typename ActionT::Impl::SendGoalService::Response send_goal_response;
  
  if (response == GoalResponse::ACCEPT_AND_EXECUTE 
    || response == GoalResponse::ACCEPT_AND_DEFER) {
    // ★ 接受 Goal：在 rcl_action 层创建 GoalHandle
    auto goal_handle = rcl_action_accept_new_goal(
      action_server.get(), &goal_info);
    
    send_goal_response.accepted = true;
    send_goal_response.stamp = now();
    
    // ★ 创建 rclcpp_action::ServerGoalHandle（封装 rcl GoalHandle）
    auto rclcpp_goal_handle = std::make_shared<ServerGoalHandle<ActionT>>(
      goal_handle,
      uuid,
      goal,
      on_terminal_state_,    // 终态回调
      on_executing_,         // execute 回调
      publish_feedback_      // feedback 回调
    );
    
    // 5. ★ 调用 handle_accepted 回调（通常在此启动执行线程）
    if (response == GoalResponse::ACCEPT_AND_EXECUTE) {
      call_handle_accepted_callback(rclcpp_goal_handle);
    }
  } else {
    send_goal_response.accepted = false;
  }
  
  // 6. 发送 SendGoal 响应（trigger client future）
  rcl_action_send_goal_response(
    action_server.get(), &request_header, &send_goal_response);
}
```

---

## 七、ServerGoalHandle<T> 接口

```cpp
// rclcpp_action/include/rclcpp_action/server_goal_handle.hpp

template<typename ActionT>
class ServerGoalHandle : public ServerGoalHandleBase
{
public:
  // 获取目标内容
  const std::shared_ptr<const typename ActionT::Goal>
  get_goal() const { return goal_; }

  // 获取 Goal UUID
  const GoalUUID & get_goal_id() const;
  
  // ★ 发布 Feedback
  void publish_feedback(
    std::shared_ptr<typename ActionT::Feedback> feedback_msg)
  {
    auto feedback_message = std::make_shared<typename ActionT::Impl::FeedbackMessage>();
    feedback_message->goal_id.uuid = get_goal_id();
    feedback_message->feedback = *feedback_msg;
    publish_feedback_(feedback_message);  // 内部调用 rcl_action_publish_feedback()
  }
  
  // ★ 报告成功
  void succeed(typename ActionT::Result::SharedPtr result_msg)
  {
    // 状态机转换：EXECUTING → SUCCEEDED
    _set_succeeded();    // rcl_action_update_goal_state(SUCCEED)
    _set_result(result_msg);   // 存储结果，等待 get_result 请求
    on_terminal_state_(get_goal_id(), result_msg);
  }
  
  // ★ 报告终止（执行失败）
  void abort(typename ActionT::Result::SharedPtr result_msg)
  {
    _set_aborted();      // rcl_action_update_goal_state(ABORT)
    _set_result(result_msg);
    on_terminal_state_(get_goal_id(), result_msg);
  }
  
  // ★ 检查是否被取消
  bool is_canceling() const
  {
    rcl_action_goal_state_t state = _get_state();
    return state == GOAL_STATE_CANCELING;
  }
};
```

---

## 八、get_result 流程（Client 阻塞等待结果）

```cpp
// Action Client 请求结果：

// ① Client 发送 send_goal 后，收到 accepted=true
// ② Client 立即发送 GetResult 请求（不等待，在 DDS 层阻塞）
//    get_result_request = { goal_id = uuid }

// ③ Server 在达到终态时（succeed/abort/canceled）：
//    → on_terminal_state_ 回调触发
//    → 结果存入 server 的 result_map_[uuid]
//    → 之前挂起的 GetResult 请求现在有了结果
//    → rcl_action_send_result_response() 发出响应

// ④ Client 收到 GetResult 响应：
//    → future.set_value(result)
//    → get_result_future 完成

// Server 侧 GetResult 处理：
void ServerBase::execute_result_request_received(...)
{
  // 从队列取请求
  rcl_action_take_result_request(...);
  
  auto goal_uuid = convert_uuid(result_request.goal_id);
  
  // 检查结果是否就绪
  if (result_map_.count(goal_uuid)) {
    // ★ 立即回应
    send_result_response(goal_uuid);
  } else {
    // ★ 挂起：等待 Goal 到达终态
    result_requests_[goal_uuid].push_back(request_header);
  }
}
```

---

## 九、完整 Action 通信时序

```
Client                    DDS 网络                    Server（Executor）
  │                          │                              │
  │ send_goal_async(goal)    │                              │
  ├──send_goal_request──────►│  rq/.../send_goalRequest     │
  │  store uuid→future       │─────────────────────────────►│ handle_goal(uuid, goal)
  │                          │                              │ → ACCEPT_AND_EXECUTE
  │                          │  rr/.../send_goalReply       │ rcl_action_accept_new_goal()
  │                          │◄─────────────────────────────┤ send_goal_response(accepted=T)
  │ goal_future.set_value()  │                              │ handle_accepted(goal_handle)
  │                          │                              │   // 用户启动执行线程
  │ get_result_async(uuid)   │                              │   goal_handle->execute()
  ├──get_result_request─────►│  rq/.../get_resultRequest    │
  │  (挂起等待结果)           │─────────────────────────────►│ (挂起，存入 result_requests_)
  │                          │                              │
  │                          │  rt/.../feedback             │
  │ feedback_callback()     ◄├──────────────────────────────┤ goal_handle->publish_feedback()
  │                          │                              │
  │                          │  rt/.../status (定期发布)    │
  │ status update            │◄─────────────────────────────┤ rcl_action_publish_status()
  │                          │                              │
  │                          │                              │ goal_handle->succeed(result)
  │                          │                              │ → state: EXECUTING→SUCCEEDED
  │                          │  rr/.../get_resultReply      │
  │                          │◄─────────────────────────────┤ send_result_response()
  │ result_future.set_value()│                              │
  │                          │                              │
```

---

## 十、Cancel Goal 流程

```cpp
// Client 发起取消：
auto cancel_future = action_client->async_cancel_goal(goal_handle);

// 内部：
// 1. 发送 CancelGoal 请求（包含要取消的 goal_id）
//    cancel_request.goal_info.goal_id = uuid
//    cancel_request.goal_info.stamp = {0,0}  // 取消特定 Goal

// Server 处理取消：
void ServerBase::execute_cancel_request_received(...)
{
  // 调用用户的 handle_cancel 回调
  CancelResponse cancel_response = call_handle_cancel_callback(goal_handle);
  
  if (cancel_response == CancelResponse::ACCEPT) {
    // ★ 状态机：EXECUTING → CANCELING
    rcl_action_update_goal_state(goal_handle, GOAL_EVENT_CANCEL_GOAL);
    // 发送 cancel 成功响应
  }
}

// Server 执行逻辑中检查：
void my_execute(GoalHandle::SharedPtr goal_handle) {
  while (/* 执行逻辑 */) {
    if (goal_handle->is_canceling()) {
      // ★ 检测到取消请求，清理并上报
      goal_handle->canceled(std::make_shared<MyAction::Result>());
      return;
    }
    // 发布 feedback
    goal_handle->publish_feedback(feedback);
  }
  goal_handle->succeed(result);
}
```
