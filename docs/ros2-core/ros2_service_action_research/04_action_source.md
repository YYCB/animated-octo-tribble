# Service/Action（四）：rclcpp_action 完整源码实现

## 一、rcl_action_server_init() 详解

```c
// rcl_action/src/rcl_action/action_server.c

rcl_ret_t
rcl_action_server_init(
  rcl_action_server_t * action_server,
  rcl_node_t * node,
  rcl_clock_t * clock,
  const rosidl_action_type_support_t * type_support,
  const char * action_name,
  const rcl_action_server_options_t * options)
{
  const char * action_name = expanded_name;
  
  // ★ 创建 3 个 rcl_service（send_goal, cancel_goal, get_result）
  // send_goal service
  char goal_service_name[256];
  snprintf(goal_service_name, sizeof(goal_service_name), "%s/_action/send_goal", action_name);
  ret = rcl_service_init(
    &action_server->impl->goal_service,
    node,
    type_support->goal,   // rosidl_service_type_support_t
    goal_service_name,
    &options->goal_service_qos
  );
  
  // cancel_goal service
  char cancel_service_name[256];
  snprintf(cancel_service_name, sizeof(cancel_service_name),
           "%s/_action/cancel_goal", action_name);
  ret = rcl_service_init(
    &action_server->impl->cancel_service,
    node,
    type_support->cancel,
    cancel_service_name,
    &options->cancel_service_qos
  );
  
  // get_result service
  char result_service_name[256];
  snprintf(result_service_name, sizeof(result_service_name),
           "%s/_action/get_result", action_name);
  ret = rcl_service_init(
    &action_server->impl->result_service,
    node,
    type_support->result,
    result_service_name,
    &options->result_service_qos
  );
  
  // ★ 创建 2 个 rcl_publisher（feedback, status）
  // feedback publisher
  char feedback_topic_name[256];
  snprintf(feedback_topic_name, sizeof(feedback_topic_name),
           "%s/_action/feedback", action_name);
  ret = rcl_publisher_init(
    &action_server->impl->feedback_publisher,
    node,
    type_support->feedback,
    feedback_topic_name,
    &options->feedback_topic_qos
  );
  
  // status publisher
  char status_topic_name[256];
  snprintf(status_topic_name, sizeof(status_topic_name),
           "%s/_action/status", action_name);
  ret = rcl_publisher_init(
    &action_server->impl->status_publisher,
    node,
    type_support->status,
    status_topic_name,
    &options->status_topic_qos
  );
  
  // ★ 创建 expire_timer（定期清理终态 Goal）
  ret = rcl_timer_init(
    &action_server->impl->expire_timer,
    clock,
    context,
    options->result_timeout.nanoseconds,   // 默认 15 分钟
    NULL,
    allocator
  );
  
  return RCL_RET_OK;
}
```

---

## 二、Client 发送 Goal 完整链路

```cpp
// rclcpp_action/include/rclcpp_action/client.hpp

template<typename ActionT>
class Client : public ClientBase
{
public:
  // ★ 发送目标（异步）
  std::shared_future<typename ClientGoalHandle<ActionT>::SharedPtr>
  async_send_goal(
    const typename ActionT::Goal & goal,
    const SendGoalOptions & options = SendGoalOptions())
  {
    // 1. 生成随机 UUID（16字节 = 128位）
    GoalUUID uuid = generate_goal_id();
    
    // 2. 构建 SendGoal 请求
    auto request = std::make_shared<typename ActionT::Impl::SendGoalService::Request>();
    convert_uuid_to_goal_id(uuid, request->goal_id);
    request->goal = goal;
    
    // 3. 存储回调（feedback, result）
    {
      std::lock_guard<std::mutex> lock(goal_handles_mutex_);
      goal_handles_[uuid] = std::weak_ptr<ClientGoalHandle<ActionT>>();
    }
    pending_goal_responses_[uuid] = {options, promise};
    
    // 4. ★ 通过 rcl_action 发送请求
    int64_t sequence_number;
    rcl_action_send_goal_request(action_client_.get(), request.get(), &sequence_number);
    
    return future;
  }
  
  // ★ 当 goal response 到达（Executor 调用）
  void handle_goal_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> response)
  {
    auto typed_response = std::static_pointer_cast<
      typename ActionT::Impl::SendGoalService::Response>(response);
    
    if (!typed_response->accepted) {
      // Goal 被拒绝
      promise->set_value(nullptr);
      return;
    }
    
    // ★ 创建 ClientGoalHandle
    auto goal_handle = std::make_shared<ClientGoalHandle<ActionT>>(
      goal_info, options.feedback_callback, options.result_callback);
    
    // ★ 立即发起 get_result 请求（不等，先挂起）
    auto get_result_future = async_get_result(goal_handle);
    
    promise->set_value(goal_handle);
  }
};
```

---

## 三、rcl_action_send_goal_request() 内部

```c
// rcl_action/src/rcl_action/action_client.c

rcl_ret_t
rcl_action_send_goal_request(
  const rcl_action_client_t * action_client,
  const void * ros_goal_request,
  int64_t * sequence_number)
{
  // ★ 直接委托给内部的 rcl_client
  return rcl_send_request(
    &action_client->impl->goal_client,
    ros_goal_request,
    sequence_number
  );
  // rcl_send_request → rmw_send_request → DDS DataWriter::write()
}
```

---

## 四、Feedback 发布与接收

```c
// Server 发布 Feedback：
rcl_ret_t
rcl_action_publish_feedback(
  const rcl_action_server_t * action_server,
  void * ros_feedback)
{
  // ★ 直接委托给内部的 rcl_publisher
  return rcl_publish(&action_server->impl->feedback_publisher, ros_feedback, NULL);
}

// Client 接收 Feedback（Executor 调用）：
// rcl_action_take_feedback() → rcl_take() → rmw_take() → DDS DataReader::read()

// rclcpp_action/src/client.cpp
void ClientBase::handle_feedback_message(std::shared_ptr<void> message)
{
  auto feedback_message = std::static_pointer_cast<
    typename ActionT::Impl::FeedbackMessage>(message);
  
  // 从 goal_id 找到对应的 ClientGoalHandle
  GoalUUID goal_id = convert_goal_id(feedback_message->goal_id);
  
  std::lock_guard<std::mutex> lock(goal_handles_mutex_);
  auto it = goal_handles_.find(goal_id);
  if (it != goal_handles_.end()) {
    auto goal_handle = it->second.lock();
    if (goal_handle && goal_handle->feedback_callback_) {
      // ★ 调用用户注册的 feedback 回调
      goal_handle->feedback_callback_(
        goal_handle,
        std::make_shared<typename ActionT::Feedback>(feedback_message->feedback)
      );
    }
  }
}
```

---

## 五、Status Topic 定期发布机制

```c
// rcl_action/src/rcl_action/action_server.c

// status 发布由 Server 的 expire_timer 触发
// 但也可以手动调用（状态变化时）

rcl_ret_t
rcl_action_publish_status(const rcl_action_server_t * action_server)
{
  // 收集所有 GoalHandle 的状态
  action_msgs__msg__GoalStatusArray status_array;
  
  for (size_t i = 0; i < action_server->impl->num_goal_handles; ++i) {
    rcl_action_goal_handle_t * handle = action_server->impl->goal_handles[i];
    
    action_msgs__msg__GoalStatus status;
    status.goal_info = handle->impl->info;
    
    // 转换内部状态码 → action_msgs 状态码
    switch (handle->impl->state) {
      case GOAL_STATE_ACCEPTED:  status.status = action_msgs__msg__GoalStatus__STATUS_ACCEPTED; break;
      case GOAL_STATE_EXECUTING: status.status = action_msgs__msg__GoalStatus__STATUS_EXECUTING; break;
      case GOAL_STATE_CANCELING: status.status = action_msgs__msg__GoalStatus__STATUS_CANCELING; break;
      case GOAL_STATE_SUCCEEDED: status.status = action_msgs__msg__GoalStatus__STATUS_SUCCEEDED; break;
      case GOAL_STATE_CANCELED:  status.status = action_msgs__msg__GoalStatus__STATUS_CANCELED; break;
      case GOAL_STATE_ABORTED:   status.status = action_msgs__msg__GoalStatus__STATUS_ABORTED; break;
    }
    
    append_status(&status_array, &status);
  }
  
  // ★ 通过 rcl_publisher 发布
  return rcl_publish(&action_server->impl->status_publisher, &status_array, NULL);
}
```

---

## 六、Executor 处理 Action 实体的扩展

```cpp
// rclcpp_action/src/server.cpp

// ServerBase 实现 rclcpp::Waitable 接口
// Executor 通过 Waitable 检测 action server 是否有数据

class ServerBase : public rclcpp::Waitable
{
  // ★ 将 action server 的 3个service + expire_timer 添加到 wait_set
  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    // 添加 3 个 service 的 rcl_service
    rcl_wait_set_add_service(wait_set, &goal_service_);
    rcl_wait_set_add_service(wait_set, &cancel_service_);
    rcl_wait_set_add_service(wait_set, &result_service_);
    
    // 添加 expire_timer
    rcl_wait_set_add_timer(wait_set, &expire_timer_);
    
    // 不需要添加 feedback/status publisher（主动发布，不等待）
  }
  
  // ★ 检查哪些实体就绪
  bool is_ready(rcl_wait_set_t * wait_set) override
  {
    // 检查 3 个 service 是否有新请求
    goal_request_ready_ = ...wait_set->services[goal_service_index_]...;
    cancel_request_ready_ = ...;
    result_request_ready_ = ...;
    expire_timer_ready_ = ...;
    
    return goal_request_ready_ || cancel_request_ready_ 
        || result_request_ready_ || expire_timer_ready_;
  }
  
  // ★ 执行就绪的实体
  void execute(std::shared_ptr<void> & data) override
  {
    if (goal_request_ready_)   execute_goal_request_received(action_server_);
    if (cancel_request_ready_) execute_cancel_request_received(action_server_);
    if (result_request_ready_) execute_result_request_received(action_server_);
    if (expire_timer_ready_)   execute_expired_goals(action_server_);
  }
};
```

---

## 七、Goal 结果过期清理

```c
// rcl_action/src/rcl_action/action_server.c

// expire_timer 定期触发，清理终态 Goal（避免内存泄漏）
rcl_ret_t
rcl_action_expire_goals(
  rcl_action_server_t * action_server,
  size_t * num_expired)
{
  rcl_time_point_value_t current_time;
  rcl_clock_get_now(action_server->impl->clock, &current_time);
  
  for (size_t i = 0; i < action_server->impl->num_goal_handles; ) {
    rcl_action_goal_handle_t * goal_handle = action_server->impl->goal_handles[i];
    
    // 只清理已达终态的 Goal
    if (!is_terminal_state(goal_handle->impl->state)) {
      ++i;
      continue;
    }
    
    // 检查是否超过 result_timeout（默认 15 分钟）
    int64_t age_ns = current_time - goal_handle->impl->info.stamp.nanosec;
    if (age_ns > action_server->impl->options.result_timeout.nanoseconds) {
      // ★ 释放 GoalHandle 内存
      rcl_action_goal_handle_fini(goal_handle);
      
      // 从数组中移除
      action_server->impl->goal_handles[i] = 
        action_server->impl->goal_handles[--action_server->impl->num_goal_handles];
      (*num_expired)++;
      
      // 发布更新的状态
      rcl_action_publish_status(action_server);
    } else {
      ++i;
    }
  }
}
```

---

## 八、使用示例：完整 Action Server/Client

```cpp
// Action Server：
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_interfaces/action/Fibonacci.hpp"

class FibonacciServer : public rclcpp::Node {
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
  
  FibonacciServer() : Node("fibonacci_server") {
    server_ = rclcpp_action::create_server<Fibonacci>(
      this, "fibonacci",
      // handle_goal
      [this](const GoalUUID &, auto goal) {
        RCLCPP_INFO(get_logger(), "Received goal: order=%d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // handle_cancel
      [this](auto goal_handle) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // handle_accepted
      [this](auto goal_handle) {
        // ★ 在新线程执行（避免阻塞 Executor）
        std::thread{std::bind(&FibonacciServer::execute, this, goal_handle)}.detach();
      }
    );
  }
  
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & seq = feedback->partial_sequence;
    seq = {0, 1};
    
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    
    for (int i = 2; i < goal->order; ++i) {
      // ★ 检查取消请求
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = seq;
        goal_handle->canceled(result);
        return;
      }
      
      seq.push_back(seq[i-1] + seq[i-2]);
      goal_handle->publish_feedback(feedback);  // ★ 发布 Feedback
      loop_rate.sleep();
    }
    
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = seq;
    goal_handle->succeed(result);  // ★ 报告成功
  }
};
```
