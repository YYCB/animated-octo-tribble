# Topic 通信（三）：接收路径完整源码追踪

## 一、Layer 4：FastDDS 收到数据

### DataReader 的监听器触发

```cpp
// FastDDS: src/cpp/fastdds/subscriber/DataReaderImpl.cpp

// 当 RTPS 层收到 DATA 子消息时触发
void DataReaderImpl::on_new_data_message(
  rtps::RTPSReader * reader,
  const rtps::CacheChange_t * const change)
{
  // ── 1. 存入读历史（ReaderHistory）────────────────────
  reader_history_->received_change(change, ...);
  
  // ── 2. 检查是否在 DDS WaitSet 中等待 ─────────────────
  // 如果有 WaitSet::wait() 在等待新数据：
  status_condition_->set_status(StatusMask::data_available(), true);
  
  // ── 3. 唤醒等待中的 WaitSet ──────────────────────────
  // 这对应 rmw_wait() 中的 condition_variable::notify_all()
  wake_up_waiters();
  
  // ── 4. 触发 DataReaderListener（如果设置了监听器模式）
  if (listener_) {
    listener_->on_data_available(data_reader_);
  }
}
```

---

## 二、rmw_wait() 被唤醒（Layer 3）

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_wait.cpp

// 当 FastDDS 数据到达时，WaitSet 的条件变量被 notify
// rmw_wait() 的 wait_for() 返回

rmw_ret_t rmw_wait(
  rmw_subscriptions_t * subscriptions,
  ...
  const rmw_time_t * wait_timeout)
{
  auto custom_wait_set = static_cast<CustomWaitSet *>(wait_set->data);
  
  // ★ 阻塞等待（被 FastDDS 数据到达事件唤醒）
  std::unique_lock<std::mutex> lock(custom_wait_set->mutex);
  custom_wait_set->conditionVariable.wait_for(
    lock,
    std::chrono::nanoseconds(timeout_ns),
    [&]() {
      return custom_wait_set->hasData();  // 任意就绪实体
    }
  );
  
  // ★ 清理未就绪实体（置 NULL）
  // wait_set 中各 subscription 句柄：
  //   - 有数据可读 → 保持非 NULL（调用者知道此订阅者就绪）
  //   - 无数据可读 → 置 NULL（调用者跳过此订阅者）
  
  for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
    auto sub_info = static_cast<CustomSubscriberInfo *>(
      subscriptions->subscribers[i]);
    
    bool has_data = sub_info->data_reader_->get_unread_count() > 0;
    if (!has_data) {
      subscriptions->subscribers[i] = nullptr;  // 置 NULL = 未就绪
    }
  }
  
  return RMW_RET_OK;
}
```

---

## 三、rcl_wait() 返回（Layer 2）

```c
// rcl/src/rcl/wait.c

// rmw_wait() 返回后：
// 1. 检查定时器就绪状态（DDS 不知道 ROS 定时器）
// 2. 返回 RCL_RET_OK（或 RCL_RET_TIMEOUT）

// wait_set->subscriptions[i] != NULL 表示第 i 个订阅者有数据
// wait_set->timers[i] != NULL 表示第 i 个定时器已到期
// 等等...
```

---

## 四、Executor::spin_once()（Layer 1 - Executor）

```cpp
// rclcpp/src/rclcpp/executor.cpp

void Executor::spin_once(std::chrono::nanoseconds timeout)
{
  if (!rclcpp::ok(this->context_) || !spinning.load()) return;
  
  AnyExecutable any_executable;
  if (get_next_executable(any_executable, timeout)) {
    execute_any_executable(any_executable);
  }
}

// get_next_executable 内部调用了 rcl_wait()，然后找到就绪的 subscription
// execute_any_executable 调用 execute_subscription()
```

---

## 五、execute_subscription()（Layer 1 - rclcpp）

```cpp
// rclcpp/src/rclcpp/executor.cpp

void Executor::execute_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;
  
  if (subscription->is_serialized()) {
    // ── 特殊路径：用户订阅了 SerializedMessage（原始字节）──
    auto serialized_msg = subscription->create_serialized_message();
    bool taken = false;
    subscription->take_serialized(serialized_msg.get(), message_info, taken);
    if (taken) {
      subscription->handle_serialized_message(serialized_msg, message_info);
    }
    subscription->return_serialized_message(std::move(serialized_msg));
    
  } else {
    // ── 标准路径（最常用）────────────────────────────────
    // 1. 创建消息缓冲区（使用节点的内存分配器）
    std::shared_ptr<void> message = subscription->create_message();
    
    // 2. 从 DDS 取消息（同时反序列化）
    bool taken = false;
    subscription->take_type_erased(message.get(), message_info, taken);
    
    if (taken) {
      // 3. 调用用户回调
      subscription->handle_message(message, message_info);
    } else {
      // 消息不存在（spurious wakeup）
    }
    
    // 4. 归还消息缓冲区
    subscription->return_message(std::move(message));
  }
}
```

---

## 六、take_type_erased()：从 DDS 取消息并反序列化

```cpp
// rclcpp/src/rclcpp/subscription_base.cpp

bool SubscriptionBase::take_type_erased(
  void * message_out,
  rclcpp::MessageInfo & message_info)
{
  // 调用 rcl_take()
  rcl_ret_t ret = rcl_take(
    subscription_handle_.get(),        // rcl_subscription_t*
    message_out,                        // 输出：反序列化后的消息（C 结构体）
    &message_info.get_rmw_message_info(), // 输出：消息元数据
    nullptr                             // allocation
  );
  
  if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
    return false;  // 没有数据（spurious wakeup）
  }
  if (RCL_RET_OK != ret) {
    throw std::runtime_error("failed to take: " + std::string(rcl_get_error_string().str));
  }
  return true;
}
```

---

## 七、rcl_take() → rmw_take_with_info()（Layer 2 & 3）

```c
// rcl/src/rcl/subscription.c

rcl_ret_t
rcl_take(
  const rcl_subscription_t * subscription,
  void * ros_message,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  bool taken = false;
  
  // ★ 委托给 rmw（无额外逻辑）
  rmw_ret_t rmw_ret = rmw_take_with_info(
    subscription->impl->rmw_handle,  // rmw_subscription_t*
    ros_message,                      // 输出：C 结构体（已反序列化）
    &taken,
    message_info,
    allocation
  );
  
  if (!taken) return RCL_RET_SUBSCRIPTION_TAKE_FAILED;
  return rcl_convert_rmw_ret_to_rcl_ret(rmw_ret);
}
```

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_take.cpp

rmw_ret_t rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  auto info = static_cast<CustomSubscriberInfo *>(subscription->data);
  
  // ── 获取序列化数据 ────────────────────────────────────
  // 从 DataReader 的 ReaderHistory 取出最旧的未读消息
  eprosima::fastrtps::SampleInfo_t sample_info;
  
  // 借用 loaned sample（FastDDS 管理的内存）
  void * untyped_ros_message = nullptr;
  bool is_loaned = info->data_reader_->loan_sample(
    untyped_ros_message,
    &sample_info
  );
  
  if (!sample_info.sampleKind == ALIVE || !is_loaned) {
    *taken = false;
    return RMW_RET_OK;
  }
  
  // ── 填充消息元数据 ────────────────────────────────────
  if (message_info) {
    message_info->source_timestamp = sample_info.sourceTimestamp.to_ns();
    message_info->received_timestamp = sample_info.receptionTimestamp.to_ns();
    message_info->publisher_gid = convert_gid(sample_info.iHandle);
    message_info->from_intra_process = false;
  }
  
  // ── ★ 反序列化（CDR → C 结构体）──────────────────────
  // 使用 rosidl_typesupport 的反序列化函数
  auto ret = info->type_support_->deserializeROSmessage(
    untyped_ros_message,  // 输入：CDR 字节（在 DDS 内存中）
    ros_message           // 输出：填充到调用者提供的 C 结构体
  );
  
  // 释放借用的 DDS 内存
  info->data_reader_->return_loan(&untyped_ros_message, &sample_info);
  
  *taken = true;
  return ret ? RMW_RET_OK : RMW_RET_ERROR;
}
```

---

## 八、反序列化细节（CDR → C 结构体）

```cpp
// rmw_fastrtps_shared_cpp/src/TypeSupport.cpp

bool TypeSupport::deserializeROSmessage(
  SerializedPayload_t * payload,
  void * ros_message)
{
  // 跳过 encapsulation header（4 字节）
  const uint8_t * data = payload->data + 4;
  size_t data_length = payload->length - 4;
  
  // 检查字节序（encapsulation header 中指定）
  bool is_little_endian = (payload->data[1] == 0x01);
  
  // 创建 FastCDR 反序列化器
  eprosima::fastcdr::FastBuffer cdr_buffer(
    reinterpret_cast<char *>(const_cast<uint8_t *>(data)),
    data_length);
  eprosima::fastcdr::Cdr deser(
    cdr_buffer,
    is_little_endian ? eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS
                     : eprosima::fastcdr::Cdr::BIG_ENDIANNESS);
  
  // ★ 递归反序列化所有字段
  deserialize_fields(ros_message, message_members_, deser);
  
  return true;
}

// 字段递归反序列化示例（std_msgs::msg::String）：
// 字段：data（string 类型）
// CDR 布局：[length(4字节)][data bytes][padding]
//
// deser.deserialize(string_length);    // 读4字节: 5
// deser.deserializeArray(string_data, string_length);  // 读5字节: "hello"
// 
// 结果：ros_message->data.data = "hello"; ros_message->data.size = 5;
```

---

## 九、handle_message() → 用户回调

```cpp
// rclcpp/include/rclcpp/subscription.hpp

template<typename MessageT, typename AllocatorT>
void Subscription<MessageT, AllocatorT>::handle_message(
  std::shared_ptr<void> & message,
  const rclcpp::MessageInfo & message_info)
{
  // 类型转换（从 void* 到具体类型）
  auto typed_message = std::static_pointer_cast<MessageT>(message);
  
  // ★ 通过 AnySubscriptionCallback 分发到用户回调
  any_callback_.dispatch(typed_message, message_info);
}

// AnySubscriptionCallback::dispatch 内部：
// 根据存储的回调类型选择调用方式：
//   ConstRefCallback: user_cb(*typed_message)
//   SharedPtrCallback: user_cb(typed_message)
//   SharedPtrWithInfoCallback: user_cb(typed_message, message_info)
//   MoveCallback: user_cb(std::move(*typed_message))
```

---

## 十、进程内通信的接收路径

```cpp
// 与进程外通信的关键区别：

// 1. IntraProcessManager 将消息存入订阅者的 ring buffer
//    并触发订阅者的 guard condition

// 2. Executor 检测到 guard condition 触发
//    调用 Subscription::execute_intra_process()

// 3. 从 ring buffer 取出 unique_ptr（无拷贝！）

// 4. 调用 handle_intra_process_message()
//    直接将 unique_ptr 传给用户回调（shared_ptr 版本）
//    或转换后以 const ref 调用（const ref 版本，此时有一次解引用）

// 进程内通信的用户感知：
//   message_info.from_intra_process == true
//   source_timestamp 可能为 0（由发布者设置）
```

---

## 十一、完整接收路径总结

```
FastDDS RTPS 层收到 UDP 数据包
         │
         ▼
解析 RTPS DATA 子消息，提取 SerializedPayload
         │
         ▼
DataReader::on_data_available() 触发
         │
         ▼ (通过 conditionVariable::notify_all)
rmw_wait() 中的 wait_for() 返回
         │
         ▼ (置 NULL 标记未就绪实体)
rcl_wait() 返回，wait_set.subscriptions[i] != NULL
         │
         ▼
Executor::get_next_ready_executable() 找到就绪订阅
         │
         ▼
Executor::execute_subscription()
  → subscription->create_message()         # 分配 C++ 消息缓冲区
  → subscription->take_type_erased()       # 调用 rcl_take() → rmw_take()
      → FastDDS DataReader::read()         # 从 ReaderHistory 取数据
      → CDR 反序列化 → 填充 C 结构体
  → subscription->handle_message()         # 类型转换
      → AnySubscriptionCallback::dispatch()  # 调用用户回调
         │
         ▼
用户回调执行：callback(const Msg & msg)
```
