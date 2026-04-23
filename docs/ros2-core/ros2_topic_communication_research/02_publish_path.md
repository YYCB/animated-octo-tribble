# Topic 通信（二）：发布路径完整源码追踪

## 一、Layer 0：用户代码

```cpp
// 用户写的代码
auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
std_msgs::msg::String msg;
msg.data = "Hello, world!";
pub->publish(msg);
```

---

## 二、Layer 1：rclcpp::Publisher<T>::publish()

```cpp
// rclcpp/include/rclcpp/publisher.hpp

template<typename MessageT, typename AllocatorT>
void Publisher<MessageT, AllocatorT>::publish(const MessageT & msg)
{
  // ── 分支1：无进程内通信 ─────────────────────────────────
  if (!intra_process_is_enabled_) {
    this->do_inter_process_publish(msg);
    return;
  }
  
  // ── 分支2：有进程内通信 ─────────────────────────────────
  // 检查是否同时有进程外订阅者
  uint64_t intra_process_publisher_id = get_intra_process_publisher_id();
  
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    // IntraProcessManager 已销毁，降级到进程外
    this->do_inter_process_publish(msg);
    return;
  }
  
  bool inter_process_publish_needed =
    ipm->get_subscription_count(intra_process_publisher_id) <
    this->get_subscription_count();
  
  if (inter_process_publish_needed) {
    // 两端都要发送：先进程外（用原始引用），再进程内（用拷贝/移动）
    this->do_inter_process_publish(msg);
    auto ptr = message_allocator_->allocate(1);
    message_allocator_->construct(ptr, msg);  // 拷贝
    std::unique_ptr<MessageT, MessageDeleter> unique_msg(ptr, ...);
    this->do_intra_process_publish(std::move(unique_msg));
  } else {
    // 只有进程内订阅者：无需序列化
    auto ptr = message_allocator_->allocate(1);
    message_allocator_->construct(ptr, msg);
    std::unique_ptr<MessageT, MessageDeleter> unique_msg(ptr, ...);
    this->do_intra_process_publish(std::move(unique_msg));
  }
}
```

---

## 三、Layer 1：do_inter_process_publish()（序列化）

```cpp
// rclcpp/include/rclcpp/publisher.hpp

template<typename MessageT, typename AllocatorT>
void Publisher<MessageT, AllocatorT>::do_inter_process_publish(const MessageT & msg)
{
  // ── 路径1：借贷消息（iceoryx 零拷贝）──────────────────
  if (this->can_loan_messages()) {
    void * loaned_msg = nullptr;
    this->borrow_loaned_message(loaned_msg);
    // 将消息内容拷贝到借贷缓冲区（共享内存）
    *reinterpret_cast<MessageT *>(loaned_msg) = msg;
    this->publish_loaned_message(loaned_msg);
    return;
  }
  
  // ── 路径2：标准序列化路径（CDR）───────────────────────
  // 获取或分配序列化消息缓冲区
  auto serialized_msg = this->create_serialized_message();
  
  // ★ 序列化：MessageT → rcl_serialized_message_t（CDR 字节）
  auto ret = this->serialize_message(&msg, serialized_msg.get());
  if (!ret) {
    throw std::runtime_error("failed to serialize message");
  }
  
  // ★★ 关键调用：rcl_publish_serialized_message()
  rcl_ret_t status = rcl_publish_serialized_message(
    this->get_publisher_handle().get(),  // rcl_publisher_t*
    serialized_msg.get(),                // rcl_serialized_message_t*
    nullptr                              // allocation（NULL=使用默认分配器）
  );
  
  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish message");
  }
  
  // 归还序列化缓冲区（复用，避免频繁 malloc）
  this->return_serialized_message(std::move(serialized_msg));
}
```

### 序列化细节：serialize_message()

```cpp
// rclcpp/include/rclcpp/serialization.hpp

template<typename MessageT>
bool serialize_message(const MessageT * msg, rcl_serialized_message_t * serialized_msg)
{
  // 调用 rmw_serialize（底层使用 rosidl_typesupport 的序列化函数）
  rmw_ret_t ret = rmw_serialize(
    msg,
    rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
    serialized_msg
  );
  return ret == RMW_RET_OK;
}

// rmw_serialize 内部（rmw_fastrtps_shared_cpp）：
rmw_ret_t rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  // 获取 FastDDS 特定的类型支持（包含 CDR 序列化函数）
  const rosidl_typesupport_introspection_cpp::MessageMembers * members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      type_support->data);
  
  // ★ 使用 FastCDR 序列化
  eprosima::fastcdr::Cdr::Endianness endianness =
    eprosima::fastcdr::Cdr::DEFAULT_ENDIAN;  // 本机字节序
  
  // 先计算序列化大小（避免重分配）
  size_t data_length = get_serialized_size(ros_message, members);
  
  // 分配/扩容缓冲区
  if (serialized_message->buffer_capacity < data_length + 4) {
    rmw_serialized_message_resize(serialized_message, data_length + 4);
  }
  
  // 写 encapsulation header（4 字节）
  serialized_message->buffer[0] = 0x00;  // scheme ID高字节
  serialized_message->buffer[1] = (endianness == eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS) ? 0x01 : 0x00;
  serialized_message->buffer[2] = 0x00;  // 选项
  serialized_message->buffer[3] = 0x00;
  
  // 创建 FastCDR 序列化器，写数据
  eprosima::fastcdr::FastBuffer cdr_buffer(
    reinterpret_cast<char *>(&serialized_message->buffer[4]),
    data_length);
  eprosima::fastcdr::Cdr ser(cdr_buffer, endianness);
  
  // 递归序列化所有字段
  serialize_fields(ros_message, members, ser);
  
  serialized_message->buffer_length = data_length + 4;
  return RMW_RET_OK;
}
```

---

## 四、Layer 2：rcl_publish_serialized_message()

```c
// rcl/src/rcl/publisher.c

rcl_ret_t
rcl_publish_serialized_message(
  const rcl_publisher_t * publisher,
  const rcl_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  // 参数验证
  if (!rcl_publisher_is_valid(publisher)) return RCL_RET_PUBLISHER_INVALID;
  
  // ★ 直接委托给 rmw（无任何额外逻辑）
  rmw_ret_t ret = rmw_publish_serialized_message(
    publisher->impl->rmw_handle,  // rmw_publisher_t*
    serialized_message,
    allocation
  );
  
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}
```

---

## 五、Layer 3：rmw_publish_serialized_message()（FastDDS 实现）

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_publish.cpp

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  
  // 获取 FastDDS 自定义发布者
  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  
  // ★ 将 rmw 序列化消息包装为 FastDDS 可以理解的类型
  // SerializedPayload_t 是 FastDDS 的序列化载荷容器
  eprosima::fastrtps::rtps::SerializedPayload_t payload;
  payload.max_size = serialized_message->buffer_capacity;
  payload.length = static_cast<uint32_t>(serialized_message->buffer_length);
  payload.data = serialized_message->buffer;
  
  // ★★ 调用 FastDDS DataWriter::write() 的底层接口
  // 注意：这里直接使用已序列化的载荷，跳过 FastDDS 自己的序列化
  bool published = info->data_writer_->write_w_timestamp(
    &payload,                         // 序列化载荷（已是 CDR 格式）
    HANDLE_NIL,                       // 实例句柄（对 Topic 类型无关）
    eprosima::fastrtps::c_TimeInvalid // 时间戳（由 DDS 层自动填充）
  );
  
  if (!published) {
    RMW_SET_ERROR_MSG("failed to publish message");
    return RMW_RET_ERROR;
  }
  
  return RMW_RET_OK;
}
```

---

## 六、Layer 4：FastDDS DataWriter::write()

```cpp
// FastDDS: src/cpp/fastdds/publisher/DataWriterImpl.cpp

ReturnCode_t DataWriterImpl::write(
  void * data,
  const InstanceHandle_t & handle)
{
  // ── 1. 检查 QoS 约束（Liveliness, Deadline）──────────
  // 更新 liveliness 计时器（防止 liveliness_lost 事件）
  liveliness_manager_->assert_liveliness(liveliness_lease_duration_ms_);
  
  // ── 2. 序列化（如果传入的是原始数据，需要序列化）────────
  // 对于 ROS2，传入的是已序列化的 SerializedPayload_t，跳过此步骤
  
  // ── 3. 向 RTPS 层提交数据 ────────────────────────────
  rtps_writer_->new_change_w_key(
    ALIVE,           // 状态：新数据（vs DISPOSE/UNREGISTER）
    data,
    handle
  );
  
  return RETCODE_OK;
}
```

### RTPS 层处理（StatefulWriter）

```cpp
// FastDDS: src/cpp/rtps/writer/StatefulWriter.cpp

void StatefulWriter::new_change(SequenceNumber_t & seqNum, CacheChange_t * change)
{
  // ── 1. 分配序列号 ─────────────────────────────────────
  // seqNum 是全局递增的（用于可靠传输的 NACK 和 HEARTBEAT）
  seqNum = next_sequence_number_++;
  
  // ── 2. 存入写历史（WriterHistory）────────────────────
  // WriterHistory 是一个环形缓冲区，保存最近 depth 条消息
  // 对于 TransientLocal，还会保存给"晚到"的订阅者
  writer_history_->add_change(change);
  
  // ── 3. 发送给所有匹配的 ReaderProxy ──────────────────
  for (auto & reader_proxy : matched_readers_) {
    if (reader_proxy->is_reliable()) {
      // 可靠模式：将消息加入待确认队列，启动重传定时器
      reader_proxy->add_unacked_change(change);
    } else {
      // 尽力模式：直接发送，不等确认
      reader_proxy->set_change_to_status(change->sequenceNumber, ACKNOWLEDGED);
    }
  }
  
  // ── 4. 实际发送（通过异步发送线程或立即发送）──────────
  if (asynchronous_mode_) {
    // 唤醒异步发送线程
    async_writer_->wake_up(this);
  } else {
    // 同步发送：立即调用
    send_any_unsent_changes();
  }
}
```

### 发送到网络

```cpp
void StatefulWriter::send_any_unsent_changes()
{
  for (auto & reader_proxy : matched_readers_) {
    CacheChange_t * change = reader_proxy->next_unsent_change();
    while (change) {
      // 构建 RTPS DATA 子消息
      CDRMessage_t msg(change->serializedPayload.length + 100);
      RTPSMessageCreator::addHeader(&msg, m_guid.guidPrefix);
      RTPSMessageCreator::addSubmessageInfoTS(&msg, source_timestamp, is_bigendian);
      RTPSMessageCreator::addSubmessageData(&msg, change, reader_proxy->reader_id());
      
      // ★ 通过 UDPTransport / TCPTransport / SharedMemTransport 发送
      for (auto & locator : reader_proxy->unicast_locator_list()) {
        mp_sender_resources->send(msg.buffer, msg.length, locator);
      }
      
      reader_proxy->mark_as_sent(change->sequenceNumber);
      change = reader_proxy->next_unsent_change();
    }
  }
}
```

---

## 七、HEARTBEAT / ACKNACK（可靠传输确认机制）

```
发布者（ReliableWriter）        订阅者（ReliableReader）
     │                                │
     ├── HEARTBEAT(firstSN=1,lastSN=5)──────►│  "我有1~5号消息"
     │                                │
     │               ◄──── ACKNACK(base=3,bits=110) ──┤  "我只有1~2，缺3~4"
     │                                │
     ├── DATA(seq=3)──────────────────►│  重传第3条
     ├── DATA(seq=4)──────────────────►│  重传第4条
     │                                │
     │               ◄──── ACKNACK(base=6,bits=000) ──┤  "1~5都收到了"
     │                                │

HEARTBEAT 周期：默认 1 秒（可通过 XML 配置）
NACK 响应延迟：默认 0（立即响应）
最大重传次数：无限（直到确认或 DataWriter 销毁）
```

---

## 八、异步发布 vs 同步发布

```xml
<!-- FastDDS XML 配置：控制同步/异步模式 -->
<data_writer>
  <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
  <qos>
    <publish_mode>
      <!-- SYNCHRONOUS_PUBLISH_MODE：在调用者线程发送（默认） -->
      <!-- ASYNCHRONOUS_PUBLISH_MODE：使用专用线程批量发送 -->
      <kind>ASYNCHRONOUS_PUBLISH_MODE</kind>
    </publish_mode>
  </qos>
</data_writer>
```

```
同步模式（SYNCHRONOUS）：
  pub->publish(msg) → write() → send() → return
  延迟低，但 publish() 会阻塞到数据发送完成

异步模式（ASYNCHRONOUS）：
  pub->publish(msg) → 放入队列 → return（立即返回）
  ↓（异步发送线程）
  从队列取出 → send() → 继续
  吞吐量高，但延迟略增
```
