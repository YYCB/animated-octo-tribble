# rclcpp（三）：Publisher/Subscription 模板实现与 TypeAdapter

## 一、Publisher<T>::publish() 完整实现

```cpp
// rclcpp/include/rclcpp/publisher.hpp

template<typename MessageT, typename AllocatorT>
class Publisher : public rclcpp::PublisherBase
{
public:
  // ── 重载 1：发布 const 引用（最常用，会拷贝）─────────────
  void publish(const MessageT & msg)
  {
    // 检查是否有进程内通信的订阅者
    if (!intra_process_is_enabled_) {
      // 纯进程外路径：序列化后发送
      return this->do_inter_process_publish(msg);
    }
    
    // 检查是否有进程外订阅者
    bool has_inter_process_subscribers =
      this->get_subscription_count() > this->get_intra_process_subscription_count();
    
    if (has_inter_process_subscribers) {
      // 既有进程内订阅者又有进程外订阅者
      // 必须先拷贝，因为进程内传递会转移所有权
      auto ptr = MessageAllocTraits::allocate(*message_allocator_, 1);
      MessageAllocTraits::construct(*message_allocator_, ptr, msg);
      auto unique_msg = std::unique_ptr<MessageT, MessageDeleter>(ptr, *message_allocator_);
      
      // 进程外发送（使用原始引用）
      this->do_inter_process_publish(msg);
      
      // 进程内发送（转移 unique_ptr 所有权，零拷贝）
      this->do_intra_process_publish(std::move(unique_msg));
    } else {
      // 只有进程内订阅者：直接转移
      auto ptr = MessageAllocTraits::allocate(*message_allocator_, 1);
      MessageAllocTraits::construct(*message_allocator_, ptr, msg);
      auto unique_msg = std::unique_ptr<MessageT, MessageDeleter>(ptr, *message_allocator_);
      this->do_intra_process_publish(std::move(unique_msg));
    }
  }
  
  // ── 重载 2：发布 unique_ptr（零拷贝，推荐高性能场景）────
  void publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    if (!intra_process_is_enabled_) {
      // 进程外：需要序列化（此时 unique_ptr 被解引用）
      return this->do_inter_process_publish(*msg);
    }
    
    bool has_inter_process_subscribers = ...;
    
    if (has_inter_process_subscribers) {
      // 需要发送到进程外：先序列化原始内容
      this->do_inter_process_publish(*msg);
      // 再以零拷贝方式传给进程内订阅者
      this->do_intra_process_publish(std::move(msg));
    } else {
      // 只有进程内订阅者：直接转移 unique_ptr（真正的零拷贝）
      this->do_intra_process_publish(std::move(msg));
    }
  }

private:
  // 进程外发送（序列化到 DDS）
  void do_inter_process_publish(const MessageT & msg)
  {
    // 序列化
    auto serialized_msg = this->create_serialized_message();
    auto ret = this->serialize_message(&msg, serialized_msg.get());
    if (!ret) throw std::runtime_error("failed to serialize message");
    
    // 调用 rcl_publish
    rcl_ret_t rcl_ret = rcl_publish(
      this->get_publisher_handle().get(),
      serialized_msg.get(),
      nullptr
    );
  }
  
  // 进程内发送（调用 IntraProcessManager）
  void do_intra_process_publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto ipm = weak_ipm_.lock();
    if (!ipm) return;
    
    ipm->do_intra_process_publish(
      intra_process_publisher_id_,
      std::move(msg),
      message_allocator_
    );
  }
};
```

---

## 二、序列化机制（rosidl_typesupport）

### 2.1 类型支持体系

```
MessageT（C++ 结构体，如 std_msgs::msg::String）
    │
    ├── rosidl_message_type_support_t（运行时类型描述）
    │   ├── typesupport_introspection_cpp（内省，字段名/类型）
    │   └── typesupport_fastrtps_cpp（FastDDS CDR 序列化）
    │       └── message_type_support.h（序列化/反序列化函数）
    │
    └── IDL 定义（.msg 文件 → 编译生成 C++ 代码）
        std_msgs/msg/string.hpp
        std_msgs/msg/string.h（C 语言版本）
```

### 2.2 获取类型支持

```cpp
// 编译期获取（推荐，零运行时开销）
const rosidl_message_type_support_t * ts =
  rosidl_typesupport_cpp::get_message_type_support_handle<std_msgs::msg::String>();

// 或使用宏（等价）
const rosidl_message_type_support_t * ts =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

// 运行时按名称查找（GenericPublisher 使用）
auto ts = rclcpp::get_typesupport_handle(
  "std_msgs/msg/String",
  "rosidl_typesupport_cpp",
  ts_library_handle
);
```

### 2.3 序列化流程（CDR 编码）

```cpp
// Publisher::serialize_message() 内部

// 1. 调用 rosidl_typesupport 的序列化函数
bool serialize_message(
  const MessageT * msg,
  rcl_serialized_message_t * serialized_msg)
{
  // type_support_handle 中包含具体的序列化函数指针
  auto ret = rmw_serialize(
    msg,
    message_ts_,             // rosidl_message_type_support_t
    serialized_msg           // 输出：CDR 字节流
  );
  return ret == RMW_RET_OK;
}

// rmw_serialize 内部（以 FastDDS 为例）：
// 1. 使用 FastCDR 库将 C++ 结构体序列化为字节流
// 2. 格式：ENCAPSULATION(4字节) + CDR序列化数据

// CDR（Common Data Representation）格式：
// ┌────────────────────────────────────────────────────┐
// │ Encapsulation ID (2 bytes): 0x00 0x01 (little-end) │
// │ Encapsulation options (2 bytes): 0x00 0x00         │
// ├────────────────────────────────────────────────────┤
// │ 字符串长度 (4 bytes): 0x06 0x00 0x00 0x00 (= 6)   │
// │ 字符串内容 (6 bytes): "hello\0"                    │
// └────────────────────────────────────────────────────┘
```

---

## 三、TypeAdapter（Galactic+ 新特性）

TypeAdapter 允许用户在**自定义类型**和**ROS 消息类型**之间透明转换，避免不必要的转换开销。

### 3.1 使用场景

```cpp
// 场景：相机节点使用 cv::Mat，但需要发布 sensor_msgs::msg::Image
// 没有 TypeAdapter 时（低效）：
cv::Mat frame;
sensor_msgs::msg::Image ros_image;
cv_bridge::CvImage cv_image;
cv_image.image = frame;
cv_image.toImageMsg(ros_image);  // 额外拷贝！
pub->publish(ros_image);

// 有 TypeAdapter 后（高效）：
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(cv::Mat, sensor_msgs::msg::Image)
// 直接发布 cv::Mat，框架自动转换
pub->publish(frame);
```

### 3.2 定义 TypeAdapter

```cpp
// 定义 TypeAdapter：cv::Mat ↔ sensor_msgs::msg::Image

template<>
struct rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = cv::Mat;
  using ros_message_type = sensor_msgs::msg::Image;

  // 自定义类型 → ROS 消息（进程外通信时序列化前调用）
  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    cv_bridge::CvImage bridge;
    bridge.image = source;
    bridge.encoding = "bgr8";
    bridge.toImageMsg(destination);
  }

  // ROS 消息 → 自定义类型（接收时反序列化后调用）
  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    auto bridge = cv_bridge::toCvCopy(source, "bgr8");
    destination = bridge->image;
  }
};

// 使用宏注册
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(cv::Mat, sensor_msgs::msg::Image)

// 创建适配 TypeAdapter 的发布者
auto pub = node->create_publisher<
  rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>>(
  "image_raw", 10);

// 直接发布 cv::Mat（零额外代码）
pub->publish(cv_mat_frame);
```

### 3.3 TypeAdapter 的零拷贝优势

```
进程内通信 + TypeAdapter：

发布者                              订阅者
 cv::Mat ──[TypeAdapter转换]──► ros_image
                │
                └─[进程内通信？]
                    YES → 直接传 cv::Mat unique_ptr（不需要转换！）
                    NO  → 转换为 ros_image → CDR 序列化 → DDS 发送

// 框架在进程内通信时智能跳过转换，直接传递原始类型
```

---

## 四、Subscription<T> 回调执行

### 4.1 execute_subscription()（Executor 中调用）

```cpp
// rclcpp/src/rclcpp/executor.cpp

void Executor::execute_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;
  
  if (subscription->is_serialized()) {
    // 接收序列化消息（用户订阅 SerializedMessage 时）
    auto serialized_msg = subscription->create_serialized_message();
    bool taken = false;
    subscription->take_serialized(serialized_msg.get(), message_info, taken);
    if (taken) {
      subscription->handle_serialized_message(serialized_msg, message_info);
    }
    subscription->return_serialized_message(std::move(serialized_msg));
    
  } else if (subscription->can_loan_messages()) {
    // 零拷贝借贷（iceoryx/shared memory 路径）
    void * loaned_msg = subscription->borrow_loaned_message();
    bool taken = false;
    subscription->take_loaned_message(loaned_msg, message_info, taken);
    if (taken) {
      subscription->handle_loaned_message(loaned_msg, message_info);
    }
    subscription->return_loaned_message(loaned_msg);
    
  } else {
    // 标准路径：分配消息，take，反序列化，调用回调
    std::shared_ptr<void> message = subscription->create_message();
    bool taken = false;
    subscription->take_type_erased(message.get(), message_info, taken);
    if (taken) {
      subscription->handle_message(message, message_info);
    }
    subscription->return_message(std::move(message));
  }
}
```

### 4.2 handle_message() → 调用用户回调

```cpp
// rclcpp/include/rclcpp/subscription.hpp

template<typename MessageT, typename AllocatorT>
void Subscription<MessageT, AllocatorT>::handle_message(
  std::shared_ptr<void> & message,
  const rclcpp::MessageInfo & message_info)
{
  // 类型转换：void* → MessageT*
  auto typed_message = std::static_pointer_cast<MessageT>(message);
  
  // 调用 AnySubscriptionCallback::dispatch()
  // 根据用户注册的回调类型选择正确的调用方式：
  //   const ref → callback(*typed_message)
  //   shared_ptr → callback(typed_message)
  //   move → callback(std::move(*typed_message))
  any_callback_.dispatch(typed_message, message_info);
}
```

---

## 五、GenericPublisher 和 GenericSubscription

允许在运行时（不是编译时）指定消息类型，用于：
- rosbag2 录制任意 topic
- 通用转发/桥接节点
- 动态类型系统

```cpp
// rclcpp/include/rclcpp/generic_publisher.hpp

class GenericPublisher : public rclcpp::PublisherBase
{
public:
  GenericPublisher(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const std::string & topic_name,
    const std::string & topic_type,  // 字符串，如 "std_msgs/msg/String"
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options = {});
  
  // 发布序列化消息（调用者负责正确序列化）
  void publish(
    const rcl_serialized_message_t & serialized_message);
  
  void publish(
    const rclcpp::SerializedMessage & serialized_message);
};
```

---

## 六、消息借贷接口（Loaned Messages，Iron+）

用于 iceoryx 共享内存零拷贝：

```cpp
// 借贷一个消息缓冲区（由 DDS 分配，在共享内存中）
auto loaned_msg = pub->borrow_loaned_message();
loaned_msg.get().data = 42;

// 发布（无需拷贝，直接告知 DDS "这块内存已写入"）
pub->publish(std::move(loaned_msg));

// 底层实现：
// 1. rmw_borrow_loaned_message() → DDS 在共享内存分配缓冲区
// 2. 用户填写数据
// 3. rmw_publish_loaned_message() → DDS 通知订阅者可读
// 4. 订阅者 rmw_take_loaned_message() → 获取同一块内存的引用
// 5. 用户读完后 rmw_return_loaned_message() → 归还内存
```

---

## 七、rclcpp 回调分类

| 回调类型 | 触发时机 | 注册方式 |
|---------|---------|---------|
| 订阅回调 | 收到新消息 | `create_subscription(callback)` |
| 定时器回调 | 定时触发 | `create_wall_timer(callback)` |
| 服务回调 | 收到服务请求 | `create_service(callback)` |
| 客户端回调 | 收到服务响应（async） | `async_send_request(callback)` |
| 参数回调 | 参数被修改 | `add_on_set_parameters_callback()` |
| Waitable 回调 | 自定义条件（如 Action） | 继承 `Waitable` |
| QoS 事件回调 | QoS 策略被违反 | `PublisherEventCallbacks` |
| 图事件回调 | 节点/topic 变化 | `add_on_change_state_callback()` |

---

## 八、源码关键路径速查

```
Publisher 实现：
  rclcpp/include/rclcpp/publisher.hpp          # ★★★★★ publish() 实现
  rclcpp/src/rclcpp/publisher_base.cpp         # ★★★★  rcl 层初始化

Subscription 实现：
  rclcpp/include/rclcpp/subscription.hpp       # ★★★★★ handle_message()
  rclcpp/include/rclcpp/any_subscription_callback.hpp  # ★★★★ 回调分发

TypeAdapter：
  rclcpp/include/rclcpp/type_adapter.hpp       # ★★★ 接口定义

进程内通信：
  rclcpp/src/rclcpp/intra_process_manager.cpp  # ★★★★★

序列化：
  rclcpp/src/rclcpp/serialization.cpp          # ★★★

GitHub：
  https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/publisher.hpp
  https://github.com/ros2/rclcpp/blob/rolling/rclcpp/include/rclcpp/subscription.hpp
```
