# tf2（三）：TransformBroadcaster/StaticBroadcaster 源码

## 一、TransformBroadcaster

```cpp
// tf2_ros/include/tf2_ros/transform_broadcaster.h

class TransformBroadcaster
{
public:
  // ★ 构造函数：创建 /tf Publisher
  explicit TransformBroadcaster(
    rclcpp::Node::SharedPtr node,
    const rclcpp::QoS & qos = DynamicBroadcasterQoS())
  {
    // 创建 Publisher（/tf Topic，消息类型 tf2_msgs/msg/TFMessage）
    publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf",
      qos  // 默认 DynamicBroadcasterQoS = KEEP_LAST(100) + BEST_EFFORT
    );
  }
  
  // ★ 发送单个变换
  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{transform});
  }
  
  // ★ 批量发送（减少 DDS 开销）
  void sendTransform(
    const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    tf2_msgs::msg::TFMessage message;
    message.transforms = transforms;
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

// DynamicBroadcasterQoS 默认参数：
class DynamicBroadcasterQoS : public rclcpp::QoS {
  DynamicBroadcasterQoS() : rclcpp::QoS(100) {
    // KEEP_LAST(100)
    // BEST_EFFORT（允许丢失，tf 高频发布，偶尔丢一帧可接受）
    best_effort();
  }
};
```

---

## 二、StaticTransformBroadcaster — TRANSIENT_LOCAL 关键

```cpp
// tf2_ros/src/static_transform_broadcaster.cpp

class StaticTransformBroadcaster
{
public:
  explicit StaticTransformBroadcaster(
    rclcpp::Node::SharedPtr node,
    const rclcpp::QoS & qos = StaticBroadcasterQoS())
  {
    // ★ /tf_static 发布（与 /tf 的关键区别：TRANSIENT_LOCAL）
    publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static",
      qos  // StaticBroadcasterQoS = KEEP_LAST(1) + RELIABLE + TRANSIENT_LOCAL
    );
  }
  
  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{transform});
  }
  
  // ★ 关键：静态变换会累积发布（不覆盖，只追加/更新）
  void sendTransform(
    const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
  {
    // 遍历新变换，更新或追加到 net_message_
    for (auto & tf : transforms) {
      bool match = false;
      for (auto & existing_tf : net_message_.transforms) {
        // ★ 如果 child_frame_id 相同，更新（覆盖）
        if (existing_tf.child_frame_id == tf.child_frame_id) {
          existing_tf = tf;
          match = true;
          break;
        }
      }
      if (!match) {
        // ★ 新的帧，追加
        net_message_.transforms.push_back(tf);
      }
    }
    
    // ★ 发布完整的静态变换集合
    // TRANSIENT_LOCAL + RELIABLE 确保新订阅者可以收到历史消息
    publisher_->publish(net_message_);
  }

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  tf2_msgs::msg::TFMessage net_message_;  // ★ 累积所有静态变换
};

// StaticBroadcasterQoS 默认参数：
class StaticBroadcasterQoS : public rclcpp::QoS {
  StaticBroadcasterQoS() : rclcpp::QoS(1) {
    // KEEP_LAST(1) - 只保留最新一条（包含所有静态变换）
    reliable();
    transient_local();  // ★ 关键：新节点上线时可收到已发布的静态变换
  }
};
```

---

## 三、为什么 /tf_static 必须用 TRANSIENT_LOCAL

```
问题场景：
  机器人启动流程：
    t=0s  StaticTransformBroadcaster 发布 base_link→camera_link
    t=5s  新节点 image_processor 启动
    
  如果使用 VOLATILE（/tf 的方式）：
    image_processor 在 t=5s 订阅 /tf_static
    此时 base_link→camera_link 已发布，image_processor 收不到
    → lookupTransform("camera_link", ...) 抛异常

  使用 TRANSIENT_LOCAL：
    image_processor 在 t=5s 订阅 /tf_static
    DDS 将历史最新消息（depth=1）重新发送给 image_processor
    → 立即收到 base_link→camera_link
    → lookupTransform 成功

实现机制（DDS层）：
  TRANSIENT_LOCAL 的 DataWriter 在内存中保留 depth 条历史消息
  新 DataReader 匹配时，DataWriter 自动重传历史消息
  对于 StaticBroadcaster：depth=1（net_message_ 包含所有静态变换的完整集合）
```

---

## 四、TransformListener 订阅 /tf 和 /tf_static

```cpp
// tf2_ros/src/transform_listener.cpp

TransformListener::TransformListener(
  tf2::BufferCore & buffer,
  rclcpp::Node::SharedPtr node,
  bool spin_thread,
  const rclcpp::QoS & qos,
  const rclcpp::QoS & static_qos)
: buffer_(buffer)
{
  // ★ 订阅 /tf（动态变换）
  message_subscription_tf_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf",
    qos,  // DynamicListenerQoS = KEEP_LAST(100) + BEST_EFFORT
    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      subscription_callback(msg, false);  // is_static = false
    }
  );
  
  // ★ 订阅 /tf_static（静态变换）
  message_subscription_tf_static_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    static_qos,  // StaticListenerQoS = KEEP_LAST(1) + RELIABLE + TRANSIENT_LOCAL
    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      subscription_callback(msg, true);  // is_static = true
    }
  );
  
  // ★ 可选：独立 spin 线程（使 tf 独立于主 Executor）
  if (spin_thread) {
    auto dedicated_node = std::make_shared<rclcpp::Node>(
      "_tf2_listener_thread", rclcpp::NodeOptions());
    dedicated_listener_node_ = dedicated_node;
    
    // 独立 SingleThreadedExecutor 运行 tf 订阅
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(dedicated_listener_node_);
    
    thread_ = std::make_unique<std::thread>([this]() {
      executor_->spin();
    });
  }
}

// ★ 回调：将收到的变换写入 BufferCore
void TransformListener::subscription_callback(
  const tf2_msgs::msg::TFMessage::SharedPtr msg,
  bool is_static)
{
  for (const auto & transform : msg->transforms) {
    try {
      // ★ 写入 BufferCore（触发 transforms_available_cond_.notify_all()）
      buffer_.setTransform(transform, "UNKNOWN", is_static);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "Failed to set transform: %s", ex.what());
    }
  }
}
```

---

## 五、tf2_ros::Buffer — 完整使用层

```cpp
// tf2_ros/include/tf2_ros/buffer.h

class Buffer : public tf2::BufferCore, public tf2::BufferInterface
{
public:
  // ★ lookupTransform（带 timeout，阻塞等待）
  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration & timeout = tf2::durationFromSec(0.0)) const override;
  
  // ★ lookupTransform（time travel：查询 source 在 t1 时，target 在 t2 时的变换）
  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame,
    const tf2::TimePoint & target_time,
    const std::string & source_frame,
    const tf2::TimePoint & source_time,
    const std::string & fixed_frame,   // 固定参考帧（world/odom）
    const tf2::Duration & timeout = tf2::durationFromSec(0.0)) const override;
  
  // ★ canTransform（非阻塞检查）
  bool canTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration & timeout = tf2::durationFromSec(0.0),
    std::string * errstr = nullptr) const override;
  
  // ★ waitForTransform（异步等待，返回 Future）
  // Humble+ 新增异步 API：
  // auto future = buffer_.waitForTransform(target, source, time, timeout, callback);
};
```

---

## 六、MessageFilter — 等待 TF 可用后处理消息

```cpp
// tf2_ros/include/tf2_ros/message_filter.h
// 解决问题：收到传感器消息时，对应时刻的 tf 可能还未到达

// 使用方式：
auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// 订阅 LaserScan，等待 tf 可用后才处理
auto scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
  this, "/scan");

auto tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
  *scan_sub_,
  *tf_buffer_,
  "base_link",    // 目标帧
  100,            // 队列大小
  get_node_logging_interface(),
  get_node_clock_interface()
);

tf_filter_->registerCallback([this](const auto & msg) {
  // 此时 tf 必然可用
  auto t = tf_buffer_->lookupTransform("base_link", msg->header.frame_id,
                                       tf2_ros::fromMsg(msg->header.stamp));
  // 处理消息...
});

// MessageFilter 内部机制：
// 1. 收到消息 msg（stamp = T）
// 2. 检查是否可以查询 tf(target, source, T)
// 3. 若不可以：暂存队列
// 4. 每次 setTransform 回调时，重新检查队列中的消息
// 5. tf 就绪后触发用户回调
```

---

## 七、robot_state_publisher — URDF → TF 管线

```
URDF（机器人描述文件）→ robot_state_publisher Node
  ├── 静态关节（fixed joint）→ StaticTransformBroadcaster → /tf_static
  └── 动态关节（revolute/prismatic）→ 订阅 /joint_states → TransformBroadcaster → /tf

/joint_states（sensor_msgs/msg/JointState）：
  - 包含各关节当前角度/位置
  - 由底盘控制器、MoveIt 等发布

robot_state_publisher 内部：
  1. 解析 URDF（KDL 运动学库）
  2. 获取 /joint_states 消息
  3. 用关节角计算正向运动学（FK）
  4. 将每个关节的变换发布到 /tf

完整 TF 树示例（humanoid）：
  /tf_static：base_link→imu_link, head→camera_link（固定零部件）
  /tf：odom→base_link（里程计），base_link→left_foot（踝关节角变化）
```
