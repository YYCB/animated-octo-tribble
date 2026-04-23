# Lifecycle Node（二）：rclcpp::LifecycleNode 完整 API

## 一、LifecycleNode 创建与继承结构

```cpp
// rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp

class LifecycleNode
  : public node_interfaces::LifecycleNodeInterface,
    public std::enable_shared_from_this<LifecycleNode>
{
public:
  explicit LifecycleNode(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions())
  : LifecycleNode(node_name, "", options) {}
  
  LifecycleNode(
    const std::string & node_name,
    const std::string & namespace_,
    const NodeOptions & options = NodeOptions())
  {
    // ★ 内部持有普通 Node（复用 rclcpp::Node 的所有功能）
    node_base_ = std::make_shared<node_interfaces::NodeBase>(
      node_name, namespace_, context, options, false, false);
    
    // ★ 初始化 rcl_lifecycle 状态机
    rcl_lifecycle_state_machine_init(
      &state_machine_,
      node_base_->get_rcl_node_handle(),
      ...
    );
    
    // ★ 注册 change_state Service 处理
    init_managed_entities();
  }

  // ★ 用户需要重写的 5 个关键回调：
  CallbackReturn on_configure(const State & previous_state) override;
  CallbackReturn on_cleanup(const State & previous_state) override;
  CallbackReturn on_activate(const State & previous_state) override;
  CallbackReturn on_deactivate(const State & previous_state) override;
  CallbackReturn on_shutdown(const State & previous_state) override;
  CallbackReturn on_error(const State & previous_state) override;  // 可选
  
  // ★ 获取当前状态
  const State & get_current_state() const;
  
  // ★ 手动触发转换（内部/外部都可调用）
  const State & trigger_transition(const Transition & transition);
  const State & trigger_transition(const std::string & transition_label);
  
  // ★ 状态变化订阅（监听 ~/transition_event）
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<T>>
  create_publisher(const std::string & topic, const rclcpp::QoS & qos);
  // LifecyclePublisher：只在 ACTIVE 状态发布（自动控制）
};
```

---

## 二、CallbackReturn 枚举

```cpp
// rclcpp_lifecycle/include/rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp

enum class CallbackReturn : uint8_t {
  SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,  // 0
  FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,  // 1
  ERROR   = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR,    // 2
};

// 宏简化（常用）：
#define ROS_LIFECYCLE_RETURN_SUCCESS CallbackReturn::SUCCESS
```

---

## 三、完整 Lifecycle Node 实现示例

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class CameraDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CameraDriver(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("camera_driver", options)
  {
    // ★ 在构造函数中只声明参数，不分配资源
    declare_parameter("device_id", 0);
    declare_parameter("frame_rate", 30.0);
    declare_parameter("resolution.width", 640);
    declare_parameter("resolution.height", 480);
  }

  // ★ UNCONFIGURED → INACTIVE：分配资源，初始化硬件
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "Configuring...");
    
    // 读取参数
    device_id_ = get_parameter("device_id").as_int();
    frame_rate_ = get_parameter("frame_rate").as_double();
    
    // ★ 创建 LifecyclePublisher（只在 ACTIVE 状态发布）
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::QoS(10));
    
    // ★ 初始化硬件（但不启动）
    try {
      camera_ = std::make_unique<CameraDevice>(device_id_);
      camera_->initialize();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Camera init failed: %s", e.what());
      return CallbackReturn::FAILURE;  // ★ 失败：回到 UNCONFIGURED
    }
    
    RCLCPP_INFO(get_logger(), "Configured successfully.");
    return CallbackReturn::SUCCESS;
  }

  // ★ INACTIVE → ACTIVE：开始工作
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating...");
    
    // ★ 激活 LifecyclePublisher
    image_pub_->on_activate();
    
    // 启动摄像头采集
    camera_->start_capture();
    
    // 启动定时器（开始发布）
    timer_ = create_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate_)),
      [this]() { capture_and_publish(); }
    );
    
    RCLCPP_INFO(get_logger(), "Activated.");
    return CallbackReturn::SUCCESS;
  }

  // ★ ACTIVE → INACTIVE：停止工作，保留资源
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    
    timer_->cancel();
    timer_.reset();
    
    camera_->stop_capture();
    
    // ★ 去激活 LifecyclePublisher（停止发布）
    image_pub_->on_deactivate();
    
    return CallbackReturn::SUCCESS;
  }

  // ★ INACTIVE → UNCONFIGURED：释放资源
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    
    camera_.reset();
    image_pub_.reset();
    
    return CallbackReturn::SUCCESS;
  }

  // ★ 任意状态 → FINALIZED
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    
    // 紧急清理
    if (camera_) camera_.reset();
    if (image_pub_) image_pub_.reset();
    
    return CallbackReturn::SUCCESS;
  }

private:
  void capture_and_publish() {
    if (!image_pub_->is_activated()) return;  // 额外检查
    auto img = camera_->capture_frame();
    image_pub_->publish(*img);
  }
  
  int device_id_;
  double frame_rate_;
  std::unique_ptr<CameraDevice> camera_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

---

## 三、LifecyclePublisher — 状态感知的发布者

```cpp
// rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_publisher.hpp

template<typename MessageT>
class LifecyclePublisher : public rclcpp::Publisher<MessageT>
{
public:
  // ★ activate/deactivate 由 LifecycleNode 的 on_activate/on_deactivate 调用
  void on_activate() { activated_.store(true); }
  void on_deactivate() { activated_.store(false); }
  
  bool is_activated() const { return activated_.load(); }
  
  // ★ 重写 publish()：未激活时自动跳过
  void publish(const MessageT & msg) override
  {
    if (!is_activated()) {
      // ★ 静默丢弃（不发布，不报错）
      RCLCPP_WARN_ONCE(get_logger(),
        "LifecyclePublisher is not activated, skipping publish");
      return;
    }
    rclcpp::Publisher<MessageT>::publish(msg);
  }

private:
  std::atomic<bool> activated_{false};
};
```

---

## 四、外部管理：lifecycle Service 调用

```python
# Python 工具调用（ros2 CLI）：
# ros2 lifecycle set /camera_driver configure
# ros2 lifecycle set /camera_driver activate
# ros2 lifecycle get /camera_driver
# ros2 lifecycle list  # 列出所有 lifecycle 节点

# Service 调用（编程）：
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleClient(Node):
    def __init__(self):
        super().__init__('lifecycle_client')
        self.cli = self.create_client(ChangeState, '/camera_driver/change_state')
    
    def configure(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        future = self.cli.call_async(req)
        # ...
    
    def activate(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.cli.call_async(req)
```

---

## 五、nav2_lifecycle_manager 模式

```cpp
// 典型 Nav2 启动流程：
// 1. 所有 lifecycle 节点以 UNCONFIGURED 状态启动
// 2. LifecycleManager 按顺序逐一 configure
// 3. 所有 configure 成功后，逐一 activate
// 4. 任一节点失败：反向 deactivate + cleanup 所有节点

// LifecycleManager 内部实现（简化）：
class LifecycleManager : public rclcpp::Node {
  void startup() {
    // 按顺序 configure
    for (auto & node_name : managed_nodes_) {
      if (!change_state(node_name, Transition::TRANSITION_CONFIGURE)) {
        cleanup_all();
        return;
      }
    }
    
    // 全部 configure 成功后，activate
    for (auto & node_name : managed_nodes_) {
      if (!change_state(node_name, Transition::TRANSITION_ACTIVATE)) {
        // 回滚：deactivate 已 activate 的节点
        rollback_activate();
        return;
      }
    }
  }
  
  bool change_state(const std::string & node_name, uint8_t transition_id) {
    auto client = change_state_clients_[node_name];
    auto req = std::make_shared<ChangeState::Request>();
    req->transition.id = transition_id;
    
    auto future = client->async_send_request(req);
    auto result = executor_->spin_until_future_complete(future, 30s);
    
    return result == rclcpp::FutureReturnCode::SUCCESS 
           && future.get()->success;
  }
};
```

---

## 六、~/transition_event — 状态变化监听

```cpp
// 订阅所有 Lifecycle 节点的状态变化：
auto sub = node->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
  "/camera_driver/transition_event",
  rclcpp::QoS(10),
  [](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
    RCLCPP_INFO(logger, "State change: %s -> %s",
                msg->start_state.label.c_str(),
                msg->goal_state.label.c_str());
  }
);

// lifecycle_msgs/msg/TransitionEvent.msg：
// builtin_interfaces/Time timestamp
// Transition transition
//   uint8 id
//   string label
// State start_state
//   uint8 id
//   string label
// State goal_state
//   uint8 id
//   string label
```
