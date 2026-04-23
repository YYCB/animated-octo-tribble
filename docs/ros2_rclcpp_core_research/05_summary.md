# rclcpp（五）：API 速查手册

## 一、常用 API 速查

### 1.1 节点操作

```cpp
// 创建节点
auto node = std::make_shared<rclcpp::Node>("node_name");
auto node = std::make_shared<rclcpp::Node>("node_name", "/namespace");
auto node = std::make_shared<rclcpp::Node>("node_name",
  rclcpp::NodeOptions()
    .use_intra_process_comms(true)
    .parameter_overrides({{"param", 42}}));

// 节点信息
node->get_name();               // "node_name"
node->get_namespace();          // "/namespace"
node->get_fully_qualified_name(); // "/namespace/node_name"
node->get_logger();             // rclcpp::Logger
node->get_clock();              // rclcpp::Clock::SharedPtr
node->get_node_names();         // std::vector<std::string>
```

### 1.2 发布者

```cpp
// 标准 QoS
auto pub = node->create_publisher<Msg>("topic", 10);  // depth=10 的 RELIABLE
auto pub = node->create_publisher<Msg>("topic", rclcpp::QoS(10));  // 等价
auto pub = node->create_publisher<Msg>("topic", rclcpp::SensorDataQoS());

// 查询
pub->get_topic_name();
pub->get_subscription_count();
pub->get_intra_process_subscription_count();

// 发布
pub->publish(msg);                           // const ref，可能拷贝
pub->publish(std::make_unique<Msg>(msg));    // unique_ptr，尽量零拷贝
pub->publish(*loaned_msg.release());         // 借贷消息
```

### 1.3 订阅者

```cpp
// 简单 lambda
auto sub = node->create_subscription<Msg>("topic", 10,
  [](const Msg & msg) { /* ... */ });

// 带元数据
auto sub = node->create_subscription<Msg>("topic", 10,
  [](const Msg & msg, const rclcpp::MessageInfo & info) {
    if (info.get_rmw_message_info().from_intra_process) {
      // 来自进程内通信
    }
  });

// shared_ptr 避免拷贝
auto sub = node->create_subscription<Msg>("topic", 10,
  [](std::shared_ptr<const Msg> msg) { /* ... */ });

// 成员函数
auto sub = node->create_subscription<Msg>("topic", 10,
  std::bind(&MyClass::callback, this, std::placeholders::_1));

// 指定 CallbackGroup
auto sub = node->create_subscription<Msg>("topic", 10,
  callback,
  rclcpp::SubscriptionOptions().callback_group(cb_group));
```

### 1.4 定时器

```cpp
// WallTimer（系统时钟）
auto timer = node->create_wall_timer(
  std::chrono::milliseconds(100),
  [this]() { /* callback */ });

// GenericTimer（支持仿真时间）
auto timer = node->create_timer(
  node->get_clock(),
  std::chrono::seconds(1),
  [this]() { /* callback */ });

// 暂停/恢复
timer->cancel();
timer->reset();
timer->is_canceled();
```

### 1.5 服务

```cpp
// 服务端
auto srv = node->create_service<Srv>("service_name",
  [](const std::shared_ptr<Srv::Request> req,
     std::shared_ptr<Srv::Response> res) {
    res->sum = req->a + req->b;
  });

// 客户端
auto client = node->create_client<Srv>("service_name");

// 等待服务可用
if (!client->wait_for_service(std::chrono::seconds(5))) {
  RCLCPP_ERROR(node->get_logger(), "Service not available");
  return;
}

// 异步发送请求
auto request = std::make_shared<Srv::Request>();
request->a = 1; request->b = 2;
auto future = client->async_send_request(request);

// 等待响应
if (rclcpp::spin_until_future_complete(node, future) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  auto response = future.get();
  RCLCPP_INFO(node->get_logger(), "Sum: %d", response->sum);
}

// 带回调的异步请求
client->async_send_request(request,
  [](std::shared_future<Srv::Response::SharedPtr> future) {
    auto response = future.get();
  });
```

### 1.6 参数

```cpp
// 声明参数（必须先声明再使用）
node->declare_parameter("my_param", 42);
node->declare_parameter<std::string>("name", "default");

// 获取参数
int value = node->get_parameter("my_param").as_int();
node->get_parameter_or("my_param", default_value);

// 设置参数
node->set_parameter(rclcpp::Parameter("my_param", 100));

// 监听参数变化
auto cb = node->add_on_set_parameters_callback(
  [](const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  });
```

---

## 二、Executor 使用模式

```cpp
// ── 模式1：最简单用法 ─────────────────────────────────────
rclcpp::spin(node);  // 内部创建 SingleThreadedExecutor

// ── 模式2：手动 spin_once ─────────────────────────────────
rclcpp::executors::SingleThreadedExecutor exec;
exec.add_node(node);
while (rclcpp::ok()) {
  exec.spin_once();
}

// ── 模式3：多节点 ─────────────────────────────────────────
auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
exec->add_node(node1);
exec->add_node(node2);
exec->spin();

// ── 模式4：单独线程 spin ──────────────────────────────────
auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
exec->add_node(node);
std::thread spin_thread([&exec]() { exec->spin(); });
// ... 在主线程做其他事情 ...
rclcpp::shutdown();
spin_thread.join();

// ── 模式5：spin_until_future_complete ─────────────────────
auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
exec->add_node(node);
auto future = client->async_send_request(request);
exec->spin_until_future_complete(future);
```

---

## 三、常见错误与解决方案

| 错误 | 原因 | 解决 |
|------|------|------|
| `spin()` called while already spinning | 从回调中再次调用 spin | 使用 `spin_once()` 或独立线程 |
| Publisher 未发布到任何 Subscriber | QoS 不兼容 | 检查 Reliability/Durability 兼容性 |
| 回调不被触发 | 节点未加入 Executor | 确保 `exec.add_node(node)` |
| 进程内通信不生效 | NodeOptions 未设置 | `NodeOptions().use_intra_process_comms(true)` |
| 参数未声明异常 | 直接 get_parameter 不存在的参数 | 先 declare_parameter |
| 定时器漂移累积 | 使用 `std::chrono::steady_clock::now() + period` | rcl_timer 内部已处理（用绝对时间） |
| 订阅者收不到消息 | 收发 QoS Reliability 不匹配 | 确保 Publisher RELIABLE + Subscriber RELIABLE 或两者 BEST_EFFORT |

---

## 四、性能优化速查

```cpp
// 1. 零拷贝发布（进程内）
auto pub = node->create_publisher<Msg>("topic", 
  rclcpp::QoS(10), 
  rclcpp::PublisherOptions()
    .use_intra_process_comm(rclcpp::IntraProcessSetting::Enable));
// 注意：NodeOptions 也要设置 use_intra_process_comms(true)

// 2. unique_ptr 发布（转移所有权）
auto msg = std::make_unique<Msg>();
msg->data = "hello";
pub->publish(std::move(msg));  // 进程内零拷贝

// 3. StaticSingleThreadedExecutor（固定拓扑）
auto exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
exec->add_node(node);
exec->spin();

// 4. 借贷消息（iceoryx 零拷贝，需要对应 rmw 支持）
auto loaned_msg = pub->borrow_loaned_message();
loaned_msg.get().data = 42;
pub->publish(std::move(loaned_msg));

// 5. ReentrantCallbackGroup（CPU 密集型计算并行化）
auto group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
auto sub = node->create_subscription<Msg>("topic", 10, callback,
  rclcpp::SubscriptionOptions().callback_group(group));
// 配合 MultiThreadedExecutor 使用
```

---

## 五、rclcpp 源码关键路径

```
rclcpp 核心（按重要性排序）：

必读：
  src/rclcpp/node.cpp                     ← Node 构造函数
  src/rclcpp/executor.cpp                 ← spin/get_next_executable
  src/rclcpp/intra_process_manager.cpp    ← 零拷贝通信
  include/rclcpp/publisher.hpp            ← publish() 实现
  include/rclcpp/subscription.hpp         ← handle_message()
  include/rclcpp/any_subscription_callback.hpp  ← 回调分发

进阶：
  src/rclcpp/executors/single_threaded_executor.cpp
  src/rclcpp/executors/multi_threaded_executor.cpp
  include/rclcpp/type_adapter.hpp
  src/rclcpp/node_interfaces/node_parameters.cpp  ← 参数系统
  src/rclcpp/node_interfaces/node_clock.cpp       ← 时钟与仿真时间

rclcpp_action（Action）：
  rclcpp_action/src/server.cpp
  rclcpp_action/src/client.cpp

GitHub 链接：
  https://github.com/ros2/rclcpp/tree/rolling/rclcpp
```

---

## 六、rclcpp 与常见 ROS2 库的集成

```cpp
// tf2_ros 集成
#include <tf2_ros/transform_broadcaster.h>
tf2_ros::TransformBroadcaster tf_broadcaster(node);
geometry_msgs::msg::TransformStamped t;
// ... 填充 t ...
tf_broadcaster.sendTransform(t);

// image_transport 集成（订阅压缩图像）
#include <image_transport/image_transport.hpp>
auto it = image_transport::create_subscription(node.get(), "image_raw",
  [](const sensor_msgs::msg::Image::ConstSharedPtr & msg) { /* ... */ },
  "compressed");

// diagnostic_updater 集成
#include <diagnostic_updater/diagnostic_updater.hpp>
diagnostic_updater::Updater updater(node);
updater.add("Heartbeat", [](diagnostic_updater::DiagnosticStatusWrapper & stat) {
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
});
```
