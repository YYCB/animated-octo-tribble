# 参数系统（三）：动态参数回调机制

## 一、add_on_set_parameters_callback() 源码

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp

// 返回 OnSetParametersCallbackHandle（持有 shared_ptr 以管理生命周期）
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
NodeParameters::add_on_set_parameters_callback(
  OnSetParametersCallbackType callback)
{
  auto handle = std::make_shared<OnSetParametersCallbackHandle>();
  handle->callback = callback;
  
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // ★ 新回调插入队列最前面（后注册的先执行）
  on_set_parameters_callbacks_.emplace_front(handle);
  
  return handle;
}

// 触发所有回调：
rcl_interfaces::msg::SetParametersResult
NodeParameters::call_on_set_parameters_callbacks(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  
  for (auto & weak_handle : on_set_parameters_callbacks_) {
    // 检查 handle 是否还存活（用 weak_ptr 防止悬挂）
    auto handle = weak_handle.lock();
    if (!handle) {
      continue;
    }
    
    // ★ 执行回调
    auto result = handle->callback(parameters);
    if (!result.successful) {
      // ★ 任一回调拒绝，立即停止（不执行后续回调）
      return result;
    }
  }
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}
```

---

## 二、完整动态参数使用模式

```cpp
class PIDController : public rclcpp::Node
{
public:
  PIDController() : Node("pid_controller") {
    // ★ Step 1: 声明参数（带描述符和范围）
    rcl_interfaces::msg::ParameterDescriptor kp_desc;
    kp_desc.description = "Proportional gain";
    kp_desc.floating_point_range.resize(1);
    kp_desc.floating_point_range[0].from_value = 0.0;
    kp_desc.floating_point_range[0].to_value = 100.0;
    
    declare_parameter("kp", 1.0, kp_desc);
    declare_parameter("ki", 0.1);
    declare_parameter("kd", 0.01);
    
    // ★ Step 2: 读取初始值
    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    
    // ★ Step 3: 注册动态回调
    param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
        -> rcl_interfaces::msg::SetParametersResult {
          return on_parameters_set(params);
        }
    );
  }

private:
  rcl_interfaces::msg::SetParametersResult
  on_parameters_set(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto & p : params) {
      if (p.get_name() == "kp") {
        double new_kp = p.as_double();
        // ★ 自定义验证（超出 ParameterDescriptor 范围的额外检查）
        if (new_kp < 0.0) {
          result.successful = false;
          result.reason = "kp must be non-negative";
          return result;
        }
        kp_ = new_kp;
        RCLCPP_INFO(get_logger(), "Updated kp: %.3f", kp_);
        
      } else if (p.get_name() == "ki") {
        ki_ = p.as_double();
      } else if (p.get_name() == "kd") {
        kd_ = p.as_double();
      }
    }
    return result;
  }

  double kp_, ki_, kd_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
```

---

## 三、参数回调的生命周期管理

```cpp
// OnSetParametersCallbackHandle 使用 RAII 管理回调生命周期

class MyNode : public rclcpp::Node {
  // ★ 必须保存 handle，否则回调会被自动移除！
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  
  MyNode() : Node("my_node") {
    callback_handle_ = add_on_set_parameters_callback([this](auto & params) {
      // 此处 this 必须有效
      return handle_params(params);
    });
  }
  
  // ❌ 错误：不保存 handle
  void wrong_usage() {
    add_on_set_parameters_callback([](auto &) { /* ... */ });
    // handle 立即析构 → 回调被移除！
  }
};

// 移除回调：
// 让 callback_handle_ 析构（= nullptr 或超出作用域）
// → NodeParameters 中的 weak_ptr 变为 expired
// → 下次 call_on_set_parameters_callbacks 时自动跳过
```

---

## 四、AsyncParametersClient — 远程参数访问

```cpp
// rclcpp/include/rclcpp/parameter_client.hpp

class AsyncParametersClient
{
public:
  explicit AsyncParametersClient(
    rclcpp::Node::SharedPtr node,
    const std::string & remote_node_name = "",  // 空=本节点
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters)
  {
    // 创建 6 个 Service Client，连接到 remote_node_name 的参数服务
    get_parameters_client_ = node->create_client<rcl_interfaces::srv::GetParameters>(
      remote_node_name + "/get_parameters");
    set_parameters_client_ = node->create_client<rcl_interfaces::srv::SetParameters>(
      remote_node_name + "/set_parameters");
    // ...
  }
  
  // ★ 异步获取参数（返回 future）
  std::shared_future<std::vector<rclcpp::Parameter>>
  get_parameters(
    const std::vector<std::string> & names,
    std::function<void(std::shared_future<std::vector<rclcpp::Parameter>>)> callback = nullptr)
  {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = names;
    
    auto future = get_parameters_client_->async_send_request(
      request,
      [callback](auto future) {
        if (callback) {
          // 将 Service Response 转换为 vector<Parameter>
          callback(/* wrapped future */);
        }
      }
    );
    return wrapped_future;
  }
  
  // ★ 异步设置参数
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);
  
  // ★ 等待参数服务可用
  bool wait_for_service(std::chrono::nanoseconds timeout = std::chrono::seconds(-1)) {
    return get_parameters_client_->wait_for_service(timeout);
  }
};

// 使用示例：
auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(
  node, "/pid_controller");

// 等待服务上线
if (param_client->wait_for_service(std::chrono::seconds(5))) {
  // 异步读取参数
  auto future = param_client->get_parameters({"kp", "ki", "kd"});
  executor.spin_until_future_complete(future);
  auto params = future.get();
  
  // 远程设置参数
  auto set_future = param_client->set_parameters({
    rclcpp::Parameter("kp", 2.0),
    rclcpp::Parameter("ki", 0.05)
  });
}
```

---

## 五、SyncParametersClient — 同步封装

```cpp
// 同步版本（内部用独立 Executor）

class SyncParametersClient
{
  SyncParametersClient(
    rclcpp::Node::SharedPtr node,
    const std::string & remote_node_name = "")
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    async_parameters_client_ = std::make_shared<AsyncParametersClient>(
      node, remote_node_name);
  }
  
  // ★ 同步获取参数（阻塞直到结果返回）
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & parameter_names)
  {
    return rclcpp::spin_until_future_complete(
      executor_,
      async_parameters_client_->get_parameters(parameter_names)
    ).get();
  }
  
  // ★ 同步设置
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    return rclcpp::spin_until_future_complete(
      executor_,
      async_parameters_client_->set_parameters(parameters)
    ).get();
  }
};
```

---

## 六、参数事件监听（ParameterEventHandler）

```cpp
// rclcpp/include/rclcpp/parameter_event_handler.hpp（Galactic+）

class ParameterEventHandler
{
public:
  ParameterEventHandler(rclcpp::Node::SharedPtr node)
  {
    // ★ 订阅 /parameter_events（全局所有节点的参数事件）
    event_subscription_ = node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events",
      rclcpp::QoS(1000),  // 大 depth 避免丢失事件
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        event_callback(event);
      }
    );
  }
  
  // ★ 注册回调：监听特定节点的特定参数
  ParameterCallbackHandle::SharedPtr
  add_parameter_callback(
    const std::string & parameter_name,
    ParameterCallbackType callback,
    const std::string & node_name = "")  // 空=当前节点
  {
    auto handle = std::make_shared<ParameterCallbackHandle>();
    handle->parameter_name = parameter_name;
    handle->node_name = resolve_node_name(node_name);
    handle->callback = callback;
    
    callbacks_[parameter_name].emplace_front(handle);
    return handle;
  }
  
private:
  void event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    // 检查 changed_parameters 和 new_parameters
    for (const auto & param_msg : event->changed_parameters) {
      auto it = callbacks_.find(param_msg.name);
      if (it != callbacks_.end()) {
        for (auto & weak_handle : it->second) {
          auto handle = weak_handle.lock();
          if (handle && (handle->node_name.empty() || handle->node_name == event->node)) {
            handle->callback(rclcpp::Parameter::from_parameter_msg(param_msg));
          }
        }
      }
    }
  }
  
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr event_subscription_;
  std::map<std::string, std::list<ParameterCallbackHandle::WeakPtr>> callbacks_;
};
```

---

## 七、参数系统调试命令

```bash
# 列出节点所有参数
ros2 param list /my_node

# 获取单个参数
ros2 param get /my_node kp

# 设置参数（运行时动态修改）
ros2 param set /my_node kp 2.5

# 导出节点参数为 YAML
ros2 param dump /my_node

# 加载 YAML 参数文件
ros2 param load /my_node config.yaml

# 查看描述符（类型、范围、只读等）
ros2 param describe /my_node kp

# 监听参数事件
ros2 topic echo /parameter_events
```
