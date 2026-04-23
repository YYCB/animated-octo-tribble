# 参数系统（二）：declare/get/set 完整源码

## 一、declare_parameter<T>() 完整流程

```cpp
// rclcpp/include/rclcpp/node_impl.hpp

template<typename ParameterT>
auto Node::declare_parameter(
  const std::string & name,
  const ParameterT & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  // ★ 委托给 NodeParameters
  return this->node_parameters_->declare_parameter(
    name,
    rclcpp::ParameterValue(default_value),
    parameter_descriptor,
    ignore_override
  ).get<ParameterT>();
}

// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp
const rclcpp::ParameterValue &
NodeParameters::declare_parameter(
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor,
  bool ignore_override)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  
  // ★ Step 1: 检查是否已声明
  if (parameters_.count(name)) {
    throw rclcpp::exceptions::ParameterAlreadyDeclaredException(
      "Parameter '" + name + "' has already been declared");
  }
  
  // ★ Step 2: 确定初始值（命令行覆盖 > 默认值）
  rclcpp::ParameterValue initial_value = default_value;
  
  if (!ignore_override) {
    auto override_it = parameter_overrides_.find(name);
    if (override_it != parameter_overrides_.end()) {
      // 命令行或 YAML 文件提供的覆盖值
      if (default_value.get_type() != rclcpp::PARAMETER_NOT_SET &&
          override_it->second.get_type() != default_value.get_type()) {
        // 类型不匹配：尝试隐式转换（integer→double 允许）
        // 否则抛出 InvalidParameterTypeException
      }
      initial_value = override_it->second;
    }
  }
  
  // ★ Step 3: 验证初始值（范围检查）
  auto result = check_parameter_value_in_range(
    parameter_descriptor, initial_value);
  if (!result.successful) {
    throw rclcpp::exceptions::InvalidParameterValueException(result.reason);
  }
  
  // ★ Step 4: 运行 on_set_parameters_callbacks_（声明时也会触发）
  std::vector<rclcpp::Parameter> params_to_set = {
    rclcpp::Parameter(name, initial_value)};
  
  rcl_interfaces::msg::SetParametersResult set_result = 
    call_on_set_parameters_callbacks(params_to_set);
  
  if (!set_result.successful) {
    throw rclcpp::exceptions::InvalidParameterValueException(
      "Parameter '" + name + "' rejected: " + set_result.reason);
  }
  
  // ★ Step 5: 存储参数
  parameters_[name] = ParameterInfo{initial_value, parameter_descriptor};
  
  // ★ Step 6: 发布参数事件（/parameter_events topic）
  auto events_pub = events_publisher_.lock();
  if (events_pub) {
    rcl_interfaces::msg::ParameterEvent event;
    event.node = combine_name_and_namespace(node_name_, node_namespace_);
    event.new_parameters.push_back(rclcpp::Parameter(name, initial_value).to_parameter_msg());
    events_pub->publish(event);
  }
  
  return parameters_[name].value;
}
```

---

## 二、get_parameter() 实现

```cpp
// rclcpp/include/rclcpp/node_impl.hpp

template<typename ParameterT>
ParameterT Node::get_parameter(const std::string & name) const
{
  rclcpp::Parameter parameter;
  if (!get_parameter(name, parameter)) {
    throw rclcpp::exceptions::ParameterNotDeclaredException(name);
  }
  return parameter.get_value<ParameterT>();
}

// 批量获取（更高效）
bool Node::get_parameters(
  const std::string & prefix,
  std::map<std::string, ParameterT> & values) const
{
  rcl_interfaces::msg::ListParametersResult list = 
    list_parameters({prefix}, 1);
  
  for (const auto & name : list.names) {
    rclcpp::Parameter param;
    if (get_parameter(name, param)) {
      // 去掉 prefix 前缀
      std::string short_name = name.substr(prefix.size() + 1);
      values[short_name] = param.get_value<ParameterT>();
    }
  }
  return !values.empty();
}

// NodeParameters::get_parameter()
bool NodeParameters::get_parameter(
  const std::string & name,
  rclcpp::Parameter & parameter) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  
  auto it = parameters_.find(name);
  if (it == parameters_.end()) {
    // 未声明的参数
    if (allow_undeclared_) {
      // 兼容模式：返回 PARAMETER_NOT_SET
      parameter = rclcpp::Parameter(name);
      return true;
    }
    return false;
  }
  
  parameter = rclcpp::Parameter(name, it->second.value);
  return true;
}
```

---

## 三、set_parameter() 完整实现

```cpp
// rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp

rcl_interfaces::msg::SetParametersResult
NodeParameters::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  
  // ★ Step 1: 检查每个参数是否已声明
  for (const auto & p : parameters) {
    auto it = parameters_.find(p.get_name());
    if (it == parameters_.end()) {
      if (!allow_undeclared_) {
        return make_failure("Parameter '" + p.get_name() + "' not declared");
      }
    }
  }
  
  // ★ Step 2: 类型检查（必须与已声明类型一致）
  for (const auto & p : parameters) {
    auto it = parameters_.find(p.get_name());
    if (it != parameters_.end()) {
      auto current_type = it->second.value.get_type();
      auto new_type = p.get_type();
      
      if (current_type != PARAMETER_NOT_SET && current_type != new_type) {
        if (!it->second.descriptor.dynamic_typing) {
          return make_failure("Type mismatch for '" + p.get_name() + "'");
        }
      }
    }
  }
  
  // ★ Step 3: 范围检查
  for (const auto & p : parameters) {
    auto it = parameters_.find(p.get_name());
    if (it != parameters_.end()) {
      auto result = check_parameter_value_in_range(
        it->second.descriptor, p.get_parameter_value());
      if (!result.successful) return result;
    }
  }
  
  // ★ Step 4: 只读检查
  for (const auto & p : parameters) {
    auto it = parameters_.find(p.get_name());
    if (it != parameters_.end() && it->second.descriptor.read_only) {
      return make_failure("Parameter '" + p.get_name() + "' is read_only");
    }
  }
  
  // ★ Step 5: 运行用户验证回调链
  auto result = call_on_set_parameters_callbacks(parameters);
  if (!result.successful) return result;
  
  // ★ Step 6: 原子性提交（所有参数一起写入）
  for (const auto & p : parameters) {
    parameters_[p.get_name()].value = p.get_parameter_value();
  }
  
  // ★ Step 7: 发布参数事件
  auto events_pub = events_publisher_.lock();
  if (events_pub) {
    rcl_interfaces::msg::ParameterEvent event;
    event.node = node_full_name_;
    for (const auto & p : parameters) {
      event.changed_parameters.push_back(p.to_parameter_msg());
    }
    events_pub->publish(event);
  }
  
  result.successful = true;
  return result;
}
```

---

## 四、ParameterService — 6个 Service 实现

```cpp
// rclcpp/src/rclcpp/parameter_service.cpp

ParameterService::ParameterService(
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base,
  const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services,
  const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_params,
  const rclcpp::QoS & qos_profile)
{
  // ★ 1. get_parameters Service
  get_parameters_service_ = create_service<rcl_interfaces::srv::GetParameters>(
    node_base, node_services,
    node_base->get_name() + std::string("/get_parameters"),
    [node_params](
      const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
        auto result = node_params->get_parameters(request->names);
        for (auto & p : result) {
          response->values.push_back(p.get_parameter_value().to_value_msg());
        }
      },
    qos_profile);
  
  // ★ 2. set_parameters Service
  set_parameters_service_ = create_service<rcl_interfaces::srv::SetParameters>(
    node_base, node_services,
    node_base->get_name() + std::string("/set_parameters"),
    [node_params](
      const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
        // ★ 逐个设置（非原子性）
        for (auto & p_msg : request->parameters) {
          auto p = rclcpp::Parameter::from_parameter_msg(p_msg);
          auto result = node_params->set_parameters_atomically({p});
          response->results.push_back(result);
        }
      },
    qos_profile);
  
  // ★ 3. set_parameters_atomically Service
  set_parameters_atomically_service_ = create_service<
    rcl_interfaces::srv::SetParametersAtomically>(
    node_base, node_services,
    node_base->get_name() + std::string("/set_parameters_atomically"),
    [node_params](
      const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response) {
        // ★ 全部成功或全部失败
        std::vector<rclcpp::Parameter> params;
        for (auto & p_msg : request->parameters) {
          params.push_back(rclcpp::Parameter::from_parameter_msg(p_msg));
        }
        response->result = node_params->set_parameters_atomically(params);
      },
    qos_profile);
  
  // 4. describe_parameters, 5. get_parameter_types, 6. list_parameters
  // （类似结构，省略）
}
```

---

## 五、/parameter_events 话题

```cpp
// 每次参数变化（declare/set/undeclare）都发布到 /parameter_events
// 消息类型：rcl_interfaces/msg/ParameterEvent

// rcl_interfaces/msg/ParameterEvent.msg
builtin_interfaces/Time stamp
string node              # 节点完整名称（/ns/node）
Parameter[] new_parameters      # 新声明的参数
Parameter[] changed_parameters  # 被修改的参数
Parameter[] deleted_parameters  # 被删除的参数（undeclare）

// 用途：
// 1. ros2 param dump 命令：监听 /parameter_events 重建状态
// 2. ParameterEventHandler（rclcpp 辅助类）：订阅特定节点参数变化
// 3. rqt_reconfigure：监听所有节点参数变化的 UI

// ParameterEventHandler 用法（Galactic+）：
auto param_handler = std::make_shared<rclcpp::ParameterEventHandler>(node);

// 监听自己的参数变化：
auto cb = param_handler->add_parameter_callback(
  "my_param",
  [](const rclcpp::Parameter & p) {
    RCLCPP_INFO(logger, "my_param changed to: %s", p.value_to_string().c_str());
  }
);

// 监听其他节点的参数变化：
auto cb2 = param_handler->add_parameter_callback(
  "max_speed",
  [](const rclcpp::Parameter & p) { /* ... */ },
  "/other_node"  // 目标节点名
);
```
