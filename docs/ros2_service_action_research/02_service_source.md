# Service/Action（二）：rclcpp Service/Client 源码全解

## 一、Node::create_service() 调用链

```cpp
// rclcpp/include/rclcpp/node.hpp → rclcpp/include/rclcpp/node_impl.hpp

template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return rclcpp::create_service<ServiceT>(
    node_base_,          // NodeBaseInterface
    node_services_,      // NodeServicesInterface
    service_name,
    std::forward<CallbackT>(callback),
    qos,
    group
  );
}

// rclcpp/include/rclcpp/create_service.hpp
template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr
create_service(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  CallbackT && callback,
  const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group)
{
  // 1. 包装用户回调为类型安全的 AnyServiceCallback<ServiceT>
  rclcpp::AnyServiceCallback<ServiceT> any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));
  
  // 2. 构建 rcl_service_options（含 QoS 转换）
  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = rclcpp::qos_check_compatible(qos).get_rmw_qos_profile();
  
  // 3. 构造 rclcpp::Service<ServiceT>（内部调用 rcl_service_init）
  auto serv = Service<ServiceT>::make_shared(
    node_base->get_shared_rcl_node_handle(),
    service_name,
    any_service_callback,
    service_options
  );
  
  // 4. ★ 注册到 NodeServicesInterface（绑定 CallbackGroup）
  auto serv_base = std::dynamic_pointer_cast<ServiceBase>(serv);
  node_services->add_service(serv_base, group);
  
  return serv;
}
```

---

## 二、AnyServiceCallback — 回调类型擦除

```cpp
// rclcpp/include/rclcpp/any_service_callback.hpp

template<typename ServiceT>
class AnyServiceCallback
{
  // 支持 4 种回调签名：
  using SharedPtrCallback = std::function<
    void(
      typename ServiceT::Request::SharedPtr,
      typename ServiceT::Response::SharedPtr)>;

  using SharedPtrWithRequestHeaderCallback = std::function<
    void(
      const std::shared_ptr<rmw_request_id_t>,
      typename ServiceT::Request::SharedPtr,
      typename ServiceT::Response::SharedPtr)>;

  // ROS2 Humble+：支持异步服务（返回 Future）
  using SharedPtrDeferredResponseCallback = std::function<
    std::shared_ptr<typename ServiceT::Response>(
      typename ServiceT::Request::SharedPtr)>;

  // 使用 std::variant 存储
  using CallbackVariant = std::variant<
    SharedPtrCallback,
    SharedPtrWithRequestHeaderCallback,
    SharedPtrDeferredResponseCallback,
    std::monostate>;  // 未设置时
    
  CallbackVariant callback_;
  
  // dispatch() 根据 variant 类型分发
  std::shared_ptr<typename ServiceT::Response>
  dispatch(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<typename ServiceT::Request> request)
  {
    auto response = std::make_shared<typename ServiceT::Response>();
    
    if (std::holds_alternative<SharedPtrCallback>(callback_)) {
      // 简单回调（无 header）
      std::get<SharedPtrCallback>(callback_)(request, response);
    } else if (std::holds_alternative<SharedPtrWithRequestHeaderCallback>(callback_)) {
      // 带 header 的回调
      std::get<SharedPtrWithRequestHeaderCallback>(callback_)(
        request_header, request, response);
    } else if (std::holds_alternative<SharedPtrDeferredResponseCallback>(callback_)) {
      // 异步回调（Humble+，服务端可延迟响应）
      response = std::get<SharedPtrDeferredResponseCallback>(callback_)(request);
    }
    return response;
  }
};
```

---

## 三、rcl_send_request() 与序列号

```c
// rcl/src/rcl/client.c

rcl_ret_t
rcl_send_request(
  const rcl_client_t * client,
  const void * ros_request,
  int64_t * sequence_number)  // ★ 输出：DDS DataWriter 的写序列号
{
  // 调用 rmw_send_request
  rmw_ret_t rmw_ret = rmw_send_request(
    client->impl->rmw_handle,
    ros_request,        // Request 消息（将被 CDR 序列化）
    sequence_number     // ★ DDS 返回的序列号
  );
  return rmw_ret == RMW_RET_OK ? RCL_RET_OK : RCL_RET_ERROR;
}

// rmw_fastrtps 层（rmw_fastrtps_cpp/src/rmw_client.cpp）：
rmw_ret_t
rmw_send_request(
  const rmw_client_t * client,
  const void * ros_request,
  int64_t * sequence_number)
{
  auto info = static_cast<CustomClientInfo *>(client->data);
  
  // 1. 序列化 Request → CDR 缓冲区
  eprosima::fastcdr::FastBuffer buffer;
  eprosima::fastcdr::Cdr ser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);
  info->type_support_->serializeROSmessage(ros_request, ser);
  
  // 2. DataWriter::write()
  eprosima::fastrtps::rtps::WriteParams wparams;
  bool res = info->request_publisher_->write(&dynamic_data, wparams);
  
  // 3. ★ 从 WriteParams 中提取序列号（DDS 自动分配的）
  *sequence_number = ((int64_t)wparams.sample_identity().sequence_number().high << 32)
                   | wparams.sample_identity().sequence_number().low;
  return RMW_RET_OK;
}
```

---

## 四、spin_until_future_complete() 内部实现

```cpp
// rclcpp/src/rclcpp/executor.cpp

template<typename FutureT, typename TimeRepT, typename TimeT>
FutureReturnCode
Executor::spin_until_future_complete(
  FutureT & future,
  std::chrono::duration<TimeRepT, TimeT> timeout)
{
  // 计算截止时间
  auto end_time = std::chrono::steady_clock::now() + timeout;
  
  while (rclcpp::ok(context_) && future.valid()) {
    // ★ 检查 future 是否已就绪（response 已到达）
    auto future_status = future.wait_for(std::chrono::seconds(0));
    if (future_status == std::future_status::ready) {
      return FutureReturnCode::SUCCESS;
    }
    
    // 超时检查
    auto remaining = end_time - std::chrono::steady_clock::now();
    if (timeout != std::chrono::duration<TimeRepT, TimeT>::zero()
        && remaining <= std::chrono::duration<TimeRepT, TimeT>::zero()) {
      return FutureReturnCode::TIMEOUT;
    }
    
    // ★ 执行一次 spin（处理所有就绪的 Executable，包括 Client 响应）
    // 这会调用 execute_client() → handle_response() → promise.set_value()
    spin_some_impl(remaining, false);
  }
  
  return FutureReturnCode::INTERRUPTED;
}
```

---

## 五、wait_for_service() 内部实现

```cpp
// rclcpp/include/rclcpp/client.hpp

template<typename RepT, typename RatioT>
bool wait_for_service(
  std::chrono::duration<RepT, RatioT> wait_for = std::chrono::duration<RepT, RatioT>(-1))
{
  return wait_for_service_nanoseconds(
    std::chrono::duration_cast<std::chrono::nanoseconds>(wait_for));
}

bool wait_for_service_nanoseconds(std::chrono::nanoseconds wait_for)
{
  auto start = std::chrono::steady_clock::now();
  
  while (rclcpp::ok(context_)) {
    // ★ 检查服务端是否在线（rmw 层：检查 DDS DataWriter 是否已匹配）
    bool is_ready;
    rcl_ret_t ret = rcl_service_server_is_available(
      node_handle_.get(),
      client_handle_.get(),
      &is_ready
    );
    
    if (is_ready) return true;
    
    // 等待超时判断
    if (wait_for >= std::chrono::nanoseconds::zero()) {
      auto elapsed = std::chrono::steady_clock::now() - start;
      if (elapsed >= wait_for) return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 轮询间隔
  }
  return false;
}

// rcl 层：rcl_service_server_is_available() → rmw_service_server_is_available()
// rmw 层（FastDDS）：检查 request DataReader 的 matched_publications > 0
// 即：服务端的 request DataWriter 已通过 SPDP/SEDP 发现并匹配
```

---

## 六、Service QoS 约束

```cpp
// Service 强制使用的 QoS（不可由用户随意更改）：
// 原因：RELIABLE 确保请求必达，VOLATILE 避免历史请求积压

rmw_qos_profile_t rmw_qos_profile_services_default = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,                               // depth = 10
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,   // ★ 必须可靠
  RMW_QOS_POLICY_DURABILITY_VOLATILE,    // ★ 不持久化
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false  // avoid_ros_namespace_conventions
};

// rclcpp 层：rclcpp::ServicesQoS 预设
// class ServicesQoS : public QoS {
//   ServicesQoS() : QoS(KeepLast(10)) {
//     reliable(); volatile_durability();
//   }
// };
```

---

## 七、Service 内省（Introspection）—— Humble 新特性

```cpp
// ROS2 Humble 引入服务内省（service introspection）：
// 可以监控 Service 请求/响应消息（类似 topic echo 但针对 service）

// 启用方式：
auto service_options = rcl_service_get_default_options();
service_options.enable_topic_endpoint_info_publisher = true;

// 效果：Node 自动创建一个 topic：
//   /<service_name>/_service_event
//   消息类型：service_msgs/msg/ServiceEventInfo + 原始请求/响应数据

// 监控命令：
// ros2 service echo /add_two_ints

// 内部实现：
// 每次 send_request / send_response 都额外发布一条 ServiceEvent 消息
// 包含：event_type（REQUEST_SENT/RESPONSE_SENT）、stamp、client_id、序列号
```
