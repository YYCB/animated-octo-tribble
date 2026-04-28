# Service/Action（一）：Service 架构与底层实现

## 一、Service 在 ROS2 的完整定位

```
┌─────────────────────────────────────────────────────────────────┐
│ 客户端进程                      服务端进程                       │
│                                                                 │
│  rclcpp::Client<T>             rclcpp::Service<T>              │
│       ↓                               ↑                         │
│  rcl_client_t                  rcl_service_t                   │
│       ↓                               ↑                         │
│  rmw_client_t                  rmw_service_t                   │
│       ↓                               ↑                         │
│  DDS DataWriter  ──rq/.../Req──► DDS DataReader                │
│  DDS DataReader  ◄─rr/.../Rep── DDS DataWriter                 │
└─────────────────────────────────────────────────────────────────┘
```

### 与 Topic 的核心区别

| 特性 | Topic | Service |
|------|-------|---------|
| 通信模式 | 发布-订阅（1:N） | 请求-响应（1:1） |
| DDS 实体 | 1个 DataWriter + N个 DataReader | 2个 DataWriter + 2个 DataReader |
| 匹配机制 | 自动匹配 | 通过 sequence_number 配对 |
| QoS | 用户配置 | 固定（RELIABLE + VOLATILE） |
| 线程 | Executor 异步 | Executor 异步（或同步 spin） |

---

## 二、DDS 层 Topic 命名规则

```
Service 名：/add_two_ints
    → 请求 Topic：rq/add_two_ints:AddTwoInts_Request_
    → 响应 Topic：rr/add_two_ints:AddTwoInts_Reply_

带命名空间：/my_ns/compute
    → 请求 Topic：rq/my_ns/compute:ComputeRequest_
    → 响应 Topic：rr/my_ns/compute:ComputeReply_
```

### 消息类型封装

```cpp
// service IDL 文件：example_interfaces/srv/AddTwoInts.srv
// ---内容---
int64 a
int64 b
---
int64 sum
// ---end---

// 编译生成的 C++ 类型：
struct AddTwoInts
{
  using Request  = example_interfaces::srv::AddTwoInts_Request;
  using Response = example_interfaces::srv::AddTwoInts_Response;
  // Request  { int64 a; int64 b; }
  // Response { int64 sum; }
  
  using ServiceTypeSupport_t = rosidl_service_type_support_t;
};
```

---

## 三、rcl_service_init() 源码

```c
// rcl/src/rcl/service.c

rcl_ret_t
rcl_service_init(
  rcl_service_t * service,
  const rcl_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rcl_service_options_t * options)
{
  // Step 1: 扩展服务名（相对 → 绝对）
  char * expanded_service_name = NULL;
  ret = rcl_expand_topic_name(
    service_name,
    rcl_node_get_name(node),
    rcl_node_get_namespace(node),
    &expanded_service_name,
    &allocator);
  
  // Step 2: 应用重映射
  ret = rcl_remap_service_name(
    &node->impl->options.arguments,
    &context->global_arguments,
    expanded_service_name,
    ..., &remapped_service_name);
  
  // Step 3: ★ 通过 rmw 创建 service（同时创建请求/响应两个 DDS Topic）
  service->impl->rmw_handle = rmw_create_service(
    node->impl->rmw_node_handle,
    type_support,               // rosidl_service_type_support_t
    remapped_service_name,      // 服务名（rmw 内部加 rq/ rr/ 前缀）
    &rmw_qos_profile            // RELIABLE + VOLATILE（固定）
  );
  
  return RCL_RET_OK;
}
```

### rmw_create_service 内部（FastDDS）

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_service.cpp

rmw_service_t * rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  // 构建请求 Topic 名：rq/<service_name>Request_
  std::string request_topic_name = _resolve_service_topic_name(
    service_name, false /* request */);  // → "rq/add_two_ints:AddTwoInts_Request_"
  
  // 构建响应 Topic 名：rr/<service_name>Reply_
  std::string response_topic_name = _resolve_service_topic_name(
    service_name, true /* response */);  // → "rr/add_two_ints:AddTwoInts_Reply_"
  
  // 创建请求 DataReader（服务端接收请求）
  info->request_reader_ = dds_participant->create_datareader(
    dds_participant->find_topic(request_topic_name, ...),
    request_datareader_qos, nullptr);
  
  // 创建响应 DataWriter（服务端发送响应）
  info->response_writer_ = dds_participant->create_datawriter(
    dds_participant->find_topic(response_topic_name, ...),
    response_datawriter_qos, nullptr);
  
  return rmw_service;
}
```

---

## 四、请求-响应序列号配对机制

```
关键数据结构：
  rmw_request_id_t {
    uint8_t writer_guid[16];  // 客户端 DataWriter 的 GUID（全局唯一）
    int64_t sequence_number;  // 递增序列号（由 DDS 自动分配）
  }

配对流程：
  Client 发送请求：
    1. rmw_send_request() → DataWriter::write(request_msg)
    2. DDS 自动分配 sequence_number = N（原子递增）
    3. 将 sequence_number N 返回给调用者

  Server 接收请求：
    4. rmw_take_request() → DataReader::read()
    5. request_header.sequence_number = N
    6. request_header.writer_guid = client 的 GUID

  Server 发送响应：
    7. rmw_send_response(request_header, response_msg)
    8. 响应 DataWriter 将 request_header 序列化到响应消息头部

  Client 接收响应：
    9. rmw_take_response() → DataReader::read()
    10. 检查 response_header.sequence_number == N（匹配）
    11. 从 pending_requests_ 映射中找到对应的 Promise/Callback
    12. 触发 Promise::set_value(response)

关键：sequence_number 是 DDS DataWriter 的写序列号，非 ROS2 层面的 ID
```

---

## 五、rcl_take_request() 与 rcl_send_response()

```c
// rcl/src/rcl/service.c

// 服务端接收请求
rcl_ret_t
rcl_take_request_with_info(
  const rcl_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request)
{
  bool taken = false;
  rmw_ret_t rmw_ret = rmw_take_request(
    service->impl->rmw_handle,
    request_header,    // 输出：{writer_guid, sequence_number, source_timestamp}
    ros_request,       // 输出：反序列化后的 Request 结构体
    &taken
  );
  return taken ? RCL_RET_OK : RCL_RET_SERVICE_TAKE_FAILED;
}

// 服务端发送响应
rcl_ret_t
rcl_send_response(
  const rcl_service_t * service,
  rmw_request_id_t * request_header,  // ★ 必须传回原始 header（含 sequence_number）
  void * ros_response)
{
  // 将 request_header 的 sequence_number 附加到响应
  // 客户端收到后依此匹配 pending_request
  return rmw_send_response(
    service->impl->rmw_handle,
    request_header,
    ros_response
  );
}
```

---

## 六、rclcpp::Service<T> 完整构造

```cpp
// rclcpp/include/rclcpp/service.hpp

template<typename ServiceT>
class Service : public ServiceBase
{
public:
  // 回调类型：(request_header, request_shared_ptr, response_shared_ptr) → void
  using CallbackType = std::function<
    void(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<typename ServiceT::Request>,
      std::shared_ptr<typename ServiceT::Response>)>;

  Service(
    std::shared_ptr<rcl_node_t> node_handle,
    const std::string & service_name,
    AnyServiceCallback<ServiceT> callback,
    rcl_service_options_t & service_options)
  : ServiceBase(node_handle)
  {
    // ★ 初始化底层 rcl service
    service_handle_ = std::make_shared<rcl_service_t>();
    *service_handle_ = rcl_get_zero_initialized_service();
    
    rcl_ret_t ret = rcl_service_init(
      service_handle_.get(),
      node_handle.get(),
      rosidl_typesupport_cpp::get_service_type_support_handle<ServiceT>(),
      service_name.c_str(),
      &service_options
    );
    
    // 存储类型安全的回调
    any_callback_ = callback;
  }
  
  // Executor 调用 execute_service() 时触发
  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> request)
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto response = std::make_shared<typename ServiceT::Response>();
    
    // 调用用户回调（填充 response）
    any_callback_.dispatch(request_header, typed_request, response);
    
    // ★ 自动发送响应
    rcl_ret_t ret = rcl_send_response(
      service_handle_.get(),
      request_header.get(),
      response.get()
    );
  }
};
```

---

## 七、rclcpp::Client<T>::async_send_request() 实现

```cpp
// rclcpp/include/rclcpp/client.hpp

template<typename ServiceT>
class Client : public ClientBase
{
public:
  // 返回 shared_future（可等待的异步结果）
  SharedFuture async_send_request(
    SharedRequest request)
  {
    return async_send_request(request, [](SharedFuture) {});
  }
  
  template<typename CallbackT>
  FutureAndRequestId async_send_request(
    SharedRequest request,
    CallbackT && callback)
  {
    // ★ 发送请求，获取 DDS 分配的序列号
    int64_t sequence_number;
    rcl_ret_t ret = rcl_send_request(
      client_handle_.get(),
      request.get(),
      &sequence_number       // 输出：DDS 分配的序列号
    );
    
    // 创建 Promise，存入 pending_requests_ 映射
    auto promise = std::make_shared<Promise>();
    SharedFuture future(promise->get_future().share());
    
    {
      std::lock_guard<std::mutex> lock(pending_requests_mutex_);
      // key = sequence_number，value = {promise, callback}
      pending_requests_[sequence_number] = std::make_tuple(
        promise,
        std::forward<CallbackT>(callback));
    }
    
    return std::make_tuple(future, sequence_number);
  }
  
  // Executor 调用：处理接收到的响应
  void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response)
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    
    // ★ 通过 sequence_number 找到对应的 Promise
    auto it = pending_requests_.find(request_header->sequence_number);
    if (it == pending_requests_.end()) {
      // 没有找到对应请求（可能已超时取消）
      return;
    }
    
    auto & [promise, callback] = it->second;
    auto typed_response = std::static_pointer_cast<
      typename ServiceT::Response>(response);
    
    // ★ 完成 Promise（唤醒 spin_until_future_complete）
    promise->set_value(typed_response);
    
    // 调用用户回调（如果注册了）
    SharedFuture shared_future(promise->get_future().share());
    callback(shared_future);
    
    pending_requests_.erase(it);
  }

private:
  // 待响应的请求映射：sequence_number → (Promise, Callback)
  std::map<int64_t, std::tuple<Promise::SharedPtr, CallbackType>> pending_requests_;
  std::mutex pending_requests_mutex_;
};
```

---

## 八、execute_service() 与 execute_client() 在 Executor 中

```cpp
// rclcpp/src/rclcpp/executor.cpp

void Executor::execute_service(rclcpp::ServiceBase::SharedPtr service)
{
  // 1. 分配请求 header 和 request 对象
  auto request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  
  // 2. 从 rcl_service 取出请求（rcl_take_request 内部调用）
  bool taken = false;
  service->take_type_erased_request(request.get(), *request_header, taken);
  if (!taken) return;
  
  // 3. 调用 handle_request（用户回调 + 自动 send_response）
  service->handle_request(request_header, request);
}

void Executor::execute_client(rclcpp::ClientBase::SharedPtr client)
{
  // 1. 分配响应 header 和 response 对象
  auto request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  
  // 2. 从 rcl_client 取出响应（rcl_take_response）
  bool taken = false;
  client->take_type_erased_response(response.get(), *request_header, taken);
  if (!taken) return;
  
  // 3. 触发 Promise 完成
  client->handle_response(request_header, response);
}
```

---

## 九、Service 请求超时处理

```cpp
// rclcpp 没有在 rcl 层实现超时，需要应用层处理

// 方式1：spin_until_future_complete 指定 timeout
auto future = client->async_send_request(request);
auto result = executor.spin_until_future_complete(future, std::chrono::seconds(5));
if (result == rclcpp::FutureReturnCode::TIMEOUT) {
  RCLCPP_WARN(logger, "Service call timed out!");
}

// 方式2：wait_for_service 检查服务是否可用
if (!client->wait_for_service(std::chrono::seconds(1))) {
  throw std::runtime_error("Service not available");
}

// wait_for_service 内部实现：
// 1. 定期调用 rcl_service_server_is_available()
// 2. rcl_service_server_is_available() → rmw_service_server_is_available()
// 3. rmw 层检查是否有 DataWriter 匹配 rq/.../Request DataReader
// （即检查服务端是否已创建并发现）
```

---

## 十、完整请求-响应时序图

```
时间轴 ───────────────────────────────────────────────────────────►

Client 线程                  DDS 网络                  Server 线程（Executor）
    │                           │                            │
    │ async_send_request(req)   │                            │
    ├──rcl_send_request()──────►│                            │
    │  seq_num=42 returned      │   rq/.../Request (seq=42)  │
    │  stored in pending_[42]   ├───────────────────────────►│
    │                           │                     rcl_wait() 返回
    │                           │                     execute_service()
    │                           │                     rcl_take_request(header={seq=42})
    │                           │                     user_callback(req, resp)
    │                           │   rr/.../Reply (seq=42)    │
    │                          ◄├───────────────────────────┤ rcl_send_response()
    │  rcl_wait() 返回          │                            │
    │  execute_client()         │                            │
    │  rcl_take_response(seq=42)│                            │
    │  pending_[42].set_value() │                            │
    │  future.get() 返回        │                            │
    │                           │                            │
```
