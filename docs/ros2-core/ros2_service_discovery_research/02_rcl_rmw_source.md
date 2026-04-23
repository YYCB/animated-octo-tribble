# ROS2 服务发现（二）：rcl/rmw 层源码详解

## 一、rcl 层服务发现 API

`rcl`（ROS Client Library C 语言层）为上层（rclcpp/rclpy）提供了一致的服务发现接口，内部委托给 `rmw`。

---

## 二、rcl_node 与 DDS 参与者的绑定

### 2.1 rcl_node_init() 源码流程

```c
// rcl/src/rcl/node.c

rcl_ret_t
rcl_node_init(
  rcl_node_t * node,
  const char * name,
  const char * namespace_,
  rcl_context_t * context,
  const rcl_node_options_t * options)
{
  // Step 1: 验证参数（名称合法性检查）
  RCL_CHECK_ARGUMENT_FOR_NULL(node, RCL_RET_INVALID_ARGUMENT);
  
  // Step 2: 通过 rmw 创建节点（最终创建 DDS 参与者）
  node->impl->rmw_node_handle = rmw_create_node(
    &context->impl->rmw_context,
    name,
    namespace_
  );
  
  // Step 3: 获取图变化 guard condition
  // 当任意发现事件发生时，这个 guard condition 会被触发
  node->impl->graph_guard_condition =
    rmw_node_get_graph_guard_condition(node->impl->rmw_node_handle);
  
  // Step 4: 如果 rosout 可用，注册日志发布者
  if (options->enable_rosout) {
    rcl_ret_t ret = rcl_logging_rosout_init_publisher_for_node(node);
  }
  
  return RCL_RET_OK;
}
```

### 2.2 rmw_create_node()（以 rmw_fastrtps 为例）

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_node.cpp

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  // 获取或创建 DomainParticipant（复用同一进程内的已有参与者）
  // ROS2 每个进程（context）共享一个 DomainParticipant
  auto common_context = static_cast<rmw_dds_common::Context *>(context->impl->common_);
  
  rmw_node_t * node = rmw_node_allocate();
  
  // ★ ROS2 节点信息通过发布特殊 Topic 来宣告
  // Topic: "ros_discovery_info"（rmw_dds_common 定义）
  // 这是 ROS2 在 DDS 之上的附加发现层
  
  // 更新参与者节点列表
  common_context->graph_cache.add_node(
    common_context->gid,
    name,
    namespace_
  );
  
  // 发布更新后的节点信息到 ros_discovery_info topic
  common_context->publish_graph_cache_update();
  
  return node;
}
```

---

## 三、ROS2 的双层发现机制

ROS2 实际使用**两层发现机制**：

```
┌─────────────────────────────────────────────────────────────────┐
│  Layer 2: ROS2 Graph Cache（上层，节点级发现）                   │
│                                                                  │
│  通过特殊 DDS Topic: "ros_discovery_info" 传递节点信息           │
│  rmw_dds_common::GraphCache 维护节点→端点的映射关系              │
│                                                                  │
│  实现：rmw_dds_common 包                                         │
└───────────────────────┬─────────────────────────────────────────┘
                        │ 依赖
┌───────────────────────▼─────────────────────────────────────────┐
│  Layer 1: DDS 底层发现（参与者/端点级发现）                       │
│                                                                  │
│  SPDP: 发现 DomainParticipant                                    │
│  SEDP: 发现 DataWriter / DataReader                              │
│                                                                  │
│  实现：FastDDS/CycloneDDS 内建协议                               │
└─────────────────────────────────────────────────────────────────┘
```

### 3.1 为什么需要两层？

DDS 层发现的是**端点（DataWriter/DataReader）**，但 ROS2 需要**节点（Node）**级别的信息。一个 ROS2 节点可能创建多个 DDS 端点，DDS 本身无法建立"哪些端点属于同一个 ROS 节点"的关联。

---

## 四、rmw_dds_common：ROS2 图缓存

### 4.1 包位置

```
ros2/rmw_dds_common/
├── rmw_dds_common/
│   ├── include/rmw_dds_common/
│   │   ├── graph_cache.hpp          # ★★★ 图缓存核心
│   │   ├── context.hpp              # rmw context 扩展
│   │   └── gid_utils.hpp            # GUID 工具
│   └── src/
│       ├── graph_cache.cpp          # ★★★
│       └── context.cpp
```

### 4.2 GraphCache 结构

```cpp
// rmw_dds_common/include/rmw_dds_common/graph_cache.hpp

class GraphCache
{
public:
  // 添加/删除参与者
  void add_participant(
    const rmw_gid_t & participant_gid,
    const std::string & enclave);
  void remove_participant(const rmw_gid_t & participant_gid);

  // 添加/删除节点（关联到参与者）
  void add_node(
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);
  void remove_node(
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  // 关联端点到节点（当 DataWriter/Reader 被发现时）
  void associate_writer(
    const rmw_gid_t & writer_gid,
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  // 查询接口
  rmw_ret_t get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_string_array_t * enclaves,
    rcutils_allocator_t * allocator) const;

  rmw_ret_t get_writer_names_and_types_by_node(
    const std::string & node_name,
    const std::string & node_namespace,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * topic_names_and_types) const;

private:
  // 核心数据结构：参与者 GID → 节点列表
  ParticipantsMapType participants_;
  
  // 端点 GID → 关联信息（参与者、节点名）
  EntityKeyValueMap<rmw_gid_t, EntityInfo> graph_by_writer_gid_;
  EntityKeyValueMap<rmw_gid_t, EntityInfo> graph_by_reader_gid_;
  
  mutable std::mutex mutex_;
};
```

### 4.3 ros_discovery_info Topic

ROS2 在所有参与者之间共享一个特殊的 DDS Topic：

```cpp
// rmw_dds_common/src/context.cpp

// Topic 名称
constexpr const char * kROS2NodeInfoTopicName = "ros_discovery_info";

// 消息类型（Protobuf-like IDL）
// rmw_dds_common/msg/ParticipantEntitiesInfo.msg:
//   rmw_dds_common/Gid gid          # 参与者 GUID
//   rmw_dds_common/NodeEntitiesInfo[] node_entities_info_seq
//     string node_name
//     string node_namespace
//     rmw_dds_common/Gid[] writer_gid_seq   # 该节点的所有 Writer
//     rmw_dds_common/Gid[] reader_gid_seq   # 该节点的所有 Reader
```

**信息流**：

```
节点 A 创建 Publisher "chatter"
    │
    ├─ DDS 层：FastDDS 自动通过 SEDP 广播新 DataWriter 存在
    │
    └─ ROS2 层：rmw_fastrtps 更新 GraphCache
               └─ 发布更新后的 ParticipantEntitiesInfo
                  到 "ros_discovery_info" Topic
                  （其他节点订阅此 Topic，更新本地 GraphCache）
```

---

## 五、FastDDS 中 rmw_get_topic_names_and_types 实现

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp

rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  auto node_info = static_cast<CustomNodeInfo *>(node->data);
  auto common_context = node_info->participant_info->common;
  
  // 直接从本地 GraphCache 查询（无需网络请求！）
  // GraphCache 已通过订阅 ros_discovery_info 保持最新
  return common_context->graph_cache.get_writer_names_and_types(
    no_demangle ? passthrough_demangle : _demangle_ros_topic_prefix_and_remove_suffix,
    no_demangle ? passthrough_demangle : _demangle_service_type_only,
    allocator,
    topic_names_and_types
  );
}
```

**关键点**：`get_topic_names_and_types` **不发网络请求**，直接读本地缓存！

---

## 六、服务发现延迟分析

```
新节点启动到被其他节点发现的时序：

T+0ms:    新节点进程启动
T+0ms:    rcl_init() → rmw_create_context() → 创建 DomainParticipant
T+0ms:    PDPSimple::initPDP() → 首次 SPDP 宣告广播
T+??ms:   其他节点的 SPDP Reader 收到广播（网络延迟，通常 < 1ms LAN）
T+??ms:   其他节点触发 SEDP：互相发送端点信息
T+??ms:   SEDP 完成 → DDS 端点匹配建立
T+??ms:   ros_discovery_info Topic 更新传播
T+??ms:   其他节点的 GraphCache 更新完成
T+??ms:   ros2 node list 可看到新节点

典型 LAN 环境总延迟：10~500ms
影响因素：
  - SPDP 宣告周期（默认 100ms，可配置）
  - 网络延迟
  - 参与者数量（SEDP 两两建立连接 = O(n²) 握手）
  - QoS 不兼容时无法匹配（静默失败）
```

---

## 七、rcl_get_node_names 完整调用链

```
rclcpp::Node::get_node_names()
    │
    └─► rcl_get_node_names(node, allocator, &names, &namespaces)
            │ (rcl/src/rcl/graph.c)
            │
            └─► rmw_get_node_names(rmw_node, &names, &namespaces)
                    │ (rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp)
                    │
                    └─► common_context->graph_cache.get_node_names(...)
                            │ (rmw_dds_common/src/graph_cache.cpp)
                            │
                            └─► 遍历 participants_ map，收集所有节点名
                                （本地内存操作，O(n)）
```

---

## 八、服务可用性检查（rmw_service_server_is_available）

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_service.cpp

rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  *is_available = false;
  
  auto client_info = static_cast<CustomClientInfo *>(client->data);
  
  // 检查 Response Subscription 是否有匹配的 Publisher
  // 即：是否存在服务端的 Response DataWriter
  size_t matched_pub_count = 0;
  client_info->response_subscriber_->get_matched_publisher_count(matched_pub_count);
  
  if (matched_pub_count == 0) {
    return RMW_RET_OK;  // 未找到服务端
  }
  
  // 同时检查 Request Publisher 是否有匹配的 Subscriber
  size_t matched_sub_count = 0;
  client_info->request_publisher_->get_matched_subscriber_count(matched_sub_count);
  
  *is_available = (matched_pub_count > 0) && (matched_sub_count > 0);
  return RMW_RET_OK;
}
```

---

## 九、ROS2 服务（Service）的 DDS 映射

ROS2 Service 不是 DDS 原生概念，它通过**两个 Topic**模拟实现：

```
ROS2 Service /add_two_ints
  │
  ├─ Request Topic:  "rq/add_two_ints/AddTwoIntsRequest_"
  │   ├─ Client 侧：DataWriter（发送请求）
  │   └─ Server 侧：DataReader（接收请求）
  │
  └─ Response Topic: "rr/add_two_ints/AddTwoIntsReply_"
      ├─ Server 侧：DataWriter（发送响应）
      └─ Client 侧：DataReader（接收响应）

请求-响应匹配机制：
  每个请求携带唯一 sequence_number，
  响应中包含对应的 sequence_number，
  Client 通过 sequence_number 匹配响应到对应的 future/promise
```

---

## 十、网络配置与跨子网发现

### 10.1 单播发现（替代多播）

在不支持多播的网络（如 AWS/K8s）中，可配置静态单播发现：

**FastDDS XML 配置**：
```xml
<!-- ros2_fastdds_profile.xml -->
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_xrce_participant" is_default_profile="true">
      <rtps>
        <builtin>
          <discovery_config>
            <discoveryProtocol>SIMPLE</discoveryProtocol>
            <discoveryServersList>
              <!-- 静态已知参与者地址 -->
              <RemoteServer prefix="44.53.00.f1.1e.a5.d9.81.aa.26.77.00">
                <metatrafficUnicastLocatorList>
                  <locator>
                    <udpv4>
                      <address>192.168.1.100</address>
                      <port>11811</port>
                    </udpv4>
                  </locator>
                </metatrafficUnicastLocatorList>
              </RemoteServer>
            </discoveryServersList>
          </discovery_config>
          <initialPeersList>
            <locator>
              <udpv4>
                <address>192.168.1.101</address>  <!-- 手动指定对端 -->
                <port>7412</port>
              </udpv4>
            </locator>
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
```

```bash
# 使用 XML 配置文件
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/ros2_fastdds_profile.xml
```

### 10.2 FastDDS Discovery Server（中心化扩展方案）

对于大规模系统（100+ 节点），多播性能下降，可使用 Discovery Server：

```
                Discovery Server（专用进程）
                        │
            ┌───────────┼───────────┐
            │           │           │
         节点A         节点B        节点C
    （只连接 Server，不做全量 SPDP 广播）
```

```bash
# 启动 Discovery Server（FastDDS 专有）
fastdds discovery -i 0 -p 11811

# 节点连接 Discovery Server
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# 使用 super client 模式（可查询所有已发现端点）
export ROS_DISCOVERY_SERVER=";;127.0.0.1:11811"
```

### 10.3 CycloneDDS 跨子网配置

```xml
<!-- cyclonedds.xml -->
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" multicast="true"/>
      </Interfaces>
      <!-- 跨子网：指定已知端点 -->
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.100"/>
        <Peer address="192.168.2.100"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

---

## 十一、ROS_DOMAIN_ID 隔离机制

```bash
# 不同 domain 的节点互相不可见（完全隔离）
export ROS_DOMAIN_ID=0   # 默认，范围 0-101
export ROS_DOMAIN_ID=42  # 自定义 domain

# domain_id 直接影响 DDS 端口号：
# SPDP 多播端口 = 7400 + 250 * domain_id
# 因此 domain 0 使用 7400，domain 1 使用 7650，domain 42 使用 17900
```

### 实现层（DDS 层）

```cpp
// FastDDS 中的 domain 隔离
eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
  ->create_participant(
    domain_id,    // 直接传入，不同 domain 使用不同端口 → 完全网络隔离
    participant_qos
  );
```

---

## 十二、常见发现问题排查

### Q1: `ros2 node list` 显示不完整

```bash
# 原因1：发现延迟（等待 SPDP 周期）
# 解决：等待 1-2 秒后重试

# 原因2：多播被防火墙/路由器阻断
# 解决：使用单播发现，或禁用防火墙
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp  # TCP 传输时

# 原因3：不同 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0  # 确保一致

# 诊断：抓包查看 SPDP 广播
sudo tcpdump -i any -n 'udp port 7400'
```

### Q2: 服务发现后 topic 不匹配

```bash
# 检查 QoS 兼容性
ros2 topic info /chatter --verbose
# 查看 Publisher 和 Subscriber 的 QoS 是否兼容

# 常见原因：Reliability 不匹配
# Publisher: BEST_EFFORT，Subscriber: RELIABLE → 不匹配
```

### Q3: Discovery Server 模式下 `ros2 topic list` 无结果

```bash
# Discovery Server 模式下，普通客户端只看到直连的端点
# 需要使用 super client 模式
export ROS_DISCOVERY_SERVER=";;127.0.0.1:11811"  # 前缀 ;; 表示 super client
```

---

## 十三、源码阅读推荐路径

```
1. ROS2 双层发现机制入口：
   rmw_dds_common/src/context.cpp   # ros_discovery_info 订阅初始化
   rmw_dds_common/src/graph_cache.cpp # 图缓存操作

2. FastDDS SPDP：
   Fast-DDS/src/cpp/rtps/builtin/discovery/protocol/PDPSimple.cpp

3. FastDDS SEDP：
   Fast-DDS/src/cpp/rtps/builtin/discovery/protocol/EDPSimple.cpp

4. rmw_fastrtps 接口实现：
   rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp
   rmw_fastrtps_shared_cpp/src/rmw_node.cpp

5. rcl 图查询：
   rcl/src/rcl/graph.c

6. rclcpp 封装：
   rclcpp/src/rclcpp/node.cpp → get_node_names()
```
