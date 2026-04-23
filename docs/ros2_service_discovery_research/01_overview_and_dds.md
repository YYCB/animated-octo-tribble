# ROS2 服务发现（Service Discovery）机制深度调研

## 一、ROS2 服务发现概述

ROS2 放弃了 ROS1 的中心化 `rosmaster`，改用**去中心化的 DDS（Data Distribution Service）服务发现协议**。这意味着节点之间无需注册到中央服务器，可以通过网络自动发现彼此。

### ROS1 vs ROS2 发现机制对比

| 维度 | ROS1 | ROS2 |
|------|------|------|
| 架构 | 集中式（rosmaster） | 去中心化（DDS Discovery） |
| 单点故障 | 有（rosmaster 崩溃=全崩） | 无 |
| 网络协议 | XMLRPC + TCPROS/UDPROS | RTPS（UDP/TCP） |
| 跨子网 | 不支持（默认） | 支持（需配置） |
| 安全 | 无内建安全 | SROS2（DDS Security） |
| 发现速度 | 快（中心化查询） | 较慢（广播+等待） |
| 离线感知 | 需手动注销 | 自动超时检测 |

---

## 二、协议栈总览

```
┌─────────────────────────────────────────────────────────────────┐
│                      ROS2 应用层                                 │
│  rcl_get_node_names()  │  rcl_get_topic_names_and_types()        │
│  rcl_get_service_names_and_types()                               │
└───────────────────────┬─────────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────────┐
│                   rcl  (C Client Library)                        │
│  rcl_node  │  rcl_context  │  graph 事件系统                     │
└───────────────────────┬─────────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────────┐
│              rmw  (ROS Middleware Interface)                      │
│  rmw_get_node_names()  │  rmw_get_topic_names_and_types()         │
│  rmw_service_server_is_available()                               │
└───────────────────────┬─────────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────────┐
│         DDS Implementation（具体中间件）                          │
│  rmw_fastrtps_cpp  │  rmw_cyclonedds_cpp  │  rmw_connextdds      │
└───────────────────────┬─────────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────────┐
│                  RTPS 协议层 (DDS 底层)                           │
│  SPDP（参与者发现）  │  SEDP（端点发现）                          │
└───────────────────────┬─────────────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────────────┐
│              UDP/TCP 传输层 + 多播/单播                           │
│  UDP 多播（发现）  │  UDP/TCP 单播（数据传输）                     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 三、DDS 核心概念速查

| 概念 | 描述 | ROS2 对应 |
|------|------|----------|
| Domain | 逻辑隔离单元（0-232） | `ROS_DOMAIN_ID` 环境变量 |
| DomainParticipant | 域中的参与者 | 每个 `rcl_context` 对应一个 |
| Publisher | DDS 发布者（包含 DataWriter） | `rclcpp::Publisher` |
| Subscriber | DDS 订阅者（包含 DataReader） | `rclcpp::Subscription` |
| Topic | 具名数据通道 | ROS2 Topic（加前缀） |
| DataWriter | 写入 Topic 数据 | Publisher 内部 |
| DataReader | 读取 Topic 数据 | Subscription 内部 |
| QoS | Quality of Service 策略 | `rclcpp::QoS` |
| RTPS | Real-Time Publish-Subscribe（DDS 底层协议） | 所有 DDS 实现 |

---

## 四、RTPS 协议详解

RTPS（Real-Time Publish-Subscribe）是 DDS 的底层协议，定义在 **OMG DDSI-RTPS** 规范中。

### 4.1 RTPS 消息结构

```
RTPS Message
├── Header
│   ├── Magic Number ("RTPS")
│   ├── Protocol Version (2.x)
│   ├── Vendor ID（FastDDS: 0x0101, CycloneDDS: 0x0110）
│   └── GuidPrefix（12字节，唯一标识发送方）
└── SubMessages[]
    ├── INFO_TS       # 时间戳
    ├── DATA          # 数据
    ├── HEARTBEAT     # 可靠传输心跳
    ├── ACKNACK       # 确认/否定确认
    └── INFO_DST      # 目标 GUID 前缀
```

### 4.2 GUID 结构

```
GUID (128 bits)
├── GuidPrefix (96 bits)
│   ├── VendorId    (16 bits)  # DDS 厂商标识
│   ├── HostId      (32 bits)  # 主机标识（IP 地址派生）
│   ├── ProcessId   (32 bits)  # 进程 PID 派生
│   └── ParticipantId (16 bits)# 同进程多 participant 区分
└── EntityId (32 bits)
    ├── EntityKey   (24 bits)  # 实体编号
    └── EntityKind  (8 bits)   # 类型标识
        ├── 0x01 = Writer with Key
        ├── 0x02 = Writer without Key
        ├── 0x04 = Reader with Key（Subscription）
        ├── 0x07 = Reader without Key
        ├── 0xC1 = Participant（特殊内建实体）
        ├── 0xC2 = SEDP Writer（内建）
        └── 0xC7 = SEDP Reader（内建）
```

---

## 五、SPDP：简单参与者发现协议

**SPDP（Simple Participant Discovery Protocol）** 负责发现网络中的 DomainParticipant。

### 5.1 工作流程

```
新节点启动
    │
    ├─► 创建 SPDP Writer（内建，预定义 GUID: ...0000C2）
    │   └─► 定期向多播地址发送 ParticipantProxyData（宣告自身存在）
    │
    ├─► 创建 SPDP Reader（内建，预定义 GUID: ...0000C7）
    │   └─► 监听多播地址上的参与者宣告
    │
    └─► 收到其他参与者的 SPDPdiscoveredParticipantData
        └─► 触发 SEDP（端点发现）
```

### 5.2 SPDP 多播地址

```
默认多播地址：239.255.0.1
默认发现端口：7400 + 250 * domain_id

# Domain 0 的 SPDP 多播端口：
发现端口 = 7400 + 250 * 0 = 7400 (SPDP 多播)
单播端口 = 7412 + 2 * participant_id (SEDP/用户数据)
```

**端口计算公式（DDS 规范）**：
```
PB = 7400                          # 端口基准
DG = 250                           # 域增量
PG = 2                             # 参与者增量
d0 = 0 (SPDP 多播偏移)
d1 = 10 (SPDP 单播偏移)
d2 = 1  (用户多播偏移)
d3 = 11 (用户单播偏移)

SPDP 多播端口 = PB + DG * domain_id + d0
SPDP 单播端口 = PB + DG * domain_id + d1 + PG * participant_id
用户多播端口  = PB + DG * domain_id + d2
用户单播端口  = PB + DG * domain_id + d3 + PG * participant_id
```

### 5.3 SPDPdiscoveredParticipantData 结构

```cpp
// 参与者宣告数据（精简）
struct SPDPdiscoveredParticipantData {
  ProtocolVersion     protocolVersion;   // RTPS 版本
  VendorId            vendorId;          // DDS 厂商
  GuidPrefix          guidPrefix;        // 参与者唯一标识
  Locator_t           defaultUnicastLocator;   // 单播地址:端口
  Locator_t           defaultMulticastLocator; // 多播地址（可选）
  Locator_t           metatrafficUnicastLocator;
  Locator_t           metatrafficMulticastLocator;
  Duration_t          leaseDuration;     // 租约时间（超时则认为参与者离线）
  BuiltinEndpointSet  availableBuiltinEndpoints; // 支持的内建端点
  // ROS2 扩展字段
  uint32_t            domainId;
};
```

---

## 六、SEDP：简单端点发现协议

**SEDP（Simple Endpoint Discovery Protocol）** 在 SPDP 发现参与者后，进一步发现 DataWriter 和 DataReader 端点。

### 6.1 SEDP 实体（内建 Topic）

DDS 为每个参与者创建 4 个内建端点：

```
内建端点（BuiltinEndpoints）
├── Publications Writer   # 发布本参与者的 DataWriter 信息
├── Publications Reader   # 接收其他参与者的 DataWriter 信息
├── Subscriptions Writer  # 发布本参与者的 DataReader 信息
└── Subscriptions Reader  # 接收其他参与者的 DataReader 信息
```

### 6.2 DiscoveredWriterData（SEDP Publications）

```cpp
struct DiscoveredWriterData {
  // 端点描述
  WriterProxy_t writerProxy;
  
  // 匹配信息
  TopicName_t     topicName;        // Topic 名称
  TypeName_t      typeName;         // 消息类型名
  
  // QoS 策略（用于兼容性检查）
  DurabilityQosPolicy         durability;
  DeadlineQosPolicy           deadline;
  OwnershipQosPolicy          ownership;
  OwnershipStrengthQosPolicy  ownershipStrength;
  LifespanQosPolicy           lifespan;
  ReliabilityQosPolicy        reliability;    // BEST_EFFORT / RELIABLE
  DestinationOrderQosPolicy   destinationOrder;
  
  // 定位信息（地址:端口）
  LocatorList_t   unicastLocatorList;
  LocatorList_t   multicastLocatorList;
};
```

### 6.3 QoS 兼容性矩阵

SEDP 发现端点后，DDS 会检查 Publisher/Subscriber 的 QoS 是否兼容（不兼容则不匹配）：

```
QoS 兼容性规则（Publisher vs Subscriber）：

Reliability:
  Publisher RELIABLE   + Subscriber RELIABLE   → ✓ 匹配
  Publisher RELIABLE   + Subscriber BEST_EFFORT→ ✓ 匹配
  Publisher BEST_EFFORT+ Subscriber RELIABLE   → ✗ 不匹配

Durability:
  Publisher TRANSIENT_LOCAL + Subscriber VOLATILE       → ✓ 匹配
  Publisher VOLATILE        + Subscriber TRANSIENT_LOCAL→ ✗ 不匹配

Deadline（Publisher.period <= Subscriber.period → 匹配）
Liveliness（Publisher.kind >= Subscriber.kind → 匹配，MANUAL > AUTOMATIC）
```

---

## 七、FastDDS 发现实现源码分析

FastDDS（eProsima）是 ROS2 默认的 DDS 实现（Humble 及之前版本）。

### 7.1 关键源码路径

```
eProsima/Fast-DDS/
├── src/cpp/
│   ├── rtps/
│   │   ├── participant/
│   │   │   ├── RTPSParticipant.cpp           # 参与者实现
│   │   │   └── RTPSParticipantImpl.cpp       # ★ 内部实现（发现初始化）
│   │   ├── builtin/
│   │   │   ├── discovery/
│   │   │   │   ├── protocol/
│   │   │   │   │   ├── PDPSimple.cpp         # ★★ SPDP 实现
│   │   │   │   │   ├── PDP.cpp               # PDP 基类
│   │   │   │   │   └── EDPSimple.cpp         # ★★ SEDP 实现
│   │   │   │   └── endpoint/
│   │   │   │       ├── EDP.cpp               # EDP 基类
│   │   │   │       └── EDPSimpleListeners.cpp # 发现事件监听
│   │   │   ├── liveliness/
│   │   │   │   └── WLP.cpp                   # Liveliness 协议
│   │   │   └── network/
│   │   │       └── NetworkFactory.cpp        # 网络接口管理
│   │   └── transport/
│   │       ├── UDPv4Transport.cpp            # UDP IPv4 传输
│   │       └── TCPv4Transport.cpp            # TCP 传输
└── include/fastdds/
    └── dds/domain/
        └── DomainParticipant.hpp
```

### 7.2 PDPSimple（SPDP 实现）核心逻辑

```cpp
// Fast-DDS: src/cpp/rtps/builtin/discovery/protocol/PDPSimple.cpp

// 初始化 SPDP（在 RTPSParticipantImpl 启动时调用）
bool PDPSimple::initPDP(RTPSParticipantImpl* part)
{
    // 1. 创建内建 SPDP Writer（用于发送本参与者信息）
    if (!createSPDPEndpoints()) return false;
    
    // 2. 读取配置（多播地址、发现端口等）
    // 从 ParticipantAttributes.builtin.discovery_config 读取
    
    // 3. 启动周期性宣告定时器
    resend_participant_info_event_ = new TimedEvent(
        part->getEventResource(),
        [this]() { announceParticipantState(false); return true; },
        TimeConv::Duration_t2MilliSecondsDouble(
            mp_builtin->m_att.discovery_config.leaseDuration_announcementperiod)
    );
    
    return true;
}

// 周期性广播参与者信息
void PDPSimple::announceParticipantState(bool new_change, bool dispose)
{
    // 构建 SPDPdiscoveredParticipantData
    ParticipantProxyData& pdata = *getLocalParticipantProxyData();
    
    // 写入内建 Writer（发送到多播地址）
    if (new_change || !dispose) {
        mp_PDPWriter->new_change(ALIVE, pdata_serialized);
    } else {
        mp_PDPWriter->new_change(NOT_ALIVE_DISPOSED, pdata_serialized);
    }
}

// 接收到其他参与者的宣告
void PDPSimple::processPDPMsg(
    const RTPSReader* reader,
    CacheChange_t* change)
{
    // 反序列化 SPDPdiscoveredParticipantData
    ParticipantProxyData participant_data;
    if (!participant_data.readFromCDRMessage(change)) return;
    
    // 注册/更新参与者代理
    auto [added, proxy] = addOrUpdateParticipantProxyData(participant_data);
    
    if (added) {
        // 新参与者发现 → 启动 SEDP 端点发现
        mp_EDP->assignRemoteEndpoints(proxy);
        
        // 重置租约过期定时器
        proxy->lease_duration_event->update_interval(participant_data.m_leaseDuration);
        proxy->lease_duration_event->restart_timer();
    }
}
```

### 7.3 EDPSimple（SEDP 实现）核心逻辑

```cpp
// Fast-DDS: src/cpp/rtps/builtin/discovery/protocol/EDPSimple.cpp

bool EDPSimple::initEDP(
    BuiltinProtocols* p_builtin,
    RTPSParticipantImpl* p_RTPSParticipant)
{
    // 创建 4 个内建端点：
    // Publications Writer + Reader
    // Subscriptions Writer + Reader
    if (!createSEDPEndpoints()) return false;
    return true;
}

// 当新的 DataWriter 在远端被发现时
void EDPSimpleListeners::onNewCacheChangeAdded(
    RTPSReader* reader,
    const CacheChange_t* const change)
{
    if (reader == publications_reader) {
        // 反序列化 DiscoveredWriterData
        WriterProxyData writer_data;
        writer_data.readFromCDRMessage(change);
        
        // 查找本地匹配的 DataReader
        for (auto& reader_proxy : local_readers_) {
            if (topicMatch(writer_data, reader_proxy) &&
                qosMatch(writer_data.qos, reader_proxy.qos)) {
                // ★ 建立匹配连接
                mp_RTPSParticipant->createReader2WriterConnection(
                    reader_proxy, writer_data.endpoint);
            }
        }
    }
}
```

---

## 八、CycloneDDS 发现实现

CycloneDDS（Eclipse）是 ROS2 Humble 之后可选的高性能 DDS。

### 8.1 关键源码路径

```
eclipse-cyclonedds/cyclonedds/
└── src/
    ├── core/ddsi/
    │   ├── src/
    │   │   ├── ddsi_discovery.c       # ★★ 发现协议主体
    │   │   ├── ddsi_pmd.c             # SPDP 数据包处理
    │   │   ├── ddsi_sedp.c            # ★★ SEDP 实现
    │   │   ├── ddsi_entity.c          # 实体（参与者/端点）管理
    │   │   ├── ddsi_participant.c     # 参与者管理
    │   │   └── ddsi_proxy_participant.c # 远端参与者代理
    │   └── include/dds/ddsi/
    │       ├── ddsi_discovery.h
    │       └── ddsi_entity.h
    └── core/ddsc/
        └── src/
            └── dds_participant.c      # DDS API 参与者实现
```

### 8.2 CycloneDDS 发现关键函数

```c
// eclipse-cyclonedds/cyclonedds/src/core/ddsi/src/ddsi_discovery.c

// SPDP 写入（宣告本参与者）
static void write_pmd_message(struct ddsi_domaingv *gv, struct ddsi_participant *pp)
{
    // 构建 SPDPdiscoveredParticipantData 序列化数据
    struct ddsi_serdata *serdata = ddsi_serdatapool_new(gv->serpool);
    
    // 写入内建 SPDP Writer
    write_and_fini_plist(pp->builtin_writers[0], serdata, false);
}

// 接收 SPDP 数据（发现远端参与者）
int ddsi_handle_spdp(
    const struct ddsi_receiver_state *rst,
    ddsi_entityid_t pwr_entityid,
    ddsi_seqno_t seq,
    const ddsi_plist_t *datap)
{
    // 查找已知参与者
    struct ddsi_proxy_participant *proxypp;
    struct ddsi_guid guid;
    // ...
    
    if (!ddsi_lookup_proxy_participant(gv, &guid, &proxypp)) {
        // 新参与者：创建代理
        ddsi_new_proxy_participant(gv, &guid, datap, rst);
        
        // 触发 SEDP：向新参与者发送本地端点信息
        ddsi_sedp_write_writer(gv, ...);
        ddsi_sedp_write_reader(gv, ...);
    } else {
        // 已知参与者：更新租约
        ddsi_update_proxy_participant_plist(proxypp, datap);
    }
    return 0;
}
```

---

## 九、ROS2 对 DDS 发现的封装（rmw 层）

### 9.1 rmw 接口

```c
// rmw/include/rmw/rmw.h

// 获取所有节点名称（查询 DDS 发现数据库）
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces);

// 获取所有 Topic 名称和类型
rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types);

// 检查服务是否可用
rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available);

// 等待图变化事件
rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout);
```

### 9.2 rmw_fastrtps 实现（rmw_get_node_names）

```cpp
// rmw_fastrtps/rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp

rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  // 从 CustomParticipantInfo 中获取 FastDDS DomainParticipant
  auto participant_info = static_cast<CustomParticipantInfo *>(node->data);
  eprosima::fastdds::dds::DomainParticipant * participant =
    participant_info->participant_;
  
  // 查询 FastDDS 发现数据库
  eprosima::fastdds::dds::builtin::ParticipantBuiltinTopicData participant_data;
  
  // 遍历已发现的参与者
  std::vector<InstanceHandle_t> handles;
  participant->get_discovered_participants(handles);
  
  for (auto & handle : handles) {
    participant->get_discovered_participant_data(participant_data, handle);
    
    // 从参与者属性中提取 ROS 节点名称
    // ROS2 将节点名称编码在 UserData QoS 中
    // 格式：enclave=<ns>;nodename=<name>;
    std::string user_data(
      participant_data.user_data.begin(),
      participant_data.user_data.end());
    
    // 解析 ROS2 节点信息
    parse_ros2_node_names(user_data, node_names, node_namespaces);
  }
  return RMW_RET_OK;
}
```

---

## 十、ROS2 节点信息编码（UserData QoS）

ROS2 在 DDS 的 `UserData QoS` 字段中编码节点元数据：

```
# UserData 编码格式（ASCII 键值对）
enclave=/;
```

在 rmw_fastrtps 中的实现：

```cpp
// rmw_fastrtps_shared_cpp/src/participant.cpp

// 创建参与者时设置 UserData
ParticipantAttributes participant_attr;

// 编码 ROS 节点信息
std::string user_data = "enclave=" + std::string(node->enclave) + ";";
participant_attr.rtps.userData = std::vector<octet>(
  user_data.begin(), user_data.end());

// 在 TopicQoS 的 TopicData 中编码 ROS2 主题信息
// Topic 名称格式：rt/<namespace>/<name>  (ros topic)
//                 rq/<namespace>/<name>Request_  (service request)
//                 rr/<namespace>/<name>Reply_     (service response)
//                 rp/<namespace>/<name>  (ros parameter)
//                 ra/<namespace>/<name>  (ros action)
```

**ROS2 Topic 名称前缀（关键）**：

| 前缀 | 含义 | 示例 |
|------|------|------|
| `rt/` | ROS Topic | `rt/chatter` |
| `rq/` | Service Request | `rq/add_two_ints/AddTwoIntsRequest_` |
| `rr/` | Service Response | `rr/add_two_ints/AddTwoIntsReply_` |
| `rp/` | Parameter | `rp/talker/ParameterRequest_` |
| `ra/` | Action | `ra/navigate/GoalRequest_` |
| `rs/` | Action Status | `rs/navigate/_action/status` |

---

## 十一、ROS2 图（Graph）事件系统

ROS2 提供了 Graph 事件机制，允许节点监听服务发现变化：

```cpp
// rclcpp 中监听图事件的示例

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // 获取图事件对象
    graph_event_ = this->get_graph_event();
  }

  // 在另一个线程中等待图变化
  void watch_graph()
  {
    while (rclcpp::ok()) {
      // 阻塞等待图变化（节点加入/离开，topic 出现/消失）
      this->wait_for_graph_change(graph_event_, std::chrono::seconds(1));
      graph_event_->check_and_clear();
      
      // 查询当前图状态
      auto node_names = this->get_node_names();
      RCLCPP_INFO(get_logger(), "Graph changed! Nodes: %zu", node_names.size());
      
      // 查询 topic 信息
      auto topics = this->get_topic_names_and_types();
      for (auto & [name, types] : topics) {
        RCLCPP_INFO(get_logger(), "  Topic: %s", name.c_str());
      }
    }
  }

private:
  rclcpp::Event::SharedPtr graph_event_;
};
```

### rcl 层图事件实现

```c
// rcl/src/rcl/graph.c

// 等待图变化（内部通过 guard condition 实现）
rcl_ret_t
rcl_wait_for_graph_change(
  const rcl_node_t * node,
  rcl_event_t * event,
  const rcl_duration_t * timeout)
{
  // rcl_graph_guard_condition 是 rmw 层图变化事件的包装
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_wait_set_init(&wait_set, 0, 1, 0, 0, 0, 0, ...);
  rcl_wait_set_add_guard_condition(&wait_set, event->impl->guard_condition);
  rcl_wait(&wait_set, timeout->nanoseconds);
}
```

---

## 十二、参考链接

- **DDSI-RTPS 规范**：https://www.omg.org/spec/DDSI-RTPS/
- **DDS 规范**：https://www.omg.org/spec/DDS/
- **FastDDS 源码**：https://github.com/eProsima/Fast-DDS
- **CycloneDDS 源码**：https://github.com/eclipse-cyclonedds/cyclonedds
- **rmw 接口**：https://github.com/ros2/rmw
- **rmw_fastrtps**：https://github.com/ros2/rmw_fastrtps
- **rcl 源码**：https://github.com/ros2/rcl
