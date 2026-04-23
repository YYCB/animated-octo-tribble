# ROS2 服务发现（三）：FastDDS 与 CycloneDDS 发现机制对比

## 一、两种 DDS 实现的定位

| 维度 | FastDDS (eProsima) | CycloneDDS (Eclipse) |
|------|-------------------|---------------------|
| ROS2 默认版本 | Galactic ~ Humble 默认 | Jazzy 起作为新默认（计划中） |
| 维护者 | eProsima（西班牙） | Eclipse Foundation + ADLINK |
| 发现协议 | SPDP/SEDP（标准）+ Discovery Server（扩展） | SPDP/SEDP（标准） + Proxy（实验性）|
| 零拷贝 | iceoryx（共享内存） | iceoryx（共享内存） |
| 大规模支持 | Discovery Server 模式 | 配置 Peers 列表 |
| 性能特点 | 功能全，可配置性强 | 延迟低，代码简洁 |
| 线程模型 | 可配置线程池 | 单 receive 线程（默认） |

---

## 二、FastDDS 发现机制深度解析

### 2.1 参与者生命周期（源码级）

```cpp
// Fast-DDS/src/cpp/rtps/participant/RTPSParticipantImpl.cpp

RTPSParticipantImpl::RTPSParticipantImpl(
  const RTPSParticipantAttributes & param,
  ...)
{
  // Step 1: 初始化网络工厂（绑定 UDP/TCP 接口）
  mp_network_factory = new NetworkFactory(param);
  
  // Step 2: 计算 GUID
  // guidPrefix.hostId = IP 地址的 hash
  // guidPrefix.processId = getpid() 的 hash
  // guidPrefix.participantId = 递增计数器
  generate_guid();
  
  // Step 3: 初始化内建协议（SPDP + SEDP + WLP）
  mp_builtinProtocols = new BuiltinProtocols();
  mp_builtinProtocols->initBuiltinProtocols(this, param.builtin);
  //   → PDPSimple::initPDP()    # SPDP 初始化
  //   → EDPSimple::initEDP()    # SEDP 初始化（在 PDPSimple 内部调用）
  //   → WLP::initWL()           # Liveliness 协议初始化
  
  // Step 4: 首次宣告自身（发送 SPDPdiscoveredParticipantData）
  mp_builtinProtocols->announceRTPSParticipantState();
}
```

### 2.2 FastDDS 线程架构

```
FastDDS 主要线程：
                                         
  SPDP 宣告线程（TimedEvent）
  └─ 每 leaseDuration/3 发送一次参与者宣告
  
  接收线程（每个 listening socket 一个）
  └─ UDPv4Transport::Receive() 循环
     └─ 处理收到的 RTPS 消息（SPDP/SEDP/DATA/HEARTBEAT）
  
  事件线程（asio::io_service）
  └─ 处理所有 TimedEvent（超时、重传等）
  
  用户线程（应用调用 DataWriter::write()）
  └─ 直接调用，无独立线程
```

### 2.3 FastDDS 租约机制（Liveliness）

```cpp
// 租约过期检测（对方进程崩溃时的感知）
// Fast-DDS/src/cpp/rtps/builtin/discovery/participant/PDPSimple.cpp

// 当收到参与者宣告时，重置对应的租约过期定时器
void PDPSimple::resetParticipantLeaseDuration(
  const GUID_t & remote_guid)
{
  auto proxy = getProxyParticipant(remote_guid);
  if (proxy) {
    // 重置定时器：leaseDuration 秒后如果未收到新宣告，
    // 认为远端参与者已离线
    proxy->lease_duration_event->restart_timer();
  }
}

// 租约过期回调
void PDPListener::onParticipantLeaseDurationExpired(
  const GUID_t & expired_guid)
{
  // 移除过期参与者的所有端点信息
  mp_PDP->removeRemoteParticipant(expired_guid, ParticipantDiscoveryInfo::DROPPED);
  
  // 通知上层（rmw 层）
  // → 触发 graph guard condition
  // → rclcpp 图事件回调触发
}
```

---

## 三、CycloneDDS 发现机制深度解析

### 3.1 CycloneDDS 核心数据结构

```c
// eclipse-cyclonedds/cyclonedds/src/core/ddsi/include/dds/ddsi/ddsi_entity.h

// CycloneDDS 实体（所有 DDS 对象的基类）
struct ddsi_entity {
  ddsi_entityid_t eid;          // 实体 ID（本地）
  struct ddsi_guid guid;        // 全局唯一 ID（GUID）
  enum ddsi_entity_kind kind;   // 类型枚举
  int32_t refc;                 // 引用计数（原子操作）
  // ...
};

// 参与者代理（远端参与者的本地表示）
struct ddsi_proxy_participant {
  struct ddsi_entity e;
  struct ddsi_addrset *as;        // 地址集合（单播+多播地址列表）
  ddsi_vendorid_t vendor;         // DDS 厂商 ID
  ddsi_duration_t lease_duration; // 租约时间
  struct ddsi_lease *lease;       // 租约对象（包含过期定时器）
  struct ddsi_plist *plist;       // 参与者属性（SPDP 数据）
  // ...
};
```

### 3.2 CycloneDDS SPDP 发送实现

```c
// eclipse-cyclonedds/cyclonedds/src/core/ddsi/src/ddsi_discovery.c

// 向所有已知参与者（或多播地址）发送 SPDP 宣告
static void write_pmd_message(struct ddsi_domaingv *gv, struct ddsi_participant *pp, bool alive)
{
  // 构建参与者代理数据（序列化）
  struct ddsi_plist ps;
  ddsi_plist_init_empty(&ps);
  
  // 填充协议版本、厂商 ID、GUID
  ps.present |= PP_PROTOCOL_VERSION | PP_VENDORID | PP_PARTICIPANT_GUID;
  ps.protocol_version = RTPS_PROTOCOL_VERSION;
  ps.vendorid = NN_VENDORID_ECLIPSE;
  ps.participant_guid = pp->e.guid;
  
  // 填充单播/多播地址
  ps.present |= PP_DEFAULT_UNICAST_LOCATOR;
  // ...（从 addrset 填充）
  
  // 填充租约时间
  ps.present |= PP_PARTICIPANT_LEASE_DURATION;
  ps.participant_lease_duration = pp->lease_duration;
  
  // 填充 UserData（ROS2 enclave 信息）
  ps.present |= PP_USER_DATA;
  // ...
  
  // 序列化并写入 SPDP builtin writer
  struct ddsi_serdata *sd = ddsi_serdata_from_sample(
    gv->spdp_type, SDK_DATA, &ps);
  
  ddsi_write_sample_nogc(gv, NULL, pp->spdp_pp_writer, sd);
  ddsi_serdata_unref(sd);
  ddsi_plist_fini(&ps);
}
```

### 3.3 CycloneDDS 接收处理线程

```c
// eclipse-cyclonedds/cyclonedds/src/core/ddsi/src/ddsi_receive.c

// 主接收循环
static void recv_thread(void *varg)
{
  struct ddsi_tran_conn *conn = varg;
  
  while (!gv->terminate) {
    // 等待数据到达（select/epoll）
    ddsi_tran_locator_t srcloc;
    ssize_t sz = ddsi_tran_read(conn, rbuf->buf, RBUF_SIZE, &srcloc);
    
    if (sz > 0) {
      // 解析 RTPS 头
      handle_rtps_message(gv, conn, &srcloc, sz, rbuf);
    }
  }
}

// 分发 RTPS 子消息
static void handle_rtps_message(...)
{
  // 解析每个 SubMessage
  while (submsg_available) {
    switch (submsg_id) {
      case SMID_DATA:
        handle_Data(gv, ...);    // 用户数据或内建 Topic 数据
        break;
      case SMID_HEARTBEAT:
        handle_Heartbeat(gv, ...); // 可靠传输心跳
        break;
      case SMID_ACKNACK:
        handle_AckNack(gv, ...);   // 确认/重传请求
        break;
      // ...
    }
  }
}
```

---

## 四、rmw_cyclonedds 实现

### 4.1 关键源码路径

```
ros2/rmw_cyclonedds/
└── rmw_cyclonedds_cpp/
    └── src/
        ├── rmw_node.cpp                    # 节点创建/销毁
        ├── rmw_publisher.cpp               # 发布者
        ├── rmw_subscription.cpp            # 订阅者
        ├── rmw_service.cpp                 # 服务
        ├── rmw_client.cpp                  # 客户端
        └── rmw_wait.cpp                    # 等待集合
```

### 4.2 CycloneDDS 中的 ROS2 图发现

```cpp
// rmw_cyclonedds_cpp/src/rmw_node.cpp

// CycloneDDS 使用与 FastDDS 相同的双层发现机制（rmw_dds_common）
// 区别在于底层 DDS API 调用不同

rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_)
{
  // 获取已创建的 DDS participant（与 FastDDS 类似，每进程共享）
  auto & dds_entity = static_cast<CddsContext *>(context->impl)->participant;
  
  // 更新 ROS2 图缓存（与 FastDDS 相同的 rmw_dds_common 路径）
  auto common = static_cast<rmw_dds_common::Context *>(context->impl->common_);
  common->graph_cache.add_node(common->gid, name, namespace_);
  common->publish_graph_cache_update();
  
  return node;
}
```

---

## 五、DDS QoS 对发现的影响

### 5.1 Topic 匹配的 QoS 检查流程

```
DataWriter 宣告（SEDP Publications）
    │
    ↓ 其他参与者的 SEDP Reader 收到
    │
    ↓ 与本地所有 DataReader 对比：
    │   1. Topic 名称匹配？
    │   2. 消息类型名匹配？
    │   3. Reliability QoS 兼容？
    │   4. Durability QoS 兼容？
    │   5. Deadline QoS 兼容？
    │   6. Liveliness QoS 兼容？
    │
    ├─[全部匹配]→ 建立逻辑连接（添加对端 Locator 到发送列表）
    └─[任意不匹配]→ 丢弃（静默，不报错！）
```

### 5.2 常见 QoS 配置的 ROS2 对应

```cpp
// rclcpp/include/rclcpp/qos.hpp

// 传感器数据 QoS（常用于高频传感器）
rclcpp::SensorDataQoS sensorDataQos;
// → Reliability: BEST_EFFORT
// → Durability: VOLATILE
// → History: KEEP_LAST(5)

// 系统默认 QoS（ROS2 默认）
rclcpp::SystemDefaultsQoS defaultQos;
// → Reliability: RELIABLE
// → Durability: VOLATILE
// → History: KEEP_LAST(10)

// 服务 QoS
rclcpp::ServicesQoS servicesQos;
// → Reliability: RELIABLE
// → Durability: VOLATILE
// → History: KEEP_LAST(10)

// 参数 QoS
rclcpp::ParametersQoS parametersQos;
// → Reliability: RELIABLE
// → Durability: VOLATILE
// → History: KEEP_LAST(1000)

// TRANSIENT_LOCAL（持久化，晚加入的订阅者也能收到历史消息）
rclcpp::QoS qos(1);
qos.transient_local();
// → Durability: TRANSIENT_LOCAL
// 典型用途：/map, /tf_static, /robot_description
```

---

## 六、内建 Topic 完整列表

DDS 标准定义的内建 Topic（每个参与者自动创建）：

| Topic 名称 | 描述 | ROS2 使用 |
|-----------|------|----------|
| `DCPSParticipant` | 参与者发现（SPDP） | 节点发现基础 |
| `DCPSPublication` | DataWriter 发现（SEDP） | Publisher 发现 |
| `DCPSSubscription` | DataReader 发现（SEDP） | Subscription 发现 |
| `DCPSTopic` | Topic 元数据 | 类型信息 |
| `ros_discovery_info` | ROS2 扩展：节点→端点映射 | 节点名称发现 |

---

## 七、使用 Wireshark 分析 DDS 发现报文

```bash
# 安装 Wireshark DDS/RTPS 插件（通常已内置）

# 过滤 SPDP 多播（Domain 0）
rtps && udp.dstport == 7400

# 查看 SPDP 宣告内容：
# Participant GUID: xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:xx:00:00:01:C1
# Default Unicast Locator: 192.168.1.x:7412
# Lease Duration: 20s

# 过滤 SEDP（端点发现，单播）
rtps && udp.dstport == 7412

# 过滤用户数据
rtps && !(udp.dstport == 7400 || udp.dstport == 7412)
```

### RTPS 报文解析示例

```
RTPS Header:
  Magic: 52 54 50 53 (RTPS)
  Version: 02 04 (RTPS 2.4)
  VendorId: 01 01 (eProsima FastDDS)
  GuidPrefix: 01 0F 92 A1 B2 C3 D4 E5 F6 07 08 09

SubMessage: DATA (0x15)
  ExtraFlags: 0x00
  OctetsToInlineQos: 16
  EntityId (reader): 00 00 01 C7  (SPDP Reader)
  EntityId (writer): 00 00 01 C2  (SPDP Writer)
  SeqNum: 1
  
  ParameterList (SPDPdiscoveredParticipantData):
    PID_PROTOCOL_VERSION: 02 04
    PID_VENDOR_ID: 01 01
    PID_DEFAULT_UNICAST_LOCATOR: [192.168.1.10]:7412
    PID_PARTICIPANT_LEASE_DURATION: 20s
    PID_USER_DATA: "enclave=/;"
```

---

## 八、参考资料

### 规范文档
- **OMG DDSI-RTPS 规范 v2.5**：https://www.omg.org/spec/DDSI-RTPS/2.5/PDF
- **OMG DDS 规范 v1.4**：https://www.omg.org/spec/DDS/1.4/PDF
- **OMG DDS-Security 规范**：https://www.omg.org/spec/DDS-SECURITY/

### 源码仓库
- **FastDDS**：https://github.com/eProsima/Fast-DDS
- **CycloneDDS**：https://github.com/eclipse-cyclonedds/cyclonedds
- **rmw_fastrtps**：https://github.com/ros2/rmw_fastrtps
- **rmw_cyclonedds**：https://github.com/ros2/rmw_cyclonedds
- **rmw_dds_common**：https://github.com/ros2/rmw_dds_common
- **rmw 接口**：https://github.com/ros2/rmw

### FastDDS 文档
- https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html
- https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/disc_server/disc_server.html
