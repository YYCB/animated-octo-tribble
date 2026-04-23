# ROS2 服务发现（四）：高级特性与工程实践

## 一、Action 的服务发现机制

ROS2 Action 是 Service + Topic 的组合，通过以下 5 个 DDS Topic 实现：

```
ROS2 Action /navigate_to_pose
  │
  ├─ Goal Service（Service 模式）：
  │   rq/navigate_to_pose/_action/send_goalRequest_
  │   rr/navigate_to_pose/_action/send_goalReply_
  │
  ├─ Cancel Service（Service 模式）：
  │   rq/navigate_to_pose/_action/cancel_goalRequest_
  │   rr/navigate_to_pose/_action/cancel_goalReply_
  │
  ├─ Result Service（Service 模式）：
  │   rq/navigate_to_pose/_action/get_resultRequest_
  │   rr/navigate_to_pose/_action/get_resultReply_
  │
  ├─ Feedback Topic：
  │   rt/navigate_to_pose/_action/feedback
  │
  └─ Status Topic：
      rt/navigate_to_pose/_action/status
```

发现机制：同 Topic 和 Service，全部走 DDS 标准 SEDP。

---

## 二、ROS2 参数系统的服务发现

ROS2 参数通过 Service 实现，因此也是 DDS 发现的一部分：

```
节点 /my_node 的参数服务（自动创建）：
  rq/my_node/get_parametersRequest_
  rr/my_node/get_parametersReply_
  rq/my_node/set_parametersRequest_
  rr/my_node/set_parametersReply_
  rq/my_node/list_parametersRequest_
  rr/my_node/list_parametersReply_
  rq/my_node/describe_parametersRequest_
  rr/my_node/describe_parametersReply_
  rt/my_node/parameter_events   (Topic，参数变化事件)
  rt/rosout                     (Topic，日志输出)
```

---

## 三、SROS2 安全发现

ROS2 支持通过 DDS Security 扩展实现安全发现（加密、认证、访问控制）。

### 3.1 DDS Security 与发现的集成

```
安全发现流程（SROS2）：

T+0: 新节点启动，携带安全证书（证书由 keystore 提供）
      │
T+1: SPDP 携带 ParticipantSecurityInfo：
      │   - IdentityToken（身份证书的 hash）
      │   - PermissionsToken（权限证书的 hash）
      │
T+2: 已存在节点验证新节点身份：
      │   - 握手协议（AuthenticationPlugin）
      │   - 验证证书链（PKI）
      │   - 交换 DH 密钥（生成会话密钥）
      │
T+3: 权限检查（AccessControlPlugin）：
      │   - 验证新节点是否有权限 publish/subscribe 特定 Topic
      │
T+4: 建立安全通道（CryptoPlugin）：
      └   - 后续数据用协商的会话密钥加密
```

### 3.2 SROS2 快速配置

```bash
# 创建 keystore
ros2 security create_keystore ~/sros2_demo/keystore

# 为节点创建密钥
ros2 security create_enclave ~/sros2_demo/keystore /talker
ros2 security create_enclave ~/sros2_demo/keystore /listener

# 运行安全节点
export ROS_SECURITY_KEYSTORE=~/sros2_demo/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce  # 拒绝未认证节点

ros2 run demo_nodes_cpp talker --ros-args --enclave /talker
```

---

## 四、ros2cli 中的发现命令实现

`ros2 node list`、`ros2 topic list` 等命令直接调用 rcl API：

### 4.1 ros2 node list 实现

```python
# ros2/ros2cli/ros2node/ros2node/verb/list.py（简化）

class ListVerb(VerbExtension):
    def main(self, *, args):
        with NodeStrategy(args) as node:
            # 调用 rclpy（Python 绑定）
            node_names = node.get_node_names_and_namespaces()
            for name, namespace in node_names:
                print(f'{namespace}/{name}')
```

```python
# rclpy/rclpy/node.py

def get_node_names_and_namespaces(self):
    # 调用 C 扩展 → rcl_get_node_names()
    return _rclpy.rclpy_get_node_names_and_namespaces(self._handle)
```

### 4.2 ros2 topic echo 的发现逻辑

```python
# ros2/ros2cli/ros2topic/ros2topic/verb/echo.py

# 首先查找 topic 的消息类型（通过服务发现）
topic_name = args.topic_name
msg_type_str = get_msg_class(node, topic_name, blocking=True, include_hidden_topics=True)

# get_msg_class 内部轮询 topic_names_and_types，等待 topic 出现
# 超时默认 5 秒
```

---

## 五、Zenoh 作为替代发现机制（前沿方向）

Zenoh 是一种新兴的、比 DDS 更轻量的发布/订阅协议，已有 ROS2 桥接：

```
Zenoh 的发现机制特点：
- 基于路由器（Router）的发现（非多播广播）
- 支持 WiFi、LTE、星型网络（DDS 多播难以穿越的场景）
- 延迟更低（纯 UDP/QUIC，无 RTPS 开销）
- 资源受限设备友好（嵌入式）

ROS2 集成：
  rmw_zenoh_cpp（实验性 rmw 实现）
  ros2/rmw_zenoh 仓库

Zenoh 发现流程：
  节点 → Zenoh Router（本地或远端）→ 其他节点
  （类似于发布/订阅的路由协议）
```

---

## 六、大规模系统中的发现优化

### 6.1 节点数量对发现性能的影响

```
DDS 发现的 O(n²) 问题：
  n 个参与者 = n*(n-1)/2 个 SEDP 握手连接
  
  n=10   → 45 个连接
  n=100  → 4950 个连接
  n=1000 → 499500 个连接（系统崩溃/卡顿边界）

推荐策略：
  - n < 50：默认 SPDP 多播，无需特殊配置
  - 50 < n < 200：调整 SPDP 宣告周期，减少广播频率
  - n > 200：使用 FastDDS Discovery Server 或 Zenoh Router
```

### 6.2 FastDDS Discovery Server 深度配置

```xml
<!-- discovery_server.xml（服务器端配置）-->
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="discovery_server" is_default_profile="true">
      <rtps>
        <prefix>44.53.00.f1.1e.a5.d9.81.aa.26.77.00</prefix>
        <builtin>
          <discovery_config>
            <!-- SERVER 模式：作为发现服务器 -->
            <discoveryProtocol>SERVER</discoveryProtocol>
          </discovery_config>
          <metatrafficUnicastLocatorList>
            <locator>
              <udpv4>
                <port>11811</port>
              </udpv4>
            </locator>
          </metatrafficUnicastLocatorList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
```

```xml
<!-- client.xml（客户端节点配置）-->
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default_xrce_participant" is_default_profile="true">
      <rtps>
        <builtin>
          <discovery_config>
            <!-- CLIENT 模式：连接到 Discovery Server -->
            <discoveryProtocol>CLIENT</discoveryProtocol>
            <discoveryServersList>
              <RemoteServer prefix="44.53.00.f1.1e.a5.d9.81.aa.26.77.00">
                <metatrafficUnicastLocatorList>
                  <locator>
                    <udpv4>
                      <address>192.168.1.1</address>
                      <port>11811</port>
                    </udpv4>
                  </locator>
                </metatrafficUnicastLocatorList>
              </RemoteServer>
            </discoveryServersList>
          </discovery_config>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
```

### 6.3 跨机器 ROS2 容器（Docker）发现配置

```bash
# 方法1：host network（最简单，仅限单机多容器）
docker run --network host my_ros2_node

# 方法2：指定 RMW IP 白名单（FastDDS）
export RMW_FASTRTPS_USE_QOS_FROM_ENV=1

# 创建 FastDDS 配置（限制接口）
cat > /tmp/fastdds_config.xml << EOF
<?xml version="1.0"?>
<dds>
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>172.17.0.0/16</address>  <!-- Docker 网桥 -->
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
  </profiles>
</dds>
EOF
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_config.xml

# 方法3：使用 Zenoh + 跨网络路由（推荐 K8s 环境）
```

---

## 七、发现机制的时序问题与解决方案

### 7.1 节点间发现竞争条件

```cpp
// 常见问题：Publisher 在 Subscriber 前启动，
// 第一条消息丢失（VOLATILE Durability）

// 错误做法：
auto pub = node->create_publisher<Msg>("topic", 10);
pub->publish(msg);  // 此时 Subscriber 可能还未发现 Publisher

// 正确做法1：使用 TRANSIENT_LOCAL（持久化最后一条）
rclcpp::QoS qos(1);
qos.transient_local();
auto pub = node->create_publisher<Msg>("topic", qos);

// 正确做法2：等待 Subscriber 出现
auto pub = node->create_publisher<Msg>("topic", 10);
while (pub->get_subscription_count() == 0) {
  rclcpp::sleep_for(std::chrono::milliseconds(100));
}
pub->publish(msg);

// 正确做法3：使用图事件等待
rclcpp::EventHandlerBase::SharedPtr pub_event_handler;
// 注册 PublisherMatchedEventCallback 回调
```

### 7.2 等待服务可用（waitForService）

```cpp
// rclcpp 中等待服务的正确方式
auto client = node->create_client<std_srvs::srv::Trigger>("/my_service");

// 阻塞等待服务端出现（内部轮询 rmw_service_server_is_available）
if (!client->wait_for_service(std::chrono::seconds(5))) {
  RCLCPP_ERROR(node->get_logger(), "Service not available after 5 seconds");
  return;
}

// 发送请求
auto future = client->async_send_request(request);
```

`wait_for_service` 内部实现：

```cpp
// rclcpp/src/rclcpp/client.cpp

bool ClientBase::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  
  // 等待图变化事件，每次变化后重新检查
  std::shared_ptr<rclcpp::Event> event = node_graph_->get_graph_event();
  
  while (rclcpp::ok(context_)) {
    // 检查服务是否可用（调用 rmw_service_server_is_available）
    if (service_is_ready()) return true;
    
    // 超时检查
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (timeout >= std::chrono::nanoseconds::zero() && elapsed >= timeout) {
      return false;
    }
    
    // 等待图变化（有新节点/端点时唤醒）
    node_graph_->wait_for_graph_change(event, std::chrono::milliseconds(10));
    event->check_and_clear();
  }
  return false;
}
```

---

## 八、服务发现与 QoS 调试工具

### 8.1 rqt_graph 原理

```python
# rqt_graph 使用 rclpy 的图查询接口实时刷新：
node.get_node_names_and_namespaces()
node.get_topic_names_and_types()
node.get_publishers_info_by_topic(topic_name)
node.get_subscriptions_info_by_topic(topic_name)
```

### 8.2 ros2 doctor 发现诊断

```bash
# 诊断 ROS2 环境和发现配置
ros2 doctor

# 输出包含：
# - DDS 实现版本
# - ROS_DOMAIN_ID 设置
# - 网络接口信息
# - 发现的节点数量
```

### 8.3 fastdds_statistics（FastDDS 专有）

```bash
# 启用 FastDDS 统计模块（收集发现延迟等数据）
export FASTDDS_STATISTICS=ALL

ros2 run fastdds_statistics_backend ros2_discovery_monitor
```

---

## 九、发现机制的安全注意事项

| 风险 | 描述 | 缓解措施 |
|------|------|---------|
| 未授权节点注入 | 任何同网段节点都能发现并订阅 | 使用 SROS2 证书认证 |
| 消息伪造 | 攻击者伪装成已知节点发布恶意消息 | DDS Security 数据签名 |
| DDS 发现广播洪泛 | 大量 SPDP 广播消耗带宽 | Discovery Server + 速率限制 |
| 跨 Domain 隔离失效 | Domain ID 被暴力扫描 | 防火墙限制 DDS 端口范围 |
| 元数据泄露 | 节点名、Topic 名暴露系统架构 | 生产环境使用 SROS2 |

---

## 十、发现机制知识总结速查

```
关键环境变量：
  ROS_DOMAIN_ID          # 0-101，控制 DDS 域隔离
  RMW_IMPLEMENTATION     # 选择 DDS 实现（rmw_fastrtps_cpp/rmw_cyclonedds_cpp）
  FASTRTPS_DEFAULT_PROFILES_FILE  # FastDDS XML 配置文件路径
  CYCLONEDDS_URI         # CycloneDDS 配置文件路径
  ROS_DISCOVERY_SERVER   # FastDDS Discovery Server 地址
  ROS_SECURITY_KEYSTORE  # SROS2 密钥库路径
  ROS_SECURITY_ENABLE    # 启用 SROS2（true/false）
  
DDS 端口（Domain 0）：
  7400 - SPDP 多播
  7401 - 用户多播
  7412+2n - SPDP 单播（参与者 n）
  7413+2n - 用户单播（参与者 n）
  
发现诊断命令：
  ros2 node list           # 已发现节点
  ros2 topic list -t       # 已发现 topic（含类型）
  ros2 service list -t     # 已发现服务
  ros2 topic info <topic> --verbose  # QoS 信息
  ros2 doctor              # 环境诊断
  sudo tcpdump -i any 'udp port 7400'  # 抓取 SPDP 广播
```
