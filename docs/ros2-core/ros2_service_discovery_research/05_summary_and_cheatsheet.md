# ROS2 服务发现 知识总结与速查手册

## 一、核心概念速查

| 概念 | 描述 | 配置入口 |
|------|------|---------|
| DDS Domain | 逻辑隔离单元 | `ROS_DOMAIN_ID` |
| DomainParticipant | DDS 参与者（每进程一个） | `rcl_context` |
| SPDP | 参与者发现协议（多播UDP） | FastDDS XML / CycloneDDS XML |
| SEDP | 端点发现协议（单播） | 自动，QoS 控制匹配 |
| GraphCache | ROS2 图缓存（节点→端点映射） | `rmw_dds_common` |
| ros_discovery_info | ROS2 双层发现的 DDS Topic | 自动管理 |
| Discovery Server | FastDDS 集中式发现扩展 | `ROS_DISCOVERY_SERVER` |
| SROS2 | DDS Security 封装 | `ROS_SECURITY_*` 环境变量 |

---

## 二、发现机制完整流程图

```
  节点A 启动                              节点B（已运行）
      │                                       │
      ▼                                       │
  rcl_init()                                  │
  rmw_create_context()                        │
      │                                       │
      ▼                                       │
  DomainParticipant 创建                      │
  GuidPrefix 生成（IP+PID 派生）              │
      │                                       │
      ▼                                       │
  SPDP Writer/Reader 初始化                   │
      │                                       │
      ├──[UDP 多播 7400]──────────────────────►│ SPDP Reader 收到
      │  SPDPdiscoveredParticipantData         │
      │  包含：GUID、地址:端口、租约时间        │
      │                                       ▼
      │                                 assignRemoteEndpoints()
      │                                 启动 SEDP 流程
      │                                       │
      │◄──[UDP 单播 7412]─────────────────────┤ 发送 Publications
      │  DiscoveredWriterData                 │ （节点B的所有 DataWriter）
      │  包含：GUID、TopicName、类型、QoS      │
      │                                       │
      ├──[UDP 单播]──────────────────────────►│ 发送 Subscriptions
      │  DiscoveredReaderData                 │ （节点A的所有 DataReader）
      │                                       │
      ▼                                       ▼
  QoS 兼容性检查                         QoS 兼容性检查
      │                                       │
      ▼                                       ▼
  建立逻辑连接                           建立逻辑连接
  （添加对端 Locator 到发送列表）         （添加对端 Locator 到发送列表）
      │                                       │
      ▼                                       ▼
  发布 ros_discovery_info 更新           接收 ros_discovery_info 更新
  （节点A的节点→端点映射）               （更新本地 GraphCache）
      │                                       │
      ▼                                       ▼
  节点B 可通过 ros2 node list 看到节点A   节点A 可通过 ros2 node list 看到节点B
```

---

## 三、发现时序总结

```
典型 LAN 环境（100Mbps，同网段）：

0ms    ── 节点启动，SPDP 首次广播
1-5ms  ── 对端收到 SPDP，开始 SEDP 握手
5-20ms ── SEDP 端点匹配完成
20-50ms── ros_discovery_info 更新传播
50ms   ── ros2 node list 可以看到新节点

影响延迟的关键参数（FastDDS）：
  leaseDuration: 默认20s（超时感知离线的时间）
  leaseDuration_announcementperiod: 默认 leaseDuration/3 ≈ 6.7s（重宣告周期）
  initial_announcements: 默认5次，间隔100ms（首次快速发现）
```

---

## 四、常用排查命令

```bash
# ============ 基础发现诊断 ============
# 查看当前域内所有节点
ros2 node list

# 查看所有 topic（含类型）
ros2 topic list -t

# 查看 topic 的 QoS 详情（用于排查不匹配）
ros2 topic info /my_topic --verbose

# 查看 service 列表
ros2 service list -t

# ============ 网络层诊断 ============
# 抓 SPDP 广播
sudo tcpdump -i any -n 'udp port 7400' -X | head -100

# 查看 DDS 端口使用
ss -ulnp | grep 74

# 测试多播（同网段两台机器执行）
# 机器1（接收）：
python3 -c "
import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 7400))
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, 
  socket.inet_aton('239.255.0.1') + socket.inet_aton('0.0.0.0'))
print(s.recv(1024))
"

# ============ FastDDS 专项 ============
# 列出 FastDDS 统计数据
ros2 doctor --report | grep -i fastdds

# 验证 Discovery Server 连接
fastdds discovery status -s 127.0.0.1:11811

# ============ 诊断 QoS 不匹配 ============
# 打开 FastDDS 调试日志（非常详细）
export FASTDDS_LOG_VERBOSITY=Info
export FASTDDS_LOG_NO_STDOUT=0
ros2 run my_pkg my_node 2>&1 | grep -i "incompatible\|unmatched\|qos"
```

---

## 五、发现机制问题速查表

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| `ros2 node list` 为空 | SPDP 多播被阻断 | 检查防火墙/路由器多播设置 |
| 只能看到本机节点 | 多播不通或 ROS_DOMAIN_ID 不同 | 用 tcpdump 确认多播可达 |
| Topic 有 Publisher/Subscriber 但不通信 | QoS 不兼容 | 用 `ros2 topic info --verbose` 检查 |
| 服务发现慢（> 1s） | SPDP 宣告周期长 | 调小 `initial_announcements` 间隔 |
| 节点崩溃后仍显示在 `ros2 node list` | 租约未过期（默认20s） | 减小 `leaseDuration` 或等待过期 |
| Docker 容器内节点不可见 | 容器网络隔离 | 使用 `--network host` 或单播发现 |
| K8s Pod 间不可见 | 多播跨 Pod 不通 | 使用 FastDDS Discovery Server 或 Zenoh |

---

## 六、关键源码文件速查索引

```
服务发现相关关键文件（按调用层次）

rclcpp 层（用户 API）：
  rclcpp/src/rclcpp/node.cpp               → get_node_names()
  rclcpp/src/rclcpp/client.cpp             → wait_for_service()

rcl 层（C API）：
  rcl/src/rcl/graph.c                      ★★★★ → rcl_get_node_names()
  rcl/src/rcl/node.c                       → rcl_node_init()

rmw 接口（抽象层）：
  rmw/include/rmw/rmw.h                    → API 声明

rmw_dds_common（ROS2 图缓存）：
  rmw_dds_common/src/graph_cache.cpp       ★★★★★ → GraphCache 核心
  rmw_dds_common/src/context.cpp           ★★★ → ros_discovery_info 管理

rmw_fastrtps（FastDDS 绑定）：
  rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp  ★★★★
  rmw_fastrtps_shared_cpp/src/participant.cpp              ★★★

FastDDS 内部（DDS 层）：
  Fast-DDS/src/cpp/rtps/builtin/discovery/protocol/PDPSimple.cpp  ★★★★★
  Fast-DDS/src/cpp/rtps/builtin/discovery/protocol/EDPSimple.cpp  ★★★★★
  Fast-DDS/src/cpp/rtps/participant/RTPSParticipantImpl.cpp        ★★★

CycloneDDS 内部（DDS 层）：
  cyclonedds/src/core/ddsi/src/ddsi_discovery.c   ★★★★★
  cyclonedds/src/core/ddsi/src/ddsi_sedp.c        ★★★★
  cyclonedds/src/core/ddsi/src/ddsi_receive.c     ★★★
```

---

## 七、参考链接汇总

### 标准规范
- DDSI-RTPS 2.5：https://www.omg.org/spec/DDSI-RTPS/2.5/PDF
- DDS 1.4：https://www.omg.org/spec/DDS/1.4/PDF

### ROS2 官方文档
- 概念：https://docs.ros.org/en/rolling/Concepts/About-Different-Middleware-Vendors.html
- DDS 调优：https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html
- Discovery Server：https://docs.ros.org/en/rolling/Tutorials/Advanced/Discovery-Server/Discovery-Server.html
- SROS2：https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html

### 源码仓库
- rmw_dds_common：https://github.com/ros2/rmw_dds_common
- rmw_fastrtps：https://github.com/ros2/rmw_fastrtps
- rmw_cyclonedds：https://github.com/ros2/rmw_cyclonedds
- FastDDS：https://github.com/eProsima/Fast-DDS
- CycloneDDS：https://github.com/eclipse-cyclonedds/cyclonedds
