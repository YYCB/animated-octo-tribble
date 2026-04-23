# Topic 通信（一）：全链路架构与数据流

## 一、完整通信架构图

```
发布端（Publisher Process）                   订阅端（Subscriber Process）
┌─────────────────────────────┐              ┌──────────────────────────────┐
│  User Code                  │              │  User Callback               │
│  pub->publish(msg)          │              │  callback(const Msg & msg)   │
├─────────────────────────────┤              ├──────────────────────────────┤
│  rclcpp Layer               │              │  rclcpp Layer                │
│  Publisher<T>::publish()    │              │  execute_subscription()      │
│  ├─ 序列化（CDR 编码）       │              │  ├─ 反序列化（CDR 解码）      │
│  └─ do_inter_process_pub()  │              │  └─ handle_message()         │
├─────────────────────────────┤              ├──────────────────────────────┤
│  rcl Layer（C）             │              │  rcl Layer（C）              │
│  rcl_publish()              │              │  rcl_take()                  │
│  └─ 无额外逻辑，直接委托     │              │  └─ 无额外逻辑，直接委托     │
├─────────────────────────────┤              ├──────────────────────────────┤
│  rmw Layer（抽象接口）       │              │  rmw Layer（抽象接口）        │
│  rmw_publish()              │              │  rmw_take_with_info()        │
├─────────────────────────────┤              ├──────────────────────────────┤
│  rmw_fastrtps_cpp           │              │  rmw_fastrtps_cpp            │
│  CustomPublisher::publish() │              │  CustomSubscriber::take()    │
├─────────────────────────────┤              ├──────────────────────────────┤
│  FastDDS Layer              │              │  FastDDS Layer               │
│  DataWriter::write()        │◄────────────►│  DataReader::read()          │
│  └─ RTPS Writer             │   UDP/TCP    │  └─ RTPS Reader              │
│     └─ 发送 RTPS 数据包     │   或共享内存  │     └─ 接收 RTPS 数据包      │
└─────────────────────────────┘              └──────────────────────────────┘
```

---

## 二、进程内通信（Intra-Process）路径

```
发布端                                        订阅端（同一进程内）
┌─────────────────────────────┐              ┌──────────────────────────────┐
│  pub->publish(unique_ptr)   │              │  execute_subscription()      │
├─────────────────────────────┤              ├──────────────────────────────┤
│  Publisher<T>::publish()    │              │  Subscription<T>::            │
│  └─ do_intra_process_pub()  │              │     handle_intra_process()   │
├─────────────────────────────┤              ├──────────────────────────────┤
│  IntraProcessManager::      │              │  IntraProcessManager::        │
│  do_intra_process_publish() │              │  take_intra_process_message() │
│  ├─ 找到所有进程内订阅者     │              │  └─ 从 ring buffer 取消息    │
│  └─ 存入各订阅者 ring buffer│────触发──────►│  guard_condition triggered   │
│     └─ 触发 guard condition │              │  └─ Executor 唤醒            │
└─────────────────────────────┘              └──────────────────────────────┘
```

**关键优势**：消息不经过序列化/反序列化，不经过 DDS，直接通过内存传递。

---

## 三、DDS 层 Topic 命名规则

ROS2 在 DDS Topic 名称上加前缀，与原生 DDS 共存时不冲突：

```
ROS2 Topic 名          → DDS Topic 名
/chatter               → rt/chatter
/ns/cmd_vel            → rt/ns/cmd_vel

ROS2 Service 名         → DDS Topic 名（两个 Topic）
/add_two_ints          → rq/add_two_ints:AddTwoIntsRequest_
                       → rr/add_two_ints:AddTwoIntsReply_

ROS2 Action 名          → DDS Topic 名（5 个 Topic）
/navigate_to_pose      → rq/navigate_to_pose/_action/send_goalRequest
                       → rr/navigate_to_pose/_action/send_goalReply
                       → rq/navigate_to_pose/_action/cancelRequest
                       → rr/navigate_to_pose/_action/cancelReply
                       → rt/navigate_to_pose/_action/feedback
                       → rt/navigate_to_pose/_action/status

ROS2 参数名             → DDS Topic 名
/node_name/get_params  → rq/node_name/get_parametersRequest
                       → rr/node_name/get_parametersReply
```

---

## 四、QoS 映射：rclcpp → rcl → rmw → DDS

```
rclcpp::QoS（C++ 类）
  .reliability(Reliable)
  .durability(TransientLocal)
  .history(KeepLast, 10)
  .deadline(std::chrono::seconds(1))
        │
        ▼ rclcpp 转换
rmw_qos_profile_t（C 结构体）
  {
    .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE
    .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST
    .depth = 10
    .deadline = {1, 0}  // seconds, nanoseconds
  }
        │
        ▼ rmw_fastrtps_cpp 转换
eprosima::fastdds::dds::TopicQos（FastDDS）
  {
    .reliability.kind = RELIABLE_RELIABILITY_QOS
    .durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS
    .history.kind = KEEP_LAST_HISTORY_QOS
    .history.depth = 10
    .deadline.period = {1, 0}
  }
```

---

## 五、关键 DDS 实体对应关系

```
ROS2 概念                DDS 概念
────────────────────────────────────────────
Context                  DomainParticipant 上下文
Node                     DomainParticipant（Humble 前）
                         Publisher 上的 UserData（Humble 后）
Publisher<T>             DataWriter
Subscription<T>          DataReader
Topic 类型               TypeSupport（序列化接口）
QoS 设置                 DataWriterQos / DataReaderQos
Guard Condition          GuardCondition
Wait Set                 WaitSet
```

> 注意：**从 Humble 起，一个 DDS DomainParticipant 对应整个进程（不是节点）**，
> 多个 ROS2 节点共享同一个 DomainParticipant，通过 UserData 区分节点身份。

---

## 六、消息流时序图（进程外通信）

```
时间轴 ──────────────────────────────────────────────────────►

Publisher                  DDS 网络层                  Subscriber
    │                          │                           │
    │ publish(msg)              │                           │
    │──序列化→CDR───►DataWriter.write()                     │
    │                          │                           │
    │                  RTPS DATA 包                        │
    │                     UDP 发送──────────────────────►  │
    │                          │                DataReader收到包
    │                          │                CDR 数据进队列
    │                          │                通知 WaitSet
    │                          │                           │
    │                          │              rcl_wait()返回
    │                          │              rcl_take()被调用
    │                          │              反序列化 CDR→Msg
    │                          │              callback(msg) 触发
    │                          │                           │

典型延迟分布（本机 UDP，std_msgs::msg::String）：
  序列化：       ~1μs
  UDP 发送/接收：~10μs
  反序列化：     ~1μs
  Executor 调度：~5μs
  总计：         ~20μs（本机）
```

---

## 七、RTPS 数据包格式

```
UDP 数据包（单条消息）：
┌────────────────────────────────────────────────────────────────────┐
│ UDP Header (8 bytes)                                               │
├────────────────────────────────────────────────────────────────────┤
│ RTPS Header (20 bytes)                                             │
│   ├─ Protocol ID: "RTPS"                                           │
│   ├─ Protocol Version: 2.3                                         │
│   ├─ Vendor ID: 0x0101 (eProsima FastDDS)                         │
│   └─ GUID Prefix (12 bytes)                                        │
├────────────────────────────────────────────────────────────────────┤
│ Submessage: INFO_TS (12 bytes)  ← 时间戳                          │
├────────────────────────────────────────────────────────────────────┤
│ Submessage: DATA                                                   │
│   ├─ ExtraFlags: 0x0000                                            │
│   ├─ OctetsToInlineQos: 16                                         │
│   ├─ ReaderEntityId (4 bytes)                                      │
│   ├─ WriterEntityId (4 bytes)                                      │
│   ├─ WriterSN (8 bytes)  ← 序列号（用于可靠传输确认）              │
│   └─ SerializedPayload:                                            │
│       ├─ RepresentationIdentifier (2 bytes): 0x00 0x01 (CDR LE)   │
│       ├─ RepresentationOptions (2 bytes): 0x00 0x00                │
│       └─ Payload: [CDR 编码的 ROS 消息]                            │
└────────────────────────────────────────────────────────────────────┘
```

---

## 八、数据流诊断命令

```bash
# 实时监控 topic 统计（延迟、频率）
ros2 topic hz /chatter
ros2 topic bw /chatter  # 带宽
ros2 topic delay /chatter  # 时间戳延迟（需 topic 有 header.stamp）

# 查看发布者/订阅者详情（含 QoS）
ros2 topic info /chatter --verbose

# 查看 DDS 实体（需 fastdds discovery tool）
fastdds discovery --server

# 查看序列化大小
ros2 topic echo /chatter --once --qos-reliability best_effort | wc -c

# 录制并分析消息间隔
ros2 bag record /chatter
ros2 bag info ros2bag_xxx.db3
```
